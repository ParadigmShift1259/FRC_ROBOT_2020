/**
 *  Turret.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Geoffrey Xue
 */


#include "Turret.h"
#include "Const.h"
//#include "Feeder.h"
//#include "Intake.h"


using namespace std;


Turret::Turret(OperatorInputs *inputs)
{
	m_inputs = inputs;

	m_flywheelmotor = nullptr;

	// P, I, D, FF, Iz, nominal, peak
	// PID Values tuned for MiniCIM 1/17/20 Geoffrey
	m_flywheelPIDvals[0] = 0.8; 
	m_flywheelPIDvals[1] = 0.0; 
	m_flywheelPIDvals[2] = 0.029; 
	m_flywheelPIDvals[3] = 0.035;
	m_flywheelPIDvals[4] = 2000;
	m_flywheelPIDvals[5] = 0;
	m_flywheelPIDvals[6] = 0.5;
	
	m_flywheelsetpoint = 0;

	//m_pigeon = nullptr;
	//m_heading = 0;
	//m_hoodmotor = nullptr;
	//m_turretmotor = nullptr;
	//m_hoodPID = nullptr;
	//m_turretPID = nullptr;
	//m_hoodPIDvals[0] = 0.0; m_hoodPIDvals[1] = 0.0; m_hoodPIDvals[2] = 0.0;
	//m_turretPIDvals[0] = 0.0; m_turretPIDvals[1] = 0.0; m_turretPIDvals[2] = 0.0;

	m_turretstate = kIdle;
	m_firemode = kHoldFire;
}


Turret::~Turret()
{
	if (m_flywheelmotor != nullptr)
		delete m_flywheelmotor;
	/*
	if (m_flywheelPID != nullptr)
		delete m_flywheelPID;
	if (m_flywheelencoder != nullptr)
		delete m_flywheelencoder;
		*/
	/*
	if (m_pigeon != nullptr)
		delete m_pigeon;
	if (m_hoodmotor != nullptr)
		delete m_hoodmotor;
	if (m_turretmotor != nullptr)
		delete m_turretmotor;
	if (m_hoodPID != nullptr)
		delete m_hoodPID;
	if (m_turretPID != nullptr)
		delete m_turretPID;
	*/
}


void Turret::Init()
{
	/**
	 * Constants needed:
	 * CAN_GYRO_TURRET
	 * CAN_FLYWHEEL_PORT
	 * CAN_HOOD_PORT
	 * CAN_TURRET_PORT
	 */

	m_flywheelmotor = new TalonSRX(0);
	m_flywheelmotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TIMEOUT_MS);
	m_flywheelmotor->SetSensorPhase(true);
	m_flywheelmotor->SetInverted(true);


	/* set the peak and nominal outputs */
	m_flywheelmotor->ConfigNominalOutputForward(m_flywheelPIDvals[5], TIMEOUT_MS);
	m_flywheelmotor->ConfigNominalOutputReverse(-1 * m_flywheelPIDvals[5], TIMEOUT_MS);
	m_flywheelmotor->ConfigPeakOutputForward(m_flywheelPIDvals[6], TIMEOUT_MS);
	m_flywheelmotor->ConfigPeakOutputReverse(-1 * m_flywheelPIDvals[6], TIMEOUT_MS);
	/* set closed loop gains in slot0 */
	m_flywheelmotor->Config_kP(0, m_flywheelPIDvals[0], TIMEOUT_MS);
	m_flywheelmotor->Config_kI(0, m_flywheelPIDvals[1], TIMEOUT_MS);
	m_flywheelmotor->Config_kD(0, m_flywheelPIDvals[2], TIMEOUT_MS);
	m_flywheelmotor->Config_kF(0, m_flywheelPIDvals[3], TIMEOUT_MS);
	m_flywheelmotor->SetNeutralMode(NeutralMode::Coast);

	//m_pigeon = new PigeonIMU(0);
	//m_heading = 0;
	//m_hoodmotor = new TalonSRX(1);
	//m_turretmotor = new TalonSRX(2);
	//m_hoodPID = new PIDController(m_hoodPIDvals[0], m_hoodPIDvals[1], m_hoodPIDvals[2], nullptr, nullptr);
	//m_turretPID = new PIDController(m_turretPIDvals[0], m_turretPIDvals[1], m_turretPIDvals[2], nullptr, nullptr);

	m_turretstate = kIdle;
	m_firemode = kHoldFire;

	// Testing purposes for PID
	SmartDashboard::PutNumber("TU1_P Gain",        m_flywheelPIDvals[0]);
	SmartDashboard::PutNumber("TU2_I Gain",        m_flywheelPIDvals[1]);
	SmartDashboard::PutNumber("TU3_D Gain",        m_flywheelPIDvals[2]);
	//SmartDashboard::PutNumber("I Zone",        m_flywheelPIDvals[3]);
	SmartDashboard::PutNumber("TU4_Feed Forward",  m_flywheelPIDvals[3]);
	SmartDashboard::PutNumber("TU5_Min Output",    m_flywheelPIDvals[5]);
	SmartDashboard::PutNumber("TU6_Max Output",    m_flywheelPIDvals[6]);
	SmartDashboard::PutNumber("TU7_RPMScaling", 100);
}


void Turret::Loop()
{
	// read PID coefficients from SmartDashboard
	double p = SmartDashboard::GetNumber("TU1_P Gain", 0);
	double i = SmartDashboard::GetNumber("TU2_I Gain", 0);
	double d = SmartDashboard::GetNumber("TU3_D Gain", 0);
	//double iz = SmartDashboard::GetNumber("I Zone", 0);
	double ff = SmartDashboard::GetNumber("TU4_Feed Forward", 0);
	double peak = SmartDashboard::GetNumber("TU5_Max Output", 0);
	double nominal = SmartDashboard::GetNumber("TU6_Min Output", 0);
	double RPMScaling = SmartDashboard::GetNumber("TU7_RPMScaling", 100);

	// if PID coefficients on SmartDashboard have changed, write new values to controller
	if((p != m_flywheelPIDvals[0])) { m_flywheelmotor->Config_kP(0, p, TIMEOUT_MS); m_flywheelPIDvals[0] = p; }
	if((i != m_flywheelPIDvals[1])) { m_flywheelmotor->Config_kI(0, i, TIMEOUT_MS); m_flywheelPIDvals[1] = i; }
	if((d != m_flywheelPIDvals[2])) { m_flywheelmotor->Config_kD(0, d, TIMEOUT_MS); m_flywheelPIDvals[2] = d; }
	//if((iz != m_flywheelPIDvals[3])) { m_flywheelPID->SetIZone(iz); m_flywheelPIDvals[3] = iz; }
	if((ff != m_flywheelPIDvals[3])) { m_flywheelmotor->Config_kF(0, ff, TIMEOUT_MS); m_flywheelPIDvals[3] = ff; }
	if((nominal != m_flywheelPIDvals[5]) || (peak != m_flywheelPIDvals[6])) 
	{ 
		m_flywheelmotor->ConfigNominalOutputForward(nominal, TIMEOUT_MS);
		m_flywheelmotor->ConfigNominalOutputReverse(-1 * nominal, TIMEOUT_MS);
		m_flywheelmotor->ConfigPeakOutputForward(peak, TIMEOUT_MS);
		m_flywheelmotor->ConfigPeakOutputReverse(-1 * peak, TIMEOUT_MS);
		m_flywheelPIDvals[5] = nominal; m_flywheelPIDvals[6] = peak; 
	}

	// Testing sample speeds

	if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
		m_flywheelsetpoint = 0;
	else if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
		m_flywheelsetpoint = 1 * RPMScaling;
	else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
		m_flywheelsetpoint = 2 * RPMScaling;
	else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
		m_flywheelsetpoint = 3 * RPMScaling;
	
	if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 0))
		m_flywheelsetpoint += 10;
	else if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 0) && (m_flywheelsetpoint >= 10))
		m_flywheelsetpoint -= 10;

	double targetVelocity_UnitsPer100ms = m_flywheelsetpoint * ENCODER_TICKS_PER_REV * MINUTES_TO_HUNDRED_MS;
	/* 500 RPM in either direction */
	m_flywheelmotor->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 
	//m_flywheelPID->SetReference(setpoint, ControlType::kVelocity);

	SmartDashboard::PutNumber("TU8_SetPoint", m_flywheelsetpoint);
	SmartDashboard::PutNumber("TU9_Encoder_Position in Revolutions", m_flywheelmotor->GetSelectedSensorPosition(0) / ENCODER_TICKS_PER_REV);
	SmartDashboard::PutNumber("TU10_Encoder_Velocity in RPM", m_flywheelmotor->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / ENCODER_TICKS_PER_REV);
	SmartDashboard::PutNumber("TU11_ClosedLoopError", m_flywheelmotor->GetClosedLoopError(0));
	SmartDashboard::PutNumber("TU12_ClosedLoopTarget", m_flywheelmotor->GetClosedLoopTarget(0));
	SmartDashboard::PutNumber("TU13_Motor Voltage", m_flywheelmotor->GetMotorOutputVoltage());

	//TurretStates();
	//FireModes();
}


void Turret::Stop()
{
	m_flywheelsetpoint = 0;
	//m_flywheelPID->SetReference(0, ControlType::kVelocity);
}


void Turret::TurretStates()
{
	switch (m_turretstate)
	{
		case kIdle:
		case kPreMove:
		case kHoming:
		case kReady:
		case kFire:
		case kReturn:
			break;
	}
}


void Turret::FireModes()
{
	switch (m_firemode)
	{
		case kFireWhenReady:
		case kHoldFire:
		case kManualFire:
		case kOff:
			break;
	}
}


void Turret::CalculateHoodFlywheel(double distance, double &hoodangle, double &flywheelspeed)
{

}