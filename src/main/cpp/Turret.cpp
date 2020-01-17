/**
 *  Turret.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Geoffrey Xue
 */


#include "Turret.h"
#include "Const.h"


using namespace std;


Turret::Turret(OperatorInputs *inputs)
{
    m_inputs = inputs;

    m_flywheelmotor = nullptr;

    // P, I, D, FF, Iz, nominal, peak
    m_flywheelPIDvals[0] = 0.0; 
    m_flywheelPIDvals[1] = 0.0; 
    m_flywheelPIDvals[2] = 0.0; 
    m_flywheelPIDvals[3] = 0.0;
    m_flywheelPIDvals[4] = 2000;
    m_flywheelPIDvals[5] = 0;
    m_flywheelPIDvals[6] = 0.5;

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
    m_flywheelmotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TUR_TIMEOUT_MS);
    m_flywheelmotor->SetSensorPhase(true);
    m_flywheelmotor->SetInverted(true);


    /* set the peak and nominal outputs */
    m_flywheelmotor->ConfigNominalOutputForward(m_flywheelPIDvals[5], TUR_TIMEOUT_MS);
    m_flywheelmotor->ConfigNominalOutputReverse(-1 * m_flywheelPIDvals[5], TUR_TIMEOUT_MS);
    m_flywheelmotor->ConfigPeakOutputForward(m_flywheelPIDvals[6], TUR_TIMEOUT_MS);
    m_flywheelmotor->ConfigPeakOutputReverse(-1 * m_flywheelPIDvals[6], TUR_TIMEOUT_MS);
    /* set closed loop gains in slot0 */
    m_flywheelmotor->Config_kP(0, m_flywheelPIDvals[0], TUR_TIMEOUT_MS);
    m_flywheelmotor->Config_kI(0, m_flywheelPIDvals[1], TUR_TIMEOUT_MS);
    m_flywheelmotor->Config_kD(0, m_flywheelPIDvals[2], TUR_TIMEOUT_MS);
    m_flywheelmotor->Config_kF(0, m_flywheelPIDvals[3], TUR_TIMEOUT_MS);

    //m_pigeon = new PigeonIMU(0);
    //m_heading = 0;
    //m_hoodmotor = new TalonSRX(1);
    //m_turretmotor = new TalonSRX(2);
    //m_hoodPID = new PIDController(m_hoodPIDvals[0], m_hoodPIDvals[1], m_hoodPIDvals[2], nullptr, nullptr);
    //m_turretPID = new PIDController(m_turretPIDvals[0], m_turretPIDvals[1], m_turretPIDvals[2], nullptr, nullptr);

    m_turretstate = kIdle;
    m_firemode = kHoldFire;

    // Testing purposes for PID
    SmartDashboard::PutNumber("P Gain",        m_flywheelPIDvals[0]);
    SmartDashboard::PutNumber("I Gain",        m_flywheelPIDvals[1]);
    SmartDashboard::PutNumber("D Gain",        m_flywheelPIDvals[2]);
    //SmartDashboard::PutNumber("I Zone",        m_flywheelPIDvals[3]);
    SmartDashboard::PutNumber("Feed Forward",  m_flywheelPIDvals[3]);
    SmartDashboard::PutNumber("Min Output",    m_flywheelPIDvals[5]);
    SmartDashboard::PutNumber("Max Output",    m_flywheelPIDvals[6]);
    SmartDashboard::PutNumber("RPMScaling", 100);
}


void Turret::Loop()
{
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard::GetNumber("P Gain", 0);
    double i = SmartDashboard::GetNumber("I Gain", 0);
    double d = SmartDashboard::GetNumber("D Gain", 0);
    //double iz = SmartDashboard::GetNumber("I Zone", 0);
    double ff = SmartDashboard::GetNumber("Feed Forward", 0);
    double peak = SmartDashboard::GetNumber("Max Output", 0);
    double nominal = SmartDashboard::GetNumber("Min Output", 0);
    double RPMScaling = SmartDashboard::GetNumber("RPMScaling", 100);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != m_flywheelPIDvals[0])) { m_flywheelmotor->Config_kP(0, p, TUR_TIMEOUT_MS); m_flywheelPIDvals[0] = p; }
    if((i != m_flywheelPIDvals[1])) { m_flywheelmotor->Config_kI(0, i, TUR_TIMEOUT_MS); m_flywheelPIDvals[1] = i; }
    if((d != m_flywheelPIDvals[2])) { m_flywheelmotor->Config_kD(0, d, TUR_TIMEOUT_MS); m_flywheelPIDvals[2] = d; }
    //if((iz != m_flywheelPIDvals[3])) { m_flywheelPID->SetIZone(iz); m_flywheelPIDvals[3] = iz; }
    if((ff != m_flywheelPIDvals[3])) { m_flywheelmotor->Config_kF(0, ff, TUR_TIMEOUT_MS); m_flywheelPIDvals[3] = ff; }
    if((nominal != m_flywheelPIDvals[5]) || (peak != m_flywheelPIDvals[6])) 
    { 
        m_flywheelmotor->ConfigNominalOutputForward(nominal, TUR_TIMEOUT_MS);
        m_flywheelmotor->ConfigNominalOutputReverse(-1 * nominal, TUR_TIMEOUT_MS);
        m_flywheelmotor->ConfigPeakOutputForward(peak, TUR_TIMEOUT_MS);
        m_flywheelmotor->ConfigPeakOutputReverse(-1 * peak, TUR_TIMEOUT_MS);
        m_flywheelPIDvals[5] = nominal; m_flywheelPIDvals[6] = peak; 
    }

    // Testing sample speeds
    double setpoint = 0;

    if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold, 0))
        setpoint = 0;
    else if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold, 0))
        setpoint = 1 * RPMScaling;
    else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold, 0))
        setpoint = 2 * RPMScaling;
    else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold, 0))
        setpoint = 3 * RPMScaling;
    

    double targetVelocity_UnitsPer100ms = setpoint * 4096 / 600;
    /* 500 RPM in either direction */
    m_flywheelmotor->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 
    //m_flywheelPID->SetReference(setpoint, ControlType::kVelocity);

    SmartDashboard::PutNumber("SetPoint", setpoint);
    SmartDashboard::PutNumber("Encoder_Position", m_flywheelmotor->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Encoder_Velocity", m_flywheelmotor->GetSelectedSensorVelocity(0));
    SmartDashboard::PutNumber("ClosedLoopError", m_flywheelmotor->GetClosedLoopError(0));
    SmartDashboard::PutNumber("ClosedLoopTarget", m_flywheelmotor->GetClosedLoopTarget(0));
    SmartDashboard::PutNumber("Motor Voltage", m_flywheelmotor->GetMotorOutputVoltage());

    //TurretStates();
    //FireModes();
}


void Turret::Stop()
{
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