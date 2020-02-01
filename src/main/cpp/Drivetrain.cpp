/**
 *  Drivetrain.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "Drivetrain.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


Drivetrain::Drivetrain(OperatorInputs *inputs)
{
	m_inputs = inputs;
	m_left1 = nullptr;
	m_left2 = nullptr;
	m_left3 = nullptr;
	m_right1 = nullptr;
	m_right2 = nullptr;
	m_right3 = nullptr;
	m_leftscgroup = nullptr;
	m_rightscgroup = nullptr;
	m_differentialdrive = nullptr;
	m_leftsensor = nullptr;
	m_rightsensor = nullptr;
	m_gyro = nullptr;

	m_leftinvert = INVERT_LEFT;
	m_rightinvert = INVERT_RIGHT;
}


Drivetrain::~Drivetrain()
{
	if (m_inputs != nullptr)
		delete m_inputs;
	if (m_left1 != nullptr)
		delete m_left1;
	if (m_left2 != nullptr)
		delete m_left2;
	if (m_left3 != nullptr) 
		delete m_left3;
	if (m_right1 != nullptr) 
		delete m_right1;
	if (m_right2 != nullptr)
		delete m_right2;
	if (m_right3 != nullptr) 
		delete m_right3;
}


void Drivetrain::Init()
{
	/*
	SupplyCurrentLimitConfiguration supplylimit;
	supplylimit.enable = MOTOR_SUPPLY_LIMIT_ENABLE;
	supplylimit.currentLimit = MOTOR_SUPPLY_CURRENT_LIMIT;
	supplylimit.triggerThresholdCurrent = MOTOR_SUPPLY_THRESHOLD_CURRENT;
	supplylimit.triggerThresholdTime = MOTOR_SUPPLY_THRESHOLD_TIME;
	*/

	if (m_left1 == nullptr)
	{
		m_left1 = new WPI_TalonFX(CAN_LEFT_PORT_1);
		m_left2 = new WPI_TalonFX(CAN_LEFT_PORT_2);
		m_left3 = new WPI_TalonFX(CAN_LEFT_PORT_3);

		m_left1->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
		//m_left1->SetSensorPhase(false);
		//m_left1->ConfigSupplyCurrentLimit(supplylimit);
		m_left1->SetNeutralMode(NeutralMode::Brake);

		m_left2->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
		//m_left2->SetSensorPhase(false);
		//m_left2->ConfigSupplyCurrentLimit(supplylimit);
		m_left2->SetNeutralMode(NeutralMode::Brake);

		m_left3->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
		//m_left3->SetSensorPhase(false);
		//m_left3->ConfigSupplyCurrentLimit(supplylimit);
		m_left3->SetNeutralMode(NeutralMode::Brake);

		m_right1 = new WPI_TalonFX(CAN_RIGHT_PORT_1);
		m_right2 = new WPI_TalonFX(CAN_RIGHT_PORT_2);
		m_right3 = new WPI_TalonFX(CAN_RIGHT_PORT_3);

		m_right1->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
		//m_right1->ConfigSupplyCurrentLimit(supplylimit);
		m_right1->SetNeutralMode(NeutralMode::Brake);
		
		m_right2->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
		//m_right2->SetSensorPhase(false);
		//m_right2->ConfigSupplyCurrentLimit(supplylimit);
		m_right2->SetNeutralMode(NeutralMode::Brake);

		m_right3->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
		//m_right3->SetSensorPhase(false);
		//m_right3->ConfigSupplyCurrentLimit(supplylimit);
		m_right3->SetNeutralMode(NeutralMode::Brake);

		m_leftscgroup = new SpeedControllerGroup(*m_left1, *m_left2, *m_left3);
		m_rightscgroup = new SpeedControllerGroup(*m_right1, *m_right2, *m_right3);

		m_leftscgroup->SetInverted(m_leftinvert);
		m_rightscgroup->SetInverted(m_rightinvert);

		m_differentialdrive = new DifferentialDrive(*m_leftscgroup, *m_rightscgroup);

		m_leftsensor = new TalonFXSensorCollection(*m_left1);
		m_rightsensor = new TalonFXSensorCollection(*m_right1);

		m_gyro = new PigeonIMU(CAN_GYRO1);
	}

	m_leftsensor->SetIntegratedSensorPosition(0.0);
	m_rightsensor->SetIntegratedSensorPosition(0.0);
	m_gyro->SetFusedHeading(0, 0);

	SmartDashboard::PutBoolean("Invert Left", m_leftinvert);
	SmartDashboard::PutBoolean("Invert Right", m_rightinvert);

}


void Drivetrain::Loop()
{
	m_differentialdrive->ArcadeDrive(
		m_inputs->xBoxLeftY(0) * DRIVE_INVERTED,		// Forward/Back input
		m_inputs->xBoxLeftX(0) * DRIVE_INVERTED,		// Rotational input
		true						// Squared inputs (decreases sensitivity in small values)
	);
}


void Drivetrain::Stop()
{
	m_leftscgroup->StopMotor();
	m_rightscgroup->StopMotor();
}


void Drivetrain::ReportData()
{
	// Positions are returned in ticks, Velocities are returned in ticks/100ms
	SmartDashboard::PutNumber("Left Encoder Position in m", m_leftsensor->GetIntegratedSensorPosition() / TICKS_PER_METER);
	SmartDashboard::PutNumber("Left Encoder Velocity in mps", m_leftsensor->GetIntegratedSensorVelocity() / TICKS_PER_METER * 10);

	SmartDashboard::PutNumber("Right Encoder Position in m", m_rightsensor->GetIntegratedSensorPosition() / TICKS_PER_METER * ENCODER_INVERTED);
	SmartDashboard::PutNumber("Right Encoder Velocity in mps", m_rightsensor->GetIntegratedSensorVelocity() / TICKS_PER_METER * 10 * ENCODER_INVERTED);

	SmartDashboard::PutNumber("Gyro Relative Heading", m_gyro->GetFusedHeading() * GYRO_INVERTED);
}


void Drivetrain::ResetEncoders()
{
	m_leftsensor->SetIntegratedSensorPosition(0.0);
	m_rightsensor->SetIntegratedSensorPosition(0.0);
}


void Drivetrain::ResetGyro()
{
	m_gyro->SetFusedHeading(0, 0);
}


void Drivetrain::ConfigureInverts()
{
	bool left = SmartDashboard::GetBoolean("Invert Left", 0);
	bool right = SmartDashboard::GetBoolean("Invert Right", 0);

	if (left != m_leftinvert)
	{
		m_leftinvert = left;
		m_leftscgroup->SetInverted(m_leftinvert);
	}
	if (right != m_rightinvert)
	{
		m_rightinvert = right;
		m_rightscgroup->SetInverted(m_rightinvert);
	}
}