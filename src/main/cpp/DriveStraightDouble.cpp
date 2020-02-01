/**
 *  DriveStraight.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "DriveStraightDouble.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


DriveStraightDouble::DriveStraightDouble(OperatorInputs *inputs, Drivetrain *drivetrain)
{
	m_inputs = inputs;
    m_drivetrain = drivetrain;

    m_constraints.maxVelocity = 0_mps;
    m_constraints.maxAcceleration = 0_mps / 1_s;
    m_tolerance = 0.1_m;

    m_leftPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_leftPIDvals[0] = 0;
    m_leftPIDvals[1] = 0;
    m_leftPIDvals[2] = 0;

    m_rightPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_rightPIDvals[0] = 0;
    m_rightPIDvals[1] = 0;
    m_rightPIDvals[2] = 0;

    m_setpoint = 0_m;
    m_finished = false;

    m_autostate = kIdle;
}


DriveStraightDouble::~DriveStraightDouble()
{
    if (m_inputs != nullptr)
        delete m_inputs;
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
}


void DriveStraightDouble::Init()
{
    if (m_leftPIDController == nullptr)
    {
        m_leftPIDController = new ProfiledPIDController<units::meters>(
            m_leftPIDvals[0],
            m_leftPIDvals[1], 
            m_leftPIDvals[2],
            m_constraints
        );
        m_rightPIDController = new ProfiledPIDController<units::meters>(
            m_rightPIDvals[0],
            m_rightPIDvals[1], 
            m_rightPIDvals[2],
            m_constraints
        );
    }
    
    m_drivetrain->Init();

    m_leftPIDController->SetPID(m_leftPIDvals[0], m_leftPIDvals[1], m_leftPIDvals[2]);
    m_rightPIDController->SetPID(m_rightPIDvals[0], m_rightPIDvals[1], m_rightPIDvals[2]);

    m_leftPIDController->SetConstraints(m_constraints);
    m_rightPIDController->SetConstraints(m_constraints);
    m_leftPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
    m_rightPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
    
    m_setpoint = 0_m;
    m_finished = false;

    m_autostate = kIdle;

	SmartDashboard::PutNumber("Left P",             m_leftPIDvals[0]);
    SmartDashboard::PutNumber("Left I",             m_leftPIDvals[1]);
    SmartDashboard::PutNumber("Left D",             m_leftPIDvals[2]);

	SmartDashboard::PutNumber("Right P",             m_rightPIDvals[0]);
    SmartDashboard::PutNumber("Right I",             m_rightPIDvals[1]);
    SmartDashboard::PutNumber("Right D",             m_rightPIDvals[2]);

    SmartDashboard::PutNumber("Encoder Max Velocity",       m_constraints.maxVelocity.to<double>());
    SmartDashboard::PutNumber("Encoder Max Acceleration",   m_constraints.maxAcceleration.to<double>());
    SmartDashboard::PutNumber("Encoder Goal Tolerance",     m_tolerance.to<double>());

    SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
}


void DriveStraightDouble::Loop()
{
    units::meter_t setpoint = units::meter_t{ SmartDashboard::GetNumber("Setpoint", 0)};
    switch (m_autostate)
    {
        case kIdle:
            m_drivetrain->GetDrive()->TankDrive(0, 0, false);
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                m_setpoint = setpoint;
                // resets encoders and gyro before attempting drivestraightdouble
                m_drivetrain->ResetEncoders();
                // Sets the goal of the motion profiled run
                m_leftPIDController->SetGoal(m_setpoint);
                m_rightPIDController->SetGoal(m_setpoint);
                m_autostate = kDrive;
                m_finished = false;
            }
            break;
        case kDrive:
            // X deviance of drive, with error being meters
            // (TAG) Make sure a positive motor ouput results in a positive encoder velocity, else flip encoder
            // (TAG) Make sure the robot drives in the right direction, else flip motor
            double leftencoder = m_drivetrain->GetLeftSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER;
            double rightencoder = m_drivetrain->GetRightSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER;
            // Calculate errors and Motion Profiled PID outputs separately
            double left = m_leftPIDController->Calculate(units::meter_t{ leftencoder});
            double right = m_rightPIDController->Calculate(units::meter_t{ rightencoder});

            m_drivetrain->GetDrive()->TankDrive(left, right, false);

            if (m_leftPIDController->AtSetpoint() || m_rightPIDController->AtSetpoint())
            {
                m_setpoint = 0_m;
                SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
                m_autostate = kIdle;
                m_finished = true;
            }
            break;
    }
}


void DriveStraightDouble::Stop()
{

}



void DriveStraightDouble::ConfigureProfile()
{
    units::meters_per_second_t v = units::meters_per_second_t{ SmartDashboard::GetNumber("Encoder Max Velocity", 0)};
    units::meters_per_second_squared_t a = units::meters_per_second_squared_t{ SmartDashboard::GetNumber("Encoder Max Acceleration", 0)};

    if (v != m_constraints.maxVelocity) 
    {
        m_constraints.maxVelocity = v; 
        m_leftPIDController->SetConstraints(m_constraints);
        m_rightPIDController->SetConstraints(m_constraints);
    }
    if (a != m_constraints.maxAcceleration)
    {
        m_constraints.maxAcceleration = a;
        m_leftPIDController->SetConstraints(m_constraints);
        m_rightPIDController->SetConstraints(m_constraints);
    }
}


void DriveStraightDouble::ConfigureLeftPID()
{
    double p = SmartDashboard::GetNumber("Left P", 0);
    double i = SmartDashboard::GetNumber("Left I", 0);
    double d = SmartDashboard::GetNumber("Left D", 0);
    units::meter_t t = units::meter_t{ SmartDashboard::GetNumber("Encoder Goal Tolerance", 0)};

    if((p != m_leftPIDvals[0])) { m_leftPIDController->SetP(p); m_leftPIDvals[0] = p; }
    if((i != m_leftPIDvals[1])) { m_leftPIDController->SetI(i); m_leftPIDvals[1] = i; }
    if((d != m_leftPIDvals[2])) { m_leftPIDController->SetD(d); m_leftPIDvals[2] = d; }
    if (t != m_tolerance)
    {
        m_tolerance = t;
        m_leftPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
        m_rightPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
    }
}


void DriveStraightDouble::ConfigureRightPID()
{
    double p = SmartDashboard::GetNumber("Right P", 0);
    double i = SmartDashboard::GetNumber("Right I", 0);
    double d = SmartDashboard::GetNumber("Right D", 0);
    units::meter_t t = units::meter_t{ SmartDashboard::GetNumber("Encoder Goal Tolerance", 0)};

    if((p != m_rightPIDvals[0])) { m_rightPIDController->SetP(p); m_rightPIDvals[0] = p; }
    if((i != m_rightPIDvals[1])) { m_rightPIDController->SetI(i); m_rightPIDvals[1] = i; }
    if((d != m_rightPIDvals[2])) { m_rightPIDController->SetD(d); m_rightPIDvals[2] = d; }
    if (t != m_tolerance)
    {
        m_tolerance = t;
        m_leftPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
        m_rightPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
    }
}