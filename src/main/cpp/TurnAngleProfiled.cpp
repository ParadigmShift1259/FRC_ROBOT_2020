/**
 *  TurnAngleProfiled.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "TurnAngleProfiled.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


TurnAngleProfiled::TurnAngleProfiled(OperatorInputs *inputs, Drivetrain *drivetrain)
{
	m_inputs = inputs;
    m_drivetrain = drivetrain;

    m_gyroPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_constraints.maxVelocity = 0_mps;
    m_constraints.maxAcceleration = 0_mps / 1_s;
    m_tolerance = 2_deg;
    m_gyroPIDvals[0] = 0;
    m_gyroPIDvals[1] = 0;
    m_gyroPIDvals[2] = 0;

    m_setpoint = 0_deg;
    m_finished = false;

    m_autostate = kIdle;
}


TurnAngleProfiled::~TurnAngleProfiled()
{
    if (m_inputs != nullptr)
        delete m_inputs;
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
}


void TurnAngleProfiled::Init()
{
    if (m_gyroPIDController == nullptr)
    {
        m_gyroPIDController = new ProfiledPIDController<units::meters>(
            m_gyroPIDvals[0],
            m_gyroPIDvals[1], 
            m_gyroPIDvals[2],
            m_constraints
        );
    }
    
    m_drivetrain->Init();

    m_gyroPIDController->SetConstraints(m_constraints);
    m_gyroPIDController->SetPID(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);
    m_tolerance = 2_deg;
    m_gyroPIDController->SetTolerance(
        units::meter_t{m_tolerance.to<double>() * DEG_TO_ROT * ROBOT_CIRCUMFERENCE}, 
        units::meters_per_second_t{HIGH_NUMBER});
    
    m_setpoint = 0_deg;
    m_finished = false;

    m_autostate = kIdle;

	SmartDashboard::PutNumber("Gyro P",         m_gyroPIDvals[0]);
    SmartDashboard::PutNumber("Gyro I",         m_gyroPIDvals[1]);
    SmartDashboard::PutNumber("Gyro D",         m_gyroPIDvals[2]);

    SmartDashboard::PutNumber("Gyro Max Velocity",       m_constraints.maxVelocity.to<double>());
    SmartDashboard::PutNumber("Gyro Max Acceleration",   m_constraints.maxAcceleration.to<double>());
    SmartDashboard::PutNumber("Gyro Goal Tolerance",     m_tolerance.to<double>());

    SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
}


void TurnAngleProfiled::Loop()
{
    units::degree_t setpoint = units::degree_t{ SmartDashboard::GetNumber("Setpoint", 0)};
    switch (m_autostate)
    {
        case kIdle:
            m_drivetrain->GetDrive()->ArcadeDrive(0, 0, false);
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                m_setpoint = setpoint;
                // resets gyro before attempting TurnAngleProfiled
                m_drivetrain->ResetGyro();
                // Sets the goal of the motion profiled run
                m_gyroPIDController->SetGoal(units::meter_t{ m_setpoint.to<double>() * DEG_TO_ROT * ROBOT_CIRCUMFERENCE});
                m_autostate = kDrive;
                m_finished = false;
            }
            break;
        case kDrive:
            // Z deviance of drive, with error being degrees
            // (TAG) Make sure to check this is in the right direction, else flip gyro in Drivetrain or flip here
            units::degree_t heading = units::degree_t{m_drivetrain->GetGyro()->GetCompassHeading()};

            double z = m_gyroPIDController->Calculate(units::meter_t{ heading.to<double>() * DEG_TO_ROT * ROBOT_CIRCUMFERENCE});

            m_drivetrain->GetDrive()->ArcadeDrive(0, z, false);

            if (m_gyroPIDController->AtSetpoint())
            {
                m_setpoint = 0_deg;
                SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
                m_autostate = kIdle;
                m_finished = true;
            }
            break;
    }
}


void TurnAngleProfiled::Stop()
{

}



void TurnAngleProfiled::ConfigureProfile()
{
    units::meters_per_second_t v = units::meters_per_second_t{ SmartDashboard::GetNumber("Gyro Max Velocity", 0)};
    units::meters_per_second_squared_t a = units::meters_per_second_squared_t{ SmartDashboard::GetNumber("Gyro Max Acceleration", 0)};

    if (v != m_constraints.maxVelocity) 
    {
        m_constraints.maxVelocity = v; 
        m_gyroPIDController->SetConstraints(m_constraints);
    }
    if (a != m_constraints.maxAcceleration)
    {
        m_constraints.maxAcceleration = a;
        m_gyroPIDController->SetConstraints(m_constraints);
    }
}


void TurnAngleProfiled::ConfigureGyroPID()
{
    double p = SmartDashboard::GetNumber("Gyro P", 0);
    double i = SmartDashboard::GetNumber("Gyro I", 0);
    double d = SmartDashboard::GetNumber("Gyro D", 0);
    units::degree_t t = units::degree_t{ SmartDashboard::GetNumber("Gyro Goal Tolerance", 0)};

    if((p != m_gyroPIDvals[0])) { m_gyroPIDController->SetP(p); m_gyroPIDvals[0] = p; }
    if((i != m_gyroPIDvals[1])) { m_gyroPIDController->SetI(i); m_gyroPIDvals[1] = i; }
    if((d != m_gyroPIDvals[2])) { m_gyroPIDController->SetD(d); m_gyroPIDvals[2] = d; }
    if (t != m_tolerance)
    {
        m_tolerance = t;
        m_gyroPIDController->SetTolerance(
            units::meter_t{ m_tolerance.to<double>() * DEG_TO_ROT * ROBOT_CIRCUMFERENCE}, 
            units::meters_per_second_t{HIGH_NUMBER}
        );
    }

}