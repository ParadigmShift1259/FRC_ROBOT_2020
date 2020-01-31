/**
 *  TurnAngleDegrees.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "TurnAngleDegrees.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


TurnAngleDegrees::TurnAngleDegrees(OperatorInputs *inputs, Drivetrain *drivetrain)
{
	m_inputs = inputs;
    m_drivetrain = drivetrain;

    m_gyroPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_gyroPIDvals[0] = 0;
    m_gyroPIDvals[1] = 0;
    m_gyroPIDvals[2] = 0;
    m_tolerance = 2_deg;

    m_setpoint = 0_deg;
    m_finished = false;

    m_autostate = kIdle;
}


TurnAngleDegrees::~TurnAngleDegrees()
{
    if (m_inputs != nullptr)
        delete m_inputs;
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
}


void TurnAngleDegrees::Init()
{
    if (m_gyroPIDController == nullptr)
    {
        m_gyroPIDController = new frc2::PIDController(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);
    }
    
    m_drivetrain->Init();

    m_gyroPIDController->SetPID(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);
    m_gyroPIDController->SetTolerance(m_tolerance.to<double>(), HIGH_NUMBER);

    m_setpoint = 0_deg;

    m_autostate = kIdle;
    m_finished = false;

	SmartDashboard::PutNumber("Gyro P",         m_gyroPIDvals[0]);
    SmartDashboard::PutNumber("Gyro I",          m_gyroPIDvals[1]);
    SmartDashboard::PutNumber("Gyro D",         m_gyroPIDvals[2]);

    SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
}


void TurnAngleDegrees::Loop()
{
   units::degree_t setpoint = units::degree_t{ SmartDashboard::GetNumber("Setpoint", 0)};
    switch (m_autostate)
    {
        case kIdle:
            m_drivetrain->GetDrive()->ArcadeDrive(0, 0, false);
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                m_setpoint = setpoint;
                // resets gyro before attempting turnangledegrees
                m_drivetrain->ResetGyro();
                m_autostate = kDrive;
                m_finished = false;
            }
            break;
        case kDrive:
            // Z deviance of drive, with error being degrees
            // (TAG) Make sure to check this is in the right direction, else flip gyro in Drivetrain or flip here
            units::degree_t heading = units::degree_t{ m_drivetrain->GetGyro()->GetCompassHeading()};
            double z = m_gyroPIDController->Calculate(heading.to<double>(), m_setpoint.to<double>());

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


void TurnAngleDegrees::Stop()
{

}


void TurnAngleDegrees::ConfigureGyroPID()
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
        m_gyroPIDController->SetTolerance(m_tolerance.to<double>());
    }
}
