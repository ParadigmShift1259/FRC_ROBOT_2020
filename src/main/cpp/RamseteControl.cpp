/**
 *  RamseteControl.cpp
 *  Date: 2/02/20
 *  Last Edited By: Geoffrey Xue
 */


#include "RamseteControl.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


RamseteControl::RamseteControl(OperatorInputs *inputs, Drivetrain *drivetrain, DriveSubsystem *drivesubsystem)
{
	m_inputs = inputs;
    m_drivetrain = drivetrain;
    m_drivesubsystem = drivesubsystem;
}


RamseteControl::~RamseteControl()
{
	if (m_inputs != nullptr)
		delete m_inputs;
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
    if (m_drivesubsystem != nullptr)
        delete m_drivesubsystem;
}


void RamseteControl::Init()
{
    m_drivetrain->Init();
 
    m_ramsetecommand = new frc2::RamseteCommand(
        m_drivesubsystem->GetTrajectory(),
        [this]() { return m_drivesubsystem->GetPose(); },
        m_drivesubsystem->GetRamseteController(),
        m_drivesubsystem->GetFeedforward(),
        m_drivesubsystem->GetKinematics(),
        [this] { return m_drivesubsystem->GetWheelSpeeds(); },
        m_drivesubsystem->GetLeftPID(),
        m_drivesubsystem->GetRightPID(),
        [this](auto left, auto right) { return m_drivesubsystem->SetOutputVolts(left, right); },
        m_drivesubsystem
    );

    m_finished = false;
}


void RamseteControl::Loop()
{
    switch (m_autostate)
    {
        case kIdle:
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                // reset both encoders and gyro before running Ramsete
                m_drivetrain->ResetEncoders();
                m_drivetrain->ResetGyro();
                m_autostate = kDrive;
                m_finished = false;
            }
            break;
        case kDrive:
            m_ramsetecommand->Initialize();
            m_ramsetecommand->Execute();
            if (m_ramsetecommand->IsFinished())
            {
                m_ramsetecommand->End(false);
                m_finished = true;
                m_autostate = kIdle;
            }
            break;
    }
}


void RamseteControl::Stop()
{

}