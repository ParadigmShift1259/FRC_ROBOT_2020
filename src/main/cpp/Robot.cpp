/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include "Robot.h"
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>


bool Debug = true;


void Robot::RobotInit()
{
	m_operatorinputs = new OperatorInputs();
    m_drivetrain = new Drivetrain(m_operatorinputs);
    m_drivestraight = new DriveStraight(m_operatorinputs, m_drivetrain);
    m_drivestraightdouble = new DriveStraightDouble(m_operatorinputs, m_drivetrain);
    m_turnangledegrees = new TurnAngleDegrees(m_operatorinputs, m_drivetrain);
    m_turnangleprofiled = new TurnAngleProfiled(m_operatorinputs, m_drivetrain);

    m_chooser.SetDefaultOption(scDrivetrain, scDrivetrain);
    m_chooser.AddOption(scDriveStraight, scDriveStraight);
    m_chooser.AddOption(scDriveStraightDouble, scDriveStraightDouble);
    m_chooser.AddOption(scTurnAngleDegrees, scTurnAngleDegrees);
    m_chooser.AddOption(scTurnAngleProfiled, scTurnAngleProfiled);
    m_chooser.AddOption(scRamseteController, scRamseteController);
    SmartDashboard::PutData("Drive Modes", &m_chooser);
}


void Robot::RobotPeriodic()
{
}


void Robot::AutonomousInit()
{
}


void Robot::AutonomousPeriodic()
{ 
}


void Robot::TestInit(){}
void Robot::TestPeriodic(){}


void Robot::TeleopInit()
{
    switch (m_selector)
    {
        case kDrivetrain:
            m_drivetrain->Init();
            break;
        case kDriveStraight:
            m_drivestraight->Init();
            break;
        case kDriveStraightDouble:
            m_drivestraightdouble->Init();
            break;
        case kTurnAngleDegrees:
            m_turnangledegrees->Init();
            break;
        case kTurnAngleProfiled:
            m_turnangleprofiled->Init();
            break;
        case kRamseteController:
            break;
    }
}


void Robot::TeleopPeriodic()
{
    m_drivetrain->ReportData();    

    switch (m_selector)
    {
        case kDrivetrain:
            m_drivetrain->ConfigureInverts();
            m_drivetrain->Loop();
            break;
        case kDriveStraight:
            // everything in meters
            m_drivestraight->ConfigureGyroPID();
            m_drivestraight->ConfigureEncoderPID();
            m_drivestraight->ConfigureProfile();
            m_drivestraight->Loop();
            SmartDashboard::PutBoolean("Finished", m_drivestraight->IsFinished());
            break;
        case kDriveStraightDouble:
            m_drivestraightdouble->ConfigureLeftPID();
            m_drivestraightdouble->ConfigureRightPID();
            m_drivestraightdouble->ConfigureProfile();
            m_drivestraightdouble->Loop();
            SmartDashboard::PutBoolean("Finished", m_drivestraightdouble->IsFinished());
            break;
        case kTurnAngleDegrees:
            // setpoint and tolerance are in degrees
            m_turnangledegrees->ConfigureGyroPID();
            m_turnangledegrees->Loop();
            SmartDashboard::PutBoolean("Finished", m_turnangledegrees->IsFinished());
            break;
        case kTurnAngleProfiled:
            // setpoint and tolerance are in degrees, while maxVel and maxAcc are in meters
            m_turnangleprofiled->ConfigureGyroPID();
            m_turnangleprofiled->ConfigureProfile();
            m_turnangleprofiled->Loop();
            SmartDashboard::PutBoolean("Finished", m_turnangleprofiled->IsFinished());
            break;
        case kRamseteController:
            break;
    }
}


void Robot::DisabledInit()
{
}


void Robot::DisabledPeriodic()
{
    ReadChooser();
}


void Robot::ReadChooser()
{
    string driveselected = m_chooser.GetSelected();
    /**
     * Selector
     * Drivetrain - Runs Arcade Drive with Left joystick, essentially teleop
     * DriveStraight - Runs Profiled Encoder PID and Gyro PID, 2 sets of PID vals + Profile setup
     * DriveStraightDouble - Runs 2 Profiled Encoder PIDs, 2 sets of encoder PID vals + Profile setup
     * TurnAngleDegrees - Runs Gyro PID, 1 set of PID vals
     * TurnAngleProfiled - Runs Profiled Gyro PID, 1 set of PID vals + Profile setup
     * RamseteController - Runs Ramsete Controller, 2 sets of PID vals + Profile setup
     */
    m_selector = kDrivetrain;
    if (driveselected == scDrivetrain)
        m_selector = kDrivetrain;
    else
    if (driveselected == scDriveStraight)
        m_selector = kDriveStraight;
    else
    if (driveselected == scDriveStraightDouble)
        m_selector = kDriveStraightDouble;
    if (driveselected == scTurnAngleDegrees)
        m_selector = kTurnAngleDegrees;
    else
    if (driveselected == scTurnAngleProfiled)
        m_selector = kTurnAngleProfiled;
    else
    if (driveselected == scRamseteController)
        m_selector = kRamseteController;
}


int main()
{
	return frc::StartRobot<Robot>();
}