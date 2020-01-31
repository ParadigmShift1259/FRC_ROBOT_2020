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
    m_drivestraight->Init();
}


void Robot::TeleopPeriodic()
{
    m_drivetrain->ReportData();
    m_drivestraight->ConfigureProfile();
    m_drivestraight->ConfigureGyroPID();
    m_drivestraight->ConfigureEncoderPID();
    

    // choosing between the two determines teleop or auto at the moment
    //m_drivetrain->Loop();
    m_drivestraight->Loop();
}


void Robot::DisabledInit()
{
}


void Robot::DisabledPeriodic()
{
}


int main()
{
	return frc::StartRobot<Robot>();
}
