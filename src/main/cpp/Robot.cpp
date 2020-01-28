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


void Robot::TestInit()
{    
}


void Robot::TestPeriodic()
{
}


void Robot::TeleopInit()
{
}


void Robot::TeleopPeriodic()
{
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
