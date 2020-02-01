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
	m_gyrodrive = new GyroDrive(m_operatorinputs);
	m_pneumatics = new Pneumatics();
	m_turret = new Turret(m_operatorinputs);
	m_controlpanel = new ControlPanel(m_operatorinputs);
	m_intake = new Intake(m_operatorinputs)
	m_feeder = new Feeder(m_operatorinputs, m_intake);
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
	m_gyrodrive->Init();
	m_pneumatics->Init();
	m_turret->Init();
	m_controlpanel->Init();
}


void Robot::TeleopPeriodic()
{
	m_gyrodrive->Loop();
	m_pneumatics->Loop();
	m_turret->Loop();
	m_controlpanel->Loop();
}


void Robot::DisabledInit()
{
	m_gyrodrive->Stop();
	m_pneumatics->Stop();
	m_turret->Stop();
	//m_controlpanel->Stop();
}


void Robot::DisabledPeriodic()
{
}


int main()
{
	return frc::StartRobot<Robot>();
}
