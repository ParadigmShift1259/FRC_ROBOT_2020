/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Logger.h"

#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>

//#include <iostream>

bool Debug = true;
Logger *g_log = nullptr;

void Robot::RobotInit()
{
	g_log = new Logger("/tmp/logfile.csv", true);
	m_operatorinputs = new OperatorInputs();
	m_vision = new Vision(m_operatorinputs);
	m_gyrodrive = new GyroDrive(m_operatorinputs, m_vision);
	m_pneumatics = new Pneumatics();
	m_intake = new Intake(m_operatorinputs);
	m_feeder = new Feeder(m_operatorinputs, m_intake);
	m_turret = new Turret(m_operatorinputs, m_intake, m_feeder, m_vision, m_gyrodrive);
	m_controlpanel = new ControlPanel(m_operatorinputs, m_gyrodrive);
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
	g_log->openLog("/tmp/logfile.csv");
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
	g_log->openLog("/tmp/logfile.csv");
	m_gyrodrive->Init();
	m_pneumatics->Init();
	m_vision->Init();
	m_intake->Init();
	m_feeder->Init();
	m_turret->Init();
	m_controlpanel->Init();
}

void Robot::TeleopPeriodic()
{
	m_gyrodrive->Loop();
	m_pneumatics->Loop();
	m_vision->Loop();
	m_intake->Loop();
	m_feeder->Loop();
	m_turret->Loop();
	m_controlpanel->Loop();
}

void Robot::DisabledInit()
{
	m_gyrodrive->Stop();
	m_pneumatics->Stop();
	m_vision->Stop();
	m_intake->Stop();
	m_feeder->Stop();
	m_turret->Stop();
	m_controlpanel->Stop();
	g_log->closeLog();
}

void Robot::DisabledPeriodic()
{
}

int main()
{
	return frc::StartRobot<Robot>();
}
