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

#include <iostream>



bool Debug = true;

void Robot::RobotInit()
{
	m_operatorinputs = new OperatorInputs();
	m_vision = new Vision(m_operatorinputs);
	m_gyrodrive = new GyroDrive(m_operatorinputs, m_vision);
	m_pneumatics = new Pneumatics();
	m_sensors = new CDSensors();
	m_intake = new Intake(m_operatorinputs, m_sensors);
	m_feeder = new Feeder(m_operatorinputs, m_intake, m_sensors);
	m_turret = new Turret(m_operatorinputs, m_intake, m_feeder, m_vision, m_gyrodrive);
	//m_controlpanel = new ControlPanel(m_operatorinputs, m_sensors, m_gyrodrive);
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
	m_sensors->Init();
	m_vision->Init();
	m_intake->Init();
	m_feeder->Init();
	m_turret->Init();
	//m_controlpanel->Init();
}


void Robot::TeleopPeriodic()
{
	m_gyrodrive->Loop();
	m_pneumatics->Loop();
	m_sensors->Loop();
	m_vision->Loop();
	m_intake->Loop();
	m_feeder->Loop();
	m_turret->Loop();
	//m_controlpanel->Loop();
	
}


void Robot::DisabledInit()
{
	m_gyrodrive->Stop();
	m_pneumatics->Stop();
	m_sensors->Stop();
	m_vision->Stop();
	m_intake->Stop();
	m_feeder->Stop();
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
