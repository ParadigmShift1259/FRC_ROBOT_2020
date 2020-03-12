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
#include <cameraserver/CameraServer.h>

#include <iostream>



bool Debug = false;
bool StartedInAuto = false;

AutoMode automode = kNoAuto;


void Robot::RobotInit()
{
	m_chooser.SetDefaultOption(kszNoAuto, kszNoAuto);
	m_chooser.AddOption(kszSimpleAuto, kszSimpleAuto);
	m_chooser.AddOption(kszDriveStraight, kszDriveStraight);
	m_chooser.AddOption(kszTrenchRun, kszTrenchRun);
	m_chooser.AddOption(kszCenterRendezvous, kszCenterRendezvous);
	m_chooser.AddOption(kszEnemyTrench, kszEnemyTrench);
	SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driverstation = &DriverStation::GetInstance();

	m_operatorinputs = new OperatorInputs();
	m_vision = new Vision(m_operatorinputs);
	m_gyrodrive = new GyroDrive(m_operatorinputs, m_vision);
	m_pneumatics = new Pneumatics();
	m_intake = new Intake(m_operatorinputs, m_vision);
	m_feeder = new Feeder(m_operatorinputs, m_intake);
	m_controlpanel = new ControlPanel(m_operatorinputs, m_gyrodrive, m_intake);
	m_climber = new Climber(m_operatorinputs);
	m_turret = new Turret(m_operatorinputs, m_gyrodrive, m_intake, m_feeder, m_controlpanel, m_climber, m_vision);
	m_autonomous = new Autonomous(m_gyrodrive, m_intake, m_feeder, m_turret, m_vision);

	//CameraServer::GetInstance()->StartAutomaticCapture();
}


void Robot::RobotPeriodic()
{
}


void Robot::AutonomousInit()
{
	ReadChooser();

	StartedInAuto = true;

	m_autonomous->Init();
	m_gyrodrive->Init();
	m_pneumatics->Init();
	m_vision->Init();
	m_intake->Init();
	m_feeder->Init();
	m_feeder->SetLoaded(true);		// ensure feeder state is loaded before loop runs
	m_turret->Init();
	//m_controlpanel->Init();
	m_climber->Init();
}


void Robot::AutonomousPeriodic()
{ 
	m_autonomous->Loop();
	m_gyrodrive->Loop();
	m_vision->Loop();
	m_intake->Loop();
	m_feeder->Loop();
	m_turret->Loop();
}


void Robot::TestInit()
{    
}


void Robot::TestPeriodic()
{
}


void Robot::TeleopInit()
{
	if (!StartedInAuto)
	{
		m_gyrodrive->Init();
		m_pneumatics->Init();
		m_vision->Init();
		m_intake->Init();
		m_feeder->Init();
		m_turret->Init();
		//m_controlpanel->Init();
		m_climber->Init();
	}
	else
	{
		m_feeder->SetStuffTime(3.0);
	}
}


void Robot::TeleopPeriodic()
{
	m_gyrodrive->Loop();
	m_pneumatics->Loop();
	m_vision->Loop();
	m_intake->Loop();
	m_feeder->Loop();
	m_turret->Loop();
	//m_controlpanel->Loop();
	m_climber->Loop();

	//if (m_operatorinputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 1))
	m_operatorinputs->xBoxRumble(1, 1);
}


void Robot::DisabledInit()
{
	if (!StartedInAuto)
	{
		m_gyrodrive->Stop();
		m_pneumatics->Stop();
		m_vision->Stop();
		m_intake->Stop();
		m_feeder->Stop();
		m_turret->Stop();
		//m_controlpanel->Stop();
		m_climber->Stop();
	}
	else
	{
		m_autonomous->Stop();
	}

	m_vision->Init();
}


void Robot::DisabledPeriodic()
{
	ReadChooser();
	m_vision->SetLED(false);
	m_vision->Loop();
}


void Robot::ReadChooser()
{
	m_autoSelected = m_chooser.GetSelected();

	automode = kNoAuto;
	if (m_autoSelected == kszNoAuto)
		automode = kNoAuto;
	else
	if (m_autoSelected == kszSimpleAuto)
		automode = kSimpleAuto;
	else
	if (m_autoSelected == kszDriveStraight)
		automode = kDriveStraight;
	else
	if (m_autoSelected == kszTrenchRun)
		automode = kTrenchRun;
	else
	if (m_autoSelected == kszCenterRendezvous)
		automode = kCenterRendezvous;
	else
	if (m_autoSelected == kszEnemyTrench)
		automode = kEnemyTrench;

	SmartDashboard::PutNumber("AU1_automode", automode);
}


int main()
{
	return frc::StartRobot<Robot>();
}
