/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include <iostream>
#include <string>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "Robot.h"


bool Debug = true;


void Robot::RobotInit()
{
    m_operatorinputs = new OperatorInputs();
    m_turret = new Turret(m_operatorinputs);
}


void Robot::RobotPeriodic(){}
void Robot::AutonomousInit(){}
void Robot::AutonomousPeriodic(){}
void Robot::TestInit(){}
void Robot::TestPeriodic(){}


void Robot::TeleopInit()
{
    m_turret->Init();
}


void Robot::TeleopPeriodic()
{
    m_turret->Loop();
}


void Robot::DisabledInit()
{
    //m_turret->Stop();
}


void Robot::DisabledPeriodic(){}


int main()
{
    return StartRobot<Robot>();
}