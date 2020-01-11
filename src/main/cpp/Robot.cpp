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
    m_drivetrain = new Drivetrain(m_operatorinputs);
}


void Robot::RobotPeriodic(){}
void Robot::AutonomousInit(){}
void Robot::AutonomousPeriodic(){}
void Robot::TestInit(){}
void Robot::TestPeriodic(){}


void Robot::TeleopInit()
{
    m_drivetrain->Init();
    //m_drivetrain->Loop();
}


void Robot::TeleopPeriodic()
{
    m_drivetrain->Loop();
}


void Robot::DisabledInit()
{
    m_drivetrain->Stop();
}


void Robot::DisabledPeriodic(){}


int main()
{
    return frc::StartRobot<Robot>();
}