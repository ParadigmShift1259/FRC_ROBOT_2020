/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <math.h>



Robot::Robot()
{
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  m_colorSensor = new rev::ColorSensorV3(i2cPort);
}


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
   double IR = m_colorSensor->GetIR();
    frc::Color detectedColor = m_colorSensor->GetColor();
    uint32_t proximity = m_colorSensor->GetProximity();
  double K = 1 - fmax(fmax(detectedColor.red,detectedColor.blue),detectedColor.green);
  double C = (1-detectedColor.red-K)/(1-K);
  double M = (1-detectedColor.green-K)/(1-K);
  double Y = (1-detectedColor.blue-K)/(1-K);
  int color = 0; // 1-4 Yellow, Green, Blue, Red
  if (C < .201){
     color = 4;
  }
  else if (Y < .201){
     color = 3;
  }
  else if (M < .200  && C > Y ){
     color = 2;
  }
  else if (M < .200  && Y > C ){
     color = 1;
  }
    frc::SmartDashboard::PutNumber("CS1_K", K );
    frc::SmartDashboard::PutNumber("CS2_C", C);
    frc::SmartDashboard::PutNumber("CS3_M", M);
    frc::SmartDashboard::PutNumber("CS4_Y", Y);
    frc::SmartDashboard::PutNumber("CS5_IR", IR);
    frc::SmartDashboard::PutNumber("CS6_Proximity",proximity);
    frc::SmartDashboard::PutNumber("CS7_Color",color);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
