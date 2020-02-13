/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

const bool c_DistSensorOn = true;
const bool c_DistSensorOff = false;


Robot::Robot()
  : m_mux(I2C::kOnboard, 0x70)
  //, m_reAddresser(I2C::kOnboard, 0x29)
{
  //m_out1 = nullptr;
  //m_out2 = nullptr;
  m_distsensor1 = nullptr;
  m_distsensor2 = nullptr;
}

void Robot::RobotInit()
{
  frc::DriverStation::ReportError("RobotInit()");

  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //MuxSelect(2);
  //m_colorsensor = new ColorSensorV3(I2C::kOnboard);

  // bool bXferAborted = m_reAddresser.AddressOnly();
  // char buf[100];
  // sprintf(buf, "Re-address AddressOnly bXferAborted = %d", bXferAborted);
  // frc::DriverStation::ReportError(buf);

  // Re-address
  // uint8_t reg = 0x8a;
  // uint8_t newAddr = 0x54; // new addr 0x54 / 2 = 0x2A
  // uint8_t muxPortBit[3];
  // //muxPortBit[0] = 0x00;
  // muxPortBit[0] = reg;
  // muxPortBit[1] = newAddr;
  // bXferAborted = m_reAddresser.WriteBulk(&muxPortBit[0], 2);
  // //bool bXferAborted = m_reAddresser.Write(reg, newAddr);
  // sprintf(buf, "Re-address bXferAborted = %d", bXferAborted);
  // frc::DriverStation::ReportError(buf);

  //MuxSelect(0);
 
#if 0
  m_out1 = new DigitalOutput(0);
  m_out2 = new DigitalOutput(1);
  
  // Turn all off first
  m_out1->Set(c_DistSensorOff);
  m_out2->Set(c_DistSensorOff);
#endif

#if 0
  using ds = rev::Rev2mDistanceSensorEx;
  const double delay = 0.150; // 10 ms
  Wait(delay);  // Keep them off for a bit

  // Turn one on at a time
  m_out1->Set(true);
  m_distsensor1 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                          , ds::DistanceUnit::kInches
                                          , ds::RangeProfile::kDefault
                                          , 0x56);

  m_out2->Set(true);
  m_distsensor2 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                          , ds::DistanceUnit::kInches
                                          , ds::RangeProfile::kDefault
                                          , 0x54);

   m_distsensor1->SetEnabled(true);
   m_distsensor2->SetEnabled(true);
   m_distsensor1->SetAutomaticMode(true);
   m_distsensor2->SetAutomaticMode(true);
#endif
  //MuxSelect(0);
  //m_distsensor2 = new Rev2mDistanceSensor(rev::Rev2mDistanceSensor::Port::kOnboard, rev::Rev2mDistanceSensor::DistanceUnit::kInches);
  //m_distsensor2->SetAutomaticMode(true);
  //m_distsensor2->SetEnabled(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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

void Robot::TeleopInit() 
{
  using ds = rev::Rev2mDistanceSensorEx;

  // Connect to I2C through mux one on at a time
  MuxSelect(0);
  m_distsensor1 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                          , ds::DistanceUnit::kInches
                                          , ds::RangeProfile::kDefault
                                          , 0x56);

  MuxSelect(1);
  m_distsensor2 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                          , ds::DistanceUnit::kInches
                                          , ds::RangeProfile::kDefault
                                          , 0x54);

  MuxSelect(2);
  m_distsensor3 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                          , ds::DistanceUnit::kInches
                                          , ds::RangeProfile::kDefault
                                          , 0x58);

  MuxSelect(3);
  m_distsensor4 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                          , ds::DistanceUnit::kInches
                                          , ds::RangeProfile::kDefault
                                          , 0x5A);

  // Connect all ports on the I2C bus                                          
  MuxSelectAll();

  m_colorsensor = new ColorSensorV3(I2C::kOnboard);

  m_distsensor1->SetEnabled(true);
  m_distsensor2->SetEnabled(true);
  m_distsensor3->SetEnabled(true);
  m_distsensor4->SetEnabled(true);

  // Only need to tell one of the sensors to use automatic mode
  m_distsensor1->SetAutomaticMode(true);
}

void Robot::TeleopPeriodic()
{
  //double IR = m_colorsensor->GetIR();
  Color detectedColor = m_colorsensor->GetColor();
  frc::SmartDashboard::PutNumber("Color R", detectedColor.red);
  frc::SmartDashboard::PutNumber("Color G", detectedColor.green);
  frc::SmartDashboard::PutNumber("Color B", detectedColor.blue);

  static double prevTimestamp1 = 0.0;
  static double prevTimestamp2 = 0.0;
  static double prevTimestamp3 = 0.0;
  static double prevTimestamp4 = 0.0;
  double newTimestamp = ReadDistance(*m_distsensor1, 1);
  frc::SmartDashboard::PutNumber("Sensor 1 Time delta", newTimestamp - prevTimestamp1);
  prevTimestamp1 = newTimestamp;

  newTimestamp = ReadDistance(*m_distsensor2, 2);
  frc::SmartDashboard::PutNumber("Sensor 2 Time delta", newTimestamp - prevTimestamp2);
  prevTimestamp2 = newTimestamp;

  newTimestamp = ReadDistance(*m_distsensor3, 3);
  frc::SmartDashboard::PutNumber("Sensor 3 Time delta", newTimestamp - prevTimestamp3);
  prevTimestamp3 = newTimestamp;

  newTimestamp = ReadDistance(*m_distsensor4, 4);
  frc::SmartDashboard::PutNumber("Sensor 4 Time delta", newTimestamp - prevTimestamp4);
  prevTimestamp4 = newTimestamp;
}

void Robot::DisabledInit()
{
  if (m_distsensor1 != nullptr)
  {
    m_distsensor1->SetAutomaticMode(false);
    delete m_distsensor1;
    m_distsensor1 = nullptr;
  }

  if (m_distsensor2 != nullptr)
  {
    m_distsensor2->SetAutomaticMode(false);
    delete m_distsensor2;
    m_distsensor2 = nullptr;
  }

  if (m_distsensor3 != nullptr)
  {
    m_distsensor3->SetAutomaticMode(false);
    delete m_distsensor3;
    m_distsensor3 = nullptr;
  }

  if (m_distsensor4 != nullptr)
  {
    m_distsensor4->SetAutomaticMode(false);
    delete m_distsensor4;
    m_distsensor4 = nullptr;
  }
}

// Returns timestamp
double Robot::ReadDistance(Rev2mDistanceSensorEx& distSensor, int sensorNum) 
{
  bool isValid = distSensor.IsRangeValid();

  char buf[100];
  sprintf(buf, "Data %d Valid", sensorNum);
  frc::SmartDashboard::PutBoolean(buf, isValid);
  //printf(buf, "Data %d Valid %d\n", sensorNum, isValid);
  //frc::DriverStation::ReportError(buf);

  double timestamp;
  if (isValid)
  {
    /**
     * The current measured range is returned from GetRange(). By default
     * this range is returned in inches.
     */
    sprintf(buf, "Distance %d (in)", sensorNum);
    auto dist = distSensor.GetRange();
    frc::SmartDashboard::PutNumber(buf, dist);
    printf("Distance %d (in) %.3f\n", sensorNum, dist);
    //frc::DriverStation::ReportError(buf);

    /**
     * The timestamp of the last valid measurement (measured in seconds since 
     * the program started), is returned by GetTimestamp().
     */
    sprintf(buf, "Timestamp %d", sensorNum);
    timestamp = distSensor.GetTimestamp();
    frc::SmartDashboard::PutNumber(buf, timestamp);
    printf("Timestamp %d (in) %.3f\n", sensorNum, timestamp);
    //sprintf(buf, "Timestamp %d (in) %.3f", sensorNum, timestamp);
    //frc::DriverStation::ReportError(buf);
  }
  else 
  {
    sprintf(buf, "Distance %d (in)", sensorNum);
    frc::SmartDashboard::PutNumber(buf, -1);
    sprintf(buf, "Timestamp %d", sensorNum);
    timestamp = distSensor.GetTimestamp();
    frc::SmartDashboard::PutNumber(buf, timestamp);
  }

  return timestamp;
}

void Robot::TestPeriodic() {}

void Robot::MuxSelect(uint8_t muxPort)
{
  if (muxPort > 7)
  {
    frc::DriverStation::ReportError("Specify a mux port between 0 and 7");
    return;
  }
 
  uint8_t muxPortBit[2];
  muxPortBit[0] = 1 << muxPort;
  muxPortBit[1] = 0;
  bool bXferAborted = m_mux.WriteBulk(&muxPortBit[0], 1);

  char buf[100];
  sprintf(buf, "Mux select %d bXferAborted = %d", muxPort, bXferAborted);
  frc::DriverStation::ReportError(buf);
}

void Robot::MuxSelectAll()
{
  uint8_t muxPortBit[2];
  muxPortBit[0] = 0xFF;
  muxPortBit[1] = 0;
  bool bXferAborted = m_mux.WriteBulk(&muxPortBit[0], 1);

  char buf[100];
  sprintf(buf, "Mux select ALL bXferAborted = %d", bXferAborted);
  frc::DriverStation::ReportError(buf);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
