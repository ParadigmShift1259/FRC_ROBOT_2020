/**
 *  CDSensors.cpp
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#include "CDSensors.h"
#include "Const.h"

#include <iostream>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

const bool c_DistSensorOn = true;
const bool c_DistSensorOff = false;


using namespace std;


CDSensors::CDSensors()
  : m_mux(I2C::kOnboard, 0x70)
{	
    m_distsensor1 = nullptr;
    m_distsensor2 = nullptr;
    m_distsensor3 = nullptr;
    m_distsensor4 = nullptr;
}


CDSensors::~CDSensors()
{        
}


void CDSensors::Init()
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



void CDSensors::Loop()
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
    Dashboard();
}


void CDSensors::Stop()
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


void CDSensors::Dashboard()
{
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
