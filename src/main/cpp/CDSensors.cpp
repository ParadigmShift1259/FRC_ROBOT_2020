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

   using ds = rev::Rev2mDistanceSensorEx;

    m_ballpresent1 = 9.0;
    m_ballpresent2 = 4.0;
    m_ballpresent3 = 4.0;
    m_ballpresent4 = 7.0;

    // Connect to I2C through mux one on at a time
    MuxSelect(0);
    m_distsensor1 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                            , ds::DistanceUnit::kInches
                                            , ds::RangeProfile::kDefault
                                            , 0x54);

    MuxSelect(1);
    m_distsensor2 = new Rev2mDistanceSensorEx(ds::Port::kOnboard
                                            , ds::DistanceUnit::kInches
                                            , ds::RangeProfile::kDefault
                                            , 0x56);

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
    // MuxSelectAll();
    
	MuxSelectMask(0x0F);

    m_colorsensor = new ColorSensorV3(I2C::kOnboard);
}


CDSensors::~CDSensors()
{
}


void CDSensors::Init()
{
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
    //Color detectedColor = m_colorsensor->GetColor();

    //frc::SmartDashboard::PutNumber("Color R", detectedColor.red);
    //frc::SmartDashboard::PutNumber("Color G", detectedColor.green);
    //frc::SmartDashboard::PutNumber("Color B", detectedColor.blue);

    frc::SmartDashboard::PutBoolean("CDS5_Roller Ball Present", BallPresent(RollerSensor));
    frc::SmartDashboard::PutBoolean("CDS6_Chute 1 Ball Present", BallPresent(Chute1Sensor));
    frc::SmartDashboard::PutBoolean("CDS7_Chute 2 Ball Present", BallPresent(Chute2Sensor));
    frc::SmartDashboard::PutBoolean("CDS8_Feeder Ball Present", BallPresent(FeederSensor));

    Dashboard();
}


void CDSensors::Stop()
{
}


void CDSensors::Dashboard()
{
}

bool CDSensors::BallPresent(int sensornum)
{
  double dist;
  bool ballpresent = false;
  
  switch (sensornum)
  {
    case RollerSensor:
      dist = ReadDistance(*m_distsensor1, sensornum);
      if (dist <= m_ballpresent1)
        ballpresent = true;
      frc::SmartDashboard::PutNumber("CDS1_Roller Dist", dist);
      break;

    case Chute2Sensor:
      dist = ReadDistance(*m_distsensor2, sensornum);
      if (dist <= m_ballpresent2)
        ballpresent = true;  
      frc::SmartDashboard::PutNumber("CDS2_Chute2 Dist", dist);  
      break;

    case Chute1Sensor:
      dist = ReadDistance(*m_distsensor3, sensornum);
      if (dist <= m_ballpresent3)
        ballpresent = true;   
      frc::SmartDashboard::PutNumber("CDS3_Chute1 Dist", dist); 
      break;

    case FeederSensor:
      dist = ReadDistance(*m_distsensor4, sensornum);
      if (dist <= m_ballpresent4)
        ballpresent = true;    
      frc::SmartDashboard::PutNumber("CDS4_Feeder Dist", dist);
      break;

    default:
      break;
  }

  return ballpresent;
}


// Returns timestamp
double CDSensors::ReadDistance(Rev2mDistanceSensorEx& distSensor, int sensorNum) 
{
  double dist = 100.0;
  bool isValid = distSensor.IsRangeValid();

  if (isValid)
  {
    /**
     * The current measured range is returned from GetRange(). By default
     * this range is returned in inches.
     */
    dist = distSensor.GetRange();
  }

  return dist;
}

void CDSensors::MuxSelect(uint8_t muxPort)
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


void CDSensors::MuxSelectMask(uint8_t mask)
{ 
  uint8_t muxPortBit[2];
  muxPortBit[0] = mask;
  muxPortBit[1] = 0;
  bool bXferAborted = m_mux.WriteBulk(&muxPortBit[0], 1);

  char buf[100];
  sprintf(buf, "Mux select %d bXferAborted = %d", mask, bXferAborted);
  frc::DriverStation::ReportError(buf);
}


void CDSensors::MuxSelectAll()
{
  uint8_t muxPortBit[2];
  muxPortBit[0] = 0xFF;
  muxPortBit[1] = 0;
  bool bXferAborted = m_mux.WriteBulk(&muxPortBit[0], 1);

  char buf[100];
  sprintf(buf, "Mux select ALL bXferAborted = %d", bXferAborted);
  frc::DriverStation::ReportError(buf);
}
