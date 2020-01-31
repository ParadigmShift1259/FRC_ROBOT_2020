/**
 *  Intake.cpp
 *  Date:
 *  Last Edited By:
 */


#include "Intake.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <rev/Rev2mDistanceSensor.h>
#include <frc/I2C.h>

using namespace std;
using namespace rev;


Intake::Intake(OperatorInputs *inputs)
{
	m_inputs = inputs;

    m_solenoid1 = nullptr;
    if (INT_SOLENOID1 != -1)
        m_solenoid1 = new Solenoid(INT_SOLENOID1);

    m_solenoid2 = nullptr;
    if (INT_SOLENOID2 != -1)
        m_solenoid2 = new Solenoid(INT_SOLENOID2);

    m_motor1 = nullptr;
    if (INT_MOTOR1 != -1)
        m_motor1 = new Spark(INT_MOTOR1);

    m_motor2 = nullptr;
    if (INT_MOTOR2 != -1)
        m_motor2 = new Spark(INT_MOTOR2);
   
    m_sensor1 = nullptr;
    if (INT_SENSOR1 != -1)
        m_sensor1 = new Rev2mDistanceSensor(Rev2mDistanceSensor::Port::kOnboard, Rev2mDistanceSensor::DistanceUnit::kInches);
        DstncSnsrModeSet(m_sensor1);

    m_sensor2 = nullptr;
    if (INT_SENSOR2 != -1)
        m_sensor2 = new Rev2mDistanceSensor(Rev2mDistanceSensor::Port::kOnboard, Rev2mDistanceSensor::DistanceUnit::kInches);
        DstncSnsrModeSet(m_sensor2);

    m_sensor3 = nullptr;
    if (INT_SENSOR3 != -1)
        m_sensor3 = new Rev2mDistanceSensor(Rev2mDistanceSensor::Port::kOnboard, Rev2mDistanceSensor::DistanceUnit::kInches);
        DstncSnsrModeSet(m_sensor3);
    
}


Intake::~Intake()
{
    if (m_solenoid1 != nullptr)
        delete m_solenoid1;

    if (m_solenoid2 != nullptr)
        delete m_solenoid2;

    if (m_motor1 != nullptr)
        delete m_motor1;

    if (m_motor2 != nullptr)
        delete m_motor2;	

    if (m_sensor1 != nullptr)
        delete m_sensor1;	   
    
    if (m_sensor2 != nullptr)
        delete m_sensor1;	
    
    if (m_sensor3 != nullptr)
        delete m_sensor1;	   
}

bool Intake::NullCheck()
{
    if (m_solenoid1 == nullptr)
        return false;

    if (m_solenoid2 == nullptr)
        return false;

    if (m_motor1 == nullptr)
        return false;

    if (m_motor2 == nullptr)
        return false;	

    if (m_sensor1 == nullptr)
        return false;	   
    
    if (m_sensor2 == nullptr)
        return false;	
    
    if (m_sensor3 == nullptr)
        return false;
    return true;
}


void Intake::Init()
{
    if (!NullCheck())
        return;
}


void Intake::Loop()
{
    if (!NullCheck())
        return;
	Dashboard();
}


void Intake::Stop()
{
    if (!NullCheck())
        return;
}


void Intake::Dashboard()
{
    if (!NullCheck())
        return;
}


bool Intake::Sensor1Chk()
{ 
  if (m_sensor1-> GetRange() <= snsrDst)  
  {
      timerSnsr1 = frc::Timer::GetFPGATimestamp	();	
      if(frc::Timer::GetFPGATimestamp() - timerSnsr1 > 0.20 && m_sensor1-> GetRange() <= snsrDst)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
  else 
  {
    return false;
  }
}

bool Intake::Sensor2Chk()
{ 
  if (m_sensor2-> GetRange() <= snsrDst)  
  {
      timerSnsr2 = frc::Timer::GetFPGATimestamp	();	
      if(frc::Timer::GetFPGATimestamp() - timerSnsr2 > 0.20 && m_sensor1-> GetRange() <= snsrDst)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
  else 
  {
    return false;
  }  
}

bool Intake::Sensor3Chk()
{ 
   if (m_sensor3-> GetRange() <= snsrDst)  
  {
      timerSnsr3 = frc::Timer::GetFPGATimestamp	();	
      if(frc::Timer::GetFPGATimestamp() - timerSnsr3 > 0.20 && m_sensor1-> GetRange() <= snsrDst)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
  else 
  {
    return false;
  } 
}

void Intake::DstncSnsrModeSet(Rev2mDistanceSensor *temp)
{
    m_sensormode = temp;
    m_sensormode->SetRangeProfile(Rev2mDistanceSensor::RangeProfile::kHighSpeed);
}
//->SetRangeProfile(rev::Rev2mDistanceSensor::RangeProfile::kHighAccuracy);