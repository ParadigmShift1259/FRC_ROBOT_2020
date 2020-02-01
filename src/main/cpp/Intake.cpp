/**
 *  Intake.cpp
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#include "Intake.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <rev/Rev2mDistanceSensor.h>
#include <frc/I2C.h>

using namespace std;
using namespace rev;


Intake::Intake(OperatorInputs *inputs, Feeder *feeder)
{
	m_inputs = inputs;
    m_feeder = feeder; 
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
    
    m_ballcount = 0;
    m_drivingbecauseshooting = false;
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

    m_intakestate = kIdle;
    m_solenoid1->Set(false);
    m_solenoid2->Set(false);
    m_ballcount = 0;
    m_drivingbecauseshooting = false;
}

// !!!!!!!!!!!!! Make sure the previous stage is acomplished to move on.
void Intake::Loop()
{
    if (!NullCheck())
        return;

    if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_intakestate = kGather;
    if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_intakestate = kIdle;

    switch (m_intakestate)
    {
    case kIdle: 
        if (m_drivingbecauseshooting)
            m_intakestate = kGather;

        m_motor1->Set(0.0);
        m_motor2->Set(0.0);
        break;

    case kGather: 
        if ((m_ballcount == 0) && m_drivingbecauseshooting)
        {
            m_drivingbecauseshooting = false;
            m_intakestate = kIdle;
        }
        if ((m_ballcount >= 3) && !m_drivingbecauseshooting)
            m_intakestate = kIdle;

        m_motor1->Set(0.5);
        m_motor2->Set(0.5);
        break;

    default: 
        break;
    }

	Dashboard();
}


void Intake::Stop()
{
    if (!NullCheck())
        return;
    
    m_intakestate = kIdle;
    m_solenoid1->Set(false);
    m_solenoid2->Set(false);
}


void Intake::Dashboard()
{
    if (!NullCheck())
        return;
}


int Intake:: BallCount()
{
    if (m_sensor1->GetRange() <= snsrDst && m_sensor2->GetRange() > snsrDst && m_sensor3->GetRange() > snsrDst)
    {
        return 1;
    }

    if (m_sensor1->GetRange() <= snsrDst && m_sensor2->GetRange() <= snsrDst && m_sensor3->GetRange() > snsrDst)
    {
        return 2;
    }

    if (m_sensor1->GetRange() <= snsrDst && m_sensor2->GetRange() <= snsrDst && m_sensor3->GetRange() <= snsrDst)
    {
        return 3;
    }
}


void Intake::DstncSnsrModeSet(Rev2mDistanceSensor *temp)
{
    m_sensormode = temp;
    m_sensormode->SetRangeProfile(Rev2mDistanceSensor::RangeProfile::kHighSpeed);
}
//->SetRangeProfile(rev::Rev2mDistanceSensor::RangeProfile::kHighAccuracy);