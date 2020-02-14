/**
 *  Intake.cpp
 *  Date:
 *  Last Edited By:
 *  Jival.C
 */


#include "Intake.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/I2C.h>

using namespace std;


Intake::Intake(OperatorInputs *inputs, CDSensors *sensors)
{
	m_inputs = inputs;
    m_sensors = sensors;
    
    m_solenoid1 = nullptr;
    if (INT_SOLENOID != -1)
        m_solenoid1 = new Solenoid(INT_SOLENOID);

    m_motor1 = nullptr;
    if (INT_MOTOR1 != -1)
    {
        m_motor1 = new Spark(INT_MOTOR1);
    }

    m_motor2 = nullptr;
    if (INT_MOTOR2 != -1)
    {
        m_motor2 = new Spark(INT_MOTOR2);
    }
    
    m_ballcount = 0;
    m_drivingbecauseshooting = false;
}


Intake::~Intake()
{
    if (m_solenoid1 != nullptr)
        delete m_solenoid1;

    if (m_motor1 != nullptr)
        delete m_motor1;

    if (m_motor2 != nullptr)
        delete m_motor2;	  
}


bool Intake::NullCheck()
{
    if (m_solenoid1 == nullptr)
        return false;
    
    if (m_motor1 == nullptr)
        return false;

    if (m_motor2 == nullptr)
        return false;	
    
    return true;
}


void Intake::Init()
{
    if (!NullCheck())
        return;

    m_intakestate = kIdle;
    m_solenoid1->Set(false);
    m_ballcount = 0;
    m_drivingbecauseshooting = false;
}

// !!!!!!!!!!!!! Make sure the previous stage is acomplished to move on.
void Intake::Loop()
{
    if (!NullCheck())
        return;
    
    if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_solenoid1->Set(true);
    if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_solenoid1->Set(false);

    switch (m_intakestate)
    {
    case kIdle: 
        if (m_drivingbecauseshooting)
            m_intakestate = kGather;
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            m_intakestate = kGather;
        //if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        //   m_intakestate = kEject;
    
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
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            m_intakestate = kIdle;

        m_motor1->Set(-INT_INTAKE_ROLLER_SPEED);
        m_motor2->Set(-INT_INTAKE_WHEEL_SPEED);
        break;
    
    case kEject:
        if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            m_intakestate = kIdle;
    
        m_motor1->Set(INT_INTAKE_ROLLER_SPEED);
        m_motor2->Set(INT_INTAKE_WHEEL_SPEED);   
        break;
        
    default: 
        break;
    }

	Dashboard();
    CountBalls();
}


void Intake::Stop()
{
    if (!NullCheck())
        return;
    
    m_intakestate = kIdle;
}


void Intake::CountBalls()
{
    m_ballcount = 0;
    if (m_sensors->BallPresent(RollerSensor))
        m_ballcount++;
    if (m_sensors->BallPresent(Chute2Sensor))
        m_ballcount++;
    if (m_sensors->BallPresent(Chute1Sensor))
        m_ballcount++;
}


void Intake::Dashboard()
{
    if (!NullCheck())
        return;
    
    SmartDashboard::PutNumber("INT1_Ball Count", m_ballcount);
    SmartDashboard::PutNumber("INT2_Intake State", m_intakestate);
}

// Returns if the feeder should refresh
bool Intake::LoadRefresh()
{
    return (m_ballcount >= 2 && !m_drivingbecauseshooting);
}