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
    m_stuffingbecauseshooting = false;
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
    m_intakeposition = kDown;
    m_solenoid1->Set(true);
    m_ballcount = 0;
    m_stuffingbecauseshooting = false;
    m_intakeup = false;
}

// !!!!!!!!!!!!! Make sure the previous stage is acomplished to move on.
void Intake::Loop()
{
    if (!NullCheck())
        return;
    
    CountBalls();

    switch (m_intakestate)
    {
    case kIdle: 
        if (m_stuffingbecauseshooting)
        {
            m_intakestate = kStuff;
            m_motor1->Feed();
            m_motor2->Feed();
        }
        else
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_intakestate = kGather;
            m_solenoid1->Set(true);
            m_intakeposition = kDown;
            m_motor1->Feed();
            m_motor2->Feed();
        }
        else
        {
            m_motor1->Set(0.0);
            m_motor2->Set(0.0);
        }
        break;

    case kGather: 
        if ((m_ballcount >= 3) && m_sensors->BallPresent(FeederSensor))
        {
            m_intakestate = kIdle;
            m_motor1->StopMotor();
            m_motor2->StopMotor();
        }
        else
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_intakestate = kIdle;
            m_motor1->StopMotor();
            m_motor2->StopMotor();
        }
        else
        {
            m_motor1->Set(-INT_INTAKE_ROLLER_SPEED);
            m_motor2->Set(-INT_INTAKE_WHEEL_SPEED);
        }
        break;

    // when shooting, intake will cycle between kStuff and kIdle until feeder decides it's done
    case kStuff:
        if ((m_ballcount == 0))
        {
            m_stuffingbecauseshooting = false;
            m_intakestate = kIdle;
            m_motor1->Feed();
            m_motor2->Feed();
        }
        else
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_intakestate = kIdle;
            m_motor1->Feed();
            m_motor2->Feed();
        }
        else
        {
            m_motor1->Set(-INT_INTAKE_ROLLER_SPEED);
            m_motor2->Set(-INT_INTAKE_WHEEL_SPEED);  
        }
        break;
        
    default: 
        break;
    }

    if (m_inputs->xBoxRightBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
    {
        m_motor1->Set(-INT_INTAKE_ROLLER_SPEED);
        m_motor2->Set(-INT_INTAKE_WHEEL_SPEED);
    }
    else
    if (m_inputs->xBoxRightTrigger(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
    {
        m_motor1->Set(INT_INTAKE_ROLLER_SPEED);
        m_motor2->Set(INT_INTAKE_WHEEL_SPEED);
    }

	Dashboard();
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


Intake::IntakePosition Intake::BringingIntakeUp(bool ready)
{

    switch (m_intakeposition)
    {
        case kDown:
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_intakeposition = kBringingUp;
            }
            break;
        
        case kBringingUp:
            if (ready)
            {
                m_solenoid1->Set(false);
                m_intakeposition = kUp;
            }
            break;
        
        case kUp:

            break;
    }

    return m_intakeposition;
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
    return (m_sensors->BallPresent(Chute1Sensor) && m_sensors->BallPresent(Chute2Sensor) && !m_stuffingbecauseshooting);
}