/**
 *  Intake.cpp
 *  Date:
 *  Last Edited By:
 *  Jival.C
 */


#include "Intake.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace std;


Intake::Intake(OperatorInputs *inputs)
{
	m_inputs = inputs;
    
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
    
    m_rollersensor = new DigitalInput(INT_ROLLER_SENSOR);
    m_chutesensor = new DigitalInput(INT_CHUTE_SENSOR);

    m_ballcount = 0;
    m_stuffing = false;
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
    m_ballstate = kZero;
    m_solenoid1->Set(true);
    m_ballcount = 0;
    m_stuffing = false;
    m_gathering = false;
    m_timer.Reset();
    m_timer.Start();
    m_balltimer.Reset();
    m_balltimer.Start();
}


void Intake::Loop()
{
    if (!NullCheck())
        return;
    
    IntakePosition();
    CountBalls();

    switch (m_intakestate)
    {
    case kIdle:
        // If stuffing, then start stuffing
        if (m_stuffing)
        {
            m_timer.Reset();
            m_motor1->Feed();
            m_motor2->Feed();
            m_intakestate = kStuff;
        }
        else
        // If manual input, then start gathering
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL) &&
            !m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
        {
            m_gathering = true;
            m_intakestate = kGather;
            m_solenoid1->Set(true);
            m_intakeposition = kDown;
            m_motor1->Feed();
            m_motor2->Feed();
        }
        else
        // If previously gathering and less balls, go back to gathering
        if (m_gathering && (m_ballcount < 3))
        {
            m_intakestate = kGather;
            m_solenoid1->Set(true);
            m_intakeposition = kDown;
            m_motor1->Feed();
            m_motor2->Feed();
        }
        // Otherwise, stop all motors
        else
        {
            m_motor1->Set(0);
            m_motor2->Set(0);
        }
        break;

    case kGather:
        // if we have three balls, back to idle, but maintain gathering
        if (m_ballcount >= 3)
        {
            m_intakestate = kIdle;
            m_motor1->Set(0);
            m_motor2->Set(0);
        }
        else
        // manual override to stop gathering
        if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL) &&
            !m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
        {
            m_gathering = false;
            m_intakestate = kIdle;
            m_motor1->Set(0);
            m_motor2->Set(0);
        }
        // Otherwise, gather the balls
        else
        {
            m_motor1->Set(INT_INTAKE_ROLLER_SPEED);
            m_motor2->Set(INT_INTAKE_WHEEL_SPEED);
        }
        break;

    case kStuff:
        // If a timer reaches a certain period, stop stuffing
        if (m_timer.Get() > INT_STUFF_TIME)
        {
            m_motor1->Set(0);
            m_motor2->Set(0);
            m_stuffing = false;
            m_intakestate = kIdle;
        }
        // Otherwise, continue stuffing balls out
        else
        {
            m_motor1->Set(INT_INTAKE_ROLLER_SPEED);
            m_motor2->Set(INT_INTAKE_WHEEL_SPEED);
        }
        break;
        
    default: 
        break;
    }

    // manual override intake
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
    {
        m_motor1->Set(INT_INTAKE_ROLLER_SPEED);
        m_motor2->Set(INT_INTAKE_WHEEL_SPEED);
    }
    else
    // manual override reverse
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
    {
        m_motor1->Set(-1.0 * INT_INTAKE_ROLLER_SPEED);
        m_motor2->Set(-1.0 * INT_INTAKE_WHEEL_SPEED);
    }

	Dashboard();
}


void Intake::Stop()
{
    if (!NullCheck())
        return;
    
    m_intakestate = kIdle;
    m_ballstate = kZero;
    m_motor1->Set(0);
    m_motor2->Set(0);
    m_ballcount = 0;
    m_stuffing = false;
    m_gathering = false;
}


void Intake::SetStuffing(bool stuff)
{
    m_stuffing = stuff;
}


bool Intake::IsStuffing()
{
    return m_stuffing;
}


void Intake::CountBalls()
{
    switch (m_ballstate)
    {
    case kZero:
        // if something is detected in chute, start timer for ball validity
        if (m_chutesensor->Get())
        {
            m_balltimer.Reset();
            m_ballstate = kTwoCheck;
        }
        else
            m_ballcount = 0;
        break;
    
    case kTwoCheck:
        // if nothing in the chute, no balls in system
        if (!m_chutesensor->Get())
        {
            m_ballstate = kZero;
        }
        else
        // if timer passes, register two balls
        if (m_balltimer.Get() > INT_BALL_CHECK_TIME)
        {
            m_ballstate = kTwo;
        }
        break;

    case kTwo:
        // if nothing in the chute, no balls in system
        if (!m_chutesensor->Get())
        {
            m_ballstate = kZero;
        }
        else
        // if ball in the rollers, three balls in system
        if (m_rollersensor->Get())
        {
            m_ballstate = kThree;
        }
        else
            m_ballcount = 2;
        break;
    
    case kThree:
        // if nothing in the chute, no balls in system
        if (!m_chutesensor->Get())
        {
            m_ballstate = kZero;
        }
        else
        // if nothing in the rollers, two balls in system
        if (!m_rollersensor->Get())
        {
            m_ballstate = kTwo;
        }
        else
            m_ballcount = 3;
        break;
    }
}


void Intake::IntakePositionLoop()
{
    switch (m_intakeposition)
    {
        case kDown:
            if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_intakeposition = kUp;
                m_solenoid1->Set(false);
            }
            break;
        
        case kUp:
            if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_intakeposition = kDown;
                m_solenoid1->Set(true);
            }
            break;
    }
}


bool Intake::IntakeUp()
{
    return (m_intakeposition == kUp);
}


void Intake::Dashboard()
{
    if (!NullCheck())
        return;
    
    SmartDashboard::PutNumber("INT1_Ball Count", m_ballcount);
    SmartDashboard::PutNumber("Roller Sensor", m_rollersensor->Get());
    SmartDashboard::PutNumber("Chute Sensor", m_chutesensor->Get());
    SmartDashboard::PutNumber("INT2_Intake State", m_intakestate);
}


// Returns if the feeder should refresh
bool Intake::CanRefresh()
{
    // if chute has 2 balls, then we can refresh
    return (m_ballcount >= 2);
}