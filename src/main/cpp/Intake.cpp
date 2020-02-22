/**
 *  Intake.cpp
 *  Date: 2/20/2020
 *  Last Edited By: Geoffrey Xue
 */


#include "Intake.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


using namespace std;


Intake::Intake(OperatorInputs *inputs)
{
    if (INT_ENABLED != 1)
    {
        DriverStation::ReportError("Intake Not Enabled");
    }

	m_inputs = inputs;
    
    m_solenoid = nullptr;
    m_rollermotor = nullptr;
    m_wheelmotor = nullptr;
    
    m_rollersensor = nullptr;
    m_chutesensor = nullptr;

    m_ballcount = 0;
    m_stuffing = false;
    m_gathering = false;
}


Intake::~Intake()
{
    if (m_solenoid != nullptr)
        delete m_solenoid;

    if (m_rollermotor != nullptr)
        delete m_rollermotor;

    if (m_wheelmotor != nullptr)
        delete m_wheelmotor;	 

    if (m_rollersensor != nullptr) 
        delete m_rollersensor;

    if (m_chutesensor != nullptr)
        delete m_chutesensor; 
}


bool Intake::NullCheck()
{
    if (m_solenoid == nullptr)
        return false;
    
    if (m_rollermotor == nullptr)
        return false;

    if (m_wheelmotor == nullptr)
        return false;	
    
    if (m_rollersensor == nullptr)
        return false;
    
    if (m_chutesensor == nullptr)
        return false;

    return true;
}


void Intake::Init()
{
    if ((m_solenoid == nullptr) && (INT_SOLENOID != -1))
        m_solenoid = new Solenoid(INT_SOLENOID);
    if ((m_rollermotor == nullptr) && (INT_ROLLER_MOTOR != -1))
        m_rollermotor = new Spark(INT_ROLLER_MOTOR);
    if ((m_wheelmotor == nullptr) && (INT_WHEEL_MOTOR != -1))
        m_wheelmotor = new Spark(INT_WHEEL_MOTOR);
    if ((m_rollersensor == nullptr) && (INT_ROLLER_SENSOR != -1))
        m_rollersensor = new DigitalInput(INT_ROLLER_SENSOR);
    if ((m_chutesensor == nullptr) && (INT_CHUTE_SENSOR != -1))
        m_chutesensor = new DigitalInput(INT_CHUTE_SENSOR);

    if (!NullCheck())
        return;

    m_intakestate = kIdle;
    m_intakeposition = kDown;
    m_ballstate = kZero;
    m_solenoid->Set(true);
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
    
    IntakePositionLoop();
    CountBalls();

    switch (m_intakestate)
    {
    case kIdle:
        // If stuffing, then start stuffing
        if (m_stuffing)
        {
            m_timer.Reset();
            m_gathering = false;
            m_rollermotor->Feed();
            m_wheelmotor->Feed();
            m_intakestate = kStuff;
        }
        else
        // If manual input, then start gathering
        if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL) &&
            !m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
        {
            m_gathering = true;
            m_intakestate = kGather;
            m_solenoid->Set(true);
            m_intakeposition = kDown;
            m_rollermotor->Feed();
            m_wheelmotor->Feed();
        }
        else
        // if B button is pressed, ensure gathering is false
        if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_gathering = false;
        }
        else
        // If previously gathering and less balls, go back to gathering
        if (m_gathering && (m_ballcount < 3))
        {
            m_intakestate = kGather;
            m_solenoid->Set(true);
            m_intakeposition = kDown;
            m_rollermotor->Feed();
            m_wheelmotor->Feed();
        }
        // Otherwise, stop all motors
        else
        {
            m_rollermotor->Set(0);
            m_wheelmotor->Set(0);
        }
        break;

    case kGather:
        // if we have three balls, back to idle, but maintain gathering
        if (m_ballcount >= 3)
        {
            m_intakestate = kIdle;
            m_rollermotor->Set(0);
            m_wheelmotor->Set(0);
        }
        else
        // manual override to stop gathering
        if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL) &&
            !m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
        {
            m_gathering = false;
            m_intakestate = kIdle;
            m_rollermotor->Set(0);
            m_wheelmotor->Set(0);
        }
        // Otherwise, gather the balls
        else
        // if ball count is one less from full, slow down speed
        if (m_ballcount >= 2)
        {
            m_rollermotor->Set(INT_INTAKE_ROLLER_SPEED);
            m_wheelmotor->Set(INT_INTAKE_WHEEL_SPEED * 0.67);
        }
        else
        {
            m_rollermotor->Set(INT_INTAKE_ROLLER_SPEED);
            m_wheelmotor->Set(INT_INTAKE_WHEEL_SPEED);
        }
        
        break;

    case kStuff:
        // If a timer reaches a certain period, stop stuffing
        if (m_timer.Get() > INT_STUFF_TIME)
        {
            m_rollermotor->Set(0);
            m_wheelmotor->Set(0);
            m_stuffing = false;
            m_intakestate = kIdle;
        }
        // Otherwise, continue stuffing balls out
        else
        {
            m_rollermotor->Set(INT_INTAKE_ROLLER_SPEED);
            m_wheelmotor->Set(INT_INTAKE_WHEEL_SPEED);
        }
        break;
        
    default: 
        break;
    }

    // manual override intake
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
    {
        m_rollermotor->Set(INT_INTAKE_ROLLER_SPEED);
        m_wheelmotor->Set(INT_INTAKE_WHEEL_SPEED);
    }
    else
    // manual override reverse
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
    {
        m_rollermotor->Set(-1.0 * INT_INTAKE_ROLLER_SPEED);
        m_wheelmotor->Set(-1.0 * INT_INTAKE_WHEEL_SPEED);
    }

	Dashboard();
}


void Intake::Stop()
{
    if (!NullCheck())
        return;
    
    m_intakestate = kIdle;
    m_ballstate = kZero;
    m_intakeposition = kUp;
    m_rollermotor->Set(0);
    m_wheelmotor->Set(0);
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
                m_solenoid->Set(false);
            }
            break;
        
        case kUp:
            if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_intakeposition = kDown;
                m_solenoid->Set(true);
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
    
    SmartDashboard::PutNumber("INT0_Ball Count", m_ballcount);
    SmartDashboard::PutBoolean("INT1_Intake On", (m_intakestate != kIdle));
    SmartDashboard::PutBoolean("INT2_Roller", m_rollersensor->Get());
    SmartDashboard::PutBoolean("INT3_Chute", m_chutesensor->Get());

    if (Debug)
    {
        SmartDashboard::PutNumber("INT4_State", m_intakestate);
    }
}


// Returns if the feeder should refresh
bool Intake::CanRefresh()
{
    // if chute has 2 balls, then we can refresh
    return (m_ballcount >= 2);
}