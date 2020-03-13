/**
 *  Climber.cpp
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#include "Climber.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


using namespace std;


Climber::Climber(OperatorInputs *inputs)
{	
    m_inputs = inputs;

    m_motor = nullptr;
}


Climber::~Climber()
{
    if (m_motor != nullptr)
        delete m_motor;
}


void Climber::Init()
{
    if (CLM_MOTOR != -1)
    {
        m_motor = new WPI_TalonSRX(CLM_MOTOR);
        m_motor->SetNeutralMode(NeutralMode::Brake);
        m_motor->SetInverted(false);
        /*
        m_motor->ConfigPeakOutputForward(1);
        m_motor->ConfigPeakOutputReverse(-1);
        m_motor->ConfigNominalOutputForward(0);
        m_motor->ConfigNominalOutputReverse(0);
        */
        //m_motor->ConfigOpenloopRamp(CLM_RAMP_RATE);
    }
    
    m_motor->SetSelectedSensorPosition(0);
    
    m_deployready = false;
    m_deployrequest = false;

    m_state = kIdle;

    m_timer.Start();
    m_timer.Reset();
}


void Climber::Loop()
{
    if (m_motor == nullptr)
        return;     

    switch (m_state)
    {
    case kIdle:
        // if start button and back button are both held, force climber
        if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL) && 
            m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
            m_motor->Set(CLM_MOTOR_SPEED);
        else
        // if start button is pressed, position turret at original position
        if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
        {
            m_deployrequest = true;
            // once ready and start button is also pressed at the same time, start climbing motor sequence
            if (m_deployready)
            {
                m_motor->SetSelectedSensorPosition(0);
                m_timer.Reset();
                m_state = kAutoDrive;
            }
        }
        else
        // if back button is pressed and we're not in mid climb sequence, disable climbing again
        if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
            m_deployrequest = false;   
        else
            m_motor->StopMotor();
        break;
    
    case kAutoDrive:
        m_state = kDrive;
        // if encoder doesn't seem to be moving after a long time of driving, go straight to manual
        if ((m_timer.Get() > CLM_ENCODER_TIMEOUT) && (m_motor->GetSelectedSensorPosition() == 0))
        {
            DriverStation::ReportError("Climber Encoder Unplugged");
            m_state = kDrive;
        }
        // if climber reaches max height, advance to manual driving
        if (m_motor->GetSelectedSensorPosition() > CLM_MAX_HEIGHT_IN_TICKS)
            m_state = kDrive;
        else
        // if back button is held, temporarily pause motor raising
        if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
        {
            m_timer.Reset();
            m_motor->StopMotor();
        }
        // otherwise, continue driving motor up
        else
            m_motor->Set(CLM_MOTOR_SPEED);
        break;
    
    case kDrive:
        // if start button pressed, manually drive motor
        if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
            m_motor->Set(CLM_MOTOR_SPEED);
        // otherwise, hold motor
        else
            m_motor->StopMotor();
        break;
    }

    Dashboard();
}


void Climber::Stop()
{
    if (m_motor == nullptr)
        return;
}


void Climber::Dashboard()
{
    if (m_motor == nullptr)
        return;

    SmartDashboard::PutBoolean("CLM0_DeployRequest", m_deployrequest);
    SmartDashboard::PutBoolean("CLM1_DeployReady", m_deployready);
    SmartDashboard::PutNumber("CLM2_Motor Encoder", m_motor->GetSelectedSensorPosition());
}


bool Climber::DeployRequest()
{
    return m_deployrequest;
}


void Climber::CanDeploy(bool deploy)
{
    if (m_deployrequest)
        m_deployready = deploy;
    else
        m_deployready = false;
}