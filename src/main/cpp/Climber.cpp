/**
 *  Climber.cpp
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#include "Climber.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>


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
}


void Climber::Loop()
{
    if (m_motor == nullptr)
        return;

    // if start button and back button are both held, force climber
    if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL) && 
        m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
        m_motor->Set(CLM_MOTOR_SPEED);
    else
    // if start button is pressed, position turret at original position
    if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
    {
        m_deployrequest = true;
        // once ready and start button is also pressed at the same time, start climber motor
        if (m_deployready)
            m_motor->Set(CLM_MOTOR_SPEED);
    }
    else
    // if back button is pressed, disable climbing again
    if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
        m_deployrequest = false;
    else
        m_motor->StopMotor();

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