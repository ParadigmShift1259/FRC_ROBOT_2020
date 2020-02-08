/**
 *  Feeder.cpp
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#include "Feeder.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/I2C.h>


using namespace std;


Feeder::Feeder(OperatorInputs *inputs, Intake *intake)
{	

    m_inputs = inputs;
    m_intake = intake;

    m_motor = nullptr;
    if (FDR_MOTOR != -1)
        m_motor = new CANSparkMax(FDR_MOTOR, CANSparkMax::MotorType::kBrushless);

    m_solenoid = nullptr;
    if (FDR_SOLENOID != -1)
        m_solenoid = new Solenoid(FDR_SOLENOID);     
}


Feeder::~Feeder()
{	
    if (m_motor != nullptr)
        delete m_motor;
     if (m_solenoid != nullptr)
        delete m_solenoid;
}


void Feeder::Init()
{
    if (m_motor == nullptr || m_solenoid == nullptr)
        return;
    m_feederstate = kIdle;
}

// /FeederBallCheck();
void Feeder::Loop()
{
     if (m_motor == nullptr || m_solenoid == nullptr)
        return;
    
    if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
        m_motor->Set(REFRESH_SPEED_LOAD);
    else
    if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
        m_motor->Set(-REFRESH_SPEED_LOAD);
    else
    {
        m_motor->Set(0);
    }
    
    if (m_inputs->xBoxDPadLeft(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_solenoid->Set(true);
    if (m_inputs->xBoxDPadRight(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_solenoid->Set(false);

    //FeederStateMachine();
    Dashboard(); 
}


void Feeder::Stop()
{
    if (m_motor == nullptr)
        return;
}


void Feeder::Dashboard()
{
    if (m_motor == nullptr)
        return;
}


void Feeder::FeederStateMachine()
{
    switch (m_feederstate)
    {
    case kIdle:
        if (m_shoot)
        {
            m_intake->SetDrivingBecauseShooting();
            m_feederstate = kFire;
        }
        else
        if (!m_hasball && m_intake->LoadRefresh())
        {
            m_feederstate = kRefresh;
        }
        m_motor->Set(0);
        break;

    case kFire:
        if (m_hasball || m_intake->GetDrivingBecauseShooting())
        {
            m_feederstate = kRefresh;
        }
        else 
        {
            m_shoot = false;
            m_feederstate = kIdle;
        }
        m_motor->Set(0);
        break;
    
    case kRefresh:
        // Run motor for specified encoder runs
        // Or Run motor until distance sensor says there is ball
        double power;
        if (m_shoot)
            power = REFRESH_SPEED_FIRE;
        else
            power = REFRESH_SPEED_LOAD;

        m_motor->Set(power);
        if (false)
        {
            if (m_shoot)
            {
                m_hasball = true;
                m_shoot = false;
                m_feederstate = kFire;
            }
            else
            if (!m_shoot)
            {
                m_hasball = true;
                m_feederstate = kIdle;
            }
        }
        break;

    case kReverse:
        m_motor-> Set(-0.3);
        break;
    } 
}

void Feeder::StartFire()
{
    m_shoot = true;
}

bool Feeder::GetFire()
{
    return m_shoot;
}