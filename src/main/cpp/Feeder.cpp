/**
 *  Feeder.cpp
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#include "Feeder.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <rev/Rev2mDistanceSensor.h>
#include <frc/I2C.h>


using namespace std;


Feeder::Feeder(OperatorInputs *inputs, Intake *intake)
{	

    m_inputs = inputs;
    m_intake = intake;

    m_motor = nullptr;
    if (FDR_MOTOR != -1)
        m_motor = new Spark(FDR_MOTOR);
    
    m_sensor = nullptr;
    if (FDR_SENSOR != -1)
        m_sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor::Port::kOnboard, Rev2mDistanceSensor::DistanceUnit::kInches);
}


Feeder::~Feeder()
{	
    if (m_motor != nullptr)
        delete m_motor;
}


void Feeder::Init()
{
    if (m_motor == nullptr)
        return;
    m_feederstate = kIdle;
}

// /FeederBallCheck();
void Feeder::Loop()
{
    if (m_motor == nullptr)
        return;

    FeederStateMachine();

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
    frc::SmartDashboard::PutNumber("FDR SNSR Distance(in)", m_sensor->GetRange());
    frc::SmartDashboard::PutNumber("FDR SNSR Time", m_sensor->GetTimestamp());
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
        if (!m_hasball && m_intake->GetBallCount() >= 2)
        {
            m_feederstate = kRefresh;
        }
        m_motor->Set(0);
        break;
    case kFire:
        if (FeederBallCheck() || m_intake->GetDrivingBecauseShooting())
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
        
        if (false /* done with running*/ && m_shoot)
            m_hasball = false;
            m_feederstate = kFire;
        else
        if (true /* done with running*/ && !m_shoot)
        {
            m_hasball = true;
            m_feederstate = kIdle;
        }
        break;
        m_motor->Set(power);
    default:
        break;
    } 
}

void Feeder::StartFire()
{
    m_feederstate = kFire;
}

bool Feeder:GetFire()
{
    m_feederstate = kIdle;
}


bool Feeder::FeederBallCheck()
{
    if (m_sensor -> GetRange() <= BALL_REGISTER_DISTANCE)
    {
        return true;
    }
    else 
    {
        return false;
    }
}