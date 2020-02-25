/**
 *  Autonomous.cpp
 *  Date:
 *  Last Edited By:
 */


#include "Autonomous.h"
#include "Const.h"


using namespace std;


Autonomous::Autonomous(GyroDrive *gyrodrive, Intake *intake, Feeder *feeder, Turret *turret, Vision *vision)
{
    m_gyrodrive = gyrodrive;
    m_intake = intake;
    m_feeder = feeder;
    m_turret = turret;
    m_vision = vision;

    m_stage = 0;
}


Autonomous::~Autonomous()
{
}


void Autonomous::Init()
{
    m_stage = 0;
    m_timer.Start();
    m_timer.Reset();
}


void Autonomous::Loop()
{
    switch (automode)
    {
        case kNoAuto:
            break;

        case kSimpleAuto:
            SimpleAuto();
            break;
    }
}


void Autonomous::Stop()
{
    m_gyrodrive->Stop();
}


void Autonomous::SimpleAuto()
{
    switch (m_stage)
    {
    case 0:
        m_turret->SetFireMode(Turret::FireMode::kHoldShoot);
        m_stage++;
        break;
    
    case 1:   
        if (m_gyrodrive->DriveStraight(48, 0.3, true))
        {
            m_timer.Reset();
            m_stage++;
        }
        break;
    
    case 2:
        if (m_turret->GetTurretState() == Turret::TurretState::kIdle)
            m_turret->SetTurretState(Turret::TurretState::kVision);

        if (m_timer.Get() > 0.25)
            m_stage++;
        break;

    case 3:
        if (m_turret->GetTurretState() == Turret::TurretState::kIdle)
            m_turret->SetTurretState(Turret::TurretState::kVision);

        m_turret->SetFireMode(Turret::FireMode::kShootWhenReady);
    }
}