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
        
        case kDriveStraight:
            DriveStraight();
            break;
        
        case kTrenchRun:
            TrenchRun();
            break;
        
        case kCenterRendezvous:
            break;
    }

    SmartDashboard::PutNumber("AUT0_Stage", m_stage);
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
        {
            m_stage++;
            m_turret->SetFireMode(Turret::FireMode::kShootWhenReady);
        }
        break;

    case 3:
        if (m_turret->GetTurretState() == Turret::TurretState::kIdle)
            m_turret->SetTurretState(Turret::TurretState::kVision);
        if (m_turret->IsFiring())
            m_stage++;
        break;

    case 4:
        break;
    }
}


void Autonomous::DriveStraight()
{
    switch (m_stage)
    {
    case 0:
        if (m_gyrodrive->StartMotion(48, 0, 0, 24, 12))
            m_stage++;
        break;
    
    case 1:
        if (m_gyrodrive->StartMotion(48, 45, 0, 24, 12))
            m_stage++;
        break;
    
    case 2:
        break;
    }
}


/*
void Autonomous::TrenchRun()
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
        m_turret->SetTurretState(Turret::TurretState::kVision);

        if (m_timer.Get() > 0.1)
        {
            m_stage++;
            m_turret->SetFireMode(Turret::FireMode::kShootWhenReady);
            m_feeder->SetStuffTime(1.5);
        }
        break;

    case 3:
        //if (m_turret->GetTurretState() == Turret::TurretState::kIdle)
        //    m_turret->SetTurretState(Turret::TurretState::kVision);
        if (m_turret->IsFiring())
            m_stage++;
        break;
    
    case 4:
        if (!m_turret->IsFiring())
        {
            m_turret->SetFireMode(Turret::FireMode::kHoldShoot);
            m_intake->SetGathering(true);
            m_stage++;
        }
        break;
    
    case 5:
        if (m_gyrodrive->DriveStraight(156, 0.2, true))
        {
            m_intake->SetGathering(false);
            m_stage++;
        }
        break;
    
    case 6:
        if (m_gyrodrive->DriveStraight(120, -0.35, true))
        {
            m_timer.Reset();
            m_stage++;
        }
        break;
    
    case 7:
        m_turret->SetTurretState(Turret::TurretState::kVision);

        if (m_timer.Get() > 0.1)
        {
            m_stage++;
        m_turret->SetFireMode(Turret::FireMode::kShootWhenReady);
        }
        break;
    
    case 8:
        if (m_turret->IsFiring())
            m_stage++;
        break;
    
    case 9:
        m_feeder->SetStuffTime(3);
        break;
    }
}
*/


void Autonomous::TrenchRun()
{
    switch (m_stage)
    {
    case 0:
        m_turret->SetTurretState(Turret::TurretState::kVision);
        m_stage++;
        break;
    
    case 1:   
        if (m_gyrodrive->DriveStraight(24, 0.3, true))
        {
            m_timer.Reset();
            m_stage++;
        }
        break;
    
    case 2:
        if (m_timer.Get() > 0.1)
        {
            m_stage++;
            m_turret->SetFireMode(Turret::FireMode::kShootWhenReady);
            m_feeder->SetStuffTime(1.5);
        }
        break;

    case 3:
        if (m_turret->IsFiring())
            m_stage++;
        break;
    
    case 4:
        if (!m_turret->IsFiring())
        {
            m_turret->SetFireMode(Turret::FireMode::kHoldShoot);
            m_turret->SetTurretState(Turret::TurretState::kVision);
            m_intake->SetGathering(true);
            m_stage++;
        }
        break;
    
    case 5:
        if (m_gyrodrive->DriveStraight(144, 0.2, true))
        {
            m_intake->SetGathering(false);
            m_stage++;
        }
        break;
    
    case 6:
        if (m_gyrodrive->DriveStraight(120, -0.35, true))
        {
            m_timer.Reset();
            m_stage++;
        }
        break;
    
    case 7:
        m_turret->SetTurretState(Turret::TurretState::kVision);

        if (m_timer.Get() > 0.1)
        {
            m_stage++;
            m_turret->SetFireMode(Turret::FireMode::kShootWhenReady);
        }
        break;
    
    case 8:
        if (m_turret->IsFiring())
            m_stage++;
        break;
    
    case 9:
        m_feeder->SetStuffTime(3);
        break;
    }
}