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
    fdrSt = Empty;
}

// /FDRBallCheck();
void Feeder::Loop()
{
    if (m_motor == nullptr)
        return;
    fdrSt = m_intake-> intkSt == m_intake -> chambering ||((m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL)) && (m_inputs-> xBoxDPadUp(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) )) 
    ? Feeding : fdrSt;
    fdrSt = FDRBallCheck () 
    ? Loaded : fdrSt;
    fdrSt = (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL)) 
    ? Firing : fdrSt;
  
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


void Feeder:: BlStMchne()
{
    switch (fdrSt)
    {
    case Idle:
        m_motor -> Set(0);
        break;
    case Feeding:
        break;
    case Loaded:
        break;
    case Firing:
        rnLp++;
        fdrSt = rnLp == 10 ? fdrSt::Empty: fdrSt;
        break;
    case Empty:
        break;
    default:
        break;
    } 
}


bool Feeder::FDRBallCheck()
{
    if (m_sensor -> GetRange() <= snsrDstFdr)
    {
        return true;
    }
    else if(m_sensor -> GetRange() > snsrDstFdr)
    {
        return false;
    }
    else 
    {
        return false;
    }
}