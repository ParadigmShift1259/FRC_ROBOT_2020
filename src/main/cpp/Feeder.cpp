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
<<<<<<< HEAD
    
    m_sensor = nullptr;
    if (FDR_SENSOR != -1)
        m_sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor::Port::kOnboard, Rev2mDistanceSensor::DistanceUnit::kInches);
=======
>>>>>>> a0b30cf6d0d0bc65cda305292f323c4e70eaf18a
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
}


void Feeder::Loop()
{
    if (m_motor == nullptr)
        return;
<<<<<<< HEAD
    BallCheck();
    Dashboard(); 
=======
>>>>>>> a0b30cf6d0d0bc65cda305292f323c4e70eaf18a
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

bool Feeder::BallCheck()
{
    if (m_sensor -> GetRange() <= 3)
    {
        return true;
    }
    else if(m_sensor -> GetRange() > 3)
    {
        return false;
    }
}