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

    if (CLM_MOTOR != -1)
        m_motor = new Spark(CLM_MOTOR);
}


Climber::~Climber()
{
    if (m_motor != nullptr)
        delete m_motor;
}


void Climber::Init()
{
    if (m_motor != nullptr)
        delete m_motor;
}


void Climber::Loop()
{
    if (m_motor != nullptr)
        delete m_motor;
    
    Dashboard(); 
}


void Climber::Stop()
{
    if (m_motor != nullptr)
        delete m_motor;
}


void Climber::Dashboard()
{
    if (m_motor != nullptr)
        delete m_motor;
}