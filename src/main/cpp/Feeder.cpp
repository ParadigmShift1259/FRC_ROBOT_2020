/**
 *  Gryo.cpp
 *  Date:
 *  Last Edited By:
 */


#include "Feeder.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>


using namespace std;

Feeder::Feeder(OperatorInputs *inputs, Intake *intake)
{	
    m_inputs = inputs;
    m_intake = intake;
    m_motor = nullptr;
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
}


void Feeder::Stop()
{
}


void Feeder::Dashboard()
{
}