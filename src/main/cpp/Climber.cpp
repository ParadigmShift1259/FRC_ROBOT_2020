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
    if (m_solenoid != nullptr)
        delete m_solenoid;
        
}


void Climber::Init()
{
    if (m_motor == nullptr || m_solenoid == nullptr)
        return;
  
}


void Climber::Loop()
{
    if (m_motor == nullptr || m_solenoid == nullptr)
        return;
     if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
         m_solenoid->Set(true);  
     if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
         m_solenoid->Set(false); 

    Dashboard(); 
}


void Climber::Stop()
{
    if (m_motor == nullptr || m_solenoid == nullptr)
        return;
}


void Climber::Dashboard()
{
    if (m_motor == nullptr || m_solenoid == nullptr)
        return;
}