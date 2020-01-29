/**
 *  Intake.cpp
 *  Date:
 *  Last Edited By:
 */


#include "Intake.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>


using namespace std;


Intake::Intake(OperatorInputs *inputs)
{
	m_inputs = inputs;

    m_solenoid1 = nullptr;
    if (INT_SOLENOID1 != -1)
        m_solenoid1 = new Solenoid(INT_SOLENOID1);

    m_solenoid2 = nullptr;
    if (INT_SOLENOID2 != -1)
        m_solenoid2 = new Solenoid(INT_SOLENOID2);

    m_motor1 = nullptr;
    if (INT_MOTOR1 != -1)
        m_motor1 = new Spark(INT_MOTOR1);

    m_motor2 = nullptr;
    if (INT_MOTOR2 != -1)
        m_motor2 = new Spark(INT_MOTOR2);
}


Intake::~Intake()
{
    if (m_solenoid1 != nullptr)
        delete m_solenoid1;

    if (m_solenoid2 != nullptr)
        delete m_solenoid2;

    if (m_motor1 != nullptr)
        delete m_motor1;

    if (m_motor2 != nullptr)
        delete m_motor2;	   
}


void Intake::Init()
{
    if ((m_solenoid1 == nullptr) || (m_solenoid2 == nullptr) ||
        (m_motor1 == nullptr) || (m_motor2 == nullptr))
        return;
}


void Intake::Loop()
{
    if ((m_solenoid1 == nullptr) || (m_solenoid2 == nullptr) ||
        (m_motor1 == nullptr) || (m_motor2 == nullptr))
        return;
	Dashboard();
}


void Intake::Stop()
{
    if ((m_solenoid1 == nullptr) || (m_solenoid2 == nullptr) ||
        (m_motor1 == nullptr) || (m_motor2 == nullptr))
        return;
}


void Intake::Dashboard()
{
    if ((m_solenoid1 == nullptr) || (m_solenoid2 == nullptr) ||
        (m_motor1 == nullptr) || (m_motor2 == nullptr))
        return;
}