/**
 *  Feeder.cpp
 *  Date: 2/20/2020
 *  Last Edited By: Geoffrey Xue
 * 
 */


#include "Feeder.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

using namespace std;


Feeder::Feeder(OperatorInputs *inputs, Intake *intake)
{
    if (FDR_ENABLED != 1)
    {
        DriverStation::ReportError("Feeder Not Enabled");
    }

    m_inputs = inputs;
    m_intake = intake;

    m_motor = nullptr;
    m_feederPID = nullptr;

    m_loaded = false;
    m_stuffing = false;
}


Feeder::~Feeder()
{	
    if (m_motor != nullptr)
        delete m_motor;
}


void Feeder::Init()
{
    m_constraints.maxVelocity = 10_in / 1_s;
    m_constraints.maxAcceleration = 10_in / 1_s / 1_s;
    m_tolerance = 2.0_in;
    m_goal = 0_in;
    m_feederstate = kIdle;
    m_feederPIDvals[0] = FDR_P;
    m_feederPIDvals[1] = FDR_I;
    m_feederPIDvals[2] = FDR_D;
    m_loaded = false;
    m_stuffing = false;

    if (FDR_MOTOR != -1 && m_motor == nullptr)
    {
        m_motor = new WPI_TalonSRX(FDR_MOTOR);
    }

    if (m_feederPID == nullptr)
    {
        m_feederPID = new ProfiledPIDController<units::inches>(
            m_feederPIDvals[0],
            m_feederPIDvals[1], 
            m_feederPIDvals[2],
            m_constraints
        );
    }

    m_feederPID->SetConstraints(m_constraints);
    m_feederPID->SetTolerance(m_tolerance, FDR_HIGH_NUMBER * 1_in / 1_s);
    m_feederPID->SetPID(m_feederPIDvals[0], m_feederPIDvals[1], m_feederPIDvals[2]);
    m_feederPID->Reset(0_in, 0_in / 1_s);
    m_motor->Set(ControlMode::PercentOutput, 0);
    m_motor->SetSelectedSensorPosition(0);
    m_motor->SetSensorPhase(true);

    m_timer.Reset();
    m_timer.Start();
}


void Feeder::Loop()
{
    if (m_motor == nullptr)
        return;

    FeederStateMachine();

    SmartDashboard::PutNumber("Encoder Raw", m_motor->GetSelectedSensorPosition());
}


void Feeder::Stop()
{
    if (m_motor == nullptr)
        return;
}


void Feeder::FeederStateMachine()
{    
    double distance;
    double power;

    switch (m_feederstate)
    {
    case kIdle:
        if ((!m_loaded && m_intake->CanRefresh()) ||
            m_inputs->xBoxDPadLeft(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_goal = FDR_REFRESH_DISTANCE * 1_in;
            m_motor->SetSelectedSensorPosition(0);
            m_feederPID->Reset(0_in, 0_in / 1_s);
            m_feederPID->SetGoal(m_goal);
            m_timer.Reset();
            m_feederstate = kRefresh;
        }
        else
        // troubleshooting to stuff
        if (m_stuffing || m_inputs->xBoxDPadRight(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_timer.Reset();
            m_intake->SetStuffing();
            m_feederstate = kStuff;
        }
        else
        {
            m_motor->Set(0);
        }
        break;
    
    case kRefresh:
        // Calculate the current distance from the encoder
        distance = m_motor->GetSelectedSensorPosition() / ENCODER_TICKS_PER_REV * FDR_WHEEL_SIZE * 3.1415926535;
        // Calculate the motor set power from the Profiled PID Controller
        power = m_feederPID->Calculate(units::inch_t{distance});
        // Set the motor power to PID output
        m_motor->Set(power);

        // If goal is reached or timeout timer hits, idle
        if (m_feederPID->AtGoal() || m_timer.Get() > FDR_TIMEOUT_TIME)
        {
            m_motor->Set(0);
            m_loaded = true;
            m_feederstate = kIdle;
        }
        break;

    case kStuff:
        if (m_timer.Get() > FDR_STUFF_TIME)
        {
            m_motor->Set(0);
            m_loaded = false;
            m_stuffing = false;
            m_feederstate = kIdle;
        }
        else
        {
            m_motor->Set(FDR_STUFF_SPEED);
        }
        break;
    }
}


void Feeder::SetStuffing(bool stuff)
{
    m_stuffing = stuff;
}


bool Feeder::IsStuffing()
{
    return m_stuffing;
}