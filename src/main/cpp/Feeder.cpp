/**
 *  Feeder.cpp
 *  Date:
 *  Last Edited By:
 * 
 */


#include "Feeder.h"
#include "Const.h"

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


using namespace std;


Feeder::Feeder(OperatorInputs *inputs, Intake *intake, CDSensors *sensors)
{	
    if (FDR_MOTOR == -1)
    {
        DriverStation::ReportError("Feeder Not Enabled");
    }

    m_constraints.maxVelocity = 20_m / 1_s;
    m_constraints.maxAcceleration = 15_m / 1_s / 1_s;
    m_tolerance = 2.0_m;
    m_setpoint = 0_m;
    m_prevgoal = 0_m;

    // Configure these after testing to lock them into place
    m_feederPIDvals[0] = 0.0595;
    m_feederPIDvals[1] = 0.005;
    m_feederPIDvals[2] = 0.0005;

    m_inputs = inputs;
    m_intake = intake;
    m_sensors = sensors;

    m_motor = new CANSparkMax(FDR_MOTOR, CANSparkMax::MotorType::kBrushless);
    m_motor->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor->SetInverted(true);
    m_encoder = new CANEncoder(*m_motor);
    //m_encoder->SetInverted(true);
    m_feederPID = new ProfiledPIDController<units::meters>(
        m_feederPIDvals[0],
        m_feederPIDvals[1], 
        m_feederPIDvals[2],
        m_constraints
    );
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

    m_setpoint = 0_m;
    m_prevgoal = 0_m;
    
    m_tolerance = 1.0_m;

    m_feederPID->SetConstraints(m_constraints);
    m_feederPID->SetTolerance(m_tolerance, FDR_HIGH_NUMBER * 1_m / 1_s);
    m_feederPID->SetPID(m_feederPIDvals[0], m_feederPIDvals[1], m_feederPIDvals[2]);
    m_feederPID->Reset(0_m);
    m_encoder->SetPosition(0);

    m_feederstate = kIdle;
    m_shoot = false;
    m_power = 0.7;

    SmartDashboard::PutNumber("FDRT1_P",             m_feederPIDvals[0]);
    SmartDashboard::PutNumber("FDRT2_I",             m_feederPIDvals[1]);
    SmartDashboard::PutNumber("FDRT3_D",             m_feederPIDvals[2]);
    SmartDashboard::PutNumber("FDRT8_Max Velocity",       m_constraints.maxVelocity.to<double>());
    SmartDashboard::PutNumber("FDRT9_Max Acceleration",   m_constraints.maxAcceleration.to<double>());
    SmartDashboard::PutNumber("FDRT10_Goal Tolerance",     m_tolerance.to<double>());
}

// /FeederBallCheck();
void Feeder::Loop()
{
     if (m_motor == nullptr)
        return;

    FeederStateMachine();

    SmartDashboard::PutNumber("FDR1_Feeder State", m_feederstate);
    SmartDashboard::PutNumber("FDR2_Calculated Set", m_power);
    SmartDashboard::PutNumber("FDR3_Setpoint Error", m_feederPID->GetPositionError().to<double>());
    SmartDashboard::PutNumber("FDR4_Motor Output", m_motor->Get());
    SmartDashboard::PutNumber("FDR5_Encoder Position", m_encoder->GetPosition() * FDR_WHEEL_SIZE * 3.1415926535);
    SmartDashboard::PutBoolean("FDR6_Refresh Finished", !(m_feederstate == kRefresh));
    SmartDashboard::PutNumber("FDR7_Previous Goal", m_prevgoal.to<double>());
}


void Feeder::Stop()
{
    if (m_motor == nullptr)
        return;
}


void Feeder::FeederStateMachine()
{
    double feedforward;
    double distance;
    double power;
    
    switch (m_feederstate)
    {
    case kIdle:
        // if shooter requests for shooting, ready for driving 
        if (m_shoot)
        {
            m_intake->SetStuffingBecauseShooting();
            m_feederstate = kDrive;
            m_motor->Set(m_power * FDR_INVERTED);
        }
        else
        // if we don't have a ball yet and we can refresh, then refresh
        if ((!m_sensors->BallPresent(FeederSensor) && m_intake->LoadRefresh()) || m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        {
            m_feederstate = kRefresh;
            m_setpoint = FDR_REFRESH_DISTANCE * 1_m;
            m_feederPID->SetGoal(m_setpoint + m_prevgoal);
            m_encoder->SetPosition(m_prevgoal.to<double>() / FDR_WHEEL_SIZE / 3.1415926535 * FDR_GEAR_RATIO);
            m_timer.Reset();
            m_timer.Start();
            m_motor->Set(0);
        }
        else
        {
            m_motor->Set(0);
        }
        
        break;
    
    case kRefresh:
        distance = m_encoder->GetPosition() * FDR_WHEEL_SIZE * 3.1415926535 / FDR_GEAR_RATIO;
        power = fabs(m_feederPID->Calculate(units::meter_t{distance}));
        feedforward = (FDR_FEED_FORWARD / 12);
        // Constraining maximum and minimum outputs
        if (power < 0)
            power = 0;
        if (power > FDR_MAX_POWER)
            power = FDR_MAX_POWER;

        m_motor->Set((power + feedforward) * FDR_INVERTED);

        // If goal is reached or timeout timer hits, idle
        if (m_feederPID->AtGoal() || m_timer.Get() > FDR_TIMEOUT_TIME || m_sensors->BallPresent(FeederSensor))
        {
            m_prevgoal += m_setpoint;
            m_feederstate = kIdle;
        }
        break;

    case kDrive:
        // If we are shooting and have a ball or the intake is still running balls in, start driving
        if (m_sensors->BallPresent(FeederSensor) || m_sensors->BallPresent(Chute1Sensor) || m_intake->GetStuffingBecauseShooting())
        {
            m_feederstate = kDriveWait;
            m_shoot = true;
            m_intake->SetStuffingBecauseShooting();
            m_timer.Reset();
            m_timer.Start();
            m_motor->Set(m_power * FDR_INVERTED);
        }
        // if not, consider shooting done and return to idle
        else 
        {
            m_shoot = false;
            m_feederstate = kIdle;
            m_motor->Set(m_power * FDR_INVERTED);
        }
        break;

    case kDriveWait:
        // if the timer has been reached, go back to drive for feedback
        if (m_timer.Get() > FDR_DRIVE_TIME)
        {
            m_feederstate = kDrive;
            m_motor->Set(m_power * FDR_INVERTED);
            m_shoot = false;
        }
        else
        {
            m_motor->Set(m_power * FDR_INVERTED);
            m_motor->Set(m_power * FDR_INVERTED);
        }
        
        break;
    }; 
}

void Feeder::StartFire()
{
    m_shoot = true;
}

bool Feeder::GetFinished()
{
    return m_shoot;
}


void Feeder::ConfigureProfile()
{
    double v = SmartDashboard::GetNumber("FDRT8_Max Velocity", 0);
    double a = SmartDashboard::GetNumber("FDRT9_Max Acceleration", 0);

    if (v * 1_m / 1_s != m_constraints.maxVelocity) 
    {
        m_constraints.maxVelocity = v * 1_m / 1_s ; 
        m_feederPID->SetConstraints(m_constraints);
    }
    if (a * 1_m / 1_s / 1_s != m_constraints.maxAcceleration)
    {
        m_constraints.maxAcceleration = a * 1_m / 1_s / 1_s;
        m_feederPID->SetConstraints(m_constraints);
    }
}


void Feeder::ConfigurePID()
{
    double p = SmartDashboard::GetNumber("FDRT1_P", 0);
    double i = SmartDashboard::GetNumber("FDRT2_I", 0);
    double d = SmartDashboard::GetNumber("FDRT3_D", 0);
    units::meter_t t = units::meter_t{ SmartDashboard::GetNumber("FDRT10_Goal Tolerance", 0)};

    if((p != m_feederPIDvals[0])) { m_feederPID->SetP(p); m_feederPIDvals[0] = p; }
    if((i != m_feederPIDvals[1])) { m_feederPID->SetI(i); m_feederPIDvals[1] = i; }
    if((d != m_feederPIDvals[2])) { m_feederPID->SetD(d); m_feederPIDvals[2] = d; }
    if (t != m_tolerance)
    {
        m_tolerance = t;
        m_feederPID->SetTolerance(m_tolerance, FDR_HIGH_NUMBER * 1_m / 1_s);
    }
}