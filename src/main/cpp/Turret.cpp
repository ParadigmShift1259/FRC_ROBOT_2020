/**
 *  Turret.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Geoffrey Xue
 */


#include "Turret.h"
#include "Const.h"


using namespace std;


Turret::Turret(OperatorInputs *inputs)
{
    m_inputs = inputs;

    m_flywheelmotor = nullptr;
    m_flywheelPID = nullptr;
    m_flywheelencoder = nullptr;
    
    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_initialfeedforward = 0;

    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_rampstate = kMaintain;
}


Turret::~Turret()
{
    if (m_flywheelmotor != nullptr)
        delete m_flywheelmotor;
}


void Turret::Init()
{

    m_flywheelmotor = new CANSparkMax(1, CANSparkMax::MotorType::kBrushless);
    m_flywheelPID = new CANPIDController(*m_flywheelmotor);
    m_flywheelencoder = new CANEncoder(*m_flywheelmotor);

    m_flywheelmotor->SetInverted(true);
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);

    m_PIDslot = 0;
    /* set the peak and nominal outputs */
    m_flywheelPID->SetOutputRange(TUR_MINOUT, TUR_MAXOUT);
    /* set increase and decrease PID gains on 0 */
    m_flywheelPID->SetP(TUR_P, 0);
    m_flywheelPID->SetI(TUR_I, 0);
    m_flywheelPID->SetD(TUR_D, 0);
    m_flywheelPID->SetFF(0, 0);
    /* set maintain PID gains on 1 */
    m_flywheelPID->SetP(TUR_MP, 1);
    m_flywheelPID->SetI(TUR_MI, 1);
    m_flywheelPID->SetD(TUR_MD, 1);

    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_initialfeedforward = 0;

    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_rampstate = kMaintain;

    m_simplemotorfeedforward = new SimpleMotorFeedforward<units::meters>(
                            units::volt_t{TUR_KS}, 
                            TUR_KV * 1_V * 1_s / 1_m, 
                            TUR_KA * 1_V * 1_s * 1_s / 1_m);
}


void Turret::Loop()
{
/*
    if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 500;
    else if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 1000;
    else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 2000;
    else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 3000;
    
    if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint += 100;
    else if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 0) && (m_flywheelsetpoint >= 50))
        m_flywheelsetpoint -= 100;
    */
    TurretStates();
    FireModes();

    RampUpSetpoint();

    SmartDashboard::PutNumber("TUR0_Setpoint", m_flywheelsetpoint);
    SmartDashboard::PutNumber("TUR1_Encoder_Position in Native units", m_flywheelencoder->GetPosition());
    SmartDashboard::PutNumber("TUR2_Encoder_Velocity in Native Speed", m_flywheelencoder->GetVelocity());
    SmartDashboard::PutNumber("TUR3_SimpleMotorFeedforward", m_initialfeedforward);
    SmartDashboard::PutNumber("TUR4_Error", m_flywheelrampedsetpoint - m_flywheelencoder->GetVelocity());
    SmartDashboard::PutNumber("TUR5_RampedSetpoint", m_flywheelrampedsetpoint);
    SmartDashboard::PutNumber("TUR6_RampState", m_rampstate);
    SmartDashboard::PutNumber("TUR7_PIDslot", m_PIDslot);
}


void Turret::Stop()
{
    m_flywheelsetpoint = 0;
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
}


void Turret::TurretStates()
{
    switch (m_turretstate)
    {
        case kIdle:
            m_flywheelsetpoint = TUR_IDLE_STATE_RPM;
            // Bring m_shooter to as close to 0 angle as possible
            break;
        case kPreMove:
            m_flywheelsetpoint = TUR_PREMOVE_STATE_RPM;
            // Manual movement of m_shooter to get closer to vision target
            break;
        case kHoming:
            // CalculateHoodFlywheel(distance, m_hoodangle, m_setpoint);
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2600;
            else if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2700;
            else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2800;
            else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2900;
            
            if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint += 100;
            else if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 0) && (m_flywheelsetpoint >= 50))
                m_flywheelsetpoint -= 100;
            break;
        case kReady:
            
            break;
        case kReturn:
            break;
    }
}


void Turret::FireModes()
{
    switch (m_firemode)
    {
        case kFireWhenReady:
        case kHoldFire:
        case kManualFire:
        case kOff:
            break;
    }
}


void Turret::CalculateHoodFlywheel(double distance, double &hoodangle, double &flywheelspeed)
{

}


void Turret::RampUpSetpoint()
{

    switch (m_rampstate)
    {
        case kMaintain:
            m_PIDslot = 1;
            if (m_flywheelsetpoint > m_flywheelrampedsetpoint)
            {
                m_rampstate = kIncrease;
            }
            else
            if (m_flywheelsetpoint < m_flywheelrampedsetpoint)
            {
                m_rampstate = kDecrease;
            }
            
            break;
        case kIncrease:
            m_PIDslot = 0;
            // Provided that the setpoint hasn't been reached and the ramping has already reached halfway
            if ((m_flywheelsetpoint > m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() - m_flywheelrampedsetpoint > -1.0 * TUR_RAMPING_RATE / 2))
            {
                m_flywheelrampedsetpoint += TUR_RAMPING_RATE;

                if (m_flywheelsetpoint <= m_flywheelrampedsetpoint)
                    m_flywheelrampedsetpoint = m_flywheelsetpoint;
            }
            else
            // If ramped setpoint has reached to the right speed
            if (m_flywheelsetpoint <= m_flywheelrampedsetpoint)
            {
                m_flywheelrampedsetpoint = m_flywheelsetpoint;
                m_rampstate = kMaintain;
                m_PIDslot = 1;
            }
            break;
        case kDecrease:
            m_PIDslot = 0;
            // Provided that the setpoint hasn't been reached and the ramping has already reached halfway
            if ((m_flywheelsetpoint < m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() - m_flywheelrampedsetpoint < TUR_RAMPING_RATE / 1.5))
            {
                m_flywheelrampedsetpoint -= TUR_RAMPING_RATE;

                if (m_flywheelsetpoint >= m_flywheelrampedsetpoint)
                    m_flywheelrampedsetpoint = m_flywheelsetpoint;
            }
            else
            // If ramped setpoint has lowered to the right speed
            if (m_flywheelsetpoint >= m_flywheelrampedsetpoint)
            {
                m_flywheelrampedsetpoint = m_flywheelsetpoint;
                m_rampstate = kMaintain;
                m_PIDslot = 1;
            }
            break;
    };

    // Ignore PIDF feedforward and substitute WPILib's SimpleMotorFeedforward class
    m_flywheelPID->SetFF(0);
    // Converting setpoint to rotations per second, plugging into simplemotorfeedforward calculate and converting to a double  
    // (TAG) Was originally m_flywheelsetpoint, not tested yet
    m_initialfeedforward = m_simplemotorfeedforward->Calculate(m_flywheelrampedsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
    m_flywheelPID->SetReference(m_flywheelrampedsetpoint, ControlType::kVelocity, m_PIDslot, m_initialfeedforward);
}