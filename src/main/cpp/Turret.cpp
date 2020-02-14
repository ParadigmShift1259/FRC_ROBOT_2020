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

    m_readytofire = false;

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
	if (!NullCheck())
		return;

	m_flywheelmotor = new CANSparkMax(TUR_SHOOTER_ID, CANSparkMax::MotorType::kBrushless);
    m_flywheelPID = new CANPIDController(*m_flywheelmotor);
    m_flywheelencoder = new CANEncoder(*m_flywheelmotor);

    m_flywheelmotor->SetInverted(true);
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);

    m_PIDslot = 0;
    // set the peak and nominal outputs
    m_flywheelPID->SetOutputRange(TUR_MINOUT, TUR_MAXOUT);
    // use WPILIB simple motor feed forward class to pair with robot characterization
    m_flywheelPID->SetFF(0, 0);
    m_simplemotorfeedforward = new SimpleMotorFeedforward<units::meters>(
                            units::volt_t{TUR_KS}, 
                            TUR_KV * 1_V * 1_s / 1_m, 
                            TUR_KA * 1_V * 1_s * 1_s / 1_m);
    // set increase and decrease PID gains on slot 0
    m_flywheelPID->SetP(TUR_P, 0);
    m_flywheelPID->SetI(TUR_I, 0);
    m_flywheelPID->SetD(TUR_D, 0);
    // set maintain PID gains on slot 1
    m_flywheelPID->SetP(TUR_MP, 1);
    m_flywheelPID->SetI(TUR_MI, 1);
    m_flywheelPID->SetD(TUR_MD, 1);

    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_initialfeedforward = 0;

    m_readytofire = false;

    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_rampstate = kMaintain;
}


void Turret::Loop()
{
	if (!NullCheck())
		return;

    //TurretStates();
    //FireModes();

    // experimental setting of setpoint with controller for now
    m_flywheelsetpoint = abs(m_inputs->xBoxRightY(1 * INP_DUAL)) * 3000;
    

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
	if (!NullCheck())
		return;

    m_flywheelsetpoint = 0;
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
}


void Turret::TurretStates()
{
    switch (m_turretstate)
    {
        case kIdle:
            m_readytofire = false;
            m_flywheelsetpoint = TUR_IDLE_STATE_RPM;
            // Bring m_shooter to as close to 0 angle as possible

            if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 0))    // and vision target is found
                m_turretstate = kHoming;
            else
            if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_turretstate = kPreMove;
            
            break;
        case kPreMove:
            m_readytofire = false;
            m_flywheelsetpoint = TUR_PREMOVE_STATE_RPM;
            // Manual movement of m_shooter to get closer to vision target

            if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 0))    // and vision target is found
                m_turretstate = kHoming;
            break;
        case kHoming:
            m_readytofire = false;
            // CalculateHoodFlywheel(distance, m_hoodangle, m_setpoint);
            /*
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2700;
            else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2800;
            else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2900;
                */
            
            if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint += 100;
            else if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 0) && (m_flywheelsetpoint >= 50))
                m_flywheelsetpoint -= 100;

            if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 0))       // needs to be changed so automatic
                m_turretstate = kReady;
            break;
    
        case kReady:
            // if turret angle is off by tolerance, set readytofire to false and go back to kHoming
            m_readytofire = true;
            // wait until FireModes turns turretstate back to kReturn
            if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 0))       // needs to be changed so automatic
                m_turretstate = kIdle;
            break;

        case kReturn:
            m_readytofire = false;
            m_flywheelsetpoint = TUR_IDLE_STATE_RPM;
            if (m_rampstate == RampState::kMaintain)
                m_turretstate = kIdle;
            break;
    }
}


void Turret::FireModes()
{
    switch (m_firemode)
    {
        case kFireWhenReady:
            // if (m_readytofire) - Call intake to retrieve balls
            break;
        case kHoldFire:
            // if (m_readytofire && m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0)) - call intake to retrieve balls
            break;
        case kManualFire:
            // if m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0) - call intake to retrieve balls
            break;
        case kOff:
            // nothing
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
            else
            {
                // ramping the setpoint is only used to prevent PID from overshooting
                m_initialfeedforward = m_simplemotorfeedforward->Calculate(m_flywheelsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
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
            // ramping the setpoint is only used to prevent PID from overshooting
            m_initialfeedforward = m_simplemotorfeedforward->Calculate(m_flywheelsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
            break;
    
        case kDecrease:
            m_PIDslot = 0;
            // Provided that the setpoint hasn't been reached and the ramping has already reached halfway
            if ((m_flywheelsetpoint < m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() - m_flywheelrampedsetpoint < (TUR_RAMPING_RATE * 1.25)))
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
            // ramping the setpoint is only used to prevent PID from overshooting
            m_initialfeedforward = m_simplemotorfeedforward->Calculate(m_flywheelrampedsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
            break;
    };

    // Ignore PIDF feedforward and substitute WPILib's SimpleMotorFeedforward class
    m_flywheelPID->SetFF(0);
    // Converting setpoint to rotations per second, plugging into simplemotorfeedforward calculate and converting to a double  
    // The feed forward is set immediately to setpoint as velocity control allows for it

    m_flywheelPID->SetReference(m_flywheelrampedsetpoint, ControlType::kVelocity, m_PIDslot, m_initialfeedforward);
}


bool Turret::NullCheck()
{
	if (TUR_SHOOTER_ID == -1)
	{
		return false;
	}

	return true;
}