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

    // P, I, D, FF, Iz, nominal, peak
    // PID Values tuned for MiniCIM 1/17/20 Geoffrey
    m_flywheelPIDvals[0] = 0; 
    m_flywheelPIDvals[1] = 0; 
    m_flywheelPIDvals[2] = 0; 
    m_flywheelPIDvals[3] = 0;
    m_flywheelPIDvals[4] = 0;
    m_flywheelPIDvals[5] = 0;
    m_flywheelPIDvals[6] = 0.5;
    
    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_flywheelsetpointreached = false;

    //m_pigeon = nullptr;
    //m_heading = 0;
    //m_hoodmotor = nullptr;
    //m_turretmotor = nullptr;
    //m_hoodPID = nullptr;
    //m_turretPID = nullptr;
    //m_hoodPIDvals[0] = 0.0; m_hoodPIDvals[1] = 0.0; m_hoodPIDvals[2] = 0.0;
    //m_turretPIDvals[0] = 0.0; m_turretPIDvals[1] = 0.0; m_turretPIDvals[2] = 0.0;

    m_turretstate = kIdle;
    m_firemode = kHoldFire;
}


Turret::~Turret()
{
    if (m_flywheelmotor != nullptr)
        delete m_flywheelmotor;
    if (m_flywheelPID != nullptr)
        delete m_flywheelPID;
    if (m_flywheelencoder != nullptr)
        delete m_flywheelencoder;

    /*
    if (m_pigeon != nullptr)
        delete m_pigeon;
    if (m_hoodmotor != nullptr)
        delete m_hoodmotor;
    if (m_turretmotor != nullptr)
        delete m_turretmotor;
    if (m_hoodPID != nullptr)
        delete m_hoodPID;
    if (m_turretPID != nullptr)
        delete m_turretPID;
    */
}


void Turret::Init()
{
    /**
     * Constants needed:
     * CAN_GYRO_TURRET
     * CAN_FLYWHEEL_PORT
     * CAN_HOOD_PORT
     * CAN_TURRET_PORT
     */

    m_flywheelmotor = new CANSparkMax(0, CANSparkMax::MotorType::kBrushless);
    m_flywheelPID = new CANPIDController(*m_flywheelmotor);
    m_flywheelencoder = new CANEncoder(*m_flywheelmotor);



    /* set the peak and nominal outputs */
    m_flywheelPID->SetOutputRange(m_flywheelPIDvals[5], m_flywheelPIDvals[6]);
    /* set closed loop gains in slot0 */
    m_flywheelPID->SetP(0, m_flywheelPIDvals[0]);
    m_flywheelPID->SetI(0, m_flywheelPIDvals[1]);
    m_flywheelPID->SetD(0, m_flywheelPIDvals[2]);
    m_flywheelPID->SetFF(0, m_flywheelPIDvals[3]);

    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);

    //m_pigeon = new PigeonIMU(0);
    //m_heading = 0;
    //m_hoodmotor = new TalonSRX(1);
    //m_turretmotor = new TalonSRX(2);
    //m_hoodPID = new PIDController(m_hoodPIDvals[0], m_hoodPIDvals[1], m_hoodPIDvals[2], nullptr, nullptr);
    //m_turretPID = new PIDController(m_turretPIDvals[0], m_turretPIDvals[1], m_turretPIDvals[2], nullptr, nullptr);

    m_turretstate = kIdle;
    m_firemode = kHoldFire;

    // Testing purposes for PID
    SmartDashboard::PutNumber("P Gain",        m_flywheelPIDvals[0]);
    SmartDashboard::PutNumber("I Gain",        m_flywheelPIDvals[1]);
    SmartDashboard::PutNumber("D Gain",        m_flywheelPIDvals[2]);
    //SmartDashboard::PutNumber("I Zone",        m_flywheelPIDvals[3]);
    SmartDashboard::PutNumber("Feed Forward",  m_flywheelPIDvals[3]);
    SmartDashboard::PutNumber("Min Output",    m_flywheelPIDvals[5]);
    SmartDashboard::PutNumber("Max Output",    m_flywheelPIDvals[6]);
    SmartDashboard::PutNumber("RPMScaling", 100);
}


void Turret::Loop()
{
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard::GetNumber("P Gain", 0);
    double i = SmartDashboard::GetNumber("I Gain", 0);
    double d = SmartDashboard::GetNumber("D Gain", 0);
    //double iz = SmartDashboard::GetNumber("I Zone", 0);
    double ff = SmartDashboard::GetNumber("Feed Forward", 0);
    double peak = SmartDashboard::GetNumber("Max Output", 0);
    double nominal = SmartDashboard::GetNumber("Min Output", 0);
    double RPMScaling = SmartDashboard::GetNumber("RPMScaling", 100);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != m_flywheelPIDvals[0])) { m_flywheelPID->SetP(p, 0); m_flywheelPIDvals[0] = p; }
    if((i != m_flywheelPIDvals[1])) { m_flywheelPID->SetI(i, 0); m_flywheelPIDvals[1] = i; }
    if((d != m_flywheelPIDvals[2])) { m_flywheelPID->SetD(d, 0); m_flywheelPIDvals[2] = d; }
    //if((iz != m_flywheelPIDvals[3])) { m_flywheelPID->SetIZone(iz); m_flywheelPIDvals[3] = iz; }
    if((ff != m_flywheelPIDvals[3])) { m_flywheelPID->SetFF(ff, 0); m_flywheelPIDvals[3] = ff; }
    if((nominal != m_flywheelPIDvals[5]) || (peak != m_flywheelPIDvals[6])) 
    { 
        m_flywheelPID->SetOutputRange(m_flywheelPIDvals[5], m_flywheelPIDvals[6], 0);
        m_flywheelPIDvals[5] = nominal; m_flywheelPIDvals[6] = peak; 
    }

    // Testing sample speeds

    if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 0;
    else if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 1 * RPMScaling;
    else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 2 * RPMScaling;
    else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint = 3 * RPMScaling;
    
    if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 0))
        m_flywheelsetpoint += 10;
    else if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 0) && (m_flywheelsetpoint >= 10))
        m_flywheelsetpoint -= 10;

    // Testing function that doesn't have much yet
    RampUpSetpoint();

    m_flywheelPID->SetReference(m_flywheelsetpoint, ControlType::kVelocity); 

    SmartDashboard::PutNumber("Setpoint", m_flywheelsetpoint);
    SmartDashboard::PutNumber("Encoder_Position in Native units", m_flywheelencoder->GetPosition());
    SmartDashboard::PutNumber("Encoder_Velocity in Native Speed", m_flywheelencoder->GetVelocity());

    //TurretStates();
    //FireModes();
}


void Turret::Stop()
{
    m_flywheelsetpoint = 0;
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    //m_flywheelPID->SetReference(0, ControlType::kVelocity);
}


void Turret::TurretStates()
{
    switch (m_turretstate)
    {
        case kIdle:
        case kPreMove:
        case kHoming:
        case kReady:
        case kFire:
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
    m_flywheelrampedsetpoint = m_flywheelsetpoint;
    SmartDashboard::PutNumber("Error", m_flywheelrampedsetpoint - m_flywheelencoder->GetVelocity());
    /*
    Issues with code before implementation:
    - Only ramps upwards
    - Does not set m_flywheelsetpointreached to false during correct cases

    if ((m_flywheelsetpoint > m_flywheelrampedsetpoint) && (m_flywheelrampedsetpoint - m_flywheelencoder->GetVelocity() > 25))
    {
        m_flywheelrampedsetpoint += TUR_RAMPING_RATE

        if (m_flywheelrampedsetpoint > m_flywheelsetpoint)
            m_flywheelrampedsetpoint = m_flywheelsetpoint;
    }
    else
    if (m_flywheelsetpoint == m_flywheelrampedsetpoint)
    {
        m_flywheelsetpointreached = true;
    }

    m_flywheelmotor->SetReference(m_flywheelrampedsetpoint, ControlType::kVelocity);
    */
}