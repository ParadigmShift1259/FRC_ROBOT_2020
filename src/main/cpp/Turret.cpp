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
    // PID Values tuned for Janky Metal NEO Shooter 1/27/29 Geoffrey
    m_flywheelPIDvals[0] = 0.000632; 
    m_flywheelPIDvals[1] = 0; 
    m_flywheelPIDvals[2] = 0.0131; 
    m_flywheelPIDvals[3] = 0.0002231;
    m_flywheelPIDvals[4] = 0;
    m_flywheelPIDvals[5] = 0;
    m_flywheelPIDvals[6] = 1;
    m_PIDslot = 0;
    // PID values for maintaining, guessed values 1/29/20
    m_flywheelmaintainPIDvals[0] = 0.00068;
    m_flywheelmaintainPIDvals[1] = 0;
    m_flywheelmaintainPIDvals[2] = 0.008110;
    // PID Values for Janky Metal NEO Shooter from Characterization
    // Going Forward -> P = 0.77 for Rotation of +- .02
    // Going Backward -> P = 0.818
    // More realistic value given -> P = 0.0161
    
    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_flywheelsetpointreached = false;
    m_initialfeedforward = 0;

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
    m_rampstate = kMaintain;
}


Turret::~Turret()
{
    if (m_flywheelmotor != nullptr)
        delete m_flywheelmotor;
    /*
    if (m_flywheelPID != nullptr)
        delete m_flywheelPID;
    if (m_flywheelencoder != nullptr)
        delete m_flywheelencoder;
    */
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

    m_flywheelmotor = new CANSparkMax(1, CANSparkMax::MotorType::kBrushless);
    m_flywheelPID = new CANPIDController(*m_flywheelmotor);
    m_flywheelencoder = new CANEncoder(*m_flywheelmotor);

    //m_flywheelencoder->SetPhase(true);
    m_flywheelmotor->SetInverted(true);

    m_PIDslot = 0;
    /* set the peak and nominal outputs */
    m_flywheelPID->SetOutputRange(m_flywheelPIDvals[5], m_flywheelPIDvals[6]);
    /* set increase and decrease PID gains on 0 */
    m_flywheelPID->SetP(m_flywheelPIDvals[0], 0);
    m_flywheelPID->SetI(m_flywheelPIDvals[1], 0);
    m_flywheelPID->SetD(m_flywheelPIDvals[2], 0);
    m_flywheelPID->SetFF(m_flywheelPIDvals[3], 0);
    /* set maintain PID gains on 1 */
    m_flywheelPID->SetP(m_flywheelmaintainPIDvals[0], 1);
    m_flywheelPID->SetI(m_flywheelmaintainPIDvals[1], 1);
    m_flywheelPID->SetD(m_flywheelmaintainPIDvals[2], 1);
    

    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);

    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_flywheelsetpointreached = false;
    m_initialfeedforward = 0;

    //m_pigeon = new PigeonIMU(0);
    //m_heading = 0;
    //m_hoodmotor = new TalonSRX(1);
    //m_turretmotor = new TalonSRX(2);
    //m_hoodPID = new PIDController(m_hoodPIDvals[0], m_hoodPIDvals[1], m_hoodPIDvals[2], nullptr, nullptr);
    //m_turretPID = new PIDController(m_turretPIDvals[0], m_turretPIDvals[1], m_turretPIDvals[2], nullptr, nullptr);

    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_rampstate = kMaintain;

    // Testing purposes for PID
    SmartDashboard::PutNumber("P Gain",        m_flywheelPIDvals[0]);
    SmartDashboard::PutNumber("I Gain",        m_flywheelPIDvals[1]);
    SmartDashboard::PutNumber("D Gain",        m_flywheelPIDvals[2]);
    //SmartDashboard::PutNumber("I Zone",        m_flywheelPIDvals[3]);
    //SmartDashboard::PutNumber("Feed Forward",  m_flywheelPIDvals[3]);
    SmartDashboard::PutNumber("Min Output",    m_flywheelPIDvals[5]);
    SmartDashboard::PutNumber("Max Output",    m_flywheelPIDvals[6]);
    SmartDashboard::PutNumber("RPMScaling", 100);

    SmartDashboard::PutNumber("Maint P Gain",   m_flywheelmaintainPIDvals[0]);
    SmartDashboard::PutNumber("Maint I Gain",   m_flywheelmaintainPIDvals[1]);
    SmartDashboard::PutNumber("Maint D Gain",   m_flywheelmaintainPIDvals[2]);

    SmartDashboard::PutNumber("PIDSlot", m_PIDslot);
    SmartDashboard::PutNumber("SimpleMotorFeedforward", m_initialfeedforward);

    m_simplemotorfeedforward = new SimpleMotorFeedforward<units::meters>(
                            units::volt_t{TUR_KS}, 
                            TUR_KV * 1_V * 1_s / 1_m, 
                            TUR_KA * 1_V * 1_s * 1_s / 1_m);

    //SmartDashboard::PutBoolean("EnableMotor", false);
}


void Turret::Loop()
{
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard::GetNumber("P Gain", 0);
    double i = SmartDashboard::GetNumber("I Gain", 0);
    double d = SmartDashboard::GetNumber("D Gain", 0);
    //double iz = SmartDashboard::GetNumber("I Zone", 0);
    //double ff = SmartDashboard::GetNumber("Feed Forward", 0);
    double peak = SmartDashboard::GetNumber("Max Output", 0);
    double nominal = SmartDashboard::GetNumber("Min Output", 0);
    double RPMScaling = SmartDashboard::GetNumber("RPMScaling", 100);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // Increasing and Decreasing PID values
    if((p != m_flywheelPIDvals[0])) { m_flywheelPID->SetP(p, 0); m_flywheelPIDvals[0] = p; }
    if((i != m_flywheelPIDvals[1])) { m_flywheelPID->SetI(i, 0); m_flywheelPIDvals[1] = i; }
    if((d != m_flywheelPIDvals[2])) { m_flywheelPID->SetD(d, 0); m_flywheelPIDvals[2] = d; }
    //if((iz != m_flywheelPIDvals[3])) { m_flywheelPID->SetIZone(iz, 0); m_flywheelPIDvals[3] = iz; }
    //if((ff != m_flywheelPIDvals[3])) { m_flywheelPID->SetFF(ff, 0); m_flywheelPIDvals[3] = ff; }
    if((nominal != m_flywheelPIDvals[5]) || (peak != m_flywheelPIDvals[6])) 
    { 
        m_flywheelPID->SetOutputRange(m_flywheelPIDvals[5], m_flywheelPIDvals[6], 0);
        m_flywheelPID->SetOutputRange(m_flywheelPIDvals[5], m_flywheelPIDvals[6], 1);
        m_flywheelPIDvals[5] = nominal; m_flywheelPIDvals[6] = peak; 
    }

    // Maintained PID values
    double pmaint = SmartDashboard::GetNumber("Maint P Gain", 0);
    double imaint = SmartDashboard::GetNumber("Maint I Gain", 0);
    double dmaint = SmartDashboard::GetNumber("Maint D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((pmaint != m_flywheelmaintainPIDvals[0])) { m_flywheelPID->SetP(pmaint, 1); m_flywheelPIDvals[0] = pmaint; }
    if((imaint != m_flywheelmaintainPIDvals[1])) { m_flywheelPID->SetI(imaint, 1); m_flywheelPIDvals[1] = imaint; }
    if((dmaint != m_flywheelmaintainPIDvals[2])) { m_flywheelPID->SetD(dmaint, 1); m_flywheelPIDvals[2] = dmaint; }

    //double PIDslot = SmartDashboard::GetNumber("PIDSlot", 0);
    //if (PIDslot != m_PIDslot) { m_PIDslot = PIDslot;    }

    // Testing sample speeds

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

 
    // Ignore PIDF feedforward and substitute WPILib's SimpleMotorFeedforward class
    m_flywheelPID->SetFF(0);
    // Testing ramping function
    m_initialfeedforward = m_simplemotorfeedforward->Calculate(m_flywheelsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
    RampUpSetpoint();

    // Converting setpoint to rotations per second, plugging into simplemotorfeedforward calculate and converting to a double 
    
    /* Uncomment below line to enable motor
    bool enable = SmartDashboard::GetBoolean("EnableMotor", false);
    if (enable)
        m_flywheelPID->SetReference(m_flywheelsetpoint, ControlType::kVelocity, 0, m_initialfeedforward);     // plug in feedforward here
    */
    SmartDashboard::PutNumber("Setpoint", m_flywheelsetpoint);
    SmartDashboard::PutNumber("Encoder_Position in Native units", m_flywheelencoder->GetPosition());
    SmartDashboard::PutNumber("Encoder_Velocity in Native Speed", m_flywheelencoder->GetVelocity());
    SmartDashboard::PutNumber("SimpleMotorFeedforward", m_initialfeedforward);

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

    m_flywheelPID->SetReference(m_flywheelrampedsetpoint, ControlType::kVelocity, m_PIDslot, m_initialfeedforward);

    SmartDashboard::PutNumber("Error", m_flywheelrampedsetpoint - m_flywheelencoder->GetVelocity());
    SmartDashboard::PutNumber("RampedSetpoint", m_flywheelrampedsetpoint);
    SmartDashboard::PutNumber("RampState", m_rampstate);
    SmartDashboard::PutNumber("PIDslot", m_PIDslot);
}