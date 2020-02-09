/**
 *  Turret.cpp
 *  Date: 2/9/20
 *  Last Edited By: Geoffrey Xue
 */


#include "Turret.h"
#include "Const.h"


using namespace std;


Turret::Turret(OperatorInputs *inputs)
{
    m_inputs = inputs;

    // Flywheel
    m_flywheelmotor = nullptr;
    m_flywheelPID = nullptr;
    m_flywheelencoder = nullptr;
    
    m_PIDslot = 0;
    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;

    m_flywheelsimplemotorfeedforward = nullptr;
    m_flywheelinitialfeedforward = 0;

    // Turret
    m_turretmotor = nullptr;
    
    m_absoluteangle = 0;
    m_robotangle = 0;
    m_turretangle = 0;      // Change these when starting orientation is different
    m_turretrampedangle = 0;

    m_turretinitialfeedforward = 0;

    
    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_flywheelrampstate = kMaintain;
    m_turretrampstate = kMaintain;
    m_readytofire = false;

    // Temp (for testing purposes)
    m_feedermotor = nullptr;
    m_robotgyro = nullptr;
}


Turret::~Turret()
{
    if (m_flywheelmotor != nullptr)
        delete m_flywheelmotor;
    if (m_turretmotor != nullptr)
        delete m_turretmotor;
    if (m_feedermotor != nullptr)
        delete m_feedermotor;
}


void Turret::Init()
{
    // Flywheel
    m_flywheelmotor = new CANSparkMax(TUR_SHOOTER_ID, CANSparkMax::MotorType::kBrushless);
    m_flywheelPID = new CANPIDController(*m_flywheelmotor);
    m_flywheelencoder = new CANEncoder(*m_flywheelmotor);

    m_flywheelmotor->SetInverted(true);
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);

    // set increase and decrease PID gains on slot 0
    m_flywheelPID->SetP(TUR_SHOOTER_P, 0);
    m_flywheelPID->SetI(TUR_SHOOTER_I, 0);
    m_flywheelPID->SetD(TUR_SHOOTER_D, 0);
    // set maintain PID gains on slot 1
    m_flywheelPID->SetP(TUR_SHOOTER_MP, 1);
    m_flywheelPID->SetI(TUR_SHOOTER_MI, 1);
    m_flywheelPID->SetD(TUR_SHOOTER_MD, 1);

    // set the peak and nominal outputs
    m_flywheelPID->SetOutputRange(TUR_SHOOTER_MINOUT, TUR_SHOOTER_MAXOUT);

    m_PIDslot = 0;
    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;

    // use WPILIB simple motor feed forward class to pair with robot characterization
    m_flywheelPID->SetFF(0, 0);
    m_flywheelsimplemotorfeedforward = new SimpleMotorFeedforward<units::meters>(
                            units::volt_t{TUR_SHOOTER_KS}, 
                            TUR_SHOOTER_KV * 1_V * 1_s / 1_m, 
                            TUR_SHOOTER_KA * 1_V * 1_s * 1_s / 1_m);
    m_flywheelinitialfeedforward = 0;

    // Turret 
    m_turretmotor = new WPI_TalonSRX(TUR_TURRET_ID);
    m_turretmotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TUR_TIMEOUT_MS);
    m_turretmotor->SetNeutralMode(NeutralMode::Brake);
    m_turretmotor->SetSensorPhase(true);
    m_turretmotor->SetInverted(true);
    // Set PID Values
    m_turretmotor->Config_kP(0, TUR_TURRET_P, TUR_TIMEOUT_MS);
    m_turretmotor->Config_kI(0, TUR_TURRET_I, TUR_TIMEOUT_MS);
    m_turretmotor->Config_kD(0, TUR_TURRET_D, TUR_TIMEOUT_MS);
    // Set Min/Max outputs for PID
    m_turretmotor->ConfigNominalOutputForward(TUR_TURRET_MINOUT, TUR_TIMEOUT_MS);
    m_turretmotor->ConfigNominalOutputReverse(-1 * TUR_TURRET_MINOUT, TUR_TIMEOUT_MS);
    m_turretmotor->ConfigPeakOutputForward(TUR_TURRET_MAXOUT, TUR_TIMEOUT_MS);
    m_turretmotor->ConfigPeakOutputReverse(-1 * TUR_TURRET_MAXOUT, TUR_TIMEOUT_MS);
    m_turretmotor->SetSelectedSensorPosition(0, 0, TUR_TIMEOUT_MS);
    
    m_absoluteangle = 0;
    m_robotangle = 0;
    m_turretangle = 0;      // Change these when starting orientation is different
    m_turretrampedangle = 0;

    m_turretinitialfeedforward = 0;


    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_flywheelrampstate = kMaintain;
    m_turretrampstate = kMaintain;
    m_readytofire = false;

    // Temp (for testing purposes)
    m_feedermotor = new CANSparkMax(9, CANSparkMax::MotorType::kBrushless);
    m_flywheelmotor->SetInverted(true);
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_robotgyro = new PigeonIMU(0);
}


void Turret::Loop()
{
    TurretStates();
    FireModes();

    RampUpSetpoint();
    RampUpAngle();

    SmartDashboard::PutNumber("TUR0_Setpoint", m_flywheelsetpoint);
    SmartDashboard::PutNumber("TUR1_Encoder_Position in Native units", m_flywheelencoder->GetPosition());
    SmartDashboard::PutNumber("TUR2_Encoder_Velocity in Native Speed", m_flywheelencoder->GetVelocity());
    SmartDashboard::PutNumber("TUR3_SimpleMotorFeedforward", m_flywheelinitialfeedforward);
    SmartDashboard::PutNumber("TUR4_Error", m_flywheelrampedsetpoint - m_flywheelencoder->GetVelocity());
    SmartDashboard::PutNumber("TUR5_RampedSetpoint", m_flywheelrampedsetpoint);
    SmartDashboard::PutNumber("TUR6_RampState", m_flywheelrampstate);
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
            m_readytofire = false;
            // run flywheel at idle RPM
            m_flywheelsetpoint = TUR_SHOOTER_IDLE_STATE_RPM;
            // maintain flywheel at last absolute angle
            CalculateAbsoluteAngle();
            // if a button is pressed, allow second driver to change absolute angle and shoot
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_turretstate = kPreMove;
            break;

        case kPreMove:
            m_readytofire = false;
            m_flywheelsetpoint = TUR_SHOOTER_PREMOVE_STATE_RPM;
            // allow for manual movement of absolute angle to find vision target
            // X and Y are flipped due to angles in tangent starting between Quad I and IV, not Quad II and I
            // theoretical, not tested
            if (m_inputs->xBoxRightX(1 * INP_DUAL) > 0.95 || m_inputs->xBoxRightY(1 * INP_DUAL) > 0.95)
            {
                // Prevent divide by 0 errors
                if (m_inputs->xBoxRightY(1 * INP_DUAL) == 0)
                {
                    m_turretangle = m_inputs->xBoxRightX(1 * INP_DUAL) > 0 ? 90 : 270;
                }
                else
                {
                    double radians = atan(m_inputs->xBoxRightX(1 * INP_DUAL) / m_inputs->xBoxRightY()); // only between +- pi/2
                    double degrees = radians / 2 / 3.1415926535 * 360;   // 90 -> -90
                    m_turretangle = m_inputs->xBoxRightY(1 * INP_DUAL) > 0 ? degrees : degrees + 180;   // -90 -> 270
                    m_turretangle = m_turretangle < 0 ? m_turretangle + 360 : m_turretangle;   // 0 -> 360
                    m_turretangle = remainder(m_turretangle, 360);   // just for safety
                }
            }
            CalculateAbsoluteAngle();
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))    // if vision target is found, progress
                m_turretstate = kHoming;
            break;

        case kHoming:
            m_readytofire = false;
            // CalculateHoodFlywheel(distance, m_hoodangle, m_setpoint);
            // currently only manual work for now
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2700;
            else if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2800;
            else if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint = 2900;
            
            if (m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 0))
                m_flywheelsetpoint += 100;
            else if (m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 0) && (m_flywheelsetpoint >= 50))
                m_flywheelsetpoint -= 100;

            VisionTurretAngle();
        
            // if turret is only off by a small amount with its error and its flywheel is up to speed, progress
            if (TicksToDegrees(m_turretmotor->GetClosedLoopError()) <= TUR_TURRET_ERROR &&
                m_flywheelsetpoint == m_flywheelrampedsetpoint)
                m_turretstate = kReady;

            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))       // forced ready
                m_turretstate = kReady;
            break;
    
        case kReady:
            VisionTurretAngle();
            m_readytofire = true;
            // If the turret is suddenly off by a large amount (hit by robot, robot turned, etc), return to homing
            if (TicksToDegrees(m_turretmotor->GetClosedLoopError()) >= TUR_TURRET_MAX_ERROR)
            {
                m_turretstate = kHoming;
                m_readytofire = false;
            }
            // wait until FireModes turns turretstate back to kReturn
            break;

        case kReturn:
            m_readytofire = false;
            m_flywheelsetpoint = TUR_SHOOTER_IDLE_STATE_RPM;
            // maintain flywheel at last absolute angle
            CalculateAbsoluteAngle();
            if (m_flywheelrampstate == kMaintain)
                m_turretstate = kIdle;
            break;
    }
}


void Turret::FireModes()
{
    switch (m_firemode)
    {
        case kFireWhenReady:
            if (m_readytofire)
            {
                m_feedermotor->Set(0.2);
            }
            else
            {
                m_feedermotor->StopMotor();
            }
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_turretstate = kReturn;
            break;

        case kHoldFire:
            if (m_readytofire && m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_feedermotor->Set(0.2);
            }
            else
            {
                m_feedermotor->StopMotor();
            }
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_turretstate = kReturn;
            break;

        case kManualFire:
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
            {
                m_feedermotor->Set(0.2);
            }
            else
            {
                m_feedermotor->StopMotor();
            }
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_turretstate = kReturn;
            break;

        case kOff:
            m_feedermotor->StopMotor();
            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_turretstate = kReturn;
            break;
    }
}

// Calculates the angle of the turret given an angle relative to the field and the robot gyro 
void Turret::CalculateAbsoluteAngle()
{
    // Making robot angle from 0 - 360
    double robotangle = remainder(m_robotgyro->GetFusedHeading(), 360);
    if (robotangle < 0)
        robotangle += 360;

    double filter1 = remainder((m_robotangle + 45), 360);
    double filter2 = remainder((m_robotangle + 135), 360);
    // When the calculate turretangle would be in the turret deadzone, don't calulcate
    if (m_absoluteangle < max(filter1, filter2) && m_absoluteangle > min(filter1, filter2))
    {
        SmartDashboard::PutBoolean("TUR_Robot Angle Ignored", true);
    }
    // If calculated turret angle is, good, set turretangle to that position
    else
    {
        SmartDashboard::PutBoolean("TUR_Robot Angle Ignored", false);
        double mirrorangle = 112.5 + m_absoluteangle / 2;
        double reflect = m_robotangle - mirrorangle;
        m_turretangle = mirrorangle - reflect;
        m_turretangle = remainder(m_turretangle, 360);
        SmartDashboard::PutNumber("TUR_Mirror Angle", mirrorangle);
        SmartDashboard::PutNumber("TUR_Reflect", reflect);
    }
}

// Uses vision input and calculates angle, adds it to the current angle and sets turretangle
bool Turret::VisionTurretAngle()
{
    return false;
}


void Turret::CalculateHoodFlywheel(double distance, double &hoodangle, double &flywheelspeed)
{

}


void Turret::RampUpSetpoint()
{

    switch (m_flywheelrampstate)
    {
        case kMaintain:
            m_PIDslot = 1;
            if (m_flywheelsetpoint > m_flywheelrampedsetpoint)
            {
                m_flywheelrampstate = kIncrease;
            }
            else
            if (m_flywheelsetpoint < m_flywheelrampedsetpoint)
            {
                m_flywheelrampstate = kDecrease;
            }
            else
            {
                // ramping the setpoint is only used to prevent PID from overshooting
                m_flywheelinitialfeedforward = m_flywheelsimplemotorfeedforward->Calculate(m_flywheelsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
            }
            break;

        case kIncrease:
            m_PIDslot = 0;
            // Provided that the setpoint hasn't been reached and the ramping has already reached halfway
            if ((m_flywheelsetpoint > m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() - m_flywheelrampedsetpoint > -1.0 * TUR_SHOOTER_RAMPING_RATE / 2))
            {
                m_flywheelrampedsetpoint += TUR_SHOOTER_RAMPING_RATE;

                if (m_flywheelsetpoint <= m_flywheelrampedsetpoint)
                    m_flywheelrampedsetpoint = m_flywheelsetpoint;
            }
            else
            // If ramped setpoint has reached to the right speed
            if (m_flywheelsetpoint <= m_flywheelrampedsetpoint)
            {
                m_flywheelrampedsetpoint = m_flywheelsetpoint;
                m_flywheelrampstate = kMaintain;
                m_PIDslot = 1;
            }
            // ramping the setpoint is only used to prevent PID from overshooting
            m_flywheelinitialfeedforward = m_flywheelsimplemotorfeedforward->Calculate(m_flywheelsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
            break;
    
        case kDecrease:
            m_PIDslot = 0;
            // Provided that the setpoint hasn't been reached and the ramping has already reached halfway
            if ((m_flywheelsetpoint < m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() - m_flywheelrampedsetpoint < TUR_SHOOTER_RAMPING_RATE / 1.5))
            {
                m_flywheelrampedsetpoint -= TUR_SHOOTER_RAMPING_RATE;

                if (m_flywheelsetpoint >= m_flywheelrampedsetpoint)
                    m_flywheelrampedsetpoint = m_flywheelsetpoint;
            }
            else
            // If ramped setpoint has lowered to the right speed
            if (m_flywheelsetpoint >= m_flywheelrampedsetpoint)
            {
                m_flywheelrampedsetpoint = m_flywheelsetpoint;
                m_flywheelrampstate = kMaintain;
                m_PIDslot = 1;
            }
            // ramping the setpoint is only used to prevent PID from overshooting
            m_flywheelinitialfeedforward = m_flywheelsimplemotorfeedforward->Calculate(m_flywheelrampedsetpoint * TUR_MINUTES_TO_SECONDS * 1_mps).to<double>();
            break;
    };

    // Ignore PIDF feedforward and substitute WPILib's SimpleMotorFeedforward class
    m_flywheelPID->SetFF(0);
    // Converting setpoint to rotations per second, plugging into simplemotorfeedforward calculate and converting to a double  
    // The feed forward is set immediately to setpoint as velocity control allows for it
    m_flywheelPID->SetReference(m_flywheelrampedsetpoint, ControlType::kVelocity, m_PIDslot, m_flywheelinitialfeedforward);
}


void Turret::RampUpAngle()
{

    switch (m_turretrampstate)
    {
        case kMaintain:
            if (m_turretangle > m_turretrampedangle)
            {
                m_turretrampstate = kIncrease;
            }
            else
            if (m_turretangle < m_turretrampedangle)
            {
                m_turretrampstate = kDecrease;
            }
            
            break;
        case kIncrease:
            // Provided that the angle hasn't been reached and the ramping has already reached halfway
            if ((m_turretangle > m_turretrampedangle) && 
                (TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()) - m_turretrampedangle > -1.0 * TUR_TURRET_RAMPING_RATE))
            {
                m_turretrampedangle += TUR_TURRET_RAMPING_RATE;

                if (m_turretangle <= m_turretrampedangle)
                    m_turretrampedangle = m_turretangle;
            }
            else
            // if turretangle is suddenly set downwards
            if (m_turretangle <= (m_turretrampedangle - (TUR_TURRET_RAMPING_RATE * 2)))
            {
                m_turretrampstate = kDecrease;
            }
            else
            // If ramped angle has reached to the right speed
            if (m_turretangle <= m_turretrampedangle)
            {
                m_turretrampedangle = m_turretangle;
                m_turretrampstate = kMaintain;
            }
            break;
        case kDecrease:
            // Provided that the angle hasn't been reached and the ramping has already reached halfway
            if ((m_turretangle < m_turretrampedangle) && 
                (TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()) - m_turretrampedangle < TUR_TURRET_RAMPING_RATE))
            {
                m_turretrampedangle -= TUR_TURRET_RAMPING_RATE;

                if (m_turretangle >= m_turretrampedangle)
                    m_turretrampedangle = m_turretangle;
            }
            else
            // if turretangle is suddenly set upwards
            if (m_turretangle >= (m_turretrampedangle + (TUR_TURRET_RAMPING_RATE * 2)))
            {
                m_turretrampstate = kIncrease;
            }
            else
            // If ramped angle has lowered to the right speed
            if (m_turretangle >= m_turretrampedangle)
            {
                m_turretrampedangle = m_turretangle;
                m_turretrampstate = kMaintain;
            }
            break;
    };

    if (m_turretmotor->GetClosedLoopError(0) > 0)       // Make sure these directions 
    {
        m_turretinitialfeedforward = TUR_TURRET_KS_BACKWARDS / 12;
    }
    else
    {
        m_turretinitialfeedforward = -1 * TUR_TURRET_KS_FORWARDS / 12;
    }
    
    m_turretmotor->Set(ControlMode::Position, DegreesToTicks(m_turretrampedangle), 
                        DemandType::DemandType_ArbitraryFeedForward, m_turretinitialfeedforward);
}


double Turret::TicksToDegrees(double ticks)
{
    double rev = ticks / ENCODER_TICKS_PER_REV;
    double turretrev = rev * REV_TO_TURRET_REV;
    return turretrev * TURRET_REV_TO_DEGREES;
}


double Turret::DegreesToTicks(double degrees)
{
    double turretrev = degrees / TURRET_REV_TO_DEGREES;
    double rev = turretrev / REV_TO_TURRET_REV;
    return rev * ENCODER_TICKS_PER_REV;
}