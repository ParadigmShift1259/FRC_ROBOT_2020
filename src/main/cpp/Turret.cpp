/**
 *  Turret.cpp
 *  Date: 2/9/20
 *  Last Edited By: Geoffrey Xue
 */


#include "Turret.h"
#include "Const.h"

#include "frc/DriverStation.h"


using namespace std;


Turret::Turret(OperatorInputs *inputs, Intake *intake, Feeder *feeder, Vision *vision, GyroDrive *gyrodrive)
{
    if (!NullCheck())
    {
        DriverStation::ReportError("Turret Not Enabled");
        return;
    }

    m_inputs = inputs;
    m_vision = vision;
    m_intake = intake;
    m_feeder = feeder;

    // Flywheel
    m_flywheelmotor = new CANSparkMax(TUR_SHOOTER_ID, CANSparkMax::MotorType::kBrushless);
    m_flywheelPID = new CANPIDController(*m_flywheelmotor);
    m_flywheelencoder = new CANEncoder(*m_flywheelmotor);

    m_flywheelmotor->SetInverted(true);
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    
    m_PIDslot = 0;
    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;

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
    
    // Hood
    m_hoodservo = new Servo(0);

    m_absoluteangle = 0;
    m_robotangle = 0;
    m_robotgyro = gyrodrive->GetGyro();
    m_robotgyro->ZeroHeading();
    m_turretangle = 0;      // Change these when starting orientation is different
    m_turretrampedangle = 0;

    m_turretinitialfeedforward = 0;

    
    m_turretstate = kIdle;
    m_firemode = kHoldFire;
    m_flywheelrampstate = kMaintain;
    m_turretrampstate = kMaintain;
    m_readytofire = false;
}


Turret::~Turret()
{
    if (m_flywheelmotor != nullptr)
        delete m_flywheelmotor;
    if (m_turretmotor != nullptr)
        delete m_turretmotor;
}


void Turret::Init()
{
    if (!NullCheck())
        return;
        
    // Flywheel
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
    m_flywheelinitialfeedforward = 0;
    
    // Turret
    // Set PID Values
    m_turretmotor->Config_kP(0, TUR_TURRET_P, TUR_TIMEOUT_MS);
    m_turretmotor->Config_kI(0, TUR_TURRET_I, TUR_TIMEOUT_MS);
    m_turretmotor->Config_kD(0, TUR_TURRET_D, TUR_TIMEOUT_MS);
    // Set Min/Max outputs for PID
    m_turretmotor->ConfigNominalOutputForward(TUR_TURRET_MINOUT, TUR_TIMEOUT_MS);
    m_turretmotor->ConfigNominalOutputReverse(-1 * TUR_TURRET_MINOUT, TUR_TIMEOUT_MS);
    m_turretmotor->ConfigPeakOutputForward(TUR_TURRET_MAXOUT, TUR_TIMEOUT_MS);
    m_turretmotor->ConfigPeakOutputReverse(-1 * TUR_TURRET_MAXOUT, TUR_TIMEOUT_MS);
    m_turretmotor->SetSelectedSensorPosition(DegreesToTicks(135), 0, TUR_TIMEOUT_MS);
    
    m_absoluteangle = 180;
    m_robotangle = 0;
    m_turretangle = 135;      // Change these when starting orientation is different
    m_turretrampedangle = 135;
    m_stopPID = false;
    
    SmartDashboard::PutNumber("Robot Setup Angle", m_robotangle);
    SmartDashboard::PutNumber("Absolute Setup Angle", m_absoluteangle);

    m_turretinitialfeedforward = 0;


    m_turretstate = kIdle;
    m_firemode = kHoldShoot;
    m_flywheelrampstate = kMaintain;
    m_turretrampstate = kMaintain;
    m_readytofire = false;
    m_firing = false;

    m_distance = 0;
}


void Turret::Loop()
{
    if (!NullCheck())
        return;

    if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
    {
        m_turretstate = kIdle;
        m_readytofire = false;
    }
    else
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
    {
        m_turretstate = kRampUp;
        m_readytofire = false;
    }
    else
    if (m_inputs->xBoxLeftY(1 * INP_DUAL) > 0.7)
    {
        m_turretstate = kVision;
        m_readytofire = false;
    }
    else
    if (m_inputs->xBoxLeftY(1 * INP_DUAL) < 0.7)
    {
        m_turretstate = kRampUp;
        m_readytofire = false;
    }

    if (m_intake->BringingIntakeUp())
    {
        m_absoluteangle = 180;
    }
    

    m_hoodservo->SetPosition(fabs(m_inputs->xBoxLeftY(1 * INP_DUAL)));

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
    double heading;
    m_robotgyro->GetHeading(heading);
    SmartDashboard::PutNumber("Robot Gyro", heading);
    SmartDashboard::PutNumber("TUR8_Turret Degrees", TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()));
    SmartDashboard::PutNumber("TUR9_Setpoint Angle", m_turretmotor->GetClosedLoopTarget(0));
    SmartDashboard::PutNumber("TUR10_Flywheel Ramp State", m_flywheelrampstate);
    SmartDashboard::PutNumber("TUR11_Turret State", m_turretstate);
}


void Turret::Stop()
{
    if (!NullCheck())
        return;

    m_flywheelsetpoint = 0;
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_turretmotor->SetNeutralMode(NeutralMode::Brake);
}

// If it doesn't pass, return false
bool Turret::NullCheck()
{
    if (TUR_SHOOTER_ID == -1)
        return false;
    if (TUR_TURRET_ID == -1)
        return false;
    return true;
}


void Turret::TurretStates()
{
    switch (m_turretstate)
    {
        case kIdle:
            // run flywheel at idle RPM
            m_flywheelsetpoint = TUR_SHOOTER_IDLE_STATE_RPM;
            // Check xBox for abs angle
            CalculateAngleXBox();
            // maintain flywheel at last absolute angle
            CalculateAbsoluteAngle();
    
            if (m_flywheelsetpoint == m_flywheelrampedsetpoint)
                m_readytofire = true;
            else
                m_readytofire = false;
            break;

        case kRampUp:
            m_flywheelsetpoint = TUR_SHOOTER_PREMOVE_STATE_RPM;
            // Check xBox for abs angle
            CalculateAngleXBox();
            // maintain flywheel at last absolute angle
            CalculateAbsoluteAngle();
        
            if (m_flywheelsetpoint == m_flywheelrampedsetpoint)
                m_readytofire = true;
            else
                m_readytofire = false;
            break;

        case kVision:
            m_readytofire = false;
            // Loop vision but also check if valid
            if (!VisionTurretAngle())
                m_turretstate = kRampUp;

            // CalculateHoodFlywheel(distance, m_hoodangle, m_setpoint);
            // currently only manual work for now
            if (m_inputs->xBoxRightBumper(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_flywheelsetpoint += 100;
            else if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL) && (m_flywheelsetpoint >= 100))
                m_flywheelsetpoint -= 100;
            
            // If driver wants to adjust turret angle, go back to rampUp
            if (fabs(m_inputs->xBoxRightX(1 * INP_DUAL)) > 0.8 || fabs(m_inputs->xBoxRightY(1 * INP_DUAL)) > 0.8)
                m_turretstate = kRampUp;
    
            // if turret is only off by a small amount with its error and its flywheel is up to speed, progress
            if (TicksToDegrees(m_turretmotor->GetClosedLoopError()) <= TUR_TURRET_ERROR &&
                m_flywheelsetpoint == m_flywheelrampedsetpoint)
                m_turretstate = kAllReady;
            
            break;
    
        case kAllReady:
            VisionTurretAngle();
            m_readytofire = true;

            // If driver wants to adjust turret angle, go back to rampUp
            if (fabs(m_inputs->xBoxRightX(1 * INP_DUAL)) > 0.8 || fabs(m_inputs->xBoxRightY(1 * INP_DUAL)) > 0.8)
                m_turretstate = kRampUp;
    
            // If the turret is suddenly off by a large amount (hit by robot, robot turned, etc), return to homing
            if (TicksToDegrees(m_turretmotor->GetClosedLoopError()) >= TUR_TURRET_MAX_ERROR)
            {
                m_readytofire = false;
                m_turretstate = kVision;
            }
            // wait until FireModes turns turretstate back to kReturn
            break;
    }
}


void Turret::FireModes()
{
    switch (m_firemode)
    {
        case kForceShoot:
            if (m_feeder->GetFinished())
            {
                m_turretstate = kIdle;
                m_firemode = kHoldShoot;
                m_firing = false;
            }
            break;

        case kShootWhenReady:
            // if ready to fire, fire all when A button is pressed
            if (m_readytofire)
            {
                m_feeder->StartFire();
                m_readytofire = false;
                m_firing = true;
            }
            else
            if (m_inputs->xBoxLeftTrigger(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_feeder->StartFire();
                m_firemode = kForceShoot;
            }

            if (m_firing && m_feeder->GetFinished())
            {
                m_firing = false;
                m_turretstate = kIdle;
                m_firemode = kHoldShoot;
            }
            break;

        case kHoldShoot:
            if (m_inputs->xBoxLeftTrigger(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_feeder->StartFire();
                m_firemode = kForceShoot;
            }
            else
            if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
                m_firemode = kShootWhenReady;
            break;
    }
}


void Turret::CalculateAngleXBox()
{
    // allow for manual movement of absolute angle to find vision target
    // X and Y are flipped due to angles in tangent starting between Quad I and IV, not Quad II and I
    // theoretical, not tested
    if (fabs(m_inputs->xBoxRightX(1 * INP_DUAL)) > 0.8 || fabs(m_inputs->xBoxRightY(1 * INP_DUAL)) > 0.8)
    {
        // Prevent divide by 0 errors
        if (m_inputs->xBoxRightY(1 * INP_DUAL) == 0)
        {
            m_absoluteangle = m_inputs->xBoxRightX(1 * INP_DUAL) > 0 ? 90 : 270;
        }
        else
        {
            double radians = atan(m_inputs->xBoxRightX(1 * INP_DUAL) / m_inputs->xBoxRightY(1 * INP_DUAL)); // only between +- pi/2
            double degrees = radians / 2 / 3.1415926535 * 360;   // 90 -> -90
            m_absoluteangle = m_inputs->xBoxRightY(1 * INP_DUAL) > 0 ? degrees : degrees + 180;   // -90 -> 270
            m_absoluteangle = m_absoluteangle < 0 ? m_absoluteangle + 360 : m_absoluteangle;   // 0 -> 360
            m_absoluteangle = fmod(m_absoluteangle, 360);   // just for safety
        }
        SmartDashboard::PutNumber("TUR11_Absolute Angle", m_absoluteangle);
    }
}


// Calculates the angle of the turret given an angle relative to the field and the robot gyro 
void Turret::CalculateAbsoluteAngle()
{
    double heading;
    m_robotgyro->GetHeading(heading);
    m_robotangle = fmod(heading, 360.0);
    if (m_robotangle < 0)
        m_robotangle += 360;

    double filter1 = fmod((m_robotangle + 45), 360.0);
    double filter2 = fmod((m_robotangle + 135), 360.0);
    // In between border of 0 - 360
    if (fmax(filter1, filter2) > (fmin(filter1, filter2) + 90))
    {
        if (m_absoluteangle > fmax(filter1, filter2) || m_absoluteangle < fmin(filter1, filter2))
            SmartDashboard::PutBoolean("Robot Angle Ignored", true);
        else
        {
            SmartDashboard::PutBoolean("Robot Angle Ignored", false);
            double mirrorangle = 112.5 + m_absoluteangle / 2;
            double reflect = m_robotangle - mirrorangle;
            m_turretangle = mirrorangle - reflect;
            SmartDashboard::PutNumber("Turret Pre-Remainder Angle", m_turretangle);
            m_turretangle = fmod(m_turretangle, 360.0);
            m_turretangle = 270 - m_turretangle;
            m_turretangle = fmod(m_turretangle, 360.0);
        }
    }
    else
    // In between zone not on border of 0 - 360
    if (m_absoluteangle < fmax(filter1, filter2) && m_absoluteangle > fmin(filter1, filter2))
    {
        SmartDashboard::PutBoolean("Robot Angle Ignored", true);
    }
    else
    {
        SmartDashboard::PutBoolean("Robot Angle Ignored", false);
        double mirrorangle = 112.5 + m_absoluteangle / 2;
        double reflect = m_robotangle - mirrorangle;
        m_turretangle = mirrorangle - reflect;
        m_turretangle = fmod(m_turretangle, 360.0); // safety
        m_turretangle = 270 - m_turretangle;        // flipped because values were originally calculated wrong
        m_turretangle = fmod(m_turretangle, 360.0); // safety

        SmartDashboard::PutNumber("Mirror Setup Angle", mirrorangle);
        SmartDashboard::PutNumber("Reflect Setup Angle", reflect);
    }
}

// Uses vision input and calculates angle, adds it to the current angle and sets turretangle
bool Turret::VisionTurretAngle()
{
    if (!m_vision->GetActive())
        return false;
    
    m_turretangle = TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()) - m_vision->GetAngle();      // Change +/- when testing
    m_distance = m_vision->GetDistance();
    return true;
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
            else
            {
                if (fabs(TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()) < TURRET_DEGREE_STOP_RANGE))
                    m_stopPID = true;
                else
                    m_stopPID = false;
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
    if (!m_stopPID)
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