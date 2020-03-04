/**
 *  Turret.cpp
 *  Date: 2/20/2020
 *  Last Edited By: Geoffrey Xue
 */


#include "Turret.h"
#include "Const.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


using namespace std;


Turret::Turret(OperatorInputs *inputs, GyroDrive *gyrodrive, Intake *intake, Feeder *feeder, ControlPanel *controlpanel, Climber *climber, Vision *vision)
{
    if (TUR_ENABLED != 1)
    {
        DriverStation::ReportError("Turret Not Enabled");
    }

    m_inputs = inputs;
    m_gyrodrive = gyrodrive;
    m_vision = vision;
    m_intake = intake;
    m_feeder = feeder;
    m_controlpanel = controlpanel;
    m_climber = climber;

    m_flywheelmotor = nullptr;
    m_flywheelPID = nullptr;
    m_flywheelencoder = nullptr;
    m_flywheelsimplemotorfeedforward = nullptr;

    m_turretmotor = nullptr;

    m_hoodservo = nullptr;

    m_turretstate = kIdle;
    m_firemode = kHoldShoot;
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
    // Flywheel
    if ((m_flywheelmotor == nullptr) && (TUR_SHOOTER_ID != -1))
    {
        m_flywheelmotor = new CANSparkMax(TUR_SHOOTER_ID, CANSparkMax::MotorType::kBrushless);
        m_flywheelPID = new CANPIDController(*m_flywheelmotor);
        m_flywheelencoder = new CANEncoder(*m_flywheelmotor);
        //m_flywheelmotor->SetInverted(true);
        m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    }
    
    if (m_flywheelsimplemotorfeedforward == nullptr)
    {
        m_flywheelsimplemotorfeedforward = new SimpleMotorFeedforward<units::meters>(
                                units::volt_t{TUR_SHOOTER_KS}, 
                                TUR_SHOOTER_KV * 1_V * 1_s / 1_m, 
                                TUR_SHOOTER_KA * 1_V * 1_s * 1_s / 1_m);
    }

    // tuning
    m_flywheelspeedinc = 0;
    m_hoodangleinc = 0;

    // Turret
    if ((m_turretmotor == nullptr) && (TUR_TURRET_ID != -1))
    {
        m_turretmotor = new WPI_TalonSRX(TUR_TURRET_ID);
        m_turretmotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TUR_TIMEOUT_MS);
        m_turretmotor->SetNeutralMode(NeutralMode::Brake);
        m_turretmotor->SetSensorPhase(true);
        m_turretmotor->SetInverted(true);
    }
    
    // Hood
    if ((m_hoodservo == nullptr) && (TUR_HOOD_ID != -1))
        m_hoodservo = new Servo(TUR_HOOD_ID);

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

    m_timer.Start();
    m_timer.Reset();
    
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
    m_turretmotor->ConfigAllowableClosedloopError(0, DegreesToTicks(TURRET_DEGREE_STOP_RANGE));

    m_fieldangle = 180;
    m_robotangle = 0;
    m_turretmotor->SetSelectedSensorPosition(DegreesToTicks(135), 0, TUR_TIMEOUT_MS);
    m_turretangle = 135;      // Change these when starting orientation is different
    m_turretrampedangle = 135;

    m_turretinitialfeedforward = 0;

    m_hoodangle = 0;
    
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

    m_hoodservo->ClearError();

    if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
    {
        m_turretstate = kIdle;
        m_vision->SetLED(false);
        m_readytofire = false;
        m_firemode = kHoldShoot;
    }
    else
    if (m_inputs->xBoxLeftY(1 * INP_DUAL) < -0.7)
    {
        m_turretstate = kVision;
        m_vision->SetLED(true);
        m_readytofire = false;
    }
    else
    if (m_inputs->xBoxLeftY(1 * INP_DUAL) > 0.7 && (m_turretstate == kVision || m_turretstate == kAllReady))
    {
        m_turretstate = kIdle;
        m_vision->SetLED(false);
        m_readytofire = false;
        m_firemode = kHoldShoot;
    }

    TurretStates();
    FireModes();

    if (m_intake->IsIntakeUp())
    {
        if (m_turretangle < 120)
            m_turretangle = 120;
    }
    else
    if (m_controlpanel->ControlPanelUp())
    {
        if (m_turretangle > 235)
            m_turretangle = 235;
    }
    else
    if (m_climber->DeployRequest())
    {
        m_turretangle = 225;
        //m_climber->CanDeploy((m_turretrampedangle == 225));
        if (fabs(TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()) - m_turretangle) < TUR_TURRET_RAMPING_RATE)
            m_climber->CanDeploy(true);
        else
            m_climber->CanDeploy(false);
    }

    RampUpFlywheel();
    RampUpTurret();
    Dashboard();
}


void Turret::Stop()
{
    if (!NullCheck())
        return;

    m_flywheelsetpoint = 0;
    m_flywheelrampedsetpoint = 0;
    m_flywheelmotor->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_turretmotor->SetNeutralMode(NeutralMode::Brake);
    m_vision->SetLED(false);
}


void Turret::Dashboard()
{
    if (!NullCheck())
        return;

    SmartDashboard::PutNumber("TUR00_State", m_turretstate);
    SmartDashboard::PutBoolean("TUR01_ReadytoFire", m_readytofire);
    SmartDashboard::PutBoolean("TUR02_Targeting", (m_turretstate == kVision));

    if (Debug)
    {
        SmartDashboard::PutNumber("TUR03_Setpoint", m_flywheelsetpoint);
        SmartDashboard::PutNumber("TUR04_Encoder_Position in Native units", m_flywheelencoder->GetPosition());
        SmartDashboard::PutNumber("TUR05_Encoder_Velocity in Native Speed", m_flywheelencoder->GetVelocity());
        SmartDashboard::PutNumber("TUR06_SimpleMotorFeedforward", m_flywheelinitialfeedforward);
        SmartDashboard::PutNumber("TUR07_Error", m_flywheelrampedsetpoint - m_flywheelencoder->GetVelocity() * TUR_DIRECTION);
        SmartDashboard::PutNumber("TUR08_RampedSetpoint", m_flywheelrampedsetpoint);
        SmartDashboard::PutNumber("TUR09_RampState", m_flywheelrampstate);
        SmartDashboard::PutNumber("TUR10_PIDslot", m_PIDslot);
        SmartDashboard::PutNumber("TUR11_Turret Degrees", TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()));
        SmartDashboard::PutNumber("TUR12_Setpoint Angle", m_turretmotor->GetClosedLoopTarget(0));
        SmartDashboard::PutNumber("TUR13_Flywheel Ramp State", m_flywheelrampstate);
        SmartDashboard::PutNumber("TUR14_Turret Angle", m_turretangle);
        SmartDashboard::PutNumber("TUR15_Firing", m_firing);
        SmartDashboard::PutNumber("TUR16_Ready to Fire", m_readytofire);
        SmartDashboard::PutNumber("TUR17_Hood Angle", m_hoodangle);
        SmartDashboard::PutNumber("TUR18_Hood True Angle", m_hoodservo->Get());
    }
}


// If it doesn't pass, return false
bool Turret::NullCheck()
{
    if (m_flywheelmotor == nullptr)
        return false;
    if (m_turretmotor == nullptr)
        return false;
    if (m_hoodservo == nullptr)
        return false;

    return true;
}


void Turret::TurretStates()
{
    
    switch (m_turretstate)
    {
        case kIdle:
            m_readytofire = false;
            // Keep limelight lights off
            m_vision->SetLED(false);
            // run flywheel at idle RPM
            m_flywheelsetpoint = TUR_SHOOTER_IDLE_STATE_RPM;
            // Check xBox for abs angle
            FindFieldXBox();
            // maintain flywheel at last absolute angle
            CalculateTurretFromField();
            m_hoodservo->Set(0);

            if (m_feeder->GetBallCount() >= 5)
            {
                m_turretstate = kRampUp;
            }
            break;

        case kRampUp:
            m_readytofire = false;
            // Turn on limelight for turret angling
            m_vision->SetLED(true);
            // maintain turret at elevated speed
            m_flywheelsetpoint = TUR_SHOOTER_RAMPUP_STATE_RPM;
            // Check xBox for field angle            
            if (!FindFieldXBox())
            {
                if (!VisionFieldAngle())
                {
                    CalculateTurretFromField();
                }
            }
            else
            {
                // maintain flywheel at last field angle
                CalculateTurretFromField();
            }
            m_hoodservo->Set(0);

            if (m_feeder->GetBallCount() < 5)
                m_turretstate = kIdle;
            break;

        case kVision:
            // maintain lights on LED
            m_vision->SetLED(true);
            m_readytofire = false;
            // automatically calculate turret angle, flywheel and hood
            if (VisionFieldAngle())
            {
                CalculateHoodFlywheel(m_vision->GetDistance(), m_hoodangle, m_flywheelsetpoint);
                m_hoodservo->Set(m_hoodangle);

                // if certain errors are met, ready for shooting
                if ((TicksToDegrees(m_turretmotor->GetClosedLoopError()) <= TUR_TURRET_ERROR) &&
                    (fabs(m_flywheelencoder->GetVelocity() * TUR_DIRECTION - m_flywheelsetpoint) <= TUR_SHOOTER_ERROR))
                {
                    m_readytofire = true;
                    m_turretstate = kAllReady;
                    //m_timer.Reset();
                }
            }
            else
            {
                m_flywheelsetpoint = TUR_SHOOTER_RAMPUP_STATE_RPM;
                CalculateTurretFromField();
                m_hoodservo->Set(0);
            }
            break;
    
        case kAllReadyWait:
            if (m_timer.Get() > 0.10)
            {
                m_readytofire = true;
                m_turretstate = kAllReady;
            }
            else
            if (VisionFieldAngle())
            {
                CalculateHoodFlywheel(m_vision->GetDistance(), m_hoodangle, m_flywheelsetpoint);
                m_hoodservo->Set(m_hoodangle);
            }
            break;

        case kAllReady:
            m_vision->SetLED(true);
            VisionFieldAngle();
            CalculateHoodFlywheel(m_vision->GetDistance(), m_hoodangle, m_flywheelsetpoint);
            m_hoodservo->Set(m_hoodangle);

            // If driver wants to adjust turret field angle, go back to rampUp
            if (fabs(m_inputs->xBoxRightX(1 * INP_DUAL)) > 0.8 || fabs(m_inputs->xBoxRightY(1 * INP_DUAL)) > 0.8)
                m_turretstate = kIdle;
    
            // If the turret is suddenly off by a large amount (hit by robot, robot turned, etc), return to homing
            if (!m_firing &&
                (TicksToDegrees(m_turretmotor->GetClosedLoopError()) >= TUR_TURRET_MAX_ERROR ||
                !m_vision->GetActive() ||
                fabs(m_flywheelencoder->GetVelocity() * TUR_DIRECTION - m_flywheelsetpoint) >= TUR_SHOOTER_MAX_ERROR))
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
            if (!m_feeder->IsStuffing())
            {
                m_turretstate = kIdle;
                m_firemode = kHoldShoot;
                m_firing = false;
            }
            break;

        case kShootWhenReady:
            // if ready to fire, fire all when A button is pressed
            if (m_readytofire && !m_firing)
            {
                m_feeder->SetStuffing();
                m_readytofire = false;
                m_firing = true;
            }
            else
            if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) && 
                m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_feeder->SetStuffing();
                m_firemode = kForceShoot;
                m_firing = false;
            }
            else
            if (!m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) && 
                m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_turretstate = kVision;
            }

            if (m_firing && !m_feeder->IsStuffing())
            {
                m_firing = false;
                m_turretstate = kIdle;
                m_firemode = kHoldShoot;
            }
            break;

        case kHoldShoot:
            if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) && 
                m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_feeder->SetStuffing();
                m_firemode = kForceShoot;
            }
            else
            if (!m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
                m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
            {
                m_firemode = kShootWhenReady;
                // Force set turret to vision to ensure quick firing 2/28/20
                m_turretstate = kVision;
            }
            break;
    }
}

/*
void Turret::FindFieldXBox()
{
    // allow for manual movement of absolute angle to find vision target
    // X and Y are flipped due to angles in tangent starting between Quad I and IV, not Quad II and I
    if (fabs(m_inputs->xBoxRightX(1 * INP_DUAL)) > 0.8 || fabs(m_inputs->xBoxRightY(1 * INP_DUAL)) > 0.8)
    {
        // Prevent divide by 0 errors
        if (m_inputs->xBoxRightY(1 * INP_DUAL) == 0)
        {
            m_fieldangle = m_inputs->xBoxRightX(1 * INP_DUAL) > 0 ? 90 : 270;
        }
        else
        {
            double radians = atan(m_inputs->xBoxRightX(1 * INP_DUAL) / m_inputs->xBoxRightY(1 * INP_DUAL)); // only between +- pi/2
            double degrees = radians / 2 / 3.1415926535 * 360;   // 90 -> -90
            m_fieldangle = m_inputs->xBoxRightY(1 * INP_DUAL) > 0 ? degrees : degrees + 180;   // -90 -> 270
            m_fieldangle = m_fieldangle < 0 ? m_fieldangle + 360 : m_fieldangle;   // 0 -> 360
            m_fieldangle = fmod(m_fieldangle, 360);   // just for safety
        }
        SmartDashboard::PutNumber("TUR11_Absolute Angle", m_fieldangle);
    }
}
*/

bool Turret::FindFieldXBox()
{
    // allow for manual movement of absolute angle to find vision target
    // X and Y are flipped due to angles in tangent starting between Quad I and IV, not Quad II and I
    if (fabs(m_inputs->xBoxRightX(1 * INP_DUAL)) > 0.8 || fabs(m_inputs->xBoxRightY(1 * INP_DUAL)) > 0.8)
    {
        // Prevent divide by 0 errors
        if (m_inputs->xBoxRightY(1 * INP_DUAL) == 0)
        {
            m_fieldangle = m_inputs->xBoxRightX(1 * INP_DUAL) > 0 ? 90 : 270;
        }
        else
        {
            double radians = atan(m_inputs->xBoxRightX(1 * INP_DUAL) / m_inputs->xBoxRightY(1 * INP_DUAL)); // only between +- pi/2
            double degrees = radians / 2 / 3.1415926535 * 360;   // 90 -> -90
            m_fieldangle = m_inputs->xBoxRightY(1 * INP_DUAL) > 0 ? degrees : degrees + 180;   // -90 -> 270
            m_fieldangle = m_fieldangle < 0 ? m_fieldangle + 360 : m_fieldangle;   // 0 -> 360
            m_fieldangle = fmod(m_fieldangle, 360);   // just for safety
        }
        SmartDashboard::PutNumber("TUR11_Absolute Angle", m_fieldangle);
        return true;
    }
    return false;
}


// Calculates the angle of the turret given an angle relative to the field and the robot gyro 
void Turret::CalculateTurretFromField()
{
    double heading;
    if (!m_gyrodrive->GetHeading(heading))
        return;

    m_robotangle = fmod(heading, 360.0);
    if (m_robotangle < 0)
        m_robotangle += 360;

    double filter1 = fmod((m_robotangle + 45), 360.0);
    double filter2 = fmod((m_robotangle + 135), 360.0);
    // In between border of 0 - 360
    if (fmax(filter1, filter2) > (fmin(filter1, filter2) + 90))
    {
        if (m_fieldangle > fmax(filter1, filter2) || m_fieldangle < fmin(filter1, filter2)) {}
        else
        {
            double mirrorangle = 112.5 + m_fieldangle / 2;
            double reflect = m_robotangle - mirrorangle;
            m_turretangle = mirrorangle - reflect;
            m_turretangle = fmod(m_turretangle, 360.0);
            m_turretangle = 270 - m_turretangle;
            m_turretangle = fmod(m_turretangle, 360.0);
        }
    }
    else
    // In between zone not on border of 0 - 360
    if (m_fieldangle < fmax(filter1, filter2) && m_fieldangle > fmin(filter1, filter2)) {}
    else
    {
        double mirrorangle = 112.5 + m_fieldangle / 2;
        double reflect = m_robotangle - mirrorangle;
        m_turretangle = mirrorangle - reflect;
        m_turretangle = fmod(m_turretangle, 360.0); // safety
        m_turretangle = 270 - m_turretangle;        // flipped because values were originally calculated wrong
        m_turretangle = fmod(m_turretangle, 360.0); // safety
    }
}


// Uses vision input and calculates angle, adds it to the current angle and sets turretangle
bool Turret::VisionFieldAngle()
{
    if (!m_vision->GetActive())
        return false;
    
    m_turretangle = TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()) + m_vision->GetAngle();      // Change +/- when testing
    
    if (m_turretangle > 270)
        m_turretangle = 270;
    if (m_turretangle < 0)
        m_turretangle = 0;
    
    m_distance = m_vision->GetDistance();
    return true;
}


void Turret::CalculateHoodFlywheel(double distance, double &hoodangle, double &flywheelspeed)
{
    if ((m_distance < 110) || (m_distance > 380))
        return;

    /*
    // tuning
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxDPadLeft(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_flywheelspeedinc -= 50;
    else
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxDPadRight(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_flywheelspeedinc += 50;

    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxDPadDown(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_hoodangleinc -= 0.005;
    else
    if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) &&
        m_inputs->xBoxDPadUp(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))
        m_hoodangleinc += 0.005;

    SmartDashboard::PutNumber("TURTUNE_Flywheel Inc", m_flywheelspeedinc);
    SmartDashboard::PutNumber("TURTUNE_HoodAngle Inc", m_hoodangleinc);

    // 2/29 Tuning
    //hoodangle = -4.308317412 * pow(10,-12) * pow(distance, 5) + 5.194744002 * pow(10, -9) * pow(distance, 4) - 2.473275858 * pow(10, -6) * pow(distance, 3) + 5.807646984 * pow(10, -4) * pow(distance, 2)- 6.749202731 * pow(10, -2) * distance + 3.209018403;
    //flywheelspeed = 5711.094 + (2521.797 - 5711.094)/(1 + pow((distance/378.657), 2.567828));
    //flywheelspeed += 100;       // artificially inflate flywheel speed by 100
    //if (m_turretangle < 100)    // if turret is pointing out the front, increase flywheel speed by a bit more
    //    flywheelspeed += 50;
    */

    // 3/1 Tuning
    flywheelspeed = 1687.747 + 15.8111 * distance - 0.0594079 * pow(distance, 2) + 0.00008292342 * pow(distance, 3);
    hoodangle = 0.06286766 + (175598.7 - 0.06286766) / (1 + pow((distance / 0.6970016), 2.811798));
    /*
    flywheelspeed += m_flywheelspeedinc;
    hoodangle += m_hoodangleinc;
    */
    if (hoodangle < 0)
        hoodangle = 0.001;
}


void Turret::RampUpFlywheel()
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
            if ((m_flywheelsetpoint > m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() * TUR_DIRECTION - m_flywheelrampedsetpoint > -1.0 * TUR_SHOOTER_RAMPING_RATE / 2))
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
            if ((m_flywheelsetpoint < m_flywheelrampedsetpoint) && (m_flywheelencoder->GetVelocity() * TUR_DIRECTION - m_flywheelrampedsetpoint < TUR_SHOOTER_RAMPING_RATE))
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
    m_flywheelPID->SetReference(m_flywheelrampedsetpoint * TUR_DIRECTION, ControlType::kVelocity, m_PIDslot, m_flywheelinitialfeedforward * TUR_DIRECTION);
}


void Turret::RampUpTurret()
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