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


Turret::Turret(OperatorInputs *inputs, DualGyro *gyro)
{
    if (TUR_ENABLED != 1)
    {
        DriverStation::ReportError("Turret Not Enabled");
    }

    m_inputs = inputs;
    m_gyro = gyro;

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
    if (m_turretmotor != nullptr)
        delete m_turretmotor;
}


void Turret::Init()
{   
    if ((m_turretmotor == nullptr) && (TUR_TURRET_ID != -1))
    {
        m_turretmotor = new WPI_TalonSRX(TUR_TURRET_ID);
        m_turretmotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TUR_TIMEOUT_MS);
        m_turretmotor->SetNeutralMode(NeutralMode::Brake);
        m_turretmotor->SetSensorPhase(true);
        m_turretmotor->SetInverted(true);
    }
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
    m_turretangle = 0;      // Change these when starting orientation is different
    m_turretrampedangle = m_turretangle;
    m_turretmotor->SetSelectedSensorPosition(DegreesToTicks(m_turretangle), 0, TUR_TIMEOUT_MS);
    m_turretinitialfeedforward = 0;

    m_turretrampstate = kMaintain;
}


void Turret::Loop()
{
    if (!NullCheck())
        return;

    // FindFieldXBox();
    // CalculateTurretFromField();
    // RampUpTurret();

    Dashboard();
}


void Turret::Stop()
{
    m_turretmotor->SetNeutralMode(NeutralMode::Brake);
}


void Turret::Dashboard()
{
    SmartDashboard::PutNumber("TUR00_State", m_turretstate);
    SmartDashboard::PutNumber("TUR11_Turret Degrees", TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()));
    SmartDashboard::PutNumber("TUR12_Setpoint Angle", m_turretmotor->GetClosedLoopTarget(0));
    SmartDashboard::PutNumber("TUR14_Turret Angle", m_turretangle);
    SmartDashboard::PutNumber("TUR14_Turret RampedAngle", m_turretrampedangle);
}


// If it doesn't pass, return false
bool Turret::NullCheck()
{
    if (m_turretmotor == nullptr)
        return false;

    return true;
}


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


// Calculates the angle of the turret given an angle relative to the field and the robot gyro 
void Turret::CalculateTurretFromField()
{
    double heading;
    if (!m_gyro->GetHeading(heading))
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
}


void Turret::SetTurretAngle(double angle)
    {
    m_turretangle = -angle; // turret angle is backwards (increasing CW)!
    m_turretmotor->Set(ControlMode::Position, DegreesToTicks(m_turretangle));
    }

double Turret::GetTurretAngle(void)
    {
    // turret angle is backwards (increasing CW)!
    return -TicksToDegrees(m_turretmotor->GetSelectedSensorPosition()); 
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