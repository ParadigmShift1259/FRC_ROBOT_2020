/**
 *  CurveAuto.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "CurveAuto.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


CurveAuto::CurveAuto(DriveTrainFX *drivetrain, DualGyro *gyro)
{
    m_drivetrain = drivetrain;
    m_gyro = gyro;

    m_gyroPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_gyroPIDvals[0] = 0.035;
    m_gyroPIDvals[1] = 0;
    m_gyroPIDvals[2] = 0;
    // These values will be scaled based on the input, so doens't really matter here
    m_gyroconstraints.maxVelocity = 5_in / 1_s;
    m_gyroconstraints.maxAcceleration = 5_in / 1_s / 1_s;
    m_gyrotolerance = 2_deg;

    m_encoderPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_encoderPIDvals[0] = 0.35;
    m_encoderPIDvals[1] = 0.0079;
    m_encoderPIDvals[2] = 0.00295;
    m_encoderconstraints.maxVelocity = 4_in / 1_s;
    m_encoderconstraints.maxAcceleration = 2_in / 1_s / 1_s;
    m_encodertolerance = 4_in;

    m_setpoint = 0_in;
    m_finished = false;

    m_autostate = kIdle;
}


CurveAuto::~CurveAuto()
{
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
    if (m_gyro != nullptr)
        delete m_gyro;
}


void CurveAuto::Init()
{    
    m_gyroPIDController = new ProfiledPIDController<units::meters>(
        m_gyroPIDvals[0],
        m_gyroPIDvals[1], 
        m_gyroPIDvals[2],
        m_gyroconstraints
    );
    
    m_encoderPIDController = new ProfiledPIDController<units::meters>(
        m_encoderPIDvals[0],
        m_encoderPIDvals[1], 
        m_encoderPIDvals[2],
        m_encoderconstraints
    );

    m_encoderfeedforward = new SimpleMotorFeedforward<units::meters>(
        units::volt_t{INITIAL_FEEDFORWARD_DRIVE * 12},
        VELOCITY_FEEDFORWARD_DRIVE * 12 * 1_V * 1_s / 1_in,
        0 * 1_V * 1_s * 1_s / 1_in
    );

    m_gyrotolerance = 3_deg;

    m_gyroPIDController->SetPID(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);
    m_gyroPIDController->SetTolerance(units::inch_t{m_gyrotolerance.to<double>()}, DT_HIGH_NUMBER * 1_in / 1_s);

    m_encodertolerance = 10_in;

    m_encoderPIDController->SetConstraints(m_encoderconstraints);

    // NOTE Not using encoder tolerance
    m_encoderPIDController->SetTolerance(0_m, units::meters_per_second_t{4 * 1_in / 1_s});
    m_encoderPIDController->SetPID(m_encoderPIDvals[0], m_encoderPIDvals[1], m_encoderPIDvals[2]);
    
    m_setpoint = 0_in;
    m_setpointangle = 0_deg;
    m_finished = false;
    m_start = false;
    m_prevvelocity = 0_in / 1_s;

    m_autostate = kIdle;

    /*
	SmartDashboard::PutNumber("Gyro P",         m_gyroPIDvals[0]);
    SmartDashboard::PutNumber("Gyro I",         m_gyroPIDvals[1]);
    SmartDashboard::PutNumber("Gyro D",         m_gyroPIDvals[2]);
    SmartDashboard::PutNumber("Gyro Goal Tolerance", m_gyrotolerance.to<double>());

	SmartDashboard::PutNumber("Encoder P",             m_encoderPIDvals[0]);
    SmartDashboard::PutNumber("Encoder I",             m_encoderPIDvals[1]);
    SmartDashboard::PutNumber("Encoder D",             m_encoderPIDvals[2]);
    SmartDashboard::PutNumber("Encoder Max Velocity",       m_encoderconstraints.maxVelocity.to<double>());
    SmartDashboard::PutNumber("Encoder Max Acceleration",   m_encoderconstraints.maxAcceleration.to<double>());
    SmartDashboard::PutNumber("Encoder Goal Tolerance",     m_encodertolerance.to<double>());
    */
}


void CurveAuto::Loop()
{
    switch (m_start)
    {
        case kIdle:
            if (m_start)
            {
                m_autostate = kDrive;
                m_finished = false;
                m_gyro->ResetDeltaHeading();
                m_drivetrain->ResetDeltaDistance();
            }
            else
            {
                //m_drivetrain->GetDrive()->ArcadeDrive(m_encoderfeedforward->Calculate(m_prevvelocity).to<double>() / 12, 0);
                m_drivetrain->Drive(0, 0, false);
            }
            break;
        case kDrive:
            // Z deviance of drive, with error being degrees
            // (TAG) Make sure to check this is in the right direction, else flip gyro in Drivetrain or flip here
            units::degree_t heading = units::degree_t{m_gyro->GetDeltaHeading() * DT_GYRO_INVERTED};
            // Currangle is technically currangle + prevgoal, but to make things relative to 0, prevgoal must be subtracted
            SmartDashboard::PutNumber("CurrAngle", heading.to<double>());
            double z = m_gyroPIDController->Calculate(units::inch_t{heading.to<double>()});
            // X deviance of drive, with error being inches
            // (TAG) Make sure a positive motor ouput results in a positive encoder velocity, else flip encoder
            // (TAG) Make sure the robot drives in the right direction, else flip motor
        
            //double encoder1 = m_drivetrain->GetLeftPosition() / DT_TICKS_PER_METER;
            //double encoder2 = m_drivetrain->GetRightPosition() / DT_TICKS_PER_METER * DT_ENCODER_INVERTED;
            // averaging two encoders to obtain final value
        
            units::inch_t currdist = m_drivetrain->GetAverageDeltaDistance() * 1_in;
            // Currdist is techincally currdist + prevgoal, but to make things relative to 0, prevgoal must be subtracted
            SmartDashboard::PutNumber("Currdist", currdist.to<double>());
            double x = m_encoderPIDController->Calculate(currdist);

            SmartDashboard::PutNumber("Z", z);
            SmartDashboard::PutNumber("X", x);
            double sign = x / abs(x);
            double anglesign = z / abs(z);
            m_drivetrain->Drive(z + anglesign * INITIAL_FEEDFORWARD_TURN/2, x + sign * INITIAL_FEEDFORWARD_DRIVE, false);

            double velocity1 = m_drivetrain->GetLeftVelocity() / DT_TICKS_PER_METER * 10;
            double velocity2 = m_drivetrain->GetRightVelocity() / DT_TICKS_PER_METER * 10 * DT_ENCODER_INVERTED;
            double avvelocity = (velocity1 + velocity2) / 2;

            if (currdist > m_setpoint || m_encoderPIDController->AtGoal())
            {
                DriverStation::ReportError("One completed");
                SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
                SmartDashboard::PutNumber("Setpoint Angle", m_setpointangle.to<double>());
                m_autostate = kIdle;
                m_finished = true;
                m_start = false;
                m_prevvelocity = avvelocity * 1_in / 1_s;
            }
            SmartDashboard::PutNumber("Robot Average Velocity", avvelocity);
            break;
    }
    SmartDashboard::PutNumber("Autostate", m_autostate);
    SmartDashboard::PutNumber("Robot Encoders", m_drivetrain->GetAverageDeltaDistance() / DT_TICKS_PER_METER);
    SmartDashboard::PutNumber("Robot Gyro", m_gyro->GetDeltaHeading() * DT_GYRO_INVERTED);
    SmartDashboard::PutNumber("Encoder Error", m_encoderPIDController->GetPositionError().to<double>());
    SmartDashboard::PutNumber("Encoder Velocity Error", m_encoderPIDController->GetVelocityError().to<double>());
    SmartDashboard::PutNumber("Encoder Position Setpoint", m_encoderPIDController->GetSetpoint().position.to<double>());
    SmartDashboard::PutNumber("Encoder Velocity Setpoint", m_encoderPIDController->GetSetpoint().velocity.to<double>());

}


void CurveAuto::Stop()
{

}


void CurveAuto::ConfigureProfiles()
{
    units::meters_per_second_t v = SmartDashboard::GetNumber("Encoder Max Velocity", 0) * 1_in / 1_s;
    units::meters_per_second_squared_t a = SmartDashboard::GetNumber("Encoder Max Acceleration", 0) * 1_in / 1_s / 1_s;

    if (v != m_encoderconstraints.maxVelocity) 
    {
        m_encoderconstraints.maxVelocity = v; 
        m_encoderPIDController->SetConstraints(m_encoderconstraints);
    }
    if (a != m_encoderconstraints.maxAcceleration)
    {
        m_encoderconstraints.maxAcceleration = a;
        m_encoderPIDController->SetConstraints(m_encoderconstraints);
    }
}


void CurveAuto::ConfigureGyroPID()
{
    double p = SmartDashboard::GetNumber("Gyro P", 0);
    double i = SmartDashboard::GetNumber("Gyro I", 0);
    double d = SmartDashboard::GetNumber("Gyro D", 0);
    units::degree_t t = units::degree_t{ SmartDashboard::GetNumber("Gyro Goal Tolerance", 0)};

    if((p != m_gyroPIDvals[0])) { m_gyroPIDController->SetP(p); m_gyroPIDvals[0] = p; }
    if((i != m_gyroPIDvals[1])) { m_gyroPIDController->SetI(i); m_gyroPIDvals[1] = i; }
    if((d != m_gyroPIDvals[2])) { m_gyroPIDController->SetD(d); m_gyroPIDvals[2] = d; }
    if (t != m_gyrotolerance)
    {
        m_gyrotolerance = t;
        m_gyroPIDController->SetTolerance(units::inch_t{m_gyrotolerance.to<double>()}, DT_HIGH_NUMBER * 1_in / 1_s);
    }
}


void CurveAuto::ConfigureEncoderPID()
{
    double p = SmartDashboard::GetNumber("Encoder P", 0);
    double i = SmartDashboard::GetNumber("Encoder I", 0);
    double d = SmartDashboard::GetNumber("Encoder D", 0);
    units::inch_t t = units::inch_t{ SmartDashboard::GetNumber("Encoder Goal Tolerance", 0)};

    if((p != m_encoderPIDvals[0])) { m_encoderPIDController->SetP(p); m_encoderPIDvals[0] = p; }
    if((i != m_encoderPIDvals[1])) { m_encoderPIDController->SetI(i); m_encoderPIDvals[1] = i; }
    if((d != m_encoderPIDvals[2])) { m_encoderPIDController->SetD(d); m_encoderPIDvals[2] = d; }
    if (t != m_encodertolerance)
    {
        m_encodertolerance = t;
        m_encoderPIDController->SetTolerance(m_encodertolerance, DT_HIGH_NUMBER * 1_in / 1_s);
    }
}


void CurveAuto::StartMotion(double distance, double angle, double targetvelocity, double maxvelocity, double maxacceleration)
{
    m_setpoint = units::inch_t{distance};
    m_setpointangle = units::degree_t{angle};
    // Reset Motion Profile and Gyro
    m_gyro->ZeroHeading();
    m_gyroPIDController->Reset(0_in);
    // Reset Motion Profile and Encoders
    m_drivetrain->ResetDeltaDistance();
    m_encoderPIDController->Reset(0_in, m_prevvelocity);

    m_encoderconstraints.maxVelocity = maxvelocity * 1_in / 1_s;
    m_encoderconstraints.maxAcceleration = maxacceleration * 1_in / 1_s / 1_s;

    m_encoderPIDController->SetConstraints(m_encoderconstraints);

    // Based on the number of turns, the gyro constraints are recalculated
    // Take the encoderconstraints and scale it up to the gyro setpointangle
    m_gyroconstraints.maxVelocity = (
        m_encoderconstraints.maxVelocity.to<double>() / fabs(m_setpoint.to<double>()) * fabs(m_setpointangle.to<double>()))
        * 1_in / 1_s;
    m_gyroconstraints.maxAcceleration = (
        m_encoderconstraints.maxAcceleration.to<double>() / fabs(m_setpoint.to<double>()) * fabs(m_setpointangle.to<double>()))
        * 1_in / 1_s / 1_s;

    m_gyroPIDController->SetConstraints(m_gyroconstraints);

    TrapezoidProfile<units::meters>::State goal = {m_setpoint, targetvelocity * 1_in / 1_s};

    m_encoderPIDController->SetGoal(goal);
    m_gyroPIDController->SetGoal(units::inch_t{m_setpointangle.to<double>()});
    m_finished = false;
    m_start = true;
    //m_prevvelocity = targetvelocity * 1_in / 1_s;
}