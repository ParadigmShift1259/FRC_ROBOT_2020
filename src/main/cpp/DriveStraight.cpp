/**
 *  DriveStraight.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "DriveStraight.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


DriveStraight::DriveStraight(OperatorInputs *inputs, Drivetrain *drivetrain)
{
	m_inputs = inputs;
    m_drivetrain = drivetrain;

    m_gyroPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_gyroPIDvals[0] = 0.045;
    m_gyroPIDvals[1] = 0;
    m_gyroPIDvals[2] = 0;

    m_encoderPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_encoderPIDvals[0] = 0.33;
    m_encoderPIDvals[1] = 0.0079;
    m_encoderPIDvals[2] = 0.00295;
    m_constraints.maxVelocity = 4_mps;
    m_constraints.maxAcceleration = 2_mps / 1_s;
    m_tolerance = 0.5_m;

    m_setpoint = 0_m;
    m_finished = false;

    m_autostate = kIdle;
}


DriveStraight::~DriveStraight()
{
    if (m_inputs != nullptr)
        delete m_inputs;
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
}


void DriveStraight::Init()
{
    m_gyroPIDController = new frc2::PIDController(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);
    m_encoderPIDController = new ProfiledPIDController<units::meters>(
        m_encoderPIDvals[0],
        m_encoderPIDvals[1], 
        m_encoderPIDvals[2],
        m_constraints
    );
    
    m_drivetrain->Init();

    m_gyroPIDController->SetPID(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);

    m_tolerance = 0.5_m;

    m_encoderPIDController->SetConstraints(m_constraints);
    m_encoderPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
    m_encoderPIDController->SetPID(m_encoderPIDvals[0], m_encoderPIDvals[1], m_encoderPIDvals[2]);

    
    m_setpoint = 0_m;
    m_finished = false;

    m_autostate = kIdle;

	SmartDashboard::PutNumber("Gyro P",         m_gyroPIDvals[0]);
    SmartDashboard::PutNumber("Gyro I",         m_gyroPIDvals[1]);
    SmartDashboard::PutNumber("Gyro D",         m_gyroPIDvals[2]);

	SmartDashboard::PutNumber("Encoder P",             m_encoderPIDvals[0]);
    SmartDashboard::PutNumber("Encoder I",             m_encoderPIDvals[1]);
    SmartDashboard::PutNumber("Encoder D",             m_encoderPIDvals[2]);
    SmartDashboard::PutNumber("Encoder Max Velocity",       m_constraints.maxVelocity.to<double>());
    SmartDashboard::PutNumber("Encoder Max Acceleration",   m_constraints.maxAcceleration.to<double>());
    SmartDashboard::PutNumber("Encoder Goal Tolerance",     m_tolerance.to<double>());

    SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
}


void DriveStraight::Loop()
{
    units::meter_t setpoint = units::meter_t{ SmartDashboard::GetNumber("Setpoint", 0)};
    switch (m_autostate)
    {
        case kIdle:
            m_drivetrain->GetDrive()->ArcadeDrive(0, 0, false);
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                m_setpoint = setpoint;
                // resets both encoders and gyro before attempting drivestraight
                //m_drivetrain->ResetEncoders();
                m_drivetrain->ResetGyro();
                // Sets the goal of the motion profiled run
                m_encoderPIDController->SetGoal(m_setpoint);
                m_autostate = kDrive;
                m_finished = false;
            }
            break;
        case kDrive:
            // Z deviance of drive, with error being degrees
            // (TAG) Make sure to check this is in the right direction, else flip gyro in Drivetrain or flip here
            double heading = m_drivetrain->GetGyro()->GetFusedHeading() * GYRO_INVERTED;
            double z = m_gyroPIDController->Calculate(heading);
            // X deviance of drive, with error being meters
            // (TAG) Make sure a positive motor ouput results in a positive encoder velocity, else flip encoder
            // (TAG) Make sure the robot drives in the right direction, else flip motor
            double encoder1 = m_drivetrain->GetLeftSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER;
            double encoder2 = m_drivetrain->GetRightSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER * ENCODER_INVERTED;
            // averaging two encoders to obtain final value
            units::meter_t currdist = units::meter_t{ (encoder1 + encoder2) / 2};
            SmartDashboard::PutNumber("Currdist", currdist.to<double>());
            double x = m_encoderPIDController->Calculate(currdist);

            SmartDashboard::PutNumber("Z", z);
            SmartDashboard::PutNumber("X", x);
            double sign = x / abs(x);
            m_drivetrain->GetDrive()->ArcadeDrive(x + sign * INITIAL_FEEDFORWARD_DRIVE, z, false);

            if (m_encoderPIDController->AtGoal())
            {
                m_setpoint = 0_m;
                m_encoderPIDController->SetGoal(m_setpoint);
                SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
                m_autostate = kIdle;
                m_finished = true;
            }
            break;
    }
    SmartDashboard::PutNumber("Autostate", m_autostate);
}


void DriveStraight::Stop()
{

}



void DriveStraight::ConfigureProfile()
{
    units::meters_per_second_t v = units::meters_per_second_t{ SmartDashboard::GetNumber("Encoder Max Velocity", 0)};
    units::meters_per_second_squared_t a = units::meters_per_second_squared_t{ SmartDashboard::GetNumber("Encoder Max Acceleration", 0)};

    if (v != m_constraints.maxVelocity) 
    {
        m_constraints.maxVelocity = v; 
        m_encoderPIDController->SetConstraints(m_constraints);
    }
    if (a != m_constraints.maxAcceleration)
    {
        m_constraints.maxAcceleration = a;
        m_encoderPIDController->SetConstraints(m_constraints);
    }
}


void DriveStraight::ConfigureGyroPID()
{
    double p = SmartDashboard::GetNumber("Gyro P", 0);
    double i = SmartDashboard::GetNumber("Gyro I", 0);
    double d = SmartDashboard::GetNumber("Gyro D", 0);

    if((p != m_gyroPIDvals[0])) { m_gyroPIDController->SetP(p); m_gyroPIDvals[0] = p; }
    if((i != m_gyroPIDvals[1])) { m_gyroPIDController->SetI(i); m_gyroPIDvals[1] = i; }
    if((d != m_gyroPIDvals[2])) { m_gyroPIDController->SetD(d); m_gyroPIDvals[2] = d; }
}


void DriveStraight::ConfigureEncoderPID()
{
    double p = SmartDashboard::GetNumber("Encoder P", 0);
    double i = SmartDashboard::GetNumber("Encoder I", 0);
    double d = SmartDashboard::GetNumber("Encoder D", 0);
    units::meter_t t = units::meter_t{ SmartDashboard::GetNumber("Encoder Goal Tolerance", 0)};

    if((p != m_encoderPIDvals[0])) { m_encoderPIDController->SetP(p); m_encoderPIDvals[0] = p; }
    if((i != m_encoderPIDvals[1])) { m_encoderPIDController->SetI(i); m_encoderPIDvals[1] = i; }
    if((d != m_encoderPIDvals[2])) { m_encoderPIDController->SetD(d); m_encoderPIDvals[2] = d; }
    if (t != m_tolerance)
    {
        m_tolerance = t;
        m_encoderPIDController->SetTolerance(m_tolerance, units::meters_per_second_t{HIGH_NUMBER});
    }
}