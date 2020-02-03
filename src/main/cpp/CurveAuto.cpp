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


CurveAuto::CurveAuto(OperatorInputs *inputs, Drivetrain *drivetrain)
{
	m_inputs = inputs;
    m_drivetrain = drivetrain;

    m_gyroPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_gyroPIDvals[0] = 0.045;
    m_gyroPIDvals[1] = 0;
    m_gyroPIDvals[2] = 0;
    // These values will be scaled based on the input, so doens't really matter here
    m_gyroconstraints.maxVelocity = 5_mps;
    m_gyroconstraints.maxAcceleration = 5_mps / 1_s;
    m_gyrotolerance = 2_deg;

    m_encoderPIDController = nullptr;
    // Configure these after testing to lock them into place
    m_encoderPIDvals[0] = 0.33;
    m_encoderPIDvals[1] = 0.0079;
    m_encoderPIDvals[2] = 0.00295;
    m_encoderconstraints.maxVelocity = 4_mps;
    m_encoderconstraints.maxAcceleration = 2_mps / 1_s;
    m_encodertolerance = 0.5_m;

    m_setpoint = 0_m;
    m_finished = false;
    m_gyroprevgoal = 0_deg;
    m_encoderprevgoal = 0_m;

    m_turns = 1;

    m_autostate = kIdle;
}


CurveAuto::~CurveAuto()
{
    if (m_inputs != nullptr)
        delete m_inputs;
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
}


void CurveAuto::Init()
{
    m_drivetrain->Init();
    
    m_turns = 1;
    
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

    m_gyrotolerance = 2_deg;

    m_gyroPIDController->SetPID(m_gyroPIDvals[0], m_gyroPIDvals[1], m_gyroPIDvals[2]);
    m_gyroPIDController->SetTolerance(units::meter_t{m_gyrotolerance.to<double>()}, units::meters_per_second_t{HIGH_NUMBER});

    m_encodertolerance = 0.5_m;

    m_encoderPIDController->SetConstraints(m_encoderconstraints);
    m_encoderPIDController->SetTolerance(m_encodertolerance, units::meters_per_second_t{HIGH_NUMBER});
    m_encoderPIDController->SetPID(m_encoderPIDvals[0], m_encoderPIDvals[1], m_encoderPIDvals[2]);
    
    m_setpoint = 0_m;
    m_setpointangle = 0_deg;
    m_gyroprevgoal = 0_deg;
    m_encoderprevgoal = 0_m;
    m_finished = false;

    m_autostate = kIdle;

	SmartDashboard::PutNumber("Gyro P",         m_gyroPIDvals[0]);
    SmartDashboard::PutNumber("Gyro I",         m_gyroPIDvals[1]);
    SmartDashboard::PutNumber("Gyro D",         m_gyroPIDvals[2]);
    SmartDashboard::PutNumber("Gyro Goal Tolerance", m_gyrotolerance.to<double>());
    SmartDashboard::PutNumber("Turns", m_turns);

	SmartDashboard::PutNumber("Encoder P",             m_encoderPIDvals[0]);
    SmartDashboard::PutNumber("Encoder I",             m_encoderPIDvals[1]);
    SmartDashboard::PutNumber("Encoder D",             m_encoderPIDvals[2]);
    SmartDashboard::PutNumber("Encoder Max Velocity",       m_encoderconstraints.maxVelocity.to<double>());
    SmartDashboard::PutNumber("Encoder Max Acceleration",   m_encoderconstraints.maxAcceleration.to<double>());
    SmartDashboard::PutNumber("Encoder Goal Tolerance",     m_encodertolerance.to<double>());

    SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
    SmartDashboard::PutNumber("Setpoint Angle", m_setpointangle.to<double>());
}


void CurveAuto::Loop()
{
    units::meter_t setpoint = units::meter_t{ SmartDashboard::GetNumber("Setpoint", 0)};
    units::degree_t setpointangle = units::degree_t{ SmartDashboard::GetNumber("Setpoint Angle", 0)};

    switch (m_autostate)
    {
        case kIdle:
            m_drivetrain->GetDrive()->ArcadeDrive(0, 0, false);
            if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                m_setpoint = setpoint;
                m_setpointangle = setpointangle;
                // Sets gyro to previous goal to match motion profile
                m_drivetrain->SetGyro(m_gyroprevgoal.to<double>());
                // Sets encoders to previous goal to match motion profile
                m_drivetrain->SetEncoders(m_encoderprevgoal.to<double>());

                // Based on the number of turns, the gyro constraints are recalculated
                // Take the encoderconstraints and scale it up to the gyro setpointangle
                m_gyroconstraints.maxVelocity = (
                    m_encoderconstraints.maxVelocity.to<double>() / m_setpoint.to<double>() * m_setpointangle.to<double>())
                    * 1_m / 1_s;
                m_gyroconstraints.maxAcceleration = (
                    m_encoderconstraints.maxAcceleration.to<double>() / m_setpoint.to<double>() * m_setpointangle.to<double>())
                    * 1_m / 1_s / 1_s;

                m_gyroPIDController->SetConstraints(m_gyroconstraints);

                // Adds the previous goal so the motion profile is effectively "reset"
                // However, if m_turns is greater than 1, the profiling must continue
                if (m_turns <= 1)
                {
                    m_encoderPIDController->SetGoal(m_setpoint + m_encoderprevgoal);
                    // if m_turns is 0, gyro goes straight
                    m_gyroPIDController->SetGoal(units::meter_t{(m_setpointangle + m_gyroprevgoal).to<double>() * m_turns});
                }
                else
                {
                    // Continute to run at full speed
                    // (TAG) this assumes that the drive will reach max spsed, which means that the curvo must be large enough
                    // The setpoint is divided by m_turns so it reaches its goal mid way
                    m_encoderPIDController->SetGoal(TrapezoidProfile<units::meters>::State{
                        m_setpoint / m_turns + m_encoderprevgoal, 
                        m_encoderconstraints.maxVelocity}
                    );
                    m_gyroPIDController->SetGoal(units::meter_t{(m_setpointangle + m_gyroprevgoal).to<double>()});
                }

                m_autostate = kDrive;
                m_finished = false;
            }
            break;
        case kDrive:
            // Z deviance of drive, with error being degrees
            // (TAG) Make sure to check this is in the right direction, else flip gyro in Drivetrain or flip here
            units::degree_t heading = units::degree_t{m_drivetrain->GetGyro()->GetFusedHeading() * GYRO_INVERTED};
            // Currangle is technically currangle + prevgoal, but to make things relative to 0, prevgoal must be subtracted
            SmartDashboard::PutNumber("CurrAngle", heading.to<double>() - m_gyroprevgoal.to<double>());
            double z = m_gyroPIDController->Calculate(units::meter_t{heading.to<double>()});
            // X deviance of drive, with error being meters
            // (TAG) Make sure a positive motor ouput results in a positive encoder velocity, else flip encoder
            // (TAG) Make sure the robot drives in the right direction, else flip motor
            double encoder1 = m_drivetrain->GetLeftSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER;
            double encoder2 = m_drivetrain->GetRightSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER * ENCODER_INVERTED;
            // averaging two encoders to obtain final value
            units::meter_t currdist = units::meter_t{ (encoder1 + encoder2) / 2};
            // Currdist is techincally currdist + prevgoal, but to make things relative to 0, prevgoal must be subtracted
            SmartDashboard::PutNumber("Currdist", currdist.to<double>() - m_encoderprevgoal.to<double>());
            double x = m_encoderPIDController->Calculate(currdist);

            SmartDashboard::PutNumber("Z", z);
            SmartDashboard::PutNumber("X", x);
            double sign = x / abs(x);
            m_drivetrain->GetDrive()->ArcadeDrive(x + sign * INITIAL_FEEDFORWARD_DRIVE, z, false);

            if (m_encoderPIDController->AtGoal())
            {
                m_encoderprevgoal = m_setpoint;
                m_gyroprevgoal = m_setpointangle;

                if (m_turns <= 1)
                {
                    m_setpoint = 0_m;
                    m_setpointangle = 0_deg;
                    SmartDashboard::PutNumber("Setpoint", m_setpoint.to<double>());
                    SmartDashboard::PutNumber("Setpoint Angle", m_setpointangle.to<double>());
                    m_autostate = kIdle;
                    m_finished = true;
                }
                else
                {
                    // If there is more than one turn, keep going for the rest of the movement
                    // (DANGER) Motion Profile has to remember the target velocity in order for this to work!!!
                    // If not, robot will spaz because of the large error to start the cycle
                    m_encoderPIDController->SetGoal(m_setpoint / m_turns + m_encoderprevgoal);
                    // Turn the other way this time
                    m_gyroPIDController->SetGoal(units::meter_t{(m_setpointangle * -1).to<double>()});
                    m_turns -= 1;
                }
                

            }
            break;

    }
    SmartDashboard::PutNumber("Autostate", m_autostate);
}


void CurveAuto::Stop()
{

}



void CurveAuto::ConfigureProfiles()
{
    units::meters_per_second_t v = units::meters_per_second_t{ SmartDashboard::GetNumber("Encoder Max Velocity", 0)};
    units::meters_per_second_squared_t a = units::meters_per_second_squared_t{ SmartDashboard::GetNumber("Encoder Max Acceleration", 0)};
    int turns = SmartDashboard::GetNumber("Turns", 0);

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
    if (turns != m_turns)
    {
        m_turns = turns;
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
        m_gyroPIDController->SetTolerance(units::meter_t{m_gyrotolerance.to<double>()}, units::meters_per_second_t{HIGH_NUMBER});
    }
}


void CurveAuto::ConfigureEncoderPID()
{
    double p = SmartDashboard::GetNumber("Encoder P", 0);
    double i = SmartDashboard::GetNumber("Encoder I", 0);
    double d = SmartDashboard::GetNumber("Encoder D", 0);
    units::meter_t t = units::meter_t{ SmartDashboard::GetNumber("Encoder Goal Tolerance", 0)};

    if((p != m_encoderPIDvals[0])) { m_encoderPIDController->SetP(p); m_encoderPIDvals[0] = p; }
    if((i != m_encoderPIDvals[1])) { m_encoderPIDController->SetI(i); m_encoderPIDvals[1] = i; }
    if((d != m_encoderPIDvals[2])) { m_encoderPIDController->SetD(d); m_encoderPIDvals[2] = d; }
    if (t != m_encodertolerance)
    {
        m_encodertolerance = t;
        m_encoderPIDController->SetTolerance(m_encodertolerance, units::meters_per_second_t{HIGH_NUMBER});
    }
}