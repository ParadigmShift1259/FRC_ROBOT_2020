/**
 *  DriveSubsystem.cpp
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#include "DriveSubsystem.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>


DriveSubsystem::DriveSubsystem(Drivetrain *drivetrain)
{
    m_drivetrain = drivetrain;
    m_gyro = m_drivetrain->GetGyro();

	m_feedforward = new SimpleMotorFeedforward<units::meters>(
        0_V,                        // kS
        0 * 1_V * 1_s / 1_m,        // kV
        0 * 1_V * 1_s * 1_s / 1_m   // kA
    );

	m_odometry = new DifferentialDriveOdometry(Rotation2d(units::degree_t{GetHeading()}));
    m_kinematics = new DifferentialDriveKinematics(units::meter_t{WHEEL_TRACK});

    m_voltageconstraint = new DifferentialDriveVoltageConstraint(
        *m_feedforward,
        *m_kinematics, 
        10_V
    );

    m_trajectoryconfig = new TrajectoryConfig(
        0_mps,
        0_mps / 1_s
    );

    m_trajectoryconfig->SetKinematics(*m_kinematics);
    m_trajectoryconfig->AddConstraint(*m_voltageconstraint);

    m_trajectory = TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        Pose2d(0_m, 0_m, Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {Translation2d(1_m, 1_m), Translation2d(2_m, -1_m)},
        // End 3 meters straight ahead of where we started, facing forward
        Pose2d(3_m, 0_m, Rotation2d(0_deg)),
        // Pass the config
        *m_trajectoryconfig
    );

	m_leftPID = new frc2::PIDController(0, 0, 0);               // P, I, D
	m_rightPID = new frc2::PIDController(0, 0, 0);				// P, I, D
    m_ramsetecontroller = new RamseteController(0.7, 2.0);      // B, Z

}


DriveSubsystem::~DriveSubsystem()
{
    if (m_drivetrain != nullptr)
        delete m_drivetrain;
}


void DriveSubsystem::Periodic() 
{
  // Implementation of subsystem periodic method goes here.
    m_odometry->Update(Rotation2d(units::degree_t(GetHeading())),
                    units::meter_t(m_drivetrain->GetLeftSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER),
                    units::meter_t(m_drivetrain->GetRightSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER * ENCODER_INVERTED));
}


void DriveSubsystem::SetOutputVolts(units::volt_t leftvolts, units::volt_t rightvolts)
{
    m_drivetrain->GetDrive()->TankDrive(
        leftvolts.to<double>() / 12,
        rightvolts.to<double>() / 12,
        false
    );
}


void DriveSubsystem::ResetEncoders()
{
  m_drivetrain->ResetEncoders();
}


double DriveSubsystem::GetAverageEncoderDistance()
{
    double left = m_drivetrain->GetLeftSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER;
    double right = m_drivetrain->GetRightSensor()->GetIntegratedSensorPosition() / TICKS_PER_METER * ENCODER_INVERTED;
    return (left + right) / 2;
}


void DriveSubsystem::SetMaxOutput(double maxOutput)
{
    m_drivetrain->GetDrive()->SetMaxOutput(maxOutput);
}


double DriveSubsystem::GetHeading()
{
    return remainder(m_drivetrain->GetGyro()->GetFusedHeading(), 360) * GYRO_INVERTED;
}


Pose2d DriveSubsystem::GetPose()
{
    return m_odometry->GetPose();
}


DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds()
{
    return {units::meters_per_second_t(m_drivetrain->GetLeftSensor()->GetIntegratedSensorVelocity() / TICKS_PER_METER * 10),
            units::meters_per_second_t(m_drivetrain->GetRightSensor()->GetIntegratedSensorVelocity() / TICKS_PER_METER * 10 * ENCODER_INVERTED)};
}


void DriveSubsystem::ResetOdometry(Pose2d pose)
{
    ResetEncoders();
    m_odometry->ResetPosition(pose, Rotation2d(units::degree_t(GetHeading())));
}