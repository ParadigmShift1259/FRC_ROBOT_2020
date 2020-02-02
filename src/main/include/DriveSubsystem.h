/**
 *  DriveSubsystem.h
 *  Date: 2/02/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_DriveSubsystem_H_
#define SRC_DriveSubsystem_H_

#include <units/units.h>


#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose2d.h>
#include <frc\geometry\Rotation2d.h>

#include <frc\controller\PIDController.h>
#include <frc\controller\SimpleMotorFeedforward.h>
#include <frc\controller\RamseteController.h>

#include <frc\trajectory\constraint\DifferentialDriveVoltageConstraint.h>
#include <frc\trajectory\Trajectory.h>
#include <frc\trajectory\TrajectoryConfig.h>
#include <frc\trajectory\TrajectoryGenerator.h>

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc\kinematics\DifferentialDriveKinematics.h>
#include <frc\kinematics\DifferentialDriveWheelSpeeds.h>

#include "Const.h"
#include "Drivetrain.h"

using namespace std;
using namespace frc;

class DriveSubsystem : public frc2::SubsystemBase 
{
 public:
    DriveSubsystem(Drivetrain *drivetrain);
    ~DriveSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void ResetEncoders();
    double GetAverageEncoderDistance();

    void SetMaxOutput(double maxOutput);
    double GetHeading();
    double GetTurnRate();
    void ResetOdometry(Pose2d pose);

    // Ramsete Control class calls these things in order
    Trajectory GetTrajectory(){return m_trajectory;}
    Pose2d GetPose();
    RamseteController GetRamseteController(){return *m_ramsetecontroller;}
    SimpleMotorFeedforward<units::meters> GetFeedforward(){return *m_feedforward;}
    DifferentialDriveKinematics GetKinematics(){return *m_kinematics;}
    DifferentialDriveWheelSpeeds GetWheelSpeeds();
    frc2::PIDController GetLeftPID(){return *m_leftPID;}
    frc2::PIDController GetRightPID(){return *m_rightPID;}
    void SetOutputVolts(units::volt_t leftvolts, units::volt_t rightvolts);


private:
    Drivetrain *m_drivetrain;

    PigeonIMU *m_gyro;

	SimpleMotorFeedforward<units::meters> *m_feedforward;
    DifferentialDriveOdometry *m_odometry;
	DifferentialDriveKinematics *m_kinematics;

    DifferentialDriveVoltageConstraint *m_voltageconstraint;
    TrajectoryConfig *m_trajectoryconfig;
	Trajectory m_trajectory;

    frc2::PIDController *m_leftPID;
	frc2::PIDController *m_rightPID;
    RamseteController *m_ramsetecontroller;
 
};

#endif /* SRC_DriveSubsystem_H_ */