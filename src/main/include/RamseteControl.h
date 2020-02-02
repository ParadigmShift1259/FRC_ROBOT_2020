/**
 *  RamseteControl.h
 *  Date: 2/02/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_RamseteControl_H_
#define SRC_RamseteControl_H_

#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "DriveSubsystem.h"

#include <ctre/Phoenix.h>

#include <units/units.h>

#include <frc\controller\PIDController.h>
#include <frc\controller\SimpleMotorFeedforward.h>
#include <frc\controller\RamseteController.h>

#include <frc\trajectory\Trajectory.h>
#include <frc\trajectory\TrajectoryConfig.h>
#include <frc\trajectory\TrajectoryGenerator.h>

#include <frc\geometry\Pose2d.h>
#include <frc\geometry\Rotation2d.h>

#include <frc\kinematics\DifferentialDriveOdometry.h>
#include <frc\kinematics\DifferentialDriveKinematics.h>
#include <frc\kinematics\DifferentialDriveWheelSpeeds.h>

#include <frc2/command/Subsystem.h>

#include <frc2/command/RamseteCommand.h>


using namespace std;
using namespace frc;


class RamseteControl
{
public:

    enum AutoState {kIdle, kDrive};

	RamseteControl(OperatorInputs *inputs, Drivetrain *drivetrain, DriveSubsystem *drivesubsystem);
	~RamseteControl();
	void Init();
	void Loop();
	void Stop();
    bool IsFinished() { return m_finished; }
    

protected:
	OperatorInputs *m_inputs;
    Drivetrain *m_drivetrain;
	DriveSubsystem *m_drivesubsystem;

	frc2::RamseteCommand *m_ramsetecommand;

    bool m_finished;

    AutoState m_autostate;
};


#endif /* SRC_RamseteControl_H_ */
