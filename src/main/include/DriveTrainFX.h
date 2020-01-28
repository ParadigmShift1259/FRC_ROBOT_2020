/**
 *  DriveTrainFX.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DriveTrainFX_H_
#define SRC_DriveTrainFX_H_


#include <frc\SpeedControllerGroup.h>
#include <frc\drive\DifferentialDrive.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"

// Ramsete Controller Implementation
#include <frc\controller\PIDController.h>
#include <frc\controller\SimpleMotorFeedforward.h>
#include <frc\geometry\Pose2d.h>
#include <frc\geometry\Rotation2d.h>
#include <frc\kinematics\DifferentialDriveOdometry.h>
#include <frc\kinematics\DifferentialDriveKinematics.h>
#include <frc\kinematics\DifferentialDriveWheelSpeeds.h>
#include <units/units.h>



using namespace frc;


class DriveTrainFX
{
public:
	// Drivetrain modes
	enum DriveMode { kNone, kFollower, kDiscrete, kTank, kArcade, kCurvature };

	DriveTrainFX(OperatorInputs *inputs, WPI_TalonFX *left1 = nullptr, WPI_TalonFX *left2 = nullptr, WPI_TalonFX *left3 = nullptr, WPI_TalonFX *right1 = nullptr, WPI_TalonFX *right2 = nullptr, WPI_TalonFX *right3 = nullptr);
	~DriveTrainFX();
	void Init(DriveMode mode = kFollower);
	void Loop();
	void Stop();
	void Drive(double x, double y, bool ramp = false);
		// change DriveTrain direction and return true if going forward
	bool ChangeDirection();
	bool ChangeLowSpeedMode();
	double Ramp(double previousPow, double desiredPow);
	double LeftMotor(double &invMaxValueXPlusY);
	double RightMotor(double &invMaxValueXPlusY);

	void SetChangeDirButton(int button) {m_changedirbutton = button;}
	void SetLowSpeedButton(int on, int off) {m_lowspeedbuttonon = on; m_lowspeedbuttonoff = off;}
	void SetLowSpeedMode(bool mode) {m_lowspeedmode = mode;}
	bool GetLowSpeedMode() {return m_lowspeedmode;}

	void enableRamp(bool newRamp) {m_ramp = newRamp;}

	double GetLeftPosition(int encoder = 0);
	double GetRightPosition(int encoder = 0);
	double GetLeftVelocity(int encoder = 0);
	double GetRightVelocity(int encoder = 0);

	Rotation2d GetHeading();
	Pose2d GetPose();
	SimpleMotorFeedforward<units::meters> GetFeedforward();
	DifferentialDriveKinematics GetKinematics();
	DifferentialDriveWheelSpeeds GetWheelSpeeds();
	frc2::PIDController GetLeftPIDController();
	frc2::PIDController GetRightPIDController();
	void SetOutputVolts(double leftvolts, double rightvolts);


protected:
	DriveMode m_mode;
	OperatorInputs *m_inputs;
	WPI_TalonFX *m_left1;
	WPI_TalonFX *m_left2;
	WPI_TalonFX *m_left3;
	WPI_TalonFX *m_right1;
	WPI_TalonFX *m_right2;
	WPI_TalonFX *m_right3;
	bool m_left1owner;
	bool m_left2owner;
	bool m_left3owner;
	bool m_right1owner;
	bool m_right2owner;
	bool m_right3owner;
	SpeedControllerGroup *m_leftscgroup;
	SpeedControllerGroup *m_rightscgroup;
	DifferentialDrive *m_differentialdrive;
	Timer *m_timerramp;

	int m_changedirbutton;
	int m_lowspeedbuttonon;
	int m_lowspeedbuttonoff;

	double m_battery;
	double m_leftpow;
	double m_rightpow;
	double m_leftspeed;
	double m_rightspeed;
	double m_leftposition;
	double m_rightposition;
	
	bool m_ramp;

	double m_prevleftdistance;
	double m_prevrightdistance;

	double m_invertleft;
	double m_invertright;
	double m_direction;

	double m_previousx;
	double m_previousy;
	bool m_lowspeedmode;

	PigeonIMU *m_gyro;
	Pose2d *m_pose;
	DifferentialDriveOdometry *m_odometry;
	SimpleMotorFeedforward<units::meters> *m_feedforward;
	DifferentialDriveKinematics *m_kinematics;
	DifferentialDriveWheelSpeeds *m_wheelspeeds;
	frc2::PIDController *m_leftPID;
	frc2::PIDController *m_rightPID;
};


#endif /* SRC_DriveTrainFX_H_ */
