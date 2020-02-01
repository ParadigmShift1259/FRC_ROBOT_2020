#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\TimedRobot.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "DriveStraight.h"
#include "DriveStraightDouble.h"
#include "TurnAngleDegrees.h"
#include "TurnAngleProfiled.h"

#include <frc\smartdashboard\SendableChooser.h>

using namespace frc;
using namespace std;


class Robot : public TimedRobot
{
public:
    /**
     * Selector
     * Drivetrain - Runs Arcade Drive with Left joystick, essentially teleop
     * DriveStraight - Runs Profiled Encoder PID and Gyro PID, 2 sets of PID vals + Profile setup
	 * DriveStraightDouble - Runs 2 Profiled Encoder PIDs, 2 sets of encoder PID vals + Profile setup
     * TurnAngleDegrees - Runs Gyro PID, 1 set of PID vals
     * TurnAngleProfiled - Runs Profiled Gyro PID, 1 set of PID vals + Profile setup
     * RamseteController - Runs Ramsete Controller, 2 sets of PID vals + Profile setup
     */
	enum TestTypes {kDrivetrain, kDriveStraight, kDriveStraightDouble, kTurnAngleDegrees, kTurnAngleProfiled, kRamseteController};

	virtual void RobotInit();
	virtual void RobotPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestInit();
	virtual void TestPeriodic();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();

protected:
	OperatorInputs *m_operatorinputs;
	Drivetrain *m_drivetrain;
	DriveStraight *m_drivestraight;
	DriveStraightDouble *m_drivestraightdouble;
	TurnAngleDegrees *m_turnangledegrees;
	TurnAngleProfiled *m_turnangleprofiled;

	TestTypes m_selector;
	const string scDrivetrain = "Drivetrain";
	const string scDriveStraight = "DriveStraight";
	const string scDriveStraightDouble = "DriveStraightDouble";
	const string scTurnAngleDegrees = "TurnAngleDegrees";
	const string scTurnAngleProfiled = "TurnAngleProfiled";
	const string scRamseteController = "RamseteController";
	SendableChooser<string> m_chooser;
	void ReadChooser();
};


#endif /* SRC_ROBOT_H_ */