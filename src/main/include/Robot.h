#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\TimedRobot.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "DriveStraight.h"
#include "TurnAngleDegrees.h"
#include "TurnAngleProfiled.h"

using namespace frc;


class Robot : public TimedRobot
{
public:
    /**
     * Selector
     * Drivetrain - Runs Arcade Drive with Left joystick, essentially teleop
     * DriveStraight - Runs Profiled Encoder PID and Gyro PID, 2 sets of PID vals + Profile setup
     * TurnAngleDegrees - Runs Gyro PID, 1 set of PID vals
     * TurnAngleProfiled - Runs Profiled Gyro PID, 1 set of PID vals + Profile setup
     * RamseteController - Runs Ramsete Controller, 2 sets of PID vals + Profile setup
     */
	enum TestTypes {kDrivetrain, kDriveStraight, kTurnAngleDegrees, kTurnAngleProfiled, kRamseteController};

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
	TurnAngleDegrees *m_turnangledegrees;
	TurnAngleProfiled *m_turnangleprofiled;

	TestTypes m_selector;
};


#endif /* SRC_ROBOT_H_ */