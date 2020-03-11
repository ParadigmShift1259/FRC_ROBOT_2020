#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\TimedRobot.h>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>

#include "Const.h"
#include "OperatorInputs.h"
#include "GyroDrive.h"
#include "Climber.h"
#include "Pneumatics.h"
#include "Turret.h"
#include "ControlPanel.h"
#include "Intake.h"
#include "Feeder.h"
#include "Vision.h"
#include "Autonomous.h"


using namespace frc;
using namespace std;


class Robot : public TimedRobot
{
public:
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
	frc::Timer m_timer;
	OperatorInputs *m_operatorinputs;
	GyroDrive *m_gyrodrive;
	Pneumatics *m_pneumatics;
	Vision *m_vision;
	Intake *m_intake;
	Feeder *m_feeder;
	Turret *m_turret;
	ControlPanel *m_controlpanel;
	Climber *m_climber;
	Autonomous *m_autonomous;
	DriverStation *m_driverstation;

private:
	SendableChooser<string> m_chooser;
	const string kszNoAuto = "No Auto";
	const string kszSimpleAuto = "Simple";
	const string kszDriveStraight = "DriveStraight";
	const string kszTrenchRun = "TrenchRun";
	const string kszCenterRendezvous = "CenterRendezvous";
	const string kszEnemyTrench = "EnemyTrench";
	string m_autoSelected;

	void ReadChooser();
};


#endif /* SRC_ROBOT_H_ */