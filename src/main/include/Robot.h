#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\TimedRobot.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "GyroDrive.h"
#include "Pneumatics.h"
#include "Turret.h"
#include "CDSensors.h"
#include "ControlPanel.h"
#include "Intake.h"
#include "Feeder.h"
#include "Vision.h"


using namespace frc;


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
	CDSensors *m_sensors;
	Vision *m_vision;
	Intake *m_intake;
	Feeder *m_feeder;
	Turret *m_turret;
	ControlPanel *m_controlpanel;
};


#endif /* SRC_ROBOT_H_ */