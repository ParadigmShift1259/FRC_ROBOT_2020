#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\TimedRobot.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Pneumatics.h"
#include "DrivetrainFX.h"
#include "Turret.h"
#include "ControlPanel.h"


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
    Timer m_timer;
    OperatorInputs *m_operatorinputs;
    DriveTrainFX *m_drivetrain;
	Pneumatics *m_pneumatics;
    Turret *m_turret;
    ControlPanel *m_controlpanel;
};


#endif /* SRC_ROBOT_H_ */