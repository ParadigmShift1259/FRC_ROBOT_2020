#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include <frc/TimedRobot.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "Const.h"
#include "OperatorInputs.h"
#include "Turret.h"


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
    OperatorInputs *m_operatorinputs;
    Turret *m_turret;

};


#endif /* SRC_ROBOT_H_ */