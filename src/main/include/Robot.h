#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\WPILib.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Drivetrain.h"
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
    Timer m_timer;
    OperatorInputs *m_operatorinputs;
    Drivetrain *m_drivetrain;
    Turret *m_turret;

};


#endif /* SRC_ROBOT_H_ */