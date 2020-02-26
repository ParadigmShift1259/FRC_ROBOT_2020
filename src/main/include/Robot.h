#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <frc\WPILib.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "Turret.h"
#include "Gyro.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>


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
    DualGyro *m_gyro;
    DifferentialDriveOdometry *m_odo;
    vector<Trajectory::State> *m_StateHist;
    Pose2d m_targetPose;
    double m_targetRange;
    double m_targetBearing;
private: 
    
};


#endif /* SRC_ROBOT_H_ */