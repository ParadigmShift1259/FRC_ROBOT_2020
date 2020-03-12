#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


//#include <frc\WPILib.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "Turret.h"
#include "Gyro.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <vector>


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
    vector<Trajectory::State> m_StateHist;
    Pose2d m_targetPose;
<<<<<<< HEAD
    int m_count;
    double m_gyroHeadingDegs;
=======

>>>>>>> 1a39cfe9f382dafd2c1f7a3951b6d998bbb07d20
    double m_targetRange;
    double m_targetBearing;
    double m_velocity;
    double m_acceleration;
    double m_yawPitchRoll[3];

    vector<int*> m_dataInt;
	vector<double*> m_dataDouble;
private: 
    
};


#endif /* SRC_ROBOT_H_ */