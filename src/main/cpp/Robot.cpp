/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include <iostream>
#include <string>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>

#include "Robot.h"

bool Debug = true;

void Robot::RobotInit()
{
    m_operatorinputs = new OperatorInputs();
    m_drivetrain = new Drivetrain(m_operatorinputs);
    m_gyro = new DualGyro(CAN_GYRO1, CAN_GYRO2);
    m_turret = new Turret(m_operatorinputs, m_gyro);

    m_odo = new DifferentialDriveOdometry(Rotation2d(0_deg));
    m_StateHist = new vector<Trajectory::State> (10000);
}


void Robot::RobotPeriodic(){}
void Robot::AutonomousInit(){}
void Robot::AutonomousPeriodic(){}
void Robot::TestInit(){}
void Robot::TestPeriodic(){}


void Robot::TeleopInit()
{
    m_drivetrain->Init();
    m_drivemode = kManual;
    m_gyro->Init();
    m_turret->Init();
    m_odo->ResetPosition(Pose2d(0_m, 0_m, 0_rad), Rotation2d(0_deg));
    
    // target pose on field x, y (rotation of target pose is meaningless)  
    //m_targetPose = Pose2d(10_m, 1_m, 0_rad);  // target initial position 10 meters ahead and 1 m left of robot ~6 deg bearing
    // m_targetPose = Pose2d(3_m, _m, 0_rad);  // target initial position 3 meters ahead of robot 0 deg bearing
    m_targetPose = Pose2d(-3_m, 0_m, 0_rad);  // target initial position 3 meters behind robot 180 deg bearing
    // m_targetPose = Pose2d(0_m, 3_m, 0_rad);  // target initial position 3 meters left of robot 90 deg bearing

    // m_PoseHist->clear() may delatocate memory so instead delete and create new pre-allocating size-10k...
    delete m_StateHist;
    m_StateHist = new vector<Trajectory::State> (10000);
    SmartDashboard::PutNumber("Current Turret Angle", 0);

}


void Robot::TeleopPeriodic()
{
    if (m_operatorinputs->xBoxLeftTrigger(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		m_drivemode = kTracking;
	else
	if (m_operatorinputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		m_drivemode = kManual;

	double ballangle = SmartDashboard::GetNumber("XAngle", 0);
	double balldistance = SmartDashboard::GetNumber("ZDistance", 0);

    switch (m_drivemode)
    {
    case kManual:
        m_drivetrain->Loop();
        break;

    case kTracking:
		if ((balldistance <= 0) || (fabs(ballangle) > 36.0))
		{
			//m_drivemode = kManual;
		}
		else
		{
            double sign = abs(ballangle) / ballangle;
			m_drivetrain->Drive(DT_TRACKING_P * ballangle + DT_TRACKING_FF * sign, m_operatorinputs->xBoxLeftY(0 * INP_DUAL));
		}
        break;
    }

    SmartDashboard::PutNumber("DT_DriveMode", m_drivemode);
    SmartDashboard::PutNumber("DT_BallAngle", ballangle);
    SmartDashboard::PutNumber("DT_Distance", balldistance);
    SmartDashboard::PutNumber("DT_Power", DT_TRACKING_P * ballangle);

    // m_turret->Loop();

    m_gyro->Loop(); // is this needed????

    double gyroHeadingDegs;
    m_gyro->GetHeading(gyroHeadingDegs);
//    units::radian_t gyroHeadingRads{3.14159*gyroHeadingDegs/180};
//    m_odo->Update(Rotation2d(gyroHeadingRads), m_drivetrain->getLeftDist(), m_drivetrain->getRightDist());
    m_odo->Update(Rotation2d(1_deg*gyroHeadingDegs), m_drivetrain->getLeftDist(), m_drivetrain->getRightDist());
    Pose2d pose = m_odo->GetPose();

    Trajectory::State state;
    state.t = 1_s*m_timer.GetFPGATimestamp();
    state.pose = pose;
    state.velocity = (pose - m_StateHist->end()->pose).Translation().Norm() / (state.t - m_StateHist->end()->t);
    state.acceleration = (state.velocity - m_StateHist->end()->velocity) / (state.t - m_StateHist->end()->t);
    
    m_StateHist->push_back(state);

    m_targetRange = (double)(m_targetPose - pose).Translation().Norm();
    double theta = 180/3.14159 * atan2((double)(m_targetPose - pose).Translation().Y(), (double)(m_targetPose - pose).Translation().X());

	m_targetBearing = fmod(theta+360, 360);

	m_turret->SetTurretAngle(m_targetBearing);
	//m_turret->Loop();

    SmartDashboard::PutNumber("Field X", (double)pose.Translation().X());
    SmartDashboard::PutNumber("Field Y", (double)pose.Translation().Y());
    SmartDashboard::PutNumber("Heading", (double)pose.Rotation().Degrees());
    SmartDashboard::PutNumber("Gyro Heading", gyroHeadingDegs);
    SmartDashboard::PutNumber("Target range", m_targetRange);
    SmartDashboard::PutNumber("Target bearing", m_targetBearing);
	 SmartDashboard::PutNumber("Current Turret Angle", m_turret->GetTurretAngle());
    /*
    double cmdAngle = SmartDashboard::GetNumber("Current Turret Angle", 0);
    cout << cmdAngle << endl;
    m_turret->SetTurretAngle(cmdAngle);
    */
 //   cout << "t: " << state.t << "  X: " << state.pose.Translation().X() << "  Y: " << state.pose.Translation().Y() << "  Heading: " << state.pose.Rotation().Degrees() << "  Speed: " << state.velocity() << "  Accel: " << state.acceleration() << endl;
}


void Robot::DisabledInit()
{
    m_drivetrain->Stop();
    //m_turret->Stop();
}


void Robot::DisabledPeriodic(){}


int main()
{
    return frc::StartRobot<Robot>();
}