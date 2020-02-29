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
    m_gyro->Init();
    m_turret->Init();
    m_odo->ResetPosition(Pose2d(0_m, 0_m, 0_rad), Rotation2d(0_deg));
    
    // target pose on field: x, y, rot (field origin = robot starting position and rotation component of target pose is meaningless)...
    m_targetPose = Pose2d(-3_m, 0_m, 0_rad);  // target initial position 3 meters behind robot 180 deg bearing
    // m_targetPose = Pose2d(0_m, 3_m, 0_rad);  // target initial position 3 meters left of robot 90 deg bearing
    // m_targetPose = Pose2d(3_m, _m, 0_rad);  // target initial position 3 meters ahead of robot 0 deg bearing
    // m_targetPose = Pose2d(10_m, 1_m, 0_rad);  // target initial position 10 meters ahead and 1 m left of robot ~6 deg bearing

    // m_PoseHist->clear() may de-allocate the vector memory so instead delete and create new pre-allocating size-10k...
    delete m_StateHist;
    m_StateHist = new vector<Trajectory::State> (10000);

}


void Robot::TeleopPeriodic()
{
    m_gyro->Loop(); // is this needed????

    // read gyro and encoders and use to update odometry...
    double gyroHeadingDegs;
    m_gyro->GetHeading(gyroHeadingDegs);
    Pose2d pose = m_odo->Update(Rotation2d(1_deg*gyroHeadingDegs), m_drivetrain->getLeftDist(), m_drivetrain->getRightDist());

    // store current robot state (i.e. time + pose + vel + accel) in state history list...
    Trajectory::State state;
    state.t = 1_s*m_timer.GetFPGATimestamp();
    state.pose = pose;
    state.velocity = (pose - m_StateHist->end()->pose).Translation().Norm() / (state.t - m_StateHist->end()->t);
    state.acceleration = (state.velocity - m_StateHist->end()->velocity) / (state.t - m_StateHist->end()->t);    
    m_StateHist->push_back(state);

    // compute range and robot-relative bearing angle to target...
    m_targetRange = (double)(m_targetPose - pose).Translation().Norm();
    m_targetBearing = fmod(360 + 180/3.14159 * atan2((double)(m_targetPose - pose).Translation().Y(), (double)(m_targetPose - pose).Translation().X()), 360);

    // point turret at target...
	m_turret->SetTurretAngle(m_targetBearing);

    m_drivetrain->Loop();    // execute teleop

    // cout << "t: " << state.t << "  X: " << state.pose.Translation().X() << "  Y: " << state.pose.Translation().Y() << "  Heading: " << state.pose.Rotation().Degrees() << "  Speed: " << state.velocity() << "  Accel: " << state.acceleration() << endl;

    SmartDashboard::PutNumber("Field X", (double)pose.Translation().X());
    SmartDashboard::PutNumber("Field Y", (double)pose.Translation().Y());
    SmartDashboard::PutNumber("Heading", (double)pose.Rotation().Degrees());
    SmartDashboard::PutNumber("Gyro Heading", gyroHeadingDegs);
    SmartDashboard::PutNumber("Target range", m_targetRange);
    SmartDashboard::PutNumber("Target bearing", m_targetBearing);
	SmartDashboard::PutNumber("Current Turret Angle", m_turret->GetTurretAngle());
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