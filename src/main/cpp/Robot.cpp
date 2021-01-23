/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//#include <iostream>
#include <string>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>

#include "Robot.h"
#include "Logger.h"

bool Debug = true;
Logger *g_log = nullptr;


void Robot::RobotInit()
{
	g_log = new Logger("/tmp/logfile.csv", false);
    m_operatorinputs = new OperatorInputs();
    m_drivetrain = new Drivetrain(m_operatorinputs);
    m_gyro = new DualGyro(CAN_GYRO1, CAN_GYRO2);
    m_turret = new Turret(m_operatorinputs, m_gyro);

    m_odo = new DifferentialDriveOdometry(Rotation2d(0_deg));
    m_StateHist.reserve(10000);

	m_count = 0;
	g_log->logMsg(eInfo, __FUNCTION__, __LINE__, "count,velocity,acceleration,targetrange,targetbearing,gyroHeadingDegs,yaw,pitch,roll");
	m_dataInt.push_back(&m_count);

	m_dataDouble.push_back(&m_velocity);	
	m_dataDouble.push_back(&m_acceleration);	
	m_dataDouble.push_back(&m_targetRange);	
	m_dataDouble.push_back(&m_targetBearing);
    m_dataDouble.push_back(&m_gyroHeadingDegs);
	m_dataDouble.push_back(&m_yawPitchRoll[0]);	
	m_dataDouble.push_back(&m_yawPitchRoll[1]);	
	m_dataDouble.push_back(&m_yawPitchRoll[2]);	
}


void Robot::RobotPeriodic(){}
void Robot::AutonomousInit()
{
	g_log->openLog();
}

void Robot::AutonomousPeriodic(){}
void Robot::TestInit(){}
void Robot::TestPeriodic(){}


void Robot::TeleopInit()
{
	g_log->openLog();

    m_drivetrain->Init(Drivetrain::DriveMotors::k6Motors);
    m_gyro->Init();
    m_turret->Init();
    m_odo->ResetPosition(Pose2d(0_m, 0_m, 0_rad), Rotation2d(0_deg));
    
    // target pose on field: x, y, rot (field origin = robot starting position and rotation component of target pose is meaningless)...
    m_targetPose = Pose2d(-3_m, 0_m, 0_rad);  // target initial position 3 meters behind robot 180 deg bearing
    // m_targetPose = Pose2d(0_m, 3_m, 0_rad);  // target initial position 3 meters left of robot 90 deg bearing
    // m_targetPose = Pose2d(3_m, _m, 0_rad);  // target initial position 3 meters ahead of robot 0 deg bearing
    // m_targetPose = Pose2d(10_m, 1_m, 0_rad);  // target initial position 10 meters ahead and 1 m left of robot ~6 deg bearing

    // clear() does not delatocate memory
    m_StateHist.clear();	
}


void Robot::TeleopPeriodic()
{
    m_gyro->Loop(); // is this needed????

    m_turret->Loop();

    m_gyro->GetHeading(m_gyroHeadingDegs);
	m_gyro->GetYawPitchRoll(m_yawPitchRoll);

//    units::radian_t gyroHeadingRads{3.14159*gyroHeadingDegs/180};
//    m_odo->Update(Rotation2d(gyroHeadingRads), m_drivetrain->getLeftDist(), m_drivetrain->getRightDist());
    m_odo->Update(Rotation2d(1_deg * m_gyroHeadingDegs), m_drivetrain->getLeftDist(), m_drivetrain->getRightDist());
    Pose2d pose = m_odo->GetPose();

    // store current robot state (i.e. time + pose + vel + accel) in state history list...
    Trajectory::State state;
    state.t = 1_s * m_timer.GetFPGATimestamp();
    state.pose = pose;
	auto& prevState = m_StateHist.back();
    state.velocity = (pose - prevState.pose).Translation().Norm() / (state.t - prevState.t);
    state.acceleration = (state.velocity - prevState.velocity) / (state.t - prevState.t);
    m_velocity = (double)state.velocity;
    m_acceleration = (double)state.acceleration;

    m_StateHist.push_back(state);

    // compute range and robot-relative bearing angle to target...
    m_targetRange = (double)(m_targetPose - pose).Translation().Norm();
    m_targetBearing = fmod(360 + 180/3.14159 * atan2((double)(m_targetPose - pose).Translation().Y(), (double)(m_targetPose - pose).Translation().X()), 360);

    // point turret at target...
	m_turret->SetTurretAngle(m_targetBearing);

    m_drivetrain->Loop();    // execute teleop

    // cout << "t: " << state.t << "  X: " << state.pose.Translation().X() << "  Y: " << state.pose.Translation().Y() << "  Heading: " << state.pose.Rotation().Degrees() << "  Speed: " << state.velocity() << "  Accel: " << state.acceleration() << endl;

    m_count++;
	g_log->logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);

    SmartDashboard::PutNumber("Field X", (double)pose.Translation().X());
    SmartDashboard::PutNumber("Field Y", (double)pose.Translation().Y());
    SmartDashboard::PutNumber("Heading", (double)pose.Rotation().Degrees());
    SmartDashboard::PutNumber("Gyro Heading", m_gyroHeadingDegs);
    SmartDashboard::PutNumber("Target range", m_targetRange);
    SmartDashboard::PutNumber("Target bearing", m_targetBearing);
	SmartDashboard::PutNumber("Current Turret Angle", m_turret->GetTurretAngle());
	SmartDashboard::PutNumber("Yaw", m_yawPitchRoll[0]);	
	SmartDashboard::PutNumber("Pitch", m_yawPitchRoll[1]);	
	SmartDashboard::PutNumber("Roll", m_yawPitchRoll[2]);	
}


void Robot::DisabledInit()
{
    m_drivetrain->Stop();
    //m_turret->Stop();
	g_log->closeLog();
}


void Robot::DisabledPeriodic(){}


int main()
{
    return frc::StartRobot<Robot>();
}