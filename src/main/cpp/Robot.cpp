/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Logger.h"

#include "Robot.h"
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#ifdef USE_ODO
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#endif

bool Debug = true;
Logger *g_log = nullptr;
bool StartedInAuto = false;

AutoMode automode = kNoAuto;


void Robot::RobotInit()
{
	g_log = new Logger("/tmp/logfile.csv", true);

	m_chooser.SetDefaultOption(kszNoAuto, kszNoAuto);
	m_chooser.AddOption(kszSimpleAuto, kszSimpleAuto);
	m_chooser.AddOption(kszDriveStraight, kszDriveStraight);
	m_chooser.AddOption(kszTrenchRun, kszTrenchRun);
	m_chooser.AddOption(kszCenterRendezvous, kszCenterRendezvous);
	SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driverstation = &DriverStation::GetInstance();
#if 1
	m_operatorinputs = new OperatorInputs();
	m_vision = new Vision(m_operatorinputs);
	m_gyrodrive = new GyroDrive(m_operatorinputs, m_vision);
	m_pneumatics = new Pneumatics();
	m_intake = new Intake(m_operatorinputs, m_vision);
	m_feeder = new Feeder(m_operatorinputs, m_intake);
	m_controlpanel = new ControlPanel(m_operatorinputs, m_gyrodrive, m_intake);
	m_climber = new Climber(m_operatorinputs);
	m_turret = new Turret(m_operatorinputs, m_gyrodrive, m_intake, m_feeder, m_controlpanel, m_climber, m_vision);
	m_autonomous = new Autonomous(m_gyrodrive, m_intake, m_feeder, m_turret, m_vision);
#ifdef USE_ODO
    m_odo = new DifferentialDriveOdometry(Rotation2d(0_deg));
    m_StateHist = new vector<Trajectory::State>;
	m_StateHist->reserve(10000);
#endif
#endif
}


void Robot::RobotPeriodic()
{
}


void Robot::AutonomousInit()
{
	g_log->openLog("/tmp/logfile.csv");

	ReadChooser();

	StartedInAuto = true;

#if 1
	m_autonomous->Init();
	m_gyrodrive->Init();
	m_pneumatics->Init();
	m_vision->Init();
	m_intake->Init();
	m_feeder->Init();
	m_feeder->SetLoaded(true);		// ensure feeder state is loaded before loop runs
	m_turret->Init();
	//m_controlpanel->Init();
	m_climber->Init();

#ifdef USE_ODO
    m_odo->ResetPosition(Pose2d(0_m, 0_m, 0_rad), Rotation2d(0_deg));

    // target pose on field: x, y, rot (field origin = robot starting position and rotation component of target pose is meaningless)...
    m_targetPose = Pose2d(-3_m, 0_m, 0_rad);  // target initial position 3 meters behind robot 180 deg bearing
    // m_targetPose = Pose2d(0_m, 3_m, 0_rad);  // target initial position 3 meters left of robot 90 deg bearing
    // m_targetPose = Pose2d(3_m, _m, 0_rad);  // target initial position 3 meters ahead of robot 0 deg bearing
    // m_targetPose = Pose2d(10_m, 1_m, 0_rad);  // target initial position 10 meters ahead and 1 m left of robot ~6 deg bearing

    // m_PoseHist->clear() may de-allocate the vector memory so instead delete and create new pre-allocating size-10k...
	m_StateHist->clear();	// clearing does not deallocate the memory
#endif
#endif
}


void Robot::AutonomousPeriodic()
{ 
#if 1
	m_autonomous->Loop();
	m_gyrodrive->Loop();
	m_vision->Loop();
	m_intake->Loop();
	m_feeder->Loop();
	m_turret->Loop();
	m_operatorinputs->Loop();		// For logging
#endif
}


void Robot::TestInit()
{    
}


void Robot::TestPeriodic()
{
}


void Robot::TeleopInit()
{
	g_log->openLog("/tmp/logfile.csv");
	
	if (!StartedInAuto)
	{
#if 1
		m_gyrodrive->Init();
		m_pneumatics->Init();
		m_vision->Init();
		m_intake->Init();
		m_feeder->Init();
		m_turret->Init();
		m_controlpanel->Init();
		m_climber->Init();
#endif
	}
}


void Robot::TeleopPeriodic()
{
#if 1
#ifdef USE_ODO
    // read gyro and encoders and use to update odometry...
    double gyroHeadingDegs;
    m_gyrodrive->GetHeading(gyroHeadingDegs);
    Pose2d pose = m_odo->Update(Rotation2d(1_deg * gyroHeadingDegs), m_gyrodrive->GetLeftDistance(), m_gyrodrive->GetRightDistance());

    // store current robot state (i.e. time + pose + vel + accel) in state history list...
    Trajectory::State state;
    state.t = 1_s * m_timer.GetFPGATimestamp();
    state.pose = pose;
	if (!m_StateHist->empty())
	{
	    state.velocity = (pose - m_StateHist->back()->pose).Translation().Norm() / (state.t - m_StateHist->back()->t);
    	state.acceleration = (state.velocity - m_StateHist->back()->velocity) / (state.t - m_StateHist->back()->t);    
	}
	else
	{
		state.velocity = (pose - m_StateHist->back()->pose).Translation().Norm() / (state.t - m_StateHist->back()->t);
		state.acceleration = (state.velocity - m_StateHist->back()->velocity) / (state.t - m_StateHist->back()->t);    
	}
    m_StateHist->push_back(state);

    // compute range and robot-relative bearing angle to target...
    m_targetRange = (double)(m_targetPose - pose).Translation().Norm();
    m_targetBearing = fmod(360 + 180/3.14159 * atan2((double)(m_targetPose - pose).Translation().Y(), (double)(m_targetPose - pose).Translation().X()), 360);

    // point turret at target...
	m_turret->SetTurretAngle(m_targetBearing);

    // cout << "t: " << state.t << "  X: " << state.pose.Translation().X() << "  Y: " << state.pose.Translation().Y() << "  Heading: " << state.pose.Rotation().Degrees() << "  Speed: " << state.velocity() << "  Accel: " << state.acceleration() << endl;

    SmartDashboard::PutNumber("Field X", (double)pose.Translation().X());
    SmartDashboard::PutNumber("Field Y", (double)pose.Translation().Y());
    SmartDashboard::PutNumber("Heading", (double)pose.Rotation().Degrees());
    SmartDashboard::PutNumber("Gyro Heading", gyroHeadingDegs);
    SmartDashboard::PutNumber("Target range", m_targetRange);
    SmartDashboard::PutNumber("Target bearing", m_targetBearing);
	SmartDashboard::PutNumber("Current Turret Angle", m_turret->GetTurretAngle());
#endif
	m_gyrodrive->Loop();
	m_pneumatics->Loop();
	m_vision->Loop();
	m_intake->Loop();
	m_feeder->Loop();
	m_turret->Loop();
	m_controlpanel->Loop();
	m_climber->Loop();
	m_operatorinputs->Loop();		// For logging
#endif
}


void Robot::DisabledInit()
{
	if (!StartedInAuto)
	{
#if 1
		m_gyrodrive->Stop();
		m_pneumatics->Stop();
		m_vision->Stop();
		m_intake->Stop();
		m_feeder->Stop();
		m_turret->Stop();
		//m_controlpanel->Stop();
		m_climber->Stop();
#endif
	}
	else
	{
		m_autonomous->Stop();
	}
	g_log->closeLog();
}


void Robot::DisabledPeriodic()
{
	ReadChooser();
	m_vision->SetLED(false);
}


void Robot::ReadChooser()
{
	m_autoSelected = m_chooser.GetSelected();

	automode = kNoAuto;
	if (m_autoSelected == kszNoAuto)
		automode = kNoAuto;
	else
	if (m_autoSelected == kszSimpleAuto)
		automode = kSimpleAuto;
	else
	if (m_autoSelected == kszDriveStraight)
		automode = kDriveStraight;
	else
	if (m_autoSelected == kszTrenchRun)
		automode = kTrenchRun;
	else
	if (m_autoSelected == kszCenterRendezvous)
		automode = kCenterRendezvous;

	SmartDashboard::PutNumber("AU1_automode", automode);
}


int main()
{
	return frc::StartRobot<Robot>();
}
