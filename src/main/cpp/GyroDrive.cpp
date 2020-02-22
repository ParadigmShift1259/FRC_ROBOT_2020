/**
 *  GryoDrive.cpp
 *  Date:
 *  Last Edited By:
 */


#include "GyroDrive.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <cmath>


using namespace std;


GyroDrive::GyroDrive(OperatorInputs *inputs, Vision *vision)
{
	m_log = g_log;
	m_inputs = inputs;
	m_drivetrain = new DriveTrainFX(inputs, vision);
	m_gyro = new DualGyro(CAN_GYRO1, CAN_GYRO2);
	m_drivepid = new DrivePID(m_drivetrain, m_gyro, m_inputs);

	m_drivemode = kManual;
	m_stage = 0;
	m_drivestate = kInit;
	m_pidstraight[0] = 0.0;
	m_pidstraight[1] = 0.0;
	m_pidstraight[2] = 0.0;
	m_pidangle[0] = 0.0;
	m_pidangle[1] = 0.0;
	m_pidangle[2] = 0.0;
	m_distance = 0;

	m_log->logMsg(eInfo, __FUNCTION__, __LINE__, "drivemode,stage,drivestate,pidstraight[0],pidstraight[1],pidstraight[2],pidangle[0],pidangle[1],pidangle[2],distance");
	m_dataInt.push_back((int*)&m_drivemode);
	m_dataInt.push_back(&m_stage);
	m_dataInt.push_back((int*)&m_drivestate);

	m_dataDouble.push_back(&m_pidstraight[0]);
	m_dataDouble.push_back(&m_pidstraight[1]);
	m_dataDouble.push_back(&m_pidstraight[2]);
	m_dataDouble.push_back(&m_pidangle[0]);
	m_dataDouble.push_back(&m_pidangle[1]);
	m_dataDouble.push_back(&m_pidangle[2]);
	m_dataDouble.push_back(&m_distance);
}


GyroDrive::~GyroDrive()
{
	if (m_drivepid != nullptr)
		delete m_drivepid;
	if (m_gyro != nullptr)
		delete m_gyro;
	if (m_drivetrain != nullptr)
		delete m_drivetrain;
}


void GyroDrive::Init()
{
	m_drivetrain->Init(DriveTrainFX::DriveMode::kFollower);
	// disable change direction in drivetrain
	m_drivetrain->SetChangeDirButton(A_BUTTON);		

	m_gyro->Init();
	m_drivestate = kInit;
	m_timer.Reset();
	m_timer.Start();
	m_drivemode = kManual;
	m_stage = 0;
	m_drivestate = kInit;

	SetStraightPID(AUT_P, AUT_I, AUT_D);
	SetAnglePID(AUT_P, AUT_I, AUT_D);
	m_distance = 0;
}


void GyroDrive::Loop()
{
	m_gyro->Loop();
	if (m_drivepid->GetEnabled())
		m_drivepid->Loop();

	switch (m_drivemode)
	{
	case kManual:
		if (!m_drivepid->GetEnabled())
		{
			m_drivetrain->Loop();
		}
		break;
	}
	if	(m_inputs->xBoxRightTrigger(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		m_drivetrain->SetLowSpeedMode(true);
	if (m_inputs->xBoxRightBumper(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		m_drivetrain->SetLowSpeedMode(false);

	m_log->logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
}


void GyroDrive::Stop()
{
	m_drivetrain->Stop();
	m_drivepid->Stop();
	m_gyro->Stop();
}


void GyroDrive::Disabled()
{
	m_gyro->Loop();
}


void GyroDrive::Drive(double x, double y, bool ramp)
{
	m_drivetrain->Drive(x, y, ramp);
}


void GyroDrive::SetStraightPID(double P, double I, double D)
{
	if (P != -1)
		m_pidstraight[0] = P;
	if (I != -1)
		m_pidstraight[1] = I;
	if (D != -1)
		m_pidstraight[2] = D;
}


void GyroDrive::SetAnglePID(double P, double I, double D)
{
	if (P != -1)
		m_pidangle[0] = P;
	if (I != -1)
		m_pidangle[1] = I;
	if (D != -1)
		m_pidangle[2] = D;
}


void GyroDrive::GetAnglePID(double &P, double &I, double &D)
{
	P = m_pidangle[0];
	I = m_pidangle[1];
	D = m_pidangle[2];
}


bool GyroDrive::DriveStraight(double targetdistance, double autopower, bool reset)
{
	double modifier;

	switch (m_drivestate)
	{
	case kInit:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetDeltaDistance();
		m_distance = m_drivetrain->GetMaxDeltaDistance();
		SmartDashboard::PutNumber("DriveStraight", m_distance);
		m_drivepid->Init(m_pidstraight[0], m_pidstraight[1], m_pidstraight[2], DrivePID::Feedback::kGyro, reset);
		m_drivepid->EnablePID();
		m_timer.Reset();
		m_drivestate = kDrive;
		break;

	case kDrive:
		m_distance = 0;
		if (m_timer.Get() > 0.1)
			m_distance = m_drivetrain->GetMaxDeltaDistance();
		SmartDashboard::PutNumber("DriveStraight", m_distance);
		modifier = (autopower > 0) ? 1 : -1;
		m_distance *= modifier;

		if (m_distance > fabs(targetdistance))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			m_drivestate = kInit;
			return true;
		}
		else
		{
			m_drivepid->Drive(-1 * autopower, true);
		}
		break;
	}
	return false;
}


bool GyroDrive::DriveAngle(double angle, bool reset)
{
	switch (m_drivestate)
	{
	case kInit:
		m_drivepid->Init(m_pidangle[0], m_pidangle[1], m_pidangle[2], DrivePID::Feedback::kGyro, reset);
		m_drivepid->EnablePID();
		m_drivepid->SetRelativeAngle(angle);
		m_drivestate = kDrive;
		break;

	case kDrive:
		m_drivepid->Drive(0, false);
		if (m_drivepid->IsOnTarget(3))
		{
			m_drivepid->DisablePID();
			m_drivestate = kInit;
			return true;
		}
		break;
	}
	return false;
}
