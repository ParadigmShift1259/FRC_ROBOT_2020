/*
 * DrivePID.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Developer
 */


#include "DrivePID.h"
#include "const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "Logger.h"

DrivePID::DrivePID(DriveTrainFX *drivetrain, DualGyro *gyro, OperatorInputs *inputs): PIDSubsystem(0.0, 0.0, 0.0)
{
	m_drivetrain = drivetrain;
	m_gyro = gyro;
	m_inputs = inputs;
	m_p = 0.0;
	m_i = 0.0;
	m_d = 0.0;
	m_y = 0.0;
	m_ramp = false;
	m_feedback = kDisabled;
	m_heading = 0;
	m_ontarget = 0;

	g_log->logMsg(eInfo, __FUNCTION__, __LINE__, "ontarget,heading,p,i,d,y,ramp");
	m_dataInt.push_back(&m_ontarget);
	
	m_dataDouble.push_back(&m_heading);
	m_dataDouble.push_back(&m_p);
	m_dataDouble.push_back(&m_i);
	m_dataDouble.push_back(&m_d);
	m_dataDouble.push_back(&m_y);
	m_dataDouble.push_back(&m_ramp);
}


DrivePID::~DrivePID()
{
}


void DrivePID::Init(double p, double i, double d, Feedback feedback, bool reset)
{
	m_p = p;
	m_i = i;
	m_d = d;
	m_feedback = feedback;
	GetPIDController()->SetPID(m_p, m_i, m_d);
	SetAbsoluteTolerance(2);
	SetOutputRange(-1.0,1.0); // orignially -0.7/0.7
	if (reset)
	{
		SetSetpoint(0);
		m_gyro->ZeroHeading();
	}
	if (feedback != kDisabled)
		EnablePID();
	m_ontarget = 0;
}


void DrivePID::Loop()
{
	double heading;

	if (m_gyro->GetHeading(heading))
		m_heading = heading;

	g_log->logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
}


void DrivePID::Stop()
{
	DisablePID();
	SetAbsoluteAngle(0);
	m_heading = 0;
	m_gyro->ZeroHeading();
	//m_pigeon->SetYaw(0,0);
}


void DrivePID::Drive(double y, bool ramp)
{
	SetY(y);
	m_ramp = ramp;
}


bool DrivePID::GetEnabled()
{
	return GetPIDController()->IsEnabled();
}


void DrivePID::SetP(double p)
{
	m_p = p;
	GetPIDController()->SetPID(m_p, m_i, m_d);
}


void DrivePID::SetI(double i)
{
	m_i = i;
	GetPIDController()->SetPID(m_p, m_i, m_d);
}


void DrivePID::SetD(double d)
{
	m_d = d;
	GetPIDController()->SetPID(m_p, m_i, m_d);
}


void DrivePID::SetY(double y)
{
	m_y = (y >= -1.0) ? ((y <= 1.0) ? y : 1.0) : -1.0;
}


void DrivePID::SetRelativeAngle(double angle)
{
	SetSetpointRelative(angle);
}


void DrivePID::SetAbsoluteAngle(double angle)
{
	SetSetpoint(angle);
}


void DrivePID::ResetGyro()
{
	m_gyro->ZeroHeading();
}


bool DrivePID::IsOnTarget(double count)
{
	if (OnTarget())
	{
		if (count == 0)
		{
			m_ontarget = 0;
			return true;
		}
		else
		{
			m_ontarget++;
			if (m_ontarget > count)
			{
				m_ontarget = 0;
				return true;
			}
			return false;
		}
	}
	m_ontarget = 0;
	return false;
}


void DrivePID::EnablePID()
{
	GetPIDController()->SetPID(m_p, m_i, m_d);
	GetPIDController()->Reset();
	m_ontarget = 0;
	Enable();
}


void DrivePID::DisablePID()
{
	if (GetPIDController()->IsEnabled())
	{
		Disable();
		GetPIDController()->Reset();
	}
}


double DrivePID::ReturnPIDInput()
{
	if (m_feedback == kEncoder)
	{
		double m_leftpos = m_drivetrain->Left1()->GetSelectedSensorPosition(0);
		m_leftpos = m_leftpos / CODES_PER_REV;

		double m_rightpos = m_drivetrain->Right1()->GetSelectedSensorPosition(0);
		m_rightpos = m_rightpos / CODES_PER_REV;

		double retval = (360 / (2 * 3.1415926535)) * (m_leftpos + m_rightpos) * WHEEL_DIAMETER * 3.1415926535 / WHEEL_TRACK;

		SmartDashboard::PutNumber("ReturnPosition(Enc)", m_leftpos + m_rightpos);
		SmartDashboard::PutNumber("ReturnCurrentPosition(Enc)", retval);

		return retval;
	}
	else
	if (m_feedback == kGyro)
	{
		double retval;

		if (!m_gyro->GetHeading(retval))
			retval = 0;

		SmartDashboard::PutNumber("ReturnPIDInput(Gyro)", retval);

		return retval;
	}
	return 0;
}


void DrivePID::UsePIDOutput(double output)
{
	m_drivetrain->Drive(output, m_y, m_ramp);
}
