/**
 *  DriveTrain.cpp
 *  Date:
 *  Last Edited By:
 */


#include <Drivetrain.h>
#include "Const.h"
#include <cmath>


using namespace std;


DriveTrain::DriveTrain(OperatorInputs *inputs, WPI_TalonSRX *left1, WPI_TalonSRX *left2, WPI_TalonSRX *left3, WPI_TalonSRX *right1, WPI_TalonSRX *right2, WPI_TalonSRX *right3)
{
	m_inputs = inputs;

	m_mode = kFollower;

	m_left1owner = false;
	m_left2owner = false;
	m_left3owner = false;
	m_right1owner = false;
	m_right2owner = false;
	m_right3owner = false;

	m_left1 = left1;
	m_left2 = left2;
	m_left3 = left3;
	m_right1 = right1;
	m_right2 = right2;
	m_right3 = right3;

	m_leftscgroup = nullptr;
	m_rightscgroup = nullptr;
	m_differentialdrive = nullptr;

	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;

	m_changedirbutton = R3_BUTTON;

	m_leftpow = 0;
	m_rightpow = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_coasting = 1;
	m_invertleft = INVERT_LEFT;
	m_invertright = INVERT_RIGHT;
	m_direction = DT_DEFAULT_DIRECTION;

	m_timerramp = new Timer();
	m_rampmax = RAMPING_RATE_MAX;

	m_prevleftdistance = 0;
	m_prevrightdistance = 0;
}


DriveTrain::~DriveTrain()
{
	if ((m_left1 != nullptr) && m_left1owner)
		delete m_left1;
	if ((m_left2 != nullptr) && m_left2owner)
		delete m_left2;
	if ((m_left3 != nullptr) && m_left3owner)
		delete m_left3;
	if ((m_right1 != nullptr) && m_right1owner)
		delete m_right1;
	if ((m_right2 != nullptr) && m_right2owner)
		delete m_right2;
	if ((m_right3 != nullptr) && m_right3owner)
		delete m_right3;
	delete m_timerramp;
}


void DriveTrain::Init(DriveMode mode)
{
	m_mode = mode;

	if ((m_left1 == nullptr) && (CAN_LEFT_PORT_1 != -1))
	{
		m_left1 = new WPI_TalonSRX(CAN_LEFT_PORT_1);
		m_left1owner = true;
	}
	if ((m_left2 == nullptr) && (CAN_LEFT_PORT_2 != -1))
	{
		m_left2 = new WPI_TalonSRX(CAN_LEFT_PORT_2);
		m_left2owner = true;
	}
	if ((m_left3 == nullptr) && (CAN_LEFT_PORT_3 != -1))
	{
		m_left3 = new WPI_TalonSRX(CAN_LEFT_PORT_3);
		m_left3owner = true;
	}
	if ((m_right1 == nullptr) && (CAN_RIGHT_PORT_1 != -1))
	{
		m_right1 = new WPI_TalonSRX(CAN_RIGHT_PORT_1);
		m_right1owner = true;
	}
	if ((m_right2 == nullptr) && (CAN_RIGHT_PORT_2 != -1))
	{
		m_right2 = new WPI_TalonSRX(CAN_RIGHT_PORT_2);
		m_right2owner = true;
	}
	if ((m_right3 == nullptr) && (CAN_RIGHT_PORT_3 != -1))
	{
		m_right3 = new WPI_TalonSRX(CAN_RIGHT_PORT_3);
		m_right3owner = true;
	}

	if ((m_left1 == nullptr) || (m_left2 == nullptr) || (m_right1 == nullptr) || (m_right2 == nullptr))
		m_mode = kNone;

	switch (m_mode)
	{
	case DriveMode::kFollower:
		m_left1->Set(ControlMode::PercentOutput, 0);
		m_left2->Set(ControlMode::Follower, CAN_LEFT_PORT_1);
		if (m_left3 != nullptr)
			m_left3->Set(ControlMode::Follower, CAN_LEFT_PORT_1);
		m_right1->Set(ControlMode::PercentOutput, 0);
		m_right2->Set(ControlMode::Follower, CAN_RIGHT_PORT_1);
		if (m_right3 != nullptr)
			m_right3->Set(ControlMode::Follower, CAN_RIGHT_PORT_1);
		break;

	case DriveMode::kTank:
	case DriveMode::kArcade:
	case DriveMode::kCurvature:
		if ((m_left3 == nullptr) || (m_right3 == nullptr))
		{
			m_leftscgroup = new SpeedControllerGroup(*m_left1, *m_left2);
			m_rightscgroup = new SpeedControllerGroup(*m_right1, *m_right2);
			m_differentialdrive = new DifferentialDrive(*m_leftscgroup, *m_rightscgroup);
		}
		else
		{
			m_leftscgroup = new SpeedControllerGroup(*m_left1, *m_left2, *m_left3);
			m_rightscgroup = new SpeedControllerGroup(*m_right1, *m_right2, *m_right3);
			m_differentialdrive = new DifferentialDrive(*m_leftscgroup, *m_rightscgroup);
		}
		// @suppress("No break at end of case")

	case DriveMode::kDiscrete:
		m_left1->Set(ControlMode::PercentOutput, 0);
		m_left2->Set(ControlMode::PercentOutput, 0);
		if (m_left3 != nullptr)
			m_left3->Set(ControlMode::PercentOutput, 0);
		m_right1->Set(ControlMode::PercentOutput, 0);
		m_right2->Set(ControlMode::PercentOutput, 0);
		if (m_right3 != nullptr)
			m_right3->Set(ControlMode::PercentOutput, 0);
		break;

	case DriveMode::kNone:
		break;
	}

	if (m_left1 != nullptr)
	{
		// configure encoder
		if (ENC_PRESENT_1)
		{
			m_left1->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
			m_left1->SetSensorPhase(false);
		}
		// configure talon current limiting
		if (MOTOR_CURRENT_LIMIT != -1)
		{
			m_left1->ConfigPeakCurrentLimit(0);
			m_left1->ConfigContinuousCurrentLimit(MOTOR_CURRENT_LIMIT);
			m_left1->EnableCurrentLimit(true);
		}
		m_left1->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_right1 != nullptr)
	{
		// configure encoder
		if (ENC_PRESENT_1)
		{
			m_right1->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
			m_right1->SetSensorPhase(false);
		}
		// configure talon current limiting
		if (MOTOR_CURRENT_LIMIT != -1)
		{
			m_right1->ConfigPeakCurrentLimit(0);
			m_right1->ConfigContinuousCurrentLimit(MOTOR_CURRENT_LIMIT);
			m_right1->EnableCurrentLimit(true);
		}
		m_right1->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_left2 != nullptr)
	{
		// configure encoder
		if (ENC_PRESENT_2)
		{
			m_left2->ConfigSelectedFeedbackSensor(ENC_TYPE_2, 0, 0);
			m_left2->SetSensorPhase(false);
		}
		// configure talon current limiting
		if (MOTOR_CURRENT_LIMIT != -1)
		{
			m_left2->ConfigPeakCurrentLimit(0);
			m_left2->ConfigContinuousCurrentLimit(MOTOR_CURRENT_LIMIT);
			m_left2->EnableCurrentLimit(true);
		}
		m_left2->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_right2 != nullptr)
	{
		// configure encoder
		if (ENC_PRESENT_2)
		{
			m_right2->ConfigSelectedFeedbackSensor(ENC_TYPE_2, 0, 0);
			m_right2->SetSensorPhase(false);
		}
		// configure talon current limiting
		if (MOTOR_CURRENT_LIMIT != -1)
		{
			m_right2->ConfigPeakCurrentLimit(0);
			m_right2->ConfigContinuousCurrentLimit(MOTOR_CURRENT_LIMIT);
			m_right2->EnableCurrentLimit(true);
		}
		m_right2->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_left3 != nullptr)
	{
		// configure talon current limiting
		if (MOTOR_CURRENT_LIMIT != -1)
		{
			m_left3->ConfigPeakCurrentLimit(0);
			m_left3->ConfigContinuousCurrentLimit(MOTOR_CURRENT_LIMIT);
			m_left3->EnableCurrentLimit(true);
		}
		m_left3->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_right3 != nullptr)
	{
		// configure talon current limiting
		if (MOTOR_CURRENT_LIMIT != -1)
		{
			m_right3->ConfigPeakCurrentLimit(0);
			m_right3->ConfigContinuousCurrentLimit(MOTOR_CURRENT_LIMIT);
			m_right3->EnableCurrentLimit(true);
		}
		m_right3->SetNeutralMode(NeutralMode::Brake);
	}

	m_leftpow = 0;
	m_rightpow = 0;
	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_coasting = 1;
	m_timerramp->Reset();
	m_timerramp->Start();
	m_direction = DT_DEFAULT_DIRECTION;

	m_prevleftdistance = 0;
	m_prevrightdistance = 0;
}


void DriveTrain::Loop()
{
	static unsigned int loopcnt = 0;
	double x;
	double y;
	bool tank = false;

	if (m_inputs->xBox(m_changedirbutton, OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		ChangeDirection();

	x = m_inputs->xBoxLeftX(0 * INP_DUAL);
	y = m_inputs->xBoxLeftY(0 * INP_DUAL);

	if ((x == 0.0) && (y == 0.0))
	{
		x = m_inputs->xBoxRightX(0 * INP_DUAL);
		y = m_inputs->xBoxRightY(0 * INP_DUAL);
		tank = true;
	}

	Drive(x, y, !tank, tank);		// ramp if arcade mode, no ramp if tank mode

	SmartDashboard::PutNumber("DT00_direction", m_direction);
	SmartDashboard::PutNumber("DT01_x", x);
	SmartDashboard::PutNumber("DT02_y", y);
	SmartDashboard::PutNumber("DT03_top", CHILD_PROOF_SPEED);
	SmartDashboard::PutNumber("DT04_loop_count", loopcnt);
	SmartDashboard::PutNumber("DT08_abs_x", (abs(m_previousx * X_SCALING) < CHILD_PROOF_SPEED));
	SmartDashboard::PutNumber("DT09_abs_y", (abs(m_previousy * Y_SCALING) < CHILD_PROOF_SPEED));
}


void DriveTrain::Stop()
{

}


void DriveTrain::Drive(double x, double y, bool ramp, bool tank)
{
	double yd = y * m_direction;
	double maxpower;
	double templeft, tempright, tempforward, temprotate;
	bool tempspin;

	if ((x == 0 || yd == 0) || tank)
	{
		maxpower = 1;
	}
	else
	{
		if (abs(x) > abs(yd))
			maxpower = (abs(yd) / abs(x)) + 1;
		else
			maxpower = (abs(x) / abs(yd)) + 1;
	}
	
	if (!ramp)
	{
		m_previousx = x;	//rampInput(previousX, joyStickX, BatteryRampingMin, BatteryRampingMax);
		m_previousy = yd;
		if (!tank)			// convert arcade values to tank power values
		{
			m_leftpow = m_previousy - m_previousx;
			m_rightpow = m_previousy + m_previousx;
		}
		else				// tank values, set the opposite side running to 0
		{
			m_leftpow = m_previousy;
			m_rightpow = m_previousy;
			if (m_previousx < 0)
				m_rightpow = 0;
			else
			if (m_previousx > 0)
				m_leftpow = 0;
		}	
	}
	else
	{
		double battery = DriverStation::GetInstance().GetBatteryVoltage();
		double rampmin = RAMPING_RATE_MIN / battery;
		double rampmax = m_rampmax / battery;
		SmartDashboard::PutNumber("DT10_battery", battery);
		m_previousx = x;	//rampInput(previousX, joyStickX, rampmin, rampmax);
		m_previousy = Ramp(m_previousy, yd, rampmin, rampmax);
		if (!tank)
		{
			m_leftpow = m_previousy * Y_SCALING - (m_previousx * X_SCALING);
			m_rightpow = m_previousy * Y_SCALING + (m_previousx * X_SCALING);
		}
		else
		{
			m_leftpow = m_previousy * Y_SCALING;
			m_rightpow = m_previousy * Y_SCALING;
			if (m_previousx < 0)
				m_rightpow = 0;
			else
			if (m_previousx > 0)
				m_leftpow = 0;
		}
	}

	if (m_left1 != nullptr)
	{
		m_leftspeed = m_left1->GetSelectedSensorVelocity(0);
		m_leftposition = m_left1->GetSelectedSensorPosition(0);
	}
	
	if (m_right1 != nullptr)
	{
		m_rightspeed = m_right1->GetSelectedSensorVelocity(0);
		m_rightposition = m_right1->GetSelectedSensorPosition(0);
	}

	switch (m_mode)
	{
	case DriveMode::kFollower:
		// can talon follower mode
		m_left1->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		m_right1->Set(m_invertright * m_coasting * RightMotor(maxpower));
		break;

	case DriveMode::kDiscrete:
		// can talon discrete mode
		m_left1->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		m_left2->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		if (m_left3 != nullptr)
			m_left3->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		m_right1->Set(m_invertright * m_coasting * RightMotor(maxpower));
		m_right2->Set(m_invertright * m_coasting * RightMotor(maxpower));
		if (m_right3 != nullptr)
			m_right3->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		break;

	case DriveMode::kTank:
		// differentialdrive tank drive
		templeft = m_invertleft * m_coasting * LeftMotor(maxpower);
		tempright = m_invertright * m_coasting * RightMotor(maxpower) * -1;		// -1 added to adjust for DifferentialDrive
		m_differentialdrive->TankDrive(templeft, tempright, false);
		break;

	case DriveMode::kArcade:
		// differentialdrive arcade drive
		templeft = m_invertleft * m_coasting * LeftMotor(maxpower);
		tempright = m_invertright * m_coasting * RightMotor(maxpower) * -1;		// -1 added to adjust for DifferentialDrive
		tempforward = (templeft + tempright) / 2.0;
		temprotate = (templeft - tempright) / 2.0;
		m_differentialdrive->ArcadeDrive(tempforward, temprotate, false);
		break;

	case DriveMode::kCurvature:
		// differentialdrive curvature drive
		templeft = m_invertleft * m_coasting * LeftMotor(maxpower);
		tempright = m_invertright * m_coasting * RightMotor(maxpower) * -1;		// -1 added to adjust for DifferentialDrive
		tempforward = (templeft + tempright) / 2.0;
		temprotate = (templeft - tempright) / 2.0;
		tempspin = abs(tempforward) < DEADZONE_Y;
		m_differentialdrive->CurvatureDrive(tempforward, temprotate, tempspin);
		break;

	case DriveMode::kNone:
		break;
	}

	SmartDashboard::PutNumber("DT11_turningramp", m_previousx); 			//Left Motors are forward=negative
	SmartDashboard::PutNumber("DT12_drivingramp", m_previousy); 			//Right Motors are forward=positive
	SmartDashboard::PutNumber("DT13_leftpow", m_invertleft*m_leftpow); 		//Left Motors are forward=negative
	SmartDashboard::PutNumber("DT14_rightpow", m_invertright*m_rightpow); 	//Right Motors are forward=positive
	SmartDashboard::PutNumber("DT15_gear", m_ishighgear);
	SmartDashboard::PutNumber("DT16_leftspeed", m_leftspeed);
	SmartDashboard::PutNumber("DT17_rightspeed", m_rightspeed);
	SmartDashboard::PutNumber("DT18_leftposition", m_leftposition);
	SmartDashboard::PutNumber("DT19_rightposition", m_rightposition);
	SmartDashboard::PutNumber("DT20_mode", m_mode);
	SmartDashboard::PutNumber("DT21_maxpower", maxpower);
}


// change direction and return true if going forward
bool DriveTrain::ChangeDirection()
{
	m_direction *= -1.0;
	return (m_direction == DT_DEFAULT_DIRECTION);
}


bool DriveTrain::ChangeLowSpeedMode()
{
	m_lowspeedmode = !m_lowspeedmode;
	return m_lowspeedmode;
}


// ramp the power
double DriveTrain::Ramp(double previous, double desired, double rampmin, double rampmax)
{
	double newpow = previous;

	bool timepassed = m_timerramp->HasPeriodPassed(RAMPING_RATE_PERIOD);

	if (timepassed)
	{
		double delta = abs(desired - previous);

		// Makes it so that robot can't go stop to full
		if (delta <= rampmin)
			newpow = desired;
		else
		if (previous < desired)
			newpow += max((delta*rampmax), rampmin);
		else
		if (previous > desired)
			newpow -= max((delta*rampmax), rampmin);
		//lefts1->Set(-previousLeftPow);
	}
	return newpow;
}


double DriveTrain::LeftMotor(double &maxpower)
{
	//moved rightSpeed to class scope, it is being set in setPower()

	double leftpow = m_leftpow * LEFT_MOTOR_SCALING / maxpower;

	/*if (m_leftpow != 0 && m_rightpow != 0)
	{
		m_leftencodermax = abs(m_leftspeed / m_leftpow);
		if (min(abs(m_leftspeed), abs(m_rightspeed)) > ENCODER_TOP_SPEED)
			CheckEncoderTimer();
		if (m_isleftfaster)
			leftpow = m_ratiolr * leftpow;
	}*/
	return leftpow;
}


double DriveTrain::RightMotor(double &maxpower)
{
	//moved rightSpeed to class scope, it is being set in setPower()

	double rightpow = m_rightpow * RIGHT_MOTOR_SCALING / maxpower;

	/*if (m_leftpow != 0 && m_rightpow != 0)
	{
		m_rightencodermax = abs(m_rightspeed / m_rightpow);
		if (min(abs(m_leftspeed), abs(m_rightspeed)) > ENCODER_TOP_SPEED)
			CheckEncoderTimer();
		if (!m_isleftfaster)
			rightpow = m_ratiolr * rightpow;
	}*/
	return rightpow;
}


double DriveTrain::GetLeftPosition(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_left1 != nullptr)
			return m_left1->GetSelectedSensorPosition(0);
		else
			return 0;
	case 1:
		if (m_left2 != nullptr)
			return m_left2->GetSelectedSensorPosition(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetRightPosition(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_right1 != nullptr)
			return m_right1->GetSelectedSensorPosition(0);
		else
			return 0;
	case 1:
		if (m_right2 != nullptr)
			return m_right2->GetSelectedSensorPosition(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetLeftVelocity(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_left1 != nullptr)
			return m_left1->GetSelectedSensorVelocity(0);
		else
			return 0;
	case 1:
		if (m_left2 != nullptr)
			return m_left2->GetSelectedSensorVelocity(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetRightVelocity(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_right1 != nullptr)
			return m_right1->GetSelectedSensorVelocity(0);
		else
			return 0;
	case 1:
		if (m_right2 != nullptr)
			return m_right2->GetSelectedSensorVelocity(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetLeftDistance(int encoder)
{
	double distance = GetLeftPosition(encoder);
	distance = (distance / CODES_PER_REV) * WHEEL_DIAMETER * 3.1415926535;
	return distance;
}


double DriveTrain::GetRightDistance(int encoder)
{
	double distance = GetRightPosition(encoder);
	distance = (distance / CODES_PER_REV) * WHEEL_DIAMETER * 3.1415926535;
	return distance;
}


double DriveTrain::GetMaxDistance(int encoder)
{
	double maxleft = GetLeftDistance(encoder);
	double maxright = GetRightDistance(encoder);
	return abs(maxleft) > abs(maxright) ? -maxleft : maxright;
}


double DriveTrain::GetAverageMaxDistance(int encoder)
{
	double maxleft = GetLeftDistance(encoder);
	double maxright = GetRightDistance(encoder);
	return (abs(maxleft) + abs(maxright)) / 2;
}


double DriveTrain::GetMaxVelocity(int encoder)
{
	double maxleft = GetLeftVelocity(encoder);
	double maxright = GetRightVelocity(encoder);
	return abs(abs(maxleft) > abs(maxright) ? -maxleft : maxright);
}


void DriveTrain::ResetDeltaDistance(int encoder)
{
	m_prevleftdistance = GetLeftDistance(encoder);
	m_prevrightdistance = GetRightDistance(encoder);
}


double DriveTrain::GetMaxDeltaDistance(int encoder)
{
	double maxleft = GetLeftDistance(encoder)- m_prevleftdistance;
	double maxright = GetRightDistance(encoder) - m_prevrightdistance;
	return abs(maxleft) > abs(maxright) ? -maxleft : maxright;
}

