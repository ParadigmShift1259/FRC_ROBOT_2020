/**
 *  DriveTrainFX.cpp
 *  Date:
 *  Last Edited By:
 */


#include "DriveTrainFX.h"
#include "Const.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "frc/DriverStation.h"
#include <cmath>


using namespace std;


DriveTrainFX::DriveTrainFX(OperatorInputs *inputs, WPI_TalonFX *left1, WPI_TalonFX *left2, WPI_TalonFX *left3, WPI_TalonFX *right1, WPI_TalonFX *right2, WPI_TalonFX *right3)
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

	m_lowspeedmode = false;

	m_changedirbutton = R3_BUTTON;
	m_lowspeedbuttonon = XBOX_RIGHT_TRIGGER_AXIS;
	m_lowspeedbuttonoff = XBOX_RIGHT_TRIGGER_AXIS;

	m_battery = 12;
	m_leftpow = 0;
	m_rightpow = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_invertleft = INVERT_LEFT;
	m_invertright = INVERT_RIGHT;
	m_direction = DT_DEFAULT_DIRECTION;

	m_timerramp = new Timer();
	m_ramp = true;

	m_prevleftdistance = 0;
	m_prevrightdistance = 0;

	m_gyro = nullptr;
}


DriveTrainFX::~DriveTrainFX()
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


void DriveTrainFX::Init(DriveMode mode)
{
	m_mode = mode;

	if ((m_left1 == nullptr) && (CAN_LEFT_PORT_1 != -1))
	{
		m_left1 = new WPI_TalonFX(CAN_LEFT_PORT_1);
		m_left1owner = true;
	}
	if ((m_left2 == nullptr) && (CAN_LEFT_PORT_2 != -1))
	{
		m_left2 = new WPI_TalonFX(CAN_LEFT_PORT_2);
		m_left2owner = true;
	}
	if ((m_left3 == nullptr) && (CAN_LEFT_PORT_3 != -1))
	{
		m_left3 = new WPI_TalonFX(CAN_LEFT_PORT_3);
		m_left3owner = true;
	}
	if ((m_right1 == nullptr) && (CAN_RIGHT_PORT_1 != -1))
	{
		m_right1 = new WPI_TalonFX(CAN_RIGHT_PORT_1);
		m_right1owner = true;
	}
	if ((m_right2 == nullptr) && (CAN_RIGHT_PORT_2 != -1))
	{
		m_right2 = new WPI_TalonFX(CAN_RIGHT_PORT_2);
		m_right2owner = true;
	}
	if ((m_right3 == nullptr) && (CAN_RIGHT_PORT_3 != -1))
	{
		m_right3 = new WPI_TalonFX(CAN_RIGHT_PORT_3);
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
		} 																						// @suppress("No break at end of case")
		m_left1->Set(ControlMode::PercentOutput, 0);
		m_left2->Set(ControlMode::PercentOutput, 0);
		if (m_left3 != nullptr)
			m_left3->Set(ControlMode::PercentOutput, 0);
		m_right1->Set(ControlMode::PercentOutput, 0);
		m_right2->Set(ControlMode::PercentOutput, 0);
		if (m_right3 != nullptr)
			m_right3->Set(ControlMode::PercentOutput, 0);
		break;

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

	SupplyCurrentLimitConfiguration supplylimit;
	supplylimit.enable = MOTOR_SUPPLY_LIMIT_ENABLE;
	supplylimit.currentLimit = MOTOR_SUPPLY_CURRENT_LIMIT;
	supplylimit.triggerThresholdCurrent = MOTOR_SUPPLY_THRESHOLD_CURRENT;
	supplylimit.triggerThresholdTime = MOTOR_SUPPLY_THRESHOLD_TIME;
	 
	if (m_left1 != nullptr)
	{
		// configure encoder
		if (ENC_PRESENT_1)
		{
			m_left1->ConfigSelectedFeedbackSensor(ENC_TYPE_1, 0, 0);
			m_left1->SetSensorPhase(false);
		}
		// configure current limiting
		if (MOTOR_SUPPLY_LIMIT_ENABLE)
			m_left1->ConfigSupplyCurrentLimit(supplylimit);
		// config neutral mode
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
		// configure current limiting
		if (MOTOR_SUPPLY_LIMIT_ENABLE)
			m_right1->ConfigSupplyCurrentLimit(supplylimit);
		// config neutral mode
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
		// configure current limiting
		if (MOTOR_SUPPLY_LIMIT_ENABLE)
			m_left2->ConfigSupplyCurrentLimit(supplylimit);
		// config neutral mode
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
		// configure current limiting
		if (MOTOR_SUPPLY_LIMIT_ENABLE)
			m_right2->ConfigSupplyCurrentLimit(supplylimit);
		// config neutral mode
		m_right2->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_left3 != nullptr)
	{
		// configure current limiting
		if (MOTOR_SUPPLY_LIMIT_ENABLE)
			m_left3->ConfigSupplyCurrentLimit(supplylimit);
		// config neutral mode
		m_left3->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_right3 != nullptr)
	{
		// configure current limiting
		if (MOTOR_SUPPLY_LIMIT_ENABLE)
			m_right3->ConfigSupplyCurrentLimit(supplylimit);
		// config neutral mode
		m_right3->SetNeutralMode(NeutralMode::Brake);
	}

	m_battery = 12;
	m_leftpow = 0;
	m_rightpow = 0;
	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_timerramp->Reset();
	m_timerramp->Start();
	m_lowspeedmode = false;
	m_direction = DT_DEFAULT_DIRECTION;
	m_ramp = true;

	m_prevleftdistance = 0;
	m_prevrightdistance = 0;

	m_gyro = new PigeonIMU(0);
	m_pose = new Pose2d();
	m_odometry = new DifferentialDriveOdometry(GetHeading());
	
	m_feedforward = new SimpleMotorFeedforward<units::meters>(0_V, 0 * 1_V * 1_s / 1_m, 0 * 1_V * 1_s * 1_s / 1_m);		// kS, kV, kA
	m_kinematics = new DifferentialDriveKinematics(10_m);		// Wheel track (Distance from left wheel to the right)
	m_wheelspeeds->left = GetLeftVelocity(0) * 1_mps;
	m_wheelspeeds->right = GetRightVelocity(0) * 1_mps;
	m_leftPID = new frc2::PIDController(0, 0, 0);
	m_rightPID = new frc2::PIDController(0, 0, 0);				// P, I, D
}


void DriveTrainFX::Loop()
{
	double x;
	double y;

	if (m_inputs->xBox(m_changedirbutton, OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		ChangeDirection();

	if (m_lowspeedbuttonoff != m_lowspeedbuttonon)
	{
		if (m_inputs->xBox(m_lowspeedbuttonon, OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL) && !m_lowspeedmode)
			ChangeLowSpeedMode();
		else
		if (m_inputs->xBox(m_lowspeedbuttonoff, OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL) && m_lowspeedmode)
			ChangeLowSpeedMode();
	}
	else
	if (m_inputs->xBox(m_lowspeedbuttonoff, OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		ChangeLowSpeedMode();

	x = m_inputs->xBoxLeftX(0 * INP_DUAL);
	y = m_inputs->xBoxLeftY(0 * INP_DUAL);

	if (m_lowspeedmode)
	{
		x = x * LOWSPEED_MODIFIER_X;
		y = y * LOWSPEED_MODIFIER_Y;
	}

	Drive(x, y, m_ramp);

	SmartDashboard::PutNumber("DT00_direction", m_direction);
	SmartDashboard::PutNumber("DT01_x", x);
	SmartDashboard::PutNumber("DT02_y", y);
}


void DriveTrainFX::Stop()
{
	//Drive(0, 0, false);
}


void DriveTrainFX::Drive(double x, double y, bool ramp)
{
	double yd = y * m_direction;
	double maxpower;
	double templeft, tempright, tempforward, temprotate;
	bool tempspin;

	if ((x == 0) || (yd == 0))
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
		m_previousx = x;
		m_previousy = yd;
		m_leftpow = m_previousy - m_previousx;
		m_rightpow = m_previousy + m_previousx;
	}
	else
	{
		m_previousx = x;
		m_previousy = Ramp(m_previousy, yd);
		m_leftpow = m_previousy * JOYSTICK_SCALING_Y - (m_previousx * JOYSTICK_SCALING_X);
		m_rightpow = m_previousy * JOYSTICK_SCALING_Y + (m_previousx * JOYSTICK_SCALING_X);
	}
	double leftpow = LeftMotor(maxpower);
	double rightpow = RightMotor(maxpower);

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
		m_left1->Set(leftpow);
		m_right1->Set(rightpow);
		break;

	case DriveMode::kDiscrete:
		// can talon discrete mode
		m_left1->Set(leftpow);
		m_left2->Set(leftpow);
		if (m_left3 != nullptr)
			m_left3->Set(leftpow);
		m_right1->Set(rightpow);
		m_right2->Set(rightpow);
		if (m_right3 != nullptr)
			m_right3->Set(leftpow);
		break;

	case DriveMode::kTank:
		// differentialdrive tank drive
		templeft = leftpow;
		tempright = rightpow * -1;					// -1 added to adjust for DifferentialDrive
		m_differentialdrive->TankDrive(templeft, tempright, false);
		break;

	case DriveMode::kArcade:
		// differentialdrive arcade drive
		templeft = leftpow;
		tempright = rightpow * -1;					// -1 added to adjust for DifferentialDrive
		tempforward = (templeft + tempright) / 2.0;
		temprotate = (templeft - tempright) / 2.0;
		m_differentialdrive->ArcadeDrive(tempforward, temprotate, false);
		break;

	case DriveMode::kCurvature:
		// differentialdrive curvature drive
		templeft = leftpow;
		tempright = rightpow * -1;					// -1 added to adjust for DifferentialDrive
		tempforward = (templeft + tempright) / 2.0;
		temprotate = (templeft - tempright) / 2.0;
		tempspin = abs(tempforward) < DEADZONE_Y;
		m_differentialdrive->CurvatureDrive(tempforward, temprotate, tempspin);
		break;

	case DriveMode::kNone:
		break;
	}

	SmartDashboard::PutNumber("DT03_leftpower", leftpow);
	SmartDashboard::PutNumber("DT04_rightpower", rightpow);
	SmartDashboard::PutNumber("DT05_leftspeed", m_leftspeed);
	SmartDashboard::PutNumber("DT06_rightspeed", m_rightspeed);
	SmartDashboard::PutNumber("DT07_leftposition", m_leftposition);
	SmartDashboard::PutNumber("DT08_rightposition", m_rightposition);
}


// change direction and return true if going forward
bool DriveTrainFX::ChangeDirection()
{
	m_direction *= -1.0;
	return (m_direction == DT_DEFAULT_DIRECTION);
}


bool DriveTrainFX::ChangeLowSpeedMode()
{
	m_lowspeedmode = !m_lowspeedmode;
	return m_lowspeedmode;
}

// ramp the power
double DriveTrainFX::Ramp(double previous, double desired)
{
	// averages battery voltage over time to reduce affects of brownouts
	double battery = DriverStation::GetInstance().GetBatteryVoltage();
	m_battery = (m_battery + battery) / 2;

	// return value defaults to previous
	double newpow = previous;

	bool timepassed = m_timerramp->HasPeriodPassed(RAMPING_RATE_PERIOD);
	if (timepassed)
	{
		double rampmax = 0;
		double rampmin = 0;
		double delta = abs(desired - previous);

		// if changing direction always use ramp down rates
		// compare signs of prev and desired to determine direction change
		if ((previous / abs(previous)) != (desired / abs(desired)))
		{
			rampmax = RAMPING_RATE_DOWN_MAX / m_battery;
			rampmin = RAMPING_RATE_DOWN_MIN / m_battery;
		}
		else
		// Previous is closer to 0 than desired value, ramping up
		if (abs(previous) < abs(desired))
		{
			rampmax = RAMPING_RATE_UP_MAX / m_battery;
			rampmin = RAMPING_RATE_UP_MIN / m_battery;
		}
		else
		// Previous is further from 0 than desired value, ramping down
		if (abs(previous) > abs(desired))
		{
			rampmax = RAMPING_RATE_DOWN_MAX / m_battery;
			rampmin = RAMPING_RATE_DOWN_MIN / m_battery;
		}

		if (delta < rampmin)
			newpow = desired;
		else
		if (previous < desired)
			newpow += max((delta * rampmax), rampmin);
		else
		if (previous > desired)
			newpow -= max((delta * rampmin), rampmin);
	}
	return newpow;
}

// return left power value
double DriveTrainFX::LeftMotor(double &maxpower)
{
	// take power value from joystick
	// maxpower is calculated between 1 and 2 based on joystick position
	// invert power based on invertleft constant
	// scale power based on scaling constant (to adjust for 1 side running faster than the other)
	double leftpow = m_leftpow / maxpower * m_invertleft * MOTOR_SCALING_LEFT;
	return leftpow;
}


// return right motor power value
double DriveTrainFX::RightMotor(double &maxpower)
{
	// take power value from joystick
	// maxpower is calculated between 1 and 2 based on joystick position
	// invert power based on invertleft constant
	// scale power based on scaling constant (to adjust for 1 side running faster than the other)
	double rightpow = m_rightpow / maxpower * m_invertright * MOTOR_SCALING_RIGHT;
	return rightpow;
}


double DriveTrainFX::GetLeftPosition(int encoder)
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


double DriveTrainFX::GetRightPosition(int encoder)
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


double DriveTrainFX::GetLeftVelocity(int encoder)
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


double DriveTrainFX::GetRightVelocity(int encoder)
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


Rotation2d DriveTrainFX::GetHeading()
{
	// need to fix the conversions
	double ypr[3] = {0, 0, 0};
	m_gyro->GetYawPitchRoll(ypr);
	return Rotation2d(units::radian_t{ypr[0]});
}


Pose2d DriveTrainFX::GetPose()
{
	return *m_pose;
}


SimpleMotorFeedforward<units::meters> DriveTrainFX::GetFeedforward()
{
	return *m_feedforward;
}


DifferentialDriveKinematics DriveTrainFX::GetKinematics()
{
	return *m_kinematics;
}


DifferentialDriveWheelSpeeds DriveTrainFX::GetWheelSpeeds()
{
	return *m_wheelspeeds;
}


frc2::PIDController DriveTrainFX::GetLeftPIDController()
{
	return *m_leftPID;
}


frc2::PIDController DriveTrainFX::GetRightPIDController()
{
	return *m_rightPID;
}



void DriveTrainFX::SetOutputVolts(double leftvolts, double rightvolts)
{

}
