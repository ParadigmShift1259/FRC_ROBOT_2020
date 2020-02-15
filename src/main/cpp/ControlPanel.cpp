/**
 *  ControlPanel.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Jival Chandrashekar
 */


#include "ControlPanel.h"
#include "Const.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "math.h"



using namespace std;


ControlPanel::ControlPanel(OperatorInputs *inputs)
{
	static constexpr auto i2cPort = I2C::Port::kOnboard;  // To Do: Move to higher level robot class

	m_inputs = inputs;
	
	m_spinner = nullptr;

	if (CPL_MOTOR != -1)
		m_spinner = new TalonSRX(CPL_MOTOR);

	m_colorsensor = new ColorSensorV3(i2cPort);
	m_timer = new Timer();

	m_stop = false;

	// P, I, D, FF, Iz, nominal, peak
	// PID Values NOT YET tuned for talonSRX Minicim on Woody 1/22/20
	m_spinnerPIDvals[0] = 0.0; 
	m_spinnerPIDvals[1] = 0.0; 
	m_spinnerPIDvals[2] = 0.0; 
	m_spinnerPIDvals[3] = 0.0;
	m_spinnerPIDvals[4] = 0;
	m_spinnerPIDvals[5] = 0;
	m_spinnerPIDvals[6] = 1;
	
	m_spinnersetpoint = 0;
}


ControlPanel::~ControlPanel()
{
	if (m_colorsensor != nullptr)
		delete m_colorsensor;
	if (m_spinner != nullptr)
		delete m_spinner;
	if (m_timer != nullptr)
		delete m_timer;
}


void ControlPanel::Init()
{
	if (m_spinner == nullptr || m_colorsensor == nullptr)
		return;

	m_colorsensor->ConfigureColorSensor(rev::ColorSensorV3::ColorResolution::k13bit, rev::ColorSensorV3::ColorMeasurementRate::k25ms);

	m_redCount = 0;
	m_blueCount = 0;
	m_currentcolor = kNone;
	m_previouscolor = kNone;
	m_targetcolor = kNone;
	m_direction = 0;
	m_spinnerstate = kOff;
	m_stop = true;

	m_spinner->SetNeutralMode(NeutralMode::Brake);
	m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
	m_spinner->SetSensorPhase(true); // What does this do / is this needed?
	m_spinner->SetInverted(true); // What does this do / is this needed?
	m_spinner->ConfigPeakOutputForward(1);
	m_spinner->ConfigPeakOutputReverse(-1);
	
}


void ControlPanel::Loop()
{
	if (m_spinner == nullptr || m_colorsensor == nullptr)
		return;

	ChangeSpinnerState();

	ControlPanelStates();

	SmartDashboard::PutNumber("CPM11_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / COUNTS_PER_CW_REV);
	SmartDashboard::PutNumber("CPM12_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / COUNTS_PER_CW_REV);
	SmartDashboard::PutNumber("CPM15_Motor Voltage", m_spinner->GetMotorOutputVoltage());
	SmartDashboard::PutNumber("CPVER3_Registered Red Count", m_redCount);
	SmartDashboard::PutNumber("CPVER5_Registered Blue Count", m_blueCount);
}


void ControlPanel::Stop()
{
	m_spinnerstate = kOff;
}



void ControlPanel::ControlPanelStates()
{

	switch (m_spinnerstate)
	{
	case kOff:
		m_spinner->Set(ControlMode::PercentOutput, 0);
		m_stop = true;
		break;

	case kRotationControl:
		m_currentcolor = GetColor();	
		
			if (m_currentcolor == kRed && m_previouscolor != kRed)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Red");
				m_redCount++;
			}
			else if (m_currentcolor == kBlue &&  m_previouscolor != kBlue )
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Blue");
				m_blueCount++;
			}	

		if (m_blueCount == ROTATION_CONTROL_COUNT_LIMIT_BLUE || m_redCount == ROTATION_CONTROL_COUNT_LIMIT_RED )
			m_stop = true;
		
		if (m_stop)
		{
			// Once stopped, use A button to restart. (TO DO: Move reading user inputs to a higher level robot class)
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				m_stop = false;
				m_redCount = 0;
				m_blueCount = 0;
			}
			m_spinner->Set(ControlMode::PercentOutput, 0);
		}
		else
		{
			//Command motor to spin at a given speed
			m_spinnersetpoint = ROTATION_CONTROL_FAST;
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint); 
			m_currentencodervalue = m_spinner->GetSelectedSensorPosition(0);
			//std::cout << m_currentcolor << ", " << m_previouscolor << ", " << m_redCount << ", " << m_blueCount << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << std::endl;
		}

		// (TO DO: Move reading user inputs to a higher level robot class)
		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0)) 
			m_stop = true;
		
		m_previouscolor = m_currentcolor;
		break;
	
	case kPositionControl:
		m_currentcolor = GetColor();
		m_currentencodervalue = m_spinner->GetSelectedSensorPosition(0);
		
		if (m_targetcolor == kNone)
			{
			//If we don't know what the target color is, read the target color from FMS and 
			m_targetcolor = GetTargetColor();
			if(m_targetcolor != kNone)
				{
				//determine the shortest path to spin the wheel
				int colordelta = m_targetcolor - m_currentcolor;
				if (colordelta == 1 || colordelta == -3)
					m_direction = -1;	
				else
					m_direction = 1;
				//Set motor speed	
				m_spinnersetpoint = POSITION_CONTROL_FAST;
				}
			}
		
		if(m_currentcolor == m_targetcolor)
			{
			m_spinnersetpoint = POSITION_CONTROL_SLOW;
			if (m_startencodervalue == 0)
				{
				//cout << "found target color!" << endl;
				//Record the current encoder value
				m_startencodervalue = m_spinner->GetSelectedSensorPosition(0);
				}
			else if (abs(m_currentencodervalue - m_startencodervalue) >= COUNTS_PER_CW_SECTOR/2 )
				{
				//cout << "done!" << endl;
				//if the wheel has spun half a sector, then stop the motor
				m_stop = true;
				}
			}
		else
			{
			//If not over the target color, spin fast and clear any encoder value that we recorded
			m_startencodervalue = 0;
			m_spinnersetpoint = POSITION_CONTROL_FAST;
			}

		
		if (m_stop)
		{
			m_spinner->Set(ControlMode::PercentOutput, 0);
			// (TO DO: Move reading user inputs to a higher level robot class)
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				// If A button is pressed, clear the target color and set m_stop=false to initiate Position Control 
				m_stop = false;
				m_targetcolor = kNone;
			}
		}
		else
		{
			//If not stopped command motor according to setpoint and direction
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint * m_direction); 
			std::cout << m_currentcolor << ", " << m_targetcolor << ", " << m_direction << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << ", " << m_startencodervalue << endl;
		}
		
		break;
	}
}


ControlPanel::ColorOptions ControlPanel::GetColor()
{
	ColorOptions color;
	
	ColorSensorV3::RawColor rawcolor = m_colorsensor->GetRawColor();
	
	double r = static_cast<double>(rawcolor.red);
    double g = static_cast<double>(rawcolor.green);
    double b = static_cast<double>(rawcolor.blue);

	double hue = fmod(((180/3.14 * atan2(sqrt(3) / 2 * (g - b), r - .5*g - .5*b)) + 360), 360);
	double M = fmax(fmax(r, g),b);
	double sat = (M - fmin(fmin(r, g),b)) / M;
	double val = (M/pow(2, 13));

	color = kNone;

	if (hue > YELLOW_MINIMUM_HUE && hue < YELLOW_MAXIMUM_HUE)
		color = kYellow;
	else if (hue > RED_MINIMUM_HUE && hue < RED_MAXIMUM_HUE)
		color = kRed;
	else if (hue > GREEN_MINIMUM_HUE && hue < GREEN_MAXIMUM_HUE)
		color = kGreen;
	else if (hue > BLUE_MINIMUM_HUE && hue < BLUE_MAXIMUM_HUE)
		color = kBlue;

	return color;
}

ControlPanel::ColorOptions ControlPanel::GetTargetColor()
{
	string message = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	/*
	Code converts the FMS data and changes the color to the color 90 degress from it because the game 
	looks for the colors underneath the bar on the control pannel
	*/
	switch(message[0])
	{
		case 'Y' :
			return kGreen;
			break;
		case 'R' :
			return kBlue;
			break;
		case 'G' :
			return kYellow;
			break;
		case 'B' :
			return kRed;
			break;
	}

	return  kNone;
}

void ControlPanel::ChangeSpinnerState()
{
	//TODO: Move reading of operator inputs to higher level class
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
	{
		m_spinnerstate = kRotationControl;
	}
		
	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
	{
		m_spinnerstate = kPositionControl;
	}
}

void ControlPanel::ChangeSpinnerState(SpinnerState state)
{
	m_spinnerstate = state;
}
