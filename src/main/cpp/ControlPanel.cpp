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
	static constexpr auto i2cPort = I2C::Port::kOnboard;

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
}


void ControlPanel::Init()
{
	if (m_spinner == nullptr)
		return;

	m_colorsensor->ConfigureColorSensor(rev::ColorSensorV3::ColorResolution::k13bit, rev::ColorSensorV3::ColorMeasurementRate::k25ms);

	m_redCount = 0;
	m_blueCount = 0;
	m_currentcolor = kNone;
	m_stop = true;
	m_previouscolor = kNone;
	m_targetcolor = kNone;
	m_direction = 0;
	m_spinnerstate = kOff;
	m_spinnerstatehelper = 1.0;

	m_spinner->SetNeutralMode(NeutralMode::Brake);
	m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
	m_spinner->SetSensorPhase(true);
	m_spinner->SetInverted(true);
	m_spinner->ConfigPeakOutputForward(1);
	m_spinner->ConfigPeakOutputReverse(-1);
	
	SmartDashboard::PutNumber("CPIN3_TargetColor", m_targetcolor);
	SmartDashboard::PutNumber("CPIN4_SpinnerState", m_spinnerstatehelper);
}


void ControlPanel::Loop()
{
	if (m_spinner == nullptr)
		return;

	ChangeSpinnerState();

	ControlPanelStates();

	SmartDashboard::PutNumber("CPM11_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / COUNTS_PER_CW_REV);
	SmartDashboard::PutNumber("CPM12_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / COUNTS_PER_CW_REV);
	SmartDashboard::PutNumber("CPM15_Motor Voltage", m_spinner->GetMotorOutputVoltage());
	SmartDashboard::PutNumber("CPIN4_SpinnerState", m_spinnerstatehelper);
}


void ControlPanel::Stop()
{
	if (m_spinner == nullptr)
		return;

	m_spinnerstate = kOff;
}



void ControlPanel::ControlPanelStates()
{
	//m_spinnerstate = (SpinnerState) SmartDashboard::GetNumber("CPIN4_SpinnerState", 0);

	switch (m_spinnerstate)
	{
	case kOff:
		m_spinner->Set(ControlMode::PercentOutput, 0);
		break;

	case kRotationControl:
		m_currentcolor = GetColor();	
		
			if (m_currentcolor == kRed && m_previouscolor != kRed)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Red");
				m_redCount ++ ;
			}
			else if (m_currentcolor == kBlue &&  m_previouscolor != kBlue )
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Blue");
				m_blueCount ++ ;
			}	

		if (m_blueCount == ROTATION_CONTROL_COUNT_LIMIT_BLUE || m_redCount == ROTATION_CONTROL_COUNT_LIMIT_RED )
			m_stop = true;
		
		// Once stopped, use A button to restart. If not, set the speed to spinpower
		if (m_stop)
		{
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
			m_currentencodervalue = m_spinner->GetSelectedSensorPosition(0);
			m_spinnersetpoint = ROTATION_CONTROL_FAST;
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint); 
			std::cout << m_currentcolor << ", " << m_previouscolor << ", " << m_redCount << ", " << m_blueCount << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << std::endl;
		}

		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
			m_stop = true;
		
		m_previouscolor = m_currentcolor;
		break;
	
	case kPositionControl:
		m_currentcolor = GetColor();
		m_currentencodervalue = m_spinner->GetSelectedSensorPosition(0);
		if (m_targetcolor == kNone)
			{
				m_targetcolor = GetTargetColor();
				m_colordelta = m_targetcolor - m_currentcolor;
				cout<< "ColorDelta:"<< m_colordelta << endl;
				if (m_colordelta == 1 || m_colordelta == -3)
					m_direction = -1;	
				else
					m_direction = 1;	
				m_spinnersetpoint = POSITION_CONTROL_FAST;
			}
		
		if(m_currentcolor == m_targetcolor)
			{
			if (m_startencodervalue == 0)
				{
				cout << "found target color!" << endl;
				m_startencodervalue = m_spinner->GetSelectedSensorPosition(0);
				}
			else if (abs(m_currentencodervalue - m_startencodervalue) >= COUNTS_PER_CW_SECTOR/2 )
				{
				//cout << "done!" << endl;
				m_stop = true;
				}
			m_spinnersetpoint = POSITION_CONTROL_SLOW;
			}
		else
			{

			m_startencodervalue = 0;
			m_spinnersetpoint = POSITION_CONTROL_FAST;
			}

		
		if (m_stop)
		{
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				m_stop = false;
				m_targetcolor = kNone;
			}
			m_spinner->Set(ControlMode::PercentOutput, 0);
		}
		else
		{
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint * m_direction); 
			std::cout << m_currentcolor << ", " << m_targetcolor << ", " << m_direction << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << ", " << m_startencodervalue << endl;
		}
		
		break;
	}

	// Displays the total registered color count of each color
	//SmartDashboard::PutNumber("CPVER2_Registered Yellow Count", m_colorregisteredcount[0]);
	SmartDashboard::PutNumber("CPVER3_Registered Red Count", m_redCount);
	//SmartDashboard::PutNumber("CPVER4_Registered Green Count", m_colorregisteredcount[2]);
	SmartDashboard::PutNumber("CPVER5_Registered Blue Count", m_blueCount);
}


ControlPanel::ColorOptions ControlPanel::GetColor()
{
	//Color detectedcolor = m_colorsensor->GetColor();
	
	ColorSensorV3::RawColor rawcolor = m_colorsensor->GetRawColor();
	
	double r = static_cast<double>(rawcolor.red);
    double g = static_cast<double>(rawcolor.green);
    double b = static_cast<double>(rawcolor.blue);

	double hue = fmod(((180/3.14 * atan2(sqrt(3) / 2 * (g - b), r - .5*g - .5*b)) + 360), 360);
	double M = fmax(fmax(r, g),b);
	double sat = (M - fmin(fmin(r, g),b)) / M;


	m_color = kNone;

	if (hue > YELLOW_MINIMUM_HUE && hue < YELLOW_MAXIMUM_HUE)
		m_color = kYellow;
	else if (hue > RED_MINIMUM_HUE && hue < RED_MAXIMUM_HUE)
		m_color = kRed;
	else
	if (hue > GREEN_MINIMUM_HUE && hue < GREEN_MAXIMUM_HUE)
		m_color = kGreen;
	else
	if (hue > BLUE_MINIMUM_HUE && hue < BLUE_MAXIMUM_HUE)
		m_color = kBlue;

	SmartDashboard::PutNumber("CPSIMP1_R", r);
	SmartDashboard::PutNumber("CPSIMP2_G", g);
	SmartDashboard::PutNumber("CPSIMP3_B", b);
	SmartDashboard::PutNumber("CPSIMP4_Color", m_color);
	SmartDashboard::PutNumber("CPSIMP5_Hue", hue);
	SmartDashboard::PutNumber("CPSIMP6_Saturation", sat);


	if(!m_stop)
		{
		//std::cout << m_timer->GetFPGATimestamp()<< ", " << rawcolor.red << ", " << rawcolor.green << ", " << rawcolor.blue << ", " << hue << ", " << sat << ", ";
		}


	return m_color;
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
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0))
	{
		m_spinnerstate = kRotationControl;
		cout<<"spinnerState =" << m_spinnerstate << endl;
	}
		
	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0))
	{
		m_spinnerstate = kPositionControl;
		cout<<"spinnerState =" << m_spinnerstate << endl;
	}

}
