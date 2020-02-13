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


static constexpr Color kYellowTarget = Color(.323364, .588501, .088013);
static constexpr Color kRedTarget = Color(0.580933, 0.310425, 0.108521);    
static constexpr Color kGreenTarget = Color(0.118286, 0.574829, .302368);
static constexpr Color kBlueTarget = Color(0.084595, 0.361450, 0.553833);


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

	m_spinnerstate = kColorSpin;

	m_colormatcher.AddColorMatch(kYellowTarget);
	m_colormatcher.AddColorMatch(kRedTarget);
	m_colormatcher.AddColorMatch(kGreenTarget);
	m_colormatcher.AddColorMatch(kBlueTarget);
	m_confidence = 0.6;
	m_redCount = 0;
	m_blueCount = 0;
	m_currentcolor = kNone;
	m_stop = true;
	m_previouscolor = kNone;
	m_targetcolor = kNone;
	m_direction = 0;

	SmartDashboard::PutNumber("CPIN1_SpinnerSetpoint", m_spinnersetpoint);
	SmartDashboard::PutNumber("CPIN2_Confidence", m_confidence);
	SmartDashboard::PutNumber("CPIN3_TargetColor", m_targetcolor);

	m_spinner->SetNeutralMode(NeutralMode::Brake);
	m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
	m_spinner->SetSensorPhase(true);
	m_spinner->SetInverted(true);
	m_spinner->ConfigPeakOutputForward(1);
	m_spinner->ConfigPeakOutputReverse(-1);
	m_spinnersetpoint = 0;
}


void ControlPanel::Loop()
{
	if (m_spinner == nullptr)
		return;

	ControlPanelStates();


	SmartDashboard::PutNumber("CPM11_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / ENCODER_TICKS_PER_REV);
	SmartDashboard::PutNumber("CPM12_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / ENCODER_TICKS_PER_REV);
	SmartDashboard::PutNumber("CPM15_Motor Voltage", m_spinner->GetMotorOutputVoltage());
}


void ControlPanel::Stop()
{
	if (m_spinner == nullptr)
		return;

	m_spinnerstate = kOff;
	m_spinnersetpoint = 0;
}


void ControlPanel::ControlPanelStates()
{
	m_spinnersetpoint = SmartDashboard::GetNumber("CPIN1_SpinnerSetpoint", 0);
	m_confidence = SmartDashboard::GetNumber("CPIN2_Confidence", 0);
	//cout << "spinner state =" << m_spinnerstate<< endl;
	//m_targetcolor = SmartDashboard::GetNumber("CPIN3_TargetColor", 0);

	switch (m_spinnerstate)
	{
	case kOff:
		m_spinner->Set(ControlMode::PercentOutput, 0);
		break;

	case kBlindSpin:
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

		if (m_blueCount == 21 || m_redCount == 21 )
			m_stop = true;
		
		// Once stopped, use A button to restart. If not, set the speed to spinpower
		if (m_stop)
		{
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				std::cout << "TimeStamp, rawcolor.red, rawcolor.green, rawcolor.blue, hue, sat, m_currentcolor, m_previouscolor, m_RedCount, m_BlueCount, m_spinnersetpoint" << std::endl;
				m_stop = false;
				m_redCount = 0;
				m_blueCount = 0;
			}
			m_spinner->Set(ControlMode::PercentOutput, 0);
		}
		else
		{
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint); 
			std::cout << m_currentcolor << ", " << m_previouscolor << ", " << m_redCount << ", " << m_blueCount << ", " << m_spinnersetpoint <<  std::endl;
		}

		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
			m_stop = true;
		
		m_previouscolor = m_currentcolor;
		break;
	
	case kColorSpin:
		m_currentcolor = GetColor();
		if (m_targetcolor == kNone)
			{
				m_targetcolor = GetTargetColor();
				m_colordelta = m_targetcolor - m_currentcolor;
				cout<< "ColorDelta:"<< m_colordelta << endl;
				m_stop = false;
				if (m_colordelta == 1 || m_colordelta == -3)
					m_direction = -1;	//m_spinnersetpoint
				else
					m_direction = 1;
					m_spinnersetpoint = 0.15;
					
			}
		
		if(m_currentcolor == m_targetcolor)
			{
			if (m_startencodervalue == 0)
				m_startencodervalue = m_spinner->GetSelectedSensorPosition(0);
			else if (abs(m_currentencodervalue - m_startencodervalue) >= COUNTS_PER_CW_SECTOR/2 )
				m_stop = true;
			m_spinner -> Set(ControlMode::PercentOutput, 0.125 * m_direction);
			}
		else
			m_startencodervalue = 0;

		
		if (m_stop)
		{
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				std::cout << "TimeStamp, rawcolor.red, rawcolor.green, rawcolor.blue, hue, sat, m_currentcolor, m_previouscolor, m_RedCount, m_BlueCount, m_spinnersetpoint" << std::endl;
				m_stop = false;
				m_targetcolor = kNone;
			}
			m_spinner->Set(ControlMode::PercentOutput, 0);
		}
		else
		{
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint * m_direction); 
			std::cout << m_currentcolor << ", " << m_targetcolor << ", " << m_direction << std::endl;
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

	if (hue > 80 && hue < 100)
		m_color = kYellow;
	else if (hue > 20 && hue < 45)
		m_color = kRed;
	else
	if (hue < 150 && hue > 120)
		m_color = kGreen;
	else
	if (hue < 210 && hue > 180)
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
