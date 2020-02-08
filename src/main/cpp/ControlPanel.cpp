/**
 *  ControlPanel.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Jival Chandrashekar
 */


#include "ControlPanel.h"
#include "Const.h"
#include <frc/smartdashboard/SmartDashboard.h>


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

	m_spinnerstate = kOff;

	m_currentcolor = 0;
	m_previouscolor = 0;
	m_registeredcolor = 0;
	m_colorbouncecount = 0;
	m_colorregisteredcount[0] = 0;
	m_colorregisteredcount[1] = 0;
	m_colorregisteredcount[2] = 0;
	m_colorregisteredcount[3] = 0;
	m_stop = false;

	m_targetcolor = 0;

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

	m_spinnerstate = kBlindSpin;

	m_colormatcher.AddColorMatch(kYellowTarget);
	m_colormatcher.AddColorMatch(kRedTarget);
	m_colormatcher.AddColorMatch(kGreenTarget);
	m_colormatcher.AddColorMatch(kBlueTarget);
	m_confidence = 0.6;

	m_currentcolor = 0;
	m_previouscolor = 0;
	m_registeredcolor = 0;
	m_colorbouncecount = 0;
	m_colorregisteredcount[0] = 0;
	m_colorregisteredcount[1] = 0;
	m_colorregisteredcount[2] = 0;
	m_colorregisteredcount[3] = 0;
	m_stop = false;

	m_targetcolor = 0;

	SmartDashboard::PutNumber("CPIN1_SpinnerSetpoint", m_spinnersetpoint);
	SmartDashboard::PutNumber("CPIN2_Confidence", m_confidence);
	SmartDashboard::PutNumber("CPIN3_TargetColor", m_targetcolor);

	m_spinner->SetNeutralMode(NeutralMode::Coast);
	m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
	m_spinner->SetSensorPhase(true);
	m_spinner->SetInverted(true);
	m_spinner->SetNeutralMode(NeutralMode::Coast);
	m_spinnersetpoint = 0;
}


void ControlPanel::Loop()
{
	if (m_spinner == nullptr)
		return;

	ControlPanelStates();

	SmartDashboard::PutNumber("CPM11_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / ENCODER_TICKS_PER_REV);
	SmartDashboard::PutNumber("CPM12_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / ENCODER_TICKS_PER_REV);
	SmartDashboard::PutNumber("CPM13_ClosedLoopError", m_spinner->GetClosedLoopError(0));
	SmartDashboard::PutNumber("CPM14_ClosedLoopTarget", m_spinner->GetClosedLoopTarget(0));
	SmartDashboard::PutNumber("CPM15_Motor Voltage", m_spinner->GetMotorOutputVoltage());
}


void ControlPanel::Stop()
{
	if (m_spinner == nullptr)
		return;

	m_spinnerstate = kOff;
	m_spinnersetpoint = 0;
}

/*
int ControlPanel::GetColor()
{
	//double IR = m_colorsensor->GetIR();
	Color detectedColor = m_colorsensor->GetColor();
	//uint32_t proximity = m_colorsensor->GetProximity();
	double K = 1 - fmax(fmax(detectedColor.red, detectedColor.blue), detectedColor.green);
	double C = (1 - detectedColor.red - K) / (1 - K);
	double M = (1 - detectedColor.green - K) / (1 - K);
	double Y = (1 - detectedColor.blue - K) / (1 - K);
	
	int color = 0; // 1-4 Yellow, Green, Blue, Red

	if (C < .300)
	{
		color = 2;
		SmartDashboard::PutString("CPTEST_Color", "Red");
	}
	else if (Y < .300)
	{
		color = 4;
		SmartDashboard::PutString("CPTEST_Color", "Blue");
	}
	else if (M < .200  && C > Y )
	{
		color = 3;
		SmartDashboard::PutString("CPTEST_Color", "Green");
	}
	else if (M < .200  && Y > C )
	{
		color = 1;
		SmartDashboard::PutString("CPTEST_Color", "Yellow");
	}

	SmartDashboard::PutNumber("CP1_K", K);
	SmartDashboard::PutNumber("CP2_C", C);
	SmartDashboard::PutNumber("CP3_M", M);
	SmartDashboard::PutNumber("CP4_Y", Y);
	SmartDashboard::PutNumber("CP1000_R", detectedColor.red);
	SmartDashboard::PutNumber("CP2000_G", detectedColor.green);
	SmartDashboard::PutNumber("CP3000_B", detectedColor.blue);
	//SmartDashboard::PutNumber("CP5_IR", IR);
	//SmartDashboard::PutNumber("CP6_Proximity",proximity);
	SmartDashboard::PutNumber("CP7_Color", color);
	return color;
}
*/


void ControlPanel::ControlPanelStates()
{
	m_spinnersetpoint = SmartDashboard::GetNumber("CPIN1_SpinnerSetpoint", 0);
	m_confidence = SmartDashboard::GetNumber("CPIN2_Confidence", 0);
	m_targetcolor = SmartDashboard::GetNumber("CPIN3_TargetColor", 0);

	switch (m_spinnerstate)
	{
	case kOff:
		m_spinner->Set(ControlMode::PercentOutput, 0);
		break;
	case kBlindSpin:
		m_currentcolor = GetColor();

		// Start bounce counting if:
		// Color is picked up twice
		// Color is not the same color as the registered one already
		// 
		// Color must pass filter that checks for sensor inconsistencies
		if (m_currentcolor == m_previouscolor && m_registeredcolor != m_currentcolor && SensorSanityCheck())
		{
			m_colorbouncecount++;
		}
		else if (SensorSanityCheck())
		{
			m_previouscolor = m_currentcolor;
		}
		// Once picked up twice, set and log the registered color and reset color bounce count
		if (m_colorbouncecount >= 2)
		{
			if (m_currentcolor == 1)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Yellow");
				m_registeredcolor = 1;
				m_colorregisteredcount[0]++;
			}
			else if (m_currentcolor == 2)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Red");
				m_registeredcolor = 2;
				m_colorregisteredcount[1]++;
			}
			else if (m_currentcolor == 3)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Green");
				m_registeredcolor = 3;
				m_colorregisteredcount[2]++;
			}
			else if (m_currentcolor == 4)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Blue");
				m_registeredcolor = 4;
				m_colorregisteredcount[3]++;
			}
			m_previouscolor = m_currentcolor;
			m_colorbouncecount = 0;          
		}

		// If one color is registered enough times, stop spinning
		if (m_colorregisteredcount[3] >= 7)
			m_stop = true;
			
		// Once stopped, use A button to restart. If not, set the speed to spinpower
		if (m_stop)
		{
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				m_stop = false;
				m_colorregisteredcount[0] = 0;
				m_colorregisteredcount[1] = 0;
				m_colorregisteredcount[2] = 0;
				m_colorregisteredcount[3] = 0;
			}
			m_spinner->Set(ControlMode::PercentOutput, 0);
		}
		else
		{
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint); 
		}

		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
		{
			m_colorregisteredcount[0] = 0;
			m_colorregisteredcount[1] = 0;
			m_colorregisteredcount[2] = 0;
			m_colorregisteredcount[3] = 0;
		}
		break;
	
	case kColorSpin:
		if (m_targetcolor == 0)
			break;
		
		m_currentcolor = GetColor();

		// Start bounce counting if:
		// Color is picked up twice
		// Color is not the same color as the registered one already
		// 
		// Color must pass filter that checks for sensor inconsistencies
		if (m_currentcolor == m_previouscolor && m_registeredcolor != m_currentcolor && SensorSanityCheck())
		{
			m_colorbouncecount++;
		}
		else if (SensorSanityCheck())
		{
			m_previouscolor = m_currentcolor;
		}

		// Once picked up twice, set and log the registered color and reset color bounce count
		if (m_colorbouncecount >= 2)
		{
			if (m_currentcolor == 1)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Yellow");
				m_registeredcolor = 1;
				m_colorregisteredcount[0]++;
			}
			else if (m_currentcolor == 2)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Red");
				m_registeredcolor = 2;
				m_colorregisteredcount[1]++;
			}
			else if (m_currentcolor == 3)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Green");
				m_registeredcolor = 3;
				m_colorregisteredcount[2]++;
			}
			else if (m_currentcolor == 4)
			{
				SmartDashboard::PutString("CPVER1_TrueColor", "Blue");
				m_registeredcolor = 4;
				m_colorregisteredcount[3]++;
			}
			m_previouscolor = m_currentcolor;
			m_colorbouncecount = 0;          
		}

		// Once stopped, use A button to restart. If not, set the speed to spinpower
		if (m_stop)
		{
			if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				m_stop = false;
				m_colorregisteredcount[0] = 0;
				m_colorregisteredcount[1] = 0;
				m_colorregisteredcount[2] = 0;
				m_colorregisteredcount[3] = 0;
			}
			m_spinner->Set(ControlMode::PercentOutput, 0);
		}
		else
		{
			m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint); 
	
			// Check for current color compared to target color, slow down over time
			double onebeforetarget = m_targetcolor - 1;
			if (onebeforetarget == 0)
				onebeforetarget = 4;
			
			double twobeforetarget = m_targetcolor - 2;
			if (twobeforetarget == 0)
				twobeforetarget = 4;
			else
			if (twobeforetarget == -1)
				twobeforetarget = 3;
			
			if (m_registeredcolor == twobeforetarget)
				m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint * 2 / 3); 
			else
			if (m_registeredcolor == onebeforetarget)
				m_spinner->Set(ControlMode::PercentOutput, m_spinnersetpoint * 1 / 3);
			else
			if (m_registeredcolor == m_targetcolor)
			{
				m_stop = true;
				m_spinner->Set(ControlMode::PercentOutput, 0);
			}
		}

		// Reset capability for debug
		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
		{
			m_colorregisteredcount[0] = 0;
			m_colorregisteredcount[1] = 0;
			m_colorregisteredcount[2] = 0;
			m_colorregisteredcount[3] = 0;
		}
		break;
	}

	// Displays the total registered color count of each color
	SmartDashboard::PutNumber("CPVER2_Registered Yellow Count", m_colorregisteredcount[0]);
	SmartDashboard::PutNumber("CPVER3_Registered Red Count", m_colorregisteredcount[1]);
	SmartDashboard::PutNumber("CPVER4_Registered Green Count", m_colorregisteredcount[2]);
	SmartDashboard::PutNumber("CPVER5_Registered Blue Count", m_colorregisteredcount[3]);
}


int ControlPanel::GetColor()
{
	Color detectedcolor = m_colorsensor->GetColor();
	Color matchedcolor = m_colormatcher.MatchClosestColor(detectedcolor, m_confidence);

	int color = 0;

	if (matchedcolor == kYellowTarget)
		color = 1;
	else 
	if (matchedcolor == kRedTarget)
		color = 2;
	else
	if (matchedcolor == kGreenTarget)
		color = 3;
	else
	if (matchedcolor == kBlueTarget)
		color = 4;

	SmartDashboard::PutNumber("CPSIMP1_R", detectedcolor.red);
	SmartDashboard::PutNumber("CPSIMP2_G", detectedcolor.green);
	SmartDashboard::PutNumber("CPSIMP3_B", detectedcolor.blue);
	SmartDashboard::PutNumber("CPSIMP4_Color", color);

	return color;
}


bool ControlPanel::SensorSanityCheck()
// Checks if it has gathered enough data
// Once it has, check for color mis-followings with the color sensor
//      For example, if a yellow shows up after a green even though the pattern is YRGB
{
	// if sensor has not gotten enough data yet, cancel out of sanity check
	for (int i = 0; i < 4; i++)
	{
		if (m_colorregisteredcount[i] < 1)
			return true;
	}

	if (m_currentcolor == 0 || m_previouscolor == 0)
		return true;

	// color can never be two states ahead/below itself
	double impossiblecolor1 = m_previouscolor - 1;

	// rescaling back to 1 - 4 bounds
	if (impossiblecolor1 == 0)
		impossiblecolor1 = 4;

	double impossiblecolor2 = m_previouscolor + 2;

	if (impossiblecolor2 == 5)
		impossiblecolor2 = 1;
	else
	if (impossiblecolor2 == 6)
		impossiblecolor2 = 2;
	
	
	if (m_currentcolor == impossiblecolor1 || m_currentcolor == impossiblecolor2)
		return false;
	else
		return true;
}