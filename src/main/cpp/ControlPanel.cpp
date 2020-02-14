/**
 *  ControlPanel.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Jival Chandrashekar
 */


#include "ControlPanel.h"
#include "Const.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


using namespace std;


static constexpr Color kYellowTarget = Color(.323364, .588501, .088013);
static constexpr Color kRedTarget = Color(0.580933, 0.310425, 0.108521);    
static constexpr Color kGreenTarget = Color(0.118286, 0.574829, .302368);
static constexpr Color kBlueTarget = Color(0.084595, 0.361450, 0.553833);


ControlPanel::ControlPanel(OperatorInputs *inputs, CDSensors *sensors)
{
	if (CPL_MOTOR == -1)
	{
		DriverStation::ReportError("Control Panel Not Enabled");
		return;
	}

	m_inputs = inputs;
	m_sensors = sensors;

	m_spinner = new TalonSRX(CPL_MOTOR);
	m_solenoid = new Solenoid(CPL_SOLENOID);
		
	m_colorsensor = m_sensors->GetColorSensor();

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

	m_solenoid->Set(false);

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

	m_spinner->SetNeutralMode(NeutralMode::Brake);
	m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TIMEOUT_MS);
	m_spinner->SetSensorPhase(true);
	m_spinner->SetInverted(true);
	m_spinnersetpoint = 0;

	/* set the peak and nominal outputs */
	m_spinner->ConfigNominalOutputForward(CPL_MIN);
	m_spinner->ConfigNominalOutputReverse(0);
	m_spinner->ConfigPeakOutputForward(CPL_MAX);
	m_spinner->ConfigPeakOutputReverse(0);
	/* set closed loop gains in slot0 */
	m_spinner->Config_kP(0, m_spinnerPIDvals[0], TIMEOUT_MS);
	m_spinner->Config_kI(0, m_spinnerPIDvals[1], TIMEOUT_MS);
	m_spinner->Config_kD(0, m_spinnerPIDvals[2], TIMEOUT_MS);
	m_spinner->Config_kF(0, 0);

	SmartDashboard::PutNumber("CPIN1_SpinnerSetpoint", m_spinnersetpoint);
	SmartDashboard::PutNumber("CPIN2_Confidence", m_confidence);
	SmartDashboard::PutNumber("CPIN3_TargetColor", m_targetcolor);
	// Testing purposes for PID
	SmartDashboard::PutNumber("CPM4_P Gain",        m_spinnerPIDvals[0]);
	SmartDashboard::PutNumber("CPM5_I Gain",        m_spinnerPIDvals[1]);
	SmartDashboard::PutNumber("CPM6_D Gain",        m_spinnerPIDvals[2]);

}


void ControlPanel::Loop()
{
	if (m_spinner == nullptr)
		return;

	if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		m_solenoid->Set(!m_solenoid->Get());

	// read PID coefficients from SmartDashboard
	double p = SmartDashboard::GetNumber("CPM4_P Gain", 0);
	double i = SmartDashboard::GetNumber("CPM5_I Gain", 0);
	double d = SmartDashboard::GetNumber("CPM6_D Gain", 0);

	// if PID coefficients on SmartDashboard have changed, write new values to controller
	if ((p != m_spinnerPIDvals[0])) { m_spinner->Config_kP(0, p, TIMEOUT_MS); m_spinnerPIDvals[0] = p; }
	if ((i != m_spinnerPIDvals[1])) { m_spinner->Config_kI(0, i, TIMEOUT_MS); m_spinnerPIDvals[1] = i; }
	if ((d != m_spinnerPIDvals[2])) { m_spinner->Config_kD(0, d, TIMEOUT_MS); m_spinnerPIDvals[2] = d; }

	//ControlPanelStates();
	if (m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 0 * INP_DUAL))
		m_spinner->Set(ControlMode::PercentOutput, 0.25);
	else
		m_spinner->Set(ControlMode::PercentOutput, 0);
		
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


void ControlPanel::ControlPanelStates()
{
	m_spinnersetpoint = SmartDashboard::GetNumber("CPIN1_SpinnerSetpoint", 0);
	m_confidence = SmartDashboard::GetNumber("CPIN2_Confidence", 0);
	m_targetcolor = SmartDashboard::GetNumber("CPIN3_TargetColor", 0);

	switch (m_spinnerstate)
	{
	case kOff:
		m_spinner->Set(ControlMode::Velocity, 0);
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
			if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
			{
				m_stop = false;
				m_colorregisteredcount[0] = 0;
				m_colorregisteredcount[1] = 0;
				m_colorregisteredcount[2] = 0;
				m_colorregisteredcount[3] = 0;
			}
			m_spinner->Set(ControlMode::Velocity, 0);
		}
		else
		{
			double targetVelocity_UnitsPer100ms = m_spinnersetpoint * ENCODER_TICKS_PER_REV * MINUTES_TO_HUNDRED_MS;
			/* 500 RPM in either direction */
			m_spinner->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 
		}

		if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
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

		double targetVelocity_UnitsPer100ms = m_spinnersetpoint * ENCODER_TICKS_PER_REV * MINUTES_TO_HUNDRED_MS;

		// Once stopped, use A button to restart. If not, set the speed to spinpower
		if (m_stop)
		{
			if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kToggle, 0))
			{
				m_stop = false;
				m_colorregisteredcount[0] = 0;
				m_colorregisteredcount[1] = 0;
				m_colorregisteredcount[2] = 0;
				m_colorregisteredcount[3] = 0;
			}
			m_spinner->Set(ControlMode::Velocity, 0);
		}
		else
		{
			m_spinner->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 
	
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
				m_spinner->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms * 2 / 3); 
			else
			if (m_registeredcolor == onebeforetarget)
				m_spinner->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms * 1 / 3);
			else
			if (m_registeredcolor == m_targetcolor)
			{
				m_stop = true;
				m_spinner->Set(ControlMode::Velocity, 0);
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