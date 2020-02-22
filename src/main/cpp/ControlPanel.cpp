/**
 *  ControlPanel.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Jival Chandrashekar
 *  Nicholas Seidl
 */


#include "ControlPanel.h"
#include "Const.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "math.h"



using namespace std;


ControlPanel::ControlPanel(OperatorInputs *inputs, GyroDrive *gyrodrive)
#ifdef USE_LOGGER
	: m_log("home/cplogfile.csv", true)
#endif
{
	m_inputs = inputs;
	m_gyrodrive = gyrodrive;
	
	m_spinner = nullptr;

	if (CPL_MOTOR != -1)
		m_spinner = new TalonSRX(CPL_MOTOR);
	
	if (CPL_SOLENOID != -1)
		m_solenoid = new Solenoid(CPL_SOLENOID);
	
	m_color[0] = "no Color";
	m_color[1] = "yellow"; 
	m_color[2] = "red"   ;
	m_color[3] = "green" ;
	m_color[4] = "blue"  ;

#ifdef USE_LOGGER
	m_log.logMsg(eInfo, __FUNCTION__, __LINE__, "currentcolor,previouscolor,targetcolor,redCount,blueCount,direction,currentencodervalue,startencodervalue,spinnersetpoint");
	m_dataInt.push_back((int*)&m_currentcolor);
	m_dataInt.push_back((int*)&m_previouscolor);
	m_dataInt.push_back(&m_redCount);
	m_dataInt.push_back(&m_blueCount);
	m_dataInt.push_back(&m_currentencodervalue);
	m_dataDouble.push_back(&m_spinnersetpoint);
			std::cout 	<< m_currentcolor << ", " 
						<< m_previouscolor << ", " 
						<< m_targetcolor << ", " 
						<< m_redCount << ", " 
						<< m_blueCount << ", " 
						<< m_direction << ", " 
						<< m_currentencodervalue << ", " 
						<< m_startencodervalue << "," 
						<< CPL_COUNTS_PER_CW_SECTOR / 2 - encoderdelta 
						<< m_spinnersetpoint << ", " 
						<< endl;
#endif

	// Integrate color sensor TAG
	m_colorsensor = new ColorSensorV3(I2C::Port::kOnboard);
	m_timer = new Timer();

	m_stop = false;

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
	if (m_spinner == nullptr || m_colorsensor == nullptr || m_solenoid == nullptr)
		return;

	m_colorsensor->ConfigureColorSensor(rev::ColorSensorV3::ColorResolution::k13bit, rev::ColorSensorV3::ColorMeasurementRate::k25ms);
	m_solenoid->Set(false);

	m_redCount = 0;
	m_blueCount = 0;
	m_currentcolor = kNone;
	m_previouscolor = kNone;
	m_targetcolor = kNone;
	m_direction = 0;
	m_spinnerstate = kOff;
	m_stop = true;
	m_startencodervalue = 0;

	m_spinner->SetNeutralMode(NeutralMode::Brake);
	m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
	m_spinner->SetSensorPhase(true); // What does this do / is this needed?
	m_spinner->SetInverted(true); // What does this do / is this needed?
	m_spinner->ConfigPeakOutputForward(1);
	m_spinner->ConfigPeakOutputReverse(-1);
	
	m_spinner->Config_kF(0, CPL_F);
	m_spinner->Config_kP(0, CPL_P);
	m_spinner->Config_kI(0, CPL_I);
}


void ControlPanel::Loop()
{
	if (m_spinner == nullptr || m_colorsensor == nullptr || m_solenoid == nullptr)
		return;

	ChangeSpinnerState();

	ControlPanelStates();

	Dashboard();
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
		m_spinner->Set(ControlMode::Velocity, 0);
		m_stop = true;
		break;

	case kRotationControl:
		m_currentcolor = GetColor();	
		
			if (m_currentcolor == kRed && m_previouscolor != kRed)
			{
				m_redCount++;
			}
			else if (m_currentcolor == kBlue &&  m_previouscolor != kBlue )
			{
				m_blueCount++;
			}	

		if (m_blueCount == CPL_ROTATION_CONTROL_COUNT_LIMIT_BLUE || m_redCount == CPL_ROTATION_CONTROL_COUNT_LIMIT_RED )
			m_stop = true;
		
		if (m_stop)
		{
			// Once stopped, use Xbox Back button to restart. (TO DO: Move reading user inputs to a higher level robot class)
			
			m_spinner->Set(ControlMode::Velocity, 0);
		}
		else
		{
			//Command motor to spin at a given speed
			m_spinnersetpoint = CPL_ROTATION_CONTROL_FAST;
			m_spinner->Set(ControlMode::Velocity, m_spinnersetpoint); 
			m_currentencodervalue = m_spinner->GetSelectedSensorPosition(0);
#ifdef USE_LOGGER
			m_log.logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
#else
			std::cout 	<< m_currentcolor << ", " 
						<< m_previouscolor << ", " 
						<< m_targetcolor << ", " 
						<< m_redCount << ", " 
						<< m_blueCount << ", " 
						<< m_direction << ", " 
						<< m_currentencodervalue << ", " 
						<< m_startencodervalue << "," 
						<< CPL_COUNTS_PER_CW_SECTOR / 2 - encoderdelta 
						<< m_spinnersetpoint << ", " 
						<< endl;
#endif
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
			// If we don't know what the target color is, read the target color from FMS and 
			m_targetcolor = GetTargetColor();
			if(m_targetcolor != kNone)
				{
				// Determine the shortest path to spin the wheel
				int colordelta = m_targetcolor - m_currentcolor;
				if (colordelta == 1 || colordelta == -3)
					m_direction = -1;	
				else
					m_direction = 1;
				// Set motor speed	
				m_spinnersetpoint = CPL_POSITION_CONTROL_FAST;
#ifdef USE_LOGGER
			m_log.logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
#else
			std::cout << m_currentcolor << ", " << m_previouscolor << ", " << m_redCount << ", " << m_blueCount << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << std::endl;
#endif
				cout<< "Target Color =" << m_targetcolor <<endl;

				}
			}
		
		if(m_currentcolor == m_targetcolor)
			{
			m_spinnersetpoint = CPL_POSITION_CONTROL_SLOW;
			if (m_startencodervalue == 0)
				{
#ifdef USE_LOGGER
			m_log.logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
#else
			std::cout << m_currentcolor << ", " << m_previouscolor << ", " << m_redCount << ", " << m_blueCount << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << std::endl;
#endif
				cout << "found target color!" << endl;
				// Record the current encoder value
				m_startencodervalue = m_spinner->GetSelectedSensorPosition(0);
				}
			else if (abs(m_currentencodervalue - m_startencodervalue) >= CPL_COUNTS_PER_CW_SECTOR/2 )
				{
#ifdef USE_LOGGER
			m_log.logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
#else
			std::cout << m_currentcolor << ", " << m_previouscolor << ", " << m_redCount << ", " << m_blueCount << ", " << m_spinnersetpoint << ", " << m_currentencodervalue << std::endl;
#endif
				cout << "done!" << endl;
				// if the wheel has spun half a sector, then stop the motor
				m_stop = true;
				}
			}
		else if(m_currentcolor != kNone) //Ensures a valid color reading
			{
			// If not over the target color, spin fast and clear any encoder value that we recorded
			m_startencodervalue = 0;
			m_spinnersetpoint = CPL_POSITION_CONTROL_FAST;
			}

		
		if (m_stop)
		{
			// Press Xbox Start Button to Restart
			m_spinner->Set(ControlMode::Velocity, 0);
			// (TO DO: Move reading user inputs to a higher level robot class)
		}
		else
		{
			//If not stopped command motor according to setpoint and direction
			m_spinner->Set(ControlMode::Velocity, m_spinnersetpoint * m_direction); 
			int encoderdelta = abs(m_currentencodervalue - m_startencodervalue);
#ifdef USE_LOGGER
			m_log.logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
#else
			std::cout 	<< m_currentcolor << ", " 
						<< m_previouscolor << ", " 
						<< m_targetcolor << ", " 
						<< m_redCount << ", " 
						<< m_blueCount << ", " 
						<< m_direction << ", " 
						<< m_currentencodervalue << ", " 
						<< m_startencodervalue << "," 
						<< CPL_COUNTS_PER_CW_SECTOR / 2 - encoderdelta 
						<< m_spinnersetpoint << ", " 
						<< endl;
#endif
		}
		
		break;
	}
}


ControlPanel::ColorOptions ControlPanel::GetColor()
{
	ColorOptions color;
	
	ColorSensorV3::RawColor rawcolor = ColorSensorV3::RawColor(0, 0, 0, 0);

	int retries = 3;
	while (retries > 0 && rawcolor.red == 0 && rawcolor.green == 0 && rawcolor.blue == 0)
	{
		rawcolor = m_colorsensor->GetRawColor();
		retries--;
	}
	
	double r = static_cast<double>(rawcolor.red);
    double g = static_cast<double>(rawcolor.green);
    double b = static_cast<double>(rawcolor.blue);

	double hue = fmod(((180/3.14 * atan2(sqrt(3) / 2 * (g - b), r - .5*g - .5*b)) + 360), 360);
	// Unused Variable double M = fmax(fmax(r, g),b);
	// Unused Variable double sat = (M - fmin(fmin(r, g),b)) / M;
	// Unused Variable double val = (M/pow(2, 13));

	color = kNone;

	// TO DO: use val and sat to help catch anomolous raw readings
	if (hue > CPL_YELLOW_MINIMUM_HUE && hue < CPL_YELLOW_MAXIMUM_HUE)
		color = kYellow;
	else if (hue > CPL_RED_MINIMUM_HUE && hue < CPL_RED_MAXIMUM_HUE)
		color = kRed;
	else if (hue > CPL_GREEN_MINIMUM_HUE && hue < CPL_GREEN_MAXIMUM_HUE)
		color = kGreen;
	else if (hue > CPL_BLUE_MINIMUM_HUE && hue < CPL_BLUE_MAXIMUM_HUE)
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
	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		m_solenoid->Set(true);
		m_gyrodrive->SetLowSpeed(true);
	}
	else
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		m_solenoid->Set(false);
		m_gyrodrive->SetLowSpeed(false);
	}

	if (!m_solenoid->Get())
		return;

	//TODO: Move reading of operator inputs to higher level class
	if (m_inputs->xBoxBackButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		m_spinnerstate = kRotationControl;
		m_stop = false;
		m_redCount = 0;
		m_blueCount = 0;
	}
	if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		m_spinnerstate = kPositionControl;
		m_stop = false;
		m_targetcolor = kNone;
		m_startencodervalue = 0;
	}
}

void ControlPanel::ChangeSpinnerState(SpinnerState state)
{
	m_spinnerstate = state;
}

void ControlPanel::Dashboard()
{
	SmartDashboard::PutNumber("CPL1_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / CPL_COUNTS_PER_CW_REV);
	SmartDashboard::PutNumber("CPL2_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / CPL_COUNTS_PER_CW_REV);
	SmartDashboard::PutNumber("CPL3_Motor Voltage", m_spinner->GetMotorOutputVoltage());
	SmartDashboard::PutNumber("CPL4_Registered Red Count", m_redCount);
	SmartDashboard::PutNumber("CPL5_Registered Blue Count", m_blueCount);

	SmartDashboard::PutString("CPL6_CurrentColor", m_color[m_currentcolor]);

}