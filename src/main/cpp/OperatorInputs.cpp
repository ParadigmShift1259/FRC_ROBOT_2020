/**
 *  OperatorInputs.cpp
 *  Date:
 *  Last Edited By:
 */


#include "OperatorInputs.h"
#include "Const.h"
#include <cmath>


#define TOGGLE_XBOXABUTTON 0
#define TOGGLE_XBOXBBUTTON 1
#define TOGGLE_XBOXXBUTTON 2
#define TOGGLE_XBOXYBUTTON 3
#define TOGGLE_XBOXLEFTBUMPER 4
#define TOGGLE_XBOXRIGHTBUMPER 5
#define TOGGLE_XBOXLEFTTRIGGER 6
#define TOGGLE_XBOXRIGHTTRIGGER 7
#define TOGGLE_XBOXSTARTBUTTON 8
#define TOGGLE_XBOXBACKBUTTON 9
#define TOGGLE_XBOXDPADUP 10
#define TOGGLE_XBOXDPADUPRIGHT 11
#define TOGGLE_XBOXDPADRIGHT 12
#define TOGGLE_XBOXDPADDOWNRIGHT 13
#define TOGGLE_XBOXDPADDOWN 14
#define TOGGLE_XBOXDPADDOWNLEFT 15
#define TOGGLE_XBOXDPADLEFT 16
#define TOGGLE_XBOXDPADUPLEFT 17
#define TOGGLE_XBOXL3 18
#define TOGGLE_XBOXR3 19


OperatorInputs::OperatorInputs()
{
	if (INP_XBOX_1 != -1)
		m_xbox.push_back(new XboxController(INP_XBOX_1));
	if (INP_XBOX_2 != -1)
		m_xbox.push_back(new XboxController(INP_XBOX_2));
	// initialize toggle variable to zero
	memset(m_toggle, 0, sizeof(m_toggle));
	g_log->logMsg(eInfo, __FUNCTION__, __LINE__, "Xbox#,A,B,X,Y,Lbumper,Rbumper,Ltrig,Rtrig,Start,Back,"
												"DpadUp,DpadUpRt,DpadRt,DpadDnRt,DpadDn,DpadDnLf,DpadLf,DpadLf"
												"L3,R3,LfStickX,LfStickY,RtStickX,RtStickY");
	for (int i = 0; i < TOGGLE_MAX_CONTROLLERS; i++)
	{
		m_logData[i].m_xBoxController = i;

		m_dataInt[i].push_back(&m_logData[i].m_xBoxController);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxAButton);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxBButton);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxXButton);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxYButton);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxLeftBumper);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxRightBumper);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxLeftTrigger);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxRightTrigger);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxStartButton);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxBackButton);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadUp);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadUpRight);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadRight);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadDownRight);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadDown);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadDownLeft);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadLeft);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxDPadUpLeft);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxL3);
		m_dataInt[i].push_back(&m_logData[i].m_xBoxR3);

		m_dataDouble[i].push_back(&m_logData[i].m_xBoxLeftX);
		m_dataDouble[i].push_back(&m_logData[i].m_xBoxLeftY);
		m_dataDouble[i].push_back(&m_logData[i].m_xBoxRightX);
		m_dataDouble[i].push_back(&m_logData[i].m_xBoxRightY);
	}
}


OperatorInputs::~OperatorInputs()
{
	for (std::vector<XboxController * >::iterator it = m_xbox.begin() ; it != m_xbox.end(); it++)
		 delete (*it);
	m_xbox.clear();
}

void OperatorInputs::Loop()
{
	g_log->logData(__FUNCTION__, __LINE__, m_dataInt[0], m_dataDouble[0]);
	g_log->logData(__FUNCTION__, __LINE__, m_dataInt[1], m_dataDouble[1]);
}


double OperatorInputs::xBoxLeftX(unsigned int i)
{
	m_logData[i].m_xBoxLeftX = 0.0;
	if (i < m_xbox.size())
		m_logData[i].m_xBoxLeftX = deadzoneFilterX(INVERT_X_AXIS * m_xbox[i]->GetX(GenericHID::JoystickHand::kLeftHand));

	return m_logData[i].m_xBoxLeftX;
}


double OperatorInputs::xBoxRightX(unsigned int i)
{
	m_logData[i].m_xBoxRightX = 0.0;
	if (i < m_xbox.size())
		m_logData[i].m_xBoxRightX = deadzoneFilterX(m_xbox[i]->GetX(GenericHID::JoystickHand::kRightHand));

	return m_logData[i].m_xBoxRightX;
}


double OperatorInputs::xBoxLeftY(unsigned int i)
{
	m_logData[i].m_xBoxLeftY = 0.0;
	if (i < m_xbox.size())
		m_logData[i].m_xBoxLeftY = deadzoneFilterY(INVERT_Y_AXIS * m_xbox[i]->GetY(GenericHID::JoystickHand::kLeftHand));
	return m_logData[i].m_xBoxLeftY;
}


double OperatorInputs::xBoxRightY(unsigned int i)
{
	m_logData[i].m_xBoxRightY = 0.0;
	if (i < m_xbox.size())
		m_logData[i].m_xBoxRightY = deadzoneFilterY(m_xbox[i]->GetY(GenericHID::JoystickHand::kRightHand));
	return m_logData[i].m_xBoxRightY;
}


bool OperatorInputs::xBoxAButton(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxAButton = false;
	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxAButton = m_xbox[i]->GetRawButton(A_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxAButton = toggle(i, TOGGLE_XBOXABUTTON, m_logData[i].m_xBoxAButton);
	}
	return m_logData[i].m_xBoxAButton;
}


bool OperatorInputs::xBoxBButton(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxBButton = false;
	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxBButton = m_xbox[i]->GetRawButton(B_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxBButton = toggle(i, TOGGLE_XBOXBBUTTON, m_logData[i].m_xBoxBButton);
	}
	return m_logData[i].m_xBoxBButton;
}


bool OperatorInputs::xBoxXButton(ToggleChoice choice, unsigned int i)
{
		m_logData[i].m_xBoxXButton = false;
	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxXButton = m_xbox[i]->GetRawButton(X_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxXButton = toggle(i, TOGGLE_XBOXXBUTTON, m_logData[i].m_xBoxXButton);
	}
	return m_logData[i].m_xBoxXButton;
}


bool OperatorInputs::xBoxYButton(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxYButton = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxYButton = m_xbox[i]->GetRawButton(Y_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxYButton = toggle(i, TOGGLE_XBOXYBUTTON, m_logData[i].m_xBoxYButton);
	}
	return m_logData[i].m_xBoxYButton;
}


bool OperatorInputs::xBoxLeftBumper(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxLeftBumper = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxLeftBumper = m_xbox[i]->GetRawButton(LEFT_BUMPER);

		if (choice == kToggle)
			m_logData[i].m_xBoxLeftBumper = toggle(i, TOGGLE_XBOXLEFTBUMPER, m_logData[i].m_xBoxLeftBumper);
	}
	return m_logData[i].m_xBoxLeftBumper;
}


bool OperatorInputs::xBoxRightBumper(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxRightBumper = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxRightBumper = m_xbox[i]->GetRawButton(RIGHT_BUMPER);

		if (choice == kToggle)
			m_logData[i].m_xBoxRightBumper = toggle(i, TOGGLE_XBOXRIGHTBUMPER, m_logData[i].m_xBoxRightBumper);
	}
	return m_logData[i].m_xBoxRightBumper;
}


bool OperatorInputs::xBoxLeftTrigger(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxLeftTrigger = false;

	if (i < m_xbox.size())
	{
		double axis = m_xbox[i]->GetRawAxis(XBOX_LEFT_TRIGGER_AXIS - 10);
		m_logData[i].m_xBoxLeftTrigger = (LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX);

		if (choice == kToggle)
			m_logData[i].m_xBoxLeftTrigger = toggle(i, TOGGLE_XBOXLEFTTRIGGER, m_logData[i].m_xBoxLeftTrigger);
	}
	return m_logData[i].m_xBoxLeftTrigger;
}


bool OperatorInputs::xBoxRightTrigger(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxRightTrigger = false;

	if (i < m_xbox.size())
	{
		double axis = m_xbox[i]->GetRawAxis(XBOX_RIGHT_TRIGGER_AXIS - 10);
		m_logData[i].m_xBoxRightTrigger = (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX);

		if (choice == kToggle)
			m_logData[i].m_xBoxRightTrigger = toggle(i, TOGGLE_XBOXRIGHTTRIGGER, m_logData[i].m_xBoxRightTrigger);
	}
	return m_logData[i].m_xBoxRightTrigger;
}


bool OperatorInputs::xBoxStartButton(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxStartButton = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxStartButton = m_xbox[i]->GetRawButton(START_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxStartButton = toggle(i, TOGGLE_XBOXSTARTBUTTON, m_logData[i].m_xBoxStartButton);
	}
	return m_logData[i].m_xBoxStartButton;
}


bool OperatorInputs::xBoxBackButton(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxBackButton = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxBackButton = m_xbox[i]->GetRawButton(BACK_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxBackButton = toggle(i, TOGGLE_XBOXBACKBUTTON, m_logData[i].m_xBoxBackButton);
	}
	return m_logData[i].m_xBoxBackButton;
}


bool OperatorInputs::xBoxDPadUp(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadUp = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadUp = (m_xbox[i]->GetPOV() == 0);

		if (choice == kToggle)
			m_logData[i].m_xBoxDPadUp = toggle(i, TOGGLE_XBOXDPADUP, m_logData[i].m_xBoxDPadUp);
	}
	return m_logData[i].m_xBoxDPadUp;
}


bool OperatorInputs::xBoxDPadUpRight(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadUpRight = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadUpRight = (m_xbox[i]->GetPOV() == 45);

		if (choice == kToggle)
			m_logData[i].m_xBoxDPadUpRight = toggle(i, TOGGLE_XBOXDPADUPRIGHT, m_logData[i].m_xBoxDPadUpRight);
	}
	return m_logData[i].m_xBoxDPadUpRight;
}


bool OperatorInputs::xBoxDPadRight(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadRight = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadRight = (m_xbox[i]->GetPOV() == 90);

		if (choice == kToggle)
			m_logData[i].m_xBoxDPadRight = toggle(i, TOGGLE_XBOXDPADRIGHT, m_logData[i].m_xBoxDPadRight);
	}
	return m_logData[i].m_xBoxDPadRight;
}


bool OperatorInputs::xBoxDPadDownRight(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadDownRight = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadDownRight = (m_xbox[i]->GetPOV() == 135);

	if (choice == kToggle)
			m_logData[i].m_xBoxDPadDownRight = toggle(i, TOGGLE_XBOXDPADDOWNRIGHT, m_logData[i].m_xBoxDPadDownRight);
	}
	return m_logData[i].m_xBoxDPadDownRight;
}


bool OperatorInputs::xBoxDPadDown(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadDown = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadDown = (m_xbox[i]->GetPOV() == 180);

	if (choice == kToggle)
			m_logData[i].m_xBoxDPadDown = toggle(i, TOGGLE_XBOXDPADDOWN, m_logData[i].m_xBoxDPadDown);
	}
	return m_logData[i].m_xBoxDPadDown;
}


bool OperatorInputs::xBoxDPadDownLeft(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadDownLeft = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadDownLeft = (m_xbox[i]->GetPOV() == 225);

		if (choice == kToggle)
			m_logData[i].m_xBoxDPadDownLeft = toggle(i, TOGGLE_XBOXDPADDOWNLEFT, m_logData[i].m_xBoxDPadDownLeft);
	}
	return m_logData[i].m_xBoxDPadDownLeft;
}


bool OperatorInputs::xBoxDPadLeft(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadLeft = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadLeft = (m_xbox[i]->GetPOV() == 270);

		if (choice == kToggle)
			m_logData[i].m_xBoxDPadLeft = toggle(i, TOGGLE_XBOXDPADLEFT, m_logData[i].m_xBoxDPadLeft);
	}
	return m_logData[i].m_xBoxDPadLeft;
}


bool OperatorInputs::xBoxDPadUpLeft(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxDPadUpLeft = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxDPadUpLeft = (m_xbox[i]->GetPOV() == 315);

		if (choice == kToggle)
			m_logData[i].m_xBoxDPadUpLeft = toggle(i, TOGGLE_XBOXDPADUPLEFT, m_logData[i].m_xBoxDPadUpLeft);
	}
	return m_logData[i].m_xBoxDPadUpLeft;
}


bool OperatorInputs::xBoxL3(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxL3 = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxL3 = m_xbox[i]->GetRawButton(L3_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxL3 = toggle(i, TOGGLE_XBOXL3, m_logData[i].m_xBoxL3);
	}
	return m_logData[i].m_xBoxL3;
}


bool OperatorInputs::xBoxR3(ToggleChoice choice, unsigned int i)
{
	m_logData[i].m_xBoxR3 = false;

	if (i < m_xbox.size())
	{
		m_logData[i].m_xBoxR3 = m_xbox[i]->GetRawButton(R3_BUTTON);

		if (choice == kToggle)
			m_logData[i].m_xBoxR3 = toggle(i, TOGGLE_XBOXR3, m_logData[i].m_xBoxR3);
	}
	return m_logData[i].m_xBoxR3;
}


bool OperatorInputs::xBox(int Button, ToggleChoice choice, unsigned int i)
{
	switch (Button)
	{
	case A_BUTTON:
		return xBoxAButton(choice, i);
	case B_BUTTON:
		return xBoxBButton(choice, i);
	case X_BUTTON:
		return xBoxXButton(choice, i);
	case Y_BUTTON:
		return xBoxYButton(choice, i);
	case LEFT_BUMPER:
		return xBoxLeftBumper(choice, i);
	case RIGHT_BUMPER:
		return xBoxRightBumper(choice, i);
	case XBOX_LEFT_TRIGGER_AXIS:
		return xBoxLeftTrigger(choice, i);
	case XBOX_RIGHT_TRIGGER_AXIS:
		return xBoxRightTrigger(choice, i);
	case START_BUTTON:
		return xBoxStartButton(choice, i);
	case BACK_BUTTON:
		return xBoxBackButton(choice, i);
	case L3_BUTTON:
		return xBoxL3(choice, i);
	case R3_BUTTON:
		return xBoxR3(choice, i);
	}
	return false;	
}



/// takes a given controller and button combination, returns true on the rising edge of value
bool OperatorInputs::toggle(unsigned int controller, int button, bool value)
{
	if ((controller < TOGGLE_MAX_CONTROLLERS) && (button < TOGGLE_MAX_BUTTONS))
	{
		bool toggleval = !m_toggle[controller][button] && value;
		m_toggle[controller][button] = value;
		return toggleval;
	}
	return false;
}


double OperatorInputs::deadzoneFilter(double joyStickValue, double axisDeadZone)
{
	if (abs(joyStickValue) <= axisDeadZone)
	{
		// If the input is small, return 0
		return 0.0;
	}

	// Normalize the output value
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * axisDeadZone)) / (1.0 - axisDeadZone);
}
