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
#define TOGGLE_JOYSTICKAXIS0LEFT 20
#define TOGGLE_JOYSTICKAXIS0RIGHT 21
#define TOGGLE_JOYSTICKAXIS1BACK 22
#define TOGGLE_JOYSTICKAXIS1FORWARD 23
#define TOGGLE_JOYSTICKTRIGGER 24
#define TOGGLE_JOYSTICKBUTTON2 25
#define TOGGLE_JOYSTICKBUTTON3 26
#define TOGGLE_JOYSTICKBUTTON5 27
#define TOGGLE_JOYSTICKBUTTON6 28
#define TOGGLE_JOYSTICKBUTTON7 29
#define TOGGLE_JOYSTICKBUTTON8 30
#define TOGGLE_JOYSTICKBUTTON9 31
#define TOGGLE_JOYSTICKBUTTON10 32


OperatorInputs::OperatorInputs()
{
    m_joystick = nullptr;
    if (INP_JOYSTICK != -1)
        m_joystick = new Joystick(INP_JOYSTICK);
    if (INP_XBOX_1 != -1)
        m_xbox.push_back(new XboxController(INP_XBOX_1));
    if (INP_XBOX_2 != -1)
        m_xbox.push_back(new XboxController(INP_XBOX_2));
    // initialize toggle variable to zero
    memset(m_toggle, 0, sizeof(m_toggle));
}


OperatorInputs::~OperatorInputs()
{
    if (m_joystick != nullptr)
        delete m_joystick;
    for (std::vector<XboxController * >::iterator it = m_xbox.begin() ; it != m_xbox.end(); it++)
         delete (*it);
    m_xbox.clear();
}


double OperatorInputs::xBoxLeftX(unsigned int i)
{
    if (i < m_xbox.size())
        return deadzoneFilterX(INVERT_X_AXIS * m_xbox[i]->GetX(GenericHID::JoystickHand::kLeftHand));
    return false;
}


double OperatorInputs::xBoxRightX(unsigned int i)
{
    if (i < m_xbox.size())
        return deadzoneFilterX(m_xbox[i]->GetX(GenericHID::JoystickHand::kRightHand));
    return false;
}


double OperatorInputs::xBoxLeftY(unsigned int i)
{
    if (i < m_xbox.size())
        return deadzoneFilterY(INVERT_Y_AXIS * m_xbox[i]->GetY(GenericHID::JoystickHand::kLeftHand));
    return false;
}


double OperatorInputs::xBoxRightY(unsigned int i)
{
    if (i < m_xbox.size())
        return deadzoneFilterY(m_xbox[i]->GetY(GenericHID::JoystickHand::kRightHand));
    return false;
}


bool OperatorInputs::xBoxAButton(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(A_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXABUTTON, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxBButton(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(B_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXBBUTTON, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxXButton(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(X_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXXBUTTON, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxYButton(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(Y_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXYBUTTON, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxLeftBumper(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(LEFT_BUMPER);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXLEFTBUMPER, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxRightBumper(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(RIGHT_BUMPER);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXRIGHTBUMPER, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxLeftTrigger(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        double axis = m_xbox[i]->GetRawAxis(XBOX_LEFT_TRIGGER_AXIS - 10);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXLEFTTRIGGER, (LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX));
        if (choice == kHold)
            return ((LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX));
    }
    return false;
}


bool OperatorInputs::xBoxRightTrigger(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        double axis = m_xbox[i]->GetRawAxis(XBOX_RIGHT_TRIGGER_AXIS - 10);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXRIGHTTRIGGER, (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX));
        if (choice == kHold)
            return (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX);
    }
    return false;
}


bool OperatorInputs::xBoxStartButton(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(START_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXSTARTBUTTON, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxBackButton(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(BACK_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXBACKBUTTON, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadUp(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = (m_xbox[i]->GetPOV() == 0);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXDPADUP, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadUpRight(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = (m_xbox[i]->GetPOV() == 45);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXDPADUPRIGHT, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadRight(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = (m_xbox[i]->GetPOV() == 90);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXDPADRIGHT, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadDownRight(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
    bool button = (m_xbox[i]->GetPOV() == 135);

    if (choice == kToggle)
        return toggle(i, TOGGLE_XBOXDPADDOWNRIGHT, button);
    if (choice == kHold)
        return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadDown(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
    bool button = (m_xbox[i]->GetPOV() == 180);

    if (choice == kToggle)
        return toggle(i, TOGGLE_XBOXDPADDOWN, button);
    if (choice == kHold)
        return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadDownLeft(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = (m_xbox[i]->GetPOV() == 225);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXDPADDOWNLEFT, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadLeft(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = (m_xbox[i]->GetPOV() == 270);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXDPADLEFT, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxDPadUpLeft(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = (m_xbox[i]->GetPOV() == 315);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXDPADUPLEFT, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxL3(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(L3_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXL3, button);
        if (choice == kHold)
            return button;
    }
    return false;
}


bool OperatorInputs::xBoxR3(ToggleChoice choice, unsigned int i)
{
    if (i < m_xbox.size())
    {
        bool button = m_xbox[i]->GetRawButton(R3_BUTTON);

        if (choice == kToggle)
            return toggle(i, TOGGLE_XBOXR3, button);
        if (choice == kHold)
            return button;
    }
    return false;
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


double OperatorInputs::joystickX()
{
    if (m_joystick != nullptr)
        return deadzoneFilterX(INVERT_X_AXIS * m_joystick->GetX());
    return false;
}


double OperatorInputs::joystickY()
{
    if (m_joystick != nullptr)
        return deadzoneFilterY(INVERT_Y_AXIS * m_joystick->GetY());
    return false;
}


double OperatorInputs::joystickZ()
{
    if (m_joystick != nullptr)
        return deadzoneFilterZ(m_joystick->GetZ());
    return false;
}


bool OperatorInputs::joystickAxis0Left(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        double axis = m_joystick->GetRawAxis(JOYSTICK_X_AXIS);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKAXIS0LEFT, (AXIS0_LEFT_MIN <= axis && axis <= AXIS0_LEFT_MAX));
        if (choice == kHold)
            return (AXIS0_LEFT_MIN <= axis && axis <= AXIS0_LEFT_MAX);
        return false;
    }
    return false;
}


bool OperatorInputs::joystickAxis0Right(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        double axis = m_joystick->GetRawAxis(JOYSTICK_X_AXIS);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKAXIS0RIGHT, (AXIS0_RIGHT_MIN <= axis && axis <= AXIS0_RIGHT_MAX));
        if (choice == kHold)
            return (AXIS0_RIGHT_MIN <= axis && axis <= AXIS0_RIGHT_MAX);
        return false;
    }
    return false;
}


bool OperatorInputs::joystickAxis1Back(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        double axis = m_joystick->GetRawAxis(JOYSTICK_Y_AXIS);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKAXIS1BACK, (AXIS1_BACK_MIN <= axis && axis <= AXIS1_BACK_MAX));
        if (choice == kHold)
            return (AXIS1_BACK_MIN <= axis && axis <= AXIS1_BACK_MAX);
        return false;
    }
    return false;
}


bool OperatorInputs::joystickAxis1Forward(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        double axis = m_joystick->GetRawAxis(JOYSTICK_Y_AXIS);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKAXIS1FORWARD, (AXIS1_FORWARD_MIN <= axis && axis <= AXIS1_FORWARD_MAX));
        if (choice == kHold)
            return (AXIS1_FORWARD_MIN <= axis && axis <= AXIS1_FORWARD_MAX);
        return false;
    }
    return false;
}


bool OperatorInputs::joystickTrigger(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetTrigger();

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKTRIGGER, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton2(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(2);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON2, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton3(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(3);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON3, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton5(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(5);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON5, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton6(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(6);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON6, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton7(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(7);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON7, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton8(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(8);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON8, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton9(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(9);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON9, button);
        if (choice == kHold)
            return button;
        return false;
    }
    return false;
}


bool OperatorInputs::joystickButton10(ToggleChoice choice)
{
    if (m_joystick != nullptr)
    {
        bool button = m_joystick->GetRawButton(10);

        if (choice == kToggle)
            return toggle(0, TOGGLE_JOYSTICKBUTTON10, button);
        if (choice == kHold)
            return button;
        return false;
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


double OperatorInputs::deadzoneFilterX(double joyStickValue)
{
    if (abs(joyStickValue) <= DEADZONE_X)
    {
        return 0;
    }
    double sub = joyStickValue / abs(joyStickValue);
    return (joyStickValue - (sub * DEADZONE_X)) / (1.0 - DEADZONE_X);
}


double OperatorInputs::deadzoneFilterY(double joyStickValue)
{
    if (abs(joyStickValue) <= DEADZONE_Y)
    {
        return 0;
    }
    double sub = joyStickValue / abs(joyStickValue);
    return (joyStickValue - (sub * DEADZONE_Y)) / (1.0 - DEADZONE_Y);
}


double OperatorInputs::deadzoneFilterZ(double joyStickValue)
{
    if (abs(joyStickValue) <= DEADZONE_Z)
    {
        return 0;
    }
    double sub = joyStickValue / abs(joyStickValue);
    return (joyStickValue - (sub * DEADZONE_Z)) / (1.0-DEADZONE_Z);
}