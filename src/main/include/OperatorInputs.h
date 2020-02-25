/**
 *  OperatorInputs.h
 *  Date:
 *  Last Edited By:
 */

#ifndef SRC_OPERATORINPUTS_H_
#define SRC_OPERATORINPUTS_H_

#include "Logger.h"

#include <frc\XboxController.h>
#include <vector>


using namespace frc;
using namespace std;

#define TOGGLE_MAX_CONTROLLERS 2
#define TOGGLE_MAX_BUTTONS 20

/// Holds the last changed button and joystick data
struct XboxLogData
{
	int m_xBoxController = 0;               //!< Xbox controller index

	int m_xBoxAButton = 0;                  //!< Circular A button (green)
	int m_xBoxBButton = 0;                  //!< Circular B button (red)
	int m_xBoxXButton = 0;                  //!< Circular X button (blue)
	int m_xBoxYButton = 0;                  //!< Circular Y button (yellow)
	int m_xBoxLeftBumper = 0;               //!< Left bumper ("shoulder")
	int m_xBoxRightBumper = 0;              //!< Right bumper ("shoulder")
	int m_xBoxLeftTrigger = 0;              //!< Left "gun" trigger on far side
	int m_xBoxRightTrigger = 0;             //!< Left "gun" trigger on far side
	int m_xBoxStartButton = 0;              //!< Small oval button, center right
	int m_xBoxBackButton = 0;               //!< Small oval button, center left
	int m_xBoxDPadUp = 0;                   //!< Up (north) on Dpad "discrete button joystick"
	int m_xBoxDPadUpRight = 0;              //!< Up and right (north east)
	int m_xBoxDPadRight = 0;                //!< Right (east)
	int m_xBoxDPadDownRight = 0;            //!< Down and right (south east)
	int m_xBoxDPadDown = 0;                 //!< Down (south)
	int m_xBoxDPadDownLeft = 0;             //!< Down left (south west)
	int m_xBoxDPadLeft = 0;                 //!< Left (west)
	int m_xBoxDPadUpLeft = 0;               //!< Up and left (north west)
	int m_xBoxL3 = 0;                       //!< 3rd axis (trigger) left side
	int m_xBoxR3 = 0;                       //!< 3rd axis (trigger) right side

	double m_xBoxLeftX = 0.0;               //!< Left stick x axis
	double m_xBoxLeftY = 0.0;               //!< Left stick y axis

	double m_xBoxRightX = 0.0;              //!< Right stick x axis
	double m_xBoxRightY = 0.0;              //!< Right stick y axis
};


/// Handle operator inputs
class OperatorInputs
{
public:
	enum ToggleChoice
	{
		kToggle = 0,    //!< Press once to turn on, remains on until pressed again ("sticky" value)
		kHold = 1       //!< Normal momentary press behavior; button is on as long as the operator holds it
	};

	OperatorInputs();
	~OperatorInputs();

	void Init();                                    //!< Call to set up the log pointer
	void Loop();                                    //!< Call periodically to log the inputs

	double xBoxLeftX(unsigned int i = 0);           //!< Left stick x axis
	double xBoxLeftY(unsigned int i = 0);           //!< Left stick y axis

	double xBoxRightX(unsigned int i = 0);          //!< Right stick x axis
	double xBoxRightY(unsigned int i = 0);          //!< Right stick y axis

	bool xBoxAButton(ToggleChoice choice = kToggle, unsigned int i = 0);       //!< Circular A button (green)
	bool xBoxBButton(ToggleChoice choice = kToggle, unsigned int i = 0);       //!< Circular B button (red)
	bool xBoxXButton(ToggleChoice choice = kToggle, unsigned int i = 0);       //!< Circular X button (blue)
	bool xBoxYButton(ToggleChoice choice = kToggle, unsigned int i = 0);       //!< Circular Y button (yellow)
	bool xBoxLeftBumper(ToggleChoice choice = kToggle, unsigned int i = 0);    //!< Left bumper ("shoulder")
	bool xBoxRightBumper(ToggleChoice choice = kToggle, unsigned int i = 0);   //!< Right bumper ("shoulder")
	bool xBoxLeftTrigger(ToggleChoice choice = kToggle, unsigned int i = 0);   //!< Left "gun" trigger on far side; triggers have a variable "pull" value that we threshold to bool
	bool xBoxRightTrigger(ToggleChoice choice = kToggle, unsigned int i = 0);  //!< Left "gun" trigger on far side; triggers have a variable "pull" value that we threshold to bool
	bool xBoxStartButton(ToggleChoice choice = kToggle, unsigned int i = 0);   //!< Small oval button, center right
	bool xBoxBackButton(ToggleChoice choice = kToggle, unsigned int i = 0);    //!< Small oval button, center left
	bool xBoxDPadUp(ToggleChoice choice = kToggle, unsigned int i = 0);        //!< Up (north) on Dpad "discrete button joystick"
	bool xBoxDPadUpRight(ToggleChoice choice = kToggle, unsigned int i = 0);   //!< Up and right (north east)
	bool xBoxDPadRight(ToggleChoice choice = kToggle, unsigned int i = 0);     //!< Right (east)
	bool xBoxDPadDownRight(ToggleChoice choice = kToggle, unsigned int i = 0); //!< Down and right (south east)
	bool xBoxDPadDown(ToggleChoice choice = kToggle, unsigned int i = 0);      //!< Down (south)
	bool xBoxDPadDownLeft(ToggleChoice choice = kToggle, unsigned int i = 0);  //!< Down left (south west)
	bool xBoxDPadLeft(ToggleChoice choice = kToggle, unsigned int i = 0);      //!< Left (west)
	bool xBoxDPadUpLeft(ToggleChoice choice = kToggle, unsigned int i = 0);    //!< Up and left (north west)
	bool xBoxL3(ToggleChoice choice = kToggle, unsigned int i = 0);            //!< 3rd axis (trigger) left side
	bool xBoxR3(ToggleChoice choice = kToggle, unsigned int i = 0);            //!< 3rd axis (trigger) right side

	/// Access button by index
	/// \param Button       Button index
	/// \param choice       Actuation style
	/// \param i            Xbox controller index
	///
	/// \returns the new button state true = on, false = off
	bool xBox(int Button, ToggleChoice choice = kToggle, unsigned int i = 0);

protected:
	vector<XboxController *> m_xbox;

private:
	bool toggle(unsigned int controller, int button, bool value);          //!< Takes a given controller and button combination, returns true on the rising edge of value
	double deadzoneFilter(double joyStickValue, double axisDeadZone);      //!< If the input is small, return 0 else normalize the output value

	double deadzoneFilterX(double joyStickValue)
	{
		return deadzoneFilter(joyStickValue, DEADZONE_X);
	}

	double deadzoneFilterY(double joyStickValue)
	{
		return deadzoneFilter(joyStickValue, DEADZONE_Y);
	}

	bool m_toggle[TOGGLE_MAX_CONTROLLERS][TOGGLE_MAX_BUTTONS];             //!< Maintation the toggle/hold states of each button on each xbox controller

	Logger *m_log;
	vector<int*> m_dataInt[TOGGLE_MAX_CONTROLLERS];                        //!< Collection of integer pointers to member variables for logging
	vector<double*> m_dataDouble[TOGGLE_MAX_CONTROLLERS];                  //!< Collection of double pointers to member variables for logging

	XboxLogData m_logData[TOGGLE_MAX_CONTROLLERS];                     //!< Holds the last changed button and joystick data
};


#endif /* SRC_OPERATORINPUTS_H_ */
