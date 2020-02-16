/**
 *  ControlPanel.h
 *  Date: 1/7/2020 
 *  Last Edited By: Jival Chandrashekar
 */


#ifndef SRC_ControlPanel_H_
#define SRC_ControlPanel_H_


#include "OperatorInputs.h"
#include "CDSensors.h"
#include "GyroDrive.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/DriverStation.h>

using namespace frc;
using namespace rev;


class ControlPanel
{
public:
	/**
	 * States
	 * Off - Nothing happens during loop
	 * BlindSpin - Attempts to spin the control panel from three to five times 
	 * ColorSpin - Attempts to spin the control panel to a specific color
	 */
	enum SpinnerState {kOff, kRotationControl, kPositionControl};
	enum ColorOptions {kNone, kYellow, kRed, kGreen, kBlue};
	ControlPanel(OperatorInputs *inputs, CDSensors *sensors, GyroDrive *gyrodrive);
	~ControlPanel();
	void Init();
	void Loop();
	void Stop();
	
	
protected:
	//Should these be public???
	void ControlPanelStates();
	void ChangeSpinnerState();
	void ChangeSpinnerState(SpinnerState);

private:
	ColorOptions GetColor();
	ColorOptions GetTargetColor();

	OperatorInputs *m_inputs;
	GyroDrive *m_gyrodrive;
	ColorOptions m_targetcolor;
	TalonSRX *m_spinner;
	Solenoid *m_solenoid;
	ColorSensorV3 *m_colorsensor;
	Timer *m_timer;
	int m_redCount;
	int m_blueCount;
	SpinnerState m_spinnerstate;
	int m_direction;
	int m_startencodervalue;
	int m_currentencodervalue;
	
	ColorOptions m_currentcolor, m_previouscolor;
	bool m_stop;

	double m_spinnersetpoint;
};


#endif /* SRC_ControlPanel_H_ */