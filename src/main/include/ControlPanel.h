/**
 *  ControlPanel.h
 *  Date: 1/7/2020 
 *  Last Edited By: Jival Chandrashekar
 */


#ifndef SRC_ControlPanel_H_
#define SRC_ControlPanel_H_


#include "OperatorInputs.h"
#include "GyroDrive.h"
#include "Intake.h"
// #define USE_LOGGER
#ifdef USE_LOGGER
#include "Logger.h"
#endif

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
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
	enum ColorOptions {kNone, kYellow, kRed, kGreen, kBlue, kSize };
	ControlPanel(OperatorInputs *inputs, GyroDrive *gyrodrive, Intake *intake);
	~ControlPanel();
	void Init();
	void Loop();
	void Stop();

	bool ControlPanelUp() { return m_solenoid->Get(); }
	
protected:
	//Should these be public???
	void ControlPanelStates();
	void ChangeSpinnerState();
	void ChangeSpinnerState(SpinnerState);
	void Dashboard();

private:
	ColorOptions GetColor();
	ColorOptions GetTargetColor();

	OperatorInputs *m_inputs;
	GyroDrive *m_gyrodrive;
	Intake *m_intake;

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
	string m_color[kSize];
	ColorOptions m_currentcolor, m_previouscolor;
	bool m_stop;
	double m_spinnersetpoint;
#ifdef USE_LOGGER
	Logger m_log;
	vector<int*> m_dataInt;
	vector<double*> m_dataDouble;
#endif
};


#endif /* SRC_ControlPanel_H_ */