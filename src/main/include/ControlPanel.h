/**
 *  ControlPanel.h
 *  Date: 1/7/2020 
 *  Last Edited By: Jival Chandrashekar
 */


#ifndef SRC_ControlPanel_H_
#define SRC_ControlPanel_H_


#include "OperatorInputs.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
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
	ControlPanel(OperatorInputs *inputs);
	~ControlPanel();
	void Init();
	void Loop();
	void Stop();
	
	
protected:
	void ControlPanelStates();
	ColorOptions GetColor();
	bool SensorSanityCheck();
	ColorOptions GetTargetColor();
	void ChangeSpinnerState();
	void ChangeSpinnerState(SpinnerState);

private:
	OperatorInputs *m_inputs;
	ColorOptions m_targetcolor;
	int m_colordelta;
	TalonSRX *m_spinner;
	ColorSensorV3 *m_colorsensor;
	Timer *m_timer;
	int m_redCount;
	int m_blueCount;
	SpinnerState m_spinnerstate;
	int m_direction;
	int m_startencodervalue;
	int m_currentencodervalue;
	

	ColorMatch m_colormatcher;
	double m_confidence;

	ColorOptions m_currentcolor,m_previouscolor;
 //1 = red,  2 = blue
	bool m_stop;

	double m_spinnerstatehelper;
	double m_spinnerPIDvals[7];
	double m_spinnersetpoint;
};


#endif /* SRC_ControlPanel_H_ */