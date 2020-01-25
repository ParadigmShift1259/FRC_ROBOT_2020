/**
 *  ControlPanel.h
 *  Date: 1/7/2020 
 *  Last Edited By: Jival Chandrashekar
 */


#ifndef SRC_ControlPanel_H_
#define SRC_ControlPanel_H_


#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <ctre\Phoenix.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "OperatorInputs.h"


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
	enum SpinnerState {kOff, kBlindSpin, kColorSpin};

	ControlPanel(OperatorInputs *inputs);
	~ControlPanel();
	void Init();
	void Loop();
	void Stop();
	
protected:
	void ControlPanelStates();
	int GetColor();
	bool SensorSanityCheck();


private:
	ColorSensorV3 *m_colorsensor;
	TalonSRX *m_spinner;
	OperatorInputs *m_inputs;

	SpinnerState m_spinnerstate;

	ColorMatch m_colormatcher;
	double m_confidence;

	double m_currentcolor;
	double m_previouscolor;
	double m_registeredcolor;
	double m_colorbouncecount;
	double m_colorregisteredcount[4];
	bool m_stop;

	double m_targetcolor;

	double m_spinnerPIDvals[7];
	double m_spinnersetpoint;
	
};


#endif /* SRC_ControlPanel_H_ */