/**
 *  ControlPanel.h
 *  Date: 1/7/2020 
 *  Last Edited By: Jival Chandrashekar
 */


#ifndef SRC_ControlPanel_H_
#define SRC_ControlPanel_H_


#include "rev/ColorSensorV3.h"
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
    int GetColor();


private:
    ColorSensorV3 *m_colorsensor;
    TalonSRX *m_spinner;
    OperatorInputs *m_inputs;

    SpinnerState m_spinnerstate;
    
    double m_spinpower;
    int m_desiredcolor; // 1-4 Yellow, Green, Blue, Red
    int m_colorcounter;
    int m_previouscolor;
    int m_currentcolor;
    int m_colorcount;
    bool m_stop;
    int m_colorregistertimes;
    int m_registeredcolor;
    
};


#endif /* SRC_ControlPanel_H_ */