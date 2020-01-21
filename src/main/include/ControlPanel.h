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
    ControlPanel(OperatorInputs *inputs);
    ~ControlPanel();
    void Init();
    void Loop();
    void Stop();


private:
    ColorSensorV3 *m_colorsensor;
    TalonSRX *m_spinner;
    OperatorInputs *m_inputs;
};


#endif /* SRC_ControlPanel_H_ */