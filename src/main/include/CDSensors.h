/**
 *  CDSensors.h
 *  Date:
 *  Last Edited By:
 *  
 */


#ifndef SRC_CDSensors_H_
#define SRC_CDSensors_H_

#include <string>

#include <frc/i2c.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "../native/include/rev/Rev2mDistanceSensorEx.h"
#include <rev/ColorSensorV3.h>


using namespace frc;
using namespace rev;
using namespace std;

enum ESensors
{
    RollerSensor = 0,
    Chute2Sensor,
    Chute1Sensor,
    FeederSensor
};

class CDSensors
{
public:
	CDSensors();
	~CDSensors();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();
    bool BallPresent(int sensornum);
    ColorSensorV3* GetColorSensor() { return m_colorsensor; }

private:
    void MuxSelect(uint8_t i);
    void MuxSelectMask(uint8_t mask);
    void MuxSelectAll();
    double ReadDistance(Rev2mDistanceSensorEx& distSensor, int sensorNum); 

    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

    I2C m_mux;

    Rev2mDistanceSensorEx* m_distsensor1;
    Rev2mDistanceSensorEx* m_distsensor2;
    Rev2mDistanceSensorEx* m_distsensor3;
    Rev2mDistanceSensorEx* m_distsensor4;
    ColorSensorV3* m_colorsensor;

    double m_ballpresent1;
    double m_ballpresent2;
    double m_ballpresent3;
    double m_ballpresent4;
};


#endif /* SRC_CDSensors_H_ */
