/**
 *  Vision.h
 *  Date: 1/7/2020 
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Vision_H_
#define SRC_Vision_H_


#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "OperatorInputs.h"


using namespace frc;
using namespace std;


class Vision
{
public:
    Vision(OperatorInputs *inputs);
    ~Vision();
    void Init();
    void Loop();
    void Stop();

    bool GetActive();
    double GetDistance();
    double GetAngle();
    void SetLED(bool on);
    void SetCamera(int camera) { m_camerachoice = camera; }

    void IntakeSensorUpdate(bool update);
    double IntakeAngle();
    double IntakeDistance();

protected:
    double DegreesToRadians(double degrees);
    double RadiansToDegrees(double radians);
private:
    OperatorInputs *m_inputs;
    
    shared_ptr<NetworkTable> m_dashboard;
    int m_camerachoice;
    
    shared_ptr<NetworkTable> m_networktable;
    bool m_led;
    double m_tx;
    double m_ty;
    double m_ta;
    double m_ts;
    vector<double> m_tcornx;
    vector<double> m_tcorny;
    bool m_active;

    double m_verticalangle;

    double m_distance;
    double m_horizontalangle;

    double m_averagedistance[3];
    double m_averageangle[3];
};


#endif /* SRC_Vision_H_ */