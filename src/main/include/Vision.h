/**
 *  Vision.h
 *  Date: 1/7/2020 
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Vision_H_
#define SRC_Vision_H_


#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "OperatorInputs.h"
#include "Logger.h"


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
    void ChangeCamera() { m_camerachoice = !m_camerachoice; }

protected:
    double DegreesToRadians(double degrees);
    double RadiansToDegrees(double radians);
private:
    OperatorInputs *m_inputs;
    
    shared_ptr<NetworkTable> m_camera;
    int m_camerachoice;
    
    shared_ptr<NetworkTable> m_networktable;
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

	Logger *m_log;
	vector<int*> m_dataInt;
	vector<double*> m_dataDouble;
};


#endif /* SRC_Vision_H_ */