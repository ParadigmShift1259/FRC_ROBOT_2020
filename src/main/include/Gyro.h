/**
 *  Gyro.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_Gyro_H_
#define SRC_Gyro_H_


#include <ctre\Phoenix.h>
#include "Logger.h"


using namespace frc;


class DualGyro
{
public:
	DualGyro(int gyro1 = -1, int gryo2 = -1);
	~DualGyro();
	void Init();
	void Loop();
	void Stop();
	bool GetHeading(double &heading);
	void Dashboard();
	void ZeroHeading();
	void ResetDeltaHeading();
	double GetDeltaHeading();

protected:
	PigeonIMU *m_pigeon1;
	PigeonIMU *m_pigeon2;
	double m_gyroval1[3];
	double m_gyroval2[3];
	double m_heading1;
	double m_heading2;
	bool m_gyrovalid1;
	bool m_gyrovalid2;
	double m_prevheading;

	Logger *m_log;
	vector<int*> m_dataInt;
	vector<double*> m_dataDouble;
};


#endif /* SRC_Gyro_H_ */
