/**
 *  Pneumatics.h
 *  Date:
 *  Last Edited By:
 */

#ifndef SRC_PNEUMATICS_H_
#define SRC_PNEUMATICS_H_


#include "frc/Compressor.h"
#include "frc/PowerDistributionPanel.h"
#include "frc/Timer.h"


using namespace frc;
using namespace std;


class Pneumatics
{
public:
	enum Stage {kRun, kWait};

	Pneumatics();
	~Pneumatics();
	void Init();
	void Loop();
	void Stop();

private:
	Compressor *m_compressor;
	PowerDistributionPanel *m_pdp;
	Stage m_stage;
	bool m_pressureSwitch;
	double m_compressorCurrent;
	double m_totalCurrent;
	double m_voltage;
	double m_temperature;

	Timer m_timer;
	
	vector<int*> m_dataInt;
	vector<double*> m_dataDouble;
};


#endif /* SRC_PNEUMATICS_H_ */
