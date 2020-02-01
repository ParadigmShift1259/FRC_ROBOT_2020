/**
 *  Feeder.h
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#ifndef SRC_Feeder_H_
#define SRC_Feeder_H_


#include "OperatorInputs.h" 
#include "Intake.h"
#include <frc/Spark.h>
#include <rev/Rev2mDistanceSensor.h>


using namespace frc;
using namespace rev;


class Feeder
{
public:
	Feeder(OperatorInputs *inputs, Intake *intake);
	~Feeder();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();
	bool FDRBallCheck();
	void BlStMchne();
	const double snsrDstFdr = 3;
	enum fdrSt {Idle, Feeding, Loaded, Firing, Empty};
	int rnLp;
	fdrSt fdrSt; 

protected:
    OperatorInputs *m_inputs;
    Intake *m_intake;
    Spark *m_motor;
	Rev2mDistanceSensor *m_sensor;
};


#endif /* SRC_Feeder_H_ */
