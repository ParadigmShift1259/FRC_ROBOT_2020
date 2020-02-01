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
	
	enum FeederState {kIdle, kFire, kRefresh};
	
	Feeder(OperatorInputs *inputs, Intake *intake);
	~Feeder();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();
	bool FeederBallCheck();
	void FeederStateMachine();

	void StartFire();
	bool GetFire() {return m_shoot};

protected:
    OperatorInputs *m_inputs;
    Intake *m_intake;
    Spark *m_motor;
	//Rev2mDistanceSensor *m_sensor;

	bool m_hasball;
	bool m_shoot;
	FeederState m_feederstate;
};


#endif /* SRC_Feeder_H_ */
