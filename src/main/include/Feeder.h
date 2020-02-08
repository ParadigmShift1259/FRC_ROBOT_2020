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
#include <frc\Solenoid.h>
#include <frc/Spark.h>
#include <rev/Rev2mDistanceSensor.h>


using namespace frc;
using namespace rev;


class Feeder
{
public:
	
	enum FeederState {kIdle, kFire, kRefresh, kReverse};
	
	Feeder(OperatorInputs *inputs, Intake *intake);
	~Feeder();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

	void FeederStateMachine();
	// Called by shooter to start one cycle
	void StartFire();
	bool GetFire();

protected:
    OperatorInputs *m_inputs;
    Intake *m_intake;
    Spark *m_motor;
	Solenoid *m_solenoid;

	bool m_hasball;
	bool m_shoot;
	FeederState m_feederstate;
};


#endif /* SRC_Feeder_H_ */
