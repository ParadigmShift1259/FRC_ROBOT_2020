/**
 *  Intake.h
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#ifndef SRC_Intake_H_
#define SRC_Intake_H_


#include "OperatorInputs.h"
#include <frc\Solenoid.h>
#include <frc\Spark.h>
#include <rev\Rev2mDistanceSensor.h>


using namespace frc;
using namespace rev;


class Intake
{
public:
	Intake(OperatorInputs *inputs);
	~Intake();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();
	void BalStateMachine();
	bool ChamberingCs();
	bool LckdNloded();
	void DstncSnsrModeSet(Rev2mDistanceSensor *temp);
	Rev2mDistanceSensor *m_sensormode;
	const double snsrDst = 2.5;
	enum intakeState {idle, gathering, chambering, lckdNloded, emptying}; // LckdNloded = Full
	intakeState intkSt;
private:
	bool NullCheck();
	

protected:
    OperatorInputs *m_inputs;
    Solenoid *m_solenoid1;
    Solenoid *m_solenoid2;
    Spark *m_motor1;
    Spark *m_motor2;
	Rev2mDistanceSensor *m_sensor1;
	Rev2mDistanceSensor *m_sensor2;
	Rev2mDistanceSensor *m_sensor3;
	
};


#endif /* SRC_Intake_H_ */
