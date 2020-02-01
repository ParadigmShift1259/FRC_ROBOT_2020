/**
 *  Intake.h
 *  Date:
 *  Last Edited By:
 *   Jival.C
 */


#ifndef SRC_Intake_H_
#define SRC_Intake_H_


#include "OperatorInputs.h"
#include "Const.h"
#include "Feeder.h"
#include <frc\Solenoid.h>
#include <frc\Spark.h>
#include <rev\Rev2mDistanceSensor.h>


using namespace frc;
using namespace rev;


class Intake
{
public:

	/**
	 * kIdle ->
	 * kGather ->
	 */
	enum IntakeState {kIdle, kGather};

	Intake(OperatorInputs *inputs);
	~Intake();
	void Init();
	void Loop();
	void Stop();

	void SetState(intkSt state);
	// Called by Feeder when Shooter requests shooting
	// If there are no balls left, shooting will be set to false
	void SetDrivingBecauseShooting() { m_drivingbecauseshooting = true;}
	bool GetDrivingBecauseShooting() { return m_drivingbecauseshooting; }

	int GetBallCount() { return m_ballcount;}

	void Dashboard();
	//int  BallCount();
	//void DstncSnsrModeSet(Rev2mDistanceSensor *temp);
	//Rev2mDistanceSensor *m_sensormode;
	//const double snsrDst = 2.5;
private:
	bool NullCheck();
	

protected:
    OperatorInputs *m_inputs;
	Feeder *m_feeder;
    Solenoid *m_solenoid1;
    Solenoid *m_solenoid2;
    Spark *m_motor1;
    Spark *m_motor2;
	//Rev2mDistanceSensor *m_sensor1;
	//Rev2mDistanceSensor *m_sensor2;
	//Rev2mDistanceSensor *m_sensor3;
	IntakeState m_intakestate;
	int m_ballcount;
	bool m_drivingbecauseshooting;
	
};


#endif /* SRC_Intake_H_ */
