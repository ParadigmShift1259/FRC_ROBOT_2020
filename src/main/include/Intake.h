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
#include "CDSensors.h"
#include <frc\Solenoid.h>
#include <frc\Spark.h>


using namespace frc;

class Intake
{
public:

	/**
	 * kIdle ->
	 * kGather ->
	 */
	enum IntakeState {kIdle, kGather, kEject};

	Intake(OperatorInputs *inputs, CDSensors *sensors);
	~Intake();
	void Init();
	void Loop();
	void Stop();

	// Called by Feeder when Shooter requests shooting
	// If there are no balls left, shooting will be set to false
	void SetDrivingBecauseShooting() { m_drivingbecauseshooting = true;}
	bool GetDrivingBecauseShooting() { return m_drivingbecauseshooting; }
	void CountBalls();

	bool LoadRefresh();

	void Dashboard();

private:
	bool NullCheck();
	

protected:
    OperatorInputs *m_inputs;
	CDSensors *m_sensors;

    Solenoid *m_solenoid1;
    Solenoid *m_solenoid2;
    Spark *m_motor1;
    Spark *m_motor2;
	IntakeState m_intakestate;
	int m_ballcount;
	bool m_drivingbecauseshooting;
	
};


#endif /* SRC_Intake_H_ */
