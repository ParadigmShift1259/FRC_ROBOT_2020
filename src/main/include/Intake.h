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
#include <frc\Solenoid.h>
#include <frc\Spark.h>
#include <frc\DigitalInput.h>


using namespace frc;

class Intake
{
public:

	/**
	 * kIdle ->
	 * kGather ->
	 */
	enum IntakeState {kIdle, kGather, kStuff};
	enum IntakePosition {kDown, kUp};
	enum BallState {kZero, kTwoCheck, kTwo, kThree};

	Intake(OperatorInputs *inputs);
	~Intake();
	void Init();
	void Loop();
	void Stop();

	// Called by Feeder when Shooter requests shooting
	// If there are no balls left, shooting will be set to false
	void SetStuffing(bool stuff = true);
	bool IsStuffing();

	bool CanRefresh();
	void IntakePositionLoop();
	bool IntakeUp();

	void Dashboard();

private:
	bool NullCheck();
	void CountBalls();

protected:
    OperatorInputs *m_inputs;

    Solenoid *m_solenoid;
    Spark *m_rollermotor;
    Spark *m_wheelmotor;
	DigitalInput *m_rollersensor;
	DigitalInput *m_chutesensor;

	Timer m_timer;
	Timer m_balltimer;

	IntakeState m_intakestate;
	IntakePosition m_intakeposition;
	BallState m_ballstate;

	int m_ballcount;
	bool m_stuffing;
	bool m_gathering;
};


#endif /* SRC_Intake_H_ */
