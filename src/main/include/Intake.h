/**
 *  Intake.h
 *  Date: 2/20/2020
 *  Last Edited By: Geoffrey Xue
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
	enum IntakeState {kIdle, kGather, kStuff};
	enum IntakePosition {kDown, kUp};
	enum BallState {kZero, kTwo, kThree};

	Intake(OperatorInputs *inputs);
	~Intake();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

	void SetStuffing(bool stuff = true);
	bool IsStuffing();

	bool CanRefresh();
	int GetBallCount() { return m_ballcount; }

	void IntakePositionLoop();
	bool IntakeUp();

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
