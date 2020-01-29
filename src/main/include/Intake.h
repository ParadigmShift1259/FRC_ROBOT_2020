/**
 *  Intake.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_Intake_H_
#define SRC_Intake_H_


#include "OperatorInputs.h"
#include <frc\Solenoid.h>
#include <frc\Spark.h>


using namespace frc;


class Intake
{
public:
	Intake(OperatorInputs *inputs);
	~Intake();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

protected:
    OperatorInputs *m_inputs;
    Solenoid *m_solenoid1;
    Solenoid *m_solenoid2;
    Spark *m_motor1;
    Spark *m_motor2;
};


#endif /* SRC_Intake_H_ */
