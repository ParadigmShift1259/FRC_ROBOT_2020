/**
 *  Intake.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_Intake_H_
#define SRC_Intake_H_


#include "OperatorInputs.h" 
#include <ctre\Phoenix.h>


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
};


#endif /* SRC_Intake_H_ */
