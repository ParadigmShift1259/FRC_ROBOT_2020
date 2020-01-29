/**
 *  Intake.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_Climber_H_
#define SRC_Climber_H_


#include "OperatorInputs.h" 
#include <ctre\Phoenix.h>


using namespace frc;


class Climber
{
public:
	Climber(OperatorInputs *inputs);
	~Climber();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

protected:
    OperatorInputs *m_inputs;
};


#endif /* SRC_Climber_H_ */
