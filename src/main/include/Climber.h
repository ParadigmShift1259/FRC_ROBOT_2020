/**
 *  Climber.h
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#ifndef SRC_Climber_H_
#define SRC_Climber_H_


#include "OperatorInputs.h" 
#include <ctre\Phoenix.h>
#include <frc\Solenoid.h>
#include <frc/Spark.h>


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
    Spark *m_motor;
	Solenoid *m_solenoid;
};


#endif /* SRC_Climber_H_ */
