#ifndef SRC_Feeder_H_
#define SRC_Feeder_H_


#include "OperatorInputs.h" 
#include "Intake.h"
#include <frc/Spark.h>


using namespace frc;


class Feeder
{
public:
	Feeder(OperatorInputs *inputs, Intake *intake);
	~Feeder();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

protected:
    OperatorInputs *m_inputs;
    Intake *m_intake;
    Spark *m_motor;
};


#endif /* SRC_Feeder_H_ */
