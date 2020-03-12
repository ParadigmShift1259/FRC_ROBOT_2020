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


using namespace frc;


class Climber
{
public:

	enum ClimberState {kIdle, kAutoDrive, kDrive};

	Climber(OperatorInputs *inputs);
	~Climber();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

	bool DeployRequest();
	void CanDeploy(bool deploy = false);

protected:
    OperatorInputs *m_inputs;
    WPI_TalonSRX *m_motor;
	Timer m_timer;

	bool m_deployrequest;
	bool m_deployready;

	ClimberState m_state;
};


#endif /* SRC_Climber_H_ */
