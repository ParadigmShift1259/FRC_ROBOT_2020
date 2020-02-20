/**
 *  Feeder.h
 *  Date:
 *  Last Edited By:
 * Jival.C
 */


#ifndef SRC_Feeder_H_
#define SRC_Feeder_H_


#include "OperatorInputs.h" 
#include "Intake.h"
#include <ctre/Phoenix.h>

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

using namespace frc;
using namespace std;


class Feeder
{
public:
	
	enum FeederState {kIdle, kRefresh, kStuff};
	
	Feeder(OperatorInputs *inputs, Intake *intake);
	~Feeder();
	void Init();
	void Loop();
	void Stop();

	void FeederStateMachine();
	void SetStuffing(bool stuff = true);
	bool IsStuffing();

protected:
    OperatorInputs *m_inputs;
	Intake *m_intake;
    WPI_TalonSRX *m_motor;
	Timer m_timer;

    TrapezoidProfile<units::inches>::Constraints m_constraints;
    units::inch_t m_tolerance;

    ProfiledPIDController<units::inches> *m_feederPID;
    double m_feederPIDvals[3];
	units::inch_t m_goal;
	bool m_loaded;
	bool m_stuffing;

	FeederState m_feederstate;
};


#endif /* SRC_Feeder_H_ */
