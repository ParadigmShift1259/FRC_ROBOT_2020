/**
 *  Feeder.h
 *  Date: 2/20/2020
 *  Last Edited By: Geoffrey Xue
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

class Feeder
{
public:
	enum FeederState {kIdle, kRefresh, kStuff};
	
	Feeder(OperatorInputs *inputs, Intake *intake);
	~Feeder();
	void Init();
	void Loop();
	void Stop();
	void Dashboard();

	void SetStuffTime(double stufftime) { m_stufftime = stufftime; }
	void SetStuffing(bool stuff = true);
	bool IsStuffing();
	int GetBallCount();

	void SetLoaded(bool loaded) {m_loaded = loaded; }

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
	double m_stufftime;

	FeederState m_feederstate;

	vector<int*> m_dataInt;
	vector<double*> m_dataDouble;
};


#endif /* SRC_Feeder_H_ */
