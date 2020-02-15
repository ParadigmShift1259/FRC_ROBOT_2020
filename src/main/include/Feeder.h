/**
 *  Feeder.h
 *  Date:
 *  Last Edited By:
 * 
 */


#ifndef SRC_Feeder_H_
#define SRC_Feeder_H_


#include "OperatorInputs.h" 
#include "Intake.h"
#include "CDSensors.h"

#include <rev\CANSparkMax.h>
#include <frc/Timer.h>

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>


using namespace frc;
using namespace rev;
using namespace std;


class Feeder
{
public:
	
	enum FeederState {kIdle, kRefresh, kDrive, kDriveWait};
	
	Feeder(OperatorInputs *inputs, Intake *intake, CDSensors *sensors);
	~Feeder();
	void Init();
	void Loop();
	void Stop();

	void FeederStateMachine();
	// Called by shooter to start one cycle
	void StartFire();
	bool GetFinished();
	void ConfigureProfile();
	void ConfigurePID();
	void ConfigureLowPID();

protected:
    OperatorInputs *m_inputs;
    Intake *m_intake;
	CDSensors *m_sensors;
	Timer m_timer;

    CANSparkMax *m_motor;
	CANEncoder *m_encoder;

    TrapezoidProfile<units::meters>::Constraints m_constraints;
    units::meter_t m_tolerance;

    ProfiledPIDController<units::meters> *m_feederPID;
    double m_feederPIDvals[3];
	units::meter_t m_setpoint;
	units::meter_t m_prevgoal;

	bool m_hasball;
	bool m_shoot;
	FeederState m_feederstate;
	double m_power;
};


#endif /* SRC_Feeder_H_ */
