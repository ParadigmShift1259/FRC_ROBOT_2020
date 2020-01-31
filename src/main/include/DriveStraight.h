/**
 *  DriveStraight.h
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_DriveStraight_H_
#define SRC_DriveStraight_H_

#include "OperatorInputs.h"
#include "Drivetrain.h"

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>



using namespace std;
using namespace frc;


class DriveStraight
{
public:

    enum AutoState {kIdle, kDrive};

	DriveStraight(OperatorInputs *inputs, Drivetrain *drivetrain);
	~DriveStraight();
	void Init();
	void Loop();
	void Stop();
    void ConfigureProfile();
    void ConfigureGyroPID();
    void ConfigureEncoderPID();

protected:
	OperatorInputs *m_inputs;
    Drivetrain *m_drivetrain;

    ProfiledPIDController<units::meters> *m_encoderPIDController;
    TrapezoidProfile<units::meters>::Constraints m_constraints;
    units::meter_t m_tolerance;
    double m_encoderPIDvals[3];

	frc2::PIDController *m_gyroPIDController;
    double m_gyroPIDvals[3];

    units::meter_t m_setpoint;
    bool m_finished;

    AutoState m_autostate;
};


#endif /* SRC_DriveStraight_H_ */
