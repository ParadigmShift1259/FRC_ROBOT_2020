/**
 *  DriveStraightDouble.h
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_DriveStraightDouble_H_
#define SRC_DriveStraightDouble_H_

#include "OperatorInputs.h"
#include "Drivetrain.h"

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>



using namespace std;
using namespace frc;


class DriveStraightDouble
{
public:

    enum AutoState {kIdle, kDrive};

	DriveStraightDouble(OperatorInputs *inputs, Drivetrain *drivetrain);
	~DriveStraightDouble();
	void Init();
	void Loop();
	void Stop();
    void ConfigureProfile();
    void ConfigureLeftPID();
    void ConfigureRightPID();
    bool IsFinished() { return m_finished; }

protected:
	OperatorInputs *m_inputs;
    Drivetrain *m_drivetrain;

    TrapezoidProfile<units::meters>::Constraints m_constraints;
    units::meter_t m_tolerance;

    ProfiledPIDController<units::meters> *m_leftPIDController;
    double m_leftPIDvals[3];

    ProfiledPIDController<units::meters> *m_rightPIDController;
    double m_rightPIDvals[3];

    units::meter_t m_setpoint;
    bool m_finished;

    AutoState m_autostate;
};


#endif /* SRC_DriveStraightDouble_H_ */
