/**
 *  TurnAngleDegrees.h
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_TurnAngleDegrees_H_
#define SRC_TurnAngleDegrees_H_

#include "OperatorInputs.h"
#include "Drivetrain.h"

#include <units/units.h>

#include <frc/controller/PIDController.h>

using namespace std;
using namespace frc;


class TurnAngleDegrees
{
public:

    enum AutoState {kIdle, kDrive};

	TurnAngleDegrees(OperatorInputs *inputs, Drivetrain *drivetrain);
	~TurnAngleDegrees();
	void Init();
	void Loop();
	void Stop();
    void ConfigureGyroPID();
    bool IsFinished() { return m_finished; }

protected:
	OperatorInputs *m_inputs;
    Drivetrain *m_drivetrain;

	frc2::PIDController *m_gyroPIDController;
    double m_gyroPIDvals[3];
    units::degree_t m_tolerance;

    units::degree_t m_setpoint;
    bool m_finished;

    AutoState m_autostate;
};


#endif /* SRC_TurnAngleDegrees_H_ */
