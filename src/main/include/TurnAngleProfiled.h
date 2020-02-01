/**
 *  TurnAngleProfiled.h
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_TurnAngleProfiled_H_
#define SRC_TurnAngleProfiled_H_

#include "OperatorInputs.h"
#include "Drivetrain.h"

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>


using namespace std;
using namespace frc;


class TurnAngleProfiled
{
public:

    enum AutoState {kIdle, kDrive};

	TurnAngleProfiled(OperatorInputs *inputs, Drivetrain *drivetrain);
	~TurnAngleProfiled();
	void Init();
	void Loop();
	void Stop();
    void ConfigureProfile();
    void ConfigureGyroPID();
    bool IsFinished() { return m_finished; }

protected:
	OperatorInputs *m_inputs;
    Drivetrain *m_drivetrain;

    ProfiledPIDController<units::meters> *m_gyroPIDController;
    // Actually degrees, but needs to be a measure of distance
    TrapezoidProfile<units::meters>::Constraints m_constraints;
    units::degree_t m_tolerance;
    double m_gyroPIDvals[3];

    units::degree_t m_setpoint;
    bool m_finished;

    AutoState m_autostate;
};


#endif /* SRC_TurnAngleProfiled_H_ */
