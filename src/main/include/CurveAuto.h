/**
 *  CurveAuto.h
 *  Date: 2/3/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_CurveAuto_H_
#define SRC_CurveAuto_H_

#include "OperatorInputs.h"
#include "Drivetrain.h"

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>


using namespace std;
using namespace frc;


class CurveAuto
{
public:

    enum AutoState {kIdle, kDrive};

	CurveAuto(OperatorInputs *inputs, Drivetrain *drivetrain);
	~CurveAuto();
	void Init();
	void Loop();
	void Stop();
    void ConfigureProfiles();
    void ConfigureGyroPID();
    void ConfigureEncoderPID();
    bool IsFinished() { return m_finished; }

protected:
	OperatorInputs *m_inputs;
    Drivetrain *m_drivetrain;

    ProfiledPIDController<units::meters> *m_encoderPIDController;
    TrapezoidProfile<units::meters>::Constraints m_encoderconstraints;
    units::meter_t m_encodertolerance;
    double m_encoderPIDvals[3];

    ProfiledPIDController<units::meters> *m_gyroPIDController;
    TrapezoidProfile<units::meters>::Constraints m_gyroconstraints;
    units::degree_t m_gyrotolerance;
    double m_gyroPIDvals[3];

    units::meter_t m_setpoint;
    units::degree_t m_setpointangle;
    units::meter_t m_encoderprevgoal;
    units::degree_t m_gyroprevgoal;
    int m_turns;

    bool m_finished;

    AutoState m_autostate;
};


#endif /* SRC_CurveAuto_H_ */
