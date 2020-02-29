/**
 *  CurveAuto.h
 *  Date: 2/3/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_CurveAuto_H_
#define SRC_CurveAuto_H_

#include "OperatorInputs.h"
#include "DriveTrainFX.h"
#include "Gyro.h"

#include <units/units.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/SimpleMotorFeedforward.h>


using namespace std;
using namespace frc;


class CurveAuto
{
public:

    enum AutoState {kIdle, kDrive};

	CurveAuto(DriveTrainFX *drivetrain, DualGyro *gyro);
	~CurveAuto();
	void Init();
	void Loop();
	void Stop();
    void ConfigureProfiles();
    void ConfigureGyroPID();
    void ConfigureEncoderPID();
    void StartMotion(double distance, double angle, double targetvelocity, double maxvelocity, double maxacceleration);
    bool IsFinished() { return m_finished; }
    bool HasStarted() { return m_start; }

protected:
    DriveTrainFX *m_drivetrain;
    DualGyro *m_gyro;

    SimpleMotorFeedforward<units::meters> *m_encoderfeedforward;
    ProfiledPIDController<units::meters> *m_encoderPIDController;
    TrapezoidProfile<units::meters>::Constraints m_encoderconstraints;
    units::meter_t m_encodertolerance;
    double m_encoderPIDvals[3];
    units::meters_per_second_t m_prevvelocity;

    ProfiledPIDController<units::meters> *m_gyroPIDController;
    TrapezoidProfile<units::meters>::Constraints m_gyroconstraints;
    units::degree_t m_gyrotolerance;
    double m_gyroPIDvals[3];

    units::inch_t m_setpoint;
    units::degree_t m_setpointangle;

    bool m_finished;
    bool m_start;

    AutoState m_autostate;
};


#endif /* SRC_CurveAuto_H_ */