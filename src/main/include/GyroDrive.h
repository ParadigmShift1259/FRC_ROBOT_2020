/**
 *  GyroDrive.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_GyroDrive_H_
#define SRC_GyroDrive_H_


#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "DriveTrainFX.h"
#include "DrivePID.h"
#include "Gyro.h"
#include "Vision.h"


using namespace frc;


class GyroDrive
{
public:
	enum DriveState { kInit, kDrive };
	enum DriveMode { kManual };

	GyroDrive(OperatorInputs *inputs, Vision *vision);
	~GyroDrive();
	void Init();
	void Loop();
	void Stop();
	void Disabled();

	DualGyro *GetGyro() { return m_gyro; }

	void Drive(double x, double y, bool ramp = false);
	void SetStraightPID(double P = -1, double I = -1, double D = -1);
	void SetAnglePID(double P = -1, double I = -1, double D = -1);
	void GetAnglePID(double &P, double &I, double &D);
	bool DriveStraight(double targetdistance, double autopower, bool reset = true);
	bool DriveAngle(double angle, bool reset = true);
	void SetLowSpeed(bool enable) { m_drivetrain->SetLowSpeedMode(enable); }

protected:
	OperatorInputs *m_inputs;
	DriveTrainFX *m_drivetrain;
	DualGyro *m_gyro;
	DrivePID *m_drivepid;

	Timer m_timer;
	DriveMode m_drivemode;
	int m_stage;
	DriveState m_drivestate;
	double m_pidstraight[3];
	double m_pidangle[3];
	double m_distance;
	
};


#endif /* SRC_GyroDrive_H_ */
