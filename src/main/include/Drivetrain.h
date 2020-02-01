/**
 *  DriveTrain.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DriveTrain_H_
#define SRC_DriveTrain_H_


#include <frc\WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"


using namespace frc;


class DriveTrain
{
public:
	// Drivetrain modes
	enum DriveMode { kNone, kFollower, kDiscrete, kTank, kArcade, kCurvature };

	DriveTrain(OperatorInputs *inputs, WPI_TalonSRX *left1 = nullptr, WPI_TalonSRX *left2 = nullptr, WPI_TalonSRX *left3 = nullptr, WPI_TalonSRX *right1 = nullptr, WPI_TalonSRX *right2 = nullptr, WPI_TalonSRX *right3 = nullptr);
	~DriveTrain();
	void Init(DriveMode mode = kFollower);
	void Loop();
	void Stop();
	void Drive(double x, double y, bool ramp = false, bool tank = false);
	void Shift();
		// change DriveTrain direction and return true if going forward
	bool ChangeDirection();
	bool ChangeLowSpeedMode();
	double Ramp(double previousPow, double desiredPow, double rampSpeedMin, double rampSpeedMax);
	double LeftMotor(double &invMaxValueXPlusY);
	double RightMotor(double &invMaxValueXPlusY);

	void SetChangeDirButton(int button) {m_changedirbutton = button;}

	void setCoasting(double newCoasting) {m_coasting = newCoasting;}
	void setRamp(double newValue) {m_rampmax = newValue;}
	bool getIsHighGear() {return m_ishighgear;}

	double GetLeftPosition(int encoder = 0);
	double GetRightPosition(int encoder = 0);
	double GetLeftVelocity(int encoder = 0);
	double GetRightVelocity(int encoder = 0);
	double GetMaxVelocity(int encoder = 0);
	double GetLeftDistance(int encoder = 0);
	double GetRightDistance(int encoder = 0);
	double GetMaxDistance(int encoder = 0);
	double GetAverageMaxDistance(int encoder = 0);
	void ResetDeltaDistance(int encoder = 0);
	double GetMaxDeltaDistance(int encoder = 0);

	WPI_TalonSRX *Left1() {return m_left1;}
	WPI_TalonSRX *Right1() {return m_right1;}
	WPI_TalonSRX *Left2() {return m_left2;}
	WPI_TalonSRX *Right2() {return m_right2;}
	WPI_TalonSRX *Left3() {return m_left3;}
	WPI_TalonSRX *Right3() {return m_right3;}

protected:
	DriveMode m_mode;
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_left1;
	WPI_TalonSRX *m_left2;
	WPI_TalonSRX *m_left3;
	WPI_TalonSRX *m_right1;
	WPI_TalonSRX *m_right2;
	WPI_TalonSRX *m_right3;
	bool m_left1owner;
	bool m_left2owner;
	bool m_left3owner;
	bool m_right1owner;
	bool m_right2owner;
	bool m_right3owner;
	SpeedControllerGroup *m_leftscgroup;
	SpeedControllerGroup *m_rightscgroup;
	DifferentialDrive *m_differentialdrive;
	Timer *m_timerramp;

	int m_changedirbutton;

	double m_leftpow;
	double m_rightpow;
	double m_leftspeed;
	double m_rightspeed;
	double m_leftposition;
	double m_rightposition;
	double m_coasting;
	double m_rampmax;

	double m_prevleftdistance;
	double m_prevrightdistance;

	double m_invertleft;
	double m_invertright;
	double m_direction;

	bool m_ishighgear;
	double m_previousx;
	double m_previousy;
	bool m_isdownshifting;
	bool m_lowspeedmode;
	bool m_shift;
};


#endif /* SRC_DriveTrain_H_ */
