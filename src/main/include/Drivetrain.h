/**
 *  Drivetrain.h
 *  Date: 6/25/19
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Drivetrain_H_
#define SRC_Drivetrain_H_


#include <frc\WPILib.h>
#include <rev\CANSparkMax.h>
#include "OperatorInputs.h"

using namespace frc;
using namespace rev;


class Drivetrain
{
public:
    // Drivetrain motors
    enum DriveMotors { k2Motors, k4Motors, k6Motors };

	// Drivetrain modes
	enum DriveMode { kNone, kFollower, kDiscrete, kTank, kArcade, kCurvature };

    Drivetrain(OperatorInputs *inputs, 
               CANSparkMax *left1 = nullptr, CANSparkMax *left2 = nullptr, CANSparkMax *left3 = nullptr,
               CANSparkMax *right1 = nullptr, CANSparkMax *right2 = nullptr, CANSparkMax *right3 = nullptr);
    ~Drivetrain();
    void Init(DriveMotors motors = k4Motors, DriveMode mode = kFollower);
    void Loop();
    void Stop();

    // init settings
    void SetRampRate(double rate);
    double GetRampRate() { return m_ramprate; }    
    void SetInvert(bool left, bool right);
    void SetBrakeMode();
    void SetCoastMode();
    void SetVoltageCompensation(double voltage);
    void SetCurrentLimit(double current);
    units::meter_t getLeftDist();
    units::meter_t getRightDist();

protected:
    void ExperimentalData();

protected:
    OperatorInputs *m_inputs;
    DriveMotors m_motors;
    DriveMode m_mode;

    CANSparkMax *m_left1;
    CANSparkMax *m_left2;
    CANSparkMax *m_left3;
    CANSparkMax *m_right1;
    CANSparkMax *m_right2;
    CANSparkMax *m_right3;
	bool m_left1owner;
	bool m_left2owner;
	bool m_left3owner;
	bool m_right1owner;
	bool m_right2owner;
	bool m_right3owner;

    CANEncoder *m_leftenc;
    CANEncoder *m_rightenc;

    DifferentialDrive *m_drive;

    bool m_inited;
    double m_ramprate;
    double m_rightoffset;
    double m_leftoffset;
};


#endif /* SRC_Drivetrain_H_ */