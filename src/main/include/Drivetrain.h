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
    Drivetrain(OperatorInputs *inputs, 
               CANSparkMax *left1 = nullptr, CANSparkMax *left2 = nullptr, CANSparkMax *left3 = nullptr,
               CANSparkMax *right1 = nullptr, CANSparkMax *right2 = nullptr, CANSparkMax *right3 = nullptr);
    ~Drivetrain();
    void Init();
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

protected:
    void ExperimentalData();

private:
    OperatorInputs *m_inputs;

    CANSparkMax *m_left1;
    CANSparkMax *m_left2;
    CANSparkMax *m_left3;
    CANSparkMax *m_right1;
    CANSparkMax *m_right2;
    CANSparkMax *m_right3;

    CANEncoder *m_leftenc;
    CANEncoder *m_rightenc;

    DifferentialDrive *m_drive;

    bool m_inited;
    double m_ramprate;
};


#endif /* SRC_Drivetrain_H_ */