/**
 *  Turret.h
 *  Date: 2/20/2020 
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Turret_H_
#define SRC_Turret_H_


#include "OperatorInputs.h"
#include "Vision.h"
#include "Feeder.h"
#include "GyroDrive.h"

#include <rev\CANSparkMax.h>
#include <rev\CANEncoder.h>
#include <rev\CANPIDController.h>
#include <units\units.h>
#include <frc\controller\SimpleMotorFeedforward.h>

#include <ctre\Phoenix.h>

#include <frc\Servo.h>


using namespace frc;
using namespace rev;


class Turret
{
public:
    enum TurretState {kIdle, kRampUp, kVision, kAllReady};
    enum FireMode {kForceShoot, kShootWhenReady, kHoldShoot};
    enum RampState {kMaintain, kIncrease, kDecrease};

    Turret(OperatorInputs *inputs, Intake *intake, Feeder *feeder, Vision *vision, GyroDrive *gyrodrive);
    ~Turret();
    void Init();
    void Loop();
    void Stop();
    void Dashboard();

protected:
    bool NullCheck();

    void TurretStates();
    void FireModes();

    // Takes the distance from the target and returns the hood angle and flywheel speed
    void CalculateHoodFlywheel(double distance, double &hoodangle, double &flywheelspeed);
    // Takes the XBox joystick and converts it into an field angle
    void FindFieldXBox();
    // Calculates the turret angle based off of the field angle
    void CalculateTurretFromField();
    // Calculates the field angle needed based off of vision
    bool VisionFieldAngle();

    void RampUpFlywheel();
    void RampUpTurret();

    double TicksToDegrees(double ticks);
    double DegreesToTicks(double degrees);

private:
    OperatorInputs *m_inputs;
    Intake *m_intake;
    Feeder *m_feeder;
    Vision *m_vision;

    // Flywheel
    CANSparkMax *m_flywheelmotor;
    CANPIDController *m_flywheelPID;
    CANEncoder *m_flywheelencoder;

    double m_PIDslot;
    double m_flywheelsetpoint;
    double m_flywheelrampedsetpoint;

    SimpleMotorFeedforward<units::meters> *m_flywheelsimplemotorfeedforward;
    double m_flywheelinitialfeedforward;

    // Turret
    WPI_TalonSRX *m_turretmotor;

    double m_fieldangle;
    double m_robotangle;
    DualGyro *m_robotgyro;
    double m_turretangle;
    double m_turretrampedangle;

    bool m_intakeup;
    double m_turretinitialfeedforward;

    // Hood
    Servo *m_hoodservo;
    double m_hoodangle;

    TurretState m_turretstate;
    FireMode m_firemode;
    RampState m_flywheelrampstate;
    RampState m_turretrampstate;
    bool m_readytofire;
    bool m_firing;

    // Hood / Vision
    double m_distance;
};


#endif /* SRC_Turret_H_ */