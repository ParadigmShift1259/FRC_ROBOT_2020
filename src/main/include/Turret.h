/**
 *  Turret.h
 *  Date: 1/7/2020 
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Turret_H_
#define SRC_Turret_H_


#include "frc/smartdashboard/SmartDashboard.h"
#include <rev\CANSparkMax.h>
#include <rev\CANEncoder.h>
#include <rev\CANPIDController.h>
#include <frc\controller\SimpleMotorFeedforward.h>
#include <units/units.h>

#include "OperatorInputs.h"


using namespace frc;
using namespace rev;


class Turret
{
public:
    /**
     * States
     * Idle -> No movement, flywheel at constant speed, at ideal position
     * PreMove -> Manual rotation of turret to lock the camera on target
     * Homing -> Continuous tracking and movement, flywheel accelerates to needed speed
     * Ready -> Turret and Hood positioned correctly, flywheel maintained at speed
     * Return -> rotates back to Idle position
     * This state machine will adjust the speed and angle of the shooter
     */
    enum TurretState {kIdle, kHoming, kPreMove, kReady, kReturn};
    /**
     * Modes
     * FireWhenReady -> When turret achieves ready state, fire immediately
     * HoldFire -> Holds fire until both ready and human interaction
     * ManualFire -> fires with only human interaction, no matter ready or not
     * Off -> Nothing will activate shooter
     * This state machine will call for balls from the intake system
     */
    enum FireMode {kHoldFire, kFireWhenReady, kManualFire, kOff};
    
    enum RampState {kMaintain, kIncrease, kDecrease};

    // Will add Drivetrain, Intake, PID, and Vision classes as pass pointers
    Turret(OperatorInputs *inputs);
    ~Turret();
    void Init();
    void Loop();
    void Stop();

protected:
    void TurretStates();
    void FireModes();
    // Takes the distance from the target and returns the hood angle and flywheel speed
    void CalculateHoodFlywheel(double distance, double &hoodangle, double &flywheelspeed);
    void RampUpSetpoint();

private:
    OperatorInputs *m_inputs;

    CANSparkMax *m_flywheelmotor;
    CANPIDController *m_flywheelPID;
    CANEncoder *m_flywheelencoder;

    double m_PIDslot;
    double m_flywheelsetpoint;
    double m_flywheelrampedsetpoint;

    SimpleMotorFeedforward<units::meters> *m_simplemotorfeedforward;
    double m_initialfeedforward;

    bool m_readytofire;

    TurretState m_turretstate;
    FireMode m_firemode;
    RampState m_rampstate;
};


#endif /* SRC_Turret_H_ */