/**
 *  Turret.h
 *  Date: 1/7/2020 
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Turret_H_
#define SRC_Turret_H_


#include <frc\WPILib.h>
#include <rev\CANSparkMax.h>

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
     * Fire -> Maintains Turret Hood, and Flywheel and requests ball from intake
     * Return -> rotates back to Idle position
     */
    enum TurretState {kIdle, kHoming, kPreMove, kReady, kFire, kReturn};
    /**
     * Modes
     * FireWhenReady -> When turret achieves ready state, fire immediately
     * HoldFire -> Holds fire until both ready and human interaction
     * ManualFire -> fires with only human interaction, no matter ready or not
     * Off -> Nothing will activate shooter
     */
    enum FireMode {kHoldFire, kFireWhenReady, kManualFire, kOff};
    
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
    
    //PigeonIMU *m_pigeon;
    //double m_heading;

    CANSparkMax *m_flywheelmotor;
    CANPIDController *m_flywheelPID;
    CANEncoder *m_flywheelencoder;
    // P, I, D, FF, Iz, nominal, peak
    double m_flywheelPIDvals[7];
    double m_flywheelsetpoint;
    double m_flywheelrampedsetpoint;
    bool m_flywheelsetpointreached;

    //TalonSRX *m_hoodmotor;
    //TalonSRX *m_turretmotor;
    //PIDController *m_hoodPID;
    //PIDController *m_turretPID;
    //double m_hoodPIDvals[3];
    //double m_turretPIDvals[3];
    TurretState m_turretstate;
    FireMode m_firemode;
};


#endif /* SRC_Turret_H_ */