/**
 *  Const.h
 *  Date: 6/25/19
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_CONST_H_
#define SRC_CONST_H_

extern bool Debug;                  // Set to true to enable additional debugging

class Logger;                       // Forward declare
extern Logger *g_log;               // Global logging object pointer

// OperatorInputs
//   Controllers
#define INP_DUAL 0
#define INP_JOYSTICK -1
#define INP_XBOX_1 0
#define INP_XBOX_2 1
//   Set to 1.0 or -1.0
#define INVERT_Y_AXIS 1.0
#define INVERT_X_AXIS -1.0
//   XBox Controller Buttons
#define A_BUTTON  1
#define B_BUTTON  2
#define X_BUTTON  3
#define Y_BUTTON  4
#define LEFT_BUMPER  5
#define RIGHT_BUMPER  6
#define BACK_BUTTON  7
#define START_BUTTON  8
#define L3_BUTTON 9
#define R3_BUTTON 10
//   XBox Triggers -- Changed for 2016, previously XBOX triggers were both on a single axis
#define XBOX_LEFT_TRIGGER_AXIS  12
#define XBOX_RIGHT_TRIGGER_AXIS  13
#define LEFT_TRIGGER_MIN  0.5
#define LEFT_TRIGGER_MAX  1.0
#define RIGHT_TRIGGER_MIN  0.5
#define RIGHT_TRIGGER_MAX  1.0
#define JOYSTICK_X_AXIS  0
#define JOYSTICK_Y_AXIS  1
#define AXIS0_LEFT_MIN -1.0
#define AXIS0_LEFT_MAX -0.75
#define AXIS0_RIGHT_MIN 0.75
#define AXIS0_RIGHT_MAX 1.0
#define AXIS1_BACK_MIN -1.0
#define AXIS1_BACK_MAX -0.75
#define AXIS1_FORWARD_MIN 0.75
#define AXIS1_FORWARD_MAX 1.0
//	 Controller Dead Zones
#define DEADZONE_Y  0.18
#define DEADZONE_X  0.18
#define DEADZONE_Z  0.18


// Drivetrain
#define INVERT_LEFT true
#define INVERT_RIGHT true
//   SparkMax Ports
//#define SIX_WHEEL_DRIVE false
#define CAN_LEFT_PORT_1 2       // Changed for test bot 2/9/19
#define CAN_LEFT_PORT_2 4
#define CAN_LEFT_PORT_3 6
#define CAN_RIGHT_PORT_1 1
#define CAN_RIGHT_PORT_2 3
#define CAN_RIGHT_PORT_3 5
//   NEO parameters
#define MOTOR_CURRENT_LIMIT 30
#define MOTOR_VOLTAGE_COMPENSATION 10
#define MOTOR_RAMP_RATE_TIME 5 // 1
// Xbox Joystick Scaling (Note: Xbox Y axis corresponds to X speed in Robot Y axis and Xbox X axis corresponds to turning) 
#define X_SCALING  0.35 // 0.5 
#define Y_SCALING 0.5 // 1.0

//   Encoders
#define ENC_PRESENT_1 true
#define ENC_TYPE_1 FeedbackDevice::QuadEncoder
#define ENC_PRESENT_2 false
#define ENC_TYPE_2 FeedbackDevice::QuadEncoder
#define CODES_PER_REV 1382.0
#define CODES_PER_INCH 73.317
#define WHEEL_DIAMETER 6.0
#define WHEEL_TRACK 23.50
#define NEO_CONVERSION (WHEEL_DIAMETER * 3.1415926535 / 5.2) // ????
#define METERS_ENCODER_CONVERSION (.02*NEO_CONVERSION) // emperically determined with above  NEO_CONVERSION 
// Gyro 
#define GRY_ENABLED 1 
#define CAN_GYRO1 0 // ENABLED(GRY_ENBALED, 0)
#define CAN_GYRO2 -1

//  Turret ---------------------------------------
#define TUR_ENABLED 1
#define TUR_SHOOTER_ID 1
#define TUR_TURRET_ID 2
#define TUR_HOOD_ID 0
#define TUR_TIMEOUT_MS 30
#define ENCODER_TICKS_PER_REV 4096.00
#define MINUTES_TO_HUNDRED_MS 0.00166666
// Shooter / Flywheel
#define TUR_SHOOTER_RAMPING_RATE 100       // in rpm
#define TUR_SHOOTER_IDLE_STATE_RPM 2000
#define TUR_SHOOTER_RAMPUP_STATE_RPM 2500
// kS, kV, kA values tuned using frc-characterization 2/20/20 for the Metal V2 Shooter Flywheel by Geoffrey
#define TUR_SHOOTER_KS 0.0876
#define TUR_SHOOTER_KV 0.124
#define TUR_SHOOTER_KA 0   // 0.0152, but probably 0
// Increasing/Decreasing PID values tuned manually 1/27/20 for the V1 Shooter Flywheel by Geoffrey
#define TUR_SHOOTER_P 0.000632
#define TUR_SHOOTER_I 0.0
#define TUR_SHOOTER_D 0.0002231
// Ball Recovery PID values tuned manually 1/29/20 for the V1 Shooter Flywheel by Geoffrey
#define TUR_SHOOTER_MP 0.00136  // doubled 2/24/20 for testing
#define TUR_SHOOTER_MI 0.0
#define TUR_SHOOTER_MD 0.008110
// Maximum percent output that PID loop can output
#define TUR_SHOOTER_MINOUT 0.0
#define TUR_SHOOTER_MAXOUT 1.0
#define TUR_SHOOTER_ERROR 100
#define TUR_SHOOTER_MAX_ERROR 100
// Turret Spinning
#define TUR_TURRET_RAMPING_RATE 7           // in degrees
// kS, kV, kA values turned using frc-characterization 2/8/20 for the Metal V2 Shooter Flywheel by Geoffrey
#define TUR_TURRET_KS_FORWARDS 0.0
#define TUR_TURRET_KS_BACKWARDS 0.0
// Turret turning PID values
#define TUR_TURRET_P 0.13114
#define TUR_TURRET_I 0.0002
#define TUR_TURRET_D 0.0975
// Maximum percent output that PID loop can output
#define TUR_TURRET_MINOUT 0.0
#define TUR_TURRET_MAXOUT 0.2
// Allowable error before shooting
#define TUR_TURRET_ERROR 0.5
#define TUR_TURRET_MAX_ERROR 5
// Miscellaneous Values for Turret
#define TUR_WHEEL_DIAMETER 0.1524    // in meters
#define TUR_ROTATIONS_TO_METERS (3.1415926535 * TUR_WHEEL_DIAMETER)
#define TUR_MINUTES_TO_SECONDS 1/60
#define TUR_PULLEY 2.7305
#define TUR_TURRET_SPINNER 29.845
#define REV_TO_TURRET_REV (TUR_PULLEY / TUR_TURRET_SPINNER) 
#define TURRET_REV_TO_DEGREES 360
#define TURRET_DEGREE_STOP_RANGE 0.25

#endif /* SRC_CONST_H_ */