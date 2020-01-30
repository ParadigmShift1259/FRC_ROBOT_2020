/**
 *  Const.h
 *  Date: 6/25/19
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_CONST_H_
#define SRC_CONST_H_


extern bool Debug;                  // Set to true to enable additional debugging



// OperatorInputs
//   Controllers
#define INP_DUAL 1
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
#define INVERT_LEFT false
#define INVERT_RIGHT false
//   SparkMax Ports
#define SIX_WHEEL_DRIVE false
#define CAN_LEFT_PORT_1 2       // Changed for test bot 2/9/19
#define CAN_LEFT_PORT_2 4
#define CAN_LEFT_PORT_3 -2
#define CAN_RIGHT_PORT_1 1
#define CAN_RIGHT_PORT_2 3
#define CAN_RIGHT_PORT_3 -3
//   NEO parameters
#define MOTOR_CURRENT_LIMIT 30
#define MOTOR_VOLTAGE_COMPENSATION 10
#define MOTOR_RAMP_RATE_TIME 1
// Scaling
#define X_SCALING 0.5
#define Y_SCALING 1.0

//   Encoders
#define ENC_PRESENT_1 true
#define ENC_TYPE_1 FeedbackDevice::QuadEncoder
#define ENC_PRESENT_2 false
#define ENC_TYPE_2 FeedbackDevice::QuadEncoder
#define CODES_PER_REV 1382.0
#define CODES_PER_INCH 73.317
#define WHEEL_DIAMETER 6.0
#define WHEEL_TRACK 23.50
#define NEO_CONVERSION (WHEEL_DIAMETER * 3.1415926535 / 5.2)

//  Turret
#define TUR_TIMEOUT_MS 30
#define ENCODER_TICKS_PER_REV 4096.00
#define MINUTES_TO_HUNDRED_MS 0.00166666
#define TUR_RAMPING_RATE 100
// kS, kV, kA values tuned using frc-characterization 1/28/20 for the V1 Shooter Flywheel by Geoffrey
// These values are only for going forwards
#define TUR_KS 0.3745
#define TUR_KV 0.126
#define TUR_KA 0   // 0.0152, but probably 0
/**
 * Values going backwards:
 * kS = 0.321
 * kV = 0.131
 * kA = 0.0176
 */

#define TUR_WHEEL_DIAMETER 0.1524    // in meters
#define TUR_ROTATIONS_TO_METERS 3.1415926535 * TUR_WHEEL_DIAMETER
#define TUR_MINUTES_TO_SECONDS 1/60



#endif /* SRC_CONST_H_ */