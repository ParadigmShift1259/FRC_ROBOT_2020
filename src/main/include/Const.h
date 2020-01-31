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
//   Joystick
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


//   Enabled macro
//   First parameter is an enabled flag 0 or 1
//   Second parameter is the value to return if enabled
#define ENABLED(a, b) ((a) == 0 ? (-1) : (b))


//   Drivetrain
#define DT_ENABLED 0            // set to 1 to enable drivetrain
//   Direction
#define DT_DEFAULT_DIRECTION -1.0
//   Inverts
#define INVERT_LEFT true
#define INVERT_RIGHT false
//   CAN Ports
#define CAN_LEFT_PORT_1 ENABLED(DT_ENABLED, 1)
#define CAN_LEFT_PORT_2 ENABLED(DT_ENABLED, 2)
#define CAN_LEFT_PORT_3 ENABLED(DT_ENABLED, 3)
#define CAN_RIGHT_PORT_1 ENABLED(DT_ENABLED, 4)
#define CAN_RIGHT_PORT_2 ENABLED(DT_ENABLED, 5)
#define CAN_RIGHT_PORT_3 ENABLED(DT_ENABLED, 6)
//   Current limiting
#define MOTOR_SUPPLY_LIMIT_ENABLE true
#define MOTOR_SUPPLY_CURRENT_LIMIT 20
#define MOTOR_SUPPLY_THRESHOLD_CURRENT 0
#define MOTOR_SUPPLY_THRESHOLD_TIME 0.1


//   Encoders
#define ENC_PRESENT_1 true
#define ENC_TYPE_1 FeedbackDevice::IntegratedSensor
#define ENC_PRESENT_2 false
#define ENC_TYPE_2 FeedbackDevice::IntegratedSensor
// All units will be in meters and seconds unless otherwise specified
#define WHEEL_DIAMETER 0.1524
#define TICKS_PER_METER 2048 * WHEEL_DIAMETER * 3.1415926535

// Gyro
#define GRY_ENABLED 0            // set to 1 to enable gyro
#define CAN_GYRO1 ENABLED(GRY_ENABLED, 0)
#define CAN_GYRO2 -1

#define HIGH_NUMBER 1000000

#endif /* SRC_CONST_H_ */
