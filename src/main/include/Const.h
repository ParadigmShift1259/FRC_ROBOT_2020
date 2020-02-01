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
#define DT_ENABLED 1            // set to 1 to enable drivetrain
//   Direction
#define DT_DEFAULT_DIRECTION -1.0
//   Inverts
#define INVERT_LEFT 1.0
#define INVERT_RIGHT -1.0		// 2017 code is 1, WPILlib DifferentialDrive is -1 (adjusted in DriveTrain::Drive())
//   CAN Ports
#define CAN_LEFT_PORT_1 ENABLED(DT_ENABLED, 1)
#define CAN_LEFT_PORT_2 ENABLED(DT_ENABLED, 3)
#define CAN_LEFT_PORT_3 ENABLED(0, 5)
#define CAN_RIGHT_PORT_1 ENABLED(DT_ENABLED, 2)
#define CAN_RIGHT_PORT_2 ENABLED(DT_ENABLED, 4)
#define CAN_RIGHT_PORT_3 ENABLED(0, 6)
//   Current limiting
#define MOTOR_SUPPLY_LIMIT_ENABLE true
#define MOTOR_CURRENT_LIMIT 20
#define MOTOR_SUPPLY_THRESHOLD_CURRENT 0
#define MOTOR_SUPPLY_THRESHOLD_TIME 0.1
#define CHILD_PROOF_SPEED 0.75

//   Ramping
#define RAMPING_RATE_PERIOD 0.10
#define RAMPING_RATE_MIN 0.6
#define RAMPING_RATE_MAX 4.0            // 4.0
#define RAMPING_RATE_DOWN_MIN 3.0
#define RAMPING_RATE_DOWN_MAX 3.0
//   Scaling
#define X_SCALING 0.75
#define Y_SCALING 1.0
#define LEFT_MOTOR_SCALING 1
#define RIGHT_MOTOR_SCALING 1
#define LOWSPEED_MODIFIER_X 0.50
#define LOWSPEED_MODIFIER_Y 0.50
//   Encoders
#define ENC_PRESENT_1 true
#define ENC_TYPE_1 FeedbackDevice::IntegratedSensor
#define ENC_PRESENT_2 false
#define ENC_TYPE_2 FeedbackDevice::IntegratedSensor
#define CODES_PER_REV 1382.0
#define CODES_PER_INCH 73.317
#define WHEEL_DIAMETER 6.0
#define WHEEL_TRACK 23.50


//   Compressor
#define PCM_ENABLED 0            // set to 1 to enable compressor
#define PCM_COMPRESSOR_SOLENOID ENABLED(PCM_ENABLED, 0)
#define CAN_POWER_DISTRIBUTION_PANEL 0
#define PNE_CURRENT_DRAW 80
#define PNE_VOLTAGE_DROP 10
#define PNE_WAITTIME 1.0


// Gyro
#define GRY_ENABLED 0            // set to 1 to enable gyro
#define CAN_GYRO1 ENABLED(GRY_ENABLED, 0)
#define CAN_GYRO2 -1


//  TalonSRX Constants
#define TIMEOUT_MS 30
#define ENCODER_TICKS_PER_REV 4096.00
#define MINUTES_TO_HUNDRED_MS 0.00166666


// Autonomous
//   PID Constants
#define AUT_P 0.05              // 0.1
#define AUT_I 0.0001            // 0.0003
#define AUT_D 0.11              // 0.11


// ControlPanel
#define CPL_ENABLED 0           // set to 1 to enable control panel motor
#define CPL_MOTOR ENABLED(CPL_ENABLED, 11)


// Intake
#define INT_ENABLED 0           // set to 1 to enable intake motors
#define INT_SOLENOID1 ENABLED(INT_ENABLED, 0)
#define INT_SOLENOID2 ENABLED(INT_ENABLED, 1)
#define INT_MOTOR1 ENABLED(INT_ENABLED, 7)
#define INT_MOTOR2 ENABLED(INT_ENABLED, 8)
#define INT_SENSOR1 ENABLED(INT_ENABLED, 1)
#define INT_SENSOR2 ENABLED(INT_ENABLED, 2)
#define INT_SENSOR3 ENABLED(INT_ENABLED, 3)


// Feeder
#define FDR_ENABLED 0           // set to 1 to enable feeder motor
#define FDR_MOTOR ENABLED(FDR_ENABLED, 9)


// Feeder
#define FDR_ENABLED 0           // set to 1 to enable intake motor
#define FDR_MOTOR ENABLED(FDR_ENABLED, 9)
#define FDR_SENSOR ENABLED(FDR_ENABLED, 0)


// Climber
#define CLM_ENABLED 0           // set to 1 to enable intake motor
#define CLM_MOTOR ENABLED(CLM_ENABLED, 10)


#endif /* SRC_CONST_H_ */
