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
//   Ramping
#define RAMPING_RATE_PERIOD 0.10
#define RAMPING_RATE_UP_MIN 0.6
#define RAMPING_RATE_UP_MAX 4.0
#define RAMPING_RATE_DOWN_MIN 0.6
#define RAMPING_RATE_DOWN_MAX 4.0
//   Scaling
#define JOYSTICK_SCALING_X 0.5
#define JOYSTICK_SCALING_Y 1.0
#define MOTOR_SCALING_LEFT 1.0
#define MOTOR_SCALING_RIGHT 1.0
#define LOWSPEED_MODIFIER_X 0.25
#define LOWSPEED_MODIFIER_Y 0.25
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
#define PCM_ENABLED 1            // set to 1 to enable compressor
#define PCM_COMPRESSOR_SOLENOID ENABLED(PCM_ENABLED, 0)
#define CAN_POWER_DISTRIBUTION_PANEL 0
#define PNE_CURRENT_DRAW 80
#define PNE_VOLTAGE_DROP 10
#define PNE_WAITTIME 1.0


// Gyro
#define GRY_ENABLED 1            // set to 1 to enable gyro
#define CAN_GYRO1 ENABLED(GRY_ENABLED, 0)
#define CAN_GYRO2 -1


//  TalonSRX Constants
#define TIMEOUT_MS 30
#define ENCODER_TICKS_PER_REV 4096.00
#define MINUTES_TO_HUNDRED_MS 0.00166666


// Autonomous ---------------------------------------
//   PID Constants
#define AUT_P 0.05              // 0.1
#define AUT_I 0.0001            // 0.0003
#define AUT_D 0.11              // 0.11


// ControlPanel ---------------------------------------
#define CPL_ENABLED 1           // set to 1 to enable control panel motor
#define CPL_MOTOR ENABLED(CPL_ENABLED, 11)
#define CPL_SOLENOID ENABLED(CPL_ENABLED, 1)
// ControlPanel
#define CPL_COUNTS_PER_CW_REV (4096 * 32 / 4)
#define CPL_COUNTS_PER_CW_SECTOR (CPL_COUNTS_PER_CW_REV / 8)
#define CPL_ONE_CW_RPM (CPL_COUNTS_PER_CW_REV / 600) /* 100ms Timebase for TalonSRX PID */

#define CPL_YELLOW_MINIMUM_HUE 80
#define CPL_YELLOW_MAXIMUM_HUE 100
#define CPL_RED_MINIMUM_HUE 20
#define CPL_RED_MAXIMUM_HUE 45
#define CPL_GREEN_MINIMUM_HUE 120
#define CPL_GREEN_MAXIMUM_HUE 150
#define CPL_BLUE_MINIMUM_HUE 180
#define CPL_BLUE_MAXIMUM_HUE 210

#define CPL_ROTATION_CONTROL_COUNT_LIMIT_RED 7
#define CPL_ROTATION_CONTROL_COUNT_LIMIT_BLUE 7

#define CPL_ROTATION_CONTROL_FAST (55 * CPL_ONE_CW_RPM)
#define CPL_POSITION_CONTROL_FAST (15 * CPL_ONE_CW_RPM)
#define CPL_POSITION_CONTROL_SLOW (5 * CPL_ONE_CW_RPM)

#define CPL_P 0.3
#define CPL_I 0.00002
#define CPL_F 0.085

// Intake ---------------------------------------
#define INT_ENABLED 1           // set to 1 to enable intake motors
#define INT_SOLENOID ENABLED(INT_ENABLED, 0)
#define INT_MOTOR1 ENABLED(INT_ENABLED, 7)
#define INT_MOTOR2 ENABLED(INT_ENABLED, 8)
#define INT_INTAKE_WHEEL_SPEED 0.5
#define INT_INTAKE_ROLLER_SPEED .6


// Feeder ---------------------------------------
#define FDR_ENABLED 1           // set to 1 to enable feeder motor
#define FDR_MOTOR ENABLED(FDR_ENABLED, 9)
#define FDR_REFRESH_SPEED_LOAD 0.2
#define FDR_REFRESH_SPEED_FIRE 0.5
#define FDR_HIGH_NUMBER 10000
#define FDR_WHEEL_SIZE 4
#define FDR_FEED_FORWARD 0.0681
// PID values for ball
#define FDR_P 
#define FDR_I 
#define FDR_D 
// Refresh distance in inches
#define FDR_REFRESH_DISTANCE 26.531
// Max percent output that feeder can apply
#define FDR_MAX_POWER 0.3
#define FDR_TIMEOUT_TIME 6
#define FDR_DRIVE_TIME 6
#define FDR_GEAR_RATIO 4.167
#define FDR_INVERTED 1.0

// Climber ---------------------------------------
#define CLM_ENABLED 0           // set to 1 to enable intake motor
#define CLM_MOTOR ENABLED(CLM_ENABLED, 10)

//  Turret ---------------------------------------
#define TUR_ENABLED 1
#define TUR_SHOOTER_ID ENABLED(TUR_ENABLED, 1)
#define TUR_TURRET_ID ENABLED(TUR_ENABLED, 7)
#define TUR_TIMEOUT_MS 30
#define ENCODER_TICKS_PER_REV 4096.00
#define MINUTES_TO_HUNDRED_MS 0.00166666
// Shooter / Flywheel
#define TUR_SHOOTER_RAMPING_RATE 100       // in rpm
#define TUR_SHOOTER_IDLE_STATE_RPM 2000
#define TUR_SHOOTER_PREMOVE_STATE_RPM 3200
// kS, kV, kA values tuned using frc-characterization 2/1/20 for the Metal V2 Shooter Flywheel by Geoffrey
#define TUR_SHOOTER_KS 0.0868
#define TUR_SHOOTER_KV 0.126    // was actually 0.128, but we'll see
#define TUR_SHOOTER_KA 0   // 0.0152, but probably 0
// Increasing/Decreasing PID values tuned manually 1/27/20 for the V1 Shooter Flywheel by Geoffrey
#define TUR_SHOOTER_P 0.000632
#define TUR_SHOOTER_I 0.0
#define TUR_SHOOTER_D 0.0002231
// Ball Recovery PID values tuned manually 1/29/20 for the V1 Shooter Flywheel by Geoffrey
#define TUR_SHOOTER_MP 0.00068
#define TUR_SHOOTER_MI 0.0
#define TUR_SHOOTER_MD 0.008110
// Maximum percent output that PID loop can output
#define TUR_SHOOTER_MINOUT 0.0
#define TUR_SHOOTER_MAXOUT 1.0
// Turret Spinning
#define TUR_TURRET_RAMPING_RATE 5           // in degrees
// kS, kV, kA values turned using frc-characterization 2/8/20 for the Metal V2 Shooter Flywheel by Geoffrey
#define TUR_TURRET_KS_FORWARDS 0.71
#define TUR_TURRET_KS_BACKWARDS 0.81
// Turret turning PID values
#define TUR_TURRET_P 0.13114
#define TUR_TURRET_I 0
#define TUR_TURRET_D 0.0975
// Maximum percent output that PID loop can output
#define TUR_TURRET_MINOUT 0.0
#define TUR_TURRET_MAXOUT 0.25
// Allowable error before shooting
#define TUR_TURRET_ERROR 1
#define TUR_TURRET_MAX_ERROR 5
// Miscellaneous Values for Turret
#define TUR_WHEEL_DIAMETER 0.1524    // in meters
#define TUR_ROTATIONS_TO_METERS (3.1415926535 * TUR_WHEEL_DIAMETER)
#define TUR_MINUTES_TO_SECONDS 1/60
#define TUR_PULLEY 2.7305
#define TUR_TURRET_SPINNER 29.845
#define REV_TO_TURRET_REV (TUR_PULLEY / TUR_TURRET_SPINNER) 
#define TURRET_REV_TO_DEGREES 360
#define TURRET_DEGREE_STOP_RANGE 1

// Vision ---------------------------------------
#define VIS_MOUNTING_ANGLE 20.6     // degrees
#define VIS_MOUNTING_HEIGHT 19.5      // inches
#define VIS_TARGET_HEIGHT 98.25     // inches
#define VIS_TARGET_SIZE 15          // inches

#endif /* SRC_CONST_H_ */
