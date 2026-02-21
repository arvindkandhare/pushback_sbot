/**
 * config_sbot.h
 *
 * Hardware configuration and control constants for the sbot robot.
 * All ports are defined with placeholder values so wiring can be
 * updated later without touching logic code.
 */

#ifndef _SBOT_CONFIG_H_
#define _SBOT_CONFIG_H_

#include "api.h"

// ============================================================================
// SMART MOTOR PORTS (real wiring)
// ============================================================================

// 6-motor drivetrain (3 per side)
// Left drive train: 13 / 14 / 15
#define SBOT_LEFT_FRONT_MOTOR_PORT     13
#define SBOT_LEFT_MIDDLE_MOTOR_PORT    14
#define SBOT_LEFT_BACK_MOTOR_PORT      15

// Right drive train: 17 / 19 / 20
#define SBOT_RIGHT_FRONT_MOTOR_PORT    17
#define SBOT_RIGHT_MIDDLE_MOTOR_PORT   19
#define SBOT_RIGHT_BACK_MOTOR_PORT     20

// Intake / scoring system
// Indexer: 2
// Lower intake: 3
// Upper intake: 21
#define SBOT_INDEXER_MOTOR_PORT        2
#define SBOT_INTAKE_HELPER_MOTOR_PORT  3   // lower intake
#define SBOT_INTAKE_MAIN_MOTOR_PORT    21  // upper intake

// Intake motor direction correction.
// If one of the intake motors spins opposite of the other during intake,
// flip the corresponding *_REVERSED flag.
#define SBOT_INTAKE_MAIN_MOTOR_REVERSED   false
#define SBOT_INTAKE_HELPER_MOTOR_REVERSED true

// Indexer motor direction correction.
// If FEED_FORWARD runs the wrong way, flip this flag.
#define SBOT_INDEXER_MOTOR_REVERSED       true

// ============================================================================
// SENSORS (real wiring)
// ============================================================================

// Inertial sensor ("gyro")
#define SBOT_INERTIAL_PORT             6

// Odometry pod rotation sensor (1D forward/backwards tracking wheel)
#define SBOT_ODOM_ROTATION_PORT        16

// Optical color sensor (V5 Optical)
#define SBOT_COLOR_SENSOR_PORT         4

// ============================================================================
// PNEUMATICS (ADI three-wire ports, placeholders)
// ============================================================================

// Scoring flap + descore arm piston (port A); default retracted/closed
#define SBOT_GOAL_FLAP_PISTON_PORT     'A'

// Match loader piston (port C); default retracted
#define SBOT_BATCH_LOADER_PISTON_PORT  'C'

// Pneumatic logical states (may be inverted once wired)
#define SBOT_PISTON_EXTENDED_STATE     true
#define SBOT_PISTON_RETRACTED_STATE    false

// ============================================================================
// CONTROLLER MAPPING
// ============================================================================

// Tank drive sticks
#define SBOT_TANK_LEFT_STICK   pros::E_CONTROLLER_ANALOG_LEFT_Y
#define SBOT_TANK_RIGHT_STICK  pros::E_CONTROLLER_ANALOG_RIGHT_Y

// Ball handling / scoring buttons (can be remapped later)
#define SBOT_COLLECT_BUTTON        pros::E_CONTROLLER_DIGITAL_R1   // Intake+helper+indexer forward
#define SBOT_TOP_GOAL_BUTTON       pros::E_CONTROLLER_DIGITAL_R2   // Same as collect + open flap
#define SBOT_MID_GOAL_BUTTON       pros::E_CONTROLLER_DIGITAL_A    // Timed middle goal drop (moved from L1)
#define SBOT_LOW_GOAL_BUTTON       pros::E_CONTROLLER_DIGITAL_L2   // Timed low goal spit out

// Reverse intake button (action button on top of joystick)
#define SBOT_REVERSE_INTAKE_BTN    pros::E_CONTROLLER_DIGITAL_X    // Manual reverse intake

// Color sorting and alliance color selection
#define SBOT_COLOR_SORT_TOGGLE_BTN pros::E_CONTROLLER_DIGITAL_Y    // Enable/disable auto rejection
//#define SBOT_SET_RED_ALLIANCE_BTN  pros::E_CONTROLLER_DIGITAL_UP
//#define SBOT_SET_BLUE_ALLIANCE_BTN pros::E_CONTROLLER_DIGITAL_DOWN
#define SBOT_SLOW_MODE_BTN         pros::E_CONTROLLER_DIGITAL_UP


// Pneumatic toggles (driver control)
#define SBOT_GOAL_FLAP_TOGGLE_BTN  pros::E_CONTROLLER_DIGITAL_L1   // Toggle scoring flap + descore arm (moved from A)
#define SBOT_BATCH_LOADER_TOGGLE_BTN pros::E_CONTROLLER_DIGITAL_B  // Toggle match loader piston

// Autonomous selector navigation (using D-pad + A)
#define SBOT_AUTO_NEXT_BTN         pros::E_CONTROLLER_DIGITAL_RIGHT
#define SBOT_AUTO_PREV_BTN         pros::E_CONTROLLER_DIGITAL_LEFT
#define SBOT_AUTO_CONFIRM_BTN      pros::E_CONTROLLER_DIGITAL_A

// ============================================================================
// DRIVE / MOTOR CONSTANTS
// ============================================================================

// Deadband and sensitivity for tank drive
#define SBOT_JOYSTICK_DEADZONE     10
#define SBOT_TANK_SENSITIVITY      0.8

// ========== RESPONSE CURVE CONTROL ==========
// Squared curve gives fine control at low speeds but can be aggressive
// Set to false to use linear response (simpler, might help with tipping)
#define SBOT_USE_SQUARED_CURVE     true

// If using squared curve, this scales down the output to make it less aggressive
// 1.0 = full squared curve, 0.5 = halfway between linear and squared
// Only used if SBOT_USE_SQUARED_CURVE is true
#define SBOT_CURVE_SCALING         1.0

// ========== ADAPTIVE SLEW RATE LIMITING ==========
// Prevents tipping by limiting how fast motor commands can change.
// Uses different rates for normal acceleration vs direction reversals.

// Normal slew rate (same direction, e.g., 50→100)
// Higher = more responsive. Lower = smoother.
#define SBOT_SLEW_RATE_NORMAL      12

// Direction reversal slew rate (e.g., 100→-100)
// Should be LOWER than normal to prevent tipping during reversals.
// Recommended: 40-60% of SBOT_SLEW_RATE_NORMAL
#define SBOT_SLEW_RATE_REVERSAL    6

// Force stop before reversing direction?
// If true, robot briefly goes to 0 before changing direction (safest)
// If false, uses SBOT_SLEW_RATE_REVERSAL to gradually reverse (faster)
#define SBOT_FORCE_STOP_ON_REVERSAL true

// When forcing stop, how close to zero before allowing direction change?
#define SBOT_REVERSAL_DEADBAND     3

// Motor gearsets and brake modes
#define SBOT_DRIVE_GEARSET         pros::v5::MotorGears::blue
#define SBOT_DRIVE_BRAKE_MODE      pros::v5::MotorBrake::coast

// Generic velocity limits (RPM)
#define SBOT_MAX_DRIVE_VELOCITY    200

// Intake and indexer speeds (RPM, sign defines direction)
#define SBOT_INTAKE_FORWARD_SPEED        600   // Pull balls into robot
#define SBOT_INTAKE_REVERSE_LOW_GOAL    -600   // Spit balls out low (much higher speed)

#define SBOT_INDEXER_FORWARD_FEED       600   // Toward top goal
#define SBOT_INDEXER_REVERSE_MIDDLE    -400   // Drop to middle goal / eject

// Timing for button-driven scoring actions (ms)
#define SBOT_MID_GOAL_SCORE_TIME_MS     700
#define SBOT_LOW_GOAL_SCORE_TIME_MS     700
#define SBOT_COLOR_EJECT_TIME_MS        500

// ============================================================================
// COLOR SORTING CONFIGURATION
// ============================================================================

enum class AllianceColor {
    UNKNOWN = 0,
    RED,
    BLUE
};

#endif // _SBOT_CONFIG_H_
