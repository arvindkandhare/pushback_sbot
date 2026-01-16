/**
 * lemlib_config_sbot.h - LemLib configuration for the sbot robot.
 *
 * This mirrors the structure of the main pushback LemLib config but is
 * simplified and tuned independently for the sbot.
 */

#ifndef _SBOT_LEMLIB_CONFIG_H_
#define _SBOT_LEMLIB_CONFIG_H_

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "config_sbot.h"

// ============================================================================
// DRIVETRAIN GEOMETRY (placeholders - tune later)
// ============================================================================

#define SBOT_DRIVE_TRACK_WIDTH        10.0   // inches between left/right wheels (measured)
#define SBOT_DRIVE_RPM                200    // approx RPM for green gearset

// Tracking wheel geometry (1D forward/backward pod near center)
// NOTE: This is the lateral (left/right) offset of the vertical tracking wheel from the robot's rotation center.
// If this is wrong, you'll see Y drift during in-place turns (rectangle test).
// Experimental estimate from in-place turn calibration (Test: Turn): ~0.06".
// If turn-induced drift direction worsens, flip the sign.
#define SBOT_TRACKING_WHEEL_DISTANCE  0.085

// ============================================================================
// GLOBAL LEMLIB OBJECTS (pointers)
// ============================================================================

// Motor groups
extern pros::MotorGroup* sbot_left_motors;
extern pros::MotorGroup* sbot_right_motors;

// Drivetrain
extern lemlib::Drivetrain* sbot_drivetrain;

// Sensors
extern pros::Imu* sbot_inertial_sensor;
extern pros::Rotation* sbot_vertical_encoder;
extern lemlib::TrackingWheel* sbot_vertical_tracking_wheel;

// Controllers
extern lemlib::ControllerSettings* sbot_linear_controller;
extern lemlib::ControllerSettings* sbot_angular_controller;

// Odometry sensors and chassis
extern lemlib::OdomSensors* sbot_odom_sensors;
extern lemlib::Chassis* sbot_chassis;

// ============================================================================
// INITIALIZATION HELPERS
// ============================================================================

void initializeSbotLemLib();
bool isSbotLemLibInitialized();
bool validateSbotLemLibInitialization();

#endif // _SBOT_LEMLIB_CONFIG_H_
