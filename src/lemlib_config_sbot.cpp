/**
 * lemlib_config_sbot.cpp - LemLib configuration for sbot.
 */

#include "lemlib_config_sbot.h"

#include "lemlib/logger/logger.hpp"

#include <cmath>

// Motor groups
pros::MotorGroup* sbot_left_motors = nullptr;
pros::MotorGroup* sbot_right_motors = nullptr;

// Drivetrain
lemlib::Drivetrain* sbot_drivetrain = nullptr;

// Sensors
pros::Imu* sbot_inertial_sensor = nullptr;
pros::Rotation* sbot_vertical_encoder = nullptr;
lemlib::TrackingWheel* sbot_vertical_tracking_wheel = nullptr;

// Controllers
lemlib::ControllerSettings* sbot_linear_controller = nullptr;
lemlib::ControllerSettings* sbot_angular_controller = nullptr;

// Odometry and chassis
lemlib::OdomSensors* sbot_odom_sensors = nullptr;
lemlib::Chassis* sbot_chassis = nullptr;

static bool sbot_lemlib_initialized = false;

void initializeSbotLemLib() {
    if (sbot_lemlib_initialized) {
        printf("Sbot LemLib already initialized\n");
        return;
    }

    printf("Initializing LemLib for sbot...\n");
    printf("SBOT BUILD TAG: %s %s\n", __DATE__, __TIME__);

    // Keep LemLib logs quiet by default; we selectively enable DEBUG around
    // specific autonomous motions (e.g., turn+drive sequences) when needed.
    lemlib::infoSink()->setLowestLevel(lemlib::Level::WARN);

    // ----------------------- Motors & Drivetrain -----------------------

    // IMPORTANT: Motor direction must match driver-control drivetrain wiring.
    // In src/drivetrain.cpp the LEFT motors are constructed with negative ports and RIGHT with positive.
    // Keep LemLib consistent so "forward" in autonomous matches forward in opcontrol.
    sbot_left_motors = new pros::MotorGroup({
        -SBOT_LEFT_FRONT_MOTOR_PORT,
        -SBOT_LEFT_MIDDLE_MOTOR_PORT,
        -SBOT_LEFT_BACK_MOTOR_PORT
    }, pros::v5::MotorGears::green);

    sbot_right_motors = new pros::MotorGroup({
        SBOT_RIGHT_FRONT_MOTOR_PORT,
        SBOT_RIGHT_MIDDLE_MOTOR_PORT,
        SBOT_RIGHT_BACK_MOTOR_PORT
    }, pros::v5::MotorGears::green);

    sbot_drivetrain = new lemlib::Drivetrain(
        sbot_left_motors,
        sbot_right_motors,
        SBOT_DRIVE_TRACK_WIDTH,
        lemlib::Omniwheel::NEW_325,
        SBOT_DRIVE_RPM,
        2 // horizontal drift for omni wheels
    );

    // ---------------------------- Sensors -----------------------------

    sbot_inertial_sensor = new pros::Imu(SBOT_INERTIAL_PORT);
    sbot_vertical_encoder = new pros::Rotation(SBOT_ODOM_ROTATION_PORT);

    // CRITICAL: Change this to match your actual tracking wheel size!
    // Options: NEW_2 (2.125"), NEW_275 (2.75"), NEW_325 (3.25"), NEW_4 (4.0")
    // If robot goes too far: wheel diameter is TOO SMALL (increase)
    // If robot doesn't go far enough: wheel diameter is TOO LARGE (decrease)
    sbot_vertical_tracking_wheel = new lemlib::TrackingWheel(
        sbot_vertical_encoder,
        lemlib::Omniwheel::NEW_2,  // Changed from NEW_275 - robot not going far enough means diameter too large
        -0.55 // Lateral offset of the vertical tracking wheel from the robot's rotation center (inches). Positive is to the right, negative is to the left. This is a critical measurement for accurate turning. If you see Y drift during in-place turns (rectangle test), adjust this value. Experimental estimate from in-place turn calibration (Test: Turn): ~0.06". If turn-induced drift direction worsens, flip the sign.
    );

    sbot_odom_sensors = new lemlib::OdomSensors(
        nullptr,//sbot_vertical_tracking_wheel, // vertical 1
        nullptr,                      // vertical 2
        nullptr,                      // horizontal 1
        nullptr,                      // horizontal 2
        sbot_inertial_sensor          // IMU
    );

    // -------------------------- Controllers --------------------------

    sbot_linear_controller = new lemlib::ControllerSettings(
        18,   // kP
        0,    // kI
        125,   // kD
        0,    // windup
        0.5,  // small error (in)
        100,  // small error timeout (ms)
        1.0,  // large error (in)
        150,  // large error timeout (ms)
        2    // max acceleration (units/s^2) - limits how fast speed changes
              // Higher = faster but more tipping. 0 = unlimited. Try 30-80.
    );

    sbot_angular_controller = new lemlib::ControllerSettings(
        3.0,  // kP
        0.0,  // kI
        22.0, // kD
        0,    // windup
        1.0,  // small error (deg)
        100,  // small error timeout (ms)
        3.0,  // large error (deg)
        150,  // large error timeout (ms)
        40    // max acceleration (deg/s^2) - limits rotation speed changes
              // Higher = faster turns but more tipping. 0 = unlimited. Try 30-60.
    );

    // -------------------------- Input Curves -------------------------

    // Throttle curve: expo gain for fine control at low speeds
    lemlib::ExpoDriveCurve* throttle_curve = new lemlib::ExpoDriveCurve(
        10,    // joystick deadband (matches SBOT_JOYSTICK_DEADZONE)
        15,    // minimum output (prevents motor creeping)
        1.019  // expo gain (standard, good balance)
    );

    // Steer curve: slightly less aggressive for precise turns
    lemlib::ExpoDriveCurve* steer_curve = new lemlib::ExpoDriveCurve(
        10,    // joystick deadband
        10,    // minimum output (lower for steering sensitivity)
        1.019  // expo gain
    );

    // ---------------------------- Chassis ----------------------------

    sbot_chassis = new lemlib::Chassis(
        *sbot_drivetrain,
        *sbot_linear_controller,
        *sbot_angular_controller,
        *sbot_odom_sensors,
        throttle_curve,
        steer_curve
    );

    printf("Calibrating sbot chassis (IMU/odometry)...\n");
    sbot_chassis->calibrate();

    sbot_lemlib_initialized = true;
    printf("Sbot LemLib initialization complete.\n");
}

bool isSbotLemLibInitialized() {
    return sbot_lemlib_initialized;
}
