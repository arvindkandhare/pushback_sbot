/**
 * drivetrain.cpp - 6-motor arcade drive using LemLib for sbot.
 */

#include "drivetrain.h"
#include "lemlib_config_sbot.h"

SbotDrivetrain::SbotDrivetrain()
        : left_front(-SBOT_LEFT_FRONT_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            left_middle(-SBOT_LEFT_MIDDLE_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            left_back(-SBOT_LEFT_BACK_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            right_front(SBOT_RIGHT_FRONT_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            right_middle(SBOT_RIGHT_MIDDLE_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            right_back(SBOT_RIGHT_BACK_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            prev_left_cmd(0), prev_right_cmd(0) {

    left_front.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    left_middle.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    left_back.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    right_front.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    right_middle.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    right_back.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
}

int SbotDrivetrain::applyDeadzone(int value) const {
    if (std::abs(value) < SBOT_JOYSTICK_DEADZONE) return 0;
    return value;
}

int SbotDrivetrain::applyCurve(int value) const {
#if SBOT_USE_SQUARED_CURVE
    if (value == 0) return 0;
    
    // Normalized input (-1.0 to 1.0)
    double normalized = value / 127.0;
    
    // Preserve sign, apply squared curve
    double squared = normalized * std::abs(normalized);
    
    // Scale by curve_scaling: 1.0 = full squared, 0.0 = linear
    double output = normalized + SBOT_CURVE_SCALING * (squared - normalized);
    
    // Convert back to motor command (-127 to 127)
    return static_cast<int>(output * 127);
#else
    return value;  // Linear response
#endif
}

int SbotDrivetrain::applySlewRate(int current, int target, int max_change) const {
    int delta = target - current;
    
    if (std::abs(delta) <= 1) return target;
    
    // Check for direction reversal
    bool is_reversing = (current > SBOT_REVERSAL_DEADBAND && target < -SBOT_REVERSAL_DEADBAND) ||
                        (current < -SBOT_REVERSAL_DEADBAND && target > SBOT_REVERSAL_DEADBAND);
    
    if (is_reversing && SBOT_FORCE_STOP_ON_REVERSAL) {
        if (std::abs(current) > SBOT_REVERSAL_DEADBAND) {
            int step = std::min(SBOT_SLEW_RATE_NORMAL, std::abs(current));
            return current > 0 ? current - step : current + step;
        } else {
            int step = std::min(SBOT_SLEW_RATE_NORMAL, std::abs(delta));
            return delta > 0 ? step : -step;
        }
    }
    
    if (std::abs(delta) <= SBOT_SLEW_RATE_NORMAL) return target;
    return current + (delta > 0 ? SBOT_SLEW_RATE_NORMAL : -SBOT_SLEW_RATE_NORMAL);
}

void SbotDrivetrain::arcadeTankControl(pros::Controller& master) {
    if (!sbot_chassis) return;
    
    // Get joystick inputs
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    
    // Apply deadzone
    throttle = applyDeadzone(throttle);
    turn = applyDeadzone(turn);
    
    // LemLib arcade handles:
    // - Input curves (expo scaling)
    // - Throttle/steer priority
    // - Motor commands
    // We set driveCurve=false to skip internal curves (already in chassis setup)
    sbot_chassis->arcade(throttle, turn, false);
    
    // Note: Slew rate limiting is now handled by LemLib's max_acceleration
    // in the controller settings. If you need custom slew for driver control,
    // we can add it back by intercepting motor commands.
}

void SbotDrivetrain::splitArcadeControl(pros::Controller& master) {
    int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Step 1: Apply deadzone
    forward = applyDeadzone(forward);
    turn = applyDeadzone(turn);

    // Step 2: Apply squared curve
    forward = applyCurve(forward);
    turn = applyCurve(turn);

    // Step 3: Calculate left and right powers
    int left_power = forward + turn;
    int right_power = forward - turn;

    // Step 4: Scale by sensitivity
    left_power = static_cast<int>(left_power * SBOT_TANK_SENSITIVITY);
    right_power = static_cast<int>(right_power * SBOT_TANK_SENSITIVITY);

    // Step 5: Clamp to valid range
    if (left_power > 127) left_power = 127;
    if (left_power < -127) left_power = -127;
    if (right_power > 127) right_power = 127;
    if (right_power < -127) right_power = -127;

    // Step 6: Apply slew rate limiting
    left_power = applySlewRate(prev_left_cmd, left_power, 0);
    right_power = applySlewRate(prev_right_cmd, right_power, 0);

    // Store for next iteration
    prev_left_cmd = left_power;
    prev_right_cmd = right_power;

    // Send commands to motors
    left_front.move(left_power);
    left_middle.move(left_power);
    left_back.move(left_power);

    right_front.move(right_power);
    right_middle.move(right_power);
    right_back.move(right_power);
}

// Modify the function signature to match the header
void SbotDrivetrain::tankControl(pros::Controller& master, bool run_slow) {
    
    // 1. Get raw inputs
    int raw_left = master.get_analog(SBOT_TANK_LEFT_STICK);
    int raw_right = master.get_analog(SBOT_TANK_RIGHT_STICK);

    // 2. Apply Deadzones & Curves
    int left_in = applyCurve(applyDeadzone(raw_left));
    int right_in = applyCurve(applyDeadzone(raw_right));

    // 3. Separate Forward vs Turn
    double forward = (left_in + right_in) / 2.0;
    double turn = (left_in - right_in) / 2.0;

    // 4. Reduce Turn Sensitivity (The permanent fix from before)
    turn *= 0.5; // Keep this at your preferred turning feel

    // 5. Recombine
    int final_left = static_cast<int>(forward + turn);
    int final_right = static_cast<int>(forward - turn);

    // --- NEW: GLOBAL SLOW MODE ---
    if (run_slow) {
        final_left *= 0.5;  // Cut speed in half
        final_right *= 0.5; // Cut speed in half
    }
    // -----------------------------

    // 6. Scale, Clamp, and Slew (Standard checks)
    final_left = static_cast<int>(final_left * SBOT_TANK_SENSITIVITY);
    final_right = static_cast<int>(final_right * SBOT_TANK_SENSITIVITY);

    if (final_left > 127) final_left = 127;
    if (final_left < -127) final_left = -127;
    if (final_right > 127) final_right = 127;
    if (final_right < -127) final_right = -127;

    final_left = applySlewRate(prev_left_cmd, final_left, 0);
    final_right = applySlewRate(prev_right_cmd, final_right, 0);

    prev_left_cmd = final_left;
    prev_right_cmd = final_right;

    // 7. Move Motors
    left_front.move(final_left);
    left_middle.move(final_left);
    left_back.move(final_left);

    right_front.move(final_right);
    right_middle.move(final_right);
    right_back.move(final_right);
}

void SbotDrivetrain::setBrakeMode(pros::v5::MotorBrake mode) {
    if (sbot_left_motors) sbot_left_motors->set_brake_mode_all(mode);
    if (sbot_right_motors) sbot_right_motors->set_brake_mode_all(mode);
}

void SbotDrivetrain::stop() {
    if (sbot_chassis) {
        sbot_chassis->arcade(0, 0);
    }
}
