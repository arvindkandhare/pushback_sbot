/**
 * autonomous_infrastructure.cpp - Part 1: Utility Functions
 * 
 * Helper functions for autonomous routines.
 * This file contains coordinate conversions, pose utilities, and wait functions.
 */

#include "autonomous_infrastructure.h"
#include "autonomous_constants.h"
#include "lemlib_config_sbot.h"
#include "config_sbot.h"
#include "intake.h"
#include "indexer.h"
#include "pneumatics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sys/_intsup.h>

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (declared as extern in .h)
// ============================================================================

double sbot_jerry_start_x = SBOT_JERRY_START_RL_X;
double sbot_jerry_start_y = SBOT_JERRY_START_RL_Y;

bool sbot_auton_elapsed_active = false;
uint32_t sbot_auton_elapsed_start_ms = 0;

// Runtime stats
static uint32_t sbot_low_goal_score_total_ms = 0;
static uint32_t sbot_low_goal_score_count = 0;

// ============================================================================
// DEBUG HELPERS
// ============================================================================

void sbot_lemlib_debug_window_begin(const char* /*label*/) {}
void sbot_lemlib_debug_window_end(const char* /*label*/) {}

void sbot_dump_jerry_point() {
    if (!SBOT_DUMP_JERRY_POINTS) return;
    if (!sbot_chassis) return;

    const auto pose = sbot_chassis->getPose();
    const double jerry_x = pose.y + sbot_jerry_start_x;
    const double jerry_y = sbot_jerry_start_y - pose.x;

    printf("%.3f,%.3f,120\n", jerry_x, jerry_y);
}

void sbot_print_jerry_pose(const char* label) {
    if (!sbot_chassis) return;
    const auto pose = sbot_chassis->getPose();
    const double jerry_x = pose.y + sbot_jerry_start_x;
    const double jerry_y = sbot_jerry_start_y - pose.x;
    printf(
        "SBOT POSE JERRY [%s]: our(%.2f,%.2f,%.2f) => jerry(%.3f,%.3f)\n",
        label,
        pose.x,
        pose.y,
        pose.theta,
        jerry_x,
        jerry_y
    );
}

void sbot_print_jerry_pose_rotated(const char* label) {
    if (!sbot_chassis) return;
    const auto pose = sbot_chassis->getPose();
    // Inverse of sbot_from_jerry_rotated: rotate by +90°
    // rotated_x = base_y, rotated_y = -base_x
    // So, base_x = -rotated_y, base_y = rotated_x
    // Jerry X = base_y + sbot_jerry_start_x
    // Jerry Y = sbot_jerry_start_y - base_x
    double base_x = -pose.y;
    double base_y = pose.x;
    double jerry_x = base_y + sbot_jerry_start_x;
    double jerry_y = sbot_jerry_start_y - base_x;
    printf(
        "SBOT POSE JERRY ROTATED [%s]: our(%.2f,%.2f,%.2f) => jerry(%.3f,%.3f)\n",
        label,
        pose.x,
        pose.y,
        pose.theta,
        jerry_x,
        jerry_y
    );
}

void sbot_print_jerry_target(const char* label, double target_x, double target_y) {
    const double jerry_x = target_y + sbot_jerry_start_x;
    const double jerry_y = sbot_jerry_start_y - target_x;
    printf(
        "SBOT TARGET JERRY [%s]: our(%.2f,%.2f) => jerry(%.3f,%.3f)\n",
        label,
        target_x,
        target_y,
        jerry_x,
        jerry_y
    );
}

void sbot_print_auton_elapsed(const char* label) {
    if (!sbot_auton_elapsed_active) return;
    const uint32_t ms = pros::millis() - sbot_auton_elapsed_start_ms;
    printf("SBOT ELAPSED [%s]: %u ms (%.2f s)\n", label, static_cast<unsigned>(ms), ms / 1000.0);
}

void sbot_trace_follow_progress(uint32_t start_ms, uint32_t now_ms) {
    if (!SBOT_TRACE_POSE_WAIT_PROGRESS) return;
    if (!sbot_chassis) return;
    
    static uint32_t last_ms = 0;
    if (now_ms - last_ms < SBOT_TRACE_POSE_WAIT_PERIOD_MS) return;
    last_ms = now_ms;
    
    const auto pose = sbot_chassis->getPose();
    printf("FOLLOW @%ums: pose(%.2f,%.2f,%.1f)\n", 
           now_ms - start_ms, pose.x, pose.y, pose.theta);
}

// ============================================================================
// TIMING UTILITIES
// ============================================================================

void sbot_run_for_ms(uint32_t duration_ms) {
    pros::delay(duration_ms);
}

// ============================================================================
// POSE AND SENSOR UTILITIES
// ============================================================================

void sbot_print_pose(const char* label) {
    if (!sbot_chassis) return;
    const auto pose = sbot_chassis->getPose();
    printf("SBOT POSE [%s]: (%.2f, %.2f, %.2f°)\n", label, pose.x, pose.y, pose.theta);
}

void sbot_print_sensors(const char* label) {
    if (!sbot_vertical_tracking_wheel || !sbot_inertial_sensor) return;
    const double vert = sbot_vertical_tracking_wheel->getDistanceTraveled();
    const double imu_rot = sbot_inertial_sensor->get_rotation();
    printf("SBOT SENSORS [%s]: vert=%.2f imu=%.2f°\n", label, vert, imu_rot);
}

void sbot_zero_pose_and_sensors(float x, float y, float theta_deg) {
    if (!sbot_chassis) return;
    
    if (sbot_vertical_tracking_wheel) {
        sbot_vertical_tracking_wheel->reset();
    }
    if (sbot_inertial_sensor) {
        sbot_inertial_sensor->tare_rotation();
    }
    
    pros::delay(40);
    sbot_chassis->setPose(x, y, theta_deg);
    pros::delay(40);
}

void sbot_set_match_start_pose() {
    sbot_zero_pose_and_sensors(0, 0, 0);
}

double sbot_get_best_heading_deg() {
    if (sbot_chassis) {
        return sbot_chassis->getPose().theta;
    }
    if (sbot_inertial_sensor) {
        return sbot_inertial_sensor->get_rotation();
    }
    return 0.0;
}

double sbot_pose_to_imu_heading(double pose_heading_target_deg) {
    if (!sbot_chassis) return pose_heading_target_deg;
    return sbot_norm_heading(pose_heading_target_deg - 90.0);
}

double sbot_heading_error_deg(double target_deg, double current_deg) {
    double err = sbot_norm_heading(target_deg) - sbot_norm_heading(current_deg);
    if (err > 180.0) err -= 360.0;
    if (err < -180.0) err += 360.0;
    return err;
}

// ============================================================================
// COORDINATE CONVERSIONS
// ============================================================================

double sbot_norm_heading(double deg) {
    while (deg < 0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

SbotPoint sbot_from_jerry(double jerry_x, double jerry_y) {
    const double our_x = sbot_jerry_start_y - jerry_y;
    const double our_y = jerry_x - sbot_jerry_start_x;
    return {our_x, our_y};
}

SbotPoint sbot_from_jerry_rotated(double jerry_x, double jerry_y) {
    // Step 1: Base conversion
    const double base_x = sbot_jerry_start_y - jerry_y;
    const double base_y = jerry_x - sbot_jerry_start_x;
    // Step 2: Rotate by -90° to match robot orientation
    //   rotated_x =  base_y
    //   rotated_y = -base_x
    return {base_y, -base_x};
}

SbotPoint sbot_mirror_point_y(const SbotPoint& p) {
    return {-p.x, p.y};
}

SbotPoint sbot_mirror_point_x(const SbotPoint& p) {
    return {p.x, -p.y};
}

double sbot_mirror_heading(double heading_deg) {
    return sbot_norm_heading(180.0 - heading_deg);
}

SbotPoint sbot_rotate180_point(const SbotPoint& p) {
    return {-p.x, -p.y};
}

double sbot_rotate180_heading(double heading_deg) {
    return sbot_norm_heading(heading_deg + 180.0);
}

SbotPoint sbot_apply_auto_transform(const SbotPoint& p, SbotAutoSide side, SbotAutoAlliance alliance) {
    SbotPoint out = p;
    if (alliance == SbotAutoAlliance::BLUE) {
        out = sbot_rotate180_point(out);
    }
    if (side == SbotAutoSide::LEFT) {
        out = sbot_mirror_point_x(out);
    }
    return out;
}

double sbot_apply_auto_transform_heading(double heading_deg, SbotAutoSide side, SbotAutoAlliance alliance) {
    double out = heading_deg;
    if (alliance == SbotAutoAlliance::BLUE) {
        out = sbot_rotate180_heading(out);
    }
    if (side == SbotAutoSide::LEFT) {
        out = sbot_mirror_heading(out);
    }
    return sbot_norm_heading(out);
}

SbotPoint sbot_apply_alliance_transform_only(const SbotPoint& p, SbotAutoAlliance alliance) {
    return (alliance == SbotAutoAlliance::BLUE) ? sbot_rotate180_point(p) : p;
}

double sbot_apply_alliance_transform_heading_only(double heading_deg, SbotAutoAlliance alliance) {
    return (alliance == SbotAutoAlliance::BLUE) ? sbot_rotate180_heading(heading_deg) : sbot_norm_heading(heading_deg);
}

double sbot_dist_in(const SbotPoint& a, const SbotPoint& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

SbotPoint sbot_offset_forward(const SbotPoint& p, double heading_deg, double distance_in) {
    const double rad = heading_deg * M_PI / 180.0;
    return {p.x + distance_in * std::cos(rad), p.y + distance_in * std::sin(rad)};
}

SbotPoint sbot_pose_from_front_contact(const SbotPoint& contact, double heading_deg, double front_bumper_in) {
    return sbot_offset_forward(contact, heading_deg, -front_bumper_in);
}

SbotPoint sbot_pose_from_back_contact(const SbotPoint& contact, double heading_deg, double back_bumper_in) {
    return sbot_offset_forward(contact, heading_deg, back_bumper_in);
}

// ============================================================================
// VALIDATION
// ============================================================================

bool validateSbotLemLibInitialization() {
    return sbot_chassis != nullptr &&
           sbot_left_motors != nullptr &&
           sbot_right_motors != nullptr &&
           sbot_drivetrain != nullptr;
}

// ============================================================================
// WAIT UTILITIES
// ============================================================================

void sbot_wait_until_done_timed(const char* label) {
    if (!sbot_chassis) return;
    if (!SBOT_PRINT_WAIT_TIMES) {
        sbot_chassis->waitUntilDone();
        return;
    }
    const uint32_t start = pros::millis();
    sbot_chassis->waitUntilDone();
    const uint32_t dur = pros::millis() - start;
    printf("SBOT WAIT [%s]: %u ms\n", label, dur);
}

void sbot_wait_until_done_or_timed_out_timed(const char* label, uint32_t overall_timeout_ms) {
    if (!sbot_chassis) return;
    
    const uint32_t start = pros::millis();
    while (sbot_chassis->isInMotion() && (pros::millis() - start < overall_timeout_ms)) {
        pros::delay(10);
    }
    
    const uint32_t dur = pros::millis() - start;
    if (SBOT_PRINT_WAIT_TIMES) {
        printf("SBOT WAIT [%s]: %u ms\n", label, dur);
    }
}

void sbot_wait_until_done_or_stalled_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in
) {
    if (!sbot_chassis) return;

    const uint32_t start = pros::millis();
    uint32_t last_moved_ms = start;
    auto last_moved_pose = sbot_chassis->getPose();
    bool has_moved = false;

    bool stalled = false;
    while (sbot_chassis->isInMotion() && (pros::millis() - start < overall_timeout_ms)) {
        pros::delay(10);
        const uint32_t now = pros::millis();

        const auto pose = sbot_chassis->getPose();
        const double dx = pose.x - last_moved_pose.x;
        const double dy = pose.y - last_moved_pose.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= stall_epsilon_in) {
            has_moved = true;
            last_moved_pose = pose;
            last_moved_ms = now;
        }

        if (has_moved && (now - last_moved_ms >= stall_window_ms)) {
            stalled = true;
            sbot_chassis->cancelAllMotions();
            break;
        }
    }

    const uint32_t dur = pros::millis() - start;
    if (SBOT_PRINT_WAIT_TIMES) {
        printf(
            "SBOT WAIT [%s]: %u ms%s\n",
            label,
            dur,
            stalled ? " (stalled early-exit)" : ""
        );
    }
}

void sbot_wait_until_done_or_stalled_near_target_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    const SbotPoint& target,
    double stall_only_within_in
) {
    if (!sbot_chassis) return;

    const uint32_t start = pros::millis();
    uint32_t last_moved_ms = start;
    auto last_moved_pose = sbot_chassis->getPose();
    bool has_moved = false;

    bool stalled = false;
    while (sbot_chassis->isInMotion() && (pros::millis() - start < overall_timeout_ms)) {
        pros::delay(10);
        const uint32_t now = pros::millis();

        const auto pose = sbot_chassis->getPose();
        const double dx = pose.x - last_moved_pose.x;
        const double dy = pose.y - last_moved_pose.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= stall_epsilon_in) {
            has_moved = true;
            last_moved_pose = pose;
            last_moved_ms = now;
        }

        const double err_x = target.x - pose.x;
        const double err_y = target.y - pose.y;
        const double dist_to_target = std::sqrt(err_x * err_x + err_y * err_y);

        if (has_moved && dist_to_target <= stall_only_within_in && (now - last_moved_ms >= stall_window_ms)) {
            stalled = true;
            sbot_chassis->cancelAllMotions();
            break;
        }
    }

    const uint32_t dur = pros::millis() - start;
    if (SBOT_PRINT_WAIT_TIMES) {
        printf(
            "SBOT WAIT [%s]: %u ms%s\n",
            label,
            dur,
            stalled ? " (stalled near target)" : ""
        );
    }
}

void sbot_wait_until_pose_close_or_timeout_timed(
    const char* label,
    uint32_t timeout_ms,
    const SbotPoint& target,
    double target_heading_deg,
    double pose_close_in,
    double heading_close_deg
) {
    if (!sbot_chassis) return;

    const uint32_t start_ms = pros::millis();
    const uint32_t motion_start_ms = start_ms;

    bool converged = false;
    while (sbot_chassis->isInMotion() && (pros::millis() - start_ms < timeout_ms)) {
        pros::delay(10);
        const uint32_t now = pros::millis();

        if (SBOT_TRACE_POSE_WAIT_PROGRESS && (now - motion_start_ms) % SBOT_TRACE_POSE_WAIT_PERIOD_MS < 20) {
            const auto pose = sbot_chassis->getPose();
            const double dx = target.x - pose.x;
            const double dy = target.y - pose.y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double hErr = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));
            printf(
                "POSE_CLOSE @%ums: dist=%.2f° hErr=%.2f°\n",
                now - motion_start_ms,
                dist,
                hErr
            );
        }

        const auto pose = sbot_chassis->getPose();
        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double hErr = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));

        if (dist <= pose_close_in && hErr <= heading_close_deg) {
            converged = true;
            sbot_chassis->cancelAllMotions();
            break;
        }
    }

    const auto pose_end = sbot_chassis->getPose();
    const double dx_end = target.x - pose_end.x;
    const double dy_end = target.y - pose_end.y;
    const double dist_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    const double hErr_end = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));

    const uint32_t dur = pros::millis() - start_ms;
    if (SBOT_PRINT_WAIT_TIMES) {
        printf(
            "SBOT WAIT [%s]: %u ms dist=%.2f hErr=%.2f%s\n",
            label,
            dur,
            dist_end,
            hErr_end,
            converged ? " (converged)" : ""
        );
    }
}


// ============================================================================
// MOVEMENT PRIMITIVES
// ============================================================================

bool sbot_drive_to(const SbotPoint& p, uint32_t timeout_ms, bool mirrored_y, bool forwards, float speed) {
    if (!sbot_chassis) return false;
    
    const SbotPoint target = mirrored_y ? sbot_mirror_point_y(p) : p;
    
    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = static_cast<int>(speed);

    sbot_chassis->moveToPoint(target.x, target.y, timeout_ms, params, true);
    return true;
}

bool sbot_turn_to(double heading_deg, uint32_t timeout_ms, bool mirrored_y, float speed) {
    if (!sbot_chassis) return false;
    
    const double target_heading = mirrored_y ? sbot_mirror_heading(heading_deg) : sbot_norm_heading(heading_deg);
    
    lemlib::TurnToHeadingParams params;
    params.maxSpeed = static_cast<int>(speed);
    sbot_chassis->turnToHeading(target_heading, timeout_ms, params, true);
    return true;
}

bool sbot_auto_drive_to(const SbotPoint& p, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance, bool forwards) {
    if (!sbot_chassis) return false;

    const SbotPoint target = sbot_apply_auto_transform(p, side, alliance);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    sbot_chassis->moveToPoint(target.x, target.y, timeout_ms, params);
    sbot_chassis->waitUntilDone();
    return true;
}

bool sbot_auto_turn_to(double heading_deg, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance) {
    if (!sbot_chassis) return false;

    const double target_heading = sbot_apply_auto_transform_heading(heading_deg, side, alliance);
    sbot_chassis->turnToHeading(target_heading, timeout_ms);
    sbot_chassis->waitUntilDone();
    return true;
}

void sbot_turn_point_turn(
    const char* label,
    float target_x,
    float target_y,
    float target_heading_deg,
    uint32_t turn_timeout_ms,
    uint32_t drive_motion_timeout_ms,
    const lemlib::TurnToHeadingParams& turn_params,
    const lemlib::MoveToPointParams& drive_params,
    uint32_t wait_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    double stall_only_within_in
) {
    if (!sbot_chassis) return;

    char step_label[128];

    // Prepare TurnToPointParams for turnToPoint
    lemlib::TurnToPointParams turn_to_point_params;

    // Optionally, copy relevant fields from turn_params to turn_to_point_params if needed
    // (You may need to set fields here if your project requires specific tuning)

    // Turn 1: Face the target
    std::snprintf(step_label, sizeof(step_label), "%s.turn1", label);
    sbot_chassis->turnToPoint(target_x, target_y, turn_timeout_ms, turn_to_point_params);
    sbot_wait_until_done_timed(step_label);
    
    // Drive to target
    std::snprintf(step_label, sizeof(step_label), "%s.drive", label);
    sbot_chassis->moveToPoint(target_x, target_y, static_cast<int>(drive_motion_timeout_ms), drive_params);
    
    sbot_wait_until_done_or_stalled_near_target_timed(
        step_label,
        wait_timeout_ms,
        stall_window_ms,
        stall_epsilon_in,
        {target_x, target_y},
        stall_only_within_in
    );
    
    // Turn 2: Face final heading
    std::snprintf(step_label, sizeof(step_label), "%s.turn2", label);
    sbot_chassis->turnToHeading(target_heading_deg, static_cast<int>(turn_timeout_ms), turn_params);
    sbot_wait_until_done_timed(step_label);
}

bool sbot_drive_relative(double distance_in, uint32_t timeout_ms, bool forwards) {
    if (!sbot_chassis) return false;
    
    const auto pose = sbot_chassis->getPose();
    const double rad = pose.theta * M_PI / 180.0;
    const double dx = distance_in * std::cos(rad);
    const double dy = distance_in * std::sin(rad);
    
    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);
    
    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    
    sbot_chassis->moveToPoint(pose.x, pose.y, timeout_ms, params);
    return true;
}

bool sbot_drive_relative_stall_exit(
    const char* label,
    double distance_in,
    uint32_t motion_timeout_ms,
    uint32_t wait_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    bool forwards
) {
    if (!sbot_chassis) return false;
    
    const auto pose = sbot_chassis->getPose();
    const double rad = pose.theta * M_PI / 180.0;
    const double dx = distance_in * std::cos(rad);
    const double dy = distance_in * std::sin(rad);
    
    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);
    
    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    
    sbot_chassis->moveToPoint(target_x, target_y, motion_timeout_ms, params);
    
    sbot_wait_until_done_or_stalled_timed(
        label,
        wait_timeout_ms,
        stall_window_ms,
        stall_epsilon_in
    );
    
    return true;
}

// ============================================================================
// MECHANISM CONTROL
// ============================================================================

void sbot_safe_stop_mechanisms() {
    if (sbot_intake) {
        sbot_intake->setMode(IntakeMode::OFF);
        sbot_intake->update();
    }
    if (sbot_indexer) {
        sbot_indexer->setMode(IndexerMode::OFF);
        sbot_indexer->update();
    }
    if (sbot_goal_flap) sbot_goal_flap->close();
    if (sbot_batch_loader) sbot_batch_loader->retract();
}

void sbot_intake_on_storage() {
    if (!sbot_intake || !sbot_indexer || !sbot_goal_flap) return;
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_intake->update();
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_indexer->update();
    sbot_goal_flap->close();
}

void sbot_score_top_for(uint32_t duration_ms) {
    if (!sbot_intake || !sbot_indexer || !sbot_goal_flap) return;
    
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_goal_flap->open();
    
    const uint32_t start = pros::millis();
    while (pros::millis() - start < duration_ms) {
        sbot_intake->update();
        sbot_indexer->update();
        pros::delay(10);
    }
    
    sbot_safe_stop_mechanisms();
}

void sbot_score_mid_for(uint32_t duration_ms) {
    if (!sbot_indexer) return;
    
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    
    const uint32_t start = pros::millis();
    while (pros::millis() - start < duration_ms) {
        sbot_indexer->update();
        pros::delay(10);
    }
    
    sbot_indexer->setMode(IndexerMode::OFF);
}

void sbot_score_low_for(uint32_t duration_ms) {
    if (!sbot_intake) return;
    
    sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    
    const uint32_t start = pros::millis();
    while (pros::millis() - start < duration_ms) {
        sbot_intake->update();
        pros::delay(10);
    }
    
    sbot_intake->setMode(IntakeMode::OFF);
}
