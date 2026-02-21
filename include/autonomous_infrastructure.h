/**
 * autonomous_infrastructure.h
 * 
 * Function declarations for autonomous helper utilities, movement primitives,
 * and scoring routines. These are shared across all autonomous modes.
 * 
 * The actual implementations are in autonomous_infrastructure.cpp.
 */

#ifndef AUTONOMOUS_INFRASTRUCTURE_H
#define AUTONOMOUS_INFRASTRUCTURE_H

#include "lemlib/api.hpp"
#include "pros/misc.h"
#include <cstdint>

// Forward declarations for subsystem classes (defined in other files)
class SbotIntake;
class SbotIndexer; 
class BatchLoaderPiston;
class GoalFlapPiston;

// External subsystem pointers (defined in main.cpp or their respective files)
extern SbotIntake* sbot_intake;
extern SbotIndexer* sbot_indexer;
extern BatchLoaderPiston* sbot_batch_loader;
extern GoalFlapPiston* sbot_goal_flap;
extern pros::Controller* sbot_master;

// ============================================================================
// ENUMS
// ============================================================================

enum class SbotAutoSide {
    RIGHT = 0,
    LEFT
};

enum class SbotAutoAlliance {
    RED = 0,
    BLUE
};

// ============================================================================
// POINT STRUCTURE
// ============================================================================

struct SbotPoint {
    double x;
    double y;
    
    SbotPoint() : x(0), y(0) {}
    SbotPoint(double x_, double y_) : x(x_), y(y_) {}
};

// ============================================================================
// GLOBAL VARIABLES (active Jerry start position)
// ============================================================================

// These track the current Jerry coordinate system start position
// Set by autonomous routines before running
extern double sbot_jerry_start_x;
extern double sbot_jerry_start_y;

// Autonomous elapsed time tracking
extern bool sbot_auton_elapsed_active;
extern uint32_t sbot_auton_elapsed_start_ms;

// ============================================================================
// COORDINATE CONVERSION (Jerry <-> Robot Frame)
// ============================================================================

// Convert Jerry absolute coordinates to robot-relative coordinates
// Uses the active Jerry start position set by the autonomous routine
SbotPoint sbot_from_jerry(double jerry_x, double jerry_y);
SbotPoint sbot_from_jerry_rotated(double jerry_x, double jerry_y);

// Coordinate transformations for mirroring
SbotPoint sbot_mirror_point_x(const SbotPoint& p);
SbotPoint sbot_mirror_point_y(const SbotPoint& p);
double sbot_mirror_heading(double heading_deg);
double sbot_norm_heading(double deg);

// 180-degree rotation transformations (for Blue alliance)
SbotPoint sbot_rotate180_point(const SbotPoint& p);
double sbot_rotate180_heading(double heading_deg);

// Auto-side and alliance transformations
SbotPoint sbot_apply_auto_transform(const SbotPoint& p, SbotAutoSide side, SbotAutoAlliance alliance);
double sbot_apply_auto_transform_heading(double heading_deg, SbotAutoSide side, SbotAutoAlliance alliance);
SbotPoint sbot_apply_alliance_transform_only(const SbotPoint& p, SbotAutoAlliance alliance);
double sbot_apply_alliance_transform_heading_only(double heading_deg, SbotAutoAlliance alliance);

// Convert contact points (where bumper touches) to pose targets (robot center)
SbotPoint sbot_pose_from_front_contact(const SbotPoint& contact, double heading_deg, double front_bumper_in);
SbotPoint sbot_pose_from_back_contact(const SbotPoint& contact, double heading_deg, double back_bumper_in);
SbotPoint sbot_offset_forward(const SbotPoint& p, double heading_deg, double distance_in);

// Distance calculation
double sbot_dist_in(const SbotPoint& a, const SbotPoint& b);

// ============================================================================
// POSE AND SENSOR UTILITIES
// ============================================================================

// Get best available heading (IMU or pose-based)
double sbot_get_best_heading_deg();
double sbot_pose_to_imu_heading(double pose_heading_target_deg);
double sbot_heading_error_deg(double target_deg, double current_deg);

// Print current pose and sensor readings (for debugging)
void sbot_print_pose(const char* label);
void sbot_print_sensors(const char* label);
void sbot_print_jerry_pose(const char* label);
void sbot_print_jerry_pose_rotated(const char* label);
void sbot_print_jerry_target(const char* label, double target_x, double target_y);

// Dump current position as Jerry coordinate (for logging/plotting)
void sbot_dump_jerry_point();

// Reset odometry and sensors to a known pose
void sbot_zero_pose_and_sensors(float x, float y, float theta_deg);

// Set the standard match start pose
void sbot_set_match_start_pose();

// ============================================================================
// TIMING AND ELAPSED TRACKING
// ============================================================================

// Simple delay helper
void sbot_run_for_ms(uint32_t ms);

// Autonomous elapsed time tracking
void sbot_print_auton_elapsed(const char* label);

// Debug helper for path following progress
void sbot_trace_follow_progress(uint32_t start_ms, uint32_t now_ms);

// ============================================================================
// WAIT UTILITIES (with timeout and stall detection)
// ============================================================================

// Basic wait for LemLib motion to complete
void sbot_wait_until_done_timed(const char* label);

// Wait with timeout only (no stall detection)
void sbot_wait_until_done_or_timed_out_timed(const char* label, uint32_t overall_timeout_ms);

// Wait with stall detection (stops early if robot gets stuck)
void sbot_wait_until_done_or_stalled_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in
);

// Wait with conditional stall detection (only near target)
void sbot_wait_until_done_or_stalled_near_target_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    const SbotPoint& target,
    double stall_only_within_in
);

// Wait with pose+heading convergence check
void sbot_wait_until_pose_close_or_timeout_timed(
    const char* label,
    uint32_t timeout_ms,
    const SbotPoint& target,
    double target_heading_deg,
    double pose_close_in,
    double heading_close_deg
);

// ============================================================================
// MOVEMENT PRIMITIVES
// ============================================================================

// Move to a point (basic, with optional Y-mirror)
bool sbot_drive_to(const SbotPoint& p, uint32_t timeout_ms, bool mirrored_y = false, bool forwards = true, float speed = 60.0);

// Turn to a heading (basic, with optional Y-mirror)
bool sbot_turn_to(double heading_deg, uint32_t timeout_ms, bool mirrored_y = false, float speed = 80.0);

// Auto-transformed movement (handles side and alliance mirroring)
bool sbot_auto_drive_to(const SbotPoint& p, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance, bool forwards = true);
bool sbot_auto_turn_to(double heading_deg, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance);

// Turn-Point-Turn sequence (turn to face target, drive, turn to final heading)
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
);

// Drive relative distance (forward or backward)
bool sbot_drive_relative(double distance_in, uint32_t timeout_ms, bool forwards = true);

// Drive relative with stall detection
bool sbot_drive_relative_stall_exit(
    const char* label,
    double distance_in,
    uint32_t motion_timeout_ms,
    uint32_t wait_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    bool forwards = true
);

// ============================================================================
// MECHANISM CONTROL
// ============================================================================

// Stop all mechanisms safely
void sbot_safe_stop_mechanisms();

// Intake control
void sbot_intake_on_storage();

// Combined mechanism control (intake + indexer + pneumatics)
void sbot_score_top_for(uint32_t duration_ms);
void sbot_score_mid_for(uint32_t duration_ms);
void sbot_score_low_for(uint32_t duration_ms);

// ============================================================================
// DEBUG HELPERS
// ============================================================================

// LemLib debug window markers (currently no-ops, but may be enabled for tuning)
void sbot_lemlib_debug_window_begin(const char* label);
void sbot_lemlib_debug_window_end(const char* label);


void sbot_run_red_left_auto();
void sbot_run_red_right_auto();
// ============================================================================
// VALIDATION
// ============================================================================

// Check if LemLib is properly initialized
bool validateSbotLemLibInitialization();

#endif // AUTONOMOUS_INFRASTRUCTURE_H