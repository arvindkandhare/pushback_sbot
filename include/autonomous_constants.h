/**
 * autonomous_constants.h
 * 
 * Jerry coordinate system definitions and field waypoints for autonomous routines.
 * These values are tuned frequently during field testing, so they're isolated here.
 * 
 * COORDINATE SYSTEM EXPLANATION:
 * - Jerry coords are absolute field inches (from field diagrams/measurement tool)
 * - Our internal coords are start-relative:
 *   - +Y is into-field from robot start
 *   - +X is robot-right at start
 *   - 0° faces +Y (into field)
 * 
 * IMPORTANT: The "Jerry start" is the LemLib pose point (drivetrain rotation center),
 * NOT the bumper contact point. Set these values to match your field measurement tool.
 */

#ifndef AUTONOMOUS_CONSTANTS_H
#define AUTONOMOUS_CONSTANTS_H

#include <cstdint>  // for uint32_t

// ============================================================================
// ROBOT GEOMETRY
// ============================================================================

// Distance from LemLib pose point (drivetrain rotation center) to bumpers (inches)
// Measured: 7.5" to both front and back bumpers
static constexpr double SBOT_FRONT_BUMPER_IN = 7.5;
static constexpr double SBOT_BACK_BUMPER_IN = 7.5;

// ============================================================================
// JERRY COORDINATE SYSTEM - START POSITIONS
// ============================================================================

// Red Left start (canonical base position in Jerry coordinates)
// This is where the LemLib pose point (rotation center) is located at match start
static constexpr double SBOT_JERRY_START_RL_X_BASE = -46.6;
static constexpr double SBOT_JERRY_START_RL_Y_BASE = 13.0;

// Fine-tuning adjustments (inches) for on-field calibration
// If robot consistently ends too far RIGHT (+X in our frame), DECREASE Y fine adjust
// (because ourX = jerry_start_y - jerry_y in the coordinate transform)
static constexpr double SBOT_JERRY_START_RL_X_FINE_ADJUST = 0.0;
static constexpr double SBOT_JERRY_START_RL_Y_FINE_ADJUST = 0.0;

// Final Red Left start position (base + fine adjustments)
static constexpr double SBOT_JERRY_START_RL_X = SBOT_JERRY_START_RL_X_BASE + SBOT_JERRY_START_RL_X_FINE_ADJUST;
static constexpr double SBOT_JERRY_START_RL_Y = SBOT_JERRY_START_RL_Y_BASE + SBOT_JERRY_START_RL_Y_FINE_ADJUST;

// Red Right start (mirrored across Jerry X-axis: Y is negated)
static constexpr double SBOT_JERRY_START_RR_X = SBOT_JERRY_START_RL_X;
static constexpr double SBOT_JERRY_START_RR_Y = -SBOT_JERRY_START_RL_Y;

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

// Minimum time to spend actively scoring at a goal (milliseconds)
static constexpr uint32_t SBOT_MIN_SCORE_TIME_MS = 1000;

// ============================================================================
// AUTONOMOUS SPEED LIMITS
// ============================================================================

// Match autonomous max speeds (LemLib units: 0-127 range)
static constexpr int SBOT_MATCH_MAX_SPEED = 127;
static constexpr int SBOT_MATCH_TURN_MAX_SPEED = 127;

// ============================================================================
// DEBUG FLAGS
// ============================================================================

// Enable Jerry coordinate point logging during autonomous
static constexpr bool SBOT_DUMP_JERRY_POINTS = true;

// Enable pose progress tracing during wait operations
static constexpr bool SBOT_TRACE_POSE_WAIT_PROGRESS = true;
static constexpr uint32_t SBOT_TRACE_POSE_WAIT_PERIOD_MS = 200;

// Enable detailed wait time printing
static constexpr bool SBOT_PRINT_WAIT_TIMES = true;

// ============================================================================
// FIELD WAYPOINTS - JERRY COORDINATES (inches)
// ============================================================================
// All coordinates below are in Jerry's absolute field coordinate system.
// They are converted to robot-relative coordinates by helper functions.
// ============================================================================

// ----------------------------------------------------------------------------
// RED LEFT WAYPOINTS
// ----------------------------------------------------------------------------
namespace RedLeft {
    // Ring clusters
    static constexpr double CLUSTER1_JERRY_X = -21.0;
    static constexpr double CLUSTER1_JERRY_Y = 21.0;
    
    // Center Goal - Middle (back bumper contact point)
    static constexpr double CENTER_MID_GOAL_JERRY_X = -8.0;
    static constexpr double CENTER_MID_GOAL_JERRY_Y = 8.0;
    
    // Post-score retreat point
    static constexpr double RETREAT_POINT_JERRY_X = -47.0;
    static constexpr double RETREAT_POINT_JERRY_Y = 47.0;
    
    // Match loader contact point (front bumper)
    static constexpr double LOADER_CONTACT_JERRY_X = -57.5;
    static constexpr double LOADER_CONTACT_JERRY_Y = 47.0;
    
    // Long goal end position
    static constexpr double LONG_GOAL_END_JERRY_X = -25.0;
    static constexpr double LONG_GOAL_END_JERRY_Y = 47.0;
}

// ----------------------------------------------------------------------------
// RED RIGHT WAYPOINTS (mirrored Y coordinates)
// ----------------------------------------------------------------------------
namespace RedRight {
    // Ring clusters
    static constexpr double CLUSTER1_JERRY_X = -21.0;
    static constexpr double CLUSTER1_JERRY_Y = -21.0;  // Y flipped
    
    static constexpr double CLUSTER2_JERRY_X = 21.0;
    static constexpr double CLUSTER2_JERRY_Y = -21.0;  // Y flipped
    
    // Center Goal - Lower (front bumper contact point)
    static constexpr double CENTER_LOW_GOAL_JERRY_X = -7.0;
    static constexpr double CENTER_LOW_GOAL_JERRY_Y = -7.0;  // Y flipped
    
    // Center Goal - Middle (back bumper contact point)
    static constexpr double CENTER_MID_GOAL_JERRY_X = -7.0;
    static constexpr double CENTER_MID_GOAL_JERRY_Y = -7.0;  // Y flipped
    
    // Solo Middle Goal (back bumper contact point, opposite side)
    static constexpr double SOLO_MID_GOAL_JERRY_X = 5.0;
    static constexpr double SOLO_MID_GOAL_JERRY_Y = -5.0;  // Y flipped
    
    // Post-score retreat point
    static constexpr double RETREAT_POINT_JERRY_X = -48.0;
    static constexpr double RETREAT_POINT_JERRY_Y = -48.0;  // Y flipped
    
    // Match loader contact point (front bumper)
    static constexpr double LOADER_CONTACT_JERRY_X = -73.0;
    static constexpr double LOADER_CONTACT_JERRY_Y = -48.0;  // Y flipped
    
    // Long goal end position
    static constexpr double LONG_GOAL_END_JERRY_X = -21.0;
    static constexpr double LONG_GOAL_END_JERRY_Y = -48.0;  // Y flipped from RedLeft (+51 -> -51)
}

// ----------------------------------------------------------------------------
// Skills WAYPOINTS
// ----------------------------------------------------------------------------
namespace Skills {
    static constexpr double SKILLS_JERRY_START_X_BASE = -46.3;
    static constexpr double SKILLS_JERRY_START_Y_BASE = 17.2;

    //Going to match loader
    static constexpr double SKILLS_TO_MATCH_LOADER_JERRY_X = -46.3;
    static constexpr double SKILLS_TO_MATCH_LOADER_JERRY_Y = 47.0;

    // Match loader contact point
    static constexpr double SKILLS_MATCH_LOADER_CONTACT_RED_JERRY_X = -57.5;
    static constexpr double SKILLS_MATCH_LOADER_CONTACT_RED_JERRY_Y = 47.0;

    static constexpr double SKILLS_MATCH_LOADER_RETREAT_JERRY_X = -51.0;
    static constexpr double SKILLS_MATCH_LOADER_RETREAT_JERRY_Y = 47.0;

    static constexpr double SKILLS_GOING_AROUND_LONG_GOAL_JERRY_X = -32.0;
    static constexpr double SKILLS_GOING_AROUND_LONG_GOAL_JERRY_Y = 58.25;

    static constexpr double SKILLS_GOING_ACROSS_LONG_GOAL_JERRY_X = 30.0;
    static constexpr double SKILLS_GOING_ACROSS_LONG_GOAL_JERRY_Y = 58.25;

    static constexpr double SKILLS_ALIGNING_TO_LONG_GOAL_JERRY_X = 42.0;
    static constexpr double SKILLS_ALIGNING_TO_LONG_GOAL_JERRY_Y = 46.5;

    static constexpr double SKILLS_LONG_GOAL_CONTACT_JERRY_X = 25.0;
    static constexpr double SKILLS_LONG_GOAL_CONTACT_JERRY_Y = 46.5;

    static constexpr double SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_X = 60.0;
    static constexpr double SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_Y = 46.5;

    static constexpr double SKILLS_LONG_GOAL_RETREAT_JERRY_X = 33.0;
    static constexpr double SKILLS_LONG_GOAL_RETREAT_JERRY_Y = 46.5;

    static constexpr double SKILLS_RIGHT_LONG_GOAL_RETREAT_X = 33.0;
    static constexpr double SKILLS_RIGHT_LONG_GOAL_RETREAT_Y = 39.0;

    static constexpr double SKILLS_RIGHT_MATCHLOAD_ALIGN_X = 33.0; 
    static constexpr double SKILLS_RIGHT_MATCHLOAD_ALIGN_Y = -48.0;

    static constexpr double SKILLS_PARK_RED_JERRY_X = -62.0;
    static constexpr double SKILLS_PARK_RED_JERRY_Y = 27.0;









    //HEADINGS
    static constexpr double SKILLS_MATCHLOADER_RED_HEADING = 270.0;
    static constexpr double SKILLS_GOING_AROUND_LONG_GOAL_HEADING = 52.1;
    static constexpr double SKILLS_GOING_ACROSS_LONG_GOAL_HEADING = 90.0;
    static constexpr double SKILLS_ALIGNING_TO_LONG_GOAL_HEADING = 134.0;
    static constexpr double SKILLS_LONG_GOAL_CONTACT_HEADING = 90.0;
    static constexpr double SKILLS_MATCHLOADER_BLUE_HEADING = 90.0;
    static constexpr double SKILLS_PARK_ONE_HEADING = 161.33;
    static constexpr double SKILLS_PARK_FINAL_HEADING = 180.0;
    static constexpr double SKILLS_BLUE_PARK_HEADING = 180.0;


}

// ============================================================================
// AUTONOMOUS TUNING PARAMETERS
// ============================================================================

// Extra distance to push into match loader after initial contact (inches)
static constexpr double SBOT_TUBE_EXTRA_SEAT_IN = 2.0;

// Loader protrusion when deployed (inches from front bumper)
static constexpr double SBOT_LOADER_DOWN_EXTRA_FRONT_IN = 6.0;

// Distance to back into long goal from tube position (inches)
static constexpr double SBOT_HIGH_GOAL_BACK_IN_FROM_TUBE_IN = 24.0;

// Collection timing (milliseconds)
static constexpr uint32_t SBOT_CLUSTER2_COLLECT_MS = 150;
static constexpr uint32_t SBOT_TUBE_PULL_MS = 500;

// Timeout values (milliseconds)
static constexpr uint32_t SBOT_DRIVE_TIMEOUT_MS = 2500;
static constexpr uint32_t SBOT_TURN_TIMEOUT_MS = 1300;

// ============================================================================
// SCORING HEADINGS (degrees)
// ============================================================================

namespace Headings {
    static constexpr double CENTER_LOW_GOAL = 45.0;
    static constexpr double CENTER_MID_GOAL = 180.0;
    static constexpr double SOLO_MID_GOAL_RL = 45.0;
    static constexpr double SOLO_MID_GOAL_RR = 135.0;  // Mirrored
    static constexpr double HIGH_GOAL = 180.0;
    static constexpr double TUBE_FACE = 180.0;
}

#endif // AUTONOMOUS_CONSTANTS_H