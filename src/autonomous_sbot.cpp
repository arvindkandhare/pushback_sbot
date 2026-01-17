/**
 * autonomous_sbot.cpp - Autonomous selector and stub routines for sbot.
 */

#include "autonomous_sbot.h"
#include "lemlib_config_sbot.h"

#include "intake.h"
#include "indexer.h"
#include "pneumatics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

// LemLib path-follow assets must be declared at global scope.
ASSET(sbot_awp_half_path_txt);
ASSET(low_txt);

// Robot geometry (distance from LemLib pose point to bumpers, in inches)
// Pose point is the drivetrain rotation center. Measured: 7.5" to both bumpers.
// These are used to convert field "contact points" (where a bumper should be) into pose targets.
static constexpr double SBOT_FRONT_BUMPER_IN = 7.5;
static constexpr double SBOT_BACK_BUMPER_IN = 7.5;

// Jerry coordinate conversion config.
// Jerry coords are absolute field inches. Our internal coords are start-relative:
// - +Y is into-field from the robot start, +X is robot-right at start
// - our 0° faces +Y
// IMPORTANT: We define the Jerry "start" as the LemLib pose point (drivetrain rotation center).
// Set the values below to whatever absolute Jerry coordinate you are using as match start.

// Canonical Jerry starts (pose-point reference).
// Red Right is mirrored across the Jerry X axis (Jerry Y is multiplied by -1).
static constexpr double SBOT_JERRY_START_RL_X_BASE = -50.0;
static constexpr double SBOT_JERRY_START_RL_Y_BASE = 15.0;

// Fine adjustments (inches) for on-field calibration.
// If the robot is consistently ending too far RIGHT (+X in our frame), DECREASE the Y fine adjust.
// (because ourX = jerry_start_y - jerry_y)
static constexpr double SBOT_JERRY_START_RL_X_FINE_ADJUST = 0.0;
static constexpr double SBOT_JERRY_START_RL_Y_FINE_ADJUST = 0.0;

static constexpr double SBOT_JERRY_START_RL_X = SBOT_JERRY_START_RL_X_BASE + SBOT_JERRY_START_RL_X_FINE_ADJUST;
static constexpr double SBOT_JERRY_START_RL_Y = SBOT_JERRY_START_RL_Y_BASE + SBOT_JERRY_START_RL_Y_FINE_ADJUST;
static constexpr double SBOT_JERRY_START_RR_X = SBOT_JERRY_START_RL_X;
static constexpr double SBOT_JERRY_START_RR_Y = -SBOT_JERRY_START_RL_Y;

// Active Jerry start used for conversions + logging during the current run.
static double sbot_jerry_start_x = SBOT_JERRY_START_RL_X;
static double sbot_jerry_start_y = SBOT_JERRY_START_RL_Y;

static constexpr bool SBOT_DUMP_JERRY_POINTS = true;

// Debug: during pose waits, periodically print pose + sensor deltas to help
// distinguish “robot not moving” vs “robot moving but odom not updating”.
static constexpr bool SBOT_TRACE_POSE_WAIT_PROGRESS = true;
static constexpr uint32_t SBOT_TRACE_POSE_WAIT_PERIOD_MS = 200;

static constexpr lemlib::Level SBOT_LEMLIB_NORMAL_LEVEL = lemlib::Level::WARN;

static void sbot_lemlib_debug_window_begin(const char* /*label*/) {}
static void sbot_lemlib_debug_window_end(const char* /*label*/) {}

static void sbot_dump_jerry_point() {
    if (!SBOT_DUMP_JERRY_POINTS) return;
    if (!sbot_chassis) return;

    const auto pose = sbot_chassis->getPose();
    const double jerry_x = pose.y + sbot_jerry_start_x;
    const double jerry_y = sbot_jerry_start_y - pose.x;

    // Plain point output (no labels, no ':'), for easy log cleanup.
    printf("%.3f,%.3f,120\n", jerry_x, jerry_y);
}

static void sbot_print_jerry_pose(const char* label) {
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

static void sbot_print_jerry_target(const char* label, double target_x, double target_y) {
    // Inverse of sbot_dump_jerry_point() mapping:
    //   jerry_x = ourY + jerry_start_x
    //   jerry_y = jerry_start_y - ourX
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

// Temporary: run match auton at reduced speed for tuning.
// LemLib maxSpeed is typically in the 0..127-ish range.
static constexpr int SBOT_MATCH_MAX_SPEED = 127;
static constexpr int SBOT_MATCH_TURN_MAX_SPEED = 127;

// Minimum time to spend actively scoring at a goal.
static constexpr uint32_t SBOT_MIN_SCORE_TIME_MS = 1000;

// Debug aid: print how long each LemLib waitUntilDone() blocks.
static constexpr bool SBOT_PRINT_WAIT_TIMES = true;

// Runtime stats (best-effort): track how long scoring routines actually take.
static uint32_t sbot_low_goal_score_total_ms = 0;
static uint32_t sbot_low_goal_score_count = 0;

// Best-effort autonomous elapsed timing (relative to when the selected auton starts running).
static bool sbot_auton_elapsed_active = false;
static uint32_t sbot_auton_elapsed_start_ms = 0;

static void sbot_print_auton_elapsed(const char* label) {
    if (!sbot_auton_elapsed_active) return;
    const uint32_t ms = pros::millis() - sbot_auton_elapsed_start_ms;
    printf("SBOT ELAPSED [%s]: %u ms (%.2f s)\n", label, static_cast<unsigned>(ms), ms / 1000.0);
}

static void sbot_wait_until_done_timed(const char* label) {
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

static void sbot_wait_until_done_or_stalled_timed(
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

        // IMPORTANT: only consider "stalled" once we've observed at least some odom translation.
        // If odom doesn't change at all (wheel slip, pushing goal, etc.), we prefer timing out rather than
        // cancelling immediately and never attempting the back-in.
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

struct SbotPoint;
static void sbot_wait_until_done_or_stalled_near_target_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    const SbotPoint& target,
    double stall_only_within_in
);

// Wait for motion to finish, but never cancel early due to "stall" heuristics.
// This is useful when we're intentionally driving into contact (goals/walls), where odom translation can freeze
// even while the drivetrain is still applying useful force.
static void sbot_wait_until_done_or_timed_out_timed(const char* label, uint32_t overall_timeout_ms) {
    if (!sbot_chassis) return;

    const uint32_t start = pros::millis();
    while (sbot_chassis->isInMotion() && (pros::millis() - start < overall_timeout_ms)) {
        pros::delay(10);
    }

    const uint32_t dur = pros::millis() - start;
    const bool timed_out = sbot_chassis->isInMotion();
    if (timed_out) sbot_chassis->cancelAllMotions();

    if (SBOT_PRINT_WAIT_TIMES) {
        printf(
            "SBOT WAIT [%s]: %u ms%s\n",
            label,
            dur,
            timed_out ? " (timed out)" : ""
        );
    }
}

// These are owned/created in src/main.cpp.
// We reference them here so autonomous can run mechanism actions.
extern SbotIntake* sbot_intake;
extern SbotIndexer* sbot_indexer;
extern BatchLoaderPiston* sbot_batch_loader;
extern GoalFlapPiston* sbot_goal_flap;

static void sbot_safe_stop_mechanisms() {
    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
    if (sbot_goal_flap) sbot_goal_flap->close();
    if (sbot_batch_loader) sbot_batch_loader->retract();

    // Apply immediately
    if (sbot_intake) sbot_intake->update();
    if (sbot_indexer) sbot_indexer->update();
}

static void sbot_run_for_ms(uint32_t duration_ms) {
    const uint32_t start = pros::millis();
    while (pros::millis() - start < duration_ms) {
        if (sbot_intake) sbot_intake->update();
        if (sbot_indexer) sbot_indexer->update();
        pros::delay(10);
    }
}

static void sbot_print_pose(const char* label) {
    if (!sbot_chassis) return;
    const auto pose = sbot_chassis->getPose();
    printf("SBOT POSE [%s]: x=%.2f y=%.2f th=%.2f\n", label, pose.x, pose.y, pose.theta);
    sbot_dump_jerry_point();
}

static void sbot_print_sensors(const char* label) {
    const double imu_heading = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;
    const double imu_rotation = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
    const double vert_pos = sbot_vertical_encoder ? sbot_vertical_encoder->get_position() : 0.0;
    const double vert_in = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;
    printf(
        "SBOT SENSORS [%s]: imu.heading=%.2f imu.rotation=%.2f vertRot.pos=%.2f vert.in=%.2f\n",
        label,
        imu_heading,
        imu_rotation,
        vert_pos,
        vert_in
    );
}

static void sbot_trace_follow_progress(uint32_t start_ms, uint32_t now_ms) {
    if (!SBOT_TRACE_POSE_WAIT_PROGRESS) return;
    if (!sbot_chassis) return;

    static uint32_t last_ms = 0;
    if (now_ms - last_ms < SBOT_TRACE_POSE_WAIT_PERIOD_MS) return;
    last_ms = now_ms;

    const auto pose = sbot_chassis->getPose(false, false);
    const auto pose_std = sbot_chassis->getPose(false, true);

    const double imu_heading = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;
    const double imu_rotation = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
    const double vert_in = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

    printf(
        "SBOT FOLLOW t=%u: pose(x=%.2f y=%.2f th=%.1f) std(x=%.2f y=%.2f th=%.1f) imu(h=%.1f r=%.1f) vert.in=%.2f\n",
        static_cast<unsigned>(now_ms - start_ms),
        pose.x,
        pose.y,
        pose.theta,
        pose_std.x,
        pose_std.y,
        pose_std.theta,
        imu_heading,
        imu_rotation,
        vert_in
    );
}

static void sbot_zero_pose_and_sensors(float x = 0, float y = 0, float theta_deg = 0) {
    if (!sbot_chassis) return;

    // IMPORTANT: Tare sensors FIRST so they start at zero, then let LemLib setPose handle the frame.
    // Do NOT manually set IMU heading/rotation after setPose - this creates a double-transform.
    if (sbot_inertial_sensor) {
        sbot_inertial_sensor->tare_rotation();
        sbot_inertial_sensor->tare_heading();
    }
    if (sbot_vertical_encoder) {
        sbot_vertical_encoder->reset_position();
    }
    if (sbot_vertical_tracking_wheel) {
        sbot_vertical_tracking_wheel->reset();
    }

    pros::delay(60);  // Let sensors settle after tare

    // Now set chassis pose - LemLib will internally manage IMU offset
    sbot_chassis->setPose(x, y, theta_deg);
}

static double sbot_norm_heading(double deg) {
    while (deg < 0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

// Convert target heading in pose frame to standard (IMU) frame for turnToHeading().
// The follow test starts with setPose(x, y, 90), creating a 90° offset between frames.
// Pose frame: 0°=+X, 90°=+Y (our logical coordinates)
// Std frame: 0°=+Y, 90°=+X (LemLib/IMU native)
// Conversion: std_heading = pose_heading - 90°
static double sbot_pose_to_imu_heading(double pose_heading_target_deg) {
    if (!sbot_chassis) return pose_heading_target_deg;
    return sbot_norm_heading(pose_heading_target_deg - 90.0);
}

static double sbot_heading_error_deg(double target_deg, double current_deg) {
    // Signed shortest-path error in degrees in [-180, 180).
    double err = sbot_norm_heading(target_deg) - sbot_norm_heading(current_deg);
    while (err < -180.0) err += 360.0;
    while (err >= 180.0) err -= 360.0;
    return err;
}

static double sbot_get_best_heading_deg() {
    // Prefer IMU heading for turn accuracy.
    if (sbot_inertial_sensor) return sbot_inertial_sensor->get_heading();
    if (sbot_chassis) return sbot_chassis->getPose().theta;
    return 0.0;
}

static double sbot_mirror_heading(double heading_deg) {
    // Mirror across the field centerline (LEFT <-> RIGHT).
    // With LemLib convention (+Y forward, +X right, 0° facing +Y), this is: heading -> 360 - heading.
    return sbot_norm_heading(360.0 - heading_deg);
}

struct SbotPoint {
    double x;
    double y;
};

// Convert a Jerry absolute field point (inches) into our internal start-relative frame.
// Jerry start is defined as the LemLib pose point at match start.
// Mapping (confirmed):
//   ourX = (Jy_start - Jy)
//   ourY = (Jx - Jx_start)
static SbotPoint sbot_from_jerry(double jerry_x, double jerry_y) {
    return {
        sbot_jerry_start_y - jerry_y,
        jerry_x - sbot_jerry_start_x,
    };
}

// Variant: only apply the "stall" early-exit when we are already near the target.
// This prevents cancelling mid-field motions due to transient odom dropouts/slip.
static void sbot_wait_until_done_or_stalled_near_target_timed(
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

        // Only consider "stalled" when we're close enough that contact/slip is expected.
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
            stalled ? " (stalled early-exit)" : ""
        );
    }
}

// Wait for a pose/heading motion to be "good enough" (distance + heading) after a minimum time,
// or until it completes, or until timeout.
// This avoids relying on LemLib waitUntilDone() semantics when a motion can appear to hang/stall.
static void sbot_wait_until_pose_close_or_timeout_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t min_time_ms,
    const SbotPoint& target,
    double close_dist_in,
    double target_heading_deg,
    double close_heading_deg
) {
    if (!sbot_chassis) return;

    const uint32_t start = pros::millis();
    bool closed = false;

    uint32_t last_trace_ms = start;
    auto last_trace_pose = sbot_chassis->getPose();
    const double last_trace_vert_in0 = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;
    double last_trace_vert_in = last_trace_vert_in0;

    // NOTE: LemLib's isInMotion() is not always a reliable indicator for motion completion.
    // If it reports false while the robot is still being commanded, we'd skip the loop and fail
    // to cancel the motion on timeout, which looks like "tiny wheel movements forever".
    while (pros::millis() - start < overall_timeout_ms) {
        pros::delay(10);
        const uint32_t now = pros::millis();

        const auto pose = sbot_chassis->getPose();
        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double hErr = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));

        if (SBOT_TRACE_POSE_WAIT_PROGRESS && (now - last_trace_ms >= SBOT_TRACE_POSE_WAIT_PERIOD_MS)) {
            const auto pose_now = pose;
            const double imu_heading = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;
            const double imu_rotation = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
            const double vert_in_now = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

            const double dpx = pose_now.x - last_trace_pose.x;
            const double dpy = pose_now.y - last_trace_pose.y;
            const double dp = std::sqrt(dpx * dpx + dpy * dpy);
            const double dvert = vert_in_now - last_trace_vert_in;

            printf(
                "SBOT WAIT TRACE [%s] t=%ums dist=%.2f hErr=%.2f pose(%.2f,%.2f,%.1f) dPose=%.2f vert.in=%.2f dVert=%.2f imu.h=%.1f imu.r=%.1f\n",
                label,
                static_cast<unsigned>(now - start),
                dist,
                hErr,
                pose_now.x,
                pose_now.y,
                pose_now.theta,
                dp,
                vert_in_now,
                dvert,
                imu_heading,
                imu_rotation
            );

            last_trace_ms = now;
            last_trace_pose = pose_now;
            last_trace_vert_in = vert_in_now;
        }

        if ((now - start >= min_time_ms) && (dist <= close_dist_in) && (hErr <= close_heading_deg)) {
            closed = true;
            sbot_chassis->cancelAllMotions();
            break;
        }

        // If LemLib says we're not in motion, don't spin forever waiting for a state change.
        // We still rely on the timeout to cancel any lingering command.
        if (!sbot_chassis->isInMotion() && (now - start >= min_time_ms)) {
            break;
        }
    }

    const uint32_t dur = pros::millis() - start;
    const bool timed_out = (dur >= overall_timeout_ms);
    if (timed_out) sbot_chassis->cancelAllMotions();

    // Debug proof: show how far we are from the target when we exit.
    const auto pose_end = sbot_chassis->getPose();
    const double dx_end = target.x - pose_end.x;
    const double dy_end = target.y - pose_end.y;
    const double dist_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    const double hErr_end = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));
    const bool in_motion_end = sbot_chassis->isInMotion();
    const bool close_end = (dist_end <= close_dist_in) && (hErr_end <= close_heading_deg);
    const bool ended_not_close = !close_end && !timed_out;

    if (SBOT_PRINT_WAIT_TIMES) {
        printf(
            "SBOT WAIT [%s]: %u ms%s%s%s endDist=%.2f endHErr=%.2f inMotion=%d\n",
            label,
            dur,
            (closed || close_end) ? " (close)" : "",
            timed_out ? " (timed out)" : "",
            ended_not_close ? " (ended not close)" : "",
            dist_end,
            hErr_end,
            in_motion_end ? 1 : 0
        );
    }
}

static double sbot_dist_in(const SbotPoint& a, const SbotPoint& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

static SbotPoint sbot_offset_forward(const SbotPoint& p, double heading_deg, double distance_in) {
    const double heading_rad = heading_deg * M_PI / 180.0;
    // LemLib convention: 0° faces +Y.
    const double fx = std::sin(heading_rad);
    const double fy = std::cos(heading_rad);
    return {p.x + fx * distance_in, p.y + fy * distance_in};
}

static SbotPoint sbot_pose_from_front_contact(const SbotPoint& contact, double heading_deg, double front_bumper_in) {
    // frontBumperPoint = pose + forward * front_bumper_in
    // => pose = contact - forward * front_bumper_in
    return sbot_offset_forward(contact, heading_deg, -front_bumper_in);
}

static SbotPoint sbot_pose_from_back_contact(const SbotPoint& contact, double heading_deg, double back_bumper_in) {
    // backBumperPoint = pose - forward * back_bumper_in
    // => pose = contact + forward * back_bumper_in
    return sbot_offset_forward(contact, heading_deg, back_bumper_in);
}

static SbotPoint sbot_mirror_point_y(const SbotPoint& p) {
    return {p.x, -p.y};
}

static SbotPoint sbot_mirror_point_x(const SbotPoint& p) {
    return {-p.x, p.y};
}

static bool sbot_drive_to(const SbotPoint& p, uint32_t timeout_ms, bool mirrored_y = false, bool forwards = true) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const SbotPoint target = mirrored_y ? sbot_mirror_point_y(p) : p;

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = SBOT_MATCH_MAX_SPEED;
    sbot_chassis->moveToPoint(target.x, target.y, timeout_ms, params);
    sbot_wait_until_done_timed("drive_to");
    return true;
}

static bool sbot_turn_to(double heading_deg, uint32_t timeout_ms, bool mirrored_y = false) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const double target_heading = mirrored_y ? sbot_mirror_heading(heading_deg) : sbot_norm_heading(heading_deg);
    lemlib::TurnToHeadingParams params;
    params.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
    params.minSpeed = 10;
    sbot_chassis->turnToHeading(target_heading, timeout_ms, params);
    sbot_wait_until_done_timed("turn_to");
    return true;
}

// Replace a "reach x/y and end at heading" action with a deterministic
// turn -> translate -> turn sequence.
static void sbot_turn_point_turn(
    const char* label,
    float target_x,
    float target_y,
    float target_heading_deg,
    uint32_t turn_timeout_ms,
    uint32_t drive_motion_timeout_ms,
    const lemlib::TurnToHeadingParams& turn_params,
    const lemlib::MoveToPointParams& drive_params,
    uint32_t drive_wait_timeout_ms = 0,
    uint32_t drive_min_time_ms = 0,
    double drive_close_dist_in = 0.0,
    double drive_close_heading_deg = 0.0,
    bool do_pre_turn = true,
    bool do_post_turn = true
) {
    if (!sbot_chassis) return;

    char stage_label[96];

    if (do_pre_turn) {
        std::snprintf(stage_label, sizeof(stage_label), "%s.pre_turn", label);
        sbot_chassis->turnToHeading(target_heading_deg, static_cast<int>(turn_timeout_ms), turn_params);
        sbot_wait_until_done_or_timed_out_timed(stage_label, turn_timeout_ms + 250);
    }

    std::snprintf(stage_label, sizeof(stage_label), "%s.drive", label);
    sbot_chassis->moveToPoint(target_x, target_y, static_cast<int>(drive_motion_timeout_ms), drive_params);
    if (drive_wait_timeout_ms > 0 && drive_close_dist_in > 0.0 && drive_close_heading_deg > 0.0) {
        sbot_wait_until_pose_close_or_timeout_timed(
            stage_label,
            drive_wait_timeout_ms,
            drive_min_time_ms,
            {target_x, target_y},
            drive_close_dist_in,
            target_heading_deg,
            drive_close_heading_deg
        );
    } else if (drive_wait_timeout_ms > 0) {
        sbot_wait_until_done_or_timed_out_timed(stage_label, drive_wait_timeout_ms);
    } else {
        sbot_wait_until_done_or_timed_out_timed(stage_label, drive_motion_timeout_ms + 250);
    }

    if (do_post_turn) {
        std::snprintf(stage_label, sizeof(stage_label), "%s.post_turn", label);
        sbot_chassis->turnToHeading(target_heading_deg, static_cast<int>(turn_timeout_ms), turn_params);
        sbot_wait_until_done_or_timed_out_timed(stage_label, turn_timeout_ms + 250);
    }
}

static bool sbot_drive_relative(double distance_in, uint32_t timeout_ms, bool forwards = true) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const auto pose = sbot_chassis->getPose();
    const double heading_rad = pose.theta * M_PI / 180.0;
    // LemLib odom convention: at theta=0°, robot faces +Y. (+X is right)
    const double dx = distance_in * std::sin(heading_rad);
    const double dy = distance_in * std::cos(heading_rad);

    // IMPORTANT:
    // - dx/dy represent the robot's *forward* direction in field coordinates.
    // - If we want to drive backwards while keeping the same heading, the target point must be
    //   behind the robot: pose - (dx,dy). (params.forwards=false)
    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = SBOT_MATCH_MAX_SPEED;
    sbot_chassis->moveToPoint(target_x, target_y, timeout_ms, params);
    sbot_wait_until_done_timed("drive_relative");
    return true;
}

static bool sbot_drive_relative_stall_exit(
    double distance_in,
    uint32_t motion_timeout_ms,
    bool forwards,
    uint32_t stall_window_ms = 300,
    double stall_epsilon_in = 0.35,
    int maxSpeed = SBOT_MATCH_MAX_SPEED
) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const auto pose = sbot_chassis->getPose();
    const double heading_rad = pose.theta * M_PI / 180.0;
    const double dx = distance_in * std::sin(heading_rad);
    const double dy = distance_in * std::cos(heading_rad);

    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = maxSpeed;

    sbot_chassis->moveToPoint(target_x, target_y, motion_timeout_ms, params);
    sbot_wait_until_done_or_stalled_timed("drive_relative_stall_exit", motion_timeout_ms, stall_window_ms, stall_epsilon_in);
    return true;
}

static void sbot_intake_on_storage() {
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    if (sbot_goal_flap) sbot_goal_flap->close();
    if (sbot_intake) sbot_intake->update();
    if (sbot_indexer) sbot_indexer->update();
}

static void sbot_score_mid_for(uint32_t ms) {
    if (!sbot_indexer) return;

    // Mimic driver helper behavior: intake forward assists while indexer reverses.
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    sbot_run_for_ms(ms);
    sbot_indexer->setMode(IndexerMode::OFF);
    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    sbot_run_for_ms(120);
}

static void sbot_score_low_for(uint32_t ms) {
    // Match driver behavior: low-goal scoring is intake reverse only.
    if (!sbot_intake) return;

    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
    sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    sbot_run_for_ms(ms);
    sbot_intake->setMode(IntakeMode::OFF);
    sbot_run_for_ms(120);
}

static void sbot_score_top_for(uint32_t ms) {
    if (!sbot_indexer) return;
    if (sbot_goal_flap) sbot_goal_flap->open();
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_run_for_ms(ms);
    sbot_safe_stop_mechanisms();
}

static void sbot_set_match_start_pose() {
    // Start pose definition (Phase 4+):
    // - Robot BACK bumper touching the black "park zone" strip at the far end of the alliance goal.
    // - Heading 0° points "away from the goal" into the field (+Y in LemLib convention).
    // This makes all coordinates in these routines local to the start placement.
    if (!sbot_chassis) return;
    sbot_zero_pose_and_sensors(0, 0, 0);
}

enum class SbotAutoSide {
    RIGHT = 0,
    LEFT
};

enum class SbotAutoAlliance {
    RED = 0,
    BLUE
};

static SbotPoint sbot_rotate180_point(const SbotPoint& p) {
    return {-p.x, -p.y};
}

static double sbot_rotate180_heading(double heading_deg) {
    return sbot_norm_heading(heading_deg + 180.0);
}

static SbotPoint sbot_apply_auto_transform(const SbotPoint& p, SbotAutoSide side, SbotAutoAlliance alliance) {
    SbotPoint out = p;
    if (alliance == SbotAutoAlliance::BLUE) {
        out = sbot_rotate180_point(out);
    }
    if (side == SbotAutoSide::LEFT) {
        // Mirror RIGHT <-> LEFT. With LemLib, X is right, so negate X.
        out = sbot_mirror_point_x(out);
    }
    return out;
}

static double sbot_apply_auto_transform_heading(double heading_deg, SbotAutoSide side, SbotAutoAlliance alliance) {
    double out = heading_deg;
    if (alliance == SbotAutoAlliance::BLUE) {
        out = sbot_rotate180_heading(out);
    }
    if (side == SbotAutoSide::LEFT) {
        out = sbot_mirror_heading(out);
    }
    return sbot_norm_heading(out);
}

// For our-side AWP routines, LEFT and RIGHT are not pure mirrors because goal types differ.
// We therefore define separate canonical tunings for Red Left and Red Right.
// Blue variants are obtained by a 180° rotation only.
static SbotPoint sbot_apply_alliance_transform_only(const SbotPoint& p, SbotAutoAlliance alliance) {
    return (alliance == SbotAutoAlliance::BLUE) ? sbot_rotate180_point(p) : p;
}

static double sbot_apply_alliance_transform_heading_only(double heading_deg, SbotAutoAlliance alliance) {
    return (alliance == SbotAutoAlliance::BLUE) ? sbot_rotate180_heading(heading_deg) : sbot_norm_heading(heading_deg);
}

static bool sbot_auto_drive_to(const SbotPoint& p, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance, bool forwards = true) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const SbotPoint target = sbot_apply_auto_transform(p, side, alliance);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    sbot_chassis->moveToPoint(target.x, target.y, timeout_ms, params);
    sbot_chassis->waitUntilDone();
    return true;
}

static bool sbot_auto_turn_to(double heading_deg, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const double target_heading = sbot_apply_auto_transform_heading(heading_deg, side, alliance);
    sbot_chassis->turnToHeading(target_heading, timeout_ms);
    sbot_chassis->waitUntilDone();
    return true;
}

struct SbotRRPathTuning {
    // Points are in a local, start-relative frame:
    // - Start pose is (0,0,0)
    // LemLib convention:
    // - +Y is "away from the goal" into the field (forward)
    // - +X is "to the robot's right" when facing into the field
    // These are first-pass guesses and should be tuned.

    // Step points
    SbotPoint step1;
    SbotPoint step2;
    SbotPoint step4;

    // Headings
    double step3_turn_heading_deg;

    // Timeouts
    uint32_t drive_timeout_ms;
    uint32_t turn_timeout_ms;

    // Scoring
    uint32_t top_score_ms;
};

static SbotRRPathTuning sbot_rr_default_tuning() {
    SbotRRPathTuning t;

    // Match the user sketch (Red Right):
    // 1) pull away from goal
    // 2) angle to the near cluster
    // 3) turn because intake and scoring are opposite sides
    // 4) return/approach goal for scoring
    // 5) score top goal
    // Previously authored as (forward, left). Convert to LemLib (right, forward):
    // new.x = -old.left
    // new.y = old.forward
    t.step1 = {0, 18};
    t.step2 = {-14, 40};
    t.step3_turn_heading_deg = 180; // turn around so scoring side leads
    t.step4 = {-4, 10};             // approach the goal/parking area (tune)

    t.drive_timeout_ms = 2500;
    t.turn_timeout_ms = 1600;
    t.top_score_ms = SBOT_MIN_SCORE_TIME_MS;
    return t;
}

static void sbot_run_red_right_1_to_5(SbotAutoSide side, SbotAutoAlliance alliance) {
    // Single source of truth for match autos. All 4 match autos call into here with transforms.
    // This follows the user's drawn steps 1→5 for Red Right; LEFT mirrors; BLUE is 180° rotated.
    printf("SBOT AUTON: MATCH AUTO RR-1to5 (%s %s)\n",
           (alliance == SbotAutoAlliance::RED) ? "RED" : "BLUE",
           (side == SbotAutoSide::RIGHT) ? "RIGHT" : "LEFT");

    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();
    sbot_set_match_start_pose();
    sbot_print_pose("start");

    const auto tune = sbot_rr_default_tuning();

    // Step 1: Start collecting immediately (store inside robot while driving out)
    printf("RR STEP 1\n");
    sbot_intake_on_storage();
    sbot_auto_drive_to(tune.step1, tune.drive_timeout_ms, side, alliance, true);
    sbot_print_pose("after step1");

    // Step 2: Continue collecting to the cluster
    printf("RR STEP 2\n");
    sbot_intake_on_storage();
    sbot_auto_drive_to(tune.step2, tune.drive_timeout_ms, side, alliance, true);
    sbot_print_pose("after step2");

    // Step 3: Turn so the scoring side is oriented correctly
    // (intake side and scoring side are opposite sides)
    printf("RR STEP 3\n");
    sbot_auto_turn_to(tune.step3_turn_heading_deg, tune.turn_timeout_ms, side, alliance);
    sbot_print_pose("after step3");

    // Step 4: Approach the goal/parking strip while keeping balls staged.
    // NOTE: Depending on how your mechanism scores (front vs back), you may flip `forwards`.
    printf("RR STEP 4\n");
    sbot_intake_on_storage();
    sbot_auto_drive_to(tune.step4, tune.drive_timeout_ms, side, alliance, false /* backwards */);
    sbot_print_pose("before score");

    // Step 5: Score top goal
    printf("RR STEP 5: TOP SCORE\n");
    sbot_score_top_for(tune.top_score_ms);
    sbot_print_pose("after top score");

    sbot_safe_stop_mechanisms();
    printf("SBOT AUTON: MATCH AUTO RR-1to5 complete\n");
}

static void sbot_run_match_auto(
    SbotAutoSide side,
    SbotAutoAlliance alliance,
    bool solo_awp,
    bool start_from_cluster_sweep = false,
    bool stop_after_stage2 = false,
    bool stage2_skip_pre_turn = false
) {
    // Match auto is currently focused on achieving our portion of the AWP tasks.
    // Tune the points in `sbot_awp_half_default_tuning()` on-field.
    struct SbotAwpHalfTuning {
        // All points are defined for RED LEFT canonical frame.
        // They are transformed for other alliances via mirror/rotation.
        // Frame is start-relative as described in `sbot_set_match_start_pose()`.

        // Stage 0: ensure we are not touching the park zone barrier
        double clear_barrier_in;

        // Stage 1: collect the nearby block cluster
        SbotPoint cluster1;                     // Target cluster position
        uint32_t cluster_collect_ms;            // Dwell time at cluster

        // Stage 2: Center Goal scoring
        // - (RED LEFT, BLUE RIGHT): Center Goal – Lower (front score)
        // - (RED RIGHT, BLUE LEFT): Center Goal – Middle (back score)
        SbotPoint low_goal_approach;            // Lower goal pose target
        double low_goal_heading_deg;            // Lower goal heading
        uint32_t low_goal_score_ms;             // Lower goal score duration
        bool use_low_goal_contact;              // Use contact point conversion
        SbotPoint low_goal_contact;             // Lower goal bumper contact point

        SbotPoint mid_goal_approach;            // Middle goal pose target
        double mid_goal_heading_deg;            // Middle goal heading
        uint32_t mid_goal_score_ms;             // Middle goal score duration
        bool use_mid_goal_contact;              // Use contact point conversion
        SbotPoint mid_goal_contact;             // Middle goal bumper contact point

        // Stage 3: Retreat after first score
        bool use_post_score_retreat_point;      // Use absolute retreat point
        SbotPoint post_score_retreat_point;     // Retreat endpoint (absolute pose)
        double tube_face_heading_deg;           // Heading to face loader

        // Stage 4: Loader pull
        double loader_down_extra_front_in;      // Extra loader protrusion when deployed
        SbotPoint tube1;                        // Loader pose target (fallback)
        uint32_t tube_pull_ms;                  // Loader pull duration
        bool use_tube1_contact;                 // Use contact point conversion
        SbotPoint tube1_contact;                // Loader bumper contact point

        // Stage 5: Long Goal scoring
        double high_goal_heading_deg;           // Long goal heading
        uint32_t high_goal_score_ms;            // Long goal score duration
        double high_goal_back_in_from_tube_in;  // Distance to back into goal from loader

        // Solo AWP: second loader pull
        SbotPoint tube2;                        // Second loader position (solo only)
        SbotPoint tube2_pulloff;                // Pulloff after second load (solo only)

        // Solo AWP: second cluster collection
        SbotPoint cluster2;                     // Second cluster position (solo AWP)
        uint32_t cluster2_collect_ms;           // Dwell time at second cluster
        
        // Solo AWP: second goal scoring (Center Middle from opposite side)
        SbotPoint mid_goal_solo_approach;       // Middle goal approach for solo (from cluster 2)
        double mid_goal_solo_heading_deg;       // Heading for solo middle goal (45° for back-score)
        bool use_mid_goal_solo_contact;         // Use contact point for solo middle
        SbotPoint mid_goal_solo_contact;        // Middle goal contact point (solo)

        // Timeouts
        uint32_t drive_timeout_ms;
        uint32_t turn_timeout_ms;
    };

    // Canonical tuning: RED LEFT is the source-of-truth.
    // Other starts are derived from this (mirror/rotate) as needed.
    auto sbot_awp_half_red_left_tuning = []() -> SbotAwpHalfTuning {
        SbotAwpHalfTuning t;

        // NOTE: These are conservative first-pass guesses.
        // Tune on a real field by logging pose prints and adjusting the points.
        // Conventions: +Y forward into field, +X to robot-right at 0°.

        // No obstacle: drive directly from start to the first cluster.
        t.clear_barrier_in = 0.0;

        // Cluster (RED LEFT) from Jerry field points.
        // Source of truth: Jerry cluster = (-24, 24)
        t.cluster1 = sbot_from_jerry(-24.0, 24.0);
        t.cluster_collect_ms = 150;

        // Center Goal – Lower (RED LEFT / BLUE RIGHT): from the cluster,
        // user-measured direction is forward-right about ~0.75 tile diagonally.
        // IMPORTANT: keep the robot on the same line from cluster -> goal for reliable scoring.
        const double center_lower_dx = 18.0;
        const double center_lower_dy = 18.0;
        const SbotPoint center_lower_approach = {t.cluster1.x + center_lower_dx, t.cluster1.y + center_lower_dy};

        // Center Goal – Middle: separate tuning (primarily for RED RIGHT / BLUE LEFT).
        const double center_middle_dx = 18;
        const double center_middle_dy = 18.0;
        const SbotPoint center_middle_approach = {t.cluster1.x - center_middle_dx, t.cluster1.y + center_middle_dy};

        // Center Goal – Lower approach (front-score).
        t.low_goal_approach = center_lower_approach;
        t.low_goal_heading_deg = 45;
        // Lower-goal scoring: add extra time to ensure balls fully clear.
        t.low_goal_score_ms = SBOT_LOW_GOAL_SCORE_TIME_MS + 750;
        // Use a measured front-bumper contact point for the Center Goal.
        // Source of truth (Jerry field coords, inches): (-9, 9)
        t.use_low_goal_contact = true;
        t.low_goal_contact = sbot_from_jerry(-9.0, 9.0);

        // Center Goal – Middle (back-score).
        t.mid_goal_approach = center_middle_approach;
        t.mid_goal_heading_deg = 180;
        t.mid_goal_score_ms = std::max<uint32_t>(SBOT_MID_GOAL_SCORE_TIME_MS, SBOT_MIN_SCORE_TIME_MS);
        // Measured back-bumper contact point for Center Goal – Middle.
        // Source of truth (Jerry field coords, inches): (-9, -9)
        t.use_mid_goal_contact = true;
        t.mid_goal_contact = sbot_from_jerry(-9.0, -9.0);

        // Stage 5: Long Goal scoring
        t.high_goal_heading_deg = 180;
        t.high_goal_score_ms = SBOT_MIN_SCORE_TIME_MS;
        // Back into long goal end from loader: drive to Jerry (-24, 48) then back in slightly.
        t.high_goal_back_in_from_tube_in = 24.0;

        // Force retreat to a measured absolute point (start-relative frame).
        // This point represents the robot pose point (drivetrain rotation center / "center" used by LemLib).
        t.use_post_score_retreat_point = true;
        // Source of truth: Jerry retreat = (-48, 48)
        t.post_score_retreat_point = sbot_from_jerry(-48.0, 48.0);

        // After retreat, turn to face alliance wall where the loader is.
        t.tube_face_heading_deg = 180;

        // Your measured loader protrusion when deployed.
        t.loader_down_extra_front_in = 6.0;

        // Loader (tube) pose points (fallback when not using contact points).
        t.tube1 = {-33, -11.0};
        t.tube_pull_ms = 1000;

        // Loader contact point (field feature, Jerry coords): (-73, 48).
        // This is where the FRONT of the robot/loader should contact the match loader.
        // Moved 2" closer to ensure full engagement.
        t.use_tube1_contact = true;
        t.tube1_contact = sbot_from_jerry(-73.0, 48.0);

        // Solo AWP Stage 6: Second cluster collection
        // Source of truth: Jerry cluster 2 = (24, 24)
        t.cluster2 = sbot_from_jerry(24.0, 24.0);
        t.cluster2_collect_ms = 150;

        // Solo AWP Stage 7: Center Middle Goal (back-score from opposite side)
        // Source of truth: Jerry goal = (9, 9), robot pose = (15, 15) for back bumper at goal
        t.use_mid_goal_solo_contact = true;
        t.mid_goal_solo_contact = sbot_from_jerry(9.0, 9.0);
        // Calculate pose target from contact point using back bumper
        const double mid_solo_heading = 45.0;
        t.mid_goal_solo_approach = sbot_pose_from_back_contact(t.mid_goal_solo_contact, mid_solo_heading, SBOT_BACK_BUMPER_IN);
        t.mid_goal_solo_heading_deg = mid_solo_heading;

        // Keep old tube2 for now (unused in new solo design)
        t.tube2 = {54, -24};
        t.tube2_pulloff = {-18, -18};

        // Timeouts: keep tight so we don't burn match time if something is slightly off.
        // We rely on pose-close exit thresholds to end motions quickly once we're in position.
        t.drive_timeout_ms = 2500;
        t.turn_timeout_ms = 1300;

        return t;
    };

    // Derived tuning: RED RIGHT is the Jerry-mirrored version of RED LEFT.
    // Per your convention: mirror across the Jerry X axis => Jerry Y is multiplied by -1.
    auto sbot_awp_half_red_right_tuning = [&]() -> SbotAwpHalfTuning {
        // Start from the same defaults as RL (timeouts, scoring times, etc).
        auto t = sbot_awp_half_red_left_tuning();

        // Replace Jerry-derived points with their mirrored Jerry counterparts.
        // NOTE: these calls depend on sbot_jerry_start_* having been set to the RR start.
        t.cluster1 = sbot_from_jerry(-24.0, -24.0);

        // Retreat point: (-48, 48) -> (-48, -48)
        t.use_post_score_retreat_point = true;
        t.post_score_retreat_point = sbot_from_jerry(-48.0, -48.0);

        // Center Goal contacts mirrored.
        if (t.use_low_goal_contact) t.low_goal_contact = sbot_from_jerry(-9.0, -9.0);
        if (t.use_mid_goal_contact) t.mid_goal_contact = sbot_from_jerry(-9.0, 9.0);

        // Tube contact mirrored: (-73, 48) -> (-73, -48)
        if (t.use_tube1_contact) t.tube1_contact = sbot_from_jerry(-73.0, -48.0);

        // Solo AWP: Mirror cluster2 and mid_goal_solo
        t.cluster2 = sbot_from_jerry(24.0, -24.0);  // (24, 24) -> (24, -24)
        if (t.use_mid_goal_solo_contact) {
            t.mid_goal_solo_contact = sbot_from_jerry(9.0, -9.0);  // (9, 9) -> (9, -9)
            const double mid_solo_heading = sbot_mirror_heading(45.0);  // 45° -> 135°
            t.mid_goal_solo_approach = sbot_pose_from_back_contact(t.mid_goal_solo_contact, mid_solo_heading, SBOT_BACK_BUMPER_IN);
            t.mid_goal_solo_heading_deg = mid_solo_heading;
        }

        // Mirror remaining internal-only geometry across the centerline.
        t.tube1 = sbot_mirror_point_x(t.tube1);
        t.low_goal_approach = sbot_mirror_point_x(t.low_goal_approach);
        t.mid_goal_approach = sbot_mirror_point_x(t.mid_goal_approach);
        t.tube2 = sbot_mirror_point_x(t.tube2);
        t.tube2_pulloff = sbot_mirror_point_x(t.tube2_pulloff);

        // Mirror headings.
        t.low_goal_heading_deg = sbot_mirror_heading(t.low_goal_heading_deg);
        t.mid_goal_heading_deg = sbot_mirror_heading(t.mid_goal_heading_deg);
        t.high_goal_heading_deg = sbot_mirror_heading(t.high_goal_heading_deg);
        t.tube_face_heading_deg = sbot_mirror_heading(t.tube_face_heading_deg);

        // Override Stage 2: RED RIGHT should use Center Goal – Middle (back-score).
        const double center_middle_dx = 0.0;
        const double center_middle_dy = 13.0;
        t.mid_goal_approach = {t.cluster1.x + center_middle_dx, t.cluster1.y + center_middle_dy};
        t.mid_goal_heading_deg = 180;

        // Keep Center-Lower distinct (not used in this path).
        const double center_lower_dx = 13.0;
        const double center_lower_dy = 13.0;
        t.low_goal_approach = {t.cluster1.x + center_lower_dx, t.cluster1.y + center_lower_dy};
        t.low_goal_heading_deg = 45;

        return t;
    };

    auto sbot_run_awp_half_field = [&](SbotAutoSide side_, SbotAutoAlliance alliance_, bool solo_) {
        const uint32_t auton_start_ms = pros::millis();
        printf("SBOT AUTON: %s (%s %s)\n",
               solo_ ? "SOLO AWP" : "AWP HALF",
               (alliance_ == SbotAutoAlliance::RED) ? "RED" : "BLUE",
               (side_ == SbotAutoSide::RIGHT) ? "RIGHT" : "LEFT");

        // Ensure run logs always prove which binary is deployed.
        printf("SBOT BUILD TAG: %s %s\n", __DATE__, __TIME__);

        if (!validateSbotLemLibInitialization()) return;

        // Match-auton drivetrain behavior.
        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        sbot_safe_stop_mechanisms();

        const bool low_goal_case =
            (alliance_ == SbotAutoAlliance::RED) ? (side_ == SbotAutoSide::LEFT) : (side_ == SbotAutoSide::RIGHT);

        // Select the Jerry start used for conversions + printing.
        // Canonical routes:
        // - low_goal_case => canonical RED LEFT geometry => RL Jerry start
        // - else          => canonical RED RIGHT geometry => RR Jerry start
        if (low_goal_case) {
            sbot_jerry_start_x = SBOT_JERRY_START_RL_X;
            sbot_jerry_start_y = SBOT_JERRY_START_RL_Y;
        } else {
            sbot_jerry_start_x = SBOT_JERRY_START_RR_X;
            sbot_jerry_start_y = SBOT_JERRY_START_RR_Y;
        }

        const auto t = low_goal_case ? sbot_awp_half_red_left_tuning() : sbot_awp_half_red_right_tuning();

        if (!start_from_cluster_sweep) {
            sbot_set_match_start_pose();
            sbot_print_pose("awp start");
        } else {
            // Focused test mode: assume we are starting from the end of the cluster sweep.
            // We reset odom/IMU to that known pose so Stage 2 runs identically to match auton.
            printf("SBOT AUTON TEST: start from cluster sweep completion\n");
            const SbotPoint sweep_pose = sbot_apply_alliance_transform_only(t.cluster1, alliance_);
            const double sweep_heading = sbot_apply_alliance_transform_heading_only(0.0, alliance_);
            sbot_zero_pose_and_sensors(sweep_pose.x, sweep_pose.y, sweep_heading);
            sbot_print_pose("test start (after sweep)");
        }

        auto drive_to = [&](const SbotPoint& p, bool forwards) {
            if (!sbot_chassis) return;
            const SbotPoint target = sbot_apply_alliance_transform_only(p, alliance_);
            lemlib::MoveToPointParams params;
            params.forwards = forwards;
            params.maxSpeed = SBOT_MATCH_MAX_SPEED;
            sbot_chassis->moveToPoint(target.x, target.y, t.drive_timeout_ms, params);
            sbot_wait_until_done_or_timed_out_timed("match.drive_to", t.drive_timeout_ms);
        };

        auto drive_to_speed = [&](const SbotPoint& p, bool forwards, int maxSpeed, const char* label) {
            if (!sbot_chassis) return;
            const SbotPoint target = sbot_apply_alliance_transform_only(p, alliance_);
            lemlib::MoveToPointParams params;
            params.forwards = forwards;
            params.maxSpeed = maxSpeed;
            sbot_chassis->moveToPoint(target.x, target.y, t.drive_timeout_ms, params);
            sbot_wait_until_done_or_timed_out_timed(label, t.drive_timeout_ms);
        };

        auto turn_to = [&](double heading_deg, int maxSpeed = SBOT_MATCH_TURN_MAX_SPEED, int minSpeed = 10) {
            if (!sbot_chassis) return;
            const double target_heading = sbot_apply_alliance_transform_heading_only(heading_deg, alliance_);
            lemlib::TurnToHeadingParams params;
            params.maxSpeed = maxSpeed;
            // Keep a small, non-zero minimum to prevent hanging near the end of the turn.
            params.minSpeed = minSpeed;
            sbot_chassis->turnToHeading(target_heading, t.turn_timeout_ms, params);
            sbot_wait_until_done_timed("match.turn_to");

            // Do NOT snap odom theta here.
            // Snapping hides real IMU error (log showed pose=180 while IMU=188),
            // and that causes tube/long-goal alignment to drift.
        };

        if (!start_from_cluster_sweep) {
            // Stage 0: optional barrier clearance.
            if (t.clear_barrier_in > 0.0) {
                printf("AWP STAGE 0: clear barrier\n");
                sbot_intake_on_storage();
                sbot_drive_relative(t.clear_barrier_in, 1200, true);
                sbot_print_pose("after clear barrier");
            }

            // Stage 1: collect nearby block cluster FIRST.
            printf("AWP STAGE 1: cluster collect\n");
            // Desired sequence:
            // 1) start approaching cluster WITHOUT intake (prevent spinning intake from hitting balls)
            // 2) deploy loader DURING forward motion (so it lands ON TOP of balls, trapping them)
            // 3) turn ON intake and continue sweep/collect
            // 4) retract + wait for retraction
            // 5) turn toward center goal
            if (!sbot_chassis) return;

            // 1) start approach to cluster WITHOUT intake running
            {
                const SbotPoint cluster_target = sbot_apply_alliance_transform_only(t.cluster1, alliance_);
                // We'll drive all the way to cluster, but deploy loader mid-drive
                const double cluster1_heading_deg = 0.0;
                const double cluster_heading = sbot_apply_alliance_transform_heading_only(cluster1_heading_deg, alliance_);

                lemlib::TurnToHeadingParams turnParams;
                turnParams.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
                turnParams.minSpeed = 0;

                lemlib::MoveToPointParams driveParams;
                driveParams.forwards = true;
                // Slower, controlled speed so loader can drop onto balls smoothly
                driveParams.maxSpeed = 45;
                driveParams.minSpeed = 0;
                driveParams.earlyExitRange = 0;

                // Start the motion (non-blocking)
                sbot_chassis->turnToHeading(cluster_heading, t.turn_timeout_ms, turnParams);
                sbot_chassis->waitUntilDone();
                
                sbot_chassis->moveToPoint(cluster_target.x, cluster_target.y, t.drive_timeout_ms, driveParams);
                
                // Deploy loader DURING the drive (delayed to account for faster pneumatic deployment)
                // Extra piston makes loader drop faster, so we wait even longer before deploying
                // This lets the loader land ON TOP of the balls as we drive over them
                pros::delay(550);
                if (sbot_batch_loader) {
                    sbot_batch_loader->extend();
                    printf("CLUSTER: loader deployed during approach\n");
                }
                
                // Shorter wait since loader descends faster now with extra piston
                pros::delay(100);
                
                // NOW turn on intake while still driving forward
                sbot_intake_on_storage();
                printf("CLUSTER: intake ON during approach\n");
                
                // Finish the motion
                sbot_wait_until_done_or_timed_out_timed("match.cluster.reach", t.drive_timeout_ms);
            }

            // 2) Small dwell at cluster to ensure collection
            // Keep the loader extended through the upcoming turn; we retract after the turn toward center goals.
            sbot_run_for_ms(t.cluster_collect_ms);

            // 4) collect dwell
            sbot_run_for_ms(t.cluster_collect_ms);

            sbot_print_pose("after cluster");
        } else {
            // Mirror the match state after Stage 1 so Stage 2 is identical.
            if (sbot_batch_loader) sbot_batch_loader->extend();
            sbot_intake_on_storage();
            sbot_print_pose("after cluster (test)" );
        }

        // Stage 2: score Center Goal – Lower (front) OR Center Goal – Middle (back)
        if (low_goal_case) {
            printf("AWP STAGE 2: CENTER LOWER (front score)\n");
            if (stage2_skip_pre_turn) {
                printf("AWP STAGE 2: skipping pre-turn (approach sequence handles turn+drive)\n");
                // If we skip the explicit pre-turn, retract before approaching the goal for safety.
                if (sbot_batch_loader) {
                    sbot_batch_loader->retract();
                    pros::delay(180);
                }
            } else {
                turn_to(t.low_goal_heading_deg);

                // Retract loader ONLY after the turn toward the center goal.
                if (sbot_batch_loader) {
                    sbot_batch_loader->retract();
                    pros::delay(180);
                }
            }
            sbot_intake_on_storage();
            // Drive to the computed goal pose target (absolute x/y). This is more robust than
            // projecting onto the heading line (which can clamp to 0" and result in no forward motion).
            if (sbot_chassis) {
                const double goal_heading = sbot_apply_alliance_transform_heading_only(t.low_goal_heading_deg, alliance_);
                SbotPoint target_canonical = t.low_goal_approach;
                SbotPoint target = sbot_apply_alliance_transform_only(target_canonical, alliance_);

                // Optional diagnostic: compare actual pose-derived bumper contact vs expected contact.
                bool has_expected_contact = false;
                SbotPoint expected_contact = {0, 0};
                if (t.use_low_goal_contact) {
                    const SbotPoint contact = sbot_apply_alliance_transform_only(t.low_goal_contact, alliance_);
                    const double front_effective = SBOT_FRONT_BUMPER_IN;
                    const double goal_heading_canonical = sbot_norm_heading(t.low_goal_heading_deg);
                    target_canonical = sbot_pose_from_front_contact(t.low_goal_contact, goal_heading_canonical, front_effective);
                    target = sbot_apply_alliance_transform_only(target_canonical, alliance_);

                    has_expected_contact = true;
                    expected_contact = contact;
                    printf(
                        "CENTER LOWER contact->pose: contact(%.2f,%.2f) heading=%.1f front=%.2f => pose(%.2f,%.2f)\n",
                        contact.x,
                        contact.y,
                        goal_heading,
                        front_effective,
                        target.x,
                        target.y
                    );
                }

                {
                    const auto pose0 = sbot_chassis->getPose();
                    const double dx = target.x - pose0.x;
                    const double dy = target.y - pose0.y;
                    printf(
                        "LOW GOAL drive_to: from(%.2f,%.2f,%.1f) to(%.2f,%.2f,%.1f) d=(%.2f,%.2f)\n",
                        pose0.x,
                        pose0.y,
                        pose0.theta,
                        target.x,
                        target.y,
                        goal_heading,
                        dx,
                        dy
                    );
                }
                sbot_print_jerry_target("low_goal_pose_target", target.x, target.y);
                // Use pose pursuit for center scoring.
                // Break out based on time + distance (and heading), rather than stall detection.
                {
                    lemlib::TurnToHeadingParams turnParams;
                    turnParams.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
                    turnParams.minSpeed = 0;

                    lemlib::MoveToPointParams driveParams;
                    driveParams.forwards = true;
                    driveParams.maxSpeed = SBOT_MATCH_MAX_SPEED;
                    // Keep minSpeed at 0 so LemLib uses the most accurate exit conditions.
                    // (Non-zero minSpeed can cause early-exit behavior that looks like "bailing".)
                    driveParams.minSpeed = 0;
                    driveParams.earlyExitRange = 0;

                    // IMPORTANT: give LemLib a longer internal timeout than our wait loop.
                    // Otherwise LemLib can stop the motion at exactly the wait timeout while still far away.
                    const uint32_t goal_wait_timeout_ms = 1900;
                    const uint32_t goal_motion_timeout_ms = 9000;

                    sbot_lemlib_debug_window_begin("match.approach_low_goal_pose");
                    sbot_turn_point_turn(
                        "match.approach_low_goal_pose",
                        target.x,
                        target.y,
                        goal_heading,
                        t.turn_timeout_ms,
                        goal_motion_timeout_ms,
                        turnParams,
                        driveParams,
                        goal_wait_timeout_ms,
                        400,
                        1.25,
                        6.0
                    );
                    sbot_lemlib_debug_window_end("match.approach_low_goal_pose");

                    // If LemLib ends the motion extremely early while still far away, retry once slower.
                    {
                        const auto pose_now = sbot_chassis->getPose();
                        const double dx = target.x - pose_now.x;
                        const double dy = target.y - pose_now.y;
                        const double dist = std::sqrt(dx * dx + dy * dy);
                        if (dist > 3.0) {
                            printf("LOW GOAL retry: dist still %.2f in\n", dist);
                            lemlib::MoveToPointParams retryDrive = driveParams;
                            retryDrive.maxSpeed = 90;
                            sbot_print_jerry_target("low_goal_pose_target.retry", target.x, target.y);

                            sbot_lemlib_debug_window_begin("match.approach_low_goal_pose.retry");
                            sbot_turn_point_turn(
                                "match.approach_low_goal_pose.retry",
                                target.x,
                                target.y,
                                goal_heading,
                                t.turn_timeout_ms,
                                5000,
                                turnParams,
                                retryDrive,
                                1100,
                                300,
                                1.25,
                                6.0
                            );
                            sbot_lemlib_debug_window_end("match.approach_low_goal_pose.retry");
                        }
                    }
                }

                // Give the intake/indexer a brief moment to finish pulling balls in before scoring.
                // Keep this tight for match timing; increase only if balls are consistently not fully loaded.
                sbot_run_for_ms(200);

                {
                    const auto pose1 = sbot_chassis->getPose();
                    const double dx1 = target.x - pose1.x;
                    const double dy1 = target.y - pose1.y;
                    const double dist1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
                    printf(
                        "LOW GOAL after turn+moveToPoint+turn: at(%.2f,%.2f,%.1f) err=(%.2f,%.2f) dist=%.2f\n",
                        pose1.x,
                        pose1.y,
                        pose1.theta,
                        dx1,
                        dy1,
                        dist1
                    );

                    if (has_expected_contact) {
                        // Estimate where the front bumper is, using the *measured* bumper offset and current heading.
                        const double h_deg = sbot_get_best_heading_deg();
                        const double h_rad = h_deg * M_PI / 180.0;
                        const double fx = std::sin(h_rad);
                        const double fy = std::cos(h_rad);
                        const SbotPoint est_contact = {pose1.x + fx * SBOT_FRONT_BUMPER_IN, pose1.y + fy * SBOT_FRONT_BUMPER_IN};
                        const double cdx = expected_contact.x - est_contact.x;
                        const double cdy = expected_contact.y - est_contact.y;
                        const double cdist = std::sqrt(cdx * cdx + cdy * cdy);
                        printf(
                            "LOW GOAL contact check: expected(%.2f,%.2f) est(%.2f,%.2f) d=(%.2f,%.2f) dist=%.2f heading=%.1f\n",
                            expected_contact.x,
                            expected_contact.y,
                            est_contact.x,
                            est_contact.y,
                            cdx,
                            cdy,
                            cdist,
                            h_deg
                        );
                    }
                }
            } else {
                drive_to(t.low_goal_approach, true /* forwards */);
            }

            // Ensure we spend at least 1s actively scoring.
            {
                const uint32_t requested_ms = std::max<uint32_t>(t.low_goal_score_ms, 1000);
                const uint32_t score_start_ms = pros::millis();
                sbot_score_low_for(requested_ms);
                const uint32_t score_dur_ms = pros::millis() - score_start_ms;
                sbot_low_goal_score_total_ms += score_dur_ms;
                sbot_low_goal_score_count += 1;
                printf(
                    "SBOT SCORE LOW: %u ms (requested %u ms, avg %u ms over %u runs)\n",
                    static_cast<unsigned>(score_dur_ms),
                    static_cast<unsigned>(requested_ms),
                    static_cast<unsigned>(sbot_low_goal_score_total_ms / sbot_low_goal_score_count),
                    static_cast<unsigned>(sbot_low_goal_score_count)
                );
            }
            sbot_print_auton_elapsed("low_goal_score_done");
            // Reasonable wait to let the last ball clear.
            pros::delay(200);
            sbot_print_pose("after center lower (front)");
            sbot_print_jerry_pose("after center lower (front)");
        } else {
            printf("AWP STAGE 2: CENTER MIDDLE (back score)\n");
            turn_to(t.mid_goal_heading_deg);

            // Retract loader ONLY after the turn toward the center goal.
            if (sbot_batch_loader) {
                sbot_batch_loader->retract();
                pros::delay(180);
            }
            sbot_intake_on_storage();
            // Confirmed: we want REAR facing the goal, so we back into the scoring spot.
            {
                const double goal_heading = sbot_apply_alliance_transform_heading_only(t.mid_goal_heading_deg, alliance_);
                SbotPoint mid_target_canonical = t.mid_goal_approach;
                SbotPoint mid_target = sbot_apply_alliance_transform_only(mid_target_canonical, alliance_);
                if (t.use_mid_goal_contact) {
                    const SbotPoint contact = sbot_apply_alliance_transform_only(t.mid_goal_contact, alliance_);
                    const double goal_heading_canonical = sbot_norm_heading(t.mid_goal_heading_deg);
                    mid_target_canonical = sbot_pose_from_back_contact(t.mid_goal_contact, goal_heading_canonical, SBOT_BACK_BUMPER_IN);
                    mid_target = sbot_apply_alliance_transform_only(mid_target_canonical, alliance_);
                    printf(
                        "CENTER MIDDLE contact->pose: contact(%.2f,%.2f) heading=%.1f back=%.2f => pose(%.2f,%.2f)\n",
                        contact.x,
                        contact.y,
                        goal_heading,
                        SBOT_BACK_BUMPER_IN,
                        mid_target.x,
                        mid_target.y
                    );
                }

                // Use pose pursuit so we converge x/y AND end square to the goal.
                {
                    lemlib::TurnToHeadingParams turnParams;
                    turnParams.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
                    turnParams.minSpeed = 0;

                    lemlib::MoveToPointParams driveParams;
                    driveParams.forwards = false; // back into the goal
                    driveParams.maxSpeed = SBOT_MATCH_MAX_SPEED;
                    driveParams.minSpeed = 0;
                    driveParams.earlyExitRange = 0;

                    const uint32_t goal_wait_timeout_ms = t.drive_timeout_ms;
                    const uint32_t goal_motion_timeout_ms = 9000;
                    {
                        const auto pose0 = sbot_chassis->getPose();
                        const double dx = mid_target.x - pose0.x;
                        const double dy = mid_target.y - pose0.y;
                        printf(
                                "MID GOAL turn+moveToPoint+turn: from(%.2f,%.2f,%.1f) to(%.2f,%.2f,%.1f) d=(%.2f,%.2f)\n",
                            pose0.x,
                            pose0.y,
                            pose0.theta,
                            mid_target.x,
                            mid_target.y,
                            goal_heading,
                            dx,
                            dy
                        );
                    }

                    sbot_print_jerry_target("mid_goal_pose_target", mid_target.x, mid_target.y);

                    sbot_lemlib_debug_window_begin("match.approach_mid_goal_pose");
                    sbot_turn_point_turn(
                        "match.approach_mid_goal_pose",
                        mid_target.x,
                        mid_target.y,
                        goal_heading,
                        t.turn_timeout_ms,
                        goal_motion_timeout_ms,
                        turnParams,
                        driveParams,
                        goal_wait_timeout_ms,
                        650,
                        0.5,
                        6.0
                    );
                    sbot_lemlib_debug_window_end("match.approach_mid_goal_pose");

                    {
                        const auto pose_now = sbot_chassis->getPose();
                        const double dx = mid_target.x - pose_now.x;
                        const double dy = mid_target.y - pose_now.y;
                        const double dist = std::sqrt(dx * dx + dy * dy);
                        if (dist > 3.0) {
                            printf("MID GOAL retry: dist still %.2f in\n", dist);
                            lemlib::MoveToPointParams retryDrive = driveParams;
                            retryDrive.maxSpeed = 85;
                            sbot_print_jerry_target("mid_goal_pose_target.retry", mid_target.x, mid_target.y);

                            sbot_lemlib_debug_window_begin("match.approach_mid_goal_pose.retry");
                            sbot_turn_point_turn(
                                "match.approach_mid_goal_pose.retry",
                                mid_target.x,
                                mid_target.y,
                                goal_heading,
                                t.turn_timeout_ms,
                                5000,
                                turnParams,
                                retryDrive,
                                1800,
                                450,
                                0.5,
                                6.0
                            );
                            sbot_lemlib_debug_window_end("match.approach_mid_goal_pose.retry");
                        }
                    }
                }

                // Give the intake/indexer a brief moment to finish pulling balls in before scoring.
                sbot_run_for_ms(200);

                {
                    const auto pose1 = sbot_chassis->getPose();
                    const double dx = mid_target.x - pose1.x;
                    const double dy = mid_target.y - pose1.y;
                    const double dist = std::sqrt(dx * dx + dy * dy);
                    printf(
                        "MID GOAL after turn+moveToPoint+turn: at(%.2f,%.2f,%.1f) err=(%.2f,%.2f) dist=%.2f\n",
                        pose1.x,
                        pose1.y,
                        pose1.theta,
                        dx,
                        dy,
                        dist
                    );
                }
            }
            // Ensure we spend at least 1s actively scoring.
            sbot_score_mid_for(std::max<uint32_t>(t.mid_goal_score_ms, 1000));
            sbot_print_pose("after center middle (back)");
        }

        if (stop_after_stage2) {
            sbot_safe_stop_mechanisms();
            printf("SBOT AUTON TEST: sweep->center score complete\n");
            return;
        }

        // Stage 3: retreat then face loader
        printf("AWP STAGE 3: retreat + face loader\n");
        sbot_safe_stop_mechanisms();
        // Retreat: either to an absolute point (preferred for RL non-solo), or straight back-out.
        if (t.use_post_score_retreat_point) {
            // Do NOT turn here. Back straight to the retreat point, then turn at the retreat.
            const SbotPoint retreat = sbot_apply_alliance_transform_only(t.post_score_retreat_point, alliance_);
            printf("RETREAT target: (%.2f, %.2f)\n", retreat.x, retreat.y);
            if (sbot_chassis) {
                const double retreat_heading = sbot_get_best_heading_deg();
                lemlib::TurnToHeadingParams turnParams;
                turnParams.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
                turnParams.minSpeed = 0;

                lemlib::MoveToPointParams driveParams;
                driveParams.forwards = false;
                driveParams.maxSpeed = 60;  // Slowed from 95 for better alignment
                driveParams.minSpeed = 0;
                driveParams.earlyExitRange = 0;

                sbot_turn_point_turn(
                    "match.retreat",
                    retreat.x,
                    retreat.y,
                    static_cast<float>(retreat_heading),
                    t.turn_timeout_ms,
                    t.drive_timeout_ms,
                    turnParams,
                    driveParams,
                    t.drive_timeout_ms,
                    300,
                    1.25,
                    12.0
                    , false /*do_pre_turn*/
                    , false /*do_post_turn*/
                );

                // If we are still noticeably off the retreat point, retry once slower.
                {
                    const auto pose_now = sbot_chassis->getPose();
                    const SbotPoint now{pose_now.x, pose_now.y};
                    if (sbot_dist_in(now, retreat) > 2.0) {
                        const double retry_heading = sbot_get_best_heading_deg();
                        lemlib::MoveToPointParams retryDrive = driveParams;
                        retryDrive.maxSpeed = 50;  // Slowed from 75 for better alignment
                        sbot_turn_point_turn(
                            "match.retreat.retry",
                            retreat.x,
                            retreat.y,
                            static_cast<float>(retry_heading),
                            t.turn_timeout_ms,
                            t.drive_timeout_ms,
                            turnParams,
                            retryDrive,
                            t.drive_timeout_ms,
                            300,
                            1.25,
                            12.0
                        );
                    }
                }
            } else {
                // If chassis isn't available (shouldn't happen), prefer the same "backwards" intent.
                drive_to(t.post_score_retreat_point, false /* backwards */);
            }
        } else {
            // Fallback: always use retreat point (post_score_retreat_back_dist_in removed).
            drive_to(t.post_score_retreat_point, false /* backwards */);
        }

        // Diagnose retreat accuracy *before* the face-loader turn (the turn can translate the robot if it pivots on contact).
        sbot_print_pose("after retreat (pre-turn)");
        sbot_print_jerry_pose("after retreat (pre-turn)");

        // Turn to face the tube/loader.
        // Single-turn, but tube/loader alignment is sensitive.
        // Use HOLD during the turn to prevent coasting a few degrees after the controller exits.
        if (sbot_chassis) {
            const double target_heading = sbot_apply_alliance_transform_heading_only(t.tube_face_heading_deg, alliance_);
            const auto pose0 = sbot_chassis->getPose();
            const double imu0 = sbot_get_best_heading_deg();
            printf(
                "FACE LOADER turn_to: canonical=%.1f transformed=%.1f startPose.th=%.2f imu=%.2f\n",
                t.tube_face_heading_deg,
                target_heading,
                pose0.theta,
                imu0
            );
        }
        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        // Slower turn for better alignment with loader.
        turn_to(t.tube_face_heading_deg, SBOT_MATCH_TURN_MAX_SPEED / 2, 10);
        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
        if (sbot_chassis) {
            const double target_heading = sbot_apply_alliance_transform_heading_only(t.tube_face_heading_deg, alliance_);
            const auto pose1 = sbot_chassis->getPose();
            const double imu1 = sbot_get_best_heading_deg();
            const double err_pose = sbot_heading_error_deg(target_heading, pose1.theta);
            const double err_imu = sbot_heading_error_deg(target_heading, imu1);
            printf(
                "FACE LOADER after turn: pose.th=%.2f imu=%.2f errPose=%.2f errImu=%.2f\n",
                pose1.theta,
                imu1,
                err_pose,
                err_imu
            );
        }

        // Deploy the match loader AFTER the face-loader turn so the pneumatic impulse doesn't disturb heading.
        if (low_goal_case && sbot_batch_loader) {
            sbot_batch_loader->extend();
            // Wait longer for loader to fully deploy before approaching tube.
            // The loader needs time to descend completely before we drive forward.
            pros::delay(400);
        }
        // Small settle after turning to face the tube/loader.
        pros::delay(20);
        sbot_print_pose("after retreat/turn");
        sbot_print_sensors("after retreat/turn");

        // Stage 4: loader pull
        printf("AWP STAGE 4: loader1 pull\n");
        sbot_intake_on_storage();
        if (low_goal_case) {
            // Loader is deployed at the end of Stage 3 (after facing it).
            pros::delay(60);

            // Red Left (and Blue Right): tube/long-goal are on the same X line.
            if (sbot_chassis) {
                // Time/robustness: skip the explicit X-line alignment step.
                // It can add lateral scrub (odom drift) and costs time; we instead go straight to the tube contact-derived pose.

                const double tube_heading = sbot_apply_alliance_transform_heading_only(t.tube_face_heading_deg, alliance_);
                SbotPoint tube_pose_target = sbot_apply_alliance_transform_only(t.tube1, alliance_);
                if (t.use_tube1_contact) {
                    const SbotPoint tube_contact = sbot_apply_alliance_transform_only(t.tube1_contact, alliance_);
                    const double front_effective = SBOT_FRONT_BUMPER_IN + t.loader_down_extra_front_in;
                    tube_pose_target = sbot_pose_from_front_contact(tube_contact, tube_heading, front_effective);
                    printf(
                        "TUBE contact->pose: contact(%.2f,%.2f) heading=%.1f frontEff=%.2f => pose(%.2f,%.2f)\n",
                        tube_contact.x,
                        tube_contact.y,
                        tube_heading,
                        front_effective,
                        tube_pose_target.x,
                        tube_pose_target.y
                    );
                } else {
                    printf(
                        "TUBE pose target: (%.2f,%.2f) heading=%.1f\n",
                        tube_pose_target.x,
                        tube_pose_target.y,
                        tube_heading
                    );
                }

                // Use moveToPose for tube approach to maintain straight heading.
                // We already turned to face the tube in Stage 3.
                {
                    const auto pose0 = sbot_chassis->getPose();
                    const double dx = tube_pose_target.x - pose0.x;
                    const double dy = tube_pose_target.y - pose0.y;
                    printf(
                        "TUBE moveToPose: from(%.2f,%.2f,%.1f) to(%.2f,%.2f,%.1f) d=(%.2f,%.2f)\n",
                        pose0.x,
                        pose0.y,
                        pose0.theta,
                        tube_pose_target.x,
                        tube_pose_target.y,
                        tube_heading,
                        dx,
                        dy
                    );
                }
                {
                    const uint32_t tube_wait_timeout_ms = 900;
                    const uint32_t tube_motion_timeout_ms = 2200;

                    lemlib::MoveToPoseParams poseParams;
                    poseParams.forwards = true;
                    poseParams.maxSpeed = SBOT_MATCH_MAX_SPEED;
                    poseParams.minSpeed = 0;

                    sbot_print_jerry_target("tube_pose_target", tube_pose_target.x, tube_pose_target.y);

                    sbot_lemlib_debug_window_begin("match.approach_tube_pose");
                    sbot_chassis->moveToPose(tube_pose_target.x, tube_pose_target.y, tube_heading, tube_motion_timeout_ms, poseParams);
                    sbot_wait_until_done_or_timed_out_timed("match.approach_tube_pose", tube_wait_timeout_ms);
                    sbot_lemlib_debug_window_end("match.approach_tube_pose");

                    {
                        const auto pose_now = sbot_chassis->getPose();
                        const double dx = tube_pose_target.x - pose_now.x;
                        const double dy = tube_pose_target.y - pose_now.y;
                        const double dist = std::sqrt(dx * dx + dy * dy);
                        if (dist > 3.0) {
                            printf("TUBE retry: dist still %.2f in\n", dist);
                            lemlib::MoveToPointParams retryDrive = driveParams;
                            retryDrive.maxSpeed = 90;
                            sbot_print_jerry_target("tube_pose_target.retry", tube_pose_target.x, tube_pose_target.y);

                            sbot_lemlib_debug_window_begin("match.approach_tube_pose.retry");
                            sbot_turn_point_turn(
                                "match.approach_tube_pose.retry",
                                tube_pose_target.x,
                                tube_pose_target.y,
                                tube_heading,
                                t.turn_timeout_ms,
                                5000,
                                turnParams,
                                retryDrive,
                                1100,
                                250,
                                1.0,
                                8.0
                            );
                            sbot_lemlib_debug_window_end("match.approach_tube_pose.retry");
                        }
                    }
                }

                {
                    const auto pose1 = sbot_chassis->getPose();
                    const double dx = tube_pose_target.x - pose1.x;
                    const double dy = tube_pose_target.y - pose1.y;
                    printf(
                        "TUBE after turn+moveToPoint+turn: at(%.2f,%.2f,%.1f) err=(%.2f,%.2f)\n",
                        pose1.x,
                        pose1.y,
                        pose1.theta,
                        dx,
                        dy
                    );
                }

                // Physical tuning: seat into the match loader a bit more.
                // If we're ~2" short, this small forward nudge (while facing the loader) helps ensure engagement.
                printf("TUBE seat push: +2in\n");
                sbot_drive_relative_stall_exit(2.0, 900, true /* forwards */, 250, 0.20, 55);
                
                // Drive two more inches into tube and hold position firmly
                printf("TUBE extra seat: +2in with HOLD\n");
                if (sbot_chassis) {
                    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
                    sbot_drive_relative(2.0, 800, true /* forwards */);
                    // Keep HOLD active during loader pull
                }
            }

            // Pull from the Loader while intaking + actuate batch loader.
            sbot_run_for_ms(t.tube_pull_ms);
            if (sbot_batch_loader) sbot_batch_loader->retract();
            pros::delay(60);
            
            // Return to BRAKE mode after loader pull
            if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

            // Pull off the loader so we can back into the long goal cleanly.
            // No extra pulloff drive: rely on the next motion to clear cleanly.
            sbot_print_pose("after loader1 (relative)");
            sbot_print_jerry_pose("after loader1 (relative)");
        } else {
            // Fallback for other starts (tube1_pulloff removed - just use tube1).
            drive_to(t.tube1, true);
            sbot_run_for_ms(t.tube_pull_ms);
            sbot_print_pose("after loader1");
        }

        if (solo_) {
            // Solo AWP Stage 6: Collect second cluster
            printf("AWP SOLO STAGE 6: cluster 2 collect\n");
            
            // Drive to cluster 2 (forward motion, similar to cluster 1 approach)
            {
                const SbotPoint cluster2_target = sbot_apply_alliance_transform_only(t.cluster2, alliance_);
                const double cluster2_heading_deg = 0.0;  // Face forward toward cluster
                const double cluster2_heading = sbot_apply_alliance_transform_heading_only(cluster2_heading_deg, alliance_);

                lemlib::TurnToHeadingParams turnParams;
                turnParams.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
                turnParams.minSpeed = 0;

                lemlib::MoveToPointParams driveParams;
                driveParams.forwards = true;
                driveParams.maxSpeed = 45;
                driveParams.minSpeed = 0;
                driveParams.earlyExitRange = 0;

                // Start the motion
                sbot_chassis->turnToHeading(cluster2_heading, t.turn_timeout_ms, turnParams);
                sbot_chassis->waitUntilDone();
                
                sbot_chassis->moveToPoint(cluster2_target.x, cluster2_target.y, t.drive_timeout_ms, driveParams);
                
                // Deploy loader during approach (same as cluster 1)
                pros::delay(200);
                if (sbot_batch_loader) {
                    sbot_batch_loader->extend();
                    printf("CLUSTER 2: loader deployed during approach\n");
                }
                
                pros::delay(150);
                sbot_intake_on_storage();
                printf("CLUSTER 2: intake ON during approach\n");
                
                sbot_wait_until_done_or_timed_out_timed("solo.cluster2.reach", t.drive_timeout_ms);
            }

            // Dwell to collect
            sbot_run_for_ms(t.cluster2_collect_ms);
            sbot_print_pose("after cluster 2");

            // Solo AWP Stage 7: Score Center Middle Goal (back-score)
            printf("AWP SOLO STAGE 7: CENTER MIDDLE (back score)\n");
            
            // Turn to 45° to face back toward middle goal
            turn_to(t.mid_goal_solo_heading_deg);
            
            // Retract loader for scoring
            if (sbot_batch_loader) {
                sbot_batch_loader->retract();
                pros::delay(180);
            }
            
            // Back into middle goal using contact point
            if (sbot_chassis && t.use_mid_goal_solo_contact) {
                const double goal_heading = sbot_apply_alliance_transform_heading_only(t.mid_goal_solo_heading_deg, alliance_);
                SbotPoint target = sbot_apply_alliance_transform_only(t.mid_goal_solo_approach, alliance_);
                
                printf("SOLO MID GOAL: heading=%.1f target=(%.2f,%.2f)\n", goal_heading, target.x, target.y);
                
                lemlib::MoveToPointParams params;
                params.forwards = false;  // Backing in
                params.maxSpeed = SBOT_MATCH_MAX_SPEED;
                sbot_chassis->moveToPoint(target.x, target.y, t.drive_timeout_ms, params);
                sbot_wait_until_done_or_timed_out_timed("solo.mid_goal.approach", t.drive_timeout_ms);
            }
            
            // Score at middle goal
            sbot_score_mid_for(t.mid_goal_score_ms);
            sbot_print_pose("after solo center middle");
            
            // Done with Solo AWP
            sbot_safe_stop_mechanisms();
            printf("SBOT AUTON: SOLO AWP complete\n");
            return;
        }

        // Stage 5: second score
        // Desired flow (short travel): cluster -> Center (Lower/Middle) -> loader(s) -> near end of Long Goal.
        printf("AWP STAGE 5: second score\n");
        printf("AWP STAGE 5: LONG GOAL END (near loader)%s\n", solo_ ? " (solo)" : "");

        if (low_goal_case) {
            // Red Left (and Blue Right): Back into long goal at Jerry (-31, 48).
            // This positions the SCORER (back of robot) at the goal, not just the center point.
            // From loader at Jerry (-58, 48), we drive backwards toward the goal (using moveToPoint).
            sbot_intake_on_storage();
            const SbotPoint long_goal_end_canonical = sbot_from_jerry(-31.0, 48.0);
            printf("LONG GOAL end: canonical(%.2f,%.2f) Jerry(-31.0,48.0)\n",
                   long_goal_end_canonical.x, long_goal_end_canonical.y);
            
            // Faster backwards approach to long goal
            if (sbot_chassis) {
                const SbotPoint target = sbot_apply_alliance_transform_only(long_goal_end_canonical, alliance_);
                const double target_heading = sbot_apply_alliance_transform_heading_only(180.0, alliance_);
                lemlib::MoveToPoseParams params;
                params.forwards = false;
                params.maxSpeed = 90;
                params.minSpeed = 0;
                
                sbot_chassis->moveToPose(target.x, target.y, target_heading, t.drive_timeout_ms, params);
                sbot_wait_until_done_or_timed_out_timed("match.long_goal_approach", t.drive_timeout_ms);
            } else {
                drive_to(long_goal_end_canonical, false /* backwards */);
            }
            
            // Final push into goal with stall detection
            sbot_drive_relative_stall_exit(4.0, 1500, false /* backwards */, 300, 0.35, 40);
        } else if (t.high_goal_back_in_from_tube_in > 0.0) {
            // We just finished loader pulling while facing the loader.
            // Backing up keeps the intake facing the loader and puts the rear into the Long Goal end.
            sbot_intake_on_storage();
            sbot_drive_relative_stall_exit(t.high_goal_back_in_from_tube_in, 4000, false /* backwards */, 300, 0.35, 80);
        } else {
            // Fallback: use relative drive (high_goal_approach removed).
            turn_to(t.high_goal_heading_deg);
            sbot_intake_on_storage();
            sbot_drive_relative_stall_exit(24.0, 4000, false /* backwards */, 300, 0.35, 80);
        }

        // Ensure we spend at least 1s actively scoring.
        sbot_score_top_for(std::max<uint32_t>(t.high_goal_score_ms, 1000));
        sbot_print_pose("after high goal");
        sbot_print_jerry_pose("after high goal");

        // Stage 6: ensure final position is clear of park-zone barrier.
        printf("AWP STAGE 6: end safe\n");
        sbot_safe_stop_mechanisms();
        // For now: do NOT drive back toward center. Stay at the long goal end.
        sbot_print_pose("end safe (no move)");

        sbot_safe_stop_mechanisms();
        printf("SBOT AUTON: %s complete\n", solo_ ? "SOLO AWP" : "AWP HALF");
    };

    sbot_run_awp_half_field(side, alliance, solo_awp);
}

static void sbot_run_skills_auto() {
    printf("SBOT AUTON: SKILLS (first-pass)\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();
    sbot_set_match_start_pose();
    sbot_print_pose("skills start");

    const uint32_t start_ms = pros::millis();
    const uint32_t hard_stop_ms = 55000; // leave time buffer

    int cycle = 0;
    while (pros::millis() - start_ms < hard_stop_ms) {
        cycle++;
        printf("SBOT SKILLS: cycle %d\n", cycle);

        // Collect run
        sbot_intake_on_storage();
        sbot_drive_to({0, 36}, 3000, false);

        // Score mid
        sbot_turn_to(90, 1500, false);
        sbot_drive_relative(8, 1200, true);
        sbot_score_mid_for(SBOT_MID_GOAL_SCORE_TIME_MS);

        // Go "back" to start area
        sbot_turn_to(270, 1500, false);
        sbot_drive_to({0, 0}, 3000, false, false);

        // Small top feed attempt
        sbot_turn_to(0, 1500, false);
        sbot_drive_relative(10, 1500, true);
        sbot_score_top_for(750);

        // Reset for next cycle
        sbot_turn_to(180, 1500, false);
        sbot_drive_relative(10, 1500, true);
        sbot_turn_to(0, 1500, false);
        sbot_safe_stop_mechanisms();

        // Prevent tight looping if time is nearly up
        pros::delay(100);
    }

    sbot_safe_stop_mechanisms();
    printf("SBOT AUTON: SKILLS complete\n");
}

static const char* sbot_mode_name(int idx) {
    static const char* mode_names[] = {
        "DISABLED",     // 0
        "Red Left",     // 1
        "Red Right",    // 2
        "Blue Left",    // 3
        "Blue Right",   // 4
        "Red Left (Solo AWP)",   // 5
        "Red Right (Solo AWP)",  // 6
        "Blue Left (Solo AWP)",  // 7
        "Blue Right (Solo AWP)", // 8
        "Skills",                // 9
        "Test: Sweep->Low Goal", // 10
        "Test: Drive",           // 11
        "Test: Turn",            // 12
        "Test: Intake",          // 13
        "Test: Indexer",         // 14
        "Test: Drive Short", // 15
        "Test: LowGoal (custom start)", // 16
        "Test: Pose Monitor (x,y,th)", // 17
        "Test: Follow Path (LemLib follow)", // 18
        "Test: Pose Finder (x0 line, 90deg)" // 19
    };

    if (idx < 0 || idx > 19) return "<invalid>";
    return mode_names[idx];
}

// =========================== Selector ===============================

SbotAutoSelector::SbotAutoSelector()
    : selected_mode(SbotAutoMode::DISABLED),
      selector_position(0),
      mode_confirmed(false) {}

void SbotAutoSelector::displayOptions() {
    // Display on controller screen (legacy project behavior)
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (!master.is_connected()) return;

    const int idx = selector_position;
    const char* name = sbot_mode_name(idx);

    if (mode_confirmed) {
        master.print(0, 0, "READY: %s", name);
        master.print(1, 0, "A: change  L/R: nav");
    } else {
        master.print(0, 0, "%d: %s", idx, name);
        master.print(1, 0, "L/R: change  A: ok");
    }
}

void SbotAutoSelector::handleInput() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    static int last_pos = -1;
    static bool last_confirmed = false;

    bool left_pressed = master.get_digital_new_press(SBOT_AUTO_PREV_BTN);
    bool right_pressed = master.get_digital_new_press(SBOT_AUTO_NEXT_BTN);
    bool a_pressed = master.get_digital_new_press(SBOT_AUTO_CONFIRM_BTN);

    const int max_index = 19; // 0..19

    if (!mode_confirmed) {
        if (left_pressed) {
            selector_position--;
            if (selector_position < 0) selector_position = max_index;
        }
        if (right_pressed) {
            selector_position++;
            if (selector_position > max_index) selector_position = 0;
        }
        if (a_pressed) {
            selected_mode = static_cast<SbotAutoMode>(selector_position);
            mode_confirmed = true;
        }
    } else {
        if (a_pressed) {
            mode_confirmed = false; // allow reselection
        }
    }

    // Print only on change to avoid spamming the terminal.
    if (selector_position != last_pos || mode_confirmed != last_confirmed) {
        last_pos = selector_position;
        last_confirmed = mode_confirmed;

        if (mode_confirmed) {
            printf("SBOT AUTO: READY %d (%s)\n", selector_position, sbot_mode_name(selector_position));
        } else {
            printf("SBOT AUTO: select %d (%s) [L/R to change, A to confirm]\n",
                   selector_position,
                   sbot_mode_name(selector_position));
        }
    }
}

bool SbotAutoSelector::update() {
    handleInput();
    displayOptions();
    return mode_confirmed;
}

// ======================= Autonomous System ==========================

SbotAutonomousSystem::SbotAutonomousSystem() {}

void SbotAutonomousSystem::initialize() {
    // Initialize LemLib for sbot (safe to call once)
    initializeSbotLemLib();
}

void SbotAutonomousSystem::updateSelector() {
    selector.update();
}

void SbotAutonomousSystem::run() {
    SbotAutoMode mode = selector.getMode();

    const bool should_time = (mode != SbotAutoMode::DISABLED);
    const uint32_t auton_total_start_ms = should_time ? pros::millis() : 0;
    if (should_time) {
        sbot_auton_elapsed_active = true;
        sbot_auton_elapsed_start_ms = auton_total_start_ms;
        sbot_print_auton_elapsed("auton_start");
    }

    switch (mode) {
        case SbotAutoMode::RED_LEFT:   runRedLeft();   break;
        case SbotAutoMode::RED_RIGHT:  runRedRight();  break;
        case SbotAutoMode::BLUE_LEFT:  runBlueLeft();  break;
        case SbotAutoMode::BLUE_RIGHT: runBlueRight(); break;
        case SbotAutoMode::RED_LEFT_SOLO_AWP:   sbot_run_match_auto(SbotAutoSide::LEFT,  SbotAutoAlliance::RED,  true); break;
        case SbotAutoMode::RED_RIGHT_SOLO_AWP:  sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED,  true); break;
        case SbotAutoMode::BLUE_LEFT_SOLO_AWP:  sbot_run_match_auto(SbotAutoSide::LEFT,  SbotAutoAlliance::BLUE, true); break;
        case SbotAutoMode::BLUE_RIGHT_SOLO_AWP: sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::BLUE, true); break;
        case SbotAutoMode::SKILLS:     runSkills();    break;
        case SbotAutoMode::TEST_SWEEP_TO_LOW_GOAL: runTestSweepToLowGoal(); break;
        case SbotAutoMode::TEST_DRIVE:   runTestDrive();   break;
        case SbotAutoMode::TEST_DRIVE_SHORT: runTestDriveShort(); break;
        case SbotAutoMode::TEST_LOW_GOAL_CUSTOM_START: runTestLowGoalCustomStart(); break;
        case SbotAutoMode::TEST_JERRY_POSE_MONITOR: runTestJerryPoseMonitor(); break;
        case SbotAutoMode::TEST_FOLLOW_JERRY_PATH: runTestFollowJerryPath(); break;
        case SbotAutoMode::TEST_POSE_FINDER_X0_LINE_90: runTestPoseFinderX0Line90(); break;
        case SbotAutoMode::TEST_TURN:    runTestTurn();    break;
        case SbotAutoMode::TEST_INTAKE:  runTestIntake();  break;
        case SbotAutoMode::TEST_INDEXER: runTestIndexer(); break;
        case SbotAutoMode::DISABLED:
        default:
            // Do nothing
            break;
    }

    if (should_time) {
        const uint32_t dur_ms = pros::millis() - auton_total_start_ms;
        printf("SBOT AUTON TOTAL: %u ms (%.2f s)\n", static_cast<unsigned>(dur_ms), dur_ms / 1000.0);
        sbot_auton_elapsed_active = false;
    }
}

void SbotAutonomousSystem::runTestJerryPoseMonitor() {
    // Manual calibration helper: push/rotate the robot by hand and watch odometry.
    // Prints pose (x,y,theta) and IMU heading on the controller screen.
    printf("SBOT AUTON TEST: Pose Monitor (controller display)\n");

    if (!validateSbotLemLibInitialization() || !sbot_chassis) {
        printf("SBOT Pose Monitor: LemLib/chassis not initialized\n");
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "POSE MON: no chassis");
            master.print(1, 0, "Check LemLib init");
        }
        pros::delay(1500);
        return;
    }

    sbot_safe_stop_mechanisms();

    // Easier to push around by hand.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // Deterministic frame for calibration.
    sbot_zero_pose_and_sensors(0, 0, 0);
    pros::delay(50);

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE MON (B=exit)");
        master.print(1, 0, "Move robot by hand");
    }
    pros::delay(600);

    uint32_t last_controller_ms = 0;
    uint32_t last_terminal_ms = 0;

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const uint32_t now = pros::millis();
        const auto pose = sbot_chassis->getPose();
        const double imu_h = sbot_get_best_heading_deg();

        if (now - last_controller_ms >= 100) {
            last_controller_ms = now;
            if (master.is_connected()) {
                // Controller screen is tight: keep it compact.
                master.print(0, 0, "x%6.2f y%6.2f", pose.x, pose.y);
                master.print(1, 0, "th%6.1f imu%5.1f", pose.theta, imu_h);
            }
        }

        if (now - last_terminal_ms >= 500) {
            last_terminal_ms = now;
            printf("POSE MON: x=%.2f y=%.2f th=%.1f imu=%.1f\n", pose.x, pose.y, pose.theta, imu_h);
        }

        pros::delay(20);
    }

    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE MON: exit");
    }

    // Restore typical behavior for other routines.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(250);
}

void SbotAutonomousSystem::runTestPoseFinderX0Line90() {
    // Starting-reference helper:
    // - Physically place the robot on the FIELD x=0 line
    // - Set the robot heading to 90° (LemLib convention: 0° is +Y, 90° is +X)
    // Then move the robot by hand to the desired match start and read off x/y/theta.
    printf("SBOT AUTON TEST: Pose Finder (x=0 line, heading=90)\n");

    if (!validateSbotLemLibInitialization() || !sbot_chassis) {
        printf("SBOT Pose Finder: LemLib/chassis not initialized\n");
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "POSE FIND: no chassis");
            master.print(1, 0, "Check LemLib init");
        }
        pros::delay(1500);
        return;
    }

    sbot_safe_stop_mechanisms();

    // Easier to push around by hand.
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // Declare the current physical placement as (0,0,90).
    // NOTE: this assumes the robot is already placed on the x=0 line with 90° heading when you start this test.
    sbot_zero_pose_and_sensors(0, 0, 90);
    pros::delay(50);

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE FIND (B=exit)");
        master.print(1, 0, "Start x0 th90, move");
    }
    pros::delay(700);

    uint32_t last_controller_ms = 0;
    uint32_t last_terminal_ms = 0;

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const uint32_t now = pros::millis();
        const auto pose = sbot_chassis->getPose();

        // "Jerry-style" mapping, but RELATIVE to this test's start.
        // We intentionally want the display to read (0,0) at the moment we set pose to (0,0,90)
        // on the x=0 line.
        // Mapping (relative):
        //   jx = ourY
        //   jy = -ourX
        const double jerry_x = pose.y;
        const double jerry_y = -pose.x;

        if (now - last_controller_ms >= 100) {
            last_controller_ms = now;
            if (master.is_connected()) {
                master.print(0, 0, "jx%7.2f jy%7.2f", jerry_x, jerry_y);
                master.print(1, 0, "th%6.1f", pose.theta);
            }
        }

        if (now - last_terminal_ms >= 500) {
            last_terminal_ms = now;
            printf("POSE FIND: rel_jerry(%.2f,%.2f) th=%.1f\n", jerry_x, jerry_y, pose.theta);
        }

        pros::delay(20);
    }

    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE FIND: exit");
    }

    // Restore typical behavior for other routines.
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(250);
}

void SbotAutonomousSystem::runTestFollowJerryPath() {
    printf("SBOT AUTON TEST: FOLLOW PATH (LemLib follow)\n");
    if (!validateSbotLemLibInitialization() || !sbot_chassis) return;

    sbot_safe_stop_mechanisms();

    // Start collecting immediately.
    sbot_intake_on_storage();

    // For following a continuous path, COAST avoids hard stops.
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // For LemLib follow(), the robot pose frame must match the path file's coordinate frame.
    // IMPORTANT: keep IMU heading/rotation consistent with the chassis pose heading.
    // This test path is authored in the same absolute Jerry frame as Red Left start.
    // Match the first waypoint in static/low.txt so follow() doesn't start with a lateral offset.
    sbot_zero_pose_and_sensors(-50.23f, 15.31f, 90.0f);

    // Record starting IMU heading for post-follow turns.
    // All turns will be calculated as offsets from this initial calibration.
    const double start_imu_heading = sbot_chassis->getPose(false, true).theta;  // Should be ~0° when pose=90°
    constexpr double kStartPoseHeading = 90.0;
    printf("SBOT FOLLOW START: pose=%.1f, std=%.1f\n", kStartPoseHeading, start_imu_heading);

    sbot_print_pose("before follow");
    sbot_print_sensors("before follow");

    printf("SBOT FOLLOW: asset bytes=%u\n", static_cast<unsigned>(low_txt.size));

    // Keep LemLib logs quiet for normal runs.
    lemlib::infoSink()->setLowestLevel(lemlib::Level::WARN);

    // Follow the compiled path asset.
    // Note: lookahead is in inches.
    // Smaller lookahead => tighter tracking (often slower/more oscillation if too small).
    constexpr float kLookaheadIn = 10.0f;
    // Keep the follow test bounded (~4-5 seconds).
    constexpr int kTimeoutMs = 5000;

    // Run async so we can report whether motion actually starts.
    sbot_chassis->follow(low_txt, kLookaheadIn, kTimeoutMs, true /*forwards*/, true /*async*/);

    const uint32_t start_ms = pros::millis();
    bool ever_in_motion = false;
    bool printed_end = false;

    // Keep waiting up to the timeout window, but avoid spamming the terminal.
    while (pros::millis() - start_ms < static_cast<uint32_t>(kTimeoutMs + 250)) {
        const bool in_motion = sbot_chassis->isInMotion();
        if (in_motion) ever_in_motion = true;

        sbot_trace_follow_progress(start_ms, pros::millis());

        if (!in_motion && ever_in_motion && !printed_end) {
            printed_end = true;
            printf("SBOT FOLLOW: motion complete at t=%u ms\n",
                   static_cast<unsigned>(pros::millis() - start_ms));
        }

        pros::delay(20);
    }

    // Wait for completion (if LemLib actually queued/runs a motion, this blocks).
    sbot_chassis->waitUntilDone();

    sbot_print_pose("after follow path");
    sbot_print_sensors("after follow path");

    // --- Manual angle finding mode ---
    // Loop to let you manually position the robot and read the IMU angles
    printf("\n=== ANGLE FINDER MODE ===\n");
    printf("Manually rotate robot to desired positions and note the IMU heading.\n");
    printf("Press B button to exit and continue.\n\n");
    
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    uint32_t last_print = pros::millis();
    
    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const uint32_t now = pros::millis();
        
        if (now - last_print >= 200) {  // Print 5 times per second
            last_print = now;
            
            const auto pose = sbot_chassis->getPose(false, false);  // pose frame
            const auto std = sbot_chassis->getPose(false, true);    // std frame
            const double imu_h = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;
            
            printf("x=%.2f y=%.2f | pose.th=%.1f std.th=%.1f imu.h=%.1f\n",
                   pose.x, pose.y, pose.theta, std.theta, imu_h);
        }
        
        pros::delay(20);
    }
    
    printf("\n=== ANGLE FINDER MODE EXIT ===\n");
    printf("Press B again to skip remaining autonomous and go to driver control.\n\n");
    
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        // Skip the rest of autonomous
        sbot_safe_stop_mechanisms();
        sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
        return;
    }

    // --- Post-run sequence (final) ---
    // Using manually measured IMU headings for each desired robot orientation.
    // All angles are absolute IMU values, eliminating coordinate frame confusion.
    printf("SBOT POSTFOLLOW: Starting post-follow sequence with measured IMU targets\n");

    // Keep post-follow actions at low speed for safety and repeatability.
    constexpr int kPostTurnMaxSpeed = SBOT_MATCH_TURN_MAX_SPEED / 2;
    constexpr int kPostDriveMaxSpeed = SBOT_MATCH_MAX_SPEED / 2;

    lemlib::TurnToHeadingParams post_turn_params;
    post_turn_params.maxSpeed = kPostTurnMaxSpeed;
    post_turn_params.minSpeed = 10;

    lemlib::MoveToPointParams post_drive_params;
    post_drive_params.forwards = true;
    post_drive_params.maxSpeed = kPostDriveMaxSpeed;

    // 1) Wait after follow completes
    pros::delay(250);

    // 2) Turn to face away from start (imu=268°)
    printf("SBOT POSTFOLLOW: turn to imu=268 (face away)\n");
    sbot_chassis->turnToHeading(268.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_away");

    // 3) Drop loader down and wait
    if (sbot_batch_loader) sbot_batch_loader->extend();
    pros::delay(1000);

    // 4) Turn to point back toward starting area (imu=313°)
    printf("SBOT POSTFOLLOW: turn to imu=313\n");
    sbot_chassis->turnToHeading(313.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_toward_start");

    // 5) Back to (-24, 24) maintaining heading imu=310°
    printf("SBOT POSTFOLLOW: back to (-24, 24) at imu=310\n");
    post_drive_params.forwards = false;  // BACKWARDS
    sbot_turn_point_turn(
        "postfollow_to_-24_24",
        -24.0f,
        24.0f,
        310.0f,
        2500,
        4000,
        post_turn_params,
        post_drive_params
    );
    post_drive_params.forwards = true;  // Reset to default

    // 6) Turn to imu=38°
    printf("SBOT POSTFOLLOW: turn to imu=38\n");
    sbot_chassis->turnToHeading(38.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_38");

    // Lift matchloader after turning
    if (sbot_batch_loader) sbot_batch_loader->retract();

    // 7) Move to (-12, 24) at imu=39°
    printf("SBOT POSTFOLLOW: move to (-12, 24) at imu=39\n");
    sbot_turn_point_turn(
        "postfollow_to_-12_24",
        -12.0f,
        24.0f,
        39.0f,
        2500,
        4000,
        post_turn_params,
        post_drive_params,
        0,
        0,
        0.0,
        0.0,
        false,
        true  // backwards
    );

    // 8) Turn to imu=170° 
    printf("SBOT POSTFOLLOW: turn to imu=170\n");
    sbot_chassis->turnToHeading(170.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_170");

    // 9) Low score
    sbot_score_low_for(1500);

    // LemLib logs remain at WARN.
    // Conversion: pose=180° → std=90° (std = pose - 90)
    // Delta from start: std_target - std_start = 90° - 0° = +90°
    // Convert to imu.heading: imu = -std mod 360 = -90° = 270°
    printf("SBOT POSTFOLLOW: turn to 180\n");
    const double delta_180 = 180.0 - kStartPoseHeading;  // +90° in pose/std frame
    const double std_target_180 = start_imu_heading + delta_180;  // 0 + 90 = 90°
    const double imu_target_180 = sbot_norm_heading(-std_target_180);  // -90 = 270°
    printf("  DEBUG: delta=%.1f, std_target=%.1f, imu_target=%.1f\n", delta_180, std_target_180, imu_target_180);
    sbot_chassis->turnToHeading(imu_target_180, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_180");
    sbot_wait_until_done_timed("postfollow.turn_to_180");

    // 3) Drop loader down and wait 1 second.
    if (sbot_batch_loader) sbot_batch_loader->extend();
    pros::delay(1000);

    // 4) Move to -24,24,135 (45° right from start).
    // Conversion: pose=135° → std=45° (std = pose - 90)
    // Delta from start: std_target - std_start = 45° - 0° = +45°
    // Convert to imu.heading: imu = -std mod 360 = -45° = 315°
    {
        const double delta_135 = 135.0 - kStartPoseHeading;  // +45° in pose/std frame
        const double std_target_135 = start_imu_heading + delta_135;  // 0 + 45 = 45°
        const float target_heading_imu = static_cast<float>(sbot_norm_heading(-std_target_135));  // -45 = 315°
        printf("SBOT POSTFOLLOW: move to (-24, 24) endHeading=135 (imu=%.1f)\n", target_heading_imu);
        sbot_turn_point_turn(
            "postfollow_to_-24_24",
            -24.0f,
            24.0f,
            target_heading_imu,
            2500,
            4000,
            post_turn_params,
            post_drive_params
        );
    }

    // 5) Turn in place to 135 degrees (45° right from start).
    // Conversion: pose=135° → std=45° (std = pose - 90)
    // Delta from start: std_target - std_start = 45° - 0° = +45°
    // Convert to imu.heading: imu = -std mod 360 = -45° = 315°
    printf("SBOT POSTFOLLOW: turn to 135\n");
    const double delta_135 = 135.0 - kStartPoseHeading;  // +45° in pose/std frame
    const double std_target_135 = start_imu_heading + delta_135;  // 0 + 45 = 45°
    const double imu_target_135 = sbot_norm_heading(-std_target_135);  // -45 = 315°
    sbot_chassis->turnToHeading(imu_target_135, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_135");

    // Lift matchloader after moving to 135 degrees.
    if (sbot_batch_loader) sbot_batch_loader->retract();

    // 6) Drive to pose -9,9,135 (backing up at the same angle, 45° right from start).
    // Conversion: pose=135° → std=45° (std = pose - 90)
    // Delta from start: std_target - std_start = 45° - 0° = +45°
    // Convert to imu.heading: imu = -std mod 360 = -45° = 315°
    {
        const double delta_135_back = 135.0 - kStartPoseHeading;  // +45° in pose/std frame
        const double std_target_135_back = start_imu_heading + delta_135_back;  // 0 + 45 = 45°
        const float target_heading_imu = static_cast<float>(sbot_norm_heading(-std_target_135_back));  // -45 = 315°
        printf("SBOT POSTFOLLOW: drive to (-9, 9) endHeading=135 (imu=%.1f)\n", target_heading_imu);
        sbot_turn_point_turn(
            "postfollow_to_-9_9",
            -9.0f,
            9.0f,
            target_heading_imu,
            2500,
            4000,
            post_turn_params,
            post_drive_params,
            0,
            0,
            0.0,
            0.0,
            false,
            true
        );
    }

    // 7) Turn to 45 degrees (45° left from start).
    // Conversion: pose=45° → std=-45°=315° (std = pose - 90)
    // Delta from start: std_target - std_start = -45° - 0° = -45°
    // Convert to imu.heading: imu = -std mod 360 = -(-45) = 45°
    printf("SBOT POSTFOLLOW: turn to 45\n");
    const double delta_45 = 45.0 - kStartPoseHeading;  // -45° in pose/std frame
    const double std_target_45 = start_imu_heading + delta_45;  // 0 + (-45) = -45° = 315°
    const double imu_target_45 = sbot_norm_heading(-std_target_45);  // -315 = 45°
    sbot_chassis->turnToHeading(imu_target_45, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_45");

    // 9) Low score
    sbot_score_low_for(1500);

    // LemLib logs remain at WARN.

    // Stop mechanisms after the post-run actions.
    sbot_safe_stop_mechanisms();

    sbot_print_pose("after follow");
    sbot_print_sensors("after follow");

    sbot_safe_stop_mechanisms();
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

// ---- Match autonomous stubs ----

void SbotAutonomousSystem::runRedLeft() {
    sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, false);
}

void SbotAutonomousSystem::runRedRight() {
    sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED, false);
}

void SbotAutonomousSystem::runBlueLeft() {
    sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::BLUE, false);
}

void SbotAutonomousSystem::runBlueRight() {
    sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::BLUE, false);
}

void SbotAutonomousSystem::runSkills() {
    sbot_run_skills_auto();
}

// ---- Test autonomous stubs ----

void SbotAutonomousSystem::runTestSweepToLowGoal() {
    // Focused test: start from the end of the cluster sweep and run ONLY the Center Lower (front) score.
    // Uses the exact same Stage 2 logic/params as match auton.
    printf("SBOT AUTON TEST: SWEEP -> CENTER LOWER (front score)\n");
    sbot_run_match_auto(
        SbotAutoSide::LEFT,
        SbotAutoAlliance::RED,
        false /* solo_awp */,
        true /* start_from_cluster_sweep */,
        true /* stop_after_stage2 */,
        true /* stage2_skip_pre_turn */
    );
}

void SbotAutonomousSystem::runTestDrive() {
    printf("SBOT AUTON TEST: RECTANGLE (replaces drive test)\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    // Continuous path: use coast so it doesn't hard-stop at each corner.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    sbot_print_pose("before start");
    sbot_print_sensors("before start");

    // Reset frame
    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("start");
    sbot_print_sensors("start");

    // Rectangle: 2 VEX tiles per side.
    // Field tiles are 24", so 2 tiles = 48".
    constexpr float kLegIn = 48.0f;
    constexpr int kTimeoutMs = 9500;

    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 70;
    turnParams.minSpeed = 0;

    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 60;      // reduced speed
    params.minSpeed = 0;       // prioritize reaching the actual corner
    params.earlyExitRange = 0; // no early exit

    struct Target {
        float x;
        float y;
        float theta;
        const char* label;
    };

    // Use segment-aligned headings so the robot doesn't start by turning.
    // Heading convention in this project: 0° is +Y (forward); clockwise-positive.
    // Segment headings (left turns): 0 -> -90 -> 180 -> 90.
    const Target targets[] = {
        {0,       kLegIn,   0,   "corner1"},
        {-kLegIn, kLegIn,  -90,  "corner2"},
        {-kLegIn, 0,       180,  "corner3"},
        {0,       0,        90,  "back home"}
    };

    for (int i = 0; i < 4; i++) {
        const auto before = sbot_chassis->getPose();

        char label[64];
        std::snprintf(label, sizeof(label), "test.drive.rect.%s", targets[i].label);
        sbot_turn_point_turn(
            label,
            targets[i].x,
            targets[i].y,
            targets[i].theta,
            2500,
            static_cast<uint32_t>(kTimeoutMs),
            turnParams,
            params,
            static_cast<uint32_t>(kTimeoutMs + 750),
            250,
            1.0,
            5.0
        );

        const auto after = sbot_chassis->getPose();
        printf(
            "SBOT RECT: %s delta: dx=%.2f dy=%.2f dth=%.2f\n",
            targets[i].label,
            after.x - before.x,
            after.y - before.y,
            after.theta - before.theta
        );
        sbot_print_pose(targets[i].label);
        sbot_print_sensors(targets[i].label);
    }

    // Optional: square ends with a final left turn to face 0° again.
    lemlib::TurnToHeadingParams leftTurnParams;
    leftTurnParams.direction = AngularDirection::CCW_COUNTERCLOCKWISE;
    leftTurnParams.maxSpeed = 70;
    leftTurnParams.minSpeed = 15;
    sbot_chassis->turnToHeading(0, 2500, leftTurnParams);
    sbot_wait_until_done_timed("test.drive.rect.turn_to_0");

    // 3) Reverse sequence, driving backwards ("back direction")
    // Start with a right turn, then drive backwards 2 tiles, and repeat.
    printf("SBOT AUTON TEST: RECTANGLE reverse (backwards)\n");

    lemlib::MoveToPointParams backParams = params;
    backParams.forwards = false;
    backParams.maxSpeed = 55; // a little slower for backwards driving

    // Reverse traversal waypoints (clockwise), but driven backwards:
    // (0,0,0) -> (-48,0,90) -> (-48,48,180) -> (0,48,270) -> (0,0,0)
    const Target backTargets[] = {
        {-kLegIn, 0,       90,  "back corner1"},
        {-kLegIn, kLegIn,  180, "back corner2"},
        {0,       kLegIn,  270, "back corner3"},
        {0,       0,       0,   "back home"}
    };

    for (int i = 0; i < 4; i++) {
        const auto before = sbot_chassis->getPose();

        char label[64];
        std::snprintf(label, sizeof(label), "test.drive.rect.back.%s", backTargets[i].label);
        sbot_turn_point_turn(
            label,
            backTargets[i].x,
            backTargets[i].y,
            backTargets[i].theta,
            2500,
            static_cast<uint32_t>(kTimeoutMs),
            turnParams,
            backParams,
            static_cast<uint32_t>(kTimeoutMs + 750),
            250,
            1.0,
            7.0
        );

        const auto after = sbot_chassis->getPose();
        printf(
            "SBOT RECT BACK: %s delta: dx=%.2f dy=%.2f dth=%.2f\n",
            backTargets[i].label,
            after.x - before.x,
            after.y - before.y,
            after.theta - before.theta
        );
        sbot_print_pose(backTargets[i].label);
        sbot_print_sensors(backTargets[i].label);
    }

    sbot_print_pose("end");
    sbot_print_sensors("end");

    sbot_safe_stop_mechanisms();

    // Restore BRAKE so subsequent match autons/tests don't inherit COAST behavior.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void SbotAutonomousSystem::runTestDriveShort() {
    // Short drive test: fixed start/end pose.
    printf("SBOT AUTON TEST: SHORT DRIVE\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    // Continuous path: use coast so it doesn't hard-stop at each corner.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    sbot_print_pose("before start");
    sbot_print_sensors("before start");

    // Reset frame: start at (0,0,0)
    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("start");
    sbot_print_sensors("start");

    // Target pose: end at (9,9,45)
    constexpr float kTargetX = 9.0f;
    constexpr float kTargetY = 9.0f;
    constexpr float kTargetTheta = 45.0f;
    constexpr int kTimeoutMs = 4500;
    printf("SBOT SHORT DRIVE: start(0,0,0) -> target(%.2f,%.2f,%.1f)\n", kTargetX, kTargetY, kTargetTheta);

    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 70;
    turnParams.minSpeed = 0;

    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 60;      // reduced speed
    params.minSpeed = 0;       // prioritize reaching the actual corner
    params.earlyExitRange = 0; // no early exit

    const auto before = sbot_chassis->getPose();
    sbot_turn_point_turn(
        "test.drive.short.target",
        kTargetX,
        kTargetY,
        kTargetTheta,
        2500,
        static_cast<uint32_t>(kTimeoutMs),
        turnParams,
        params,
        static_cast<uint32_t>(kTimeoutMs + 500),
        200,
        0.75,
        7.0
    );

    const auto after = sbot_chassis->getPose();
    printf(
        "SBOT SHORT DRIVE delta: dx=%.2f dy=%.2f dth=%.2f\n",
        after.x - before.x,
        after.y - before.y,
        after.theta - before.theta
    );
    sbot_print_pose("after short drive");
    sbot_print_sensors("after short drive");

    sbot_print_pose("end");
    sbot_print_sensors("end");

    sbot_safe_stop_mechanisms();

    // Restore BRAKE so subsequent match autons/tests don't inherit COAST behavior.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void SbotAutonomousSystem::runTestLowGoalCustomStart() {
    printf("SBOT AUTON TEST: Low Goal (custom start)\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    // You will place the robot physically, then we declare that placement as the start pose here.
    // Update these constants between runs.
    constexpr float kStartX = 0.0f;
    constexpr float kStartY = 0.0f;
    constexpr float kStartThetaDeg = 0.0f;

    // Red Left Center Lower (front score) pose target (same as match logic contact->pose):
    // contact(9.0, 40.5) heading=45°, front=SBOT_FRONT_BUMPER_IN => pose(3.70, 35.20)
    const SbotPoint target = sbot_pose_from_front_contact({9.0, 40.5}, 45.0, SBOT_FRONT_BUMPER_IN);
    constexpr double kTargetHeadingDeg = 45.0;

    // Reset sensors/odom for deterministic starting state.
    sbot_zero_pose_and_sensors(0, 0, 0);
    pros::delay(40);
    sbot_chassis->setPose(kStartX, kStartY, kStartThetaDeg);
    pros::delay(40);
    sbot_chassis->setPose(kStartX, kStartY, kStartThetaDeg);

    sbot_print_pose("custom start");
    sbot_print_sensors("custom start");

    printf(
        "SBOT TEST: start(%.2f,%.2f,%.1f) -> target(%.2f,%.2f,%.1f)\n",
        kStartX,
        kStartY,
        kStartThetaDeg,
        target.x,
        target.y,
        kTargetHeadingDeg
    );
    sbot_print_jerry_target("test_low_goal_pose_target", target.x, target.y);

    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 70;
    turnParams.minSpeed = 0;

    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 95;
    params.minSpeed = 0;
    params.earlyExitRange = 0;

    // Make LemLib's internal timeout long so our wait loop determines whether it converged.
    const uint32_t motion_timeout_ms = 15000;
    const uint32_t wait_timeout_ms = 6000;

    sbot_lemlib_debug_window_begin("test.low_goal_custom_start");
    sbot_turn_point_turn(
        "test.low_goal_custom_start",
        target.x,
        target.y,
        static_cast<float>(kTargetHeadingDeg),
        2500,
        motion_timeout_ms,
        turnParams,
        params,
        wait_timeout_ms,
        500,
        0.5,
        6.0
    );
    sbot_lemlib_debug_window_end("test.low_goal_custom_start");

    sbot_print_pose("after test approach");
    sbot_print_sensors("after test approach");

    sbot_safe_stop_mechanisms();
}

void SbotAutonomousSystem::runTestTurn() {
    printf("SBOT AUTON TEST: TURN\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    // Match-auton turning behavior (avoid post-turn coasting).
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    // Deterministic sensor + pose frame for turn/odometry tuning.
    // This makes the tracking-wheel offset estimate meaningful across runs.
    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("start");
    sbot_print_sensors("start");

    // During a perfect in-place turn, a vertical tracking wheel that's laterally offset from the
    // rotation center will roll an arc length proportional to the offset and the turn angle:
    //   dWheelIn ~= offsetIn * dThetaRad
    // => offsetIn ~= dWheelIn / dThetaRad
    // This is exactly the value LemLib expects as the TrackingWheel "distance" (left/right offset).
    auto estimate_offset_for_turn = [&](double target_heading_deg, const char* label, double& total_dtheta_rad, double& total_dvert_in) {
        const double imu_rot0 = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
        const double vert0 = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

        lemlib::TurnToHeadingParams params;
        // Slower turn reduces tracking-wheel slip and improves offset estimation stability.
        params.maxSpeed = 50;
        params.minSpeed = 0;

        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        sbot_chassis->turnToHeading(target_heading_deg, 3000, params);
        sbot_wait_until_done_timed(label);
        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        const double imu_rot1 = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
        const double vert1 = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

        const double dtheta_deg = imu_rot1 - imu_rot0;
        const double dtheta_rad = dtheta_deg * M_PI / 180.0;
        const double dvert = vert1 - vert0;

        if (std::fabs(dtheta_rad) < 1e-3) {
            printf("SBOT TURN OFFSET EST [%s]: dTheta too small (%.3f deg)\n", label, dtheta_deg);
            return;
        }

        const double offset_est = dvert / dtheta_rad;
        printf(
            "SBOT TURN OFFSET EST [%s]: dTheta=%.2fdeg dVert=%.2fin => offset=%.3fin (sign from sensor)\n",
            label,
            dtheta_deg,
            dvert,
            offset_est
        );

        total_dtheta_rad += dtheta_rad;
        total_dvert_in += dvert;
    };

    // Accumulate multiple quarter-turns to reduce noise/slip effects.
    // Total offset is computed from total dVert / total dTheta, which is much more stable than
    // any single segment.
    double total_dtheta_rad = 0.0;
    double total_dvert_in = 0.0;

    constexpr int kCycles = 3; // 1 cycle = 360deg total
    for (int cycle = 1; cycle <= kCycles; cycle++) {
        char label[64];
        std::snprintf(label, sizeof(label), "test.turn.c%d.to90", cycle);
        estimate_offset_for_turn(90, label, total_dtheta_rad, total_dvert_in);

        std::snprintf(label, sizeof(label), "test.turn.c%d.to180", cycle);
        estimate_offset_for_turn(180, label, total_dtheta_rad, total_dvert_in);

        std::snprintf(label, sizeof(label), "test.turn.c%d.to270", cycle);
        estimate_offset_for_turn(270, label, total_dtheta_rad, total_dvert_in);

        std::snprintf(label, sizeof(label), "test.turn.c%d.to0", cycle);
        estimate_offset_for_turn(0, label, total_dtheta_rad, total_dvert_in);
    }

    if (std::fabs(total_dtheta_rad) > 1e-3) {
        const double offset_total = total_dvert_in / total_dtheta_rad;
        printf(
            "SBOT TURN OFFSET RECOMMEND: total dTheta=%.1fdeg total dVert=%.2fin => offset=%.3fin\n",
            total_dtheta_rad * 180.0 / M_PI,
            total_dvert_in,
            offset_total
        );
        printf(
            "SBOT TURN OFFSET NOTE: set SBOT_TRACKING_WHEEL_DISTANCE to %.3f (or flip sign if drift direction worsens)\n",
            offset_total
        );
    } else {
        printf("SBOT TURN OFFSET RECOMMEND: total dTheta too small; rerun on-field\n");
    }

    sbot_print_pose("after turns");
    sbot_print_sensors("after turns");

    sbot_safe_stop_mechanisms();
}

void SbotAutonomousSystem::runTestIntake() {
    printf("SBOT AUTON TEST: INTAKE\n");

    sbot_safe_stop_mechanisms();

    if (!sbot_intake) {
        printf("SBOT AUTON TEST: INTAKE missing sbot_intake\n");
        return;
    }

    // Forward intake
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_run_for_ms(800);

    // Stop
    sbot_intake->setMode(IntakeMode::OFF);
    sbot_run_for_ms(150);

    printf("SBOT AUTON TEST: INTAKE done\n");
}

void SbotAutonomousSystem::runTestIndexer() {
    printf("SBOT AUTON TEST: INDEXER\n");

    sbot_safe_stop_mechanisms();

    if (!sbot_indexer) {
        printf("SBOT AUTON TEST: INDEXER missing sbot_indexer\n");
        return;
    }

    // Forward feed
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_run_for_ms(650);

    // Reverse briefly (middle/eject direction)
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    sbot_run_for_ms(350);

    // Stop
    sbot_indexer->setMode(IndexerMode::OFF);
    sbot_run_for_ms(150);

    printf("SBOT AUTON TEST: INDEXER done\n");
}
