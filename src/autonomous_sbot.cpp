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

// Robot geometry (distance from LemLib pose point to bumpers, in inches)
// Pose point is the drivetrain rotation center. Measured: 7.5" to both bumpers.
// These are used to convert field "contact points" (where a bumper should be) into pose targets.
static constexpr double SBOT_FRONT_BUMPER_IN = 7.5;
static constexpr double SBOT_BACK_BUMPER_IN = 7.5;

// Jerry coordinate conversion config.
// Jerry coords are absolute field inches. Our internal coords are start-relative:
// - +Y is into-field from the robot start, +X is robot-right at start
// - our 0° faces +Y
// From prior calibration: Jerry "start" point supplied was the IMU/gyro location.
// Our LemLib pose point (drivetrain rotation center) is 4.5" FORWARD of that.
static constexpr double SBOT_GYRO_TO_POSE_FORWARD_IN = 4.5;

// Default Jerry start point for RED LEFT (can be updated if you want other starts).
static double sbot_jerry_start_x = -51.0;
static double sbot_jerry_start_y = 15.0;

static constexpr bool SBOT_DUMP_JERRY_POINTS = true;

static void sbot_dump_jerry_point() {
    if (!SBOT_DUMP_JERRY_POINTS) return;
    if (!sbot_chassis) return;

    const auto pose = sbot_chassis->getPose();
    const double jerry_x = pose.y + SBOT_GYRO_TO_POSE_FORWARD_IN + sbot_jerry_start_x;
    const double jerry_y = sbot_jerry_start_y - pose.x;

    // Plain point output (no labels, no ':'), for easy log cleanup.
    printf("%.3f,%.3f,120\n", jerry_x, jerry_y);
}

// Temporary: run match auton at reduced speed for tuning.
// LemLib maxSpeed is typically in the 0..127-ish range.
static constexpr int SBOT_MATCH_MAX_SPEED = 127;
static constexpr int SBOT_MATCH_TURN_MAX_SPEED = 127;

// Minimum time to spend actively scoring at a goal.
static constexpr uint32_t SBOT_MIN_SCORE_TIME_MS = 1000;

// Debug aid: print how long each LemLib waitUntilDone() blocks.
static constexpr bool SBOT_PRINT_WAIT_TIMES = true;

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

static void sbot_zero_pose_and_sensors(float x = 0, float y = 0, float theta_deg = 0) {
    if (!sbot_chassis) return;
    sbot_chassis->setPose(x, y, theta_deg);

    // IMPORTANT: LemLib IMU integration often relies on IMU *rotation* (continuous), not just heading.
    // Your logs showed heading got reset but rotation did not, which causes odom to think it is already turned.
    if (sbot_inertial_sensor) {
        sbot_inertial_sensor->tare_rotation();
        sbot_inertial_sensor->tare_heading();
        // Be explicit about the requested frame.
        sbot_inertial_sensor->set_rotation(theta_deg);
        sbot_inertial_sensor->set_heading(theta_deg);
    }
    if (sbot_vertical_encoder) {
        sbot_vertical_encoder->reset_position();
    }
    pros::delay(60);
}

static double sbot_norm_heading(double deg) {
    while (deg < 0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
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

static void sbot_run_match_auto(SbotAutoSide side, SbotAutoAlliance alliance, bool solo_awp) {
    // Match auto is currently focused on achieving our portion of the AWP tasks.
    // Tune the points in `sbot_awp_half_default_tuning()` on-field.
    struct SbotAwpHalfTuning {
        // All points are defined for RED + RIGHT.
        // They are transformed for BLUE (180° rotate) and LEFT (mirror X).
        // Frame is start-relative as described in `sbot_set_match_start_pose()`.

        // Stage 0: ensure we are not touching the park zone barrier
        double clear_barrier_in;

        // Stage 1: collect the nearby block cluster (not tubes yet)
        SbotPoint cluster1;
        SbotPoint cluster1_sweep;
        uint32_t cluster_collect_ms;

        // Stage 2: first scoring
        // Field detail: the Center Goals provide multiple scoring positions.
        // For our-side AWP flow, we treat these as two distinct targets:
        // - (RED LEFT, BLUE RIGHT): Center Goal – Lower (front score)
        // - (RED RIGHT, BLUE LEFT): Center Goal – Middle (back score)
        SbotPoint low_goal_approach;
        double low_goal_heading_deg;
        uint32_t low_goal_score_ms;

        // Optional: define Center Goal – Lower as a FRONT-bumper contact point.
        bool use_low_goal_contact;
        SbotPoint low_goal_contact;

        SbotPoint mid_goal_approach;
        double mid_goal_heading_deg;
        uint32_t mid_goal_score_ms;

        // Optional: define Center Goal – Middle as a BACK-bumper contact point.
        bool use_mid_goal_contact;
        SbotPoint mid_goal_contact;

        // Second score target (manual terms): near end of the Long Goal adjacent to the Loader.
        SbotPoint high_goal_approach;
        double high_goal_heading_deg;
        uint32_t high_goal_score_ms;

        // If non-zero, finish by backing straight from the Loader-facing posture into the nearby Long Goal end.
        // This is especially useful on Red Left (and Blue Right) where that Long Goal end is adjacent to the Loader.
        double high_goal_back_in_from_tube_in;

        // Stage 3: retreat + tube load + score
        // Retreat model: back straight out from the Center Goal (reverse of the approach direction).
        // This avoids "cutting through" the goal and keeps motion simple.
        double post_score_retreat_back_dist_in;
        // Optional: force an absolute retreat endpoint (used to stay within our quarter).
        bool use_post_score_retreat_point;
        SbotPoint post_score_retreat_point;
        double tube_face_heading_deg;

        // When facing the Loader (RL plan), align to a constant-X line so the subsequent
        // Loader pull (+1 tile) and Long Goal score (-2 tiles) are collinear.
        double loader_long_goal_line_x;

        // Extra forward protrusion (in) when the loader mechanism is deployed.
        // Used when converting a front-contact point into a pose target.
        double loader_down_extra_front_in;

        // Loader adjacent to alliance station (our-side)
        SbotPoint tube1;
        SbotPoint tube1_pulloff;
        uint32_t tube_pull_ms;

        // Optional: specify Loader/Long-Goal points as *bumper contact points*.
        // If enabled, we compute the pose target using SBOT_FRONT_BUMPER_IN / SBOT_BACK_BUMPER_IN.
        bool use_tube1_contact;
        SbotPoint tube1_contact;

        bool use_long_goal_end_contact;
        SbotPoint long_goal_end_contact;

        // Optional Loader 2 (Solo AWP)
        SbotPoint tube2;
        SbotPoint tube2_pulloff;

        // Timeouts
        uint32_t drive_timeout_ms;
        uint32_t turn_timeout_ms;

        // End pose (keep clear of park zone barrier)
        SbotPoint end_safe;
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

        // Cluster (RED LEFT): measured from Jerry field coords and converted to our start-relative frame.
        // Jerry frame (inches):
        //   Start: (-51, 15)
        //   Cluster: (-24, 24)
        // Jerry axes (confirmed):
        //   Into-field => Jerry X increases (less negative)
        //   Robot-right => Jerry Y decreases
        // Conversion used:
        //   ourX = (Jy_start - Jy)
        //   ourY = (Jx - Jx_start)
        // Result (gyro-origin):
        //   cluster1 = (-9, 27)
        // IMPORTANT: the Jerry "start" you provided was the gyro location.
        // Our LemLib pose point is the drivetrain rotation center.
        // From your measurements: gyro is 3" from the back bumper while the rotation center is 7.5".
        // => gyro is 4.5" BEHIND the pose point.
        // Convert gyro-origin -> pose-origin by shifting -4.5" in our +Y for all Jerry-derived points.
        t.cluster1 = {-9, 27 - SBOT_GYRO_TO_POSE_FORWARD_IN};
        // No separate sweep point was provided; start with a small forward continuation to ensure collection.
        t.cluster1_sweep = {-9, 33 - SBOT_GYRO_TO_POSE_FORWARD_IN};
        // Reduce the initial dwell; intake stays on while moving anyway.
        t.cluster_collect_ms = 150;

        // Center Goal – Lower (RED LEFT / BLUE RIGHT): from the cluster sweep,
        // user-measured direction is forward-right about ~0.75 tile diagonally.
        // IMPORTANT: keep the robot on the same line from cluster -> goal for reliable scoring.
        const double center_lower_dx = 18.0;
        const double center_lower_dy = 18.0;
        const SbotPoint center_lower_approach = {t.cluster1.x + center_lower_dx, t.cluster1.y + center_lower_dy};

        // Center Goal – Middle: separate tuning (primarily for RED RIGHT / BLUE LEFT).
        // Directional placeholder is more straight from the sweep: (0, +13).
        const double center_middle_dx = 18;
        const double center_middle_dy = 18.0;
        const SbotPoint center_middle_approach = {t.cluster1_sweep.x - center_middle_dx, t.cluster1_sweep.y + center_middle_dy};

        // Center Goal – Lower approach (front-score).
        t.low_goal_approach = center_lower_approach;
        t.low_goal_heading_deg = 45;
        // Lower-goal scoring: run longer to fully clear balls.
        t.low_goal_score_ms = SBOT_LOW_GOAL_SCORE_TIME_MS + 900;
        // Use a measured front-bumper contact point for the Center Goal.
        // Provided point (Jerry field coords, inches): Center Goal contact = (-9, 9)
        // Converted to our gyro-origin frame:
        //   ourX = (Jy_start - Jy) = (15 - 9) = 6
        //   ourY = (Jx - Jx_start) = (-9 - -51) = 42
        // Then shift gyro-origin -> pose-origin by -4.5" in +Y.
        // => (6, 37.5)
        t.use_low_goal_contact = true;
        t.low_goal_contact = {6.0, 37.5};

        // Center Goal – Middle (back-score).
        t.mid_goal_approach = center_middle_approach;
        t.mid_goal_heading_deg = 180;
        t.mid_goal_score_ms = std::max<uint32_t>(SBOT_MID_GOAL_SCORE_TIME_MS, SBOT_MIN_SCORE_TIME_MS);
        // Measured back-bumper contact point for Center Goal – Middle.
        // Provided point (Jerry field coords, inches): (-9, -9)
        // Converted (gyro-origin): ourX = 15 - (-9) = 24, ourY = -9 - (-51) = 42
        // Shift gyro-origin -> pose-origin by -4.5" in +Y => (24, 37.5)
        t.use_mid_goal_contact = true;
        t.mid_goal_contact = {24.0, 37.5};

        // Long Goal (near end by Loader) scoring position (fallback/placeholder).
        t.high_goal_approach = {6, 14};
        t.high_goal_heading_deg = 180;
        t.high_goal_score_ms = SBOT_MIN_SCORE_TIME_MS;
        // Field geometry: tube contact <-> long goal end contact are exactly 48 inches apart.
        // Note: when using contact points we compute the pose target directly; this is the fallback distance.
        // Back into long goal end from the *pulloff* position.
        // Empirically, ~27" hits the correct depth without timing out/pushing too deep.
        t.high_goal_back_in_from_tube_in = 27.0;

        // After first score: back straight out (used only when not using forced retreat point).
        t.post_score_retreat_back_dist_in = 68.0;

        // Force retreat to a measured absolute point (start-relative frame).
        // This point represents the robot pose point (drivetrain rotation center / "center" used by LemLib).
        t.use_post_score_retreat_point = true;
        // Jerry retreat: (-48,48) -> our (-33,3)
        t.post_score_retreat_point = {-33.0, 3.0 - SBOT_GYRO_TO_POSE_FORWARD_IN};

        // Red Left non-solo plan: after retreat, turn right 135° from diagonal (45° -> 180°) to face alliance wall,
        // then drive forward 1 tile to the Loader.
        t.tube_face_heading_deg = 180;

        // First-pass guess: the Loader and the near Long Goal end share an X-line.
        // Tune this on-field so RL does not drift left/right during the loader->long-goal sequence.
        // Jerry tube: (-51,48) -> our (-33,0)
        t.loader_long_goal_line_x = -33.0;

        // Your measured loader protrusion when deployed.
        t.loader_down_extra_front_in = 6.0;

        // Loader (tube) pose points (fallback).
        // Primary behavior uses field feature contact points + offsets.
        t.tube1 = {-33, -11.0};
        t.tube1_pulloff = {-33, -8.0};
        // Wait at the tube longer so intake has time to load.
        t.tube_pull_ms = 2000;

        // Loader contact point (field feature, Jerry coords) provided: (-71, 48).
        // Convert (gyro-origin): ourX = 15 - 48 = -33, ourY = -71 - (-51) = -20
        // Shift gyro-origin -> pose-origin by -4.5" in +Y => ourY = -24.5
        // This is where the FRONT of the robot/loader should contact the match loader.
        t.use_tube1_contact = true;
        t.tube1_contact = {-33.0, -24.5};

        // Long goal end contact points remain disabled for now (we back-in by distance from the loader).
        t.use_long_goal_end_contact = false;
        t.long_goal_end_contact = {0, 0};

        t.tube2 = {54, -24};
        t.tube2_pulloff = {-18, -18};

        // Timeouts: keep reasonable so we don't burn match time if something is slightly off.
        t.drive_timeout_ms = 4000;
        // Keep turns from burning too much time if we can't fully settle.
        t.turn_timeout_ms = 1800;

        t.end_safe = {0, 14};
        return t;
    };

    // Derived tuning: RED RIGHT is mirrored from RED LEFT for shared geometry.
    // Stage 2 goal identity differs (Center-Middle vs Center-Lower), so we override those fields explicitly.
    auto sbot_awp_half_red_right_tuning = [&]() -> SbotAwpHalfTuning {
        auto t = sbot_awp_half_red_left_tuning();

        // Use the same retreat concept as RL, mirrored across the centerline.
        // This keeps the retreat deterministic (point-based) instead of distance-based.
        t.use_post_score_retreat_point = true;

        // Mirror shared geometry across the centerline.
        t.cluster1 = sbot_mirror_point_x(t.cluster1);
        t.cluster1_sweep = sbot_mirror_point_x(t.cluster1_sweep);
        if (t.use_post_score_retreat_point) t.post_score_retreat_point = sbot_mirror_point_x(t.post_score_retreat_point);
        t.tube1 = sbot_mirror_point_x(t.tube1);
        t.tube1_pulloff = sbot_mirror_point_x(t.tube1_pulloff);
        t.low_goal_approach = sbot_mirror_point_x(t.low_goal_approach);
        t.mid_goal_approach = sbot_mirror_point_x(t.mid_goal_approach);
        if (t.use_low_goal_contact) t.low_goal_contact = sbot_mirror_point_x(t.low_goal_contact);
        if (t.use_mid_goal_contact) t.mid_goal_contact = sbot_mirror_point_x(t.mid_goal_contact);

        // Mirror loader/long-goal line and contact points.
        t.loader_long_goal_line_x = -t.loader_long_goal_line_x;
        if (t.use_tube1_contact) t.tube1_contact = sbot_mirror_point_x(t.tube1_contact);
        if (t.use_long_goal_end_contact) t.long_goal_end_contact = sbot_mirror_point_x(t.long_goal_end_contact);
        t.tube2 = sbot_mirror_point_x(t.tube2);
        t.tube2_pulloff = sbot_mirror_point_x(t.tube2_pulloff);
        t.high_goal_approach = sbot_mirror_point_x(t.high_goal_approach);

        // Mirror headings.
        t.low_goal_heading_deg = sbot_mirror_heading(t.low_goal_heading_deg);
        t.mid_goal_heading_deg = sbot_mirror_heading(t.mid_goal_heading_deg);
        t.high_goal_heading_deg = sbot_mirror_heading(t.high_goal_heading_deg);
        t.tube_face_heading_deg = sbot_mirror_heading(t.tube_face_heading_deg);

        // Override Stage 2: RED RIGHT should use Center Goal – Middle (back-score).
        const double center_middle_dx = 0.0;
        const double center_middle_dy = 13.0;
        t.mid_goal_approach = {t.cluster1_sweep.x + center_middle_dx, t.cluster1_sweep.y + center_middle_dy};
        t.mid_goal_heading_deg = 180;

        // Keep Center-Lower distinct (not used in this path).
        const double center_lower_dx = 13.0;
        const double center_lower_dy = 13.0;
        t.low_goal_approach = {t.cluster1_sweep.x + center_lower_dx, t.cluster1_sweep.y + center_lower_dy};
        t.low_goal_heading_deg = 45;

        return t;
    };

    auto sbot_run_awp_half_field = [&](SbotAutoSide side_, SbotAutoAlliance alliance_, bool solo_) {
        printf("SBOT AUTON: %s (%s %s)\n",
               solo_ ? "SOLO AWP" : "AWP HALF",
               (alliance_ == SbotAutoAlliance::RED) ? "RED" : "BLUE",
               (side_ == SbotAutoSide::RIGHT) ? "RIGHT" : "LEFT");

        if (!validateSbotLemLibInitialization()) return;

        sbot_safe_stop_mechanisms();
        sbot_set_match_start_pose();
        sbot_print_pose("awp start");

        const bool low_goal_case =
            (alliance_ == SbotAutoAlliance::RED) ? (side_ == SbotAutoSide::LEFT) : (side_ == SbotAutoSide::RIGHT);

        const auto t = low_goal_case ? sbot_awp_half_red_left_tuning() : sbot_awp_half_red_right_tuning();

        auto drive_to = [&](const SbotPoint& p, bool forwards) {
            if (!sbot_chassis) return;
            const SbotPoint target = sbot_apply_alliance_transform_only(p, alliance_);
            lemlib::MoveToPointParams params;
            params.forwards = forwards;
            params.maxSpeed = SBOT_MATCH_MAX_SPEED;
            sbot_chassis->moveToPoint(target.x, target.y, t.drive_timeout_ms, params);
            sbot_wait_until_done_timed("match.drive_to");
        };

        auto drive_to_speed = [&](const SbotPoint& p, bool forwards, int maxSpeed, const char* label) {
            if (!sbot_chassis) return;
            const SbotPoint target = sbot_apply_alliance_transform_only(p, alliance_);
            lemlib::MoveToPointParams params;
            params.forwards = forwards;
            params.maxSpeed = maxSpeed;
            sbot_chassis->moveToPoint(target.x, target.y, t.drive_timeout_ms, params);
            sbot_wait_until_done_timed(label);
        };

        auto turn_to = [&](double heading_deg) {
            if (!sbot_chassis) return;
            const double target_heading = sbot_apply_alliance_transform_heading_only(heading_deg, alliance_);
            lemlib::TurnToHeadingParams params;
            params.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
            // Avoid stalling near the end of the turn (which causes full timeouts).
            params.minSpeed = 10;
            sbot_chassis->turnToHeading(target_heading, t.turn_timeout_ms, params);
            sbot_wait_until_done_timed("match.turn_to");

            // Fine correction (IMU-based).
            // Keep this VERY short: we want to avoid burning full timeouts here.
            {
                const double current_heading = sbot_get_best_heading_deg();
                const double err = sbot_heading_error_deg(target_heading, current_heading);
                if (std::fabs(err) > 4.0) {
                    lemlib::TurnToHeadingParams fine;
                    fine.maxSpeed = 45;
                    fine.minSpeed = 10;
                    sbot_chassis->turnToHeading(target_heading, 350, fine);
                    sbot_wait_until_done_timed("match.turn_to.fine");
                }
            }

            // Do NOT snap odom theta here.
            // Snapping hides real IMU error (log showed pose=180 while IMU=188),
            // and that causes tube/long-goal alignment to drift.
        };

        // Stage 0: optional barrier clearance.
        if (t.clear_barrier_in > 0.0) {
            printf("AWP STAGE 0: clear barrier\n");
            sbot_intake_on_storage();
            sbot_drive_relative(t.clear_barrier_in, 1200, true);
            sbot_print_pose("after clear barrier");
        }

        // Stage 1: collect nearby block cluster FIRST.
        printf("AWP STAGE 1: cluster collect\n");
        sbot_intake_on_storage();
        // Slow down the cluster run so balls don't fly out.
        drive_to_speed(t.cluster1, true, 55, "match.drive_to.cluster1");
        drive_to_speed(t.cluster1_sweep, true, 50, "match.drive_to.cluster1_sweep");
        sbot_run_for_ms(t.cluster_collect_ms);
        sbot_print_pose("after cluster");

        // Stage 2: score Center Goal – Lower (front) OR Center Goal – Middle (back)
        if (low_goal_case) {
            printf("AWP STAGE 2: CENTER LOWER (front score)\n");
            turn_to(t.low_goal_heading_deg);
            sbot_intake_on_storage();
            // Drive to the computed goal pose target (absolute x/y). This is more robust than
            // projecting onto the heading line (which can clamp to 0" and result in no forward motion).
            if (sbot_chassis) {
                const double goal_heading = sbot_apply_alliance_transform_heading_only(t.low_goal_heading_deg, alliance_);
                SbotPoint target = sbot_apply_alliance_transform_only(t.low_goal_approach, alliance_);
                if (t.use_low_goal_contact) {
                    const SbotPoint contact = sbot_apply_alliance_transform_only(t.low_goal_contact, alliance_);
                    target = sbot_pose_from_front_contact(contact, goal_heading, SBOT_FRONT_BUMPER_IN);
                    printf(
                        "CENTER LOWER contact->pose: contact(%.2f,%.2f) heading=%.1f front=%.2f => pose(%.2f,%.2f)\n",
                        contact.x,
                        contact.y,
                        goal_heading,
                        SBOT_FRONT_BUMPER_IN,
                        target.x,
                        target.y
                    );
                }

                // Enforce ending square to the goal; otherwise the final push-in can go sideways.
                lemlib::MoveToPoseParams goalParams;
                goalParams.forwards = true;
                goalParams.maxSpeed = 110;
                goalParams.minSpeed = 12;
                goalParams.earlyExitRange = 0;
                goalParams.lead = 0.35;

                {
                    const auto pose0 = sbot_chassis->getPose();
                    const double dx = target.x - pose0.x;
                    const double dy = target.y - pose0.y;
                    printf(
                        "LOW GOAL moveToPose: from(%.2f,%.2f,%.1f) to(%.2f,%.2f,%.1f) d=(%.2f,%.2f)\n",
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
                sbot_chassis->moveToPose(target.x, target.y, goal_heading, 2500, goalParams);
                sbot_wait_until_done_timed("match.approach_low_goal");

                {
                    const auto pose1 = sbot_chassis->getPose();
                    const double dx = target.x - pose1.x;
                    const double dy = target.y - pose1.y;
                    printf(
                        "LOW GOAL after moveToPose: at(%.2f,%.2f,%.1f) err=(%.2f,%.2f)\n",
                        pose1.x,
                        pose1.y,
                        pose1.theta,
                        dx,
                        dy
                    );
                }
            } else {
                drive_to(t.low_goal_approach, true /* forwards */);
            }

            // If we're still short, push in a bit more before scoring.
            turn_to(t.low_goal_heading_deg);
            sbot_drive_relative(4.0, 800, true /* forwards */);
            // Ensure we spend at least 1s actively scoring.
            sbot_score_low_for(std::max<uint32_t>(t.low_goal_score_ms, 1000));
            // Reasonable wait to let the last ball clear.
            pros::delay(200);
            sbot_print_pose("after center lower (front)");
        } else {
            printf("AWP STAGE 2: CENTER MIDDLE (back score)\n");
            turn_to(t.mid_goal_heading_deg);
            sbot_intake_on_storage();
            // Confirmed: we want REAR facing the goal, so we back into the scoring spot.
            {
                const double goal_heading = sbot_apply_alliance_transform_heading_only(t.mid_goal_heading_deg, alliance_);
                SbotPoint mid_target = sbot_apply_alliance_transform_only(t.mid_goal_approach, alliance_);
                if (t.use_mid_goal_contact) {
                    const SbotPoint contact = sbot_apply_alliance_transform_only(t.mid_goal_contact, alliance_);
                    mid_target = sbot_pose_from_back_contact(contact, goal_heading, SBOT_BACK_BUMPER_IN);
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

                // IMPORTANT: mid-goal has been approaching at an angle.
                // Use pose pursuit so we converge x/y AND end square to the goal.
                if (sbot_chassis) {
                    lemlib::MoveToPoseParams poseParams;
                    poseParams.forwards = false; // back into the goal
                    poseParams.maxSpeed = 110;
                    poseParams.minSpeed = 12;
                    poseParams.earlyExitRange = 0;
                    poseParams.lead = 0.35;

                    {
                        const auto pose0 = sbot_chassis->getPose();
                        const double dx = mid_target.x - pose0.x;
                        const double dy = mid_target.y - pose0.y;
                        printf(
                            "MID GOAL moveToPose: from(%.2f,%.2f,%.1f) to(%.2f,%.2f,%.1f) d=(%.2f,%.2f)\n",
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
                    sbot_chassis->moveToPose(mid_target.x, mid_target.y, goal_heading, t.drive_timeout_ms, poseParams);
                    sbot_wait_until_done_timed("match.approach_mid_goal_pose");

                    {
                        const auto pose1 = sbot_chassis->getPose();
                        const double dx = mid_target.x - pose1.x;
                        const double dy = mid_target.y - pose1.y;
                        printf(
                            "MID GOAL after moveToPose: at(%.2f,%.2f,%.1f) err=(%.2f,%.2f)\n",
                            pose1.x,
                            pose1.y,
                            pose1.theta,
                            dx,
                            dy
                        );
                    }
                } else {
                    drive_to(mid_target, false /* backwards */);
                }
            }
            // Ensure we spend at least 1s actively scoring.
            sbot_score_mid_for(std::max<uint32_t>(t.mid_goal_score_ms, 1000));
            sbot_print_pose("after center middle (back)");
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
                // Drive to the retreat point while reversing (keeps the retreat "straight back" concept,
                // but actually lands on the desired endpoint instead of a 1D projection).
                lemlib::MoveToPointParams params;
                params.forwards = false;
                // Retreat accuracy matters a lot for the loader line-up; slow slightly to reduce skid/overshoot.
                params.maxSpeed = 80;
                params.minSpeed = 0;
                params.earlyExitRange = 0;
                sbot_chassis->moveToPoint(retreat.x, retreat.y, t.drive_timeout_ms, params);
                sbot_wait_until_done_timed("match.retreat");

                // If we are still noticeably off the retreat point, retry once slower.
                {
                    const auto pose_now = sbot_chassis->getPose();
                    const SbotPoint now{pose_now.x, pose_now.y};
                    if (sbot_dist_in(now, retreat) > 2.0) {
                        lemlib::MoveToPointParams retry = params;
                        retry.maxSpeed = 60;
                        sbot_chassis->moveToPoint(retreat.x, retreat.y, t.drive_timeout_ms, retry);
                        sbot_wait_until_done_timed("match.retreat.retry");
                    }
                }
            } else {
                // If chassis isn't available (shouldn't happen), prefer the same "backwards" intent.
                drive_to(t.post_score_retreat_point, false /* backwards */);
            }

            // Immediately deploy the match loader once we're at the retreat point.
            if (low_goal_case && sbot_batch_loader) {
                sbot_batch_loader->extend();
                pros::delay(120);
            }
        } else {
            // Back straight out from the goal (reverse of the approach direction).
            // Using LemLib params.forwards=false means we keep heading but drive backwards.
            sbot_drive_relative(t.post_score_retreat_back_dist_in, 2500, false /* backwards */);
        }
        // Turn to face the tube/loader. Use shortest-path and then a fine correction
        // so we land very close to the requested heading (e.g. 180°).
        turn_to(t.tube_face_heading_deg);
        // Small settle after turning to face the tube/loader.
        pros::delay(80);
        sbot_print_pose("after retreat/turn");
        sbot_print_sensors("after retreat/turn");

        // Stage 4: loader pull
        printf("AWP STAGE 4: loader1 pull\n");
        sbot_intake_on_storage();
        if (low_goal_case) {
            // Loader should already be deployed right after the retreat.
            pros::delay(60);

            // Red Left (and Blue Right): tube/long-goal are on the same X line.
            if (sbot_chassis) {
                // First, explicitly get onto the shared X-line (tube + long goal).
                {
                    const auto pose0 = sbot_chassis->getPose();
                    const double line_x = sbot_apply_alliance_transform_only({t.loader_long_goal_line_x, 0.0}, alliance_).x;
                    lemlib::MoveToPointParams xParams;
                    xParams.forwards = true;
                    xParams.maxSpeed = 55;
                    xParams.minSpeed = 0;
                    xParams.earlyExitRange = 0;
                    sbot_chassis->moveToPoint(line_x, pose0.y, 1600, xParams);
                    sbot_wait_until_done_timed("match.align_line_x");
                }

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

                // Fast, deterministic tube approach:
                // - stay on the shared X-line (done above)
                // - ensure we're facing the tube
                // - drive straight along the tube heading to the target Y
                // We just turned to face the tube in Stage 3.
                // Only do a very short correction here if we're clearly off.
                {
                    const double current_heading = sbot_get_best_heading_deg();
                    const double err = sbot_heading_error_deg(tube_heading, current_heading);
                    if (std::fabs(err) > 6.0) {
                        lemlib::TurnToHeadingParams quick;
                        quick.maxSpeed = 55;
                        quick.minSpeed = 10;
                        sbot_chassis->turnToHeading(tube_heading, 700, quick);
                        sbot_wait_until_done_timed("match.turn_to.tube_correct");
                    }
                }
                // Enforce ending square to the loader: use pose pursuit (fixes wrong-angle tube contact).
                {
                    lemlib::MoveToPoseParams poseParams;
                    poseParams.forwards = true;
                    poseParams.maxSpeed = 110;
                    poseParams.minSpeed = 12;
                    poseParams.earlyExitRange = 0;
                    poseParams.lead = 0.35;

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
                    sbot_chassis->moveToPose(tube_pose_target.x, tube_pose_target.y, tube_heading, 2600, poseParams);
                }
                sbot_wait_until_done_timed("match.approach_tube_pose");

                {
                    const auto pose1 = sbot_chassis->getPose();
                    const double dx = tube_pose_target.x - pose1.x;
                    const double dy = tube_pose_target.y - pose1.y;
                    printf(
                        "TUBE after moveToPose: at(%.2f,%.2f,%.1f) err=(%.2f,%.2f)\n",
                        pose1.x,
                        pose1.y,
                        pose1.theta,
                        dx,
                        dy
                    );
                }
            }

            // Pull from the Loader while intaking + actuate batch loader.
            sbot_run_for_ms(t.tube_pull_ms);
            if (sbot_batch_loader) sbot_batch_loader->retract();
            pros::delay(60);

            // Pull off the loader so we can back into the long goal cleanly.
            if (sbot_chassis) {
                const SbotPoint pulloff = sbot_apply_alliance_transform_only(t.tube1_pulloff, alliance_);
                const double tube_heading = sbot_apply_alliance_transform_heading_only(t.tube_face_heading_deg, alliance_);

                // Back out using a short, robust relative move.
                // The pulloff point is roughly 2-3" away; using odom-point pursuit here can time out.
                (void)tube_heading;
                (void)pulloff;
                sbot_drive_relative(3.0, 900, false /* backwards */);
            }
            sbot_print_pose("after loader1 (relative)");
        } else {
            // Other starts: keep the existing absolute-point approach for now.
            drive_to(t.tube1, true);
            sbot_run_for_ms(t.tube_pull_ms);
            drive_to(t.tube1_pulloff, true);
            sbot_print_pose("after loader1");
        }

        if (solo_) {
            // Solo AWP: do a second loader pull before the second score.
            printf("AWP STAGE 4b: loader2 pull (solo)\n");
            sbot_intake_on_storage();
            drive_to(t.tube2, true);
            sbot_run_for_ms(t.tube_pull_ms);
            drive_to(t.tube2_pulloff, true);
            sbot_print_pose("after loader2");
        }

        // Stage 5: second score
        // Desired flow (short travel): cluster -> Center (Lower/Middle) -> loader(s) -> near end of Long Goal.
        printf("AWP STAGE 5: second score\n");
        printf("AWP STAGE 5: LONG GOAL END (near loader)%s\n", solo_ ? " (solo)" : "");

        if (low_goal_case) {
            // Red Left (and Blue Right): after loading while facing the alliance wall, back up the measured distance into the Long Goal end.
            sbot_intake_on_storage();
            // Long goal can involve pushing the goal, so odom may not "arrive". Use stall-detect to early-exit.
            sbot_drive_relative_stall_exit(t.high_goal_back_in_from_tube_in, 3200, false /* backwards */, 300, 0.35, 80);
        } else if (t.high_goal_back_in_from_tube_in > 0.0) {
            // We just finished loader pulling while facing the loader.
            // Backing up keeps the intake facing the loader and puts the rear into the Long Goal end.
            sbot_intake_on_storage();
            sbot_drive_relative_stall_exit(t.high_goal_back_in_from_tube_in, 4000, false /* backwards */, 300, 0.35, 80);
        } else {
            turn_to(t.high_goal_heading_deg);
            sbot_intake_on_storage();
            // Default to backing in (common when scoring from the rear). Tune as needed.
            drive_to(t.high_goal_approach, false /* backwards */);
        }

        // Ensure we spend at least 1s actively scoring.
        sbot_score_top_for(std::max<uint32_t>(t.high_goal_score_ms, 1000));
        sbot_print_pose("after high goal");

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
        "Test: Drive",           // 10
        "Test: Turn",            // 11
        "Test: Intake",          // 12
        "Test: Indexer"          // 13
    };

    if (idx < 0) return "<invalid>";
    if (idx > 13) return "<invalid>";
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

    const int max_index = 13; // 0..13

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
        case SbotAutoMode::TEST_DRIVE:   runTestDrive();   break;
        case SbotAutoMode::TEST_TURN:    runTestTurn();    break;
        case SbotAutoMode::TEST_INTAKE:  runTestIntake();  break;
        case SbotAutoMode::TEST_INDEXER: runTestIndexer(); break;
        case SbotAutoMode::DISABLED:
        default:
            // Do nothing
            break;
    }
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

    // LemLib convention: +Y forward at 0°, +X right.
    // We use boomerang (moveToPose) to blend the corner turn into the approach so it doesn't stop.
    lemlib::MoveToPoseParams params;
    params.forwards = true;
    params.maxSpeed = 60;      // reduced speed
    params.minSpeed = 0;       // prioritize reaching the actual corner (less corner-cutting)
    params.earlyExitRange = 0; // no early exit
    params.lead = 0.35;        // straighter approach

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

        sbot_chassis->moveToPose(targets[i].x, targets[i].y, targets[i].theta, kTimeoutMs, params);
        sbot_chassis->waitUntilDone();

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
    sbot_chassis->waitUntilDone();

    // 3) Reverse sequence, driving backwards ("back direction")
    // Start with a right turn, then drive backwards 2 tiles, and repeat.
    printf("SBOT AUTON TEST: RECTANGLE reverse (backwards)\n");

    lemlib::MoveToPoseParams backParams = params;
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

        sbot_chassis->moveToPose(backTargets[i].x, backTargets[i].y, backTargets[i].theta, kTimeoutMs, backParams);
        sbot_chassis->waitUntilDone();

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
}

void SbotAutonomousSystem::runTestTurn() {
    printf("SBOT AUTON TEST: TURN\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("after setPose");
    sbot_chassis->turnToHeading(90, 3000);
    sbot_chassis->waitUntilDone();
    sbot_print_pose("after 90");
    sbot_chassis->turnToHeading(0, 3000);
    sbot_chassis->waitUntilDone();
    sbot_print_pose("after 0");

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
