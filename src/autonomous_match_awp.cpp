/**
 * autonomous_match_awp.cpp
 *
 * AWP match autonomous routines (half-field + solo).
 */
#include "autonomous_match_awp.h"
#include "autonomous_match_helpers.h"

#include "autonomous_constants.h"
#include "intake.h"
#include "indexer.h"
#include "lemlib_config_sbot.h"
#include "pneumatics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

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
    sbot_match_score_top_for(tune.top_score_ms);
    sbot_print_pose("after top score");

    sbot_safe_stop_mechanisms();
    printf("SBOT AUTON: MATCH AUTO RR-1to5 complete\n");
}

void sbot_run_match_auto(
    SbotAutoSide side,
    SbotAutoAlliance alliance,
    bool solo_awp,
    bool start_from_cluster_sweep,
    bool stop_after_stage2,
    bool stage2_skip_pre_turn
) {
    // Match auto is currently focused on achieving our portion of the AWP tasks.
    auto sbot_run_awp_half_field = [&](SbotAutoSide side_, SbotAutoAlliance alliance_, bool solo_) {
        const uint32_t auton_start_ms = pros::millis();
        printf("SBOT AUTON: %s (%s %s)\n",
               solo_ ? "SOLO AWP" : "AWP HALF",
               (alliance_ == SbotAutoAlliance::RED) ? "RED" : "BLUE",
               (side_ == SbotAutoSide::RIGHT) ? "RIGHT" : "LEFT");

        // Ensure run logs always prove which binary is deployed.
        printf("MARKER05\n");
        printf("\n=== sbot_run_match_auto() ENTER ===\n");
        printf("SBOT: side=%d alliance=%d solo=%d\n", (int)side_, (int)alliance_, solo_awp);
        printf("SBOT BUILD TAG: %s %s\n", __DATE__, __TIME__);
        fflush(stdout);

        if (!validateSbotLemLibInitialization()) {
            printf("MARKER99 ERROR: LemLib validation failed!\n");
            fflush(stdout);
            return;
        }
        printf("MARKER06\n");
        printf("SBOT: LemLib validation passed\n");
        fflush(stdout);

        // Match-auton drivetrain behavior.
        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        sbot_safe_stop_mechanisms();

        const bool low_goal_case =
            (alliance_ == SbotAutoAlliance::RED) ? (side_ == SbotAutoSide::RIGHT) : (side_ == SbotAutoSide::RIGHT);

        // Select the Jerry start used for conversions + printing.
        // Canonical routes:
        // - low_goal_case => canonical RED RIGHT geometry => RR Jerry start
        // - else          => canonical RED LEFT geometry => RL Jerry start
        if (low_goal_case) {
            sbot_jerry_start_x = SBOT_JERRY_START_RR_X;
            sbot_jerry_start_y = SBOT_JERRY_START_RR_Y;
        } else {
            sbot_jerry_start_x = SBOT_JERRY_START_RL_X;
            sbot_jerry_start_y = SBOT_JERRY_START_RL_Y;
        }

        const auto t = low_goal_case ? sbot_awp_half_red_right_tuning() : sbot_awp_half_red_left_tuning();

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

        // Start intake at the very beginning of autonomous
        sbot_intake_on_storage();
        printf("AUTONOMOUS: intake started at beginning\n");

        if (!start_from_cluster_sweep) {
            // Stage 0: optional barrier clearance.
            if (t.clear_barrier_in > 0.0) {
                printf("AWP STAGE 0: clear barrier [T+%ums]\n", pros::millis() - auton_start_ms);
                sbot_match_drive_relative(t.clear_barrier_in, 1200, true);
                sbot_print_pose("after clear barrier");
            }

            // Stage 1: collect nearby block cluster FIRST.
            printf("MARKER07\n");
            printf("AWP STAGE 1: cluster collect [T+%ums]\n", pros::millis() - auton_start_ms);
            // Desired sequence (intake is already running from the start):
            // 1) approach cluster with intake running
            // 2) deploy loader DURING forward motion (so it lands ON TOP of balls, trapping them)
            // 3) finish drive to cluster
            // 4) dwell to collect
            // 5) turn toward center goal
            if (!sbot_chassis) return;

            // 1) approach cluster (intake already running)
            {
                const SbotPoint cluster_target = sbot_apply_alliance_transform_only(t.cluster1, alliance_);
                printf("CLUSTER: Jerry coord (-21, 21) -> target (%.2f, %.2f)\n", cluster_target.x, cluster_target.y);
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
                pros::delay(1200); // Increased delay by 750ms to deploy matchloader later
                if (sbot_batch_loader) {
                    sbot_batch_loader->extend();
                    printf("CLUSTER: loader deployed during approach\n");
                }

                // Shorter wait since loader descends faster now with extra piston
                pros::delay(100);

                // Intake is already running from the start of autonomous

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
        printf("MARKER08 low_goal_case=%d [T+%ums]\n", low_goal_case, pros::millis() - auton_start_ms);
        fflush(stdout);
        if (low_goal_case) {
            printf("AWP STAGE 2: CENTER LOWER (front score) [T+%ums]\n", pros::millis() - auton_start_ms);
            if (stage2_skip_pre_turn) {
                printf("AWP STAGE 2: skipping pre-turn (approach sequence handles turn+drive)\n");
                // If we skip the explicit pre-turn, retract before approaching the goal for safety.
                if (sbot_batch_loader) {
                    sbot_batch_loader->retract();
                    pros::delay(100);
                }
            } else {
                turn_to(t.low_goal_heading_deg);

                // Retract loader ONLY after the turn toward the center goal.
                if (sbot_batch_loader) {
                    sbot_batch_loader->retract();
                    pros::delay(100);
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
                    target_canonical = sbot_match_pose_from_front_contact(t.low_goal_contact, goal_heading_canonical, front_effective);
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
                    sbot_match_turn_point_turn(
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
                            sbot_match_turn_point_turn(
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
                sbot_run_for_ms(50);

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

            // Score at low goal (front-score: intake reverses to spit balls out)
            printf("AWP STAGE 2: LOW GOAL SCORING [T+%ums]\n", pros::millis() - auton_start_ms);
            sbot_match_score_low_for(std::max<uint32_t>(t.low_goal_score_ms, 2000));
            sbot_print_auton_elapsed("low_goal_score_done");
            sbot_print_pose("after center lower (front)");
            sbot_print_jerry_pose("after center lower (front)");
        } else {
            printf("MARKER09 [T+%ums]\n", pros::millis() - auton_start_ms);
            printf("AWP STAGE 2: CENTER MIDDLE (back score) [T+%ums]\n", pros::millis() - auton_start_ms);
            turn_to(t.mid_goal_heading_deg);

            // Retract loader ONLY after the turn toward the center goal.
            if (sbot_batch_loader) {
                sbot_batch_loader->retract();
                pros::delay(100);
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
                    mid_target_canonical = sbot_match_pose_from_back_contact(t.mid_goal_contact, goal_heading_canonical, SBOT_BACK_BUMPER_IN);
                    mid_target = t.mid_goal_approach;
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
                    driveParams.maxSpeed = SBOT_MATCH_MAX_SPEED / 2;
                    driveParams.minSpeed = 0;
                    driveParams.earlyExitRange = 0;

                    const uint32_t goal_wait_timeout_ms = 2000;
                    const uint32_t goal_motion_timeout_ms = 3000;
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
                    sbot_match_turn_point_turn(
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
                            sbot_match_turn_point_turn(
                                "match.approach_mid_goal_pose.retry",
                                mid_target.x,
                                mid_target.y,
                                goal_heading,
                                t.turn_timeout_ms,
                                2000,
                                turnParams,
                                retryDrive,
                                800,
                                450,
                                0.5,
                                6.0
                            );
                            sbot_lemlib_debug_window_end("match.approach_mid_goal_pose.retry");
                        }
                    }
                }

                // Start reverse unjam immediately on arrival (runs during settle + debug)
                if (sbot_intake) {
                    sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
                    sbot_intake->update();
                }

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
            // Ensure we spend at least 2s actively scoring.
            // Reverse already running from above — let it clear for a bit more.
            pros::delay(200);
            sbot_match_score_mid_for(std::max<uint32_t>(t.mid_goal_score_ms, 2000), true);
            printf("MID GOAL scoring done [T+%ums]\n", pros::millis() - auton_start_ms);
            sbot_print_pose("after center middle (back)");
        }

        if (stop_after_stage2) {
            sbot_safe_stop_mechanisms();
            printf("SBOT AUTON TEST: sweep->center score complete\n");
            return;
        }

        // Stage 3: retreat then face loader
        printf("AWP STAGE 3: retreat + face loader [T+%ums]\n", pros::millis() - auton_start_ms);
        sbot_safe_stop_mechanisms();
        // Retreat: either to an absolute point (preferred for RL non-solo), or straight back-out.
        if (t.use_post_score_retreat_point) {
            // Drive directly to the retreat point without turning first.
            const SbotPoint retreat = sbot_apply_alliance_transform_only(t.post_score_retreat_point, alliance_);
            printf("RETREAT target: (%.2f, %.2f)\n", retreat.x, retreat.y);
            if (sbot_chassis) {
                const double retreat_heading = sbot_get_best_heading_deg();
                lemlib::TurnToHeadingParams turnParams;
                turnParams.maxSpeed = SBOT_MATCH_TURN_MAX_SPEED;
                turnParams.minSpeed = 0;

                lemlib::MoveToPointParams driveParams;
                driveParams.forwards = false;  // Drive backwards to retreat (avoids 180° spin)
                driveParams.maxSpeed = 60;  // Controlled speed for alignment
                driveParams.minSpeed = 0;
                driveParams.earlyExitRange = 0;

                sbot_match_turn_point_turn(
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
                    12.0,
                    false,
                    false
                );

                // If we are still noticeably off the retreat point, retry once slower.
                {
                    const auto pose_now = sbot_chassis->getPose();
                    const SbotPoint now{pose_now.x, pose_now.y};
                    if (sbot_dist_in(now, retreat) > 2.0) {
                        const double retry_heading = sbot_get_best_heading_deg();
                        lemlib::MoveToPointParams retryDrive = driveParams;
                        retryDrive.forwards = false;  // Same as primary: backwards
                        retryDrive.maxSpeed = 50;  // Controlled speed for alignment
                        sbot_match_turn_point_turn(
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
        if (sbot_batch_loader) {
            sbot_batch_loader->extend();
            pros::delay(250);
        }
        sbot_print_pose("after retreat/turn");
        sbot_print_sensors("after retreat/turn");

        // Stage 4: loader pull
        printf("AWP STAGE 4: loader1 pull [T+%ums]\n", pros::millis() - auton_start_ms);
        sbot_intake_on_storage();
        {
            // Loader is deployed at the end of Stage 3 (after facing it).
            pros::delay(30);

            if (sbot_chassis) {
                const double tube_heading = sbot_apply_alliance_transform_heading_only(t.tube_face_heading_deg, alliance_);
                SbotPoint tube_pose_target = sbot_apply_alliance_transform_only(t.tube1, alliance_);
                if (t.use_tube1_contact) {
                    const SbotPoint tube_contact = sbot_apply_alliance_transform_only(t.tube1_contact, alliance_);
                    const double front_effective = SBOT_FRONT_BUMPER_IN + t.loader_down_extra_front_in;
                    tube_pose_target = sbot_match_pose_from_front_contact(tube_contact, tube_heading, front_effective);
                    printf(
                        "TUBE contact->pose: contact(%.2f,%.2f) heading=%.1f frontEff=%.2f => pose(%.2f,%.2f)\n",
                        tube_contact.x,
                        tube_contact.y,
                        tube_heading,
                        front_effective,
                        tube_pose_target.x,
                        tube_pose_target.y
                    );
                }

                tube_pose_target = sbot_from_jerry(t.tube1.x, t.tube1.y);  // OVERRIDE for testing specific target pose

                // Use moveToPoint (not moveToPose) to drive straight — heading is already set from Stage 3.
                {
                    lemlib::MoveToPointParams driveParams;
                    driveParams.forwards = true;
                    driveParams.maxSpeed = SBOT_MATCH_MAX_SPEED / 2;
                    driveParams.minSpeed = 0;

                    sbot_print_jerry_target("tube_pose_target", tube_pose_target.x, tube_pose_target.y);
                    sbot_chassis->moveToPoint(tube_pose_target.x, tube_pose_target.y, 1500, driveParams);
                    sbot_wait_until_done_or_timed_out_timed("match.approach_tube", 800);
                }

                // Cancel any lingering LemLib motion from the approach before starting extra seat
                sbot_chassis->cancelAllMotions();

                // Seat into the match loader using configurable extra distance.
                if (t.tube_extra_seat_in > 0.0) {
                    printf("TUBE extra seat: +%.1fin\n", t.tube_extra_seat_in);
                    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
                    sbot_match_drive_relative(t.tube_extra_seat_in, 250, true /* forwards */);
                }
            } else {
                drive_to(t.tube1, true);
            }

            // Pull from the Loader while intaking — push forward slowly to maintain contact.
            if (sbot_chassis) {
                sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
                // Slowly creep forward into loader while collecting
                sbot_match_drive_relative_stall_exit(3.0, t.tube_pull_ms, true /* forwards */, 200, 0.15, 55);
            } else {
                sbot_run_for_ms(t.tube_pull_ms);
            }

            // IMMEDIATELY stop intake and open flap after matchloading
            if (sbot_intake) {
                sbot_intake->setMode(IntakeMode::OFF);
                sbot_intake->update();
            }
            if (sbot_goal_flap) sbot_goal_flap->close();
            printf("MATCHLOADER: intake OFF, flap CLOSED after pull\n");

            if (sbot_batch_loader) sbot_batch_loader->retract();
            pros::delay(30);

            // Return to BRAKE mode after loader pull
            if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

            sbot_print_pose("after loader1");
            sbot_print_jerry_pose("after loader1");
            printf("LOADER done [T+%ums]\n", pros::millis() - auton_start_ms);
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
            sbot_match_score_mid_for(t.mid_goal_score_ms);
            sbot_print_pose("after solo center middle");

            // Done with Solo AWP
            sbot_safe_stop_mechanisms();
            printf("SBOT AUTON: SOLO AWP complete\n");
            return;
        }

        // Stage 5: second score
        // Desired flow (short travel): cluster -> Center (Lower/Middle) -> loader(s) -> near end of Long Goal.
        printf("AWP STAGE 5: second score [T+%ums]\n", pros::millis() - auton_start_ms);
        printf("AWP STAGE 5: LONG GOAL END (near loader)%s [T+%ums]\n", solo_ ? " (solo)" : "", pros::millis() - auton_start_ms);

        if (low_goal_case) {
            // Red Right (and Blue Left): Back into long goal at Jerry (-31, 48).
            // This positions the SCORER (back of robot) at the goal, not just the center point.
            // From loader, we drive backwards toward the goal.
            // Start intake/indexer but keep flap CLOSED during approach.
            if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
            if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
            printf("LONG GOAL: scorer started during approach (flap closed)\n");
            const SbotPoint long_goal_end_canonical = sbot_from_jerry(RedRight::LONG_GOAL_END_JERRY_X, RedRight::LONG_GOAL_END_JERRY_Y);
            printf("LONG GOAL end: canonical(%.2f,%.2f) Jerry(%.1f,%.1f)\n",
                   long_goal_end_canonical.x, long_goal_end_canonical.y,
                   RedRight::LONG_GOAL_END_JERRY_X, RedRight::LONG_GOAL_END_JERRY_Y);

            // Faster backwards approach to long goal — score during the drive
            if (sbot_chassis) {
                const SbotPoint target = sbot_apply_alliance_transform_only(long_goal_end_canonical, alliance_);
                const double target_heading = sbot_apply_alliance_transform_heading_only(180.0, alliance_);
                {
                    const auto p = sbot_chassis->getPose();
                    printf("LONG GOAL start: pose(%.2f,%.2f,%.1f) target(%.2f,%.2f) heading=%.1f dist=%.1f\n",
                           p.x, p.y, p.theta, target.x, target.y, target_heading,
                           std::sqrt((target.x-p.x)*(target.x-p.x) + (target.y-p.y)*(target.y-p.y)));
                }
                lemlib::MoveToPoseParams params;
                params.forwards = false;
                params.maxSpeed = 90;
                params.minSpeed = 0;

                sbot_chassis->moveToPose(target.x, target.y, target_heading, 4000, params);
                // Update intake/indexer during approach; stall-detect to exit early if stuck
                const uint32_t approach_start = pros::millis();
                auto last_pose = sbot_chassis->getPose();
                uint32_t last_moved_ms = approach_start;
                bool approach_stalled = false;
                while (sbot_chassis->isInMotion() && (pros::millis() - approach_start < 4000)) {
                    if (sbot_intake) sbot_intake->update();
                    if (sbot_indexer) sbot_indexer->update();
                    pros::delay(10);
                    const auto now_pose = sbot_chassis->getPose();
                    const double ddx = now_pose.x - last_pose.x;
                    const double ddy = now_pose.y - last_pose.y;
                    if (std::sqrt(ddx*ddx + ddy*ddy) >= 0.3) {
                        last_pose = now_pose;
                        last_moved_ms = pros::millis();
                    }
                    if ((pros::millis() - approach_start > 500) && (pros::millis() - last_moved_ms >= 400)) {
                        approach_stalled = true;
                        printf("LONG GOAL approach stall-exit after %ums\n", pros::millis() - approach_start);
                        break;
                    }
                }
                sbot_chassis->cancelAllMotions();
            } else {
                drive_to(long_goal_end_canonical, false /* backwards */);
            }

            // Open flap NOW so it starts actuating during the final push
            if (sbot_goal_flap) sbot_goal_flap->open();
            // Final push into goal with stall detection
            sbot_match_drive_relative_stall_exit(4.0, 1500, false /* backwards */, 300, 0.35, 40);
        } else {
            // Red Left: Start intake/indexer but keep flap CLOSED during approach.
            if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
            if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
            printf("LONG GOAL: top scorer started during approach (flap closed)\n");
            const SbotPoint long_goal_end_canonical = sbot_from_jerry(RedLeft::LONG_GOAL_END_JERRY_X, RedLeft::LONG_GOAL_END_JERRY_Y);
            printf("LONG GOAL end: canonical(%.2f,%.2f) Jerry(%.1f,%.1f)\n",
                   long_goal_end_canonical.x, long_goal_end_canonical.y,
                   RedLeft::LONG_GOAL_END_JERRY_X, RedLeft::LONG_GOAL_END_JERRY_Y);

            if (sbot_chassis) {
                //const SbotPoint target = sbot_apply_alliance_transform_only(long_goal_end_canonical, alliance_);
                const double target_heading = sbot_apply_alliance_transform_heading_only(180.0, alliance_);
                lemlib::MoveToPoseParams params;
                params.forwards = false;
                params.maxSpeed = 90;
                params.minSpeed = 0;

                sbot_chassis->moveToPose(long_goal_end_canonical.x, long_goal_end_canonical.y, target_heading, 4000, params);
                // Update intake/indexer during approach; stall-detect to exit early if stuck
                const uint32_t approach_start = pros::millis();
                auto last_pose = sbot_chassis->getPose();
                uint32_t last_moved_ms = approach_start;
                bool approach_stalled = false;
                while (sbot_chassis->isInMotion() && (pros::millis() - approach_start < 4000)) {
                    if (sbot_intake) sbot_intake->update();
                    if (sbot_indexer) sbot_indexer->update();
                    pros::delay(10);
                    const auto now_pose = sbot_chassis->getPose();
                    const double ddx = now_pose.x - last_pose.x;
                    const double ddy = now_pose.y - last_pose.y;
                    if (std::sqrt(ddx*ddx + ddy*ddy) >= 0.3) {
                        last_pose = now_pose;
                        last_moved_ms = pros::millis();
                    }
                    if ((pros::millis() - approach_start > 500) && (pros::millis() - last_moved_ms >= 400)) {
                        approach_stalled = true;
                        printf("LONG GOAL approach stall-exit after %ums\n", pros::millis() - approach_start);
                        break;
                    }
                }
                sbot_chassis->cancelAllMotions();
            } else {
                drive_to(long_goal_end_canonical, false /* backwards */);
            }

            // Open flap NOW so it starts actuating during the final push
            if (sbot_goal_flap) sbot_goal_flap->open();
            // Final push into goal with stall detection
            sbot_match_drive_relative_stall_exit(4.0, 1500, false /* backwards */, 300, 0.35, 40);
        }

        // Score for maximum time - let autonomous end while scoring (30s ensures we're always scoring)
        printf("LONG GOAL: scoring begins [T+%ums]\n", pros::millis() - auton_start_ms);
        sbot_match_score_top_for(30000);
        sbot_print_pose("after high goal");
        sbot_print_jerry_pose("after high goal");

        // Stage 6: ensure final position is clear of park-zone barrier.
        printf("AWP STAGE 6: end safe [T+%ums]\n", pros::millis() - auton_start_ms);
        sbot_safe_stop_mechanisms();
        // For now: do NOT drive back toward center. Stay at the long goal end.
        sbot_print_pose("end safe (no move)");

        sbot_safe_stop_mechanisms();
        printf("SBOT AUTON: %s complete [T+%ums]\n", solo_ ? "SOLO AWP" : "AWP HALF", pros::millis() - auton_start_ms);
    };

    sbot_run_awp_half_field(side, alliance, solo_awp);
}
