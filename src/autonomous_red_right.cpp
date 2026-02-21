/**
 * red_left.cpp
 *
 * Cleaned up Red left autonomous routine for sbot.
 */
#include "autonomous_match_awp.h"
#include "autonomous_infrastructure.h"
#include "autonomous_constants.h"
#include "config_sbot.h"
#include "lemlib_config_sbot.h"
#include "intake.h"
#include "indexer.h"
#include "pneumatics.h"
#include <cmath>
#include <cstdio>
#include <algorithm>

// Anti-stall move: cancels if movement <0.1in over 10 intervals (200ms)
static void moveToPointWithAntiStall(double target_x, double target_y, double heading_deg, int timeout_ms, int maxSpeed = 90, bool forwards = true) {
    if (!sbot_chassis) return;
    
    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = maxSpeed;
    
    // Start the movement
    sbot_chassis->moveToPoint(target_x, target_y, timeout_ms, params);
    
    // Wait with anti-stall detection
    double last_x = sbot_chassis->getPose().x;
    double last_y = sbot_chassis->getPose().y;
    int stall_count = 0;
    const uint32_t start = pros::millis();
    
    while (sbot_chassis->isInMotion() && (pros::millis() - start < static_cast<uint32_t>(timeout_ms))) {
        pros::delay(20);
        double cur_x = sbot_chassis->getPose().x;
        double cur_y = sbot_chassis->getPose().y;
        double dist = std::sqrt((cur_x - last_x)*(cur_x - last_x) + (cur_y - last_y)*(cur_y - last_y));
        
        if (dist < 0.1) {
            stall_count++;
        } else {
            stall_count = 0;
        }
        
        last_x = cur_x;
        last_y = cur_y;
        
        if (stall_count >= 10) {
            printf("ANTI-STALL: Movement <0.1in for 10 intervals, cancelling move\n");
            sbot_chassis->cancelAllMotions();
            break;
        }
    }
}

void sbot_run_red_right_auto() {
    using namespace RedLeft;
    if (!validateSbotLemLibInitialization()) return;
    sbot_safe_stop_mechanisms();

    // 1. Initialization 
    sbot_jerry_start_x = SBOT_JERRY_START_RL_X_BASE;
    sbot_jerry_start_y = SBOT_JERRY_START_RL_Y_BASE * -1;
    sbot_zero_pose_and_sensors(0, 0, 0);  // Robot-relative frame
    sbot_print_pose("red right start");

    // Convert Jerry coords to robot-relative
    auto cluster = sbot_from_jerry(CLUSTER1_JERRY_X, CLUSTER1_JERRY_Y * -1);
    auto midgoal = sbot_from_jerry(CENTER_MID_GOAL_JERRY_X, CENTER_MID_GOAL_JERRY_Y * -1);
    auto matchload_retreat = sbot_from_jerry(RETREAT_POINT_JERRY_X, RETREAT_POINT_JERRY_Y * -1);
    auto match_loader = sbot_from_jerry(LOADER_CONTACT_JERRY_X-2.2, LOADER_CONTACT_JERRY_Y * -1);
    auto long_goal = sbot_from_jerry(LONG_GOAL_END_JERRY_X, LONG_GOAL_END_JERRY_Y * -1);

    //------Start of AUTO-------
    
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();
    printf("Intake set to COLLECT_FORWARD at start\n");
    sbot_print_jerry_pose_rotated("red left start");

    // 2. Match Load Approach
    // Start the movement (this runs in the background automatically)
    sbot_chassis->moveToPoint(cluster.x, cluster.y, 10000, {.forwards = true, .maxSpeed = 40}); 
    
    // Wait exactly half a second (500ms) while the robot is driving
    pros::delay(1000);
    
    // Extend loader mid-movement
    if (sbot_batch_loader) sbot_batch_loader->extend();

    // Now pause the main code and wait for the REST of the driving to finish
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to cluster");

    pros::delay(250); // Short delay to allow batch loader to start extending before we turn towards the match loader

    sbot_chassis->turnToHeading(313.5-180, 1000, {.maxSpeed = 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to matchloader heading");

    //if (sbot_batch_loader) sbot_batch_loader->retract();

    /*

    // 3. Drive to Mid Goal
    sbot_chassis->moveToPoint(midgoal.x, midgoal.y, 10000, {false, 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to mid goal");

    // --- NEW: MIDSCORE SEQUENCE ---
    // Reverse for 0.3 seconds to settle the piece
    if (sbot_intake) sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_EJECT);
    sbot_intake->update();
    sbot_indexer->update();
    pros::delay(100);

    // Score middle for 3 seconds
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    sbot_intake->update();
    sbot_indexer->update();
    pros::delay(2000);
    // ------------------------------
    */

    // 4. Retreat and go to Match Loader
    sbot_chassis->moveToPoint(matchload_retreat.x, matchload_retreat.y, 10000, {.forwards = true, .maxSpeed = 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After retreat from match loader");

    sbot_chassis->turnToHeading(270-90, 10000, {.maxSpeed = 80, .minSpeed = 20});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to cross field");

    //if (sbot_batch_loader) sbot_batch_loader->extend();

    // 5. Match Loading (with Anti-Stall)
    //set robot to hold mode
    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);

    moveToPointWithAntiStall(match_loader.x, match_loader.y, 270-90, 2000, 40.0, true);
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After anti-stall move to match loader contact red");
    pros::delay(400); // Wait 0.4s to collect balls

    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);

    // 6. Drive to Long Goal
    moveToPointWithAntiStall(long_goal.x, long_goal.y, 270-90, 10000, 70, false);
    sbot_chassis->waitUntilDone(); 
    sbot_print_jerry_pose_rotated("After anti-stall move to long goal contact");

    // 7. Final Scoring
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();
    
    if (sbot_goal_flap) sbot_goal_flap->open();  // Lift scoring flap
    pros::delay(2000); // Wait 2s to score

    // 7. Reverse intake
    if (sbot_intake) sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_EJECT);
    sbot_intake->update();
    sbot_indexer->update();

    pros::delay(150); // Wait 0.5s to reverse out of the goal


    //7. Final Scoring
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();
    
    if (sbot_goal_flap) sbot_goal_flap->open();  // Lift scoring flap
    pros::delay(10000); // Wait 10s to score

}

SbotAwpHalfTuning sbot_awp_half_red_right_tuning() {
    auto t = sbot_awp_half_red_right_tuning();
    return t;
}