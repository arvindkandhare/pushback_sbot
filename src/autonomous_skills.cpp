/**
 * autonomous_skills.cpp
 *
 * Skills autonomous routine for sbot.
 */
#include "autonomous_skills.h"
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
    
    // Then turn to heading
    //sbot_chassis->turnToHeading(heading_deg, 1000);
    
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

void sbot_run_skills_auto() {
    using namespace Skills;
    if (!validateSbotLemLibInitialization()) return;
    sbot_safe_stop_mechanisms();

    // 1. Initialization (set Jerry start for Skills)
    sbot_jerry_start_x = SKILLS_JERRY_START_X_BASE;
    sbot_jerry_start_y = SKILLS_JERRY_START_Y_BASE;
    sbot_zero_pose_and_sensors(0, 0, 0);  // Robot-relative frame
    sbot_print_pose("skills start");

    // Convert Jerry coords to robot-relative
    auto test_point = sbot_from_jerry_rotated(SKILLS_TO_MATCH_LOADER_JERRY_X, 77.3);
    auto to_match_loader = sbot_from_jerry_rotated(SKILLS_TO_MATCH_LOADER_JERRY_X, SKILLS_TO_MATCH_LOADER_JERRY_Y);
    auto match_loader_contact_red = sbot_from_jerry_rotated(SKILLS_MATCH_LOADER_CONTACT_RED_JERRY_X, SKILLS_MATCH_LOADER_CONTACT_RED_JERRY_Y);
    auto match_loader_retreat = sbot_from_jerry_rotated(SKILLS_MATCH_LOADER_RETREAT_JERRY_X, SKILLS_MATCH_LOADER_RETREAT_JERRY_Y);
    auto going_around_long = sbot_from_jerry_rotated(SKILLS_GOING_AROUND_LONG_GOAL_JERRY_X, SKILLS_GOING_AROUND_LONG_GOAL_JERRY_Y);
    auto going_across_long = sbot_from_jerry_rotated(SKILLS_GOING_ACROSS_LONG_GOAL_JERRY_X, SKILLS_GOING_ACROSS_LONG_GOAL_JERRY_Y);
    auto aligning_to_long = sbot_from_jerry_rotated(SKILLS_ALIGNING_TO_LONG_GOAL_JERRY_X, SKILLS_ALIGNING_TO_LONG_GOAL_JERRY_Y);
    auto long_goal_contact = sbot_from_jerry_rotated(SKILLS_LONG_GOAL_CONTACT_JERRY_X, SKILLS_LONG_GOAL_CONTACT_JERRY_Y);
    auto long_goal_retreat = sbot_from_jerry_rotated(SKILLS_LONG_GOAL_RETREAT_JERRY_X, SKILLS_LONG_GOAL_RETREAT_JERRY_Y);
    auto right_long_goal_retreat = sbot_from_jerry_rotated(SKILLS_RIGHT_LONG_GOAL_RETREAT_X, SKILLS_RIGHT_LONG_GOAL_RETREAT_Y);
    auto match_loader_contact_blue = sbot_from_jerry_rotated(SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_X, SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_Y);
    auto right_matchload_align = sbot_from_jerry_rotated(SKILLS_RIGHT_MATCHLOAD_ALIGN_X, SKILLS_RIGHT_MATCHLOAD_ALIGN_Y);
    auto park_red = sbot_from_jerry_rotated(SKILLS_PARK_RED_JERRY_X, SKILLS_PARK_RED_JERRY_Y);

    auto robot_reset = sbot_from_jerry_rotated(25, 47);


    
    //------Start of AUTO-------
    
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();
    printf("Intake set to COLLECT_FORWARD at start\n");
    sbot_print_jerry_pose_rotated("skills start");


    // 2. Match Load Approach
    sbot_drive_to(to_match_loader, 10000, false, true); // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to match loader");
    

    // Extend loader and start intake during turn
    if (sbot_batch_loader) sbot_batch_loader->extend();


    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING, 1000, {.maxSpeed = 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to matchloader heading");


    // 3. Match Loading (with Anti-Stall)
    //set robot to hold mode
    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);

    moveToPointWithAntiStall(match_loader_contact_red.x, match_loader_contact_red.y, SKILLS_MATCHLOADER_RED_HEADING, 2000, 40.0, true);
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After anti-stall move to match loader contact red");
    pros::delay(2250); // Wait 2.25s to collect balls

    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);


    // 4. Retreat & Realign
    //sbot_drive_to(match_loader_retreat, 10000, false, false);  // backwards
    //sbot_chassis->waitUntilDone();
    //sbot_print_jerry_pose_rotated("After retreat from match loader");
    

    
    //sbot_turn_to(SKILLS_GOING_AROUND_LONG_GOAL_HEADING, 1000);
    //sbot_chassis->waitUntilDone();
    //sbot_print_jerry_pose_rotated("After turn to go around long goal");

    sbot_chassis->moveToPoint(match_loader_retreat.x, match_loader_retreat.y, 10000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 2});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After going around long goal");

    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
    sbot_intake->update();
    sbot_indexer->update();

    if (sbot_batch_loader) sbot_batch_loader->retract();

        // 5. Cross Field
    sbot_chassis->turnToHeading(62.5, 1000, {.maxSpeed = 80, .minSpeed = 20});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to cross field");

    
    sbot_chassis->moveToPoint(going_around_long.x, going_around_long.y, 10000, {.forwards = true, .maxSpeed = 60});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After going around long goal");

    

    // 5. Cross Field
    sbot_chassis->turnToHeading(90, 1000, {.maxSpeed = 80, .minSpeed = 20});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to cross field");


    sbot_chassis->moveToPoint(going_across_long.x, going_across_long.y, 10000, {.forwards = true, .maxSpeed = 60});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After crossing field");
    
    // 6. Align for Goal
    //sbot_chassis->turnToHeading(SKILLS_ALIGNING_TO_LONG_GOAL_HEADING, 10000, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 2});
    //sbot_chassis->waitUntilDone();
    //sbot_print_jerry_pose_rotated("After turn to align for goal");

    
    sbot_chassis->turnToHeading(SKILLS_ALIGNING_TO_LONG_GOAL_HEADING, 1000, {.maxSpeed = 70});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to align for goal");

    sbot_chassis->moveToPoint(aligning_to_long.x,aligning_to_long.y, 10000, {.forwards = true, .maxSpeed = 60});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to align for goal");


    sbot_chassis->turnToHeading(SKILLS_LONG_GOAL_CONTACT_HEADING, 1000, {.maxSpeed = 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to matchloader heading");


    if (sbot_batch_loader) sbot_batch_loader->extend();


    
// 1. Start reversing the intake BEFORE the robot starts moving
    if (sbot_intake) sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_EJECT);
    sbot_intake->update();
    sbot_indexer->update();

    // 2. Create a flag to track if the chassis has finished its movement early
    bool movement_done = false;

    // 3. Launch a background task to act as our 300ms watchdog timer
    pros::Task watchdog_task([&movement_done]() {
        pros::delay(200); // Wait the maximum allowed reverse time

        // If the chassis is STILL moving after 300ms, turn the intake off safely
        if (!movement_done) {
            if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
            if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
            sbot_intake->update();
            sbot_indexer->update();
        }
    });

    // 4. Start the chassis movement towards the goal
    moveToPointWithAntiStall(long_goal_contact.x, long_goal_contact.y, SKILLS_LONG_GOAL_CONTACT_HEADING, 10000, 70, false);
    sbot_chassis->waitUntilDone(); 
    sbot_print_jerry_pose_rotated("After anti-stall move to long goal contact");

    // 5. The chassis has arrived! Update the flag so the background timer knows
    movement_done = true; 

    // 6. Switch intake to scoring mode (spins forward) instantly upon arrival
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();
    
    if (sbot_goal_flap) sbot_goal_flap->open();  // Lift scoring flap
    pros::delay(2500); // Wait 2.5s to score



    // Reload Step: Move forward to blue loader

    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);

    moveToPointWithAntiStall(match_loader_contact_blue.x, match_loader_contact_blue.y, SKILLS_MATCHLOADER_BLUE_HEADING, 2000, 40, true);
    sbot_chassis->waitUntilDone();
    if (sbot_goal_flap) sbot_goal_flap->close();  // Flap down BEFORE moving
    sbot_print_jerry_pose_rotated("After anti-stall move to match loader contact blue");
    pros::delay(2500); // Wait 2.0s to

    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    /*

    if (sbot_intake) sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_EJECT);
    sbot_intake->update();
    sbot_indexer->update();

    // 2. Create a flag to track if the chassis has finished its movement early
    movement_done = false;

    // 3. Launch a background task to act as our 300ms watchdog timer
    pros::Task watchdog_task_2([&movement_done]() {
        pros::delay(300); // Wait the maximum allowed reverse time

        // If the chassis is STILL moving after 300ms, turn the intake off safely
        if (!movement_done) {
            if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
            if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
            sbot_intake->update();
            sbot_indexer->update();
        }
    });
    */

    // 4. Start the chassis movement towards the goal
    moveToPointWithAntiStall(long_goal_contact.x, long_goal_contact.y, SKILLS_LONG_GOAL_CONTACT_HEADING, 10000, 60, false);
    
    // This will pause the main code right here until the robot physically arrives
    sbot_chassis->waitUntilDone(); 
    sbot_print_jerry_pose_rotated("After anti-stall move to long goal contact 2");

    // 5. The chassis has arrived! Update the flag so the background timer knows
    movement_done = true; 

    // 6. Switch intake to scoring mode (spins forward) instantly upon arrival
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();

    if (sbot_goal_flap) sbot_goal_flap->open();  // Lift scoring flap

    pros::delay(2500); // Wait 2.5s to score

    // 8. Final Push & Park
    // Go to retreat point
    sbot_chassis->moveToPoint(long_goal_retreat.x, long_goal_retreat.y, 10000, {.forwards = true, .maxSpeed = 70});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After final push forward 5 inches");
    
    if (sbot_batch_loader) sbot_batch_loader->retract();
    if (sbot_goal_flap) sbot_goal_flap->close();
    
    // Push: Move backward slowly to long goal
    sbot_chassis->moveToPoint(long_goal_contact.x, long_goal_contact.y, 2000, {.forwards = false, .maxSpeed = 20});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After anti-stall final move to long goal");
    

    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);

    pros::delay(250); //Time to settle the robot to reset position



    sbot_zero_pose_and_sensors(robot_reset.x, robot_reset.y, sbot_chassis->getPose().theta); // Reset pose at long goal contact for accurate parking

    pros::delay(250); // Time to settle the robot to reset position


    sbot_chassis->moveToPoint(long_goal_retreat.x, long_goal_retreat.y, 2000, {.forwards = true, .maxSpeed = 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After final push forward 5 inches");
    
    
    
    sbot_chassis->turnToHeading(180, 3000, {.maxSpeed = 60});
    sbot_chassis->waitUntilDone();



    sbot_chassis->moveToPoint(right_matchload_align.x, right_matchload_align.y, 10000, {.forwards = true, .maxSpeed = 70});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After move to right match loader align");

    //deploy matchloader
    if (sbot_batch_loader) sbot_batch_loader->extend();

    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING, 10000, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 2});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to match loader heading for parking");

    //------------------------------------------------------------------------
    //Start on the other side of the field, every point is the same, but mirrored across the Y axis and X axis, before the jerry conversion
    //------------------------------------------------------------------------
    auto right_match_loader_contact_red = sbot_from_jerry_rotated(SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_X, SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_Y*-1 - 1);
    auto right_match_loader_retreat = sbot_from_jerry_rotated(SKILLS_MATCH_LOADER_RETREAT_JERRY_X * -1, SKILLS_MATCH_LOADER_RETREAT_JERRY_Y * -1 - 1);
    auto right_going_around_long = sbot_from_jerry_rotated(SKILLS_GOING_AROUND_LONG_GOAL_JERRY_X * -1, SKILLS_GOING_AROUND_LONG_GOAL_JERRY_Y * -1 - 1);
    auto right_going_across_long = sbot_from_jerry_rotated(SKILLS_GOING_ACROSS_LONG_GOAL_JERRY_X * -1, SKILLS_GOING_ACROSS_LONG_GOAL_JERRY_Y * -1 - 1);
    auto right_aligning_to_long = sbot_from_jerry_rotated(SKILLS_ALIGNING_TO_LONG_GOAL_JERRY_X * -1, SKILLS_ALIGNING_TO_LONG_GOAL_JERRY_Y * -1 - 1);
    auto right_long_goal_contact = sbot_from_jerry_rotated(SKILLS_LONG_GOAL_CONTACT_JERRY_X * -1, SKILLS_LONG_GOAL_CONTACT_JERRY_Y * -1);
    auto right_match_loader_contact_blue = sbot_from_jerry_rotated(SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_X * -1, SKILLS_MATCH_LOADER_CONTACT_BLUE_JERRY_Y * -1);


    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);

    sbot_drive_to(right_match_loader_contact_red, 2000, false, true, 40.0);
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to right match loader contact red");

    pros::delay(2500);

    /*

    // 2. Wiggle 1: Right 3 degrees
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 3. Wiggle 2: Left 3 degrees from center (This creates the 6-degree leftward swing!)
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING - 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 4. Wiggle 3: Right 3 degrees from center (6-degree rightward swing)
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 5. Wiggle 4: Left 3 degrees from center
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING - 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 6. Wiggle 5: Right 3 degrees from center
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 7. Final move: Snap perfectly back to the exact center heading we started with
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_BLUE_HEADING, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    pros::delay(500); // Wait 0.5s to collect balls

    */


    

    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);

    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
   
    sbot_intake->update();
    sbot_indexer->update();

    sbot_chassis->moveToPoint(right_match_loader_retreat.x, right_match_loader_retreat.y, 10000, {.forwards = false, .maxSpeed = 80});  // 1/4 speed
    sbot_chassis->waitUntilDone();

    sbot_chassis->turnToHeading(62.5 + 180, 1000, {.maxSpeed = 80, .minSpeed = 20});

    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
    sbot_intake->update();
    sbot_indexer->update();

    if (sbot_batch_loader) sbot_batch_loader->retract();


    sbot_chassis->moveToPoint(right_going_around_long.x, right_going_around_long.y, 10000, {.forwards = true, .maxSpeed = 60});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After going around long goal");

    

    // 5. Cross Field
    sbot_chassis->turnToHeading(90+180, 1000, {.maxSpeed = 80, .minSpeed = 20});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to cross field");


    sbot_chassis->moveToPoint(right_going_across_long.x, right_going_across_long.y, 10000, {.forwards = true, .maxSpeed = 60});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After crossing field");



    sbot_chassis->turnToHeading(SKILLS_ALIGNING_TO_LONG_GOAL_HEADING+180, 1000, {.maxSpeed = 70});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to align for goal");

    sbot_chassis->moveToPoint(right_aligning_to_long.x,right_aligning_to_long.y, 10000, {.forwards = true, .maxSpeed = 60});  // 1/4 speed
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to align for goal");


    sbot_chassis->turnToHeading(SKILLS_LONG_GOAL_CONTACT_HEADING+180, 1000, {.maxSpeed = 60});
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After turn to matchloader heading");






    if (sbot_intake) sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_EJECT);
    sbot_intake->update();
    sbot_indexer->update();

    
    // 2. Create a flag to track if the chassis has finished its movement early
    movement_done = false;

    // 3. Launch a background task to act as our 500ms watchdog timer
    pros::Task watchdog_task_3([&movement_done]() {
        pros::delay(200); // Wait the maximum allowed reverse time

        // If the chassis is STILL moving after 500ms, turn the intake off safely
        if (!movement_done) {
            if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
            if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
            sbot_intake->update();
            sbot_indexer->update();
        }
    });

    // 4. Start the chassis movement towards the goal

    moveToPointWithAntiStall(right_long_goal_contact.x, right_long_goal_contact.y, SKILLS_LONG_GOAL_CONTACT_HEADING + 180, 1000, 40, false); 
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to right long goal contact");

    // 5. The chassis has arrived! Update the flag so the background timer knows
    movement_done = true; 

    // 6. Switch intake to scoring mode (spins forward) instantly upon arrival
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();


    if (sbot_goal_flap) sbot_goal_flap->open();  // Lift scoring flap
    if (sbot_batch_loader) sbot_batch_loader->extend();

    pros::delay(2500); // Wait 2.5s to score

    //LAST MATCHLOADER
    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::hold);

    sbot_drive_to(right_match_loader_contact_blue, 2000, false, true, 40.0);
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to right match loader contact blue");

    if (sbot_goal_flap) sbot_goal_flap->close();

    pros::delay(2500);

    /*

    // 2. Wiggle 1: Right 3 degrees
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 3. Wiggle 2: Left 3 degrees from center (This creates the 6-degree leftward swing!)
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING - 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 2. Wiggle 1: Right 3 degrees
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 3. Wiggle 2: Left 3 degrees from center (This creates the 6-degree leftward swing!)
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING - 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 4. Wiggle 3: Right 3 degrees from center (6-degree rightward swing)
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 5. Wiggle 4: Left 3 degrees from center
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING - 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 6. Wiggle 5: Right 3 degrees from center
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING + 3, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    // 7. Final move: Snap perfectly back to the exact center heading we started with
    sbot_chassis->turnToHeading(SKILLS_MATCHLOADER_RED_HEADING, 500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1});
    sbot_chassis->waitUntilDone();

    pros::delay(500); // Wait 0.5s to collect balls

    */



    

    sbot_left_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    sbot_right_motors->set_brake_mode_all(pros::v5::MotorBrake::coast);

    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
   
    sbot_intake->update();
    sbot_indexer->update();


    if (sbot_intake) sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_EJECT);
    sbot_intake->update();
    sbot_indexer->update();


    // 2. Create a flag to track if the chassis has finished its movement early
    movement_done = false;

    // 3. Launch a background task to act as our 300ms watchdog timer
    pros::Task watchdog_task_4([&movement_done]() {
        pros::delay(300); // Wait the maximum allowed reverse time
        
        // If the chassis is STILL moving after 300ms, turn the intake off safely
        if (!movement_done) {
            if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
            if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
            sbot_intake->update();
            sbot_indexer->update();
        }
    });

    // 4. Start the chassis movement towards the goal

    moveToPointWithAntiStall(right_long_goal_contact.x, right_long_goal_contact.y, SKILLS_LONG_GOAL_CONTACT_HEADING + 180, 10000, 40, false); 
    sbot_chassis->waitUntilDone();
    sbot_print_jerry_pose_rotated("After drive to right long goal contact");
    // 5. The chassis has arrived! Update the flag so the background timer knows
    movement_done = true; 

    // 6. Switch intake to scoring mode (spins forward) instantly upon arrival
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_intake->update();
    sbot_indexer->update();

    if (sbot_batch_loader) sbot_batch_loader->retract();


    if (sbot_goal_flap) sbot_goal_flap->open();  // Lift scoring flap
    pros::delay(2000); // Wait 2.0s to score


    
    sbot_zero_pose_and_sensors( right_long_goal_contact.x, right_long_goal_contact.y, sbot_chassis->getPose().theta); // Reset pose at long goal contact for accurate parking



    sbot_chassis->moveToPose(park_red.x, park_red.y, 0, 10000, {.forwards = true, .maxSpeed = 120, .minSpeed = 100});
    sbot_chassis->waitUntilDone();

    sbot_safe_stop_mechanisms();
    
    printf("SBOT AUTON: SKILLS complete\n");
}