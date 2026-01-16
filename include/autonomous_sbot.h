/**
 * autonomous_sbot.h - Autonomous selector and routines for sbot.
 */

#ifndef _SBOT_AUTONOMOUS_SBOT_H_
#define _SBOT_AUTONOMOUS_SBOT_H_

#include "api.h"
#include "config_sbot.h"

// NOTE: LemLib will be configured in a separate file; here we just
// declare usage and provide a selector with the requested modes.

enum class SbotAutoMode {
    DISABLED = 0,
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT,
    RED_LEFT_SOLO_AWP,
    RED_RIGHT_SOLO_AWP,
    BLUE_LEFT_SOLO_AWP,
    BLUE_RIGHT_SOLO_AWP,
    SKILLS,
    TEST_SWEEP_TO_LOW_GOAL,
    TEST_DRIVE,
    TEST_TURN,
    TEST_INTAKE,
    TEST_INDEXER,
    TEST_DRIVE_SHORT,
    TEST_LOW_GOAL_CUSTOM_START,
    TEST_JERRY_POSE_MONITOR,
        TEST_FOLLOW_JERRY_PATH,
        TEST_POSE_FINDER_X0_LINE_90
};

class SbotAutoSelector {
public:
    SbotAutoSelector();

    bool update();              // handle input and refresh display; true when confirmed
    SbotAutoMode getMode() const { return selected_mode; }
    bool isConfirmed() const { return mode_confirmed; }

private:
    SbotAutoMode selected_mode;
    int selector_position;
    bool mode_confirmed;

    void displayOptions();
    void handleInput();
};

class SbotAutonomousSystem {
public:
    SbotAutonomousSystem();

    void initialize();
    void updateSelector();
    void run(); // call from autonomous()

    SbotAutoSelector& getSelector() { return selector; }

private:
    SbotAutoSelector selector;

    // Simple stubs for now – can be filled with LemLib paths later
    void runRedLeft();
    void runRedRight();
    void runBlueLeft();
    void runBlueRight();
    void runSkills();

    void runTestSweepToLowGoal();

    void runTestDrive();
    void runTestDriveShort();
    void runTestLowGoalCustomStart();
    void runTestTurn();
    void runTestIntake();
    void runTestIndexer();
    void runTestJerryPoseMonitor();
    void runTestFollowJerryPath();
    void runTestPoseFinderX0Line90();
};

#endif // _SBOT_AUTONOMOUS_SBOT_H_
