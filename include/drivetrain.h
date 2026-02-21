/**
 * drivetrain.h - 6-motor tank drive for sbot.
 */

#ifndef _SBOT_DRIVETRAIN_H_
#define _SBOT_DRIVETRAIN_H_

#include "api.h"
#include "config_sbot.h"

class SbotDrivetrain {
public:
    SbotDrivetrain();

    void arcadeTankControl(pros::Controller& master);
    void splitArcadeControl(pros::Controller& master);
    void tankControl(pros::Controller& master, bool run_slow);
    void setBrakeMode(pros::v5::MotorBrake mode);
    void stop();
    

private:
    pros::Motor left_front;
    pros::Motor left_middle;
    pros::Motor left_back;
    pros::Motor right_front;
    pros::Motor right_middle;
    pros::Motor right_back;

    // Previous motor commands for slew rate limiting
    int prev_left_cmd = 0;
    int prev_right_cmd = 0;

    int applyDeadzone(int value) const;
    int applyCurve(int value) const;
    int applySlewRate(int current, int target, int max_change) const;
};

#endif // _SBOT_DRIVETRAIN_H_
