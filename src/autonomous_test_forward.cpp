#include "autonomous_test_forward.h"
#include "autonomous_infrastructure.h"
#include "lemlib_config_sbot.h"
#include <cmath>
#include <cstdio>

void sbot_run_test_forward_auto() {
    if (!validateSbotLemLibInitialization() || !sbot_chassis) return;
    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 30;
    sbot_chassis->moveToPoint(0, 1, 1000, params); // Move forward 1 inch
    sbot_chassis->waitUntilDone();
}