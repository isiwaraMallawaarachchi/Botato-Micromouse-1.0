#include "Phase2Test.hpp"
#include "Robot.hpp"

// TEST 1 — commanded 90 deg pivot, then hold.
void P2_PivotTurn_Init() {
    robot.control().resetControllers();   // zeroes gyro angle
    robot.control().enable(true);
    robot.control().holdHeading(45.0f);   // turn to +90 and hold
}

// TEST 2 — drive straight one cell (~180mm) holding heading 0.
void P2_OneCellDrive_Init() {
    robot.control().resetControllers();
    robot.control().enable(true);
    robot.control().driveStraight(500.0f, 0.0f);  // 150 mm/s, heading 0
}
