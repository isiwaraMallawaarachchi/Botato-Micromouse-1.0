#ifndef APP_TESTS_HPP
#define APP_TESTS_HPP

/*
 * Phase 1 test harnesses — one function per subsystem, same logic and timing
 * as the earlier C tests. Select ONE by the #define in main.cpp. Each Test_*
 * pair is an init (call once, USER CODE 2) + update (call each loop / tick).
 *
 * Watch the exposed variables in Live Expressions via the global `robot`.
 *
 *   TEST_ENCODER : turn wheels by hand -> robot.encoderL()/R() distance+speed
 *   TEST_GYRO    : gyro.angleDeg() stable; responds to rotation
 *   TEST_TOF     : walls.distanceMm(i) filtered+calibrated
 *   TEST_MOTOR   : both motors ramp fwd then rev
 *   TEST_DRIVE   : hold heading (twist by hand -> returns to 0)
 *   TEST_MODES   : press buttons -> robot.modes().state() changes
 *
 * These are thin: they mostly just let robot.onControlTick()/onMainLoop() run
 * and, for the motor/drive tests, command targets. The heavy lifting is in the
 * classes, exactly as it will be in the real firmware.
 */

void Test_Motor_Init();
void Test_Motor_Update();     // ramps both motors fwd then reverse, blocking

void Test_Drive_Init();       // hold heading at 0
// (drive/gyro/encoder/tof/modes need no special update — onMainLoop +
//  onControlTick already drive them; just watch the accessors.)

#endif // APP_TESTS_HPP
