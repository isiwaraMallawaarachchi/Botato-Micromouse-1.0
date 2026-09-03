#ifndef TEST_MOTOR_H
#define TEST_MOTOR_H

#include "main.h"
#include <stdint.h>

/**
 * Motor driver — TB6612FNG PWM + direction, open loop (no encoders).
 * Both motors are driven SIMULTANEOUSLY with the identical duty value.
 *
 * SAFETY: secure the robot with its wheels off the ground before
 * running this — both motors WILL spin.
 *
 * Direction polarity is INVERTED vs the TB6612 datasheet truth table
 * to match how the motor leads are physically wired on this board.
 * See DECISIONS.md #15 — do not "correct" this back.
 */

typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_DONE
} MotorTestPhase;

extern volatile MotorTestPhase motor_phase;

/** Ramps both motors up/hold/down forward, then the same in reverse.
 *  Blocking — takes a few seconds. Call once from USER CODE BEGIN 2.  */
void Test_Motor_Run(void);

#endif /* TEST_MOTOR_H */
