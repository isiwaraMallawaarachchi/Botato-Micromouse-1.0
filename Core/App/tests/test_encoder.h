#ifndef TEST_ENCODER_H
#define TEST_ENCODER_H

#include "main.h"
#include <stdint.h>

/**
 * Encoders — hardware quadrature readout, no motors involved.
 * Spin each wheel BY HAND and watch the counts change.
 *
 * TIM5 (PA0/PA1) = Left, TIM2 (PA15/PB3) = Right. Both 32-bit, so
 * counts never need overflow handling. Reading costs one register
 * access — no ISR, no polling overhead.
 */

extern volatile int32_t enc_left_count;    /* 0 at startup, +/- as you turn */
extern volatile int32_t enc_right_count;
extern volatile int32_t enc_left_delta;    /* ticks in the last 200ms       */
extern volatile int32_t enc_right_delta;

/** Starts both encoder timers. Call once from USER CODE BEGIN 2. */
void Test_Encoder_Init(void);

/** Refreshes counts; recomputes deltas every 200ms. Call from main loop. */
void Test_Encoder_Update(void);

#endif /* TEST_ENCODER_H */
