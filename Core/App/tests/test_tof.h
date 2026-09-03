#ifndef TEST_TOF_H
#define TEST_TOF_H

#include "main.h"
#include <stdint.h>

/**
 * ToF sensors — all 5 VL53L0X units on I2C1 (PB6/PB7).
 * Covers everything: XSHUT sequencing, unique address assignment,
 * simultaneous coexistence on the bus, and live distance reads.
 *
 * Index order (matches PINOUT.md XSHUT boot order):
 *   0 = Left        -> 0x30
 *   1 = LeftFront   -> 0x31
 *   2 = Front       -> 0x32
 *   3 = RightFront  -> 0x33
 *   4 = Right       -> 0x34
 */

typedef enum {
    TOF_NOT_RUN = 0,
    TOF_PASS,
    TOF_FAIL
} ToFTestResult;

extern volatile ToFTestResult tof_result[5];
extern volatile uint8_t       tof_addr[5];          /* 7-bit, expect 0x30..0x34 */
extern volatile uint16_t      tof_distance_mm[5];   /* live range, mm           */
extern volatile uint32_t      tof_error_count[5];   /* failed reads, expect 0   */

/** Drives all 5 XSHUT pins LOW, disabling every sensor. */
void Test_ToF_DisableAll(void);

/** Full bring-up on I2C1: XSHUT sequence, address assignment, driver
 *  init, and a final coexistence check with all sensors live.        */
void Test_ToF_Init(void);

/** One blocking ranging read per passing sensor. Call from main loop. */
void Test_ToF_ReadDistances(void);

#endif /* TEST_TOF_H */
