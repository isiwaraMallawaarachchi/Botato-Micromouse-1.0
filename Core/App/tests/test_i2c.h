#ifndef TEST_I2C_H
#define TEST_I2C_H

#include "main.h"
#include <stdint.h>

/**
 * I2C bus health — recovery and device scan.
 *
 * This board runs TWO separate I2C buses:
 *   I2C1 (PB6/PB7)  — five ToF sensors
 *   I2C2 (PB10/PB9) — MPU6050 only
 *
 * They were split because five ToF breakout boards each carry a 10k
 * pull-up; in parallel they dragged the shared bus down to ~1.5k,
 * below what the MPU6050 could reliably pull low. See DECISIONS.md.
 */

/* Scan results. Bus 1 expects 0x30..0x34 (five ToF, after addressing).
 * Bus 2 expects 0x68 (MPU6050).                                       */
extern volatile uint8_t i2c1_device_count;
extern volatile uint8_t i2c1_found_addr[16];
extern volatile uint8_t i2c2_device_count;
extern volatile uint8_t i2c2_found_addr[16];

/**
 * Clears a stuck bus. If a debug session was interrupted mid-transaction,
 * a device can hold SDA low and wedge the bus for every address — a
 * reflash alone won't clear it. Bit-bangs up to 9 SCL pulses to force
 * the device to release SDA, issues a STOP, then re-inits the peripheral.
 * Safe to call even when the bus is healthy.
 */
void I2C1_BusRecover(void);   /* PB6 = SCL, PB7 = SDA  */
void I2C2_BusRecover(void);   /* PB10 = SCL, PB9 = SDA */

/** Scans both buses. Brings up + addresses the ToF sensors first so
 *  they hold unique addresses rather than all sharing 0x29.           */
void Test_I2C_Scan(void);

#endif /* TEST_I2C_H */
