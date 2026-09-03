#ifndef TEST_IMU_H
#define TEST_IMU_H

#include "main.h"
#include <stdint.h>

/**
 * MPU6050 IMU — presence check, wake, calibration, and Z-axis yaw.
 *
 * KNOWN ISSUE: as of last testing this sensor intermittently fails to
 * ACK on the bus (imu_ready stays 0). Hardware fault suspected, not
 * yet root-caused. Everything below is correct and previously worked.
 *
 * Keep the robot STILL during the ~1s calibration after reset.
 */

typedef enum {
    IMU_NOT_RUN = 0,
    IMU_PASS,
    IMU_FAIL
} IMUTestResult;

extern volatile IMUTestResult imu_result;
extern volatile uint8_t       imu_ready;        /* 1 = ACKed on the bus     */
extern volatile uint8_t       imu_who_am_i;     /* expect 0x68 (104)        */
extern volatile uint8_t       imu_fail_step;    /* see table below          */
extern volatile uint32_t      imu_tick_count;   /* proves TIM3 ISR fires    */
extern volatile uint32_t      imu_error_count;  /* failed reads in the loop */

extern volatile int16_t imu_gyro_z_raw;
extern volatile float   imu_gyro_z_rate;  /* deg/s, offset-corrected  */
extern volatile float   imu_yaw;           /* integrated heading, deg  */

/* imu_fail_step (only meaningful when imu_result == IMU_FAIL):
 *   1 = no ACK at 0x68        4 = wake-up write failed
 *   2 = WHO_AM_I read failed  5 = gyro range write failed
 *   3 = WHO_AM_I wrong value  6 = DLPF write failed                  */

/** Init + calibrate, then start TIM3 at 1kHz. TIM3 starts regardless of
 *  whether the IMU succeeded — it is the whole robot's control clock. */
void Test_IMU_Init(void);

/** Reads the gyro and integrates yaw. Call from the main loop. */
void Test_IMU_Process(void);

#endif /* TEST_IMU_H */
