#include "test_imu.h"
#include "test_i2c.h"
#include "i2c.h"
#include "tim.h"

#define MPU_ADDR         (0x68 << 1)   /* AD0 tied to GND */
#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_CONFIG       0x1A          /* DLPF */
#define REG_GYRO_CONFIG  0x1B
#define REG_GYRO_ZOUT_H  0x47

/* FS_SEL=1 -> +/-500 dps, 65.5 LSB per deg/s. A fast in-place turn can
 * exceed 500 dps and saturate. If yaw flattens during quick turns, set
 * REG_GYRO_CONFIG to 0x10 (+/-1000 dps) and change this to 32.8f.     */
#define GYRO_SENSITIVITY 65.5f

volatile IMUTestResult imu_result      = IMU_NOT_RUN;
volatile uint8_t       imu_ready        = 0;
volatile uint8_t       imu_who_am_i     = 0;
volatile uint8_t       imu_fail_step    = 0;
volatile uint32_t      imu_tick_count   = 0;
volatile uint32_t      imu_error_count  = 0;

volatile int16_t imu_gyro_z_raw  = 0;
volatile float   imu_gyro_z_rate = 0.0f;
volatile float   imu_yaw         = 0.0f;

static float            gyro_z_offset = 0.0f;
static volatile uint8_t tick_flag     = 0;
static float            gz_prev       = 0.0f;

static HAL_StatusTypeDef MPU_Init(void)
{
    uint8_t d;

    if (HAL_I2C_IsDeviceReady(&hi2c2, MPU_ADDR, 3, 100) != HAL_OK)
    {
        imu_ready = 0; imu_fail_step = 1; return HAL_ERROR;
    }
    imu_ready = 1;

    if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, REG_WHO_AM_I, 1,
                          (uint8_t *)&imu_who_am_i, 1, 100) != HAL_OK)
    {
        imu_fail_step = 2; return HAL_ERROR;
    }
    if (imu_who_am_i != 0x68) { imu_fail_step = 3; return HAL_ERROR; }

    d = 0x00;   /* wake: clear SLEEP bit */
    if (HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, REG_PWR_MGMT_1, 1, &d, 1, 100) != HAL_OK)
    {
        imu_fail_step = 4; return HAL_ERROR;
    }

    d = 0x08;   /* gyro +/-500 dps */
    if (HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, REG_GYRO_CONFIG, 1, &d, 1, 100) != HAL_OK)
    {
        imu_fail_step = 5; return HAL_ERROR;
    }

    d = 0x03;   /* DLPF ~44Hz, suits a 1kHz sample loop */
    if (HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, REG_CONFIG, 1, &d, 1, 100) != HAL_OK)
    {
        imu_fail_step = 6; return HAL_ERROR;
    }

    return HAL_OK;
}

static void MPU_Calibrate(void)
{
    int32_t  sum     = 0;
    uint16_t samples = 0;
    uint8_t  buf[2];

    for (int i = 0; i < 1000; i++)
    {
        if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, REG_GYRO_ZOUT_H, 1, buf, 2, 10) == HAL_OK)
        {
            sum += (int16_t)(buf[0] << 8 | buf[1]);
            samples++;
        }
        HAL_Delay(1);
    }

    /* Divide by reads that ACTUALLY succeeded — a fixed /1000 would
     * skew the offset if the bus hiccupped during calibration.        */
    if (samples > 0)
    {
        gyro_z_offset = ((float)sum / (float)samples) / GYRO_SENSITIVITY;
    }
}

void Test_IMU_Init(void)
{
    /* The IMU now sits alone on I2C2 (PB10/PB9), separate from the five
     * ToF sensors on I2C1. The ToF EXTI masking that used to wrap this
     * function is gone — ToF activity can no longer disturb this bus,
     * and their combined pull-ups no longer load it either.           */
    I2C2_BusRecover();

    HAL_StatusTypeDef st = MPU_Init();
    imu_result = (st == HAL_OK) ? IMU_PASS : IMU_FAIL;

    if (st == HAL_OK)
    {
        MPU_Calibrate();
    }

    /* TIM3 drives the whole robot's control loop — never gate it on
     * one sensor's success, or a dead IMU stops everything.           */
    HAL_TIM_Base_Start_IT(&htim3);
}

void Test_IMU_Process(void)
{
    if (!tick_flag) { return; }
    tick_flag = 0;

    if (imu_result != IMU_PASS) { return; }

    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, REG_GYRO_ZOUT_H, 1, buf, 2, 2) == HAL_OK)
    {
        imu_gyro_z_raw  = (int16_t)(buf[0] << 8 | buf[1]);
        imu_gyro_z_rate = ((float)imu_gyro_z_raw / GYRO_SENSITIVITY) - gyro_z_offset;

        /* Trapezoidal integration. dt = 0.001s fixed — TIM3's 1kHz is
         * a known hardware rate, no need to measure elapsed time.      */
        imu_yaw += ((imu_gyro_z_rate + gz_prev) / 2.0f) * 0.001f;
        if (imu_yaw > 180.0f)       imu_yaw -= 360.0f;
        else if (imu_yaw <= -180.0f) imu_yaw += 360.0f;

        gz_prev = imu_gyro_z_rate;
    }
    else
    {
        imu_error_count++;
    }
}

/* ---- TIM3 callback -----------------------------------------------------
 * Flag only, nothing else — all I2C work happens in Test_IMU_Process().
 *
 * NOTE: this weak HAL callback may be defined only ONCE project-wide.
 * When real motor/encoder drivers need periodic work, add it HERE
 * (still flag-only) rather than redefining this elsewhere, or the
 * linker will reject it as a duplicate symbol.                        */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        imu_tick_count++;
        tick_flag = 1;
    }
}
