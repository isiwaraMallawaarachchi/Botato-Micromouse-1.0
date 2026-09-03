#include "test_tof.h"
#include "test_i2c.h"
#include "tof.h"
#include "i2c.h"

volatile ToFTestResult tof_result[5]      = { TOF_NOT_RUN, TOF_NOT_RUN,
                                               TOF_NOT_RUN, TOF_NOT_RUN,
                                               TOF_NOT_RUN };
volatile uint8_t       tof_addr[5]         = {0};
volatile uint16_t      tof_distance_mm[5]  = {0};
volatile uint32_t      tof_error_count[5]  = {0};

static VL53L0X_Dev_t tof_dev[5];

#define VL53_DEFAULT_ADDR   (0x29 << 1)
#define VL53_ADDR_REG       0x8A
#define XSHUT_BOOT_DELAY_MS 10

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} XShutPin;

static const XShutPin xshut[5] = {
    { XSHUT_LEFT_GPIO_Port,  XSHUT_LEFT_Pin  },   /* 0 Left       */
    { XSHUT_LF_GPIO_Port,    XSHUT_LF_Pin    },   /* 1 LeftFront  */
    { XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin },   /* 2 Front      */
    { XSHUT_RF_GPIO_Port,    XSHUT_RF_Pin    },   /* 3 RightFront */
    { XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin },   /* 4 Right (PB8) */
};

static const uint8_t target_addr[5] = { 0x30, 0x31, 0x32, 0x33, 0x34 };

void Test_ToF_DisableAll(void)
{
    for (int i = 0; i < 5; i++)
    {
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
    }
    HAL_Delay(XSHUT_BOOT_DELAY_MS);
}

void Test_ToF_Init(void)
{
    I2C1_BusRecover();
    Test_ToF_DisableAll();

    /* Bring each sensor up one at a time and give it a unique address.
     * Unlike a simple presence check, each sensor STAYS enabled after
     * being addressed — so by the end all 5 are live simultaneously.  */
    for (int i = 0; i < 5; i++)
    {
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_SET);
        HAL_Delay(XSHUT_BOOT_DELAY_MS);

        /* Responds at the shared factory default? */
        if (HAL_I2C_IsDeviceReady(&hi2c1, VL53_DEFAULT_ADDR, 2, 100) != HAL_OK)
        {
            tof_result[i] = TOF_FAIL;
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }

        /* Move it to its own address. */
        uint8_t addr = target_addr[i];
        if (HAL_I2C_Mem_Write(&hi2c1, VL53_DEFAULT_ADDR, VL53_ADDR_REG,
                               1, &addr, 1, 100) != HAL_OK)
        {
            tof_result[i] = TOF_FAIL;
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }
        HAL_Delay(2);

        /* Answers at the new address? */
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 100) != HAL_OK)
        {
            tof_result[i] = TOF_FAIL;
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }

        /* Full driver bring-up: SPAD config, tuning, ref calibration. */
        if (VL53L0X_Init(&tof_dev[i], &hi2c1, (uint8_t)(addr << 1)) != HAL_OK)
        {
            tof_result[i] = TOF_FAIL;
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }

        tof_result[i] = TOF_PASS;
        tof_addr[i]   = addr;
    }

    /* Coexistence check — re-verify each passing sensor now that every
     * other sensor is also live.                                      */
    for (int i = 0; i < 5; i++)
    {
        if (tof_result[i] == TOF_PASS &&
            HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(tof_addr[i] << 1), 2, 100) != HAL_OK)
        {
            tof_result[i] = TOF_FAIL;
        }
    }
}

void Test_ToF_ReadDistances(void)
{
    for (int i = 0; i < 5; i++)
    {
        if (tof_result[i] != TOF_PASS)
        {
            continue;
        }

        uint16_t mm = 0;
        if (VL53L0X_ReadRangeSingleMillimeters(&tof_dev[i], &mm) == HAL_OK)
        {
            tof_distance_mm[i] = mm;
        }
        else
        {
            tof_error_count[i]++;
        }
    }
}
