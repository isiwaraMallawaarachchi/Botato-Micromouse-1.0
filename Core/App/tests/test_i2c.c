#include "test_i2c.h"
#include "test_tof.h"
#include "i2c.h"

volatile uint8_t i2c1_device_count   = 0;
volatile uint8_t i2c1_found_addr[16] = {0};
volatile uint8_t i2c2_device_count   = 0;
volatile uint8_t i2c2_found_addr[16] = {0};

/* Shared bit-bang recovery. Caller supplies the pins and the re-init
 * function for whichever bus is being cleared.                        */
static void BusRecover(uint16_t scl_pin, uint16_t sda_pin,
                        I2C_HandleTypeDef *handle, void (*reinit)(void))
{
    HAL_I2C_DeInit(handle);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = scl_pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = sda_pin;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, sda_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    for (int i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, sda_pin) == GPIO_PIN_SET)
        {
            break;   /* SDA released — bus is free */
        }
        HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Manual STOP: SDA low-to-high while SCL is high. */
    HAL_GPIO_WritePin(GPIOB, sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, sda_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    reinit();
}

void I2C1_BusRecover(void)
{
    BusRecover(GPIO_PIN_6, GPIO_PIN_7, &hi2c1, MX_I2C1_Init);
}

void I2C2_BusRecover(void)
{
    BusRecover(GPIO_PIN_10, GPIO_PIN_9, &hi2c2, MX_I2C2_Init);
}

static void ScanBus(I2C_HandleTypeDef *hi2c,
                     volatile uint8_t *count, volatile uint8_t *found)
{
    *count = 0;
    for (int i = 0; i < 16; i++)
    {
        found[i] = 0;
    }

    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 5) == HAL_OK)
        {
            if (*count < 16)
            {
                found[*count] = addr;
            }
            (*count)++;
        }
    }
}

void Test_I2C_Scan(void)
{
    /* Reuse the ToF module's own bring-up: XSHUT sequencing plus unique
     * address assignment. Without this, all five sensors sit at the
     * shared factory default 0x29 and bus 1 shows only one device.
     * Test_ToF_Init() calls I2C1_BusRecover() internally.              */
    Test_ToF_Init();

    I2C2_BusRecover();

    ScanBus(&hi2c1, &i2c1_device_count, i2c1_found_addr);
    ScanBus(&hi2c2, &i2c2_device_count, i2c2_found_addr);
}
