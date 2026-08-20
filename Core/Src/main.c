/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define MPU6050_ADDR (0x68 << 1) // 0xD0 when AD0 is wired to GND
#define PWR_MGMT_1_REG 0x6B
#define CONFIG_REG 0x1A          // DLPF Configuration Register
#define GYRO_CONFIG_REG 0x1B
#define GYRO_ZOUT_H_REG 0x47

// MUST be volatile so the debugger can read them while the ISR updates them
volatile int16_t Gyro_Z_RAW;
volatile float Gyro_Z_Rate;
volatile float yaw = 0.0;
volatile float Gyro_Z_Offset = 0.0;
volatile uint32_t i2c_error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU6050_Init(void) {
    uint8_t Data = 0;

    // 1. Wake up the sensor
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 100);

    // 2. Configure Gyroscope to ±500 °/s (FS_SEL=1)
    Data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 100);

    // 3. Configure Digital Low Pass Filter (DLPF) for the 1 kHz loop (~42Hz filter)
    Data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 100);
}

void MPU6050_Calibrate(void) {
    int32_t gyro_sum = 0;
    uint8_t Rec_Data[2];

    // Sample the gyro 1000 times to calculate the zero-drift offset.
    for (int i = 0; i < 1000; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 10);
        gyro_sum += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        HAL_Delay(1);
    }
    Gyro_Z_Offset = (float)gyro_sum / 1000.0 / 65.5;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  // Initialize and calibrate the IMU
  MPU6050_Init();
  MPU6050_Calibrate();

  // Start the 1 kHz Master Control Loop timer interrupt
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        uint8_t Rec_Data[2];

        // Synchronous I2C read inside the 1 kHz ISR
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 2) == HAL_OK) {

            // Reconstruct the 16-bit raw data
            Gyro_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

            // Apply scale factor and subtract the calibration offset
            Gyro_Z_Rate = (Gyro_Z_RAW / 65.5) - Gyro_Z_Offset;

            // Integrate angular velocity to calculate the absolute heading (dt = 0.001s)
            yaw += Gyro_Z_Rate * 0.001;
        } else {
            // Track missed/failed I2C transactions
            i2c_error_count++;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
