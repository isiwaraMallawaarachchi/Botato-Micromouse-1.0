/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body  (TESTING build)
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

/* ===== PICK ONE TEST — comment out the rest ===== *
 * Phase 1 (watch robot.<member> in Live Expressions):
 *   ENCODER : robot.encL_.distanceMm_ / encR_.distanceMm_
 *   GYRO    : robot.gyro_.angleDeg_ (+90 must be a physical RIGHT turn)
 *   TOF     : robot.walls_.distMm_[0..4]
 *   MOTOR   : wheels ramp fwd/rev (Test_Motor_*)
 *   DRIVE   : hold heading, twist by hand -> returns
 *   MODES   : press buttons -> robot.modes_.state_
 * Phase 2:
 *   P2_PIVOT : commanded 90 deg turn + hold  (wheels on ground)
 *   P2_DRIVE : drive straight ~1 cell holding heading (clear space ahead)
 * Bus tests (watch the i2cGyro global, NOT robot.*):
 *   RUN_TEST_I2C_GYRO : I2C2 + MPU6050. STANDALONE — skips robot.init()
 *                       entirely so the TIM3 ISR never touches I2C2 while
 *                       the test owns the bus.
 * Don't enable two motion tests at once. */
//#define RUN_TEST_ENCODER
//#define RUN_TEST_GYRO
#define RUN_TEST_TOF
//#define RUN_TEST_MOTOR
//#define RUN_TEST_DRIVE
//#define RUN_TEST_MODES
//#define P2_TEST_PIVOT
//#define P2_TEST_DRIVE
//#define RUN_TEST_I2C_GYRO

#include "Robot.hpp"
#include "Tests.hpp"
#include "Phase2Test.hpp"
#include "Tests_I2C.hpp"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* USER CODE BEGIN PV */
/* USER CODE END PV */

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
#ifdef RUN_TEST_I2C_GYRO
  /* Bus test owns I2C2 exclusively. robot.init() is deliberately NOT called:
   * it blocks ~5s in gyro_.calibrate() and then starts the 1kHz TIM3 ISR,
   * which reads the MPU6050 four times per millisecond. Those reads would
   * interleave with the test's transactions and invalidate every result. */
  Test_I2C_Gyro_Init();
#else
  robot.init();

#ifdef RUN_TEST_MOTOR
  Test_Motor_Init();
#endif
#ifdef RUN_TEST_DRIVE
  Test_Drive_Init();
#endif
#ifdef P2_TEST_PIVOT
  P2_PivotTurn_Init();
#endif
#ifdef P2_TEST_DRIVE
  P2_OneCellDrive_Init();
#endif
#endif  /* RUN_TEST_I2C_GYRO */
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef RUN_TEST_I2C_GYRO
    Test_I2C_Gyro_Update();
#else
    robot.onMainLoop();

#ifdef RUN_TEST_MOTOR
    Test_Motor_Update();
#endif
#endif  /* RUN_TEST_I2C_GYRO */
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    robot.onControlTick();
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
