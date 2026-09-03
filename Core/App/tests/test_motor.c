#include "test_motor.h"
#include "tim.h"

volatile MotorTestPhase motor_phase = MOTOR_IDLE;

/* TIM1 ARR is 4999, so 4999 = 100% duty. 1500 is a gentle ~30%. */
#define DUTY_MAX           4500
#define RAMP_STEPS         20
#define RAMP_STEP_DELAY_MS 20
#define HOLD_TIME_MS       1000
#define PAUSE_MS           500

typedef struct {
    uint32_t      channel;
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
} MotorPins;

static const MotorPins motors[2] = {
    { TIM_CHANNEL_1, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin }, /* Left  PA8 */
    { TIM_CHANNEL_2, AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin }, /* Right PA9 */
};

/* Positive duty = physical forward. IN1/IN2 are swapped relative to the
 * datasheet truth table — verified by physical test on this board.    */
static void SetDuty(const MotorPins *m, int16_t duty)
{
    if (duty >= 0)
    {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, m->channel, duty);
    }
    else
    {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, m->channel, -duty);
    }
}

/* Both IN pins LOW = coast (no active brake). */
static void StopBoth(void)
{
    for (int i = 0; i < 2; i++)
    {
        HAL_GPIO_WritePin(motors[i].in1_port, motors[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motors[i].in2_port, motors[i].in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, motors[i].channel, 0);
    }
}

/* Ramp instead of snapping to full duty — avoids a current inrush and
 * a sudden mechanical jerk.                                           */
static void RampBoth(int16_t target)
{
    int16_t step = target / RAMP_STEPS;

    for (int i = 1; i <= RAMP_STEPS; i++)
    {
        SetDuty(&motors[0], (int16_t)(step * i));
        SetDuty(&motors[1], (int16_t)(step * i));
        HAL_Delay(RAMP_STEP_DELAY_MS);
    }

    HAL_Delay(HOLD_TIME_MS);

    for (int i = RAMP_STEPS; i >= 0; i--)
    {
        SetDuty(&motors[0], (int16_t)(step * i));
        SetDuty(&motors[1], (int16_t)(step * i));
        HAL_Delay(RAMP_STEP_DELAY_MS);
    }

    StopBoth();
    HAL_Delay(PAUSE_MS);
}

void Test_Motor_Run(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    StopBoth();
    HAL_Delay(200);

    motor_phase = MOTOR_FORWARD;
    RampBoth(DUTY_MAX);

    motor_phase = MOTOR_REVERSE;
    RampBoth(-DUTY_MAX);

    motor_phase = MOTOR_DONE;
}
