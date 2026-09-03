#include "test_encoder.h"
#include "tim.h"

volatile int32_t enc_left_count  = 0;
volatile int32_t enc_right_count = 0;
volatile int32_t enc_left_delta  = 0;
volatile int32_t enc_right_delta = 0;

/* Both counters start mid-range so turning backward reads as a small
 * negative number instead of wrapping to ~4 billion.                  */
#define ENC_START_VALUE  0x80000000
#define DELTA_PERIOD_MS  200

static int32_t prev_left  = 0;
static int32_t prev_right = 0;

void Test_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);   /* Left  */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   /* Right */

    __HAL_TIM_SET_COUNTER(&htim5, ENC_START_VALUE);
    __HAL_TIM_SET_COUNTER(&htim2, ENC_START_VALUE);

    prev_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    prev_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

void Test_Encoder_Update(void)
{
    int32_t left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    int32_t right = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    enc_left_count  = left  - (int32_t)ENC_START_VALUE;
    enc_right_count = -(right - (int32_t)ENC_START_VALUE);

    /* Deltas only every 200ms. Computed every loop pass they'd be 0
     * almost always at hand-turning speed, and unreadable live.       */
    static uint32_t last_tick = 0;
    if ((HAL_GetTick() - last_tick) < DELTA_PERIOD_MS)
    {
        return;
    }
    last_tick = HAL_GetTick();

    enc_left_delta  = left  - prev_left;
    enc_right_delta = -(right - prev_right);

    prev_left  = left;
    prev_right = right;
}
