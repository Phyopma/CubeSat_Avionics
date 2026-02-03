#include "hbridge.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

void HBridge_Init(void)
{
    // Start PWM generation on both channels
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // Enable the Driver (Release the Sleep Pin)
    HAL_GPIO_WritePin(MTQ_SLEEP_GPIO_Port, MTQ_SLEEP_Pin, GPIO_PIN_SET);
}

void HBridge_SetVoltage(float voltage, float max_supply)
{
    // 1. Clamp voltage request to available supply
    if (voltage > max_supply)
        voltage = max_supply;
    if (voltage < -max_supply)
        voltage = -max_supply;

    // 2. Calculate Duty Cycle (0.0 to 1.0)
    float duty = fabsf(voltage) / max_supply;

    // 3. Convert Duty Cycle to Timer Compare Value
    // Get Auto-Reload Register (ARR) value (999 in your config)
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = (uint32_t)(duty * (float)arr);

    // 4. Set PWM Channels based on Direction
    if (voltage >= 0.0f)
    {
        // Forward: IN1 = PWM, IN2 = 0
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
    else
    {
        // Reverse: IN1 = 0, IN2 = PWM
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
    }
}

void HBridge_Sleep(bool enable_sleep)
{
    HAL_GPIO_WritePin(MTQ_SLEEP_GPIO_Port, MTQ_SLEEP_Pin,
                      enable_sleep ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
