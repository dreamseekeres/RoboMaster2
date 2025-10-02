#include "main.h"
#include "gpio.h"
#include "tim.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BUTTON_Pin)
    {
        uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
        uint32_t brightness = (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2) + 100) % arr_value;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
    }
}