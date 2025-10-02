#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BUTTON_Pin)
    {
        uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
        uint32_t brightness = (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2) + 100) % arr_value;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
    }
}

uint32_t count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        count++;
    }
}

extern uint8_t rx_msg[4];
extern uint8_t tx_msg[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7)
    {
        tx_msg[0] = rx_msg[0];
        tx_msg[1] = rx_msg[1];
        tx_msg[2] = rx_msg[2];
        tx_msg[3] = rx_msg[3];
    }
    HAL_UART_Receive_IT(&huart7, rx_msg,3);
}
