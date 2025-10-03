#include "can.h"
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "M3508_Motor.h"

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if(GPIO_Pin == BUTTON_Pin)
//     {
//         uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
//         uint32_t brightness = (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2) + 100) % arr_value;
//         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
//     }
// }

// uint32_t count = 0;
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim == &htim1)
//     {
//         count++;
//     }
// }

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


M3508_Motor Motor(19.2);
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];
extern uint32_t can_tx_mail_box_;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan -> Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x201)
        {
            Motor.canRxMsgCallback(rx_data);
        }
    }
}

uint32_t cnt;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim -> Instance == htim6.Instance)
    {
        cnt++;
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data,&can_tx_mail_box_);
    }
}
