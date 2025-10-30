#include <cmath>

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
extern uint8_t stop_flag;
extern float target_angle;


/* USER CODE BEGIN 4 */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == BUTTON_Pin)  // 检查是否是BUTTON引脚触发的中断
//     {
//         static uint32_t last_button_time = 0;
//         uint32_t current_time = HAL_GetTick();
//
//         // 按钮消抖处理，200ms内不重复触发
//         if (current_time - last_button_time > 200)
//         {
//             stop_flag = !stop_flag;  // 切换stop_flag状态
//             last_button_time = current_time;
//         }
//     }
// }
/* USER CODE END 4 */
/* USER CODE END 4 */
// 在callback.c的Motor实例化处：
// M3508_Motor Motor(19.2,
//                   // 速度环PID: Kp, Ki, Kd, I_max, Out_max
//                   2.5f, 0.8f, 0.1f, 5.0f, 8.0f,
//                   // 位置环PID: Kp, Ki, Kd, I_max, Out_max
//                   15.0f, 0.5f, 1.2f, 30.0f, 30.0f);
M3508_Motor Motor(19.2,
                  // 速度环PID - 更快的响应
                  0.6f, 0.0f, 0.0005f, 0.0f, 20.0f,
                  // 位置环PID - 更高的增益
                  2.6f, 1.0f, 0.006f, 0.9f, 20.0f);





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
    if (htim->Instance == htim6.Instance)
    {
        cnt++;
        // static uint8_t step_applied = 0;
        // if (cnt == 1000 && !step_applied) {  // 1秒后施加阶跃
        //     target_angle += 10.0f;
        //     step_applied = 1;
        // }
        if (stop_flag)
        {
            // 停止时不输出电流
            tx_data[0] = 0x00;
            tx_data[1] = 0x00;
        }
        else
        {
            //前馈补偿
            float current_angle = Motor.getAngle();
            float feedforward_current = Motor.FeedforwardIntensityCalc(current_angle);

            // //速度环调试
            // float target_speed = 100.0f;  // 30度/秒
            // Motor.SetSpeed(target_speed, feedforward_current);
            // Motor.handle();

            // Motor.SetPosition(current_angle,0.0f, feedforward_current);
            // Motor.handle();

            //
            //最终控制
             Motor.SetPosition(target_angle, 0.0f, feedforward_current);
             Motor.handle();
            float current = Motor.getOutputIntensity();
            int16_t current_int = (int16_t)(current * 16384.0f / 20.0f);
            tx_data[0] = (current_int >> 8) & 0xFF;
            tx_data[1] = current_int & 0xFF;
        }

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
    }
}


