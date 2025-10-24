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

// 按钮回调函数 - 切换stop_flag
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BUTTON_Pin)
    {
        static uint32_t last_button_time = 0;
        uint32_t current_time = HAL_GetTick();

        // 简单的按钮消抖，200ms内不重复触发
        if(current_time - last_button_time > 200)
        {
            stop_flag = !stop_flag;  // 切换stop_flag状态
            last_button_time = current_time;
        }
    }
}



// 在callback.c的Motor实例化处：
M3508_Motor Motor(19.2,
                  // 速度环PID: Kp, Ki, Kd, I_max, Out_max
                  2.5f, 0.8f, 0.1f, 5.0f, 8.0f,
                  // 位置环PID: Kp, Ki, Kd, I_max, Out_max
                  15.0f, 0.5f, 1.2f, 30.0f, 30.0f);



// 阶跃测试函数
void step_test(void) {
    static uint32_t last_step_time = 0;
    static uint8_t test_state = 0;
    static float initial_angle = 0.0f;  // 保存初始角度
    uint32_t current_time = HAL_GetTick();

    if (test_state == 0) {
        // 初始化，保存当前角度
        initial_angle = target_angle;
        test_state = 1;
        last_step_time = current_time;
    }
    else if (test_state == 1 && current_time - last_step_time > 2000) {
        // 2秒后施加10°阶跃
        target_angle = initial_angle + 10.0f;  // 使用全局变量
        test_state = 2;
        last_step_time = current_time;
        // printf("Step applied: +10 deg, Target: %.1f\n", target_angle);
    }
    else if (test_state == 2 && current_time - last_step_time > 100) {
        // 0.1秒后检查误差
        float error = fabsf(target_angle - Motor.getAngle());
        if (error < 0.1f) {
            // printf("Test PASSED! Error: %.3f deg\n", error);
        } else {
            // printf("Test FAILED! Error: %.3f deg\n", error);
        }
        test_state = 3;
    }
    else if (test_state == 3 && current_time - last_step_time > 3000) {
        // 3秒后重置测试
        test_state = 0;
    }
}







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
    if (htim -> Instance == htim6.Instance)
    {
        cnt++;

        // 执行阶跃测试
        step_test();

        // 控制逻辑
        if (stop_flag) {
            // 停止时发送0电流
            tx_data[0] = 0x00;
            tx_data[1] = 0x00;
        } else {
            // 使用全局target_angle变量，不再定义局部变量

            // 计算前馈力矩
            float ff_intensity = Motor.FeedforwardIntensityCalc(target_angle);

            // 设置位置控制
            Motor.SetPosition(target_angle, 0, ff_intensity);
            Motor.handle();

            float current = Motor.getOutputIntensity();
            int16_t current_int = (int16_t)(current * 16384.0f / 20.0f);

            // 只更新电流部分，其他字节保持初始值
            tx_data[0] = (current_int >> 8) & 0xFF;
            tx_data[1] = current_int & 0xFF;
        }

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
    }
}