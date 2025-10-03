//
// Created by 43034 on 2025/10/3.
//
#ifndef TRY_M3508_MOTOR_H
#define TRY_M3508_MOTOR_H
#include <cstdint>

class M3508_Motor
{
private:
    const float ratio_;
    float angle_ = 0.0f;
    float delta_angle_ = 0.0f;
    float ecd_angle_ = 0.0f;
    float last_ecd_angle_ = 0.0f;
    float delta_ecd_angle_ = 0.0f;
    float rotate_speed_ = 0.0f;
    float current_ = 0.0f;
    float temp_ = 0.0f;
public:
    float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);
    explicit M3508_Motor(const float ratio):ratio_(ratio){ };
    void canRxMsgCallback(const uint8_t rx_data[8]);

};



#endif //TRY_M3508_MOTOR_H