#include "M3508_Motor.h"


float M3508_Motor::linearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    if (in_max == in_min && out_min == out_max) return out_min;
    float ratio = (float)(in - in_min) / (float)(in_max - in_min);
    return out_min + ratio * (out_max - out_min);
}



void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]){
    uint16_t read_ecd_angle_ = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    int16_t read_rotate_speed_ = static_cast<int16_t>((uint16_t(rx_data[2])<< 8) | rx_data[3]);
    int16_t read_current_ = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    rotate_speed_   = static_cast<float>(read_rotate_speed_);
    ecd_angle_ = linearMapping(read_ecd_angle_, 0, 8191, 0.0, 360.0);
    current_ = linearMapping(read_current_, -16384, 16384, -20.0, 20.0);
    temp_ = (float)rx_data[6];
}


