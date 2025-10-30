//
// Created by 43034 on 2025/10/3.
//
#ifndef TRY_M3508_MOTOR_H
#define TRY_M3508_MOTOR_H
#include <cstdint>
#include "pid.h"
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
    // PID控制器和控制变量
    PID spid_, ppid_;  // 速度环和位置环PID
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;

    //负重计算
    const float load_mass_ = 0.5f;           // 500g负重
    const float arm_length_ = 0.05524f;      // 55.24mm力臂
    //const float torque_constant_ = 0.286f;    // 扭矩常数
    const float torque_constant_ = 0.31f;    // 扭矩常数



public:
    float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);
    explicit M3508_Motor(float ratio,
                    float sp_kp = 0, float sp_ki = 0, float sp_kd = 0, float sp_i_max = 0, float sp_out_max = 0,
                    float pp_kp = 0, float pp_ki = 0, float pp_kd = 0, float pp_i_max = 0, float pp_out_max = 0);
    void canRxMsgCallback(const uint8_t rx_data[8]);


    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);

    void handle();  // 控制处理函数
    float FeedforwardIntensityCalc(float current_angle);

    // 获取状态信息
    float getAngle() const { return angle_; }
    float getSpeed() const { return rotate_speed_ / ratio_; }  // 返回机械转速
    float getCurrent() const { return current_; }
    float getOutputIntensity() const { return output_intensity_; }

};



#endif //TRY_M3508_MOTOR_H