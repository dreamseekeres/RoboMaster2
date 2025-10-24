#include "M3508_Motor.h"


#include <cmath>


M3508_Motor::M3508_Motor(const float ratio,
                         float sp_kp, float sp_ki, float sp_kd, float sp_i_max, float sp_out_max,
                         float pp_kp, float pp_ki, float pp_kd, float pp_i_max, float pp_out_max)
    : ratio_(ratio),
      spid_(sp_kp, sp_ki, sp_kd, sp_i_max, sp_out_max),
      ppid_(pp_kp, pp_ki, pp_kd, pp_i_max, pp_out_max),
      control_method_(TORQUE)
{
    // 初始化变量
    target_angle_ = 0.0f;
    fdb_angle_ = 0.0f;
    target_speed_ = 0.0f;
    fdb_speed_ = 0.0f;
    feedforward_speed_ = 0.0f;
    feedforward_intensity_ = 0.0f;
    output_intensity_ = 0.0f;
}



void M3508_Motor::handle() {
    switch (control_method_) {
    case TORQUE:
        // 直接力矩控制，output_intensity_已在SetIntensity中设置
        break;

    case SPEED:
        // 速度环控制
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        break;

    case POSITION_SPEED:
        // 位置-速度串级控制
        // 位置环输出作为速度环的目标
        float speed_target = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
        // 速度环计算输出电流
        output_intensity_ = spid_.calc(speed_target, fdb_speed_) + feedforward_intensity_;
        break;
    }

    // 限幅保护
    output_intensity_ = std::fmax(std::fmin(output_intensity_, 20.0f), -20.0f);
}








float M3508_Motor::linearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    if (in_max == in_min && out_min == out_max) return out_min;
    float ratio = (float)(in - in_min) / (float)(in_max - in_min);
    return out_min + ratio * (out_max - out_min);
}



void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]){
    //用uint16转换报文
    uint16_t read_ecd_angle_ = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    int16_t read_rotate_speed_ = static_cast<int16_t>((uint16_t(rx_data[2])<< 8) | rx_data[3]);
    int16_t read_current_ = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    //解算 delta_ecd_angle_
    last_ecd_angle_ = linearMapping(read_ecd_angle_, 0, 8191, 0.0, 360.0);
    if(last_ecd_angle_ > ecd_angle_) {
        delta_ecd_angle_ = last_ecd_angle_ - ecd_angle_;
        if(delta_ecd_angle_ > 180) delta_ecd_angle_ -= 360;
    }
    else{
        delta_ecd_angle_ = last_ecd_angle_ - ecd_angle_;
        if(delta_ecd_angle_ < -180) delta_ecd_angle_ += 360;
    }
    //temp、current、ecd_angle_解算
    rotate_speed_   = static_cast<float>(read_rotate_speed_);
    ecd_angle_ = linearMapping(read_ecd_angle_, 0, 8191, 0.0, 360.0);
    current_ = linearMapping(read_current_, -16384, 16384, -20.0, 20.0);
    temp_ = (float)rx_data[6];
    //计算累计
    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;
}



void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    output_intensity_ = intensity;
}



float M3508_Motor::FeedforwardIntensityCalc(float current_angle)
{
    const float g = 9.8f;              // 重力加速度

    // 计算重力产生的力矩
    float angle_rad = current_angle * 3.1415926535f / 180.0f;
    float gravity_torque = load_mass_ * g * arm_length_ * sinf(angle_rad);

    // 计算电机轴需要的力矩 (考虑减速比)
    float motor_torque = gravity_torque / ratio_;

    // 计算需要的电流
    float intensity = motor_torque / torque_constant_;

    return intensity;
}