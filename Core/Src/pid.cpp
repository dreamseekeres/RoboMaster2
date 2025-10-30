#include "PID.h"


PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k)
  : kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), out_max_(out_max), d_filter_k_(d_filter_k) {
    reset();
}

void PID::reset(void) {
    ref_ = 0.0f;
    fdb_ = 0.0f;
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    last_dout_ = 0.0f;
    output_ = 0.0f;
}

float PID::calc(float ref, float fdb) {
    // 保存当前参考值和反馈值
    ref_ = ref;
    fdb_ = fdb;

    // 计算当前误差
    err_ = ref_ - fdb_;

    // 比例项
    pout_ = kp_ * err_;

    // 积分项
    float i_increment = ki_ * err_ * 0.001;
    err_sum_ += i_increment;

    // 积分限幅
    if (err_sum_ > i_max_) {
        err_sum_ = i_max_;
    } else if (err_sum_ < -i_max_) {
        err_sum_ = -i_max_;
    }
    iout_ = err_sum_;

    // 微分项（带滤波）
    float d_raw = kd_ * (err_ - last_err_) / 0.001 ;
    dout_ = d_filter_k_ * last_dout_ + (1 - d_filter_k_) * d_raw;

    // 计算总输出
    output_ = pout_ + iout_ + dout_;

    // 输出限幅
    if (output_ > out_max_) {
        output_ = out_max_;
    } else if (output_ < -out_max_) {
        output_ = -out_max_;
    }

    // 更新历史值
    last_err_ = err_;
    last_dout_ = dout_;

    return output_;
}