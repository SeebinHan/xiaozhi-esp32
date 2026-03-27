#pragma once

#include <cstdint>
#include "pca9685.h"

/* 双轴尾巴舵机 - 通过 PCA9685 输出 50Hz PWM */
class TailServo {
public:
    TailServo(Pca9685* driver, uint8_t horizontal_channel, uint8_t vertical_channel);
    void Initialize();

    /* 设置角度 0~180（90=居中） */
    void SetHorizontal(int angle);
    void SetVertical(int angle);
    void SetAngle(int angle);  /* 同时设置两轴到同一角度 */

    /* 预设动作 */
    void Wag();         /* 快速左右摇 */
    void WagSlow();     /* 慢速左右摇 */
    void Droop();       /* 下垂（难过） */
    void Perk();        /* 竖起（警觉） */

private:
    Pca9685* driver_ = nullptr;
    uint8_t ch_h_ = 0;
    uint8_t ch_v_ = 1;

    int h_angle_ = 90;
    int v_angle_ = 90;

    /* 舵机常见 0.5ms~2.5ms 脉宽 */
    static constexpr int kPulseMinUs = 500;
    static constexpr int kPulseMaxUs = 2500;

    float AngleToPulseUs(int angle) const;
    void SmoothH(int target, int step_delay_ms = 12);
    void SmoothV(int target, int step_delay_ms = 12);
};
