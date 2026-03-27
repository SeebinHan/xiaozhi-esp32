/*
 * 双轴云台驱动 - 猫咪头部（左右 + 俯仰），通过 PCA9685 输出 PWM
 */

#pragma once

#include <cstdint>
#include "pca9685.h"

class HeadGimbal {
public:
    HeadGimbal(Pca9685* driver, uint8_t pan_channel, uint8_t tilt_channel);
    void Initialize();

    /* 设置角度：0~180 (90=居中) */
    void SetPan(int angle);
    void SetTilt(int angle);

    /* 预设动作 */
    void LookCenter();     /* 正前方 */
    void LookUp();         /* 抬头 */
    void LookDown();       /* 低头 */
    void Nod();            /* 点头 */
    void Shake();          /* 左右摇头 */

private:
    Pca9685* driver_ = nullptr;
    uint8_t ch_pan_ = 0;
    uint8_t ch_tilt_ = 1;

    /* 舵机常见 0.5ms~2.5ms 脉宽 */
    static constexpr int kPulseMinUs = 500;
    static constexpr int kPulseMaxUs = 2500;

    int pan_angle_ = 90;
    int tilt_angle_ = 90;

    float AngleToPulseUs(int angle) const;
    void SmoothPan(int target, int step_delay_ms = 12);
    void SmoothTilt(int target, int step_delay_ms = 12);
};

