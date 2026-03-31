#pragma once

#include <cstdint>
#include "pca9685.h"

/* 双轴尾巴舵机 - 通过 PCA9685 输出 50Hz PWM */
class TailServo {
public:
    TailServo(Pca9685* driver, uint8_t horizontal_channel, uint8_t vertical_channel);
    void Initialize();

    void SetHorizontal(int angle);
    void SetVertical(int angle);
    void SetAngle(int angle);
    int horizontal_angle() const { return h_angle_; }
    int vertical_angle() const { return v_angle_; }

private:
    Pca9685* driver_ = nullptr;
    uint8_t ch_h_ = 0;
    uint8_t ch_v_ = 1;

    int h_angle_ = 90;
    int v_angle_ = 90;

    static constexpr int kPulseMinUs = 500;
    static constexpr int kPulseMaxUs = 2500;

    float AngleToPulseUs(int angle) const;
};
