#pragma once

#include <cstdint>

#include "pca9685.h"

class HeadGimbal {
public:
    HeadGimbal(Pca9685* driver, uint8_t pan_channel, uint8_t tilt_channel);
    void Initialize();
    void SetPan(int angle);
    void SetTilt(int angle);
    int pan_angle() const { return pan_angle_; }
    int tilt_angle() const { return tilt_angle_; }

private:
    Pca9685* driver_ = nullptr;
    uint8_t ch_pan_ = 0;
    uint8_t ch_tilt_ = 1;
    static constexpr int kPulseMinUs = 500;
    static constexpr int kPulseMaxUs = 2500;
    int pan_angle_ = 90;
    int tilt_angle_ = 90;
    float AngleToPulseUs(int angle) const;
};
