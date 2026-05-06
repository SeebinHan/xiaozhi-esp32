#pragma once

#include <cstdint>

#include "motion_math.h"
#include "pca9685.h"

class HeadGimbal {
public:
    HeadGimbal(Pca9685* driver, uint8_t pan_channel, uint8_t tilt_channel);
    void Initialize();
    void SetPan(int angle);
    void SetTilt(int angle);
    void SetPanCalibration(const ServoAxisCalibration& calibration) {
        pan_calibration_ = calibration;
        last_written_pan_ = -1;
    }
    void SetTiltCalibration(const ServoAxisCalibration& calibration) {
        tilt_calibration_ = calibration;
        last_written_tilt_ = -1;
    }
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
    int last_written_pan_ = -1;
    int last_written_tilt_ = -1;
    ServoAxisCalibration pan_calibration_{};
    ServoAxisCalibration tilt_calibration_{};
    float AngleToPulseUs(int angle) const;
};
