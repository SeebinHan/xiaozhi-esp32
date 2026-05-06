#include "tail_servo.h"

#include <esp_log.h>

#define TAG "TailServo"

TailServo::TailServo(Pca9685* driver, uint8_t horizontal_channel, uint8_t vertical_channel)
    : driver_(driver), ch_h_(horizontal_channel), ch_v_(vertical_channel) {}

void TailServo::Initialize() {
    ESP_LOGI(TAG, "Initializing dual tail servo on PCA9685: H=CH%u, V=CH%u", ch_h_, ch_v_);
}

float TailServo::AngleToPulseUs(int angle) const {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return kPulseMinUs + (kPulseMaxUs - kPulseMinUs) * (angle / 180.0f);
}

void TailServo::SetHorizontal(int angle) {
    if (!driver_) return;
    int calibrated = ApplyCalibration(angle, horizontal_calibration_);
    if (!NeedsServoWrite(last_written_horizontal_, calibrated, horizontal_calibration_.deadband_deg)) {
        UpdateCachedAngleOnWriteResult(h_angle_, angle, true);
        return;
    }
    bool write_success = driver_->SetServoPulse(ch_h_, AngleToPulseUs(calibrated));
    UpdateCachedAngleOnWriteResult(h_angle_, angle, write_success);
    if (write_success) {
        last_written_horizontal_ = calibrated;
    }
}

void TailServo::SetVertical(int angle) {
    if (!driver_) return;
    int calibrated = ApplyCalibration(angle, vertical_calibration_);
    if (!NeedsServoWrite(last_written_vertical_, calibrated, vertical_calibration_.deadband_deg)) {
        UpdateCachedAngleOnWriteResult(v_angle_, angle, true);
        return;
    }
    bool write_success = driver_->SetServoPulse(ch_v_, AngleToPulseUs(calibrated));
    UpdateCachedAngleOnWriteResult(v_angle_, angle, write_success);
    if (write_success) {
        last_written_vertical_ = calibrated;
    }
}

void TailServo::SetAngle(int angle) {
    SetHorizontal(angle);
    SetVertical(angle);
}
