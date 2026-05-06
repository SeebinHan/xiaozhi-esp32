#include "head_gimbal.h"

#include <esp_log.h>

#define TAG "HeadGimbal"

HeadGimbal::HeadGimbal(Pca9685* driver, uint8_t pan_channel, uint8_t tilt_channel)
    : driver_(driver), ch_pan_(pan_channel), ch_tilt_(tilt_channel) {}

void HeadGimbal::Initialize() {
    ESP_LOGI(TAG, "Initializing head gimbal on PCA9685: pan=CH%u, tilt=CH%u", ch_pan_, ch_tilt_);
}

float HeadGimbal::AngleToPulseUs(int angle) const {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return kPulseMinUs + (kPulseMaxUs - kPulseMinUs) * (angle / 180.0f);
}

void HeadGimbal::SetPan(int angle) {
    if (!driver_) return;
    int calibrated = ApplyCalibration(angle, pan_calibration_);
    if (!NeedsServoWrite(last_written_pan_, calibrated, pan_calibration_.deadband_deg)) {
        UpdateCachedAngleOnWriteResult(pan_angle_, angle, true);
        return;
    }
    bool write_success = driver_->SetServoPulse(ch_pan_, AngleToPulseUs(calibrated));
    UpdateCachedAngleOnWriteResult(pan_angle_, angle, write_success);
    if (write_success) {
        last_written_pan_ = calibrated;
    }
}

void HeadGimbal::SetTilt(int angle) {
    if (!driver_) return;
    int calibrated = ApplyCalibration(angle, tilt_calibration_);
    if (!NeedsServoWrite(last_written_tilt_, calibrated, tilt_calibration_.deadband_deg)) {
        UpdateCachedAngleOnWriteResult(tilt_angle_, angle, true);
        return;
    }
    bool write_success = driver_->SetServoPulse(ch_tilt_, AngleToPulseUs(calibrated));
    UpdateCachedAngleOnWriteResult(tilt_angle_, angle, write_success);
    if (write_success) {
        last_written_tilt_ = calibrated;
    }
}
