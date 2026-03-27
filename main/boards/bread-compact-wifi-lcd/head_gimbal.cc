/*
 * 双轴云台驱动实现（PCA9685）
 */

#include "head_gimbal.h"

#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdlib>

static const char* TAG = "HeadGimbal";

HeadGimbal::HeadGimbal(Pca9685* driver, uint8_t pan_channel, uint8_t tilt_channel)
    : driver_(driver), ch_pan_(pan_channel), ch_tilt_(tilt_channel) {}

void HeadGimbal::Initialize() {
    ESP_LOGI(TAG, "Initializing head gimbal on PCA9685: pan=CH%u, tilt=CH%u", ch_pan_, ch_tilt_);
    if (driver_) {
        SetPan(pan_angle_);
        SetTilt(tilt_angle_);
    }
}

float HeadGimbal::AngleToPulseUs(int angle) const {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return kPulseMinUs + (kPulseMaxUs - kPulseMinUs) * (angle / 180.0f);
}

void HeadGimbal::SetPan(int angle) {
    pan_angle_ = angle;
    if (!driver_) return;
    driver_->SetServoPulse(ch_pan_, AngleToPulseUs(angle));
}

void HeadGimbal::SetTilt(int angle) {
    tilt_angle_ = angle;
    if (!driver_) return;
    driver_->SetServoPulse(ch_tilt_, AngleToPulseUs(angle));
}

void HeadGimbal::SmoothPan(int target, int step_delay_ms) {
    int start = pan_angle_;
    int diff = target - start;
    int steps = std::abs(diff);
    if (steps == 0) return;
    int dir = (diff > 0) ? 1 : -1;
    for (int i = 1; i <= steps; i++) {
        SetPan(start + i * dir);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

void HeadGimbal::SmoothTilt(int target, int step_delay_ms) {
    int start = tilt_angle_;
    int diff = target - start;
    int steps = std::abs(diff);
    if (steps == 0) return;
    int dir = (diff > 0) ? 1 : -1;
    for (int i = 1; i <= steps; i++) {
        SetTilt(start + i * dir);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

void HeadGimbal::LookCenter() {
    SmoothPan(90, 10);
    SmoothTilt(90, 10);
}

void HeadGimbal::LookUp() {
    SmoothPan(90, 10);
    SmoothTilt(130, 10);
}

void HeadGimbal::LookDown() {
    SmoothPan(90, 10);
    SmoothTilt(60, 10);
}

void HeadGimbal::Nod() {
    for (int i = 0; i < 2; i++) {
        SmoothTilt(70, 8);
        SmoothTilt(105, 8);
    }
    SmoothPan(90, 10);
    SmoothTilt(90, 10);
}

void HeadGimbal::Shake() {
    for (int i = 0; i < 2; i++) {
        SmoothPan(60, 8);
        SmoothPan(120, 8);
    }
    SmoothPan(90, 10);
    SmoothTilt(90, 10);
}
