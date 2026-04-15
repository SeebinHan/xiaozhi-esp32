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
    h_angle_ = angle;
    if (!driver_) return;
    driver_->SetServoPulse(ch_h_, AngleToPulseUs(angle));
}

void TailServo::SetVertical(int angle) {
    v_angle_ = angle;
    if (!driver_) return;
    driver_->SetServoPulse(ch_v_, AngleToPulseUs(angle));
}

void TailServo::SetAngle(int angle) {
    SetHorizontal(angle);
    SetVertical(angle);
}
