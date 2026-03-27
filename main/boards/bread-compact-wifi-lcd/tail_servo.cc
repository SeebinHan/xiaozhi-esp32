/*
 * 双轴尾巴舵机驱动实现（PCA9685）
 */

#include "tail_servo.h"

#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdlib>

static const char* TAG = "TailServo";

TailServo::TailServo(Pca9685* driver, uint8_t horizontal_channel, uint8_t vertical_channel)
    : driver_(driver), ch_h_(horizontal_channel), ch_v_(vertical_channel) {}

void TailServo::Initialize() {
    ESP_LOGI(TAG, "Initializing dual tail servo on PCA9685: H=CH%u, V=CH%u", ch_h_, ch_v_);
    if (driver_) {
        // 假设外部已调用 InitializeServoMode，一般在板级初始化完成
        SetHorizontal(h_angle_);
        SetVertical(v_angle_);
    }
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

void TailServo::SmoothH(int target, int step_delay_ms) {
    int start = h_angle_;
    int diff = target - start;
    int steps = std::abs(diff);
    if (steps == 0) return;
    int dir = (diff > 0) ? 1 : -1;
    for (int i = 1; i <= steps; i++) {
        SetHorizontal(start + i * dir);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

void TailServo::SmoothV(int target, int step_delay_ms) {
    int start = v_angle_;
    int diff = target - start;
    int steps = std::abs(diff);
    if (steps == 0) return;
    int dir = (diff > 0) ? 1 : -1;
    for (int i = 1; i <= steps; i++) {
        SetVertical(start + i * dir);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

void TailServo::Wag() {
    /* 快速左右摇3次，垂直轴微微翘起增加活力 */
    SmoothV(110, 10);
    for (int i = 0; i < 3; i++) {
        SmoothH(55, 8);
        SmoothH(125, 8);
    }
    SmoothH(90, 10);
    SmoothV(90, 10);
}

void TailServo::WagSlow() {
    /* 慢速柔和摇摆 */
    for (int i = 0; i < 2; i++) {
        SmoothH(65, 18);
        SmoothH(115, 18);
    }
    SmoothH(90, 20);
}

void TailServo::Droop() {
    /* 尾巴下垂：垂直轴向下，水平居中 */
    SmoothH(90, 15);
    SmoothV(20, 20);
}

void TailServo::Perk() {
    /* 尾巴竖起：垂直轴向上，水平居中 */
    SmoothH(90, 10);
    SmoothV(160, 10);
}
