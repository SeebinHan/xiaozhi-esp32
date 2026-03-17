/*
 * 单轴云台驱动实现（仅俯仰）
 * 共享 LEDC_TIMER_1（与尾巴舵机共用）
 */

#include "head_gimbal.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HeadGimbal";

HeadGimbal::HeadGimbal(gpio_num_t tilt_pin)
    : tilt_pin_(tilt_pin) {}

void HeadGimbal::Initialize() {
    ESP_LOGI(TAG, "Initializing head gimbal (tilt only): tilt=GPIO%d", tilt_pin_);

    /* TIMER1 已由 TailServo 初始化，重复配置无害 */
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = kTimer;
    timer_conf.duty_resolution = LEDC_TIMER_14_BIT;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    /* Tilt channel */
    ledc_channel_config_t ch_conf = {};
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = kTiltChannel;
    ch_conf.timer_sel = kTimer;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.gpio_num = tilt_pin_;
    ch_conf.duty = AngleToDuty(90);
    ch_conf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ESP_LOGI(TAG, "Head gimbal ready (tilt only)");
}

int HeadGimbal::AngleToDuty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return kDutyMin + (kDutyMax - kDutyMin) * angle / 180;
}

void HeadGimbal::SetServo(ledc_channel_t ch, int angle) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, AngleToDuty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void HeadGimbal::SetTilt(int angle) {
    SetServo(kTiltChannel, angle);
    tilt_angle_ = angle;
}

void HeadGimbal::SmoothTilt(int target, int step_delay_ms) {
    int start = tilt_angle_;
    int diff = target - start;
    int steps = abs(diff);
    if (steps == 0) return;
    int dir = (diff > 0) ? 1 : -1;
    for (int i = 1; i <= steps; i++) {
        SetTilt(start + i * dir);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

void HeadGimbal::LookCenter() {
    SmoothTilt(90, 12);
}

void HeadGimbal::LookUp() {
    SmoothTilt(135, 12);
}

void HeadGimbal::LookDown() {
    SmoothTilt(60, 12);
}

void HeadGimbal::Nod() {
    /* 点头：上下点两次 */
    for (int i = 0; i < 2; i++) {
        SmoothTilt(70, 10);
        SmoothTilt(100, 10);
    }
    SmoothTilt(90, 12);
}

void HeadGimbal::Shake() {
    /* 无水平轴，改为快速连续点头表示不满 */
    for (int i = 0; i < 3; i++) {
        SmoothTilt(75, 6);
        SmoothTilt(105, 6);
    }
    SmoothTilt(90, 10);
}
