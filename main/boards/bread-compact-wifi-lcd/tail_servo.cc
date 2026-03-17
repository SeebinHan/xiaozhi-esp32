/*
 * 双轴尾巴舵机驱动实现
 * 水平轴 + 垂直轴共享 LEDC_TIMER_1
 */

#include "tail_servo.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TailServo";

TailServo::TailServo(gpio_num_t horizontal_pin, gpio_num_t vertical_pin)
    : horizontal_pin_(horizontal_pin), vertical_pin_(vertical_pin) {}

void TailServo::Initialize() {
    ESP_LOGI(TAG, "Initializing dual tail servo: H=GPIO%d, V=GPIO%d",
             horizontal_pin_, vertical_pin_);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = kTimer;
    timer_conf.duty_resolution = LEDC_TIMER_14_BIT;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    /* 水平轴 channel */
    ledc_channel_config_t ch_conf = {};
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = kHorizontalChannel;
    ch_conf.timer_sel = kTimer;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.gpio_num = horizontal_pin_;
    ch_conf.duty = AngleToDuty(90);
    ch_conf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    /* 垂直轴 channel */
    ch_conf.channel = kVerticalChannel;
    ch_conf.gpio_num = vertical_pin_;
    ch_conf.duty = AngleToDuty(90);
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ESP_LOGI(TAG, "Dual tail servo ready");
}

int TailServo::AngleToDuty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return kDutyMin + (kDutyMax - kDutyMin) * angle / 180;
}

void TailServo::SetHorizontal(int angle) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, kHorizontalChannel, AngleToDuty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, kHorizontalChannel);
    h_angle_ = angle;
}

void TailServo::SetVertical(int angle) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, kVerticalChannel, AngleToDuty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, kVerticalChannel);
    v_angle_ = angle;
}

void TailServo::SetAngle(int angle) {
    SetHorizontal(angle);
    SetVertical(angle);
}

void TailServo::SmoothH(int target, int step_delay_ms) {
    int start = h_angle_;
    int diff = target - start;
    int steps = abs(diff);
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
    int steps = abs(diff);
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
