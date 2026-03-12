/*
 * MG90S 舵机驱动实现
 */

#include "tail_servo.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TailServo";

TailServo::TailServo(gpio_num_t pin) : pin_(pin) {}

void TailServo::Initialize() {
    ESP_LOGI(TAG, "Initializing tail servo on GPIO %d", pin_);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = timer_;
    timer_conf.duty_resolution = LEDC_TIMER_14_BIT;  /* 16384 ticks */
    timer_conf.freq_hz = 50;  /* 50Hz for servo */
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {};
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = channel_;
    ch_conf.timer_sel = timer_;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.gpio_num = pin_;
    ch_conf.duty = AngleToDuty(90);  /* 初始居中 */
    ch_conf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ESP_LOGI(TAG, "Tail servo ready");
}

int TailServo::AngleToDuty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return kDutyMin + (kDutyMax - kDutyMin) * angle / 180;
}

void TailServo::SetAngle(int angle) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, AngleToDuty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
}

void TailServo::Wag() {
    /* 快速摇尾巴：左右摆动3次 */
    for (int i = 0; i < 3; i++) {
        SetAngle(45);
        vTaskDelay(pdMS_TO_TICKS(150));
        SetAngle(135);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    SetAngle(90);
}

void TailServo::WagSlow() {
    /* 慢速摇尾巴：放松/舒服 */
    for (int i = 0; i < 2; i++) {
        SetAngle(60);
        vTaskDelay(pdMS_TO_TICKS(400));
        SetAngle(120);
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    SetAngle(90);
}

void TailServo::Droop() {
    /* 尾巴下垂：难过 */
    SetAngle(20);
}

void TailServo::Perk() {
    /* 尾巴竖起：警觉/开心 */
    SetAngle(160);
}
