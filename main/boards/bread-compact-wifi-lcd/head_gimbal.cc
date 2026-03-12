/*
 * 二维云台驱动实现
 * Pan/Tilt 共享 LEDC_TIMER_1（与尾巴舵机共用）
 */

#include "head_gimbal.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HeadGimbal";

HeadGimbal::HeadGimbal(gpio_num_t pan_pin, gpio_num_t tilt_pin)
    : pan_pin_(pan_pin), tilt_pin_(tilt_pin) {}

void HeadGimbal::Initialize() {
    ESP_LOGI(TAG, "Initializing head gimbal: pan=GPIO%d, tilt=GPIO%d", pan_pin_, tilt_pin_);

    /* TIMER1 已由 TailServo 初始化，这里只添加 channel */
    /* 如果 TailServo 未初始化，需要自己配置 timer */
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = kTimer;
    timer_conf.duty_resolution = LEDC_TIMER_14_BIT;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);  /* 重复配置无害 */

    /* Pan channel */
    ledc_channel_config_t ch_conf = {};
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = kPanChannel;
    ch_conf.timer_sel = kTimer;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.gpio_num = pan_pin_;
    ch_conf.duty = AngleToDuty(90);
    ch_conf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    /* Tilt channel */
    ch_conf.channel = kTiltChannel;
    ch_conf.gpio_num = tilt_pin_;
    ch_conf.duty = AngleToDuty(90);
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    ESP_LOGI(TAG, "Head gimbal ready");
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

void HeadGimbal::SetPan(int angle) {
    SetServo(kPanChannel, angle);
}

void HeadGimbal::SetTilt(int angle) {
    SetServo(kTiltChannel, angle);
}

void HeadGimbal::SetAngles(int pan, int tilt) {
    SetPan(pan);
    SetTilt(tilt);
}

void HeadGimbal::LookCenter() {
    SetAngles(90, 90);
}

void HeadGimbal::LookLeft() {
    SetPan(135);
}

void HeadGimbal::LookRight() {
    SetPan(45);
}

void HeadGimbal::LookUp() {
    SetTilt(135);
}

void HeadGimbal::LookDown() {
    SetTilt(60);
}

void HeadGimbal::Nod() {
    /* 点头：上下点两次 */
    for (int i = 0; i < 2; i++) {
        SetTilt(70);
        vTaskDelay(pdMS_TO_TICKS(200));
        SetTilt(100);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    SetTilt(90);
}

void HeadGimbal::Shake() {
    /* 摇头：左右摇两次 */
    for (int i = 0; i < 2; i++) {
        SetPan(70);
        vTaskDelay(pdMS_TO_TICKS(250));
        SetPan(110);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    SetPan(90);
}
