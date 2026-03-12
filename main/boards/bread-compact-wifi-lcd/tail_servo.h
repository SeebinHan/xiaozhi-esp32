/*
 * MG90S 舵机驱动 - 猫咪尾巴
 * 使用 LEDC PWM 输出 50Hz 信号控制舵机角度
 */

#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

class TailServo {
public:
    TailServo(gpio_num_t pin);
    void Initialize();

    /* 设置舵机角度 0~180 */
    void SetAngle(int angle);

    /* 预设动作 */
    void Wag();         /* 摇尾巴 */
    void WagSlow();     /* 慢摇（放松） */
    void Droop();       /* 下垂（难过） */
    void Perk();        /* 竖起（警觉） */

private:
    gpio_num_t pin_;
    ledc_channel_t channel_ = LEDC_CHANNEL_1;  /* CH0 被背光灯占用 */
    ledc_timer_t timer_ = LEDC_TIMER_1;        /* TIMER0 被背光灯占用 */

    /* 角度 -> LEDC duty 值
     * 50Hz = 20ms 周期, 14-bit 分辨率 = 16384 ticks
     * 0.5ms = 410 ticks (0°), 2.5ms = 2048 ticks (180°) */
    static constexpr int kDutyMin = 410;   /* 0° */
    static constexpr int kDutyMax = 2048;  /* 180° */

    int AngleToDuty(int angle);
};
