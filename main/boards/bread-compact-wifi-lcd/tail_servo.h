/*
 * 双轴尾巴舵机驱动 - 猫咪尾巴
 * 水平轴（左右摆动）+ 垂直轴（上下翘/垂）
 * 使用 LEDC PWM 输出 50Hz 信号控制舵机角度
 */

#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

class TailServo {
public:
    TailServo(gpio_num_t horizontal_pin, gpio_num_t vertical_pin);
    void Initialize();

    /* 设置角度 0~180（90=居中） */
    void SetHorizontal(int angle);
    void SetVertical(int angle);
    void SetAngle(int angle);  /* 兼容：同时设置两轴到同一角度 */

    /* 预设动作 */
    void Wag();         /* 快速左右摇 */
    void WagSlow();     /* 慢速左右摇 */
    void Droop();       /* 下垂（难过） */
    void Perk();        /* 竖起（警觉） */

private:
    gpio_num_t horizontal_pin_;
    gpio_num_t vertical_pin_;

    int h_angle_ = 90;
    int v_angle_ = 90;

    static constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
    static constexpr ledc_channel_t kHorizontalChannel = LEDC_CHANNEL_1;
    static constexpr ledc_channel_t kVerticalChannel   = LEDC_CHANNEL_2;

    /* 14-bit, 50Hz: 0.5ms=410, 2.5ms=2048 */
    static constexpr int kDutyMin = 410;
    static constexpr int kDutyMax = 2048;

    int AngleToDuty(int angle);
    void SmoothH(int target, int step_delay_ms = 12);
    void SmoothV(int target, int step_delay_ms = 12);
};
