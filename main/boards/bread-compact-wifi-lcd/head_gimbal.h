/*
 * 单轴云台驱动 - 猫咪头部（仅俯仰）
 * 水平轴已移除，GPIO 释放给尾巴第二舵机
 */

#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

class HeadGimbal {
public:
    HeadGimbal(gpio_num_t tilt_pin);
    void Initialize();

    /* 设置俯仰角度：0~180 (90=水平) */
    void SetTilt(int angle);

    /* 预设动作 */
    void LookCenter();     /* 正前方 */
    void LookUp();         /* 抬头 */
    void LookDown();       /* 低头 */
    void Nod();            /* 点头 */
    void Shake();          /* 原摇头 → 改为快速点头（无水平轴） */

private:
    gpio_num_t tilt_pin_;

    /* 共享 TIMER1（和尾巴舵机相同的 50Hz timer） */
    static constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
    static constexpr ledc_channel_t kTiltChannel = LEDC_CHANNEL_3;

    /* 14-bit, 50Hz: 0.5ms=410, 2.5ms=2048 */
    static constexpr int kDutyMin = 410;
    static constexpr int kDutyMax = 2048;

    int tilt_angle_ = 90;

    int AngleToDuty(int angle);
    void SetServo(ledc_channel_t ch, int angle);
    void SmoothTilt(int target, int step_delay_ms = 12);
};
