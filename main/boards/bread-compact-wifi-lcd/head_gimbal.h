/*
 * 二维云台驱动 - 猫咪头部
 * Pan（水平旋转）+ Tilt（俯仰）两轴 PWM 舵机
 */

#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

class HeadGimbal {
public:
    HeadGimbal(gpio_num_t pan_pin, gpio_num_t tilt_pin);
    void Initialize();

    /* 设置角度：pan 0~180 (90=正前方), tilt 0~180 (90=水平) */
    void SetPan(int angle);
    void SetTilt(int angle);
    void SetAngles(int pan, int tilt);

    /* 预设动作 */
    void LookCenter();     /* 正前方 */
    void LookLeft();       /* 向左看 */
    void LookRight();      /* 向右看 */
    void LookUp();         /* 抬头 */
    void LookDown();       /* 低头 */
    void Nod();            /* 点头 */
    void Shake();          /* 摇头 */

private:
    gpio_num_t pan_pin_;
    gpio_num_t tilt_pin_;

    /* 共享 TIMER1（和尾巴舵机相同的 50Hz timer） */
    static constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
    static constexpr ledc_channel_t kPanChannel  = LEDC_CHANNEL_2;
    static constexpr ledc_channel_t kTiltChannel = LEDC_CHANNEL_3;

    /* 14-bit, 50Hz: 0.5ms=410, 2.5ms=2048 */
    static constexpr int kDutyMin = 410;
    static constexpr int kDutyMax = 2048;

    int AngleToDuty(int angle);
    void SetServo(ledc_channel_t ch, int angle);
};
