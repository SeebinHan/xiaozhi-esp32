#pragma once

#include <driver/i2c_master.h>
#include "i2c_device.h"

/* PCA9685 16通道PWM驱动（用于舵机）
 * - 通过 I2CDevice 封装
 * - 只暴露 50Hz 舵机控制相关接口
 */
class Pca9685 : public I2cDevice {
public:
    /* addr 一般为 0x40（A0~A5 全接地） */
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x40);

    /* 初始化为 50Hz，用于舵机控制 */
    void InitializeServoMode();

    /* 设置单个通道脉宽（微秒），典型范围 500~2500us */
    void SetServoPulse(uint8_t channel, float pulse_us);

private:
    /* 写 PCA9685 寄存器 */
    void WriteRegister(uint8_t reg, uint8_t value) { WriteReg(reg, value); }
};
