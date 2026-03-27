#include "pca9685.h"

#include <esp_log.h>

static const char* TAG_PCA = "Pca9685";

/* PCA9685 寄存器定义 */
static constexpr uint8_t PCA_MODE1      = 0x00;
static constexpr uint8_t PCA_PRESCALE   = 0xFE;
static constexpr uint8_t PCA_LED0_ON_L  = 0x06;

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr) {}

void Pca9685::InitializeServoMode() {
    // 关闭休眠，重置 MODE1
    WriteRegister(PCA_MODE1, 0x00);  // 自动增址关闭，正常模式

    // 计算 prescale：freq = 25MHz / (4096 * (prescale+1))
    // 目标 freq = 50Hz → prescale ≈ 121
    uint8_t prescale = 121;

    // 进入休眠设置 prescale
    WriteRegister(PCA_MODE1, 0x10);         // SLEEP=1
    WriteRegister(PCA_PRESCALE, prescale);
    // 打开自动递增，退出休眠
    WriteRegister(PCA_MODE1, 0x20);         // AI=1, SLEEP=0
    ESP_LOGI(TAG_PCA, "PCA9685 initialized for 50Hz servo output, prescale=%u", prescale);
}

void Pca9685::SetServoPulse(uint8_t channel, float pulse_us) {
    if (channel >= 16) return;

    // 50Hz → 周期 20000us，12bit 0-4095
    float ticks_f = pulse_us / 20000.0f * 4096.0f;
    if (ticks_f < 0) ticks_f = 0;
    if (ticks_f > 4095) ticks_f = 4095;
    uint16_t ticks = static_cast<uint16_t>(ticks_f);

    uint8_t reg = static_cast<uint8_t>(PCA_LED0_ON_L + 4 * channel);
    uint8_t buf[5];
    buf[0] = reg;             // 起始寄存器地址
    buf[1] = 0x00;            // ON_L
    buf[2] = 0x00;            // ON_H
    buf[3] = ticks & 0xFF;    // OFF_L
    buf[4] = (ticks >> 8) & 0x0F; // OFF_H

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buf, sizeof(buf), 100));
}
