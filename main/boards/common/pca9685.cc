#include "pca9685.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "Pca9685"

static constexpr uint8_t PCA_MODE1 = 0x00;
static constexpr uint8_t PCA_MODE2 = 0x01;
static constexpr uint8_t PCA_PRESCALE = 0xFE;
static constexpr uint8_t PCA_LED0_ON_L = 0x06;

static constexpr uint8_t MODE1_RESTART = 0x80;
static constexpr uint8_t MODE1_SLEEP = 0x10;
static constexpr uint8_t MODE1_AI = 0x20;
static constexpr uint8_t MODE2_OUTDRV = 0x04;

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr) {}

void Pca9685::InitializeServoMode() {
    uint8_t prescale = 121;
    WriteRegister(PCA_MODE2, MODE2_OUTDRV);
    WriteRegister(PCA_MODE1, MODE1_SLEEP);
    WriteRegister(PCA_PRESCALE, prescale);
    WriteRegister(PCA_MODE1, MODE1_AI);
    vTaskDelay(pdMS_TO_TICKS(5));
    WriteRegister(PCA_MODE1, MODE1_AI | MODE1_RESTART);
    ESP_LOGI(TAG, "PCA9685 initialized for 50Hz servo output, prescale=%u", prescale);
}

void Pca9685::SetServoPulse(uint8_t channel, float pulse_us) {
    if (channel >= 16) {
        return;
    }

    float ticks_f = pulse_us / 20000.0f * 4096.0f;
    if (ticks_f < 0) ticks_f = 0;
    if (ticks_f > 4095) ticks_f = 4095;
    uint16_t ticks = static_cast<uint16_t>(ticks_f);

    uint8_t reg = static_cast<uint8_t>(PCA_LED0_ON_L + 4 * channel);
    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = ticks & 0xFF;
    buf[4] = (ticks >> 8) & 0x0F;

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buf, sizeof(buf), 100));
}
