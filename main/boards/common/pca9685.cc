#include "pca9685.h"

#include <atomic>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
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

static std::atomic<uint32_t> g_pca_write_seq{0};
static std::atomic<uint32_t> g_pca_fail_count{0};
static std::atomic<uint32_t> g_pca_consecutive_failures{0};
static std::atomic<int64_t> g_pca_last_attempt_us{0};
static std::atomic<int64_t> g_pca_last_success_us{0};

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

bool Pca9685::SetServoPulse(uint8_t channel, float pulse_us) {
    if (channel >= 16) {
        return false;
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

    uint32_t seq = g_pca_write_seq.fetch_add(1) + 1;
    int64_t attempt_us = esp_timer_get_time();
    int64_t prev_attempt_us = g_pca_last_attempt_us.exchange(attempt_us);
    const char* task_name = pcTaskGetName(nullptr);
    if (task_name == nullptr) {
        task_name = "unknown";
    }

    esp_err_t err = i2c_master_transmit(i2c_device_, buf, sizeof(buf), 100);
    if (err == ESP_OK) {
        g_pca_last_success_us.store(attempt_us);
        g_pca_consecutive_failures.store(0);
        return true;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    int64_t retry_us = esp_timer_get_time();
    err = i2c_master_transmit(i2c_device_, buf, sizeof(buf), 100);
    if (err == ESP_OK) {
        g_pca_last_success_us.store(retry_us);
        g_pca_consecutive_failures.store(0);
        ESP_LOGW(TAG,
                 "SetServoPulse recovered after retry: seq=%lu channel=%u pulse_us=%.1f ticks=%u task=%s prev_attempt_delta_us=%lld retry_gap_us=%lld",
                 static_cast<unsigned long>(seq),
                 channel,
                 pulse_us,
                 ticks,
                 task_name,
                 static_cast<long long>(prev_attempt_us == 0 ? -1 : attempt_us - prev_attempt_us),
                 static_cast<long long>(retry_us - attempt_us));
        return true;
    }

    uint32_t fail_count = g_pca_fail_count.fetch_add(1) + 1;
    uint32_t consecutive_failures = g_pca_consecutive_failures.fetch_add(1) + 1;
    int64_t last_success_us = g_pca_last_success_us.load();
    ESP_LOGE(TAG,
             "SetServoPulse transmit failed: seq=%lu channel=%u pulse_us=%.1f ticks=%u err=0x%x(%s) task=%s prev_attempt_delta_us=%lld since_last_success_us=%lld fail_count=%lu consecutive_failures=%lu",
             static_cast<unsigned long>(seq),
             channel,
             pulse_us,
             ticks,
             err,
             esp_err_to_name(err),
             task_name,
             static_cast<long long>(prev_attempt_us == 0 ? -1 : attempt_us - prev_attempt_us),
             static_cast<long long>(last_success_us == 0 ? -1 : retry_us - last_success_us),
             static_cast<unsigned long>(fail_count),
             static_cast<unsigned long>(consecutive_failures));
    return false;
}
