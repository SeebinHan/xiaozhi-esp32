#include "presence_sensor.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

#define TAG "PresenceSensor"

// 检测到有人后的最短重触发间隔（防止频繁唤醒）
static constexpr int kRetriggerIntervalMs = 30000;  // 30 秒
// GPIO 采样间隔
static constexpr int kPollIntervalMs = 500;

PresenceSensor::PresenceSensor(gpio_num_t gpio) : gpio_(gpio) {}

void PresenceSensor::Initialize() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << gpio_);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;  // LD2410B OUT 低电平无人
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);

    xTaskCreate(PollTask, "presence_poll", 2048, this, 2, nullptr);
    ESP_LOGI(TAG, "Initialized on GPIO%d", gpio_);
}

void PresenceSensor::OnPresenceDetected(std::function<void()> callback) {
    on_detected_ = std::move(callback);
}

void PresenceSensor::PollTask(void* arg) {
    auto* self = static_cast<PresenceSensor*>(arg);
    bool last_present = false;
    TickType_t last_trigger_tick = 0;
    int log_counter = 0;
    static constexpr int kLogIntervalCount = 10;  // 10×500ms = 5秒打印一次

    while (true) {
        bool present = gpio_get_level(self->gpio_) == 1;

        if (++log_counter >= kLogIntervalCount) {
            log_counter = 0;
            ESP_LOGI(TAG, "GPIO%d: %s", self->gpio_, present ? "有人" : "无人");
        }

        // 上升沿（从无人→有人）且超过重触发间隔，才调用回调
        if (present && !last_present) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_trigger_tick) * portTICK_PERIOD_MS >= kRetriggerIntervalMs) {
                last_trigger_tick = now;
                ESP_LOGI(TAG, "Presence detected");
                if (self->on_detected_) {
                    self->on_detected_();
                }
            }
        }

        last_present = present;
        vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
    }
}
