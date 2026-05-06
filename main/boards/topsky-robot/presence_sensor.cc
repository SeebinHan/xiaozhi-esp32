#include "presence_sensor.h"
#include "presence_sensor_logic.h"

#include <esp_check.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <functional>

#define TAG "PresenceSensor"

static constexpr uint32_t kPresenceNotifyBit = 0x01;
static constexpr uint32_t kLeaveResetMs = 3000;

PresenceSensor::PresenceSensor(gpio_num_t gpio) : gpio_(gpio) {}

void PresenceSensor::Initialize() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << gpio_);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    cfg.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    xTaskCreate(EventTask, "presence_evt", 2048, this, 2, &event_task_handle_);

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_, GpioIsrHandler, this));

    ESP_LOGI(TAG, "Initialized on GPIO%d", gpio_);
}

void PresenceSensor::OnPresenceDetected(std::function<void()> callback) {
    on_detected_ = std::move(callback);
}

void PresenceSensor::SetEnabled(bool enabled, uint32_t arm_delay_ms) {
    enabled_.store(enabled, std::memory_order_relaxed);
    enabled_tick_.store(enabled ? xTaskGetTickCount() : 0, std::memory_order_relaxed);
    arm_delay_ticks_.store(enabled ? pdMS_TO_TICKS(arm_delay_ms) : 0, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Presence sensor %s (arm_delay_ms=%lu)",
             enabled ? "enabled" : "disabled",
             static_cast<unsigned long>(enabled ? arm_delay_ms : 0));
    if (event_task_handle_ != nullptr) {
        xTaskNotify(event_task_handle_, kPresenceNotifyBit, eSetBits);
    }
}

void IRAM_ATTR PresenceSensor::GpioIsrHandler(void* arg) {
    auto* self = static_cast<PresenceSensor*>(arg);
    if (self == nullptr || self->event_task_handle_ == nullptr) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(self->event_task_handle_, kPresenceNotifyBit, eSetBits, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void PresenceSensor::EventTask(void* arg) {
    auto* self = static_cast<PresenceSensor*>(arg);
    PresenceDetectionState state{};
    state.last_present = gpio_get_level(self->gpio_) == 1;

    while (true) {
        TickType_t now = xTaskGetTickCount();
        TickType_t enabled_tick = self->enabled_tick_.load(std::memory_order_relaxed);
        TickType_t arm_delay_ticks = self->arm_delay_ticks_.load(std::memory_order_relaxed);
        bool enabled = self->enabled_.load(std::memory_order_relaxed);
        bool present_now = gpio_get_level(self->gpio_) == 1;

        TickType_t wait_ticks = portMAX_DELAY;
        if (enabled && present_now && !state.has_triggered_in_current_presence) {
            if (arm_delay_ticks == 0) {
                wait_ticks = 0;
            } else {
                TickType_t arm_elapsed = now - enabled_tick;
                wait_ticks = arm_elapsed >= arm_delay_ticks ? 0 : (arm_delay_ticks - arm_elapsed);
            }
        } else if (!present_now && state.has_triggered_in_current_presence && state.absent_since_tick != 0) {
            TickType_t leave_reset_ticks = pdMS_TO_TICKS(kLeaveResetMs);
            TickType_t absent_elapsed = now - state.absent_since_tick;
            wait_ticks = absent_elapsed >= leave_reset_ticks ? 0 : (leave_reset_ticks - absent_elapsed);
        }

        uint32_t notified = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notified, wait_ticks);

        bool present = gpio_get_level(self->gpio_) == 1;
        now = xTaskGetTickCount();
        enabled_tick = self->enabled_tick_.load(std::memory_order_relaxed);
        arm_delay_ticks = self->arm_delay_ticks_.load(std::memory_order_relaxed);
        enabled = self->enabled_.load(std::memory_order_relaxed);

        if (ShouldTriggerPresenceDetection(
                state,
                present,
                enabled,
                static_cast<uint32_t>(enabled_tick),
                static_cast<uint32_t>(arm_delay_ticks),
                pdMS_TO_TICKS(kLeaveResetMs),
                static_cast<uint32_t>(now))) {
            ESP_LOGI(TAG, "Presence detected");
            if (self->on_detected_) {
                self->on_detected_();
            }
        }
    }
}
