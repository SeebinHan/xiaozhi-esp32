#pragma once
#include <driver/gpio.h>
#include <functional>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class PresenceSensor {
public:
    explicit PresenceSensor(gpio_num_t gpio);
    void Initialize();
    void OnPresenceDetected(std::function<void()> callback);
    void SetEnabled(bool enabled, uint32_t arm_delay_ms = 0);

private:
    gpio_num_t gpio_;
    std::function<void()> on_detected_;
    std::atomic<bool> enabled_{false};
    std::atomic<TickType_t> enabled_tick_{0};
    std::atomic<TickType_t> arm_delay_ticks_{0};
    TaskHandle_t event_task_handle_ = nullptr;

    static void EventTask(void* arg);
    static void IRAM_ATTR GpioIsrHandler(void* arg);
};
