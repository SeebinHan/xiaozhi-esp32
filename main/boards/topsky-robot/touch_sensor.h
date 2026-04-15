#pragma once

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>

#include "companion/touch_coordinator.h"

class TouchSensor {
public:
    TouchSensor(gpio_num_t adc_pin, adc_channel_t channel);
    ~TouchSensor();

    void Initialize();
    TouchLevel ReadLevel();
    TouchLevel ReadRawAndLevel(int& out_raw);
    int ReadRaw();

private:
    gpio_num_t pin_;
    adc_channel_t channel_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;

    static constexpr int kIdleMin = 3600;
    static constexpr int kThresholdLight = 3000;
    static constexpr int kThresholdHard = 1600;

    TouchLevel RawToLevel(int raw);
};
