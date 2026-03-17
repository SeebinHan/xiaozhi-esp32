/*
 * 薄膜压力传感器驱动
 * 通过 ADC 读取模拟电压，映射为触摸压力等级
 */

#pragma once

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <string>
#include <functional>

/* 压力等级 */
enum class TouchLevel {
    kNone = 0,    /* 无触摸 */
    kLight,       /* 轻触 */
    kMedium,      /* 中等按压 */
    kHard,        /* 重按 */
};

class TouchSensor {
public:
    using TouchCallback = std::function<void(TouchLevel level)>;

    TouchSensor(gpio_num_t adc_pin, adc_channel_t channel);
    ~TouchSensor();

    void Initialize();
    void SetCallback(TouchCallback cb) { callback_ = cb; }

    /* 单次读取当前压力等级 */
    TouchLevel ReadLevel();
    /* 读取 raw 并同时返回对应 level（避免双重读取不一致） */
    TouchLevel ReadRawAndLevel(int& out_raw);
    int ReadRaw();

private:
    gpio_num_t pin_;
    adc_channel_t channel_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    TouchCallback callback_;

    /* ADC 阈值（反向逻辑：无压力=4095，压力越大值越低）
     * 实测 idle 噪声 3700-4080，按压时直接跳到 500-1300 */
    static constexpr int kIdleMin        = 3400;  /* 高于此值=无触摸（留足噪声余量） */
    static constexpr int kThresholdLight = 2500;  /* 轻触上限 */
    static constexpr int kThresholdHard  = 1200;  /* 重按上限（值更低=压力更大） */

    /* 将 raw 值映射为 level */
    TouchLevel RawToLevel(int raw);
};
