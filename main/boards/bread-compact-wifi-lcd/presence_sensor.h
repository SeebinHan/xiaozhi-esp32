#pragma once
#include <driver/gpio.h>
#include <functional>

// LD2410B 人体存在传感器
// 接线：OUT 引脚 → PRESENCE_SENSOR_GPIO
// 高电平 = 有人，低电平 = 无人
// 有人时调用回调，可用于触发主动对话
class PresenceSensor {
public:
    explicit PresenceSensor(gpio_num_t gpio);
    void Initialize();

    // 设置"有人检测到"回调（在 ISR 上下文外调用，通过任务中转）
    void OnPresenceDetected(std::function<void()> callback);

private:
    gpio_num_t gpio_;
    std::function<void()> on_detected_;

    static void PollTask(void* arg);
};
