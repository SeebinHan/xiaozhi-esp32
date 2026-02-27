/**
 * @file robot_eyes_bridge.h
 * @brief 小智AI情绪 -> 机器人眼睛表情 桥接层
 */

#ifndef ROBOT_EYES_BRIDGE_H
#define ROBOT_EYES_BRIDGE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化机器人眼睛桥接 (只初始化UART，不启动任务)
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_bridge_init(void);

/**
 * @brief 启动眼睛控制任务 (系统就绪后调用)
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_bridge_start(void);

/**
 * @brief 根据小智AI情绪触发眼睛表情
 * @param emotion 情绪字符串 (如 "happy", "sad", "angry" 等)
 */
void robot_eyes_bridge_set_emotion(const char* emotion);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_EYES_BRIDGE_H
