/**
 * @file robot_eyes.h
 * @brief LT168串口屏机器人眼睛控制驱动
 *
 * 基于168A AI宠物眼睛项目的Levetop协议
 */

#ifndef ROBOT_EYES_H
#define ROBOT_EYES_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============== 引脚配置 ==============
#define EYES_UART_NUM       UART_NUM_1
#define EYES_UART_TX_PIN    CONFIG_ROBOT_EYES_UART_TX_PIN
#define EYES_UART_RX_PIN    UART_PIN_NO_CHANGE
#define EYES_UART_BAUDRATE  CONFIG_ROBOT_EYES_UART_BAUDRATE

// ============== 表情定义 ==============
typedef enum {
    EXPR_LIGHTNING_L = 0,   // 闪电 (左眼)
    EXPR_LIGHTNING_R = 1,   // 闪电 (右眼)
    EXPR_WRONGED_L   = 2,   // 委屈 (左眼)
    EXPR_WRONGED_R   = 3,   // 委屈 (右眼)
    EXPR_LAUGH_CRY_L = 4,   // 笑哭 (左眼)
    EXPR_LAUGH_CRY_R = 5,   // 笑哭 (右眼)
    EXPR_DAZE_L      = 6,   // 发呆 (左眼)
    EXPR_DAZE_R      = 7,   // 发呆 (右眼)
    EXPR_SMIRK_L     = 8,   // 坏笑 (左眼)
    EXPR_SMIRK_R     = 9,   // 坏笑 (右眼)
    EXPR_CRY_L       = 10,  // 哭泣 (左眼)
    EXPR_CRY_R       = 11,  // 哭泣 (右眼)
    EXPR_LOVE_L      = 12,  // 喜欢 (左眼)
    EXPR_LOVE_R      = 13,  // 喜欢 (右眼)
    EXPR_ANGRY_L     = 14,  // 生气 (左眼)
    EXPR_ANGRY_R     = 15,  // 生气 (右眼)
    EXPR_DIZZY_L     = 16,  // 晕 (左眼)
    EXPR_DIZZY_R     = 17,  // 晕 (右眼)
    EXPR_BLINK_L     = 18,  // 眨眼 (左眼)
    EXPR_BLINK_R     = 19,  // 眨眼 (右眼)
} robot_expression_t;

// ============== 心情表情定义 (用于触发) ==============
typedef enum {
    MOOD_LIGHTNING = 0,  // 闪电
    MOOD_WRONGED,        // 委屈
    MOOD_LAUGH_CRY,      // 笑哭
    MOOD_DAZE,           // 发呆
    MOOD_SMIRK,          // 坏笑
    MOOD_CRY,            // 哭泣
    MOOD_LOVE,           // 喜欢
    MOOD_ANGRY,          // 生气
    MOOD_DIZZY,          // 晕
    MOOD_COUNT           // 心情数量
} robot_mood_t;

// ============== API函数 ==============

/**
 * @brief 初始化机器人眼睛UART
 * @return ESP_OK成功，其他失败
 */
esp_err_t robot_eyes_init(void);

/**
 * @brief 启动眼睛任务 (默认眨眼状态)
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_start(void);

/**
 * @brief 触发心情表情 (播放完后自动返回眨眼)
 * @param mood 心情ID
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_trigger_mood(robot_mood_t mood);

/**
 * @brief 触发随机心情表情
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_trigger_random_mood(void);

/**
 * @brief 设置眼睛表情 (底层API)
 * @param expr 表情ID (参考robot_expression_t)
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_set_expression(robot_expression_t expr);

/**
 * @brief 设置屏幕亮度
 * @param brightness 亮度值 0-255
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_set_brightness(uint8_t brightness);

/**
 * @brief 播放音效
 * @param music_id 音乐ID
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_play_sound(uint8_t music_id);

/**
 * @brief 设置音量
 * @param volume 音量 0-15
 * @return ESP_OK成功
 */
esp_err_t robot_eyes_set_volume(uint8_t volume);

/**
 * @brief 获取表情名称字符串
 * @param expr 表情ID
 * @return 表情名称
 */
const char* robot_eyes_get_expr_name(robot_expression_t expr);

/**
 * @brief 获取心情名称字符串
 * @param mood 心情ID
 * @return 心情名称
 */
const char* robot_eyes_get_mood_name(robot_mood_t mood);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_EYES_H
