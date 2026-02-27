/**
 * @file robot_eyes_bridge.c
 * @brief 小智AI情绪 -> 机器人眼睛表情 桥接层实现
 *
 * 将小智AI的情绪字符串映射到LT168串口屏的表情
 */

#include "robot_eyes_bridge.h"
#include "robot_eyes.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RobotEyesBridge";

// 情绪->心情映射表
typedef struct {
    const char* emotion;    // 小智AI情绪名称
    robot_mood_t mood;      // 对应的眼睛心情
} emotion_map_t;

// 映射表: 小智AI情绪 -> 机器人眼睛心情
static const emotion_map_t emotion_map[] = {
    // 开心类 -> 笑哭
    { "happy",      MOOD_LAUGH_CRY },
    { "laughing",   MOOD_LAUGH_CRY },
    { "funny",      MOOD_LAUGH_CRY },

    // 喜爱类 -> 喜欢(爱心眼)
    { "loving",     MOOD_LOVE },
    { "delicious",  MOOD_LOVE },
    { "kissy",      MOOD_LOVE },

    // 悲伤类 -> 哭泣
    { "sad",        MOOD_CRY },
    { "crying",     MOOD_CRY },

    // 愤怒 -> 生气
    { "angry",      MOOD_ANGRY },

    // 惊讶类 -> 闪电
    { "surprised",  MOOD_LIGHTNING },
    { "shocked",    MOOD_LIGHTNING },

    // 困惑/思考类 -> 发呆
    { "confused",   MOOD_DAZE },
    { "thinking",   MOOD_DAZE },

    // 尴尬/委屈 -> 委屈
    { "embarrassed", MOOD_WRONGED },

    // 困倦类 -> 晕
    { "sleepy",     MOOD_DIZZY },
    { "relaxed",    MOOD_DIZZY },

    // 自信/傻笑/眨眼类 -> 坏笑
    { "confident",  MOOD_SMIRK },
    { "silly",      MOOD_SMIRK },
    { "winking",    MOOD_SMIRK },
    { "cool",       MOOD_SMIRK },

    // 结束标记
    { NULL,         MOOD_COUNT }
};

// 上一次的情绪，避免重复触发
static char last_emotion[32] = "";

esp_err_t robot_eyes_bridge_init(void)
{
    ESP_LOGI(TAG, "初始化机器人眼睛UART...");

    // 只初始化串口，不启动任务
    esp_err_t ret = robot_eyes_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "机器人眼睛UART初始化失败");
        return ret;
    }

    ESP_LOGI(TAG, "机器人眼睛UART初始化成功!");
    return ESP_OK;
}

esp_err_t robot_eyes_bridge_start(void)
{
    ESP_LOGI(TAG, "启动机器人眼睛控制任务...");

    // 系统就绪后启动眼睛控制任务(默认眨眼)
    esp_err_t ret = robot_eyes_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动眼睛任务失败");
        return ret;
    }

    ESP_LOGI(TAG, "机器人眼睛控制任务已启动!");
    return ESP_OK;
}

void robot_eyes_bridge_set_emotion(const char* emotion)
{
    if (emotion == NULL) {
        return;
    }

    // 如果是 neutral 或 idle，不触发心情(保持眨眼)
    if (strcmp(emotion, "neutral") == 0 || strcmp(emotion, "idle") == 0) {
        ESP_LOGD(TAG, "中性情绪，保持眨眼");
        last_emotion[0] = '\0';  // 清空，下次非中性可触发
        return;
    }

    // 避免连续重复触发相同情绪
    if (strcmp(emotion, last_emotion) == 0) {
        ESP_LOGD(TAG, "情绪未变化: %s", emotion);
        return;
    }

    // 查找映射
    for (int i = 0; emotion_map[i].emotion != NULL; i++) {
        if (strcmp(emotion, emotion_map[i].emotion) == 0) {
            ESP_LOGI(TAG, "情绪映射: %s -> %s",
                     emotion, robot_eyes_get_mood_name(emotion_map[i].mood));
            robot_eyes_trigger_mood(emotion_map[i].mood);

            // 记录当前情绪
            strncpy(last_emotion, emotion, sizeof(last_emotion) - 1);
            last_emotion[sizeof(last_emotion) - 1] = '\0';
            return;
        }
    }

    // 未知情绪，使用默认(坏笑)
    ESP_LOGW(TAG, "未知情绪: %s, 使用默认表情", emotion);
    robot_eyes_trigger_mood(MOOD_SMIRK);
    strncpy(last_emotion, emotion, sizeof(last_emotion) - 1);
}
