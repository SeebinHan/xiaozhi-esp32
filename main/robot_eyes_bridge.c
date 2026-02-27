/**
 * @file robot_eyes_bridge.c
 * @brief AI emotion to robot eyes expression bridge
 *
 * Maps AI emotion strings to LT168 serial display expressions.
 */

#include "robot_eyes_bridge.h"
#include "robot_eyes.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RobotEyesBridge";

typedef struct {
    const char* emotion;
    robot_mood_t mood;
} emotion_map_t;

// clang-format off
static const emotion_map_t emotion_map[] = {
    // Happy -> laugh_cry
    { "happy",      MOOD_LAUGH_CRY },
    { "laughing",   MOOD_LAUGH_CRY },
    { "funny",      MOOD_LAUGH_CRY },

    // Love -> love (heart eyes)
    { "loving",     MOOD_LOVE },
    { "delicious",  MOOD_LOVE },
    { "kissy",      MOOD_LOVE },

    // Sad -> cry
    { "sad",        MOOD_CRY },
    { "crying",     MOOD_CRY },

    // Angry -> angry
    { "angry",      MOOD_ANGRY },

    // Surprised -> lightning
    { "surprised",  MOOD_LIGHTNING },
    { "shocked",    MOOD_LIGHTNING },

    // Confused -> daze
    { "confused",   MOOD_DAZE },
    { "thinking",   MOOD_DAZE },

    // Embarrassed -> wronged
    { "embarrassed", MOOD_WRONGED },

    // Sleepy -> dizzy
    { "sleepy",     MOOD_DIZZY },
    { "relaxed",    MOOD_DIZZY },

    // Confident/silly -> smirk
    { "confident",  MOOD_SMIRK },
    { "silly",      MOOD_SMIRK },
    { "winking",    MOOD_SMIRK },
    { "cool",       MOOD_SMIRK },

    { NULL,         MOOD_COUNT }
};
// clang-format on

static char last_emotion[32] = "";

esp_err_t robot_eyes_bridge_init(void) {
    esp_err_t ret = robot_eyes_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Robot eyes init failed");
        return ret;
    }
    return ESP_OK;
}

esp_err_t robot_eyes_bridge_start(void) {
    esp_err_t ret = robot_eyes_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start eyes task");
        return ret;
    }
    ESP_LOGI(TAG, "Robot eyes started");
    return ESP_OK;
}

void robot_eyes_bridge_set_emotion(const char* emotion) {
    if (emotion == NULL) {
        return;
    }

    if (strcmp(emotion, "neutral") == 0 || strcmp(emotion, "idle") == 0) {
        last_emotion[0] = '\0';
        return;
    }

    if (strcmp(emotion, last_emotion) == 0) {
        return;
    }

    for (int i = 0; emotion_map[i].emotion != NULL; i++) {
        if (strcmp(emotion, emotion_map[i].emotion) == 0) {
            ESP_LOGI(TAG, "Emotion: %s -> %s",
                     emotion, robot_eyes_get_mood_name(emotion_map[i].mood));
            robot_eyes_trigger_mood(emotion_map[i].mood);

            strncpy(last_emotion, emotion, sizeof(last_emotion) - 1);
            last_emotion[sizeof(last_emotion) - 1] = '\0';
            return;
        }
    }

    ESP_LOGW(TAG, "Unknown emotion: %s, using default", emotion);
    robot_eyes_trigger_mood(MOOD_SMIRK);
    strncpy(last_emotion, emotion, sizeof(last_emotion) - 1);
}
