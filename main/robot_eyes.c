/**
 * @file robot_eyes.c
 * @brief LT168 serial display robot eyes driver
 */

#include "robot_eyes.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "RobotEyes";

static SemaphoreHandle_t uart_mutex = NULL;

// Mood info table
typedef struct {
    robot_expression_t expr;
    const char *name;
    uint16_t duration_ms;
} mood_info_t;

static const mood_info_t mood_table[] = {
    { EXPR_LIGHTNING_L, "lightning", 5000 },
    { EXPR_WRONGED_L,   "wronged",  6000 },
    { EXPR_LAUGH_CRY_L, "laugh_cry", 5000 },
    { EXPR_DAZE_L,      "daze",     7000 },
    { EXPR_SMIRK_L,     "smirk",    8000 },
    { EXPR_CRY_L,       "cry",      5000 },
    { EXPR_LOVE_L,      "love",     5000 },
    { EXPR_ANGRY_L,     "angry",    5000 },
    { EXPR_DIZZY_L,     "dizzy",    5000 },
};

#define BLINK_EXPR          EXPR_BLINK_L
#define BLINK_DURATION_MS   2400

static QueueHandle_t mood_queue = NULL;
static TaskHandle_t eyes_task_handle = NULL;

// Levetop protocol address definitions
#define ADDR_PAGE_SWITCH    0x7000
#define ADDR_BRIGHTNESS     0x7001
#define ADDR_PLAY_MUSIC     0x700A
#define ADDR_VOLUME         0x700B

// Protocol frame header
#define FRAME_HEADER_H      0x5A
#define FRAME_HEADER_L      0xA5

// Command types
#define CMD_WRITE           0x10
#define CMD_READ            0x03

// CRC16 lookup table (Modbus CRC16)
static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

static const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

static uint16_t calc_crc16(uint8_t *data, uint16_t len) {
    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    uint16_t index;

    while (len--) {
        index = crc_lo ^ *data++;
        crc_lo = crc_hi ^ auchCRCHi[index];
        crc_hi = auchCRCLo[index];
    }

    return (crc_hi << 8) | crc_lo;
}

static void build_frame(uint8_t *frame, uint16_t addr, uint16_t data) {
    uint16_t crc;

    frame[0] = FRAME_HEADER_H;
    frame[1] = FRAME_HEADER_L;
    frame[2] = 0x07;
    frame[3] = CMD_WRITE;
    frame[4] = (addr >> 8) & 0xFF;
    frame[5] = addr & 0xFF;
    frame[6] = (data >> 8) & 0xFF;
    frame[7] = data & 0xFF;

    crc = calc_crc16(&frame[3], 5);
    frame[8] = crc & 0xFF;
    frame[9] = (crc >> 8) & 0xFF;
}

static esp_err_t send_command(uint16_t addr, uint16_t data) {
    uint8_t frame[10];
    build_frame(frame, addr, data);

    if (uart_mutex && xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int written = uart_write_bytes(EYES_UART_NUM, frame, sizeof(frame));
        xSemaphoreGive(uart_mutex);

        if (written < 0) {
            ESP_LOGE(TAG, "UART write failed");
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    ESP_LOGW(TAG, "UART mutex timeout");
    return ESP_ERR_TIMEOUT;
}

esp_err_t robot_eyes_init(void) {
    ESP_LOGI(TAG, "Init UART%d (TX:%d)", EYES_UART_NUM, EYES_UART_TX_PIN);

    if (uart_mutex == NULL) {
        uart_mutex = xSemaphoreCreateMutex();
        if (uart_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create UART mutex");
            return ESP_FAIL;
        }
    }

    uart_config_t uart_config = {
        .baud_rate = EYES_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(EYES_UART_NUM, 256, 256, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(EYES_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(EYES_UART_NUM, EYES_UART_TX_PIN, EYES_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Robot eyes initialized");
    return ESP_OK;
}

// GIF control values
#define GIF_LOOP_VALUE      0x000A  // V_start - loop play
#define GIF_ONCE_VALUE      0x000B  // V_once  - play once
#define GIF_STOP_VALUE      0x0064  // V_stop  - stop

static robot_expression_t last_expression = 0xFF;

esp_err_t robot_eyes_set_expression(robot_expression_t expr) {
    if (expr > EXPR_BLINK_R) {
        ESP_LOGW(TAG, "Invalid expression ID: %d", expr);
        return ESP_ERR_INVALID_ARG;
    }

    if (expr == last_expression) {
        return ESP_OK;
    }

    if (uart_mutex && xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t frames[20];

        build_frame(&frames[0], expr, GIF_LOOP_VALUE);
        build_frame(&frames[10], ADDR_PAGE_SWITCH, expr);

        int written = uart_write_bytes(EYES_UART_NUM, frames, sizeof(frames));
        xSemaphoreGive(uart_mutex);

        if (written < 0) {
            ESP_LOGE(TAG, "UART write failed");
            return ESP_FAIL;
        }

        last_expression = expr;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "UART mutex timeout");
    return ESP_ERR_TIMEOUT;
}

static void eyes_control_task(void *arg) {
    robot_mood_t triggered_mood;

    robot_eyes_set_expression(BLINK_EXPR);

    while (1) {
        if (xQueueReceive(mood_queue, &triggered_mood, portMAX_DELAY) == pdTRUE) {
            if (triggered_mood < MOOD_COUNT) {
                const mood_info_t *mood = &mood_table[triggered_mood];

                ESP_LOGI(TAG, "Mood: %s (%dms)", mood->name, mood->duration_ms);
                robot_eyes_set_expression(mood->expr);
                vTaskDelay(pdMS_TO_TICKS(mood->duration_ms));
                robot_eyes_set_expression(BLINK_EXPR);
            }
        }
    }
}

esp_err_t robot_eyes_start(void) {
    if (eyes_task_handle != NULL) {
        return ESP_OK;
    }

    if (mood_queue == NULL) {
        mood_queue = xQueueCreate(4, sizeof(robot_mood_t));
        if (mood_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create mood queue");
            return ESP_FAIL;
        }
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        eyes_control_task, "eyes_ctrl", 2048, NULL, 2, &eyes_task_handle, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create eyes task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Eyes control started");
    return ESP_OK;
}

esp_err_t robot_eyes_trigger_mood(robot_mood_t mood) {
    if (mood >= MOOD_COUNT) {
        ESP_LOGW(TAG, "Invalid mood ID: %d", mood);
        return ESP_ERR_INVALID_ARG;
    }

    if (mood_queue == NULL) {
        ESP_LOGW(TAG, "Eyes task not started");
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(mood_queue, &mood, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Mood queue full");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t robot_eyes_trigger_random_mood(void) {
    robot_mood_t random_mood = esp_random() % MOOD_COUNT;
    return robot_eyes_trigger_mood(random_mood);
}

esp_err_t robot_eyes_set_brightness(uint8_t brightness) {
    return send_command(ADDR_BRIGHTNESS, brightness);
}

esp_err_t robot_eyes_play_sound(uint8_t music_id) {
    return send_command(ADDR_PLAY_MUSIC, music_id);
}

esp_err_t robot_eyes_set_volume(uint8_t volume) {
    if (volume > 15) {
        volume = 15;
    }
    return send_command(ADDR_VOLUME, volume);
}

const char* robot_eyes_get_expr_name(robot_expression_t expr) {
    static const char* names[] = {
        "lightning-L", "lightning-R",
        "wronged-L",   "wronged-R",
        "laugh_cry-L", "laugh_cry-R",
        "daze-L",      "daze-R",
        "smirk-L",     "smirk-R",
        "cry-L",       "cry-R",
        "love-L",      "love-R",
        "angry-L",     "angry-R",
        "dizzy-L",     "dizzy-R",
        "blink-L",     "blink-R",
    };

    if (expr <= EXPR_BLINK_R) {
        return names[expr];
    }
    return "unknown";
}

const char* robot_eyes_get_mood_name(robot_mood_t mood) {
    if (mood < MOOD_COUNT) {
        return mood_table[mood].name;
    }
    return "unknown";
}
