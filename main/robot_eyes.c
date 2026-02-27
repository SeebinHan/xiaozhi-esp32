/**
 * @file robot_eyes.c
 * @brief LT168串口屏机器人眼睛控制驱动实现
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

// UART互斥锁，防止并发发送
static SemaphoreHandle_t uart_mutex = NULL;

// ============== 心情信息表 ==============
typedef struct {
    robot_expression_t expr;    // 对应的表情ID（左眼）
    const char *name;           // 心情名称
    uint16_t duration_ms;       // 播放时长(ms)
} mood_info_t;

static const mood_info_t mood_table[] = {
    { EXPR_LIGHTNING_L, "闪电", 5000 },   // 5秒
    { EXPR_WRONGED_L,   "委屈", 6000 },   // 6秒
    { EXPR_LAUGH_CRY_L, "笑哭", 5000 },   // 5秒
    { EXPR_DAZE_L,      "发呆", 7000 },   // 7秒
    { EXPR_SMIRK_L,     "坏笑", 8000 },   // 8秒
    { EXPR_CRY_L,       "哭泣", 5000 },   // 5秒
    { EXPR_LOVE_L,      "喜欢", 5000 },   // 5秒
    { EXPR_ANGRY_L,     "生气", 5000 },   // 5秒
    { EXPR_DIZZY_L,     "晕",   5000 },   // 5秒
};

// 眨眼参数
#define BLINK_EXPR          EXPR_BLINK_L
#define BLINK_DURATION_MS   2400    // 眨眼GIF播放时长

// 心情触发队列
static QueueHandle_t mood_queue = NULL;
static TaskHandle_t eyes_task_handle = NULL;

// Levetop协议地址定义
#define ADDR_PAGE_SWITCH    0x7000  // 页面切换
#define ADDR_BRIGHTNESS     0x7001  // 亮度控制
#define ADDR_PLAY_MUSIC     0x700A  // 播放音乐
#define ADDR_VOLUME         0x700B  // 音量控制

// 协议帧头
#define FRAME_HEADER_H      0x5A
#define FRAME_HEADER_L      0xA5

// 命令类型 (Levetop协议)
#define CMD_WRITE           0x10    // 写命令
#define CMD_READ            0x03    // 读命令

// ============== CRC16查表 (Modbus CRC16) ==============
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

/**
 * @brief 计算CRC16 (Modbus)
 */
static uint16_t calc_crc16(uint8_t *data, uint16_t len)
{
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

/**
 * @brief 构建Levetop协议数据包
 */
static void build_frame(uint8_t *frame, uint16_t addr, uint16_t data)
{
    uint16_t crc;

    frame[0] = FRAME_HEADER_H;      // 0x5A
    frame[1] = FRAME_HEADER_L;      // 0xA5
    frame[2] = 0x07;                // 长度
    frame[3] = CMD_WRITE;           // 0x10
    frame[4] = (addr >> 8) & 0xFF;
    frame[5] = addr & 0xFF;
    frame[6] = (data >> 8) & 0xFF;
    frame[7] = data & 0xFF;

    crc = calc_crc16(&frame[3], 5);
    frame[8] = crc & 0xFF;
    frame[9] = (crc >> 8) & 0xFF;
}

/**
 * @brief 发送Levetop协议数据包 (带互斥锁保护)
 */
static esp_err_t send_command(uint16_t addr, uint16_t data)
{
    uint8_t frame[10];
    build_frame(frame, addr, data);

    // 获取互斥锁（最多等待100ms）
    if (uart_mutex && xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int written = uart_write_bytes(EYES_UART_NUM, frame, sizeof(frame));
        xSemaphoreGive(uart_mutex);

        if (written < 0) {
            ESP_LOGE(TAG, "UART write failed");
            return ESP_FAIL;
        }
        ESP_LOGD(TAG, "发送: 地址=0x%04X 数据=0x%04X", addr, data);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "获取UART锁超时");
    return ESP_ERR_TIMEOUT;
}

esp_err_t robot_eyes_init(void)
{
    ESP_LOGI(TAG, "初始化机器人眼睛 UART%d (TX:%d, RX:%d)",
             EYES_UART_NUM, EYES_UART_TX_PIN, EYES_UART_RX_PIN);

    // 创建UART互斥锁
    if (uart_mutex == NULL) {
        uart_mutex = xSemaphoreCreateMutex();
        if (uart_mutex == NULL) {
            ESP_LOGE(TAG, "创建UART互斥锁失败");
            return ESP_FAIL;
        }
    }

    // UART配置
    uart_config_t uart_config = {
        .baud_rate = EYES_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动
    esp_err_t ret = uart_driver_install(EYES_UART_NUM, 256, 256, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(EYES_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(EYES_UART_NUM, EYES_UART_TX_PIN, EYES_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART引脚配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "机器人眼睛初始化成功!");
    return ESP_OK;
}

// GIF控制值
// V_start = 10 (0x000A) → 循环播放
// V_once  = 11 (0x000B) → 播放一次
// V_stop  = 100 (0x0064) → 停止播放
#define GIF_LOOP_VALUE      0x000A  // V_start - 循环播放
#define GIF_ONCE_VALUE      0x000B  // V_once  - 播放一次
#define GIF_STOP_VALUE      0x0064  // V_stop  - 停止

// 上一次的表情，避免重复发送
static robot_expression_t last_expression = 0xFF;

esp_err_t robot_eyes_set_expression(robot_expression_t expr)
{
    if (expr > EXPR_BLINK_R) {
        ESP_LOGW(TAG, "无效的表情ID: %d", expr);
        return ESP_ERR_INVALID_ARG;
    }

    // 避免重复发送相同表情
    if (expr == last_expression) {
        ESP_LOGD(TAG, "表情未变化，跳过发送");
        return ESP_OK;
    }

    ESP_LOGD(TAG, "切换表情: %s (页面%d)", robot_eyes_get_expr_name(expr), expr);

    // 批量发送两条命令（在一次互斥锁内完成）
    if (uart_mutex && xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t frames[20];  // 两个帧

        // 构建两个命令帧
        build_frame(&frames[0], expr, GIF_LOOP_VALUE);      // GIF控制
        build_frame(&frames[10], ADDR_PAGE_SWITCH, expr);   // 页面切换

        // 一次性发送两条命令
        int written = uart_write_bytes(EYES_UART_NUM, frames, sizeof(frames));
        xSemaphoreGive(uart_mutex);

        if (written < 0) {
            ESP_LOGE(TAG, "UART write failed");
            return ESP_FAIL;
        }

        last_expression = expr;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "获取UART锁超时");
    return ESP_ERR_TIMEOUT;
}

// ============== 眼睛控制任务 ==============

/**
 * @brief 眼睛控制任务 - 默认眨眼，响应心情触发
 * 优化：只在状态变化时发送命令，避免频繁通信
 */
static void eyes_control_task(void *arg)
{
    robot_mood_t triggered_mood;

    ESP_LOGI(TAG, "眼睛控制任务启动 - 默认眨眼模式");

    // 初始显示眨眼（只发送一次）
    robot_eyes_set_expression(BLINK_EXPR);

    while (1) {
        // 无限等待心情触发（不再定时发送眨眼命令）
        if (xQueueReceive(mood_queue, &triggered_mood, portMAX_DELAY) == pdTRUE) {
            // 收到心情触发
            if (triggered_mood < MOOD_COUNT) {
                const mood_info_t *mood = &mood_table[triggered_mood];

                ESP_LOGI(TAG, ">>> 触发心情: %s", mood->name);

                // 播放心情表情
                robot_eyes_set_expression(mood->expr);

                // 等待心情表情播放完成
                vTaskDelay(pdMS_TO_TICKS(mood->duration_ms));

                ESP_LOGI(TAG, "<<< 心情结束，恢复眨眼");

                // 恢复眨眼（只在心情结束时发送一次）
                robot_eyes_set_expression(BLINK_EXPR);
            }
        }
    }
}

esp_err_t robot_eyes_start(void)
{
    if (eyes_task_handle != NULL) {
        ESP_LOGW(TAG, "眼睛任务已在运行");
        return ESP_OK;
    }

    // 创建心情触发队列
    if (mood_queue == NULL) {
        mood_queue = xQueueCreate(4, sizeof(robot_mood_t));
        if (mood_queue == NULL) {
            ESP_LOGE(TAG, "创建心情队列失败");
            return ESP_FAIL;
        }
    }

    // 创建眼睛控制任务 - 固定到Core 0实现真正并行
    // Core 0: 协议核心(WiFi/BT)，适合低负载任务
    // Core 1: 应用核心(音频处理)
    // 优先级2：较低优先级，避免影响WiFi和LCD
    BaseType_t ret = xTaskCreatePinnedToCore(
        eyes_control_task,      // 任务函数
        "eyes_ctrl",            // 任务名
        2048,                   // 栈大小(减小)
        NULL,                   // 参数
        2,                      // 优先级(降低)
        &eyes_task_handle,      // 任务句柄
        0                       // 固定到Core 0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建眼睛任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "眼睛控制已启动!");
    return ESP_OK;
}

esp_err_t robot_eyes_trigger_mood(robot_mood_t mood)
{
    if (mood >= MOOD_COUNT) {
        ESP_LOGW(TAG, "无效的心情ID: %d", mood);
        return ESP_ERR_INVALID_ARG;
    }

    if (mood_queue == NULL) {
        ESP_LOGW(TAG, "眼睛任务未启动，请先调用 robot_eyes_start()");
        return ESP_ERR_INVALID_STATE;
    }

    // 发送心情到队列（不等待，如果队列满则丢弃）
    if (xQueueSend(mood_queue, &mood, 0) != pdTRUE) {
        ESP_LOGW(TAG, "心情队列已满，忽略触发");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t robot_eyes_trigger_random_mood(void)
{
    robot_mood_t random_mood = esp_random() % MOOD_COUNT;
    ESP_LOGI(TAG, "随机心情: %s", mood_table[random_mood].name);
    return robot_eyes_trigger_mood(random_mood);
}

esp_err_t robot_eyes_set_brightness(uint8_t brightness)
{
    ESP_LOGI(TAG, "设置亮度: %d", brightness);
    return send_command(ADDR_BRIGHTNESS, brightness);
}

esp_err_t robot_eyes_play_sound(uint8_t music_id)
{
    ESP_LOGI(TAG, "播放音效: %d", music_id);
    return send_command(ADDR_PLAY_MUSIC, music_id);
}

esp_err_t robot_eyes_set_volume(uint8_t volume)
{
    if (volume > 15) {
        volume = 15;
    }
    ESP_LOGI(TAG, "设置音量: %d", volume);
    return send_command(ADDR_VOLUME, volume);
}

const char* robot_eyes_get_expr_name(robot_expression_t expr)
{
    static const char* names[] = {
        "闪电-左", "闪电-右",
        "委屈-左", "委屈-右",
        "笑哭-左", "笑哭-右",
        "发呆-左", "发呆-右",
        "坏笑-左", "坏笑-右",
        "哭泣-左", "哭泣-右",
        "喜欢-左", "喜欢-右",
        "生气-左", "生气-右",
        "晕-左",   "晕-右",
        "眨眼-左", "眨眼-右",
    };

    if (expr <= EXPR_BLINK_R) {
        return names[expr];
    }
    return "未知";
}

const char* robot_eyes_get_mood_name(robot_mood_t mood)
{
    if (mood < MOOD_COUNT) {
        return mood_table[mood].name;
    }
    return "未知";
}
