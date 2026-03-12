/*
 * GC9D01 双眼圆形 LCD 驱动实现
 */

#include "cat_eye_display.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "CatEye";

CatEyeDisplay::CatEyeDisplay() {}

CatEyeDisplay::~CatEyeDisplay() {
    if (frame_buf_) {
        free(frame_buf_);
    }
}

/* ================================================================
 *  Low-level SPI
 * ================================================================ */
void CatEyeDisplay::LcdCmd(spi_device_handle_t spi, uint8_t cmd) {
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(EYE_PIN_DC, 0);
    spi_device_polling_transmit(spi, &t);
}

void CatEyeDisplay::LcdData(spi_device_handle_t spi, const uint8_t* data, int len) {
    if (len == 0) return;
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(EYE_PIN_DC, 1);
    spi_device_polling_transmit(spi, &t);
}

void CatEyeDisplay::LcdDataByte(spi_device_handle_t spi, uint8_t val) {
    LcdData(spi, &val, 1);
}

/* ================================================================
 *  GC9D01 init sequence (from vendor code)
 * ================================================================ */
void CatEyeDisplay::Gc9d01Init(spi_device_handle_t spi) {
    LcdCmd(spi, 0xFE);
    LcdCmd(spi, 0xEF);

    for (uint8_t reg = 0x80; reg <= 0x8F; reg++) {
        LcdCmd(spi, reg);
        LcdDataByte(spi, 0xFF);
    }

    LcdCmd(spi, 0x3A); LcdDataByte(spi, 0x05);
    LcdCmd(spi, 0xEC); LcdDataByte(spi, 0x01);

    LcdCmd(spi, 0x74);
    { uint8_t d[] = {0x02,0x0E,0x00,0x00,0x00,0x00,0x00}; LcdData(spi, d, sizeof(d)); }

    LcdCmd(spi, 0x98); LcdDataByte(spi, 0x3E);
    LcdCmd(spi, 0x99); LcdDataByte(spi, 0x3E);

    LcdCmd(spi, 0xB5); { uint8_t d[]={0x0D,0x0D}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x60); { uint8_t d[]={0x38,0x0F,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x61); { uint8_t d[]={0x38,0x11,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x64); { uint8_t d[]={0x38,0x17,0x71,0x5F,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x65); { uint8_t d[]={0x38,0x13,0x71,0x5B,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x6A); { uint8_t d[]={0x00,0x00}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x6C); { uint8_t d[]={0x22,0x02,0x22,0x02,0x22,0x22,0x50}; LcdData(spi,d,sizeof(d)); }

    LcdCmd(spi, 0x6E);
    { uint8_t d[]={0x03,0x03,0x01,0x01,0x00,0x00,0x0F,0x0F,
                   0x0D,0x0D,0x0B,0x0B,0x09,0x09,0x00,0x00,
                   0x00,0x00,0x0A,0x0A,0x0C,0x0C,0x0E,0x0E,
                   0x10,0x10,0x00,0x00,0x02,0x02,0x04,0x04};
      LcdData(spi, d, sizeof(d)); }

    LcdCmd(spi, 0xBF); LcdDataByte(spi, 0x01);
    LcdCmd(spi, 0xF9); LcdDataByte(spi, 0x40);
    LcdCmd(spi, 0x9B); LcdDataByte(spi, 0x3B);
    LcdCmd(spi, 0x93); { uint8_t d[]={0x33,0x7F,0x00}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x7E); LcdDataByte(spi, 0x30);
    LcdCmd(spi, 0x70); { uint8_t d[]={0x0D,0x02,0x08,0x0D,0x02,0x08}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x71); { uint8_t d[]={0x0D,0x02,0x08}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x91); { uint8_t d[]={0x0E,0x09}; LcdData(spi,d,sizeof(d)); }

    LcdCmd(spi, 0xC3); LcdDataByte(spi, 0x19);
    LcdCmd(spi, 0xC4); LcdDataByte(spi, 0x19);
    LcdCmd(spi, 0xC9); LcdDataByte(spi, 0x3C);

    LcdCmd(spi, 0xF0); { uint8_t d[]={0x53,0x15,0x0A,0x04,0x00,0x3E}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0xF2); { uint8_t d[]={0x53,0x15,0x0A,0x04,0x00,0x3A}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0xF1); { uint8_t d[]={0x56,0xA8,0x7F,0x33,0x34,0x5F}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0xF3); { uint8_t d[]={0x52,0xA4,0x7F,0x33,0x34,0xDF}; LcdData(spi,d,sizeof(d)); }

    LcdCmd(spi, 0x36); LcdDataByte(spi, 0x00);

    LcdCmd(spi, 0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    LcdCmd(spi, 0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
}

/* ================================================================
 *  Drawing helpers
 * ================================================================ */
void CatEyeDisplay::SetWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LcdCmd(spi, 0x2A);
    uint8_t col[] = {(uint8_t)(x0>>8),(uint8_t)x0,(uint8_t)(x1>>8),(uint8_t)x1};
    LcdData(spi, col, 4);
    LcdCmd(spi, 0x2B);
    uint8_t row[] = {(uint8_t)(y0>>8),(uint8_t)y0,(uint8_t)(y1>>8),(uint8_t)y1};
    LcdData(spi, row, 4);
    LcdCmd(spi, 0x2C);
}

void CatEyeDisplay::SendFrameBuffer(spi_device_handle_t spi) {
    SetWindow(spi, 0, 0, EYE_WIDTH - 1, EYE_HEIGHT - 1);
    gpio_set_level(EYE_PIN_DC, 1);
    /* Send in chunks to avoid DMA limits */
    const int chunk_lines = 10;
    for (int y = 0; y < EYE_HEIGHT; y += chunk_lines) {
        int lines = (y + chunk_lines <= EYE_HEIGHT) ? chunk_lines : (EYE_HEIGHT - y);
        spi_transaction_t t = {};
        t.length = EYE_WIDTH * lines * 16;
        t.tx_buffer = &frame_buf_[y * EYE_WIDTH];
        spi_device_polling_transmit(spi, &t);
    }
}

void CatEyeDisplay::FillColor(spi_device_handle_t spi, uint16_t color) {
    uint16_t sw = SwapBytes(color);
    for (int i = 0; i < EYE_WIDTH * EYE_HEIGHT; i++) {
        frame_buf_[i] = sw;
    }
    SendFrameBuffer(spi);
}

/* ================================================================
 *  Eye pattern generators
 * ================================================================ */
void CatEyeDisplay::DrawCatEye(uint16_t iris_r, uint16_t iris_g, uint16_t iris_b,
                                float pupil_openness, float look_x, float look_y) {
    const int cx = 80 + (int)(look_x * 15);
    const int cy = 80 + (int)(look_y * 15);
    const int r_eye = 70;
    const int r_iris = 55;
    const int pupil_hw = (int)(4 + pupil_openness * 4);  /* wider when open */
    const int pupil_hh = 50;
    const int hl_cx = cx - 20, hl_cy = cy - 25;
    const int hl_r = 12;

    for (int y = 0; y < EYE_HEIGHT; y++) {
        for (int x = 0; x < EYE_WIDTH; x++) {
            int dx = x - cx;
            int dy = y - cy;
            int dist2 = dx * dx + dy * dy;
            uint16_t color;

            if (dist2 > r_eye * r_eye) {
                color = Rgb565(0, 0, 0);
            } else if (dist2 > r_iris * r_iris) {
                float f = (float)(dist2 - r_iris * r_iris) / (float)(r_eye * r_eye - r_iris * r_iris);
                color = Rgb565((uint8_t)(40 + f * 20), (uint8_t)(25 + f * 15), (uint8_t)(5 + f * 10));
            } else {
                int hdx = x - hl_cx;
                int hdy = y - hl_cy;
                if (hdx * hdx + hdy * hdy < hl_r * hl_r) {
                    color = Rgb565(255, 255, 255);
                } else if (abs(dx) < pupil_hw && abs(dy) < pupil_hh) {
                    color = Rgb565(0, 0, 0);
                } else {
                    float f = (float)dist2 / (float)(r_iris * r_iris);
                    uint8_t r = (uint8_t)(iris_r - f * (iris_r * 0.4f));
                    uint8_t g = (uint8_t)(iris_g - f * (iris_g * 0.3f));
                    uint8_t b = (uint8_t)(iris_b + f * 20);
                    color = Rgb565(r, g, b);
                }
            }
            frame_buf_[y * EYE_WIDTH + x] = SwapBytes(color);
        }
    }
}

void CatEyeDisplay::DrawClosedEye() {
    /* Black background with a curved line for closed eyelid */
    for (int y = 0; y < EYE_HEIGHT; y++) {
        for (int x = 0; x < EYE_WIDTH; x++) {
            int dx = x - 80;
            int dy = y - 80;
            /* Closed eye: curved downward (happy smile) */
            float curve_y = 80.0f - 20.0f * cosf((float)dx * 3.14159f / 70.0f);
            float dist_to_curve = fabsf((float)y - curve_y);
            uint16_t color;
            if (dx * dx + dy * dy > 70 * 70) {
                color = Rgb565(0, 0, 0);
            } else if (dist_to_curve < 3.0f && abs(dx) < 60) {
                color = Rgb565(180, 200, 20);
            } else {
                color = Rgb565(0, 0, 0);
            }
            frame_buf_[y * EYE_WIDTH + x] = SwapBytes(color);
        }
    }
}

void CatEyeDisplay::DrawHeartEye() {
    for (int y = 0; y < EYE_HEIGHT; y++) {
        for (int x = 0; x < EYE_WIDTH; x++) {
            /* Heart shape formula */
            float fx = (float)(x - 80) / 40.0f;
            float fy = (float)(y - 90) / -40.0f;
            float heart = (fx * fx + fy * fy - 1.0f);
            heart = heart * heart * heart - fx * fx * fy * fy * fy;
            uint16_t color;
            if (heart <= 0.0f) {
                color = Rgb565(255, 30, 60);
            } else if (heart < 0.1f) {
                color = Rgb565(200, 20, 40);
            } else {
                color = Rgb565(0, 0, 0);
            }
            frame_buf_[y * EYE_WIDTH + x] = SwapBytes(color);
        }
    }
}

/* ================================================================
 *  Initialize
 * ================================================================ */
void CatEyeDisplay::Initialize() {
    ESP_LOGI(TAG, "Initializing cat eye displays...");

    /* Allocate frame buffer */
    frame_buf_ = (uint16_t*)heap_caps_malloc(EYE_WIDTH * EYE_HEIGHT * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!frame_buf_) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer!");
        return;
    }

    /* Configure DC and RST GPIO */
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << EYE_PIN_DC) | (1ULL << EYE_PIN_RST);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level(EYE_PIN_DC, 0);
    gpio_set_level(EYE_PIN_RST, 1);

    /* Initialize SPI2 bus */
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = EYE_PIN_MOSI;
    bus_cfg.miso_io_num = -1;
    bus_cfg.sclk_io_num = EYE_PIN_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = EYE_WIDTH * 10 * 2 + 8;
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI2 bus init failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    /* Add left eye */
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = EYE_SPI_CLK_HZ;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = EYE_PIN_CS_L;
    dev_cfg.queue_size = 1;
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_left_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Left eye device add failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    /* Add right eye */
    dev_cfg.spics_io_num = EYE_PIN_CS_R;
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_right_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Right eye device add failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    /* Hardware reset */
    gpio_set_level(EYE_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(EYE_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Init both displays */
    Gc9d01Init(spi_left_);
    Gc9d01Init(spi_right_);

    /* Show default neutral cat eye */
    SetEmotion("neutral");

    ESP_LOGI(TAG, "Cat eye displays ready!");
}

/* ================================================================
 *  SetEmotion - map emotion string to eye pattern
 * ================================================================ */
void CatEyeDisplay::SetEmotion(const std::string& emotion) {
    if (emotion == current_emotion_) return;
    current_emotion_ = emotion;

    ESP_LOGI(TAG, "Eye emotion: %s", emotion.c_str());

    if (emotion == "happy" || emotion == "laughing" || emotion == "funny") {
        /* Happy: closed/squinting eyes */
        DrawClosedEye();
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "love" || emotion == "heart_eyes") {
        /* Love: heart-shaped eyes */
        DrawHeartEye();
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "angry" || emotion == "hateful") {
        /* Angry: red iris, narrow pupil */
        DrawCatEye(220, 60, 20, 0.2f, 0, 0);
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "sad" || emotion == "crying") {
        /* Sad: blue-ish iris, looking down */
        DrawCatEye(100, 140, 200, 0.8f, 0, 0.3f);
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "surprised" || emotion == "shocked") {
        /* Surprised: wide pupil, big iris */
        DrawCatEye(180, 200, 20, 1.0f, 0, 0);
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "sleepy" || emotion == "tired") {
        /* Sleepy: half-closed */
        DrawClosedEye();
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "thinking" || emotion == "confused") {
        /* Thinking: looking up-right */
        DrawCatEye(180, 200, 20, 0.6f, 0.5f, -0.5f);
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);

    } else if (emotion == "wink") {
        /* Wink: left eye closed, right eye open */
        DrawClosedEye();
        SendFrameBuffer(spi_left_);
        DrawCatEye(180, 200, 20, 0.6f, 0, 0);
        SendFrameBuffer(spi_right_);

    } else {
        /* Default neutral: standard cat eye */
        DrawCatEye(180, 200, 20, 0.6f, 0, 0);
        SendFrameBuffer(spi_left_);
        SendFrameBuffer(spi_right_);
    }
}
