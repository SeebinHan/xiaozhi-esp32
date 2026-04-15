#include "cat_eye_display.h"

#include "assets.h"
#include "cat_eye_sequences.h"
#include "application.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <cstring>
#include <cstdlib>

#define TAG "CatEyeDisplay"

namespace {
static const char* const kFullSequenceNames[] = {
    "blink_natural",
    "look_left_pupil",
    "look_right_pupil",
    "look_up_down_pupil",
};

static const char* const kListeningSequenceNames[] = {
    "blink_natural",
    "look_left_pupil",
    "look_right_pupil",
};
}

CatEyeDisplay::CatEyeDisplay() {
    width_ = EYE_WIDTH;
    height_ = EYE_HEIGHT;
}

CatEyeDisplay::~CatEyeDisplay() {
    if (blink_task_ != nullptr) {
        vTaskDelete(blink_task_);
    }
    if (frame_buf_ != nullptr) {
        heap_caps_free(frame_buf_);
    }
    if (render_mtx_ != nullptr) {
        vSemaphoreDelete(render_mtx_);
    }
}

bool CatEyeDisplay::Lock(int timeout_ms) {
    if (render_mtx_ == nullptr) {
        return true;
    }
    TickType_t ticks = timeout_ms <= 0 ? 0 : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(render_mtx_, ticks) == pdTRUE;
}

void CatEyeDisplay::Unlock() {
    if (render_mtx_ != nullptr) {
        xSemaphoreGive(render_mtx_);
    }
}

void CatEyeDisplay::LcdCmd(spi_device_handle_t spi, uint8_t cmd) {
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(EYE_PIN_DC, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

void CatEyeDisplay::LcdData(spi_device_handle_t spi, const uint8_t* data, int len) {
    if (len == 0) return;
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(EYE_PIN_DC, 1);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

void CatEyeDisplay::LcdDataByte(spi_device_handle_t spi, uint8_t val) {
    LcdData(spi, &val, 1);
}

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

void CatEyeDisplay::SetWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LcdCmd(spi, 0x2A);
    uint8_t col[] = {(uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1};
    LcdData(spi, col, 4);
    LcdCmd(spi, 0x2B);
    uint8_t row[] = {(uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1};
    LcdData(spi, row, 4);
    LcdCmd(spi, 0x2C);
}

void CatEyeDisplay::ShowSequenceFrame(spi_device_handle_t spi, const uint8_t* frame) {
    SetWindow(spi, 0, 0, EYE_WIDTH - 1, EYE_HEIGHT - 1);
    gpio_set_level(EYE_PIN_DC, 1);

    constexpr size_t kLineBytes = EYE_WIDTH * 2;
    spi_transaction_t transaction = {};

    for (int y = 0; y < EYE_HEIGHT; y += EYE_SPI_CHUNK_LINES) {
        int lines = (y + EYE_SPI_CHUNK_LINES <= EYE_HEIGHT) ? EYE_SPI_CHUNK_LINES : (EYE_HEIGHT - y);
        size_t bytes = kLineBytes * static_cast<size_t>(lines);
        memcpy(frame_buf_, frame + y * kLineBytes, bytes);

        transaction = {};
        transaction.length = static_cast<uint32_t>(bytes * 8);
        transaction.tx_buffer = frame_buf_;
        esp_err_t err = spi_device_polling_transmit(spi, &transaction);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Frame transmit failed: y=%d lines=%d err=%s", y, lines, esp_err_to_name(err));
            return;
        }
    }
}

bool CatEyeDisplay::ShowAssetFrameOnBothEyes(const char* frame_name) {
    if (frame_name == nullptr || frame_name[0] == '\0') {
        return false;
    }

    Assets& assets = Assets::GetInstance();
    void* frame_ptr = nullptr;
    size_t frame_size = 0;
    if (!assets.GetAssetData(frame_name, frame_ptr, frame_size) ||
        frame_size < static_cast<size_t>(EYE_WIDTH * EYE_HEIGHT * 2)) {
        ESP_LOGW(TAG, "Missing robot eye frame: %s", frame_name);
        return false;
    }

    if (!Lock(30000)) {
        ESP_LOGW(TAG, "Display lock timeout before asset frame: %s", frame_name);
        return false;
    }

    const uint8_t* frame = static_cast<const uint8_t*>(frame_ptr);
    ShowSequenceFrame(spi_left_, frame);
    ShowSequenceFrame(spi_right_, frame);
    Unlock();
    return true;
}

CatEyeDisplay::AnimationBudget CatEyeDisplay::GetAnimationBudget() const {
    auto state = Application::GetInstance().GetDeviceState();
    switch (state) {
        case kDeviceStateIdle:
            return AnimationBudget::kFull;
        case kDeviceStateListening:
        case kDeviceStateSpeaking:
            return AnimationBudget::kLight;
        default:
            return AnimationBudget::kMinimal;
    }
}

const AnimationSequence* CatEyeDisplay::PickSequenceForBudget(AnimationBudget budget) const {
    auto state = Application::GetInstance().GetDeviceState();

    if (!current_emotion_.empty() && current_emotion_ != "neutral" && state == kDeviceStateIdle) {
        if (const auto* mapped = FindAnimationSequence(current_emotion_)) {
            return mapped;
        }
    }

    if (state == kDeviceStateSpeaking) {
        int pick = rand() % 100;
        if (pick < 70) {
            return FindAnimationSequence("blink_natural");
        }
        return FindAnimationSequence("look_up_down_pupil");
    }

    if (state == kDeviceStateListening) {
        int pick = rand() % 100;
        if (pick < 50) {
            return FindAnimationSequence("blink_natural");
        }
        return FindAnimationSequence(kListeningSequenceNames[1 + (rand() % 2)]);
    }

    if (budget == AnimationBudget::kMinimal) {
        return FindAnimationSequence("blink_natural");
    }

    int pick = rand() % 100;
    if (pick < 40) {
        return FindAnimationSequence("blink_natural");
    }
    return FindAnimationSequence(kFullSequenceNames[1 + (rand() % 3)]);
}

void CatEyeDisplay::ResetActiveSequence() {
    active_sequence_ = nullptr;
    active_frame_index_ = 0;
    pending_delay_ms_ = 0;
    sequence_started_ = false;
}

bool CatEyeDisplay::ShowSequenceStep() {
    if (active_sequence_ == nullptr) {
        return false;
    }

    if (active_frame_index_ >= active_sequence_->frame_count) {
        ResetActiveSequence();
        return false;
    }

    const char* frame_name = active_sequence_->frame_names[active_frame_index_];
    if (frame_name == nullptr || frame_name[0] == '\0') {
        ResetActiveSequence();
        return false;
    }

    Assets& assets = Assets::GetInstance();
    void* frame_ptr = nullptr;
    size_t frame_size = 0;
    if (!assets.GetAssetData(frame_name, frame_ptr, frame_size) ||
        frame_size < static_cast<size_t>(EYE_WIDTH * EYE_HEIGHT * 2)) {
        ESP_LOGW(TAG, "Missing robot eye frame: %s", frame_name);
        ResetActiveSequence();
        return false;
    }

    if (!Lock(30000)) {
        ESP_LOGW(TAG, "Display lock timeout before frame: %s", frame_name);
        return false;
    }

    const uint8_t* frame = static_cast<const uint8_t*>(frame_ptr);
    ShowSequenceFrame(spi_left_, frame);
    ShowSequenceFrame(spi_right_, frame);
    Unlock();

    constexpr int kBaseFrameDelayMs = 120;
    int delay_ms = 0;
    if (!sequence_started_ && active_sequence_->lead_hold_frames > 0) {
        delay_ms += active_sequence_->lead_hold_frames * kBaseFrameDelayMs;
    }
    if (active_frame_index_ + 1 < active_sequence_->frame_count) {
        delay_ms += kBaseFrameDelayMs;
    } else {
        if (active_sequence_->tail_hold_frames > 0) {
            delay_ms += active_sequence_->tail_hold_frames * kBaseFrameDelayMs;
        }
        if (active_sequence_->hold_ms > 0) {
            delay_ms += active_sequence_->hold_ms;
        }
    }

    sequence_started_ = true;
    active_frame_index_++;
    pending_delay_ms_ = delay_ms;

    if (active_frame_index_ >= active_sequence_->frame_count) {
        active_sequence_ = nullptr;
        active_frame_index_ = 0;
        sequence_started_ = false;
    }

    return true;
}

void CatEyeDisplay::PlayIdleSequence() {
    AnimationBudget budget = GetAnimationBudget();

    if (active_sequence_ == nullptr) {
        active_sequence_ = PickSequenceForBudget(budget);
        active_frame_index_ = 0;
        pending_delay_ms_ = 0;
        sequence_started_ = false;
    }

    ShowSequenceStep();
}

void CatEyeDisplay::BlinkTaskEntry(void* arg) {
    auto* self = static_cast<CatEyeDisplay*>(arg);
    vTaskDelay(pdMS_TO_TICKS(2000));
    self->ShowAssetFrameOnBothEyes("blink_natural_00.rgb565");

    while (true) {
        int delay_ms = self->pending_delay_ms_;
        if (delay_ms <= 0) {
            switch (Application::GetInstance().GetDeviceState()) {
                case kDeviceStateSpeaking:
                    delay_ms = 1200 + (rand() % 1200);
                    break;
                case kDeviceStateListening:
                    delay_ms = 1600 + (rand() % 1600);
                    break;
                case kDeviceStateIdle:
                    delay_ms = 2500 + (rand() % 2500);
                    break;
                default:
                    delay_ms = 3200 + (rand() % 2400);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        self->pending_delay_ms_ = 0;
        self->PlayIdleSequence();
    }
}

void CatEyeDisplay::Initialize() {
    ESP_LOGI(TAG, "Initializing cat eye displays...");

    const size_t chunk_bytes = static_cast<size_t>(EYE_WIDTH * EYE_SPI_CHUNK_LINES * 2);
    frame_buf_ = static_cast<uint16_t*>(heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA));
    if (frame_buf_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return;
    }

    render_mtx_ = xSemaphoreCreateMutex();
    ResetActiveSequence();

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << EYE_PIN_DC) | (1ULL << EYE_PIN_RST);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level(EYE_PIN_DC, 0);
    gpio_set_level(EYE_PIN_RST, 1);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = EYE_PIN_MOSI;
    bus_cfg.miso_io_num = -1;
    bus_cfg.sclk_io_num = EYE_PIN_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = static_cast<int>(chunk_bytes + 8);
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = EYE_SPI_CLK_HZ;
    dev_cfg.mode = 0;
    dev_cfg.queue_size = 1;

    dev_cfg.spics_io_num = EYE_PIN_CS_L;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_left_));
    dev_cfg.spics_io_num = EYE_PIN_CS_R;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_right_));

    gpio_set_level(EYE_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(EYE_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    Gc9d01Init(spi_left_);
    Gc9d01Init(spi_right_);
}

void CatEyeDisplay::SetupUI() {
    Display::SetupUI();
    if (blink_task_ == nullptr) {
        xTaskCreatePinnedToCore(BlinkTaskEntry, "cat_eye_blink", 4096, this, 1, &blink_task_, 1);
    }
}

void CatEyeDisplay::SetEmotion(const char* emotion) {
    current_emotion_ = emotion == nullptr ? "neutral" : emotion;
}

void CatEyeDisplay::SetStatus(const char* status) {
    (void)status;
}

void CatEyeDisplay::ShowNotification(const char* notification, int duration_ms) {
    (void)notification;
    (void)duration_ms;
}

void CatEyeDisplay::SetChatMessage(const char* role, const char* content) {
    (void)role;
    (void)content;
}

void CatEyeDisplay::UpdateStatusBar(bool update_all) {
    (void)update_all;
}
