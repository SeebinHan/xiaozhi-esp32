#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <string>

#include "display/display.h"

#define EYE_PIN_MOSI    GPIO_NUM_1
#define EYE_PIN_SCLK    GPIO_NUM_2
#define EYE_PIN_CS_L    GPIO_NUM_3
#define EYE_PIN_CS_R    GPIO_NUM_8
#define EYE_PIN_DC      GPIO_NUM_13
#define EYE_PIN_RST     GPIO_NUM_14

#define EYE_WIDTH       160
#define EYE_HEIGHT      160
#define EYE_SPI_CLK_HZ  (60 * 1000 * 1000)
#define EYE_SPI_CHUNK_LINES 20
#define EYE_SPI_QUEUE_DEPTH 2

struct AnimationSequence;

class CatEyeDisplay : public Display {
public:
    CatEyeDisplay();
    ~CatEyeDisplay() override;

    void Initialize();
    void SetupUI() override;
    void SetEmotion(const char* emotion) override;
    void SetStatus(const char* status) override;
    void ShowNotification(const char* notification, int duration_ms = 3000) override;
    void SetChatMessage(const char* role, const char* content) override;
    void UpdateStatusBar(bool update_all = false) override;

protected:
    bool Lock(int timeout_ms = 0) override;
    void Unlock() override;

private:
    enum class AnimationBudget {
        kFull = 0,
        kLight,
        kMinimal,
    };

    enum class IdleAnchorFrame {
        kNeutral = 0,
        kLookLeft,
        kLookRight,
    };

    spi_device_handle_t spi_left_ = nullptr;
    spi_device_handle_t spi_right_ = nullptr;
    SemaphoreHandle_t render_mtx_ = nullptr;
    TaskHandle_t blink_task_ = nullptr;
    uint16_t* frame_buf_ = nullptr;
    std::string current_emotion_ = "neutral";
    const AnimationSequence* active_sequence_ = nullptr;
    size_t active_frame_index_ = 0;
    int pending_delay_ms_ = 0;
    bool sequence_started_ = false;
    IdleAnchorFrame idle_anchor_frame_ = IdleAnchorFrame::kNeutral;

    void LcdCmd(spi_device_handle_t spi, uint8_t cmd);
    void LcdData(spi_device_handle_t spi, const uint8_t* data, int len);
    void LcdDataByte(spi_device_handle_t spi, uint8_t val);
    void Gc9d01Init(spi_device_handle_t spi);
    void SetWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void ShowSequenceFrame(spi_device_handle_t spi, const uint8_t* frame);
    bool ShowAssetFrameOnBothEyes(const char* frame_name);
    bool ShowSequenceStep();
    void PlayIdleSequence();
    AnimationBudget GetAnimationBudget() const;
    const AnimationSequence* PickSequenceForBudget(AnimationBudget budget) const;
    const char* PickMinimalFrame(AnimationBudget budget) const;
    void ResetActiveSequence();
    static void BlinkTaskEntry(void* arg);
};
