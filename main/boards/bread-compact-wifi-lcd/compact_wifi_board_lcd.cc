#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/led.h"
#include "cat_eye_display.h"
#include "touch_sensor.h"
#include "tail_servo.h"
#include "head_gimbal.h"
#include "esp32_camera.h"

#include <esp_log.h>

#define TAG "CompactWifiBoardLCD"

/* 无主 LCD 的显示包装：通过 NoDisplay 提供空显示，同时在 SetEmotion 时
   联动猫眼、尾巴、头部 */
class CatDisplayWithPeripherals : public NoDisplay {
private:
    CatEyeDisplay* cat_eyes_;
    TailServo* tail_;
    HeadGimbal* head_;
public:
    CatDisplayWithPeripherals(CatEyeDisplay* eyes, TailServo* tail, HeadGimbal* head)
        : cat_eyes_(eyes), tail_(tail), head_(head) {}

    void SetEmotion(const char* emotion) override {
        NoDisplay::SetEmotion(emotion);
        if (cat_eyes_) {
            cat_eyes_->SetEmotion(emotion);
        }
        /* 尾巴随表情联动 */
        if (tail_) {
            std::string emo(emotion);
            if (emo == "happy" || emo == "laughing" || emo == "funny") {
                tail_->Wag();
            } else if (emo == "love" || emo == "heart_eyes") {
                tail_->WagSlow();
            } else if (emo == "sad" || emo == "crying") {
                tail_->Droop();
            } else if (emo == "surprised" || emo == "angry") {
                tail_->Perk();
            } else {
                tail_->SetAngle(90);
            }
        }
        /* 头部随表情联动 */
        if (head_) {
            std::string emo(emotion);
            if (emo == "happy" || emo == "laughing") {
                head_->Nod();
            } else if (emo == "sad" || emo == "crying") {
                head_->LookDown();
            } else if (emo == "thinking" || emo == "confused") {
                head_->LookUp();
            } else if (emo == "angry" || emo == "hateful") {
                head_->Shake();
            } else {
                head_->LookCenter();
            }
        }
    }
};

class CompactWifiBoardLCD : public WifiBoard {
private:

    Button boot_button_;
    Display* display_;
    CatEyeDisplay* cat_eyes_ = nullptr;
    TouchSensor* touch_sensor_ = nullptr;
    TailServo* tail_servo_ = nullptr;
    HeadGimbal* head_gimbal_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    bool touch_override_ = false;  /* 触摸时临时覆盖眼睛表情 */

    void InitializeEyeDisplays() {
        cat_eyes_ = new CatEyeDisplay();
        cat_eyes_->Initialize();
    }

    void InitializeTailServo() {
        tail_servo_ = new TailServo(TAIL_SERVO_GPIO);
        tail_servo_->Initialize();
    }

    void InitializeHeadGimbal() {
        head_gimbal_ = new HeadGimbal(HEAD_PAN_GPIO, HEAD_TILT_GPIO);
        head_gimbal_->Initialize();
    }

    void InitializeDisplay() {
        display_ = new CatDisplayWithPeripherals(cat_eyes_, tail_servo_, head_gimbal_);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RST;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTouchSensor() {
        touch_sensor_ = new TouchSensor(TOUCH_SENSOR_GPIO, TOUCH_SENSOR_ADC_CHANNEL);
        touch_sensor_->Initialize();

        /* 轮询任务：检测触摸并切换眼睛表情 */
        xTaskCreate([](void* arg) {
            auto* board = static_cast<CompactWifiBoardLCD*>(arg);
            TouchLevel prev_level = TouchLevel::kNone;
            TouchLevel pending_level = TouchLevel::kNone;
            int debounce_count = 0;
            int debug_counter = 0;
            static constexpr int kDebounceThreshold = 2;   /* 连续2次相同即确认（快速响应） */
            static constexpr int kPollIntervalMs    = 50;  /* 50ms 轮询（比原来快一倍） */
            static constexpr int kDebugIntervalCount = 40; /* 40×50ms = 2秒打印一次 */
            while (true) {
                int raw = 0;
                TouchLevel level = board->touch_sensor_->ReadRawAndLevel(raw);

                /* 每2秒打印一次 raw 值，方便调试 */
                if (++debug_counter >= kDebugIntervalCount) {
                    debug_counter = 0;
                    ESP_LOGI(TAG, "Touch raw: %d, level: %d", raw, (int)level);
                }

                /* 防抖：连续 N 次读到相同等级才确认变化 */
                if (level != prev_level) {
                    if (level == pending_level) {
                        debounce_count++;
                    } else {
                        pending_level = level;
                        debounce_count = 1;
                    }
                    if (debounce_count >= kDebounceThreshold) {
                        ESP_LOGI(TAG, "Touch changed: level %d -> %d, raw: %d",
                                 (int)prev_level, (int)level, raw);
                        if (level != TouchLevel::kNone) {
                            board->touch_override_ = true;
                            if (level == TouchLevel::kLight) {
                                board->cat_eyes_->SetEmotion("happy");
                                if (board->tail_servo_) board->tail_servo_->WagSlow();
                            } else if (level == TouchLevel::kMedium) {
                                board->cat_eyes_->SetEmotion("love");
                                if (board->tail_servo_) board->tail_servo_->Wag();
                            } else {
                                board->cat_eyes_->SetEmotion("surprised");
                                if (board->tail_servo_) board->tail_servo_->Perk();
                            }
                        } else if (board->touch_override_) {
                            board->touch_override_ = false;
                            board->cat_eyes_->SetEmotion("neutral");
                            if (board->tail_servo_) board->tail_servo_->SetAngle(90);
                        }
                        prev_level = level;
                        debounce_count = 0;
                    }
                } else {
                    debounce_count = 0;
                    pending_level = prev_level;
                }
                vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
            }
        }, "touch_poll", 3072, this, 2, nullptr);
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeTools() {
        // 灯控已移除（G18 分配给摄像头）
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeEyeDisplays();
        InitializeTailServo();
        InitializeHeadGimbal();
        InitializeDisplay();
        InitializeButtons();
        InitializeTouchSensor();
        InitializeCamera();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static NoLed led;
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual Backlight* GetBacklight() override {
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
