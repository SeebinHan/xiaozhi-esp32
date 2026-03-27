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
#include "pca9685.h"
#include "presence_sensor.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string>

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
    i2c_master_bus_handle_t servo_i2c_bus_ = nullptr;   // I2C bus for PCA9685 + camera SCCB
    Pca9685* servo_driver_ = nullptr;
    bool touch_override_ = false;  /* 触摸时临时覆盖眼睛表情 */
    PresenceSensor* presence_sensor_ = nullptr;

    void InitializeEyeDisplays() {
        cat_eyes_ = new CatEyeDisplay();
        cat_eyes_->Initialize();
    }

    void InitializeServoBus() {
        // I2C_NUM_0 on CAMERA_PIN_SIOD/CAMERA_PIN_SIOC, shared with OV2640 SCCB
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = CAMERA_PIN_SIOD,
            .scl_io_num = CAMERA_PIN_SIOC,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = true},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &servo_i2c_bus_));
        servo_driver_ = new Pca9685(servo_i2c_bus_, 0x40);
        servo_driver_->InitializeServoMode();
    }

    void InitializeTailServo() {
        tail_servo_ = new TailServo(servo_driver_, 2, 3);
        tail_servo_->Initialize();
    }

    void InitializeHeadGimbal() {
        head_gimbal_ = new HeadGimbal(servo_driver_, 1, 0);
        head_gimbal_->Initialize();
    }

    void RunServoSelfTest() {
        if (!head_gimbal_ || !tail_servo_) return;

        ESP_LOGI(TAG, "Running servo self-test: CH1(head pan) -> CH0(head tilt) -> CH2(tail H) -> CH3(tail V)");

        head_gimbal_->LookCenter();
        vTaskDelay(pdMS_TO_TICKS(250));

        head_gimbal_->SetPan(60);
        vTaskDelay(pdMS_TO_TICKS(350));
        head_gimbal_->SetPan(120);
        vTaskDelay(pdMS_TO_TICKS(350));
        head_gimbal_->SetPan(90);
        vTaskDelay(pdMS_TO_TICKS(250));

        head_gimbal_->SetTilt(60);
        vTaskDelay(pdMS_TO_TICKS(350));
        head_gimbal_->SetTilt(120);
        vTaskDelay(pdMS_TO_TICKS(350));
        head_gimbal_->SetTilt(90);
        vTaskDelay(pdMS_TO_TICKS(250));

        tail_servo_->SetHorizontal(60);
        vTaskDelay(pdMS_TO_TICKS(350));
        tail_servo_->SetHorizontal(120);
        vTaskDelay(pdMS_TO_TICKS(350));
        tail_servo_->SetHorizontal(90);
        vTaskDelay(pdMS_TO_TICKS(250));

        tail_servo_->SetVertical(60);
        vTaskDelay(pdMS_TO_TICKS(350));
        tail_servo_->SetVertical(120);
        vTaskDelay(pdMS_TO_TICKS(350));
        tail_servo_->SetVertical(90);
        vTaskDelay(pdMS_TO_TICKS(250));
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
        config.pin_sccb_sda = -1;            // 复用已初始化的 I2C 总线（与 PCA9685 共线）
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RST;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        // QVGA is a better first-fit for visual QA on this board:
        // it keeps enough scene detail for cloud understanding while cutting
        // capture, copy and JPEG encode cost to roughly one quarter of VGA.
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
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

    void InitializePresenceSensor() {
        presence_sensor_ = new PresenceSensor(PRESENCE_SENSOR_GPIO);
        presence_sensor_->Initialize();
        presence_sensor_->OnPresenceDetected([]() {
            auto& app = Application::GetInstance();
            // 仅在空闲状态下主动触发对话，避免打断正在进行的交互
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.ToggleChatState();
            }
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
            static constexpr int kDebounceThreshold = 2;
            static constexpr int kPollIntervalMs    = 50;
            while (true) {
                int raw = 0;
                TouchLevel level = board->touch_sensor_->ReadRawAndLevel(raw);

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
                                if (board->head_gimbal_) board->head_gimbal_->Nod();
                            } else if (level == TouchLevel::kMedium) {
                                board->cat_eyes_->SetEmotion("love");
                                if (board->tail_servo_) board->tail_servo_->Wag();
                                if (board->head_gimbal_) board->head_gimbal_->LookUp();
                            } else {
                                board->cat_eyes_->SetEmotion("surprised");
                                if (board->tail_servo_) board->tail_servo_->Perk();
                                if (board->head_gimbal_) board->head_gimbal_->Shake();
                            }
                        } else if (board->touch_override_) {
                            board->touch_override_ = false;
                            board->cat_eyes_->SetEmotion("neutral");
                            if (board->tail_servo_) board->tail_servo_->SetAngle(90);
                            if (board->head_gimbal_) board->head_gimbal_->LookCenter();
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
        InitializeServoBus();
        InitializeTailServo();
        InitializeHeadGimbal();
        RunServoSelfTest();
        InitializeDisplay();
        InitializeButtons();
        InitializeTouchSensor();
        InitializeCamera();
        InitializePresenceSensor();
        InitializeTools();
    }

    virtual std::string GetBoardType() override {
        return "bread-compact-wifi-lcd";
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

#ifdef CONFIG_ENABLE_4G
#include "dual_network_board.h"

// 4G 版本：继承 DualNetworkBoard，外设与 WiFi 版完全一致
// 到货后只需在 menuconfig 打开 CONFIG_ENABLE_4G 并重新编译
class CompactWifiBoardLCD4G : public DualNetworkBoard {
private:
    Button boot_button_;
    Display* display_;
    CatEyeDisplay* cat_eyes_ = nullptr;
    TouchSensor* touch_sensor_ = nullptr;
    TailServo* tail_servo_ = nullptr;
    HeadGimbal* head_gimbal_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    i2c_master_bus_handle_t servo_i2c_bus_ = nullptr;
    Pca9685* servo_driver_ = nullptr;
    bool touch_override_ = false;
    PresenceSensor* presence_sensor_ = nullptr;

    void InitializeEyeDisplays() {
        cat_eyes_ = new CatEyeDisplay();
        cat_eyes_->Initialize();
    }

    void InitializeServoBus() {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = CAMERA_PIN_SIOD,
            .scl_io_num = CAMERA_PIN_SIOC,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = true},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &servo_i2c_bus_));
        servo_driver_ = new Pca9685(servo_i2c_bus_, 0x40);
        servo_driver_->InitializeServoMode();
    }

    void InitializeTailServo() {
        tail_servo_ = new TailServo(servo_driver_, 2, 3);
        tail_servo_->Initialize();
    }

    void InitializeHeadGimbal() {
        head_gimbal_ = new HeadGimbal(servo_driver_, 1, 0);
        head_gimbal_->Initialize();
    }

    void InitializeDisplay() {
        display_ = new CatDisplayWithPeripherals(cat_eyes_, tail_servo_, head_gimbal_);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTouchSensor() {
        touch_sensor_ = new TouchSensor(TOUCH_SENSOR_GPIO, TOUCH_SENSOR_ADC_CHANNEL);
        touch_sensor_->Initialize();
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
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RST;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
    }

    void InitializePresenceSensor() {
        presence_sensor_ = new PresenceSensor(PRESENCE_SENSOR_GPIO);
        presence_sensor_->Initialize();
        presence_sensor_->OnPresenceDetected([]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.ToggleChatState();
            }
        });
    }

public:
    CompactWifiBoardLCD4G() :
        DualNetworkBoard(ML307_TX_GPIO, ML307_RX_GPIO, GPIO_NUM_NC, 1, 115200),
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeEyeDisplays();
        InitializeServoBus();
        InitializeTailServo();
        InitializeHeadGimbal();
        InitializeDisplay();
        InitializeButtons();
        InitializeTouchSensor();
        InitializeCamera();
        InitializePresenceSensor();
    }

    virtual std::string GetBoardType() override {
        return "bread-compact-wifi-lcd";
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

DECLARE_BOARD(CompactWifiBoardLCD4G);

#else

DECLARE_BOARD(CompactWifiBoardLCD);

#endif  // CONFIG_ENABLE_4G
