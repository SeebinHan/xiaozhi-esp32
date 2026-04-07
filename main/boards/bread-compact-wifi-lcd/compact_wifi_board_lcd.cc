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
#include "audio/sfx_assets.h"
#include "presence_sensor.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>

#define TAG "CompactWifiBoardLCD"

namespace {

constexpr int kMotionTickMs = 20;

struct ServoPose {
    int head_pan;
    int head_tilt;
    int tail_horizontal;
    int tail_vertical;
};

struct ServoOffset {
    int head_pan = 0;
    int head_tilt = 0;
    int tail_horizontal = 0;
    int tail_vertical = 0;
};

struct MotionSegment {
    ServoOffset offset;
    int move_ms;
    int hold_ms;
};

constexpr ServoPose kServoZeroPose = {
    .head_pan = 50,
    .head_tilt = 85,
    .tail_horizontal = 5,
    .tail_vertical = 0,
};

static int ClampAngle(int angle) {
    return std::clamp(angle, 0, 180);
}

static ServoPose AddOffset(const ServoPose& base, const ServoOffset& offset) {
    return {
        .head_pan = ClampAngle(base.head_pan + offset.head_pan),
        .head_tilt = ClampAngle(base.head_tilt + offset.head_tilt),
        .tail_horizontal = ClampAngle(base.tail_horizontal + offset.tail_horizontal),
        .tail_vertical = ClampAngle(base.tail_vertical + offset.tail_vertical),
    };
}

static ServoPose CapturePose(HeadGimbal* head, TailServo* tail) {
    ServoPose pose = kServoZeroPose;
    if (head) {
        pose.head_pan = head->pan_angle();
        pose.head_tilt = head->tilt_angle();
    }
    if (tail) {
        pose.tail_horizontal = tail->horizontal_angle();
        pose.tail_vertical = tail->vertical_angle();
    }
    return pose;
}

static void ApplyPoseInstant(HeadGimbal* head, TailServo* tail, const ServoPose& pose) {
    if (head) {
        head->SetPan(pose.head_pan);
        head->SetTilt(pose.head_tilt);
    }
    if (tail) {
        tail->SetHorizontal(pose.tail_horizontal);
        tail->SetVertical(pose.tail_vertical);
    }
}

static void ApplyPoseSmooth(HeadGimbal* head, TailServo* tail, const ServoPose& target, int duration_ms) {
    ServoPose start = CapturePose(head, tail);
    int steps = std::max(1, duration_ms / kMotionTickMs);
    for (int step = 1; step <= steps; ++step) {
        auto lerp = [step, steps](int from, int to) {
            return from + (to - from) * step / steps;
        };
        ServoPose current = {
            .head_pan = lerp(start.head_pan, target.head_pan),
            .head_tilt = lerp(start.head_tilt, target.head_tilt),
            .tail_horizontal = lerp(start.tail_horizontal, target.tail_horizontal),
            .tail_vertical = lerp(start.tail_vertical, target.tail_vertical),
        };
        ApplyPoseInstant(head, tail, current);
        vTaskDelay(pdMS_TO_TICKS(kMotionTickMs));
    }
}

static void RunMotionSegment(HeadGimbal* head, TailServo* tail, const ServoPose& base, const MotionSegment& segment) {
    ApplyPoseSmooth(head, tail, AddOffset(base, segment.offset), segment.move_ms);
    if (segment.hold_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(segment.hold_ms));
    }
}

static void ApplyZeroPose(HeadGimbal* head, TailServo* tail) {
    ApplyPoseInstant(head, tail, kServoZeroPose);
}

static void PlayTouchSfx(TouchLevel level) {
    auto& app = Application::GetInstance();
    auto state = app.GetDeviceState();
    if (!(state == kDeviceStateIdle || state == kDeviceStateStarting ||
          state == kDeviceStateListening)) {
        return;
    }

    if (level == TouchLevel::kLight) {
        app.PlaySound(SfxSounds::OGG_MEOW_SOFT);
    } else if (level == TouchLevel::kMedium) {
        app.PlaySound(SfxSounds::OGG_MEOW_CUTE);
    } else {
        app.PlaySound(SfxSounds::OGG_MEOW_LOUD);
    }
}

static void RunTouchReaction(TouchLevel level, CatEyeDisplay* eyes, HeadGimbal* head, TailServo* tail, const ServoPose& base_pose) {
    if (!eyes) {
        return;
    }

    PlayTouchSfx(level);

    if (level == TouchLevel::kLight) {
        eyes->SetEmotion("happy");
        RunMotionSegment(head, tail, base_pose, {{0, -8, 0, +14}, 160, 80});
        RunMotionSegment(head, tail, base_pose, {{0, -10, +26, +16}, 180, 70});
        RunMotionSegment(head, tail, base_pose, {{0, -10, -26, +16}, 220, 70});
        RunMotionSegment(head, tail, base_pose, {{0, -9, +20, +14}, 220, 60});
        RunMotionSegment(head, tail, base_pose, {{0, -6, 0, +8}, 240, 120});
    } else if (level == TouchLevel::kMedium) {
        eyes->SetEmotion("love");
        RunMotionSegment(head, tail, base_pose, {{-6, -14, 0, +22}, 180, 90});
        RunMotionSegment(head, tail, base_pose, {{-4, -16, +36, +28}, 200, 80});
        RunMotionSegment(head, tail, base_pose, {{+2, -14, -36, +28}, 230, 80});
        RunMotionSegment(head, tail, base_pose, {{-2, -12, +28, +24}, 230, 80});
        RunMotionSegment(head, tail, base_pose, {{0, -10, 0, +18}, 260, 140});
    } else {
        eyes->SetEmotion("surprised");
        RunMotionSegment(head, tail, base_pose, {{-10, -20, 0, +34}, 160, 100});
        RunMotionSegment(head, tail, base_pose, {{-6, -16, +16, +38}, 220, 90});
        RunMotionSegment(head, tail, base_pose, {{+4, -10, -14, +30}, 260, 100});
        RunMotionSegment(head, tail, base_pose, {{0, -8, 0, +20}, 300, 180});
    }
}


enum class EmotionMotionType {
    None,
    HeadOnly,
    TailOnly,
    HeadTailCombo,
};

enum class EmotionKind {
    None,
    Happy,
    Love,
    Surprised,
    Sad,
    Thinking,
};

struct EmotionMotion {
    EmotionMotionType type;
    EmotionKind kind;
};

static EmotionMotion GetEmotionMotion(const char* emotion) {
    std::string emo = emotion ? emotion : "";
    if (emo == "happy" || emo == "laughing" || emo == "funny") {
        return {EmotionMotionType::TailOnly, EmotionKind::Happy};
    }
    if (emo == "love" || emo == "heart_eyes") {
        return {EmotionMotionType::HeadTailCombo, EmotionKind::Love};
    }
    if (emo == "surprised" || emo == "angry" || emo == "hateful") {
        return {EmotionMotionType::HeadTailCombo, EmotionKind::Surprised};
    }
    if (emo == "sad" || emo == "crying") {
        return {EmotionMotionType::HeadOnly, EmotionKind::Sad};
    }
    if (emo == "thinking" || emo == "confused") {
        return {EmotionMotionType::HeadOnly, EmotionKind::Thinking};
    }
    return {EmotionMotionType::None, EmotionKind::None};
}

static void RunEmotionReaction(const EmotionMotion& motion, HeadGimbal* head, TailServo* tail) {
    ServoPose base_pose = CapturePose(head, tail);
    switch (motion.type) {
    case EmotionMotionType::TailOnly:
        RunMotionSegment(head, tail, base_pose, {{0, 0, 0, +8}, 150, 60});
        RunMotionSegment(head, tail, base_pose, {{0, 0, +14, +10}, 170, 60});
        RunMotionSegment(head, tail, base_pose, {{0, 0, -14, +10}, 210, 80});
        RunMotionSegment(head, tail, base_pose, {{0, 0, +10, +8}, 220, 80});
        RunMotionSegment(head, tail, base_pose, {{0, 0, 0, +4}, 220, 100});
        ApplyPoseSmooth(head, tail, kServoZeroPose, 260);
        break;
    case EmotionMotionType::HeadOnly:
        if (motion.kind == EmotionKind::Sad) {
            RunMotionSegment(head, tail, base_pose, {{0, +12, 0, -6}, 220, 220});
        } else {
            RunMotionSegment(head, tail, base_pose, {{-8, -6, 0, 0}, 200, 120});
            RunMotionSegment(head, tail, base_pose, {{+6, -4, 0, 0}, 240, 140});
        }
        ApplyPoseSmooth(head, tail, kServoZeroPose, 280);
        break;
    case EmotionMotionType::HeadTailCombo:
        if (motion.kind == EmotionKind::Love) {
            RunMotionSegment(head, tail, base_pose, {{-4, -12, 0, +14}, 180, 80});
            RunMotionSegment(head, tail, base_pose, {{-2, -14, +20, +18}, 220, 70});
            RunMotionSegment(head, tail, base_pose, {{+2, -12, -20, +18}, 250, 90});
            RunMotionSegment(head, tail, base_pose, {{0, -10, 0, +12}, 260, 100});
        } else {
            RunMotionSegment(head, tail, base_pose, {{-10, -18, 0, +24}, 160, 90});
            RunMotionSegment(head, tail, base_pose, {{-6, -14, +10, +28}, 220, 90});
            RunMotionSegment(head, tail, base_pose, {{+4, -10, -10, +20}, 260, 110});
        }
        ApplyPoseSmooth(head, tail, kServoZeroPose, 300);
        break;
    case EmotionMotionType::None:
    default:
        break;
    }
}

struct TouchReactionContext {
    CatEyeDisplay* eyes;
    TailServo* tail;
    HeadGimbal* head;
    TouchSensor* sensor;
    bool* touch_override;
};

static void EnablePresenceSensorWhenReadyTask(void* arg) {
    auto* sensor = static_cast<PresenceSensor*>(arg);
    auto& app = Application::GetInstance();
    bool was_enabled = false;
    int stable_count = 0;
    while (stable_count < 8) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (app.IsProactiveGreetingTransportReady()) {
            stable_count++;
        } else {
            stable_count = 0;
        }
    }
    sensor->SetEnabled(true);
    was_enabled = true;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        bool ready = app.IsProactiveGreetingTransportReady();
        if (ready && !was_enabled) {
            sensor->SetEnabled(true);
            was_enabled = true;
        } else if (!ready && was_enabled) {
            sensor->SetEnabled(false);
            was_enabled = false;
        }
    }
}

static void TouchReactionTask(void* arg) {
    auto* ctx = static_cast<TouchReactionContext*>(arg);
    TouchLevel prev_level = TouchLevel::kNone;
    TouchLevel pending_level = TouchLevel::kNone;
    ServoPose saved_pose = kServoZeroPose;
    int debounce_count = 0;
    constexpr int kDebounceThreshold = 2;
    constexpr int kPollIntervalMs = 50;

    while (true) {
        int raw = 0;
        TouchLevel level = ctx->sensor->ReadRawAndLevel(raw);

        if (level != prev_level) {
            if (level == pending_level) {
                debounce_count++;
            } else {
                pending_level = level;
                debounce_count = 1;
            }

            if (debounce_count >= kDebounceThreshold) {
                ESP_LOGI(TAG, "Touch changed: level %d -> %d, raw: %d", (int)prev_level, (int)level, raw);
                if (level != TouchLevel::kNone) {
                    *ctx->touch_override = true;
                    saved_pose = CapturePose(ctx->head, ctx->tail);
                    RunTouchReaction(level, ctx->eyes, ctx->head, ctx->tail, saved_pose);
                } else if (*ctx->touch_override) {
                    *ctx->touch_override = false;
                    if (ctx->eyes) {
                        ctx->eyes->SetEmotion("neutral");
                    }
                    ApplyPoseSmooth(ctx->head, ctx->tail, kServoZeroPose, 280);
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
}

} // namespace

/* 无主 LCD 的显示包装：通过 NoDisplay 提供空显示，同时在 SetEmotion 时
   联动猫眼显示，并在非触摸占用时触发情绪动作。 */
class CatDisplayWithPeripherals : public NoDisplay {
private:
    CatEyeDisplay* cat_eyes_;
    TailServo* tail_;
    HeadGimbal* head_;
    bool* touch_override_;
    const char* last_emotion_ = nullptr;
public:
    CatDisplayWithPeripherals(CatEyeDisplay* eyes, TailServo* tail, HeadGimbal* head, bool* touch_override)
        : cat_eyes_(eyes), tail_(tail), head_(head), touch_override_(touch_override) {}

    void SetEmotion(const char* emotion) override {
        NoDisplay::SetEmotion(emotion);
        if (cat_eyes_) {
            cat_eyes_->SetEmotion(emotion);
        }

        if (!emotion || !tail_ || !head_) {
            last_emotion_ = emotion;
            return;
        }

        if (touch_override_ && *touch_override_) {
            last_emotion_ = emotion;
            return;
        }

        if (last_emotion_ && std::string(last_emotion_) == emotion) {
            return;
        }
        last_emotion_ = emotion;

        EmotionMotion motion = GetEmotionMotion(emotion);
        if (motion.type == EmotionMotionType::None) {
            return;
        }
        RunEmotionReaction(motion, head_, tail_);
    }
};

class CompactWifiBoardLCD : public WifiBoard {

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

    void InitializeDisplay() {
        display_ = new CatDisplayWithPeripherals(cat_eyes_, tail_servo_, head_gimbal_, &touch_override_);
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
        camera_->SetVFlip(true);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                this->EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializePresenceSensor() {
        presence_sensor_ = new PresenceSensor(PRESENCE_SENSOR_GPIO);
        presence_sensor_->Initialize();
        // Sensor starts disabled; EnablePresenceSensorWhenReadyTask enables after network+MCP ready
        presence_sensor_->OnPresenceDetected([]() {
            Application::GetInstance().RequestPresenceGreeting();
        });
        xTaskCreate(EnablePresenceSensorWhenReadyTask, "presence_arm", 2048, presence_sensor_, 1, nullptr);
    }

    void InitializeTouchSensor() {
        touch_sensor_ = new TouchSensor(TOUCH_SENSOR_GPIO, TOUCH_SENSOR_ADC_CHANNEL);
        touch_sensor_->Initialize();

        auto* ctx = new TouchReactionContext{
            .eyes = cat_eyes_,
            .tail = tail_servo_,
            .head = head_gimbal_,
            .sensor = touch_sensor_,
            .touch_override = &touch_override_,
        };
        xTaskCreate(TouchReactionTask, "touch_poll", 4096, ctx, 2, nullptr);
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
        ApplyZeroPose(head_gimbal_, tail_servo_);
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
        display_ = new CatDisplayWithPeripherals(cat_eyes_, tail_servo_, head_gimbal_, &touch_override_);
    }

    void InitializeButtons() {
        boot_button_.OnClick([]() {
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

        auto* ctx = new TouchReactionContext{
            .eyes = cat_eyes_,
            .tail = tail_servo_,
            .head = head_gimbal_,
            .sensor = touch_sensor_,
            .touch_override = &touch_override_,
        };
        xTaskCreate(TouchReactionTask, "touch_poll", 4096, ctx, 2, nullptr);
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
        camera_->SetVFlip(true);
    }

    void InitializePresenceSensor() {
        presence_sensor_ = new PresenceSensor(PRESENCE_SENSOR_GPIO);
        presence_sensor_->Initialize();
        presence_sensor_->OnPresenceDetected([]() {
            Application::GetInstance().RequestPresenceGreeting();
        });
        xTaskCreate(EnablePresenceSensorWhenReadyTask, "presence_arm", 2048, presence_sensor_, 1, nullptr);
    }

public:
    CompactWifiBoardLCD4G() :
        DualNetworkBoard(ML307_TX_GPIO, ML307_RX_GPIO, GPIO_NUM_NC, 1, 115200),
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeEyeDisplays();
        InitializeServoBus();
        InitializeTailServo();
        InitializeHeadGimbal();
        ApplyZeroPose(head_gimbal_, tail_servo_);
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
