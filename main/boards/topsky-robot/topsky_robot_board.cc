#include "dual_network_board.h"
#include "audio/codecs/no_audio_codec.h"
#include "config.h"
#include "display/display.h"
#include "esp32_camera.h"
#include "led/single_led.h"
#include "presence_sensor.h"
#include "touch_sensor.h"
#include "head_gimbal.h"
#include "tail_servo.h"
#include "pca9685.h"
#include "application.h"
#include "assets/lang_config.h"

#include <algorithm>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "TopskyRobotBoard"

namespace {
constexpr uint32_t kPresenceInitialArmDelayMs = 0;
constexpr uint32_t kPresenceRearmDelayMs = 75000;
constexpr TickType_t kTouchPollInterval = pdMS_TO_TICKS(20);
constexpr int kTouchStableThreshold = 3;
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

PresenceSensor* g_presence_sensor = nullptr;
TouchSensor* g_touch_sensor = nullptr;

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

static void PlayTouchSfx(int level) {
    auto& app = Application::GetInstance();
    auto state = app.GetDeviceState();
    if (!(state == kDeviceStateIdle || state == kDeviceStateStarting)) {
        return;
    }

    if (level == static_cast<int>(TouchLevel::kLight)) {
        app.PlaySound(Lang::Sounds::OGG_MEOW_SOFT);
    } else if (level == static_cast<int>(TouchLevel::kMedium)) {
        app.PlaySound(Lang::Sounds::OGG_MEOW_CUTE);
    } else {
        app.PlaySound(Lang::Sounds::OGG_MEOW_LOUD);
    }
}

void EnablePresenceSensorWhenReadyTask(void* arg) {
    auto* sensor = static_cast<PresenceSensor*>(arg);
    bool last_enabled = false;
    bool has_armed_once = false;
    while (true) {
        auto& app = Application::GetInstance();
        bool enabled = app.GetDeviceState() == kDeviceStateIdle && app.IsPresenceTransportReady();
        if (enabled != last_enabled) {
            uint32_t arm_delay_ms = 0;
            if (enabled) {
                arm_delay_ms = has_armed_once ? kPresenceRearmDelayMs : kPresenceInitialArmDelayMs;
                has_armed_once = true;
            }
            sensor->SetEnabled(enabled, arm_delay_ms);
            last_enabled = enabled;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TouchPollTask(void* arg) {
    auto* sensor = static_cast<TouchSensor*>(arg);
    TouchLevel stable_level = TouchLevel::kNone;
    TouchLevel candidate_level = TouchLevel::kNone;
    int candidate_count = 0;
    bool touch_armed = true;

    while (true) {
        int raw = 0;
        TouchLevel level = sensor->ReadRawAndLevel(raw);

        if (level == candidate_level) {
            candidate_count++;
        } else {
            candidate_level = level;
            candidate_count = 1;
        }

        if (candidate_count >= kTouchStableThreshold && candidate_level != stable_level) {
            stable_level = candidate_level;

            if (stable_level == TouchLevel::kNone) {
                touch_armed = true;
            } else if (touch_armed) {
                TouchEvent event{};
                event.level = stable_level;
                event.raw = raw;
                Application::GetInstance().SubmitTouchEvent(event);
                touch_armed = false;
            }
        }

        vTaskDelay(kTouchPollInterval);
    }
}
}  // namespace

class TopskyRobotBoard : public DualNetworkBoard {
private:
    Esp32Camera* camera_ = nullptr;
    i2c_master_bus_handle_t servo_i2c_bus_ = nullptr;
    Pca9685* servo_driver_ = nullptr;
    HeadGimbal* head_gimbal_ = nullptr;
    TailServo* tail_servo_ = nullptr;

    void InitializeServoBus() {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = CAMERA_PIN_SIOD,
            .scl_io_num = CAMERA_PIN_SIOC,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = true,
                .allow_pd = false,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &servo_i2c_bus_));
        servo_driver_ = new Pca9685(servo_i2c_bus_, 0x40);
        servo_driver_->InitializeServoMode();
    }

    void InitializeTailServo() {
        tail_servo_ = new TailServo(servo_driver_, 2, 3);
        tail_servo_->Initialize();
        tail_servo_->SetHorizontal(kServoZeroPose.tail_horizontal);
        tail_servo_->SetVertical(kServoZeroPose.tail_vertical);
    }

    void InitializeHeadGimbal() {
        head_gimbal_ = new HeadGimbal(servo_driver_, 1, 0);
        head_gimbal_->Initialize();
        head_gimbal_->SetPan(kServoZeroPose.head_pan);
        head_gimbal_->SetTilt(kServoZeroPose.head_tilt);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
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
        config.sccb_i2c_port = I2C_NUM_1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
        camera_->SetVFlip(true);
    }

    void InitializePresenceSensor() {
        g_presence_sensor = new PresenceSensor(PRESENCE_SENSOR_GPIO);
        g_presence_sensor->OnPresenceDetected([]() {
            Application::GetInstance().SubmitPresenceDetection();
        });
        g_presence_sensor->Initialize();
        g_presence_sensor->SetEnabled(false, kPresenceInitialArmDelayMs);
        xTaskCreate(EnablePresenceSensorWhenReadyTask, "presence_arm", 2048, g_presence_sensor, 1, nullptr);
    }

    void InitializeTouchSensor() {
        g_touch_sensor = new TouchSensor(TOUCH_SENSOR_GPIO, TOUCH_SENSOR_ADC_CHANNEL);
        g_touch_sensor->Initialize();
        xTaskCreate(TouchPollTask, "touch_poll", 4096, g_touch_sensor, 3, nullptr);
    }

public:
    TopskyRobotBoard()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, ML307_DTR_PIN, 0) {
        InitializeServoBus();
        InitializeHeadGimbal();
        InitializeTailServo();
        InitializeCamera();
        InitializePresenceSensor();
        InitializeTouchSensor();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS,
            AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual Display* GetDisplay() override {
        static NoDisplay display;
        return &display;
    }

    virtual Led* GetLed() override {
        if (BUILTIN_LED_GPIO != GPIO_NUM_NC) {
            static SingleLed led(BUILTIN_LED_GPIO);
            return &led;
        }
        return DualNetworkBoard::GetLed();
    }

    virtual void ExecuteTouchAction(int level, int raw) override {
        ESP_LOGI(TAG, "Execute touch servo action: level=%d raw=%d", level, raw);
        if (head_gimbal_ == nullptr || tail_servo_ == nullptr) {
            return;
        }

        PlayTouchSfx(level);
        ServoPose base_pose = CapturePose(head_gimbal_, tail_servo_);

        if (level == static_cast<int>(TouchLevel::kLight)) {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -8, 0, +14}, 160, 80});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -10, +26, +16}, 180, 70});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -10, -26, +16}, 220, 70});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -9, +20, +14}, 220, 60});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -6, 0, +8}, 240, 120});
        } else if (level == static_cast<int>(TouchLevel::kMedium)) {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{-6, -14, 0, +22}, 180, 90});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{-4, -16, +36, +28}, 200, 80});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{+2, -14, -36, +28}, 230, 80});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{-2, -12, +28, +24}, 230, 80});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -10, 0, +18}, 260, 140});
        } else {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{-10, -20, 0, +34}, 160, 100});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{-6, -16, +16, +38}, 220, 90});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{+4, -10, -14, +30}, 260, 100});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose, {{0, -8, 0, +20}, 300, 180});
        }

        ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, 280);
    }
};

DECLARE_BOARD(TopskyRobotBoard);
