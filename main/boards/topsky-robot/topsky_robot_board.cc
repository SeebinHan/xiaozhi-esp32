#include "dual_network_board.h"
#include "audio/codecs/no_audio_codec.h"
#include "config.h"
#include "display/display.h"
#include "boards/common/esp32_camera.h"
#include "led/single_led.h"
#include "presence_sensor.h"
#include "touch_sensor.h"
#include "head_gimbal.h"
#include "tail_servo.h"
#include "pca9685.h"
#include "cat_eye_display.h"
#include "mcp_server.h"
#include "application.h"
#include "assets/lang_config.h"
#include "motion_math.h"

#include <algorithm>
#include <esp_timer.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define TAG "TopskyRobotBoard"

namespace {
constexpr uint32_t kPresenceInitialArmDelayMs = 0;
constexpr uint32_t kPresenceRearmDelayMs = 75000;
constexpr TickType_t kTouchPollInterval = pdMS_TO_TICKS(20);
constexpr int kTouchStableThreshold = 3;
constexpr int kMotionTickMs = 20;
constexpr UBaseType_t kTouchActionWorkerPriority = 1;
constexpr size_t kTouchActionQueueLength = 1;

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
    EaseCurve curve = EaseCurve::kEaseInOutCubic;
    int head_phase_delay_percent = 0;
    int tail_phase_delay_percent = 0;
};

struct AxisTarget {
    int start;
    int target;
};

struct AxisTargets {
    AxisTarget head_pan;
    AxisTarget head_tilt;
    AxisTarget tail_horizontal;
    AxisTarget tail_vertical;
};

constexpr ServoPose kServoZeroPose = {
    .head_pan = 50,
    .head_tilt = 85,
    .tail_horizontal = 35,
    .tail_vertical = 0,
};

constexpr ServoAxisCalibration kHeadPanCalibration = {
    .trim_deg = 0,
    .min_deg = 15,
    .max_deg = 110,
    .reversed = false,
    .deadband_deg = 1,
};

constexpr ServoAxisCalibration kHeadTiltCalibration = {
    .trim_deg = 0,
    .min_deg = 45,
    .max_deg = 125,
    .reversed = false,
    .deadband_deg = 1,
};

constexpr ServoAxisCalibration kTailHorizontalCalibration = {
    .trim_deg = 0,
    .min_deg = 0,
    .max_deg = 90,
    .reversed = false,
    .deadband_deg = 2,
};

constexpr ServoAxisCalibration kTailVerticalCalibration = {
    .trim_deg = 0,
    .min_deg = 0,
    .max_deg = 60,
    .reversed = false,
    .deadband_deg = 1,
};

PresenceSensor* g_presence_sensor = nullptr;
TouchSensor* g_touch_sensor = nullptr;

static int ClampAngle(int angle) {
    if (angle < 0) {
        return 0;
    }
    if (angle > 180) {
        return 180;
    }
    return angle;
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

static AxisTargets BuildAxisTargets(const ServoPose& start, const ServoPose& target) {
    return {
        .head_pan = {start.head_pan, target.head_pan},
        .head_tilt = {start.head_tilt, target.head_tilt},
        .tail_horizontal = {start.tail_horizontal, target.tail_horizontal},
        .tail_vertical = {start.tail_vertical, target.tail_vertical},
    };
}

static int EvaluateAxis(const AxisTarget& axis, int progress_q10) {
    return axis.start + ((axis.target - axis.start) * progress_q10) / 1024;
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

static void ApplyPoseSmooth(HeadGimbal* head, TailServo* tail, const ServoPose& target, int duration_ms,
                            EaseCurve curve = EaseCurve::kEaseInOutCubic,
                            int head_phase_delay_percent = 0,
                            int tail_phase_delay_percent = 0) {
    ServoPose start = CapturePose(head, tail);
    AxisTargets axes = BuildAxisTargets(start, target);
    int total_ms = std::max(kMotionTickMs, duration_ms);
    for (int elapsed_ms = 0; elapsed_ms <= total_ms; elapsed_ms += kMotionTickMs) {
        int clamped_elapsed_ms = std::min(elapsed_ms, total_ms);
        int head_progress = ApplyEasing(curve, ComputeDelayedProgress(clamped_elapsed_ms, total_ms, head_phase_delay_percent));
        int tail_progress = ApplyEasing(curve, ComputeDelayedProgress(clamped_elapsed_ms, total_ms, tail_phase_delay_percent));
        ServoPose current = {
            .head_pan = EvaluateAxis(axes.head_pan, head_progress),
            .head_tilt = EvaluateAxis(axes.head_tilt, head_progress),
            .tail_horizontal = EvaluateAxis(axes.tail_horizontal, tail_progress),
            .tail_vertical = EvaluateAxis(axes.tail_vertical, tail_progress),
        };
        ApplyPoseInstant(head, tail, current);
        if (clamped_elapsed_ms < total_ms) {
            vTaskDelay(pdMS_TO_TICKS(kMotionTickMs));
        }
    }
}

static void RunMotionSegment(HeadGimbal* head, TailServo* tail, const ServoPose& base, const MotionSegment& segment) {
    ApplyPoseSmooth(head, tail, AddOffset(base, segment.offset), segment.move_ms, segment.curve,
                    segment.head_phase_delay_percent, segment.tail_phase_delay_percent);
    if (segment.hold_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(segment.hold_ms));
    }
}

static void PlayTouchSfx(const TouchAction& action) {
    if (!action.allow_sound) {
        return;
    }

    auto& app = Application::GetInstance();
    auto state = app.GetDeviceState();
    if (!(state == kDeviceStateIdle || state == kDeviceStateStarting)) {
        return;
    }

    if (action.level == TouchLevel::kLight) {
        app.PlaySound(Lang::Sounds::OGG_MEOW_SOFT);
    } else if (action.level == TouchLevel::kMedium) {
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
    Camera* camera_ = nullptr;
    CatEyeDisplay display_;
    i2c_master_bus_handle_t servo_i2c_bus_ = nullptr;
    Pca9685* servo_driver_ = nullptr;
    HeadGimbal* head_gimbal_ = nullptr;
    TailServo* tail_servo_ = nullptr;
    QueueHandle_t touch_action_queue_ = nullptr;
    TaskHandle_t touch_action_task_ = nullptr;

    static void TouchActionWorkerEntry(void* arg) {
        auto* board = static_cast<TopskyRobotBoard*>(arg);
        TouchAction action{};
        while (true) {
            if (xQueueReceive(board->touch_action_queue_, &action, portMAX_DELAY) != pdTRUE) {
                continue;
            }
            board->RunTouchAction(action);
        }
    }

    void RunTouchAction(const TouchAction& action) {
        ESP_LOGI(TAG, "Run touch servo action: level=%d raw=%d motion=%d sound=%d compact=%d preset=%d",
                 static_cast<int>(action.level), action.raw, static_cast<int>(action.motion),
                 action.allow_sound, action.compact_motion, static_cast<int>(action.preset));
        if (head_gimbal_ == nullptr || tail_servo_ == nullptr) {
            return;
        }

        PlayTouchSfx(action);
        ServoPose base_pose = CapturePose(head_gimbal_, tail_servo_);
        bool compact = action.compact_motion;

        if (action.preset != TouchMotionPreset::kNone) {
            RunPresetMotion(action.preset, base_pose);
            return;
        }

        if (action.motion == TouchMotionMode::kNone) {
            return;
        }

        if (action.motion == TouchMotionMode::kTailOnly) {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -6, +12, +2}, 140, 20, EaseCurve::kEaseInOutCubic, 18, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -10, +42, +4}, 240, 35, EaseCurve::kEaseOutQuad, 20, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -7, -42, +3}, 250, 35, EaseCurve::kEaseInOutCubic, 20, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -4, +16, +2}, 220, compact ? 20 : 40, EaseCurve::kEaseInOutCubic, 15, 0});
        } else if (compact) {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{-6, -12, +14, +3}, 140, 20, EaseCurve::kEaseInOutCubic, 10, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{+4, -18, +32, +5}, 240, 30, EaseCurve::kEaseOutQuad, 15, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -10, -30, +4}, 220, 25, EaseCurve::kEaseInOutCubic, 15, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -5, +12, +2}, 190, 30, EaseCurve::kEaseInOutCubic, 12, 0});
        } else if (action.level == TouchLevel::kMedium) {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{-8, -14, +18, +4}, 150, 20, EaseCurve::kEaseInOutCubic, 12, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{-14, -26, +55, +6}, 280, 40, EaseCurve::kEaseOutQuad, 18, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{+8, -22, -55, +5}, 300, 35, EaseCurve::kEaseInOutCubic, 22, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{-5, -16, +28, +4}, 260, 30, EaseCurve::kEaseInOutCubic, 18, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -10, -18, +3}, 220, 30, EaseCurve::kEaseInOutCubic, 16, 0});
        } else {
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{-7, -12, +16, +4}, 150, 20, EaseCurve::kEaseInOutCubic, 12, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{-10, -18, +42, +6}, 260, 35, EaseCurve::kEaseOutQuad, 18, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{+6, -16, -42, +5}, 270, 35, EaseCurve::kEaseInOutCubic, 20, 0});
            RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                             {{0, -8, +18, +3}, 220, 35, EaseCurve::kEaseInOutCubic, 14, 0});
        }

        ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, compact ? 280 : 420,
                        EaseCurve::kEaseInOutCubic, 0, compact ? 8 : 12);
    }

    void RunPresetMotion(TouchMotionPreset preset, const ServoPose& base_pose) {
        switch (preset) {
            case TouchMotionPreset::kNodYes:
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, +4, 0, +1}, 100, 10, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -18, 0, +2}, 150, 25, EaseCurve::kEaseOutQuad, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -10, 0, +1}, 120, 15, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -14, 0, +2}, 130, 20, EaseCurve::kEaseOutQuad, 0, 0});
                ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, 260, EaseCurve::kEaseInOutCubic, 0, 0);
                break;
            case TouchMotionPreset::kShakeNo:
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{-14, -4, 0, 0}, 130, 15, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{+18, -6, 0, 0}, 170, 20, EaseCurve::kEaseOutQuad, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{-14, -5, 0, 0}, 160, 20, EaseCurve::kEaseInOutCubic, 0, 0});
                ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, 260, EaseCurve::kEaseInOutCubic, 0, 0);
                break;
            case TouchMotionPreset::kWagTail:
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -2, +18, +2}, 120, 15, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -3, +48, +3}, 180, 20, EaseCurve::kEaseOutQuad, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -2, -48, +3}, 200, 20, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -2, +42, +3}, 180, 20, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -1, -32, +2}, 170, 15, EaseCurve::kEaseInOutCubic, 0, 0});
                ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, 260, EaseCurve::kEaseInOutCubic, 0, 0);
                break;
            case TouchMotionPreset::kGreetCombo:
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, +6, 0, 0}, 110, 10, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -4, +40, +3}, 180, 20, EaseCurve::kEaseOutQuad, 10, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -4, -40, +3}, 180, 20, EaseCurve::kEaseInOutCubic, 10, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -12, +14, +2}, 160, 20, EaseCurve::kEaseOutQuad, 0, 0});
                ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, 320, EaseCurve::kEaseInOutCubic, 0, 0);
                break;
            case TouchMotionPreset::kCalmCombo:
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -6, +16, +1}, 180, 20, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -6, -16, +1}, 200, 20, EaseCurve::kEaseInOutCubic, 0, 0});
                RunMotionSegment(head_gimbal_, tail_servo_, base_pose,
                                 {{0, -8, 0, 0}, 180, 20, EaseCurve::kEaseInOutCubic, 0, 0});
                ApplyPoseSmooth(head_gimbal_, tail_servo_, kServoZeroPose, 360, EaseCurve::kEaseInOutCubic, 0, 0);
                break;
            case TouchMotionPreset::kNone:
                break;
        }
    }
    bool CanQueueMcpMotion(const char*& reason) {
        auto& app = Application::GetInstance();
        if (app.GetRuntimeLoadLevel() == RuntimeLoadLevel::kHeavy || app.IsTouchHeavyLoadCooldownActive()) {
            reason = "heavy_load_or_cooldown";
            return false;
        }
        reason = "allowed";
        return true;
    }

    bool QueuePresetMotion(const std::string& preset) {
        if (touch_action_queue_ == nullptr || head_gimbal_ == nullptr || tail_servo_ == nullptr) {
            return false;
        }

        const char* reason = "unknown";
        if (!CanQueueMcpMotion(reason)) {
            ESP_LOGI(TAG, "Skip MCP preset motion: preset=%s reason=%s", preset.c_str(), reason);
            return false;
        }

        TouchAction action{};
        action.raw = 0;
        action.allow_sound = false;
        action.preset = ParseTouchMotionPreset(preset.c_str());

        if (action.preset == TouchMotionPreset::kNone) {
            return false;
        }

        xQueueOverwrite(touch_action_queue_, &action);
        return true;
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool(
            "self.robot.motion.nod_yes",
            "Use this tool to make the robot physically nod once when expressing agreement, acknowledgement, confirmation, or compliance. User requests like 想看你点头, 点点头, 你点一下头 must call this tool. Trigger the real motion instead of describing nodding in text.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return QueuePresetMotion("nod_yes");
            });

        mcp_server.AddTool(
            "self.robot.motion.shake_no",
            "Use this tool to make the robot physically shake its head once when expressing negation, refusal, correction, or uncertainty. User requests like 想看你摇头, 摇摇头 must call this tool. Trigger the real motion instead of describing head shaking in text.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return QueuePresetMotion("shake_no");
            });

        mcp_server.AddTool(
            "self.robot.motion.wag_tail",
            "Use this tool to make the robot physically wag its tail once when expressing happiness, welcome, affection, playfulness, or closeness. User requests like 想看你摇尾巴, 摇摇尾巴, 尾巴动一下 must call this tool. Trigger the real motion instead of describing tail wagging in text.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return QueuePresetMotion("wag_tail");
            });

        PropertyList properties;
        properties.AddProperty(Property("preset", kPropertyTypeString));
        mcp_server.AddTool(
            "self.robot.motion.play",
            "Use this tool for preset robot motions such as greet_combo or calm_combo when a single nod, shake, or tail wag is not enough. Do not mention preset names in speech.",
            properties,
            [this](const PropertyList& properties) -> ReturnValue {
                auto preset = properties["preset"].value<std::string>();
                return QueuePresetMotion(preset);
            });
    }

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
        tail_servo_->SetHorizontalCalibration(kTailHorizontalCalibration);
        tail_servo_->SetVerticalCalibration(kTailVerticalCalibration);
        tail_servo_->Initialize();
        tail_servo_->SetHorizontal(kServoZeroPose.tail_horizontal);
        tail_servo_->SetVertical(kServoZeroPose.tail_vertical);
    }

    void InitializeHeadGimbal() {
        head_gimbal_ = new HeadGimbal(servo_driver_, 1, 0);
        head_gimbal_->SetPanCalibration(kHeadPanCalibration);
        head_gimbal_->SetTiltCalibration(kHeadTiltCalibration);
        head_gimbal_->Initialize();
        head_gimbal_->SetPan(kServoZeroPose.head_pan);
        head_gimbal_->SetTilt(kServoZeroPose.head_tilt);
    }

    void InitializeTouchActionWorker() {
        touch_action_queue_ = xQueueCreate(kTouchActionQueueLength, sizeof(TouchAction));
        if (touch_action_queue_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create touch action queue");
            return;
        }
        xTaskCreate(TouchActionWorkerEntry, "touch_action", 4096, this,
                    kTouchActionWorkerPriority, &touch_action_task_);
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
        config.frame_size = FRAMESIZE_QVGA;
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
        InitializeTouchActionWorker();
        RegisterMcpTools();
        InitializeCamera();
        display_.Initialize();
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
        return &display_;
    }

    virtual Led* GetLed() override {
        if (BUILTIN_LED_GPIO != GPIO_NUM_NC) {
            static SingleLed led(BUILTIN_LED_GPIO);
            return &led;
        }
        return DualNetworkBoard::GetLed();
    }

    virtual void ExecuteTouchAction(const TouchAction& action) override {
        ESP_LOGI(TAG, "Queue touch servo action: level=%d raw=%d motion=%d sound=%d compact=%d",
                 static_cast<int>(action.level), action.raw, static_cast<int>(action.motion),
                 action.allow_sound, action.compact_motion);
        if (touch_action_queue_ == nullptr || action.motion == TouchMotionMode::kNone) {
            return;
        }
        xQueueOverwrite(touch_action_queue_, &action);
    }
};

DECLARE_BOARD(TopskyRobotBoard);
