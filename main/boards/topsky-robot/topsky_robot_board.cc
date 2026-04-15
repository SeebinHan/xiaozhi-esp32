#include "dual_network_board.h"
#include "audio/codecs/no_audio_codec.h"
#include "config.h"
#include "display/display.h"
#include "esp32_camera.h"
#include "led/single_led.h"
#include "presence_sensor.h"
#include "application.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "TopskyRobotBoard"

namespace {
constexpr uint32_t kPresenceInitialArmDelayMs = 0;
constexpr uint32_t kPresenceRearmDelayMs = 75000;

PresenceSensor* g_presence_sensor = nullptr;

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
}  // namespace

class TopskyRobotBoard : public DualNetworkBoard {
private:
    Esp32Camera* camera_ = nullptr;

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
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
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

public:
    TopskyRobotBoard()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, ML307_DTR_PIN, 0) {
        InitializeCamera();
        InitializePresenceSensor();
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
};

DECLARE_BOARD(TopskyRobotBoard);
