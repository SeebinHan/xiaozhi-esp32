#include "wifi_board.h"
#include "audio/codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "esp32_camera.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include "xl9555.h"

#define TAG "TopSkyPetS3"

class TopSkyPetS3 : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    i2c_master_bus_handle_t i2c_master_bus_ = nullptr;   // I2C_NUM_1: camera SCCB + sensors
    i2c_master_bus_handle_t xl9555_i2c_bus_ = nullptr;   // I2C_NUM_0: XL9555 + sensor hub
    xl9555_handle_t xl9555_ = nullptr;

    /* ---------- Initialization ---------- */

    void InitializeXL9555Bus() {
        // I2C_NUM_0 bus for XL9555 IO expander (and optional I2C sensors on the same bus)
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = XL9555_I2C_PORT,
            .sda_io_num = XL9555_I2C_SDA,
            .scl_io_num = XL9555_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = {.enable_internal_pullup = true},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &xl9555_i2c_bus_));

        xl9555_config_t xl_cfg = {
            .bus = xl9555_i2c_bus_,
            .i2c_addr = XL9555_I2C_ADDR,
            .scl_speed_hz = 400000,
        };
        ESP_ERROR_CHECK(xl9555_new(&xl_cfg, &xl9555_));

        // Enable audio amplifier (XL9555 pin 15 = output, high)
        ESP_ERROR_CHECK(xl9555_set_pin_mode(xl9555_, XL9555_AUDIO_EN_PIN, false));
        ESP_ERROR_CHECK(xl9555_write_pin(xl9555_, XL9555_AUDIO_EN_PIN, true));

        // Enable LCD backlight (XL9555 pin 14 = output, high)
        ESP_ERROR_CHECK(xl9555_set_pin_mode(xl9555_, XL9555_BACKLIGHT_PIN, false));
        ESP_ERROR_CHECK(xl9555_write_pin(xl9555_, XL9555_BACKLIGHT_PIN, true));
    }

    void InitializeI2cMaster() {
        // I2C_NUM_1 bus for camera SCCB and board sensors (AHT30, AP3216C, QMI8658, FT6206)
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_MASTER_PORT,
            .sda_io_num = I2C_MASTER_SDA,
            .scl_io_num = I2C_MASTER_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = true},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_master_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_GPIO;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_GPIO;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_PORT, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_GPIO;
        io_config.dc_gpio_num = DISPLAY_DC_GPIO;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_PORT, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_GPIO;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                     DISPLAY_SWAP_XY);
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
        config.pin_sccb_sda = -1;            // use already-initialized I2C master bus
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = I2C_MASTER_PORT;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = CAMERA_XCLK_FREQ;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;   // 320x240
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;

        camera_ = new Esp32Camera(config);
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

        boot_button_.OnLongPress([this]() {
            ESP_LOGI(TAG, "Long press: enter WiFi config mode");
            EnterWifiConfigMode();
        });
    }

public:
    TopSkyPetS3() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeXL9555Bus();
        InitializeI2cMaster();
        InitializeSpi();
        InitializeDisplay();
        InitializeCamera();
        InitializeButtons();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_master_bus_,
            I2C_MASTER_PORT,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_AMPLIFIER_EN,
            AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(TopSkyPetS3);
