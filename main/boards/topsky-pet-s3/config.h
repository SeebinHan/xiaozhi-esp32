#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

/* ---- Display (ST7789, 320x240, SPI) ---- */
#define DISPLAY_SPI_PORT        SPI3_HOST
#define DISPLAY_CLK_GPIO        GPIO_NUM_21
#define DISPLAY_MOSI_GPIO       GPIO_NUM_47
#define DISPLAY_DC_GPIO         GPIO_NUM_3
#define DISPLAY_CS_GPIO         GPIO_NUM_2
#define DISPLAY_RST_GPIO        GPIO_NUM_NC

#define SCREEN_W  320
#define SCREEN_H  240

/* 270° rotation (landscape, default orientation for this board) */
#define DISPLAY_WIDTH    SCREEN_W
#define DISPLAY_HEIGHT   SCREEN_H
#define DISPLAY_SWAP_XY  true
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0

/* ---- XL9555 IO Expander ---- */
#define XL9555_I2C_SDA          GPIO_NUM_38
#define XL9555_I2C_SCL          GPIO_NUM_48
#define XL9555_I2C_PORT         I2C_NUM_0
#define XL9555_I2C_ADDR         0x20
#define XL9555_AUDIO_EN_PIN     15      /* P1.7 - audio amplifier enable */
#define XL9555_BACKLIGHT_PIN    14      /* P1.6 - LCD backlight enable */

/* ---- Audio (ES8311 + NS4150B amplifier) ---- */
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

#define AUDIO_I2S_GPIO_MCLK     GPIO_NUM_45
#define AUDIO_I2S_GPIO_BCLK     GPIO_NUM_39
#define AUDIO_I2S_GPIO_WS       GPIO_NUM_41
#define AUDIO_I2S_GPIO_DOUT     GPIO_NUM_42
#define AUDIO_I2S_GPIO_DIN      GPIO_NUM_40
#define AUDIO_CODEC_ES8311_ADDR ES8311_CODEC_DEFAULT_ADDR
#define AUDIO_AMPLIFIER_EN      GPIO_NUM_NC  /* controlled via XL9555 pin 15 */

/* ---- I2C Master (for camera SCCB + board sensors) ---- */
#define I2C_MASTER_PORT         I2C_NUM_1
#define I2C_MASTER_SDA          GPIO_NUM_4
#define I2C_MASTER_SCL          GPIO_NUM_5

/* ---- Camera (GC2145, DVP 8-bit) ---- */
#define CAMERA_XCLK_FREQ        10000000
#define CAMERA_PIN_D0           GPIO_NUM_11
#define CAMERA_PIN_D1           GPIO_NUM_9
#define CAMERA_PIN_D2           GPIO_NUM_8
#define CAMERA_PIN_D3           GPIO_NUM_10
#define CAMERA_PIN_D4           GPIO_NUM_12
#define CAMERA_PIN_D5           GPIO_NUM_18
#define CAMERA_PIN_D6           GPIO_NUM_17
#define CAMERA_PIN_D7           GPIO_NUM_16
#define CAMERA_PIN_XCLK         GPIO_NUM_15
#define CAMERA_PIN_PCLK         GPIO_NUM_13
#define CAMERA_PIN_VSYNC        GPIO_NUM_6
#define CAMERA_PIN_HREF         GPIO_NUM_7
#define CAMERA_PIN_SIOD         GPIO_NUM_NC  /* camera on shared I2C_MASTER bus */
#define CAMERA_PIN_SIOC         GPIO_NUM_NC
#define CAMERA_PIN_PWDN         GPIO_NUM_NC
#define CAMERA_PIN_RESET        GPIO_NUM_NC

/* ---- Boot Button ---- */
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

#endif // _BOARD_CONFIG_H_
