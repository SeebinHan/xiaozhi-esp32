/*
 * GC9D01 双眼圆形 LCD 驱动
 * 用于猫咪 AI 伴侣的眼睛表情显示
 *
 * 接线（面包板 DevKitC-1）：
 *   MOSI  -> GPIO1   (两眼并联)
 *   SCLK  -> GPIO2   (两眼并联)
 *   CS左  -> GPIO3   (左眼独立)
 *   CS右  -> GPIO8   (右眼独立)
 *   DC    -> GPIO9   (两眼并联)
 *   RST   -> GPIO10  (两眼并联)
 *   BL    -> 3.3V    (背光常亮)
 */

#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <string>

/* Pin definitions */
#define EYE_PIN_MOSI    GPIO_NUM_1
#define EYE_PIN_SCLK    GPIO_NUM_2
#define EYE_PIN_CS_L    GPIO_NUM_3
#define EYE_PIN_CS_R    GPIO_NUM_8
#define EYE_PIN_DC      GPIO_NUM_9
#define EYE_PIN_RST     GPIO_NUM_10

#define EYE_WIDTH       160
#define EYE_HEIGHT      160
#define EYE_SPI_CLK_HZ  (20 * 1000 * 1000)

class CatEyeDisplay {
public:
    CatEyeDisplay();
    ~CatEyeDisplay();

    /* Initialize SPI bus and both eye displays */
    void Initialize();

    /* Set emotion - draws corresponding eye pattern on both displays */
    void SetEmotion(const std::string& emotion);

private:
    spi_device_handle_t spi_left_ = nullptr;
    spi_device_handle_t spi_right_ = nullptr;
    std::string current_emotion_;
    uint16_t* frame_buf_ = nullptr;  /* Full frame buffer for one eye */

    /* Low-level SPI */
    void LcdCmd(spi_device_handle_t spi, uint8_t cmd);
    void LcdData(spi_device_handle_t spi, const uint8_t* data, int len);
    void LcdDataByte(spi_device_handle_t spi, uint8_t val);

    /* GC9D01 init sequence */
    void Gc9d01Init(spi_device_handle_t spi);

    /* Drawing helpers */
    void SetWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void SendFrameBuffer(spi_device_handle_t spi);
    void FillColor(spi_device_handle_t spi, uint16_t color);

    /* Eye pattern generators - draw into frame_buf_ */
    void DrawCatEye(uint16_t iris_r, uint16_t iris_g, uint16_t iris_b,
                    float pupil_openness, float look_x, float look_y);
    void DrawClosedEye();
    void DrawHeartEye();

    static inline uint16_t Rgb565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
    static inline uint16_t SwapBytes(uint16_t c) {
        return (c >> 8) | (c << 8);
    }
};
