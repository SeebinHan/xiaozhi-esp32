/*
 * GC9D01 双眼圆形 LCD 驱动实现
 * 仿真猫眼渲染：竖瞳、虹膜纤维纹理、眼睑、多高光、抗锯齿
 */

#include "cat_eye_display.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "CatEye";

/* ================================================================
 *  Math helpers
 * ================================================================ */
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float smoothstep(float edge0, float edge1, float x) {
    float t = (x - edge0) / (edge1 - edge0);
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

CatEyeDisplay::CatEyeDisplay() {}

CatEyeDisplay::~CatEyeDisplay() {
    if (frame_buf_) {
        free(frame_buf_);
    }
}

/* ================================================================
 *  Low-level SPI
 * ================================================================ */
void CatEyeDisplay::LcdCmd(spi_device_handle_t spi, uint8_t cmd) {
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(EYE_PIN_DC, 0);
    spi_device_polling_transmit(spi, &t);
}

void CatEyeDisplay::LcdData(spi_device_handle_t spi, const uint8_t* data, int len) {
    if (len == 0) return;
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(EYE_PIN_DC, 1);
    spi_device_polling_transmit(spi, &t);
}

void CatEyeDisplay::LcdDataByte(spi_device_handle_t spi, uint8_t val) {
    LcdData(spi, &val, 1);
}

/* ================================================================
 *  GC9D01 init sequence (from vendor code)
 * ================================================================ */
void CatEyeDisplay::Gc9d01Init(spi_device_handle_t spi) {
    LcdCmd(spi, 0xFE);
    LcdCmd(spi, 0xEF);

    for (uint8_t reg = 0x80; reg <= 0x8F; reg++) {
        LcdCmd(spi, reg);
        LcdDataByte(spi, 0xFF);
    }

    LcdCmd(spi, 0x3A); LcdDataByte(spi, 0x05);
    LcdCmd(spi, 0xEC); LcdDataByte(spi, 0x01);

    LcdCmd(spi, 0x74);
    { uint8_t d[] = {0x02,0x0E,0x00,0x00,0x00,0x00,0x00}; LcdData(spi, d, sizeof(d)); }

    LcdCmd(spi, 0x98); LcdDataByte(spi, 0x3E);
    LcdCmd(spi, 0x99); LcdDataByte(spi, 0x3E);

    LcdCmd(spi, 0xB5); { uint8_t d[]={0x0D,0x0D}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x60); { uint8_t d[]={0x38,0x0F,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x61); { uint8_t d[]={0x38,0x11,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x64); { uint8_t d[]={0x38,0x17,0x71,0x5F,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x65); { uint8_t d[]={0x38,0x13,0x71,0x5B,0x79,0x67}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x6A); { uint8_t d[]={0x00,0x00}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x6C); { uint8_t d[]={0x22,0x02,0x22,0x02,0x22,0x22,0x50}; LcdData(spi,d,sizeof(d)); }

    LcdCmd(spi, 0x6E);
    { uint8_t d[]={0x03,0x03,0x01,0x01,0x00,0x00,0x0F,0x0F,
                   0x0D,0x0D,0x0B,0x0B,0x09,0x09,0x00,0x00,
                   0x00,0x00,0x0A,0x0A,0x0C,0x0C,0x0E,0x0E,
                   0x10,0x10,0x00,0x00,0x02,0x02,0x04,0x04};
      LcdData(spi, d, sizeof(d)); }

    LcdCmd(spi, 0xBF); LcdDataByte(spi, 0x01);
    LcdCmd(spi, 0xF9); LcdDataByte(spi, 0x40);
    LcdCmd(spi, 0x9B); LcdDataByte(spi, 0x3B);
    LcdCmd(spi, 0x93); { uint8_t d[]={0x33,0x7F,0x00}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x7E); LcdDataByte(spi, 0x30);
    LcdCmd(spi, 0x70); { uint8_t d[]={0x0D,0x02,0x08,0x0D,0x02,0x08}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x71); { uint8_t d[]={0x0D,0x02,0x08}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0x91); { uint8_t d[]={0x0E,0x09}; LcdData(spi,d,sizeof(d)); }

    LcdCmd(spi, 0xC3); LcdDataByte(spi, 0x19);
    LcdCmd(spi, 0xC4); LcdDataByte(spi, 0x19);
    LcdCmd(spi, 0xC9); LcdDataByte(spi, 0x3C);

    LcdCmd(spi, 0xF0); { uint8_t d[]={0x53,0x15,0x0A,0x04,0x00,0x3E}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0xF2); { uint8_t d[]={0x53,0x15,0x0A,0x04,0x00,0x3A}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0xF1); { uint8_t d[]={0x56,0xA8,0x7F,0x33,0x34,0x5F}; LcdData(spi,d,sizeof(d)); }
    LcdCmd(spi, 0xF3); { uint8_t d[]={0x52,0xA4,0x7F,0x33,0x34,0xDF}; LcdData(spi,d,sizeof(d)); }

    LcdCmd(spi, 0x36); LcdDataByte(spi, 0x00);

    LcdCmd(spi, 0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    LcdCmd(spi, 0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
}

/* ================================================================
 *  Drawing helpers
 * ================================================================ */
void CatEyeDisplay::SetWindow(spi_device_handle_t spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LcdCmd(spi, 0x2A);
    uint8_t col[] = {(uint8_t)(x0>>8),(uint8_t)x0,(uint8_t)(x1>>8),(uint8_t)x1};
    LcdData(spi, col, 4);
    LcdCmd(spi, 0x2B);
    uint8_t row[] = {(uint8_t)(y0>>8),(uint8_t)y0,(uint8_t)(y1>>8),(uint8_t)y1};
    LcdData(spi, row, 4);
    LcdCmd(spi, 0x2C);
}

void CatEyeDisplay::SendFrameBuffer(spi_device_handle_t spi) {
    SetWindow(spi, 0, 0, EYE_WIDTH - 1, EYE_HEIGHT - 1);
    gpio_set_level(EYE_PIN_DC, 1);
    const int chunk_lines = 10;
    for (int y = 0; y < EYE_HEIGHT; y += chunk_lines) {
        int lines = (y + chunk_lines <= EYE_HEIGHT) ? chunk_lines : (EYE_HEIGHT - y);
        spi_transaction_t t = {};
        t.length = EYE_WIDTH * lines * 16;
        t.tx_buffer = &frame_buf_[y * EYE_WIDTH];
        spi_device_polling_transmit(spi, &t);
    }
}

void CatEyeDisplay::FillColor(spi_device_handle_t spi, uint16_t color) {
    uint16_t sw = SwapBytes(color);
    for (int i = 0; i < EYE_WIDTH * EYE_HEIGHT; i++) {
        frame_buf_[i] = sw;
    }
    SendFrameBuffer(spi);
}

/* ================================================================
 *  仿真猫眼渲染
 *
 *  特性：
 *  - 竖直椭圆瞳孔（猫瞳标志性特征）
 *  - 虹膜径向纤维纹理 + 暗环（limbal ring）
 *  - 瞳孔边缘发光
 *  - 双高光（主高光 + 副高光）带柔和衰减
 *  - 上下眼睑曲线 + 眼睑边缘阴影
 *  - 全局抗锯齿（显示边缘、眼睑边缘、瞳孔边缘）
 *  - 深色毛皮背景
 *
 *  参数：
 *    iris_r/g/b      虹膜基础色 (0-255)
 *    pupil_dilation  瞳孔扩张 0.0=极细缝 ~ 1.0=圆形全开
 *    look_x/y        注视方向 -1.0~+1.0
 *    upper_lid       上眼睑开度 0.0=完全闭合 ~ 1.0=完全张开
 *    lower_lid       下眼睑开度
 *    brow_anger      眉毛倾斜 0.0=正常 ~ 1.0=愤怒下压
 * ================================================================ */
void CatEyeDisplay::DrawCatEye(uint8_t iris_r, uint8_t iris_g, uint8_t iris_b,
                                float pupil_dilation, float look_x, float look_y,
                                float upper_lid, float lower_lid, float brow_anger) {
    const float CX = 80.0f, CY = 80.0f;
    const float DISP_R = 78.0f;

    /* 注视偏移 */
    const float gx = CX + look_x * 12.0f;
    const float gy = CY + look_y * 10.0f;

    /* 虹膜参数 */
    const float IRIS_R = 63.0f;
    const float LIMBAL_START = 0.80f;

    /* 瞳孔：竖直椭圆 */
    const float PH = 54.0f;
    const float PW_MIN = 3.0f;
    const float PW_MAX = 30.0f;
    const float pw = PW_MIN + pupil_dilation * (PW_MAX - PW_MIN);

    /* 双高光位置 */
    const float HL1_X = gx - 20.0f, HL1_Y = gy - 22.0f, HL1_R = 14.0f;
    const float HL2_X = gx + 15.0f, HL2_Y = gy + 18.0f, HL2_R = 7.0f;

    /* 眼睑几何 */
    const float EYE_HW = 74.0f;
    const float UPPER_MAX = 60.0f;
    const float LOWER_MAX = 50.0f;
    const float u_open = UPPER_MAX * clampf(upper_lid, 0, 1);
    const float l_open = LOWER_MAX * clampf(lower_lid, 0, 1);

    /* 毛皮色（深紫灰） */
    const uint8_t FUR_R = 195, FUR_G = 185, FUR_B = 175;

    for (int y = 0; y < EYE_HEIGHT; y++) {
        for (int x = 0; x < EYE_WIDTH; x++) {
            float dx = (float)x - CX;
            float dy = (float)y - CY;
            float dist2 = dx * dx + dy * dy;

            /* 圆形显示边界外 → 纯黑 */
            if (dist2 > (DISP_R + 1.0f) * (DISP_R + 1.0f)) {
                frame_buf_[y * EYE_WIDTH + x] = 0;
                continue;
            }
            float dist = sqrtf(dist2);
            float disp_aa = smoothstep(DISP_R + 0.5f, DISP_R - 1.0f, dist);

            /* 眼睑曲线 */
            float nx = dx / EYE_HW;
            float nx2 = nx * nx;
            float upper_y = CY - u_open + u_open * nx2 * 0.9f
                            + brow_anger * dx * 0.18f;
            float lower_y = CY + l_open - l_open * nx2 * 0.7f;

            bool in_h = fabsf(dx) < EYE_HW;
            bool in_eye = in_h && (float)y > upper_y && (float)y < lower_y;

            uint8_t pr, pg, pb;

            if (!in_eye) {
                /* ---- 眼睑/毛皮区域 ---- */
                float eu = fabsf((float)y - upper_y);
                float el = fabsf((float)y - lower_y);
                float edge = fminf(eu, el);

                /* 眼睑边缘微微发亮（模拟眼睑反光） */
                if (edge < 3.5f && in_h) {
                    float rim = smoothstep(3.5f, 0.5f, edge) * 0.45f;
                    pr = (uint8_t)(FUR_R + rim * 50);
                    pg = (uint8_t)(FUR_G + rim * 40);
                    pb = (uint8_t)(FUR_B + rim * 30);
                } else {
                    pr = FUR_R; pg = FUR_G; pb = FUR_B;
                }
                pr = (uint8_t)(pr * disp_aa);
                pg = (uint8_t)(pg * disp_aa);
                pb = (uint8_t)(pb * disp_aa);
                frame_buf_[y * EYE_WIDTH + x] = SwapBytes(Rgb565(pr, pg, pb));
                continue;
            }

            /* ---- 眼球内部 ---- */
            float idx = (float)x - gx;
            float idy = (float)y - gy;
            float iris_dist2 = idx * idx + idy * idy;
            float iris_dist = sqrtf(iris_dist2);
            float iris_n = iris_dist / IRIS_R;

            if (iris_n > 1.0f) {
                /* 巩膜（白眼球，猫眼中只露出薄薄一圈） */
                float sf = smoothstep(1.15f, 1.0f, iris_n);
                pr = (uint8_t)(235 - sf * 35);
                pg = (uint8_t)(232 - sf * 32);
                pb = (uint8_t)(228 - sf * 28);
            } else {
                /* ---- 虹膜渲染 ---- */
                float angle = atan2f(idy, idx);

                /* 径向纤维纹理（柔和细腻，匹配玩偶猫眼质感） */
                float fiber1 = sinf(angle * 11.0f) * 0.06f;
                float fiber2 = sinf(angle * 5.0f + 1.5f) * 0.04f;
                float fibers = fiber1 + fiber2;

                /* 径向渐变：内亮外暗 */
                float radial = 1.0f - iris_n * 0.3f + fibers;

                /* 瞳孔周围发光环（内圈更亮） */
                float inner_ring = smoothstep(0.45f, 0.18f, iris_n) * 0.25f;
                radial += inner_ring;

                /* Limbal ring（虹膜外缘暗环，真实眼睛特征） */
                float limbal = smoothstep(LIMBAL_START, 0.97f, iris_n);

                radial = clampf(radial, 0.0f, 1.4f);
                float dark = 1.0f - limbal * 0.65f;

                pr = (uint8_t)clampf(iris_r * radial * dark, 0, 255);
                pg = (uint8_t)clampf(iris_g * radial * dark, 0, 255);
                pb = (uint8_t)clampf(iris_b * radial * dark + limbal * 10, 0, 255);

                /* ---- 瞳孔（竖直椭圆，猫眼标志） ---- */
                float pxn = idx / pw;
                float pyn = idy / PH;
                float pdist = pxn * pxn + pyn * pyn;

                if (pdist < 1.0f) {
                    /* 瞳孔内部 → 黑色，边缘平滑过渡 */
                    float pupil_aa = smoothstep(0.80f, 1.0f, pdist);
                    pr = (uint8_t)(pr * pupil_aa);
                    pg = (uint8_t)(pg * pupil_aa);
                    pb = (uint8_t)(pb * pupil_aa);
                }

                /* 瞳孔边缘微光（虹膜色向瞳孔边缘渗透） */
                float pglow = smoothstep(1.35f, 1.0f, pdist) *
                              smoothstep(0.72f, 1.0f, pdist) * 0.22f;
                if (pglow > 0.01f) {
                    pr = (uint8_t)clampf(pr + pglow * iris_r, 0, 255);
                    pg = (uint8_t)clampf(pg + pglow * iris_g, 0, 255);
                    pb = (uint8_t)clampf(pb + pglow * iris_b * 0.5f, 0, 255);
                }
            }

            /* ---- 高光（镜面反射，赋予眼睛灵气） ---- */
            /* 主高光：大而明亮，左上方 */
            float h1d2 = ((float)x - HL1_X) * ((float)x - HL1_X) +
                         ((float)y - HL1_Y) * ((float)y - HL1_Y);
            if (h1d2 < HL1_R * HL1_R) {
                float f = smoothstep(HL1_R, HL1_R * 0.25f, sqrtf(h1d2));
                pr = (uint8_t)clampf(pr + f * (255 - pr), 0, 255);
                pg = (uint8_t)clampf(pg + f * (255 - pg), 0, 255);
                pb = (uint8_t)clampf(pb + f * (255 - pb), 0, 255);
            }
            /* 副高光：小而柔和，右下方 */
            float h2d2 = ((float)x - HL2_X) * ((float)x - HL2_X) +
                         ((float)y - HL2_Y) * ((float)y - HL2_Y);
            if (h2d2 < HL2_R * HL2_R) {
                float f = smoothstep(HL2_R, HL2_R * 0.15f, sqrtf(h2d2)) * 0.6f;
                pr = (uint8_t)clampf(pr + f * (240 - pr), 0, 255);
                pg = (uint8_t)clampf(pg + f * (240 - pg), 0, 255);
                pb = (uint8_t)clampf(pb + f * (240 - pb), 0, 255);
            }

            /* ---- 眼睑投射阴影 ---- */
            float u_shadow = smoothstep(10.0f, 0.0f, (float)y - upper_y) * 0.35f;
            float l_shadow = smoothstep(6.0f, 0.0f, lower_y - (float)y) * 0.2f;
            float shadow = fmaxf(u_shadow, l_shadow);
            pr = (uint8_t)(pr * (1.0f - shadow));
            pg = (uint8_t)(pg * (1.0f - shadow));
            pb = (uint8_t)(pb * (1.0f - shadow));

            /* ---- 眼睑边缘抗锯齿 ---- */
            float aa_u = smoothstep(upper_y - 0.8f, upper_y + 1.2f, (float)y);
            float aa_l = smoothstep(lower_y + 0.8f, lower_y - 1.2f, (float)y);
            float aa = fminf(aa_u, aa_l);
            pr = (uint8_t)(pr * aa + FUR_R * (1.0f - aa));
            pg = (uint8_t)(pg * aa + FUR_G * (1.0f - aa));
            pb = (uint8_t)(pb * aa + FUR_B * (1.0f - aa));

            /* 显示边缘 */
            pr = (uint8_t)(pr * disp_aa);
            pg = (uint8_t)(pg * disp_aa);
            pb = (uint8_t)(pb * disp_aa);

            frame_buf_[y * EYE_WIDTH + x] = SwapBytes(Rgb565(pr, pg, pb));
        }
    }
}

/* ================================================================
 *  开心眯眼 - 弯弯的弧线（^_^ 表情）
 *  厚实的发光弧线 + 眼睑下方微亮区域 + 毛皮背景
 * ================================================================ */
void CatEyeDisplay::DrawClosedEye() {
    const float CX = 80.0f, CY = 80.0f;
    const float DISP_R = 78.0f;
    const uint8_t FUR_R = 195, FUR_G = 185, FUR_B = 175;

    for (int y = 0; y < EYE_HEIGHT; y++) {
        for (int x = 0; x < EYE_WIDTH; x++) {
            float dx = (float)x - CX;
            float dy = (float)y - CY;
            float dist2 = dx * dx + dy * dy;

            if (dist2 > (DISP_R + 1.0f) * (DISP_R + 1.0f)) {
                frame_buf_[y * EYE_WIDTH + x] = 0;
                continue;
            }
            float dist = sqrtf(dist2);
            float disp_aa = smoothstep(DISP_R + 0.5f, DISP_R - 1.0f, dist);

            /* 弧线: 向上弯的 cos 曲线，模拟眯眼笑 */
            float arc_y = CY - 14.0f * cosf(dx * 3.14159f / 65.0f);
            float dist_to_arc = (float)y - arc_y;
            float abs_dist = fabsf(dist_to_arc);
            float abs_dx = fabsf(dx);

            uint8_t pr, pg, pb;

            if (abs_dist < 5.0f && abs_dx < 62.0f) {
                /* 主弧线 - 厚实发光 */
                float line_f = smoothstep(5.0f, 1.0f, abs_dist);
                /* 端部淡出 */
                float end_fade = smoothstep(62.0f, 48.0f, abs_dx);
                float intensity = line_f * end_fade;
                /* 弧线颜色：柔和蓝白色，中心最亮 */
                pr = (uint8_t)(FUR_R + intensity * (235 - FUR_R));
                pg = (uint8_t)(FUR_G + intensity * (240 - FUR_G));
                pb = (uint8_t)(FUR_B + intensity * (250 - FUR_B));
            } else if (dist_to_arc > 0 && dist_to_arc < 22.0f && abs_dx < 58.0f) {
                /* 弧线下方（眼睑内侧）- 微微发亮，暗示闭着的眼球 */
                float inner_f = smoothstep(22.0f, 2.0f, dist_to_arc);
                float h_fade = smoothstep(58.0f, 40.0f, abs_dx);
                float glow = inner_f * h_fade * 0.25f;
                pr = (uint8_t)(FUR_R + glow * 40);
                pg = (uint8_t)(FUR_G + glow * 35);
                pb = (uint8_t)(FUR_B + glow * 20);
            } else {
                pr = FUR_R; pg = FUR_G; pb = FUR_B;
            }

            pr = (uint8_t)(pr * disp_aa);
            pg = (uint8_t)(pg * disp_aa);
            pb = (uint8_t)(pb * disp_aa);

            frame_buf_[y * EYE_WIDTH + x] = SwapBytes(Rgb565(pr, pg, pb));
        }
    }
}

/* ================================================================
 *  爱心眼 - 心形 + 内部渐变高光 + 外部发光
 * ================================================================ */
void CatEyeDisplay::DrawHeartEye() {
    const float CX = 80.0f, CY = 80.0f;
    const float DISP_R = 78.0f;
    const uint8_t FUR_R = 195, FUR_G = 185, FUR_B = 175;

    for (int y = 0; y < EYE_HEIGHT; y++) {
        for (int x = 0; x < EYE_WIDTH; x++) {
            float dx = (float)x - CX;
            float dy = (float)y - CY;
            float dist2 = dx * dx + dy * dy;

            if (dist2 > (DISP_R + 1.0f) * (DISP_R + 1.0f)) {
                frame_buf_[y * EYE_WIDTH + x] = 0;
                continue;
            }
            float dist = sqrtf(dist2);
            float disp_aa = smoothstep(DISP_R + 0.5f, DISP_R - 1.0f, dist);

            /* 心形公式 */
            float fx = dx / 38.0f;
            float fy = (float)(y - 88) / -38.0f;
            float heart = (fx * fx + fy * fy - 1.0f);
            heart = heart * heart * heart - fx * fx * fy * fy * fy;

            uint8_t pr, pg, pb;

            if (heart <= -0.02f) {
                /* 心形内部 - 带高光渐变 */
                float depth = clampf(-heart * 3.0f, 0, 1);
                /* 左上高光 */
                float hl_x = (fx + 0.35f);
                float hl_y = (fy - 0.35f);
                float hl = smoothstep(0.7f, 0.0f, sqrtf(hl_x * hl_x + hl_y * hl_y));
                pr = (uint8_t)clampf(195 + depth * 60 + hl * 60, 0, 255);
                pg = (uint8_t)clampf(15 + depth * 15 + hl * 90, 0, 255);
                pb = (uint8_t)clampf(45 + depth * 15 + hl * 70, 0, 255);
            } else if (heart <= 0.08f) {
                /* 边缘抗锯齿 */
                float edge_f = smoothstep(0.08f, -0.02f, heart);
                pr = (uint8_t)(FUR_R + edge_f * (220 - FUR_R));
                pg = (uint8_t)(FUR_G + edge_f * (30 - FUR_G));
                pb = (uint8_t)(FUR_B + edge_f * (50 - FUR_B));
            } else if (heart <= 0.35f) {
                /* 心形外圈光晕 */
                float glow = smoothstep(0.35f, 0.08f, heart) * 0.25f;
                pr = (uint8_t)(FUR_R + glow * 90);
                pg = (uint8_t)(FUR_G + glow * 10);
                pb = (uint8_t)(FUR_B + glow * 18);
            } else {
                pr = FUR_R; pg = FUR_G; pb = FUR_B;
            }

            pr = (uint8_t)(pr * disp_aa);
            pg = (uint8_t)(pg * disp_aa);
            pb = (uint8_t)(pb * disp_aa);

            frame_buf_[y * EYE_WIDTH + x] = SwapBytes(Rgb565(pr, pg, pb));
        }
    }
}

/* ================================================================
 *  眨眼动画系统
 *
 *  原理：后台任务每 3~6 秒触发一次眨眼
 *  眨眼 = 上眼睑快速闭合再慢速恢复（不对称，模拟真实眨眼）
 *  通过 mutex 保护 frame_buf_ 和 SPI，确保线程安全
 *  SetEmotion 可随时打断眨眼（blink_stop_ 标记）
 * ================================================================ */
void CatEyeDisplay::RenderAndSend() {
    /* 仅对普通猫眼状态有效 */
    DrawCatEye(params_.ir, params_.ig, params_.ib,
               params_.dilation, params_.lx, params_.ly,
               params_.upper, params_.lower, params_.anger);
    SendFrameBuffer(spi_left_);
    SendFrameBuffer(spi_right_);
}

void CatEyeDisplay::DoBlink() {
    if (eye_type_ != kEyeNormal) return;

    EyeParams& p = params_;
    float orig_upper = p.upper;
    float orig_lower = p.lower;

    /* 闭眼阶段（快，3帧，主要动上眼睑） */
    static const float kClose[] = {0.55f, 0.12f, 0.0f};
    for (int i = 0; i < 3; i++) {
        if (blink_stop_) goto restore;
        xSemaphoreTake(render_mtx_, portMAX_DELAY);
        if (blink_stop_) { xSemaphoreGive(render_mtx_); goto restore; }
        p.upper = orig_upper * kClose[i];
        p.lower = orig_lower * (0.6f + 0.4f * kClose[i]);
        RenderAndSend();
        xSemaphoreGive(render_mtx_);
    }

    /* 闭合停留 */
    vTaskDelay(pdMS_TO_TICKS(35));

    /* 睁眼阶段（慢，4帧） */
    {
        static const float kOpen[] = {0.15f, 0.45f, 0.78f, 1.0f};
        for (int i = 0; i < 4; i++) {
            if (blink_stop_) goto restore;
            xSemaphoreTake(render_mtx_, portMAX_DELAY);
            if (blink_stop_) { xSemaphoreGive(render_mtx_); goto restore; }
            p.upper = orig_upper * kOpen[i];
            p.lower = orig_lower * (0.6f + 0.4f * kOpen[i]);
            RenderAndSend();
            xSemaphoreGive(render_mtx_);
        }
    }

restore:
    p.upper = orig_upper;
    p.lower = orig_lower;
}

void CatEyeDisplay::BlinkTaskEntry(void* arg) {
    auto* self = static_cast<CatEyeDisplay*>(arg);
    /* 启动后等 2 秒再开始第一次眨眼 */
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (true) {
        /* 随机间隔 3~6 秒 */
        int delay_ms = 3000 + (rand() % 3000);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        self->DoBlink();
    }
}

/* ================================================================
 *  Initialize
 * ================================================================ */
void CatEyeDisplay::Initialize() {
    ESP_LOGI(TAG, "Initializing cat eye displays...");

    frame_buf_ = (uint16_t*)heap_caps_malloc(EYE_WIDTH * EYE_HEIGHT * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!frame_buf_) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer!");
        return;
    }

    /* 创建渲染互斥锁 */
    render_mtx_ = xSemaphoreCreateMutex();

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << EYE_PIN_DC) | (1ULL << EYE_PIN_RST);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level(EYE_PIN_DC, 0);
    gpio_set_level(EYE_PIN_RST, 1);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = EYE_PIN_MOSI;
    bus_cfg.miso_io_num = -1;
    bus_cfg.sclk_io_num = EYE_PIN_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = EYE_WIDTH * 10 * 2 + 8;
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI2 bus init failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = EYE_SPI_CLK_HZ;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = EYE_PIN_CS_L;
    dev_cfg.queue_size = 1;
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_left_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Left eye device add failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    dev_cfg.spics_io_num = EYE_PIN_CS_R;
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_right_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Right eye device add failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    gpio_set_level(EYE_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(EYE_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    Gc9d01Init(spi_left_);
    Gc9d01Init(spi_right_);

    SetEmotion("neutral");

    /* 启动眨眼后台任务 */
    xTaskCreate(BlinkTaskEntry, "eye_blink", 4096, this, 1, &blink_task_);

    ESP_LOGI(TAG, "Cat eye displays ready!");
}

/* ================================================================
 *  SetEmotion - 表情 → 眼睛参数映射
 *
 *  DrawCatEye(虹膜R, G, B, 瞳孔扩张, 视线X, 视线Y,
 *             上眼睑, 下眼睑, 眉毛怒气)
 * ================================================================ */
void CatEyeDisplay::SetEmotion(const std::string& emotion) {
    if (emotion == current_emotion_) return;
    current_emotion_ = emotion;

    ESP_LOGI(TAG, "Eye emotion: %s", emotion.c_str());

    /* 打断正在进行的眨眼动画 */
    blink_stop_ = true;
    xSemaphoreTake(render_mtx_, portMAX_DELAY);
    blink_stop_ = false;

    if (emotion == "happy" || emotion == "laughing" || emotion == "funny") {
        /* 真实感开心：瞳孔略放大，眼裂略柔和，轻微偏视线 */
        eye_type_ = kEyeNormal;
        params_ = {100, 150, 190, 0.78f, 0.18f, -0.05f, 0.88f, 0.98f, -0.05f};
        RenderAndSend();

    } else if (emotion == "love" || emotion == "heart_eyes") {
        /* 真实感喜欢：更柔和、更专注，避免爱心眼卡通效果 */
        eye_type_ = kEyeNormal;
        params_ = {110, 155, 195, 0.86f, 0.10f, -0.02f, 0.82f, 0.97f, -0.08f};
        RenderAndSend();

    } else if (emotion == "angry" || emotion == "hateful") {
        eye_type_ = kEyeNormal;
        params_ = {180, 90, 100, 0.15f, 0, 0, 0.7f, 0.85f, 0.75f};
        RenderAndSend();

    } else if (emotion == "sad" || emotion == "crying") {
        eye_type_ = kEyeNormal;
        params_ = {75, 120, 185, 0.75f, 0, 0.2f, 0.6f, 0.9f, -0.2f};
        RenderAndSend();

    } else if (emotion == "surprised" || emotion == "shocked") {
        /* 真实感惊讶：眼裂略开大、瞳孔稍收、视线定住，不做夸张卡通大眼 */
        eye_type_ = kEyeNormal;
        params_ = {105, 160, 205, 0.52f, 0.0f, -0.02f, 1.02f, 1.00f, 0.02f};
        RenderAndSend();

    } else if (emotion == "sleepy" || emotion == "tired") {
        eye_type_ = kEyeNormal;
        params_ = {80, 125, 165, 0.55f, 0, 0.1f, 0.3f, 0.35f, 0};
        RenderAndSend();

    } else if (emotion == "thinking" || emotion == "confused") {
        eye_type_ = kEyeNormal;
        params_ = {90, 140, 180, 0.6f, 0.4f, -0.35f, 0.85f, 0.95f, 0};
        RenderAndSend();

    } else if (emotion == "wink") {
        eye_type_ = kEyeClosed;
        DrawClosedEye();
        SendFrameBuffer(spi_left_);
        DrawCatEye(90, 140, 180, 0.65f, 0, 0, 0.9f, 0.95f, 0);
        SendFrameBuffer(spi_right_);

    } else {
        /* 默认待命：真实感中性眼神 */
        eye_type_ = kEyeNormal;
        params_ = {90, 140, 180, 0.7f, 0, 0, 0.95f, 0.98f, 0};
        RenderAndSend();
    }

    xSemaphoreGive(render_mtx_);
}
