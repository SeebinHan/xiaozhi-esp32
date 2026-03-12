/*
 * 薄膜压力传感器驱动实现
 * 使用 ADC1（GPIO 1-10），不与 WiFi 冲突，读取稳定可靠
 */

#include "touch_sensor.h"
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>

static const char* TAG = "TouchSensor";

TouchSensor::TouchSensor(gpio_num_t adc_pin, adc_channel_t channel)
    : pin_(adc_pin), channel_(channel) {}

TouchSensor::~TouchSensor() {
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
}

void TouchSensor::Initialize() {
    ESP_LOGI(TAG, "Initializing touch sensor on GPIO %d, ADC1 channel %d", pin_, channel_);

    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle_));

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, channel_, &chan_cfg));

    ESP_LOGI(TAG, "Touch sensor ready");
}

int TouchSensor::ReadRaw() {
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle_, channel_, &raw);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return raw;
}

TouchLevel TouchSensor::ReadLevel() {
    int raw = ReadRaw();
    /* 反向逻辑：无压力=4095，按下时值下降 */
    if (raw >= kIdleMin) {
        return TouchLevel::kNone;
    } else if (raw >= kThresholdLight) {
        return TouchLevel::kLight;       /* 3000~3800 */
    } else if (raw >= kThresholdHard) {
        return TouchLevel::kMedium;      /* 1500~3000 */
    } else {
        return TouchLevel::kHard;        /* <1500 */
    }
}
