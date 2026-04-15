#include <cstdint>
#include <string>

#include <esp_heap_caps.h>
#include <cstdio>
#include <cstring>
#include <esp_log.h>
#include <img_converters.h>

#include "esp32_camera.h"
#include "board.h"
#include "system_info.h"
#include "jpg/image_to_jpeg.h"
#include "esp_timer.h"

#define TAG "Esp32Camera"

Esp32Camera::Esp32Camera(const camera_config_t &config) {
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        if (s->id.PID == GC0308_PID) {
            s->set_hmirror(s, 0);
        }
        ESP_LOGI(TAG, "Camera initialized: format=%d", config.pixel_format);
    }

    streaming_on_ = true;
}

Esp32Camera::~Esp32Camera() {
    if (streaming_on_) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
            current_fb_ = nullptr;
        }
        if (encode_buf_) {
            heap_caps_free(encode_buf_);
            encode_buf_ = nullptr;
            encode_buf_size_ = 0;
        }
        esp_camera_deinit();
        streaming_on_ = false;
    }
}

void Esp32Camera::SetExplainUrl(const std::string &url, const std::string &token,
                                const std::string &session_id) {
    explain_url_ = url;
    explain_token_ = token;
    explain_session_id_ = session_id;
}

bool Esp32Camera::Capture() {
    if (!streaming_on_) {
        return false;
    }

    for (int i = 0; i < 2; i++) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
        }
        current_fb_ = esp_camera_fb_get();
        if (!current_fb_) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    if (current_fb_->format == PIXFORMAT_RGB565) {
        size_t pixel_count = current_fb_->width * current_fb_->height;
        size_t data_size = pixel_count * 2;

        if (encode_buf_size_ < data_size) {
            if (encode_buf_) {
                heap_caps_free(encode_buf_);
            }
            encode_buf_ = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (encode_buf_ == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate memory for encode buffer");
                encode_buf_size_ = 0;
                return false;
            }
            encode_buf_size_ = data_size;
        }

        uint16_t *src = (uint16_t *)current_fb_->buf;
        uint16_t *dst = (uint16_t *)encode_buf_;
        if (swap_bytes_enabled_) {
            for (size_t i = 0; i < pixel_count; i++) {
                dst[i] = __builtin_bswap16(src[i]);
            }
        } else {
            memcpy(encode_buf_, current_fb_->buf, data_size);
        }
    } else if (current_fb_->format == PIXFORMAT_JPEG) {
        ESP_LOGW(TAG, "JPEG capture success, len=%zu, but not supported for preview", current_fb_->len);
    }

    ESP_LOGI(TAG, "Captured frame: %dx%d, len=%zu, format=%d",
             current_fb_->width, current_fb_->height, current_fb_->len, current_fb_->format);

    return true;
}

bool Esp32Camera::SetHMirror(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_hmirror(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetVFlip(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_vflip(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetSwapBytes(bool enabled) {
    swap_bytes_enabled_ = enabled;
    return true;
}

std::string Esp32Camera::CaptureAndExplain(const std::string &question) {
    std::lock_guard<std::mutex> lock{capture_mutex_};
    if (!Capture()) {
        ESP_LOGE(TAG, "Failed to capture photo");
        return "";
    }
    return Explain(question);
}

bool Esp32Camera::HasExplainUrl() const {
    return !explain_url_.empty();
}

std::string Esp32Camera::CaptureAndExplainToEndpoint(const std::string &endpoint_path, const std::string &question) {
    std::lock_guard<std::mutex> lock{capture_mutex_};
    if (explain_url_.empty()) {
        ESP_LOGE(TAG, "Image explain URL or token is not set");
        return "";
    }
    if (!Capture()) {
        ESP_LOGE(TAG, "Failed to capture photo");
        return "";
    }

    auto scheme_end = explain_url_.find("://");
    if (scheme_end == std::string::npos) {
        ESP_LOGE(TAG, "Invalid explain URL format");
        return "";
    }
    auto path_start = explain_url_.find('/', scheme_end + 3);
    std::string base_url = (path_start != std::string::npos) ? explain_url_.substr(0, path_start) : explain_url_;

    std::string saved_url = explain_url_;
    explain_url_ = base_url + endpoint_path;
    std::string result = Explain(question);
    explain_url_ = saved_url;
    return result;
}

std::string Esp32Camera::Explain(const std::string &question) {
    if (explain_url_.empty()) {
        ESP_LOGE(TAG, "Image explain URL or token is not set");
        return "";
    }

    if (current_fb_ == nullptr) {
        ESP_LOGE(TAG, "No camera frame captured");
        return "";
    }

    int64_t encode_start = esp_timer_get_time();
    uint16_t w = current_fb_->width;
    uint16_t h = current_fb_->height;
    v4l2_pix_fmt_t enc_fmt;
    switch (current_fb_->format) {
        case PIXFORMAT_RGB565:    enc_fmt = V4L2_PIX_FMT_RGB565; break;
        case PIXFORMAT_YUV422:    enc_fmt = V4L2_PIX_FMT_YUYV; break;
        case PIXFORMAT_YUV420:    enc_fmt = V4L2_PIX_FMT_YUV420; break;
        case PIXFORMAT_GRAYSCALE: enc_fmt = V4L2_PIX_FMT_GREY; break;
        case PIXFORMAT_JPEG:      enc_fmt = V4L2_PIX_FMT_JPEG; break;
        case PIXFORMAT_RGB888:    enc_fmt = V4L2_PIX_FMT_RGB24; break;
        default:
            ESP_LOGE(TAG, "Unsupported pixel format: %d", current_fb_->format);
            return "";
    }

    uint8_t *jpeg_src_buf = current_fb_->buf;
    size_t jpeg_src_len = current_fb_->len;
    if (current_fb_->format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
        jpeg_src_buf = encode_buf_;
        jpeg_src_len = encode_buf_size_;
    }

    std::string jpeg_data;
    bool encode_ok = image_to_jpeg_cb(
        jpeg_src_buf, jpeg_src_len, w, h, enc_fmt, 80,
        [](void* arg, size_t index, const void* data, size_t len) -> size_t {
            auto* output = static_cast<std::string*>(arg);
            if (data != nullptr && len > 0) {
                output->append(static_cast<const char*>(data), len);
            }
            return len;
        },
        &jpeg_data);

    int64_t encode_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPEG encoding time: %ld ms, size: %d",
             static_cast<long>((encode_end - encode_start) / 1000),
             static_cast<int>(jpeg_data.size()));

    if (!encode_ok || jpeg_data.empty()) {
        ESP_LOGE(TAG, "JPEG encoder failed or produced empty output");
        return "";
    }

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";

    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_session_id_.empty()) {
        http->SetHeader("Session-Id", explain_session_id_);
    }
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }

    std::string body;
    body.reserve(question.size() + jpeg_data.size() + 512);
    body += "--" + boundary + "\r\n";
    body += "Content-Disposition: form-data; name=\"question\"\r\n";
    body += "\r\n";
    body += question + "\r\n";
    body += "--" + boundary + "\r\n";
    body += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
    body += "Content-Type: image/jpeg\r\n";
    body += "\r\n";
    body.append(jpeg_data.data(), jpeg_data.size());
    body += "\r\n--" + boundary + "--\r\n";

    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Content-Length", std::to_string(body.size()));
    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        return "";
    }

    http->Write(body.data(), body.size());

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        http->Close();
        return "";
    }

    std::string result = http->ReadAll();
    http->Close();

    size_t remain_stack_size = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "Explain image size=%dx%d, compressed size=%d, remain stack size=%d, question=%s\n%s",
             current_fb_->width, current_fb_->height, static_cast<int>(jpeg_data.size()),
             static_cast<int>(remain_stack_size), question.c_str(), result.c_str());

    esp_camera_fb_return(current_fb_);
    current_fb_ = nullptr;
    return result;
}
