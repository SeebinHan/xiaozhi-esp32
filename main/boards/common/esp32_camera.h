#pragma once

#include <cstddef>
#include <mutex>
#include <string>

#include "camera.h"
#include "esp_camera.h"

class Esp32Camera : public Camera {
private:
    bool streaming_on_ = false;
    bool swap_bytes_enabled_ = true;
    std::string explain_url_;
    std::string explain_token_;
    std::string explain_session_id_;
    std::mutex capture_mutex_;
    camera_fb_t* current_fb_ = nullptr;
    uint8_t* encode_buf_ = nullptr;
    size_t encode_buf_size_ = 0;

public:
    explicit Esp32Camera(const camera_config_t& config);
    ~Esp32Camera();

    void SetExplainUrl(const std::string& url, const std::string& token,
                       const std::string& session_id) override;
    bool Capture() override;
    bool SetHMirror(bool enabled) override;
    bool SetVFlip(bool enabled) override;
    bool SetSwapBytes(bool enabled) override;
    std::string Explain(const std::string& question) override;
    std::string CaptureAndExplain(const std::string& question) override;
    std::string CaptureAndExplainToEndpoint(const std::string& endpoint_path,
                                           const std::string& question) override;
    bool HasExplainUrl() const override;
    bool IsAvailable() const override { return streaming_on_; }
};
