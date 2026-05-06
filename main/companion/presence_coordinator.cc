#include "companion/presence_coordinator.h"

#include <cstring>

#include <cJSON.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "PresenceCoordinator"

namespace {
constexpr const char* kGreetingVisionTaskName = "presence_greet";
constexpr uint32_t kVisionTaskStackSize = 4096 * 3;
constexpr UBaseType_t kWorkerTaskPriority = 2;
}

PresenceCoordinator::PresenceCoordinator(PresenceCoordinatorHost& host) : host_(host) {}

bool PresenceCoordinator::CanStartGreeting(const char*& reason) const {
    if (!host_.IsPresenceTransportReady()) {
        reason = "transport_not_ready";
        return false;
    }
    if (host_.IsHeavyLoadCooldownActiveForPresence()) {
        reason = "heavy_load_cooldown";
        return false;
    }
    if (host_.GetPresenceDeviceState() != kDeviceStateIdle) {
        reason = "state_not_idle";
        return false;
    }
    if (!host_.IsPresenceAudioIdle()) {
        reason = "audio_not_idle";
        return false;
    }
    if (host_.IsPresenceAudioProcessorRunning()) {
        reason = "audio_processor_running";
        return false;
    }
    if (!host_.IsPresencePlaybackDrained()) {
        reason = "playback_not_drained";
        return false;
    }
    if (host_.IsPresenceAudioBackpressured()) {
        reason = "audio_backpressured";
        return false;
    }
    if (greeting_in_progress_) {
        reason = "greeting_in_progress";
        return false;
    }
    if (last_greeting_time_us_ != 0 &&
        (esp_timer_get_time() - last_greeting_time_us_) < kGreetingCooldownUs) {
        reason = "cooldown_active";
        return false;
    }
    reason = "allowed";
    return true;
}

void PresenceCoordinator::MaybeLogSkippedDetection(const char* reason) {
    int64_t now = esp_timer_get_time();
    DeviceState state = host_.GetPresenceDeviceState();
    bool same_reason = last_skip_reason_ != nullptr && reason != nullptr && strcmp(last_skip_reason_, reason) == 0;
    bool same_state = last_skip_state_ == state;
    bool within_throttle = last_skip_log_time_us_ != 0 && (now - last_skip_log_time_us_) < kSkipLogThrottleUs;
    if (same_reason && same_state && within_throttle) {
        return;
    }

    ESP_LOGI(TAG, "Presence detection skipped: reason=%s state=%d", reason,
             static_cast<int>(state));
    last_skip_reason_ = reason;
    last_skip_state_ = state;
    last_skip_log_time_us_ = now;
}

void PresenceCoordinator::SubmitPresenceDetection() {
    const char* reason = "unknown";
    if (!CanStartGreeting(reason)) {
        MaybeLogSkippedDetection(reason);
        return;
    }

    greeting_in_progress_ = true;
    bootstrap_in_progress_ = false;
    bootstrap_retry_pending_ = false;
    pending_greeting_text_.clear();
    last_greeting_time_us_ = esp_timer_get_time();
    last_skip_reason_ = nullptr;

    if (!host_.HasPresenceExplainUrl()) {
        ESP_LOGI(TAG, "Presence greeting: no explain_url, opening channel for MCP bootstrap");
        bootstrap_in_progress_ = true;
        host_.SchedulePresence([this]() {
            if (!host_.OpenPresenceAudioChannel()) {
                ESP_LOGW(TAG, "Presence greeting: failed to open channel for MCP bootstrap");
                ResetGreetingState();
            }
        });
        return;
    }

    StartGreetingVisionTask();
}

bool PresenceCoordinator::BeginBootstrapVisionRetry(const char* source) {
    if (!bootstrap_in_progress_ || !host_.HasPresenceExplainUrl()) {
        return false;
    }

    if (source != nullptr) {
        ESP_LOGI(TAG, "Presence bootstrap %s explain_url, closing channel and retrying vision", source);
    }
    bootstrap_in_progress_ = false;
    bootstrap_retry_pending_ = true;
    host_.SchedulePresence([this]() {
        host_.ClosePresenceAudioChannel(false);
    });
    return true;
}

void PresenceCoordinator::OnPresenceAudioChannelOpened() {
    if (!bootstrap_in_progress_) {
        if (!pending_greeting_text_.empty()) {
            std::string text = std::move(pending_greeting_text_);
            pending_greeting_text_.clear();
            ESP_LOGI(TAG, "Sending presence greeting request");
            host_.SendPresenceGreetingText(text);
        }
        return;
    }

    BeginBootstrapVisionRetry(nullptr);
}

void PresenceCoordinator::OnPresenceAudioChannelClosed() {
    if (bootstrap_retry_pending_) {
        bootstrap_retry_pending_ = false;
        StartGreetingVisionTask();
        return;
    }

    if (!pending_greeting_text_.empty()) {
        pending_greeting_text_.clear();
    }

    if (!bootstrap_in_progress_) {
        ResetGreetingState();
    }
}

void PresenceCoordinator::OnPresenceMcpStateUpdated() {
    BeginBootstrapVisionRetry("MCP updated");
}

void PresenceCoordinator::StartGreetingVisionTask() {
    ESP_LOGI(TAG, "Start presence greeting vision check");

    BaseType_t ret = xTaskCreate([](void* arg) {
        auto* coordinator = static_cast<PresenceCoordinator*>(arg);
        try {
            std::string response = coordinator->host_.CapturePresenceGreetingDecision();
            std::string greet_text;
            if (!coordinator->ParseGreetingDecision(response, greet_text)) {
                coordinator->ResetGreetingState();
                vTaskDelete(NULL);
                return;
            }
            coordinator->QueueOrSendGreeting(greet_text);
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "Presence greeting failed: %s", e.what());
            coordinator->ResetGreetingState();
        } catch (...) {
            ESP_LOGE(TAG, "Presence greeting failed with unknown exception");
            coordinator->ResetGreetingState();
        }
        vTaskDelete(NULL);
    }, kGreetingVisionTaskName, kVisionTaskStackSize, this, kWorkerTaskPriority, nullptr);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create presence greeting vision task (ret=%d)", static_cast<int>(ret));
        ResetGreetingState();
    }
}

bool PresenceCoordinator::ParseGreetingDecision(const std::string& response, std::string& scene_summary) const {
    cJSON* root = cJSON_Parse(response.c_str());
    if (root == nullptr) {
        ESP_LOGW(TAG, "Presence greeting: failed to parse response");
        return false;
    }

    bool person_present = false;
    double confidence = 0.0;
    std::string reason_code;

    auto* item = cJSON_GetObjectItem(root, "person_present");
    if (cJSON_IsBool(item)) {
        person_present = cJSON_IsTrue(item);
    }
    item = cJSON_GetObjectItem(root, "confidence");
    if (cJSON_IsNumber(item)) {
        confidence = item->valuedouble;
    }
    item = cJSON_GetObjectItem(root, "scene_summary");
    if (cJSON_IsString(item)) {
        scene_summary = item->valuestring;
    }
    item = cJSON_GetObjectItem(root, "reason_code");
    if (cJSON_IsString(item)) {
        reason_code = item->valuestring;
    }
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Greeting vision: person_present=%d conf=%.2f reason=%s scene=%s",
             person_present, confidence, reason_code.c_str(), scene_summary.c_str());

    if (!person_present || confidence < kMinGreetingConfidence || scene_summary.empty()) {
        return false;
    }
    if (scene_summary.size() > kMaxGreetingTextLength) {
        scene_summary.resize(kMaxGreetingTextLength);
    }
    return true;
}

void PresenceCoordinator::QueueOrSendGreeting(const std::string& scene_summary) {
    host_.SchedulePresence([this, scene_summary]() {
        if (host_.GetPresenceDeviceState() != kDeviceStateIdle) {
            ResetGreetingState();
            return;
        }

        pending_greeting_text_ = scene_summary;
        if (host_.IsPresenceAudioChannelOpened()) {
            std::string text = std::move(pending_greeting_text_);
            pending_greeting_text_.clear();
            host_.SendPresenceGreetingText(text);
            return;
        }

        ESP_LOGI(TAG, "Opening audio channel for presence greeting");
        if (!host_.OpenPresenceAudioChannel()) {
            ESP_LOGW(TAG, "Presence greeting: failed to open channel");
            ResetGreetingState();
        }
    });
}

void PresenceCoordinator::ResetGreetingState() {
    bootstrap_in_progress_ = false;
    bootstrap_retry_pending_ = false;
    pending_greeting_text_.clear();
    greeting_in_progress_ = false;
}
