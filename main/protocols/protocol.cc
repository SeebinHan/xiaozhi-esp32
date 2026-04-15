#include "protocol.h"

#include <esp_log.h>
#include <esp_timer.h>

#define TAG "Protocol"

void Protocol::OnIncomingJson(std::function<void(const cJSON* root)> callback) {
    on_incoming_json_ = callback;
}

void Protocol::OnIncomingAudio(std::function<void(std::unique_ptr<AudioStreamPacket> packet)> callback) {
    on_incoming_audio_ = callback;
}

void Protocol::OnAudioChannelOpened(std::function<void()> callback) {
    on_audio_channel_opened_ = callback;
}

void Protocol::OnAudioChannelClosed(std::function<void()> callback) {
    on_audio_channel_closed_ = callback;
}

void Protocol::OnNetworkError(std::function<void(const std::string& message)> callback) {
    on_network_error_ = callback;
}

void Protocol::OnConnected(std::function<void()> callback) {
    on_connected_ = callback;
}

void Protocol::OnDisconnected(std::function<void()> callback) {
    on_disconnected_ = callback;
}

void Protocol::SetError(const std::string& message) {
    error_occurred_ = true;
    if (on_network_error_ != nullptr) {
        on_network_error_(message);
    }
}

void Protocol::SendAbortSpeaking(AbortReason reason) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"abort\"";
    if (reason == kAbortReasonWakeWordDetected) {
        message += ",\"reason\":\"wake_word_detected\"";
    }
    message += "}";
    SendText(message);
}

void Protocol::SendWakeWordDetected(const std::string& wake_word) {
    std::string json = "{\"session_id\":\"" + session_id_ + 
                      "\",\"type\":\"listen\",\"state\":\"detect\",\"text\":\"" + wake_word + "\"}";
    SendText(json);
}

void Protocol::SendStartListening(ListeningMode mode) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\"";
    message += ",\"type\":\"listen\",\"state\":\"start\"";
    if (mode == kListeningModeRealtime) {
        message += ",\"mode\":\"realtime\"";
    } else if (mode == kListeningModeAutoStop) {
        message += ",\"mode\":\"auto\"";
    } else {
        message += ",\"mode\":\"manual\"";
    }
    message += "}";
    SendText(message);
}

void Protocol::SendStopListening() {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"listen\",\"state\":\"stop\"}";
    SendText(message);
}

void Protocol::SendMcpMessage(const std::string& payload) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"mcp\",\"payload\":" + payload + "}";
    SendText(message);
}

void Protocol::SendProactiveGreetingRequest(const std::string& text) {
    cJSON* msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "session_id", session_id_.c_str());
    cJSON_AddStringToObject(msg, "type", "proactive_greeting_request");
    cJSON_AddStringToObject(msg, "text", text.c_str());
    char* json_str = cJSON_PrintUnformatted(msg);
    SendText(json_str);
    free(json_str);
    cJSON_Delete(msg);
}

void Protocol::SendVisualContext(const std::string& description) {
    cJSON* msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "session_id", session_id_.c_str());
    cJSON_AddStringToObject(msg, "type", "visual_context");
    cJSON_AddStringToObject(msg, "description", description.c_str());
    char* json_str = cJSON_PrintUnformatted(msg);
    SendText(json_str);
    free(json_str);
    cJSON_Delete(msg);
}

void Protocol::SendVisualContextStructured(const std::string& summary, const std::string& person_emotion,
                                           const std::string& person_state, const std::string& environment) {
    cJSON* msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "session_id", session_id_.c_str());
    cJSON_AddStringToObject(msg, "type", "visual_context");
    cJSON_AddStringToObject(msg, "summary", summary.c_str());
    cJSON_AddStringToObject(msg, "person_emotion", person_emotion.c_str());
    cJSON_AddStringToObject(msg, "person_state", person_state.c_str());
    cJSON_AddStringToObject(msg, "environment", environment.c_str());
    char* json_str = cJSON_PrintUnformatted(msg);
    SendText(json_str);
    free(json_str);
    cJSON_Delete(msg);
}

bool Protocol::IsTimeout() const {
    const int64_t kTimeoutMs = 120LL * 1000;
    if (last_incoming_time_ms_ == 0) {
        return false;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t duration_ms = now_ms - static_cast<int64_t>(last_incoming_time_ms_);
    bool timeout = duration_ms > kTimeoutMs;
    if (timeout) {
        ESP_LOGE(TAG, "Channel timeout %lld ms", static_cast<long long>(duration_ms));
    }
    return timeout;
}
