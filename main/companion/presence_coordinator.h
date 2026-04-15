#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "device_state.h"

class PresenceCoordinatorHost {
public:
    virtual ~PresenceCoordinatorHost() = default;

    virtual DeviceState GetPresenceDeviceState() const = 0;
    virtual bool IsPresenceAudioIdle() const = 0;
    virtual bool IsPresenceAudioProcessorRunning() const = 0;
    virtual bool IsPresencePlaybackDrained() const = 0;
    virtual bool IsPresenceAudioBackpressured() const = 0;
    virtual bool IsPresenceTransportReady() const = 0;
    virtual bool IsPresenceAudioChannelOpened() const = 0;
    virtual bool HasPresenceExplainUrl() const = 0;
    virtual bool OpenPresenceAudioChannel() = 0;
    virtual void ClosePresenceAudioChannel(bool send_goodbye) = 0;
    virtual std::string CapturePresenceGreetingDecision() = 0;
    virtual void SendPresenceGreetingText(const std::string& text) = 0;
    virtual void SchedulePresence(std::function<void()>&& callback) = 0;
};

class PresenceCoordinator {
public:
    explicit PresenceCoordinator(PresenceCoordinatorHost& host);

    void SubmitPresenceDetection();
    void OnPresenceAudioChannelOpened();
    void OnPresenceAudioChannelClosed();
    void OnPresenceMcpStateUpdated();

private:
    bool CanStartGreeting(const char*& reason) const;
    bool BeginBootstrapVisionRetry(const char* source);
    void MaybeLogSkippedDetection(const char* reason);
    void StartGreetingVisionTask();
    bool ParseGreetingDecision(const std::string& response, std::string& greet_text) const;
    void QueueOrSendGreeting(const std::string& greet_text);
    void ResetGreetingState();

    PresenceCoordinatorHost& host_;
    int64_t last_greeting_time_us_ = 0;
    bool greeting_in_progress_ = false;
    bool bootstrap_in_progress_ = false;
    bool bootstrap_retry_pending_ = false;
    std::string pending_greeting_text_;
    DeviceState last_skip_state_ = kDeviceStateUnknown;
    const char* last_skip_reason_ = nullptr;
    int64_t last_skip_log_time_us_ = 0;

    static constexpr int64_t kGreetingCooldownUs = 75LL * 1000 * 1000;
    static constexpr int64_t kSkipLogThrottleUs = 3LL * 1000 * 1000;
    static constexpr double kMinGreetingConfidence = 0.5;
    static constexpr size_t kMaxGreetingTextLength = 160;
};
