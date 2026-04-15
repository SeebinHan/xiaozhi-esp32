#include "companion/touch_coordinator.h"

#include <esp_log.h>
#include <esp_timer.h>

#define TAG "TouchCoordinator"

TouchCoordinator::TouchCoordinator(TouchCoordinatorHost& host) : host_(host) {}

bool TouchCoordinator::CanStartTouchAction(const char*& reason) const {
    if (!host_.IsTouchExecutorReady()) {
        reason = "executor_not_ready";
        return false;
    }
    if (host_.GetTouchDeviceState() != kDeviceStateIdle) {
        reason = "state_not_idle";
        return false;
    }
    if (!host_.IsTouchAudioIdle()) {
        reason = "audio_not_idle";
        return false;
    }
    if (host_.IsTouchAudioProcessorRunning()) {
        reason = "audio_processor_running";
        return false;
    }
    if (!host_.IsTouchPlaybackDrained()) {
        reason = "playback_not_drained";
        return false;
    }
    if (host_.IsTouchAudioBackpressured()) {
        reason = "audio_backpressured";
        return false;
    }
    if (touch_action_in_progress_) {
        reason = "touch_in_progress";
        return false;
    }
    if (last_touch_action_time_us_ != 0 &&
        (esp_timer_get_time() - last_touch_action_time_us_) < kTouchCooldownUs) {
        reason = "cooldown_active";
        return false;
    }
    reason = "allowed";
    return true;
}

void TouchCoordinator::MaybeLogSkippedEvent(const TouchEvent& event, const char* reason) {
    int64_t now = esp_timer_get_time();
    DeviceState state = host_.GetTouchDeviceState();
    bool same_reason = last_skip_reason_ != nullptr && reason != nullptr && strcmp(last_skip_reason_, reason) == 0;
    bool same_state = last_skip_state_ == state;
    bool same_level = last_skip_level_ == event.level;
    bool within_throttle = last_skip_log_time_us_ != 0 && (now - last_skip_log_time_us_) < kSkipLogThrottleUs;
    if (same_reason && same_state && same_level && within_throttle) {
        return;
    }

    ESP_LOGI(TAG, "Touch event skipped: level=%d raw=%d reason=%s state=%d",
             static_cast<int>(event.level), event.raw, reason, static_cast<int>(state));
    last_skip_reason_ = reason;
    last_skip_state_ = state;
    last_skip_level_ = event.level;
    last_skip_log_time_us_ = now;
}

void TouchCoordinator::SubmitTouchEvent(const TouchEvent& event) {
    if (event.level == TouchLevel::kNone) {
        return;
    }

    const char* reason = "unknown";
    if (!CanStartTouchAction(reason)) {
        MaybeLogSkippedEvent(event, reason);
        return;
    }

    touch_action_in_progress_ = true;
    last_touch_action_time_us_ = esp_timer_get_time();
    last_skip_reason_ = nullptr;
    ESP_LOGI(TAG, "Touch event accepted: level=%d raw=%d", static_cast<int>(event.level), event.raw);

    host_.ScheduleTouch([this, event]() {
        TouchAction action{};
        action.level = event.level;
        action.raw = event.raw;
        ESP_LOGI(TAG, "Touch action dispatched: level=%d raw=%d",
                 static_cast<int>(action.level), action.raw);
        host_.DispatchTouchAction(action);
        ResetTouchState();
    });
}

void TouchCoordinator::ResetTouchState() {
    touch_action_in_progress_ = false;
}
