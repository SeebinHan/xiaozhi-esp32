#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "device_state.h"

enum class TouchLevel {
    kNone = 0,
    kLight,
    kMedium,
    kHard,
};

struct TouchEvent {
    TouchLevel level = TouchLevel::kNone;
    int raw = 0;
};

struct TouchAction {
    TouchLevel level = TouchLevel::kNone;
    int raw = 0;
};

class TouchCoordinatorHost {
public:
    virtual ~TouchCoordinatorHost() = default;

    virtual DeviceState GetTouchDeviceState() const = 0;
    virtual bool IsTouchAudioIdle() const = 0;
    virtual bool IsTouchAudioProcessorRunning() const = 0;
    virtual bool IsTouchPlaybackDrained() const = 0;
    virtual bool IsTouchAudioBackpressured() const = 0;
    virtual bool IsTouchExecutorReady() const = 0;
    virtual void DispatchTouchAction(const TouchAction& action) = 0;
    virtual void ScheduleTouch(std::function<void()>&& callback) = 0;
};

class TouchCoordinator {
public:
    explicit TouchCoordinator(TouchCoordinatorHost& host);

    void SubmitTouchEvent(const TouchEvent& event);

private:
    bool CanStartTouchAction(const char*& reason) const;
    void MaybeLogSkippedEvent(const TouchEvent& event, const char* reason);
    void ResetTouchState();

    TouchCoordinatorHost& host_;
    int64_t last_touch_action_time_us_ = 0;
    bool touch_action_in_progress_ = false;
    DeviceState last_skip_state_ = kDeviceStateUnknown;
    TouchLevel last_skip_level_ = TouchLevel::kNone;
    const char* last_skip_reason_ = nullptr;
    int64_t last_skip_log_time_us_ = 0;

    static constexpr int64_t kTouchCooldownUs = 3LL * 1000 * 1000;
    static constexpr int64_t kSkipLogThrottleUs = 2LL * 1000 * 1000;
};
