#pragma once

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstring>

enum class EaseCurve {
    kLinear,
    kEaseInOutCubic,
    kEaseOutQuad,
};

enum class TouchMotionPreset {
    kNone = 0,
    kNodYes,
    kShakeNo,
    kWagTail,
    kGreetCombo,
    kCalmCombo,
};

struct ServoAxisCalibration {
    int trim_deg = 0;
    int min_deg = 0;
    int max_deg = 180;
    bool reversed = false;
    int deadband_deg = 0;
};

inline int ApplyCalibration(int logical_angle, const ServoAxisCalibration& calibration) {
    int angle = logical_angle;
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }
    if (calibration.reversed) {
        angle = 180 - angle;
    }
    angle += calibration.trim_deg;
    if (angle < calibration.min_deg) {
        angle = calibration.min_deg;
    } else if (angle > calibration.max_deg) {
        angle = calibration.max_deg;
    }
    return angle;
}

inline bool NeedsServoWrite(int last_written_angle, int next_angle, int deadband_deg) {
    if (last_written_angle < 0) {
        return true;
    }
    if (next_angle == last_written_angle) {
        return false;
    }
    int threshold = deadband_deg <= 0 ? 1 : deadband_deg;
    return std::abs(next_angle - last_written_angle) >= threshold;
}

inline void UpdateCachedAngleOnWriteResult(int& cached_angle, int next_angle, bool write_success) {
    if (write_success) {
        cached_angle = next_angle;
    }
}

inline TouchMotionPreset ParseTouchMotionPreset(const char* preset_name) {
    if (preset_name == nullptr) {
        return TouchMotionPreset::kNone;
    }
    if (std::strcmp(preset_name, "nod_yes") == 0) {
        return TouchMotionPreset::kNodYes;
    }
    if (std::strcmp(preset_name, "shake_no") == 0) {
        return TouchMotionPreset::kShakeNo;
    }
    if (std::strcmp(preset_name, "wag_tail") == 0) {
        return TouchMotionPreset::kWagTail;
    }
    if (std::strcmp(preset_name, "greet_combo") == 0) {
        return TouchMotionPreset::kGreetCombo;
    }
    if (std::strcmp(preset_name, "calm_combo") == 0) {
        return TouchMotionPreset::kCalmCombo;
    }
    return TouchMotionPreset::kNone;
}

inline int ApplyEasing(EaseCurve curve, int progress_q10) {
    int t = progress_q10;
    if (t < 0) {
        t = 0;
    } else if (t > 1024) {
        t = 1024;
    }
    switch (curve) {
        case EaseCurve::kLinear:
            return t;
        case EaseCurve::kEaseInOutCubic: {
            if (t < 512) {
                int64_t tt = t;
                return static_cast<int>((4 * tt * tt * tt) / (1024LL * 1024LL));
            }
            int64_t inv = 1024 - t;
            return 1024 - static_cast<int>((4 * inv * inv * inv) / (1024LL * 1024LL));
        }
        case EaseCurve::kEaseOutQuad: {
            int64_t inv = 1024 - t;
            return 1024 - static_cast<int>((inv * inv) / 1024LL);
        }
    }
    return t;
}

inline int ComputeDelayedProgress(int elapsed_ms, int duration_ms, int phase_delay_percent) {
    if (duration_ms <= 0) {
        return 1024;
    }
    int delay_percent = phase_delay_percent;
    if (delay_percent < 0) {
        delay_percent = 0;
    } else if (delay_percent > 95) {
        delay_percent = 95;
    }
    int delay_ms = (duration_ms * delay_percent) / 100;
    if (elapsed_ms <= delay_ms) {
        return 0;
    }
    int remaining_ms = duration_ms - delay_ms;
    if (remaining_ms <= 0) {
        return 1024;
    }
    int adjusted = ((elapsed_ms - delay_ms) * 1024) / remaining_ms;
    if (adjusted < 0) {
        return 0;
    }
    if (adjusted > 1024) {
        return 1024;
    }
    return adjusted;
}
