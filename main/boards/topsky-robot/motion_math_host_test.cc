#include "motion_math.h"

#include <cassert>
#include <cstring>

int main() {
    ServoAxisCalibration calibration{};
    calibration.trim_deg = 5;
    calibration.min_deg = 10;
    calibration.max_deg = 120;
    calibration.reversed = true;
    calibration.deadband_deg = 2;

    assert(ApplyCalibration(0, calibration) == 120);
    assert(ApplyCalibration(60, calibration) == 120);
    assert(ApplyCalibration(170, calibration) == 15);

    assert(NeedsServoWrite(-1, 50, calibration.deadband_deg));
    assert(!NeedsServoWrite(50, 51, calibration.deadband_deg));
    assert(NeedsServoWrite(50, 52, calibration.deadband_deg));
    assert(!NeedsServoWrite(50, 50, 0));
    assert(NeedsServoWrite(50, 51, 0));

    int cached_angle = 40;
    UpdateCachedAngleOnWriteResult(cached_angle, 72, true);
    assert(cached_angle == 72);
    UpdateCachedAngleOnWriteResult(cached_angle, 18, false);
    assert(cached_angle == 72);

    assert(ApplyEasing(EaseCurve::kLinear, 256) == 256);
    assert(ApplyEasing(EaseCurve::kEaseInOutCubic, 256) < 256);
    assert(ApplyEasing(EaseCurve::kEaseOutQuad, 256) > 256);

    assert(ComputeDelayedProgress(0, 100, 20) == 0);
    assert(ComputeDelayedProgress(20, 100, 20) == 0);
    assert(ComputeDelayedProgress(60, 100, 20) == 512);
    assert(ComputeDelayedProgress(100, 100, 20) == 1024);

    TouchMotionPreset preset = ParseTouchMotionPreset("nod_yes");
    assert(preset == TouchMotionPreset::kNodYes);
    assert(ParseTouchMotionPreset("shake_no") == TouchMotionPreset::kShakeNo);
    assert(ParseTouchMotionPreset("wag_tail") == TouchMotionPreset::kWagTail);
    assert(ParseTouchMotionPreset("greet_combo") == TouchMotionPreset::kGreetCombo);
    assert(ParseTouchMotionPreset("calm_combo") == TouchMotionPreset::kCalmCombo);
    assert(ParseTouchMotionPreset("unknown") == TouchMotionPreset::kNone);
    return 0;
}
