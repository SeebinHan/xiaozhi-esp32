#pragma once

#include <cstdint>

struct PresenceDetectionState {
    bool last_present = false;
    uint32_t last_trigger_tick = 0;
    bool has_triggered_in_current_presence = false;
    uint32_t absent_since_tick = 0;
};

bool ShouldTriggerPresenceDetection(PresenceDetectionState& state,
                                    bool present,
                                    bool enabled,
                                    uint32_t enabled_tick,
                                    uint32_t arm_delay_ticks,
                                    uint32_t leave_reset_ticks,
                                    uint32_t now_tick);
