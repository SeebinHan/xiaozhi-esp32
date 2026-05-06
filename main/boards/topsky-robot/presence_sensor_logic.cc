#include "presence_sensor_logic.h"

bool ShouldTriggerPresenceDetection(PresenceDetectionState& state,
                                    bool present,
                                    bool enabled,
                                    uint32_t enabled_tick,
                                    uint32_t arm_delay_ticks,
                                    uint32_t leave_reset_ticks,
                                    uint32_t now_tick) {
    if (!present) {
        if (state.absent_since_tick == 0) {
            state.absent_since_tick = now_tick;
        }
        if (state.absent_since_tick != 0 &&
            (now_tick - state.absent_since_tick) >= leave_reset_ticks) {
            state.has_triggered_in_current_presence = false;
        }
        state.last_present = false;
        return false;
    }

    if (state.absent_since_tick != 0 &&
        (now_tick - state.absent_since_tick) >= leave_reset_ticks) {
        state.has_triggered_in_current_presence = false;
    }
    state.absent_since_tick = 0;

    bool arm_delay_elapsed = arm_delay_ticks == 0 || (now_tick - enabled_tick) >= arm_delay_ticks;
    bool should_trigger = present && enabled && arm_delay_elapsed &&
        !state.has_triggered_in_current_presence;

    state.last_present = true;
    if (should_trigger) {
        state.last_trigger_tick = now_tick;
        state.has_triggered_in_current_presence = true;
    }
    return should_trigger;
}
