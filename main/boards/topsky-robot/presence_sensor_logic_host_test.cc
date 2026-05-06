#include "presence_sensor_logic.h"

#include <cassert>

int main() {
    PresenceDetectionState state{};
    state.last_present = true;
    state.last_trigger_tick = 0;
    state.has_triggered_in_current_presence = false;
    state.absent_since_tick = 0;

    assert(ShouldTriggerPresenceDetection(state, true, true, 100, 0, 3000, 100));
    assert(!ShouldTriggerPresenceDetection(state, true, true, 100, 0, 3000, 30100));

    assert(!ShouldTriggerPresenceDetection(state, true, false, 100, 0, 3000, 30110));
    assert(!ShouldTriggerPresenceDetection(state, true, true, 30120, 0, 3000, 30120));

    assert(!ShouldTriggerPresenceDetection(state, false, true, 30120, 0, 3000, 30130));
    assert(state.last_trigger_tick == 100);
    assert(!ShouldTriggerPresenceDetection(state, true, true, 30140, 0, 3000, 30140));

    assert(!ShouldTriggerPresenceDetection(state, false, true, 30140, 0, 3000, 34000));
    assert(ShouldTriggerPresenceDetection(state, true, true, 30140, 0, 3000, 37050));

    PresenceDetectionState delayed{};
    delayed.last_present = false;
    delayed.last_trigger_tick = 0;
    delayed.has_triggered_in_current_presence = false;
    delayed.absent_since_tick = 0;
    assert(!ShouldTriggerPresenceDetection(delayed, true, true, 100, 50, 3000, 120));
    assert(ShouldTriggerPresenceDetection(delayed, true, true, 100, 50, 3000, 150));
    assert(!ShouldTriggerPresenceDetection(delayed, true, true, 100, 50, 3000, 200));

    return 0;
}
