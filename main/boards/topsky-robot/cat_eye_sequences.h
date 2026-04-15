#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

struct AnimationSequence {
    const char* name;
    const char* const* frame_names;
    size_t frame_count;
    int lead_hold_frames;
    int tail_hold_frames;
    int hold_ms;
};

const AnimationSequence* FindAnimationSequence(const std::string& emotion);
