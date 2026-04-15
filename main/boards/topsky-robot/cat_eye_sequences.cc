#include "cat_eye_sequences.h"

static const char* k_blink_natural_frames[6] = {
    "blink_natural_00.rgb565",
    "blink_natural_01.rgb565",
    "blink_natural_02.rgb565",
    "blink_natural_03.rgb565",
    "blink_natural_04.rgb565",
    "blink_natural_05.rgb565",
};

static const char* k_look_left_pupil_frames[25] = {
    "look_left_pupil_00.rgb565", "look_left_pupil_01.rgb565", "look_left_pupil_02.rgb565",
    "look_left_pupil_03.rgb565", "look_left_pupil_04.rgb565", "look_left_pupil_05.rgb565",
    "look_left_pupil_06.rgb565", "look_left_pupil_07.rgb565", "look_left_pupil_08.rgb565",
    "look_left_pupil_09.rgb565", "look_left_pupil_10.rgb565", "look_left_pupil_11.rgb565",
    "look_left_pupil_12.rgb565", "look_left_pupil_13.rgb565", "look_left_pupil_14.rgb565",
    "look_left_pupil_15.rgb565", "look_left_pupil_16.rgb565", "look_left_pupil_17.rgb565",
    "look_left_pupil_18.rgb565", "look_left_pupil_19.rgb565", "look_left_pupil_20.rgb565",
    "look_left_pupil_21.rgb565", "look_left_pupil_22.rgb565", "look_left_pupil_23.rgb565",
    "look_left_pupil_24.rgb565",
};

static const char* k_look_right_pupil_frames[25] = {
    "look_right_pupil_00.rgb565", "look_right_pupil_01.rgb565", "look_right_pupil_02.rgb565",
    "look_right_pupil_03.rgb565", "look_right_pupil_04.rgb565", "look_right_pupil_05.rgb565",
    "look_right_pupil_06.rgb565", "look_right_pupil_07.rgb565", "look_right_pupil_08.rgb565",
    "look_right_pupil_09.rgb565", "look_right_pupil_10.rgb565", "look_right_pupil_11.rgb565",
    "look_right_pupil_12.rgb565", "look_right_pupil_13.rgb565", "look_right_pupil_14.rgb565",
    "look_right_pupil_15.rgb565", "look_right_pupil_16.rgb565", "look_right_pupil_17.rgb565",
    "look_right_pupil_18.rgb565", "look_right_pupil_19.rgb565", "look_right_pupil_20.rgb565",
    "look_right_pupil_21.rgb565", "look_right_pupil_22.rgb565", "look_right_pupil_23.rgb565",
    "look_right_pupil_24.rgb565",
};

static const char* k_look_up_down_pupil_frames[37] = {
    "look_up_down_pupil_00.rgb565", "look_up_down_pupil_01.rgb565", "look_up_down_pupil_02.rgb565",
    "look_up_down_pupil_03.rgb565", "look_up_down_pupil_04.rgb565", "look_up_down_pupil_05.rgb565",
    "look_up_down_pupil_06.rgb565", "look_up_down_pupil_07.rgb565", "look_up_down_pupil_08.rgb565",
    "look_up_down_pupil_09.rgb565", "look_up_down_pupil_10.rgb565", "look_up_down_pupil_11.rgb565",
    "look_up_down_pupil_12.rgb565", "look_up_down_pupil_13.rgb565", "look_up_down_pupil_14.rgb565",
    "look_up_down_pupil_15.rgb565", "look_up_down_pupil_16.rgb565", "look_up_down_pupil_17.rgb565",
    "look_up_down_pupil_18.rgb565", "look_up_down_pupil_19.rgb565", "look_up_down_pupil_20.rgb565",
    "look_up_down_pupil_21.rgb565", "look_up_down_pupil_22.rgb565", "look_up_down_pupil_23.rgb565",
    "look_up_down_pupil_24.rgb565", "look_up_down_pupil_25.rgb565", "look_up_down_pupil_26.rgb565",
    "look_up_down_pupil_27.rgb565", "look_up_down_pupil_28.rgb565", "look_up_down_pupil_29.rgb565",
    "look_up_down_pupil_30.rgb565", "look_up_down_pupil_31.rgb565", "look_up_down_pupil_32.rgb565",
    "look_up_down_pupil_33.rgb565", "look_up_down_pupil_34.rgb565", "look_up_down_pupil_35.rgb565",
    "look_up_down_pupil_36.rgb565",
};

static const AnimationSequence kAnimationSequences[] = {
    {"blink_natural", k_blink_natural_frames, 6, 12, 14, 1200},
    {"look_left_pupil", k_look_left_pupil_frames, 25, 4, 16, 800},
    {"look_right_pupil", k_look_right_pupil_frames, 25, 4, 16, 800},
    {"look_up_down_pupil", k_look_up_down_pupil_frames, 37, 1, 2, 800},
};

static const AnimationSequence* FindSequenceByName(const char* name) {
    for (const auto& sequence : kAnimationSequences) {
        if (std::string(name) == sequence.name) {
            return &sequence;
        }
    }
    return nullptr;
}

const AnimationSequence* FindAnimationSequence(const std::string& emotion) {
    if (const auto* direct = FindSequenceByName(emotion.c_str())) {
        return direct;
    }

    if (emotion == "neutral" || emotion == "happy" || emotion == "laughing" || emotion == "funny") {
        return FindSequenceByName("blink_natural");
    }
    if (emotion == "thinking" || emotion == "confused") {
        return FindSequenceByName("look_left_pupil");
    }
    if (emotion == "surprised" || emotion == "shocked") {
        return FindSequenceByName("look_right_pupil");
    }
    if (emotion == "sad" || emotion == "crying" || emotion == "sleepy" || emotion == "tired") {
        return FindSequenceByName("look_up_down_pupil");
    }

    return FindSequenceByName("blink_natural");
}
