#pragma once

#include <string_view>

namespace SfxSounds {
    extern const char ogg_meow_soft_start[] asm("_binary_meow_soft_ogg_start");
    extern const char ogg_meow_soft_end[] asm("_binary_meow_soft_ogg_end");
    inline const std::string_view OGG_MEOW_SOFT{
        static_cast<const char*>(ogg_meow_soft_start),
        static_cast<size_t>(ogg_meow_soft_end - ogg_meow_soft_start)
    };

    extern const char ogg_meow_cute_start[] asm("_binary_meow_cute_ogg_start");
    extern const char ogg_meow_cute_end[] asm("_binary_meow_cute_ogg_end");
    inline const std::string_view OGG_MEOW_CUTE{
        static_cast<const char*>(ogg_meow_cute_start),
        static_cast<size_t>(ogg_meow_cute_end - ogg_meow_cute_start)
    };

    extern const char ogg_meow_loud_start[] asm("_binary_meow_loud_ogg_start");
    extern const char ogg_meow_loud_end[] asm("_binary_meow_loud_ogg_end");
    inline const std::string_view OGG_MEOW_LOUD{
        static_cast<const char*>(ogg_meow_loud_start),
        static_cast<size_t>(ogg_meow_loud_end - ogg_meow_loud_start)
    };
}
