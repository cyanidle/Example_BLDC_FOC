#pragma once

#include <cstdint>

namespace cycle_counter {

struct Extension {
    uint32_t previous = 0;
    uint64_t high = 0;
};

inline uint64_t extend(Extension& state, uint32_t now) {
    if (now < state.previous) {
        state.high += uint64_t{1} << 32;
    }
    state.previous = now;
    return state.high + now;
}

}
