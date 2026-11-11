#include "loop_trace.h"

namespace loop_trace {

namespace {

Sample ring[DEPTH];
size_t head = 0;  // next write index
size_t used = 0;
Counters event_counters = {};
bool armed_flag = false;
bool triggered = false;
bool lock_flag = false;

}

Counters& counters() {
    return event_counters;
}

void reset() {
    head = 0;
    used = 0;
    armed_flag = false;
    triggered = false;
    lock_flag = false;
}

void push(const Sample& sample) {
    if (lock_flag) {
        return;
    }
    ring[head] = sample;
    head = (head + 1) % DEPTH;
    if (used < DEPTH) {
        ++used;
    }
    if (armed_flag && (sample.flags & TRIGGER_FLAGS) != 0) {
        armed_flag = false;
        triggered = true;
    }
}

size_t count() {
    return used;
}

const Sample& at(size_t index) {
    const size_t oldest = (head + DEPTH - used) % DEPTH;
    return ring[(oldest + index) % DEPTH];
}

void arm(bool value) {
    armed_flag = value;
    if (value) {
        triggered = false;
    }
}

bool armed() {
    return armed_flag;
}

bool take_trigger() {
    const bool latched = triggered;
    triggered = false;
    return latched;
}

void lock(bool value) {
    lock_flag = value;
}

bool locked() {
    return lock_flag;
}

}
