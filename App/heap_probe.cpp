#include "heap_probe.h"

#include "app.h"
#include "loop_trace.h"

namespace {

heap_probe::Stats probe = {};
// Read by the allocator wrappers, written by app(). Aligned word: the write is
// atomic on Cortex-M4, so a wrapper sees either the old or the new state.
volatile bool hot_flag = false;

void publish() {
    loop_trace::Counters& counters = loop_trace::counters();
    counters.heap_allocations = probe.allocations;
    counters.heap_hot_allocations = probe.hot_allocations;
    counters.heap_max_us = probe.max_us;
    counters.heap_hot_max_us = probe.hot_max_us;
}

void account(micros_t start_us) {
    const uint32_t elapsed_us = (uint32_t)(micros_64() - start_us);
    ++probe.allocations;
    if (elapsed_us > probe.max_us) {
        probe.max_us = elapsed_us;
    }
    if (hot_flag) {
        ++probe.hot_allocations;
        if (elapsed_us > probe.hot_max_us) {
            probe.hot_max_us = elapsed_us;
        }
    }
    publish();
}

}

namespace heap_probe {

void hot(bool active) {
    hot_flag = active;
    if (!active) {
        publish();
    }
}

}

extern "C" {

void* __real_malloc(size_t size);
void* __real_calloc(size_t count, size_t size);
void* __real_realloc(void* pointer, size_t size);
void __real_free(void* pointer);

void* __wrap_malloc(size_t size) {
    const micros_t start = micros_64();
    void* pointer = __real_malloc(size);
    account(start);
    return pointer;
}

void* __wrap_calloc(size_t count, size_t size) {
    const micros_t start = micros_64();
    void* pointer = __real_calloc(count, size);
    account(start);
    return pointer;
}

void* __wrap_realloc(void* pointer, size_t size) {
    const micros_t start = micros_64();
    void* result = __real_realloc(pointer, size);
    account(start);
    return result;
}

void __wrap_free(void* pointer) {
    const micros_t start = micros_64();
    __real_free(pointer);
    account(start);
}

}
