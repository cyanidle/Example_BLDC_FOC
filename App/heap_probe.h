#pragma once

#include <cstddef>
#include <cstdint>

/*
 * Heap-use probe for the control loop (App/heap_probe.cpp).
 *
 * The stutters show up as control periods of 3..32 ms. A free-store allocation
 * on the hot path is one way to lose that much time — a fragmented newlib heap
 * has no bounded cost — and unlike the protection mechanisms it leaves no other
 * trace. app() therefore flags its loop body and every allocator call is timed:
 * an allocation that runs while the flag is set proves the loop can block on
 * the heap, and its duration is what it cost.
 *
 * The wrappers are installed by the linker (--wrap=malloc,calloc,realloc,free),
 * which covers operator new as well because libstdc++'s operator new calls
 * malloc. Newlib's internal _malloc_r path (printf-family buffers) is not
 * wrapped: a stdio buffer allocated inside newlib is invisible here. The
 * numbers are mirrored into loop_trace::Counters (the trace never counts
 * anything itself), which is what `K` reports.
 */
namespace heap_probe {

struct Stats {
    uint32_t allocations;      // allocator calls since boot, any context
    uint32_t hot_allocations;  // ... of those, while the loop body was running
    uint32_t max_us;           // worst single allocator call, us, any context
    uint32_t hot_max_us;       // worst single allocator call on the hot path
};

// Set by app() around its loop body; cleared while the loop is idle.
void hot(bool active);

}
