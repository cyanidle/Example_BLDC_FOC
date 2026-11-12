#pragma once

#include <cstddef>
#include <cstdint>

/*
 * Bench "black box" for the drive loop: a pre-trigger ring of loop state plus
 * event counters, dumped over the serial channel (`W`, `K`).
 *
 * The 100 Hz `D` telemetry line cannot show why a drive stutters — a stutter is
 * an event of a few control periods, and what matters inside it is the angle
 * estimate, the d/q currents and the applied voltage. This keeps the last ~50 ms
 * at a few kHz, so an event can be looked at with the history that led into it.
 *
 * Thread context: the control loop pushes samples, the Hall EXTI counts Hall
 * events. Every field is a single aligned 32-bit word, so a reader sees either
 * the old or the new value (Cortex-M4 word accesses are atomic).
 */
namespace loop_trace {

struct Sample {
    uint32_t time_us;        // free-running microsecond clock, wraps ~71 min
    float angle_error_rad;   // sector centre - tracked angle, motor side, rad
    float angle_deg;         // tracked (applied) motor angle, deg
    float velocity_rad_s;    // wheel angular velocity, rad/s
    float q_current_a;       // filtered q (torque) current
    float d_current_a;       // filtered d current
    float phase_current_a;   // unfiltered phase-current magnitude
    float q_voltage_v;       // applied q voltage
    float d_voltage_v;       // applied d voltage
    float bus_voltage_v;
    uint8_t hall_raw;        // raw 3-bit Hall state of the last decoder event
    uint16_t flags;          // Flag bits carried by this sample
};

// Reasons a sample is marked. A sample carries the events that happened since
// the previous one, so the dump reads as a timeline of what the loop noticed —
// and, when the drive stutters, of which mechanism was active at that moment.
enum Flag: uint16_t {
    HALL_ILLEGAL = 1u << 0,      // a Hall transition no single step can explain
    RATE_IMPLAUSIBLE = 1u << 1,  // a Hall rate sample outside the drive's range
    GUARD_TRIP = 1u << 2,        // the current guard parked the bridge
    SUPPLY_DERATE = 1u << 3,     // the rail left its window and the torque was cut
    DRIVER_FAULT = 1u << 4,      // the gate driver asserted its FAULT line
    TARGET_CLAMPED = 1u << 5,    // the speed target was clamped to the ceiling
    ANGLE_SNAP = 1u << 6,        // the angle estimate was pulled onto a sector edge
    ANGLE_WRAP = 1u << 7,        // the angle estimate crossed 0/2pi
    SLOW_LOOP = 1u << 8,         // a control period ran over its budget
    CURRENT_LIMIT = 1u << 9,     // the velocity loop demanded more than current_limit
    VOLTAGE_LIMIT = 1u << 10,    // the current loop ran out of phase voltage
    BRAKE_CLAMP = 1u << 11,      // the regen ceiling clipped the torque demand
    COAST = 1u << 12,            // the open-loop channel parked into a coast
    REGULATOR_RESET = 1u << 13,  // the torque regulators were reset under the drive
    VELOCITY_TIMEOUT = 1u << 14, // the Hall rate sample timed out and was zeroed
};

// Events the sample flags are derived from; absolute counts since boot. The
// owner of each mechanism keeps its own count and mirrors it here before a
// sample is pushed (the trace never counts anything itself).
//
// This is the blame list: every mechanism that can stop the drive or take
// authority away from a regulator has its own counter, so a stutter can be
// attributed from `K` (who moved?) and from the dump's flags (which pass?).
struct Counters {
    volatile uint32_t hall_illegal;
    volatile uint32_t hall_counted;
    volatile uint32_t rate_implausible;
    volatile uint32_t angle_snaps;   // sector-boundary pulls (one per Hall edge)
    volatile uint32_t angle_wraps;
    volatile uint32_t angle_calls;
    volatile uint32_t velocity_timeouts;
    volatile uint32_t guard_trips;
    volatile uint32_t guard_overcurrent;
    volatile uint32_t guard_stalled;
    volatile uint32_t derate_entries;
    volatile uint32_t derate_high;
    volatile uint32_t derate_low;
    volatile uint32_t driver_faults;
    volatile uint32_t target_clamps;
    volatile uint32_t current_limits;
    volatile uint32_t voltage_limits;
    volatile uint32_t brake_clamps;
    volatile uint32_t coast_entries;
    volatile uint32_t regulator_resets;
    volatile uint32_t slow_loops;
    volatile uint32_t loop_max_us;
    // Worst duration of each phase of the app loop, us (written by app()): a
    // stall is only useful to know about if it can be attributed to a phase.
    volatile uint32_t phase_pid_max_us;
    volatile uint32_t phase_update_max_us;
    volatile uint32_t phase_poll_max_us;
    volatile uint32_t phase_persist_max_us;
    volatile uint32_t phase_cyphal_max_us;
    volatile uint32_t cyphal_transport_max_us;
    volatile uint32_t cyphal_report_max_us;
    volatile uint32_t cyphal_heartbeat_max_us;
    // Free-store use (App/heap_probe.h): the control loop must make none.
    volatile uint32_t heap_allocations;
    volatile uint32_t heap_hot_allocations;
    volatile uint32_t heap_max_us;
    volatile uint32_t heap_hot_max_us;
};

// Ring depth at the sampling period of the SimpleFOC backend (400 us): 128
// samples are ~51 ms of history, which is a couple of stutter periods.
constexpr size_t DEPTH = 128;

// Events that arm the automatic dump: each of them is the drive telling the
// operator that something it cannot explain happened. A stalled control period
// and an angle snap belong there: both are exactly what a stutter looks like
// from inside the loop.
constexpr uint16_t TRIGGER_FLAGS =
    HALL_ILLEGAL | RATE_IMPLAUSIBLE | GUARD_TRIP | SUPPLY_DERATE | DRIVER_FAULT |
    ANGLE_SNAP | SLOW_LOOP;

Counters& counters();

void reset();

// Copy one sample into the ring (newest wins) and latch the trigger when the
// sample carries a TRIGGER_FLAGS event while armed. A locked ring (a dump is
// being read out) drops the sample instead of overwriting the history.
void push(const Sample& sample);

size_t count();                 // samples currently in the ring
const Sample& at(size_t index); // 0 = oldest; valid while the ring is locked

// Automatic dumping: `arm(true)` makes the next triggering sample latch a
// trigger, which `take_trigger()` reports exactly once.
void arm(bool armed);
bool armed();
bool take_trigger();

// The dump is emitted by the serial poll over many passes; the ring must not
// advance underneath it or the history is overwritten while it is read.
void lock(bool locked);
bool locked();

}
