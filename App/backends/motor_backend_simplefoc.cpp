#include "motor_backend.h"

#include <algorithm>
#include <cmath>

#include "Arduino.h"
#include "app.h"
#include "current_guard.h"
#include "loop_trace.h"
#include "motor_limits.h"
#include "motor_tuning.h"
#include "serial_line.h"
#include "voltage_slew.h"
#include "BLDCMotor.h"
#include "common/base_classes/CurrentSense.h"
#include "common/base_classes/Sensor.h"
#include "drivers/BLDCDriver3PWM.h"

#ifndef VB_SIMPLEFOC_SENSOR_DIRECTION
#define VB_SIMPLEFOC_SENSOR_DIRECTION CW
#endif

// Bench-aligned with VB_SIMPLEFOC_ALIGN_ON_INIT=1: SimpleFOC reported
// zero_electric_angle = 0.5236 rad (exactly 30 electrical degrees — the
// half-sector between the counting and the sector-centre angle conventions)
// and sensor direction CW on this module. The value is a wiring property, not
// a boot-position artifact: libvoltbro's HallSensor re-bases its count on the
// absolute Hall sector in resync(), so two consecutive align boots both
// measured 0.5236. Re-measure if the motor or Hall harness is rewired.
#ifndef VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE
#define VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE 0.52359878f
#endif

#ifndef VB_SIMPLEFOC_ALIGN_ON_INIT
// 1 = find the zero electrical angle at boot (the motor turns and needs room).
#define VB_SIMPLEFOC_ALIGN_ON_INIT 0
#endif

// Current-sense polarity per phase: +1 when the Inverter's get_X() reading is
// positive while the driver sources current INTO phase X, -1 when it is
// inverted. The d/q regulators close over these signs: with a wrong one the
// measured q-current moves against the set-point, the current PID saturates at
// the voltage limit and the wheel hums instead of turning. Bench check: lift
// the wheel, E1, command a small direct voltage (Q0.5) and read `S ... i=`:
// the reported q-current must have the same sign as the applied voltage, and
// `i` must settle near zero at zero voltage. Flip the sign of whichever phase
// disagrees; the zero electrical angle above is only valid for one polarity
// set, so re-verify the drive direction after any flip.
#ifndef VB_SIMPLEFOC_CURRENT_SIGN_A
#define VB_SIMPLEFOC_CURRENT_SIGN_A 1.0f
#endif
#ifndef VB_SIMPLEFOC_CURRENT_SIGN_B
#define VB_SIMPLEFOC_CURRENT_SIGN_B 1.0f
#endif
#ifndef VB_SIMPLEFOC_CURRENT_SIGN_C
#define VB_SIMPLEFOC_CURRENT_SIGN_C 1.0f
#endif

namespace {

namespace tuning = motor_tuning;

enum : uint8_t {
    PIN_INH_A = 0,
    PIN_INH_B = 1,
    PIN_INH_C = 2,
    PIN_INL_A = 3,
    PIN_INL_B = 4,
    PIN_INL_C = 5,
    PIN_DRV_ENABLE = 6,
};

// SimpleFOC sensor over the board's Hall decoder: the EXTI handler in app.cpp
// keeps the count, this class turns it into SimpleFOC's mechanical angle and
// the shaft rate that its LPF_velocity then filters.
//
// A count identifies a Hall sector, not a point: its angle is the sector centre
// and the rotor sits anywhere within +/- half a sector of it
// (zero_electric_angle is measured against that definition). Feeding the
// resulting staircase straight into the current loop re-applies the voltage
// vector one sector late at every edge, which is the drive's audible FOC
// ripple. The angle is therefore tracked: it free-runs on the measured shaft
// rate and is pulled towards the centre of whichever sector the count reports,
// so the staircase is smoothed without extrapolating a whole sector from one
// rate sample.
class HallSensorAdapter final: public Sensor {
private:
    HallSensor& halls;
    int32_t last_counts = 0;
    micros_t last_velocity_us = 0;
    bool started = false;
    // Shaft rate is measured on the motor side of the gearbox; the rest of the
    // firmware (speed targets, reported velocity, odometry) speaks wheel rad/s.
    float velocity_scale = 1.0f;
    // Angle tracking state (motor side).
    float sector_centre = 0.0f;     // centre of the sector the count reports, rad
    float tracked_angle = 0.0f;     // the estimate handed to SimpleFOC, rad 0..2pi
    micros_t angle_us = 0;          // timestamp of the previous angle update
    float motor_rate_rad_s = 0.0f;  // measured shaft rate, motor side
    // The rate above is a one-edge measurement (2 ms apart on this motor), so
    // it carries tens of percent of noise; the free-running angle needs the
    // smoothed value instead.
    float prediction_rate_rad_s = 0.0f;
    bool angle_started = false;
    // Rates beyond this are physically impossible for this drive and are
    // treated as Hall jitter (motor side).
    float rate_limit_rad_s = 0.0f;
    // Loop-trace inputs (App/loop_trace.h): the last angle residual the tracker
    // worked against — the trace records the whole error history, so there is no
    // angle counter (a per-call threshold cannot separate the ordinary
    // post-edge catch-up from a real disagreement: 1640 of 1676 edges tripped a
    // naive one on the bench) — and how often a rate sample was outside the
    // drive's range.
    float last_error_rad = 0.0f;
    uint32_t implausible_rate_count = 0;
    // Blame counters for the stutter hunt (see loop_trace.h): how often the
    // estimate had to be pulled onto a sector edge (a phase discontinuity of up
    // to half a sector), how often it free-ran across 0/2pi (one rotor
    // revolution, the absolute-angle wrap), and how often the rate sample
    // timed out (no Hall edge for simplefoc_velocity_timeout_ms).
    uint32_t sector_snap_count = 0;
    uint32_t wrap_count = 0;
    uint32_t velocity_timeout_count = 0;

    static float wrap_2pi(float angle) {
        if (angle < 0.0f) return angle + _2PI;
        if (angle >= _2PI) return angle - _2PI;
        return angle;
    }

public:
    explicit HallSensorAdapter(HallSensor& hall): halls(hall) {}

    void set_velocity_scale(float scale) {
        velocity_scale = scale;
    }

    // Diagnostics for the loop trace.
    float get_last_error_rad() const { return last_error_rad; }
    float get_tracked_angle_rad() const { return tracked_angle; }
    uint32_t get_implausible_rate_count() const { return implausible_rate_count; }
    uint32_t get_sector_snap_count() const { return sector_snap_count; }
    uint32_t get_wrap_count() const { return wrap_count; }
    uint32_t get_velocity_timeout_count() const { return velocity_timeout_count; }

    // Ceiling a single rate sample may report, motor rad/s (see
    // simplefoc_hall_rate_plausible_multiple).
    void set_rate_limit(float rate_rad_s) {
        rate_limit_rad_s = rate_rad_s;
    }

    using Sensor::init;

    // Absolute rotor count: wraps per revolution in get_value(), so the turn
    // count has to be folded in and the pair sampled atomically.
    int32_t get_counts() const {
        int32_t counts;
        CRITICAL_SECTION(
            counts = halls.get_revolutions() * (int32_t)halls.CPR + (int32_t)halls.get_value();
        )
        return counts;
    }

    float getSensorAngle() override {
        const float sector = _2PI / (float)halls.CPR;
        const float centre = (((float)halls.get_value() + 0.5f) / (float)halls.CPR) * _2PI;
        const micros_t now = micros_64();
        if (!angle_started) {
            angle_started = true;
            angle_us = now;
            sector_centre = centre;
            tracked_angle = centre;
            return tracked_angle;
        }
        const float dt = (float)subtract_64(now, angle_us) * 1e-6f;
        angle_us = now;
        if (!(dt > 0.0f)) {
            return tracked_angle;
        }
        const float free_run = tracked_angle + prediction_rate_rad_s * dt;
        tracked_angle = wrap_2pi(free_run);
        if (free_run < 0.0f || free_run >= _2PI) {
            ++wrap_count;
        }
        float error = centre - tracked_angle;
        if (error > _PI) error -= _2PI;
        else if (error < -_PI) error += _2PI;
        last_error_rad = error;
        // Only the reported sector is evidence: never pull the estimate across
        // more than half a sector, so a wrong rate shows up as a bounded offset
        // instead of a slip into the wrong sector.
        const float half = 0.5f * sector;
        // A single bad rate sample (a Hall bounce halves one interval, which
        // doubles the rate) can free-run the estimate a whole sector or more in
        // one pass, and pulling it back at the gain below then takes tens of
        // milliseconds — with the current loops fighting the resulting 45..170
        // degree error, that is the 20..40 A burst that trips the supply. The
        // sector is hard evidence, so a saturated error snaps the estimate onto
        // the sector boundary instead: the worst case stays half a sector.
        if (error > half) {
            tracked_angle = wrap_2pi(centre - half);
            error = half;
            ++sector_snap_count;
        }
        else if (error < -half) {
            tracked_angle = wrap_2pi(centre + half);
            error = -half;
            ++sector_snap_count;
        }
        sector_centre = centre;
        const float gain = dt / (dt + tuning::simplefoc_hall_angle_tau_s);
        tracked_angle = wrap_2pi(tracked_angle + gain * error);
        return tracked_angle;
    }

    float getVelocity() override {
        const int32_t counts = get_counts();
        const micros_t now = micros_64();
        const float counts_to_rad = pi2 / (float)halls.CPR;
        if (!started) {
            last_counts = counts;
            last_velocity_us = now;
            started = true;
            return velocity;
        }
        const int32_t delta = counts - last_counts;
        const int64_t dt_us = subtract_64(now, last_velocity_us);
        if (delta != 0) {
            if (dt_us <= 0) {
                return velocity;
            }
            const float interval_s = (float)dt_us * 1e-6f;
            const float rate = ((float)delta * counts_to_rad) / interval_s;
            // Gate on magnitude: a jittering Hall line alternates two sectors
            // within a fraction of the edge interval, which claims several
            // times any speed this drive can reach. The measurement itself is
            // coarse (one edge per 2..5 ms, so consecutive samples of the same
            // motion differ by tens of percent), so the gate only rejects the
            // impossible — the ceiling comes from the drive's voltage limit and
            // its EMF constant.
            const bool plausible = std::abs(rate) <= rate_limit_rad_s;
            last_counts = counts;
            last_velocity_us = now;
            if (!plausible) {
                ++implausible_rate_count;
                return velocity;
            }
            motor_rate_rad_s = rate;
            velocity = motor_rate_rad_s * velocity_scale;
            const float alpha = interval_s / (interval_s + tuning::simplefoc_hall_rate_tau_s);
            prediction_rate_rad_s += alpha * (motor_rate_rad_s - prediction_rate_rad_s);
        }
        else if (dt_us >= (int64_t)tuning::simplefoc_velocity_timeout_ms * 1000) {
            ++velocity_timeout_count;
            motor_rate_rad_s = 0.0f;
            prediction_rate_rad_s = 0.0f;
            velocity = 0.0f;
            last_velocity_us = now;
        }
        return velocity;
    }
};

// SimpleFOC CurrentSense over the board's 3-shunt front end. The libvoltbro
// Inverter already owns the ADC+DMA and the zero-current offsets (recorded in
// its start(), with the bridge disabled), so this adapter only converts its
// amp-valued phase readings into SimpleFOC's PhaseCurrent_s — no pins, no ADC
// setup of its own. driverAlign() is skipped: the bridge is disabled while
// initFOC() runs, so the library's phase-probing alignment would read zero
// current and fail; the phase mapping and polarities are compile-time bench
// knowledge instead (VB_SIMPLEFOC_CURRENT_SIGN_*).
class InverterCurrentSense final: public CurrentSense {
private:
    BaseInverter& inverter;

public:
    explicit InverterCurrentSense(BaseInverter& inv): inverter(inv) {
        skip_align = true;
        gain_a = VB_SIMPLEFOC_CURRENT_SIGN_A;
        gain_b = VB_SIMPLEFOC_CURRENT_SIGN_B;
        gain_c = VB_SIMPLEFOC_CURRENT_SIGN_C;
    }

    int init() override {
        initialized = inverter.has_started();
        return initialized ? 1 : 0;
    }

    PhaseCurrent_s getPhaseCurrents() override {
        // The Inverter's amp readings are refreshed by its update(), which the
        // control loop calls once per pass. A blocking bench routine
        // (characteriseMotor runs a tight loop without returning to the loop)
        // would read frozen values, so refresh them here.
        inverter.update();
        PhaseCurrent_s currents;
        currents.a = gain_a * inverter.get_A();
        currents.b = gain_b * inverter.get_B();
        currents.c = gain_c * inverter.get_C();
        return currents;
    }
};

class SimpleFocBackend final: public MotorBackend {
private:
    HallSensor& halls;
    HallSensorAdapter sensor;
    BaseInverter& inverter;
    InverterCurrentSense current_sense;
    BLDCDriver3PWM driver;
    BLDCMotor motor;
    PIDConfig velocity_pid;
    bool speed_mode = false;
    float voltage_target = 0.0f;
    // Voltage the open-loop channel is applying right now, V: it ramps towards
    // voltage_target so a Q/Cyphal 4050 step cannot jerk the bridge (the
    // velocity path gets its soft start from PID_velocity.output_ramp).
    float voltage_applied = 0.0f;
    float speed_target = 0.0f;
    // Operator stop (a zero speed command): the loop stays in current control
    // with a zero q-current set-point instead of holding a braking torque. The
    // windings then see nothing but their own back-EMF, so the wheel freewheels
    // and nothing is pushed back into a supply that cannot sink it.
    bool coast_mode = false;
    float coast_current_a = 0.0f;
    MotorFault fault = MotorFault::NONE;
    // Set while the current guard holds the bridge parked. The trip is latched
    // (`E0` clears it) so the operator sees why the wheel stopped, and the
    // zero-command re-arm is what pulses the driver back on.
    bool guard_parked = false;
    // Wheel rate above which clearing the d/q regulator state would drop the
    // voltage that holds the back-EMF, wheel rad/s. Derived from the drive's
    // EMF constant and the current loop's voltage floor (see
    // simplefoc_regulator_reset_emf_fraction).
    float regulator_reset_wheel_rate = 0.0f;
    // Slew of the velocity loop's q-current output, A/s (config id 14).
    float torque_ramp_a_s = tuning::simplefoc_torque_ramp_a_s;
    // Braking-current ceiling against the measured rotation, A (config id 15):
    // the regen limit. Zero leaves a moving wheel to coast.
    float brake_current_a = tuning::simplefoc_brake_current_a;
    // Wheel rate below which the drive counts as stopped for the brake clamp,
    // rad/s: the stall guard's velocity ceiling (config id 10), cached so the
    // clamp follows a config change. Below it either torque sign is allowed,
    // so a reversal can finish through standstill.
    float stopped_wheel_rate = tuning::simplefoc_stall_velocity_rad_s;
    // The gate driver's FAULT line is its own status: high while the bridge is
    // usable, low while it has switched itself off. `driver_fault_release_s`
    // counts how long it has been released again, so a transient trip cannot
    // re-arm the loop on the first quiet pass.
    bool driver_fault_active = false;
    float driver_fault_release_s = 0.0f;
    // 1 while the supply rail is healthy, 0 when it has drooped: scales the
    // torque demand (see the derate block in update()).
    float supply_derate = 1.0f;
    // Wheel (gearbox output) radians per rotor Hall count.
    float wheel_rad_per_count = 0.0f;
    // Timestamp of the previous update(), for the open-loop voltage ramp.
    micros_t last_update_us = 0;
    // Last measured q-current, A (telemetry; refreshed in update()).
    float torque_current_a = 0.0f;
    // Soft current ceiling for the open-loop voltage channel, V: integrates
    // down while the filtered current is over the limit and recovers at the
    // ramp rate below it, so a Q step accelerates the wheel current-limited.
    float voltage_ceiling_v = 0.0f;
    // Back-EMF per wheel rad/s, V/(rad/s): DriveInfo::torque_const is volts per
    // ROTOR rad/s, the gearbox scales it to the wheel side. Every voltage and
    // speed budget below is this number against the modulation ceiling.
    float emf_per_wheel_rad_s = 0.0f;
    // Loop trace (App/loop_trace.h): counters this backend owns, plus the
    // snapshot the sample flags are diffed against. One counter per mechanism
    // that can stop the drive or take authority from a regulator, so a stutter
    // can be attributed instead of guessed at (see loop_trace::Counters).
    uint32_t guard_overcurrent_count = 0;
    uint32_t guard_stalled_count = 0;
    uint32_t derate_high_count = 0;
    uint32_t derate_low_count = 0;
    uint32_t driver_fault_count = 0;
    uint32_t target_clamp_count = 0;
    uint32_t current_limit_count = 0;
    uint32_t voltage_limit_count = 0;
    uint32_t brake_clamp_count = 0;
    uint32_t coast_entry_count = 0;
    uint32_t regulator_reset_count = 0;
    uint32_t slow_loop_count = 0;
    uint32_t loop_max_us = 0;
    // Rising-edge latches, so a counter counts episodes of a limit being
    // active and not every control pass it stays active for.
    bool current_limit_active = false;
    bool voltage_limit_active = false;
    bool brake_clamp_active = false;
    micros_t trace_time_us = 0;
    loop_trace::Counters trace_seen = {};
    bool trace_derated = false;
    bool trace_guard_counted = false;
    bool clamp_logged = false;

    // One trace sample: the loop state plus the events since the previous
    // sample, so a dump reads as a timeline. Counters owned elsewhere (the Hall
    // decoder, the angle tracker) are mirrored into the trace's block here.
    void push_trace_sample(micros_t now) {
        if (loop_trace::locked()) {
            return;
        }
        const uint32_t now_us = (uint32_t)now;
        if (trace_time_us != 0 &&
            (uint32_t)(now_us - (uint32_t)trace_time_us) < tuning::simplefoc_trace_period_us) {
            return;
        }
        trace_time_us = now;

        loop_trace::Counters& counters = loop_trace::counters();
        counters.hall_illegal = halls.get_illegal_transitions();
        counters.hall_counted = halls.get_counted_edges();
        counters.rate_implausible = sensor.get_implausible_rate_count();
        counters.angle_snaps = sensor.get_sector_snap_count();
        counters.angle_wraps = sensor.get_wrap_count();
        counters.velocity_timeouts = sensor.get_velocity_timeout_count();
        counters.guard_overcurrent = guard_overcurrent_count;
        counters.guard_stalled = guard_stalled_count;
        counters.guard_trips = guard_overcurrent_count + guard_stalled_count;
        counters.derate_high = derate_high_count;
        counters.derate_low = derate_low_count;
        counters.derate_entries = derate_high_count + derate_low_count;
        counters.driver_faults = driver_fault_count;
        counters.target_clamps = target_clamp_count;
        counters.current_limits = current_limit_count;
        counters.voltage_limits = voltage_limit_count;
        counters.brake_clamps = brake_clamp_count;
        counters.coast_entries = coast_entry_count;
        counters.regulator_resets = regulator_reset_count;
        counters.slow_loops = slow_loop_count;
        counters.loop_max_us = loop_max_us;

        loop_trace::Sample sample = {};
        sample.time_us = now_us;
        sample.angle_error_rad = sensor.get_last_error_rad();
        sample.angle_deg = sensor.get_tracked_angle_rad() * (180.0f / (pi2 * 0.5f));
        sample.velocity_rad_s = motor.shaft_velocity;
        sample.q_current_a = motor.current.q;
        sample.d_current_a = motor.current.d;
        const PhaseCurrent_s phase = current_sense.getPhaseCurrents();
        sample.phase_current_a = current_guard::phase_current_magnitude(
            phase.a, phase.b, phase.c
        );
        sample.q_voltage_v = motor.voltage.q;
        sample.d_voltage_v = motor.voltage.d;
        sample.bus_voltage_v = inverter.get_busV();
        sample.hall_raw = (uint8_t)halls.get_step();
        sample.flags = 0;
        mark(sample.flags, counters.hall_illegal, trace_seen.hall_illegal, loop_trace::HALL_ILLEGAL);
        mark(sample.flags, counters.rate_implausible, trace_seen.rate_implausible, loop_trace::RATE_IMPLAUSIBLE);
        mark(sample.flags, counters.angle_snaps, trace_seen.angle_snaps, loop_trace::ANGLE_SNAP);
        mark(sample.flags, counters.angle_wraps, trace_seen.angle_wraps, loop_trace::ANGLE_WRAP);
        mark(sample.flags, counters.velocity_timeouts, trace_seen.velocity_timeouts,
             loop_trace::VELOCITY_TIMEOUT);
        mark(sample.flags, counters.guard_trips, trace_seen.guard_trips, loop_trace::GUARD_TRIP);
        mark(sample.flags, counters.derate_entries, trace_seen.derate_entries, loop_trace::SUPPLY_DERATE);
        mark(sample.flags, counters.driver_faults, trace_seen.driver_faults, loop_trace::DRIVER_FAULT);
        mark(sample.flags, counters.current_limits, trace_seen.current_limits, loop_trace::CURRENT_LIMIT);
        mark(sample.flags, counters.voltage_limits, trace_seen.voltage_limits, loop_trace::VOLTAGE_LIMIT);
        mark(sample.flags, counters.brake_clamps, trace_seen.brake_clamps, loop_trace::BRAKE_CLAMP);
        mark(sample.flags, counters.coast_entries, trace_seen.coast_entries, loop_trace::COAST);
        mark(sample.flags, counters.regulator_resets, trace_seen.regulator_resets,
             loop_trace::REGULATOR_RESET);
        mark(sample.flags, counters.slow_loops, trace_seen.slow_loops, loop_trace::SLOW_LOOP);
        // A clamped target is a state, not an event: the set-point either is
        // held below the ceiling right now or it is not.
        if (is_target_clamped()) sample.flags |= loop_trace::TARGET_CLAMPED;
        trace_seen = counters;

        loop_trace::push(sample);
    }

    static void mark(uint16_t& flags, uint32_t now, uint32_t seen, loop_trace::Flag flag) {
        if (now != seen) {
            flags = (uint16_t)(flags | (uint16_t)flag);
        }
    }

    void apply_velocity_pid() {
        motor.PID_velocity.P = velocity_pid.kp;
        motor.PID_velocity.I = velocity_pid.ki;
        motor.PID_velocity.D = velocity_pid.kd;
        // Soft start: slew the q-current set-point instead of stepping it. A
        // PID reset (enable, mode switch) restarts the ramp from zero.
        motor.PID_velocity.output_ramp = torque_ramp_a_s;
        motor.PID_velocity.limit = velocity_pid.max_output;
    }

    void apply_current_pid() {
        motor.PID_current_q.P = tuning::simplefoc_current_kp;
        motor.PID_current_q.I = tuning::simplefoc_current_ki;
        motor.PID_current_q.D = 0.0f;
        motor.PID_current_q.output_ramp = 0.0f;
        motor.PID_current_d = motor.PID_current_q;
        motor.LPF_current_q.Tf = tuning::simplefoc_current_lpf_tf;
        motor.LPF_current_d.Tf = tuning::simplefoc_current_lpf_tf;
    }

    void reset_torque_regulators() {
        ++regulator_reset_count;
        motor.PID_velocity.reset();
        // reset() clears the PID states but not the set-point SimpleFOC is
        // holding: a voltage-mode value (V) surviving into current mode would
        // be read as amps until the next move() overwrites it.
        motor.current_sp = 0.0f;
        // The d/q integrals are what hold the phase voltage against the
        // back-EMF at speed: clearing them there shorts the windings for a few
        // milliseconds (see simplefoc_regulator_reset_wheel_rate_rad_s). A mode
        // switch mid-spin keeps them and only a parked or nearly stopped drive
        // starts from zero.
        if (motor.enabled &&
            std::abs(motor.shaft_velocity) > regulator_reset_wheel_rate) {
            return;
        }
        motor.PID_current_q.reset();
        motor.PID_current_d.reset();
    }

    // Advance `value` towards `target` by at most `max_step` up / `down_step`
    // down (a zero or invalid step leaves it alone: the caller passes
    // ramp * dt). The downward slew is faster so the current-limiting ceiling
    // never leaves a large voltage sitting on a stalled phase.
    static float ramp_towards(float value, float target, float max_step, float down_step = 0.0f) {
        if (!(max_step > 0.0f)) {
            return value;
        }
        const float down = down_step > 0.0f ? down_step : max_step;
        if (value < target) return std::min(target, value + max_step);
        if (value > target) return std::max(target, value - down);
        return value;
    }

    // DRV8323 hardware FAULT line (open drain, active low): the gate driver
    // latches its outputs off on overcurrent/undervoltage/overtemperature and
    // only an enable-line edge clears it.
    static bool driver_fault_asserted() {
        return HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET;
    }

    // Centered SinePWM swings each phase around driver->voltage_limit/2, so the
    // largest undistorted q-voltage is half the driver limit; anything above it
    // only flat-tops the sine. The headroom keeps the peak phase below 100%
    // duty (see simplefoc_modulation_headroom).
    static float modulation_limit(float driver_voltage_limit) {
        return std::min(
            driver_voltage_limit * 0.5f * tuning::simplefoc_modulation_headroom,
            tuning::simplefoc_motor_voltage_limit_v
        );
    }

public:
    SimpleFocBackend(
        DriveRuntimeConfig&,
        DriveInfo& drive_info,
        TIM_HandleTypeDef*,
        BaseInverter& inverter_,
        HallSensor& hall,
        const PIDConfig& pid_config,
        float wheel_radius_m
    ):
        MotorBackend(wheel_radius_m),
        halls(hall),
        sensor(hall),
        inverter(inverter_),
        current_sense(inverter_),
        driver(PIN_INH_A, PIN_INH_B, PIN_INH_C, PIN_INL_A, PIN_INL_B, PIN_INL_C),
        motor(drive_info.common.ppairs),
        velocity_pid(pid_config)
    {
        sensor.set_velocity_scale(1.0f / (float)drive_info.common.gear_ratio);
        wheel_rad_per_count =
            (pi2 / (float)hall.CPR) / (float)drive_info.common.gear_ratio;
        // Every rate threshold below follows from the drive's own parameters:
        // the EMF constant (torque_const, V per rotor rad/s) times the gearbox
        // ratio is the phase voltage per wheel rad/s, and the modulation ceiling
        // divided by it is the top rotor rate this supply can drive.
        emf_per_wheel_rad_s =
            drive_info.torque_const * (float)drive_info.common.gear_ratio;
        regulator_reset_wheel_rate =
            tuning::simplefoc_current_loop_v_floor *
            tuning::simplefoc_regulator_reset_emf_fraction / emf_per_wheel_rad_s;
        sensor.set_rate_limit(
            tuning::simplefoc_hall_rate_plausible_multiple *
            tuning::simplefoc_motor_voltage_limit_v / drive_info.torque_const
        );
        arduino_pwm_pin_register(PIN_INH_A, 1);
        arduino_pwm_pin_register(PIN_INH_B, 2);
        arduino_pwm_pin_register(PIN_INH_C, 3);
        arduino_pin_register(PIN_INL_A, INLA_GPIO_Port, INLA_Pin);
        arduino_pin_register(PIN_INL_B, INLB_GPIO_Port, INLB_Pin);
        arduino_pin_register(PIN_INL_C, INLC_GPIO_Port, INLC_Pin);
        arduino_pin_register(PIN_DRV_ENABLE, DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin);
    }

    const char* name() const override {
        return "simplefoc";
    }

    HAL_StatusTypeDef init() override {
        inverter.start();
        inverter.update();
        const float bus_voltage = inverter.get_busV();
        if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0f) {
            return HAL_ERROR;
        }

        driver.voltage_power_supply = bus_voltage;
        driver.voltage_limit = std::min(bus_voltage, tuning::simplefoc_driver_voltage_limit_v);
        driver.pwm_frequency = tuning::simplefoc_pwm_frequency_hz;

        motor.linkDriver(&driver);
        motor.linkSensor(&sensor);
        // Current-based torque control: the velocity PID's output is a q-current
        // set-point (A) and the d/q current loops close over the measured phase
        // currents from the Inverter.
        current_sense.linkDriver(&driver);
        motor.linkCurrentSense(&current_sense);
        motor.torque_controller = TorqueControlType::foc_current;
        motor.controller = MotionControlType::torque;
        motor.foc_modulation = FOCModulationType::SinePWM;
        motor.voltage_limit = modulation_limit(driver.voltage_limit);
        voltage_ceiling_v = motor.voltage_limit;
        // The ceiling is reported here as well as in update(), so the boot
        // default of the normalized full-scale speed is the speed this rail can
        // regulate (app.cpp reads it right after init()).
        set_speed_ceiling(
            motor_limits::reachable_wheel_rate(
                motor.voltage_limit,
                emf_per_wheel_rad_s,
                tuning::simplefoc_current_loop_v_floor
            ) * wheel_radius()
        );
        motor.current_limit = tuning::simplefoc_current_limit_a;
        motor.velocity_limit = tuning::simplefoc_max_shaft_velocity_rad_s;
        motor.LPF_velocity.Tf = tuning::simplefoc_velocity_lpf_tf;
#if !VB_SIMPLEFOC_ALIGN_ON_INIT
        motor.sensor_direction = VB_SIMPLEFOC_SENSOR_DIRECTION;
        motor.zero_electric_angle = VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE;
#else
        // The direction is a wiring property (verified on both drive
        // backends); presetting it leaves initFOC() only the zero-angle park,
        // turning the two-revolution direction sweep into a sub-second hold.
        motor.sensor_direction = VB_SIMPLEFOC_SENSOR_DIRECTION;
        motor.voltage_sensor_align = tuning::simplefoc_align_voltage_v;
#endif
        apply_velocity_pid();
        apply_current_pid();

        sensor.init();
        if (driver.init() != 1) {
            return HAL_ERROR;
        }
        if (current_sense.init() != 1) {
            return HAL_ERROR;
        }
        if (motor.init() != 1) {
            return HAL_ERROR;
        }
#if VB_SIMPLEFOC_ALIGN_ON_INIT
        // Alignment turns the rotor, so the bridge has to be live while
        // initFOC() runs; it is parked again right after.
        driver.enable();
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
#endif
        if (motor.initFOC() != 1) {
            return HAL_ERROR;
        }
#if VB_SIMPLEFOC_ALIGN_ON_INIT
        driver.disable();
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
#endif
        motor.disable();
        return HAL_OK;
    }

    HAL_StatusTypeDef start() override {
        if (motor.enabled && !driver_fault_active) {
            return HAL_OK;
        }
        serial_line_debug("simplefoc start: zero_electric_angle", motor.zero_electric_angle);
        serial_line_debug("simplefoc start: sensor_direction", (float)(int)motor.sensor_direction);
        serial_line_debug("simplefoc start: voltage_limit", motor.voltage_limit);
        serial_line_debug("simplefoc start: current_limit", motor.current_limit);
        // Zero-current sanity: with the bridge just parked these should read
        // near zero; a large value means the Inverter's offsets are stale (the
        // bridge was live during its start()) or a sign/gain is wrong.
        serial_line_debug("simplefoc start: current a (A)", inverter.get_A());
        serial_line_debug("simplefoc start: current b (A)", inverter.get_B());
        serial_line_debug("simplefoc start: current c (A)", inverter.get_C());
        driver.setPwm(0.0f, 0.0f, 0.0f);
        // The pulse train clears a latched gate-driver fault, so it also runs
        // when the motor was left enabled by an earlier `E1` (a faulted driver
        // keeps its own outputs off, the backend's enabled flag does not).
        for (int i = 0; i < 3; ++i) {
            HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
            HAL_Delay(10);
        }
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
        driver_fault_active = driver_fault_asserted();
        driver_fault_release_s = 0.0f;
        motor.enable();
        return HAL_OK;
    }

    HAL_StatusTypeDef stop() override {
        motor.disable();
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
        // An explicit `E0` outranks the guard's re-arm: the operator parked the
        // bridge, so a zero command must not pulse it back on.
        guard_parked = false;
        return HAL_OK;
    }

    HAL_StatusTypeDef set_state(bool state) override {
        return state ? start() : stop();
    }

    bool is_on() const override {
        return motor.enabled != 0;
    }

    void update() override {
        const micros_t now = micros_64();
        const int64_t elapsed_us = last_update_us ? subtract_64(now, last_update_us) : 0;
        last_update_us = now;
        // Guard the ramp the same way SimpleFOC guards its PID dt: a wrap or a
        // stall long enough to make dt useless skips the step for this pass.
        const float dt_s =
            (elapsed_us > 0 && elapsed_us <= 500000) ? (float)elapsed_us * 1e-6f : 0.0f;
        // A control period past its budget is the prime suspect for a stutter:
        // while it lasts the bridge holds the last vector and the rotor turns
        // through it. Counted per overrun, with the worst case since boot.
        if (elapsed_us > (int64_t)tuning::simplefoc_loop_overrun_us) {
            ++slow_loop_count;
        }
        if (elapsed_us > 0 && (uint32_t)elapsed_us > loop_max_us) {
            loop_max_us = (uint32_t)elapsed_us;
        }

        inverter.update();
        const float bus_voltage = inverter.get_busV();
        if (std::isfinite(bus_voltage) && bus_voltage > 0.0f) {
            driver.voltage_power_supply = bus_voltage;
            driver.voltage_limit = std::min(bus_voltage, tuning::simplefoc_driver_voltage_limit_v);
            motor.voltage_limit = modulation_limit(driver.voltage_limit);
            voltage_ceiling_v = std::min(voltage_ceiling_v, motor.voltage_limit);
            // The current loops output phase volts, and the velocity loop
            // outputs the q-current set-point, so both limits follow the bus.
            const float speed_abs = std::abs(motor.shaft_velocity);
            // The current loop has to be able to apply the back-EMF plus the
            // low-speed floor at the speed the wheel is running. Sized from the
            // drive's EMF constant (not from a fixed V/(rad/s) guess): a cap
            // that trails the EMF cannot hold the current at speed, and the
            // vector it saturates to brakes harder the faster the wheel turns.
            const float low_speed_v = motor_limits::current_loop_voltage_limit(
                emf_per_wheel_rad_s, speed_abs, tuning::simplefoc_current_loop_v_floor
            );
            const float current_loop_v = std::min(motor.voltage_limit, low_speed_v);
            motor.PID_current_q.limit = current_loop_v;
            motor.PID_current_d.limit = current_loop_v;
            motor.PID_velocity.limit =
                std::min(velocity_pid.max_output, motor.current_limit) * supply_derate;
            // Largest wheel speed this rail can still regulate; the façade
            // clamps the speed target to it, so a command past the modulation
            // ceiling cannot saturate the current loops (see motor_limits.h).
            set_speed_ceiling(
                motor_limits::reachable_wheel_rate(
                    motor.voltage_limit,
                    emf_per_wheel_rad_s,
                    tuning::simplefoc_current_loop_v_floor
                ) * wheel_radius()
            );
            // The bus rail is the only supply-current proxy this board has.
            // Braking pushes energy back into it and the bench supply neither
            // sinks nor tolerates it: a stop from 0.3 m/s raised the rail to
            // 22.1 V and the supply then cut out (4.1 V), which is what trips
            // the gate driver's undervoltage protection. Pulling the torque
            // demand back on an over-voltage drops the braking and lets the
            // wheel coast; the droop side of the same derate keeps the supply
            // out of current limit on acceleration.
            const bool rail_high = bus_voltage > tuning::simplefoc_supply_derate_over_v;
            const bool rail_low = bus_voltage < tuning::simplefoc_supply_derate_low_v;
            if (rail_high || rail_low) {
                // Immediate: the rail crosses these thresholds in tens of
                // milliseconds, so there is no time to ramp the demand down.
                supply_derate = 0.0f;
                if (!trace_derated) {
                    trace_derated = true;
                    // Which side of the window: the droop side is a supply that
                    // cannot feed the demand, the high side is the load pushing
                    // energy back into a supply that cannot sink it.
                    if (rail_high) {
                        ++derate_high_count;
                    }
                    else {
                        ++derate_low_count;
                    }
                }
            }
            else {
                trace_derated = false;
                if (bus_voltage > tuning::simplefoc_supply_derate_full_v) {
                    supply_derate = std::min(
                        1.0f,
                        supply_derate + tuning::simplefoc_supply_derate_rate_per_s * dt_s
                    );
                }
            }
            voltage_ceiling_v = std::min(voltage_ceiling_v, motor.voltage_limit * supply_derate);
        }
        else {
            fault = MotorFault::INVALID_SUPPLY;
        }

        // A rejected open-loop target is sticky: the supply branch above would
        // clear it on the very next pass while the bad command is still set.
        if (!speed_mode && !std::isfinite(voltage_target)) {
            fault = MotorFault::INVALID_COMMAND;
        }

        // The gate driver trips its own protection on FAULT and switches the
        // bridge off underneath us: the loop then sees no current and winds its
        // regulators up, which is exactly what it must not do. Park while the
        // line is asserted. An automatically recovering fault (PVDD
        // undervoltage — the supply collapsing under a transient) releases the
        // line by itself and the drive resumes after a short quiet period; a
        // latched one (VDS, GVDD/boot, thermal) holds the line low and needs
        // `E1`, which pulses nSLEEP.
        if (driver_fault_asserted()) {
            if (!driver_fault_active) {
                serial_line_debug("simplefoc: gate driver FAULT, bridge parked");
                ++driver_fault_count;
            }
            driver_fault_active = true;
            driver_fault_release_s = 0.0f;
        }
        else if (driver_fault_active) {
            driver_fault_release_s += dt_s;
        }
        if (driver_fault_active) {
            fault = MotorFault::DRIVER_FAULT;
            voltage_applied = 0.0f;
            if (motor.enabled) {
                motor.disable();
                // Keep nSLEEP asserted: nFAULT is the driver's own status line
                // and it is held low while the driver is asleep, which would
                // hide a fault that has already cleared.
                HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
            }
            if (!driver_fault_asserted() &&
                driver_fault_release_s >= tuning::simplefoc_driver_fault_quiet_s) {
                // The driver released its line: the fault was one that clears
                // by itself (a supply dip), so re-arm the loop.
                driver_fault_active = false;
                reset_torque_regulators();
                motor.enable();
                fault = MotorFault::NONE;
            }
        }
        else if (fault != MotorFault::INVALID_SUPPLY && fault != MotorFault::INVALID_COMMAND &&
                 !guard_parked && !guard.is_tripped()) {
            fault = MotorFault::NONE;
        }

        // "Send zero to rearm" (the six-step stall contract): the protection
        // latch clears once both channels command zero, and a bridge the guard
        // parked is pulsed back on here (the enable state otherwise belongs to
        // the serial `E` channel).
        if (guard.is_tripped() && voltage_target == 0.0f && speed_target == 0.0f) {
            guard.reset();
            trace_guard_counted = false;
            if (guard_parked) {
                guard_parked = false;
                serial_line_debug("simplefoc: guard re-armed");
                if (!driver_fault_active) {
                    start();
                }
            }
        }

        // Protection input: the common-mode-removed phase-current magnitude.
        // A locked rotor parks its current on the d-axis, where the q-current
        // the regulators close over reads ~0 — only the phase magnitude sees
        // it. Sampled before move()/loopFOC() drive the bridge this pass; the
        // blocking bench routines (initFOC alignment, `A` characterisation)
        // suspend update() entirely, so their drive currents cannot trip it.
        const PhaseCurrent_s phase = current_sense.getPhaseCurrents();
        const bool command_active = motor.enabled &&
            ((!speed_mode && voltage_target != 0.0f) ||
             (speed_mode && speed_target != 0.0f));
        const current_guard::Fault guard_fault = guard.step(
            current_guard::phase_current_magnitude(phase.a, phase.b, phase.c),
            motor.shaft_velocity,
            command_active,
            dt_s
        );
        if (guard.is_tripped()) {
            if (!trace_guard_counted) {
                trace_guard_counted = true;
                if (guard_fault == current_guard::Fault::OVERCURRENT) {
                    ++guard_overcurrent_count;
                }
                else {
                    ++guard_stalled_count;
                }
            }
            fault = guard_fault == current_guard::Fault::OVERCURRENT ?
                MotorFault::OVERCURRENT : MotorFault::STALLED;
            voltage_applied = 0.0f;
            coast_current_a = 0.0f;
            if (motor.enabled) {
                motor.disable();
                HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
            }
            guard_parked = true;
        }

        // move() computes the set-points, loopFOC() realises them: run move
        // first, and only on this path — in speed mode move() runs from the
        // fixed 200 Hz slot instead. The ramp only advances while enabled, so
        // a target commanded while parked applies as a soft start on E1.
        if (coast_mode) {
            // Zero torque, still current-controlled: holding the q-current at
            // zero makes the current loops apply exactly the back-EMF, so the
            // wheel freewheels with no braking current and no regen. Slewed so
            // the torque does not step off.
            if (motor.enabled && !guard.is_tripped()) {
                coast_current_a = ramp_towards(
                    coast_current_a, 0.0f, torque_ramp_a_s * dt_s
                );
            }
            motor.move(coast_current_a);
        }
        else if (!speed_mode) {
            if (motor.enabled && !guard.is_tripped()) {
                // Soft current ceiling: while the filtered phase current is
                // over the current limit the ceiling integrates down (and the
                // applied voltage slews to it fast); under 80% of the limit
                // it recovers at the ramp rate — so a step command accelerates
                // the motor current-limited instead of at V/phase-R.
                const float soft_limit = motor.current_limit;
                if (guard.filtered_current() > soft_limit) {
                    voltage_ceiling_v = std::max(
                        0.0f,
                        voltage_ceiling_v - tuning::simplefoc_oc_backoff_v_s * dt_s
                    );
                }
                else if (guard.filtered_current() < soft_limit * 0.8f) {
                    voltage_ceiling_v = std::min(
                        motor.voltage_limit,
                        voltage_ceiling_v + tuning::simplefoc_voltage_ramp_v_s * dt_s
                    );
                }
                const float requested_magnitude = std::abs(voltage_target);
                const float magnitude = std::min(requested_magnitude, voltage_ceiling_v);
                // A zero command is an operator stop, not a fault: shed it at
                // the gentle rate so the wheel follows the voltage down instead
                // of the 0 V phase short braking it against its own back-EMF.
                // The same gentle rate covers a change of sign (a reversal
                // through 0 V); only a drop the current ceiling forces uses the
                // fast down-slew, and only while the sign is unchanged.
                const bool ceiling_forced = magnitude < requested_magnitude;
                voltage_applied = voltage_slew::slew(
                    voltage_applied,
                    std::copysign(magnitude, voltage_target),
                    tuning::simplefoc_voltage_ramp_v_s,
                    ceiling_forced ? tuning::simplefoc_voltage_down_ramp_v_s
                                   : tuning::simplefoc_voltage_stop_ramp_v_s,
                    tuning::simplefoc_voltage_stop_ramp_v_s,
                    dt_s
                );
            }
            motor.move(voltage_applied);
        }

        motor.loopFOC();

        // Telemetry current: while running in current mode loopFOC has just
        // filtered the measurement into motor.current; when disabled (loopFOC
        // stops updating it) or in voltage mode take one unfiltered sample
        // (the ADC is 16x oversampled already).
        if (motor.enabled && motor.torque_controller == TorqueControlType::foc_current) {
            torque_current_a = motor.current.q;
        }
        else {
            torque_current_a = current_sense.getFOCCurrents(motor.electrical_angle).q;
        }

        // A target past the modulation ceiling is refused instead of chased
        // (the current loops have no authority there), so say so once per
        // clamp episode rather than letting the wheel quietly under-run.
        const bool clamped = is_target_clamped();
        if (clamped && !clamp_logged) {
            ++target_clamp_count;
            serial_line_debug("simplefoc: target clamped, ceiling (m/s)", get_speed_ceiling());
        }
        clamp_logged = clamped;

        // Which regulator ran out of authority this pass. The velocity loop
        // output is the q-current demand; the current loops output phase volts,
        // and each saturating at its own limit is a different explanation for a
        // wheel that will not hold its speed.
        const float current_demand = std::abs(motor.current_sp);
        const bool at_current_limit = speed_mode && motor.enabled &&
            motor.PID_velocity.limit > 0.0f &&
            current_demand >= motor.PID_velocity.limit * 0.999f;
        if (at_current_limit && !current_limit_active) {
            ++current_limit_count;
        }
        current_limit_active = at_current_limit;

        const float current_loop_limit = motor.PID_current_q.limit;
        const bool at_voltage_limit = motor.enabled && current_loop_limit > 0.0f &&
            (std::abs(motor.voltage.q) >= current_loop_limit * 0.999f ||
             std::abs(motor.voltage.d) >= current_loop_limit * 0.999f);
        if (at_voltage_limit && !voltage_limit_active) {
            ++voltage_limit_count;
        }
        voltage_limit_active = at_voltage_limit;

        push_trace_sample(now);
    }

    void set_voltage_point(float volts) override {
        speed_mode = false;
        motor.controller = MotionControlType::torque;
        if (volts == 0.0f) {
            // A zero open-loop command (`X`, `Q0`) is the operator stop: coast
            // through the current loop rather than holding 0 V, which would
            // short the windings against their back-EMF (~15 A at 0.3 m/s).
            voltage_target = 0.0f;
            voltage_applied = 0.0f;
            motor.torque_controller = TorqueControlType::foc_current;
            if (!coast_mode) {
                coast_mode = true;
                ++coast_entry_count;
                coast_current_a = motor.enabled ? motor.current.q : 0.0f;
            }
            reset_torque_regulators();
            return;
        }
        coast_mode = false;
        // The open-loop channel is volts, not amps: leave current mode, or the
        // number would be applied as a q-current set-point.
        motor.torque_controller = TorqueControlType::voltage;
        if (!std::isfinite(volts)) {
            // A malformed payload (Cyphal 4050 carries a raw float) would park
            // the ramp: a NaN target survives ramp_towards and freezes the
            // bridge at the last applied voltage. Stop the channel and report
            // the command, the same fault the six-step backend raises.
            voltage_applied = 0.0f;
            voltage_target = 0.0f;
            reset_torque_regulators();
            fault = MotorFault::INVALID_COMMAND;
            return;
        }
        // Ramp from whatever the loop is applying now, so the switch itself is
        // seamless.
        voltage_applied = motor.enabled ? motor.voltage.q : 0.0f;
        voltage_target = volts;
        reset_torque_regulators();
    }

    void run_speed_loop(float) override {
        if (!speed_mode) {
            return;
        }
        motor.move(speed_target);
        // Braking-current ceiling: a torque that opposes the measured rotation
        // is regenerative — it pumps the rail up, and the bench supply neither
        // sinks nor tolerates being back-driven (measured: 19..21 V trips it,
        // the rail then collapses and the gate driver drops out on
        // undervoltage). The demand is clamped to brake_current_a against the
        // motion instead of being zeroed: a bounded, smooth brake keeps the
        // loop stable (a zero-clamp limit-cycles the overshoot correction into
        // the audible kicks), and 0 A restores a pure coast. Below the stall
        // guard's velocity threshold the wheel counts as stopped and the full
        // current limit applies both ways, so a reversal finishes through
        // standstill.
        const float motion = motor.shaft_velocity;
        const float demand = motor.current_sp;
        bool clipped = false;
        if (motion > stopped_wheel_rate) {
            if (demand < -brake_current_a) {
                motor.current_sp = -brake_current_a;
                clipped = true;
            }
        }
        else if (motion < -stopped_wheel_rate) {
            if (demand > brake_current_a) {
                motor.current_sp = brake_current_a;
                clipped = true;
            }
        }
        // Episodes, not passes: the clamp stays engaged for as long as the
        // loop keeps asking for more brake than the regen ceiling allows.
        if (clipped && !brake_clamp_active) {
            ++brake_clamp_count;
        }
        brake_clamp_active = clipped;
    }

    void set_velocity_pid(const PIDConfig& config) override {
        velocity_pid = config;
        apply_velocity_pid();
    }

    // The brake clamp's "stopped" threshold follows the stall guard's velocity
    // ceiling, so one config id (10) drives both.
    void set_current_guard(const current_guard::Config& config) override {
        MotorBackend::set_current_guard(config);
        stopped_wheel_rate = config.stall_velocity_rad_s;
    }

    bool auto_calibrate() override {
        if (!motor.enabled) {
            serial_line_debug("simplefoc calibrate: driver disabled");
            return false;
        }
        // Bench-only and blocking for a few seconds: characteriseMotor drives
        // the phases directly, and because this runs from the serial poll the
        // whole control loop stands still while it does (nothing fights the
        // measurement waveforms, and the phases are back at zero volts when it
        // returns). The BLDCMotor wrapper pins the 3-phase correction factor,
        // so reach the base overload to keep both knobs bench-tunable.
        serial_line_debug("simplefoc calibrate: measuring R and L");
        const int characterise_result = motor.FOCMotor::characteriseMotor(
            tuning::simplefoc_characterise_voltage_v,
            tuning::simplefoc_characterise_correction
        );
        if (characterise_result != 0) {
            // The library's own error text is compiled out
            // (SIMPLEFOC_DISABLE_DEBUG): 1 = current sense not initialised,
            // 2 = bad voltage, 3 = phase current too low, 4 = estimated R<=0.
            serial_line_debug("simplefoc calibrate: characterise failed", (float)characterise_result);
            return false;
        }
        serial_line_debug("simplefoc calibrate: phase_resistance (ohm)", motor.phase_resistance);
        serial_line_debug("simplefoc calibrate: inductance_d (mH)", motor.axis_inductance.d * 1000.0f);
        serial_line_debug("simplefoc calibrate: inductance_q (mH)", motor.axis_inductance.q * 1000.0f);
        if (motor.tuneCurrentController(tuning::simplefoc_current_bandwidth_hz) != 0) {
            serial_line_debug("simplefoc calibrate: current tune failed");
            return false;
        }
        serial_line_debug("simplefoc calibrate: current_kp", motor.PID_current_q.P);
        serial_line_debug("simplefoc calibrate: current_ki", motor.PID_current_q.I);
        serial_line_debug("simplefoc calibrate: current_lpf_tf (s)", motor.LPF_current_q.Tf);
        // The tuned gains live until reboot: bake them into motor_tuning.h
        // (simplefoc_current_kp/ki) once the bench run looks good.
        return true;
    }

    float get_velocity() const override {
        return motor.shaft_velocity;
    }

    float get_wheel_angle() const override {
        return (float)sensor.get_counts() * wheel_rad_per_count;
    }

    MotorFault get_fault() const override {
        return fault;
    }

    float get_torque_current() override {
        return torque_current_a;
    }

    bool get_diagnostics(MotorDiagnostics& out) override {
        out.speed_target_m_s = get_applied_speed_target();
        out.q_current_setpoint_a = speed_mode ? motor.current_sp : 0.0f;
        if (motor.enabled) {
            // The regulators close over the filtered d/q currents, so those are
            // what tells a torque ripple from an angle fault; the phase
            // magnitude is one unfiltered sample of the same front end.
            out.q_current_a = motor.current.q;
            out.d_current_a = motor.current.d;
            const PhaseCurrent_s phase = current_sense.getPhaseCurrents();
            out.phase_current_a = current_guard::phase_current_magnitude(
                phase.a, phase.b, phase.c
            );
            out.q_voltage_v = motor.voltage.q;
            out.d_voltage_v = motor.voltage.d;
        }
        out.bus_voltage_v = inverter.get_busV();
        out.hall_counts = sensor.get_counts();
        return true;
    }

protected:
    void set_wheel_speed_target(float wheel_rad_s) override {
        if (wheel_rad_s == 0.0f) {
            // A zero speed command is an operator stop, not a regulation
            // problem: coast by holding a zero q-current instead of asking the
            // loop to brake. Active braking slams the gearbox through zero
            // speed (the audible hit when a moving wheel is stopped) and pushes
            // the energy back into the supply, whose rail then rises and cuts
            // out; a voltage-mode ramp is worse still — it leaves the windings
            // across a decaying back-EMF, which is a 15 A brake at 0.3 m/s.
            if (speed_mode) {
                speed_mode = false;
                motor.controller = MotionControlType::torque;
                motor.torque_controller = TorqueControlType::foc_current;
                coast_mode = true;
                ++coast_entry_count;
                coast_current_a = motor.current_sp;
                reset_torque_regulators();
            }
            speed_target = 0.0f;
            return;
        }
        if (!speed_mode) {
            speed_mode = true;
            coast_mode = false;
            motor.controller = MotionControlType::velocity;
            motor.torque_controller = TorqueControlType::foc_current;
            // A stale integral from the previous mode must not kick the motor;
            // the reset also restarts the torque ramp from zero.
            reset_torque_regulators();
        }
        speed_target = wheel_rad_s;
    }

    bool set_velocity_lpf(float tf_s) override {
        if (!(tf_s >= 0.0f) || tf_s > 1.0f) {
            return false;
        }
        motor.LPF_velocity.Tf = tf_s;
        return true;
    }

    bool set_torque_ramp(float a_s) override {
        if (!(a_s > 0.0f) || a_s > 1000.0f) {
            return false;
        }
        torque_ramp_a_s = a_s;
        apply_velocity_pid();
        return true;
    }

    bool set_brake_current(float a) override {
        if (!(a >= 0.0f) || a > 40.0f) {
            return false;
        }
        brake_current_a = a;
        return true;
    }

    void reset_speed_regulator() override {
        motor.PID_velocity.reset();
    }
};

}

std::unique_ptr<MotorBackend> make_motor_backend(
    DriveRuntimeConfig& runtime_config,
    DriveInfo& drive_info,
    TIM_HandleTypeDef* htim,
    BaseInverter& inverter,
    HallSensor& hall,
    const PIDConfig& velocity_pid,
    float wheel_radius_m
) {
    return std::make_unique<SimpleFocBackend>(
        runtime_config,
        drive_info,
        htim,
        inverter,
        hall,
        velocity_pid,
        wheel_radius_m
    );
}
