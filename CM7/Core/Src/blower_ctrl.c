/**
  ******************************************************************************
  * @file    blower_ctrl.c
  * @brief   Gain-scheduled pressure-control PID for the CPAP blower.
  *
  *  Design notes (read these before tuning)
  *  ---------------------------------------
  *  1. Units: setpoint and measurement are in cmH2O (the unit clinicians
  *     and the device GUI use). The mask pressure sensor (hamsPS) returns
  *     mbar; we convert via 1 cmH2O = 0.980665 mbar.
  *
  *  2. Three gain zones: LOW (4-8), MID (8-15), HIGH (15-25). At low
  *     therapy pressures the loop is highly sensitive (steep dP/dRPM), so
  *     gains are small. At high pressures gains rise to keep the time
  *     constant roughly uniform across the therapy range.
  *
  *  3. Feed-forward (rpm_ff): a coarse linear estimate of the steady-state
  *     RPM needed to hold a given pressure. With FF active, the integrator
  *     only has to handle residual error - this dramatically improves
  *     ramp-up time when the user changes the setpoint. Set
  *     g_blower_ctrl_ff_enable=0 to disable it (pure PID).
  *
  *  4. Anti-windup: classic conditional integration. We only accumulate
  *     the integrator when the output is NOT saturated, and when the
  *     accumulation would not push the output further into saturation.
  *
  *  5. Derivative on measurement (not error): prevents derivative kick
  *     when the user steps the setpoint. The setpoint slew limiter would
  *     also smooth a step but D-on-M is robust regardless.
  *
  *  6. Setpoint slew: the user-facing setpoint can step instantly (e.g. a
  *     ramp profile or pressure relief), but the *internal* setpoint that
  *     feeds the PID is slew-limited so we never command the loop to do
  *     something physically impossible.
  *
  *  7. Output post-filter: the raw PID output is mildly low-passed (1st
  *     order) to remove sensor noise before posting to the IPC. Disable
  *     by setting g_blower_ctrl_out_filter_alpha = 1.0f.
  *
  *  Default gains
  *  -------------
  *  These are CONSERVATIVE starting points. Expect to tune Kp/Ki up at
  *  least 2-4x once you see the actual mask + hose + leak response. Keep
  *  Kd small initially; aggressive D-term will amplify the 200 Hz sensor
  *  noise. The recommended bring-up procedure:
  *      a. Set ki=0, kd=0; raise kp until the response shows mild
  *         oscillation, then back off ~30%.
  *      b. Raise ki until steady-state error is removed within 200-500 ms.
  *      c. Add a tiny kd (5-30) only if you see overshoot you cannot
  *         dial out by reducing kp.
  ******************************************************************************
  */

#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "blower_osc.h"
#include "pressure_sensors.h"
#include "flow_sensor.h"
#include "tube_comp.h"

#include <math.h>
#include <stdint.h>

/* === External plumbing === */
extern AMS5935_Data_t measPS;     /* mask/patient pressure sensor (mbar) */

/* === Conversion === */
#define MBAR_PER_CMH2O   0.980665f
#define MBAR_TO_CMH2O(x) ((x) * (1.0f / MBAR_PER_CMH2O))

/* === Tunable gains (volatile so they live in a debugger-writable global) ===
 *
 *  Default values were chosen ASSUMING the sqrt-based feed-forward below is
 *  active and well-calibrated. With FF carrying the steady-state RPM, the
 *  PID only needs to handle disturbances (leaks, mask resistance, breath
 *  cycle) so Kp can be modest and Ki can be small. If you turn FF off
 *  (g_blower_ctrl_ff_enable=0) you will need to re-tune Kp/Ki up by 2-3x.
 */
/* Low zone: 4 - 8 cmH2O.
 *
 * Kp was bumped 300 -> 700 (2.3x) for breath-cycle disturbance rejection.
 * A 2 cmH2O inhale dip at Kp=300 produced only +600 RPM correction; at
 * Kp=700 it now produces +1400, which the motor can deliver inside the
 * ~300 ms inhale window. Ki went 100 -> 250 (similar reasoning -
 * residual error during the breath has to be cleared in <0.5 s). */
volatile float g_blower_ctrl_kp_low  = 700.0f;
volatile float g_blower_ctrl_ki_low  = 250.0f;
volatile float g_blower_ctrl_kd_low  = 2.0f;

/* Mid zone: 8 - 15 cmH2O. */
volatile float g_blower_ctrl_kp_mid  = 1000.0f;
volatile float g_blower_ctrl_ki_mid  = 350.0f;
volatile float g_blower_ctrl_kd_mid  = 5.0f;

/* High zone: 15 - 25 cmH2O. */
volatile float g_blower_ctrl_kp_high = 1500.0f;
volatile float g_blower_ctrl_ki_high = 500.0f;
volatile float g_blower_ctrl_kd_high = 10.0f;

/* === Feed-forward (open-loop steady-state RPM model) =======================
 *
 *  Centrifugal blowers obey P ~ K * w^2, so the inverse mapping that gives
 *  us "the RPM needed to hold a given pressure" is a square root. The
 *  default coefficient was extracted from a calibration sweep on this
 *  exact board (constant ubBlrFlg, varying usSpeed, no breathing load):
 *
 *      RPM    P(mbar)  P(cmH2O)  rpm/sqrt(P_cmh2o)
 *      ----------------------------------------------
 *      12000   3.775    3.85         6116
 *      15000   5.870    5.99         6131
 *      18000   8.420    8.59         6143
 *      21000  11.450   11.68         6144
 *      24000  14.950   15.25         6147
 *      27000  18.900   19.27         6151
 *      30000  23.300   23.76         6156
 *      33000  28.200   28.76         6154
 *      36000  33.500   34.16         6160
 *
 *  Average K_sqrt ~6141 across the 4-25 cmH2O therapy range (scatter
 *  <0.5%). Below 9 krpm the linear/static-friction term dominates so the
 *  ratio falls; the therapeutic range is far above that knee.
 *
 *  Re-calibrate per-mask/per-hose if you find the integrator pegging in
 *  one direction in steady state (it means K_sqrt is off for your test
 *  setup).
 *
 *      rpm_ff = K_sqrt * sqrt(max(0, setpoint_cmh2o)) + offset_rpm
 */
volatile uint8_t g_blower_ctrl_ff_enable           = 1U;
volatile float   g_blower_ctrl_ff_k_rpm_per_sqrtcmh = 6141.0f;
volatile float   g_blower_ctrl_ff_offset_rpm        = 0.0f;

/* Setpoint slew rate. 6 cmH2O/s lets a 0 -> 8 cmH2O step finish in
 * ~1.3 s, which is fast enough to feel responsive but gentle enough that
 * the integrator can track the rise without winding up - a faster slew
 * was causing a +1 cmH2O overshoot followed by a long settle as the
 * integrator unwound. Raise to 25 for deliberate bench step-response
 * testing. */
volatile float g_blower_ctrl_slew_cmh2o_per_s = 6.0f;

/* Output post-filter (1st order IIR). 1.0 = pass-through, 0.2 = heavy.
 *
 * Set to 1.0 (pass-through) so that PID output reaches the motor with
 * zero filter lag - we need every millisecond of bandwidth to fight
 * breath-cycle pressure swings. The motor's own mechanical inertia
 * provides any smoothing the system needs. Drop to 0.5-0.7 only if
 * downstream noise (audible whine) becomes a problem. */
volatile float g_blower_ctrl_out_filter_alpha = 1.0f;

/* RPM clamp. Defaults to the IPC ceiling but kept tunable in case you
 * want to limit the blower further during development. */
volatile int32_t g_blower_ctrl_rpm_max = (int32_t)BLOWER_IPC_MAX_SPEED_RPM;
volatile int32_t g_blower_ctrl_rpm_min = 0;          /* never reverse */

/* Integrator deadband and magnitude clamp (anti-windup).
 *
 *   - Deadband: |error| > deadband -> freeze integrator. This stops the
 *     I-term from winding up during the slewing phase of a step change
 *     (where P+FF should carry the load) and dedicates I purely to
 *     steady-state trim near the target. Default 2.5 cmH2O comfortably
 *     covers the breath-cycle pressure swing on any normal mask.
 *   - Magnitude clamp: hard limit on |I| in RPM. Even a very leaky
 *     circuit shouldn't need more than 4 krpm of integrator trim - past
 *     that is windup, not real correction.
 *
 * Both are tunable from the watch window. Set deadband to 1e6 to
 * recover the original "always integrate" behaviour. */
volatile float   g_blower_ctrl_integ_deadband_cmh2o = 2.5f;
volatile float   g_blower_ctrl_integ_max_abs_rpm    = 4000.0f;

/* === Public configuration (debugger-visible) === */
volatile uint8_t g_blower_ctrl_enabled        = 0U;       /* default OFF */
volatile float   g_blower_ctrl_setpoint_user  = 6.0f;     /* cmH2O */

/* === State (debugger-visible) === */
volatile float   g_blower_ctrl_setpoint_slew  = 0.0f;
volatile float   g_blower_ctrl_meas_cmh2o     = 0.0f;
volatile float   g_blower_ctrl_error_cmh2o    = 0.0f;
volatile float   g_blower_ctrl_integ          = 0.0f;
volatile float   g_blower_ctrl_deriv_rpm_per_s = 0.0f;
volatile float   g_blower_ctrl_p_term         = 0.0f;
volatile float   g_blower_ctrl_i_term         = 0.0f;
volatile float   g_blower_ctrl_d_term         = 0.0f;
volatile float   g_blower_ctrl_ff_term        = 0.0f;
volatile float   g_blower_ctrl_out_unfilt     = 0.0f;
volatile float   g_blower_ctrl_out_filt       = 0.0f;
volatile int32_t g_blower_ctrl_out_rpm        = 0;
volatile uint8_t g_blower_ctrl_zone           = (uint8_t)BLOWER_CTRL_ZONE_LOW;
volatile uint8_t g_blower_ctrl_saturated      = 0U;
volatile uint32_t g_blower_ctrl_step_count    = 0U;

/* Previous-tick measurement for derivative-on-measurement. */
static float s_prev_meas_cmh2o = 0.0f;
static bool  s_prev_meas_valid = false;

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Hysteretic gain-scheduling: pick the new zone given the slewed setpoint
 * and the previously-active zone. The hysteresis band is small (0.5
 * cmH2O); this only matters if a setpoint sits exactly on a boundary. */
static BlowerCtrlZone_t selectZone(float setpoint_cmh2o, BlowerCtrlZone_t prev)
{
    const float h = BLOWER_CTRL_ZONE_HYST_CMH2O;
    const float low_hi = BLOWER_CTRL_ZONE_LOW_HI_CMH2O;
    const float mid_hi = BLOWER_CTRL_ZONE_MID_HI_CMH2O;

    switch (prev)
    {
    case BLOWER_CTRL_ZONE_LOW:
        if (setpoint_cmh2o > (low_hi + h)) return BLOWER_CTRL_ZONE_MID;
        return BLOWER_CTRL_ZONE_LOW;
    case BLOWER_CTRL_ZONE_MID:
        if (setpoint_cmh2o < (low_hi - h)) return BLOWER_CTRL_ZONE_LOW;
        if (setpoint_cmh2o > (mid_hi + h)) return BLOWER_CTRL_ZONE_HIGH;
        return BLOWER_CTRL_ZONE_MID;
    case BLOWER_CTRL_ZONE_HIGH:
    default:
        if (setpoint_cmh2o < (mid_hi - h)) return BLOWER_CTRL_ZONE_MID;
        return BLOWER_CTRL_ZONE_HIGH;
    }
}

static void getActiveGains(BlowerCtrlZone_t z, float *kp, float *ki, float *kd)
{
    switch (z)
    {
    case BLOWER_CTRL_ZONE_LOW:
        *kp = g_blower_ctrl_kp_low;
        *ki = g_blower_ctrl_ki_low;
        *kd = g_blower_ctrl_kd_low;
        break;
    case BLOWER_CTRL_ZONE_HIGH:
        *kp = g_blower_ctrl_kp_high;
        *ki = g_blower_ctrl_ki_high;
        *kd = g_blower_ctrl_kd_high;
        break;
    case BLOWER_CTRL_ZONE_MID:
    default:
        *kp = g_blower_ctrl_kp_mid;
        *ki = g_blower_ctrl_ki_mid;
        *kd = g_blower_ctrl_kd_mid;
        break;
    }
}

/* === Public API === */

void BlowerCtrl_Init(void)
{
    g_blower_ctrl_setpoint_slew  = g_blower_ctrl_setpoint_user;
    g_blower_ctrl_meas_cmh2o     = 0.0f;
    g_blower_ctrl_error_cmh2o    = 0.0f;
    g_blower_ctrl_integ          = 0.0f;
    g_blower_ctrl_deriv_rpm_per_s = 0.0f;
    g_blower_ctrl_p_term         = 0.0f;
    g_blower_ctrl_i_term         = 0.0f;
    g_blower_ctrl_d_term         = 0.0f;
    g_blower_ctrl_ff_term        = 0.0f;
    g_blower_ctrl_out_unfilt     = 0.0f;
    g_blower_ctrl_out_filt       = 0.0f;
    g_blower_ctrl_out_rpm        = 0;
    g_blower_ctrl_zone           = (uint8_t)BLOWER_CTRL_ZONE_LOW;
    g_blower_ctrl_saturated      = 0U;
    g_blower_ctrl_step_count     = 0U;

    s_prev_meas_cmh2o = 0.0f;
    s_prev_meas_valid = false;

    /* Co-init the FOT oscillator. Defaults to disabled so this is
     * invisible until the user explicitly turns it on. */
    BlowerOsc_Init();
}

void BlowerCtrl_SetEnable(bool en)
{
    if (en && !g_blower_ctrl_enabled)
    {
        /* Bumpless re-enable: snap the slewed setpoint to the current
         * measurement and seed the integrator/output filter so we don't
         * jump-start the blower with a huge first command. */
        g_blower_ctrl_setpoint_slew = g_blower_ctrl_meas_cmh2o;
        g_blower_ctrl_integ         = 0.0f;
        g_blower_ctrl_out_filt      = (float)g_blower_ctrl_out_rpm;
        s_prev_meas_valid           = false;
    }
    g_blower_ctrl_enabled = en ? 1U : 0U;
}

bool BlowerCtrl_IsEnabled(void)
{
    return (g_blower_ctrl_enabled != 0U);
}

void BlowerCtrl_SetSetpoint(float cmh2o)
{
    g_blower_ctrl_setpoint_user = clampf(cmh2o,
                                         BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                         BLOWER_CTRL_SETPOINT_MAX_CMH2O);
}

float BlowerCtrl_GetSetpoint(void)        { return g_blower_ctrl_setpoint_user; }
float BlowerCtrl_GetSetpointSlewed(void)  { return g_blower_ctrl_setpoint_slew; }
float BlowerCtrl_GetMeasured(void)        { return g_blower_ctrl_meas_cmh2o; }
float BlowerCtrl_GetOutputRpm(void)       { return (float)g_blower_ctrl_out_rpm; }

void BlowerCtrl_Step(float dt_s)
{
    /* Refuse silly dt values - bad dt is the #1 cause of PID misbehaviour. */
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    /* --- 1. Pull the latest measurement and convert to cmH2O ---
     *
     *  We then push the sensor reading through the tube-loss compensator
     *  so the PID closes the loop on the *mask*-side pressure estimate
     *  rather than the device-side sensor reading. When TubeComp is
     *  disabled (boot default), this is a no-op pass-through and
     *  meas_cmh2o equals the raw sensor reading. */
    const float meas_mbar       = measPS.pressure;
    const float p_sensor_cmh2o  = MBAR_TO_CMH2O(meas_mbar);
    const float q_slm           = Flow_GetSlm();
    const float meas_cmh2o      = TubeComp_Apply(p_sensor_cmh2o, q_slm);
    g_blower_ctrl_meas_cmh2o    = meas_cmh2o;

    /* --- 2. Slew the user setpoint toward the target --- */
    {
        const float target = clampf(g_blower_ctrl_setpoint_user,
                                    BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                    BLOWER_CTRL_SETPOINT_MAX_CMH2O);
        const float max_step = g_blower_ctrl_slew_cmh2o_per_s * dt_s;
        float       sp       = g_blower_ctrl_setpoint_slew;
        const float diff     = target - sp;

        if (diff >  max_step)      sp += max_step;
        else if (diff < -max_step) sp -= max_step;
        else                       sp  = target;

        g_blower_ctrl_setpoint_slew = sp;
    }

    /* --- 3. If disabled, just track the measurement and bail out --- */
    if (g_blower_ctrl_enabled == 0U)
    {
        s_prev_meas_cmh2o = meas_cmh2o;
        s_prev_meas_valid = true;
        return;
    }

    /* --- 4. Pick the active gain set ---
     * Schedule on the *DC* (slewed) setpoint, not on the slew+osc value:
     * otherwise a 4 Hz oscillation that crosses a zone boundary (e.g.
     * setpoint=8 cmH2O with 1 cmH2O amplitude crosses LOW/MID at 8) would
     * flap the gain set 8x/s. */
    BlowerCtrlZone_t zone = selectZone(g_blower_ctrl_setpoint_slew,
                                       (BlowerCtrlZone_t)g_blower_ctrl_zone);
    g_blower_ctrl_zone = (uint8_t)zone;

    float kp, ki, kd;
    getActiveGains(zone, &kp, &ki, &kd);

    /* --- 5. PID terms ---
     * The actual pressure target tracked by the PID is
     *     setpoint = slewed_DC + sinusoidal_perturbation
     * The slew limiter has already smoothed user-driven setpoint changes
     * and is therefore upstream of the oscillator: we *want* the AC
     * component to pass through unattenuated so the closed loop produces
     * a clean pressure perturbation that a future flow sensor can use to
     * compute airway impedance. */
    const float osc      = BlowerOsc_StepAndGetSample(dt_s);
    const float setpoint = g_blower_ctrl_setpoint_slew + osc;
    const float error    = setpoint - meas_cmh2o;
    g_blower_ctrl_error_cmh2o = error;

    /* P. */
    const float p_term = kp * error;

    /* D-on-measurement (negated; positive d/dt of meas pushes output down). */
    float d_meas_per_s = 0.0f;
    if (s_prev_meas_valid)
    {
        d_meas_per_s = (meas_cmh2o - s_prev_meas_cmh2o) / dt_s;
    }
    g_blower_ctrl_deriv_rpm_per_s = d_meas_per_s;
    const float d_term = -kd * d_meas_per_s;

    /* Feed-forward (open-loop estimate of steady-state RPM).
     * Square-root law derived from the bench calibration table at the
     * top of this file: rpm = K_sqrt * sqrt(P_cmh2o) + offset.
     * Negative setpoints clamp to 0 so we never feed sqrtf() a negative
     * arg (which would return NaN and poison the loop). */
    float ff_term = 0.0f;
    if (g_blower_ctrl_ff_enable != 0U)
    {
        float ff_input = (setpoint > 0.0f) ? setpoint : 0.0f;
        ff_term = g_blower_ctrl_ff_k_rpm_per_sqrtcmh * sqrtf(ff_input)
                + g_blower_ctrl_ff_offset_rpm;
    }

    /* I-term: provisionally update integrator, then anti-windup checks.
     *
     * Anti-windup layer 1: deadband. Outside |error| > deadband, freeze
     * the integrator entirely - the FF and P terms carry the transient,
     * so I has no business growing during the slew/rise. */
    float i_step = ki * error * dt_s;
    if (fabsf(error) > g_blower_ctrl_integ_deadband_cmh2o)
    {
        i_step = 0.0f;
    }

    float integ_new = g_blower_ctrl_integ + i_step;

    /* Compute provisional output with the candidate integrator. */
    float out_unfilt = ff_term + p_term + integ_new + d_term;

    /* Anti-windup layer 2: conditional integration on output saturation.
     * If the unsaturated output is outside the rpm window AND the new
     * integrator step is pushing us deeper into saturation, reject the
     * integrator step (don't wind up). */
    const float rpm_min_f = (float)g_blower_ctrl_rpm_min;
    const float rpm_max_f = (float)g_blower_ctrl_rpm_max;
    bool        sat_high  = (out_unfilt > rpm_max_f) && (i_step > 0.0f);
    bool        sat_low   = (out_unfilt < rpm_min_f) && (i_step < 0.0f);
    if (sat_high || sat_low)
    {
        integ_new = g_blower_ctrl_integ;                 /* freeze   */
        out_unfilt = ff_term + p_term + integ_new + d_term;
    }

    /* Anti-windup layer 3: hard magnitude clamp on the integrator
     * itself. Even after the deadband and saturation guards above, a
     * pathological case (sensor stuck just inside the deadband for
     * minutes) could let I drift unboundedly. The clamp guarantees we
     * can never have more than `integ_max_abs` worth of I-term to
     * unwind. */
    if (integ_new > g_blower_ctrl_integ_max_abs_rpm)
    {
        integ_new = g_blower_ctrl_integ_max_abs_rpm;
        out_unfilt = ff_term + p_term + integ_new + d_term;
    }
    else if (integ_new < -g_blower_ctrl_integ_max_abs_rpm)
    {
        integ_new = -g_blower_ctrl_integ_max_abs_rpm;
        out_unfilt = ff_term + p_term + integ_new + d_term;
    }

    /* Commit and clamp. */
    g_blower_ctrl_integ      = integ_new;
    g_blower_ctrl_p_term     = p_term;
    g_blower_ctrl_i_term     = integ_new;
    g_blower_ctrl_d_term     = d_term;
    g_blower_ctrl_ff_term    = ff_term;
    g_blower_ctrl_out_unfilt = out_unfilt;

    const float out_clamped = clampf(out_unfilt, rpm_min_f, rpm_max_f);
    g_blower_ctrl_saturated  = (out_clamped != out_unfilt) ? 1U : 0U;

    /* --- 6. Output low-pass filter --- */
    float       a = g_blower_ctrl_out_filter_alpha;
    if (a < 0.05f) a = 0.05f;
    if (a > 1.0f)  a = 1.0f;
    g_blower_ctrl_out_filt = a * out_clamped
                           + (1.0f - a) * g_blower_ctrl_out_filt;

    /* --- 7. Quantise and post --- */
    int32_t out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    out_rpm         = clampi(out_rpm,
                             g_blower_ctrl_rpm_min,
                             g_blower_ctrl_rpm_max);
    g_blower_ctrl_out_rpm = out_rpm;

    (void)BlowerIpc_CM7_SetSpeedRpmSilent(out_rpm);

    /* --- 8. Bookkeeping --- */
    s_prev_meas_cmh2o        = meas_cmh2o;
    s_prev_meas_valid        = true;
    g_blower_ctrl_step_count++;
}
