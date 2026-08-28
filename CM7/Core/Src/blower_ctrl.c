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
 *  3. Feed-forward: Physics FF is the default (hose + fan load on Q).
 *     Sqrt FF and flow FF are off; they are mutually exclusive with Physics
 *     FF. Dip FF can still run. Plateau/Hybrid profiles override these.
 *
 *  8. WM6850 reference mode: set g_blower_ctrl_wm6850_mode=1 to run the
 *     pressure_pid.c-style loop at 100 Hz (every 2nd SensorTask tick, dt=10 ms;
 *     gains scaled for cmH2O units), no FF, no setpoint slew, sensor-side
 *     measurement, I starts at 0. Speed cap uses sqrt bench calibration
 *     ×1.3 (not the dev-board interp_occ_speed table, ~6 krpm max).
 *
 *  9. CPAP Plateau preset (g_blower_ctrl_cpap_plateau_prefer): AirSense-like
 *     flat pressure platform — sqrt FF only, flow/physics/dip FF off, soft
 *     under/over-P gains, heavier meas/out filters. Auto-applied on CPAP
 *     therapy start/mode; restored when switching to Autoset or prefer=0.
 *
 * 10. CPAP Hybrid (g_blower_ctrl_cpap_hybrid_prefer): Physics FF tracks flow
 *     on inspiration; expiration holds vent-floor RPM (plateau-like). Mutex
 *     with Plateau. Toggle off if A/B shows little benefit.
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
#include "epr_ctrl.h"
#include "leak_estimator.h"
#include "therapy.h"

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
 * Tuned from open-loop FOPDT bench fit @ ~7.8 cmH2O (fopdt_fit.py, post-
 * reprofile motor): K=0.000934 cmH2O/RPM, tau=0.068 s, theta=0.009 s.
 * SIMC @ tau_c=0.60 s, gain-scheduled by K_sqrt/K at zone (LOW@8 fit). */
volatile float g_blower_ctrl_kp_low  = 1500.0f;
volatile float g_blower_ctrl_ki_low  = 4500.0f;
volatile float g_blower_ctrl_kd_low  = 2.0f;

/* Mid zone: 8 - 15 cmH2O (SIMC gain-scheduled @ MID@12). */
volatile float g_blower_ctrl_kp_mid  = 1200.0f;
volatile float g_blower_ctrl_ki_mid  = 4200.0f;
volatile float g_blower_ctrl_kd_mid  = 1.5f;

/* High zone: 15 - 25 cmH2O (SIMC gain-scheduled @ HIGH@20). */
volatile float g_blower_ctrl_kp_high = 1000.0f;
volatile float g_blower_ctrl_ki_high = 3900.0f;
volatile float g_blower_ctrl_kd_high = 0.8f;

/* WM6850 pressure_pid.c reference mode (see design note 8 above).
 * Gains mapped from fKP=1.2, fKI=26, fKD=0.06 with pressure in cmH2O×100
 * @ 100 Hz -> cmH2O @ 200 Hz (Kp/Ki scale; Kd ×100 for derivative units). */
volatile uint8_t g_blower_ctrl_wm6850_mode             = 0U;
volatile float   g_blower_ctrl_wm6850_kp               = 250.0f;
volatile float   g_blower_ctrl_wm6850_ki               = 3400.0f;
volatile float   g_blower_ctrl_wm6850_kd               = 1.0f;
volatile float   g_blower_ctrl_wm6850_meas_alpha       = 0.50f;
volatile float   g_blower_ctrl_wm6850_deriv_alpha      = 0.20f;
volatile float   g_blower_ctrl_wm6850_speed_lim_scale  = 1.30f;

/* === Feed-forward (open-loop steady-state RPM model) =======================
 *
 *  Centrifugal blowers obey P ~ K * w^2, so the inverse mapping that gives
 *  us "the RPM needed to hold a given pressure" is a square root. The
 *  default coefficient was extracted from a calibration sweep on this
 *  exact board (constant g_bench_blower_enable, varying g_bench_blower_speed_rpm):
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
volatile uint8_t g_blower_ctrl_ff_enable           = 0U;
volatile float   g_blower_ctrl_ff_k_rpm_per_sqrtcmh = 6141.0f;
volatile float   g_blower_ctrl_ff_offset_rpm        = 0.0f;

/* Flow disturbance feed-forward: proactive RPM boost on patient inhalation.
 * On this hardware Q_patient is positive on inhale (see telemetry plot).
 * g_blower_ctrl_flow_ff_inhale_sign: +1 = positive inhale, -1 = negative.
 * Uses the raw breath signal (fastest) for the target magnitude. */
volatile uint8_t g_blower_ctrl_flow_ff_enable     = 0U;
volatile int8_t  g_blower_ctrl_flow_ff_inhale_sign = 1;
volatile float   g_blower_ctrl_flow_ff_k_rpm_per_slm = 58.0f;
volatile float   g_blower_ctrl_flow_ff_max_rpm    = 3000.0f;
volatile float   g_blower_ctrl_flow_ff_deadband_slm = 3.0f;
volatile float   g_blower_ctrl_flow_ff_rise_rpm_per_s = 10000.0f;
volatile float   g_blower_ctrl_flow_ff_fall_rpm_per_s = 7000.0f;

/* Extra flow-FF from dQ_patient/dt at inhale onset (before flow magnitude
 * peaks). Catches the breath earlier than magnitude-only FF. */
volatile uint8_t g_blower_ctrl_flow_ff_onset_enable          = 0U;
volatile float   g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s = 6.0f;
volatile float   g_blower_ctrl_flow_ff_onset_thresh_slm_per_s    = 60.0f;
volatile float   g_blower_ctrl_flow_ff_onset_max_rpm             = 1200.0f;

/* Immediate RPM boost when raw sensor pressure is falling quickly
 * (catches inhale dips even if patient-flow sign or leak estimate lags). */
volatile uint8_t g_blower_ctrl_dip_ff_enable              = 1U;
volatile float   g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s = 400.0f;
volatile float   g_blower_ctrl_dip_ff_thresh_cmh2o_per_s   = 10.0f;
volatile float   g_blower_ctrl_dip_ff_max_rpm              = 1500.0f;
volatile float   g_blower_ctrl_dip_ff_err_min_cmh2o        = 0.04f;

/* Physics-based combined FF (mutually exclusive with sqrt + flow + dip FF):
 *      P_equiv = SP_mask + dP_hose(|Q|) + c_fan * max(Q,0)^2
 *      rpm_ff  = K_sqrt * sqrt(P_equiv) + offset
 * Uses tube-comp R_lam/R_turb for hose drop; c_fan models blower fan curve. */
volatile uint8_t g_blower_ctrl_physics_ff_enable = 1U;
/* Set by CPAP Hybrid profile while active (insp Physics FF / exp hold). */
volatile uint8_t g_blower_ctrl_cpap_hybrid_prefer = 0U;
volatile uint8_t g_blower_ctrl_cpap_hybrid_active = 0U;
/* Hybrid exp hold: only bleed RPM when P > setpoint + band; boost if below. */
volatile float   g_blower_ctrl_hybrid_exp_hold_band_cmh2o = 0.12f;
volatile float   g_blower_ctrl_hybrid_exp_fall_rpm_per_s  = 1500.0f;
volatile float   g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o = 2500.0f;
volatile float   g_blower_ctrl_hybrid_exp_boost_rpm_per_s = 8000.0f;
volatile uint8_t g_blower_ctrl_physics_ff_use_total_flow = 1U;
volatile uint8_t g_blower_ctrl_physics_ff_exhale_mit_enable = 1U;
volatile float   g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2 = 0.00012f;
volatile float   g_blower_ctrl_physics_ff_flow_lpf_hz = 4.0f;
volatile float   g_blower_ctrl_physics_ff_flow_lpf_inhale_hz = 12.0f;
volatile float   g_blower_ctrl_physics_ff_flow_lpf_exhale_hz = 10.0f;
volatile uint8_t g_blower_ctrl_physics_ff_inhale_mit_enable = 1U;
volatile uint8_t g_blower_ctrl_physics_ff_onset_enable = 1U;
volatile float   g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s = 4.0f;
volatile float   g_blower_ctrl_physics_ff_onset_thresh_slm_per_s = 80.0f;
volatile float   g_blower_ctrl_physics_ff_onset_max_rpm = 2500.0f;
volatile float   g_blower_ctrl_physics_ff_inhale_rise_boost = 2.0f;
volatile float   g_blower_ctrl_physics_ff_rise_rpm_per_s = 6000.0f;
volatile float   g_blower_ctrl_physics_ff_fall_rpm_per_s = 4000.0f;
volatile float   g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s = 8000.0f;
volatile uint8_t g_blower_ctrl_physics_ff_exp_tail_hold_enable = 1U;
volatile float   g_blower_ctrl_physics_ff_exp_tail_flow_slm = 3.0f;
volatile float   g_blower_ctrl_physics_ff_exp_tail_hold_cmh2o = 0.08f;
volatile float   g_blower_ctrl_physics_ff_vent_floor_rpm = 0.0f;
volatile float   g_blower_ctrl_physics_ff_p_equiv_cmh2o = 0.0f;
volatile float   g_blower_ctrl_physics_ff_hose_drop_cmh2o = 0.0f;
volatile float   g_blower_ctrl_physics_ff_fan_load_cmh2o  = 0.0f;
volatile float   g_blower_ctrl_physics_ff_q_filt_slm = 0.0f;
volatile float   g_blower_ctrl_physics_ff_rpm_raw = 0.0f;

/* During a pressure sag (setpoint - raw_meas > thresh), blend the PID
 * measurement toward the unfiltered tube-comp estimate so a heavy
 * meas_alpha (e.g. 0.9) does not hide the dip from P/D for 50-100 ms. */
volatile uint8_t g_blower_ctrl_fast_meas_enable       = 1U;
volatile float   g_blower_ctrl_fast_meas_thresh_cmh2o  = 0.05f;
volatile float   g_blower_ctrl_fast_meas_full_cmh2o     = 0.20f;

/* Extra P gain when below setpoint during inhalation (1.0 = off). */
volatile float   g_blower_ctrl_under_p_gain          = 1.5f;
volatile float   g_blower_ctrl_under_p_thresh_cmh2o  = 0.08f;

/* Extra P gain when above setpoint (1.0 = off; WM6850-style needs no boost). */
volatile float   g_blower_ctrl_over_p_gain          = 6.0f;
volatile float   g_blower_ctrl_over_p_thresh_cmh2o  = 0.12f;

/* Overpressure output brake (0 = disabled). */
volatile float   g_blower_ctrl_over_brake_rpm_per_s = 6000.0f;

/* EPR expiratory bump mitigation: zero disturbance FF + integrator bleed on
 * exhale onset, and gate flow/dip FF while EPR is at expiratory pressure.
 * Default off — set to 1 in debugger to re-enable. */
volatile uint8_t g_blower_ctrl_epr_exhale_mit_enable = 0U;

/* When EPR is off: do not command RPM below sqrt(P) steady-state for the
 * active slewed setpoint. Prevents post-inhale undershoot from flow-FF
 * release and negative D/I trim. Disabled automatically while EPR is on
 * (expiratory target is lower; floor will be added for EPR later). */
volatile uint8_t g_blower_ctrl_rpm_floor_enable = 1U;

/* Setpoint slew rate (down / general). 6 cmH2O/s for downward steps and
 * manual ramp-up from idle. EPR recovery uses g_blower_ctrl_slew_up. */
volatile float g_blower_ctrl_slew_cmh2o_per_s = 18.0f;

/* Faster slew when the user setpoint rises (EPR restore, inhale onset).
 * 18 cmH2O/s -> 3 cmH2O step in ~170 ms. */
volatile float g_blower_ctrl_slew_up_cmh2o_per_s = 18.0f;

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
volatile float   g_blower_ctrl_integ_deadband_cmh2o = 1.2f;
volatile float   g_blower_ctrl_integ_max_abs_rpm    = 4000.0f;

/* IIR on pressure before PID (WM6850 used alpha_meas = 0.50). */
volatile float   g_blower_ctrl_meas_filter_alpha    = 0.45f;

/* LPF on D-on-measurement (WM6850 d_alpha ~ 0.20). */
volatile float   g_blower_ctrl_deriv_filter_alpha = 0.25f;

/* Freeze I while internal setpoint slew is catching up at therapy start. */
volatile float   g_blower_ctrl_startup_slew_gap_cmh2o = 0.25f;

/* === Public configuration (debugger-visible) === */
volatile uint8_t g_blower_ctrl_enabled        = 0U;       /* default OFF */
volatile uint8_t g_blower_ctrl_pid_enable     = 1U;       /* P/I/D trim on */
volatile uint8_t g_blower_manual_active       = 0U;
volatile float   g_blower_ctrl_setpoint_user  = 6.0f;     /* cmH2O */

/* === State (debugger-visible) === */
volatile float   g_blower_ctrl_setpoint_slew  = 0.0f;
volatile float   g_blower_ctrl_meas_cmh2o     = 0.0f;
volatile float   g_blower_ctrl_sensor_cmh2o  = 0.0f;   /* AMS5935 at blower */
volatile float   g_blower_ctrl_meas_filt_cmh2o = 0.0f;
volatile float   g_blower_ctrl_error_cmh2o    = 0.0f;
volatile float   g_blower_ctrl_integ          = 0.0f;
volatile float   g_blower_ctrl_deriv_rpm_per_s = 0.0f;
volatile float   g_blower_ctrl_p_term         = 0.0f;
volatile float   g_blower_ctrl_i_term         = 0.0f;
volatile float   g_blower_ctrl_d_term         = 0.0f;
volatile float   g_blower_ctrl_ff_term        = 0.0f;
volatile float   g_blower_ctrl_ff_flow_term   = 0.0f;
volatile float   g_blower_ctrl_ff_dip_term    = 0.0f;
volatile float   g_blower_ctrl_out_unfilt     = 0.0f;
volatile float   g_blower_ctrl_out_filt       = 0.0f;
volatile int32_t g_blower_ctrl_out_rpm        = 0;
volatile uint8_t g_blower_ctrl_zone           = (uint8_t)BLOWER_CTRL_ZONE_LOW;
volatile uint8_t g_blower_ctrl_saturated      = 0U;
volatile uint32_t g_blower_ctrl_step_count    = 0U;

/* Previous-tick measurement for derivative-on-measurement. */
static float s_prev_meas_cmh2o = 0.0f;
static bool  s_prev_meas_valid = false;
static float s_d_meas_filt_per_s = 0.0f;
static float s_ff_flow_slew_rpm = 0.0f;
static float s_ff_dip_slew_rpm  = 0.0f;
static float s_prev_q_pat_slm   = 0.0f;
static bool  s_prev_q_pat_valid = false;
/* After mask reconnect, Vivo Q_patient is briefly huge (MeanRatio reseeds to
 * standard while total Q is still filling) — suppress patient-flow FF so
 * pressure does not overshoot until Q_pat returns near zero. */
static uint32_t s_patient_ff_suppress_ms = 0U;
static float    s_resume_rpm_floor = 0.0f;
static uint32_t s_resume_rpm_floor_ms = 0U;
static float s_p_drop_filt_per_s = 0.0f;
static bool  s_p_drop_filt_valid = false;
static float s_prev_p_sensor_cmh2o = 0.0f;
static bool  s_prev_p_sensor_valid = false;
static float s_meas_filt_cmh2o  = 0.0f;
static bool  s_meas_filt_valid  = false;
static float s_physics_ff_q_filt_slm = 0.0f;
static bool  s_physics_ff_q_filt_valid = false;
static float s_physics_ff_slew_rpm = 0.0f;
static bool  s_physics_ff_was_active = false;
static bool  s_physics_prev_inhaling = false;
static float s_prev_physics_q_in_slm = 0.0f;
static bool  s_prev_physics_q_valid  = false;
static bool  s_epr_insp_pressure_prev = false;
/* WM6850 mode: SensorTask still calls Step @ 200 Hz; PID runs every 2nd tick. */
static uint8_t s_wm6850_skip_phase = 0U;

#define BLOWER_CTRL_WM6850_DT_S 0.01f
#define BLOWER_CTRL_TWO_PI      6.28318530717958647692f

static void resetPhysicsFfState(void)
{
    s_physics_ff_q_filt_slm    = 0.0f;
    s_physics_ff_q_filt_valid  = false;
    s_physics_ff_slew_rpm      = 0.0f;
    s_prev_physics_q_in_slm    = 0.0f;
    s_prev_physics_q_valid     = false;
    g_blower_ctrl_physics_ff_q_filt_slm = 0.0f;
    g_blower_ctrl_physics_ff_rpm_raw    = 0.0f;
}

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

static void slewTowardFloat(float *val, float target, float max_step)
{
    const float diff = target - *val;
    if (diff > max_step)
    {
        *val += max_step;
    }
    else if (diff < -max_step)
    {
        *val -= max_step;
    }
    else
    {
        *val = target;
    }
}

/* Raw vs LPF patient flow in the inhale direction (earliest / largest).
 * Raw is only mixed in once filtered flow is already past the deadband;
 * pause-region sensor hash must not drive dQ/dt onset FF. */
static float inhalePatientFlowSlm(void)
{
    const float q_filt = Flow_GetPatientSlm();
    const float q_raw  = Flow_GetPatientRawSlm();
    const float db = clampf(g_blower_ctrl_flow_ff_deadband_slm, 0.0f, 15.0f);
    if (g_blower_ctrl_flow_ff_inhale_sign >= 0)
    {
        if (q_filt > db)
        {
            return (q_raw > q_filt) ? q_raw : q_filt;
        }
        return q_filt;
    }
    if (q_filt < -db)
    {
        return (q_raw < q_filt) ? q_raw : q_filt;
    }
    return q_filt;
}

static bool patientInhaling(void)
{
    const float db = clampf(g_blower_ctrl_flow_ff_deadband_slm, 0.0f, 15.0f);
    const float q  = inhalePatientFlowSlm();
    if (g_blower_ctrl_flow_ff_inhale_sign >= 0)
    {
        return q > db;
    }
    return q < -db;
}

/* End of expiration: patient flow returning to zero before next inhale. */
static bool physicsFfExpTailNearZero(void)
{
    if (g_blower_ctrl_physics_ff_exp_tail_hold_enable == 0U)
    {
        return false;
    }
    if (patientInhaling())
    {
        return false;
    }

    const float thresh = clampf(g_blower_ctrl_physics_ff_exp_tail_flow_slm, 0.5f, 10.0f);
    const float q_pat = Flow_GetPatientSlm();
    return fabsf(q_pat) < thresh;
}

/* EPR lowers expiratory setpoint; flow/dip FF must not fight that baseline. */
static bool eprDisturbanceFfAllowed(void)
{
    if (!Epr_IsEnabled())
    {
        return true;
    }
    if (g_blower_ctrl_epr_exhale_mit_enable == 0U)
    {
        return true;
    }
    return Epr_IsInspiratoryPressureActive();
}

static float steadyStateRpmForSetpoint(float setpoint_cmh2o)
{
    float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    return g_blower_ctrl_ff_k_rpm_per_sqrtcmh * sqrtf(sp)
         + g_blower_ctrl_ff_offset_rpm;
}

static bool wm6850ModeActive(void);

static bool physicsFfActive(void)
{
    return (g_blower_ctrl_physics_ff_enable != 0U) && !wm6850ModeActive();
}

static void eprExhaleOnsetTrim(float setpoint_cmh2o)
{
    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm  = 0.0f;

    const float ss = steadyStateRpmForSetpoint(setpoint_cmh2o);
    if (physicsFfActive())
    {
        s_physics_ff_slew_rpm = ss;
        g_blower_ctrl_physics_ff_rpm_raw = ss;
    }

    if (g_blower_ctrl_integ > ss)
    {
        g_blower_ctrl_integ = ss;
    }
}

static bool anyOpenLoopFfActive(void)
{
    return (g_blower_ctrl_ff_enable != 0U)
        || (g_blower_ctrl_flow_ff_enable != 0U)
        || (g_blower_ctrl_dip_ff_enable != 0U)
        || physicsFfActive();
}

static float hoseDropMagnitudeCmh2o(float q_slm)
{
    const float qabs = fabsf(q_slm);
    return TubeComp_GetRLam() * qabs + TubeComp_GetRTurb() * qabs * qabs;
}

static float physicsFfRpmForSetpointAndFlow(float setpoint_cmh2o, float q_slm);

static float physicsFfFlowInputSlm(void)
{
    if (g_blower_ctrl_physics_ff_use_total_flow != 0U)
    {
        const float q_filt = Flow_GetSlm();
        const float q_raw  = Flow_GetSlmRaw();
        float q = (q_filt > 0.0f) ? q_filt : 0.0f;

        if (q_raw > q)
        {
            q = q_raw;
        }
        return q;
    }

    return inhalePatientFlowSlm();
}

static float physicsFfVentFloorQSlm(void)
{
    float q = Leak_GetVentSlm();
    const float reported = Leak_GetReportedLeakSlm();

    if (reported > q)
    {
        q = reported;
    }
    if (q < 10.0f)
    {
        q = 10.0f;
    }
    return q;
}

static float physicsFfVentFloorRpm(float setpoint_cmh2o)
{
    const float q_vent = physicsFfVentFloorQSlm();
    const float rpm = physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_vent);

    g_blower_ctrl_physics_ff_vent_floor_rpm = rpm;
    return rpm;
}

static float physicsFfClampRpmTarget(float setpoint_cmh2o, float rpm_target)
{
    const float ss = steadyStateRpmForSetpoint(setpoint_cmh2o);
    const float vent_floor = physicsFfVentFloorRpm(setpoint_cmh2o);
    float rpm = rpm_target;

    if (rpm < vent_floor)
    {
        rpm = vent_floor;
    }
    if (rpm < ss)
    {
        rpm = ss;
    }
    return rpm;
}

static float physicsFfRpmForSetpointAndFlow(float setpoint_cmh2o, float q_slm)
{
    float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    const float hose_drop = hoseDropMagnitudeCmh2o(q_slm);
    const float c_fan = clampf(g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2,
                               0.0f, 0.001f);
    const float q_pos = (q_slm > 0.0f) ? q_slm : 0.0f;
    const float fan_load = c_fan * q_pos * q_pos;
    const float p_equiv  = sp + hose_drop + fan_load;

    g_blower_ctrl_physics_ff_hose_drop_cmh2o = hose_drop;
    g_blower_ctrl_physics_ff_fan_load_cmh2o  = fan_load;
    g_blower_ctrl_physics_ff_p_equiv_cmh2o   = p_equiv;

    return g_blower_ctrl_ff_k_rpm_per_sqrtcmh * sqrtf(p_equiv)
         + g_blower_ctrl_ff_offset_rpm;
}

static float physicsFfFilterFlow(float q_slm, float dt_s, bool inhaling, bool exhaling)
{
    float fc;
    if (exhaling)
    {
        fc = clampf(g_blower_ctrl_physics_ff_flow_lpf_exhale_hz, 0.5f, 25.0f);
    }
    else if (inhaling)
    {
        fc = clampf(g_blower_ctrl_physics_ff_flow_lpf_inhale_hz, 0.5f, 25.0f);
    }
    else
    {
        fc = clampf(g_blower_ctrl_physics_ff_flow_lpf_hz, 0.5f, 15.0f);
    }
    const float omega_dt = BLOWER_CTRL_TWO_PI * fc * dt_s;
    const float alpha = omega_dt / (1.0f + omega_dt);

    if (!s_physics_ff_q_filt_valid)
    {
        s_physics_ff_q_filt_slm   = q_slm;
        s_physics_ff_q_filt_valid = true;
    }
    else
    {
        s_physics_ff_q_filt_slm = alpha * q_slm
                              + (1.0f - alpha) * s_physics_ff_q_filt_slm;
    }

    g_blower_ctrl_physics_ff_q_filt_slm = s_physics_ff_q_filt_slm;
    return s_physics_ff_q_filt_slm;
}

static void physicsFfOnInhaleOnset(void)
{
    const float q_now = physicsFfFlowInputSlm();

    s_physics_ff_q_filt_slm = q_now;
    g_blower_ctrl_physics_ff_q_filt_slm = q_now;
    s_physics_ff_q_filt_valid = true;
    s_prev_physics_q_in_slm = q_now;
    s_prev_physics_q_valid  = true;
}

static float physicsFfOnsetBoostRpm(float q_slm, float dt_s)
{
    if (g_blower_ctrl_physics_ff_onset_enable == 0U)
    {
        return 0.0f;
    }

    float dq_dt = 0.0f;
    if (s_prev_physics_q_valid && (dt_s > 0.0f))
    {
        dq_dt = (q_slm - s_prev_physics_q_in_slm) / dt_s;
    }
    s_prev_physics_q_in_slm = q_slm;
    s_prev_physics_q_valid  = true;

    const float thresh = clampf(g_blower_ctrl_physics_ff_onset_thresh_slm_per_s,
                                10.0f, 500.0f);
    if (dq_dt <= thresh)
    {
        return 0.0f;
    }

    float boost = g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s * (dq_dt - thresh);
    const float boost_max = clampf(g_blower_ctrl_physics_ff_onset_max_rpm, 0.0f, 5000.0f);
    if (boost > boost_max)
    {
        boost = boost_max;
    }
    return boost;
}

static void physicsFfOnExhaleOnset(float setpoint_cmh2o)
{
    const float q_now = physicsFfFlowInputSlm();
    const float vent_floor = physicsFfVentFloorRpm(setpoint_cmh2o);

    /* Hybrid: freeze RPM / I-term — do not snap FF or bleed integrator.
     * Coasting here is the main source of expiratory undershoot. */
    if (g_blower_ctrl_cpap_hybrid_active != 0U)
    {
        const float q_vent = physicsFfVentFloorQSlm();
        s_physics_ff_q_filt_slm = q_vent;
        g_blower_ctrl_physics_ff_q_filt_slm = q_vent;
        s_physics_ff_q_filt_valid = true;
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            s_physics_ff_slew_rpm = vent_floor;
        }
        g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
        return;
    }

    float rpm_tgt = physicsFfClampRpmTarget(
        setpoint_cmh2o,
        physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_now));

    /* Drop the flow LPF to measured total flow so expiratory vent leak is
     * not carried over from the inspiratory peak. */
    s_physics_ff_q_filt_slm = q_now;
    g_blower_ctrl_physics_ff_q_filt_slm = q_now;
    s_physics_ff_q_filt_valid = true;

    if (g_blower_ctrl_integ > vent_floor)
    {
        g_blower_ctrl_integ = vent_floor * 0.25f;
    }
    else if (g_blower_ctrl_integ < 0.0f)
    {
        g_blower_ctrl_integ = 0.0f;
    }

    if (s_physics_ff_slew_rpm > rpm_tgt)
    {
        float new_slew = 0.5f * s_physics_ff_slew_rpm + 0.5f * rpm_tgt;
        if (new_slew < vent_floor)
        {
            new_slew = vent_floor;
        }
        s_physics_ff_slew_rpm = new_slew;
        g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
    }
    else if (s_physics_ff_slew_rpm < vent_floor)
    {
        s_physics_ff_slew_rpm = vent_floor;
        g_blower_ctrl_physics_ff_rpm_raw = vent_floor;
    }
}

static float physicsFfStep(float setpoint_cmh2o, float q_slm, float dt_s,
                           bool inhaling, bool exhaling, float meas_cmh2o)
{
    const float vent_floor = physicsFfVentFloorRpm(setpoint_cmh2o);
    const bool hybrid = (g_blower_ctrl_cpap_hybrid_active != 0U);
    /* Hybrid: hold plateau-style for the whole expiration, not only |Q|→0. */
    const bool exp_tail = hybrid ? false : physicsFfExpTailNearZero();
    float q_model = physicsFfFilterFlow(q_slm, dt_s, inhaling, exhaling);
    float rpm_target;

    float rise_step = clampf(g_blower_ctrl_physics_ff_rise_rpm_per_s,
                             100.0f, 20000.0f) * dt_s;
    if (inhaling && (meas_cmh2o < (setpoint_cmh2o - 0.04f)))
    {
        rise_step *= clampf(g_blower_ctrl_physics_ff_inhale_rise_boost, 1.0f, 4.0f);
    }

    /* --- Hybrid expiration: freeze RPM; only bleed if over-pressure ------ */
    if (hybrid && exhaling)
    {
        const float hold_band =
            clampf(g_blower_ctrl_hybrid_exp_hold_band_cmh2o, 0.02f, 1.0f);
        const float err = setpoint_cmh2o - meas_cmh2o; /* >0 → under-pressure */

        q_model = physicsFfVentFloorQSlm();
        rpm_target = physicsFfClampRpmTarget(
            setpoint_cmh2o,
            physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_model));
        if (rpm_target < vent_floor)
        {
            rpm_target = vent_floor;
        }

        /* Floor: never sit below vent-leak RPM for the therapy setpoint. */
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            s_physics_ff_slew_rpm = vent_floor;
        }

        if (err > 0.02f)
        {
            /* Below setpoint: raise FF (do not fall). Extra lift vs. sag. */
            float boost = err * clampf(g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o,
                                       0.0f, 8000.0f);
            float lift_tgt = rpm_target + boost;
            if (lift_tgt < s_physics_ff_slew_rpm)
            {
                lift_tgt = s_physics_ff_slew_rpm;
            }
            const float boost_step =
                clampf(g_blower_ctrl_hybrid_exp_boost_rpm_per_s,
                       500.0f, 20000.0f) * dt_s;
            slewTowardFloat(&s_physics_ff_slew_rpm, lift_tgt, boost_step);
            g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
            return s_physics_ff_slew_rpm;
        }

        if (err < -hold_band)
        {
            /* Clearly above setpoint: very slow bleed toward vent floor. */
            const float fall_step =
                clampf(g_blower_ctrl_hybrid_exp_fall_rpm_per_s,
                       50.0f, 4000.0f) * dt_s;
            if (s_physics_ff_slew_rpm > rpm_target)
            {
                slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, fall_step);
            }
            if (s_physics_ff_slew_rpm < vent_floor)
            {
                s_physics_ff_slew_rpm = vent_floor;
            }
            g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
            return s_physics_ff_slew_rpm;
        }

        /* Inside hold band: freeze RPM (true plateau). */
        g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
        return s_physics_ff_slew_rpm;
    }

    if (exp_tail)
    {
        /* Patient flow → 0: model vent leak, not zero-flow hose drop. */
        q_model = physicsFfVentFloorQSlm();
        rpm_target = physicsFfClampRpmTarget(
            setpoint_cmh2o,
            physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_model));
    }
    else
    {
        rpm_target = physicsFfClampRpmTarget(
            setpoint_cmh2o,
            physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_model));
    }

    if (inhaling)
    {
        rpm_target += physicsFfOnsetBoostRpm(q_slm, dt_s);
    }

    g_blower_ctrl_physics_ff_rpm_raw = rpm_target;

    float fall_rate = clampf(g_blower_ctrl_physics_ff_fall_rpm_per_s,
                           100.0f, 20000.0f);
    if (exhaling && !exp_tail && (meas_cmh2o > (setpoint_cmh2o + 0.04f)))
    {
        fall_rate = clampf(g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s,
                           fall_rate, 25000.0f);
    }
    const float fall_step = fall_rate * dt_s;

    /* Exp tail (|Q_pat| small): hold vent RPM — no coast-down through Q→0. */
    if (exp_tail)
    {
        if (rpm_target < vent_floor)
        {
            rpm_target = vent_floor;
        }
        if (s_physics_ff_slew_rpm < rpm_target)
        {
            slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, rise_step * 2.0f);
        }
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            s_physics_ff_slew_rpm = vent_floor;
        }
        return s_physics_ff_slew_rpm;
    }

    /* Pressure below setpoint during expiration: recover RPM, don't coast down. */
    if (exhaling && (meas_cmh2o < (setpoint_cmh2o - 0.04f)))
    {
        if (rpm_target < vent_floor)
        {
            rpm_target = vent_floor;
        }
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            slewTowardFloat(&s_physics_ff_slew_rpm, vent_floor, rise_step * 2.0f);
            return s_physics_ff_slew_rpm;
        }
    }

    if (rpm_target > s_physics_ff_slew_rpm)
    {
        slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, rise_step);
    }
    else
    {
        float tgt_fall = rpm_target;
        if (exhaling && (tgt_fall < vent_floor))
        {
            tgt_fall = vent_floor;
        }
        slewTowardFloat(&s_physics_ff_slew_rpm, tgt_fall, fall_step);
    }

    if (s_physics_ff_slew_rpm < vent_floor)
    {
        s_physics_ff_slew_rpm = vent_floor;
    }

    return s_physics_ff_slew_rpm;
}

void BlowerCtrl_EnforceFfMutualExclusion(void)
{
    if (g_blower_ctrl_physics_ff_enable != 0U)
    {
        g_blower_ctrl_ff_enable            = 0U;
        g_blower_ctrl_flow_ff_enable       = 0U;
        g_blower_ctrl_flow_ff_onset_enable = 0U;
        return;
    }

    if ((g_blower_ctrl_ff_enable != 0U)
        || (g_blower_ctrl_flow_ff_enable != 0U))
    {
        g_blower_ctrl_physics_ff_enable = 0U;
    }
}

/* --- CPAP Plateau + Hybrid profiles -------------------------------------- */

volatile uint8_t g_blower_ctrl_cpap_plateau_prefer = 0U;
volatile uint8_t g_blower_ctrl_cpap_plateau_active = 0U;
/* g_blower_ctrl_cpap_hybrid_prefer/active declared with Physics FF globals. */

typedef struct
{
    uint8_t ff_enable;
    uint8_t flow_ff_enable;
    uint8_t flow_ff_onset_enable;
    uint8_t dip_ff_enable;
    uint8_t physics_ff_enable;
    uint8_t physics_ff_exp_tail_hold_enable;
    float   under_p_gain;
    float   over_p_gain;
    float   out_filter_alpha;
    float   meas_filter_alpha;
    float   deriv_filter_alpha;
    float   integ_deadband_cmh2o;
} CpapProfileSnapshot_t;

static CpapProfileSnapshot_t s_plateau_snap;
static uint8_t               s_plateau_snap_valid = 0U;
static CpapProfileSnapshot_t s_hybrid_snap;
static uint8_t               s_hybrid_snap_valid = 0U;

static void profileCaptureSnapshot(CpapProfileSnapshot_t *snap, uint8_t *valid)
{
    snap->ff_enable                     = g_blower_ctrl_ff_enable;
    snap->flow_ff_enable                = g_blower_ctrl_flow_ff_enable;
    snap->flow_ff_onset_enable          = g_blower_ctrl_flow_ff_onset_enable;
    snap->dip_ff_enable                 = g_blower_ctrl_dip_ff_enable;
    snap->physics_ff_enable             = g_blower_ctrl_physics_ff_enable;
    snap->physics_ff_exp_tail_hold_enable =
        g_blower_ctrl_physics_ff_exp_tail_hold_enable;
    snap->under_p_gain                  = g_blower_ctrl_under_p_gain;
    snap->over_p_gain                   = g_blower_ctrl_over_p_gain;
    snap->out_filter_alpha              = g_blower_ctrl_out_filter_alpha;
    snap->meas_filter_alpha             = g_blower_ctrl_meas_filter_alpha;
    snap->deriv_filter_alpha            = g_blower_ctrl_deriv_filter_alpha;
    snap->integ_deadband_cmh2o          = g_blower_ctrl_integ_deadband_cmh2o;
    *valid = 1U;
}

static void profileRestoreSnapshot(const CpapProfileSnapshot_t *snap, uint8_t valid)
{
    if (valid == 0U)
    {
        return;
    }
    g_blower_ctrl_ff_enable              = snap->ff_enable;
    g_blower_ctrl_flow_ff_enable         = snap->flow_ff_enable;
    g_blower_ctrl_flow_ff_onset_enable   = snap->flow_ff_onset_enable;
    g_blower_ctrl_dip_ff_enable          = snap->dip_ff_enable;
    g_blower_ctrl_physics_ff_enable      = snap->physics_ff_enable;
    g_blower_ctrl_physics_ff_exp_tail_hold_enable =
        snap->physics_ff_exp_tail_hold_enable;
    g_blower_ctrl_under_p_gain           = snap->under_p_gain;
    g_blower_ctrl_over_p_gain            = snap->over_p_gain;
    g_blower_ctrl_out_filter_alpha       = snap->out_filter_alpha;
    g_blower_ctrl_meas_filter_alpha      = snap->meas_filter_alpha;
    g_blower_ctrl_deriv_filter_alpha     = snap->deriv_filter_alpha;
    g_blower_ctrl_integ_deadband_cmh2o   = snap->integ_deadband_cmh2o;
    BlowerCtrl_EnforceFfMutualExclusion();
}

static void plateauApplyPreset(void)
{
    /* Hold mask pressure: sqrt(P) baseline + soft PI; do not chase flow. */
    g_blower_ctrl_ff_enable              = 1U;
    g_blower_ctrl_flow_ff_enable         = 0U;
    g_blower_ctrl_flow_ff_onset_enable   = 0U;
    g_blower_ctrl_dip_ff_enable          = 0U;
    g_blower_ctrl_physics_ff_enable      = 0U;
    g_blower_ctrl_under_p_gain           = 1.0f;
    g_blower_ctrl_over_p_gain            = 1.0f;
    g_blower_ctrl_out_filter_alpha       = 0.40f;
    g_blower_ctrl_meas_filter_alpha      = 0.70f;
    g_blower_ctrl_deriv_filter_alpha     = 0.20f;
    g_blower_ctrl_integ_deadband_cmh2o   = 1.5f;
    BlowerCtrl_EnforceFfMutualExclusion();
    g_blower_ctrl_cpap_plateau_active = 1U;
    g_blower_ctrl_cpap_hybrid_active  = 0U;
}

static void plateauRestoreSnapshot(void)
{
    profileRestoreSnapshot(&s_plateau_snap, s_plateau_snap_valid);
    g_blower_ctrl_cpap_plateau_active = 0U;
}

static void hybridApplyPreset(void)
{
    /* Insp: Physics FF. Exp: freeze RPM + boost if under-pressure. */
    g_blower_ctrl_ff_enable              = 0U;
    g_blower_ctrl_flow_ff_enable         = 0U;
    g_blower_ctrl_flow_ff_onset_enable   = 0U;
    g_blower_ctrl_dip_ff_enable          = 0U;
    g_blower_ctrl_physics_ff_enable      = 1U;
    g_blower_ctrl_physics_ff_exp_tail_hold_enable = 1U;
    g_blower_ctrl_under_p_gain           = 1.6f;
    g_blower_ctrl_over_p_gain            = 1.0f;
    g_blower_ctrl_out_filter_alpha       = 0.50f;
    g_blower_ctrl_meas_filter_alpha      = 0.55f;
    g_blower_ctrl_deriv_filter_alpha     = 0.20f;
    g_blower_ctrl_integ_deadband_cmh2o   = 1.5f;
    g_blower_ctrl_hybrid_exp_hold_band_cmh2o = 0.12f;
    g_blower_ctrl_hybrid_exp_fall_rpm_per_s  = 1500.0f;
    g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o = 2500.0f;
    g_blower_ctrl_hybrid_exp_boost_rpm_per_s = 8000.0f;
    BlowerCtrl_EnforceFfMutualExclusion();
    g_blower_ctrl_cpap_hybrid_active  = 1U;
    g_blower_ctrl_cpap_plateau_active = 0U;
}

static void hybridRestoreSnapshot(void)
{
    profileRestoreSnapshot(&s_hybrid_snap, s_hybrid_snap_valid);
    g_blower_ctrl_cpap_hybrid_active = 0U;
}

static void clearPlateauPreferInternal(void)
{
    g_blower_ctrl_cpap_plateau_prefer = 0U;
    if (g_blower_ctrl_cpap_plateau_active != 0U)
    {
        plateauRestoreSnapshot();
    }
}

static void clearHybridPreferInternal(void)
{
    g_blower_ctrl_cpap_hybrid_prefer = 0U;
    if (g_blower_ctrl_cpap_hybrid_active != 0U)
    {
        hybridRestoreSnapshot();
    }
}

void BlowerCtrl_SetCpapPlateauPrefer(bool prefer)
{
    if (prefer)
    {
        clearHybridPreferInternal();
    }
    g_blower_ctrl_cpap_plateau_prefer = prefer ? 1U : 0U;
    if (!prefer && (g_blower_ctrl_cpap_plateau_active != 0U))
    {
        plateauRestoreSnapshot();
    }
}

void BlowerCtrl_SetCpapHybridPrefer(bool prefer)
{
    if (prefer)
    {
        clearPlateauPreferInternal();
    }
    g_blower_ctrl_cpap_hybrid_prefer = prefer ? 1U : 0U;
    if (!prefer && (g_blower_ctrl_cpap_hybrid_active != 0U))
    {
        hybridRestoreSnapshot();
    }
}

static void syncPlateauForMode(uint8_t therapy_mode)
{
    const bool cpap = (therapy_mode == (uint8_t)THERAPY_MODE_CPAP);

    if (cpap && (g_blower_ctrl_cpap_plateau_prefer != 0U)
        && (g_blower_ctrl_cpap_hybrid_prefer == 0U))
    {
        if (g_blower_ctrl_cpap_plateau_active == 0U)
        {
            profileCaptureSnapshot(&s_plateau_snap, &s_plateau_snap_valid);
            plateauApplyPreset();
        }
        else
        {
            plateauApplyPreset();
        }
        return;
    }

    if ((!cpap || (g_blower_ctrl_cpap_plateau_prefer == 0U))
        && (g_blower_ctrl_cpap_plateau_active != 0U))
    {
        plateauRestoreSnapshot();
    }
}

static void syncHybridForMode(uint8_t therapy_mode)
{
    const bool cpap = (therapy_mode == (uint8_t)THERAPY_MODE_CPAP);

    if (cpap && (g_blower_ctrl_cpap_hybrid_prefer != 0U))
    {
        if (g_blower_ctrl_cpap_hybrid_active == 0U)
        {
            profileCaptureSnapshot(&s_hybrid_snap, &s_hybrid_snap_valid);
            hybridApplyPreset();
        }
        else
        {
            hybridApplyPreset();
        }
        return;
    }

    if ((!cpap || (g_blower_ctrl_cpap_hybrid_prefer == 0U))
        && (g_blower_ctrl_cpap_hybrid_active != 0U))
    {
        hybridRestoreSnapshot();
    }
}

void BlowerCtrl_SyncCpapProfilesForMode(uint8_t therapy_mode)
{
    /* Plateau first (no-op if hybrid preferred), then hybrid. */
    syncPlateauForMode(therapy_mode);
    syncHybridForMode(therapy_mode);
}

void BlowerCtrl_SyncCpapPlateauForMode(uint8_t therapy_mode)
{
    BlowerCtrl_SyncCpapProfilesForMode(therapy_mode);
}

bool BlowerCtrl_IsCpapPlateauPreferred(void)
{
    return (g_blower_ctrl_cpap_plateau_prefer != 0U);
}

bool BlowerCtrl_IsCpapPlateauActive(void)
{
    return (g_blower_ctrl_cpap_plateau_active != 0U);
}

bool BlowerCtrl_IsCpapHybridPreferred(void)
{
    return (g_blower_ctrl_cpap_hybrid_prefer != 0U);
}

bool BlowerCtrl_IsCpapHybridActive(void)
{
    return (g_blower_ctrl_cpap_hybrid_active != 0U);
}

/* With FF on, |I| is a small trim (default cap 4 krpm). With FF off, I
 * must carry the full steady-state RPM (~17 krpm @ 8 cmH2O). */
static float effectiveIntegMaxAbsRpm(float user_sp_cmh2o)
{
    if (g_blower_ctrl_wm6850_mode != 0U)
    {
        return (float)g_blower_ctrl_rpm_max;
    }
    if (anyOpenLoopFfActive())
    {
        return g_blower_ctrl_integ_max_abs_rpm;
    }
    const float ss = steadyStateRpmForSetpoint(user_sp_cmh2o);
    const float ff_off_cap = ss * 1.25f;
    return (ff_off_cap > g_blower_ctrl_integ_max_abs_rpm)
             ? ff_off_cap : g_blower_ctrl_integ_max_abs_rpm;
}

static float wm6850SpeedCapRpm(float setpoint_cmh2o)
{
    /* WM6850 dev firmware used interp_occ_speed(P)*1.3 (6 krpm class blower).
     * Same motor on the custom CPAP board uses the sqrt bench map instead. */
    float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    return steadyStateRpmForSetpoint(sp) * g_blower_ctrl_wm6850_speed_lim_scale;
}

static bool wm6850ModeActive(void)
{
    return (g_blower_ctrl_wm6850_mode != 0U);
}

static bool rpmFloorActive(void)
{
    return (g_blower_ctrl_rpm_floor_enable != 0U)
        && anyOpenLoopFfActive()
        && !wm6850ModeActive()
        && !Epr_IsEnabled();
}

static float rpmFloorForSetpointCmh2o(float setpoint_cmh2o)
{
    const float ss = steadyStateRpmForSetpoint(setpoint_cmh2o);

    if (physicsFfActive())
    {
        const float vent = physicsFfVentFloorRpm(setpoint_cmh2o);
        return (vent > ss) ? vent : ss;
    }
    return ss;
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
    g_blower_ctrl_sensor_cmh2o  = 0.0f;
    g_blower_ctrl_error_cmh2o    = 0.0f;
    g_blower_ctrl_integ          = 0.0f;
    g_blower_ctrl_deriv_rpm_per_s = 0.0f;
    g_blower_ctrl_p_term         = 0.0f;
    g_blower_ctrl_i_term         = 0.0f;
    g_blower_ctrl_d_term         = 0.0f;
    g_blower_ctrl_ff_term        = 0.0f;
    g_blower_ctrl_ff_flow_term   = 0.0f;
    g_blower_ctrl_ff_dip_term    = 0.0f;
    g_blower_ctrl_out_unfilt     = 0.0f;
    g_blower_ctrl_out_filt       = 0.0f;
    g_blower_ctrl_out_rpm        = 0;
    g_blower_ctrl_zone           = (uint8_t)BLOWER_CTRL_ZONE_LOW;
    g_blower_ctrl_saturated      = 0U;
    g_blower_ctrl_step_count     = 0U;
    g_blower_ctrl_meas_filt_cmh2o = 0.0f;

    s_prev_meas_cmh2o = 0.0f;
    s_prev_meas_valid = false;
    s_d_meas_filt_per_s = 0.0f;
    s_ff_flow_slew_rpm  = 0.0f;
    s_ff_dip_slew_rpm   = 0.0f;
    s_prev_q_pat_slm    = 0.0f;
    s_prev_q_pat_valid  = false;
    s_p_drop_filt_per_s = 0.0f;
    s_p_drop_filt_valid = false;
    s_prev_p_sensor_cmh2o = 0.0f;
    s_prev_p_sensor_valid = false;
    s_meas_filt_cmh2o  = 0.0f;
    s_meas_filt_valid  = false;
    s_wm6850_skip_phase = 0U;
    s_physics_ff_was_active = false;
    s_physics_prev_inhaling = false;
    s_epr_insp_pressure_prev = false;
    resetPhysicsFfState();
    BlowerCtrl_EnforceFfMutualExclusion();

    /* Co-init the FOT oscillator. Defaults to disabled so this is
     * invisible until the user explicitly turns it on. */
    BlowerOsc_Init();
}

void BlowerCtrl_SetEnable(bool en)
{
    if (en && !g_blower_ctrl_enabled)
    {
        const float user_sp = g_blower_ctrl_setpoint_user;
        const float meas    = g_blower_ctrl_meas_cmh2o;

        if (wm6850ModeActive())
        {
            /* PressurePID_Reset(): zero state, no sqrt seed. */
            g_blower_ctrl_setpoint_slew   = user_sp;
            g_blower_ctrl_integ           = 0.0f;
            g_blower_ctrl_out_filt        = 0.0f;
            g_blower_ctrl_out_rpm         = 0;
            s_ff_flow_slew_rpm            = 0.0f;
            s_ff_dip_slew_rpm             = 0.0f;
            s_prev_q_pat_valid            = false;
            s_d_meas_filt_per_s           = 0.0f;
            s_meas_filt_cmh2o             = meas;
            s_meas_filt_valid               = true;
            g_blower_ctrl_meas_filt_cmh2o   = meas;
            s_prev_meas_cmh2o               = meas;
            s_prev_meas_valid               = true;
            s_wm6850_skip_phase             = 0U;
        }
        else
        {
            /* Default to the requested setpoint (e.g. EPR-adjusted therapy
             * target). Only nudge slew to measured pressure when already
             * within bumpless band — never leave a stale slew from a prior
             * session when FF is enabled. */
            const float bumpless_band = 0.75f;
            g_blower_ctrl_setpoint_slew = user_sp;
            if (fabsf(meas - user_sp) <= bumpless_band)
            {
                g_blower_ctrl_setpoint_slew = meas;
            }
            g_blower_ctrl_integ         = 0.0f;
            s_ff_flow_slew_rpm          = 0.0f;
            s_ff_dip_slew_rpm           = 0.0f;
            s_prev_q_pat_valid          = false;
            s_d_meas_filt_per_s         = 0.0f;
            s_meas_filt_cmh2o           = meas;
            s_meas_filt_valid           = (meas > 0.01f);
            g_blower_ctrl_meas_filt_cmh2o = meas;
            s_prev_meas_valid           = false;

            const float ss_rpm = steadyStateRpmForSetpoint(g_blower_ctrl_setpoint_slew);
            if (physicsFfActive())
            {
                /* I is trim only; seed Physics FF to steady-state RPM. */
                s_physics_ff_slew_rpm = ss_rpm;
                g_blower_ctrl_physics_ff_rpm_raw = ss_rpm;
                s_physics_ff_was_active = true;
                g_blower_ctrl_integ = 0.0f;
                g_blower_ctrl_out_filt = ss_rpm;
            }
            else if (g_blower_ctrl_ff_enable != 0U)
            {
                g_blower_ctrl_out_filt = ss_rpm;
            }
            else
            {
                g_blower_ctrl_integ    = ss_rpm;
                g_blower_ctrl_out_filt = ss_rpm;
            }
            g_blower_ctrl_out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
        }
    }
    g_blower_ctrl_enabled = en ? 1U : 0U;
}

bool BlowerCtrl_IsEnabled(void)
{
    return (g_blower_ctrl_enabled != 0U);
}

void BlowerCtrl_ManualStart(int32_t rpm, uint16_t ramp_ms)
{
    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Start();
    (void)BlowerIpc_CM7_SetSpeedRpm(rpm, ramp_ms);
    g_blower_manual_active = 1U;
}

void BlowerCtrl_ManualSetSpeed(int32_t rpm, uint16_t ramp_ms)
{
    if (g_blower_manual_active == 0U)
    {
        (void)BlowerIpc_CM7_Start();
        g_blower_manual_active = 1U;
    }
    (void)BlowerIpc_CM7_SetSpeedRpm(rpm, ramp_ms);
}

void BlowerCtrl_ManualStop(void)
{
    (void)BlowerIpc_CM7_Stop();
    g_blower_manual_active = 0U;
}

void BlowerCtrl_ClearManualFlag(void)
{
    /* Drop manual mode without STOP — used on mask-on resume so we do not
     * race a STOP mailbox write against the therapy SetSpeed that follows. */
    g_blower_manual_active = 0U;
}

bool BlowerCtrl_IsManualActive(void)
{
    return (g_blower_manual_active != 0U);
}

void BlowerCtrl_SnapSetpoint(float cmh2o)
{
    const float sp = clampf(cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    g_blower_ctrl_setpoint_user = sp;
    g_blower_ctrl_setpoint_slew = sp;
}

void BlowerCtrl_SetUserSetpointOnly(float cmh2o)
{
    g_blower_ctrl_setpoint_user = clampf(cmh2o,
                                         BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                         BLOWER_CTRL_SETPOINT_MAX_CMH2O);
}

void BlowerCtrl_ReseedForTherapyResume(float target_cmh2o)
{
    const float sp = clampf(target_cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    BlowerCtrl_SnapSetpoint(sp);

    const float ss_rpm = steadyStateRpmForSetpoint(sp);
    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm = 0.0f;
    s_prev_q_pat_valid = false;
    s_d_meas_filt_per_s = 0.0f;
    s_prev_meas_valid = false;
    s_physics_prev_inhaling = false;

    if (physicsFfActive())
    {
        s_physics_ff_q_filt_valid = false;
        s_physics_ff_slew_rpm = ss_rpm;
        g_blower_ctrl_physics_ff_rpm_raw = ss_rpm;
        s_physics_ff_was_active = true;
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = ss_rpm;
    }
    else if (g_blower_ctrl_ff_enable != 0U)
    {
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = ss_rpm;
    }
    else
    {
        g_blower_ctrl_integ = ss_rpm;
        g_blower_ctrl_out_filt = ss_rpm;
    }

    g_blower_ctrl_out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    g_blower_manual_active = 0U;
}

void BlowerCtrl_BeginMaskResume(float target_cmh2o, float start_rpm)
{
    const float sp = clampf(target_cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    g_blower_ctrl_setpoint_user = sp;

    /* Start slewed SP near measured pressure so sqrt-FF tracks a rising
     * trajectory instead of the full therapy target on the first tick. */
    float slew0 = g_blower_ctrl_meas_cmh2o;
    if (slew0 < 0.5f)
    {
        slew0 = 0.5f;
    }
    if (slew0 > sp)
    {
        slew0 = sp;
    }
    g_blower_ctrl_setpoint_slew = slew0;

    float rpm = start_rpm;
    if (rpm < 1000.0f)
    {
        rpm = 1000.0f;
    }
    if (rpm > (float)g_blower_ctrl_rpm_max)
    {
        rpm = (float)g_blower_ctrl_rpm_max;
    }

    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm = 0.0f;
    s_prev_q_pat_valid = false;
    s_d_meas_filt_per_s = 0.0f;
    s_prev_meas_valid = false;
    s_physics_prev_inhaling = false;

    if (physicsFfActive())
    {
        s_physics_ff_q_filt_valid = false;
        s_physics_ff_slew_rpm = rpm;
        g_blower_ctrl_physics_ff_rpm_raw = rpm;
        s_physics_ff_was_active = true;
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = rpm;
    }
    else if (g_blower_ctrl_ff_enable != 0U)
    {
        /* I trim only; FF will follow slewed SP from ~meas upward. */
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = rpm;
    }
    else
    {
        g_blower_ctrl_integ = rpm;
        g_blower_ctrl_out_filt = rpm;
    }

    g_blower_ctrl_out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    g_blower_manual_active = 0U;
}

void BlowerCtrl_SuppressPatientFlowFf(uint32_t duration_ms)
{
    s_patient_ff_suppress_ms = duration_ms;
    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm = 0.0f;
    s_prev_q_pat_valid = false;
    g_blower_ctrl_ff_flow_term = 0.0f;
    g_blower_ctrl_ff_dip_term = 0.0f;
}

void BlowerCtrl_SetResumeRpmFloor(float rpm, uint32_t duration_ms)
{
    if (rpm < 0.0f)
    {
        rpm = 0.0f;
    }
    s_resume_rpm_floor = rpm;
    s_resume_rpm_floor_ms = duration_ms;
}

float BlowerCtrl_EstimateSteadyStateRpm(float target_cmh2o)
{
    const float sp = clampf(target_cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    return steadyStateRpmForSetpoint(sp);
}

void BlowerCtrl_SetSetpoint(float cmh2o)
{
    g_blower_ctrl_setpoint_user = clampf(cmh2o,
                                         BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                         BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    /* While the PID is off, keep slew aligned so therapy start (EPR, ramp)
     * does not inherit the previous session's slewed target. */
    if ((g_blower_ctrl_enabled == 0U) || Epr_IsEnabled())
    {
        /* EPR steps the therapy target each breath; do not second-slew here
         * (6 cmH2O/s down made status/pressure linger at 6–7 cmH2O). */
        g_blower_ctrl_setpoint_slew = g_blower_ctrl_setpoint_user;
    }
}

float BlowerCtrl_GetSetpoint(void)        { return g_blower_ctrl_setpoint_user; }
float BlowerCtrl_GetSetpointSlewed(void)  { return g_blower_ctrl_setpoint_slew; }
float BlowerCtrl_GetMeasured(void)        { return g_blower_ctrl_meas_cmh2o; }
float BlowerCtrl_GetSensorCmh2o(void)     { return g_blower_ctrl_sensor_cmh2o; }
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
    float p_drop_rate_cmh2o_per_s = 0.0f;
    if (s_prev_p_sensor_valid)
    {
        const float p_drop_raw = (p_sensor_cmh2o - s_prev_p_sensor_cmh2o) / dt_s;
        if (!s_p_drop_filt_valid)
        {
            s_p_drop_filt_per_s = p_drop_raw;
            s_p_drop_filt_valid = true;
        }
        else
        {
            s_p_drop_filt_per_s = 0.35f * p_drop_raw + 0.65f * s_p_drop_filt_per_s;
        }
        p_drop_rate_cmh2o_per_s = s_p_drop_filt_per_s;
    }
    const float q_slm           = Flow_GetSlm();
    g_blower_ctrl_sensor_cmh2o  = p_sensor_cmh2o;
    const float meas_pid_cmh2o  = wm6850ModeActive()
                                    ? p_sensor_cmh2o
                                    : TubeComp_Apply(p_sensor_cmh2o, q_slm);
    g_blower_ctrl_meas_cmh2o    = meas_pid_cmh2o;

    /* WM6850: pressure_pid.c runs at 100 Hz. SensorTask stays at 200 Hz so
     * therapy/safety still see fresh g_blower_ctrl_sensor_cmh2o every tick;
     * filter/PID/output run only on alternating ticks with dt = 10 ms. */
    if (wm6850ModeActive())
    {
        s_wm6850_skip_phase ^= 1U;
        if (s_wm6850_skip_phase != 0U)
        {
            return;
        }
        dt_s = BLOWER_CTRL_WM6850_DT_S;
    }

    if (!s_meas_filt_valid)
    {
        s_meas_filt_cmh2o = meas_pid_cmh2o;
        s_meas_filt_valid = true;
    }
    else
    {
        float am = clampf(g_blower_ctrl_meas_filter_alpha, 0.0f, 1.0f);
        if (wm6850ModeActive())
        {
            am = clampf(g_blower_ctrl_wm6850_meas_alpha, 0.0f, 1.0f);
        }
        s_meas_filt_cmh2o = am * s_meas_filt_cmh2o + (1.0f - am) * meas_pid_cmh2o;
    }
    g_blower_ctrl_meas_filt_cmh2o = s_meas_filt_cmh2o;

    /* --- 2. Slew the user setpoint toward the target --- */
    {
        const float target = clampf(g_blower_ctrl_setpoint_user,
                                    BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                    BLOWER_CTRL_SETPOINT_MAX_CMH2O);
        if (wm6850ModeActive())
        {
            g_blower_ctrl_setpoint_slew = target;
        }
        else
        {
        float       sp       = g_blower_ctrl_setpoint_slew;
        const float diff     = target - sp;
        const float slew_rate = (diff > 0.0f)
                                  ? g_blower_ctrl_slew_up_cmh2o_per_s
                                  : g_blower_ctrl_slew_cmh2o_per_s;
        const float max_step = slew_rate * dt_s;

        if (diff >  max_step)      sp += max_step;
        else if (diff < -max_step) sp -= max_step;
        else                       sp  = target;

        g_blower_ctrl_setpoint_slew = sp;
        }
    }

    if (Epr_IsEnabled() && (g_blower_ctrl_epr_exhale_mit_enable != 0U))
    {
        const bool epr_insp_now = Epr_IsInspiratoryPressureActive();
        if (s_epr_insp_pressure_prev && !epr_insp_now)
        {
            eprExhaleOnsetTrim(g_blower_ctrl_setpoint_slew);
        }
        s_epr_insp_pressure_prev = epr_insp_now;
    }
    else
    {
        s_epr_insp_pressure_prev = false;
    }

    /* --- 3. If disabled, just track the measurement and bail out --- */
    if (g_blower_ctrl_enabled == 0U)
    {
        s_prev_meas_cmh2o = s_meas_filt_cmh2o;
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
    if (wm6850ModeActive())
    {
        kp = g_blower_ctrl_wm6850_kp;
        ki = g_blower_ctrl_wm6850_ki;
        kd = g_blower_ctrl_wm6850_kd;
        g_blower_ctrl_zone = (uint8_t)BLOWER_CTRL_ZONE_LOW;
    }
    else
    {
        getActiveGains(zone, &kp, &ki, &kd);
    }

    /* --- 5. PID terms --- */
    const float osc      = wm6850ModeActive() ? 0.0f : BlowerOsc_StepAndGetSample(dt_s);
    const float setpoint = g_blower_ctrl_setpoint_slew + osc;

    /* Blend toward unfiltered pressure only during inhale sags — avoids
     * measurement chatter on expiration / overshoot recovery. */
    float meas_cmh2o = s_meas_filt_cmh2o;
    if (!wm6850ModeActive() && (g_blower_ctrl_fast_meas_enable != 0U))
    {
        const float err_raw = setpoint - meas_pid_cmh2o;
        const float thresh  = clampf(g_blower_ctrl_fast_meas_thresh_cmh2o, 0.0f, 1.0f);
        const float full    = clampf(g_blower_ctrl_fast_meas_full_cmh2o,
                                     thresh + 0.05f, 2.0f);
        if (patientInhaling() && (err_raw > thresh))
        {
            const float blend = clampf((err_raw - thresh) / (full - thresh),
                                       0.0f, 1.0f);
            meas_cmh2o = (1.0f - blend) * s_meas_filt_cmh2o
                       + blend * meas_pid_cmh2o;
        }
    }

    const float error    = setpoint - meas_cmh2o;
    g_blower_ctrl_error_cmh2o = error;

    /* P. Extra gain when above setpoint to settle overshoot faster. */
    float p_term = kp * error;
    if (!wm6850ModeActive()
        && patientInhaling()
        && (error > g_blower_ctrl_under_p_thresh_cmh2o))
    {
        p_term *= g_blower_ctrl_under_p_gain;
    }
    else if (!wm6850ModeActive()
        && (error < -g_blower_ctrl_over_p_thresh_cmh2o))
    {
        p_term *= g_blower_ctrl_over_p_gain;
    }

    /* D-on-measurement (negated; positive d/dt of meas pushes output down). */
    float d_meas_per_s = 0.0f;
    if (s_prev_meas_valid)
    {
        d_meas_per_s = (meas_cmh2o - s_prev_meas_cmh2o) / dt_s;
    }
    {
        float a = clampf(g_blower_ctrl_deriv_filter_alpha, 0.05f, 1.0f);
        if (wm6850ModeActive())
        {
            a = clampf(g_blower_ctrl_wm6850_deriv_alpha, 0.05f, 1.0f);
        }
        if (wm6850ModeActive())
        {
            /* Match pressure_pid.c: d_alpha weights the previous filtered D. */
            s_d_meas_filt_per_s = a * s_d_meas_filt_per_s + (1.0f - a) * d_meas_per_s;
        }
        else
        {
            s_d_meas_filt_per_s = a * d_meas_per_s + (1.0f - a) * s_d_meas_filt_per_s;
        }
        d_meas_per_s = s_d_meas_filt_per_s;
    }
    g_blower_ctrl_deriv_rpm_per_s = d_meas_per_s;
    float d_term = -kd * d_meas_per_s;

    /* Feed-forward: legacy (sqrt + flow + dip) OR physics-based combined FF.
     * The two paths are mutually exclusive (see BlowerCtrl_EnforceFfMutualExclusion). */
    float ff_term = 0.0f;
    float ff_flow = 0.0f;
    float ff_dip  = 0.0f;
    const bool epr_ff_ok = eprDisturbanceFfAllowed();

    bool suppress_pat_ff = false;
    if (s_patient_ff_suppress_ms > 0U)
    {
        const uint32_t step_ms = (uint32_t)lrintf(dt_s * 1000.0f);
        if (step_ms >= s_patient_ff_suppress_ms)
        {
            s_patient_ff_suppress_ms = 0U;
        }
        else
        {
            s_patient_ff_suppress_ms -= step_ms;
        }
        suppress_pat_ff = true;
        s_ff_flow_slew_rpm = 0.0f;
        s_ff_dip_slew_rpm = 0.0f;
    }

    if (!wm6850ModeActive() && physicsFfActive())
    {
        const bool inhaling = suppress_pat_ff ? false : patientInhaling();
        if ((g_blower_ctrl_physics_ff_exhale_mit_enable != 0U)
            && s_physics_prev_inhaling
            && !inhaling)
        {
            physicsFfOnExhaleOnset(setpoint);
        }
        else if ((g_blower_ctrl_physics_ff_inhale_mit_enable != 0U)
                 && !s_physics_prev_inhaling
                 && inhaling)
        {
            physicsFfOnInhaleOnset();
        }
        s_physics_prev_inhaling = inhaling;

        if (epr_ff_ok && !suppress_pat_ff)
        {
            const float q_ff_in = physicsFfFlowInputSlm();
            ff_term = physicsFfStep(setpoint, q_ff_in, dt_s, inhaling, !inhaling,
                                    meas_pid_cmh2o);
        }
        else
        {
            /* During reconnect suppress: plateau at sqrt SS (no patient-Q boost). */
            ff_term = steadyStateRpmForSetpoint(setpoint);
            s_physics_ff_slew_rpm = ff_term;
            g_blower_ctrl_physics_ff_rpm_raw = ff_term;
        }
        s_physics_ff_was_active = true;
        s_ff_flow_slew_rpm = 0.0f;
        g_blower_ctrl_ff_flow_term = 0.0f;
    }
    else if (!wm6850ModeActive())
    {
        if (s_physics_ff_was_active)
        {
            resetPhysicsFfState();
            s_physics_ff_was_active = false;
            s_physics_prev_inhaling = false;
        }
    /* Square-root steady-state RPM at setpoint. */
    if (g_blower_ctrl_ff_enable != 0U)
    {
        float ff_input = (setpoint > 0.0f) ? setpoint : 0.0f;
        ff_term = steadyStateRpmForSetpoint(ff_input);
    }

    /* Flow disturbance FF: proactive RPM boost on patient inhalation. */
    float ff_flow_target = 0.0f;
    const float q_pat = inhalePatientFlowSlm();
    float q_pat_rate_slm_per_s = 0.0f;
    if (s_prev_q_pat_valid)
    {
        q_pat_rate_slm_per_s = (q_pat - s_prev_q_pat_slm) / dt_s;
    }
    s_prev_q_pat_slm   = q_pat;
    s_prev_q_pat_valid = true;

    if (suppress_pat_ff)
    {
        ff_flow_target = 0.0f;
    }
    else if (g_blower_ctrl_flow_ff_enable != 0U && epr_ff_ok)
    {
        const float db = clampf(g_blower_ctrl_flow_ff_deadband_slm, 0.0f, 15.0f);
        if (g_blower_ctrl_flow_ff_inhale_sign >= 0)
        {
            if (q_pat > db)
            {
                ff_flow_target = g_blower_ctrl_flow_ff_k_rpm_per_slm * (q_pat - db);
            }
            if (g_blower_ctrl_flow_ff_onset_enable != 0U)
            {
                const float dq_thresh = clampf(
                    g_blower_ctrl_flow_ff_onset_thresh_slm_per_s, 10.0f, 500.0f);
                if (q_pat_rate_slm_per_s > dq_thresh)
                {
                    float ff_onset = g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s
                                   * (q_pat_rate_slm_per_s - dq_thresh);
                    const float onset_max = clampf(
                        g_blower_ctrl_flow_ff_onset_max_rpm, 0.0f, 4000.0f);
                    if (ff_onset > onset_max)
                    {
                        ff_onset = onset_max;
                    }
                    ff_flow_target += ff_onset;
                }
            }
        }
        else if (q_pat < -db)
        {
            ff_flow_target = g_blower_ctrl_flow_ff_k_rpm_per_slm * (-q_pat - db);
            if (g_blower_ctrl_flow_ff_onset_enable != 0U)
            {
                const float dq_thresh = clampf(
                    g_blower_ctrl_flow_ff_onset_thresh_slm_per_s, 10.0f, 500.0f);
                if (q_pat_rate_slm_per_s < -dq_thresh)
                {
                    float ff_onset = g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s
                                   * (-q_pat_rate_slm_per_s - dq_thresh);
                    const float onset_max = clampf(
                        g_blower_ctrl_flow_ff_onset_max_rpm, 0.0f, 4000.0f);
                    if (ff_onset > onset_max)
                    {
                        ff_onset = onset_max;
                    }
                    ff_flow_target += ff_onset;
                }
            }
        }
        if (ff_flow_target > 0.0f)
        {
            const float ff_max = clampf(g_blower_ctrl_flow_ff_max_rpm, 0.0f, 8000.0f);
            if (ff_flow_target > ff_max)
            {
                ff_flow_target = ff_max;
            }
        }
    }
    else if (!epr_ff_ok)
    {
        ff_flow_target = 0.0f;
        s_ff_flow_slew_rpm = 0.0f;
    }
    {
        const float rise_step = clampf(g_blower_ctrl_flow_ff_rise_rpm_per_s,
                                       100.0f, 20000.0f) * dt_s;
        const float fall_step = clampf(g_blower_ctrl_flow_ff_fall_rpm_per_s,
                                       100.0f, 20000.0f) * dt_s;
        if (ff_flow_target > s_ff_flow_slew_rpm)
        {
            slewTowardFloat(&s_ff_flow_slew_rpm, ff_flow_target, rise_step);
        }
        else
        {
            slewTowardFloat(&s_ff_flow_slew_rpm, ff_flow_target, fall_step);
        }
    }
    ff_flow = s_ff_flow_slew_rpm;
    g_blower_ctrl_ff_flow_term = ff_flow;
    } /* legacy FF path */

    if (!wm6850ModeActive())
    {
        float ff_dip_target = 0.0f;
        if (!suppress_pat_ff
            && (g_blower_ctrl_dip_ff_enable != 0U) && epr_ff_ok)
        {
            const float press_sag = setpoint - meas_pid_cmh2o;
            const float err_min   = clampf(g_blower_ctrl_dip_ff_err_min_cmh2o, 0.0f, 1.0f);
            const bool  dip_phase = patientInhaling() || physicsFfExpTailNearZero();
            if (dip_phase && (press_sag > err_min))
            {
                const float thresh = clampf(g_blower_ctrl_dip_ff_thresh_cmh2o_per_s,
                                            1.0f, 80.0f);
                if (p_drop_rate_cmh2o_per_s < -thresh)
                {
                    ff_dip_target = g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s
                                  * (-p_drop_rate_cmh2o_per_s - thresh);
                    const float dip_max = clampf(g_blower_ctrl_dip_ff_max_rpm,
                                                 0.0f, 8000.0f);
                    if (ff_dip_target > dip_max)
                    {
                        ff_dip_target = dip_max;
                    }
                }
            }
        }
        else if (!epr_ff_ok)
        {
            ff_dip_target = 0.0f;
            s_ff_dip_slew_rpm = 0.0f;
        }
        {
            const float rise_step = clampf(g_blower_ctrl_flow_ff_rise_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            const float fall_step = clampf(g_blower_ctrl_flow_ff_fall_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            if (ff_dip_target > s_ff_dip_slew_rpm)
            {
                slewTowardFloat(&s_ff_dip_slew_rpm, ff_dip_target, rise_step);
            }
            else
            {
                slewTowardFloat(&s_ff_dip_slew_rpm, ff_dip_target, fall_step);
            }
        }
        ff_dip = s_ff_dip_slew_rpm;
        g_blower_ctrl_ff_dip_term = ff_dip;
    }

    const float ff_dist = ff_flow + ff_dip;

    const float rpm_min_f = (float)g_blower_ctrl_rpm_min;
    const float rpm_max_f = (float)g_blower_ctrl_rpm_max;
    float active_rpm_max_f = rpm_max_f;
    if (wm6850ModeActive())
    {
        const float speed_cap = wm6850SpeedCapRpm(setpoint);
        if (speed_cap < active_rpm_max_f)
        {
            active_rpm_max_f = speed_cap;
        }
        if (active_rpm_max_f < rpm_min_f)
        {
            active_rpm_max_f = rpm_min_f;
        }
    }

    /* I-term: WM6850-style bidirectional conditional integration.
     * When pressure > setpoint (err < 0), I ramps down and pulls RPM
     * lower quickly — key to fast expiratory return. Freeze I only while
     * the internal setpoint is still slewing up at therapy start. */
    const float sp_gap = g_blower_ctrl_setpoint_user - g_blower_ctrl_setpoint_slew;
    const bool  sp_catching_up = (!wm6850ModeActive())
        && (sp_gap > g_blower_ctrl_startup_slew_gap_cmh2o);

    float i_step = 0.0f;
    if (g_blower_ctrl_pid_enable != 0U)
    {
        if (!sp_catching_up)
        {
            const float trim_try = p_term + g_blower_ctrl_integ + d_term;
            const float out_try  = ff_term + ff_dist + trim_try;
            const bool  sat_hi   = (out_try > active_rpm_max_f);
            const bool  sat_lo   = (out_try < rpm_min_f);

            if ((!sat_hi && !sat_lo)
                || (sat_hi && (error < 0.0f))
                || (sat_lo && (error > 0.0f)))
            {
                i_step = ki * error * dt_s;
            }
        }
    }

    float integ_new = g_blower_ctrl_integ + i_step;

    /* Physics FF: negative I after expiratory overshoot deepens undershoot. */
    if (physicsFfActive()
        && !patientInhaling()
        && (error > 0.04f)
        && (integ_new < 0.0f))
    {
        integ_new = 0.0f;
    }

    /* Compute provisional output with the candidate integrator. */
    float out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;

    /* Overpressure brake (optional; default off). */
    if (g_blower_ctrl_pid_enable != 0U
        && !wm6850ModeActive()
        && (g_blower_ctrl_over_brake_rpm_per_s > 0.0f)
        && (error < -g_blower_ctrl_over_p_thresh_cmh2o))
    {
        const float brake = clampf(g_blower_ctrl_over_brake_rpm_per_s, 500.0f, 15000.0f) * dt_s;
        const float floor_out = g_blower_ctrl_out_filt - brake;
        if (out_unfilt > floor_out)
        {
            out_unfilt = floor_out;
        }
    }

    /* Anti-windup layer 2: reject integrator step if it deepens saturation. */
    bool        sat_high  = (out_unfilt > active_rpm_max_f) && (i_step > 0.0f);
    bool        sat_low   = (out_unfilt < rpm_min_f) && (i_step < 0.0f);
    if (sat_high || sat_low)
    {
        integ_new = g_blower_ctrl_integ;
        out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
    }
    else if (g_blower_ctrl_pid_enable != 0U && !sp_catching_up)
    {
        /* WM6850-style integrator clamp to remaining RPM headroom. */
        const float trim_min = rpm_min_f - ff_term - ff_dist - p_term - d_term;
        const float trim_max = active_rpm_max_f - ff_term - ff_dist - p_term - d_term;
        if (integ_new < trim_min) { integ_new = trim_min; }
        if (integ_new > trim_max) { integ_new = trim_max; }
        out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
    }

    /* Anti-windup layer 3: hard magnitude clamp on the integrator. */
    if (g_blower_ctrl_pid_enable != 0U)
    {
        const float integ_max = effectiveIntegMaxAbsRpm(g_blower_ctrl_setpoint_user);
        if (integ_new > integ_max)
        {
            integ_new = integ_max;
            out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
        }
        else if (integ_new < -integ_max)
        {
            integ_new = -integ_max;
            out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
        }
    }

    if (g_blower_ctrl_pid_enable == 0U)
    {
        out_unfilt = ff_term + ff_dist;
        integ_new  = g_blower_ctrl_integ;
        p_term     = 0.0f;
        d_term     = 0.0f;
    }

    const float rpm_floor = rpmFloorActive()
                              ? rpmFloorForSetpointCmh2o(setpoint)
                              : 0.0f;
    if (rpm_floor > 0.0f && out_unfilt < rpm_floor)
    {
        out_unfilt = rpm_floor;
    }

    /* Commit and clamp. */
    g_blower_ctrl_integ      = integ_new;
    g_blower_ctrl_p_term     = p_term;
    g_blower_ctrl_i_term     = integ_new;
    g_blower_ctrl_d_term     = d_term;
    g_blower_ctrl_ff_term    = ff_term;
    g_blower_ctrl_out_unfilt = out_unfilt;

    const float out_clamped = clampf(out_unfilt, rpm_min_f, active_rpm_max_f);
    g_blower_ctrl_saturated  = (out_clamped != out_unfilt) ? 1U : 0U;

    float out_post_sat = out_clamped;

    /* --- 6. Output low-pass filter --- */
    float       a = g_blower_ctrl_out_filter_alpha;
    if (a < 0.05f) a = 0.05f;
    if (a > 1.0f)  a = 1.0f;
    g_blower_ctrl_out_filt = a * out_post_sat
                           + (1.0f - a) * g_blower_ctrl_out_filt;
    if (rpm_floor > 0.0f && g_blower_ctrl_out_filt < rpm_floor)
    {
        g_blower_ctrl_out_filt = rpm_floor;
    }
    const float out_filt_clamped = clampf(g_blower_ctrl_out_filt,
                                          rpm_min_f,
                                          active_rpm_max_f);
    if (out_filt_clamped != g_blower_ctrl_out_filt)
    {
        g_blower_ctrl_out_filt = out_filt_clamped;
        g_blower_ctrl_saturated = 1U;
    }

    /* --- 7. Quantise and post --- */
    int32_t out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    out_rpm         = clampi(out_rpm,
                             g_blower_ctrl_rpm_min,
                             g_blower_ctrl_rpm_max);

    if (s_resume_rpm_floor_ms > 0U)
    {
        const uint32_t step_ms = (uint32_t)lrintf(dt_s * 1000.0f);
        if (step_ms >= s_resume_rpm_floor_ms)
        {
            s_resume_rpm_floor_ms = 0U;
            s_resume_rpm_floor = 0.0f;
        }
        else
        {
            s_resume_rpm_floor_ms -= step_ms;
        }
        const int32_t floor_i = (int32_t)lrintf(s_resume_rpm_floor);
        if ((s_resume_rpm_floor > 0.0f) && (out_rpm < floor_i))
        {
            out_rpm = floor_i;
            g_blower_ctrl_out_filt = (float)out_rpm;
        }
    }

    g_blower_ctrl_out_rpm = out_rpm;

    (void)BlowerIpc_CM7_SetSpeedRpmSilent(out_rpm);

    /* --- 8. Bookkeeping --- */
    s_prev_meas_cmh2o        = meas_cmh2o;
    s_prev_meas_valid        = true;
    s_prev_p_sensor_cmh2o    = p_sensor_cmh2o;
    s_prev_p_sensor_valid    = true;
    g_blower_ctrl_step_count++;
}
