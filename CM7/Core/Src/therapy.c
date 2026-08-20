/**
  ******************************************************************************
  * @file    therapy.c
  * @brief   CPAP therapy state machine.
  *
  *  See therapy.h for the architecture overview. This file owns the
  *  state transitions, ramp interpolator, run-time counter and the
  *  glue calls into blower_ctrl / blower_ipc.
  ******************************************************************************
  */

#include "therapy.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "leak_estimator.h"
#include "flow_sensor.h"
#include "epr_ctrl.h"
#include "apap_ctrl.h"
#include "session_edf_log.h"
#include "safety_supervisor.h"
#include "dbg_log.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdint.h>

/* === Compile-time defaults === */
#define THERAPY_RAMP_START_CMH2O    4.0f         /* AirSense-style start    */
#define THERAPY_TARGET_DEFAULT      8.0f         /* sensible adult default  */
#define THERAPY_PRESS_MIN_CMH2O     4.0f
#define THERAPY_PRESS_MAX_CMH2O     25.0f
#define THERAPY_RAMP_MAX_MINUTES    45U
#define THERAPY_STOP_DURATION_S     2.0f         /* spin-down hold time     */

/* Safety tunables live in safety_supervisor.c (g_safety_*). */

/* === Public configuration (debugger-writable volatiles) === */
volatile float    g_therapy_target_cmh2o   = THERAPY_TARGET_DEFAULT;
volatile float    g_therapy_apap_min_cmh2o = 5.0f;
volatile float    g_therapy_apap_max_cmh2o = 20.0f;
volatile uint8_t  g_therapy_ramp_minutes   = 0U;
volatile uint8_t  g_therapy_mode           = (uint8_t)THERAPY_MODE_CPAP;

/* Mask-off (high-leak / disconnect) during RAMP/RUNNING.
 *
 * Uses RAW orifice total flow Flow_GetSlm() — NOT patient flow and NOT
 * Q_total−Q_vent "excess" (those wait on leak/vent estimates and false-clear
 * while the blower is still ramping). Enter needs high |Q_total| + pressure
 * collapse; clear needs sustained low |Q_total| vs the open-hose peak. */
volatile uint8_t  g_therapy_mask_off_enabled     = 1U;
volatile int32_t  g_therapy_mask_off_rpm         = 6000;  /* hold for reconnect sense */
/* One-shot soft decel (brake on CM4). Do not re-post while still above hold —
 * that restarts the FOC ramp and makes the drop look endless. */
volatile uint32_t g_therapy_mask_off_ramp_ms     = 1000U;
volatile float    g_therapy_mask_off_enter_slm   = 45.0f;  /* |Q_total| enter          */
volatile float    g_therapy_mask_off_clear_slm   = 40.0f;  /* optional flow assist   */
/* At hold RPM (~6 k), sealed tip is only ~0.5–0.8 cmH2O — use that rise vs
 * open-hose baseline as the primary mask-on detector (not a 2 cmH2O floor). */
volatile float    g_therapy_mask_off_reconnect_press_delta_cmh2o = 0.35f;
volatile float    g_therapy_mask_off_reconnect_abs_cmh2o = 0.55f;
volatile float    g_therapy_mask_off_reconnect_flow_max_slm = 55.0f;
volatile float    g_therapy_mask_off_reconnect_flow_margin_slm = 25.0f;
volatile float    g_therapy_mask_off_flow_drop_frac = 0.55f;
volatile uint32_t g_therapy_mask_off_baseline_ms = 600U;   /* learn @ hold RPM only */
volatile uint32_t g_therapy_mask_off_enter_ms    = 600U;   /* fast-path confirm       */
volatile uint32_t g_therapy_mask_off_clear_ms    = 300U;   /* tip confirm            */
volatile uint32_t g_therapy_mask_off_min_dwell_ms = 400U;
/* Short: only debounce mask-on chatter. Safety uses a separate longer grace. */
volatile float    g_therapy_mask_off_regrace_s   = 2.5f;
/* Require therapy pressure collapse before declaring mask-off (blocks breath FPs). */
volatile uint8_t  g_therapy_mask_off_require_press = 1U;
volatile float    g_therapy_mask_off_press_drop_cmh2o = 1.2f; /* vs slewed setpoint */
volatile float    g_therapy_mask_off_press_abs_cmh2o  = 4.0f; /* absolute floor    */
/* During fast-arm window, min |Q| must stay elevated (open hose ≠ breath). */
volatile float    g_therapy_mask_off_enter_min_frac = 0.35f;
/* Inhibit fast enter briefly after a breath phase edge if pressure is OK.
 * Not applied when |Q| already looks like open hose. */
volatile uint8_t  g_therapy_mask_off_breath_inhibit_enable = 1U;
volatile uint32_t g_therapy_mask_off_breath_inhibit_ms     = 1500U;
/* Vivo-style disconnect: 10 s mean leak ratio > limit (MaskOn_Off_Check.c). */
volatile uint8_t  g_therapy_mask_off_ratio_enable      = 1U;
volatile int32_t  g_therapy_mask_off_ratio_10s_limit   = 2000;
volatile uint32_t g_therapy_mask_off_ratio_enter_ms    = 8000U;
volatile uint8_t  g_therapy_mask_off_enter_via_ratio   = 0U; /* diagnostic     */
volatile uint8_t  g_therapy_mask_off_active      = 0U;
volatile uint32_t g_therapy_mask_off_enter_count = 0U; /* session mask-off events */
volatile uint32_t g_therapy_mask_on_count        = 0U; /* session reconnects     */
volatile uint8_t  g_therapy_mask_off_reconn_flow_ok  = 0U;
volatile uint8_t  g_therapy_mask_off_reconn_press_ok = 0U;
/* Enter diagnostics (1 = that gate currently satisfied). */
volatile uint8_t  g_therapy_mask_off_diag_excess  = 0U;
volatile uint8_t  g_therapy_mask_off_diag_min     = 0U;
volatile uint8_t  g_therapy_mask_off_diag_press   = 0U;
volatile uint8_t  g_therapy_mask_off_diag_breath  = 0U; /* 1 = breath inhibit active */

/* === Live state (debugger-readable volatiles) === */
volatile uint8_t  g_therapy_state          = (uint8_t)THERAPY_STATE_IDLE;
volatile float    g_therapy_setpoint_cmh2o = 0.0f;
volatile float    g_therapy_measured_cmh2o = 0.0f;
volatile float    g_therapy_leak_slm       = 0.0f;
volatile uint32_t g_therapy_run_ms         = 0U;     /* time in RAMP+RUNNING*/
volatile uint32_t g_therapy_ramp_total_ms  = 0U;     /* this session        */

/* Session pressure percentiles (#63). Histogram 0..30 cmH2O @ 0.2 step. */
#define THERAPY_P_HIST_STEP_CMH2O  0.2f
#define THERAPY_P_HIST_MAX_CMH2O   30.0f
#define THERAPY_P_HIST_N           151U   /* 0.0 .. 30.0 inclusive */

static uint32_t s_p_hist[THERAPY_P_HIST_N];
static uint32_t s_p_hist_count = 0U;
static float    s_p_sum_cmh2o  = 0.0f;
static float    s_p_min_cmh2o  = 0.0f;
static float    s_p_max_cmh2o  = 0.0f;
volatile float  g_therapy_session_p_avg = 0.0f;
volatile float  g_therapy_session_p_min = 0.0f;
volatile float  g_therapy_session_p_max = 0.0f;
volatile float  g_therapy_session_p50   = 0.0f;
volatile float  g_therapy_session_p90   = 0.0f;
volatile float  g_therapy_session_p95   = 0.0f;
volatile uint32_t g_therapy_session_p_samples = 0U;
volatile uint32_t g_therapy_ramp_elapsed_ms = 0U;
volatile uint32_t g_therapy_stop_elapsed_ms = 0U;

volatile uint8_t  g_therapy_fault          = (uint8_t)THERAPY_FAULT_NONE;

/* === Internal === */
static float s_session_target_cmh2o  = THERAPY_TARGET_DEFAULT;
static uint32_t s_therapy_start_ms   = 0U;
static bool     s_mask_off_active    = false;
static uint32_t s_mask_off_fast_enter_ms  = 0U;
static uint32_t s_mask_off_ratio_enter_ms = 0U;
static uint32_t s_mask_off_clear_ms  = 0U;
static uint32_t s_mask_off_hold_ms   = 0U;
static uint32_t s_mask_off_active_ms = 0U;
static uint32_t s_mask_off_ramp_left_ms = 0U;
static uint32_t s_mask_off_hold_elapsed_ms = 0U; /* time at hold RPM only */
static float    s_mask_off_p_baseline  = 0.0f;
static bool     s_mask_off_p_baseline_valid = false;
static float    s_mask_off_q_peak_slm  = 0.0f;  /* peak |Q| while holding */
static float    s_mask_off_arm_excess_min = 0.0f;
static uint8_t  s_mask_off_prev_breath_insp = 0U;
static uint32_t s_mask_off_last_breath_edge_ms = 0U;
static uint32_t s_reconnect_grace_until_ms = 0U;       /* mask-off re-entry block */
static uint32_t s_safety_reconnect_grace_until_ms = 0U; /* low-P safety grace     */
static uint32_t s_mask_off_resume_boost_ms = 0U;
static uint32_t s_mask_off_resume_repost_ms = 0U;

/* Last EPR settings from USB/GUI — re-applied atomically at Therapy_Start so
 * relief level cannot be left at the default 1 cmH2O if commands reorder. */
static uint8_t  s_epr_cfg_enable = 0U;
static uint8_t  s_epr_cfg_level  = 1U;
static uint8_t  s_epr_cfg_type   = 0U; /* 0=full time, 1=ramp only */

static void applyStoredEprConfig(void)
{
    Epr_SetType((s_epr_cfg_type == 1U) ? EPR_TYPE_RAMP_ONLY : EPR_TYPE_FULL_TIME);
    Epr_SetLevel(s_epr_cfg_level);
    Epr_SetEnable(s_epr_cfg_enable != 0U);
}

static bool therapyAutosetActive(void)
{
    return (g_therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET)
        || (g_therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
}

static void applyTherapySetpoint(float base_sp_cmh2o, float dt_s)
{
    const float sp = Epr_ModifySetpoint(base_sp_cmh2o, dt_s, g_therapy_state);
    BlowerCtrl_SetSetpoint(sp);
}

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void sessionPressureReset(void)
{
    for (uint32_t i = 0U; i < THERAPY_P_HIST_N; i++)
    {
        s_p_hist[i] = 0U;
    }
    s_p_hist_count = 0U;
    s_p_sum_cmh2o  = 0.0f;
    s_p_min_cmh2o  = 0.0f;
    s_p_max_cmh2o  = 0.0f;
    g_therapy_session_p_avg = 0.0f;
    g_therapy_session_p_min = 0.0f;
    g_therapy_session_p_max = 0.0f;
    g_therapy_session_p50   = 0.0f;
    g_therapy_session_p90   = 0.0f;
    g_therapy_session_p95   = 0.0f;
    g_therapy_session_p_samples = 0U;
}

static float sessionPressurePercentile(float pct)
{
    if (s_p_hist_count == 0U)
    {
        return 0.0f;
    }
    /* pct in [0,100]; target rank is ceil(pct/100 * N) style. */
    uint32_t target = (uint32_t)((pct / 100.0f) * (float)s_p_hist_count);
    if (target < 1U)
    {
        target = 1U;
    }
    if (target > s_p_hist_count)
    {
        target = s_p_hist_count;
    }

    uint32_t cum = 0U;
    for (uint32_t i = 0U; i < THERAPY_P_HIST_N; i++)
    {
        cum += s_p_hist[i];
        if (cum >= target)
        {
            return (float)i * THERAPY_P_HIST_STEP_CMH2O;
        }
    }
    return THERAPY_P_HIST_MAX_CMH2O;
}

static void sessionPressureRefreshCaches(void)
{
    if (s_p_hist_count == 0U)
    {
        g_therapy_session_p_avg = 0.0f;
        g_therapy_session_p_min = 0.0f;
        g_therapy_session_p_max = 0.0f;
        g_therapy_session_p50   = 0.0f;
        g_therapy_session_p90   = 0.0f;
        g_therapy_session_p95   = 0.0f;
        g_therapy_session_p_samples = 0U;
        return;
    }
    g_therapy_session_p_avg = s_p_sum_cmh2o / (float)s_p_hist_count;
    g_therapy_session_p_min = s_p_min_cmh2o;
    g_therapy_session_p_max = s_p_max_cmh2o;
    g_therapy_session_p50   = sessionPressurePercentile(50.0f);
    g_therapy_session_p90   = sessionPressurePercentile(90.0f);
    g_therapy_session_p95   = sessionPressurePercentile(95.0f);
    g_therapy_session_p_samples = s_p_hist_count;
}

static void sessionPressureSample(float p_cmh2o)
{
    float p = p_cmh2o;
    if (p < 0.0f)
    {
        p = 0.0f;
    }
    if (p > THERAPY_P_HIST_MAX_CMH2O)
    {
        p = THERAPY_P_HIST_MAX_CMH2O;
    }

    uint32_t bin = (uint32_t)(p / THERAPY_P_HIST_STEP_CMH2O + 0.5f);
    if (bin >= THERAPY_P_HIST_N)
    {
        bin = THERAPY_P_HIST_N - 1U;
    }

    s_p_hist[bin]++;
    s_p_hist_count++;
    s_p_sum_cmh2o += p;
    if (s_p_hist_count == 1U)
    {
        s_p_min_cmh2o = p;
        s_p_max_cmh2o = p;
    }
    else
    {
        if (p < s_p_min_cmh2o) { s_p_min_cmh2o = p; }
        if (p > s_p_max_cmh2o) { s_p_max_cmh2o = p; }
    }

    /* Refresh caches ~1 Hz equivalent (every 200 samples @ 200 Hz). */
    if ((s_p_hist_count % 200U) == 0U)
    {
        sessionPressureRefreshCaches();
    }
}

static void setState(TherapyState_t s)
{
    if ((uint8_t)s != g_therapy_state)
    {
        DBG_I("THR", "state %u -> %u",
              (unsigned)g_therapy_state, (unsigned)s);
        g_therapy_state = (uint8_t)s;
    }
}

bool Therapy_SafetyGraceActive(void)
{
    const uint32_t now = HAL_GetTick();
    if (g_safety_grace_s > 0.0f)
    {
        const uint32_t grace_ms = (uint32_t)(g_safety_grace_s * 1000.0f);
        if ((now - s_therapy_start_ms) < grace_ms)
        {
            return true;
        }
    }
    /* Post mask-on: allow pressure rebuild without low-P-at-high-RPM fault.
     * Kept separate from mask-off re-entry grace (which is much shorter). */
    if ((s_safety_reconnect_grace_until_ms != 0U)
        && (now < s_safety_reconnect_grace_until_ms))
    {
        return true;
    }
    return false;
}

static int32_t clampMaskOffRpm(int32_t rpm)
{
    if (rpm < 1000) return 1000;
    if (rpm > 12000) return 12000;
    return rpm;
}

static bool maskOffReconnectGraceActive(void)
{
    const uint32_t now = HAL_GetTick();
    return (s_reconnect_grace_until_ms != 0U) && (now < s_reconnect_grace_until_ms);
}

/* Returns the CM4 ramp duration actually posted (ms), or 0 if skipped. */
static uint32_t maskOffApplyHoldRpm(bool soft_ramp)
{
    /* ONE IPC write only. CM4 SET_SPEED auto-starts / ack-faults when idle.
     * Never re-post a long soft ramp while mech is still above hold — that
     * restarts FOC's ramp and the blower crawls down forever. */
    const int32_t rpm = clampMaskOffRpm(g_therapy_mask_off_rpm);

    BlowerIpcStatus_t st;
    (void)BlowerIpc_CM7_GetStatus(&st);
    const int32_t mech = st.mech_speed_rpm;
    const bool faults = (st.current_faults != 0U);
    const bool lost = (!st.motor_running) || faults || (mech < (rpm - 1000));

    /* Still decelerating toward hold: leave the in-flight CM4 ramp alone. */
    if (!soft_ramp && !lost && (mech > (rpm + 500)))
    {
        return 0U;
    }

    int32_t delta = mech - rpm;
    if (delta < 0)
    {
        delta = -delta;
    }

    uint32_t ramp = 150U;
    if (soft_ramp || lost || (delta > 400))
    {
        /* ~4 kRPM/s with regen brake; cap so large drops stay under ~2.5 s. */
        ramp = g_therapy_mask_off_ramp_ms;
        const uint32_t from_delta = (uint32_t)((delta * 1000) / 4000);
        if (from_delta > ramp)
        {
            ramp = from_delta;
        }
        if (lost && (mech < rpm) && (ramp < 800U))
        {
            ramp = 800U; /* re-accel from coast */
        }
        if (ramp < 400U)
        {
            ramp = 400U;
        }
        if (ramp > 2500U)
        {
            ramp = 2500U;
        }
    }

    (void)BlowerIpc_CM7_SetSpeedRpm(rpm, (uint16_t)ramp);
    return ramp;
}

static void maskOffTrackBreathEdges(void)
{
    const uint8_t insp = Leak_GetBreathInspiration();
    if (insp != s_mask_off_prev_breath_insp)
    {
        s_mask_off_last_breath_edge_ms = HAL_GetTick();
        s_mask_off_prev_breath_insp = insp;
    }
}

static bool maskOffBreathInhibitActive(void)
{
    if (g_therapy_mask_off_breath_inhibit_enable == 0U)
    {
        return false;
    }

    /* Do NOT use sustained breath-rate here — Vivo often keeps rate ≥4 while
     * the hose is already open, which blocked mask-off for many seconds. */

    if (s_mask_off_last_breath_edge_ms == 0U)
    {
        return false;
    }
    const uint32_t now = HAL_GetTick();
    const uint32_t win = (g_therapy_mask_off_breath_inhibit_ms > 0U)
                           ? g_therapy_mask_off_breath_inhibit_ms : 1U;
    return ((now - s_mask_off_last_breath_edge_ms) <= win);
}

static bool maskOffPressureCollapsed(void)
{
    const float p_meas = BlowerCtrl_GetMeasured();
    const float p_sens = BlowerCtrl_GetSensorCmh2o();
    const float sp = BlowerCtrl_GetSetpointSlewed();
    const float drop = clampf(g_therapy_mask_off_press_drop_cmh2o, 0.3f, 10.0f);
    const float abs_floor = clampf(g_therapy_mask_off_press_abs_cmh2o, 0.5f, 15.0f);

    const bool vs_setpoint = (p_meas < (sp - drop)) || (p_sens < (sp - drop));
    const bool abs_low = (p_meas < abs_floor) && (p_sens < abs_floor);
    return vs_setpoint || abs_low;
}

static float maskOffTotalFlowSlm(void)
{
    /* Orifice / SFM path — immediate, no leak or patient-flow filtering. */
    return Flow_GetSlm();
}

static bool maskOffEnterFastCondition(void)
{
    if (maskOffReconnectGraceActive())
    {
        g_therapy_mask_off_diag_excess = 0U;
        g_therapy_mask_off_diag_min = 0U;
        g_therapy_mask_off_diag_press = 0U;
        g_therapy_mask_off_diag_breath = 0U;
        return false;
    }

    maskOffTrackBreathEdges();

    /* Raw |Q_total| from the flow element. Open hose >> sealed breath peaks
     * at therapy RPM; still require pressure collapse so a strong inhale
     * alone cannot trip mask-off. */
    const float q_abs = fabsf(maskOffTotalFlowSlm());
    const float enter = clampf(g_therapy_mask_off_enter_slm, 10.0f, 150.0f);
    const bool flow_ok = (q_abs >= enter);
    g_therapy_mask_off_diag_excess = flow_ok ? 1U : 0U;

    const bool press_ok = maskOffPressureCollapsed();
    g_therapy_mask_off_diag_press = press_ok ? 1U : 0U;

    /* Breath-edge inhibit only for ambiguous mid flows. Clear open-hose
     * |Q| (≥ enter) must not wait on Vivo breath state. */
    const bool breath_block = maskOffBreathInhibitActive()
                           && !press_ok
                           && (q_abs < (enter * 1.15f));
    g_therapy_mask_off_diag_breath = breath_block ? 1U : 0U;

    if (!flow_ok)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false;
    }

    if ((g_therapy_mask_off_require_press != 0U) && !press_ok)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false;
    }

    if (breath_block)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false;
    }

    return true;
}

/* Vivo50 vCheckDisconnect: liGetMeanRatio10Sec() > DISCONNECT_LIMIT (2000). */
static bool maskOffEnterRatioCondition(void)
{
    if (maskOffReconnectGraceActive())
    {
        return false;
    }

    if (g_therapy_mask_off_ratio_enable == 0U)
    {
        return false;
    }
    if (!Leak_IsVivoEnabled())
    {
        return false;
    }

    const int32_t limit = g_therapy_mask_off_ratio_10s_limit;
    if (limit <= 0)
    {
        return false;
    }

    if (Leak_GetMeanRatio10Sec() <= limit)
    {
        return false;
    }

    /* Same pressure gate as fast path — MeanRatio can rise while breathing. */
    if ((g_therapy_mask_off_require_press != 0U) && !maskOffPressureCollapsed())
    {
        return false;
    }

    return true;
}

/* Vivo hysteresis: abort slow ratio arming when instant ratio falls well
 * below the disconnect threshold while 10 s mean is still elevated. */
static void maskOffRatioEnterHysteresis(void)
{
    if (!Leak_IsVivoEnabled())
    {
        return;
    }

    const int32_t limit = g_therapy_mask_off_ratio_10s_limit;
    if (limit <= 0)
    {
        return;
    }

    const int32_t ratio10s = Leak_GetMeanRatio10Sec();
    const int32_t ratio_inst = Leak_GetInstantRatio();

    if ((ratio10s > limit)
        && (ratio_inst > 0)
        && (ratio_inst < (limit * 10)))
    {
        s_mask_off_ratio_enter_ms = 0U;
    }
}

/* High-leak still present on the *instant* residual only. Do not use
 * Leak_IsHigh() here — its 10 s LPF stays elevated long after reconnect. */
static void maskOffTrackOpenCircuit(float p_sensor_cmh2o, float q_total_slm)
{
    const float q_abs = fabsf(q_total_slm);

    if (!s_mask_off_p_baseline_valid)
    {
        s_mask_off_p_baseline = p_sensor_cmh2o;
        s_mask_off_p_baseline_valid = true;
    }
    else if (p_sensor_cmh2o < s_mask_off_p_baseline)
    {
        s_mask_off_p_baseline = p_sensor_cmh2o;
    }

    if (q_abs > s_mask_off_q_peak_slm)
    {
        s_mask_off_q_peak_slm = q_abs;
    }
}

static bool maskOffClearCondition(void)
{
    /* Do not clear while still decelerating to hold — residual therapy
     * pressure would look like a seal and abort the 6 kRPM park. */
    BlowerIpcStatus_t st;
    (void)BlowerIpc_CM7_GetStatus(&st);
    const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
    if (st.mech_speed_rpm > (hold + 1200))
    {
        g_therapy_mask_off_reconn_press_ok = 0U;
        g_therapy_mask_off_reconn_flow_ok  = 0U;
        return false;
    }

    const float q_abs = fabsf(maskOffTotalFlowSlm());
    const float p_sensor = BlowerCtrl_GetSensorCmh2o();
    const float delta = clampf(g_therapy_mask_off_reconnect_press_delta_cmh2o,
                               0.15f, 6.0f);
    const float abs_p = clampf(g_therapy_mask_off_reconnect_abs_cmh2o, 0.35f, 10.0f);
    const float flow_frac = clampf(g_therapy_mask_off_flow_drop_frac, 0.20f, 0.90f);
    const float clear_slm = clampf(g_therapy_mask_off_clear_slm, 5.0f, 80.0f);

    /* Open hose tip at hold is near 0; require that before trusting ~0.7 tip. */
    const bool saw_open = s_mask_off_p_baseline_valid
                       && (s_mask_off_p_baseline < 0.50f);

    const bool tip_rise = saw_open
        && (p_sensor >= (s_mask_off_p_baseline + delta));
    /* Bench observation: mask-on at 6 kRPM tips ~0.7 cmH2O. */
    const bool tip_abs = saw_open && (p_sensor >= abs_p);

    const bool peak_valid = (s_mask_off_q_peak_slm >= 25.0f);
    const bool flow_from_peak =
        peak_valid && (q_abs < (s_mask_off_q_peak_slm * flow_frac));
    const bool flow_low = (q_abs < clear_slm);

    g_therapy_mask_off_reconn_press_ok = (tip_rise || tip_abs) ? 1U : 0U;
    g_therapy_mask_off_reconn_flow_ok  = (flow_low || flow_from_peak) ? 1U : 0U;

    if (!saw_open)
    {
        return false;
    }

    /* Primary: pressure tip vs open-hose baseline (no flow gate — vented
     * masks often keep |Q| high at hold RPM). */
    if (tip_rise || tip_abs)
    {
        return true;
    }

    /* Secondary assist: large flow collapse with any tip rise. */
    if ((flow_low || flow_from_peak)
        && (p_sensor >= (s_mask_off_p_baseline + 0.20f)))
    {
        return true;
    }
    return false;
}

static float currentRampSetpointCmh2o(void)
{
    if (g_therapy_ramp_total_ms == 0U)
    {
        return s_session_target_cmh2o;
    }
    if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms)
    {
        return s_session_target_cmh2o;
    }
    const float frac = (float)g_therapy_ramp_elapsed_ms
                     / (float)g_therapy_ramp_total_ms;
    return THERAPY_RAMP_START_CMH2O
         + frac * (s_session_target_cmh2o - THERAPY_RAMP_START_CMH2O);
}

static float therapyBaseSetpointCmh2o(void)
{
    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_RAMP:
        return currentRampSetpointCmh2o();
    case THERAPY_STATE_RUNNING:
        return clampf(g_therapy_target_cmh2o,
                      THERAPY_PRESS_MIN_CMH2O,
                      THERAPY_PRESS_MAX_CMH2O);
    default:
        return s_session_target_cmh2o;
    }
}

/* AirSense-style: begin at expiratory (CPAP - level), not full CPAP. */
static void applyEprStartupSnap(float base_sp_cmh2o)
{
    const float exp_sp =
        Epr_GetExpiratorySetpointCmh2o(base_sp_cmh2o, g_therapy_state);
    Epr_SetStartupHoldTarget(exp_sp);
    BlowerCtrl_SnapSetpoint(exp_sp);
}

static void enterMaskOff(void)
{
    if (s_mask_off_active)
    {
        return;
    }

    s_mask_off_active = true;
    g_therapy_mask_off_active = 1U;
    g_therapy_mask_off_enter_count++;
    s_mask_off_hold_ms = 0U;
    s_mask_off_active_ms = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_clear_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_p_baseline = 0.0f;
    /* Peak is learned only after hold RPM settles — not at therapy RPM. */
    s_mask_off_q_peak_slm = 0.0f;

    Leak_SetDisconnectFreeze(true);

    Epr_ResetBreathState();
    BlowerCtrl_SetEnable(false);
    /* Adaptive soft ramp (may be > default) — block clear until it finishes. */
    s_mask_off_ramp_left_ms = maskOffApplyHoldRpm(true);

    DBG_I("THR", "mask off: ramp to %ld rpm in %lu ms (ratio=%u)",
          (long)clampMaskOffRpm(g_therapy_mask_off_rpm),
          (unsigned long)s_mask_off_ramp_left_ms,
          (unsigned)g_therapy_mask_off_enter_via_ratio);
}

static void exitMaskOff(float dt_s)
{
    if (!s_mask_off_active)
    {
        return;
    }

    (void)dt_s;

    s_mask_off_active = false;
    g_therapy_mask_off_active = 0U;
    g_therapy_mask_on_count++;
    s_mask_off_fast_enter_ms = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms = 0U;
    s_mask_off_active_ms = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_ramp_left_ms = 0U;
    s_mask_off_arm_excess_min = 0.0f;
    g_therapy_mask_off_enter_via_ratio = 0U;

    Leak_SetDisconnectFreeze(false);

    {
        /* Short re-entry debounce only — do not block the next mask-off for 12 s. */
        float regrace_s = g_therapy_mask_off_regrace_s;
        if (regrace_s < 0.5f)
        {
            regrace_s = 0.5f;
        }
        if (regrace_s > 6.0f)
        {
            regrace_s = 6.0f;
        }
        s_reconnect_grace_until_ms =
            HAL_GetTick() + (uint32_t)(regrace_s * 1000.0f);

        /* Longer safety grace while pressure climbs from tip (~0.7) to set CPAP. */
        s_safety_reconnect_grace_until_ms = HAL_GetTick() + 12000U;
    }

    const float base_sp = therapyBaseSetpointCmh2o();
    float resume_sp = base_sp;
    if (Epr_IsEnabled())
    {
        Epr_SetStartupHoldTarget(
            Epr_GetExpiratorySetpointCmh2o(base_sp, g_therapy_state));
    }

    DBG_I("THR", "mask on: resume pressure control -> target %.1f",
          (double)resume_sp);

    Leak_Reseed();
    Apap_OnMaskReconnect();
    Epr_OnTherapyStart();

    /* Never STOP here — a STOP+SetSpeed race leaves the rotor coasting to 0. */
    BlowerCtrl_ClearManualFlag();

    const float hold_rpm = (float)clampMaskOffRpm(g_therapy_mask_off_rpm);
    const float ss_rpm = BlowerCtrl_EstimateSteadyStateRpm(resume_sp);
    float seed_rpm = ss_rpm;
    if (seed_rpm < hold_rpm)
    {
        seed_rpm = hold_rpm;
    }

    /* Assertive resume: keep PWM armed at ≥ hold RPM while pressure rebuilds. */
    Safety_Supervisor_ResetFaultTimers();

    BlowerCtrl_SetEnable(true);
    BlowerCtrl_SnapSetpoint(resume_sp);
    BlowerCtrl_ReseedForTherapyResume(resume_sp);
    BlowerCtrl_SetResumeRpmFloor(hold_rpm, 5000U);
    /* Vivo MeanRatio reseed attributes hose-fill flow to Q_patient briefly. */
    BlowerCtrl_SuppressPatientFlowFf(3000U);
    {
        int32_t tgt = (int32_t)lrintf(seed_rpm);
        BlowerIpcStatus_t st;
        (void)BlowerIpc_CM7_GetStatus(&st);
        int32_t delta = tgt - st.mech_speed_rpm;
        if (delta < 0)
        {
            delta = -delta;
        }
        /* Prefer a brisk climb back to therapy speed (accel ≠ regen OV). */
        uint32_t ramp = 500U;
        if (!st.motor_running || (st.mech_speed_rpm < 1500))
        {
            ramp = 1200U; /* soft re-accel from near-stop / after OV */
        }
        const uint32_t from_delta = (uint32_t)((delta * 1000) / 4000);
        if (from_delta > ramp)
        {
            ramp = from_delta;
        }
        if (ramp > 3000U)
        {
            ramp = 3000U;
        }
        /* ONE IPC write — CM4 SET_SPEED auto ack/start when idle/fault-cleared. */
        (void)BlowerIpc_CM7_SetSpeedRpm(tgt, (uint16_t)ramp);
    }

    s_mask_off_resume_boost_ms = 5000U;
    s_mask_off_resume_repost_ms = 0U;
    g_therapy_mask_off_reconn_flow_ok  = 0U;
    g_therapy_mask_off_reconn_press_ok = 0U;
}

/* Keep commanding therapy pressure after mask-on so RPM cannot linger at
 * hold speed if the first IPC set was lost or the PID seed was soft. */
static void maskOffResumeBoost(float dt_s, uint32_t dt_ms)
{
    if (s_mask_off_resume_boost_ms == 0U)
    {
        return;
    }

    if (dt_ms >= s_mask_off_resume_boost_ms)
    {
        s_mask_off_resume_boost_ms = 0U;
    }
    else
    {
        s_mask_off_resume_boost_ms -= dt_ms;
    }

    if (!BlowerCtrl_IsEnabled())
    {
        BlowerCtrl_SetEnable(true);
    }

    const float base_sp = therapyBaseSetpointCmh2o();
    const float sp = Epr_ModifySetpoint(base_sp, dt_s, g_therapy_state);
    /* Keep full target during resume so pressure actually returns to set CPAP. */
    BlowerCtrl_SnapSetpoint(sp);

    /* Only re-seed IPC if collapsing toward stop or stuck near hold. */
    s_mask_off_resume_repost_ms += dt_ms;
    if (s_mask_off_resume_repost_ms >= 250U)
    {
        s_mask_off_resume_repost_ms = 0U;
        const float meas = BlowerCtrl_GetMeasured();
        BlowerIpcStatus_t st;
        (void)BlowerIpc_CM7_GetStatus(&st);
        const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
        const int32_t ss = (int32_t)lrintf(BlowerCtrl_EstimateSteadyStateRpm(sp));
        const bool collapsing = (!st.motor_running)
            || (st.mech_speed_rpm < 2000)
            || (st.current_faults != 0U);
        const bool stuck_low = (meas < (sp - 1.0f))
            && (st.mech_speed_rpm < (hold + 800));
        if (collapsing || stuck_low)
        {
            int32_t rpm = ss;
            if (rpm < hold)
            {
                rpm = hold;
            }
            const uint16_t ramp = collapsing ? 1000U : 400U;
            (void)BlowerIpc_CM7_SetSpeedRpm(rpm, ramp);
        }
    }
}

static bool maskOffResumeBoostActive(void)
{
    return (s_mask_off_resume_boost_ms > 0U);
}

/* Returns true when mask-off is active and normal RAMP/RUNNING work should
 * be skipped for this tick. */
static bool updateMaskOff(float dt_s, uint32_t dt_ms)
{
    if ((g_therapy_mask_off_enabled == 0U)
        || ((g_therapy_state != (uint8_t)THERAPY_STATE_RAMP)
            && (g_therapy_state != (uint8_t)THERAPY_STATE_RUNNING)))
    {
        if (s_mask_off_active)
        {
            exitMaskOff(dt_s);
        }
        return false;
    }

    if (!s_mask_off_active)
    {
        const bool fast_cond  = maskOffEnterFastCondition();
        const bool ratio_cond = maskOffEnterRatioCondition();
        const float q_abs = fabsf(maskOffTotalFlowSlm());
        const float enter = clampf(g_therapy_mask_off_enter_slm, 10.0f, 150.0f);
        const float min_frac = clampf(g_therapy_mask_off_enter_min_frac, 0.15f, 0.90f);

        if (fast_cond)
        {
            if (s_mask_off_fast_enter_ms == 0U)
            {
                s_mask_off_arm_excess_min = q_abs;
            }
            else if (q_abs < s_mask_off_arm_excess_min)
            {
                s_mask_off_arm_excess_min = q_abs;
            }
            s_mask_off_fast_enter_ms += dt_ms;
        }
        else
        {
            s_mask_off_fast_enter_ms = 0U;
            s_mask_off_arm_excess_min = 0.0f;
            g_therapy_mask_off_diag_min = 0U;
        }

        if (ratio_cond)
        {
            s_mask_off_ratio_enter_ms += dt_ms;
        }
        else
        {
            s_mask_off_ratio_enter_ms = 0U;
        }

        maskOffRatioEnterHysteresis();

        const uint32_t fast_need_nom = (g_therapy_mask_off_enter_ms > 0U)
                                         ? g_therapy_mask_off_enter_ms : 1U;
        /* Strong open-hose: high |Q| + collapsed P → shorter confirm (~0.3 s). */
        const bool strong_open = fast_cond
            && (q_abs >= (enter * 1.25f))
            && (g_therapy_mask_off_diag_press != 0U);
        const uint32_t fast_need = strong_open
            ? ((fast_need_nom > 300U) ? 300U : fast_need_nom)
            : fast_need_nom;
        const uint32_t ratio_need = (g_therapy_mask_off_ratio_enter_ms > 0U)
                                      ? g_therapy_mask_off_ratio_enter_ms : 8000U;

        const bool min_ok = (s_mask_off_fast_enter_ms > 0U)
            && (s_mask_off_arm_excess_min >= (enter * min_frac));
        g_therapy_mask_off_diag_min = min_ok ? 1U : 0U;

        const bool fast_ready  = fast_cond
                              && (s_mask_off_fast_enter_ms >= fast_need)
                              && min_ok;
        const bool ratio_ready = ratio_cond && (s_mask_off_ratio_enter_ms >= ratio_need);

        if (fast_ready || ratio_ready)
        {
            g_therapy_mask_off_enter_via_ratio = ratio_ready && !fast_ready ? 1U : 0U;
            enterMaskOff();
            s_mask_off_fast_enter_ms = 0U;
            s_mask_off_ratio_enter_ms = 0U;
            s_mask_off_arm_excess_min = 0.0f;
        }

        return false;
    }

    /* Soft-ramp to hold RPM — track open-hose pressure min. Never clear on
     * absolute P here: residual therapy pressure after disconnect looks
     * sealed and aborts the 6 kRPM park (RPM then runs away open-hose). */
    if (s_mask_off_ramp_left_ms > 0U)
    {
        s_mask_off_active_ms += dt_ms;
        {
            const float p_sensor = BlowerCtrl_GetSensorCmh2o();
            if (!s_mask_off_p_baseline_valid)
            {
                s_mask_off_p_baseline = p_sensor;
                s_mask_off_p_baseline_valid = true;
            }
            else if (p_sensor < s_mask_off_p_baseline)
            {
                s_mask_off_p_baseline = p_sensor;
            }
        }

        /* True loss of control only — never re-post while still decelerating. */
        {
            BlowerIpcStatus_t st;
            (void)BlowerIpc_CM7_GetStatus(&st);
            const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
            const bool lost = (!st.motor_running)
                || (st.current_faults != 0U)
                || (st.mech_speed_rpm < (hold / 2));
            if (lost)
            {
                s_mask_off_ramp_left_ms = maskOffApplyHoldRpm(true);
                s_mask_off_clear_ms = 0U;
                return true;
            }
        }

        if (dt_ms >= s_mask_off_ramp_left_ms)
        {
            s_mask_off_ramp_left_ms = 0U;
            s_mask_off_hold_ms = 0U;
            s_mask_off_hold_elapsed_ms = 0U;
            /* Keep open-hose P baseline from the ramp. Reset Q peak only so
             * hold-RPM open flow can be learned if the hose is still open. */
            s_mask_off_q_peak_slm = 0.0f;
            (void)maskOffApplyHoldRpm(false);
        }
        else
        {
            s_mask_off_ramp_left_ms -= dt_ms;
        }
        s_mask_off_clear_ms = 0U;
        return true;
    }

    /* Hold low RPM; learn open-hose Q only if still open. */
    s_mask_off_hold_ms += dt_ms;
    s_mask_off_active_ms += dt_ms;
    s_mask_off_hold_elapsed_ms += dt_ms;
    maskOffTrackOpenCircuit(BlowerCtrl_GetSensorCmh2o(), maskOffTotalFlowSlm());
    if (s_mask_off_hold_ms >= 750U)
    {
        s_mask_off_hold_ms = 0U;
        (void)maskOffApplyHoldRpm(false);
    }

    const uint32_t min_dwell = (g_therapy_mask_off_min_dwell_ms > 0U)
                                 ? g_therapy_mask_off_min_dwell_ms : 1U;
    const uint32_t learn_ms  = (g_therapy_mask_off_baseline_ms > 0U)
                                 ? g_therapy_mask_off_baseline_ms : 1U;
    if (s_mask_off_hold_elapsed_ms < min_dwell)
    {
        s_mask_off_clear_ms = 0U;
        return true;
    }
    if (s_mask_off_hold_elapsed_ms < learn_ms)
    {
        s_mask_off_clear_ms = 0U;
        return true;
    }

    if (maskOffClearCondition())
    {
        s_mask_off_clear_ms += dt_ms;
        const uint32_t need = (g_therapy_mask_off_clear_ms > 0U)
                                ? g_therapy_mask_off_clear_ms : 1U;
        if (s_mask_off_clear_ms >= need)
        {
            exitMaskOff(dt_s);
            return false;
        }
    }
    else
    {
        s_mask_off_clear_ms = 0U;
    }

    return true;
}

/* Latch a fault: log it, kill the blower, transition to FAULT. */
void Therapy_LatchFault(TherapyFault_t f, const char *why)
{
    if (g_therapy_fault != (uint8_t)THERAPY_FAULT_NONE)
    {
        return;  /* first fault wins */
    }

    DBG_E("THR", "FAULT %u: %s (rpm=%ld meas=%.2f sp=%.2f)",
          (unsigned)f, why,
          (long)BlowerCtrl_GetOutputRpm(),
          (double)g_therapy_measured_cmh2o,
          (double)g_therapy_setpoint_cmh2o);

    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();

    g_therapy_fault = (uint8_t)f;
    setState(THERAPY_STATE_FAULT);
}

void Therapy_Init(void)
{
    g_therapy_target_cmh2o    = THERAPY_TARGET_DEFAULT;
    g_therapy_apap_min_cmh2o  = 5.0f;
    g_therapy_apap_max_cmh2o  = 20.0f;
    g_therapy_ramp_minutes    = 0U;
    g_therapy_mode            = (uint8_t)THERAPY_MODE_CPAP;
    g_therapy_state           = (uint8_t)THERAPY_STATE_IDLE;
    g_therapy_setpoint_cmh2o  = 0.0f;
    g_therapy_measured_cmh2o  = 0.0f;
    g_therapy_leak_slm        = 0.0f;
    g_therapy_run_ms          = 0U;
    g_therapy_ramp_total_ms   = 0U;
    g_therapy_ramp_elapsed_ms = 0U;
    g_therapy_stop_elapsed_ms = 0U;
    sessionPressureReset();

    g_therapy_fault           = (uint8_t)THERAPY_FAULT_NONE;
    s_mask_off_active         = false;
    g_therapy_mask_off_active = 0U;
    s_mask_off_fast_enter_ms  = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms       = 0U;
    s_mask_off_hold_ms        = 0U;
    s_mask_off_active_ms      = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_q_peak_slm = 0.0f;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_prev_breath_insp = 0U;
    s_mask_off_last_breath_edge_ms = 0U;
    s_mask_off_resume_boost_ms = 0U;
    s_mask_off_resume_repost_ms = 0U;
    s_reconnect_grace_until_ms = 0U;
    s_safety_reconnect_grace_until_ms = 0U;

    Safety_Supervisor_Init();

    Epr_Init();
    Apap_Init();
    SessionEdf_Init();
}

void Therapy_SetTargetPressure(float cmh2o)
{
    g_therapy_target_cmh2o = clampf(cmh2o,
                                    THERAPY_PRESS_MIN_CMH2O,
                                    THERAPY_PRESS_MAX_CMH2O);
}

float Therapy_GetTargetPressure(void) { return g_therapy_target_cmh2o; }

void Therapy_SetRampMinutes(uint8_t minutes)
{
    if (minutes > THERAPY_RAMP_MAX_MINUTES) minutes = THERAPY_RAMP_MAX_MINUTES;
    g_therapy_ramp_minutes = minutes;
}

uint8_t Therapy_GetRampMinutes(void) { return g_therapy_ramp_minutes; }

static void therapyNormalizeApapRange(void)
{
    float min_p = clampf(g_therapy_apap_min_cmh2o,
                         THERAPY_PRESS_MIN_CMH2O,
                         THERAPY_PRESS_MAX_CMH2O);
    float max_p = clampf(g_therapy_apap_max_cmh2o,
                         THERAPY_PRESS_MIN_CMH2O,
                         THERAPY_PRESS_MAX_CMH2O);
    if (max_p < (min_p + 0.5f))
    {
        max_p = min_p + 0.5f;
        if (max_p > THERAPY_PRESS_MAX_CMH2O)
        {
            max_p = THERAPY_PRESS_MAX_CMH2O;
            min_p = max_p - 0.5f;
        }
    }
    g_therapy_apap_min_cmh2o = min_p;
    g_therapy_apap_max_cmh2o = max_p;
}

void Therapy_SetMode(TherapyMode_t m)
{
    g_therapy_mode = (uint8_t)m;
    BlowerCtrl_SyncCpapProfilesForMode((uint8_t)m);
}

TherapyMode_t Therapy_GetMode(void)   { return (TherapyMode_t)g_therapy_mode; }

const char* Therapy_GetModeName(void)
{
    switch ((TherapyMode_t)g_therapy_mode)
    {
    case THERAPY_MODE_CPAP:             return "CPAP";
    case THERAPY_MODE_AUTOSET:          return "Autoset";
    case THERAPY_MODE_AUTOSET_FOR_HER:  return "Autoset for her";
    default:                            return "?";
    }
}

void Therapy_SetApapMinPressure(float cmh2o)
{
    g_therapy_apap_min_cmh2o = clampf(cmh2o,
                                      THERAPY_PRESS_MIN_CMH2O,
                                      THERAPY_PRESS_MAX_CMH2O);
    therapyNormalizeApapRange();
}

void Therapy_SetApapMaxPressure(float cmh2o)
{
    g_therapy_apap_max_cmh2o = clampf(cmh2o,
                                      THERAPY_PRESS_MIN_CMH2O,
                                      THERAPY_PRESS_MAX_CMH2O);
    therapyNormalizeApapRange();
}

float Therapy_GetApapMinPressure(void) { return g_therapy_apap_min_cmh2o; }
float Therapy_GetApapMaxPressure(void) { return g_therapy_apap_max_cmh2o; }

const char* Therapy_GetApapStateName(void)
{
    if (!therapyAutosetActive())
    {
        return "";
    }
    return Apap_GetStateName();
}

TherapyState_t Therapy_GetState(void) { return (TherapyState_t)g_therapy_state; }

const char* Therapy_GetStateName(void)
{
    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_IDLE:      return "Idle";
    case THERAPY_STATE_RAMP:      return "Ramp";
    case THERAPY_STATE_RUNNING:   return "Running";
    case THERAPY_STATE_STOPPING:  return "Stopping";
    case THERAPY_STATE_FAULT:     return "Fault";
    default:                      return "?";
    }
}

TherapyFault_t Therapy_GetFault(void) { return (TherapyFault_t)g_therapy_fault; }

const char* Therapy_GetFaultName(void)
{
    switch ((TherapyFault_t)g_therapy_fault)
    {
    case THERAPY_FAULT_NONE:                 return "OK";
    case THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM:return "Sensor/Hose Lost";
    case THERAPY_FAULT_OVER_PRESSURE:        return "Over Pressure";
    case THERAPY_FAULT_SENSOR_OUT_OF_RANGE:  return "Sensor Out Of Range";
    case THERAPY_FAULT_RPM_SATURATED:        return "Cannot Reach Target";
    case THERAPY_FAULT_CM4_HEARTBEAT:        return "CM4 Heartbeat Lost";
    default:                                  return "?";
    }
}

void Therapy_ClearFault(void)
{
    g_therapy_fault = (uint8_t)THERAPY_FAULT_NONE;
    Safety_Supervisor_ResetFaultTimers();
    if (g_therapy_state == (uint8_t)THERAPY_STATE_FAULT)
    {
        setState(THERAPY_STATE_IDLE);
    }
}

float    Therapy_GetSetpointCmh2o(void) { return g_therapy_setpoint_cmh2o; }
float    Therapy_GetMeasuredCmh2o(void) { return g_therapy_measured_cmh2o; }
float    Therapy_GetLeakSlm(void)       { return g_therapy_leak_slm; }
bool     Therapy_IsLeakHigh(void)       { return Leak_IsHigh(); }
bool     Therapy_IsLeakLow(void)        { return Leak_IsLow(); }
LeakSeverity_t Therapy_GetLeakSeverity(void) { return Leak_GetSeverity(); }
const char* Therapy_GetLeakSeverityName(void) { return Leak_GetSeverityName(); }
float    Therapy_GetLeakPercentOfVent(void) { return Leak_GetPercentOfVent(); }
uint32_t Therapy_GetMaskOffEnterCount(void) { return g_therapy_mask_off_enter_count; }
uint32_t Therapy_GetMaskOnCount(void)       { return g_therapy_mask_on_count; }
uint32_t Therapy_GetRunSeconds(void)    { return g_therapy_run_ms / 1000U; }

float Therapy_GetSessionPressureAvg(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p_avg;
}
float Therapy_GetSessionPressureMin(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p_min;
}
float Therapy_GetSessionPressureMax(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p_max;
}
float Therapy_GetSessionPressureP50(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p50;
}
float Therapy_GetSessionPressureP90(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p90;
}
float Therapy_GetSessionPressureP95(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p95;
}
uint32_t Therapy_GetSessionPressureSamples(void)
{
    return s_p_hist_count;
}

uint32_t Therapy_GetRampRemainingSeconds(void)
{
    if (g_therapy_state != (uint8_t)THERAPY_STATE_RAMP) return 0U;
    if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms) return 0U;
    return (g_therapy_ramp_total_ms - g_therapy_ramp_elapsed_ms) / 1000U;
}

void Therapy_SetEprEnabled(bool en)
{
    const bool was_enabled = Epr_IsEnabled();
    s_epr_cfg_enable = en ? 1U : 0U;
    Epr_SetEnable(en);

    if (!en || was_enabled)
    {
        return;
    }

    const TherapyState_t st = (TherapyState_t)g_therapy_state;
    if ((st != THERAPY_STATE_RAMP) && (st != THERAPY_STATE_RUNNING))
    {
        return;
    }

    applyStoredEprConfig();
    Epr_OnTherapyStart();
    applyEprStartupSnap(therapyBaseSetpointCmh2o());
}

bool Therapy_IsEprEnabled(void)
{
    return Epr_IsEnabled();
}

void Therapy_SetEprType(uint8_t type)
{
    s_epr_cfg_type = (type == 1U) ? 1U : 0U;
    Epr_SetType((type == 1U) ? EPR_TYPE_RAMP_ONLY : EPR_TYPE_FULL_TIME);
}

uint8_t Therapy_GetEprType(void)
{
    return (Epr_GetType() == EPR_TYPE_RAMP_ONLY) ? 1U : 0U;
}

void Therapy_SetEprLevel(uint8_t level)
{
    if (level < 1U) { level = 1U; }
    if (level > 3U) { level = 3U; }
    s_epr_cfg_level = level;
    Epr_SetLevel(level);
}

uint8_t Therapy_GetEprLevel(void)
{
    return Epr_GetLevel();
}

void Therapy_SetEprReliefCmh2o(float cmh2o)
{
    uint8_t level = (uint8_t)lrintf(cmh2o);
    if (level < 1U) { level = 1U; }
    if (level > 3U) { level = 3U; }
    s_epr_cfg_level = level;
    Epr_SetReliefCmh2o(cmh2o);
}

float Therapy_GetEprReliefCmh2o(void)
{
    return Epr_GetReliefCmh2o();
}

bool Therapy_IsMaskOffActive(void)
{
    return s_mask_off_active;
}

void Therapy_SetMaskOffEnabled(bool en)
{
    g_therapy_mask_off_enabled = en ? 1U : 0U;
    if (!en && s_mask_off_active)
    {
        exitMaskOff(0.005f);
    }
}

bool Therapy_IsMaskOffEnabled(void)
{
    return (g_therapy_mask_off_enabled != 0U);
}

void Therapy_ForceMaskOn(void)
{
    if (s_mask_off_active)
    {
        exitMaskOff(0.005f);
    }
}

void Therapy_Start(void)
{
    /* A pending FAULT must be cleared before we'll re-arm the blower. */
    if (g_therapy_fault != (uint8_t)THERAPY_FAULT_NONE)
    {
        Therapy_ClearFault();
    }

    /* Re-apply the safety RPM ceiling in case it was poked. */
    extern volatile int32_t g_blower_ctrl_rpm_max;
    g_blower_ctrl_rpm_max = g_safety_safe_rpm_max;

    /* Snap a self-consistent set of session parameters now, so changes
     * the GUI makes during therapy don't yank the running ramp around. */
    if (therapyAutosetActive())
    {
        Apap_OnTherapyStart(g_therapy_apap_min_cmh2o,
                            g_therapy_apap_max_cmh2o,
                            g_therapy_mode);
        s_session_target_cmh2o = Apap_GetTargetPressure();
        g_therapy_target_cmh2o = s_session_target_cmh2o;
    }
    else
    {
        s_session_target_cmh2o   = clampf(g_therapy_target_cmh2o,
                                          THERAPY_PRESS_MIN_CMH2O,
                                          THERAPY_PRESS_MAX_CMH2O);
    }
    g_therapy_ramp_total_ms  = (uint32_t)g_therapy_ramp_minutes * 60U * 1000U;
    g_therapy_ramp_elapsed_ms = 0U;
    g_therapy_stop_elapsed_ms = 0U;
    g_therapy_run_ms         = 0U;
    sessionPressureReset();
    Safety_Supervisor_ResetFaultTimers();
    s_therapy_start_ms       = HAL_GetTick();
    s_reconnect_grace_until_ms = 0U;
    s_safety_reconnect_grace_until_ms = 0U;
    s_mask_off_active        = false;
    g_therapy_mask_off_active = 0U;
    s_mask_off_fast_enter_ms  = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms      = 0U;
    s_mask_off_hold_ms       = 0U;
    s_mask_off_active_ms     = 0U;
    s_mask_off_ramp_left_ms  = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_q_peak_slm = 0.0f;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_prev_breath_insp = 0U;
    s_mask_off_last_breath_edge_ms = 0U;
    s_mask_off_resume_boost_ms = 0U;
    s_mask_off_resume_repost_ms = 0U;
    g_therapy_mask_off_enter_count = 0U;
    g_therapy_mask_on_count = 0U;

    /* Configure the PID, then enable. Set therapy state first so EPR
     * logic (Full time / Ramp only) applies on the very first setpoint. */
    if (g_therapy_ramp_total_ms > 0U)
    {
        setState(THERAPY_STATE_RAMP);
    }
    else
    {
        setState(THERAPY_STATE_RUNNING);
    }

    Leak_SessionReset();
    Leak_SetSessionActive(true);
    Leak_Reseed();
    applyStoredEprConfig();
    Epr_OnTherapyStart();

    /* CPAP Plateau / Hybrid: auto-apply preferred profile on CPAP start. */
    BlowerCtrl_SyncCpapProfilesForMode(g_therapy_mode);

    if (BlowerCtrl_IsManualActive())
    {
        BlowerCtrl_ManualStop();
    }

    const float start_sp = (g_therapy_ramp_total_ms > 0U)
                              ? THERAPY_RAMP_START_CMH2O
                              : s_session_target_cmh2o;
    if (Epr_IsEnabled())
    {
        applyEprStartupSnap(start_sp);
    }
    else
    {
        applyTherapySetpoint(start_sp, 0.005f);
        BlowerCtrl_SnapSetpoint(BlowerCtrl_GetSetpoint());
    }
    BlowerCtrl_SetEnable(true);

    /* Clear any CM4 FOC latch (e.g. over-voltage from a prior hard
     * deceleration) before START — otherwise MC_StartMotor1() refuses. */
    (void)BlowerIpc_CM7_AcknowledgeFault();
    (void)BlowerIpc_CM7_Start();

    SessionEdf_OnTherapyStart();

    DBG_I("THR", "start target=%.1f epr=%u lvl=%u exp=%.1f cmH2O ramp=%u min",
          (double)s_session_target_cmh2o,
          (unsigned)Epr_IsEnabled(),
          (unsigned)s_epr_cfg_level,
          (double)Epr_GetExpiratorySetpointCmh2o(s_session_target_cmh2o,
                                                 g_therapy_state),
          (unsigned)g_therapy_ramp_minutes);
}

void Therapy_Stop(void)
{
    if (g_therapy_state == (uint8_t)THERAPY_STATE_IDLE)
    {
        return;
    }

    Apap_OnTherapyStop();
    Leak_SetSessionActive(false);
    SessionEdf_OnTherapyStop();

    /* Drop PID control immediately so the motor coasts down rather than
     * fighting to maintain pressure during spin-down. */
    if (s_mask_off_active)
    {
        s_mask_off_active = false;
        g_therapy_mask_off_active = 0U;
        s_mask_off_ramp_left_ms = 0U;
        Leak_SetDisconnectFreeze(false);
    }
    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();
    /* Leave FAULT_OVER so the next Start can arm the motor again. */
    (void)BlowerIpc_CM7_AcknowledgeFault();

    g_therapy_stop_elapsed_ms = 0U;
    /* If we're in FAULT, the supervisor latched the cause and the blower
     * is already off. A user Stop is the operator acknowledging the
     * fault, so go straight to STOPPING (which then transitions to IDLE
     * after the spin-down hold) and clear the fault flag. */
    if (g_therapy_state == (uint8_t)THERAPY_STATE_FAULT)
    {
        Therapy_ClearFault();
    }
    setState(THERAPY_STATE_STOPPING);

    sessionPressureRefreshCaches();
    DBG_I("THR", "stop after %lu s  P50=%.1f P90=%.1f P95=%.1f avg=%.1f",
          (unsigned long)Therapy_GetRunSeconds(),
          (double)g_therapy_session_p50,
          (double)g_therapy_session_p90,
          (double)g_therapy_session_p95,
          (double)g_therapy_session_p_avg);
}

void Therapy_Update(float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.5f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    /* Always refresh observable state (so the GUI can show current
     * values regardless of state). */
    g_therapy_measured_cmh2o = BlowerCtrl_GetMeasured();
    g_therapy_leak_slm       = Leak_GetUnintentionalSlm();

    /* Breath detection for EPR runs every therapy tick. */
    Epr_Update(Flow_GetPatientSlm(), BlowerCtrl_GetMeasured(),
               therapyBaseSetpointCmh2o(), dt_s);
    SessionEdf_Update(dt_s);

    if (Therapy_GetFault() != THERAPY_FAULT_NONE)
    {
        g_therapy_setpoint_cmh2o = BlowerCtrl_GetSetpoint();
        return;
    }

    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_IDLE:
    {
        /* Nothing to do; waiting for Therapy_Start(). */
        break;
    }

    case THERAPY_STATE_RAMP:
    {
        if (updateMaskOff(dt_s, dt_ms))
        {
            g_therapy_run_ms += dt_ms;
            break;
        }
        maskOffResumeBoost(dt_s, dt_ms);

        /* Linear ramp from THERAPY_RAMP_START_CMH2O to session target
         * over ramp_total_ms. The PID's internal slew limiter (15
         * cmH2O/s) is far faster than this slow ramp (~0.01 cmH2O/s
         * typical) so the per-tick step is, for all practical purposes,
         * a smooth continuous trajectory. */
        g_therapy_ramp_elapsed_ms += dt_ms;
        g_therapy_run_ms          += dt_ms;

        if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms)
        {
            applyTherapySetpoint(s_session_target_cmh2o, dt_s);
            setState(THERAPY_STATE_RUNNING);
        }
        else
        {
            const float frac = (float)g_therapy_ramp_elapsed_ms
                             / (float)g_therapy_ramp_total_ms;
            const float sp   = THERAPY_RAMP_START_CMH2O
                             + frac * (s_session_target_cmh2o
                                       - THERAPY_RAMP_START_CMH2O);
            applyTherapySetpoint(sp, dt_s);
        }
        sessionPressureSample(g_therapy_measured_cmh2o);
        break;
    }

    case THERAPY_STATE_RUNNING:
    {
        if (updateMaskOff(dt_s, dt_ms))
        {
            g_therapy_run_ms += dt_ms;
            break;
        }
        maskOffResumeBoost(dt_s, dt_ms);

        /* Always apply therapy setpoint while resuming — boost only snaps SP
         * and optionally kick-starts RPM if still parked at hold. */
        if (therapyAutosetActive())
        {
            const float apap_target = Apap_Update(dt_s,
                                                  g_therapy_state,
                                                  s_mask_off_active);
            s_session_target_cmh2o = apap_target;
            g_therapy_target_cmh2o   = apap_target;
            applyTherapySetpoint(s_session_target_cmh2o, dt_s);
        }
        else
        {
            /* Hold target. The user may have edited the target via the GUI
             * during therapy - re-apply each tick so the PID tracks. */
            const float live_target = clampf(g_therapy_target_cmh2o,
                                             THERAPY_PRESS_MIN_CMH2O,
                                             THERAPY_PRESS_MAX_CMH2O);
            if (fabsf(live_target - s_session_target_cmh2o) > 0.05f)
            {
                s_session_target_cmh2o = live_target;
            }
            applyTherapySetpoint(s_session_target_cmh2o, dt_s);
        }
        g_therapy_run_ms += dt_ms;
        sessionPressureSample(g_therapy_measured_cmh2o);
        break;
    }

    case THERAPY_STATE_STOPPING:
    {
        g_therapy_stop_elapsed_ms += dt_ms;
        if (g_therapy_stop_elapsed_ms
              >= (uint32_t)(THERAPY_STOP_DURATION_S * 1000.0f))
        {
            setState(THERAPY_STATE_IDLE);
        }
        break;
    }

    case THERAPY_STATE_FAULT:
    {
        /* No automatic recovery yet. User has to call Stop() then Start()
         * once the offending condition (motor fault, persistent high
         * leak) is cleared. */
        BlowerCtrl_SetEnable(false);
        break;
    }

    default:
        setState(THERAPY_STATE_IDLE);
        break;
    }

    g_therapy_setpoint_cmh2o = BlowerCtrl_GetSetpoint();
}
