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

/* Mask-off (high-leak) handling during RAMP/RUNNING.
 *
 * False-positive note: |Q_total − Q_vent| ≈ patient flow on a sealed circuit,
 * so strong inspiration alone can look like "disconnect". Enter therefore
 * requires sustained excess AND pressure collapse (see maskOffEnter*). */
volatile uint8_t  g_therapy_mask_off_enabled     = 1U;
volatile int32_t  g_therapy_mask_off_rpm         = 9000;  /* hold for reconnect sense */
volatile float    g_therapy_mask_off_enter_slm   = 40.0f;  /* instant excess  */
volatile float    g_therapy_mask_off_reconnect_press_delta_cmh2o = 0.20f;
volatile float    g_therapy_mask_off_reconnect_abs_cmh2o = 0.40f;
volatile float    g_therapy_mask_off_reconnect_flow_max_slm = 55.0f;
volatile float    g_therapy_mask_off_reconnect_flow_margin_slm = 25.0f;
volatile float    g_therapy_mask_off_flow_drop_frac = 0.55f;
volatile uint32_t g_therapy_mask_off_baseline_ms = 800U;   /* learn open-circuit P min */
volatile uint32_t g_therapy_mask_off_enter_ms    = 1500U;
volatile uint32_t g_therapy_mask_off_clear_ms    = 400U;   /* faster mask-on confirm */
volatile uint32_t g_therapy_mask_off_min_dwell_ms = 800U;
volatile float    g_therapy_mask_off_regrace_s   = 4.0f;   /* block re-entry after clear */
/* Require therapy pressure collapse before declaring mask-off (blocks breath FPs). */
volatile uint8_t  g_therapy_mask_off_require_press = 1U;
volatile float    g_therapy_mask_off_press_drop_cmh2o = 1.5f; /* vs slewed setpoint */
volatile float    g_therapy_mask_off_press_abs_cmh2o  = 3.0f; /* absolute floor    */
/* During fast-arm window, min |excess| must stay elevated (open hose ≠ breath). */
volatile float    g_therapy_mask_off_enter_min_frac = 0.40f;
/* Inhibit fast enter briefly after a breath phase edge if pressure is OK. */
volatile uint8_t  g_therapy_mask_off_breath_inhibit_enable = 1U;
volatile uint32_t g_therapy_mask_off_breath_inhibit_ms     = 6000U;
/* Vivo-style disconnect: 10 s mean leak ratio > limit (MaskOn_Off_Check.c). */
volatile uint8_t  g_therapy_mask_off_ratio_enable      = 1U;
volatile int32_t  g_therapy_mask_off_ratio_10s_limit   = 2000;
volatile uint32_t g_therapy_mask_off_ratio_enter_ms    = 15000U;
volatile uint8_t  g_therapy_mask_off_enter_via_ratio   = 0U; /* diagnostic     */
volatile uint8_t  g_therapy_mask_off_active      = 0U;
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
static float    s_mask_off_p_baseline  = 0.0f;
static bool     s_mask_off_p_baseline_valid = false;
static float    s_mask_off_q_peak_slm  = 0.0f;
static float    s_mask_off_arm_excess_min = 0.0f;
static uint8_t  s_mask_off_prev_breath_insp = 0U;
static uint32_t s_mask_off_last_breath_edge_ms = 0U;
static uint32_t s_reconnect_grace_until_ms = 0U;
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
    if ((s_reconnect_grace_until_ms != 0U) && (now < s_reconnect_grace_until_ms))
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

static void maskOffApplyHoldRpm(void)
{
    /* Keep CM4 FOC running at fixed RPM for reconnect sensing (never STOP). */
    (void)BlowerIpc_CM7_Start();
    (void)BlowerIpc_CM7_SetSpeedRpmSilent(clampMaskOffRpm(g_therapy_mask_off_rpm));
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

    /* Live breath rate is the strongest "patient present" signal. */
    if (Leak_GetBreathRateBpm() >= 4.0f)
    {
        return true;
    }

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

    /* Instant excess ≈ patient flow on a sealed mask — peaks alone are NOT
     * disconnect. Need sustained excess + pressure collapse. */
    const float inst = fabsf(Leak_GetInstantExcessSlm());
    const float enter = clampf(g_therapy_mask_off_enter_slm, 5.0f, 80.0f);
    const bool excess_ok = (inst >= enter);
    g_therapy_mask_off_diag_excess = excess_ok ? 1U : 0U;

    const bool press_ok = maskOffPressureCollapsed();
    g_therapy_mask_off_diag_press = press_ok ? 1U : 0U;

    const bool breath_block = maskOffBreathInhibitActive() && !press_ok;
    g_therapy_mask_off_diag_breath = breath_block ? 1U : 0U;

    if (!excess_ok)
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

    /* Min-excess gate is evaluated in updateMaskOff while arming. */
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
    if (!s_mask_off_p_baseline_valid)
    {
        s_mask_off_p_baseline = p_sensor_cmh2o;
        s_mask_off_p_baseline_valid = true;
    }
    else if (p_sensor_cmh2o < s_mask_off_p_baseline)
    {
        s_mask_off_p_baseline = p_sensor_cmh2o;
    }

    if (q_total_slm > s_mask_off_q_peak_slm)
    {
        s_mask_off_q_peak_slm = q_total_slm;
    }
}

static bool maskOffClearCondition(void)
{
    const float q_total = Leak_GetTotalSlm();
    const float p_sensor = BlowerCtrl_GetSensorCmh2o();
    const float delta = clampf(g_therapy_mask_off_reconnect_press_delta_cmh2o,
                               0.08f, 6.0f);
    const float abs_p = clampf(g_therapy_mask_off_reconnect_abs_cmh2o, 0.20f, 10.0f);
    const float flow_max = clampf(g_therapy_mask_off_reconnect_flow_max_slm, 15.0f, 120.0f);
    const float flow_frac = clampf(g_therapy_mask_off_flow_drop_frac, 0.30f, 0.90f);

    /* Primary reconnect cue at hold RPM: pressure rises vs open-circuit min. */
    bool press_rise = false;
    if (s_mask_off_p_baseline_valid)
    {
        press_rise = (p_sensor >= (s_mask_off_p_baseline + delta));
    }
    const bool press_abs = (p_sensor >= abs_p);

    /* Flow drop vs open-hose peak (breathing spikes OK — use peak compare). */
    const bool flow_from_peak =
        (s_mask_off_q_peak_slm >= 18.0f)
        && (q_total < (s_mask_off_q_peak_slm * flow_frac));
    const bool flow_vs_cap = (q_total < flow_max);

    g_therapy_mask_off_reconn_press_ok = (press_rise || press_abs) ? 1U : 0U;
    g_therapy_mask_off_reconn_flow_ok  = (flow_from_peak || flow_vs_cap) ? 1U : 0U;

    /* 1) Clear pressure rise from open-circuit baseline — strongest signal. */
    if (press_rise)
    {
        return true;
    }
    /* 2) Absolute pressure with flow drop from open peak. */
    if (press_abs && flow_from_peak)
    {
        return true;
    }
    /* 3) Strong flow collapse with any small pressure tip. */
    if (flow_from_peak && s_mask_off_p_baseline_valid
        && (p_sensor >= (s_mask_off_p_baseline + 0.10f)))
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
    s_mask_off_hold_ms = 0U;
    s_mask_off_active_ms = 0U;
    s_mask_off_clear_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_p_baseline = 0.0f;
    s_mask_off_q_peak_slm = Leak_GetTotalSlm();

    Leak_SetDisconnectFreeze(true);

    DBG_I("THR", "mask off: hold %ld rpm (ratio=%u)",
          (long)clampMaskOffRpm(g_therapy_mask_off_rpm),
          (unsigned)g_therapy_mask_off_enter_via_ratio);

    Epr_ResetBreathState();
    BlowerCtrl_SetEnable(false);
    maskOffApplyHoldRpm();
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
    s_mask_off_fast_enter_ms = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms = 0U;
    s_mask_off_active_ms = 0U;
    s_mask_off_arm_excess_min = 0.0f;
    g_therapy_mask_off_enter_via_ratio = 0U;

    Leak_SetDisconnectFreeze(false);

    if (g_therapy_mask_off_regrace_s > 0.0f)
    {
        s_reconnect_grace_until_ms =
            HAL_GetTick() + (uint32_t)(g_therapy_mask_off_regrace_s * 1000.0f);
    }

    DBG_I("THR", "mask on: resume pressure control -> target %.1f",
          (double)therapyBaseSetpointCmh2o());

    Leak_Reseed();
    Apap_OnMaskReconnect();
    Epr_OnTherapyStart();

    const float base_sp = therapyBaseSetpointCmh2o();
    float resume_sp = base_sp;
    if (Epr_IsEnabled())
    {
        resume_sp = Epr_GetExpiratorySetpointCmh2o(base_sp, g_therapy_state);
        Epr_SetStartupHoldTarget(resume_sp);
    }

    /* Seed FF/PID at therapy RPM, then ramp CM4 off the hold speed. */
    BlowerCtrl_ReseedForTherapyResume(resume_sp);
    (void)BlowerIpc_CM7_Start();
    BlowerCtrl_SetEnable(true);

    {
        int32_t rpm = (int32_t)lrintf(BlowerCtrl_GetOutputRpm());
        if (rpm < 10000)
        {
            rpm = 10000;
        }
        (void)BlowerIpc_CM7_SetSpeedRpm(rpm, 750U);
    }

    s_mask_off_resume_boost_ms = 3000U;
    s_mask_off_resume_repost_ms = 0U;
    g_therapy_mask_off_reconn_flow_ok  = 0U;
    g_therapy_mask_off_reconn_press_ok = 0U;
}

/* Keep commanding therapy pressure for a few seconds after mask-on so RPM
 * cannot linger at the hold speed if the first IPC set was lost. */
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

    applyTherapySetpoint(therapyBaseSetpointCmh2o(), dt_s);

    s_mask_off_resume_repost_ms += dt_ms;
    if (s_mask_off_resume_repost_ms >= 250U)
    {
        s_mask_off_resume_repost_ms = 0U;
        int32_t rpm = (int32_t)lrintf(BlowerCtrl_GetOutputRpm());
        if (rpm < 8000)
        {
            rpm = 8000;
        }
        (void)BlowerIpc_CM7_SetSpeedRpmSilent(rpm);
    }
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
        const float inst = fabsf(Leak_GetInstantExcessSlm());
        const float enter = clampf(g_therapy_mask_off_enter_slm, 5.0f, 80.0f);
        const float min_frac = clampf(g_therapy_mask_off_enter_min_frac, 0.15f, 0.90f);

        if (fast_cond)
        {
            if (s_mask_off_fast_enter_ms == 0U)
            {
                s_mask_off_arm_excess_min = inst;
            }
            else if (inst < s_mask_off_arm_excess_min)
            {
                s_mask_off_arm_excess_min = inst;
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

        const uint32_t fast_need = (g_therapy_mask_off_enter_ms > 0U)
                                     ? g_therapy_mask_off_enter_ms : 1U;
        const uint32_t ratio_need = (g_therapy_mask_off_ratio_enter_ms > 0U)
                                      ? g_therapy_mask_off_ratio_enter_ms : 15000U;

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

    /* Hold low RPM; re-post periodically in case IPC was busy. */
    s_mask_off_hold_ms += dt_ms;
    s_mask_off_active_ms += dt_ms;
    maskOffTrackOpenCircuit(BlowerCtrl_GetSensorCmh2o(), Leak_GetTotalSlm());
    if (s_mask_off_hold_ms >= 200U)
    {
        s_mask_off_hold_ms = 0U;
        maskOffApplyHoldRpm();
    }

    const uint32_t min_dwell = (g_therapy_mask_off_min_dwell_ms > 0U)
                                 ? g_therapy_mask_off_min_dwell_ms : 1U;
    const uint32_t learn_ms  = (g_therapy_mask_off_baseline_ms > 0U)
                                 ? g_therapy_mask_off_baseline_ms : 1U;
    if (s_mask_off_active_ms < min_dwell)
    {
        s_mask_off_clear_ms = 0U;
        return true;
    }
    if (s_mask_off_active_ms < learn_ms)
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

    g_therapy_fault           = (uint8_t)THERAPY_FAULT_NONE;
    s_mask_off_active         = false;
    g_therapy_mask_off_active = 0U;
    s_mask_off_fast_enter_ms  = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms       = 0U;
    s_mask_off_hold_ms        = 0U;
    s_mask_off_active_ms      = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_q_peak_slm = 0.0f;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_prev_breath_insp = 0U;
    s_mask_off_last_breath_edge_ms = 0U;
    s_mask_off_resume_boost_ms = 0U;
    s_mask_off_resume_repost_ms = 0U;
    s_reconnect_grace_until_ms = 0U;

    Safety_Supervisor_Init();

    Epr_Init();
    Apap_Init();
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
uint32_t Therapy_GetRunSeconds(void)    { return g_therapy_run_ms / 1000U; }

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
    Safety_Supervisor_ResetFaultTimers();
    s_therapy_start_ms       = HAL_GetTick();
    s_reconnect_grace_until_ms = 0U;
    s_mask_off_active        = false;
    g_therapy_mask_off_active = 0U;
    s_mask_off_fast_enter_ms  = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms      = 0U;
    s_mask_off_hold_ms       = 0U;
    s_mask_off_active_ms     = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_q_peak_slm = 0.0f;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_prev_breath_insp = 0U;
    s_mask_off_last_breath_edge_ms = 0U;
    s_mask_off_resume_boost_ms = 0U;
    s_mask_off_resume_repost_ms = 0U;

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

    /* Tell CM4 to spin the motor up. The PID's first SET_SPEED command
     * arrives within 5 ms and supersedes whatever rpm CM4 chose at
     * START. */
    (void)BlowerIpc_CM7_Start();

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

    /* Drop PID control immediately so the motor coasts down rather than
     * fighting to maintain pressure during spin-down. */
    if (s_mask_off_active)
    {
        s_mask_off_active = false;
        g_therapy_mask_off_active = 0U;
        Leak_SetDisconnectFreeze(false);
    }
    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();

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

    DBG_I("THR", "stop after %lu s", (unsigned long)Therapy_GetRunSeconds());
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
