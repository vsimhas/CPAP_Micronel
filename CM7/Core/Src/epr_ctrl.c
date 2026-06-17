/**
  ******************************************************************************
  * @file    epr_ctrl.c
  * @brief   EPR breath detection + setpoint relief.
  ******************************************************************************
  */

#include "epr_ctrl.h"

#include <math.h>

/* === Tunables (debugger-writable volatiles) === */
volatile uint8_t g_epr_enabled                 = 0U;
volatile float   g_epr_relief_cmh2o            = 3.0f;
volatile float   g_epr_min_pressure_cmh2o      = 4.0f;

/* Relief apply (exhale drop) is slewed for comfort; relief restore (inhale)
 * is instant so therapy pressure returns as soon as exhale ends. */
volatile float   g_epr_relief_apply_slew_cmh2o_per_s = 12.0f;

/* Breath detection thresholds (absolute minimums in slm). Adaptive logic
 * also scales thresholds as a fraction of the running flow baseline so a
 * high-leak bench rig and a masked patient both work without retuning. */
volatile float   g_epr_insp_thresh_slm         = 4.0f;
volatile float   g_epr_exp_enter_thresh_slm    = 3.0f;   /* enter exhale    */
volatile float   g_epr_exp_end_hyst_slm        = 0.5f;   /* end exhale      */

/* Fraction of flow baseline added to the absolute thresholds above.
 * With patient flow (~zero-mean) baseline is small; keep fractions low. */
volatile float   g_epr_insp_thresh_frac        = 0.02f;
volatile float   g_epr_exp_thresh_frac         = 0.02f;

volatile float   g_epr_flow_baseline_tau_s     = 2.5f;
volatile float   g_epr_baseline_track_deadband_slm = 5.0f;

volatile uint32_t g_epr_insp_confirm_ms        = 150U;
volatile uint32_t g_epr_exp_confirm_ms         = 200U;
volatile float   g_epr_min_insp_peak_slm       = 4.0f;

/* === Live state === */
volatile uint8_t g_epr_phase                   = (uint8_t)EPR_PHASE_UNKNOWN;
volatile float   g_epr_active_relief_cmh2o     = 0.0f;
volatile float   g_epr_flow_baseline_slm       = 0.0f;
volatile uint32_t g_epr_step_count             = 0U;
volatile uint8_t  g_epr_insp_armed             = 0U;
volatile float    g_epr_insp_peak_delta_slm    = 0.0f;
volatile float    g_epr_flow_delta_slm         = 0.0f;  /* Q - baseline    */
volatile uint8_t  g_epr_breath_fsm             = 0U;    /* BreathFsm_t     */
volatile int8_t   g_epr_insp_polarity          = 0;     /* +1/-1 auto      */

/* === Internal breath FSM === */
typedef enum
{
    BREATH_IDLE = 0,
    BREATH_INSP_PENDING,
    BREATH_INSPIRATION,
    BREATH_EXP_PENDING,
    BREATH_EXPIRATION
} BreathFsm_t;

static BreathFsm_t s_breath_fsm = BREATH_IDLE;
static uint32_t    s_phase_timer_ms = 0U;
static float       s_cycle_insp_peak = 0.0f;
static bool        s_baseline_seeded = false;
static int8_t      s_insp_polarity = 0;   /* +1: inhale = flow up; -1: inhale = flow down */

static float effectiveInspThresh(void)
{
    const float base = (fabsf(g_epr_flow_baseline_slm) > 2.0f)
                         ? fabsf(g_epr_flow_baseline_slm) : 2.0f;
    return g_epr_insp_thresh_slm + base * g_epr_insp_thresh_frac;
}

static float effectiveExpEnterThresh(void)
{
    const float base = (fabsf(g_epr_flow_baseline_slm) > 2.0f)
                         ? fabsf(g_epr_flow_baseline_slm) : 2.0f;
    return g_epr_exp_enter_thresh_slm + base * g_epr_exp_thresh_frac;
}

static void syncFsmDebug(void)
{
    g_epr_breath_fsm = (uint8_t)s_breath_fsm;
    g_epr_insp_polarity = s_insp_polarity;
}

/* Map flow delta into "inspiration-positive" coordinates. Auto-detects
 * polarity on the first strong breath (handles bench test lungs where
 * inhale may appear as a flow drop instead of a rise). */
static float directedDelta(float delta)
{
    if (s_insp_polarity < 0)
    {
        return -delta;
    }
    return delta;
}

static void lockPolarityFromDelta(float delta, float th)
{
    if (s_insp_polarity != 0)
    {
        return;
    }
    if (delta > th)
    {
        s_insp_polarity = 1;
    }
    else if (delta < -th)
    {
        s_insp_polarity = -1;
    }
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void slewToward(float *val, float target, float max_step)
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

/* === Public API === */

void Epr_Init(void)
{
    g_epr_enabled             = 0U;
    g_epr_relief_cmh2o        = 3.0f;
    g_epr_min_pressure_cmh2o  = 4.0f;
    g_epr_relief_apply_slew_cmh2o_per_s = 12.0f;
    g_epr_flow_baseline_tau_s = 2.5f;
    g_epr_insp_thresh_slm     = 4.0f;
    g_epr_exp_enter_thresh_slm = 3.0f;
    g_epr_exp_end_hyst_slm    = 0.5f;
    g_epr_insp_thresh_frac    = 0.02f;
    g_epr_exp_thresh_frac     = 0.02f;
    g_epr_baseline_track_deadband_slm = 5.0f;
    g_epr_insp_confirm_ms     = 150U;
    g_epr_exp_confirm_ms      = 200U;
    g_epr_min_insp_peak_slm   = 4.0f;

    g_epr_phase               = (uint8_t)EPR_PHASE_UNKNOWN;
    g_epr_active_relief_cmh2o = 0.0f;
    g_epr_flow_baseline_slm   = 0.0f;
    g_epr_step_count          = 0U;
    g_epr_insp_armed          = 0U;
    g_epr_insp_peak_delta_slm = 0.0f;
    g_epr_flow_delta_slm      = 0.0f;
    g_epr_breath_fsm          = 0U;
    g_epr_insp_polarity      = 0;
    s_baseline_seeded         = false;
    s_breath_fsm              = BREATH_IDLE;
    s_phase_timer_ms          = 0U;
    s_cycle_insp_peak         = 0.0f;
    s_insp_polarity           = 0;
}

void  Epr_SetEnable(bool en)              { g_epr_enabled = en ? 1U : 0U; }
bool  Epr_IsEnabled(void)                 { return (g_epr_enabled != 0U); }

void Epr_SetReliefCmh2o(float cmh2o)
{
    g_epr_relief_cmh2o = clampf(cmh2o, 1.0f, 3.0f);
}

float Epr_GetReliefCmh2o(void)            { return g_epr_relief_cmh2o; }

void Epr_SetMinPressureCmh2o(float cmh2o)
{
    g_epr_min_pressure_cmh2o = clampf(cmh2o, 4.0f, 10.0f);
}

float Epr_GetMinPressureCmh2o(void)       { return g_epr_min_pressure_cmh2o; }

EprPhase_t Epr_GetPhase(void)             { return (EprPhase_t)g_epr_phase; }

const char* Epr_GetPhaseName(void)
{
    switch ((EprPhase_t)g_epr_phase)
    {
    case EPR_PHASE_INSPIRATION:  return "Inhale";
    case EPR_PHASE_EXPIRATION:   return "Exhale";
    case EPR_PHASE_UNKNOWN:
    default:                     return "---";
    }
}

float Epr_GetActiveReliefCmh2o(void)      { return g_epr_active_relief_cmh2o; }
float Epr_GetFlowBaselineSlm(void)        { return g_epr_flow_baseline_slm; }

void Epr_ResetBreathState(void)
{
    g_epr_phase               = (uint8_t)EPR_PHASE_UNKNOWN;
    g_epr_active_relief_cmh2o = 0.0f;
    g_epr_flow_baseline_slm   = 0.0f;
    g_epr_insp_armed          = 0U;
    g_epr_insp_peak_delta_slm = 0.0f;
    g_epr_flow_delta_slm      = 0.0f;
    g_epr_breath_fsm          = 0U;
    g_epr_insp_polarity      = 0;
    s_baseline_seeded         = false;
    s_breath_fsm              = BREATH_IDLE;
    s_phase_timer_ms          = 0U;
    s_cycle_insp_peak         = 0.0f;
    s_insp_polarity           = 0;
}

static void mapPhaseForGui(BreathFsm_t fsm)
{
    switch (fsm)
    {
    case BREATH_INSP_PENDING:
    case BREATH_INSPIRATION:
        g_epr_phase = (uint8_t)EPR_PHASE_INSPIRATION;
        break;
    case BREATH_EXP_PENDING:
    case BREATH_EXPIRATION:
        g_epr_phase = (uint8_t)EPR_PHASE_EXPIRATION;
        break;
    case BREATH_IDLE:
    default:
        g_epr_phase = (uint8_t)EPR_PHASE_UNKNOWN;
        break;
    }
}

static bool eprReliefAllowed(void)
{
    /* EPR only after a full inhale -> confirmed exhale sequence. */
    return (s_breath_fsm == BREATH_EXPIRATION) && (g_epr_insp_armed != 0U);
}

void Epr_Update(float q_slm, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    /* Slow baseline tracks mid-breath flow. Only update when flow is near
     * the current baseline so inspiration/exhalation excursions do not drag
     * the reference with them (major noise-immunity improvement). */
    const float tau = clampf(g_epr_flow_baseline_tau_s, 0.5f, 5.0f);
    const float alpha = dt_s / (tau + dt_s);
    const float deadband = clampf(g_epr_baseline_track_deadband_slm, 2.0f, 20.0f);

    if (!s_baseline_seeded)
    {
        g_epr_flow_baseline_slm = q_slm;
        s_baseline_seeded = true;
    }
    else
    {
        const float delta_for_base = q_slm - g_epr_flow_baseline_slm;
        if (fabsf(delta_for_base) < deadband)
        {
            g_epr_flow_baseline_slm = alpha * q_slm
                                    + (1.0f - alpha) * g_epr_flow_baseline_slm;
        }
    }

    const float delta = q_slm - g_epr_flow_baseline_slm;
    g_epr_flow_delta_slm = delta;

    const float insp_th   = effectiveInspThresh();
    const float exp_enter = effectiveExpEnterThresh();
    const float exp_end   = clampf(g_epr_exp_end_hyst_slm, 0.5f, 10.0f);
    const float min_peak  = clampf(g_epr_min_insp_peak_slm, 2.0f, 30.0f);
    const uint32_t insp_confirm = (g_epr_insp_confirm_ms > 0U)
                                    ? g_epr_insp_confirm_ms : 1U;
    const uint32_t exp_confirm  = (g_epr_exp_confirm_ms > 0U)
                                    ? g_epr_exp_confirm_ms : 1U;

    const float dir = directedDelta(delta);

    /* --- Breath FSM: confirm inhale, then accept moderate exhale dip --- */
    switch (s_breath_fsm)
    {
    case BREATH_IDLE:
        g_epr_insp_armed = 0U;
        s_cycle_insp_peak = 0.0f;
        if ((delta > insp_th) || (delta < -insp_th))
        {
            lockPolarityFromDelta(delta, insp_th);
            if (((s_insp_polarity > 0) && (delta > insp_th)) ||
                ((s_insp_polarity < 0) && (delta < -insp_th)))
            {
                s_breath_fsm = BREATH_INSP_PENDING;
                s_phase_timer_ms = 0U;
                s_cycle_insp_peak = fabsf(dir);
            }
        }
        break;

    case BREATH_INSP_PENDING:
        if (dir > insp_th)
        {
            s_phase_timer_ms += dt_ms;
            if (dir > s_cycle_insp_peak)
            {
                s_cycle_insp_peak = dir;
            }
            if (s_phase_timer_ms >= insp_confirm)
            {
                s_breath_fsm = BREATH_INSPIRATION;
                g_epr_insp_armed = 1U;
                g_epr_insp_peak_delta_slm = s_cycle_insp_peak;
                s_phase_timer_ms = 0U;
            }
        }
        else
        {
            s_breath_fsm = BREATH_IDLE;
            s_phase_timer_ms = 0U;
            s_cycle_insp_peak = 0.0f;
            s_insp_polarity = 0;
        }
        break;

    case BREATH_INSPIRATION:
        if (dir > s_cycle_insp_peak)
        {
            s_cycle_insp_peak = dir;
            g_epr_insp_peak_delta_slm = s_cycle_insp_peak;
        }

        if ((dir < -exp_enter) && (s_cycle_insp_peak >= min_peak))
        {
            s_breath_fsm = BREATH_EXP_PENDING;
            s_phase_timer_ms = 0U;
        }
        break;

    case BREATH_EXP_PENDING:
        if ((dir < -exp_enter) && (g_epr_insp_armed != 0U))
        {
            s_phase_timer_ms += dt_ms;
            if (s_phase_timer_ms >= exp_confirm)
            {
                s_breath_fsm = BREATH_EXPIRATION;
                s_phase_timer_ms = 0U;
            }
        }
        else if (dir > -exp_end)
        {
            s_breath_fsm = BREATH_INSPIRATION;
            s_phase_timer_ms = 0U;
        }
        break;

    case BREATH_EXPIRATION:
        /* Inhalation onset: end EPR immediately (pressure restores in
         * Epr_ModifySetpoint). */
        if (dir > exp_enter)
        {
            s_breath_fsm = BREATH_INSP_PENDING;
            s_phase_timer_ms = 0U;
            s_cycle_insp_peak = dir;
            g_epr_insp_armed = 0U;
        }
        else if (dir > -exp_end)
        {
            /* Expiratory flow fell — restore pressure before inhale. */
            s_breath_fsm = BREATH_IDLE;
            g_epr_insp_armed = 0U;
            s_phase_timer_ms = 0U;
            s_cycle_insp_peak = 0.0f;
            s_insp_polarity = 0;
        }
        break;

    default:
        s_breath_fsm = BREATH_IDLE;
        break;
    }

    mapPhaseForGui(s_breath_fsm);
    syncFsmDebug();
    g_epr_step_count++;
}

float Epr_ModifySetpoint(float base_setpoint_cmh2o, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return base_setpoint_cmh2o;
    }

    float target_relief = 0.0f;
    if ((g_epr_enabled != 0U) && eprReliefAllowed())
    {
        target_relief = g_epr_relief_cmh2o;
    }

    if (target_relief <= 0.0f)
    {
        /* Instant restore at end of exhale / start of inhale. */
        g_epr_active_relief_cmh2o = 0.0f;
    }
    else
    {
        const float slew = clampf(g_epr_relief_apply_slew_cmh2o_per_s, 1.0f, 50.0f);
        const float max_step = slew * dt_s;
        slewToward((float *)&g_epr_active_relief_cmh2o, target_relief, max_step);
    }

    float sp = base_setpoint_cmh2o - g_epr_active_relief_cmh2o;
    if (sp < g_epr_min_pressure_cmh2o)
    {
        sp = g_epr_min_pressure_cmh2o;
    }
    return sp;
}
