/**
  ******************************************************************************
  * @file    apap_ctrl.c
  * @brief   Autoset (APAP) titration state machine.
  *
  *  Functional-equivalent prototype: FOT OSA/CA, FL, hypopnea, snore,
  *  leak-gated raises, down-titration, session AHI/RDI. All thresholds are
  *  debugger-tunable heuristics — not ResMed constants; clinical validation
  *  required before therapeutic claims.
  ******************************************************************************
  */

#include "apap_ctrl.h"
#include "therapy.h"
#include "epr_ctrl.h"
#include "blower_osc.h"
#include "blower_ctrl.h"
#include "flow_sensor.h"
#include "leak_estimator.h"
#include "stm32h7xx_hal.h"

#include <math.h>

/* === Tunables (debugger-writable) === */
/* Apnea timing - 2026-10-12, BreathSimulator AutoSet trials
 * (docs/APAP_FINDINGS_2026-10-12_breathsim.md, finding 1). MONITOR only
 * leaves for NO_BREATH once Epr_HasRecentBreath() has seen nothing for
 * breath_window_s; FOT starts no_breath_timeout_s after that; an event is
 * scored once s_episode_ms >= apnea_min_s. The episode clock used to start
 * at zero on NO_BREATH entry, so window + apnea_min = 12 + 10 = ~22 s with
 * no breath were needed and 10-20 s apneas were never probed or scored
 * (bench: FOT 14-17 s into the apnea; 20 s apneas scored 3/4). The clock
 * now starts at the window (that much breathless time has already passed),
 * and the defaults are 12 -> 8 s and 4 -> 2 s, so FOT starts ~10 s after
 * the last qualified breath (stamped at end of inspiration, i.e. ~7-8 s
 * into an apnea at I:E 1:1) and the first 2 s evaluation can score a
 * ~10 s apnea. The window doubles as the slow-breathing guard: breath
 * periods up to ~10 s (6 bpm) still never start FOT; slower breathing gets
 * a brief probe that the next qualified breath ends. Not yet bench-verified
 * with the new defaults - re-run the 10-25 s apnea-length sweep. */
volatile float   g_apap_no_breath_timeout_s        = 2.0f;  /* FOT start, after the window */
volatile float   g_apap_breath_window_s            = 8.0f;
/* Exit from NO_BREATH / FOT. Finding 3: a FIXED 10 slm made the outcome
 * depend on patient size - with ~20 slm normal peaks, a 60 % hypopnea
 * (peaks ~8 slm) kept FOT running and was scored as a central apnea (2/2).
 * The threshold is now resume_peak_ratio x the peak baseline the FL/Hyp
 * detector held when the episode began, never below resume_min_floor_slm
 * (the EPR breath qualifier is 4 slm). resume_min_peak_slm is only the
 * fallback when no baseline was established yet (start of therapy). */
volatile float   g_apap_resume_min_peak_slm        = 10.0f; /* fallback, no baseline */
volatile float   g_apap_resume_peak_ratio          = 0.30f; /* x baseline; 0 = fixed 10 */
volatile float   g_apap_resume_min_floor_slm       = 4.0f;
volatile float   g_apap_startup_grace_s            = 15.0f;
/* FOT is the most power-hungry state: on the bench (2026-10-12) supply
 * current peaked 1.05-1.12 A only while oscillating, vs <= 0.78 A for
 * breathing (finding 6). Brief peaks, but size the supply for them. */
volatile float   g_apap_osc_amplitude_cmh2o        = 1.0f;   /* setpoint peak; ~2.0 p-p */
volatile float   g_apap_osc_amplitude_max_cmh2o    = 1.75f; /* setpoint peak clamp */
volatile float   g_apap_osc_freq_hz                = 4.0f;
volatile float   g_apap_fot_eval_period_s          = 2.0f;   /* ~8 cycles @ 4 Hz */
volatile float   g_apap_osc_min_p_pp_cmh2o         = 0.15f;
volatile float   g_apap_osa_flow_pp_thresh_slm     = 2.0f;
volatile float   g_apap_apnea_min_s                = 10.0f; /* score/raise only after */
volatile float   g_apap_oa_raise_cmh2o_per_s       = 0.08f; /* independent duration heuristic */
volatile float   g_apap_oa_raise_max_cmh2o         = 2.0f;
volatile float   g_apap_her_oa_above_cmh2o         = 12.0f; /* For Her: milder OA above this */
volatile float   g_apap_her_oa_above_scale         = 0.50f;
volatile float   g_apap_raise_rate_standard_cmh2o_per_s = 0.050f; /* ~3/min */
volatile float   g_apap_raise_rate_soft_cmh2o_per_s     = 0.020f; /* ~1.2/min */
volatile float   g_apap_raise_rate_her_scale     = 0.75f;
volatile uint8_t g_apap_response_soft              = 0U;

/* Flow-limitation (MONITOR). */
volatile uint8_t g_apap_fl_enable                  = 1U;
volatile float   g_apap_fl_ratio                   = 0.70f;  /* peak < ratio*baseline */
volatile uint8_t g_apap_fl_breaths_needed          = 3U;
volatile uint8_t g_apap_fl_clear_breaths           = 2U;
volatile float   g_apap_fl_raise_scale             = 0.50f;
volatile float   g_apap_fl_min_peak_slm            = 4.0f;
volatile float   g_apap_fl_baseline_alpha          = 0.25f;
volatile float   g_apap_fl_flat_ratio              = 0.78f; /* mean/peak plateau */
/* Finding 4: the baseline only updates on non-limited breaths, so one or
 * two large arousal breaths (1.5x) lifted it until normal breaths read as
 * "peak < 0.70 x baseline", could no longer update it, and held a false FL
 * call for ~90 s (with arousals off: no FL call). Breaths above this
 * multiple of the baseline still count as non-limited but are kept out of
 * the baseline. 0 disables the rejection. */
volatile float   g_apap_fl_baseline_reject_ratio   = 1.40f;

/* Hypopnea (MONITOR): peak drop lasting ~>=10 s. */
volatile uint8_t g_apap_hyp_enable                 = 1U;
volatile float   g_apap_hyp_ratio                  = 0.50f;  /* peak < 50% baseline */
volatile uint8_t g_apap_hyp_breaths_needed         = 2U;
volatile float   g_apap_hyp_min_duration_s         = 10.0f;
volatile float   g_apap_hyp_raise_scale            = 0.70f;
volatile float   g_apap_hyp_hold_s                 = 20.0f;  /* raise window after detect */

/* Snore: HF residual RMS on patient raw vs breath-band flow. */
volatile uint8_t g_apap_snore_enable               = 1U;
volatile float   g_apap_snore_rms_thresh_slm       = 0.35f;
volatile float   g_apap_snore_on_s                 = 1.5f;
volatile float   g_apap_snore_off_s                = 2.0f;
volatile float   g_apap_snore_raise_scale          = 0.35f;
volatile float   g_apap_snore_hp_hz                = 20.0f;  /* residual HP corner */

/* Down-titration + leak gate. */
volatile uint8_t g_apap_lower_enable               = 1U;
volatile float   g_apap_lower_rate_cmh2o_per_s     = 0.008f; /* ~0.5/min */
volatile float   g_apap_stable_before_lower_s      = 120.0f;
volatile uint8_t g_apap_leak_gate_enable           = 1U;     /* block raises on HIGH */

/* === Live state === */
volatile uint8_t  g_apap_state                     = (uint8_t)APAP_STATE_IDLE;
volatile float    g_apap_target_cmh2o              = 8.0f;
volatile uint32_t g_apap_no_breath_ms              = 0U;
volatile uint32_t g_apap_osc_eval_ms               = 0U;
volatile float    g_apap_osc_p_min_cmh2o           = 0.0f;
volatile float    g_apap_osc_p_max_cmh2o           = 0.0f;
volatile float    g_apap_osc_q_min_slm             = 0.0f;
volatile float    g_apap_osc_q_max_slm             = 0.0f;
volatile uint8_t  g_apap_last_osa_detected         = 0U;
volatile uint8_t  g_apap_osa_treating              = 0U;
volatile float    g_apap_fot_p_pp_cmh2o           = 0.0f;
volatile float    g_apap_fot_q_pp_slm             = 0.0f;
volatile float    g_apap_fot_z_cmh2o_per_slm      = 0.0f;
volatile uint8_t  g_apap_fot_osa_likely           = 0U;
volatile uint8_t  g_apap_fl_active                = 0U;
volatile float    g_apap_fl_peak_baseline_slm     = 0.0f;
volatile float    g_apap_fl_last_peak_slm         = 0.0f;
volatile float    g_apap_fl_last_flat             = 0.0f; /* insp mean/peak */
volatile uint8_t  g_apap_hyp_active               = 0U;
volatile uint8_t  g_apap_snore_active             = 0U;
volatile float    g_apap_snore_rms_slm            = 0.0f;
volatile uint32_t g_apap_stable_ms                = 0U;
volatile uint8_t  g_apap_lowering                 = 0U;
volatile uint8_t  g_apap_raise_blocked_leak       = 0U;

volatile uint32_t g_apap_evt_oa                   = 0U;
volatile uint32_t g_apap_evt_ca                   = 0U;
volatile uint32_t g_apap_evt_hyp                  = 0U;
volatile uint32_t g_apap_evt_fl                   = 0U;
volatile uint32_t g_apap_evt_snore                = 0U;
volatile uint32_t g_apap_therapy_ms               = 0U;

#define FOT_Q_PP_EPS_SLM  0.15f
#define FOT_Z_BLOCKED     999.0f

/* === Internal === */
static float     s_min_cmh2o = 5.0f;
static float     s_max_cmh2o = 20.0f;
static bool      s_for_her = false;
static bool      s_active = false;
static bool      s_osa_treating = false;
static FlowFotBp_t s_fot_bp_p;
static FlowFotBp_t s_fot_bp_q;
static float       s_fot_p_bp = 0.0f;
static float       s_fot_q_bp = 0.0f;
static bool       s_osa_event_armed = true;  /* count once per apnea episode */
static bool       s_ca_event_armed = true;
static uint32_t   s_running_enter_ms = 0U;
static uint32_t   s_episode_ms = 0U;
static float      s_oa_raise_applied = 0.0f;

/* FL / hypopnea share inspiratory peak tracking. */
static EprPhase_t s_fl_prev_phase = EPR_PHASE_UNKNOWN;
static float      s_fl_insp_peak_q = 0.0f;
static float      s_fl_baseline_slm = 0.0f;
static uint8_t    s_fl_streak = 0U;
static uint8_t    s_fl_good_streak = 0U;
static bool       s_fl_active = false;
static bool       s_fl_was_active = false;

static uint8_t    s_hyp_streak = 0U;
static uint32_t   s_hyp_limited_ms = 0U;
static bool       s_hyp_active = false;
static uint32_t   s_hyp_hold_ms = 0U;

/* Snore detector. */
static float      s_snore_prev_q = 0.0f;
static float      s_snore_hp = 0.0f;
static float      s_snore_rms2 = 0.0f;
static uint32_t   s_snore_on_ms = 0U;
static uint32_t   s_snore_off_ms = 0U;
static bool       s_snore_active = false;
static bool       s_snore_was_active = false;
static uint32_t   s_insp_ms = 0U;
static float      s_fl_insp_sum_q = 0.0f;
static uint32_t   s_fl_insp_n = 0U;
static float      s_fl_last_flat = 0.0f;
static uint32_t   s_since_insp_end_ms = 0U;   /* breath period for Hyp duration */
static float      s_resume_ref_slm = 0.0f;    /* baseline held at episode start */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool autosetMode(uint8_t mode)
{
    return (mode == (uint8_t)THERAPY_MODE_AUTOSET)
        || (mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
}

static float raiseRate(void)
{
    float rate = (g_apap_response_soft != 0U)
                     ? g_apap_raise_rate_soft_cmh2o_per_s
                     : g_apap_raise_rate_standard_cmh2o_per_s;
    if (s_for_her)
    {
        rate *= clampf(g_apap_raise_rate_her_scale, 0.25f, 1.0f);
    }
    return clampf(rate, 0.005f, 0.2f);
}

static void setState(ApapState_t st)
{
    g_apap_state = (uint8_t)st;
}

static void disableOsc(void)
{
    BlowerOsc_SetEnable(false);
}

static void configureOsc(void)
{
    /* Commanded on the PID setpoint. Mask p-p is often ~30-50% of this
     * (tube/mask compliance, leak). Tune g_apap_osc_amplitude_cmh2o. */
    const float ampl_max = clampf(g_apap_osc_amplitude_max_cmh2o, 0.5f, 2.5f);
    BlowerOsc_SetAmplitude(clampf(g_apap_osc_amplitude_cmh2o, 0.05f, ampl_max));
    BlowerOsc_SetFrequency(clampf(g_apap_osc_freq_hz, 2.0f, 8.0f));
    BlowerOsc_SetEnable(true);
}

static void resetOscStats(void)
{
    /* Restart p-p window on the last bandpassed samples. Do not reset
     * the biquads here — that would inject a transient every eval. */
    g_apap_osc_p_min_cmh2o = s_fot_p_bp;
    g_apap_osc_p_max_cmh2o = s_fot_p_bp;
    g_apap_osc_q_min_slm = s_fot_q_bp;
    g_apap_osc_q_max_slm = s_fot_q_bp;
    g_apap_osc_eval_ms = 0U;
}

static void trackOscStats(float dt_s)
{
    const float f0 = clampf(g_apap_osc_freq_hz, 2.0f, 8.0f);
    s_fot_p_bp = Flow_FotBpStep(&s_fot_bp_p, BlowerCtrl_GetMeasured(), f0, dt_s);
    s_fot_q_bp = Flow_FotBpStep(&s_fot_bp_q, Flow_GetPatientRawSlm(), f0, dt_s);
    const float p = s_fot_p_bp;
    const float q = s_fot_q_bp;
    if (p < g_apap_osc_p_min_cmh2o) { g_apap_osc_p_min_cmh2o = p; }
    if (p > g_apap_osc_p_max_cmh2o) { g_apap_osc_p_max_cmh2o = p; }
    if (q < g_apap_osc_q_min_slm)   { g_apap_osc_q_min_slm = q; }
    if (q > g_apap_osc_q_max_slm)   { g_apap_osc_q_max_slm = q; }
}

static bool startupGraceActive(void)
{
    if (g_apap_startup_grace_s <= 0.0f)
    {
        return false;
    }
    const uint32_t grace_ms = (uint32_t)(g_apap_startup_grace_s * 1000.0f);
    return (HAL_GetTick() - s_running_enter_ms) < grace_ms;
}

static bool recentBreath(void)
{
    const uint32_t window_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                                  4.0f, 30.0f) * 1000.0f);
    return Epr_HasRecentBreath(window_ms);
}

/* Leave FOT / no-breath only on a recovery-sized inspiration, not an
 * obstructed gasp. MONITOR still uses recentBreath() so hypopnea can run. */
static bool recoveryBreath(void)
{
    if (!recentBreath())
    {
        return false;
    }
    float need = clampf(g_apap_resume_min_peak_slm, 4.0f, 40.0f);
    const float ratio = clampf(g_apap_resume_peak_ratio, 0.0f, 0.8f);
    if ((ratio > 0.0f) && (s_resume_ref_slm > 0.0f))
    {
        const float floor_slm = clampf(g_apap_resume_min_floor_slm, 2.0f, 20.0f);
        need = ratio * s_resume_ref_slm;
        if (need < floor_slm) { need = floor_slm; }
    }
    return Epr_GetLastInspPeakSlm() >= need;
}

static bool raiseAllowed(void)
{
    if (g_apap_leak_gate_enable == 0U)
    {
        g_apap_raise_blocked_leak = 0U;
        return true;
    }
    const bool blocked = (Leak_GetSeverity() == LEAK_SEV_HIGH);
    g_apap_raise_blocked_leak = blocked ? 1U : 0U;
    return !blocked;
}

static void applyRaise(float scale, float dt_s)
{
    if (!raiseAllowed())
    {
        return;
    }
    g_apap_target_cmh2o += raiseRate() * clampf(scale, 0.05f, 1.0f) * dt_s;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
}

static bool applyStepRaise(float delta_cmh2o)
{
    if (!(delta_cmh2o > 0.0f) || !raiseAllowed())
    {
        return false;
    }
    g_apap_target_cmh2o += delta_cmh2o;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    return true;
}

/* OA raise after scored apnea (~10 s): amount tracks episode length, not a
 * continuous FOT ramp. Central (s_osa_treating false) does not raise. */
static void applyOaDurationRaise(void)
{
    if (!s_osa_treating)
    {
        return;
    }

    const float apnea_min_s = clampf(g_apap_apnea_min_s, 6.0f, 30.0f);
    const float episode_s = (float)s_episode_ms / 1000.0f;
    if (episode_s < apnea_min_s)
    {
        return;
    }

    const float per_s = clampf(g_apap_oa_raise_cmh2o_per_s, 0.01f, 0.50f);
    const float cap = clampf(g_apap_oa_raise_max_cmh2o, 0.2f, 6.0f);
    float want = (episode_s - apnea_min_s) * per_s;
    if (s_for_her
        && (g_apap_target_cmh2o
            >= clampf(g_apap_her_oa_above_cmh2o, 8.0f, 18.0f)))
    {
        want *= clampf(g_apap_her_oa_above_scale, 0.10f, 1.0f);
    }
    want = clampf(want, 0.0f, cap);

    const float delta = want - s_oa_raise_applied;
    if (delta > 0.02f && applyStepRaise(delta))
    {
        s_oa_raise_applied += delta;
    }
}

static float sessionHours(void)
{
    if (g_apap_therapy_ms < 1000U)
    {
        return 0.0f;
    }
    return (float)g_apap_therapy_ms / 3600000.0f;
}

static void refreshFotMetrics(void)
{
    const float p_pp = g_apap_osc_p_max_cmh2o - g_apap_osc_p_min_cmh2o;
    const float q_pp = g_apap_osc_q_max_slm - g_apap_osc_q_min_slm;
    const float p_min = clampf(g_apap_osc_min_p_pp_cmh2o, 0.05f, 0.8f);
    const float q_thr = clampf(g_apap_osa_flow_pp_thresh_slm, 0.5f, 10.0f);

    g_apap_fot_p_pp_cmh2o = p_pp;
    g_apap_fot_q_pp_slm = q_pp;
    if (q_pp > FOT_Q_PP_EPS_SLM)
    {
        g_apap_fot_z_cmh2o_per_slm = p_pp / q_pp;
    }
    else
    {
        g_apap_fot_z_cmh2o_per_slm = FOT_Z_BLOCKED;
    }

    const bool osa = (p_pp >= p_min) && (q_pp < q_thr);
    g_apap_fot_osa_likely = osa ? 1U : 0U;
}

static bool evaluateOscForOsa(void)
{
    refreshFotMetrics();
    return (g_apap_fot_osa_likely != 0U);
}

static void enterFotProbe(void)
{
    Flow_FotBpReset(&s_fot_bp_p);
    Flow_FotBpReset(&s_fot_bp_q);
    s_fot_p_bp = 0.0f;
    s_fot_q_bp = 0.0f;
    resetOscStats();
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_last_osa_detected = 0U;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    configureOsc();
    setState(APAP_STATE_OSCILLATE);
}

static void exitProbeToMonitor(void)
{
    disableOsc();
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_no_breath_ms = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    setState(APAP_STATE_MONITOR);
}

static void resetBreathDetectors(void)
{
    s_fl_prev_phase = EPR_PHASE_UNKNOWN;
    s_fl_insp_peak_q = 0.0f;
    s_fl_baseline_slm = 0.0f;
    s_fl_streak = 0U;
    s_fl_good_streak = 0U;
    s_fl_active = false;
    s_fl_was_active = false;
    g_apap_fl_active = 0U;
    g_apap_fl_peak_baseline_slm = 0.0f;
    g_apap_fl_last_peak_slm = 0.0f;
    g_apap_fl_last_flat = 0.0f;
    s_fl_insp_sum_q = 0.0f;
    s_fl_insp_n = 0U;
    s_fl_last_flat = 0.0f;

    s_hyp_streak = 0U;
    s_hyp_limited_ms = 0U;
    s_hyp_active = false;
    s_hyp_hold_ms = 0U;
    g_apap_hyp_active = 0U;

    s_snore_prev_q = 0.0f;
    s_snore_hp = 0.0f;
    s_snore_rms2 = 0.0f;
    s_snore_on_ms = 0U;
    s_snore_off_ms = 0U;
    s_snore_active = false;
    s_snore_was_active = false;
    g_apap_snore_active = 0U;
    g_apap_snore_rms_slm = 0.0f;
    s_insp_ms = 0U;
    s_fl_insp_sum_q = 0.0f;
    s_fl_insp_n = 0U;
    s_since_insp_end_ms = 0U;
}

static void resetSessionEvents(void)
{
    g_apap_evt_oa = 0U;
    g_apap_evt_ca = 0U;
    g_apap_evt_hyp = 0U;
    g_apap_evt_fl = 0U;
    g_apap_evt_snore = 0U;
    g_apap_therapy_ms = 0U;
}

static void noteFlEdge(void)
{
    if (s_fl_active && !s_fl_was_active)
    {
        g_apap_evt_fl++;
    }
    s_fl_was_active = s_fl_active;
}

static void noteSnoreEdge(void)
{
    if (s_snore_active && !s_snore_was_active)
    {
        g_apap_evt_snore++;
    }
    s_snore_was_active = s_snore_active;
}

static void finalizeInspBreath(float peak_slm, uint32_t approx_breath_ms)
{
    const float min_peak = clampf(g_apap_fl_min_peak_slm, 1.0f, 30.0f);
    const float fl_ratio = clampf(g_apap_fl_ratio, 0.40f, 0.95f);
    const float hyp_ratio = clampf(g_apap_hyp_ratio, 0.20f, 0.80f);
    const float alpha = clampf(g_apap_fl_baseline_alpha, 0.05f, 0.75f);
    const float flat_thr = clampf(g_apap_fl_flat_ratio, 0.55f, 0.95f);
    const uint8_t fl_need = (g_apap_fl_breaths_needed < 1U) ? 1U
                          : ((g_apap_fl_breaths_needed > 10U) ? 10U
                                                             : g_apap_fl_breaths_needed);
    const uint8_t fl_clear = (g_apap_fl_clear_breaths < 1U) ? 1U
                           : ((g_apap_fl_clear_breaths > 10U) ? 10U
                                                             : g_apap_fl_clear_breaths);
    const uint8_t hyp_need = (g_apap_hyp_breaths_needed < 1U) ? 1U
                           : ((g_apap_hyp_breaths_needed > 10U) ? 10U
                                                               : g_apap_hyp_breaths_needed);
    const uint32_t hyp_need_ms =
        (uint32_t)(clampf(g_apap_hyp_min_duration_s, 5.0f, 60.0f) * 1000.0f);

    g_apap_fl_last_peak_slm = peak_slm;

    /* Independent flattening heuristic: insp mean/peak. Square/plateau
     * waveforms sit near 1; peaked breaths sit nearer 0.5. */
    {
        const float mean_q = (s_fl_insp_n > 0U)
                                 ? (s_fl_insp_sum_q / (float)s_fl_insp_n)
                                 : 0.0f;
        const float flat = (peak_slm > 0.5f)
                               ? clampf(mean_q / peak_slm, 0.0f, 1.0f)
                               : 0.0f;
        s_fl_last_flat = flat;
        g_apap_fl_last_flat = flat;
    }

    if (!(peak_slm >= min_peak))
    {
        return;
    }

    if (!(s_fl_baseline_slm >= min_peak))
    {
        s_fl_baseline_slm = peak_slm;
        g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
        return;
    }

    const bool fl_peak_drop = (peak_slm < (fl_ratio * s_fl_baseline_slm));
    const bool fl_flat = (s_fl_last_flat >= flat_thr);
    const bool fl_limited = fl_peak_drop || fl_flat;
    const bool hyp_limited = (peak_slm < (hyp_ratio * s_fl_baseline_slm));

    /* --- Flow limitation --- */
    if (g_apap_fl_enable != 0U)
    {
        if (fl_limited)
        {
            if (s_fl_streak < 255U) { s_fl_streak++; }
            s_fl_good_streak = 0U;
        }
        else
        {
            if (s_fl_good_streak < 255U) { s_fl_good_streak++; }
            s_fl_streak = 0U;
            const float rej = clampf(g_apap_fl_baseline_reject_ratio, 0.0f, 4.0f);
            const bool arousal_sized = (rej >= 1.05f)
                                    && (peak_slm > (rej * s_fl_baseline_slm));
            if (!arousal_sized)
            {
                s_fl_baseline_slm =
                    ((1.0f - alpha) * s_fl_baseline_slm) + (alpha * peak_slm);
            }
        }
        if (s_fl_streak >= fl_need) { s_fl_active = true; }
        if (s_fl_good_streak >= fl_clear) { s_fl_active = false; }
        g_apap_fl_active = s_fl_active ? 1U : 0U;
        noteFlEdge();
    }

    /* --- Hypopnea --- */
    if (g_apap_hyp_enable != 0U)
    {
        if (hyp_limited)
        {
            if (s_hyp_streak < 255U) { s_hyp_streak++; }
            s_hyp_limited_ms += (approx_breath_ms > 0U) ? approx_breath_ms : 3000U;
            if ((s_hyp_streak >= hyp_need) && (s_hyp_limited_ms >= hyp_need_ms)
                && !s_hyp_active)
            {
                s_hyp_active = true;
                g_apap_evt_hyp++;
                s_hyp_hold_ms = (uint32_t)(clampf(g_apap_hyp_hold_s, 5.0f, 120.0f)
                                           * 1000.0f);
            }
        }
        else
        {
            s_hyp_streak = 0U;
            s_hyp_limited_ms = 0U;
            if (!s_hyp_active)
            {
                /* keep baseline updates via FL path when not limited */
            }
        }
    }

    g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
    g_apap_hyp_active = s_hyp_active ? 1U : 0U;
}

static void updateFlHypDetector(uint32_t dt_ms)
{
    const EprPhase_t ph = Epr_GetPhase();
    const float q = Flow_GetPatientSlm();

    if (ph == EPR_PHASE_INSPIRATION)
    {
        if (s_fl_prev_phase != EPR_PHASE_INSPIRATION)
        {
            s_fl_insp_peak_q = q;
            s_insp_ms = 0U;
            s_fl_insp_sum_q = 0.0f;
            s_fl_insp_n = 0U;
        }
        else if (q > s_fl_insp_peak_q)
        {
            s_fl_insp_peak_q = q;
        }
        s_insp_ms += dt_ms;
        s_fl_insp_sum_q += q;
        if (s_fl_insp_n < 100000U)
        {
            s_fl_insp_n++;
        }
    }
    else if (s_fl_prev_phase == EPR_PHASE_INSPIRATION)
    {
        /* Finding 2: the hypopnea's 10 s is wall-clock time, so pass the
         * breath PERIOD (end of inspiration to end of inspiration), not the
         * inspiratory time - that needed >= ~20 s of hypopnea at I:E 1:1
         * (bench: 50 % scored only at the event's end, 55 % never). The
         * first breath after a reset has no period yet (0 -> the old 3 s
         * fallback in finalizeInspBreath); a gap longer than the breath
         * window cannot be one breath, so it falls back too. */
        const uint32_t cap_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                                  4.0f, 30.0f) * 1000.0f);
        uint32_t period_ms = s_since_insp_end_ms;
        if (period_ms > cap_ms) { period_ms = 0U; }
        finalizeInspBreath(s_fl_insp_peak_q, period_ms);
        s_insp_ms = 0U;
        s_since_insp_end_ms = 0U;
    }
    if (s_since_insp_end_ms < 600000U) { s_since_insp_end_ms += dt_ms; }

    s_fl_prev_phase = ph;

    /* Hypopnea hold timer (raise window after event). */
    if (s_hyp_active)
    {
        if (s_hyp_hold_ms > dt_ms)
        {
            s_hyp_hold_ms -= dt_ms;
        }
        else
        {
            s_hyp_hold_ms = 0U;
            s_hyp_active = false;
            s_hyp_streak = 0U;
            s_hyp_limited_ms = 0U;
        }
        g_apap_hyp_active = s_hyp_active ? 1U : 0U;
    }
}

static void updateSnoreDetector(float dt_s, uint32_t dt_ms)
{
    if (g_apap_snore_enable == 0U)
    {
        s_snore_active = false;
        g_apap_snore_active = 0U;
        g_apap_snore_rms_slm = 0.0f;
        return;
    }

    const float q_raw = Flow_GetPatientRawSlm();
    const float fc = clampf(g_apap_snore_hp_hz, 5.0f, 80.0f);
    /* First-order HP: y += a*(y + x - x_prev) with a = 1/(1+2*pi*fc*dt)
     * approx via residual of raw vs breath LPF when dt small. */
    const float a = clampf((2.0f * 3.14159265f * fc * dt_s)
                               / (1.0f + 2.0f * 3.14159265f * fc * dt_s),
                           0.01f, 0.95f);
    s_snore_hp = a * (s_snore_hp + q_raw - s_snore_prev_q);
    s_snore_prev_q = q_raw;

    /* Also blend residual vs breath-band patient flow (snore = HF). */
    const float resid = q_raw - Flow_GetPatientSlm();
    const float hf = 0.5f * (s_snore_hp + resid);

    const float alpha_rms = clampf(dt_s / 0.20f, 0.01f, 0.5f); /* ~200 ms */
    s_snore_rms2 = ((1.0f - alpha_rms) * s_snore_rms2) + (alpha_rms * hf * hf);
    const float rms = sqrtf(s_snore_rms2);
    g_apap_snore_rms_slm = rms;

    const float thr = clampf(g_apap_snore_rms_thresh_slm, 0.05f, 5.0f);
    const uint32_t on_need =
        (uint32_t)(clampf(g_apap_snore_on_s, 0.3f, 10.0f) * 1000.0f);
    const uint32_t off_need =
        (uint32_t)(clampf(g_apap_snore_off_s, 0.3f, 15.0f) * 1000.0f);

    /* Prefer snore during inspiration (effort + vibration). */
    const bool insp = (Epr_GetPhase() == EPR_PHASE_INSPIRATION);
    if (insp && (rms >= thr))
    {
        s_snore_on_ms += dt_ms;
        s_snore_off_ms = 0U;
        if (s_snore_on_ms >= on_need)
        {
            s_snore_active = true;
        }
    }
    else
    {
        s_snore_off_ms += dt_ms;
        s_snore_on_ms = 0U;
        if (s_snore_off_ms >= off_need)
        {
            s_snore_active = false;
        }
    }

    g_apap_snore_active = s_snore_active ? 1U : 0U;
    noteSnoreEdge();
}

static void applyMonitorTitration(float dt_s, uint32_t dt_ms)
{
    g_apap_lowering = 0U;
    (void)raiseAllowed(); /* refresh leak flag */

    /* Priority: hypopnea > FL > snore > stable lower. */
    if ((g_apap_hyp_enable != 0U) && s_hyp_active)
    {
        applyRaise(g_apap_hyp_raise_scale, dt_s);
        return;
    }
    if ((g_apap_fl_enable != 0U) && s_fl_active)
    {
        float fl_scale = clampf(g_apap_fl_raise_scale, 0.05f, 1.0f);
        const float flat_thr = clampf(g_apap_fl_flat_ratio, 0.55f, 0.95f);
        if (s_fl_last_flat >= flat_thr)
        {
            const float span = 1.0f - flat_thr;
            const float sev = (span > 0.05f)
                                  ? clampf((s_fl_last_flat - flat_thr) / span,
                                           0.25f, 1.0f)
                                  : 1.0f;
            fl_scale *= sev;
        }
        else if ((s_fl_baseline_slm > 0.5f) && (g_apap_fl_last_peak_slm > 0.0f))
        {
            const float rel = g_apap_fl_last_peak_slm / s_fl_baseline_slm;
            const float fl_r = clampf(g_apap_fl_ratio, 0.40f, 0.95f);
            const float sev = clampf((fl_r - rel) / fl_r, 0.25f, 1.0f);
            fl_scale *= sev;
        }
        applyRaise(fl_scale, dt_s);
        return;
    }
    if ((g_apap_snore_enable != 0U) && s_snore_active)
    {
        const float thr = clampf(g_apap_snore_rms_thresh_slm, 0.05f, 5.0f);
        const float rms = g_apap_snore_rms_slm;
        const float sev = (thr > 0.05f)
                              ? clampf((rms - thr) / thr, 0.25f, 1.0f)
                              : 1.0f;
        applyRaise(g_apap_snore_raise_scale * sev, dt_s);
        return;
    }

    g_apap_stable_ms += dt_ms;
    if (g_apap_lower_enable == 0U)
    {
        return;
    }

    const uint32_t stable_need_ms =
        (uint32_t)(clampf(g_apap_stable_before_lower_s, 30.0f, 600.0f) * 1000.0f);
    if (g_apap_stable_ms < stable_need_ms)
    {
        return;
    }

    const float rate = clampf(g_apap_lower_rate_cmh2o_per_s, 0.001f, 0.05f);
    g_apap_target_cmh2o -= rate * dt_s;
    g_apap_lowering = 1U;
}

void Apap_Init(void)
{
    /* Must match the initialisers above (2026-10-12 findings 1/3) - this
     * runs at boot and would otherwise silently restore the old 4/12 s. */
    g_apap_no_breath_timeout_s = 2.0f;
    g_apap_breath_window_s = 8.0f;
    g_apap_resume_min_peak_slm = 10.0f;
    g_apap_resume_peak_ratio = 0.30f;
    g_apap_resume_min_floor_slm = 4.0f;
    g_apap_startup_grace_s = 15.0f;
    g_apap_osc_amplitude_cmh2o = 1.0f;
    g_apap_osc_amplitude_max_cmh2o = 1.75f;
    g_apap_osc_freq_hz = 4.0f;
    g_apap_fot_eval_period_s = 2.0f;
    g_apap_osc_min_p_pp_cmh2o = 0.15f;
    g_apap_osa_flow_pp_thresh_slm = 2.0f;
    g_apap_apnea_min_s = 10.0f;
    g_apap_oa_raise_cmh2o_per_s = 0.08f;
    g_apap_oa_raise_max_cmh2o = 2.0f;
    g_apap_her_oa_above_cmh2o = 12.0f;
    g_apap_her_oa_above_scale = 0.50f;
    g_apap_raise_rate_standard_cmh2o_per_s = 0.050f;
    g_apap_raise_rate_soft_cmh2o_per_s = 0.020f;
    g_apap_raise_rate_her_scale = 0.75f;
    g_apap_response_soft = 0U;
    g_apap_fl_enable = 1U;
    g_apap_fl_ratio = 0.70f;
    g_apap_fl_breaths_needed = 3U;
    g_apap_fl_clear_breaths = 2U;
    g_apap_fl_raise_scale = 0.50f;
    g_apap_fl_min_peak_slm = 4.0f;
    g_apap_fl_baseline_alpha = 0.25f;
    g_apap_fl_flat_ratio = 0.78f;
    g_apap_fl_baseline_reject_ratio = 1.40f;
    g_apap_hyp_enable = 1U;
    g_apap_hyp_ratio = 0.50f;
    g_apap_hyp_breaths_needed = 2U;
    g_apap_hyp_min_duration_s = 10.0f;
    g_apap_hyp_raise_scale = 0.70f;
    g_apap_hyp_hold_s = 20.0f;
    g_apap_snore_enable = 1U;
    g_apap_snore_rms_thresh_slm = 0.35f;
    g_apap_snore_on_s = 1.5f;
    g_apap_snore_off_s = 2.0f;
    g_apap_snore_raise_scale = 0.35f;
    g_apap_snore_hp_hz = 20.0f;
    g_apap_lower_enable = 1U;
    g_apap_lower_rate_cmh2o_per_s = 0.008f;
    g_apap_stable_before_lower_s = 120.0f;
    g_apap_leak_gate_enable = 1U;

    s_active = false;
    s_for_her = false;
    s_osa_treating = false;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_running_enter_ms = 0U;
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    g_apap_target_cmh2o = 8.0f;
    g_apap_no_breath_ms = 0U;
    g_apap_osa_treating = 0U;
    g_apap_fot_p_pp_cmh2o = 0.0f;
    g_apap_fot_q_pp_slm = 0.0f;
    g_apap_fot_z_cmh2o_per_slm = 0.0f;
    g_apap_fot_osa_likely = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    g_apap_raise_blocked_leak = 0U;
    Flow_FotBpReset(&s_fot_bp_p);
    Flow_FotBpReset(&s_fot_bp_q);
    s_fot_p_bp = 0.0f;
    s_fot_q_bp = 0.0f;
    resetBreathDetectors();
    resetSessionEvents();
    setState(APAP_STATE_IDLE);
    disableOsc();
}

void Apap_SetResponseSoft(bool soft)
{
    g_apap_response_soft = soft ? 1U : 0U;
}

bool Apap_IsResponseSoft(void)
{
    return (g_apap_response_soft != 0U);
}

void Apap_OnTherapyStart(float min_cmh2o, float max_cmh2o, uint8_t therapy_mode)
{
    if (!autosetMode(therapy_mode))
    {
        s_active = false;
        setState(APAP_STATE_IDLE);
        disableOsc();
        return;
    }

    s_active = true;
    s_for_her = (therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
    if (s_for_her)
    {
        /* AirSense for Her: gentler rises + slightly more FL-sensitive. */
        g_apap_response_soft = 1U;
        g_apap_fl_breaths_needed = 2U;
        g_apap_fl_raise_scale = 0.55f;
        g_apap_fl_flat_ratio = 0.74f;
    }
    s_min_cmh2o = clampf(min_cmh2o, THERAPY_PRESS_MIN_CMH2O, THERAPY_PRESS_MAX_CMH2O);
    s_max_cmh2o = clampf(max_cmh2o, s_min_cmh2o + 0.5f, THERAPY_PRESS_MAX_CMH2O);
    if (s_max_cmh2o < s_min_cmh2o)
    {
        s_max_cmh2o = s_min_cmh2o;
    }

    g_apap_target_cmh2o = s_min_cmh2o;
    g_apap_no_breath_ms = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_osa_treating = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    g_apap_raise_blocked_leak = 0U;
    s_osa_treating = false;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_running_enter_ms = HAL_GetTick();
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    s_resume_ref_slm = 0.0f;
    resetBreathDetectors();
    resetSessionEvents();
    disableOsc();
    setState(APAP_STATE_MONITOR);
}

void Apap_OnTherapyStop(void)
{
    s_active = false;
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_no_breath_ms = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    Flow_FotBpReset(&s_fot_bp_p);
    Flow_FotBpReset(&s_fot_bp_q);
    s_fot_p_bp = 0.0f;
    s_fot_q_bp = 0.0f;
    resetBreathDetectors();
    setState(APAP_STATE_IDLE);
    disableOsc();
}

void Apap_OnMaskReconnect(void)
{
    if (!s_active)
    {
        return;
    }
    exitProbeToMonitor();
}

float Apap_GetTargetPressure(void)
{
    return g_apap_target_cmh2o;
}

ApapState_t Apap_GetState(void)
{
    return (ApapState_t)g_apap_state;
}

const char* Apap_GetStateName(void)
{
    switch ((ApapState_t)g_apap_state)
    {
    case APAP_STATE_MONITOR:
        if (s_hyp_active) { return "Monitor+Hyp"; }
        if (s_fl_active)  { return "Monitor+FL"; }
        if (s_snore_active) { return "Monitor+Snore"; }
        if (g_apap_lowering != 0U) { return "Monitor+Lower"; }
        return "Monitor";
    case APAP_STATE_NO_BREATH:  return "No breath";
    case APAP_STATE_OSCILLATE:
        return s_osa_treating ? "FOT+Raise" : "FOT probe";
    case APAP_STATE_RESPOND:    return "Raise P";
    case APAP_STATE_IDLE:
    default:                    return "Idle";
    }
}

bool Apap_IsOscillating(void)
{
    return ((ApapState_t)g_apap_state == APAP_STATE_OSCILLATE)
        && BlowerOsc_IsEnabled();
}

float Apap_GetFotPressurePpCmh2o(void)     { return g_apap_fot_p_pp_cmh2o; }
float Apap_GetFotFlowPpSlm(void)           { return g_apap_fot_q_pp_slm; }
float Apap_GetFotImpedanceCmh2oPerSlm(void) { return g_apap_fot_z_cmh2o_per_slm; }
float Apap_GetOsaFlowPpThreshSlm(void)     { return g_apap_osa_flow_pp_thresh_slm; }
bool  Apap_IsOsaLikely(void)               { return (g_apap_fot_osa_likely != 0U); }
bool  Apap_IsOsaTreating(void)             { return (g_apap_osa_treating != 0U); }
bool  Apap_IsFlowLimited(void)             { return (g_apap_fl_active != 0U); }
bool  Apap_IsHypopneaActive(void)          { return (g_apap_hyp_active != 0U); }
bool  Apap_IsSnoreActive(void)             { return (g_apap_snore_active != 0U); }
bool  Apap_IsRaiseBlockedByLeak(void)      { return (g_apap_raise_blocked_leak != 0U); }
float Apap_GetFlPeakBaselineSlm(void)      { return g_apap_fl_peak_baseline_slm; }
float Apap_GetFlLastPeakSlm(void)          { return g_apap_fl_last_peak_slm; }
float Apap_GetFlLastFlat(void)             { return g_apap_fl_last_flat; }
float Apap_GetSnoreRmsSlm(void)            { return g_apap_snore_rms_slm; }
uint32_t Apap_GetStableMs(void)            { return g_apap_stable_ms; }
bool  Apap_IsLowering(void)                { return (g_apap_lowering != 0U); }

uint32_t Apap_GetEventOaCount(void)        { return g_apap_evt_oa; }
uint32_t Apap_GetEventCaCount(void)        { return g_apap_evt_ca; }
uint32_t Apap_GetEventHypCount(void)       { return g_apap_evt_hyp; }
uint32_t Apap_GetEventFlCount(void)        { return g_apap_evt_fl; }
uint32_t Apap_GetEventSnoreCount(void)     { return g_apap_evt_snore; }
uint32_t Apap_GetTherapyMs(void)           { return g_apap_therapy_ms; }

float Apap_GetSessionAhi(void)
{
    const float hrs = sessionHours();
    if (!(hrs > 0.0f)) { return 0.0f; }
    /* Finding 5: AHI counts ALL apneas (it read 0.00 with CA 3 when it was
     * (OA + Hyp) / h). */
    return ((float)(g_apap_evt_oa + g_apap_evt_ca + g_apap_evt_hyp)) / hrs;
}

float Apap_GetSessionRdi(void)
{
    const float hrs = sessionHours();
    if (!(hrs > 0.0f)) { return 0.0f; }
    return ((float)(g_apap_evt_oa + g_apap_evt_ca + g_apap_evt_hyp
                    + g_apap_evt_fl)) / hrs;
}

float Apap_GetSessionSnoreIndex(void)
{
    const float hrs = sessionHours();
    if (!(hrs > 0.0f)) { return 0.0f; }
    return ((float)g_apap_evt_snore) / hrs;
}

float Apap_Update(float dt_s, uint8_t therapy_state, bool mask_off)
{
    if (!s_active
        || mask_off
        || (therapy_state != (uint8_t)THERAPY_STATE_RUNNING))
    {
        if (mask_off && s_active)
        {
            disableOsc();
            s_osa_treating = false;
            g_apap_osa_treating = 0U;
            g_apap_lowering = 0U;
        }
        return g_apap_target_cmh2o;
    }

    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return g_apap_target_cmh2o;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);
    g_apap_therapy_ms += dt_ms;

    if (startupGraceActive())
    {
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        g_apap_lowering = 0U;
        setState(APAP_STATE_MONITOR);
        return g_apap_target_cmh2o;
    }

    /* Recovery-sized breath ended the apnea / FOT episode. */
    if (recoveryBreath()
        && ((ApapState_t)g_apap_state == APAP_STATE_OSCILLATE
            || (ApapState_t)g_apap_state == APAP_STATE_NO_BREATH))
    {
        exitProbeToMonitor();
    }

    switch ((ApapState_t)g_apap_state)
    {
    case APAP_STATE_MONITOR:
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        if (!recentBreath())
        {
            /* Keep the breath-size reference for the recovery test before
             * the detectors (and their baseline) are reset - finding 3. */
            s_resume_ref_slm = s_fl_baseline_slm;
            resetBreathDetectors();
            g_apap_stable_ms = 0U;
            g_apap_lowering = 0U;
            setState(APAP_STATE_NO_BREATH);
            g_apap_no_breath_ms = 0U;
            /* Finding 1: the episode began at the last qualified breath, a
             * full window ago - not now. */
            s_episode_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                             4.0f, 30.0f) * 1000.0f);
            s_oa_raise_applied = 0.0f;
            break;
        }
        updateFlHypDetector(dt_ms);
        updateSnoreDetector(dt_s, dt_ms);
        applyMonitorTitration(dt_s, dt_ms);
        break;

    case APAP_STATE_NO_BREATH:
        disableOsc();
        g_apap_no_breath_ms += dt_ms;
        s_episode_ms += dt_ms;
        g_apap_stable_ms = 0U;
        g_apap_lowering = 0U;
        if (recoveryBreath())
        {
            exitProbeToMonitor();
            break;
        }
        {
            const uint32_t timeout_ms =
                (uint32_t)(clampf(g_apap_no_breath_timeout_s, 2.0f, 30.0f)
                           * 1000.0f);
            if (g_apap_no_breath_ms >= timeout_ms)
            {
                enterFotProbe();
            }
        }
        break;

    case APAP_STATE_OSCILLATE:
        configureOsc();
        trackOscStats(dt_s);
        refreshFotMetrics();
        g_apap_osc_eval_ms += dt_ms;
        s_episode_ms += dt_ms;
        g_apap_stable_ms = 0U;
        g_apap_lowering = 0U;

        if (recoveryBreath())
        {
            exitProbeToMonitor();
            break;
        }

        {
            const uint32_t eval_ms =
                (uint32_t)(clampf(g_apap_fot_eval_period_s, 0.5f, 10.0f)
                           * 1000.0f);
            const uint32_t apnea_min_ms =
                (uint32_t)(clampf(g_apap_apnea_min_s, 6.0f, 30.0f) * 1000.0f);
            if (g_apap_osc_eval_ms >= eval_ms)
            {
                const bool osa = evaluateOscForOsa();
                g_apap_last_osa_detected = osa ? 1U : 0U;
                s_osa_treating = osa;
                g_apap_osa_treating = osa ? 1U : 0U;

                if ((s_episode_ms >= apnea_min_ms)
                    && osa && s_osa_event_armed)
                {
                    g_apap_evt_oa++;
                    s_osa_event_armed = false;
                    s_ca_event_armed = false;
                }
                else if ((s_episode_ms >= apnea_min_ms)
                         && !osa && s_ca_event_armed
                         && (g_apap_fot_p_pp_cmh2o
                             >= clampf(g_apap_osc_min_p_pp_cmh2o, 0.05f, 0.8f)))
                {
                    /* Adequate FOT pressure with open-airway flow → central. */
                    g_apap_evt_ca++;
                    s_ca_event_armed = false;
                }
                resetOscStats();
            }
        }

        applyOaDurationRaise();
        {
            const uint32_t apnea_min_ms =
                (uint32_t)(clampf(g_apap_apnea_min_s, 6.0f, 30.0f) * 1000.0f);
            if ((s_episode_ms >= apnea_min_ms)
                && s_osa_treating && s_osa_event_armed)
            {
                g_apap_evt_oa++;
                s_osa_event_armed = false;
                s_ca_event_armed = false;
            }
        }
        break;

    case APAP_STATE_RESPOND:
        enterFotProbe();
        break;

    case APAP_STATE_IDLE:
    default:
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        g_apap_lowering = 0U;
        break;
    }

    g_apap_target_cmh2o = clampf(g_apap_target_cmh2o, s_min_cmh2o, s_max_cmh2o);
    return g_apap_target_cmh2o;
}
