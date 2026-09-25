/**
  ******************************************************************************
  * @file    flow_sensor.c
  * @brief   Flow-element ΔP conditioning - implementation.
  *
  *  The 1st-order IIR LPF used here is the discrete-time bilinear (Tustin)
  *  approximation of an analog RC low-pass:
  *
  *      H(s) = 1 / (1 + s/(2*pi*fc))
  *
  *      In discrete form, with omega = 2*pi*fc and dt = sample period:
  *          y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  *      with
  *          alpha = (omega * dt) / (1 + omega * dt)
  *
  *  At fc = 10 Hz, dt = 5 ms:
  *          omega*dt = 2*pi*10*0.005 = 0.314
  *          alpha    = 0.314 / 1.314  = 0.239
  *
  *  Step response settles to 95% in ~3 time constants, i.e. ~50 ms - well
  *  inside the 0.5-2 s breath cycle so respiratory features are preserved.
  *
  *  Why bilinear instead of impulse-invariance (alpha = 1 - exp(-omega*dt)):
  *  both behave identically for omega*dt << 1; the bilinear form has no
  *  expf() call so it stays cheap even if you ever crank dt down to 1 ms.
  ******************************************************************************
  */

#include "flow_sensor.h"
#include "leak_estimator.h"
#include "leak_btps.h"

#include <math.h>
#include <stddef.h>

#define FLOW_TWO_PI   6.28318530717958647692f

/* SFM3300-250-D reference is SLM at 20 °C / 1013.25 mbar.
 * BTPS: 37 °C, ambient P, water vapor saturated (PH2O ≈ 62.67 mbar). */
#define FLOW_BTPS_T_K           310.15f
#define FLOW_SFM_T_K            293.15f   /* 20 °C Sensirion cal           */
#define FLOW_BTPS_PH2O_MBAR     62.67f    /* saturated vapor @ 37 °C       */
#define FLOW_BTPS_P_DEFAULT     1013.25f
#define FLOW_ALT_P_REF_MBAR     1013.25f

/* === Public configuration (debugger-visible volatiles) === */
volatile float    g_flow_lpf_cutoff_hz   = 10.0f;      /* tunable cutoff    */
volatile float    g_flow_zero_offset_mbar = -0.033f;   /* Round Hole_2 zero */

/* Optional SLM (SFM3300) → BTPS scaling applied after density correction.
 * Ambient P defaults to 1013.25 mbar and is overwritten live from
 * measAS.pressure (AMS5935 absolute, mbar) in SensorTask @ ~10 Hz. */
volatile uint8_t  g_flow_btps_enable     = 0U;         /* 0=SLM, 1=BTPS     */
volatile float    g_flow_btps_amb_mbar   = FLOW_BTPS_P_DEFAULT;
volatile float    g_flow_btps_factor     = 1.128f;     /* cached BTPS scale */

/* Density / altitude compensation on LUT→SLM (default ON). */
volatile uint8_t  g_flow_alt_enable      = 1U;
volatile float    g_flow_alt_factor      = 1.0f;      /* sqrt(P/Pref)       */
volatile float    g_flow_alt_meters      = 0.0f;

/* Median spike guard on raw ΔP before the LPF.
 *
 * We poll the AMS5935 every 5 ms for a conversion that needs ~4 ms, with no
 * EOC line (hamsFS.use_eoc = 0). The sensor's internal oscillator drifts
 * against our tick, so the phase between "register updated" and "we read"
 * creeps; once per beat period the read lands on the update and returns a
 * torn 24-bit value. A ~0.1 mbar tear becomes a ~10 slm needle through the
 * near-zero LUT (~100 slm/mbar).
 *
 * Taps = 3 rejects lone tears; taps = 5 also rejects two consecutive ones,
 * which is what a clock beat produces. Cost is 2 extra ticks (10 ms) of
 * group delay, far inside the breath band. */
volatile uint8_t  g_flow_median_enable = 1U;
volatile uint8_t  g_flow_median_taps   = 5U;   /* 3 or 5 */
volatile uint32_t g_flow_median_reject_count = 0U;

/* Coarse plausibility guard for gross tears (a corrupted upper byte moves ΔP
 * by whole mbar). Deliberately loose so real therapy transients - a cough or
 * mask-off can slew several mbar/s - are never rejected; subtle 1-2 tick
 * tears are the median's job, not this one. */
volatile float    g_flow_dp_max_slew_mbar_per_s = 50.0f;
volatile uint8_t  g_flow_slew_max_reject = 3U;
volatile uint32_t g_flow_slew_reject_count = 0U;

/* Per-sample cap on the LUT flow after the median. A torn ~1–2 mbar ΔP is
 * ~100–200 slm through the near-zero curve; the 2 Hz patient LPF then prints
 * that one tick as a ~12 slm needle. Real breath onset is ~2 slm / 10 ms.
 * A genuine mask-off is allowed to catch up at this rate (~600 slm/s). */
volatile float    g_flow_q_max_step_slm = 6.0f;
volatile uint32_t g_flow_impulse_reject_count = 0U;

/* Sample freshness. The AMS5935 needs ~4 ms per conversion, so a 200 Hz
 * poll can land on a busy/incomplete conversion; the driver then repeats
 * the last good sample. A repeated ΔP is NOT quiet data - it has zero
 * peak-peak, which would otherwise satisfy the auto-zero quiet gate. */
volatile uint8_t  g_flow_stale_now    = 1U;
volatile uint8_t  g_flow_sample_saturated = 0U;
volatile uint32_t g_flow_stale_count  = 0U;
volatile uint32_t g_flow_stale_run    = 0U;   /* consecutive stale ticks */
volatile uint32_t g_flow_stale_run_max = 0U;

/* Spike forensics. A Q_patient needle is either pneumatic/sensor (ΔP moved)
 * or arithmetic (zero offset / leak DC moved). Latch everything at the peak
 * so one "flow spike" tells us which, plus the gap between events. */
volatile float    g_flow_spike_thresh_slm = 5.0f;
volatile uint32_t g_flow_spike_count    = 0U;
volatile uint32_t g_flow_ms             = 0U;   /* ms since Flow_Init */
volatile uint32_t g_flow_spike_ms       = 0U;   /* time of last spike */
volatile uint32_t g_flow_spike_gap_ms   = 0U;   /* since previous spike */
volatile float    g_flow_spike_q_patient = 0.0f;
volatile float    g_flow_spike_dp_raw   = 0.0f;
volatile float    g_flow_spike_dp_filt  = 0.0f;
volatile float    g_flow_spike_q_total  = 0.0f;
volatile float    g_flow_spike_vent     = 0.0f;
volatile float    g_flow_spike_uninten  = 0.0f;
volatile float    g_flow_spike_zero     = 0.0f;
volatile uint32_t g_flow_spike_az_count = 0U;
volatile uint32_t g_flow_spike_stale    = 0U;
volatile uint32_t g_flow_spike_medrej   = 0U;

/* === State (debugger-visible volatiles) === */
volatile float    g_flow_dp_raw_mbar   = 0.0f;
volatile float    g_flow_dp_filt_mbar  = 0.0f;
volatile float    g_flow_dp_corr_mbar  = 0.0f;        /* filtered minus zero */
volatile float    g_flow_q_slm_lut     = 0.0f;        /* raw LUT (sea-level) */
volatile float    g_flow_q_slm_std     = 0.0f;        /* density-corrected SLM */
volatile float    g_flow_q_slm         = 0.0f;        /* SLM or BTPS (active) */
volatile float    g_flow_q_slm_raw     = 0.0f;        /* from raw ΔP (active) */
volatile float    g_flow_lpf_alpha     = 0.0f;        /* read-only output  */
volatile uint32_t g_flow_step_count    = 0U;

/* Idle / boot auto-zero (#51).
 *
 * Why a snap is poison on this hardware:
 *   Q = LUT(ΔP_filt − zero). Near zero the Round Hole_2 curve is
 *   ~100 slm/mbar, so copying one 15 Hz LPF sample into `zero` moves
 *   Q_total by tens of slm. Q_patient is that residual, so the plot
 *   shows a needle; the 10 s leak LPF then eats the DC (spike + decay).
 *   Idle 1–3 slm hash used to unlatch the old “quiet 2.5 s then snap”
 *   loop every ~15–20 s.
 *
 * Fix: gate on ΔP AC (peak-peak), average the quiet DC, slew `zero`
 * toward that mean at ~0.3 slm/s. Never copy a single sample. */
volatile uint8_t  g_flow_az_enable           = 1U;
volatile float    g_flow_az_quiet_pp_mbar    = 0.025f; /* AC gate (not Q)   */
volatile float    g_flow_az_slew_mbar_per_s  = 0.003f; /* ~0.3 slm/s        */
volatile float    g_flow_az_done_mbar        = 0.0005f; /* ~0.05 slm band   */
volatile uint32_t g_flow_az_quiet_ms         = 5000U;  /* mean window       */
volatile uint32_t g_flow_az_boot_delay_ms    = 2000U;  /* LPF settle @ idle */
volatile uint32_t g_flow_az_min_latch_ms     = 120000U; /* 2 min between trims */
volatile uint32_t g_flow_az_count            = 0U;
volatile uint8_t  g_flow_az_latched          = 0U;     /* 1 = settled/hold  */
volatile uint8_t  g_flow_az_quiet_now        = 0U;
volatile uint8_t  g_flow_az_slewing          = 0U;
volatile float    g_flow_az_target_mbar      = 0.0f;
volatile float    g_flow_az_last_pp_mbar     = 0.0f;
volatile float    g_flow_az_last_q_pp_slm    = 0.0f;

/* === Internal === */
static bool s_filt_seeded = false;       /* first sample bypasses filter */
static uint32_t s_az_elapsed_ms = 0U;
static uint32_t s_az_quiet_ms = 0U;
static uint32_t s_az_latch_ms = 0U;
static float    s_az_dp_min = 0.0f;
static float    s_az_dp_max = 0.0f;
static uint32_t s_az_win_ms = 0U;
static bool     s_az_win_seeded = false;
static bool     s_az_mean_seeded = false;
static float    s_az_mean_dp = 0.0f;
static uint8_t  s_az_cmd_slew = 0U;      /* USB "flow autozero" */
static uint32_t s_az_block_ms = 0U;      /* consecutive !allow time */
static uint32_t s_az_tile_ticks = 0U;    /* samples in current 0.5 s tile  */
static uint32_t s_az_tile_fresh = 0U;    /* of which not held/stale        */
static float    s_az_tile_sum = 0.0f;    /* sum of fresh ΔP in this tile   */
static float    s_dp_hist[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static uint8_t  s_dp_hist_n = 0U;
static float    s_dp_last_good = 0.0f;
static float    s_q_last_good = 0.0f;
static float    s_reject_dp = 0.0f;
static uint8_t  s_slew_rejects = 0U;
static uint8_t  s_agree_rejects = 0U;
static bool     s_spike_armed = true;

/* === Calibration table =====================================================
 *
 *  Round Hole_2 element, bench vs SFM3300-250-D (SLM @ 20 °C / 1013 mbar).
 *  Spreadsheet column 1 is g_flow_dp_filt_mbar (raw filtered ΔP).
 *  Zero flow reads -0.033 mbar → g_flow_zero_offset_mbar = -0.033.
 *  Table below stores ΔP_corrected = filt − zero → Q_SLM.
 *
 *      filt ΔP (mbar)  ΔP_corr (mbar)   Q (slm SFM)
 *      ------------------------------------------------
 *      -4.86           -4.827            -83.50
 *      -3.82           -3.787            -72.50
 *      -2.75           -2.717            -59.50
 *      -1.96           -1.927            -48.00
 *      -1.29           -1.257            -36.30
 *      -0.765          -0.732            -25.20
 *      -0.373          -0.340            -14.10
 *      -0.136          -0.103             -5.333
 *      -0.083          -0.050             -2.815
 *      -0.047          -0.014             -0.835
 *      -0.033           0.000              0.00
 *      -0.019           0.014              0.835
 *       0.016           0.049              2.835
 *       0.068           0.101              5.38
 *       0.298           0.331             14.375
 *       0.678           0.711             25.55
 *       1.175           1.208             37.10
 *       1.79            1.823             49.25
 *       2.55            2.583             61.50
 *       3.48            3.513             74.50
 *       4.52            4.553             85.50
 *       4.95            4.983             93.25
 *
 *  LUT reproduces measured points exactly; linear interpolation between,
 *  end-segment slope beyond table limits.
 */
typedef struct
{
    float dp_mbar;     /* ΔP after zero-offset correction */
    float q_slm;
} FlowCalPoint;

static const FlowCalPoint kFlowCal[] =
{
    { -4.827f,   -83.50f },
    { -3.787f,   -72.50f },
    { -2.717f,   -59.50f },
    { -1.927f,   -48.00f },
    { -1.257f,   -36.30f },
    { -0.732f,   -25.20f },
    { -0.340f,   -14.10f },
    { -0.103f,    -5.333f },
    { -0.050f,    -2.815f },
    { -0.014f,    -0.835f },
    {  0.000f,     0.00f },
    {  0.014f,     0.835f },
    {  0.049f,     2.835f },
    {  0.101f,     5.38f },
    {  0.331f,    14.375f },
    {  0.711f,    25.55f },
    {  1.208f,    37.10f },
    {  1.823f,    49.25f },
    {  2.583f,    61.50f },
    {  3.513f,    74.50f },
    {  4.553f,    85.50f },
    {  4.983f,    93.25f },
};

#define FLOW_CAL_N   (sizeof(kFlowCal) / sizeof(kFlowCal[0]))

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float computeBtpsFactor(void)
{
    const float p = clampf(g_flow_btps_amb_mbar, 600.0f, 1100.0f);
    float denom = p - FLOW_BTPS_PH2O_MBAR;
    if (denom < 100.0f)
    {
        denom = 100.0f;
    }
    /* Sensirion SLM @ 20 °C / 1013 mbar (dry) → BTPS @ 37 °C saturated. */
    return (FLOW_BTPS_T_K / FLOW_SFM_T_K) * (p / denom);
}

static float computeAltitudeFactor(float mbar)
{
    const float p = clampf(mbar, 600.0f, 1100.0f);
    float f = sqrtf(p / FLOW_ALT_P_REF_MBAR);
    return clampf(f, 0.70f, 1.15f);
}

static float estimateAltitudeMeters(float mbar)
{
    /* International Standard Atmosphere approximation. */
    const float p = clampf(mbar, 600.0f, 1100.0f);
    const float ratio = p / FLOW_ALT_P_REF_MBAR;
    float meters = 44330.0f * (1.0f - powf(ratio, 0.1903f));
    return clampf(meters, -500.0f, 6000.0f);
}

static void refreshAmbientDerived(void)
{
    g_flow_btps_factor = computeBtpsFactor();
    if (g_flow_alt_enable != 0U)
    {
        g_flow_alt_factor = computeAltitudeFactor(g_flow_btps_amb_mbar);
    }
    else
    {
        g_flow_alt_factor = 1.0f;
    }
    g_flow_alt_meters = estimateAltitudeMeters(g_flow_btps_amb_mbar);

    /* Keep Vivo table altitude factor in lock-step with orifice density. */
    LeakBtps_SetAmbientMbar(g_flow_btps_amb_mbar);
    LeakBtps_SetAltitudeEnable(g_flow_alt_enable != 0U);
}

static float applyDensityIfEnabled(float q_lut_slm)
{
    if (g_flow_alt_enable == 0U)
    {
        return q_lut_slm;
    }
    return q_lut_slm * g_flow_alt_factor;
}

static float applyBtpsIfEnabled(float q_slm_std)
{
    if (g_flow_btps_enable == 0U)
    {
        return q_slm_std;
    }
    return q_slm_std * g_flow_btps_factor;
}

float Flow_DpToSlm(float dp_corrected_mbar)
{
    const FlowCalPoint *first = &kFlowCal[0];
    const FlowCalPoint *next  = &kFlowCal[1];
    const FlowCalPoint *last  = &kFlowCal[FLOW_CAL_N - 1U];
    const FlowCalPoint *prev  = &kFlowCal[FLOW_CAL_N - 2U];

    if (dp_corrected_mbar <= first->dp_mbar)
    {
        const float slope = (next->q_slm - first->q_slm)
                          / (next->dp_mbar - first->dp_mbar);
        return first->q_slm + slope * (dp_corrected_mbar - first->dp_mbar);
    }

    if (dp_corrected_mbar >= last->dp_mbar)
    {
        const float slope = (last->q_slm - prev->q_slm)
                          / (last->dp_mbar - prev->dp_mbar);
        return last->q_slm + slope * (dp_corrected_mbar - last->dp_mbar);
    }

    for (size_t i = 0U; i < (FLOW_CAL_N - 1U); ++i)
    {
        const FlowCalPoint *a = &kFlowCal[i];
        const FlowCalPoint *b = &kFlowCal[i + 1U];
        if (dp_corrected_mbar <= b->dp_mbar)
        {
            const float frac = (dp_corrected_mbar - a->dp_mbar)
                             / (b->dp_mbar - a->dp_mbar);
            return a->q_slm + frac * (b->q_slm - a->q_slm);
        }
    }

    return last->q_slm;
}

/* === Public API === */

void Flow_Init(void)
{
    g_flow_lpf_cutoff_hz    = 10.0f;
    g_flow_zero_offset_mbar = -0.033f;
    g_flow_btps_enable      = 0U;
    g_flow_btps_amb_mbar    = FLOW_BTPS_P_DEFAULT;
    g_flow_alt_enable       = 1U;
    refreshAmbientDerived();
    g_flow_dp_raw_mbar      = 0.0f;
    g_flow_dp_filt_mbar     = 0.0f;
    g_flow_dp_corr_mbar     = 0.0f;
    g_flow_q_slm_lut        = 0.0f;
    g_flow_q_slm_std        = 0.0f;
    g_flow_q_slm            = 0.0f;
    g_flow_q_slm_raw        = 0.0f;
    g_flow_lpf_alpha        = 0.0f;
    g_flow_step_count       = 0U;
    g_flow_az_enable           = 1U;
    g_flow_az_quiet_pp_mbar    = 0.025f;
    g_flow_az_slew_mbar_per_s  = 0.003f;
    g_flow_az_done_mbar        = 0.0005f;
    g_flow_az_quiet_ms         = 5000U;
    g_flow_az_boot_delay_ms    = 2000U;
    g_flow_az_min_latch_ms     = 120000U;
    g_flow_az_count            = 0U;
    g_flow_az_latched          = 0U;
    g_flow_az_quiet_now        = 0U;
    g_flow_az_slewing          = 0U;
    g_flow_az_target_mbar      = 0.0f;
    g_flow_az_last_pp_mbar     = 0.0f;
    g_flow_az_last_q_pp_slm    = 0.0f;
    s_filt_seeded              = false;
    s_az_elapsed_ms            = 0U;
    s_az_quiet_ms              = 0U;
    s_az_latch_ms              = 0U;
    s_az_win_ms                = 0U;
    s_az_win_seeded            = false;
    s_az_mean_seeded           = false;
    s_az_mean_dp               = 0.0f;
    s_az_cmd_slew              = 0U;
    s_az_block_ms              = 0U;
    s_az_tile_ticks            = 0U;
    s_az_tile_fresh            = 0U;
    s_az_tile_sum              = 0.0f;
    s_dp_hist_n                = 0U;
    s_dp_last_good             = 0.0f;
    s_q_last_good              = 0.0f;
    s_reject_dp                = 0.0f;
    s_slew_rejects             = 0U;
    s_agree_rejects            = 0U;
    g_flow_median_taps         = 5U;
    g_flow_dp_max_slew_mbar_per_s = 50.0f;
    g_flow_slew_max_reject     = 3U;
    g_flow_slew_reject_count   = 0U;
    g_flow_q_max_step_slm      = 6.0f;
    g_flow_impulse_reject_count = 0U;
    s_spike_armed              = true;
    g_flow_spike_thresh_slm    = 5.0f;
    g_flow_spike_count         = 0U;
    g_flow_ms                  = 0U;
    g_flow_spike_ms            = 0U;
    g_flow_spike_gap_ms        = 0U;
    g_flow_median_enable       = 1U;
    g_flow_median_reject_count = 0U;
    g_flow_stale_now           = 1U; /* no input yet */
    g_flow_sample_saturated    = 0U;
    g_flow_stale_count         = 0U;
    g_flow_stale_run           = 0U;
    g_flow_stale_run_max       = 0U;
    LeakBtps_Init();
    refreshAmbientDerived();
}

void Flow_SetLpfCutoff(float hz)
{
    g_flow_lpf_cutoff_hz = clampf(hz,
                                  FLOW_LPF_CUTOFF_MIN_HZ,
                                  FLOW_LPF_CUTOFF_MAX_HZ);
}

float Flow_GetLpfCutoff(void)       { return g_flow_lpf_cutoff_hz; }
float Flow_GetDpRaw(void)           { return g_flow_dp_raw_mbar; }
float Flow_GetDpFiltered(void)      { return g_flow_dp_filt_mbar; }
float Flow_GetDpFilteredCorr(void)  { return g_flow_dp_corr_mbar; }
float Flow_GetSlm(void)             { return g_flow_q_slm; }
float Flow_GetSlmStd(void)          { return g_flow_q_slm_std; }
float Flow_GetSlmRaw(void)          { return g_flow_q_slm_raw; }
float Flow_GetPatientSlm(void)      { return Leak_GetPatientSlm(); }
float Flow_GetPatientRawSlm(void)   { return g_flow_patient_raw_slm; }
float Flow_GetZeroOffset(void)      { return g_flow_zero_offset_mbar; }

/* RBJ constant-peak-gain bandpass. Q=2 → ~2 Hz BW at 4 Hz so the FOT
 * carrier is kept and pause-DC / breath leftover / sensor hash are not. */
#define FLOW_FOT_BP_Q  2.0f

void Flow_FotBpReset(FlowFotBp_t *s)
{
    if (s == NULL)
    {
        return;
    }
    s->x1 = 0.0f;
    s->x2 = 0.0f;
    s->y1 = 0.0f;
    s->y2 = 0.0f;
    s->seeded = 0U;
}

float Flow_FotBpStep(FlowFotBp_t *s, float x, float f_center_hz, float dt_s)
{
    if (s == NULL)
    {
        return 0.0f;
    }
    if (!(dt_s > 0.0f) || (dt_s > 0.1f))
    {
        return s->y1;
    }
    if (s->seeded == 0U)
    {
        s->x1 = x;
        s->x2 = x;
        s->y1 = 0.0f;
        s->y2 = 0.0f;
        s->seeded = 1U;
        return 0.0f;
    }

    const float fs = 1.0f / dt_s;
    const float f0 = clampf(f_center_hz, 1.0f, 0.45f * fs);
    const float w = FLOW_TWO_PI * f0 * dt_s;
    const float sn = sinf(w);
    const float cs = cosf(w);
    const float alpha = sn / (2.0f * FLOW_FOT_BP_Q);
    const float a0 = 1.0f + alpha;
    const float b0 = alpha / a0;
    const float b1 = 0.0f;
    const float b2 = -alpha / a0;
    const float a1 = (-2.0f * cs) / a0;
    const float a2 = (1.0f - alpha) / a0;

    const float y = (b0 * x) + (b1 * s->x1) + (b2 * s->x2)
                  - (a1 * s->y1) - (a2 * s->y2);
    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = y;
    return y;
}

static void refreshFlowFromOffset(void)
{
    const float zero = g_flow_zero_offset_mbar;
    g_flow_dp_corr_mbar = g_flow_dp_filt_mbar - zero;
    g_flow_q_slm_lut    = Flow_DpToSlm(g_flow_dp_corr_mbar);
    g_flow_q_slm_std    = applyDensityIfEnabled(g_flow_q_slm_lut);
    g_flow_q_slm        = applyBtpsIfEnabled(g_flow_q_slm_std);
    g_flow_q_slm_raw    = applyBtpsIfEnabled(
        applyDensityIfEnabled(Flow_DpToSlm(g_flow_dp_raw_mbar - zero)));
}

void Flow_SetZeroOffset(float mbar)
{
    /* Cal sheet zero is ~-0.033 mbar; clamp to a generous ±0.5 mbar
     * window so a typo in the watch window cannot break the flow chain. */
    const float q_before = g_flow_q_slm;
    g_flow_zero_offset_mbar = clampf(mbar, -0.5f, 0.5f);
    refreshFlowFromOffset();
    /* Offset trim moves Q_total without real flow. Nudge leak DC so
     * Q_patient does not show a needle (near-zero LUT ~100 slm/mbar). */
    Leak_NudgeUnintentional(g_flow_q_slm - q_before);
}

void Flow_AutoZero(void)
{
    /* Commanded trim: slew toward the current LPF ΔP. Still no snap. */
    g_flow_az_target_mbar = g_flow_dp_filt_mbar;
    g_flow_az_slewing = 1U;
    g_flow_az_latched = 0U;
    s_az_cmd_slew = 1U;
    s_az_latch_ms = 0U;
}

void Flow_SetAutoZeroEnable(bool en)
{
    g_flow_az_enable = en ? 1U : 0U;
}

bool Flow_IsAutoZeroEnabled(void)
{
    return (g_flow_az_enable != 0U);
}

uint32_t Flow_GetAutoZeroCount(void)
{
    return g_flow_az_count;
}

uint8_t Flow_GetAutoZeroLatched(void)
{
    return g_flow_az_latched;
}

static void autoZeroResetAcWindow(float dp)
{
    s_az_dp_min = dp;
    s_az_dp_max = dp;
    s_az_win_ms = 0U;
    s_az_tile_ticks = 0U;
    s_az_tile_fresh = 0U;
    s_az_tile_sum = 0.0f;
    s_az_win_seeded = true;
}

static void autoZeroSlewToward(float target_mbar, float dt_s)
{
    const float rate = clampf(g_flow_az_slew_mbar_per_s, 0.0005f, 0.020f);
    const float done = clampf(g_flow_az_done_mbar, 0.0005f, 0.020f);
    const float cur = g_flow_zero_offset_mbar;
    float err = target_mbar - cur;
    const float max_step = rate * dt_s;

    if (err > max_step)
    {
        err = max_step;
    }
    else if (err < -max_step)
    {
        err = -max_step;
    }

    Flow_SetZeroOffset(cur + err);
    g_flow_az_slewing = 1U;

    if (fabsf(target_mbar - g_flow_zero_offset_mbar) <= done)
    {
        g_flow_az_slewing = 0U;
        s_az_cmd_slew = 0U;
        if (g_flow_az_latched == 0U)
        {
            g_flow_az_count++;
        }
        g_flow_az_latched = 1U;
        s_az_latch_ms = 0U;
    }
}

void Flow_AutoZeroService(float dt_s, bool allow)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);
    if (dt_ms == 0U)
    {
        return;
    }

    if ((g_flow_az_enable == 0U) && (s_az_cmd_slew == 0U))
    {
        g_flow_az_slewing = 0U;
        return;
    }

    const float dp = g_flow_dp_filt_mbar;

    if (!allow)
    {
        /* Therapy running / blower spinning: the zero offset is frozen and
         * NO trim may run - not the idle service and not a commanded one.
         * A therapy-time trim would bias delivered flow and Q_patient. */
        if (s_az_block_ms < 0xFFFFFFF0U)
        {
            s_az_block_ms += dt_ms;
        }
        g_flow_az_slewing = 0U;
        g_flow_az_quiet_now = 0U;
        s_az_cmd_slew = 0U;
        /* Debounce so a brief blower-status glitch does not wipe the
         * quiet window we may be part-way through building. */
        if (s_az_block_ms >= 1000U)
        {
            s_az_elapsed_ms = 0U;
            s_az_quiet_ms = 0U;
            s_az_win_seeded = false;
            s_az_mean_seeded = false;
        }
        return;
    }

    s_az_block_ms = 0U;

    /* Commanded trim (USB "flow autozero"), idle only: slew, never snap. */
    if (s_az_cmd_slew != 0U)
    {
        autoZeroSlewToward(g_flow_az_target_mbar, dt_s);
        if (s_az_cmd_slew != 0U)
        {
            return;
        }
    }

    /* One-shot per idle: after settle, hold forever until armed again. */
    if (g_flow_az_latched != 0U)
    {
        g_flow_az_slewing = 0U;
        return;
    }

    if (s_az_elapsed_ms < 0xFFFFFFF0U)
    {
        s_az_elapsed_ms += dt_ms;
    }

    if (s_az_elapsed_ms < g_flow_az_boot_delay_ms)
    {
        autoZeroResetAcWindow(dp);
        s_az_quiet_ms = 0U;
        s_az_mean_seeded = false;
        return;
    }

    if (!s_az_win_seeded)
    {
        autoZeroResetAcWindow(dp);
        return;
    }

    s_az_tile_ticks++;
    if (g_flow_stale_now == 0U)
    {
        s_az_tile_fresh++;
        s_az_tile_sum += dp;
        if (dp < s_az_dp_min) { s_az_dp_min = dp; }
        if (dp > s_az_dp_max) { s_az_dp_max = dp; }
    }
    s_az_win_ms += dt_ms;

    if (s_az_win_ms >= 500U)
    {
        const float pp = s_az_dp_max - s_az_dp_min;
        g_flow_az_last_pp_mbar = pp;
        g_flow_az_last_q_pp_slm = 0.0f;

        /* A held/repeated ΔP has zero peak-peak, so require the tile to be
         * mostly fresh conversions before calling it quiet. Otherwise we
         * would trim the zero onto a stale sample. */
        const bool fresh_enough =
            (s_az_tile_ticks > 0U)
            && ((s_az_tile_fresh * 5U) >= (s_az_tile_ticks * 4U));

        const float pp_lim = clampf(g_flow_az_quiet_pp_mbar, 0.008f, 0.10f);
        const bool quiet = fresh_enough && (pp <= pp_lim);
        g_flow_az_quiet_now = quiet ? 1U : 0U;

        if (quiet)
        {
            /* True sample mean, not the midrange: (min+max)/2 is biased
             * whenever the noise is asymmetric, and that bias lands
             * straight on the zero offset. */
            const float mid = (s_az_tile_fresh > 0U)
                                  ? (s_az_tile_sum / (float)s_az_tile_fresh)
                                  : (0.5f * (s_az_dp_min + s_az_dp_max));
            if (!s_az_mean_seeded)
            {
                s_az_mean_dp = mid;
                s_az_mean_seeded = true;
            }
            else
            {
                /* ~2.5 s IIR on 0.5 s quiet tiles. */
                s_az_mean_dp = (0.8f * s_az_mean_dp) + (0.2f * mid);
            }
            s_az_quiet_ms += s_az_win_ms;
        }
        else
        {
            s_az_quiet_ms = 0U;
            s_az_mean_seeded = false;
            g_flow_az_slewing = 0U;
        }

        autoZeroResetAcWindow(dp);
    }

    const uint32_t need_ms = (g_flow_az_quiet_ms < 1000U) ? 1000U : g_flow_az_quiet_ms;
    if ((s_az_quiet_ms >= need_ms) && s_az_mean_seeded)
    {
        g_flow_az_target_mbar = s_az_mean_dp;
        autoZeroSlewToward(g_flow_az_target_mbar, dt_s);
    }
}

void Flow_AutoZeroArm(void)
{
    /* Call on therapy stop: allow one idle trim after the next quiet window. */
    g_flow_az_latched = 0U;
    s_az_latch_ms = 0U;
    s_az_quiet_ms = 0U;
    s_az_elapsed_ms = 0U;
    s_az_win_seeded = false;
    s_az_mean_seeded = false;
    s_az_tile_ticks = 0U;
    s_az_tile_fresh = 0U;
    s_az_tile_sum = 0.0f;
    g_flow_az_slewing = 0U;
    s_az_cmd_slew = 0U;
}

void Flow_SetBtpsEnable(bool en)
{
    g_flow_btps_enable = en ? 1U : 0U;
    refreshAmbientDerived();
    /* Re-apply on latest sample so GUI toggle takes effect immediately. */
    g_flow_q_slm = applyBtpsIfEnabled(g_flow_q_slm_std);
}

bool Flow_IsBtpsEnabled(void)
{
    return (g_flow_btps_enable != 0U);
}

void Flow_SetBtpsAmbientMbar(float mbar)
{
    /* Prefer live measAS.pressure from SensorTask; USB/debug may override
     * until the next AS sample (~10 Hz) refreshes it. */
    g_flow_btps_amb_mbar = clampf(mbar, 600.0f, 1100.0f);
    refreshAmbientDerived();
    g_flow_q_slm_std = applyDensityIfEnabled(g_flow_q_slm_lut);
    g_flow_q_slm = applyBtpsIfEnabled(g_flow_q_slm_std);
}

float Flow_GetBtpsAmbientMbar(void)
{
    return g_flow_btps_amb_mbar;
}

float Flow_GetBtpsFactor(void)
{
    return g_flow_btps_factor;
}

float Flow_SlmToBtps(float q_slm_std)
{
    refreshAmbientDerived();
    return q_slm_std * g_flow_btps_factor;
}

void Flow_SetAltitudeEnable(bool en)
{
    g_flow_alt_enable = en ? 1U : 0U;
    refreshAmbientDerived();
    g_flow_q_slm_std = applyDensityIfEnabled(g_flow_q_slm_lut);
    g_flow_q_slm = applyBtpsIfEnabled(g_flow_q_slm_std);
}

bool Flow_IsAltitudeEnabled(void)
{
    return (g_flow_alt_enable != 0U);
}

float Flow_GetAltitudeFactor(void)
{
    return (g_flow_alt_enable != 0U) ? g_flow_alt_factor : 1.0f;
}

float Flow_GetAltitudeMeters(void)
{
    return g_flow_alt_meters;
}

float Flow_GetAltitudeRefMbar(void)
{
    return FLOW_ALT_P_REF_MBAR;
}

/* Median of the first n entries of hist (n = 3 or 5). Selection up to the
 * middle only; n is tiny and fixed so this is a handful of compares. */
static float medianN(const float *hist, uint8_t n)
{
    float v[5];
    for (uint8_t i = 0U; i < n; i++)
    {
        v[i] = hist[i];
    }
    const uint8_t mid = (uint8_t)(n / 2U);
    for (uint8_t i = 0U; i <= mid; i++)
    {
        uint8_t m = i;
        for (uint8_t j = (uint8_t)(i + 1U); j < n; j++)
        {
            if (v[j] < v[m]) { m = j; }
        }
        const float t = v[i]; v[i] = v[m]; v[m] = t;
    }
    return v[mid];
}

void Flow_NoteStaleSample(void)
{
    /* Held / failed FS read: leave the LPF, the LUT outputs and the
     * auto-zero statistics untouched. Repeating a sample would look like
     * a perfectly quiet signal and then step when fresh data resumes. */
    g_flow_stale_now = 1U;
    g_flow_stale_count++;
    g_flow_stale_run++;
    if (g_flow_stale_run > g_flow_stale_run_max)
    {
        g_flow_stale_run_max = g_flow_stale_run;
    }
}

bool Flow_IsSampleStale(void)
{
    return (g_flow_stale_now != 0U);
}

uint32_t Flow_GetStaleRun(void)
{
    return g_flow_stale_run;
}

bool Flow_IsSampleSaturated(void)
{
    return (g_flow_sample_saturated != 0U);
}

void Flow_SpikeWatch(float q_patient_slm, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    g_flow_ms += (uint32_t)(dt_s * 1000.0f + 0.5f);

    const float thr = clampf(g_flow_spike_thresh_slm, 1.0f, 100.0f);
    const float aq = fabsf(q_patient_slm);

    if (s_spike_armed && (aq >= thr))
    {
        s_spike_armed = false;
        g_flow_spike_gap_ms   = g_flow_ms - g_flow_spike_ms;
        g_flow_spike_ms       = g_flow_ms;
        g_flow_spike_count++;
        g_flow_spike_q_patient = q_patient_slm;
        g_flow_spike_dp_raw   = g_flow_dp_raw_mbar;
        g_flow_spike_dp_filt  = g_flow_dp_filt_mbar;
        g_flow_spike_q_total  = g_flow_q_slm;
        g_flow_spike_vent     = Leak_GetVentSlm();
        g_flow_spike_uninten  = Leak_GetUnintentionalSlm();
        g_flow_spike_zero     = g_flow_zero_offset_mbar;
        g_flow_spike_az_count = g_flow_az_count;
        g_flow_spike_stale    = g_flow_stale_count;
        g_flow_spike_medrej   = g_flow_median_reject_count;
    }
    else if (aq < (0.5f * thr))
    {
        s_spike_armed = true;
    }
}

void Flow_Update(float dp_mbar_raw, float dt_s, bool saturated)
{
    /* Same dt sanity guard pattern as the rest of the control code. A
     * missed tick is not new data either, so account for it as stale. */
    if (!(dt_s > 0.0f) || dt_s > 0.1f || !isfinite(dp_mbar_raw))
    {
        Flow_NoteStaleSample();
        return;
    }

    g_flow_stale_now = 0U;
    g_flow_stale_run = 0U;
    g_flow_sample_saturated = saturated ? 1U : 0U;

    g_flow_dp_raw_mbar = dp_mbar_raw;

    /* Gross ΔP tear (corrupted upper byte, whole mbar). Isolated outliers
     * freeze the LPF. A real step (mask-off) persists: three consecutive
     * samples that agree with each other are accepted and then rate-limited
     * in Q below. The old "resync after N rejects" published uncorrelated
     * garbage and the 2 Hz patient LPF printed it as a ~12 slm needle. */
    const float max_dp = clampf(g_flow_dp_max_slew_mbar_per_s, 1.0f, 500.0f) * dt_s;
    const uint8_t agree_n = (g_flow_slew_max_reject < 2U) ? 2U : g_flow_slew_max_reject;
    if ((s_dp_hist_n > 0U)
        && (fabsf(dp_mbar_raw - s_dp_last_good) > max_dp))
    {
        if ((s_slew_rejects > 0U)
            && (fabsf(dp_mbar_raw - s_reject_dp) <= max_dp))
        {
            s_agree_rejects++;
        }
        else
        {
            s_agree_rejects = 1U;
        }
        s_reject_dp = dp_mbar_raw;
        s_slew_rejects++;
        g_flow_slew_reject_count++;
        if (s_agree_rejects < agree_n)
        {
            Flow_NoteStaleSample();
            return;
        }
        s_slew_rejects = 0U;
        s_agree_rejects = 0U;
    }
    else
    {
        s_slew_rejects = 0U;
        s_agree_rejects = 0U;
    }

    /* Median spike guard: drop 1–2 tick tears the ΔP guard let through. */
    float dp_in = dp_mbar_raw;
    for (uint8_t i = 4U; i > 0U; i--)
    {
        s_dp_hist[i] = s_dp_hist[i - 1U];
    }
    s_dp_hist[0] = dp_mbar_raw;
    if (s_dp_hist_n < 5U)
    {
        s_dp_hist_n++;
    }
    const uint8_t taps = (g_flow_median_taps >= 5U) ? 5U : 3U;
    if ((g_flow_median_enable != 0U) && (s_dp_hist_n >= taps))
    {
        dp_in = medianN(s_dp_hist, taps);
        if (dp_in != dp_mbar_raw)
        {
            g_flow_median_reject_count++;
        }
    }

    /* Flow-unit impulse cap. Near zero the LUT is ~100 slm/mbar, so a
     * 0.12 mbar tear that passed ΔP slew is still 12 slm — and a 2 mbar
     * tear is 200 slm, which the 2 Hz patient LPF turns into the classic
     * 12 slm needle (α ≈ 0.06 × 200). Climb at most q_max_step per sample. */
    {
        const float q_in = Flow_DpToSlm(dp_in - g_flow_zero_offset_mbar);
        const float max_q = clampf(g_flow_q_max_step_slm, 1.0f, 80.0f);
        if ((s_dp_hist_n > 1U) && (fabsf(q_in - s_q_last_good) > max_q))
        {
            const float q_lim = (q_in > s_q_last_good)
                                    ? (s_q_last_good + max_q)
                                    : (s_q_last_good - max_q);
            /* Invert through the local slope so the LPF sees a 6 slm step
             * rather than the torn ΔP. Near zero that is ~0.06 mbar. */
            const float q0 = Flow_DpToSlm(s_dp_last_good - g_flow_zero_offset_mbar);
            const float dq = q_in - q0;
            if (fabsf(dq) > 0.01f)
            {
                const float frac = (q_lim - q0) / dq;
                dp_in = s_dp_last_good + frac * (dp_in - s_dp_last_good);
            }
            else
            {
                dp_in = s_dp_last_good;
            }
            g_flow_impulse_reject_count++;
        }
        s_q_last_good = Flow_DpToSlm(dp_in - g_flow_zero_offset_mbar);
    }
    s_dp_last_good = dp_in;
    dp_mbar_raw = dp_in;

    /* Compute alpha from the *currently configured* cutoff every tick.
     * That's only ~5 float ops; doing it every call lets the user retune
     * fc from the watch window without an explicit "apply" step. */
    const float fc = clampf(g_flow_lpf_cutoff_hz,
                            FLOW_LPF_CUTOFF_MIN_HZ,
                            FLOW_LPF_CUTOFF_MAX_HZ);
    const float omega_dt = FLOW_TWO_PI * fc * dt_s;
    const float alpha    = omega_dt / (1.0f + omega_dt);
    g_flow_lpf_alpha = alpha;

    /* First-sample seed: rather than ramping from 0 (which can take ~50 ms
     * to settle and momentarily looks like a transient), snap the
     * filter state to the first valid input. */
    if (!s_filt_seeded)
    {
        g_flow_dp_filt_mbar = dp_mbar_raw;
        s_filt_seeded       = true;
    }
    else
    {
        g_flow_dp_filt_mbar = alpha * dp_mbar_raw
                            + (1.0f - alpha) * g_flow_dp_filt_mbar;
    }

    /* Zero-offset → LUT (sea-level cal) → density → optional BTPS. */
    refreshFlowFromOffset();

    g_flow_step_count++;
}
