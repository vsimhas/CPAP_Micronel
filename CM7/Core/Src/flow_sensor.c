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

#include <math.h>
#include <stddef.h>

#define FLOW_TWO_PI   6.28318530717958647692f

/* === Public configuration (debugger-visible volatiles) === */
volatile float    g_flow_lpf_cutoff_hz   = 10.0f;      /* tunable cutoff    */
volatile float    g_flow_zero_offset_mbar = -0.026f;   /* default from cal  */

/* === State (debugger-visible volatiles) === */
volatile float    g_flow_dp_raw_mbar   = 0.0f;
volatile float    g_flow_dp_filt_mbar  = 0.0f;
volatile float    g_flow_dp_corr_mbar  = 0.0f;        /* filtered minus zero */
volatile float    g_flow_q_slm         = 0.0f;        /* from filtered ΔP   */
volatile float    g_flow_q_slm_raw     = 0.0f;        /* from raw ΔP        */
volatile float    g_flow_lpf_alpha     = 0.0f;        /* read-only output  */
volatile uint32_t g_flow_step_count    = 0U;

/* === Internal === */
static bool s_filt_seeded = false;       /* first sample bypasses filter */

/* === Calibration table =====================================================
 *
 *  Bench data (user-supplied), with the static -0.026 mbar zero offset
 *  already subtracted so the table is purely "ΔP_corrected -> slm".
 *  Entries must be sorted ascending by ΔP and start at exactly (0, 0).
 *
 *      raw ΔP (mbar)  ΔP_corr (mbar)   Q (slm)
 *      ---------------------------------------------
 *      -0.026          0.000             0.0
 *      -0.020          0.006             6.3
 *       0.033          0.059            18.7
 *       0.134          0.160            33.5
 *       0.320          0.346            49.5
 *       0.600          0.626            67.0
 *       0.910          0.936            82.0
 *       1.330          1.356            97.5
 *       1.890          1.916           114.0
 *       2.400          2.426           132.0
 *       3.100          3.126           150.0
 *       3.700          3.726           165.0
 *       4.400          4.426           181.0
 *
 *  The curve is approximately Q ~ 84.85 * sqrt(ΔP_corr), i.e. an orifice
 *  law. At each measured point the LUT reproduces the user value
 *  exactly; between points we linearly interpolate. Beyond the highest
 *  point the last segment slope is used to extrapolate.
 *
 *  Negative ΔP (which can occur with reverse flow or just sensor noise
 *  near zero) is mirrored about the origin: an orifice is symmetric.
 */
typedef struct
{
    float dp_mbar;     /* ΔP after zero-offset correction */
    float q_slm;
} FlowCalPoint;

static const FlowCalPoint kFlowCal[] =
{
    {  0.000f,    0.0f },
    {  0.006f,    6.3f },
    {  0.059f,   18.7f },
    {  0.160f,   33.5f },
    {  0.346f,   49.5f },
    {  0.626f,   67.0f },
    {  0.936f,   82.0f },
    {  1.356f,   97.5f },
    {  1.916f,  114.0f },
    {  2.426f,  132.0f },
    {  3.126f,  150.0f },
    {  3.726f,  165.0f },
    {  4.426f,  181.0f },
};

#define FLOW_CAL_N   (sizeof(kFlowCal) / sizeof(kFlowCal[0]))

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Interpolate the cal LUT for a non-negative ΔP. Linear between points,
 * linear-extrapolated beyond the last point using the slope of the
 * last segment. Returns 0 for ΔP <= 0; bidirectional handling lives in
 * Flow_DpToSlm() one level up. */
static float interpDpToQ(float dp_corr)
{
    if (dp_corr <= 0.0f)
    {
        return 0.0f;
    }

    /* Above the table - extrapolate using the last segment's slope. */
    const FlowCalPoint *last  = &kFlowCal[FLOW_CAL_N - 1U];
    const FlowCalPoint *plast = &kFlowCal[FLOW_CAL_N - 2U];
    if (dp_corr >= last->dp_mbar)
    {
        const float slope = (last->q_slm - plast->q_slm)
                          / (last->dp_mbar - plast->dp_mbar);
        return last->q_slm + slope * (dp_corr - last->dp_mbar);
    }

    /* Inside the table - linear scan is fine for 13 entries. */
    for (size_t i = 0U; i < (FLOW_CAL_N - 1U); ++i)
    {
        const FlowCalPoint *a = &kFlowCal[i];
        const FlowCalPoint *b = &kFlowCal[i + 1U];
        if (dp_corr <= b->dp_mbar)
        {
            const float frac = (dp_corr - a->dp_mbar)
                             / (b->dp_mbar - a->dp_mbar);
            return a->q_slm + frac * (b->q_slm - a->q_slm);
        }
    }

    /* Should be unreachable given the bounds above, but be defensive. */
    return last->q_slm;
}

float Flow_DpToSlm(float dp_corrected_mbar)
{
    /* Symmetric orifice: mirror the curve through the origin so a
     * momentary negative reading (sensor noise near 0 or reverse flow)
     * yields a sensible negative slm rather than 0. */
    if (dp_corrected_mbar >= 0.0f)
    {
        return interpDpToQ(dp_corrected_mbar);
    }
    return -interpDpToQ(-dp_corrected_mbar);
}

/* === Public API === */

void Flow_Init(void)
{
    g_flow_lpf_cutoff_hz    = 10.0f;
    g_flow_zero_offset_mbar = -0.026f;
    g_flow_dp_raw_mbar      = 0.0f;
    g_flow_dp_filt_mbar     = 0.0f;
    g_flow_dp_corr_mbar     = 0.0f;
    g_flow_q_slm            = 0.0f;
    g_flow_q_slm_raw        = 0.0f;
    g_flow_lpf_alpha        = 0.0f;
    g_flow_step_count       = 0U;
    s_filt_seeded           = false;
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
float Flow_GetSlmRaw(void)          { return g_flow_q_slm_raw; }
float Flow_GetZeroOffset(void)      { return g_flow_zero_offset_mbar; }

void Flow_SetZeroOffset(float mbar)
{
    /* Cal sheet zero is ~-0.026 mbar; clamp to a generous ±0.5 mbar
     * window so a typo in the watch window cannot break the flow chain. */
    g_flow_zero_offset_mbar = clampf(mbar, -0.5f, 0.5f);
}

void Flow_AutoZero(void)
{
    /* Snap the offset to whatever the LPF currently reads. Caller is
     * responsible for ensuring the blower is off / flow is zero when
     * this is invoked - we have no way to verify that here. */
    Flow_SetZeroOffset(g_flow_dp_filt_mbar);
}

void Flow_Update(float dp_mbar_raw, float dt_s)
{
    /* Same dt sanity guard pattern as the rest of the control code. */
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    g_flow_dp_raw_mbar = dp_mbar_raw;

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

    /* Apply zero-offset correction and calibration LUT. We compute Q
     * for both the filtered and the raw signals: filtered is what the
     * therapy/diagnostics layers should consume; raw is exposed for
     * future band-pass / FOT processing. */
    const float zero  = g_flow_zero_offset_mbar;
    g_flow_dp_corr_mbar = g_flow_dp_filt_mbar - zero;
    g_flow_q_slm        = Flow_DpToSlm(g_flow_dp_corr_mbar);
    g_flow_q_slm_raw    = Flow_DpToSlm(dp_mbar_raw - zero);

    g_flow_step_count++;
}
