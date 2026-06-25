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

#include "leak_estimator.h"

#define FLOW_TWO_PI   6.28318530717958647692f

/* === Public configuration (debugger-visible volatiles) === */
volatile float    g_flow_lpf_cutoff_hz   = 10.0f;      /* tunable cutoff    */
volatile float    g_flow_zero_offset_mbar = -0.024f;   /* new flow element  */

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
 *  New flow element bench data (user-supplied). Raw ΔP table includes both
 *  flow directions and the zero point at -0.024 mbar. After subtracting the
 *  configured zero offset, the table below is ΔP_corrected -> slm.
 *
 *      raw ΔP (mbar)  ΔP_corr (mbar)   Q (slm)
 *      ---------------------------------------------
 *      -6.230         -6.206          -148.0
 *      -5.400         -5.376          -136.0
 *      -4.400         -4.376          -121.5
 *      -3.490         -3.466          -108.0
 *      -2.720         -2.696           -95.0
 *      -2.080         -2.056           -82.0
 *      -1.460         -1.436           -70.0
 *      -0.982         -0.958           -55.0
 *      -0.610         -0.586           -41.5
 *      -0.360         -0.336           -29.0
 *      -0.150         -0.126           -17.0
 *      -0.040         -0.016            -6.75
 *      -0.024          0.000             0.0
 *       0.010          0.034             6.8
 *       0.081          0.105            17.1
 *       0.185          0.209            29.4
 *       0.418          0.442            42.2
 *       0.735          0.759            54.0
 *       1.200          1.224            70.0
 *       1.710          1.734            82.5
 *       2.340          2.364            95.5
 *       3.130          3.154           108.0
 *       3.900          3.924           121.0
 *       4.850          4.874           135.0
 *       5.780          5.804           148.0
 *
 *  Curve follows orifice law Q ~ K*sqrt(ΔP_corr) with K ~ 61-64 (vs ~84
 *  on the previous element). LUT reproduces measured points exactly;
 *  linear interpolation between, end-segment slope beyond table limits.
 */
typedef struct
{
    float dp_mbar;     /* ΔP after zero-offset correction */
    float q_slm;
} FlowCalPoint;

static const FlowCalPoint kFlowCal[] =
{
    { -6.206f, -148.0f },
    { -5.376f, -136.0f },
    { -4.376f, -121.5f },
    { -3.466f, -108.0f },
    { -2.696f,  -95.0f },
    { -2.056f,  -82.0f },
    { -1.436f,  -70.0f },
    { -0.958f,  -55.0f },
    { -0.586f,  -41.5f },
    { -0.336f,  -29.0f },
    { -0.126f,  -17.0f },
    { -0.016f,   -6.75f },
    {  0.000f,    0.0f },
    {  0.034f,    6.8f },
    {  0.105f,   17.1f },
    {  0.209f,   29.4f },
    {  0.442f,   42.2f },
    {  0.759f,   54.0f },
    {  1.224f,   70.0f },
    {  1.734f,   82.5f },
    {  2.364f,   95.5f },
    {  3.154f,  108.0f },
    {  3.924f,  121.0f },
    {  4.874f,  135.0f },
    {  5.804f,  148.0f },
};

#define FLOW_CAL_N   (sizeof(kFlowCal) / sizeof(kFlowCal[0]))

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

    /* Inside the table - linear scan is fine for 25 entries. */
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

    /* Should be unreachable given the bounds above, but be defensive. */
    return last->q_slm;
}

/* === Public API === */

void Flow_Init(void)
{
    g_flow_lpf_cutoff_hz    = 10.0f;
    g_flow_zero_offset_mbar = -0.024f;
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
float Flow_GetPatientSlm(void)      { return Leak_GetPatientSlm(); }
float Flow_GetZeroOffset(void)      { return g_flow_zero_offset_mbar; }

void Flow_SetZeroOffset(float mbar)
{
    /* Cal sheet zero is ~-0.024 mbar; clamp to a generous ±0.5 mbar
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
