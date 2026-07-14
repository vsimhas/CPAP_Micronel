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

#include <math.h>
#include <stddef.h>

#define FLOW_TWO_PI   6.28318530717958647692f

/* === Public configuration (debugger-visible volatiles) === */
volatile float    g_flow_lpf_cutoff_hz   = 15.0f;      /* tunable cutoff    */
volatile float    g_flow_zero_offset_mbar = -0.025f;   /* new flow element  */

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
 *  New flow element bench data (user-supplied, 2026-07). Raw ΔP is the LPF
 *  output g_flow_dp_filt_mbar; zero flow reads -0.025 mbar. After subtracting
 *  the configured zero offset, the table below is ΔP_corrected -> slm.
 *
 *      raw ΔP (mbar)  ΔP_corr (mbar)   Q (slm)
 *      ---------------------------------------------
 *      -6.070         -6.045           -99.70
 *      -5.170         -5.145           -91.70
 *      -4.010         -3.985           -80.00
 *      -2.940         -2.915           -68.20
 *      -2.030         -2.005           -54.00
 *      -1.320         -1.295           -41.40
 *      -0.736         -0.711           -28.90
 *      -0.335         -0.310           -16.70
 *      -0.108         -0.083            -6.58
 *      -0.065         -0.040            -3.70
 *      -0.036         -0.011            -1.20
 *      -0.025          0.000             0.00
 *      -0.016          0.009             1.233
 *       0.014          0.039             3.80
 *       0.058          0.083             6.75
 *       0.314          0.339            17.20
 *       0.801          0.826            29.75
 *       1.530          1.555            42.70
 *       2.570          2.595            55.70
 *       3.900          3.925            70.00
 *       5.460          5.485            82.25
 *       6.020          6.045            86.00
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
    { -6.045f,  -99.70f },
    { -5.145f,  -91.70f },
    { -3.985f,  -80.00f },
    { -2.915f,  -68.20f },
    { -2.005f,  -54.00f },
    { -1.295f,  -41.40f },
    { -0.711f,  -28.90f },
    { -0.310f,  -16.70f },
    { -0.083f,   -6.58f },
    { -0.040f,   -3.70f },
    { -0.011f,   -1.20f },
    {  0.000f,    0.00f },
    {  0.009f,    1.233f },
    {  0.039f,    3.80f },
    {  0.083f,    6.75f },
    {  0.339f,   17.20f },
    {  0.826f,   29.75f },
    {  1.555f,   42.70f },
    {  2.595f,   55.70f },
    {  3.925f,   70.00f },
    {  5.485f,   82.25f },
    {  6.045f,   86.00f },
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
    g_flow_zero_offset_mbar = -0.025f;
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
float Flow_GetPatientRawSlm(void)   { return g_flow_patient_raw_slm; }
float Flow_GetZeroOffset(void)      { return g_flow_zero_offset_mbar; }

void Flow_SetZeroOffset(float mbar)
{
    /* Cal sheet zero is ~-0.025 mbar; clamp to a generous ±0.5 mbar
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
