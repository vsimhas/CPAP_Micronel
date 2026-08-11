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

/* SFM3300-250-D reference is SLM at 20 °C / 1013.25 mbar.
 * BTPS: 37 °C, ambient P, water vapor saturated (PH2O ≈ 62.67 mbar). */
#define FLOW_BTPS_T_K           310.15f
#define FLOW_SFM_T_K            293.15f   /* 20 °C Sensirion cal           */
#define FLOW_BTPS_PH2O_MBAR     62.67f    /* saturated vapor @ 37 °C       */
#define FLOW_BTPS_P_DEFAULT     1013.25f

/* === Public configuration (debugger-visible volatiles) === */
volatile float    g_flow_lpf_cutoff_hz   = 15.0f;      /* tunable cutoff    */
volatile float    g_flow_zero_offset_mbar = -0.029f;   /* Round Hole_2 zero */

/* Optional SLM (SFM3300) → BTPS scaling applied after the LUT.
 * Ambient P defaults to 1013.25 mbar and is overwritten live from
 * measAS.pressure (AMS5935 absolute, mbar) in SensorTask @ ~10 Hz. */
volatile uint8_t  g_flow_btps_enable     = 0U;         /* 0=SLM, 1=BTPS     */
volatile float    g_flow_btps_amb_mbar   = FLOW_BTPS_P_DEFAULT;
volatile float    g_flow_btps_factor     = 1.128f;     /* cached scale       */

/* === State (debugger-visible volatiles) === */
volatile float    g_flow_dp_raw_mbar   = 0.0f;
volatile float    g_flow_dp_filt_mbar  = 0.0f;
volatile float    g_flow_dp_corr_mbar  = 0.0f;        /* filtered minus zero */
volatile float    g_flow_q_slm_std     = 0.0f;        /* LUT output (SFM SLM) */
volatile float    g_flow_q_slm         = 0.0f;        /* SLM or BTPS (active) */
volatile float    g_flow_q_slm_raw     = 0.0f;        /* from raw ΔP (active) */
volatile float    g_flow_lpf_alpha     = 0.0f;        /* read-only output  */
volatile uint32_t g_flow_step_count    = 0U;

/* === Internal === */
static bool s_filt_seeded = false;       /* first sample bypasses filter */

/* === Calibration table =====================================================
 *
 *  Round Hole_2 element, bench vs SFM3300-250-D (SLM @ 20 °C / 1013 mbar).
 *  Spreadsheet column 1 is g_flow_dp_filt_mbar (raw filtered ΔP).
 *  Zero flow reads -0.029 mbar → g_flow_zero_offset_mbar = -0.029.
 *  Table below stores ΔP_corrected = filt − zero → Q_SLM.
 *
 *      filt ΔP (mbar)  ΔP_corr (mbar)   Q (slm SFM)
 *      ------------------------------------------------
 *      -4.820          -4.791           -146.00
 *      -4.120          -4.091           -133.50
 *      -3.490          -3.461           -119.50
 *      -2.940          -2.911           -106.50
 *      -2.360          -2.331            -93.75
 *      -1.920          -1.891            -81.25
 *      -1.460          -1.431            -68.75
 *      -1.090          -1.061            -54.00
 *      -0.730          -0.701            -41.00
 *      -0.442          -0.413            -27.30
 *      -0.236          -0.207            -15.80
 *      -0.0999         -0.0709            -6.15
 *      -0.066          -0.037             -3.368
 *      -0.052          -0.023             -2.10
 *      -0.041          -0.012             -1.045
 *      -0.029           0.000              0.00
 *      -0.019           0.010              1.10
 *      -0.008           0.021              2.21
 *       0.0056          0.0346             3.47
 *       0.037           0.066              6.35
 *       0.163           0.192             16.225
 *       0.353           0.382             28.10
 *       0.602           0.631             41.75
 *       0.909           0.938             55.00
 *       1.270           1.299             70.00
 *       1.690           1.719             82.50
 *       2.140           2.169             95.00
 *       2.620           2.649            108.00
 *       3.190           3.219            121.00
 *       3.780           3.809            134.50
 *       4.440           4.469            146.50
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
    { -4.791f,  -146.00f },
    { -4.091f,  -133.50f },
    { -3.461f,  -119.50f },
    { -2.911f,  -106.50f },
    { -2.331f,   -93.75f },
    { -1.891f,   -81.25f },
    { -1.431f,   -68.75f },
    { -1.061f,   -54.00f },
    { -0.701f,   -41.00f },
    { -0.413f,   -27.30f },
    { -0.207f,   -15.80f },
    { -0.0709f,   -6.15f },
    { -0.037f,    -3.368f },
    { -0.023f,    -2.10f },
    { -0.012f,    -1.045f },
    {  0.000f,     0.00f },
    {  0.010f,     1.10f },
    {  0.021f,     2.21f },
    {  0.0346f,    3.47f },
    {  0.066f,     6.35f },
    {  0.192f,    16.225f },
    {  0.382f,    28.10f },
    {  0.631f,    41.75f },
    {  0.938f,    55.00f },
    {  1.299f,    70.00f },
    {  1.719f,    82.50f },
    {  2.169f,    95.00f },
    {  2.649f,   108.00f },
    {  3.219f,   121.00f },
    {  3.809f,   134.50f },
    {  4.469f,   146.50f },
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

static float applyBtpsIfEnabled(float q_slm_std)
{
    if (g_flow_btps_enable == 0U)
    {
        return q_slm_std;
    }
    g_flow_btps_factor = computeBtpsFactor();
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
    g_flow_lpf_cutoff_hz    = 15.0f;
    g_flow_zero_offset_mbar = -0.029f;
    g_flow_btps_enable      = 0U;
    g_flow_btps_amb_mbar    = FLOW_BTPS_P_DEFAULT;
    g_flow_btps_factor      = computeBtpsFactor();
    g_flow_dp_raw_mbar      = 0.0f;
    g_flow_dp_filt_mbar     = 0.0f;
    g_flow_dp_corr_mbar     = 0.0f;
    g_flow_q_slm_std        = 0.0f;
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
float Flow_GetSlmStd(void)          { return g_flow_q_slm_std; }
float Flow_GetSlmRaw(void)          { return g_flow_q_slm_raw; }
float Flow_GetPatientSlm(void)      { return Leak_GetPatientSlm(); }
float Flow_GetPatientRawSlm(void)   { return g_flow_patient_raw_slm; }
float Flow_GetZeroOffset(void)      { return g_flow_zero_offset_mbar; }

void Flow_SetZeroOffset(float mbar)
{
    /* Cal sheet zero is ~-0.029 mbar; clamp to a generous ±0.5 mbar
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

void Flow_SetBtpsEnable(bool en)
{
    g_flow_btps_enable = en ? 1U : 0U;
    g_flow_btps_factor = computeBtpsFactor();
    /* Re-apply on latest LUT sample so GUI toggle takes effect immediately. */
    g_flow_q_slm = applyBtpsIfEnabled(g_flow_q_slm_std);
}

bool Flow_IsBtpsEnabled(void)
{
    return (g_flow_btps_enable != 0U);
}

void Flow_SetBtpsAmbientMbar(float mbar)
{
    /* Prefer live measAS.pressure from SensorTask; USB/debug may override
     * until the next AS sample (~10 Hz) refreshes it. Next Flow_Update
     * refreshes both filtered and raw active flows with the new factor. */
    g_flow_btps_amb_mbar = clampf(mbar, 600.0f, 1100.0f);
    g_flow_btps_factor = computeBtpsFactor();
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
    g_flow_btps_factor = computeBtpsFactor();
    return q_slm_std * g_flow_btps_factor;
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

    /* Apply zero-offset correction and calibration LUT (SFM3300 SLM).
     * Optional BTPS scaling is applied after the LUT. */
    const float zero  = g_flow_zero_offset_mbar;
    g_flow_dp_corr_mbar = g_flow_dp_filt_mbar - zero;
    g_flow_q_slm_std    = Flow_DpToSlm(g_flow_dp_corr_mbar);
    g_flow_q_slm        = applyBtpsIfEnabled(g_flow_q_slm_std);
    g_flow_q_slm_raw    = applyBtpsIfEnabled(Flow_DpToSlm(dp_mbar_raw - zero));

    g_flow_step_count++;
}
