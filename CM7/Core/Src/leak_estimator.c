/**
  ******************************************************************************
  * @file    leak_estimator.c
  * @brief   Implementation: total - vent, slow-LPF, threshold compare.
  ******************************************************************************
  */

#include "leak_estimator.h"

#include <math.h>

/* === Public configuration (debugger-writable volatiles) === */
volatile float    g_leak_vent_k         = 11.0f;       /* slm / sqrt(cmH2O) */
volatile float    g_leak_tau_seconds    = 10.0f;       /* slow LPF time const*/
volatile float    g_leak_high_threshold = 24.0f;       /* slm                */

/* === State (debugger-readable volatiles) === */
volatile float    g_leak_total_slm      = 0.0f;
volatile float    g_leak_vent_slm       = 0.0f;
volatile float    g_leak_uninten_slm    = 0.0f;
volatile uint8_t  g_leak_is_high        = 0U;
volatile uint32_t g_leak_step_count     = 0U;

/* === Internal === */
static bool s_seeded = false;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* === Public API === */

void Leak_Init(void)
{
    g_leak_vent_k         = 11.0f;
    g_leak_tau_seconds    = 10.0f;
    g_leak_high_threshold = 24.0f;

    g_leak_total_slm   = 0.0f;
    g_leak_vent_slm    = 0.0f;
    g_leak_uninten_slm = 0.0f;
    g_leak_is_high     = 0U;
    g_leak_step_count  = 0U;
    s_seeded           = false;
}

void Leak_SetVentCoefficient(float k)
{
    g_leak_vent_k = clampf(k, 5.0f, 20.0f);
}

void Leak_SetTimeConstant(float seconds)
{
    g_leak_tau_seconds = clampf(seconds, 1.0f, 60.0f);
}

void Leak_SetHighThreshold(float slm)
{
    g_leak_high_threshold = clampf(slm, 5.0f, 60.0f);
}

float Leak_GetVentCoefficient(void) { return g_leak_vent_k; }
float Leak_GetTimeConstant(void)    { return g_leak_tau_seconds; }
float Leak_GetHighThreshold(void)   { return g_leak_high_threshold; }
float Leak_GetTotalSlm(void)        { return g_leak_total_slm; }
float Leak_GetVentSlm(void)         { return g_leak_vent_slm; }
float Leak_GetUnintentionalSlm(void){ return g_leak_uninten_slm; }
bool  Leak_IsHigh(void)             { return (g_leak_is_high != 0U); }

void Leak_Reseed(void)
{
    /* Next Leak_Update() will snap the slow LPF to its instantaneous
     * input value, skipping the ~30 s warm-up. */
    s_seeded = false;
}

void Leak_Update(float total_q_slm, float mask_pressure_cmh2o, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    g_leak_total_slm = total_q_slm;

    /* Modelled vent flow. Negative pressure makes no physical sense for
     * the vent (the orifice is one-way for our purposes), so clamp. */
    const float p = (mask_pressure_cmh2o > 0.0f) ? mask_pressure_cmh2o : 0.0f;
    g_leak_vent_slm = g_leak_vent_k * sqrtf(p);

    /* Instantaneous "everything that isn't vent" flow. Over a complete
     * breath cycle this averages to Q_leak; cycle-by-cycle it swings
     * with tidal volume. The slow LPF below extracts the DC component. */
    const float instant_excess = total_q_slm - g_leak_vent_slm;

    /* Slow first-order IIR. tau = 10 s with 5 ms tick gives:
     *      alpha = dt / (tau + dt) = 0.005 / 10.005 = 4.998e-4
     * which is small (long memory) - exactly what we want for averaging
     * out the breath cycle. We use the bilinear form to stay accurate
     * if dt or tau is poked from the watch window. */
    const float tau = clampf(g_leak_tau_seconds, 1.0f, 60.0f);
    const float alpha = dt_s / (tau + dt_s);

    if (!s_seeded)
    {
        /* Reseed: snap to the instantaneous excess so we don't have to
         * wait ~30 s for the LPF to climb out of zero on therapy start. */
        g_leak_uninten_slm = instant_excess;
        s_seeded = true;
    }
    else
    {
        g_leak_uninten_slm = alpha * instant_excess
                           + (1.0f - alpha) * g_leak_uninten_slm;
    }

    g_leak_is_high = (fabsf(g_leak_uninten_slm) > g_leak_high_threshold)
                       ? 1U : 0U;
    g_leak_step_count++;
}
