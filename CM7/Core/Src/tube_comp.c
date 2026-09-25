/**
  ******************************************************************************
  * @file    tube_comp.c
  * @brief   Implementation: dP_hose = R_lam*|Q| + R_turb*Q*|Q|.
  ******************************************************************************
  */

#include "tube_comp.h"

#include <math.h>

/* === Tunables (debugger-writable volatiles) ================================ */
volatile uint8_t g_tubecomp_enabled       = 1U;
/* 2026-09-22: fitted against an external pressure sensor at the mask end.
 * Was R_lam 0.002 / R_turb 0.00022 (a 15 mm slimline book value, then bench
 * nudged). Those over-estimate the drop badly once flow rises:
 *
 *      flow      fitted   previous
 *      15 LPM     0.080     0.080
 *      25         0.162     0.188
 *      35         0.268     0.340
 *      45         0.398     0.536
 *      55         0.552     0.775
 *
 * They agree at 15 LPM and diverge with flow, so the old pair was too
 * quadratic: at peak inspiratory flow it subtracted 0.22 cmH2O too much, and
 * since P_mask is the PID's feedback that is a breath-synchronous error
 * injected straight into the loop. Fit residual 0.011 cmH2O against 0.097 for
 * the old pair, which also carried a -0.070 bias.
 *
 * HOW THIS WAS MEASURED, because the obvious experiment cannot work. In a
 * vent-dominated circuit the vent flow goes as sqrt(P), so Q^2 is exactly
 * proportional to P: "drop proportional to Q^2" and "drop proportional to
 * pressure" are the same curve, and no sweep driven by vent flow can tell a
 * hose drop from a sensor scale difference. Two independent fits collapsed on
 * that (basis correlation 0.9965, one returning a negative R_turb).
 *
 * It takes three measurements:
 *   1. Static sweep 6-18 cmH2O, circuit occluded downstream of the vent so
 *      the external sensor sits in a dead-end branch and reads mask pressure.
 *      Pressure varies, flow follows it.
 *   2. Breathing at a regulated 15 cmH2O. Pressure fixed, flow swings 30-54
 *      LPM. This is what decorrelates flow from pressure.
 *   3. Vent AND outlet blocked, zero flow everywhere. A hose cannot drop
 *      pressure with nothing moving through it, so the residual difference is
 *      sensor mismatch: measured +0.237 cmH2O at 15.01, i.e. 1.58% of reading,
 *      over 6000 samples.
 * Subtract that 1.58% from (1) and (2), and the flow range widens to 16-54 LPM
 * with the basis correlation down to 0.9894 - enough to fit both terms.
 *
 * OPEN: the 1.58% is a DIFFERENCE between two sensors. If it is ours reading
 * high, patients get 1.6% less than the display claims (0.3 cmH2O at 20) and
 * that matters more than this model. A reference manometer is the only way to
 * tell which unit is honest. These coefficients are unaffected either way -
 * the mismatch is common-mode and was removed before fitting.
 *
 * Re-fit per hose/mask: pc_tools has no tool for this yet; the three captures
 * above were driven by scratch scripts against PlotFlowP.py's packet format.
 *
 * 2026-09-25: these never took effect after a boot. TubeComp_Init() - called
 * from main() - reset both to the old 0.002 / 0.00022, so the fitted values
 * only applied in sessions that set them over USB ('tubecomp set R_lam ...'),
 * and the D-line hose term shows every capture of 2026-09-24 ran on the old
 * pair. One definition now feeds both the initialisers and TubeComp_Init().
 *
 * What moves: P_mask (the PID's feedback) reads higher at high inspiratory
 * flow - +0.2 cmH2O at 55 LPM, +0.67 at the 89 slm clip - so the loop
 * pushes that much less at the sensor, which is the point: the old pair made
 * the true mask pressure overshoot setpoint by that amount on every breath.
 * The physics FF does NOT move: its hose term was fitted jointly with K,
 * offset and c_fan on the old pair and now has its own frozen coefficients
 * (g_blower_ctrl_physics_ff_hose_*, blower_ctrl.c). */
#define TUBECOMP_R_LAM_DEFAULT   0.00353f
#define TUBECOMP_R_TURB_DEFAULT  0.000118f
volatile float   g_tubecomp_R_lam_cmh2o_per_lpm  = TUBECOMP_R_LAM_DEFAULT;
volatile float   g_tubecomp_R_turb_cmh2o_per_lpm2 = TUBECOMP_R_TURB_DEFAULT;

/* === Live values (debugger-readable volatiles) === */
volatile float   g_tubecomp_p_sensor_cmh2o = 0.0f;
volatile float   g_tubecomp_p_mask_cmh2o   = 0.0f;
volatile float   g_tubecomp_q_slm          = 0.0f;
volatile float   g_tubecomp_drop_cmh2o     = 0.0f;
volatile uint32_t g_tubecomp_step_count    = 0U;

/* === Public API === */

void TubeComp_Init(void)
{
    g_tubecomp_enabled                = 1U;
    g_tubecomp_R_lam_cmh2o_per_lpm    = TUBECOMP_R_LAM_DEFAULT;
    g_tubecomp_R_turb_cmh2o_per_lpm2  = TUBECOMP_R_TURB_DEFAULT;
    g_tubecomp_p_sensor_cmh2o         = 0.0f;
    g_tubecomp_p_mask_cmh2o           = 0.0f;
    g_tubecomp_q_slm                  = 0.0f;
    g_tubecomp_drop_cmh2o             = 0.0f;
    g_tubecomp_step_count             = 0U;
}

void  TubeComp_SetEnable(bool en) { g_tubecomp_enabled = en ? 1U : 0U; }
bool  TubeComp_IsEnabled(void)    { return (g_tubecomp_enabled != 0U); }

void TubeComp_SetCoefficients(float R_lam, float R_turb)
{
    /* Defensive clamping: physically sensible adult-CPAP hose has
     * R_turb in [0, 0.001] and R_lam in [0, 0.05]. Reject obviously
     * silly numbers - they'd swamp the loop. */
    if (R_lam  < 0.0f)     R_lam  = 0.0f;
    if (R_lam  > 0.05f)    R_lam  = 0.05f;
    if (R_turb < 0.0f)     R_turb = 0.0f;
    if (R_turb > 0.001f)   R_turb = 0.001f;
    g_tubecomp_R_lam_cmh2o_per_lpm    = R_lam;
    g_tubecomp_R_turb_cmh2o_per_lpm2  = R_turb;
}

float TubeComp_GetRLam(void)             { return g_tubecomp_R_lam_cmh2o_per_lpm; }
float TubeComp_GetRTurb(void)            { return g_tubecomp_R_turb_cmh2o_per_lpm2; }
float TubeComp_GetLastSensorCmh2o(void)  { return g_tubecomp_p_sensor_cmh2o; }
float TubeComp_GetLastMaskCmh2o(void)    { return g_tubecomp_p_mask_cmh2o; }
float TubeComp_GetLastDropCmh2o(void)    { return g_tubecomp_drop_cmh2o; }
float TubeComp_GetLastFlowSlm(void)      { return g_tubecomp_q_slm; }

float TubeComp_Apply(float p_sensor_cmh2o, float q_slm)
{
    g_tubecomp_p_sensor_cmh2o = p_sensor_cmh2o;
    g_tubecomp_q_slm          = q_slm;
    g_tubecomp_step_count++;

    if (g_tubecomp_enabled == 0U)
    {
        /* Pass-through: mask = sensor, no drop, no surprise. */
        g_tubecomp_drop_cmh2o   = 0.0f;
        g_tubecomp_p_mask_cmh2o = p_sensor_cmh2o;
        return p_sensor_cmh2o;
    }

    /* dP magnitude is always positive; sign comes from flow direction.
     * Convention: Q > 0 means flow from device toward patient (the usual
     * direction during inspiration). With positive Q, P_mask < P_sensor.
     * During exhalation Q is negative and P_mask > P_sensor for the same
     * reason - the air piles up against the closed inspiratory limb. */
    const float qabs    = fabsf(q_slm);
    const float drop_mag = g_tubecomp_R_lam_cmh2o_per_lpm   * qabs
                         + g_tubecomp_R_turb_cmh2o_per_lpm2 * qabs * qabs;
    const float drop    = (q_slm >= 0.0f) ? drop_mag : -drop_mag;

    g_tubecomp_drop_cmh2o   = drop;
    g_tubecomp_p_mask_cmh2o = p_sensor_cmh2o - drop;
    return g_tubecomp_p_mask_cmh2o;
}
