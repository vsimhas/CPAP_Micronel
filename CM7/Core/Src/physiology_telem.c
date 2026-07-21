/**
  ******************************************************************************
  * @file    physiology_telem.c
  * @brief   Breath Vt + periodic low-amplitude FOT for R_eff / C_proxy telemetry.
  ******************************************************************************
  */

#include "physiology_telem.h"

#if PHYSIO_TELEM_ENABLED

#include <math.h>

#include "telemetry_stream.h"
#include "therapy.h"
#include "leak_estimator.h"
#include "blower_osc.h"
#include "blower_ctrl.h"
#include "flow_sensor.h"
#include "tube_comp.h"
#include "apap_ctrl.h"
#include "stm32h7xx_hal.h"

#define PHYSIO_TWO_PI          6.28318530717958647692f
#define PHYSIO_Q_PP_MIN_SLM    0.15f
#define PHYSIO_P_PP_MIN_CMH2O  0.05f
#define PHYSIO_R_EFF_MAX       200.0f
#define PHYSIO_C_PROXY_MAX     500.0f

typedef enum
{
  PHYSIO_FOT_IDLE = 0,
  PHYSIO_FOT_PROBE,
} PhysioFotState_t;

volatile uint8_t g_physio_telem_enable              = 1U;
volatile uint8_t g_physio_fot_enable                = 1U;
volatile float   g_physio_fot_interval_s            = 30.0f;
volatile float   g_physio_fot_duration_s            = 2.0f;
volatile float   g_physio_fot_amplitude_cmh2o       = 0.40f;
volatile float   g_physio_fot_freq_hz               = 4.0f;
volatile float   g_physio_last_vt_ml                = 0.0f;
volatile float   g_physio_last_r_eff                = 0.0f;
volatile float   g_physio_last_c_proxy_ml_cmh2o     = 0.0f;
volatile float   g_physio_last_rr_bpm               = 0.0f;

static PhysioFotState_t s_fot_state = PHYSIO_FOT_IDLE;
static uint32_t         s_cooldown_ms = 0U;
static uint32_t         s_probe_ms = 0U;
static uint8_t          s_prev_breath_insp = 0U;
static uint8_t          s_physio_owns_osc = 0U;
static float            s_p_min = 0.0f;
static float            s_p_max = 0.0f;
static float            s_q_min = 0.0f;
static float            s_q_max = 0.0f;
static float            s_last_apap_p_pp = -1.0f;

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool therapyActive(void)
{
  const TherapyState_t st = Therapy_GetState();
  return (st == THERAPY_STATE_RUNNING) || (st == THERAPY_STATE_RAMP);
}

static void resetFotStats(void)
{
  const float p = BlowerCtrl_GetMeasured();
  const float q = Flow_GetPatientSlm();
  s_p_min = p;
  s_p_max = p;
  s_q_min = q;
  s_q_max = q;
}

static void trackFotStats(void)
{
  const float p = BlowerCtrl_GetMeasured();
  const float q = Flow_GetPatientSlm();
  if (p < s_p_min) { s_p_min = p; }
  if (p > s_p_max) { s_p_max = p; }
  if (q < s_q_min) { s_q_min = q; }
  if (q > s_q_max) { s_q_max = q; }
}

static void computeFotMetrics(float freq_hz, float p_pp, float q_pp,
                              float *r_eff, float *c_proxy)
{
  if ((p_pp < PHYSIO_P_PP_MIN_CMH2O) || (q_pp < PHYSIO_Q_PP_MIN_SLM))
  {
    *r_eff = 0.0f;
    *c_proxy = 0.0f;
    return;
  }

  *r_eff = clampf(p_pp / q_pp, 0.0f, PHYSIO_R_EFF_MAX);

  /* Oscillatory compliance proxy at f: C ≈ 1000·Q_pp / (120·π·f·P_pp) mL/cmH2O. */
  const float f = clampf(freq_hz, 0.5f, 20.0f);
  *c_proxy = (1000.0f * q_pp) / (120.0f * PHYSIO_TWO_PI * f * p_pp);
  *c_proxy = clampf(*c_proxy, 0.0f, PHYSIO_C_PROXY_MAX);
}

static void stopPhysioProbe(void)
{
  if (s_physio_owns_osc != 0U)
  {
    BlowerOsc_SetEnable(false);
    s_physio_owns_osc = 0U;
  }
  s_fot_state = PHYSIO_FOT_IDLE;
  s_probe_ms = 0U;
}

static void startPhysioProbe(void)
{
  if (Apap_IsOscillating())
  {
    return;
  }

  resetFotStats();
  BlowerOsc_SetAmplitude(clampf(g_physio_fot_amplitude_cmh2o, 0.05f, 1.0f));
  BlowerOsc_SetFrequency(clampf(g_physio_fot_freq_hz, 2.0f, 8.0f));
  BlowerOsc_SetEnable(true);
  s_physio_owns_osc = 1U;
  s_fot_state = PHYSIO_FOT_PROBE;
  s_probe_ms = 0U;
}

static void publishPhysioLine(uint8_t fot_src, float p_pp, float q_pp)
{
  Telem_PushPhysiology(g_physio_last_vt_ml,
                       g_physio_last_r_eff,
                       g_physio_last_c_proxy_ml_cmh2o,
                       g_physio_last_rr_bpm,
                       fot_src,
                       p_pp,
                       q_pp);
}

static void finalizeFotWindow(uint8_t fot_src, float freq_hz)
{
  const float p_pp = s_p_max - s_p_min;
  const float q_pp = s_q_max - s_q_min;
  computeFotMetrics(freq_hz, p_pp, q_pp,
                   (float *)&g_physio_last_r_eff,
                   (float *)&g_physio_last_c_proxy_ml_cmh2o);
  publishPhysioLine(fot_src, p_pp, q_pp);
}

void PhysioTelem_Init(void)
{
  g_physio_telem_enable = 1U;
  g_physio_fot_enable = 1U;
  g_physio_fot_interval_s = 30.0f;
  g_physio_fot_duration_s = 2.0f;
  g_physio_fot_amplitude_cmh2o = 0.40f;
  g_physio_fot_freq_hz = 4.0f;
  g_physio_last_vt_ml = 0.0f;
  g_physio_last_r_eff = 0.0f;
  g_physio_last_c_proxy_ml_cmh2o = 0.0f;
  g_physio_last_rr_bpm = 0.0f;

  s_fot_state = PHYSIO_FOT_IDLE;
  s_cooldown_ms = 0U;
  s_probe_ms = 0U;
  s_prev_breath_insp = 0U;
  s_physio_owns_osc = 0U;
  s_last_apap_p_pp = -1.0f;
  stopPhysioProbe();
}

void PhysioTelem_SetFotEnable(bool enable)
{
  g_physio_fot_enable = enable ? 1U : 0U;
  if (!enable)
  {
    stopPhysioProbe();
    s_cooldown_ms = 0U;
  }
}

bool PhysioTelem_IsFotEnabled(void)
{
  return (g_physio_fot_enable != 0U);
}

void PhysioTelem_Update(float dt_s)
{
  if ((g_physio_telem_enable == 0U) || !(dt_s > 0.0f) || (dt_s > 0.1f))
  {
    return;
  }

  const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);
  const uint8_t breath_insp = Leak_GetBreathInspiration();
  const float rr = Leak_GetBreathRateBpm();

  if (rr > 0.0f)
  {
    g_physio_last_rr_bpm = rr;
  }

  /* New inspiration → report completed breath tidal volume. */
  if ((breath_insp != 0U) && (s_prev_breath_insp == 0U))
  {
    const float vt = Leak_GetLastInspVolumeMl();
    if (vt > 0.0f)
    {
      g_physio_last_vt_ml = vt;
    }
    publishPhysioLine(0U, 0.0f, 0.0f);
  }
  s_prev_breath_insp = breath_insp;

  if (!therapyActive() || Therapy_IsMaskOffActive())
  {
    stopPhysioProbe();
    s_cooldown_ms = 0U;
    s_last_apap_p_pp = -1.0f;
    return;
  }

  /* APAP clinical FOT: mirror metrics when the autoset probe is active. */
  if (Apap_IsOscillating())
  {
    stopPhysioProbe();

    const float p_pp = Apap_GetFotPressurePpCmh2o();
    const float q_pp = Apap_GetFotFlowPpSlm();
    if (fabsf(p_pp - s_last_apap_p_pp) > 0.02f)
    {
      s_last_apap_p_pp = p_pp;
      computeFotMetrics(BlowerOsc_GetFrequency(), p_pp, q_pp,
                        (float *)&g_physio_last_r_eff,
                        (float *)&g_physio_last_c_proxy_ml_cmh2o);
      publishPhysioLine(2U, p_pp, q_pp);
    }
    return;
  }

  s_last_apap_p_pp = -1.0f;

  /* Periodic physiology FOT can be disabled for waveform A/B (Vt still reported). */
  if (g_physio_fot_enable == 0U)
  {
    stopPhysioProbe();
    s_cooldown_ms = 0U;
    return;
  }

  switch (s_fot_state)
  {
  case PHYSIO_FOT_PROBE:
    if (Apap_IsOscillating())
    {
      stopPhysioProbe();
      break;
    }

    trackFotStats();
    s_probe_ms += dt_ms;

    {
      const uint32_t probe_limit_ms =
          (uint32_t)(clampf(g_physio_fot_duration_s, 0.5f, 10.0f) * 1000.0f);
      if (s_probe_ms >= probe_limit_ms)
      {
        stopPhysioProbe();
        finalizeFotWindow(1U, g_physio_fot_freq_hz);
        s_cooldown_ms = 0U;
      }
    }
    break;

  case PHYSIO_FOT_IDLE:
  default:
    s_cooldown_ms += dt_ms;
    {
      const uint32_t interval_ms =
          (uint32_t)(clampf(g_physio_fot_interval_s, 5.0f, 300.0f) * 1000.0f);
      if (s_cooldown_ms >= interval_ms)
      {
        startPhysioProbe();
      }
    }
    break;
  }
}

#endif /* PHYSIO_TELEM_ENABLED */
