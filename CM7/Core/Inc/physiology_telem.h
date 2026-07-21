/**
  ******************************************************************************
  * @file    physiology_telem.h
  * @brief   Low-rate physiology telemetry: Vt per breath + FOT R_eff / C_proxy.
  *
  *  USB line (once per completed breath and after each FOT window):
  *    PH,<ms>,<Vt_ml>,<R_eff>,<C_proxy>,<RR_bpm>,<fot_src>,<p_pp>,<q_pp>
  *
  *  fot_src: 0=none/held, 1=periodic physiology probe, 2=APAP OSA FOT
  *
  *  R_eff  = ΔP_pp / ΔQ_pp           [cmH2O/slm]  (respiratory-system)
  *  C_proxy ≈ 1000·ΔQ_pp / (120·π·f·ΔP_pp) [mL/cmH2O] at probe frequency f
  ******************************************************************************
  */

#ifndef PHYSIOLOGY_TELEM_H
#define PHYSIOLOGY_TELEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PHYSIO_TELEM_ENABLED
#define PHYSIO_TELEM_ENABLED  1
#endif

#if PHYSIO_TELEM_ENABLED

extern volatile uint8_t  g_physio_telem_enable;
extern volatile uint8_t  g_physio_fot_enable;   /* periodic phys FOT probe */
extern volatile float  g_physio_fot_interval_s;
extern volatile float  g_physio_fot_duration_s;
extern volatile float  g_physio_fot_amplitude_cmh2o;
extern volatile float  g_physio_fot_freq_hz;
extern volatile float  g_physio_last_vt_ml;
extern volatile float  g_physio_last_r_eff;
extern volatile float  g_physio_last_c_proxy_ml_cmh2o;
extern volatile float  g_physio_last_rr_bpm;

void PhysioTelem_Init(void);
void PhysioTelem_Update(float dt_s);
void PhysioTelem_SetFotEnable(bool enable);
bool PhysioTelem_IsFotEnabled(void);

#else

#define PhysioTelem_Init() ((void)0)
#define PhysioTelem_Update(dt_s) ((void)0)
#define PhysioTelem_SetFotEnable(en) ((void)(en))
#define PhysioTelem_IsFotEnabled() (false)

#endif /* PHYSIO_TELEM_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* PHYSIOLOGY_TELEM_H */
