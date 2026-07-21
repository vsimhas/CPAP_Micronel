/**
  ******************************************************************************
  * @file    telemetry_stream.h
  * @brief   Live pressure / patient-flow CSV stream over USB CDC VCP.
  ******************************************************************************
  *
 * Wire format:
 *   T,<ms>,<P_sensor>,<Q>,<pid_rpm>,<mech_rpm>,<P_mask> — 50 Hz
 *   PH,<ms>,<Vt>,<R_eff>,<C_proxy>,<RR>,<fot_src>,<p_pp>,<q_pp> — per breath / FOT
 *   R,<ms>,<rpm_cmd>,<rpm_act>,<pressure_cmh2o>  — on manual g_bench_blower_speed_rpm change
 *
 * P_sensor = AMS5935 at blower; P_mask = tube-compensated estimate (PID measurement).
  *
  * Lines are pushed from SensorTask and drained by Telem_Pump() from
  * SystemTask (alongside Dbg_Pump). Debug log text and telemetry can
  * coexist on the same VCP; the PC plotter parses T and R lines.
  *
 * Enable at runtime:
 *   - Set g_telem_enabled = 1 in the debugger watch window, or
 *   - Send "telem on" over the VCP from the PC plotter (default on boot).
  ******************************************************************************
  */

#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TELEM_ENABLED
#define TELEM_ENABLED  1
#endif

#if TELEM_ENABLED

extern volatile uint8_t  g_telem_enabled;
extern volatile uint32_t g_telem_sample_count;
extern volatile uint32_t g_telem_dropped_count;

void Telem_Init(void);
void Telem_Pump(void);
void Telem_ProcessCommands(void);
void Telem_PushSample(float pressure_sensor_cmh2o, float patient_flow_slm,
                      int32_t pid_rpm_cmd, int32_t mech_rpm,
                      float pressure_mask_cmh2o);
void Telem_PushBenchRpm(int32_t rpm_cmd, int32_t rpm_act, float pressure_cmh2o);
void Telem_PushPhysiology(float vt_ml, float r_eff, float c_proxy_ml_cmh2o,
                          float rr_bpm, uint8_t fot_src,
                          float p_pp_cmh2o, float q_pp_slm);
void Telem_OnRx(const uint8_t *data, uint32_t len);

#else

#define Telem_Init() ((void)0)
#define Telem_Pump() ((void)0)
#define Telem_ProcessCommands() ((void)0)
#define Telem_PushSample(ps, q, c, a, pm) ((void)0)
#define Telem_PushBenchRpm(c, a, p) ((void)0)
#define Telem_PushPhysiology(vt, r, c, rr, src, pp, qp) ((void)0)

#endif /* TELEM_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_STREAM_H */
