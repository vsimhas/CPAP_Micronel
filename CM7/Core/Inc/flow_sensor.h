/**
  ******************************************************************************
  * @file    flow_sensor.h
  * @brief   Flow-element differential-pressure conditioning.
  *
  *  The flow element at the blower outlet generates a small differential
  *  pressure (read as `measFS.pressure`, ±5 mbar full-scale). At constant
  *  flow this signal is intrinsically noisy because:
  *      - centrifugal blowers produce ~1-30 Hz pressure pulsations,
  *      - turbulence around the orifice/Pitot adds broadband 5-100 Hz hash,
  *      - housing vibration couples mechanically into the ΔP ports.
  *
  *  This module owns the conditioning pipeline:
  *
  *      raw ΔP (measFS.pressure)
  *           |
  *           v
  *      [1st-order IIR LPF, fc = 10 Hz default]
  *           |
  *           v
  *      filtered ΔP   (->  future: bandpass for FOT, ΔP -> Q calibration)
  *
  *  The filtered value is the right input for all "bulk flow" use cases
  *  (leak compensation, breath-shape detection, minute-volume estimate).
  *  When the FOT impedance computation is wired up later, it will need
  *  the *raw* signal (or a separate bandpass) to retain the 4 Hz
  *  perturbation that the LPF would attenuate by ~3 dB.
  *
  *  Cutoff selection cheat-sheet
  *  ----------------------------
  *      Use case                                 fc
  *      ---------------------------------------------
  *      Bulk flow / leak compensation            10-15 Hz
  *      Apnea / hypopnea breath envelope         3-5 Hz
  *      FOT (forced oscillation, ~4 Hz carrier)  do NOT use this LPF -
  *                                               use a bandpass
  *
  *  All limits and gains are debugger-writable volatile globals so you
  *  can tune the cutoff live with no rebuild.
  ******************************************************************************
  */

#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Cutoff clamps. Below 0.1 Hz the time constant (>1.6 s) is longer than
 * a breath cycle and you start corrupting the breathing waveform. Above
 * 50 Hz the filter starts losing meaning at our 200 Hz sample rate
 * (Nyquist = 100 Hz, but anti-aliasing margin says stay <= fs/4). */
#define FLOW_LPF_CUTOFF_MIN_HZ   0.1f
#define FLOW_LPF_CUTOFF_MAX_HZ   50.0f

/* Reset filter state and load defaults (10 Hz LPF). */
void  Flow_Init(void);

/* Live-tunable cutoff. Clamps inside; pass any value, get a safe one. */
void  Flow_SetLpfCutoff(float hz);
float Flow_GetLpfCutoff(void);

/* Push one new raw ΔP sample (mbar) into the filter. dt_s is the elapsed
 * time since the previous call. Designed to be called once per sensor
 * tick (every 5 ms in SensorTask). After the LPF runs we also recompute
 * the calibrated flow rate so Flow_GetSlm() is fresh every tick. */
void  Flow_Update(float dp_mbar_raw, float dt_s);

/* Read the most recent values (no side effect). */
float Flow_GetDpRaw(void);          /* last raw input,        mbar */
float Flow_GetDpFiltered(void);     /* LPF output,            mbar */
float Flow_GetDpFilteredCorr(void); /* LPF output minus zero, mbar */
float Flow_GetSlm(void);            /* active total flow (SLM or BTPS) */
float Flow_GetSlmStd(void);         /* LUT output always as SFM3300 SLM */
float Flow_GetSlmRaw(void);         /* same calc on raw ΔP (active units) */
float Flow_GetPatientSlm(void);     /* vent+leak corrected, breath-band LPF */
float Flow_GetPatientRawSlm(void);  /* vent+leak corrected, no breath LPF */

/* === Zero-offset handling ============================================
 *
 *  At zero flow the AMS5935 doesn't read exactly 0 mbar - Round Hole_2
 *  cal sheet zeros at -0.029 mbar / 0 SLM. The offset is part-and-board
 *  specific and drifts a little with temperature, so we expose two ways
 *  to manage it:
 *
 *      (a) Flow_SetZeroOffset(mbar) - apply a known offset (default is
 *          -0.029 mbar from the Round Hole_2 / SFM3300 sheet).
 *      (b) Flow_AutoZero() - sample the LPF output right now and use
 *          that as the new offset. Call this once at boot or anytime
 *          you can guarantee the blower is off and the airway is still.
 *
 *  After offset removal: q_slm = LUT_interp(filtered_dp - zero_offset).
 *  Optional BTPS scaling may then be applied (see below). */
void  Flow_SetZeroOffset(float mbar);
float Flow_GetZeroOffset(void);
void  Flow_AutoZero(void);

/* === SLM (SFM3300) ↔ BTPS ============================================
 *
 *  The Round Hole_2 LUT is referenced to SFM3300-250-D SLM
 *  (Sensirion standard: dry gas @ 20 °C / 1013.25 mbar).
 *  When BTPS is enabled, active flow is scaled to body conditions
 *  (37 °C, saturated, ambient pressure from measAS.pressure / AMS absolute):
 *
 *      Q_BTPS = Q_SLM * (310.15/293.15) * (P_amb / (P_amb - 62.67))
 *
 *  SensorTask refreshes P_amb from measAS.pressure at ~10 Hz.
 *  Flow_SetBtpsAmbientMbar() remains as a USB/debug override between AS samples.
 *  Therapy, leak, and USB telemetry all consume Flow_GetSlm() so the
 *  toggle applies end-to-end. Flow_GetSlmStd() always remains SLM. */
void  Flow_SetBtpsEnable(bool en);
bool  Flow_IsBtpsEnabled(void);
void  Flow_SetBtpsAmbientMbar(float mbar);
float Flow_GetBtpsAmbientMbar(void);
float Flow_GetBtpsFactor(void);
float Flow_SlmToBtps(float q_slm_std);

/* Standalone calibration evaluator: pure function, no state. Useful if
 * you want to apply the cal curve to a different ΔP source (e.g., the
 * raw band-passed FOT signal once impedance computation is wired).
 * Returns SFM3300 SLM (no BTPS). */
float Flow_DpToSlm(float dp_corrected_mbar);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_SENSOR_H */
