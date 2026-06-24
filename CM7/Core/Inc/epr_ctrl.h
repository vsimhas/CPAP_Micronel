/**
  ******************************************************************************
  * @file    epr_ctrl.h
  * @brief   Expiratory Pressure Relief (EPR / Flex-style).
  *
  *  During exhalation the therapy setpoint is reduced by a fixed relief
  *  amount so the patient exhales against a lower pressure. This matches
  *  ResMed EPR / Philips Flex behaviour at a basic level:
  *
  *      P_therapy_exhale = max(P_min, P_CPAP - relief)
  *
 *  Breath phase is inferred from vent/leak-corrected patient flow
 *  (Flow_GetPatientSlm) using a gated state machine:
 *
 *      IDLE -> (sustained insp) -> INSPIRATION -> (sustained exp) -> EXPIRATION
 *
 *  EPR relief is applied only in confirmed EXPIRATION after a confirmed
 *  INSPIRATION in the same breath cycle. Short noise blips are rejected
 *  by minimum phase hold times and a minimum inspiratory peak requirement.
  *
 *  The GUI exposes On/Off only (per GUI_Requirements.txt). When On, the
 *  default relief is 1.5 cmH2O. Tune via
  *  g_epr_relief_cmh2o in the watch window.
  ******************************************************************************
  */

#ifndef EPR_CTRL_H
#define EPR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    EPR_PHASE_UNKNOWN = 0,
    EPR_PHASE_INSPIRATION,
    EPR_PHASE_EXPIRATION
} EprPhase_t;

void       Epr_Init(void);

void       Epr_SetEnable(bool en);
bool       Epr_IsEnabled(void);

/* Relief magnitude in cmH2O when enabled (default 1.5). Clamped 1..3. */
void       Epr_SetReliefCmh2o(float cmh2o);
float      Epr_GetReliefCmh2o(void);

/* Minimum pressure floor during EPR (default 4.0 cmH2O). */
void       Epr_SetMinPressureCmh2o(float cmh2o);
float      Epr_GetMinPressureCmh2o(void);

/* Per-tick update: feed patient flow (slm) and dt. Call from the same
 * task/rate as Therapy_Update (200 Hz), after Leak_Update(). */
void       Epr_Update(float q_slm, float dt_s);

/* Apply EPR to a base CPAP setpoint (cmH2O). Returns the modified value
 * that should be passed to BlowerCtrl_SetSetpoint(). Relief is slewed in
 * and out to avoid step changes at phase boundaries. */
float      Epr_ModifySetpoint(float base_setpoint_cmh2o, float dt_s);

EprPhase_t Epr_GetPhase(void);
const char* Epr_GetPhaseName(void);
float      Epr_GetActiveReliefCmh2o(void);   /* currently applied relief  */
float      Epr_GetFlowBaselineSlm(void);

/* Reset breath-phase state at therapy start (baseline re-seeds on next
 * flow sample). Does not change enable/relief settings. */
void       Epr_ResetBreathState(void);

#ifdef __cplusplus
}
#endif

#endif /* EPR_CTRL_H */
