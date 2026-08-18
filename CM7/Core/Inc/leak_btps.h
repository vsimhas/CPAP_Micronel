/**
  ******************************************************************************
  * @file    leak_btps.h
  * @brief   STPD/BTPS + altitude factors for Vivo leak table path.
  *
  *  Measured orifice flow is density-corrected in flow_sensor.c
  *  (sqrt(P_amb/P_ref)). This module:
  *    - scales the sea-level STPD leak *table* by the same altitude factor
  *      so model and measurement stay consistent at altitude;
  *    - applies optional temperature / humidity BTPS factors;
  *    - does NOT re-apply altitude on measured flow (avoids double scaling).
  ******************************************************************************
  */

#ifndef LEAK_BTPS_H
#define LEAK_BTPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define LEAK_BTPS_P_REF_MBAR  1013.25f

void LeakBtps_Init(void);

/* Push live ambient absolute pressure [mbar] (from measAS / Flow path). */
void LeakBtps_SetAmbientMbar(float mbar);
float LeakBtps_GetAmbientMbar(void);

/* Enable/disable altitude scaling of the STPD table (default ON). */
void LeakBtps_SetAltitudeEnable(bool en);
bool LeakBtps_IsAltitudeEnable(void);

/* Reference table value [ml/s STPD @ sea level] -> local/BTPS [ml/s].
 * Applies altitude * temperature * humidity factors. */
int32_t LeakBtps_TableToBtpsMlps(int32_t flow_mlps_stpd);

/* Measured flow already density-corrected in flow_sensor [ml/s SLM-like]
 * -> BTPS [ml/s]. Applies temperature * humidity only (no altitude). */
int32_t LeakBtps_MeasuredStpdToBtpsMlps(int32_t flow_mlps_stpd);

/* Runtime overrides (default 1.0). Future: drive T/RH from Sensirion. */
void    LeakBtps_SetTemperatureFactor(float factor);
void    LeakBtps_SetHumidityFactor(float factor);
void    LeakBtps_SetAltitudeFactor(float factor); /* normally from SetAmbient */

float   LeakBtps_GetTemperatureFactor(void);
float   LeakBtps_GetHumidityFactor(void);
float   LeakBtps_GetAltitudeFactor(void);

#ifdef __cplusplus
}
#endif

#endif /* LEAK_BTPS_H */
