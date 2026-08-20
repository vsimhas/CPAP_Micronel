/**
  ******************************************************************************
  * @file    session_edf_log.h
  * @brief   Therapy session waveform logger → SD card EDF files.
  *
  *  Layout on a FAT-formatted SD card:
  *      0:/SLINK/YYmmddNN.EDF
  *
  *  Signals (1 s records @ 25 Hz): Flow, Pressure, Leak, TargetP.
  *  Standard EDF for EDFbrowser / analysis tools. Not a ResMed OSCAR card
  *  clone — OSCAR will not import these without a custom machine loader.
  ******************************************************************************
  */

#ifndef SESSION_EDF_LOG_H
#define SESSION_EDF_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SESSION_EDF_IDLE = 0,
    SESSION_EDF_LOGGING,
    SESSION_EDF_ERROR
} SessionEdfState_t;

void SessionEdf_Init(void);

/* Enable/disable auto-start on Therapy_Start (default ON when SD ready). */
void SessionEdf_SetEnable(bool en);
bool SessionEdf_IsEnabled(void);

/* Wall-clock for EDF header (no RTC on board — set via USB). */
void SessionEdf_SetDateTime(uint8_t yy, uint8_t mo, uint8_t dd,
                            uint8_t hh, uint8_t mi, uint8_t ss);

/* Called from Therapy_Start / Therapy_Stop. */
void SessionEdf_OnTherapyStart(void);
void SessionEdf_OnTherapyStop(void);

/* Call every therapy tick (~200 Hz). Internally downsamples to 25 Hz. */
void SessionEdf_Update(float dt_s);

SessionEdfState_t SessionEdf_GetState(void);
const char*       SessionEdf_GetStateName(void);
const char*       SessionEdf_GetLastPath(void);
uint32_t          SessionEdf_GetRecords(void);
uint32_t          SessionEdf_GetLastError(void); /* FatFs FRESULT or custom */

/* Mount/unmount helpers (also used by USB status). */
bool SessionEdf_Mount(void);
void SessionEdf_Unmount(void);
bool SessionEdf_IsMounted(void);

#ifdef __cplusplus
}
#endif

#endif /* SESSION_EDF_LOG_H */
