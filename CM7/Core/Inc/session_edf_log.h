/**
  ******************************************************************************
  * @file    session_edf_log.h
  * @brief   Therapy session waveform logger → SD card EDF files.
  *
 *  ResMed-style card layout, so OSCAR's loader recognises it:
 *      0:/Identification.json, 0:/STR.edf
 *      0:/DATALOG/YYYYMMDD/YYYYMMDD_HHMMSS_BRP.edf   25 Hz waveforms
 *      0:/DATALOG/YYYYMMDD/YYYYMMDD_HHMMSS_PLD.edf    1 Hz summary
 *
 *  BRP (1 s records @ 25 Hz): Flow.40ms (L/s), Press.40ms (cmH2O).
 *  PLD (1 s records @ 1 Hz):  MaskPress.2s, Press.2s, Leak.2s.
 *  Only the layout is borrowed - Identification.json carries our own
 *  identity, not a ResMed serial. Disabled by default
 *  (therapy edf enable 1) so FatFs/SD cannot stall USB telemetry.
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

/* Enable/disable auto-start on Therapy_Start (default OFF). */
void SessionEdf_SetEnable(bool en);
bool SessionEdf_IsEnabled(void);

/* Wall-clock for EDF header (no RTC on board — set via USB). */
void SessionEdf_SetDateTime(uint8_t yy, uint8_t mo, uint8_t dd,
                            uint8_t hh, uint8_t mi, uint8_t ss);

/* Called from Therapy_Start / Therapy_Stop. */
void SessionEdf_OnTherapyStart(void);
void SessionEdf_OnTherapyStop(void);

/* Call every therapy tick (~200 Hz). Internally downsamples to 25 Hz.
 * RAM only — never blocks on SD. */
void SessionEdf_Update(float dt_s);

/* FatFs/SD I/O — call from SelfTestTask (low priority) only. */
void SessionEdf_Service(void);

SessionEdfState_t SessionEdf_GetState(void);
const char*       SessionEdf_GetStateName(void);
const char*       SessionEdf_GetLastPath(void);
uint32_t          SessionEdf_GetRecords(void);
uint32_t          SessionEdf_GetLastError(void); /* FatFs FRESULT or custom */
uint32_t          SessionEdf_GetFileSize(void);  /* last known FatFs size */

/* PLD companion file (OSCAR's source for the default Pressure / Mask
 * Pressure / Leak graphs). Written best-effort, so check these when OSCAR
 * shows flow but no pressure. Error encoding: 0x4Sxx open failure with
 * S = Edf_Open step, 0x42xx write failure; xx is the FatFs FRESULT. */
uint32_t          SessionEdf_GetPldRecords(void);
uint32_t          SessionEdf_GetPldError(void);
const char*       SessionEdf_GetPldPath(void);

/* Mount/unmount helpers (also used by USB status). */
bool SessionEdf_Mount(void);
void SessionEdf_Unmount(void);
bool SessionEdf_IsMounted(void);

#ifdef __cplusplus
}
#endif

#endif /* SESSION_EDF_LOG_H */
