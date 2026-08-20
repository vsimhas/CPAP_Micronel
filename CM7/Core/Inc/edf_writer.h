/**
  ******************************************************************************
  * @file    edf_writer.h
  * @brief   Minimal EDF (European Data Format) writer for therapy waveforms.
  *
  *  Writes classic EDF (not ResMed proprietary SD layout). Viewable in
  *  EDFbrowser / MATLAB / Python. OSCAR requires a vendor-specific card
  *  structure and will not import these files directly.
  ******************************************************************************
  */

#ifndef EDF_WRITER_H
#define EDF_WRITER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ff.h"

#define EDF_MAX_SIGNALS  8U

typedef struct
{
    char     label[17];       /* max 16 + NUL */
    char     transducer[81];
    char     phys_dim[9];
    float    phys_min;
    float    phys_max;
    int16_t  dig_min;         /* typically -32768 */
    int16_t  dig_max;         /* typically  32767 */
    char     prefilter[81];
    uint16_t samples_per_record;
} EdfSignalDesc_t;

typedef struct
{
    FIL             fil;
    bool            open;
    uint8_t         nsig;
    uint16_t        record_duration_s;
    uint32_t        n_records;
    uint32_t        header_bytes;
    EdfSignalDesc_t sig[EDF_MAX_SIGNALS];
    /* Per-record sample scratch (filled by caller before flush). */
    int16_t        *sample_buf;     /* size = sum(samples_per_record) */
    uint32_t        sample_buf_n;
} EdfWriter_t;

bool Edf_Open(EdfWriter_t *w,
              const char *path,
              const char *patient_id,
              const char *recording_id,
              uint8_t start_dd, uint8_t start_mm, uint8_t start_yy,
              uint8_t start_hh, uint8_t start_nn, uint8_t start_ss,
              uint16_t record_duration_s,
              const EdfSignalDesc_t *signals,
              uint8_t nsig,
              int16_t *sample_buf,
              uint32_t sample_buf_n);

/* Append one completed data record (sample_buf already filled). */
bool Edf_WriteRecord(EdfWriter_t *w);

/* Patch header record count and close. */
bool Edf_Close(EdfWriter_t *w);

/* Convert physical value to EDF digital sample for one signal. */
int16_t Edf_PhysToDig(const EdfSignalDesc_t *s, float phys);

#ifdef __cplusplus
}
#endif

#endif /* EDF_WRITER_H */
