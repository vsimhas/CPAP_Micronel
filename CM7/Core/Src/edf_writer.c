/**
  ******************************************************************************
  * @file    edf_writer.c
  * @brief   Classic EDF header + data-record writer (FatFs FIL sink).
  ******************************************************************************
  */

#include "edf_writer.h"

#include <stdio.h>
#include <string.h>

static void padCopy(char *dst, size_t dst_n, const char *src)
{
    size_t i = 0U;
    if (src != NULL)
    {
        while ((i < (dst_n - 1U)) && (src[i] != '\0'))
        {
            dst[i] = src[i];
            i++;
        }
    }
    while (i < dst_n)
    {
        dst[i++] = ' ';
    }
}

static bool writeExact(FIL *fil, const void *data, UINT n)
{
    UINT bw = 0U;
    if (f_write(fil, data, n, &bw) != FR_OK)
    {
        return false;
    }
    return (bw == n);
}

static bool writeAsciiField(FIL *fil, const char *text, UINT width)
{
    char tmp[128];
    if (width > sizeof(tmp))
    {
        return false;
    }
    padCopy(tmp, width, text);
    return writeExact(fil, tmp, width);
}

int16_t Edf_PhysToDig(const EdfSignalDesc_t *s, float phys)
{
    if (s == NULL)
    {
        return 0;
    }
    const float pmin = s->phys_min;
    const float pmax = s->phys_max;
    if (!(pmax > pmin))
    {
        return 0;
    }
    float t = (phys - pmin) / (pmax - pmin);
    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }
    const float dig = (float)s->dig_min
                    + t * ((float)s->dig_max - (float)s->dig_min);
    if (dig < (float)s->dig_min) { return s->dig_min; }
    if (dig > (float)s->dig_max) { return s->dig_max; }
    return (int16_t)dig;
}

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
              uint32_t sample_buf_n)
{
    char num[16];
    uint32_t need = 0U;
    uint8_t i;

    if ((w == NULL) || (path == NULL) || (signals == NULL) || (sample_buf == NULL)
        || (nsig == 0U) || (nsig > EDF_MAX_SIGNALS) || (record_duration_s == 0U))
    {
        return false;
    }

    memset(w, 0, sizeof(*w));
    for (i = 0U; i < nsig; i++)
    {
        w->sig[i] = signals[i];
        need += (uint32_t)signals[i].samples_per_record;
    }
    if (sample_buf_n < need)
    {
        return false;
    }

    if (f_open(&w->fil, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        return false;
    }

    w->open = true;
    w->nsig = nsig;
    w->record_duration_s = record_duration_s;
    w->n_records = 0U;
    w->header_bytes = 256U + ((uint32_t)nsig * 256U);
    w->sample_buf = sample_buf;
    w->sample_buf_n = sample_buf_n;

    /* --- Fixed header (256 bytes) --- */
    if (!writeAsciiField(&w->fil, "0", 8U)) { goto fail; }
    if (!writeAsciiField(&w->fil, patient_id, 80U)) { goto fail; }
    if (!writeAsciiField(&w->fil, recording_id, 80U)) { goto fail; }

    {
        char date[9];
        char time[9];
        (void)snprintf(date, sizeof(date), "%02u.%02u.%02u",
                       (unsigned)start_dd, (unsigned)start_mm, (unsigned)start_yy);
        (void)snprintf(time, sizeof(time), "%02u.%02u.%02u",
                       (unsigned)start_hh, (unsigned)start_nn, (unsigned)start_ss);
        if (!writeAsciiField(&w->fil, date, 8U)) { goto fail; }
        if (!writeAsciiField(&w->fil, time, 8U)) { goto fail; }
    }

    (void)snprintf(num, sizeof(num), "%lu", (unsigned long)w->header_bytes);
    if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }
    if (!writeAsciiField(&w->fil, "", 44U)) { goto fail; } /* reserved */

    /* Number of data records unknown until close → -1 */
    if (!writeAsciiField(&w->fil, "-1", 8U)) { goto fail; }

    (void)snprintf(num, sizeof(num), "%u", (unsigned)record_duration_s);
    if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }

    (void)snprintf(num, sizeof(num), "%u", (unsigned)nsig);
    if (!writeAsciiField(&w->fil, num, 4U)) { goto fail; }

    /* --- Per-signal header fields (nsig * 256) --- */
    for (i = 0U; i < nsig; i++)
    {
        if (!writeAsciiField(&w->fil, w->sig[i].label, 16U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        if (!writeAsciiField(&w->fil, w->sig[i].transducer, 80U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        if (!writeAsciiField(&w->fil, w->sig[i].phys_dim, 8U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%.4g", (double)w->sig[i].phys_min);
        if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%.4g", (double)w->sig[i].phys_max);
        if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%d", (int)w->sig[i].dig_min);
        if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%d", (int)w->sig[i].dig_max);
        if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        if (!writeAsciiField(&w->fil, w->sig[i].prefilter, 80U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%u",
                       (unsigned)w->sig[i].samples_per_record);
        if (!writeAsciiField(&w->fil, num, 8U)) { goto fail; }
    }
    for (i = 0U; i < nsig; i++)
    {
        if (!writeAsciiField(&w->fil, "", 32U)) { goto fail; }
    }

    (void)f_sync(&w->fil);
    return true;

fail:
    (void)f_close(&w->fil);
    w->open = false;
    return false;
}

bool Edf_WriteRecord(EdfWriter_t *w)
{
    UINT n;
    if ((w == NULL) || !w->open || (w->sample_buf == NULL))
    {
        return false;
    }

    n = 0U;
    {
        uint8_t i;
        for (i = 0U; i < w->nsig; i++)
        {
            n = (UINT)(n + w->sig[i].samples_per_record);
        }
    }
    n = (UINT)(n * sizeof(int16_t));
    if (!writeExact(&w->fil, w->sample_buf, n))
    {
        return false;
    }
    w->n_records++;
    /* Sync every ~10 records to limit SD wear/latency on power loss. */
    if ((w->n_records % 10U) == 0U)
    {
        (void)f_sync(&w->fil);
    }
    return true;
}

bool Edf_Close(EdfWriter_t *w)
{
    char num[16];
    char field[8];
    UINT bw;

    if ((w == NULL) || !w->open)
    {
        return false;
    }

    /* Patch "number of data records" at offset 236 (8 ascii). */
    (void)snprintf(num, sizeof(num), "%lu", (unsigned long)w->n_records);
    padCopy(field, sizeof(field), num);
    if (f_lseek(&w->fil, 236U) != FR_OK)
    {
        (void)f_close(&w->fil);
        w->open = false;
        return false;
    }
    if (f_write(&w->fil, field, 8U, &bw) != FR_OK || bw != 8U)
    {
        (void)f_close(&w->fil);
        w->open = false;
        return false;
    }
    (void)f_sync(&w->fil);
    (void)f_close(&w->fil);
    w->open = false;
    return true;
}
