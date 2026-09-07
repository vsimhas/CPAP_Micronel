/**
  ******************************************************************************
  * @file    edf_writer.c
  * @brief   Classic EDF header + data-record writer (FatFs FIL sink).
  *
  *  Layout written (EDF spec):
  *    fixed header 256 B: version(8) patient(80) recording(80) startdate(8)
  *                        starttime(8) header_bytes(8) reserved(44)
  *                        n_records(8) rec_duration(8) nsig(4)
  *    per signal  256 B: label(16) transducer(80) phys_dim(8) phys_min(8)
  *                        phys_max(8) dig_min(8) dig_max(8) prefilter(80)
  *                        samples_per_record(8) reserved(32)
  *    data records      : per record, per signal, samples_per_record int16 LE
  *
  *  SD notes: header is assembled in RAM and written in 512-byte chunks; the
  *  n_records field is only rewritten on Close (mid-session header seeks were
  *  aborting sessions). Data is synced periodically so the directory entry
  *  keeps a non-zero size even if the session ends badly.
  ******************************************************************************
  */

#include "edf_writer.h"

#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"

#define EDF_HDR_FIXED   256U
#define EDF_HDR_PER_SIG 256U
#define EDF_HDR_MAX     (EDF_HDR_FIXED + (EDF_MAX_SIGNALS * EDF_HDR_PER_SIG))

/* Offset of the 8-byte "number of data records" ASCII field. */
#define EDF_NREC_OFFSET 236U

/* Sync data (and therefore the directory size) every N records. */
#define EDF_DATA_SYNC_EVERY 5U

static void padCopy(char *dst, size_t width, const char *src)
{
    size_t i = 0U;
    /* EDF header fields are fixed-width ASCII, not C strings — fill all
     * `width` bytes (no reserved NUL). Truncate/pad with spaces as needed. */
    if (src != NULL)
    {
        while ((i < width) && (src[i] != '\0'))
        {
            dst[i] = src[i];
            i++;
        }
    }
    while (i < width)
    {
        dst[i++] = ' ';
    }
}

static void putField(char *hdr, uint32_t *off, uint32_t hdr_n,
                     const char *text, UINT width)
{
    if ((*off + width) > hdr_n)
    {
        return;
    }
    padCopy(&hdr[*off], width, text);
    *off += width;
}

/* Write all n bytes, resuming after short writes so a retry never re-sends
 * bytes the card already accepted. */
static FRESULT writeExactRetry(FIL *fil, const void *data, UINT n)
{
    const uint8_t *p = (const uint8_t *)data;
    UINT left = n;
    uint8_t attempt = 0U;
    FRESULT fr = FR_DISK_ERR;

    while (left > 0U)
    {
        UINT bw = 0U;
        fr = f_write(fil, p, left, &bw);
        if (bw > 0U)
        {
            p += bw;
            left -= bw;
            attempt = 0U;   /* progress made — allow a fresh retry budget */
        }
        if (left == 0U)
        {
            return FR_OK;
        }
        if (++attempt >= 4U)
        {
            return (fr != FR_OK) ? fr : FR_DISK_ERR;
        }
        HAL_Delay(30);
    }
    return FR_OK;
}

static FRESULT syncRetry(FIL *fil, uint8_t tries, uint32_t gap_ms)
{
    uint8_t attempt;
    FRESULT fr = FR_DISK_ERR;
    for (attempt = 0U; attempt < tries; attempt++)
    {
        fr = f_sync(fil);
        if (fr == FR_OK)
        {
            return FR_OK;
        }
        HAL_Delay(gap_ms);
    }
    return fr;
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

static void buildHeader(char *hdr, uint32_t hdr_bytes,
                        const EdfWriter_t *w,
                        const char *patient_id, const char *recording_id,
                        uint8_t dd, uint8_t mm, uint8_t yy,
                        uint8_t hh, uint8_t nn, uint8_t ss)
{
    char num[16];
    char date[16];
    char time[16];
    uint32_t off = 0U;
    uint8_t i;

    memset(hdr, ' ', hdr_bytes);

    putField(hdr, &off, hdr_bytes, "0", 8U);
    putField(hdr, &off, hdr_bytes, patient_id, 80U);
    putField(hdr, &off, hdr_bytes, recording_id, 80U);

    (void)snprintf(date, sizeof(date), "%02u.%02u.%02u",
                   (unsigned)dd, (unsigned)mm, (unsigned)yy);
    (void)snprintf(time, sizeof(time), "%02u.%02u.%02u",
                   (unsigned)hh, (unsigned)nn, (unsigned)ss);
    putField(hdr, &off, hdr_bytes, date, 8U);
    putField(hdr, &off, hdr_bytes, time, 8U);

    (void)snprintf(num, sizeof(num), "%lu", (unsigned long)hdr_bytes);
    putField(hdr, &off, hdr_bytes, num, 8U);
    /* Reserved field. ResMed writes "EDF" in its data files (and "EDF+D" in
     * the annotation files), and OSCAR's loader expects to see it; a blank
     * field here is legal classic EDF but is not what the loader matches. */
    putField(hdr, &off, hdr_bytes, "EDF", 44U);
    putField(hdr, &off, hdr_bytes, "-1", 8U);  /* n records, patched on Close */

    (void)snprintf(num, sizeof(num), "%u", (unsigned)w->record_duration_s);
    putField(hdr, &off, hdr_bytes, num, 8U);
    (void)snprintf(num, sizeof(num), "%u", (unsigned)w->nsig);
    putField(hdr, &off, hdr_bytes, num, 4U);

    for (i = 0U; i < w->nsig; i++)
    {
        putField(hdr, &off, hdr_bytes, w->sig[i].label, 16U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        putField(hdr, &off, hdr_bytes, w->sig[i].transducer, 80U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        putField(hdr, &off, hdr_bytes, w->sig[i].phys_dim, 8U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%d", (int)w->sig[i].phys_min);
        putField(hdr, &off, hdr_bytes, num, 8U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%d", (int)w->sig[i].phys_max);
        putField(hdr, &off, hdr_bytes, num, 8U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%d", (int)w->sig[i].dig_min);
        putField(hdr, &off, hdr_bytes, num, 8U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%d", (int)w->sig[i].dig_max);
        putField(hdr, &off, hdr_bytes, num, 8U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        putField(hdr, &off, hdr_bytes, w->sig[i].prefilter, 80U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        (void)snprintf(num, sizeof(num), "%u",
                       (unsigned)w->sig[i].samples_per_record);
        putField(hdr, &off, hdr_bytes, num, 8U);
    }
    for (i = 0U; i < w->nsig; i++)
    {
        putField(hdr, &off, hdr_bytes, "", 32U);
    }
}

FRESULT Edf_Open(EdfWriter_t *w,
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
    uint32_t need = 0U;
    uint32_t hdr_bytes;
    uint8_t i;
    FRESULT fr;
    static char s_hdr[EDF_HDR_MAX];

    if ((w == NULL) || (path == NULL) || (signals == NULL) || (sample_buf == NULL)
        || (nsig == 0U) || (nsig > EDF_MAX_SIGNALS) || (record_duration_s == 0U))
    {
        return FR_INVALID_PARAMETER;
    }

    memset(w, 0, sizeof(*w));
    for (i = 0U; i < nsig; i++)
    {
        w->sig[i] = signals[i];
        need += (uint32_t)signals[i].samples_per_record;
    }
    if (sample_buf_n < need)
    {
        return FR_INVALID_PARAMETER;
    }

    hdr_bytes = EDF_HDR_FIXED + ((uint32_t)nsig * EDF_HDR_PER_SIG);
    if (hdr_bytes > sizeof(s_hdr))
    {
        return FR_INVALID_PARAMETER;
    }

    w->nsig = nsig;
    w->record_duration_s = record_duration_s;
    w->n_records = 0U;
    w->header_bytes = hdr_bytes;
    w->bytes_per_record = need * (uint32_t)sizeof(int16_t);
    w->since_flush = 0U;
    w->sample_buf = sample_buf;
    w->sample_buf_n = sample_buf_n;

    w->open_step = EDF_STEP_CREATE;
    fr = f_open(&w->fil, path, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (fr != FR_OK)
    {
        return fr;
    }
    w->open = true;

    buildHeader(s_hdr, hdr_bytes, w, patient_id, recording_id,
                start_dd, start_mm, start_yy, start_hh, start_nn, start_ss);

    w->open_step = EDF_STEP_HDR_WRITE;
    {
        uint32_t left = hdr_bytes;
        uint32_t pos = 0U;
        while (left > 0U)
        {
            UINT chunk = (left > 512U) ? 512U : (UINT)left;
            fr = writeExactRetry(&w->fil, &s_hdr[pos], chunk);
            if (fr != FR_OK)
            {
                (void)f_close(&w->fil);
                w->open = false;
                (void)f_unlink(path);
                return fr;
            }
            pos += chunk;
            left -= chunk;
        }
    }

    /* Commit the directory entry now so the file is never left at size 0. */
    w->open_step = EDF_STEP_HDR_SYNC;
    fr = syncRetry(&w->fil, 4U, 40U);
    if (fr != FR_OK)
    {
        (void)f_close(&w->fil);
        w->open = false;
        (void)f_unlink(path);
        return fr;
    }

    w->open_step = EDF_STEP_VERIFY;
    if (f_size(&w->fil) < (FSIZE_t)hdr_bytes)
    {
        (void)f_close(&w->fil);
        w->open = false;
        (void)f_unlink(path);
        return FR_DISK_ERR;
    }

    w->open_step = EDF_STEP_OK;
    return FR_OK;
}

FRESULT Edf_WriteRecords(EdfWriter_t *w, const int16_t *buf, uint32_t n_rec)
{
    FRESULT fr;
    UINT nbytes;

    if ((w == NULL) || !w->open || (buf == NULL) || (n_rec == 0U)
        || (w->bytes_per_record == 0U))
    {
        return FR_INVALID_OBJECT;
    }

    nbytes = (UINT)(n_rec * w->bytes_per_record);
    fr = writeExactRetry(&w->fil, buf, nbytes);
    if (fr != FR_OK)
    {
        /* Rewind to the last complete record so the caller's retry of this
         * same batch cannot leave a torn record in the stream. */
        const FSIZE_t good = (FSIZE_t)w->header_bytes
                           + ((FSIZE_t)w->n_records * (FSIZE_t)w->bytes_per_record);
        (void)f_lseek(&w->fil, good);
        return fr;
    }

    w->n_records += n_rec;
    w->since_flush = (uint16_t)(w->since_flush + n_rec);

    if (w->since_flush >= EDF_DATA_SYNC_EVERY)
    {
        /* Best effort: a failed sync must not duplicate the samples above. */
        if (f_sync(&w->fil) == FR_OK)
        {
            w->since_flush = 0U;
        }
    }
    return FR_OK;
}

FRESULT Edf_WriteRecord(EdfWriter_t *w)
{
    if ((w == NULL) || (w->sample_buf == NULL))
    {
        return FR_INVALID_OBJECT;
    }
    return Edf_WriteRecords(w, w->sample_buf, 1U);
}

FRESULT Edf_Close(EdfWriter_t *w)
{
    char num[16];
    char field[8];
    FSIZE_t expect;
    FRESULT fr = FR_DISK_ERR;
    uint8_t attempt;

    if ((w == NULL) || !w->open)
    {
        return FR_INVALID_OBJECT;
    }

    expect = (FSIZE_t)w->header_bytes
           + ((FSIZE_t)w->n_records * (FSIZE_t)w->bytes_per_record);

    (void)snprintf(num, sizeof(num), "%lu", (unsigned long)w->n_records);
    padCopy(field, sizeof(field), num);

    for (attempt = 0U; attempt < 4U; attempt++)
    {
        UINT bw = 0U;

        /* 1. flush pending data so the size on the medium is real */
        if (f_sync(&w->fil) != FR_OK)
        {
            HAL_Delay(50);
            continue;
        }
        /* 2. patch the record count in the fixed header */
        if (f_lseek(&w->fil, EDF_NREC_OFFSET) != FR_OK)
        {
            HAL_Delay(20);
            continue;
        }
        if ((f_write(&w->fil, field, 8U, &bw) != FR_OK) || (bw != 8U))
        {
            HAL_Delay(20);
            continue;
        }
        /* 3. drop any partial trailing record */
        if (f_lseek(&w->fil, expect) != FR_OK)
        {
            HAL_Delay(20);
            continue;
        }
        if (f_truncate(&w->fil) != FR_OK)
        {
            HAL_Delay(20);
            continue;
        }
        fr = f_sync(&w->fil);
        if (fr == FR_OK)
        {
            break;
        }
        HAL_Delay(50);
    }

    (void)f_close(&w->fil);
    w->open = false;
    w->since_flush = 0U;
    return fr;
}
