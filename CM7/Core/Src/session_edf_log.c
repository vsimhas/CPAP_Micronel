/**
  ******************************************************************************
  * @file    session_edf_log.c
  * @brief   Session EDF logger: 25 Hz therapy waveforms → SD FAT file.
  *
  *  RTOS safety: FatFs/SDMMC I/O runs ONLY in SessionEdf_Service() (call from
  *  SelfTestTask). Therapy_Start / SensorTask never block on the SD card.
  ******************************************************************************
  */

#include "session_edf_log.h"
#include "edf_writer.h"
#include "therapy.h"
#include "flow_sensor.h"
#include "blower_ctrl.h"
#include "leak_estimator.h"
#include "main.h"
#include "ff.h"

#include <stdio.h>
#include <string.h>

volatile uint8_t  g_session_edf_enable = 0U;
volatile uint8_t  g_session_edf_state  = (uint8_t)SESSION_EDF_IDLE;
volatile uint32_t g_session_edf_records = 0U;
volatile uint32_t g_session_edf_error  = 0U;
volatile uint32_t g_session_edf_fsize  = 0U;

#define EDF_HZ              25U
#define EDF_REC_S           1U
#define EDF_NSIG            4U
#define EDF_SAMPLES_PER_SIG (EDF_HZ * EDF_REC_S)
#define EDF_SAMPLE_BUF_N    (EDF_NSIG * EDF_SAMPLES_PER_SIG)
/* Queue several 1 s records so FatFs can write larger chunks (fewer SD txns). */
#define EDF_PENDING_MAX     8U

static FATFS s_fs;
static uint8_t s_mounted = 0U;
static EdfWriter_t s_edf;
static int16_t s_sample_buf[EDF_SAMPLE_BUF_N];
static int16_t s_pending_flat[EDF_PENDING_MAX * EDF_SAMPLE_BUF_N];
static int16_t s_write_flat[EDF_PENDING_MAX * EDF_SAMPLE_BUF_N];
static volatile uint8_t s_pend_count = 0U;
static uint8_t s_write_n = 0U;           /* records awaiting successful SD write */
static uint8_t s_write_fail_streak = 0U;
static uint16_t s_fill[EDF_NSIG];
static float s_accum_s = 0.0f;
static char s_path[32];
static uint8_t s_yy = 26U, s_mo = 1U, s_dd = 1U;
static uint8_t s_hh = 0U, s_mi = 0U, s_ss = 0U;
static uint8_t s_file_seq = 0U;

static volatile uint8_t s_req_start = 0U;
static volatile uint8_t s_req_stop = 0U;
static volatile uint8_t s_req_mount = 0U;
static volatile uint8_t s_collecting = 0U;

static const EdfSignalDesc_t s_sigs[EDF_NSIG] = {
    { "Flow",     "Patient flow", "L/min", -120.0f, 120.0f, -32768, 32767, "LP ~15Hz", EDF_SAMPLES_PER_SIG },
    { "Pressure", "Mask pressure", "cmH2O", 0.0f, 25.0f, -32768, 32767, "none", EDF_SAMPLES_PER_SIG },
    { "Leak",     "Unintentional leak", "L/min", 0.0f, 100.0f, -32768, 32767, "LP", EDF_SAMPLES_PER_SIG },
    { "TargetP",  "Therapy target", "cmH2O", 0.0f, 25.0f, -32768, 32767, "none", EDF_SAMPLES_PER_SIG },
};

static void setErr(uint32_t e)
{
    g_session_edf_error = e;
    g_session_edf_state = (uint8_t)SESSION_EDF_ERROR;
    s_collecting = 0U;
}

bool SessionEdf_IsMounted(void)
{
    return (s_mounted != 0U);
}

bool SessionEdf_Mount(void)
{
    FRESULT fr;
    if (s_mounted != 0U)
    {
        return true;
    }
    if (sd_card_present == 0U)
    {
        setErr(0x1000U); /* no card */
        return false;
    }
    /* Self-test still running/failed — keep Service retrying; not a permanent EDF fault. */
    if (sd_test_result == HAL_BUSY)
    {
        g_session_edf_error = 0x1002U;
        return false;
    }
    if (sd_test_result != HAL_OK)
    {
        g_session_edf_error = 0x1000U | ((uint32_t)sd_test_result & 0xFFU);
        return false;
    }
    if (sd_fatfs_ready == 0U)
    {
        g_session_edf_error = 0x1002U; /* settling after self-test */
        return false;
    }
    fr = f_mount(&s_fs, "0:", 1);
    if (fr != FR_OK)
    {
        /* One remount attempt after discarding a stale FatFs object. */
        (void)f_mount(NULL, "0:", 0);
        memset(&s_fs, 0, sizeof(s_fs));
        fr = f_mount(&s_fs, "0:", 1);
    }
    if (fr != FR_OK)
    {
        setErr((uint32_t)fr);
        return false;
    }
    s_mounted = 1U;
    g_session_edf_error = 0U;
    if (g_session_edf_state == (uint8_t)SESSION_EDF_ERROR)
    {
        g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
    }
    return true;
}

void SessionEdf_Unmount(void)
{
    if (s_edf.open)
    {
        (void)Edf_Close(&s_edf);
    }
    if (s_mounted != 0U)
    {
        (void)f_mount(NULL, "0:", 0);
        s_mounted = 0U;
    }
    s_collecting = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;
}

void SessionEdf_Init(void)
{
    g_session_edf_enable = 0U;
    g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
    g_session_edf_records = 0U;
    g_session_edf_error = 0U;
    s_mounted = 0U;
    s_path[0] = '\0';
    s_accum_s = 0.0f;
    s_req_start = 0U;
    s_req_stop = 0U;
    s_req_mount = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;
    s_collecting = 0U;
    memset(s_fill, 0, sizeof(s_fill));
    memset(&s_edf, 0, sizeof(s_edf));
}

void SessionEdf_SetEnable(bool en)
{
    g_session_edf_enable = en ? 1U : 0U;
    /* Mount (or leave unmounted) asynchronously in SessionEdf_Service(). */
    if (en)
    {
        s_req_mount = 1U;
    }
}

bool SessionEdf_IsEnabled(void)
{
    return (g_session_edf_enable != 0U);
}

void SessionEdf_SetDateTime(uint8_t yy, uint8_t mo, uint8_t dd,
                            uint8_t hh, uint8_t mi, uint8_t ss)
{
    s_yy = (yy > 99U) ? 99U : yy;
    s_mo = ((mo < 1U) || (mo > 12U)) ? 1U : mo;
    s_dd = ((dd < 1U) || (dd > 31U)) ? 1U : dd;
    s_hh = (hh > 23U) ? 0U : hh;
    s_mi = (mi > 59U) ? 0U : mi;
    s_ss = (ss > 59U) ? 0U : ss;
}

static const char* monthAbbrev(uint8_t mo)
{
    static const char *k[12] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                 "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };
    return ((mo >= 1U) && (mo <= 12U)) ? k[mo - 1U] : "JAN";
}

/* Pick the next unused 8.3 name of the day: YYMMDDNN.EDF, NN = 00..99. */
static bool pickSessionPath(void)
{
    FILINFO fno;
    uint16_t tries;

    for (tries = 0U; tries < 100U; tries++)
    {
        char name8[16];
        const uint8_t idx = (uint8_t)(s_file_seq % 100U);

        (void)snprintf(name8, sizeof(name8), "%02u%02u%02u%02u.EDF",
                       (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd,
                       (unsigned)idx);
        (void)snprintf(s_path, sizeof(s_path), "0:/%s", name8);
        s_file_seq++;

        if (f_stat(s_path, &fno) == FR_NO_FILE)
        {
            return true;
        }
    }
    /* All 100 slots for today exist — reuse the current one. */
    return true;
}

static bool openSessionFile(void)
{
    char patient[80];
    char recording[80];
    FRESULT fr;

    if (!SessionEdf_Mount())
    {
        return false;
    }

    if (!pickSessionPath())
    {
        setErr(0x2000U | (uint32_t)FR_DENIED);
        return false;
    }

    (void)snprintf(patient, sizeof(patient), "MCH-0000 X X SomnaLink_CPAP");
    (void)snprintf(recording, sizeof(recording),
                   "Startdate %02u-%s-20%02u SomnaLink CPAP session",
                   (unsigned)s_dd, monthAbbrev(s_mo), (unsigned)s_yy);

    memset(s_fill, 0, sizeof(s_fill));
    s_accum_s = 0.0f;
    g_session_edf_records = 0U;
    g_session_edf_fsize = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;

    fr = Edf_Open(&s_edf, s_path, patient, recording,
                  s_dd, s_mo, s_yy, s_hh, s_mi, s_ss,
                  EDF_REC_S, s_sigs, EDF_NSIG,
                  s_sample_buf, EDF_SAMPLE_BUF_N);
    if (fr != FR_OK)
    {
        /* 0x2sxx: s = Edf_Open stage that failed, xx = FRESULT. */
        setErr(0x2000U
               | (((uint32_t)s_edf.open_step & 0xFU) << 8)
               | ((uint32_t)fr & 0xFFU));
        return false;
    }
    g_session_edf_state = (uint8_t)SESSION_EDF_LOGGING;
    g_session_edf_error = 0U;
    g_session_edf_fsize = (uint32_t)f_size(&s_edf.fil);
    s_collecting = 1U;
    return true;
}

void SessionEdf_OnTherapyStart(void)
{
    /* Non-blocking: SelfTestTask opens the file via SessionEdf_Service(). */
    if (g_session_edf_enable == 0U)
    {
        return;
    }
    s_req_stop = 0U;
    s_req_start = 1U;
}

void SessionEdf_OnTherapyStop(void)
{
    s_req_start = 0U;
    s_req_stop = 1U;
    s_collecting = 0U;
}

static void pushSample(float flow, float press, float leak, float target)
{
    uint8_t i;
    const float vals[EDF_NSIG] = { flow, press, leak, target };

    /* Full queue checked when committing a 1 s record (see below). */

    for (i = 0U; i < EDF_NSIG; i++)
    {
        const uint16_t idx = s_fill[i];
        if (idx < EDF_SAMPLES_PER_SIG)
        {
            const uint32_t off = ((uint32_t)i * EDF_SAMPLES_PER_SIG) + idx;
            s_sample_buf[off] = Edf_PhysToDig(&s_sigs[i], vals[i]);
            s_fill[i] = (uint16_t)(idx + 1U);
        }
    }

    if (s_fill[0] >= EDF_SAMPLES_PER_SIG)
    {
        uint8_t slot;
        __disable_irq();
        slot = s_pend_count;
        if (slot < EDF_PENDING_MAX)
        {
            const uint32_t dst =
                (uint32_t)slot * (uint32_t)EDF_SAMPLE_BUF_N;
            memcpy(&s_pending_flat[dst], s_sample_buf, sizeof(s_sample_buf));
            s_pend_count = (uint8_t)(slot + 1U);
        }
        __enable_irq();
        memset(s_fill, 0, sizeof(s_fill));
    }
}

void SessionEdf_Update(float dt_s)
{
    /* SensorTask: RAM sampling only — never FatFs. */
    if ((s_collecting == 0U) || (g_session_edf_enable == 0U))
    {
        return;
    }
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    s_accum_s += dt_s;
    const float period = 1.0f / (float)EDF_HZ;
    while (s_accum_s >= period)
    {
        s_accum_s -= period;
        pushSample(Flow_GetPatientSlm(),
                   BlowerCtrl_GetMeasured(),
                   Leak_GetUnintentionalSlm(),
                   Therapy_GetTargetPressure());
    }
}

void SessionEdf_Service(void)
{
    /* SelfTestTask / low-priority context only. */
    if (s_req_stop != 0U)
    {
        s_req_stop = 0U;
        s_collecting = 0U;
        if (s_edf.open)
        {
            uint8_t n;
            /* Drain pending queue into write buffer if idle. */
            if (s_write_n == 0U)
            {
                __disable_irq();
                n = s_pend_count;
                if (n != 0U)
                {
                    memcpy(s_write_flat, s_pending_flat,
                           (size_t)n * sizeof(s_sample_buf));
                    s_pend_count = 0U;
                }
                __enable_irq();
                s_write_n = n;
            }
            if (s_write_n != 0U)
            {
                (void)Edf_WriteRecords(&s_edf, s_write_flat, s_write_n);
                s_write_n = 0U;
            }
            {
                FRESULT cr = Edf_Close(&s_edf);
                g_session_edf_records = s_edf.n_records;
                if (cr != FR_OK)
                {
                    /* Close/flush failed — keep mount; report write error only. */
                    g_session_edf_error = 0x3100U | ((uint32_t)cr & 0xFFU);
                }
                else
                {
                    g_session_edf_fsize =
                        (uint32_t)(s_edf.header_bytes
                                   + (s_edf.n_records * s_edf.bytes_per_record));
                }
            }
        }
        if (g_session_edf_state == (uint8_t)SESSION_EDF_LOGGING)
        {
            g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
        }
        /* Volume stays mounted while EDF remains enabled. */
        s_pend_count = 0U;
        s_write_n = 0U;
        s_write_fail_streak = 0U;
    }

    if (s_req_mount != 0U)
    {
        if (g_session_edf_enable == 0U)
        {
            s_req_mount = 0U;
        }
        else if (SessionEdf_Mount())
        {
            s_req_mount = 0U;
            g_session_edf_error = 0U;
            if (g_session_edf_state == (uint8_t)SESSION_EDF_ERROR)
            {
                g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
            }
        }
        else if ((sd_test_result != HAL_OK) || (sd_fatfs_ready == 0U))
        {
            /* Keep trying while self-test retries / settle completes. */
            s_req_mount = 1U;
        }
        else
        {
            /* Hard FatFs/disk failure — stop hammering SDMMC. Re-Apply Enable to retry. */
            s_req_mount = 0U;
            if (g_session_edf_state == (uint8_t)SESSION_EDF_ERROR)
            {
                /* leave ERROR so Query shows err; clear on next successful mount */
            }
        }
    }

    if (s_req_start != 0U)
    {
        s_req_start = 0U;
        if (g_session_edf_enable == 0U)
        {
            return;
        }
        if ((sd_card_present == 0U) || (sd_test_result != HAL_OK) || (sd_fatfs_ready == 0U))
        {
            /* Soft fail — keep start request until SD is ready, or user stops. */
            g_session_edf_error = 0x1001U;
            s_req_start = 1U;
            return;
        }
        if (s_edf.open)
        {
            (void)Edf_Close(&s_edf);
        }
        if (!openSessionFile())
        {
            /* openSessionFile already set err; leave ERROR so status shows it. */
            return;
        }
    }

    if (s_edf.open)
    {
        /* Pull new samples only when the previous SD write finished. */
        if (s_write_n == 0U)
        {
            uint8_t n;
            __disable_irq();
            n = s_pend_count;
            if (n != 0U)
            {
                memcpy(s_write_flat, s_pending_flat,
                       (size_t)n * sizeof(s_sample_buf));
                s_pend_count = 0U;
            }
            __enable_irq();
            s_write_n = n;
        }

        if (s_write_n != 0U)
        {
            if (Edf_WriteRecords(&s_edf, s_write_flat, s_write_n) != FR_OK)
            {
                s_write_fail_streak++;
                g_session_edf_error = 0x3000U; /* sticky; keep logging while retrying */
                if (s_write_fail_streak >= 8U)
                {
                    setErr(0x3000U);
                    (void)Edf_Close(&s_edf);
                    s_collecting = 0U;
                    s_write_n = 0U;
                    s_write_fail_streak = 0U;
                }
                /* else: leave s_write_n set and retry next Service tick */
            }
            else
            {
                g_session_edf_records = s_edf.n_records;
                g_session_edf_error = 0U;
                g_session_edf_fsize = (uint32_t)f_size(&s_edf.fil);
                s_write_n = 0U;
                s_write_fail_streak = 0U;
            }
        }
    }
}

SessionEdfState_t SessionEdf_GetState(void)
{
    return (SessionEdfState_t)g_session_edf_state;
}

const char* SessionEdf_GetStateName(void)
{
    switch ((SessionEdfState_t)g_session_edf_state)
    {
    case SESSION_EDF_LOGGING: return "Logging";
    case SESSION_EDF_ERROR:   return "Error";
    case SESSION_EDF_IDLE:
    default:                  return "Idle";
    }
}

const char* SessionEdf_GetLastPath(void)
{
    return s_path;
}

uint32_t SessionEdf_GetRecords(void)
{
    return g_session_edf_records;
}

uint32_t SessionEdf_GetLastError(void)
{
    return g_session_edf_error;
}

uint32_t SessionEdf_GetFileSize(void)
{
    return g_session_edf_fsize;
}
