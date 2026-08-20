/**
  ******************************************************************************
  * @file    session_edf_log.c
  * @brief   Session EDF logger: 25 Hz therapy waveforms → SD FAT file.
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

volatile uint8_t  g_session_edf_enable = 1U;
volatile uint8_t  g_session_edf_state  = (uint8_t)SESSION_EDF_IDLE;
volatile uint32_t g_session_edf_records = 0U;
volatile uint32_t g_session_edf_error  = 0U;

#define EDF_HZ              25U
#define EDF_REC_S           1U
#define EDF_NSIG            4U
#define EDF_SAMPLES_PER_SIG (EDF_HZ * EDF_REC_S)
#define EDF_SAMPLE_BUF_N    (EDF_NSIG * EDF_SAMPLES_PER_SIG)

static FATFS s_fs;
static uint8_t s_mounted = 0U;
static EdfWriter_t s_edf;
static int16_t s_sample_buf[EDF_SAMPLE_BUF_N];
static uint16_t s_fill[EDF_NSIG];
static float s_accum_s = 0.0f;
static char s_path[32];
static uint8_t s_yy = 26U, s_mo = 1U, s_dd = 1U;
static uint8_t s_hh = 0U, s_mi = 0U, s_ss = 0U;
static uint8_t s_file_seq = 0U;

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
    if ((sd_card_present == 0U) || (sd_test_result != HAL_OK))
    {
        setErr(0x1000U);
        return false;
    }
    fr = f_mount(&s_fs, "0:", 1);
    if (fr != FR_OK)
    {
        setErr((uint32_t)fr);
        return false;
    }
    s_mounted = 1U;
    /* Ensure directory exists (8.3 name). */
    (void)f_mkdir("0:/SLINK");
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
}

void SessionEdf_Init(void)
{
    g_session_edf_enable = 1U;
    g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
    g_session_edf_records = 0U;
    g_session_edf_error = 0U;
    s_mounted = 0U;
    s_path[0] = '\0';
    s_accum_s = 0.0f;
    memset(s_fill, 0, sizeof(s_fill));
    memset(&s_edf, 0, sizeof(s_edf));
}

void SessionEdf_SetEnable(bool en)
{
    g_session_edf_enable = en ? 1U : 0U;
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

static bool openSessionFile(void)
{
    char patient[40];
    char recording[40];

    if (!SessionEdf_Mount())
    {
        return false;
    }

    s_file_seq++;
    (void)snprintf(s_path, sizeof(s_path), "0:/SLINK/%02u%02u%02u%02u.EDF",
                   (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd,
                   (unsigned)(s_file_seq % 100U));

    (void)snprintf(patient, sizeof(patient), "X X X SomnaLink");
    (void)snprintf(recording, sizeof(recording),
                   "Startdate %02u-%02u-20%02u CPAP session",
                   (unsigned)s_dd, (unsigned)s_mo, (unsigned)s_yy);

    memset(s_fill, 0, sizeof(s_fill));
    s_accum_s = 0.0f;
    g_session_edf_records = 0U;

    if (!Edf_Open(&s_edf, s_path, patient, recording,
                  s_dd, s_mo, s_yy, s_hh, s_mi, s_ss,
                  EDF_REC_S, s_sigs, EDF_NSIG,
                  s_sample_buf, EDF_SAMPLE_BUF_N))
    {
        setErr(0x2000U);
        return false;
    }
    g_session_edf_state = (uint8_t)SESSION_EDF_LOGGING;
    g_session_edf_error = 0U;
    return true;
}

void SessionEdf_OnTherapyStart(void)
{
    if (g_session_edf_enable == 0U)
    {
        return;
    }
    if (s_edf.open)
    {
        (void)Edf_Close(&s_edf);
    }
    if (!openSessionFile())
    {
        /* Soft-fail: therapy continues without logging. */
        return;
    }
}

void SessionEdf_OnTherapyStop(void)
{
    if (s_edf.open)
    {
        /* Flush partial record with last sample held if needed — skip partial. */
        (void)Edf_Close(&s_edf);
        g_session_edf_records = s_edf.n_records;
    }
    if (g_session_edf_state == (uint8_t)SESSION_EDF_LOGGING)
    {
        g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
    }
}

static void pushSample(float flow, float press, float leak, float target)
{
    uint8_t i;
    const float vals[EDF_NSIG] = { flow, press, leak, target };

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

    /* Record complete when first signal buffer is full (all share same rate). */
    if (s_fill[0] >= EDF_SAMPLES_PER_SIG)
    {
        if (!Edf_WriteRecord(&s_edf))
        {
            setErr(0x3000U);
            (void)Edf_Close(&s_edf);
            return;
        }
        g_session_edf_records = s_edf.n_records;
        memset(s_fill, 0, sizeof(s_fill));
    }
}

void SessionEdf_Update(float dt_s)
{
    if ((g_session_edf_state != (uint8_t)SESSION_EDF_LOGGING) || !s_edf.open)
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
        if (g_session_edf_state != (uint8_t)SESSION_EDF_LOGGING)
        {
            break;
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
