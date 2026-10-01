/**
  ******************************************************************************
  * @file    bench_playback.c
  * @brief   Implementation: tick-rate, ramp-0 replay of a recorded speed
  *          command (bench only - see bench_playback.h).
  ******************************************************************************
  */

#include "bench_playback.h"

#if BENCH_PLAYBACK

#include <stdio.h>
#include <stdlib.h>

#include "stm32h7xx_hal.h"
#include "blower_ipc.h"
#include "blower_ctrl.h"
#include "therapy.h"
#include "telemetry_stream.h"

extern BlowerIpcStatus_t g_blower_status;     /* main.c, refreshed at 100 Hz */
extern volatile int32_t  g_blower_ctrl_rpm_max;

#define BENCH_PLAY_TICK_MS      5U
#define BENCH_PLAY_PERIOD_MAX   1000U

/* uint16_t: rpm fits (< 65536) and 3000 samples cost 6 KB instead of 12.
 *
 * Threads: the commands (Clear/Append/Start/Stop) run in SystemTask, the
 * tick in SensorTask, which has the higher priority and can preempt them.
 * Start fills every field before setting s_active, and Append/SetPeriod are
 * refused while active, so the tick never sees a half-loaded state. */
static uint16_t          s_rpm[BENCH_PLAY_MAX_SAMPLES];
static volatile uint32_t s_count     = 0U;
static volatile uint32_t s_period_ms = 20U;
static volatile bool     s_active    = false;
static float             s_elapsed_s = 0.0f;   /* tick only */
static volatile uint32_t s_index     = 0U;
static volatile int32_t  s_last_rpm  = 0;

static void benchPlayNoteEnd(const char *reason)
{
    char line[80];
    const int n = snprintf(line, sizeof(line), "# PLAY,end,%lu,%s,%lu\r\n",
                           (unsigned long)HAL_GetTick(), reason,
                           (unsigned long)s_index);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
        Telem_PushNote(line);
    }
}

/* Bench mode only: manual blower, controller off, therapy idle. */
static bool benchPlayModeOk(void)
{
    return BlowerCtrl_IsManualActive()
        && !BlowerCtrl_IsEnabled()
        && (Therapy_GetState() == THERAPY_STATE_IDLE);
}

static int32_t benchPlayClamp(int32_t rpm)
{
    int32_t hi = g_blower_ctrl_rpm_max;
    if (hi < BENCH_PLAY_MIN_RPM)
    {
        hi = BENCH_PLAY_MIN_RPM;
    }
    if (rpm < BENCH_PLAY_MIN_RPM) { return BENCH_PLAY_MIN_RPM; }
    if (rpm > hi)                 { return hi; }
    return rpm;
}

void BenchPlay_Clear(void)
{
    if (s_active)
    {
        BenchPlay_Stop();
    }
    s_count = 0U;
}

int BenchPlay_Append(const char *csv)
{
    uint16_t vals[32];
    uint32_t n = 0U;
    const char *p = csv;

    if ((csv == NULL) || s_active)
    {
        return -1;
    }
    /* Parse first, commit only if the whole line is valid and fits. */
    while (*p != '\0')
    {
        char *end = NULL;
        const long v = strtol(p, &end, 10);
        if ((end == p) || (v < 0L) || (v > 65535L) || (n >= 32U))
        {
            return -1;
        }
        vals[n++] = (uint16_t)v;
        p = end;
        while ((*p == ',') || (*p == ' '))
        {
            p++;
        }
    }
    if ((n == 0U) || ((s_count + n) > BENCH_PLAY_MAX_SAMPLES))
    {
        return -1;
    }
    for (uint32_t i = 0U; i < n; i++)
    {
        s_rpm[s_count + i] = vals[i];
    }
    s_count += n;
    return (int)n;
}

bool BenchPlay_SetPeriodMs(uint32_t period_ms)
{
    if (s_active || (period_ms < BENCH_PLAY_TICK_MS)
        || (period_ms > BENCH_PLAY_PERIOD_MAX)
        || ((period_ms % BENCH_PLAY_TICK_MS) != 0U))
    {
        return false;
    }
    s_period_ms = period_ms;
    return true;
}

bool BenchPlay_Start(void)
{
    if ((s_count == 0U) || s_active || !benchPlayModeOk()
        || (g_blower_status.mc_state != 6U))      /* MCSDK RUN */
    {
        return false;
    }
    s_elapsed_s = 0.0f;
    s_index     = 0U;
    s_last_rpm  = benchPlayClamp((int32_t)s_rpm[0]);
    s_active    = true;                    /* last: the tick may run next */

    char line[80];
    const int n = snprintf(line, sizeof(line), "# PLAY,start,%lu,%lu,%lu\r\n",
                           (unsigned long)HAL_GetTick(), (unsigned long)s_count,
                           (unsigned long)s_period_ms);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
        Telem_PushNote(line);
    }
    return true;
}

void BenchPlay_Stop(void)
{
    if (s_active)
    {
        s_active = false;
        benchPlayNoteEnd("stop");
    }
}

void BenchPlay_Tick(float dt_s)
{
    if (!s_active)
    {
        return;
    }
    if (!benchPlayModeOk())
    {
        s_active = false;
        benchPlayNoteEnd("mode");
        return;
    }
    /* FAULT_NOW / FAULT_OVER: CM4 has already switched PWM off; stop posting
     * so the replay does not drive an automatic restart. */
    if ((g_blower_status.mc_state == 10U) || (g_blower_status.mc_state == 11U))
    {
        s_active = false;
        benchPlayNoteEnd("fault");
        return;
    }

    if ((dt_s > 0.0f) && (dt_s < 0.1f))
    {
        s_elapsed_s += dt_s;
    }
    const uint32_t idx = (uint32_t)((s_elapsed_s * 1000.0f) / (float)s_period_ms);
    if (idx >= s_count)
    {
        s_active = false;
        s_index  = s_count;
        benchPlayNoteEnd("done");          /* last sample stays in force */
        return;
    }
    s_index    = idx;
    s_last_rpm = benchPlayClamp((int32_t)s_rpm[idx]);

    /* Every tick, ramp 0 - the same call and cadence as BlowerCtrl_Step. */
    (void)BlowerIpc_CM7_SetSpeedRpmSilent(s_last_rpm);
}

bool     BenchPlay_IsActive(void)   { return s_active; }
int32_t  BenchPlay_CurrentRpm(void) { return s_last_rpm; }
uint32_t BenchPlay_Count(void)      { return s_count; }
uint32_t BenchPlay_Index(void)      { return s_index; }
uint32_t BenchPlay_PeriodMs(void)   { return s_period_ms; }

#endif /* BENCH_PLAYBACK */
