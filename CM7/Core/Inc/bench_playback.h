/**
  ******************************************************************************
  * @file    bench_playback.h
  * @brief   Bench only: replay a recorded speed command to CM4 at the control
  *          tick, exactly the way therapy posts it.
  *
  *  Why this exists
  *  ---------------
  *  In therapy BlowerCtrl_Step posts a new speed to CM4 every 5 ms tick with
  *  ramp 0, so the CM4 speed reference STEPS. On 2026-10-01 a stiffer CM4
  *  speed loop (speed_kp 4000) passed every PC-driven bench test and then
  *  tripped DP_FAULT six times in therapy: the PC can only send ~10 commands
  *  a second (each one returns a status line), so its replays used 100 ms
  *  ramps and smoothed away the very steps that caused the trips.
  *
  *  This module holds a recorded command in RAM and posts it from the control
  *  loop at tick rate with ramp 0 (sample-and-hold between samples), so CM4
  *  changes can be tested on the leak rig against therapy-like references
  *  without anyone on the mask.
  *
  *  Use (telemetry commands, see telemetry_stream.c):
  *      blower manual start 30000 3000      (manual mode, running)
  *      bench play clear
  *      bench play period 20                (ms per sample, multiple of 5)
  *      bench play add 31000,31250,...      (repeat; <= 96 chars per line)
  *      bench play start                    -> "# PLAY,start,<ms>,<n>,<period>"
  *      bench play stop / status            -> "# PLAY,end,<ms>,<reason>,<idx>"
  *
  *  Guards: runs only in manual blower mode with therapy idle and the
  *  pressure controller disabled; stops on leaving that, or on a CM4 fault
  *  state. Samples are clamped to [BENCH_PLAY_MIN_RPM, g_blower_ctrl_rpm_max]
  *  (the safety ceiling). When the recording ends the last sample is held
  *  (manual mode continues).
  *
  *  Build with BENCH_PLAYBACK 0 to compile it out (all calls become no-ops).
  ******************************************************************************
  */

#ifndef BENCH_PLAYBACK_H
#define BENCH_PLAYBACK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BENCH_PLAYBACK
#define BENCH_PLAYBACK  1
#endif

#define BENCH_PLAY_MAX_SAMPLES  3000U    /* 60 s at 20 ms */
#define BENCH_PLAY_MIN_RPM      15000

#if BENCH_PLAYBACK

void     BenchPlay_Clear(void);
/* Append comma-separated rpm values. Returns the number appended, or -1 on a
 * parse error or a full buffer (nothing appended in that case). */
int      BenchPlay_Append(const char *csv);
bool     BenchPlay_SetPeriodMs(uint32_t period_ms);
/* false (and nothing changes) unless loaded, manual mode, therapy idle and
 * CM4 in RUN. */
bool     BenchPlay_Start(void);
void     BenchPlay_Stop(void);
/* Call once per control tick, after BlowerCtrl_Step. */
void     BenchPlay_Tick(float dt_s);
bool     BenchPlay_IsActive(void);
int32_t  BenchPlay_CurrentRpm(void);
uint32_t BenchPlay_Count(void);
uint32_t BenchPlay_Index(void);
uint32_t BenchPlay_PeriodMs(void);

#else

#define BenchPlay_Clear()           ((void)0)
#define BenchPlay_Append(c)         (-1)
#define BenchPlay_SetPeriodMs(p)    (false)
#define BenchPlay_Start()           (false)
#define BenchPlay_Stop()            ((void)0)
#define BenchPlay_Tick(dt)          ((void)0)
#define BenchPlay_IsActive()        (false)
#define BenchPlay_CurrentRpm()      (0)
#define BenchPlay_Count()           (0U)
#define BenchPlay_Index()           (0U)
#define BenchPlay_PeriodMs()        (0U)

#endif /* BENCH_PLAYBACK */

#ifdef __cplusplus
}
#endif

#endif /* BENCH_PLAYBACK_H */
