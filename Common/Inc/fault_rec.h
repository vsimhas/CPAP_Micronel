/**
  * @file    fault_rec.h
  * @brief   Drive fault recorder, shared by both cores in D3 SRAM4.
  *
  * Why: DP_FAULT (the STSPIN830 fault pin -> TIM8 break) trips the U65 at
  * ~33 krpm on the open air path, while the 25 Hz B-line telemetry shows
  * nothing unusual before it (iq ~1.25 A sampled, bus flat). Whatever trips
  * the driver is faster than that, so the CM4 keeps the last FAULT_REC_N
  * FOC ticks (20 kHz: 25.6 ms) in a ring and freezes it on the break IRQ or
  * on any MC fault state. It re-arms on the next start.
  *
  * First result (2026-10-29): the 25.6 ms before two U65 trips were clean -
  * sinusoidal phase currents peaking 1.6-1.7 A, sampling windows >= 5 us,
  * |V| 78 % of linear, bus flat - so the trips were the STSPIN830's thermal
  * shutdown (sustained ~1.4 A; the chip was hot), not the control loop. See
  * the I2t fold-back in blower_ipc_cm4.c.
  *
  * CM4 writes (high-frequency task, every FOC tick while armed); CM7 only
  * reads, through "foc rec" on the USB console. The CM7 maps this region
  * non-cacheable (MPU region 4, 16 KB at 0x38000000 - keep in step with the
  * RW_FREC scatter region in both .sct files).
  *
  * Values are raw drive units; the header carries what the PC needs to
  * convert them:
  *   ia, ib, iq, id   current, A = value * amp_per_digit
  *   vq, vd           voltage, fraction of full scale = value / 32768
  *                    (full scale = Vbus / sqrt3, linear-modulation limit)
  *   el_angle         electrical angle, deg = value * 180 / 32768
  *   ccr1..3          TIM8 compare values, duty = ccr / pwm_period
  *   sector           SVPWM sector (also picks the two sampled phases)
  */
#ifndef FAULT_REC_H
#define FAULT_REC_H

#include <stdint.h>

#define FAULT_REC_MAGIC   0x46524543U   /* 'FREC' */
#define FAULT_REC_N       512U          /* 25.6 ms at 20 kHz */

#define FAULT_REC_SRC_NONE   0U
#define FAULT_REC_SRC_BREAK  1U         /* TIM8 break IRQ (DP_FAULT) */
#define FAULT_REC_SRC_STATE  2U         /* MC state machine in FAULT_NOW / FAULT_OVER */
#define FAULT_REC_SRC_MANUAL 3U         /* "foc rec freeze" from the CM7 console (a snapshot, no fault) */

typedef struct
{
  int16_t  ia;
  int16_t  ib;
  int16_t  iq;
  int16_t  id;
  int16_t  vq;
  int16_t  vd;
  int16_t  el_angle;
  uint16_t ccr1;
  uint16_t ccr2;
  uint16_t ccr3;
  uint8_t  sector;
  uint8_t  pad;
  uint16_t pad2;
} FaultRecSample_t;                     /* 24 bytes */

typedef struct
{
  uint32_t          magic;
  volatile uint32_t arm_seq;            /* +1 on every arm */
  volatile uint32_t freeze_seq;         /* +1 on every freeze */
  volatile uint16_t head;               /* next slot to write */
  volatile uint8_t  armed;
  volatile uint8_t  wrapped;            /* ring has filled at least once */
  volatile uint8_t  frozen;
  volatile uint8_t  trig_source;        /* FAULT_REC_SRC_* */
  volatile uint16_t trig_faults;        /* MC fault bits at the freeze */
  volatile uint16_t trig_head;          /* head at the freeze */
  volatile uint16_t trig_vbus_mv;
  volatile int32_t  trig_speed_rpm;
  volatile uint32_t pwm_hz;
  volatile uint16_t pwm_period;         /* TIM8 ARR: ccr / pwm_period = duty */
  uint16_t          rsv;
  volatile float    amp_per_digit;
  uint32_t          rsv2[4];
  FaultRecSample_t  s[FAULT_REC_N];
} FaultRec_t;

extern FaultRec_t g_fault_rec;

#if defined(CORE_CM4)
void FaultRec_Arm(void);
void FaultRec_Record(void);             /* FOC high-frequency task, after the current loop */
void FaultRec_Freeze(uint8_t source, uint16_t faults);
#endif

#endif /* FAULT_REC_H */
