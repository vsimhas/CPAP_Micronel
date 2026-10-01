/**
  * @file    blower_ipc_cm4.c
  * @brief   CM4-side blower IPC command handling and regen brake control.
  */

#include "blower_ipc.h"
#include "main.h"
#include "mc_api.h"
#include "mc_config_common.h"
#include "mc_stm_types.h"
#include "mc_interface.h"
#include "bus_voltage_sensor.h"
#include "mc_config.h"
#include "pid_regulator.h"
#include "parameters_conversion.h"   /* CURRENT_CONV_FACTOR_INV */

extern TIM_HandleTypeDef htim3;

#define BRAKE_PWM_FREQ_HZ           10000U
#define BRAKE_TIM_CLK_HZ            200000000UL
/* FOC OV trips at OV_VOLTAGE_THRESHOLD_V (28.0 V since 2026-09-25; 24.9 V
 * when the text below was written). Engage brake early so a mask-off /
 * therapy soft decel does not kill PWM and coast to 0.
 *
 * 2026-09-24: ON 22 -> 26 to DISABLE the brake for now. These thresholds were
 * set against the mis-scaled bus reading, back when a real 23.9 V bus was
 * reported as 20-21 V, so 22 was comfortably above anything ever seen and the
 * brake in practice never armed. Correcting VBUS_PARTITIONING_FACTOR on
 * 2026-09-24 made the same bus report 23, which is >= 22, so the first
 * deceleration latched the brake and it then never released - release needs
 * vbus <= BRAKE_VBUS_OFF_V, and a 23.9 V supply never goes there. Measured
 * result: 250 permille continuous into R154, about 5.7 W of standing heat in
 * a 25 W resistor. Not damaging, but wrong, and it was found by the resistor
 * getting hot rather than by anything in firmware noticing.
 *
 * 26 V is above the supply, so the brake cannot arm. That restores the
 * behaviour the machine has actually had all along.
 *
 * TO RE-ENABLE PROPERLY, two things are needed and neither is this line:
 *   1. Whole-volt resolution cannot express a usable window. The bus sits at
 *      23.9 V and OV trips at 24.9 V, so the entire brake range is 1 V and
 *      VBS_GetAvBusVoltage_V() can only say 24 or 25 inside it. Use
 *      VBS_GetAvBusVoltage_d() (~0.8 mV/count, already used for
 *      g_blower_ipc.bus_voltage_mv) and thresholds in mV.
 *   2. The window has to be widened, which means knowing the DC-link
 *      capacitor rating and raising OV_VOLTAGE_THRESHOLD_V to suit. Sizing a
 *      brake against a trip point that may move is wasted work.
 *
 * Note the brake is what stops a fast decel tripping OV, so while it is
 * disabled, watch for Over Voltage on mask-off parks from high pressure.
 *
 * 2026-09-25: OV raised 24.9 -> 28.0 V (drive_parameters.h), which also
 * answers point 2 above. ON moves 26 -> 29 with it so the brake STAYS
 * disabled: at 26 it would now start switching between 26 and 28 V, i.e.
 * a silent re-enable. Re-enabling is a separate decision (R154 rating,
 * whole-volt resolution - point 1). */
#define BRAKE_VBUS_ON_V             29U
#define BRAKE_VBUS_OFF_V            21U
#define BRAKE_DECEL_RPM_PER_MS      8
#define BRAKE_MAX_PERMILLE            900U

static uint32_t s_last_cmd_seq;
static uint32_t s_last_foc_tuning_seq;
static int32_t s_prev_speed_rpm;
static bool s_brake_latched;

static int16_t BlowerIpc_RpmToSpeedUnit(int32_t rpm)
{
  /* SPEED_UNIT/U_RPM is typically 10/60 = 1/6, so even at 36528 RPM
   * (workbench app limit) the result is ~6088, which fits comfortably in
   * an int16. We still do the math in int32 to avoid intermediate overflow. */
  return (int16_t)((rpm * (int32_t)SPEED_UNIT) / (int32_t)U_RPM);
}

static int32_t BlowerIpc_SpeedUnitToRpm(int16_t speed_unit)
{
  return ((int32_t)speed_unit * (int32_t)U_RPM) / (int32_t)SPEED_UNIT;
}

/* === Restart-while-coasting guard ========================================
 *
 * Never start the motor while the rotor may still be coasting.
 *
 * The sensorless start (charge boot cap -> offset calib -> alignment ->
 * open-loop rev-up -> switch-over) assumes a rotor at rest. Measured
 * 2026-09-25 on the bench:
 *   - therapy restarted 3.1 s after a stop from 18 krpm, rotor still at
 *     roughly 2 krpm: the drive went to RUN and reported mech tracking the
 *     command up to 32 krpm, but iq ~0 and pressure/flow ~0 - it was not
 *     driving the fan at all, and nothing faulted;
 *   - an OVER_VOLT trip at ~32 krpm switched PWM off; SET_SPEED kept
 *     arriving and each one acknowledged and restarted into the spinning
 *     rotor: six DP_FAULTs in ~1 s, then a start drawing 2.66 A.
 *
 * Once PWM is off the CM4 cannot see the rotor - the angle/speed estimator
 * is disabled in STOP/IDLE/FAULT and its output freezes - so the guard is
 * time-based. When drive stops (leaving RUN/START/SWITCH_OVER/ALIGNMENT, by
 * STOP or by a fault) it records the last driven speed and holds any start
 * off for BASE + PER_KRPM x speed.
 *
 * Sizing, from the sensor-pressure decay after therapy stops (pressure
 * tracks rotor speed, closed circuit with vent): the rotor is below ~1.5
 * krpm 3.3-3.7 s after a stop from 18 k, 3.8-4.3 s from 23 k, 4.2-5.2 s from
 * 33 k. Below that pressure cannot follow it, so the hold-off leaves a wide
 * margin for the slow bearing-friction tail: 6.7 s from 18 k (the failed
 * restart was at 3.1 s), 9.0 s from 33 k, 10 s from 40 k. Restarts that
 * waited 10-12 s all started cleanly (18 starts on 2026-09-25).
 *
 * A start that arrives inside the hold-off is not dropped: it is kept
 * pending and issued from the main loop once the hold-off has run out and
 * the state machine is in IDLE. STOP cancels it. The remaining hold-off is
 * published as restart_holdoff_ms so CM7 can hold its low-pressure grace
 * and so it shows on the B line. */
#define RESTART_HOLDOFF_BASE_MS      4000U
#define RESTART_HOLDOFF_MS_PER_KRPM   150U
/* Below this the rotor was never really spun up (alignment, a failed start
 * at crawl speed): nothing to wait for. */
#define RESTART_HOLDOFF_MIN_RPM      1000U

static bool     s_drive_on;          /* PWM was driving the rotor           */
static uint32_t s_drive_rpm;         /* |mech| while it was                 */
static uint32_t s_coast_start_ms;    /* when drive stopped                  */
static uint32_t s_coast_holdoff_ms;  /* hold-off length; 0 = none           */
static bool     s_start_pending;     /* start requested during the hold-off */
/* STOP accepted but the state machine has not left the driven states yet
 * (it acts on the next medium-frequency tick). A START in that window must
 * survive the stop, not be mistaken for "already running". */
static bool     s_stop_in_flight;

static bool BlowerIpc_StateDrives(MCI_State_t st)
{
  return (st == RUN) || (st == START) || (st == SWITCH_OVER) ||
         (st == ALIGNMENT);
}

/* Main-loop context only: it is the sole writer of the coast state. */
static void BlowerIpc_CM4_TrackCoast(void)
{
  const MCI_State_t st = MC_GetSTMStateMotor1();
  if (BlowerIpc_StateDrives(st))
  {
    int32_t mech = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
    if (mech < 0)
    {
      mech = -mech;
    }
    s_drive_rpm = (uint32_t)mech;
    s_drive_on = true;
    s_coast_holdoff_ms = 0U;
  }
  else if (s_drive_on)
  {
    /* Drive just stopped. The estimate froze at the last driven speed,
     * which is what the rotor is coasting down from. */
    s_drive_on = false;
    s_stop_in_flight = false;
    s_coast_start_ms = HAL_GetTick();
    s_coast_holdoff_ms = (s_drive_rpm < RESTART_HOLDOFF_MIN_RPM)
        ? 0U
        : RESTART_HOLDOFF_BASE_MS
          + ((s_drive_rpm * RESTART_HOLDOFF_MS_PER_KRPM) / 1000U);
  }
}

/* Pure read, safe from the medium-frequency hook as well. */
static uint32_t BlowerIpc_CM4_HoldoffRemainingMs(void)
{
  const uint32_t holdoff = s_coast_holdoff_ms;
  if (holdoff == 0U)
  {
    return 0U;
  }
  const uint32_t elapsed = HAL_GetTick() - s_coast_start_ms;
  return (elapsed >= holdoff) ? 0U : (holdoff - elapsed);
}

/* Replaces the direct MC_StartMotor1() calls in START and SET_SPEED. */
static void BlowerIpc_CM4_RequestStart(void)
{
  HAL_GPIO_WritePin(STSPIN_STBY_GPIO_Port, STSPIN_STBY_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOT_POW_EN_GPIO_Port, MOT_POW_EN_Pin, GPIO_PIN_SET);
  s_start_pending = true;
}

/* Issues a pending start once the rotor has had time to stop. Called every
 * main-loop pass. MCI_StartMotor only accepts IDLE, so acknowledge a latched
 * fault first and start on a later pass - the ack takes effect on the next
 * state-machine tick, and a start queued behind it would be refused. */
static void BlowerIpc_CM4_ServicePendingStart(void)
{
  if (!s_start_pending)
  {
    return;
  }
  const MCI_State_t st = MC_GetSTMStateMotor1();
  if (BlowerIpc_StateDrives(st))
  {
    if (!s_stop_in_flight)
    {
      s_start_pending = false;   /* already running (e.g. Motor Pilot) */
    }
    /* else: a STOP is about to take effect; keep the start for after it. */
    return;
  }
  if (BlowerIpc_CM4_HoldoffRemainingMs() > 0U)
  {
    return;
  }
  if (st == FAULT_OVER)
  {
    (void)MC_AcknowledgeFaultMotor1();
    return;
  }
  if (st == IDLE)
  {
    s_start_pending = false;
    (void)MC_StartMotor1();
  }
  /* STOP / WAIT_STOP_MOTOR / FAULT_NOW: settling, try again next pass. */
}

static void BlowerBrake_ApplyPermille(uint16_t permille)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
  uint32_t pulse;

  if (permille > 1000U)
  {
    permille = 1000U;
  }

  pulse = ((uint32_t)permille * (period + 1U)) / 1000U;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
  g_blower_ipc.brake_pwm_permille = permille;
  g_blower_ipc.brake_active = (permille > 0U) ? 1U : 0U;
}

void BlowerBrake_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  uint32_t prescaler;
  uint32_t period;

  prescaler = (BRAKE_TIM_CLK_HZ / (BRAKE_PWM_FREQ_HZ * 1000UL)) - 1U;
  if (prescaler == 0xFFFFFFFFUL)
  {
    prescaler = 0U;
  }
  period = 999U;

  htim3.Init.Prescaler = (uint16_t)prescaler;
  htim3.Init.Period = period;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  BlowerBrake_ApplyPermille(0U);
  s_prev_speed_rpm = 0;
  s_brake_latched = false;
}

void BlowerBrake_Update(void)
{
  uint16_t vbus_v = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
  int32_t curr_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  int32_t decel_rpm = (int32_t)s_prev_speed_rpm - curr_rpm;
  uint16_t brake_permille = 0U;

  g_blower_ipc.bus_voltage_v = vbus_v;

  if ((vbus_v >= BRAKE_VBUS_ON_V) &&
      ((decel_rpm >= BRAKE_DECEL_RPM_PER_MS) || s_brake_latched))
  {
    uint32_t over_v = (uint32_t)vbus_v - (uint32_t)BRAKE_VBUS_ON_V;

    s_brake_latched = true;
    brake_permille = (uint16_t)(over_v * 250U);
    if (brake_permille > BRAKE_MAX_PERMILLE)
    {
      brake_permille = BRAKE_MAX_PERMILLE;
    }
  }

  if (vbus_v <= BRAKE_VBUS_OFF_V)
  {
    s_brake_latched = false;
    brake_permille = 0U;
  }

  BlowerBrake_ApplyPermille(brake_permille);
  s_prev_speed_rpm = curr_rpm;
}

static void BlowerIpc_CM4_SeedFocTuning(void)
{
  g_blower_ipc.foc_speed_kp = PIDSpeedHandle_M1.hKpGain;
  g_blower_ipc.foc_speed_ki = PIDSpeedHandle_M1.hKiGain;
  g_blower_ipc.foc_torque_kp = PIDIqHandle_M1.hKpGain;
  g_blower_ipc.foc_torque_ki = PIDIqHandle_M1.hKiGain;
  g_blower_ipc.foc_flux_kp = PIDIdHandle_M1.hKpGain;
  g_blower_ipc.foc_flux_ki = PIDIdHandle_M1.hKiGain;
  __DMB();
  s_last_foc_tuning_seq = g_blower_ipc.cm7_foc_tuning_seq;
  g_blower_ipc.cm4_foc_tuning_ack_seq = s_last_foc_tuning_seq;
  __DMB();
}

static int16_t BlowerIpc_PickFocGain(int16_t mailbox, int16_t current)
{
  return (mailbox > 0) ? mailbox : current;
}

static void BlowerIpc_CM4_ApplyFocTuning(void)
{
  uint32_t seq = g_blower_ipc.cm7_foc_tuning_seq;
  if (seq == s_last_foc_tuning_seq)
  {
    return;
  }

  __DMB();
  PID_SetKP(&PIDSpeedHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_speed_kp, PIDSpeedHandle_M1.hKpGain));
  PID_SetKI(&PIDSpeedHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_speed_ki, PIDSpeedHandle_M1.hKiGain));
  PID_SetKP(&PIDIqHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_torque_kp, PIDIqHandle_M1.hKpGain));
  PID_SetKI(&PIDIqHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_torque_ki, PIDIqHandle_M1.hKiGain));
  PID_SetKP(&PIDIdHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_flux_kp, PIDIdHandle_M1.hKpGain));
  PID_SetKI(&PIDIdHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_flux_ki, PIDIdHandle_M1.hKiGain));
  __DMB();

  s_last_foc_tuning_seq = seq;
  g_blower_ipc.cm4_foc_tuning_ack_seq = seq;
  __DMB();
}

void BlowerIpc_CM4_Init(void)
{
  BlowerIpc_EnableSharedRamClock();
  __HAL_RCC_HSEM_CLK_ENABLE();

  BlowerIpc_InitSharedMemory();
  s_last_cmd_seq = g_blower_ipc.cm7_cmd_seq;
  BlowerIpc_CM4_SeedFocTuning();
  BlowerBrake_Init();
}

static void BlowerIpc_CM4_HandleCommand(BlowerIpcCommand_t cmd, int32_t rpm, uint16_t ramp_ms)
{
  switch (cmd)
  {
    case BLOWER_CMD_START:
      /* Clear latched FOC faults (e.g. over-voltage) before arming — CM7
       * may post ACK then START back-to-back and only the last IPC lands.
       * The start itself waits out any coast-down hold-off. */
      (void)MC_AcknowledgeFaultMotor1();
      BlowerIpc_CM4_RequestStart();
      BlowerIpc_CM4_ServicePendingStart();
      break;

    case BLOWER_CMD_STOP:
    {
      /* In FAULT_OVER acknowledge; otherwise stop, and look at the answer.
       *
       * MCI_StopMotor refuses while any fault is latched (PastFaults or
       * CurrentFaults) and in IDLE; MCI_FaultAcknowledged only acts in
       * FAULT_OVER. This used to stop first and acknowledge second with both
       * results discarded, so a STOP arriving in FAULT_OVER was refused and
       * only the ack that followed it made the NEXT stop possible.
       *
       * What that looked like on the bench (2026-09-24, 34000 rpm open loop,
       * port wide): the drive faulted, PWM went off and the rotor coasted -
       * but the angle/speed estimator is disabled in FAULT_NOW, FAULT_OVER
       * and IDLE (AngleSpeedEstimation[] in mc_tasks_foc.c), so the reported
       * speed FROZE at 34002 rpm and the blower looked like it was still
       * being driven while repeated STOPs "did nothing". Read mc_state and
       * occurred_faults on the B line, not mech_rpm, to know whether the
       * power stage is actually on.
       *
       * In FAULT_OVER, acknowledge ONLY. The ack clears PastFaults and queues
       * MCI_ACK_FAULTS; a MC_StopMotor1() after it then sees no faults and a
       * non-IDLE state, is accepted, and overwrites the queued command with
       * MCI_STOP - which FAULT_OVER ignores (mc_tasks_foc.c). Measured
       * 2026-09-24 after a DP_FAULT at 40 krpm: the drive stayed latched in
       * FAULT_OVER through every STOP. PWM is already off there, so the ack
       * alone is the whole job: FAULT_OVER -> IDLE on the next state-machine
       * tick. The ack is refused while the fault condition is still present
       * (CurrentFaults), so this cannot clear an active fault. */
      s_start_pending = false;    /* a start held off for coast-down is void */
      if (MC_GetSTMStateMotor1() == FAULT_OVER)
      {
        (void)MC_AcknowledgeFaultMotor1();
      }
      else if (MC_StopMotor1())
      {
        s_stop_in_flight = BlowerIpc_StateDrives(MC_GetSTMStateMotor1());
      }
      else
      {
        const MCI_State_t st = MC_GetSTMStateMotor1();
        /* Refused. In IDLE, STOP, WAIT_STOP_MOTOR, ICLWAIT or FAULT_* the
         * power stage is already off or on its way off: nothing to force.
         *
         * Refused while still POWERING the motor should be impossible - a
         * latched fault forces FAULT_OVER and switches PWM off - so this is
         * defence in depth, not the fix for the bench incident. If it ever
         * happens, do not leave the rotor driven with no report: raise the
         * SDK's own software-error stop (the same call TSK_HardwareFaultTask
         * makes). The safety task then switches PWM off, the state machine
         * goes to FAULT_NOW, MC_SW_ERROR shows in occurred_faults, and the
         * next START acknowledges it and runs normally. */
        if ((st == RUN) || (st == START) || (st == SWITCH_OVER) ||
            (st == ALIGNMENT) || (st == CHARGE_BOOT_CAP) ||
            (st == OFFSET_CALIB))
        {
          MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0U);
        }
      }
      BlowerBrake_ApplyPermille(0U);
      s_brake_latched = false;
      break;
    }

    case BLOWER_CMD_SET_SPEED:
    {
      /* CM7 often posts Start then SetSpeed in consecutive IPC writes; only
       * the last seq is observed. Ensure the motor is armed here so a lone
       * SET_SPEED (mask-off hold, resume ramp) cannot leave the rotor coasting.
       *
       * Do not call Start while FAULT_NOW still has an *active* fault (typical
       * regen OV during soft decel) — Ack only after the condition clears
       * (FAULT_OVER / IDLE). The ramp is still programmed so it runs on Start. */
      MCI_State_t state = MC_GetSTMStateMotor1();
      const uint16_t cur_faults = MC_GetCurrentFaultsMotor1();
      const int32_t mech_now =
          BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());

      /* Large soft decels pump the bus; latch brake early (OV at OV_VOLTAGE_THRESHOLD_V). */
      if ((mech_now > (rpm + 1500)) && (ramp_ms > 0U))
      {
        s_brake_latched = true;
        BlowerBrake_ApplyPermille(400U);
      }

      if ((state == FAULT_OVER) || ((state == FAULT_NOW) && (cur_faults == 0U)))
      {
        (void)MC_AcknowledgeFaultMotor1();
        state = MC_GetSTMStateMotor1();
      }

      if ((state == IDLE) || (state == FAULT_OVER) ||
          (state == STOP) || (state == WAIT_STOP_MOTOR))
      {
        /* Deferred while the rotor may still be coasting - see the
         * restart-while-coasting guard. This is the path that restarted
         * into a 32 krpm rotor after an OV trip on 2026-09-25. */
        (void)MC_AcknowledgeFaultMotor1();
        BlowerIpc_CM4_RequestStart();
        BlowerIpc_CM4_ServicePendingStart();
      }
      MC_ProgramSpeedRampMotor1(BlowerIpc_RpmToSpeedUnit(rpm), ramp_ms);
      break;
    }

    case BLOWER_CMD_ACK_FAULT:
      (void)MC_AcknowledgeFaultMotor1();
      break;

    default:
      break;
  }
}

void BlowerIpc_CM4_ProcessCommands(void)
{
  BlowerIpc_CM4_ApplyFocTuning();
  /* Every pass, not only on new commands: a held-off start must still be
   * issued when CM7 has gone quiet. */
  BlowerIpc_CM4_TrackCoast();
  BlowerIpc_CM4_ServicePendingStart();

  uint32_t seq;
  BlowerIpcCommand_t cmd;
  int32_t rpm;
  uint16_t ramp_ms;

  seq = g_blower_ipc.cm7_cmd_seq;
  if (seq == s_last_cmd_seq)
  {
    return;
  }

  __DMB();
  cmd = (BlowerIpcCommand_t)g_blower_ipc.command;
  rpm = g_blower_ipc.target_speed_rpm;
  ramp_ms = g_blower_ipc.ramp_duration_ms;
  __DMB();

  BlowerIpc_CM4_HandleCommand(cmd, rpm, ramp_ms);
  s_last_cmd_seq = seq;
  g_blower_ipc.cm4_cmd_ack_seq = seq;
  __DMB();
}

void BlowerIpc_CM4_UpdateStatus(void)
{
  MCI_State_t state = MC_GetSTMStateMotor1();

  g_blower_ipc.mc_state = (uint16_t)state;
  g_blower_ipc.current_faults = MC_GetCurrentFaultsMotor1();
  g_blower_ipc.occurred_faults = MC_GetOccurredFaultsMotor1();
  g_blower_ipc.mech_speed_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  g_blower_ipc.speed_ref_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedReferenceMotor1());
  g_blower_ipc.motor_running = ((state == RUN) || (state == START) || (state == SWITCH_OVER) ||
                                (state == STOP) || (state == WAIT_STOP_MOTOR)) ? 1U : 0U;
  {
    /* Coast-down hold-off still to run, ms; 1 while a start is pending but
     * the state machine has not reached IDLE yet. 0 = no start held off. */
    uint32_t hold = BlowerIpc_CM4_HoldoffRemainingMs();
    if (s_start_pending && (hold == 0U))
    {
      hold = 1U;
    }
    g_blower_ipc.restart_holdoff_ms = (uint16_t)((hold > 65535U) ? 65535U : hold);
  }

  /* Electrical diagnostics - see the note in blower_ipc.h. Converted here
   * because CURRENT_CONV_FACTOR_INV exists on this core only. If iq_ref sits
   * at NOMINAL_CURRENT (1345 mA) with iq tracking it and the speed still will
   * not rise, the drive is torque limited; if the bus sags under the same
   * load instead, it is the supply. Speed alone cannot separate the two. */
  {
    const qd_t iqd     = MC_GetIqdMotor1();
    const qd_t iqd_ref = MC_GetIqdrefMotor1();
    g_blower_ipc.iq_ma =
        (int16_t)((float)iqd.q * (float)CURRENT_CONV_FACTOR_INV * 1000.0f);
    g_blower_ipc.iq_ref_ma =
        (int16_t)((float)iqd_ref.q * (float)CURRENT_CONV_FACTOR_INV * 1000.0f);
    g_blower_ipc.id_ma =
        (int16_t)((float)iqd.d * (float)CURRENT_CONV_FACTOR_INV * 1000.0f);
  }
  /* Bus voltage at real resolution.
   *
   * NOT VBS_GetAvBusVoltage_V() - that returns WHOLE VOLTS, so scaling it by
   * 1000 yields exactly 20000 mV every sample and reports a rock-steady bus
   * whether or not one exists. The first capture with this line did exactly
   * that: 2137 samples, one distinct value, and an apparent 0.00 V sag that
   * was pure quantisation.
   *
   * _d is the averaged reading in u16Volt format, i.e. full scale = the
   * divider's range, ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR =
   * 52.8 V. That is ~0.8 mV per count, which will resolve a sag under a
   * one-second inspiratory load. */
  {
    const uint32_t vbus_d =
        (uint32_t)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super);
    const uint32_t full_scale_mv =
        (uint32_t)((ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR) * 1000.0f);
    g_blower_ipc.bus_voltage_mv =
        (uint16_t)((vbus_d * full_scale_mv) / 65535U);
  }

  /* Rotor-frame stator voltage - see the note in blower_ipc.h. s16V to volts
   * per mc_api.c: V = s16V * Vbus / (sqrt(3) * 32768), 0-to-peak phase.
   * This hook runs once per medium-frequency tick (1 kHz); a 1/16 IIR gives
   * a ~16 ms time constant, enough to drop the PI ripple without smearing a
   * breath. Reset while the drive is not in RUN so a stale value is never
   * reported against a stopped rotor. */
  {
    static float s_vq_mv = 0.0f;
    static float s_vd_mv = 0.0f;
    if (state == RUN)
    {
      const qd_t  vqd  = MC_GetVqdMotor1();
      const float k_mv = (float)g_blower_ipc.bus_voltage_mv / (1.7320508f * 32768.0f);
      s_vq_mv += ((float)vqd.q * k_mv - s_vq_mv) * (1.0f / 16.0f);
      s_vd_mv += ((float)vqd.d * k_mv - s_vd_mv) * (1.0f / 16.0f);
    }
    else
    {
      s_vq_mv = 0.0f;
      s_vd_mv = 0.0f;
    }
    g_blower_ipc.vq_mv = (int16_t)s_vq_mv;
    g_blower_ipc.vd_mv = (int16_t)s_vd_mv;
  }

  __DMB();
  g_blower_ipc.cm4_status_seq++;
  __DMB();
  (void)HAL_HSEM_Release(BLOWER_IPC_HSEM_STATUS_ID, 0U);
}
