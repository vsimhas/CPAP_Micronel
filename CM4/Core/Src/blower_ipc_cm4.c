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
/* FOC OV trips at 24.9 V (drive_parameters.h). Engage brake early so a
 * mask-off / therapy soft decel does not kill PWM and coast to 0. */
#define BRAKE_VBUS_ON_V             22U
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
      HAL_GPIO_WritePin(STSPIN_STBY_GPIO_Port, STSPIN_STBY_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOT_POW_EN_GPIO_Port, MOT_POW_EN_Pin, GPIO_PIN_SET);
      /* Clear latched FOC faults (e.g. over-voltage) before arming — CM7
       * may post ACK then START back-to-back and only the last IPC lands. */
      (void)MC_AcknowledgeFaultMotor1();
      (void)MC_StartMotor1();
      break;

    case BLOWER_CMD_STOP:
      (void)MC_StopMotor1();
      (void)MC_AcknowledgeFaultMotor1();
      BlowerBrake_ApplyPermille(0U);
      s_brake_latched = false;
      break;

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

      /* Large soft decels pump the bus; latch brake early (OV is only 24.9 V). */
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
        HAL_GPIO_WritePin(STSPIN_STBY_GPIO_Port, STSPIN_STBY_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOT_POW_EN_GPIO_Port, MOT_POW_EN_Pin, GPIO_PIN_SET);
        (void)MC_AcknowledgeFaultMotor1();
        (void)MC_StartMotor1();
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

  __DMB();
  g_blower_ipc.cm4_status_seq++;
  __DMB();
  (void)HAL_HSEM_Release(BLOWER_IPC_HSEM_STATUS_ID, 0U);
}
