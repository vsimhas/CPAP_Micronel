/**
  * @file    blower_ipc_cm7.c
  * @brief   CM7-side blower IPC command interface.
  */

#include "blower_ipc.h"
#include "stm32h7xx_hal.h"
#include "dbg_log.h"

static bool BlowerIpc_CM7_PostCommand(BlowerIpcCommand_t cmd, int32_t rpm, uint16_t ramp_ms)
{
  if (g_blower_ipc.magic != BLOWER_IPC_MAGIC)
  {
    BlowerIpc_InitSharedMemory();
  }

  g_blower_ipc.command = (uint32_t)cmd;
  g_blower_ipc.target_speed_rpm = rpm;
  g_blower_ipc.ramp_duration_ms = ramp_ms;
  __DMB();
  g_blower_ipc.cm7_cmd_seq++;
  __DMB();

  (void)HAL_HSEM_Release(BLOWER_IPC_HSEM_CMD_ID, 0U);
  return true;
}

void BlowerIpc_CM7_Init(void)
{
  BlowerIpc_EnableSharedRamClock();
  __HAL_RCC_HSEM_CLK_ENABLE();
  BlowerIpc_InitSharedMemory();
}

bool BlowerIpc_CM7_Start(void)
{
  DBG_I("BLR", "cmd START");
  return BlowerIpc_CM7_PostCommand(BLOWER_CMD_START, 0, 0U);
}

bool BlowerIpc_CM7_Stop(void)
{
  DBG_I("BLR", "cmd STOP");
  return BlowerIpc_CM7_PostCommand(BLOWER_CMD_STOP, 0, 0U);
}

bool BlowerIpc_CM7_SetSpeedRpm(int32_t rpm, uint16_t ramp_duration_ms)
{
  /* The MCWB-generated firmware on CM4 clamps internally too, but
   * clamping here keeps the IPC mailbox contents truthful and makes
   * watch-window debugging match what the motor actually targets. */
  int32_t clamped = rpm;
  if (clamped >  (int32_t)BLOWER_IPC_MAX_SPEED_RPM) { clamped =  (int32_t)BLOWER_IPC_MAX_SPEED_RPM; }
  if (clamped < -(int32_t)BLOWER_IPC_MAX_SPEED_RPM) { clamped = -(int32_t)BLOWER_IPC_MAX_SPEED_RPM; }

  if (clamped != rpm)
  {
    DBG_W("BLR", "speed %ld clamped to %ld (max=%d)",
          (long)rpm, (long)clamped, BLOWER_IPC_MAX_SPEED_RPM);
  }
  else
  {
    DBG_I("BLR", "set speed %ld rpm, ramp=%u ms",
          (long)clamped, (unsigned)ramp_duration_ms);
  }

  return BlowerIpc_CM7_PostCommand(BLOWER_CMD_SET_SPEED, clamped, ramp_duration_ms);
}

bool BlowerIpc_CM7_AcknowledgeFault(void)
{
  DBG_I("BLR", "ack fault");
  return BlowerIpc_CM7_PostCommand(BLOWER_CMD_ACK_FAULT, 0, 0U);
}

bool BlowerIpc_CM7_GetStatus(BlowerIpcStatus_t *status)
{
  if (status == NULL)
  {
    return false;
  }

  __DMB();
  status->mc_state = g_blower_ipc.mc_state;
  status->current_faults = g_blower_ipc.current_faults;
  status->occurred_faults = g_blower_ipc.occurred_faults;
  status->mech_speed_rpm = g_blower_ipc.mech_speed_rpm;
  status->speed_ref_rpm = g_blower_ipc.speed_ref_rpm;
  status->bus_voltage_v = g_blower_ipc.bus_voltage_v;
  status->brake_pwm_permille = g_blower_ipc.brake_pwm_permille;
  status->brake_active = (g_blower_ipc.brake_active != 0U);
  status->motor_running = (g_blower_ipc.motor_running != 0U);
  status->status_seq = g_blower_ipc.cm4_status_seq;
  status->cmd_ack_seq = g_blower_ipc.cm4_cmd_ack_seq;
  return true;
}
