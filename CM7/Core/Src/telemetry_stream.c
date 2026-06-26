/**
  ******************************************************************************
  * @file    telemetry_stream.c
  * @brief   Live pressure / patient-flow CSV stream over USB CDC VCP.
  ******************************************************************************
  */

#include "telemetry_stream.h"

#if TELEM_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"
#include "therapy.h"
#include "blower_ipc.h"

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#ifndef TELEM_BUF_SIZE
#define TELEM_BUF_SIZE   2048U
#endif

#ifndef TELEM_TX_CHUNK
#define TELEM_TX_CHUNK   64U
#endif

#ifndef TELEM_FORMAT_BUF
#define TELEM_FORMAT_BUF 64U
#endif

#if (TELEM_BUF_SIZE & (TELEM_BUF_SIZE - 1U)) != 0U
#error "TELEM_BUF_SIZE must be a power of two"
#endif

#define TELEM_BUF_MASK   (TELEM_BUF_SIZE - 1U)
#define TELEM_CMD_QUEUE_LEN  48U

volatile uint8_t  g_telem_enabled      = 1U;
volatile uint32_t g_telem_sample_count = 0U;
volatile uint32_t g_telem_dropped_count = 0U;

extern volatile float   g_blower_ctrl_kp_low;
extern volatile float   g_blower_ctrl_ki_low;
extern volatile float   g_blower_ctrl_kd_low;
extern volatile float   g_blower_ctrl_kp_mid;
extern volatile float   g_blower_ctrl_ki_mid;
extern volatile float   g_blower_ctrl_kd_mid;
extern volatile float   g_blower_ctrl_kp_high;
extern volatile float   g_blower_ctrl_ki_high;
extern volatile float   g_blower_ctrl_kd_high;
extern volatile uint8_t g_blower_ctrl_ff_enable;
extern volatile float   g_blower_ctrl_ff_k_rpm_per_sqrtcmh;
extern volatile uint8_t g_blower_ctrl_flow_ff_enable;
extern volatile float   g_blower_ctrl_flow_ff_k_rpm_per_slm;
extern volatile float   g_blower_ctrl_flow_ff_max_rpm;
extern volatile float   g_blower_ctrl_flow_ff_deadband_slm;
extern volatile float   g_blower_ctrl_flow_ff_rise_rpm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_over_p_gain;
extern volatile float   g_blower_ctrl_over_brake_rpm_per_s;
extern volatile float   g_blower_ctrl_slew_cmh2o_per_s;
extern volatile float   g_blower_ctrl_slew_up_cmh2o_per_s;
extern volatile float   g_blower_ctrl_out_filter_alpha;
extern volatile float   g_blower_ctrl_integ_deadband_cmh2o;
extern volatile float   g_blower_ctrl_integ_max_abs_rpm;
extern volatile float   g_blower_ctrl_meas_filter_alpha;
extern volatile float   g_blower_ctrl_deriv_filter_alpha;
extern volatile int32_t g_blower_ctrl_rpm_max;

static uint8_t            s_buf[TELEM_BUF_SIZE];
static volatile uint32_t  s_head;
static volatile uint32_t  s_tail;
static uint8_t            s_tx_chunk[TELEM_TX_CHUNK];
static volatile uint8_t   s_inited = 0U;
static char               s_rx_line[96];
static uint8_t            s_rx_len = 0U;
static char               s_cmd_queue[TELEM_CMD_QUEUE_LEN][96];
static volatile uint8_t   s_cmd_head = 0U;
static volatile uint8_t   s_cmd_tail = 0U;

static void Telem_BufPush(const uint8_t *data, uint32_t len);
static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen);
static void Telem_QueueCommand(const char *cmd);

static void Telem_PushRaw(const char *text)
{
  Telem_BufPush((const uint8_t *)text, (uint32_t)strlen(text));
}

static void Telem_HandleCommand(const char *cmd)
{
  if (cmd == NULL) { return; }

  if ((strcmp(cmd, "telem on") == 0) || (strcmp(cmd, "telem 1") == 0))
  {
    g_telem_enabled = 1U;
    Telem_PushRaw("# telem on\r\n");
  }
  else if ((strcmp(cmd, "telem off") == 0) || (strcmp(cmd, "telem 0") == 0))
  {
    g_telem_enabled = 0U;
    Telem_PushRaw("# telem off\r\n");
  }
  else
  {
    Telem_QueueCommand(cmd);
  }
}

static void Telem_QueueCommand(const char *cmd)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint8_t next = (uint8_t)((s_cmd_head + 1U) % TELEM_CMD_QUEUE_LEN);
  if (next == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    Telem_PushRaw("# err cmd queue full\r\n");
    return;
  }

  (void)strncpy(s_cmd_queue[s_cmd_head], cmd, sizeof(s_cmd_queue[0]) - 1U);
  s_cmd_queue[s_cmd_head][sizeof(s_cmd_queue[0]) - 1U] = '\0';
  s_cmd_head = next;

  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Telem_OnRx(const uint8_t *data, uint32_t len)
{
  if ((data == NULL) || (len == 0U)) { return; }

  for (uint32_t i = 0U; i < len; i++)
  {
    char c = (char)data[i];
    if ((c == '\r') || (c == '\n'))
    {
      if (s_rx_len > 0U)
      {
        s_rx_line[s_rx_len] = '\0';
        Telem_HandleCommand(s_rx_line);
        s_rx_len = 0U;
      }
    }
    else if (s_rx_len < (sizeof(s_rx_line) - 1U))
    {
      s_rx_line[s_rx_len++] = c;
    }
    else
    {
      s_rx_len = 0U;
    }
  }
}

static void Telem_PushAck(const char *cmd)
{
  char line[112];
  int n = snprintf(line, sizeof(line), "# ack %s\r\n", cmd);
  if (n > 0)
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static void Telem_PushErr(const char *reason)
{
  char line[80];
  int n = snprintf(line, sizeof(line), "# err %s\r\n", reason);
  if (n > 0)
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static void Telem_PushStatus(void)
{
  char line[128];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line),
                   "S,%lu,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%u\r\n",
                   (unsigned long)now,
                   (unsigned)Therapy_GetState(),
                   (double)Therapy_GetTargetPressure(),
                   (double)Therapy_GetSetpointCmh2o(),
                   (double)Therapy_GetMeasuredCmh2o(),
                   (double)Therapy_GetLeakSlm(),
                   Therapy_IsEprEnabled() ? 1U : 0U,
                   Therapy_IsMaskOffActive() ? 1U : 0U,
                   (unsigned)Therapy_GetFault());
  if (n > 0)
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static int Telem_ParseFloat(const char *text, float *out)
{
  char *end = NULL;
  float v = strtof(text, &end);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  *out = v;
  return 1;
}

static int Telem_ParseLong(const char *text, long *out)
{
  char *end = NULL;
  long v = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  *out = v;
  return 1;
}

static int Telem_SetFloatParam(const char *name, float value)
{
  if (strcmp(name, "kp_low") == 0) { g_blower_ctrl_kp_low = value; return 1; }
  if (strcmp(name, "ki_low") == 0) { g_blower_ctrl_ki_low = value; return 1; }
  if (strcmp(name, "kd_low") == 0) { g_blower_ctrl_kd_low = value; return 1; }
  if (strcmp(name, "kp_mid") == 0) { g_blower_ctrl_kp_mid = value; return 1; }
  if (strcmp(name, "ki_mid") == 0) { g_blower_ctrl_ki_mid = value; return 1; }
  if (strcmp(name, "kd_mid") == 0) { g_blower_ctrl_kd_mid = value; return 1; }
  if (strcmp(name, "kp_high") == 0) { g_blower_ctrl_kp_high = value; return 1; }
  if (strcmp(name, "ki_high") == 0) { g_blower_ctrl_ki_high = value; return 1; }
  if (strcmp(name, "kd_high") == 0) { g_blower_ctrl_kd_high = value; return 1; }

  if (strcmp(name, "ff_k") == 0) { g_blower_ctrl_ff_k_rpm_per_sqrtcmh = value; return 1; }
  if (strcmp(name, "flow_ff_k") == 0) { g_blower_ctrl_flow_ff_k_rpm_per_slm = value; return 1; }
  if (strcmp(name, "flow_ff_max") == 0) { g_blower_ctrl_flow_ff_max_rpm = value; return 1; }
  if (strcmp(name, "flow_ff_deadband") == 0) { g_blower_ctrl_flow_ff_deadband_slm = value; return 1; }
  if (strcmp(name, "flow_ff_rise") == 0) { g_blower_ctrl_flow_ff_rise_rpm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_fall") == 0) { g_blower_ctrl_flow_ff_fall_rpm_per_s = value; return 1; }
  if (strcmp(name, "over_p_gain") == 0) { g_blower_ctrl_over_p_gain = value; return 1; }
  if (strcmp(name, "over_brake") == 0) { g_blower_ctrl_over_brake_rpm_per_s = value; return 1; }
  if (strcmp(name, "slew_down") == 0) { g_blower_ctrl_slew_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "slew_up") == 0) { g_blower_ctrl_slew_up_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "out_alpha") == 0) { g_blower_ctrl_out_filter_alpha = value; return 1; }
  if (strcmp(name, "integ_deadband") == 0) { g_blower_ctrl_integ_deadband_cmh2o = value; return 1; }
  if (strcmp(name, "integ_max") == 0) { g_blower_ctrl_integ_max_abs_rpm = value; return 1; }
  if (strcmp(name, "meas_alpha") == 0) { g_blower_ctrl_meas_filter_alpha = value; return 1; }
  if (strcmp(name, "deriv_alpha") == 0) { g_blower_ctrl_deriv_filter_alpha = value; return 1; }
  return 0;
}

static int Telem_SetIntParam(const char *name, long value)
{
  if (strcmp(name, "ff_enable") == 0)
  {
    g_blower_ctrl_ff_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "flow_ff_enable") == 0)
  {
    g_blower_ctrl_flow_ff_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "rpm_max") == 0)
  {
    if (value < 0L) { value = 0L; }
    if (value > 45000L) { value = 45000L; }
    g_blower_ctrl_rpm_max = (int32_t)value;
    return 1;
  }
  return 0;
}

static int Telem_SetFocParam(const char *name, long value)
{
  BlowerFocGainId_t id;
  int16_t gain;

  if (value < 0L) { value = 0L; }
  if (value > 20000L) { value = 20000L; }
  gain = (int16_t)value;

  if (strcmp(name, "speed_kp") == 0) { id = FOC_GAIN_SPEED_KP; }
  else if (strcmp(name, "speed_ki") == 0) { id = FOC_GAIN_SPEED_KI; }
  else if (strcmp(name, "torque_kp") == 0) { id = FOC_GAIN_TORQUE_KP; }
  else if (strcmp(name, "torque_ki") == 0) { id = FOC_GAIN_TORQUE_KI; }
  else if (strcmp(name, "flux_kp") == 0) { id = FOC_GAIN_FLUX_KP; }
  else if (strcmp(name, "flux_ki") == 0) { id = FOC_GAIN_FLUX_KI; }
  else { return 0; }

  return BlowerIpc_CM7_SetFocGain(id, gain) ? 1 : 0;
}

void Telem_ProcessCommands(void)
{
  char cmd[sizeof(s_cmd_queue[0])];

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cmd_head == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }
  (void)strncpy(cmd, s_cmd_queue[s_cmd_tail], sizeof(cmd) - 1U);
  cmd[sizeof(cmd) - 1U] = '\0';
  s_cmd_tail = (uint8_t)((s_cmd_tail + 1U) % TELEM_CMD_QUEUE_LEN);
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (strcmp(cmd, "therapy start") == 0)
  {
    Therapy_Start();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy stop") == 0)
  {
    Therapy_Stop();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy status") == 0)
  {
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure ", 21U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[21], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure range 4..25");
      return;
    }
    Therapy_SetTargetPressure(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set ramp ", 17U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[17], &v) || (v < 0L) || (v > 45L))
    {
      Telem_PushErr("ramp range 0..45");
      return;
    }
    Therapy_SetRampMinutes((uint8_t)v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr ", 16U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[16], &v) || ((v != 0L) && (v != 1L)))
    {
      Telem_PushErr("epr must be 0 or 1");
      return;
    }
    Therapy_SetEprEnabled(v != 0L);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_relief ", 23U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[23], &v) || (v < 1.0f) || (v > 3.0f))
    {
      Telem_PushErr("epr_relief range 1..3");
      return;
    }
    Therapy_SetEprReliefCmh2o(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set maskoff ", 20U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[20], &v) || ((v != 0L) && (v != 1L)))
    {
      Telem_PushErr("maskoff must be 0 or 1");
      return;
    }
    Therapy_SetMaskOffEnabled(v != 0L);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "blower set ", 11U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[11], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("blower set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetIntParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetFloatParam(name, fv))
    {
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown blower param");
    return;
  }

  if (strncmp(cmd, "foc set ", 8U) == 0)
  {
    char name[32];
    char value_text[32];
    long iv;
    int matched = sscanf(&cmd[8], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("foc set needs name value");
      return;
    }
    if (!Telem_ParseLong(value_text, &iv))
    {
      Telem_PushErr("foc value must be integer");
      return;
    }
    if (Telem_SetFocParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    Telem_PushErr("unknown foc param");
    return;
  }

  Telem_PushErr("unknown command");
}

static void Telem_BufPush(const uint8_t *data, uint32_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t next = (s_head + 1U) & TELEM_BUF_MASK;
    if (next == s_tail)
    {
      s_tail = (s_tail + 1U) & TELEM_BUF_MASK;
      g_telem_dropped_count++;
    }
    s_buf[s_head] = data[i];
    s_head = next;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t head = s_head;
  uint32_t tail = s_tail;
  uint32_t avail;

  if (head >= tail)
  {
    avail = head - tail;
  }
  else
  {
    avail = TELEM_BUF_SIZE - tail;
  }

  if (avail > maxLen)
  {
    avail = maxLen;
  }

  for (uint32_t i = 0U; i < avail; i++)
  {
    dst[i] = s_buf[(tail + i) & TELEM_BUF_MASK];
  }
  s_tail = (tail + avail) & TELEM_BUF_MASK;

  if (primask == 0U)
  {
    __enable_irq();
  }

  return avail;
}

void Telem_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  s_rx_len = 0U;
  g_telem_sample_count = 0U;
  g_telem_dropped_count = 0U;
  s_inited = 1U;
  Telem_PushRaw("# telem ready (T,ms,P,Q,pid_rpm,mech_rpm | R,ms,rpm_cmd,rpm_act,P)\r\n");
}

void Telem_PushSample(float pressure_cmh2o, float patient_flow_slm,
                      int32_t pid_rpm_cmd, int32_t mech_rpm)
{
  if ((s_inited == 0U) || (g_telem_enabled == 0U))
  {
    return;
  }

  char line[TELEM_FORMAT_BUF];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "T,%lu,%.2f,%.2f,%ld,%ld\r\n",
                   (unsigned long)now,
                   (double)pressure_cmh2o,
                   (double)patient_flow_slm,
                   (long)pid_rpm_cmd,
                   (long)mech_rpm);
  if (n <= 0)
  {
    return;
  }

  Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  g_telem_sample_count++;
}

void Telem_PushBenchRpm(int32_t rpm_cmd, int32_t rpm_act, float pressure_cmh2o)
{
  if (s_inited == 0U)
  {
    return;
  }

  char line[56];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "R,%lu,%ld,%ld,%.2f\r\n",
                   (unsigned long)now,
                   (long)rpm_cmd,
                   (long)rpm_act,
                   (double)pressure_cmh2o);
  if (n <= 0)
  {
    return;
  }

  Telem_BufPush((const uint8_t *)line, (uint32_t)n);
}

void Telem_Pump(void)
{
  if (s_inited == 0U) { return; }
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }

  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) { return; }
  if (hcdc->TxState != 0U) { return; }

  uint32_t n = Telem_BufPopChunk(s_tx_chunk, sizeof(s_tx_chunk));
  if (n == 0U) { return; }

  (void)CDC_Transmit_FS(s_tx_chunk, (uint16_t)n);
}

#endif /* TELEM_ENABLED */
