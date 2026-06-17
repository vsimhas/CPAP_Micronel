/**
  ******************************************************************************
  * @file    telemetry_stream.c
  * @brief   Live pressure / patient-flow CSV stream over USB CDC VCP.
  ******************************************************************************
  */

#include "telemetry_stream.h"

#if TELEM_ENABLED

#include <stdio.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"

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
#define TELEM_FORMAT_BUF 48U
#endif

#if (TELEM_BUF_SIZE & (TELEM_BUF_SIZE - 1U)) != 0U
#error "TELEM_BUF_SIZE must be a power of two"
#endif

#define TELEM_BUF_MASK   (TELEM_BUF_SIZE - 1U)

volatile uint8_t  g_telem_enabled      = 1U;
volatile uint32_t g_telem_sample_count = 0U;
volatile uint32_t g_telem_dropped_count = 0U;

static uint8_t            s_buf[TELEM_BUF_SIZE];
static volatile uint32_t  s_head;
static volatile uint32_t  s_tail;
static uint8_t            s_tx_chunk[TELEM_TX_CHUNK];
static volatile uint8_t   s_inited = 0U;
static char               s_rx_line[32];
static uint8_t            s_rx_len = 0U;

static void Telem_BufPush(const uint8_t *data, uint32_t len);
static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen);

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
  Telem_PushRaw("# telem ready (T,ms,P,Q | R,ms,rpm_cmd,rpm_act,P)\r\n");
}

void Telem_PushSample(float pressure_cmh2o, float patient_flow_slm)
{
  if ((s_inited == 0U) || (g_telem_enabled == 0U))
  {
    return;
  }

  char line[TELEM_FORMAT_BUF];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "T,%lu,%.2f,%.2f\r\n",
                   (unsigned long)now,
                   (double)pressure_cmh2o,
                   (double)patient_flow_slm);
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
