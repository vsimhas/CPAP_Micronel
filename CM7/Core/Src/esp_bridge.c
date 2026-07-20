/**
  ******************************************************************************
  * @file    esp_bridge.c
  * @brief   USART2 bridge: CM7 therapy telemetry → ESP32 for MQTT publish.
  ******************************************************************************
  */

#include "esp_bridge.h"

#if ESP_BRIDGE_ENABLED

#include <stdio.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"

extern UART_HandleTypeDef huart2;

#ifndef ESP_BRIDGE_BUF_SIZE
#define ESP_BRIDGE_BUF_SIZE  4096U
#endif

#ifndef ESP_BRIDGE_TX_CHUNK
#define ESP_BRIDGE_TX_CHUNK  128U
#endif

#if (ESP_BRIDGE_BUF_SIZE & (ESP_BRIDGE_BUF_SIZE - 1U)) != 0U
#error "ESP_BRIDGE_BUF_SIZE must be a power of two"
#endif

#define ESP_BRIDGE_BUF_MASK  (ESP_BRIDGE_BUF_SIZE - 1U)

volatile uint8_t  g_esp_bridge_enabled    = 1U;
volatile uint32_t g_esp_bridge_tx_count   = 0U;
volatile uint32_t g_esp_bridge_drop_count = 0U;

static uint8_t           s_buf[ESP_BRIDGE_BUF_SIZE];
static volatile uint32_t s_head = 0U;
static volatile uint32_t s_tail = 0U;
static uint8_t           s_tx_chunk[ESP_BRIDGE_TX_CHUNK];
static volatile uint8_t  s_inited = 0U;

static void EspBridge_BufPush(const uint8_t *data, uint32_t len)
{
  uint32_t primask;
  uint32_t head;
  uint32_t tail;
  uint32_t free_space;
  uint32_t i;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  head = s_head;
  tail = s_tail;
  free_space = (tail - head - 1U) & ESP_BRIDGE_BUF_MASK;
  if (len > free_space)
  {
    g_esp_bridge_drop_count++;
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  for (i = 0U; i < len; i++)
  {
    s_buf[head] = data[i];
    head = (head + 1U) & ESP_BRIDGE_BUF_MASK;
  }
  s_head = head;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint32_t EspBridge_BufPopChunk(uint8_t *dst, uint32_t max_len)
{
  uint32_t primask;
  uint32_t head;
  uint32_t tail;
  uint32_t avail;
  uint32_t n;
  uint32_t i;

  if ((dst == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  head = s_head;
  tail = s_tail;
  avail = (head - tail) & ESP_BRIDGE_BUF_MASK;
  n = (avail < max_len) ? avail : max_len;
  for (i = 0U; i < n; i++)
  {
    dst[i] = s_buf[tail];
    tail = (tail + 1U) & ESP_BRIDGE_BUF_MASK;
  }
  s_tail = tail;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return n;
}

void EspBridge_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  g_esp_bridge_tx_count = 0U;
  g_esp_bridge_drop_count = 0U;
  g_esp_bridge_enabled = 1U;
  s_inited = 1U;
}

void EspBridge_PushSample(float p_sensor_cmh2o,
                          float p_mask_cmh2o,
                          float q_patient_slm,
                          float q_total_slm,
                          float setpoint_cmh2o,
                          int32_t pid_rpm,
                          int32_t mech_rpm,
                          float leak_reported_slm,
                          float vent_slm,
                          uint8_t therapy_state,
                          uint8_t therapy_mode,
                          uint8_t mask_off)
{
  char line[160];
  int n;

  if ((s_inited == 0U) || (g_esp_bridge_enabled == 0U))
  {
    return;
  }

  n = snprintf(line, sizeof(line),
               "M,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%ld,%ld,%.2f,%.2f,%u,%u,%u\r\n",
               (unsigned long)HAL_GetTick(),
               (double)p_sensor_cmh2o,
               (double)p_mask_cmh2o,
               (double)q_patient_slm,
               (double)q_total_slm,
               (double)setpoint_cmh2o,
               (long)pid_rpm,
               (long)mech_rpm,
               (double)leak_reported_slm,
               (double)vent_slm,
               (unsigned)therapy_state,
               (unsigned)therapy_mode,
               (unsigned)mask_off);

  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    EspBridge_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

void EspBridge_Pump(void)
{
  uint32_t n;

  if ((s_inited == 0U) || (g_esp_bridge_enabled == 0U))
  {
    return;
  }

  /* Drain up to a few chunks per SystemTask tick. Blocking TX is fine here
   * (SystemTask is low-rate); SensorTask never waits on USART. */
  for (uint8_t i = 0U; i < 4U; i++)
  {
    n = EspBridge_BufPopChunk(s_tx_chunk, ESP_BRIDGE_TX_CHUNK);
    if (n == 0U)
    {
      break;
    }

    if (HAL_UART_Transmit(&huart2, s_tx_chunk, (uint16_t)n, 20U) == HAL_OK)
    {
      g_esp_bridge_tx_count++;
    }
    else
    {
      g_esp_bridge_drop_count++;
      break;
    }
  }
}

#endif /* ESP_BRIDGE_ENABLED */
