/**
  ******************************************************************************
  * @file    esp_bridge.h
  * @brief   CM7 → ESP32 therapy telemetry over USART2 (MQTT uplink).
  *
 *  Wire format (one CSV line every 10 ms / 100 Hz when enabled):
 *    M,<ms>,<P_sensor>,<P_mask>,<Q_patient>,<Q_total>,<setpoint>,
 *      <pid_rpm>,<mech_rpm>,<leak_rep>,<vent>,<state>,<mode>,<mask_off>\r\n
 *
 *  ESP32 converts each CSV frame to MQTT JSON (schema v1). Keep this
 *  UART format compact; do not emit JSON on the CM7 serial link.
 *
 *  Frames are queued from SensorTask and drained by EspBridge_Pump()
 *  from SystemTask so USART TX never blocks the 200 Hz control loop.
  ******************************************************************************
  */

#ifndef ESP_BRIDGE_H
#define ESP_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ESP_BRIDGE_ENABLED
#define ESP_BRIDGE_ENABLED  1
#endif

#if ESP_BRIDGE_ENABLED

extern volatile uint8_t  g_esp_bridge_enabled;
extern volatile uint32_t g_esp_bridge_tx_count;
extern volatile uint32_t g_esp_bridge_drop_count;

void EspBridge_Init(void);
void EspBridge_Pump(void);

/* Queue one therapy sample (call at 100 Hz from SensorTask). */
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
                          uint8_t mask_off);

#else

#define EspBridge_Init() ((void)0)
#define EspBridge_Pump() ((void)0)
#define EspBridge_PushSample(...) ((void)0)

#endif /* ESP_BRIDGE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* ESP_BRIDGE_H */
