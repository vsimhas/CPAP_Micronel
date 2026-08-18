/**
  ******************************************************************************
  * @file    leak_btps.c
  * @brief   Ambient-driven altitude factor + optional T/RH BTPS factors.
  ******************************************************************************
  */

#include "leak_btps.h"

#include <math.h>

static float s_temp_factor = 1.0f;
static float s_humid_factor = 1.0f;
static float s_alt_factor = 1.0f;
static float s_amb_mbar = LEAK_BTPS_P_REF_MBAR;
static uint8_t s_alt_enable = 1U;

static float clampUnity(float f)
{
    if (f < 0.25f) return 0.25f;
    if (f > 4.0f)  return 4.0f;
    return f;
}

static float clampAmb(float mbar)
{
    if (mbar < 600.0f) return 600.0f;
    if (mbar > 1100.0f) return 1100.0f;
    return mbar;
}

static float computeAltitudeFactor(float mbar)
{
    /* Orifice / vent mass-flow scale vs sea-level calibration density. */
    const float p = clampAmb(mbar);
    float f = sqrtf(p / LEAK_BTPS_P_REF_MBAR);
    /* ~0..3500 m envelope with margin. */
    if (f < 0.70f) f = 0.70f;
    if (f > 1.15f) f = 1.15f;
    return f;
}

static int32_t scaleFlow(int32_t flow_mlps, float factor)
{
    const float out = (float)flow_mlps * factor;
    if (out <= 0.0f) return 0;
    return (int32_t)(out + 0.5f);
}

void LeakBtps_Init(void)
{
    s_temp_factor  = 1.0f;
    s_humid_factor = 1.0f;
    s_amb_mbar    = LEAK_BTPS_P_REF_MBAR;
    s_alt_enable  = 1U;
    s_alt_factor   = 1.0f;
}

void LeakBtps_SetAmbientMbar(float mbar)
{
    s_amb_mbar = clampAmb(mbar);
    if (s_alt_enable != 0U)
    {
        s_alt_factor = computeAltitudeFactor(s_amb_mbar);
    }
    else
    {
        s_alt_factor = 1.0f;
    }
}

float LeakBtps_GetAmbientMbar(void)
{
    return s_amb_mbar;
}

void LeakBtps_SetAltitudeEnable(bool en)
{
    s_alt_enable = en ? 1U : 0U;
    if (s_alt_enable != 0U)
    {
        s_alt_factor = computeAltitudeFactor(s_amb_mbar);
    }
    else
    {
        s_alt_factor = 1.0f;
    }
}

bool LeakBtps_IsAltitudeEnable(void)
{
    return (s_alt_enable != 0U);
}

int32_t LeakBtps_TableToBtpsMlps(int32_t flow_mlps_stpd)
{
    const float f = s_temp_factor * s_humid_factor * s_alt_factor;
    return scaleFlow(flow_mlps_stpd, f);
}

int32_t LeakBtps_MeasuredStpdToBtpsMlps(int32_t flow_mlps_stpd)
{
    /* Measured orifice flow is already density-corrected in flow_sensor. */
    const float f = s_temp_factor * s_humid_factor;
    return scaleFlow(flow_mlps_stpd, f);
}

void LeakBtps_SetTemperatureFactor(float factor) { s_temp_factor  = clampUnity(factor); }
void LeakBtps_SetHumidityFactor(float factor)    { s_humid_factor = clampUnity(factor); }
void LeakBtps_SetAltitudeFactor(float factor)
{
    s_alt_factor = clampUnity(factor);
}

float LeakBtps_GetTemperatureFactor(void) { return s_temp_factor; }
float LeakBtps_GetHumidityFactor(void)    { return s_humid_factor; }
float LeakBtps_GetAltitudeFactor(void)    { return s_alt_factor; }
