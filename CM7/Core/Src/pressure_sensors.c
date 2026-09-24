/**
  ******************************************************************************
  * @file    pressure_sensors.c
  * @brief   AMS5935 SPI pressure sensors and analog backup pressure ADC.
  ******************************************************************************
  */

#include "pressure_sensors.h"

#define AMS5935_CMD_SINGLE_MEAS       0xAAU
#define AMS5935_CMD_OVERSAMPLE_MEAS   0xADU
#define AMS5935_CMD_READ              0xF0U

#define AMS5935_SPI_TIMEOUT_MS        100U
#define AMS5935_SPI_CS_HOLD_NOP_COUNT 200U

#define AMS5935_DIGOUT_MIN            (0.1f * 16777216.0f)
#define AMS5935_DIGOUT_MAX            (0.9f * 16777216.0f)
#define AMS5935_DIGOUT_MIN_U32        1677722UL
#define AMS5935_DIGOUT_MAX_U32        15099494UL
/* Two back-to-back F0 reads of a finished conversion must match; a torn
 * 24-bit update disagrees by thousands of counts. ~1024 counts ≈ 0.0008 mbar. */
#define AMS5935_TEAR_MAX_COUNTS       1024UL

#define PRESSURE_ANALOG_ADC_FULL_SCALE  65535.0f
#define PRESSURE_ANALOG_REF_VOLTAGE_V   3.3f
#define PRESSURE_ANALOG_DIVIDER_GAIN    3.0f
#define PRESSURE_ANALOG_DIVIDER_ATTEN   2.0f
#define PRESSURE_ANALOG_V_MIN_V         0.5f
#define PRESSURE_ANALOG_V_SPAN_V        4.0f

static void ams5935_cs_low(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void ams5935_cs_high(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

volatile HAL_StatusTypeDef g_pressure_spi_last_status = HAL_OK;

static HAL_StatusTypeDef ams5935_tx_rx(AMS5935_HandleTypeDef *dev,
                                       uint8_t *tx,
                                       uint8_t *rx,
                                       uint16_t len)
{
    HAL_StatusTypeDef ret;

    ams5935_cs_low(dev);
    ret = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len, AMS5935_SPI_TIMEOUT_MS);
    g_pressure_spi_last_status = ret;

    /* Datasheet: short wait before deasserting CS. */
    for (volatile uint32_t i = 0U; i < AMS5935_SPI_CS_HOLD_NOP_COUNT; i++)
    {
        __NOP();
    }

    ams5935_cs_high(dev);
    return ret;
}

HAL_StatusTypeDef AMS5935_Init(AMS5935_HandleTypeDef *dev)
{
    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    ams5935_cs_high(dev);
    dev->last_valid = 0U;
    dev->held_last = 0U;
    dev->held_count = 0U;
    /* saturate_on_overflow is configuration, set by the caller like p_min /
     * p_max - deliberately NOT reset here. */
    dev->saturated_last = 0U;
    dev->saturated_count = 0U;
    /* Datasheet startup: 2.5 ms, no SCLK during this window. */
    HAL_Delay(5U);
    return HAL_OK;
}

HAL_StatusTypeDef AMS5935_StartSingleMeasurement(AMS5935_HandleTypeDef *dev)
{
    uint8_t tx[3] = { AMS5935_CMD_SINGLE_MEAS, 0x00U, 0x00U };
    uint8_t rx[3] = { 0U };

    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    return ams5935_tx_rx(dev, tx, rx, 3U);
}

HAL_StatusTypeDef AMS5935_StartOversamplingMeasurement(AMS5935_HandleTypeDef *dev)
{
    uint8_t tx[3] = { AMS5935_CMD_OVERSAMPLE_MEAS, 0x00U, 0x00U };
    uint8_t rx[3] = { 0U };

    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    return ams5935_tx_rx(dev, tx, rx, 3U);
}

static void ams5935_parse_frame(const uint8_t *rx, AMS5935_Data_t *data,
                                float p_min, float p_max)
{
    data->status = rx[0];
    data->pressure_raw =
        ((uint32_t)rx[1] << 16) |
        ((uint32_t)rx[2] << 8)  |
        ((uint32_t)rx[3]);
    data->temperature_raw =
        ((uint32_t)rx[4] << 16) |
        ((uint32_t)rx[5] << 8)  |
        ((uint32_t)rx[6]);
    data->pressure = AMS5935_ConvertPressure(data->pressure_raw, p_min, p_max);
    data->temperature_c = AMS5935_ConvertTemperature(data->temperature_raw);
}

static uint8_t ams5935_payload_unreliable(const AMS5935_Data_t *data)
{
    if ((AMS5935_StatusBusy(data->status) != 0U)
        || (AMS5935_StatusMemoryError(data->status) != 0U)
        || (AMS5935_StatusOverflow(data->status) != 0U))
    {
        return 1U;
    }
    if ((data->pressure_raw < AMS5935_DIGOUT_MIN_U32)
        || (data->pressure_raw > AMS5935_DIGOUT_MAX_U32))
    {
        return 1U;
    }
    return 0U;
}

static HAL_StatusTypeDef ams5935_read_frame(AMS5935_HandleTypeDef *dev,
                                            AMS5935_Data_t *data)
{
    uint8_t tx[7] = { AMS5935_CMD_READ, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U };
    uint8_t rx[7] = { 0U };
    const HAL_StatusTypeDef ret = ams5935_tx_rx(dev, tx, rx, 7U);
    if (ret != HAL_OK)
    {
        return ret;
    }
    ams5935_parse_frame(rx, data, dev->p_min, dev->p_max);
    return HAL_OK;
}

/* A genuine over-range reading, as opposed to a glitch. Busy and
 * memory-error take precedence: a busy frame with the overflow bit set is
 * still an unfinished conversion, not evidence of over-range. */
static uint8_t ams5935_payload_over_range(const AMS5935_Data_t *data)
{
    if ((AMS5935_StatusBusy(data->status) != 0U)
        || (AMS5935_StatusMemoryError(data->status) != 0U))
    {
        return 0U;
    }
    if (AMS5935_StatusOverflow(data->status) != 0U)
    {
        return 1U;
    }
    return ((data->pressure_raw < AMS5935_DIGOUT_MIN_U32)
            || (data->pressure_raw > AMS5935_DIGOUT_MAX_U32)) ? 1U : 0U;
}

/* Replace an over-range reading with full scale. See saturate_on_overflow
 * in pressure_sensors.h for why this must not simply hold the last sample.
 *
 * Direction comes from the payload when it is clearly past one end of the
 * digital span. With only the overflow status bit to go on, the payload
 * cannot be trusted, so it continues the sign of the last good reading -
 * over-range is reached by a flow ramping one way, not by a jump across
 * zero. last_good is then updated to the clipped value, so any busy / torn
 * read that follows during the same episode repeats full scale rather than
 * dropping back to a pre-overflow number. */
static HAL_StatusTypeDef ams5935_saturate(AMS5935_HandleTypeDef *dev,
                                          AMS5935_Data_t *data)
{
    uint8_t positive;

    if (data->pressure_raw > AMS5935_DIGOUT_MAX_U32)
    {
        positive = 1U;
    }
    else if (data->pressure_raw < AMS5935_DIGOUT_MIN_U32)
    {
        positive = 0U;
    }
    else
    {
        positive = ((dev->last_valid == 0U) || (dev->last_good.pressure >= 0.0f))
                       ? 1U : 0U;
    }

    data->pressure_raw = (positive != 0U) ? AMS5935_DIGOUT_MAX_U32
                                          : AMS5935_DIGOUT_MIN_U32;
    data->pressure = (positive != 0U) ? dev->p_max : dev->p_min;
    if (dev->last_valid != 0U)
    {
        /* Temperature rides on the same frame; keep the last trusted one. */
        data->temperature_raw = dev->last_good.temperature_raw;
        data->temperature_c = dev->last_good.temperature_c;
    }

    dev->last_good = *data;
    dev->last_valid = 1U;
    dev->held_last = 0U;
    dev->saturated_last = 1U;
    dev->saturated_count++;
    return HAL_OK;
}

static HAL_StatusTypeDef ams5935_hold_last(AMS5935_HandleTypeDef *dev,
                                           AMS5935_Data_t *data)
{
    dev->held_last = 1U;
    dev->held_count++;
    dev->saturated_last = 0U;
    if (dev->last_valid != 0U)
    {
        const uint8_t bad_status = data->status;
        *data = dev->last_good;
        data->status = bad_status;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef AMS5935_ReadRaw(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data)
{
    AMS5935_Data_t second;
    uint32_t tear;
    HAL_StatusTypeDef ret;

    if ((dev == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    ret = ams5935_read_frame(dev, data);
    if (ret != HAL_OK)
    {
        return ret;
    }

    /* Busy / memory-error / analog-overflow / out-of-span: the 24-bit
     * payload is not a finished conversion. Publishing it through the
     * ±5 mbar flow LUT (~100 slm/mbar near zero) makes a single bad tick
     * look like a multi-slm Q_patient spike. Hold the last good sample. */
    if (ams5935_payload_unreliable(data) != 0U)
    {
        /* Over-range is information, not a glitch - on devices that opt in,
         * report full scale rather than freezing on a pre-overflow value. */
        if ((dev->saturate_on_overflow != 0U)
            && (ams5935_payload_over_range(data) != 0U))
        {
            return ams5935_saturate(dev, data);
        }
        return ams5935_hold_last(dev, data);
    }

    /* Status can read "ready" while the 24-bit pressure register is still
     * updating (no EOC pin). A second F0 must match; a torn byte does not. */
    ret = ams5935_read_frame(dev, &second);
    if (ret != HAL_OK)
    {
        return ams5935_hold_last(dev, data);
    }
    if (ams5935_payload_unreliable(&second) != 0U)
    {
        return ams5935_hold_last(dev, data);
    }

    tear = (data->pressure_raw > second.pressure_raw)
               ? (data->pressure_raw - second.pressure_raw)
               : (second.pressure_raw - data->pressure_raw);
    if (tear > AMS5935_TEAR_MAX_COUNTS)
    {
        return ams5935_hold_last(dev, data);
    }

    dev->last_good = *data;
    dev->last_valid = 1U;
    dev->held_last = 0U;
    dev->saturated_last = 0U;
    return HAL_OK;
}

uint8_t AMS5935_WasSampleSaturated(const AMS5935_HandleTypeDef *dev)
{
    return (dev != NULL) ? dev->saturated_last : 0U;
}

uint8_t AMS5935_WasSampleHeld(const AMS5935_HandleTypeDef *dev)
{
    return (dev != NULL) ? dev->held_last : 1U;
}

HAL_StatusTypeDef AMS5935_ReadMeasurement(AMS5935_HandleTypeDef *dev,
                                          AMS5935_Data_t *data,
                                          uint8_t oversampling)
{
    HAL_StatusTypeDef ret;

    if ((dev == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    if (oversampling != 0U)
    {
        ret = AMS5935_StartOversamplingMeasurement(dev);
    }
    else
    {
        ret = AMS5935_StartSingleMeasurement(dev);
    }

    if (ret != HAL_OK)
    {
        return ret;
    }

    if (dev->use_eoc != 0U)
    {
        uint32_t t0_ms = HAL_GetTick();
        while (HAL_GPIO_ReadPin(dev->eoc_port, dev->eoc_pin) == GPIO_PIN_RESET)
        {
            if ((HAL_GetTick() - t0_ms) > 50U)
            {
                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        if (oversampling != 0U)
        {
            HAL_Delay(15U); /* datasheet: 14.5 ms */
        }
        else
        {
            HAL_Delay(6U);  /* datasheet: 4.0 ms */
        }
    }

    return AMS5935_ReadRaw(dev, data);
}

float AMS5935_ConvertPressure(uint32_t raw_p, float p_min, float p_max)
{
    const float sensp = (AMS5935_DIGOUT_MAX - AMS5935_DIGOUT_MIN) / (p_max - p_min);

    return (((float)raw_p - AMS5935_DIGOUT_MIN) / sensp) + p_min;
}

float AMS5935_ConvertTemperature(uint32_t raw_t)
{
    return (((float)raw_t * 165.0f) / 16777216.0f) - 40.0f;
}

uint8_t AMS5935_StatusBusy(uint8_t status)
{
    return (status >> 5) & 0x01U;
}

uint8_t AMS5935_StatusMemoryError(uint8_t status)
{
    return (status >> 2) & 0x01U;
}

uint8_t AMS5935_StatusOverflow(uint8_t status)
{
    return status & 0x01U;
}

extern ADC_HandleTypeDef hadc3;

volatile uint16_t g_pressure_analog_adc_raw = 0U;
volatile HAL_StatusTypeDef g_pressure_analog_adc_start_status = HAL_OK;
volatile HAL_StatusTypeDef g_pressure_analog_adc_poll_status = HAL_OK;
float g_pressure_analog_voltage_v = 0.0f;
float g_pressure_analog_mbar = 0.0f;
float g_pressure_analog_min_mbar = -50.0f;
float g_pressure_analog_max_mbar = 50.0f;

uint16_t pressure_sensor_read_analog_adc_raw(void)
{
    g_pressure_analog_adc_start_status = HAL_ADC_Start(&hadc3);
    if (g_pressure_analog_adc_start_status != HAL_OK)
    {
        return g_pressure_analog_adc_raw;
    }

    g_pressure_analog_adc_poll_status = HAL_ADC_PollForConversion(&hadc3, 10U);
    if (g_pressure_analog_adc_poll_status == HAL_OK)
    {
        g_pressure_analog_adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc3);
    }

    g_pressure_analog_voltage_v =
        ((((float)g_pressure_analog_adc_raw) / PRESSURE_ANALOG_ADC_FULL_SCALE)
         * PRESSURE_ANALOG_REF_VOLTAGE_V * PRESSURE_ANALOG_DIVIDER_GAIN)
        / PRESSURE_ANALOG_DIVIDER_ATTEN;

    /* AMS5105-0050-D-B: 0.5 V @ -50 mbar, 2.5 V @ 0, 4.5 V @ +50 mbar */
    g_pressure_analog_mbar = g_pressure_analog_min_mbar
        + ((g_pressure_analog_voltage_v - PRESSURE_ANALOG_V_MIN_V) / PRESSURE_ANALOG_V_SPAN_V)
          * (g_pressure_analog_max_mbar - g_pressure_analog_min_mbar);

    (void)HAL_ADC_Stop(&hadc3);
    return g_pressure_analog_adc_raw;
}
