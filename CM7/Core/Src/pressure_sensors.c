
#include "pressure_sensors.h"

#define AMS5935_CMD_SINGLE_MEAS      0xAA
#define AMS5935_CMD_OVERSAMPLE_MEAS  0xAD
#define AMS5935_CMD_READ             0xF0

#define AMS5935_SPI_TIMEOUT_MS       100

static void AMS5935_CS_Low(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void AMS5935_CS_High(AMS5935_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef spiRet = HAL_OK;

static HAL_StatusTypeDef AMS5935_TxRx(AMS5935_HandleTypeDef *dev,
                                      uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t len)
{
    AMS5935_CS_Low(dev);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len, AMS5935_SPI_TIMEOUT_MS);
	  spiRet = ret;

    // Datasheet notes a short wait (~1 us) before deactivating SS may be needed
    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    AMS5935_CS_High(dev);
    return ret;
}

HAL_StatusTypeDef AMS5935_Init(AMS5935_HandleTypeDef *dev)
{
    // Keep CS inactive
    AMS5935_CS_High(dev);

    // Datasheet startup time: 2.5 ms, and no SCLK transition during this time
    HAL_Delay(5);

    return HAL_OK;
}

HAL_StatusTypeDef AMS5935_StartSingleMeasurement(AMS5935_HandleTypeDef *dev)
{
    uint8_t tx[3] = { AMS5935_CMD_SINGLE_MEAS, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };

    return AMS5935_TxRx(dev, tx, rx, 3);
}

HAL_StatusTypeDef AMS5935_StartOversamplingMeasurement(AMS5935_HandleTypeDef *dev)
{
    uint8_t tx[3] = { AMS5935_CMD_OVERSAMPLE_MEAS, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };

    return AMS5935_TxRx(dev, tx, rx, 3);
}
uint8_t rdrx[7] = { 0 };
HAL_StatusTypeDef AMS5935_ReadRaw(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data)
{
    uint8_t tx[7] = { AMS5935_CMD_READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    

    HAL_StatusTypeDef ret = AMS5935_TxRx(dev, tx, rdrx, 7);
    if (ret != HAL_OK)
        return ret;

    data->status = rdrx[0];
    data->pressure_raw =
        ((uint32_t)rdrx[1] << 16) |
        ((uint32_t)rdrx[2] << 8)  |
        ((uint32_t)rdrx[3]);

    data->temperature_raw =
        ((uint32_t)rdrx[4] << 16) |
        ((uint32_t)rdrx[5] << 8)  |
        ((uint32_t)rdrx[6]);

    data->pressure = AMS5935_ConvertPressure(data->pressure_raw, dev->p_min, dev->p_max);
    data->temperature_c = AMS5935_ConvertTemperature(data->temperature_raw);

    return HAL_OK;
}

HAL_StatusTypeDef AMS5935_ReadMeasurement(AMS5935_HandleTypeDef *dev, AMS5935_Data_t *data, uint8_t oversampling)
{
    HAL_StatusTypeDef ret;

    if (oversampling)
        ret = AMS5935_StartOversamplingMeasurement(dev);
    else
        ret = AMS5935_StartSingleMeasurement(dev);

    if (ret != HAL_OK)
        return ret;

    if (dev->use_eoc)
    {
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(dev->eoc_port, dev->eoc_pin) == GPIO_PIN_RESET)
        {
            if ((HAL_GetTick() - t0) > 50)
                return HAL_TIMEOUT;
        }
    }
    else
    {
        if (oversampling)
            HAL_Delay(15);   // datasheet: 14.5 ms
        else
            HAL_Delay(6);    // datasheet: 4.0 ms
    }

    return AMS5935_ReadRaw(dev, data);
}

float AMS5935_ConvertPressure(uint32_t raw_p, float p_min, float p_max)
{
    // Datasheet:
    // Digout_p(min) = 0.1 * 2^24
    // Digout_p(max) = 0.9 * 2^24
    // pressure = (raw - Digout_min)/Sensp + p_min
    // Sensp = (Digout_max - Digout_min)/(p_max - p_min)

    const float digout_min = 0.1f * 16777216.0f;  // 2^24
    const float digout_max = 0.9f * 16777216.0f;
    const float sensp = (digout_max - digout_min) / (p_max - p_min);

    return (((float)raw_p - digout_min) / sensp) + p_min;
}

float AMS5935_ConvertTemperature(uint32_t raw_t)
{
    // Datasheet:
    // T = (DigoutT * 165 / 2^24) - 40
    return (((float)raw_t * 165.0f) / 16777216.0f) - 40.0f;
}

uint8_t AMS5935_StatusBusy(uint8_t status)
{
    return (status >> 5) & 0x01;
}

uint8_t AMS5935_StatusMemoryError(uint8_t status)
{
    return (status >> 2) & 0x01;
}

uint8_t AMS5935_StatusOverflow(uint8_t status)
{
    return status & 0x01;
}

extern ADC_HandleTypeDef hadc3;
uint16_t usAnalogPresRaw = 0;
volatile HAL_StatusTypeDef adc_start_status = HAL_OK;
volatile HAL_StatusTypeDef adc_poll_status = HAL_OK;
float fVoltageRaw = 0.0f;
float fPresAnalog = 0.0f;
float fPresAnalogMin = -50.0f;
float fPresAnalogMax = 50.0f;
uint16_t usGetAnalagPresValueRaw(void)
{
  adc_start_status = HAL_ADC_Start(&hadc3);
  if (adc_start_status != HAL_OK)
  {
    return usAnalogPresRaw;
  }

  adc_poll_status = HAL_ADC_PollForConversion(&hadc3, 10U);
  if (adc_poll_status == HAL_OK)
  {
    usAnalogPresRaw = (uint16_t)HAL_ADC_GetValue(&hadc3);
  }
	
	fVoltageRaw = ((((float)usAnalogPresRaw)/65535.0f)*3.3f*3.0f)/2.0f;
	
	/* AMS5105-0050-D-B: 0.5 V @ -50 mbar, 2.5 V @ 0, 4.5 V @ +50 mbar */
	fPresAnalog = fPresAnalogMin + ((fVoltageRaw - 0.5f) / 4.0f) * (fPresAnalogMax - fPresAnalogMin);

  (void)HAL_ADC_Stop(&hadc3);
  return usAnalogPresRaw;
}

