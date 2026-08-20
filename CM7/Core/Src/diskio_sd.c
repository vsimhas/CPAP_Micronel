/**
  ******************************************************************************
  * @file    diskio_sd.c
  * @brief   FatFs disk I/O glue for STM32H7 SDMMC1 (HAL_SD polling).
  ******************************************************************************
  */

#include "diskio.h"
#include "main.h"
#include "stm32h7xx_hal.h"

extern SD_HandleTypeDef hsd1;
extern volatile uint8_t sd_card_present;
extern volatile HAL_StatusTypeDef sd_test_result;

static DSTATUS s_stat = STA_NOINIT;

static int sdReady(void)
{
    return (sd_card_present != 0U)
        && (sd_test_result == HAL_OK)
        && (hsd1.State != HAL_SD_STATE_RESET);
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0U)
    {
        return STA_NOINIT;
    }
    if (!sdReady())
    {
        s_stat = STA_NOINIT | STA_NODISK;
    }
    return s_stat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0U)
    {
        return STA_NOINIT;
    }
    if (!sdReady())
    {
        s_stat = STA_NOINIT | STA_NODISK;
        return s_stat;
    }
    s_stat = 0U;
    return s_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }
    if (!sdReady())
    {
        return RES_NOTRDY;
    }

    if (HAL_SD_ReadBlocks(&hsd1, buff, sector, count, 5000U) != HAL_OK)
    {
        return RES_ERROR;
    }
    {
        uint32_t t0 = HAL_GetTick();
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
        {
            if ((HAL_GetTick() - t0) > 2000U)
            {
                return RES_ERROR;
            }
        }
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }
    if (!sdReady())
    {
        return RES_NOTRDY;
    }

    if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)buff, sector, count, 5000U) != HAL_OK)
    {
        return RES_ERROR;
    }
    {
        uint32_t t0 = HAL_GetTick();
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
        {
            if ((HAL_GetTick() - t0) > 2000U)
            {
                return RES_ERROR;
            }
        }
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0U)
    {
        return RES_PARERR;
    }
    if (!sdReady() && (cmd != CTRL_SYNC))
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        if (buff == NULL) { return RES_PARERR; }
        *(DWORD *)buff = sd_card_info.LogBlockNbr;
        return RES_OK;
    case GET_SECTOR_SIZE:
        if (buff == NULL) { return RES_PARERR; }
        *(WORD *)buff = 512U;
        return RES_OK;
    case GET_BLOCK_SIZE:
        if (buff == NULL) { return RES_PARERR; }
        *(DWORD *)buff = 1U;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}
