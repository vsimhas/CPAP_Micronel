/**
  ******************************************************************************
  * @file    diskio_sd.c
  * @brief   FatFs disk I/O glue for STM32H7 SDMMC1 (HAL_SD polling).
  *
  *  Single-block transfers only. Multi-block (CMD25) polling writes were tried
  *  and produced SDMMC_ERROR_CMD_RSP_TIMEOUT on this board.
  ******************************************************************************
  */

#include "diskio.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#include <string.h>

extern SD_HandleTypeDef hsd1;
extern volatile uint8_t sd_card_present;
extern volatile HAL_StatusTypeDef sd_test_result;

volatile uint32_t g_diskio_last_hal_err = 0U;
volatile uint32_t g_diskio_fail_count = 0U;

static DSTATUS s_stat = STA_NOINIT;

__attribute__((aligned(32))) static uint8_t s_bounce[512];

static int sdReady(void)
{
    return (sd_card_present != 0U)
        && (sd_test_result == HAL_OK)
        && (hsd1.State != HAL_SD_STATE_RESET);
}

/* Wait until the card is ready for the next command. */
static int waitCardReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
    {
        if ((HAL_GetTick() - t0) > timeout_ms)
        {
            return 0;
        }
    }
    return 1;
}

static void sdClearFlags(void)
{
    __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
    if (hsd1.State != HAL_SD_STATE_RESET)
    {
        hsd1.State = HAL_SD_STATE_READY;
        hsd1.Context = SD_CONTEXT_NONE;
    }
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
    /* Do not HAL_SD_Abort here — it often causes subsequent CMD timeouts. */
    (void)waitCardReady(1000U);
    sdClearFlags();
    s_stat = 0U;
    return s_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    UINT i;

    if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }
    if (!sdReady())
    {
        return RES_NOTRDY;
    }

    for (i = 0U; i < count; i++)
    {
        uint8_t attempt;
        HAL_StatusTypeDef st = HAL_ERROR;

        for (attempt = 0U; attempt < 3U; attempt++)
        {
            if (!waitCardReady(1000U))
            {
                sdClearFlags();
                (void)waitCardReady(1000U);
            }
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)s_bounce,
                                              (int32_t)sizeof(s_bounce));
            st = HAL_SD_ReadBlocks(&hsd1, s_bounce, sector + i, 1U, 5000U);
            if (st == HAL_OK)
            {
                break;
            }
            g_diskio_last_hal_err = hsd1.ErrorCode;
            sdClearFlags();
        }
        if (st != HAL_OK)
        {
            g_diskio_fail_count++;
            g_diskio_last_hal_err = hsd1.ErrorCode;
            return RES_ERROR;
        }

        (void)waitCardReady(1000U);
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_bounce, (int32_t)sizeof(s_bounce));
        (void)memcpy(buff + ((size_t)i * 512U), s_bounce, 512U);
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    UINT i;

    if ((pdrv != 0U) || (buff == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }
    if (!sdReady())
    {
        return RES_NOTRDY;
    }

    for (i = 0U; i < count; i++)
    {
        uint8_t attempt;
        HAL_StatusTypeDef st = HAL_ERROR;

        (void)memcpy(s_bounce, buff + ((size_t)i * 512U), 512U);

        for (attempt = 0U; attempt < 6U; attempt++)
        {
            if (!waitCardReady(3000U))
            {
                sdClearFlags();
                (void)waitCardReady(3000U);
            }
            SCB_CleanDCache_by_Addr((uint32_t *)s_bounce, (int32_t)sizeof(s_bounce));
            st = HAL_SD_WriteBlocks(&hsd1, s_bounce, sector + i, 1U, 8000U);
            if (st == HAL_OK)
            {
                break;
            }
            g_diskio_last_hal_err = hsd1.ErrorCode;
            sdClearFlags();
            {
                /* Brief pause before retry — CMD_RSP_TIMEOUT often needs settle. */
                uint32_t t0 = HAL_GetTick();
                while ((HAL_GetTick() - t0) < 20U) { }
            }
        }
        if (st != HAL_OK)
        {
            g_diskio_fail_count++;
            g_diskio_last_hal_err = hsd1.ErrorCode;
            return RES_ERROR;
        }
        if (!waitCardReady(5000U))
        {
            g_diskio_fail_count++;
            g_diskio_last_hal_err = hsd1.ErrorCode;
            sdClearFlags();
            return RES_ERROR;
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
        (void)waitCardReady(1000U);
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
