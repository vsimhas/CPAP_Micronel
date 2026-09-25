#ifndef TEST_STM32H7XX_HAL_H
#define TEST_STM32H7XX_HAL_H
/* Host-only time source. Never add this directory to firmware include paths. */
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
