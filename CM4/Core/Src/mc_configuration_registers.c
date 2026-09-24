
/*******************************************************************************
  * @file    mc_configuration_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides project configuration information registers.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "mc_type.h"
#include "mc_configuration_registers.h"
#include "register_interface.h"
#include "parameters_conversion.h"

#define FIRMWARE_NAME_STR "ST MC SDK\tVer.6.4.2"

const char_t CTL_BOARD[] = "NUCLEO-H745ZI-Q";
static const char_t M1_PWR_BOARD[] = "X-NUCLEO-IHM16M1";
const char_t FIRMWARE_NAME [] = FIRMWARE_NAME_STR;

const GlobalConfig_reg_t globalConfig_reg =
{
  .SDKVersion     = SDK_VERSION,
  .MotorNumber    =  1 ,
  .MCP_Flag       = FLAG_MCP_OVER_STLINK + FLAG_MCP_OVER_UARTA + FLAG_MCP_OVER_UARTB,
  .MCPA_UARTA_LOG = 10, /* Used by Motor Pilot to detect data log enabled. */
  .MCPA_UARTB_LOG = 0,
  .MCPA_STLNK_LOG = 0,
};

/* 2026-09-24: these were hardcoded literals and had gone stale.
 *
 * This struct is what ST Motor Pilot reads to configure its own UI, and it
 * bounds the speed setpoint the tool will send. maxMechanicalSpeed sat at
 * 36012 - the pre-2026-09-23 MAX_APPLICATION_SPEED_RPM - so once that macro
 * went to 40000 the firmware accepted 40000 while Motor Pilot refused
 * anything past roughly 38000, it allowing a few percent over what it reads.
 * The wall was in the tool's copy of a number that nothing kept in sync.
 *
 * nominalVoltage was 21, which was never the supply: it was the bus as
 * mis-scaled by the wrong VBUS_PARTITIONING_FACTOR. The bench supply is 24 V.
 * Two independent places had been quietly calibrated to a broken reading
 * instead of to the hardware.
 *
 * Everything here now derives from the macros so it cannot drift again. To
 * change a value, change the macro. */
static const ApplicationConfig_reg_t M1_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = MAX_APPLICATION_SPEED_RPM,
  .maxReadableCurrent = M1_MAX_READABLE_CURRENT,
  .nominalCurrent     = NOMINAL_CURRENT_A,
  .nominalVoltage     = NOMINAL_BUS_VOLTAGE_V,
  .driveType          = DRIVE_TYPE_M1,
};

//cstat !MISRAC2012-Rule-9.2
/* Same treatment: these duplicated pmsm_motor_parameters.h by hand.
 *
 * ratedFlux is MOTOR_VOLTAGE_CONSTANT, and that one is still suspect - 0.5 V
 * RMS ph-ph per kRPM does not fit the bench. The shaft reaches ~37.7 kRPM on
 * a 23.9 V bus, which at 0.5 would need more line-to-line volts than the bus
 * can supply; the implied constant is nearer 0.45. It feeds MAX_BEMF_VOLTAGE
 * and observer gain C3, so it is worth measuring properly - back-drive the
 * blower and scope phase-to-phase AC against rpm - rather than leaving a
 * datasheet number nobody has checked. */
static const MotorConfig_reg_t M1_MotorConfig_reg =
{
  .polePairs  = POLE_PAIR_NUM,
  .ratedFlux  = MOTOR_VOLTAGE_CONSTANT,
  .rs         = RS,
  .ls         = LS,
  .ld         = LS,
  .maxCurrent = NOMINAL_CURRENT_A,
  .name = "WM6850_H747_LowRes_OpnAt"
};

static const FOCFwConfig_reg_t M1_FOCConfig_reg =
{
  .primarySensor      = (uint8_t)PRIM_SENSOR_M1,
  .auxiliarySensor    = (uint8_t)AUX_SENSOR_M1,
  .topology           = (uint8_t)TOPOLOGY_M1,
  .FOCRate            = (uint8_t)REGULATION_EXECUTION_RATE,
  .PWMFrequency       = (uint32_t)PWM_FREQUENCY,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t)configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .configurationFlag2 = (uint16_t)configurationFlag2_M1, //cstat !MISRAC2012-Rule-10.1_R6
};

const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD};
const FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS] = {&M1_FOCConfig_reg};
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS] = {&M1_MotorConfig_reg};
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS] = {&M1_ApplicationConfig_reg};

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
