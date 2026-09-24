
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/
#define HW_DEAD_TIME_NS                      550 /*!< Dead-time inserted
                                                      by HW if low side signals
                                                      are not used */
/*********** Bus voltage sensing section ****************/
/* 2026-09-24: 0.0625 -> 0.05543, measured on the board.
 *
 * The schematic divider is R137 180K / R138 12K = 12/192 = 0.0625, which is
 * what this was. The fitted hardware does not match it. With V_MOTOR at
 * 23.87 V the V_MOT node measures 1.323 V, not the 1.492 V that 0.0625
 * predicts - so the real ratio is 1.323/23.87 = 0.05543, about 1/18.0.
 *
 * Best-fit standard pairs against that measurement:
 *      R137 180K / R138 12.0K  (as drawn)   1.4919 V   +12.8%
 *      R137 180K / R138 10.5K               1.3157 V    -0.6%   <-- likely
 *      R137 200K / R138 12.0K               1.3511 V    +2.1%
 * so a 10.5K appears to have been fitted where the schematic calls for 12K.
 * CONFIRM WITH AN OHMMETER, power off, and correct this number if the pair
 * turns out different - the value here should always be the measured ratio,
 * not the drawn one.
 *
 * Why it mattered far beyond reporting: the raw ADC word is fed straight to
 * the sensorless observer (STO_Inputs.Vbus in mc_tasks_foc.c), which
 * reconstructs the applied phase voltage from it. Believing full scale was
 * 52.8 V when it is really 59.5 V made the observer underestimate applied
 * voltage, hence back-EMF, hence SPEED. Every rpm this firmware has reported
 * was low by roughly this factor - the "34500 rpm ceiling" that four bench
 * captures agreed on was nearer 39000.
 *
 * !! CHECK OVER-VOLTAGE HEADROOM AFTER FLASHING !!
 * OVERVOLTAGE_THRESHOLD_d is derived from this factor, so correcting it moves
 * the trip from an effective 28.1 V of real bus down to the intended 24.9 V.
 * Against a 23.9 V supply that leaves about 1.0 V, where there used to be
 * 4.2 V, and mask-off parks regenerate into this bus. If nuisance OV faults
 * appear, that is why - raise OV_VOLTAGE_THRESHOLD_V to suit the DC-link
 * capacitor rating rather than putting this number back. */
#define VBUS_PARTITIONING_FACTOR             0.05543 /*!< It expresses how
                                                      much the Vbus is attenuated
                                                      before being converted into
                                                      digital value */
/* Was 21U, which was the mis-scaled reading rather than the supply. The bench
 * supply is 24 V and V_MOTOR measures 23.87 V at the terminals. */
#define NOMINAL_BUS_VOLTAGE_V                24U
/******** Current reading parameters section ******/
/*** Topology ***/
#define THREE_SHUNT

#define RSHUNT                               0.165

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN                   1.53

/*** Noise parameters ***/
#define TNOISE_NS                            500
#define TRISE_NS                             1000
#define MAX_TNTR_NS                          TRISE_NS

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                                 0.579 /*!< in Volts */
#define T0_C                                 25 /*!< in Celsius degrees */
#define dV_dT                                0.027 /*!< V/Celsius degrees */
#define T_MAX                                110 /*!< Sensor measured
                                                     temperature at maximum
                                                     power stage working
                                                     temperature, Celsius degrees */

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
