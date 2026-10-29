/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
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
#ifndef PMSM_MOTOR_PARAMETERS_H
#define PMSM_MOTOR_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
/* Micronel U65ML-024KEE-5 radial blower, profiled with ST Motor Control
 * Workbench: profile "Micronel_U65" (description "U65 motor Micronel"),
 * ~/.st_workbench/hardware/motor/Micronel_U65.json:
 *   rs 0.7367 ohm, ls 0.11 mH, BEmfConstant 0.4014 V rms ph-ph / krpm,
 *   polePairs 1, nominalCurrent 1.3165 A, maxRatedSpeed 40626 rpm,
 *   nominalDCVoltage 20.74 V, inertia 0.56, friction 1.51.
 * Values below are what Workbench wrote in its reference project
 * D:\Code\CPAP_Micronel (STM32H745, same STSPIN830 current sensing as this
 * board: 0.165 ohm shunt, x1.53).
 * Datasheet (PRELIMINARY, 27.03.2026): 24 V nominal, 12-26.4 V; 1 pole pair;
 * shut-off 56 hPa at 46000 rpm, free air 405 L/min at 40000 rpm (continuous
 * operation); max acceleration 150 rpm/ms; resistance, inductance, speed and
 * torque constants "tbd". Counter-clockwise viewed on the inlet.
 * Previous blowers: WM6850 (RS 0.26, LS 0.00007, Ke 0.5, 1.345 A) - the values
 * this file was copied with; TKFan BA7060H24B (CPAP_TKFan: RS 1.07, LS 0.00021,
 * Ke 0.5, 1.314 A). */
#define POLE_PAIR_NUM           1 /* Number of motor pole pairs */
#define RS                      0.74 /* Stator resistance , ohm*/
#define LS                      0.00011 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

/* Measured 0.4014; Workbench writes 0.4 (one decimal), and its observer gain
 * GAIN2 (drive_parameters.h) was computed for 0.4 through MAX_BEMF_VOLTAGE ->
 * C3. Change the two together. */
#define MOTOR_VOLTAGE_CONSTANT  0.4 /*!< Volts RMS ph-ph /kRPM */
/* The profiler's figure, measured on a 20.74 V bus. The datasheet's maximum
 * speed is "tbd" (its curves run to 46-50.6 krpm at 24 V); the speed this
 * drive reaches is set by back-EMF against the bus (see the note on
 * MAX_APPLICATION_SPEED_RPM). Documentation only - nothing in CM4 consumes
 * this macro. */
#define MOTOR_MAX_SPEED_RPM     40626 /*!< Maximum rated speed  */
#define NOMINAL_CURRENT_A       1.316

#define ID_DEMAG_A              -1.316 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        300 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR          400  /*!< Number of pulses per
                                            revolution */

#endif /* PMSM_MOTOR_PARAMETERS_H */
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
