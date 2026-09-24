
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/* Over-modulation. 0 = classic SVPWM (as shipped), 1 = PWMC_SetPhaseVoltage_OVM.
 *
 * WHAT IT BUYS. The fundamental line-to-line RMS a three-phase inverter can
 * produce is 0.707*Vbus under linear SVPWM and up to 0.780*Vbus in six-step,
 * so the ceiling moves by about 10%. Measured base: 37700 rpm on a 23.9 V
 * bus, back-EMF limited, with Iq at only 58-65% of its limit - the drive has
 * torque in hand and simply runs out of volts. Full six-step would give
 * ~41600 rpm; partial over-modulation, which is what is realistically usable
 * here, more like 39500.
 *
 * HAZARD 1 - THE OBSERVER. This is sensorless (STO+PLL), and the state
 * observer reconstructs applied phase voltage as Vbus * Valfa_beta, which
 * assumes LINEAR modulation. In over-modulation the real fundamental is up to
 * 10% above what that product implies, so the back-EMF estimate - and with it
 * speed and rotor angle - degrades exactly in the region being entered. This
 * is the same failure mode as the VBUS_PARTITIONING_FACTOR error fixed on
 * 2026-09-24, which made every reported rpm ~11% low for months. Watch for
 * MC_SPEED_FDBK: VARIANCE_THRESHOLD is 0.25 with 3 errors before fault.
 *
 * HAZARD 2 - CURRENT SAMPLING. Three-shunt reconstruction needs a minimum
 * duty window, TW_AFTER = DEADTIME + MAX_TNTR = 550 + 1000 = 1550 ns, which
 * is 3.1% of the 50 us period at 20 kHz. Over-modulation works BY pushing two
 * of the three duties toward the rails, so it eats that window as it makes
 * the extra voltage. Partial over-modulation should survive; deep
 * over-modulation approaching six-step probably will not on 3 shunts. The
 * symptom is erratic Iq rather than a clean fault.
 *
 * Also expect more torque and current ripple and more acoustic noise, which
 * matters in a device that sits next to someone sleeping. And more speed
 * means more regenerative energy on deceleration, into a bus whose
 * over-voltage margin is already down to ~1 V.
 *
 * HOW TO TEST. In ST Motor Pilot only, never in therapy. Ramp slowly past
 * 37700 and watch Speed Feedback and Iq. If the observer holds to ~39500 the
 * gain is real; if it faults, set this back to 0 and rebuild - nothing else
 * has to change, the SVPWM path is still compiled in. */
#ifndef FOC_OVERMODULATION_ENABLE
#define FOC_OVERMODULATION_ENABLE           1
#endif

/*** Speed measurement settings ***/
/* 2026-09-23: 36012 -> 40000, raised so CPAP can hold high setpoints against
 * real inspiratory flow. At 18 cmH2O the pressure loop was commanding 36000
 * for 40% of every inspiratory limb and still sagging 1.8-2.4 cmH2O; six of
 * fifteen measured breaths needed more than 36528 rpm to hold setpoint.
 *
 * READ THIS BEFORE TRUSTING THE NUMBER. Raising the reference saturation does
 * not raise the speed the motor can actually reach, and on this power stage
 * the two are not the same:
 *
 *   MOTOR_VOLTAGE_CONSTANT   0.5 V RMS ph-ph per kRPM
 *   OV_VOLTAGE_THRESHOLD_V   24.9  (so the bus is ~24 V)
 *   flux weakening           NOT generated - no FW source files exist
 *
 * Back-EMF at 40 kRPM is 20 V RMS line-to-line. With SVPWM a 24 V bus yields
 * about 24/sqrt(2) = 17 V RMS line-to-line before over-modulation, i.e. a
 * BEMF-limited ceiling near 34 kRPM, before IR drop and dead-time. That
 * matches the bench exactly: across three captures on 2026-09-23 the measured
 * mechanical speed never exceeded 35742 rpm and typically plateaued at
 * 34500-35400 while the command sat at 36000. The command-to-actual gap was
 * SUSTAINED through the flat top of inspiration rather than closing, which is
 * the signature of a voltage ceiling and not of rotor inertia.
 *
 * So 36012 was very close to what this bus can deliver, and the profiler
 * arriving at that figure was it measuring the voltage limit. Reaching 40000
 * needs more bus voltage or flux weakening; neither is a firmware constant.
 *
 * What this change does buy, and why it is still worth making: the speed
 * reference is no longer clipped below the voltage limit, so whatever
 * headroom does exist between ~36 k and the real ceiling becomes usable.
 *
 * RISK, accepted deliberately. With the reference above the reachable speed
 * the speed PI cannot null its error, so it will wind up and hold q-axis
 * current at NOMINAL_CURRENT (1.345 A) for the whole of a large inspiration.
 * Watch motor temperature, and watch for MC_SPEED_FDBK - VARIANCE_THRESHOLD
 * is 0.25 with M1_SS_MEAS_ERRORS_BEFORE_FAULTS of 3. Derived observer
 * constant C3 also scales with MAX_BEMF_VOLTAGE and therefore moves by
 * 40000/36012 = 1.11x, so observer behaviour is not identical at any speed,
 * not just the new top end. Revert this line alone to undo all of it. */
/* 2026-09-24: 40000 -> 43000, so the blower's full 42500 rpm rating can be
 * commanded with a little margin above it. Note this also raises
 * MAX_BEMF_VOLTAGE and therefore observer constant C3 by 43000/40000 = 1.075.
 *
 * Whether the shaft actually gets there is a separate question and the bus
 * probably decides it - see the back-EMF note above, now to be re-read with
 * the CORRECTED bus figure of ~23.9 V rather than the 21 V the broken divider
 * scaling reported. */
#define MAX_APPLICATION_SPEED_RPM           43000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD                  0.25 /*!< Maximum accepted variance on speed estimates (percentage) */

/* State observer scaling factors F1 */
#define F1                                  8192
#define F2                                  16384
#define F1_LOG                              LOG2((8192))
#define F2_LOG                              LOG2((16384))

/* State observer constants */
#define GAIN1                               -11146
#define GAIN2                               5275

/* Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                         958
#define PLL_KI_GAIN                         34
#define PLL_KPDIV                           16384
#define PLL_KPDIV_LOG                       LOG2((PLL_KPDIV))
#define PLL_KIDIV                           65535
#define PLL_KIDIV_LOG                       LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP                  64 /*!< Depth of the FIFO used  to average mechanical speed in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG              LOG2((64))
#define STO_FIFO_DEPTH_UNIT                 64 /*!< Depth of the FIFO used to average mechanical speed in the unit defined by #SPEED_UNIT */
#define M1_BEMF_CONSISTENCY_TOL             64 /* Parameter for B-emf amplitude-speed consistency */
#define M1_BEMF_CONSISTENCY_GAIN            64 /* Parameter for B-emf amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       20000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           ES_GPIO

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT               2168
#define PID_TORQUE_KI_DEFAULT               806
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 2168
#define PID_FLUX_KI_DEFAULT                 806
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            8192
#define TF_KIDIV                            16384
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((8192))
#define TF_KIDIV_LOG                        LOG2((16384))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

#define PID_SPEED_KP_DEFAULT                1637/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT                256/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            128
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((128))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             1.445

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            12965
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V              24.9 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V              7.0 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          110 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

/* Phase 1 */
#define PHASE1_DURATION                     320 /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT_A              1.345

/* Phase 2 */
#define PHASE2_DURATION                     4322 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT_A              1.345

/* Phase 3 */
#define PHASE3_DURATION                     0 /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT_A              1.345

/* Phase 4 */
#define PHASE4_DURATION                     0 /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT_A              1.345

/* Phase 5 */
#define PHASE5_DURATION                     0 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT             (14405*SPEED_UNIT/U_RPM)
#define PHASE5_FINAL_CURRENT_A              1.345

#define ENABLE_SL_ALGO_FROM_PHASE           2

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                  0  /*!< degrees [0...359] */

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM               12965
#define NB_CONSECUTIVE_TESTS                2 /* corresponding to former
                                                 NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE / MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT              17 /*!< It expresses how much estimated speed can exceed forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT              15 /*!< It expresses how much estimated speed can be below forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */

#define TRANSITION_DURATION                 25 /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(64)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(64)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (8 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
