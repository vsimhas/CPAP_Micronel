
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

/* 2026-10-01: KP 1637 -> 4000 (Workbench value was 1637).
 * 1637/128 is ~0.43 mA per rpm of speed error; against the measured inertia
 * (~40 mA per krpm/s) that is a ~1.7 Hz speed loop. In breathing at 16 cmH2O
 * the rotor had to be >1000 rpm off its reference before the PI pushed hard
 * either way, and it stayed high while the command fell in the second half
 * of each inhale - most of the inspiratory over-pressure. Bench, leak rig,
 * open loop, 2026-10-01:
 *   31<->37k ramps at 15 krpm/s (kp_track.py):
 *      kp     lag      max err      overshoot    |id|max
 *      1637   55-60ms  900-1010rpm  190/440 rpm  0.23 A
 *      4000   30-35ms  510-580rpm   <= 40 rpm    0.19 A
 *      8000   unstable near 36-37k (+/-800-950 rpm, |id| 0.96 A)
 *   replay of recorded 16 cmH2O breath trajectories (traj_kp.py), ideal
 *   speed: rms error 712 -> 511 rpm, rotor-high-while-falling +591 -> +168
 *   rpm (~0.6 -> ~0.2 cmH2O), |id|max 0.70 -> 0.24 A, no faults to 38.5k.
 * Hold ripple at 31k unchanged (3 rpm, ~37 mA). 4000 keeps ~2x margin to the
 * 8000 oscillation. The 2400/512 entry below was large STEPS at 38k, which
 * are current-limited whatever kp is; this is tracking a moving reference. */
#define PID_SPEED_KP_DEFAULT                4000/(SPEED_UNIT/10) /* Workbench computed 1637 for the 01Hz unit */
/* 2026-09-25: KI 256 -> 1024 (Workbench value was 256).
 * Speed-step tests at the 38 krpm top, closed circuit, open loop:
 *      kp/ki        t90 (steps)    settle       reaches target
 *      1637/256     1.16-1.70 s    0.9-2.4 s    ~500 rpm short at 2.5 s
 *      1637/512     0.46-0.82 s    0.36-1.2 s   yes
 *      2400/512     1.04-1.22 s    0.9-1.7 s    50-170 short (kp does not help)
 *      1637/1024    0.24-0.44 s    0.2-0.54 s   yes
 * At 1024 large steps are current-limited (iq_ref at IQMAX 50-58 % of the
 * rise, ~21 krpm/s), current noise at 30-38 krpm unchanged (iq sd 59-70 mA,
 * id sd 65-85 mA), no faults. Ramp overshoot 430 rpm (was 10).
 * In therapy at 20 cmH2O the rotor had been ~1.2 krpm short of its command at
 * the pressure minimum; with 1024 (and the therapy ceiling at 38500,
 * safety_supervisor.c) the mask-end dip at matched flow fell 2.53 -> 2.10
 * cmH2O with no drive stops or faults. Pair the two: with the faster loop
 * the rotor really reaches its command, and above ~39 krpm under load
 * current control breaks down. */
#define PID_SPEED_KI_DEFAULT                1024/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
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
/* Speed-PI output / integral limit (boot default). 2026-10-01: can be changed
 * at runtime with "foc set iq_max_ma <mA>" (CM4 clamps to 500-1800 mA, see
 * blower_ipc_cm4.c), so a higher limit can be trialled against this one in
 * the same session and is gone on reboot.
 *
 * Why a higher limit is being tried: at 20 cmH2O, breath-onset pressure
 * dips are acceleration-limited. At
 * ~32 krpm holding speed against the airflow takes ~850-1000 mA, leaving
 * only ~450-600 mA of 1445 to accelerate (rotor ~17 krpm/s, command pinned
 * at the ceiling ~100 ms before flow onset). 1.65 A adds ~40% accelerating
 * current.
 *
 * RISK - read before raising it: the STSPIN830 trips its internal
 * over-current protection at 3.0 A min / 3.5 A typ (DP_FAULT), and with
 * this motor's ~50-70 uH the 20 kHz ripple is ~1.4-2.0 A p-p, i.e. peaks
 * ~0.7-1.0 A above the sampled current. At 1.445 A that was already enough
 * to trip during the old full-current start (fixed by the CM7 post-hand-
 * over ramp). Phase-current peaks have NOT been measured on a scope.
 * Only make a higher value the default once starts and breathing show no
 * DP_FAULT and the STSPIN830 does not run hot. Motor copper loss at the
 * limit: ~1.55 W at 1.445 A, ~2.0 W at 1.65 A.
 *
 * Result (breathing A/B, 2026-10-01): 1.65 A flat helped at 16 cmH2O but
 * tripped DP_FAULT once at 38.2 krpm at 20 cmH2O. This value is therefore
 * kept as the limit above ~37 krpm, and a speed-dependent boost (1.65 A up
 * to 35.5 krpm, tapering to this by 37 krpm) is applied on top - see
 * BlowerIpc_CM4_UpdateIqLimit() in blower_ipc_cm4.c. */
#define IQMAX_A                             1.445

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            12965
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
/* 2026-09-25: 24.9 -> 28.0 V.
 *
 * 24.9 was harmless while VBUS_PARTITIONING_FACTOR was mis-scaled - the real
 * trip was then ~28.1 V. Correcting the factor on 2026-09-24 made 24.9 real,
 * leaving ~1 V above the 23.9 V supply, and with the brake resistor disabled
 * a hard end-of-inspiration decel now regenerates past it. Measured in
 * therapy at 20 cmH2O (human breathing): pressure overshoot to 23.4 cmH2O,
 * command 40000 -> 29300 rpm in 180 ms, rotor -32 krpm/s, drive stopped at
 * ~32.6 krpm with the bus at 24.6 V just after. The restart-while-coasting
 * hold-off then kept it off for 8.8 s: ~13 s without pressure.
 *
 * 28.0 restores the headroom the machine ran with before the scaling fix.
 * Ratings on the regen path (BLOWER_BOARD_FINAL_SCH): 24 V input bulk
 * 330 uF / 35 V (C5, C6, C10, C13) and 330 uF / 50 V (C1); motor bus
 * 33 uF / 50 V (C89, C90); STSPIN830 45 V max. 28 V is 80 % of the lowest
 * (35 V). The bench supply also sees the regen - check it tolerates being
 * pushed to 28 V. The brake stays disabled (BRAKE_VBUS_ON_V, blower_ipc_cm4.c
 * is kept above this trip). */
#define OV_VOLTAGE_THRESHOLD_V              28.0 /*!< Over-voltage threshold */
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
