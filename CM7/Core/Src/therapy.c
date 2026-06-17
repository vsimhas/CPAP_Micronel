/**
  ******************************************************************************
  * @file    therapy.c
  * @brief   CPAP therapy state machine.
  *
  *  See therapy.h for the architecture overview. This file owns the
  *  state transitions, ramp interpolator, run-time counter and the
  *  glue calls into blower_ctrl / blower_ipc.
  ******************************************************************************
  */

#include "therapy.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "leak_estimator.h"
#include "dbg_log.h"

#include <math.h>
#include <stdint.h>

/* === Compile-time defaults === */
#define THERAPY_RAMP_START_CMH2O    4.0f         /* AirSense-style start    */
#define THERAPY_TARGET_DEFAULT      8.0f         /* sensible adult default  */
#define THERAPY_PRESS_MIN_CMH2O     4.0f
#define THERAPY_PRESS_MAX_CMH2O     25.0f
#define THERAPY_RAMP_MAX_MINUTES    45U
#define THERAPY_STOP_DURATION_S     2.0f         /* spin-down hold time     */

/* Safety-supervisor compile-time limits (AMS5935 sensor range).  */
#define THERAPY_SENSOR_MIN_CMH2O    (-10.0f)
#define THERAPY_SENSOR_MAX_CMH2O    ( 50.0f)
#define THERAPY_SENSOR_TIMEOUT_S    ( 2.0f)

/* === Public configuration (debugger-writable volatiles) === */
volatile float    g_therapy_target_cmh2o   = THERAPY_TARGET_DEFAULT;
volatile uint8_t  g_therapy_ramp_minutes   = 0U;     /* OFF: hit target now */
volatile uint8_t  g_therapy_mode           = (uint8_t)THERAPY_MODE_CPAP;

/* === Safety-supervisor tunables ============================================
 *
 *  All thresholds and timeouts are debugger-writable so faults can be
 *  tightened/loosened without rebuilding firmware.
 */
/* Hard RPM ceiling enforced *during therapy*. The PID's own clamp
 * (g_blower_ctrl_rpm_max) is the absolute hardware safety limit; this is a
 * tighter clinical ceiling that gives ~6 krpm of headroom before the
 * hardware clamp engages, so you see the saturation fault fire instead of
 * silently pinning at hardware max. */
volatile int32_t  g_therapy_safe_rpm_max         = 30000;

/* THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM thresholds. */
volatile float    g_therapy_low_press_cmh2o      = 1.5f;
volatile int32_t  g_therapy_high_rpm_threshold   = 12000;
volatile float    g_therapy_low_press_timeout_s  = 4.0f;

/* THERAPY_FAULT_OVER_PRESSURE thresholds. */
volatile float    g_therapy_overpress_band_cmh2o = 5.0f;
volatile float    g_therapy_overpress_timeout_s  = 3.0f;

/* THERAPY_FAULT_RPM_SATURATED thresholds. */
volatile float    g_therapy_saturate_band_cmh2o  = 1.5f;
volatile float    g_therapy_saturated_timeout_s  = 6.0f;

/* Master safety enable. Set to 0 in the watch window if you need to
 * defeat the supervisor for a deliberately-extreme bench test. Note this
 * does NOT defeat the PID's own absolute RPM clamp. */
volatile uint8_t  g_therapy_safety_enabled       = 1U;

/* === Live state (debugger-readable volatiles) === */
volatile uint8_t  g_therapy_state          = (uint8_t)THERAPY_STATE_IDLE;
volatile float    g_therapy_setpoint_cmh2o = 0.0f;
volatile float    g_therapy_measured_cmh2o = 0.0f;
volatile float    g_therapy_leak_slm       = 0.0f;
volatile uint32_t g_therapy_run_ms         = 0U;     /* time in RAMP+RUNNING*/
volatile uint32_t g_therapy_ramp_total_ms  = 0U;     /* this session        */
volatile uint32_t g_therapy_ramp_elapsed_ms = 0U;
volatile uint32_t g_therapy_stop_elapsed_ms = 0U;

/* Safety-supervisor state. Each fault has its own running timer so a
 * brief excursion (e.g. a single tick of bad SPI) doesn't trip; the fault
 * only fires after the offending condition persists for its timeout. */
volatile uint8_t  g_therapy_fault          = (uint8_t)THERAPY_FAULT_NONE;
volatile uint32_t g_therapy_low_press_ms   = 0U;
volatile uint32_t g_therapy_overpress_ms   = 0U;
volatile uint32_t g_therapy_outrange_ms    = 0U;
volatile uint32_t g_therapy_saturated_ms   = 0U;

/* === Internal === */
static float s_session_target_cmh2o  = THERAPY_TARGET_DEFAULT;

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void setState(TherapyState_t s)
{
    if ((uint8_t)s != g_therapy_state)
    {
        DBG_I("THR", "state %u -> %u",
              (unsigned)g_therapy_state, (unsigned)s);
        g_therapy_state = (uint8_t)s;
    }
}

static void resetFaultTimers(void)
{
    g_therapy_low_press_ms = 0U;
    g_therapy_overpress_ms = 0U;
    g_therapy_outrange_ms  = 0U;
    g_therapy_saturated_ms = 0U;
}

/* Latch a fault: log it, kill the blower, transition to FAULT. Once
 * latched, only Therapy_ClearFault() (or Therapy_Stop -> Start) clears it
 * - we never auto-recover from a safety event. */
static void latchFault(TherapyFault_t f, const char* why)
{
    if (g_therapy_fault != (uint8_t)THERAPY_FAULT_NONE)
    {
        return;  /* first fault wins */
    }

    DBG_E("THR", "FAULT %u: %s (rpm=%ld meas=%.2f sp=%.2f)",
          (unsigned)f, why,
          (long)BlowerCtrl_GetOutputRpm(),
          (double)g_therapy_measured_cmh2o,
          (double)g_therapy_setpoint_cmh2o);

    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();

    g_therapy_fault = (uint8_t)f;
    setState(THERAPY_STATE_FAULT);
}

/* Per-tick safety check during RAMP/RUNNING. Returns true if a fault was
 * latched this tick; the caller should bail out of the rest of the
 * state-machine work. */
static bool runSafetyChecks(uint32_t dt_ms)
{
    if (g_therapy_safety_enabled == 0U)
    {
        return false;
    }

    const float meas    = g_therapy_measured_cmh2o;
    const float target  = g_therapy_target_cmh2o;
    const int32_t rpm   = (int32_t)BlowerCtrl_GetOutputRpm();

    /* --- 1. Sensor out-of-range --- */
    if ((meas < THERAPY_SENSOR_MIN_CMH2O) || (meas > THERAPY_SENSOR_MAX_CMH2O))
    {
        g_therapy_outrange_ms += dt_ms;
        if (g_therapy_outrange_ms
              >= (uint32_t)(THERAPY_SENSOR_TIMEOUT_S * 1000.0f))
        {
            latchFault(THERAPY_FAULT_SENSOR_OUT_OF_RANGE,
                       "pressure reading outside plausible range");
            return true;
        }
    }
    else
    {
        g_therapy_outrange_ms = 0U;
    }

    /* --- 2. Low pressure at high RPM ---
     * Symptom of a disconnected / unplugged pressure sensor or a
     * disconnected hose. The PID would otherwise wind up to max RPM
     * trying to chase a target it can never measure. */
    if ((rpm > g_therapy_high_rpm_threshold) &&
        (meas < g_therapy_low_press_cmh2o))
    {
        g_therapy_low_press_ms += dt_ms;
        if (g_therapy_low_press_ms
              >= (uint32_t)(g_therapy_low_press_timeout_s * 1000.0f))
        {
            latchFault(THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM,
                       "low pressure while RPM high (sensor/hose lost?)");
            return true;
        }
    }
    else
    {
        g_therapy_low_press_ms = 0U;
    }

    /* --- 3. Over-pressure --- */
    if (meas > (target + g_therapy_overpress_band_cmh2o))
    {
        g_therapy_overpress_ms += dt_ms;
        if (g_therapy_overpress_ms
              >= (uint32_t)(g_therapy_overpress_timeout_s * 1000.0f))
        {
            latchFault(THERAPY_FAULT_OVER_PRESSURE,
                       "measured pressure exceeds target+band");
            return true;
        }
    }
    else
    {
        g_therapy_overpress_ms = 0U;
    }

    /* --- 4. RPM saturated without convergence --- */
    const bool at_safe_max  = (rpm >= g_therapy_safe_rpm_max);
    const bool not_close    = (fabsf(target - meas)
                                > g_therapy_saturate_band_cmh2o);
    if (at_safe_max && not_close)
    {
        g_therapy_saturated_ms += dt_ms;
        if (g_therapy_saturated_ms
              >= (uint32_t)(g_therapy_saturated_timeout_s * 1000.0f))
        {
            latchFault(THERAPY_FAULT_RPM_SATURATED,
                       "RPM saturated without reaching target");
            return true;
        }
    }
    else
    {
        g_therapy_saturated_ms = 0U;
    }

    return false;
}

/* === Public API === */

void Therapy_Init(void)
{
    g_therapy_target_cmh2o    = THERAPY_TARGET_DEFAULT;
    g_therapy_ramp_minutes    = 0U;     /* match GUI Settings default */
    g_therapy_mode            = (uint8_t)THERAPY_MODE_CPAP;
    g_therapy_state           = (uint8_t)THERAPY_STATE_IDLE;
    g_therapy_setpoint_cmh2o  = 0.0f;
    g_therapy_measured_cmh2o  = 0.0f;
    g_therapy_leak_slm        = 0.0f;
    g_therapy_run_ms          = 0U;
    g_therapy_ramp_total_ms   = 0U;
    g_therapy_ramp_elapsed_ms = 0U;
    g_therapy_stop_elapsed_ms = 0U;

    g_therapy_fault           = (uint8_t)THERAPY_FAULT_NONE;
    resetFaultTimers();

    /* Tighten the PID's RPM clamp to the therapy-safe ceiling. The PID
     * still has its own absolute hardware ceiling baked in via
     * BLOWER_IPC_MAX_SPEED_RPM, so this is layered defence: the safety
     * supervisor will trip a FAULT before the PID ever reaches its
     * absolute clamp. */
    extern volatile int32_t g_blower_ctrl_rpm_max;
    g_blower_ctrl_rpm_max = g_therapy_safe_rpm_max;
}

void Therapy_SetTargetPressure(float cmh2o)
{
    g_therapy_target_cmh2o = clampf(cmh2o,
                                    THERAPY_PRESS_MIN_CMH2O,
                                    THERAPY_PRESS_MAX_CMH2O);
}

float Therapy_GetTargetPressure(void) { return g_therapy_target_cmh2o; }

void Therapy_SetRampMinutes(uint8_t minutes)
{
    if (minutes > THERAPY_RAMP_MAX_MINUTES) minutes = THERAPY_RAMP_MAX_MINUTES;
    g_therapy_ramp_minutes = minutes;
}

uint8_t Therapy_GetRampMinutes(void) { return g_therapy_ramp_minutes; }

void Therapy_SetMode(TherapyMode_t m) { g_therapy_mode = (uint8_t)m; }
TherapyMode_t Therapy_GetMode(void)   { return (TherapyMode_t)g_therapy_mode; }

TherapyState_t Therapy_GetState(void) { return (TherapyState_t)g_therapy_state; }

const char* Therapy_GetStateName(void)
{
    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_IDLE:      return "Idle";
    case THERAPY_STATE_RAMP:      return "Ramp";
    case THERAPY_STATE_RUNNING:   return "Running";
    case THERAPY_STATE_STOPPING:  return "Stopping";
    case THERAPY_STATE_FAULT:     return "Fault";
    default:                      return "?";
    }
}

TherapyFault_t Therapy_GetFault(void) { return (TherapyFault_t)g_therapy_fault; }

const char* Therapy_GetFaultName(void)
{
    switch ((TherapyFault_t)g_therapy_fault)
    {
    case THERAPY_FAULT_NONE:                 return "OK";
    case THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM:return "Sensor/Hose Lost";
    case THERAPY_FAULT_OVER_PRESSURE:        return "Over Pressure";
    case THERAPY_FAULT_SENSOR_OUT_OF_RANGE:  return "Sensor Out Of Range";
    case THERAPY_FAULT_RPM_SATURATED:        return "Cannot Reach Target";
    default:                                  return "?";
    }
}

void Therapy_ClearFault(void)
{
    g_therapy_fault = (uint8_t)THERAPY_FAULT_NONE;
    resetFaultTimers();
    if (g_therapy_state == (uint8_t)THERAPY_STATE_FAULT)
    {
        setState(THERAPY_STATE_IDLE);
    }
}

float    Therapy_GetSetpointCmh2o(void) { return g_therapy_setpoint_cmh2o; }
float    Therapy_GetMeasuredCmh2o(void) { return g_therapy_measured_cmh2o; }
float    Therapy_GetLeakSlm(void)       { return g_therapy_leak_slm; }
bool     Therapy_IsLeakHigh(void)       { return Leak_IsHigh(); }
uint32_t Therapy_GetRunSeconds(void)    { return g_therapy_run_ms / 1000U; }

uint32_t Therapy_GetRampRemainingSeconds(void)
{
    if (g_therapy_state != (uint8_t)THERAPY_STATE_RAMP) return 0U;
    if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms) return 0U;
    return (g_therapy_ramp_total_ms - g_therapy_ramp_elapsed_ms) / 1000U;
}

void Therapy_Start(void)
{
    /* A pending FAULT must be cleared before we'll re-arm the blower. */
    if (g_therapy_fault != (uint8_t)THERAPY_FAULT_NONE)
    {
        Therapy_ClearFault();
    }

    /* Re-apply the safety RPM ceiling in case it was poked. */
    extern volatile int32_t g_blower_ctrl_rpm_max;
    g_blower_ctrl_rpm_max = g_therapy_safe_rpm_max;

    /* Snap a self-consistent set of session parameters now, so changes
     * the GUI makes during therapy don't yank the running ramp around. */
    s_session_target_cmh2o   = clampf(g_therapy_target_cmh2o,
                                      THERAPY_PRESS_MIN_CMH2O,
                                      THERAPY_PRESS_MAX_CMH2O);
    g_therapy_ramp_total_ms  = (uint32_t)g_therapy_ramp_minutes * 60U * 1000U;
    g_therapy_ramp_elapsed_ms = 0U;
    g_therapy_stop_elapsed_ms = 0U;
    g_therapy_run_ms         = 0U;
    resetFaultTimers();

    /* Configure the PID with our starting setpoint, then enable. The PID
     * already does bumpless re-enable (snaps slewed setpoint to current
     * measurement, zeros integrator), so the blower won't lurch even if
     * we are starting from a non-zero state. */
    const float start_sp = (g_therapy_ramp_total_ms > 0U)
                              ? THERAPY_RAMP_START_CMH2O
                              : s_session_target_cmh2o;
    BlowerCtrl_SetSetpoint(start_sp);
    BlowerCtrl_SetEnable(true);

    /* Reseed leak estimator so the THERAPY screen shows a meaningful
     * value within ~1 s instead of slewing up over half a minute. */
    Leak_Reseed();

    /* Tell CM4 to spin the motor up. The PID's first SET_SPEED command
     * arrives within 5 ms and supersedes whatever rpm CM4 chose at
     * START. */
    (void)BlowerIpc_CM7_Start();

    DBG_I("THR", "start target=%.1f cmH2O ramp=%u min",
          (double)s_session_target_cmh2o,
          (unsigned)g_therapy_ramp_minutes);

    if (g_therapy_ramp_total_ms > 0U)
    {
        setState(THERAPY_STATE_RAMP);
    }
    else
    {
        setState(THERAPY_STATE_RUNNING);
    }
}

void Therapy_Stop(void)
{
    if (g_therapy_state == (uint8_t)THERAPY_STATE_IDLE)
    {
        return;
    }

    /* Drop PID control immediately so the motor coasts down rather than
     * fighting to maintain pressure during spin-down. */
    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();

    g_therapy_stop_elapsed_ms = 0U;
    /* If we're in FAULT, the supervisor latched the cause and the blower
     * is already off. A user Stop is the operator acknowledging the
     * fault, so go straight to STOPPING (which then transitions to IDLE
     * after the spin-down hold) and clear the fault flag. */
    if (g_therapy_state == (uint8_t)THERAPY_STATE_FAULT)
    {
        Therapy_ClearFault();
    }
    setState(THERAPY_STATE_STOPPING);

    DBG_I("THR", "stop after %lu s", (unsigned long)Therapy_GetRunSeconds());
}

void Therapy_Update(float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.5f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    /* Always refresh observable state (so the GUI can show current
     * values regardless of state). */
    g_therapy_measured_cmh2o = BlowerCtrl_GetMeasured();
    g_therapy_leak_slm       = Leak_GetUnintentionalSlm();
    g_therapy_setpoint_cmh2o = BlowerCtrl_GetSetpointSlewed();

    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_IDLE:
    {
        /* Nothing to do; waiting for Therapy_Start(). */
        break;
    }

    case THERAPY_STATE_RAMP:
    {
        /* Run safety checks first. A latched fault transitions us to
         * THERAPY_STATE_FAULT and disables the PID; subsequent code in
         * this case must not run when that happens. */
        if (runSafetyChecks(dt_ms))
        {
            break;
        }

        /* Linear ramp from THERAPY_RAMP_START_CMH2O to session target
         * over ramp_total_ms. The PID's internal slew limiter (15
         * cmH2O/s) is far faster than this slow ramp (~0.01 cmH2O/s
         * typical) so the per-tick step is, for all practical purposes,
         * a smooth continuous trajectory. */
        g_therapy_ramp_elapsed_ms += dt_ms;
        g_therapy_run_ms          += dt_ms;

        if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms)
        {
            BlowerCtrl_SetSetpoint(s_session_target_cmh2o);
            setState(THERAPY_STATE_RUNNING);
        }
        else
        {
            const float frac = (float)g_therapy_ramp_elapsed_ms
                             / (float)g_therapy_ramp_total_ms;
            const float sp   = THERAPY_RAMP_START_CMH2O
                             + frac * (s_session_target_cmh2o
                                       - THERAPY_RAMP_START_CMH2O);
            BlowerCtrl_SetSetpoint(sp);
        }
        break;
    }

    case THERAPY_STATE_RUNNING:
    {
        if (runSafetyChecks(dt_ms))
        {
            break;
        }

        /* Hold target. The user may have edited the target via the GUI
         * during therapy - re-apply each tick so the PID tracks. */
        const float live_target = clampf(g_therapy_target_cmh2o,
                                         THERAPY_PRESS_MIN_CMH2O,
                                         THERAPY_PRESS_MAX_CMH2O);
        if (fabsf(live_target - s_session_target_cmh2o) > 0.05f)
        {
            s_session_target_cmh2o = live_target;
        }
        BlowerCtrl_SetSetpoint(s_session_target_cmh2o);
        g_therapy_run_ms += dt_ms;
        break;
    }

    case THERAPY_STATE_STOPPING:
    {
        g_therapy_stop_elapsed_ms += dt_ms;
        if (g_therapy_stop_elapsed_ms
              >= (uint32_t)(THERAPY_STOP_DURATION_S * 1000.0f))
        {
            setState(THERAPY_STATE_IDLE);
        }
        break;
    }

    case THERAPY_STATE_FAULT:
    {
        /* No automatic recovery yet. User has to call Stop() then Start()
         * once the offending condition (motor fault, persistent high
         * leak) is cleared. */
        BlowerCtrl_SetEnable(false);
        break;
    }

    default:
        setState(THERAPY_STATE_IDLE);
        break;
    }
}
