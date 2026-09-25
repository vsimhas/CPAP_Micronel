/* Exercises the actual therapy predicates and updateMaskOff timers, not a
 * translated model. All motor I/O below is a host stub: no serial access. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _MSC_VER
#define __attribute__(x)
#endif
#include "../../CM7/Core/Src/therapy.c"

static float input_q, input_ps, input_pm, input_sp, input_rpm;
static bool input_stale, input_clipped, input_limited, pid_enabled;
static uint32_t input_stale_run;
static uint32_t now_ms;
static unsigned park_commands;
static int32_t input_mean_ratio, input_instant_ratio;

volatile float g_safety_grace_s = 0.0f;
volatile int32_t g_safety_safe_rpm_max = 40000;
volatile int32_t g_blower_ctrl_rpm_max = 40000;
uint32_t HAL_GetTick(void) { return now_ms; }
float Flow_GetSlm(void) { return input_q; }
bool Flow_IsSampleStale(void) { return input_stale; }
uint32_t Flow_GetStaleRun(void) { return input_stale_run; }
bool Flow_IsSampleSaturated(void) { return input_clipped; }
float BlowerCtrl_GetMeasured(void) { return input_pm; }
float BlowerCtrl_GetSensorCmh2o(void) { return input_ps; }
float BlowerCtrl_GetSetpointSlewed(void) { return input_sp; }
float BlowerCtrl_GetSetpoint(void) { return input_sp; }
float BlowerCtrl_GetOutputRpm(void) { return input_rpm; }
bool BlowerCtrl_IsUpperLimited(void) { return input_limited; }
bool BlowerCtrl_IsEnabled(void) { return pid_enabled; }
void BlowerCtrl_SetEnable(bool en) { pid_enabled = en; }
float BlowerCtrl_EstimateSteadyStateRpm(float sp) { return 6535.0f * sqrtf(sp) + 853.0f; }
void BlowerCtrl_NoteParkedRpm(float rpm) { input_rpm = rpm; }
void BlowerCtrl_ClearManualFlag(void) {}
void BlowerCtrl_SnapSetpoint(float sp) { input_sp = sp; }
void BlowerCtrl_SetSetpoint(float sp) { input_sp = sp; }
void BlowerCtrl_SetUserSetpointOnly(float sp) { input_sp = sp; }
void BlowerCtrl_ReseedForTherapyResume(float sp) { (void)sp; }
void BlowerCtrl_SetResumeRpmFloor(float rpm, uint32_t ms) { (void)rpm; (void)ms; }
void BlowerCtrl_SuppressPatientFlowFf(uint32_t ms) { (void)ms; }
bool BlowerIpc_CM7_GetStatus(BlowerIpcStatus_t *st)
{
    memset(st, 0, sizeof(*st));
    st->motor_running = true;
    st->mech_speed_rpm = (int32_t)input_rpm;
    return true;
}
bool BlowerIpc_CM7_SetSpeedRpm(int32_t rpm, uint16_t ms)
{
    (void)ms;
    if (rpm == 6000) { park_commands++; }
    return true;
}
float Leak_GetVentCoefficient(void) { return 8.7f; }
bool Leak_IsVivoEnabled(void) { return true; }
int32_t Leak_GetMeanRatio10Sec(void) { return input_mean_ratio; }
int32_t Leak_GetInstantRatio(void) { return input_instant_ratio; }
uint8_t Leak_GetBreathInspiration(void) { return 1U; }
void Leak_SetDisconnectFreeze(bool freeze) { (void)freeze; }
void Leak_Reseed(void) {}
void Epr_Init(void) {}
void Epr_ResetBreathState(void) {}
float Epr_ModifySetpoint(float sp, float dt, uint8_t state)
{ (void)dt; (void)state; return sp; }
void Apap_Init(void) {}
void Apap_OnMaskReconnect(void) {}
float Apap_GetTargetPressure(void) { return input_sp; }

/* Link-only stubs: therapy.c references these outside the mask-off paths
 * under test. Neutral values; none of them feeds the predicates checked here
 * (maskOffEnterThresholdSlm uses Flow_GetAltitudeFactor, hence 1.0). */
float Leak_GetUnintentionalSlm(void) { return 0.0f; }
bool Leak_IsHigh(void) { return false; }
bool Leak_IsLow(void) { return false; }
LeakSeverity_t Leak_GetSeverity(void) { return (LeakSeverity_t)0; }
const char *Leak_GetSeverityName(void) { return ""; }
float Leak_GetPercentOfVent(void) { return 0.0f; }
void Leak_SessionReset(void) {}
void Leak_SetSessionActive(bool active) { (void)active; }
void BlowerCtrl_ManualStop(void) {}
bool BlowerCtrl_IsManualActive(void) { return false; }
void BlowerCtrl_SyncCpapProfilesForMode(uint8_t mode) { (void)mode; }
bool BlowerIpc_CM7_Start(void) { return true; }
bool BlowerIpc_CM7_Stop(void) { return true; }
bool BlowerIpc_CM7_AcknowledgeFault(void) { return true; }
float Flow_GetPatientSlm(void) { return 0.0f; }
void Flow_AutoZeroArm(void) {}
float Flow_GetAltitudeFactor(void) { return 1.0f; }
void Epr_SetEnable(bool en) { (void)en; }
bool Epr_IsEnabled(void) { return false; }
void Epr_SetType(EprType_t type) { (void)type; }
EprType_t Epr_GetType(void) { return (EprType_t)0; }
void Epr_SetLevel(uint8_t level) { (void)level; }
uint8_t Epr_GetLevel(void) { return 0U; }
void Epr_SetReliefCmh2o(float cmh2o) { (void)cmh2o; }
float Epr_GetReliefCmh2o(void) { return 0.0f; }
void Epr_Update(float q, float p, float sp, float dt)
{ (void)q; (void)p; (void)sp; (void)dt; }
float Epr_GetExpiratorySetpointCmh2o(float sp, uint8_t state)
{ (void)state; return sp; }
void Epr_SetStartupHoldTarget(float sp) { (void)sp; }
void Epr_OnTherapyStart(void) {}
void Apap_OnTherapyStart(float lo, float hi, uint8_t mode)
{ (void)lo; (void)hi; (void)mode; }
void Apap_OnTherapyStop(void) {}
float Apap_Update(float dt, uint8_t state, bool mask_off)
{ (void)dt; (void)state; (void)mask_off; return input_sp; }
const char *Apap_GetStateName(void) { return ""; }
void SessionEdf_Init(void) {}
void SessionEdf_OnTherapyStart(void) {}
void SessionEdf_OnTherapyStop(void) {}
void SessionEdf_Update(float dt) { (void)dt; }
void Safety_Supervisor_Init(void) {}
void Safety_Supervisor_ResetFaultTimers(void) {}

static void reset_case(void)
{
    Therapy_Init();
    now_ms = 60000U;
    s_therapy_start_ms = 0U;
    s_reconnect_grace_until_ms = s_safety_reconnect_grace_until_ms = 0U;
    s_mask_off_verify_ms = 0U;
    s_mask_off_press_only_ms = s_mask_off_rpm_ratio_ms = 0U;
    g_therapy_state = THERAPY_STATE_RUNNING;
    g_therapy_mode = THERAPY_MODE_CPAP;
    g_therapy_mask_off_enabled = 1U;
    g_therapy_mask_off_require_press = 1U;
    g_therapy_mask_off_press_only_enable = 1U;
    g_therapy_mask_off_rpm_ratio_enable = 1U;
    g_therapy_mask_off_ratio_enable = 1U;
    g_therapy_mask_off_enter_count = 0U;
    g_therapy_target_cmh2o = s_session_target_cmh2o = 20.0f;
    input_q = 89.1f;
    input_ps = 17.77f;
    input_pm = 15.85f;
    input_sp = 20.0f;
    input_rpm = 38500.0f;
    input_stale = input_clipped = input_limited = false;
    input_stale_run = 0U;
    g_therapy_mask_off_flow_stale_ticks_max = 6U;
    g_therapy_mask_off_press_sensor_allow_cmh2o = 2.5f;
    pid_enabled = true;
    input_mean_ratio = input_instant_ratio = 108;
    park_commands = 0U;
}

static void ticks(unsigned ms)
{
    assert(ms % 5U == 0U);
    for (unsigned t = 0; t < ms; t += 5U)
    {
        now_ms += 5U;
        (void)updateMaskOff(0.005f, 5U);
    }
}

/* The real flow-read pattern (bench logs 2026-09-24): single held reads,
 * each exactly one tick long, stale on 10-17 % of ticks and more often at
 * high flow. Every `every`-th tick is held (stale, run 1, value unchanged);
 * the others are fresh. The phase carries across calls. */
static unsigned tick_phase;
static void ticks_intermittent(unsigned ms, unsigned every)
{
    assert(ms % 5U == 0U && every >= 2U);
    for (unsigned t = 0; t < ms; t += 5U)
    {
        const bool held = ((++tick_phase % every) == 0U);
        input_stale = held;
        input_stale_run = held ? 1U : 0U;
        now_ms += 5U;
        (void)updateMaskOff(0.005f, 5U);
    }
    input_stale = false;
    input_stale_run = 0U;
}

int main(void)
{
    reset_case();
    input_clipped = input_limited = true;
    ticks(1200); /* reported deep-inspiration duration, longer than fast confirm */
    assert(!Therapy_IsMaskOffActive() && park_commands == 0);
    assert(s_mask_off_fast_enter_ms == 0);
    assert(g_therapy_mask_off_diag_fast_block == 6U);
    puts("PASS recorded 20 cmH2O false-trigger conditions do not fast-park");

    reset_case(); input_clipped = true;
    assert(!maskOffEnterFastCondition());
    assert(g_therapy_mask_off_diag_fast_block == 2U);
    reset_case(); input_limited = true;
    assert(!maskOffEnterFastCondition());
    assert(g_therapy_mask_off_diag_fast_block == 4U);
    g_therapy_mask_off_require_press = 0U;
    assert(!maskOffEnterFastCondition());
    puts("PASS clipping and upper-limit guards independently apply, including gate override");

    reset_case(); input_clipped = input_limited = true;
    input_ps = 0.33f; input_pm = 0.16f;
    ticks(295); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive() && park_commands == 1);
    assert(!g_therapy_mask_off_enter_via_press && !g_therapy_mask_off_enter_via_rpm);
    puts("PASS clipped-flow, ceiling-limited floor-pressure disconnect still enters at 300 ms");

    /* Bench mask pull at 20 cmH2O (2026-09-25): flow clipped, command at
     * the ceiling, mask estimate floored but the device-side sensor still
     * 4.6-4.8 from hose drop. Previously blocked -> rpm-ratio at 2.5 s. The
     * sensor allowance lets it through on the nominal 600 ms confirm; the
     * 300 ms strong-open shortcut still needs the sensor under 4. */
    reset_case(); input_clipped = input_limited = true;
    input_q = 89.3f; input_ps = 4.77f; input_pm = 2.84f;
    ticks(595); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive() && park_commands == 1);
    assert(!g_therapy_mask_off_enter_via_press && !g_therapy_mask_off_enter_via_rpm);
    puts("PASS 20 cmH2O pull (sensor 4.77 / mask 2.84) enters via the fast path at 600 ms");

    /* Boundaries of the open-circuit test at 20: sensor floor 4.0 + 2.5 =
     * 6.5, mask floor 4.0. */
    reset_case(); input_clipped = input_limited = true;
    input_pm = 2.84f;
    input_ps = 6.4f; assert(maskOffEnterFastCondition());
    input_ps = 6.6f; assert(!maskOffEnterFastCondition());
    assert(g_therapy_mask_off_diag_fast_block == 6U);
    input_ps = 3.0f; input_pm = 4.1f; assert(!maskOffEnterFastCondition());
    puts("PASS open-circuit floor: sensor < 6.5 and mask < 4.0, each required");

    /* Allowance 0 restores the single 4.0 floor: the 20 cmH2O pull falls
     * back to the rpm-ratio backstop at 2.5 s, as measured before. */
    reset_case(); input_clipped = input_limited = true;
    g_therapy_mask_off_press_sensor_allow_cmh2o = 0.0f;
    input_q = 89.3f; input_ps = 4.77f; input_pm = 2.84f;
    ticks(2495); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive() && g_therapy_mask_off_enter_via_rpm);
    puts("PASS allowance 0 reproduces the pre-fix 2.5 s rpm-ratio entry");

    /* Low setpoint: the sensor floor is capped at sp - drop (6 -> 4.8), so
     * the allowance never accepts less than a real shortfall. */
    reset_case(); input_sp = 6.0f; input_clipped = true;
    input_pm = 3.5f;
    input_ps = 4.9f; assert(!maskOffEnterFastCondition());
    input_ps = 4.7f; assert(maskOffEnterFastCondition());
    puts("PASS at 6 cmH2O the sensor floor is capped at sp - drop (4.8)");

    reset_case(); input_ps = 0.33f; input_pm = 0.16f;
    /* Above enter but below 1.25 * enter: full confirmation. enter is
     * vent-referenced (default on): 8.7 * sqrt(20) + 30 = 68.9 slm, not the
     * fixed 45 floor, so 50 slm would never arm at all. */
    input_q = 75.0f;
    ticks(595); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive());
    puts("PASS ordinary floor disconnect retains 600 ms confirmation");

    reset_case(); input_rpm = 35000.0f;
    ticks(595); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive());
    puts("PASS fresh unclipped flow with headroom retains relative-pressure fast path");

    reset_case(); input_rpm = 35000.0f;
    ticks(400); assert(s_mask_off_fast_enter_ms == 400U);
    input_clipped = true;
    ticks(5); assert(s_mask_off_fast_enter_ms == 0U);
    assert(!Therapy_IsMaskOffArming());
    input_clipped = false;
    ticks(595); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive());
    puts("PASS rejected evidence resets confirmation and does not leave safety inhibited");

    reset_case(); input_sp = 4.0f; input_ps = input_pm = 3.9f;
    input_clipped = input_limited = true;
    ticks(3000); assert(!Therapy_IsMaskOffActive());
    assert(!maskOffPressureFloored());
    input_ps = 2.5f; input_pm = 2.4f;
    ticks(300); assert(Therapy_IsMaskOffActive());
    puts("PASS 4 cmH2O near-target pressure is not a floor; genuine shortfall remains detectable");

    reset_case(); input_clipped = input_limited = true;
    input_ps = 0.2f; input_pm = 15.0f;
    assert(!maskOffEnterFastCondition());
    input_ps = 17.0f; input_pm = 0.2f;
    assert(!maskOffEnterFastCondition());
    puts("PASS floor exception requires BOTH pressure signals");

    reset_case(); input_stale = true;
    input_ps = input_pm = 0.3f;
    input_stale_run = 0U;   /* stale with no run: nothing fresh since init */
    assert(!maskOffEnterFastCondition());
    assert(g_therapy_mask_off_diag_fast_block & 1U);
    input_stale_run = 100U; /* sensor dead for 500 ms */
    assert(!maskOffEnterFastCondition());
    ticks(1995); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive() && g_therapy_mask_off_enter_via_press);
    puts("PASS dead/never-read flow cannot arm fast; flow-independent pressure backstop still enters at 2 s");

    /* Routine one-tick held reads must not reset the fast confirm. Before
     * the stale-run tolerance, any of these ticks zeroed the timer. */
    reset_case(); tick_phase = 0U; input_clipped = input_limited = true;
    input_ps = 0.33f; input_pm = 0.16f;
    ticks_intermittent(295, 8); assert(!Therapy_IsMaskOffActive());
    ticks_intermittent(5, 8);
    assert(Therapy_IsMaskOffActive() && park_commands == 1);
    puts("PASS floor disconnect with a held read every 8th tick still enters at 300 ms");

    reset_case(); tick_phase = 0U; input_rpm = 35000.0f;
    ticks_intermittent(595, 3); assert(!Therapy_IsMaskOffActive());
    ticks_intermittent(5, 3); assert(Therapy_IsMaskOffActive());
    puts("PASS relative-pressure disconnect with a held read every 3rd tick still enters at 600 ms");

    reset_case(); tick_phase = 0U; input_clipped = input_limited = true;
    ticks_intermittent(1200, 8);
    assert(!Therapy_IsMaskOffActive() && park_commands == 0);
    assert(g_therapy_mask_off_diag_fast_block == 6U);
    puts("PASS recorded false trigger stays blocked with routine held reads");

    /* Boundary: runs up to flow_stale_ticks_max (6 = 30 ms) keep the
     * confirm running on the held value; the 7th consecutive held tick
     * means the flow is genuinely old and resets it. */
    reset_case(); input_rpm = 35000.0f;
    ticks(400); assert(s_mask_off_fast_enter_ms == 400U);
    input_stale = true;
    for (uint32_t r = 1U; r <= 6U; r++) { input_stale_run = r; ticks(5); }
    assert(s_mask_off_fast_enter_ms == 430U);
    input_stale_run = 7U; ticks(5);
    assert(s_mask_off_fast_enter_ms == 0U);
    assert(g_therapy_mask_off_diag_fast_block & 1U);
    input_stale = false; input_stale_run = 0U;
    ticks(595); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive());
    puts("PASS held runs up to flow_stale_ticks_max are tolerated; one tick longer resets the confirm");

    /* 0 restores "any held tick invalidates": the old behaviour, which the
     * intermittent pattern defeats. Documents why the default is not 0. */
    reset_case(); tick_phase = 0U; input_rpm = 35000.0f;
    g_therapy_mask_off_flow_stale_ticks_max = 0U;
    ticks_intermittent(1200, 8);
    assert(!Therapy_IsMaskOffActive());
    puts("PASS with tolerance 0 routine held reads defeat the fast path (why the default is 6)");

    reset_case(); input_q = NAN;
    assert(!maskOffEnterFastCondition());
    input_q = INFINITY;
    assert(!maskOffEnterFastCondition());
    input_q = -INFINITY;
    assert(!maskOffEnterFastCondition());
    puts("PASS nonfinite flow is rejected");

    reset_case(); input_clipped = input_limited = true;
    ticks(2495); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive() && g_therapy_mask_off_enter_via_rpm);
    puts("PASS sustained non-floor shortfall retains 2.5 s RPM backstop (not globally inhibited)");

    reset_case(); input_clipped = input_limited = true;
    g_therapy_mask_off_rpm_ratio_enable = 0U;
    input_mean_ratio = 2500; input_instant_ratio = 25000;
    ticks(7995); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(Therapy_IsMaskOffActive() && g_therapy_mask_off_enter_via_ratio);
    puts("PASS existing slow leak-ratio path is unchanged");

    reset_case(); input_ps = input_pm = 0.3f;
    s_reconnect_grace_until_ms = now_ms + 1000U;
    ticks(995); assert(!Therapy_IsMaskOffActive());
    ticks(5); assert(s_mask_off_fast_enter_ms == 5U);
    puts("PASS reconnect grace still blocks entry");

    reset_case(); g_therapy_mask_off_enabled = 0U;
    input_ps = input_pm = 0.3f;
    ticks(3000); assert(!Therapy_IsMaskOffActive() && park_commands == 0);
    puts("PASS disabled mask-off issues no park commands");

    /* ---- Reconnect side. Bench values at the 6000 rpm hold, 2026-09-25:
     * mask OFF P_sensor ~0.07, |Q| 15.2 slm; mask ON ~0.6 cmH2O, 5.2 slm. */

    /* Mask stays off after a fast-path entry: must stay parked. Before the
     * fix the flow-only hatch read 15.2 slm as "sealed" (peak under the 25
     * slm bar, under clear_slm 40) and cleared at ramp 2.5 + learn 0.6 +
     * confirm 1.5 = 4.6 s - exactly what the bench showed. */
    reset_case(); input_clipped = input_limited = true;
    input_ps = 0.33f; input_pm = 0.16f;
    ticks(300); assert(Therapy_IsMaskOffActive());
    input_clipped = input_limited = false;
    input_ps = 0.07f; input_pm = 0.05f; input_q = 15.2f;
    {
        /* Every tick, not just at the end: without the fix the probation
         * check re-parks ~4 s after the false clear, so a single check at
         * 12 s can land in a re-parked phase and pass. */
        const uint32_t reconnects = g_therapy_mask_on_count;
        for (unsigned t = 0; t < 12000U; t += 5U)
        {
            ticks(5);
            assert(Therapy_IsMaskOffActive());
        }
        assert(g_therapy_mask_on_count == reconnects);
    }
    puts("PASS mask left off at hold (0.07 cmH2O, 15.2 slm) stays parked - no flow-only false clear");

    /* ...then the mask goes back on: the pressure tip (0.6 vs 0.07 ref)
     * clears it on the 1.2 s tip confirm, no flow hatch needed. */
    input_ps = 0.60f; input_pm = 0.58f; input_q = 5.2f;
    ticks(1100); assert(Therapy_IsMaskOffActive());
    ticks(200); assert(!Therapy_IsMaskOffActive());
    puts("PASS real reconnect at hold clears via the pressure tip in ~1.2 s");

    /* The case the hatch exists for: mask back on before the decel ends,
     * so the hold reference is the sealed pressure and no tip can appear.
     * Flow-only must still clear it (ramp 2.5 + learn 0.6 + confirm 1.5). */
    reset_case(); input_clipped = input_limited = true;
    input_ps = 0.33f; input_pm = 0.16f;
    ticks(300); assert(Therapy_IsMaskOffActive());
    input_clipped = input_limited = false;
    input_ps = 0.60f; input_pm = 0.58f; input_q = 5.2f;
    ticks(4000); assert(Therapy_IsMaskOffActive());
    ticks(1500); assert(!Therapy_IsMaskOffActive());
    puts("PASS mask back on during the decel (sealed hold ref) still clears via the flow-only hatch");

    puts("All mask-off host regressions passed; no hardware was accessed.");
    return 0;
}
