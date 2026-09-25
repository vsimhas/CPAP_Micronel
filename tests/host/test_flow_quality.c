#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "flow_sensor.h"
#include "leak_estimator.h"

/* Only leak hooks needed by the real flow module. */
float Leak_GetFlowLeakSlm(void) { return 0.0f; }
float Leak_GetUnintentionalSlm(void) { return 0.0f; }
float Leak_GetVentSlm(void) { return 0.0f; }
float Leak_PatientFiltStep(float q, float dt) { (void)dt; return q; }
void Leak_PatientFiltReset(void) {}
float Leak_GetPatientSlm(void) { return 0.0f; }
void Leak_NudgeUnintentional(float delta_slm) { (void)delta_slm; }
/* Defined in the leak estimator on target; flow_sensor.c only reads it. */
volatile float g_flow_patient_raw_slm = 0.0f;

int main(void)
{
    Flow_Init();
    assert(Flow_IsSampleStale() && !Flow_IsSampleSaturated());
    Flow_Update(5.0f, 0.01f, true);
    assert(!Flow_IsSampleStale() && Flow_IsSampleSaturated());
    float q = Flow_GetSlm();
    Flow_NoteStaleSample();
    assert(Flow_IsSampleStale() && Flow_IsSampleSaturated());
    assert(Flow_GetSlm() == q);
    Flow_Update(5.0f, 0.01f, false);
    assert(!Flow_IsSampleStale() && !Flow_IsSampleSaturated());
    Flow_Update(NAN, 0.01f, false);
    assert(Flow_IsSampleStale() && isfinite(Flow_GetSlm()));
    Flow_Update(INFINITY, 0.01f, false);
    assert(Flow_IsSampleStale() && isfinite(Flow_GetSlm()));
    Flow_Update(5.0f, 0.0f, false);
    assert(Flow_IsSampleStale());
    Flow_Update(5.0f, 0.01f, true);
    assert(!Flow_IsSampleStale() && Flow_IsSampleSaturated());
    Flow_Init();
    assert(Flow_IsSampleStale() && !Flow_IsSampleSaturated());
    puts("PASS real Flow_Update quality propagation, held samples, invalid inputs and reset");

    /* Stale run: 0 with stale set = nothing fresh since init; counts held
     * ticks; a fresh sample clears it. therapy.c tolerates short runs. */
    Flow_Init();
    assert(Flow_IsSampleStale() && Flow_GetStaleRun() == 0U);
    Flow_Update(5.0f, 0.01f, false);
    assert(!Flow_IsSampleStale() && Flow_GetStaleRun() == 0U);
    Flow_NoteStaleSample();
    Flow_NoteStaleSample();
    assert(Flow_IsSampleStale() && Flow_GetStaleRun() == 2U);
    Flow_Update(5.0f, 0.01f, false);
    assert(!Flow_IsSampleStale() && Flow_GetStaleRun() == 0U);
    puts("PASS real stale-run count: zero before first sample, counts held ticks, clears on fresh");
    return 0;
}
