/* Tube compensation start-up values. TubeComp_Init() used to reset the
 * coefficients fitted on 2026-09-22 (0.00353 / 0.000118) to the older
 * 0.002 / 0.00022, so the calibration never applied after a boot. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "tube_comp.h"

static int near(float a, float b) { return fabsf(a - b) < 1e-7f; }

int main(void)
{
    /* Before and after Init, and after a runtime change + re-Init. */
    assert(near(TubeComp_GetRLam(), 0.00353f));
    assert(near(TubeComp_GetRTurb(), 0.000118f));
    TubeComp_Init();
    assert(near(TubeComp_GetRLam(), 0.00353f));
    assert(near(TubeComp_GetRTurb(), 0.000118f));
    TubeComp_SetCoefficients(0.002f, 0.00022f);
    TubeComp_Init();
    assert(near(TubeComp_GetRLam(), 0.00353f));
    assert(near(TubeComp_GetRTurb(), 0.000118f));
    puts("PASS TubeComp_Init keeps the calibrated 0.00353 / 0.000118");

    /* The drop it applies: 55 LPM -> 0.552 cmH2O (the fit table), and the
     * sign follows flow direction. */
    const float pm_in  = TubeComp_Apply(20.0f, 55.0f);
    const float pm_out = TubeComp_Apply(20.0f, -55.0f);
    assert(fabsf((20.0f - pm_in) - 0.5511f) < 0.001f);
    assert(fabsf((pm_out - 20.0f) - 0.5511f) < 0.001f);
    puts("PASS drop at 55 LPM matches the fitted table (0.552), signed by flow");
    return 0;
}
