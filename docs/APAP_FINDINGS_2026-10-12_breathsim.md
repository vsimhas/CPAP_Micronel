# APAP findings from the BreathSimulator — 12 October 2026

Bench results from four AutoSet trials, explained against `CM7/Core/Src/apap_ctrl.c`.
Written from the BreathSimulator session.

This is **evidence and analysis, not authorization to change firmware, flash, or commit.**
Every code reference is to `apap_ctrl.c` at repo HEAD `0589158`. The build actually
running on the CPAP during the trials was not independently verified against that HEAD.

> **Status after re-testing commit `5ff3544`** (same day, see
> [Re-test of the fixes in `5ff3544`](#re-test-of-the-fixes-in-5ff3544)):
> findings 1 (apnea timing) and 5 (AHI) are **fixed**.
> Finding 2 (hypopnea duration) works but now **counts a long hypopnea twice**.
> Findings 3 and 4 are **not fixed**, and the 55 % hypopnea got **worse**. All of
> those trace back to one cause: the baseline is zeroed on every NO_BREATH entry.
> Two code changes are proposed below, with line numbers against `5ff3544`.

## Summary

The APAP does what it was built to do on the clear-cut cases. It detected flow
limitation, scored apneas of 30 s and longer as central every time, raised pressure for
flow limitation and hypopnea, and did not raise it for central apneas. Breath-phase
polarity was correct throughout.

Five things need attention. Three of them come straight from the code:

1. **Apneas shorter than about 22 s cannot be scored.** The 12 s "recent breath"
   window and the 10 s minimum apnea time add up, because the episode clock only
   starts once the 12 s window has expired. The clinical minimum is 10 s.
2. **The hypopnea duration counts inspiratory time only.** The 10 s threshold therefore
   needs at least about 20 s of actual hypopnea at I:E 1:1. That likely explains why a
   50 % hypopnea scores only at its very end, and a deeper 55 % one never scores.
3. **The breath baseline only updates on breaths that aren't limited.** Large arousal
   breaths push it up, and normal breaths that then read as "limited" can't pull it back
   down. The result is a prolonged false flow-limitation call.
4. **The reported AHI leaves out central apneas.** It is (OA + Hyp) / h, so it read
   **0.00** in a run with three scored central apneas.
5. **FOT oscillation drives the CPAP supply current to about 1.05–1.12 A** in brief peaks.
   Breathing alone never exceeded 0.78 A. This matters for the 2026-10-05 fuse limits.

## Rig and method

```
CPAP (COM13) -> 22 mm hose -> vent -> external sensor -> collapsible chamber
            -> BreathSimulator (COM15, its blower is the patient's muscle) -> 2 L lung
```

- **CPAP:** AutoSet, range 8–12 cmH₂O, EPR off, ramp 0. The range was held near
  10 cmH₂O because the bench current guard (0.8 A sustained) stayed in place.
- **Chamber:** the squeeze blower was off. At CPAP pressure the tube stays open, so
  every simulated apnea is **central-looking**, with an open airway.
- **Simulator:** closed-loop flow control with pressure-compensated feed-forward,
  12 bpm, I:E 1:1 (Ti 2.5 s), 500 mL normal breaths with a peak of about 20.3 slm.
- **Ground truth:** the simulator marks every event on the breath boundary where it
  actually took effect (`E,` lines). It also reports per breath the delivered volume,
  measured peak flow and measured flattening index (inspiratory mean/peak, the same
  quantity the APAP thresholds at 0.78) (`V,` lines).
- **CPAP side:** `therapy apap events` was polled about every 4 s for counters and live
  flags, `therapy status` likewise for state, target, oscillation and OSA call, and the
  `B` line (`blower set telem_motor 1`) at about 25 Hz for supply current, using
  P = 1.5·(vq·iq + vd·id).

CPAP lines arrive about 2 s late and are sampled every ~4 s, so timings below are good
to about ±4 s.

## What was delivered, and what the APAP scored

| stimulus | delivered (simulator sensor) | APAP response | runs |
|---|---|---|---|
| normal breathing | 500 mL, peak 20.3 slm, flat 0.59 | nothing | all |
| flow limitation (plateau, sev 1.0) | 498 mL, peak 14.2, **flat 0.84** | FL flag ~25 s in, FL counted, target 8.0 → 8.6 | 1 |
| hypopnea 30 % | 350 mL, peak 72 % of normal | nothing | 1 |
| hypopnea 45 % | 274 mL, peak 57 % | FL only | 1 |
| hypopnea 50 % | 250 mL, peak 51 % | FL, then **Hyp scored at the event's end** | 2/2 |
| hypopnea 55 % | 224 mL, peak 43–44 % | **FL only — no Hyp** | **3/3** |
| hypopnea 60 % | 204 mL, peak 39 % | oscillation → **central apnea** | 2/2 |
| apnea 20 s | ~0 | oscillation from ~14 s in; CA scored | **3/4** |
| apnea 30 s / 40 s | ~0 | oscillation from 14.3 / 16.8 s in; CA scored | 2/2 |
| normal breaths after two 1.5× arousal breaths | 500 mL | **false FL**, flag on ~90 s | 1 (0/1 with arousals off) |

No OSA calls were made in any run, as expected with an open airway.

## Findings

### 1. The effective minimum apnea is about 22 s, not 10 s

**Code:**

- In MONITOR, the state only moves to NO_BREATH when `recentBreath()` (line 235) has
  seen no breath for `g_apap_breath_window_s` = **12 s** (line 26). Entering NO_BREATH
  resets `s_episode_ms` to 0 (line 1028).
- NO_BREATH waits `g_apap_no_breath_timeout_s` = **4 s** (lines 25, 1052) before starting
  FOT, so **FOT begins about 16 s after the last breath.**
- In OSCILLATE, an event is scored only on an evaluation tick (every 2 s of FOT), and
  only once `s_episode_ms >= apnea_min_ms` (10 s) (line 1087).
- `s_episode_ms` started at the 12 s mark, so **scoring needs about 12 + 10 = 22 s
  without a breath.**

**Bench:** FOT began 14.2, 14.3 and 16.8 s after apnea onset. The previous inspiration
ended about 2.5 s before the onset marker, which puts FOT at about 16–19 s after the last
breath. A 20 s apnea gives the APAP roughly 23.5–24.5 s with no breath, only 1.5–2.5 s
over the threshold, and was scored in 3 of 4 runs.

**Consequence:** apneas of about 10–20 s are invisible to the APAP: no FOT, no
classification, no score. Most of the 4 s figure quoted for "no breath → FOT" in earlier
notes is really the 12 s window.

**Suggested direction:** measure the apnea episode from the last detected breath rather
than from NO_BREATH entry, or subtract the window from `apnea_min`. It's also worth
checking whether a 12 s window is needed at all: it doubles as the slow-breathing guard.

### 2. Hypopnea duration accumulates inspiratory time only

**Code:**

- `finalizeInspBreath()` is called with `s_insp_ms`, the detected *inspiration* length
  (line 587), and adds it to `s_hyp_limited_ms` (line 535).
- The hypopnea is scored when `s_hyp_limited_ms >= 10 s` and the hypopnea streak is at
  least 2 breaths.
- At I:E 1:1, 10 s of inspiration is at least about 20 s of hypopnea, and more if the
  breath detector finds shorter inspirations on weak breaths.
- The fallback of `3000U` when no time is available suggests the author intended a
  per-breath period.

**Bench:** this fits both results.

- The 50 % hypopnea (8 breaths over 40 s) scored only **at the end of the event**: by
  the last breath, the inspiratory time had only just reached 10 s.
- The deeper 55 % hypopnea **never scored, in 3 of 3 runs.**

The second point rests on an assumption I could not measure: that the weaker 55 % breaths
register shorter detected inspirations. The CPAP doesn't report its detected inspiration
times.

**Suggested direction:** accumulate the breath period (time between breath starts), or
the wall-clock time since the first limited breath.

### 3. The hypopnea/apnea boundary sits right where hypopneas are

- **Code:** NO_BREATH exits only on a `recoveryBreath()` (line 244). That is a breath with
  a peak of at least `g_apap_resume_min_peak_slm` = **10 slm** (line 27), a fixed value,
  not relative to the baseline.
- **Bench:** with normal peaks around 20 slm, the 60 % hypopnea (peaks around 8 slm) was
  routed through NO_BREATH and FOT and **scored as a central apnea**, in 2 of 2 runs.
- **Inference:** that it got there because the breath detector (`Epr_HasRecentBreath`)
  doesn't count breaths that weak. I didn't examine its threshold.
- **Consequence:** whether a deep hypopnea counts as a breath depends on a fixed flow
  threshold, so the outcome changes with patient size. Combined with finding 2, only a
  narrow band of depths (around 50 % here) is ever scored as a hypopnea. Shallower ones
  score as flow limitation; deeper ones as central apnea.
- **Suggested direction:** make the recovery-breath and breath-detection thresholds
  relative to the running peak baseline.

### 4. The baseline can't come down once it has been inflated

**Code:**

- `s_fl_baseline_slm` is updated, with alpha 0.25, only in the **non-limited** branch
  (line 521).
- A large breath that isn't limited, such as an arousal breath, pulls it up.
- Normal breaths that then fall below 70 % of the inflated value are classed as limited,
  so they can't update it, and the baseline stays high.

**Bench:**

- With two 1.5× recovery breaths after an apnea, the normal breaths that followed were
  scored as flow-limited, and the FL flag stayed on for about 90 s.
- The same sequence with arousal breaths off gave no FL call.

That is one run each way: supportive, not conclusive. Real arousal breaths after a real
apnea would trigger this.

**Suggested direction:** reject breaths well above the baseline (more than about 1.4×) from
the update, and/or let the baseline drift slowly downward during a long limited streak.

### 5. The reported AHI leaves out central apneas

- **Code:** `Apap_GetSessionAhi()` returns `(OA + Hyp) / h` (line 956);
  `Apap_GetSessionRdi()` returns `(OA + Hyp + FL) / h` (line 963).
- **Bench:** **AHI 0.00** in a run with **CA 3**. The earlier values, 8.30 and 5.68,
  match one hypopnea per session length.
- **Consequence:** clinical AHI counts all apneas, central included. If an obstructive-only
  index is intended, it should be labelled as one; otherwise CA belongs in the sum.

### 6. FOT oscillation is the CPAP's current peak

- **Bench:** supply current was logged at about 25 Hz across two runs. Peaks above 1 A
  (1.05–1.12 A) happened **only while oscillating**. Ordinary breathing reached at most
  0.78 A, with no sample above 0.8 A in about 27 000 breathing samples.
- The FOT setpoint swing is ±1 cmH₂O at 4 Hz (`g_apap_osc_amplitude_cmh2o`, line 29),
  with a clamp at 1.75.
- The peaks are brief (11 samples above 0.8 A in about 65 s of FOT), and a 0.8 A guard
  sustained for 0.7 s never tripped, even through 29 s of FOT.
- **Relevance:** not a fault. But FOT is the CPAP's most power-hungry state, and the
  2026-10-05 fuse event makes this worth recording against the supply sizing.

## What the simulator can and can't test, as of today

**Can:**

- Flow limitation, with a measured flattening index. Plateau at severity 1.0 gives about
  0.83 at CPAP 10, normal breaths about 0.59. The M shape at 0.9 gives about 0.73, a
  morphology that is flow-limited but below the 0.78 threshold.
- Central apneas, with a true zero-flow apnea.
- Hypopneas at a chosen depth.
- Arousal breaths up to 1.5× (fully delivered).
- Per-breath ground truth for automated scoring.

**Can't:**

- **Obstructive apneas.** The chamber can't close the airway. The earlier chamber tests
  also found that the vent's own oscillating flow (about 2 slm p-p) exceeds the 2.0 slm
  OSA threshold, so even a closed airway might not be called OSA. That is still untested.
- **Snore.** Commanded, but the simulator can't confirm it above about 45 Hz: both flow
  sensors sample at about 100 Hz.

**Test-design notes:**

- Use apneas of at least 30 s if they must score; that's finding 1.
- Expect only around 50 % hypopneas to be scored as hypopneas; that's findings 2–3.
- Turn arousal breaths off unless the test is about them; that's finding 4.

## Suggested next tests

1. **Apnea length 10–25 s in 3 s steps,** to pin down the scoring edge. The prediction
   from the code is about 22 s from the last breath.
2. **Hypopnea depth 50/55 % at I:E 1:2** (shorter inspirations). If finding 2 is right,
   the 50 % hypopnea should stop scoring too.
3. **Arousal gain 1.2× vs 1.5×,** to find where the baseline inflation starts producing
   false FL calls.
4. **Any change to `apap_ctrl.c`** can be checked against the same scripted sequences.
   The simulator runs them unattended and scores them per breath.

## Re-test of the fixes in `5ff3544`

Trial 5 used the same rig, CPAP settings (AutoSet 8–12 cmH₂O) and simulator settings
as before. Before therapy started, the device answered
`apap set fl_baseline_reject 1.4` with an ack. Only the `5ff3544` build has that
setting, and 1.4 is its default, so the probe changes nothing.

**Script:** recovery breaths were OFF for everything except the last apnea.

```
normal 90 | apnea 5 | normal 45 | apnea 10 | normal 60 | apnea 20 | normal 60 |
hyp 50% 40 | normal 60 | hyp 55% 40 | normal 60 | hyp 60% 40 | normal 60 |
[recovery 2 x 1.5 on] apnea 20 | normal 90
```

The hypopneas came out as before: 50 % → peak 10.6 slm (52 %); 55 % → 8.9 slm (44 %);
60 % → 7.9 slm (39 %).

### Scorecard

| finding | before `5ff3544` | after `5ff3544` | verdict |
|---|---|---|---|
| 1. apnea timing | 10–20 s apneas never probed; FOT 14–17 s in | 5 s not scored; **10 s scored** (FOT 7.9 s in); 20 s scored (FOT 10.0 s in) | **fixed** |
| 2. hypopnea duration (50 %) | Hyp only at the event's end | **Hyp 8.1 s in**, but **scored twice** (8 s and 32 s) in one 40 s event | **fixed, but double-counts** |
| 2. hypopnea duration (55 %) | FL only (3/3) | **nothing at all** | **worse** |
| 3. recovery threshold (60 %) | scored as central apnea | no CA, **no Hyp either**: the event is invisible | **partly** |
| 4. arousal baseline | false FL, flag on ~90 s | **false FL still**, 30 s after the apnea, flag on 12 s | **not fixed** |
| 5. AHI includes CA | AHI 0.00 with CA 3 | AHI 23.49 = (OA 0 + CA 3 + Hyp 2) / 0.213 h | **fixed** |

The APAP state log, sampled every ~4 s, shows why 55 % and 60 % failed and 50 % didn't:

| hypopnea | APAP state during the event |
|---|---|
| 50 % | MONITOR throughout |
| 55 % | **NO_BREATH at 6 s in**, then MONITOR |
| 60 % | **OSCILLATE at 7 s in** (one brief FOT burst), then MONITOR |

### R1. The baseline is zeroed on every NO_BREATH entry and re-seeded from the next breath

**This is the single cause behind findings 3 and 4 failing, and behind the 55 % regression.**

**Code (`5ff3544`):**

- On MONITOR → NO_BREATH, the pre-episode baseline is saved to `s_resume_ref_slm`
  (line 1098). Then `resetBreathDetectors()` is called (line 1099), which sets
  `s_fl_baseline_slm = 0` (line 440).
- `exitProbeToMonitor()` (line 420) returns to MONITOR **without restoring it**.
- The next breath therefore re-seeds the baseline directly, with no limit or
  rejection test (line 541: `if (!(s_fl_baseline_slm >= min_peak)) { s_fl_baseline_slm = peak_slm; ... }`).
- `s_resume_ref_slm` is currently used only for the recovery-breath test.

**Consequences on the bench:**

- **After an apnea with arousal breaths:** the first breath is the ~31 slm arousal
  breath, so the baseline restarts at ~31. Normal ~20 slm breaths are then below
  0.7 × 31 and count as limited, which gives the **false FL**.
  - The 1.4× arousal rejection (line 566) can't help: it compares against the
    already inflated baseline.
  - This was predicted from the code before the run.
- **During a weak hypopnea:**
  - The breath detector apparently misses a breath or two as flow drops. That is
    inferred; the CPAP doesn't report its detected breaths.
  - With the window shortened to **8 s** (line 42, finding 1's fix), the gap now
    reaches NO_BREATH. With the old 12 s window it didn't.
  - The detectors reset, and the baseline is re-seeded from the hypopnea's own
    8–9 slm breaths.
  - The remaining hypopnea breaths then look normal against that baseline, so
    **nothing is scored**. Before `5ff3544`, the same 55 % hypopnea scored FL.

**Proposed change:**

- In `exitProbeToMonitor()`, restore the baseline instead of leaving it at zero:

  ```c
  /* Restore the pre-episode baseline instead of letting the first breath
   * after the episode re-seed it. That first breath is the arousal breath
   * after an apnea, or a hypopnea's own weak breath after a brief NO_BREATH
   * visit - both made the baseline wrong (bench re-test of 5ff3544). */
  if (s_resume_ref_slm >= clampf(g_apap_fl_min_peak_slm, 1.0f, 30.0f))
  {
      s_fl_baseline_slm = s_resume_ref_slm;
      g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
  }
  ```

- **Expected effect:**
  - After the apnea, the baseline stays at ~20. The 1.4× rejection then keeps the
    31 slm arousal breaths out, and normal breaths stay above 0.7 × 20, so there is
    no false FL.
  - The 55 % / 60 % hypopnea breaths stay below 0.5 × 20 after the brief NO_BREATH
    visit, so a Hyp is scored about 10 s after MONITOR resumes.
- **Things to decide when implementing:**
  - Whether to cap how old the saved baseline may be. Episodes here were 5–40 s; a
    multi-minute episode might warrant re-learning.
  - Whether a NO_BREATH visit that **never reached FOT** should keep the hypopnea
    progress (`s_hyp_streak`, `s_hyp_limited_ms`) as well as the baseline. Keeping it
    would let a hypopnea that briefly tripped NO_BREATH score without starting over.

### R2. One long hypopnea is counted more than once

**Code (`5ff3544`):**

- A hypopnea is counted at line 591 when the streak and duration are met and
  `!s_hyp_active`. `s_hyp_active` is then held for `g_apap_hyp_hold_s` = 20 s.
- When the hold expires, the hold-expiry block in `updateFlHypDetector()`
  (lines 663–669) clears `s_hyp_active` **and** resets `s_hyp_streak` and
  `s_hyp_limited_ms`. It does this even if the breaths are still limited.
- The still-limited breaths then build up again (about 2 breaths / 10 s now that the
  duration uses the breath period) and **count a second event**.

**Bench:** a single 40 s, 50 % hypopnea scored Hyp at 8.1 s and again at 32.2 s. The
AHI of 23.49 above includes that duplicate. Any hypopnea longer than about
hold + 10 s ≈ 30 s is counted at least twice, which inflates AHI and RDI.

**Proposed change:** don't re-arm a hypopnea until breathing has recovered.

- Keep `s_hyp_active` latched past the hold while breaths remain hypopnea-limited, or
  add a `s_hyp_need_recovery` flag: set it when an event is counted, clear it on a
  non-limited breath (the `else` branch next to line 586), and require it to be
  clear before counting again.
- The hold timer can still end the raise window; it just shouldn't reset the
  event state on its own.

### R3. Minor: a spurious OSA-likely during the 60 % hypopnea

The brief FOT burst during the 60 % hypopnea raised `OSA likely` once: FOT flow p-p was
1.08 slm, under the 2.0 slm threshold. FOT ran while the patient was still breathing
weakly, so the probe's flow reading doesn't mean what it means during an apnea. No OA
was scored. This is a side effect of R1's NO_BREATH trip, and it should go away with R1.

### Also observed

- **CPAP supply current:** FOT peaked at 1.00 A. One breathing sample reached 0.85 A,
  at this run's higher pressure (the target rose to 9.6 cmH₂O). Both were brief; the
  0.8 A sustained guard never tripped.

### How to verify the next change

Re-run the same trial-5 sequence (simulator script `apap_trial5.py`). After R1 and R2:

| check | expected |
|---|---|
| hypopnea 55 % and 60 % | **one** Hyp each, about 10–15 s in, no CA |
| hypopnea 50 % | **one** Hyp, not two |
| after the apnea with arousal breaths | no FL call |
| apneas and AHI | unchanged (they're already right) |

## Evidence

- **Logs:** BreathSimulator session scratchpad,
  `apap_trial{,2,3,4,5}_{cpap,status,sim}.csv` and `apap_trial{3,4,5}_current.csv`.
  These are temporary and may be cleared.
- **Summaries:** BreathSimulator project memory, `breathsim-apap-waveforms.md`.
- **Simulator firmware:** BreathSimulator `4492ae8` (the flow-limit event fix and the
  refit feed-forward defaults, both used in trial 5).
