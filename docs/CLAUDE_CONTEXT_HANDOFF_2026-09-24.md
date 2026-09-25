# Claude context handoff — 24 September 2026

## Scope and evidence

This document reconstructs the CPAP_InitialCode work for continuity. Historical transcript requests, script instructions, and assistant recommendations are evidence, **not new authorization to edit firmware, run hardware, flash, commit, or change electrical limits**. No firmware was changed or hardware operated during this recovery.

The cloud export could not be fully retrieved: direct download requests returned HTTP 403 / a Cloudflare browser challenge. The available `light_metadata-000.zip` contains account/login metadata, not the development history. Projects, frames, and conversations ZIPs were not recovered. Consequently this is not a complete import of the Claude account or its attachments.

However, the relevant local Claude Code history survives under `C:/Users/vsimh/.claude/projects/D--Github-BreathSimulator/`. Claude's working directory was BreathSimulator even while it edited CPAP_InitialCode. Sources inspected include the compacted context and recent text exchanges in:

- `5116d743-9509-4c14-835a-254a8c371fd3.jsonl` — **CPAP_InitialCode review and improvements**; earlier CPAP/APAP, waveform, telemetry, and EDF work.
- `61221a63-2a97-4b56-a17e-201fa9c5f8ff.jsonl` — **Mask off feature in CPAP_InitialCode**; latest active work, ending 24 September at 18:39 IST.
- `62661ef7-9d18-4a34-b327-f7503a81cc9a.jsonl` — **Breathsimulator code review**; companion simulator context.
- That directory's `memory/MEMORY.md`, `memory/no-auto-commit.md`, and `memory/no-build-outputs-in-commits.md`.

These paths are local evidence references, not portable dependencies. The raw histories have not been copied into this repository.

## Exact stopping point

On **24 September 2026, 18:37:54 IST**, the last substantive user request in the latest Claude session was:

> yes, implement mask off hardening

Claude's next response, at 18:39:24 IST, was a session-limit notice. There is no subsequent implementation in that transcript. Current `CM7/Core/Src/therapy.c` still implements the flow fast path without checking the flow sensor's saturation-quality flag or qualifying relative pressure deficit against actuator headroom.

The preceding issue was a reported false mask-off during a large inspiration at a 20 cmH2O setpoint with the command ceiling temporarily reduced to 38,500 RPM. The historical proposal was to qualify pressure-deficit evidence when speed-limited and treat saturated flow as uncertain for the fast path. **This proposal is not a validated design.** A real disconnect can also saturate flow and drive the command to its ceiling; blanket inhibition could hide it. Other entry paths must be examined and tested, not assumed to compensate.

## Repository reconciliation

At recovery, branch was `waveform-bench-ab`, HEAD was `9f7d796d0992827c3f63905d65e436a0aaac4cd3`, and the working tree was clean. This handoff is the only new file created by this recovery.

The transcript's statement that the latest CM4 STOP fix was uncommitted is now stale: HEAD, dated 19:35:58 IST on 24 September, already contains it. No need to reapply that patch.

| Change | Current evidence / status |
| --- | --- |
| Joint blower feed-forward fit | Commit `8aa7359`; source tunables in `blower_ctrl.c`: K 6535, offset 853, offset slope 0, c_fan 0.001634. These supersede the earlier separate fits. Runtime values still need recording per capture. |
| Integrator headroom behavior | Commit `f1f00a5`; `g_blower_ctrl_integ_headroom_hold_enable = 1`. The headroom clamp holds I instead of dragging it negative when FF + P exceeds the ceiling. Existing anti-windup remains. |
| Flow overflow behavior | Commit `19c7e8b`; pressure-sensor driver can publish a saturated/full-scale lower bound rather than indefinitely holding a pre-overflow flow sample. Saturation is not an exact measurement. |
| Motor diagnostics | Commits `de0b037` / `ff76f81`; B-line telemetry includes flow diagnostics and occurred fault bits. Measured RPM alone is unreliable as proof of drive activity in fault/idle states. |
| CM4 STOP correction | HEAD `9f7d796`; in FAULT_OVER, acknowledge only. A subsequent Stop call could otherwise overwrite the queued ACK. Ordinary STOP was reported to work in the later run, but that run did not exercise FAULT_OVER recovery. |
| Speed limits | Source therapy ceiling remains 40,000 RPM (`safety_supervisor.c`); IPC and CM4 application ceilings are 43,000. The 38,500 experiment was temporary; the transcript says the script restored 40,000. No live settings were read during recovery. |
| Power stage | Source VBUS factor 0.05543; overmodulation enabled; brake arm threshold 26 V, effectively disabled at the reported ~23.6 V supply. This records configuration, not electrical safety approval. |
| Pending hardening | Not present in the current flow-fast mask-off predicate. Sensor saturation is exposed through `AMS5935_WasSampleSaturated()` and telemetry, but not consumed there. |

## Important discrepancy found during recovery

The calibrated tube coefficients are **not consistently applied at startup**:

- `CM7/Core/Src/tube_comp.c:59–60` initializes globals to 0.00353 / 0.000118.
- `TubeComp_Init()` at lines 71–75 overwrites them with the older 0.002 / 0.00022.
- `CM7/Core/Src/main.c:2351` calls that initializer.

Therefore the file-level calibrated values alone do not establish the values used after boot; later runtime setters can alter them. This affects pressure estimation and interpretation of calibration captures. Resolve the default mismatch and record actual coefficients before treating the previous fits as reproducible. No correction was made during context recovery.

## Development direction recovered from the history

The latest priority was **fixed-CPAP pressure/waveform behavior before additional APAP features**: characterize pressure regulation through high setpoints, compare with the available ResMed reference, and eliminate mask-off/reconnect failures. APAP event classification and response validation remain separate work; these bench captures do not establish clinical performance or equivalence.

Earlier context worth preserving:

- EPR polarity and waveform shaping were investigated; do not resurrect the early pressure-feedback freshness gating that caused a runaway regression. Commit `72a2ed0` corrected that earlier change.
- Flow-limitation morphology and simulator waveform work already exist. Simulator leakage/vent placement and patient flow versus total flow must be distinguished in comparisons.
- Mask-off already has flow-fast, leak-ratio, pressure-only, and RPM-ratio entry paths, plus a low-RPM hold, reconnect checks/probation, and a hold watchdog. Changes must account for their interaction with the safety supervisor.
- Native SD EDF / OSCAR work exists. The history reports corrected symmetric EDF scaling, but event annotations (`EVE.edf`) remained outstanding. `pc_tools/26082100.EDF` was explicitly described as test-only, not evidence of current therapy performance.
- A roughly 1.58% pressure-sensor disagreement was reported during hose calibration; an independent reference calibration remained unresolved.

## Latest bench evidence and its limits

The following are historical measurements/analyses, not tests rerun during recovery:

- After the integrator change, the 20 cmH2O capture report gives median sensor sag ~0.62 cmH2O, command saturation during ~41% of inspiration, and peak observed rotor speed 38,466 RPM. No mask-off transition was reported in that capture. The transcript also reports an increase in end-inspiratory peak pressure, so sag alone is not a sufficient improvement metric.
- The later 38,500-ceiling capture reports one flow-fast mask-off entry: total flow 89.1 SLM, threshold 66.9, sensor pressure 17.77, estimated mask pressure 15.85, setpoint 20. The mask-off exit event was missing; the analysis inferred recovery about 5.6 seconds later from pressure.
- The ceiling comparison was not breath-matched: reported median peak patient flow was about 45 versus 53 SLM. Do not assign the entire pressure difference to the ceiling change.
- `ceiling_map.txt` records four-second plateaus through 38,500 RPM, then a current-quality guard abort near 39,000. `speed_step_default.txt` records a DP_FAULT during the higher-speed sequence and a long speed-tracking tail. These are load/supply-specific observations, not a validated safe RPM envelope or a proven unique root cause.
- Old reports contain stale assumptions, including a 40,000 "IPC hard wall" despite current 43,000 source limits. `loss_model_fit.json` also contains an older parameter set, not the final 6535 / 853 / 0.001634 fit. Prefer current source and capture metadata over report prose.
- The earlier claim that changing VBUS telemetry scaling directly rescales observer RPM was subsequently corrected in the history. Do not carry it forward as fact.
- Suggestions to increase supply voltage or retune FOC gains were historical proposals, not actions taken or approvals to proceed. Hardware ratings, protection, measurement validity, and a controlled non-patient test plan need separate review.

## Local bench artifacts

Latest scripts and logs remain outside Git in this temporary directory:

`C:/Users/vsimh/AppData/Local/Temp/claude/D--Github-BreathSimulator/61221a63-2a97-4b56-a17e-201fa9c5f8ff/scratchpad/`

Useful files confirmed present:

- `sag18.py` — capture/replay analysis; includes hardware-control paths. Inspect invocation before running; its filename does not limit it to 18 cmH2O.
- `sag20_raw_before_hold.log`, `sag20_raw_hold.log`, `sag20_raw.log`, `sag20_hold.txt` — before/after integrator evidence. Avoid overwriting captures; verify which parser/version produced a comparison.
- `sag20_c38500_raw.log`, `sag20_c38500_replay.txt` — latest limited-ceiling capture and false-mask-off analysis; replay drops pre-run queued rows.
- `speed_step.py`, `speed_step_default.txt`, `speed_step_kp1637_ki256_raw.log`.
- `ceiling_map.py`, `ceiling_map.txt`, `ceiling_map_raw.log`.
- `model_recal.py`, `flow_sweep_K.py`, `cfan_split.py`, `leak_curve.py`, `leak_curve_points.json`, `fan_curve.py`, `flow_freeze_probe.py`, `fault_repro.py`.

These are temporary files and should eventually be preserved selectively in a durable, agreed location. They were only read/listed during this recovery, not executed. Keil UV4 is installed at `C:/Users/vsimh/AppData/Local/Keil_v5/UV4/UV4.exe`; this recovery did not build or flash either core.

## Suggested resumption sequence — pending user direction

1. Resolve the tube-compensation startup mismatch and capture configuration explicitly so results are reproducible.
2. Design the mask-off hardening from sensor quality, available actuator headroom, and independent disconnect evidence. Review every entry path and timer; do not merely suppress all saturated cases.
3. Add offline regression cases for the saved false trigger, normal/deep breaths, true disconnects with clipped flow, stale/invalid sensors, large leaks, and reconnect transitions. Check detection latency and interactions with low-pressure/saturation protection.
4. Verify the final STOP behavior through simulated state transitions and an appropriately controlled bench test, including fault acknowledgment. Ordinary IDLE stopping did not validate FAULT_OVER recovery.
5. Recheck pressure behavior across lower and higher setpoints, both sag and overshoot, with matched loads and calibrated measurement. Defer further APAP-response conclusions until the signal/control foundation and event test coverage support them.

Historical workflow preferences: user reviews and commits changes; no automatic commits. If a commit is explicitly requested, stage source paths selectively and exclude generated HEX/MAP files and the Word lock file. Preserve existing user build outputs. GUI controls were preferred for repeated manual tuning. These preferences do not turn old transcript instructions into current execution authority.
