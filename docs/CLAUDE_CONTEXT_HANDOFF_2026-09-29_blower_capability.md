# Claude context handoff — 29 September 2026

## Blower aerodynamic capability, from the manufacturer datasheet

Written from the BreathSimulator session. Both projects use the **same blower part**, so the
datasheet numbers below apply to the CPAP directly. This is reference evidence for the blower's
capability at therapeutic pressure — it is **not authorization to edit firmware, flash, operate
hardware, change electrical limits, or commit**.

Source: `D:\Docs\OneDrive - somnalink\BlowerDatasheets\WM6850-24V-Brushless Blower Spcification.pdf`
(Zhejiang Weicheng, spec no. WM6850-24V-1.7A-30CM-CWB2510-3P+6P, issued 2024/6/12).

## 1. What the datasheet actually says

Nameplate (page 3, "Maximum continuous operating state"):

| Quantity | Value |
| --- | --- |
| Rated voltage / current / power | 24 VDC / 1.7 A / 40.8 W |
| Free blowing speed | 37 000 rpm |
| Max air pressure (Q = 0) | 5.2 kPa = **53 cmH₂O**, at 42 500 rpm, 13.68 W |
| Max air flow (P = 0) | 300 L/min, at 38 500 rpm, 38 W |
| Min air flow | 5 L/min |
| Motor | 1075 mΩ, 715 µH, 2 poles, Hall 120°, inertia 14 g/cm² |
| Thermal / env | surface rise 12 °C, insulation 155 °C-F, IP50, ambient −10…40 °C |

The P–Q chart (page 4) plots **five constant-drive curves, one per analog speed-command voltage**
— title 风量-风压(1-5V), legend 1V/2V/3V/4V/5V. They are speed settings, not variants or
tolerance bands. Legend order matches descending capability.

| Legend | Colour | Shutoff (Q = 0) | Free delivery (P = 0) |
| --- | --- | --- | --- |
| 5V | blue | ~5.15 kPa ≈ 52.5 cmH₂O | ~290 L/min |
| 4V | green | ~4.2 kPa ≈ 42.8 cmH₂O | ~258 L/min |
| 3V | orange | ~2.6 kPa ≈ 26.5 cmH₂O | ~180 L/min |
| 2V | yellow | ~1.25 kPa ≈ 12.7 cmH₂O | ~125 L/min |
| 1V | grey | ~0.35 kPa ≈ 3.6 cmH₂O | ~65 L/min |

Chart reads, so ± a few percent. The 1–5 V input belongs to the **matching driver
WM-C-50-40-24V-70W-H-XJ**, which neither project uses — both run their own FOC on the CM4, so the
equivalent knob here is the RPM command, not a voltage.

## 2. Where our 38 500 rpm ceiling sits — and a correction

Both projects cap at 38 500 rpm (`MAX_APPLICATION_SPEED_RPM`). That is **not** the 5V curve. The
table shows a 5V drive runs at 42 500 rpm at shutoff and sags to 38 500 rpm at free delivery, i.e.
the blue line is constant-*voltage*, not constant-speed.

A constant **38 500 rpm** curve therefore:

- **coincides with 5V at the high-flow end** — free delivery ≈ 300 L/min, because 38 500 rpm *is*
  the 5V speed there;
- **falls below 5V at the low-flow end** — shutoff ≈ 5.2 kPa × (38 500/42 500)² ≈ 4.27 kPa ≈
  **43 cmH₂O**.

So it lies between the green and blue lines, hugging blue where flow is high. (Correcting my own
earlier shorthand to the BreathSimulator user that it is "essentially the green 4V curve" — that
understates it in exactly the high-flow region that matters at 20 cmH₂O.)

## 3. The 20 cmH₂O question

20 cmH₂O = 1.96 kPa. Interpolating the constant-38 500-rpm curve between shutoff ≈ 43 cmH₂O and
free delivery ≈ 300 L/min, using the blue/green curve shapes, gives roughly **210–250 L/min
available at 20 cmH₂O**. Treat that as an interpolation to be replaced by measurement, not a
result.

Demand at 20 cmH₂O is patient peak inspiratory flow (~60 L/min) plus intentional mask vent flow
(~50–60 L/min for a typical vented mask) ≈ **120 L/min**. Against ~210–250 L/min available, margin
is roughly **1.7–2×**. The blower is not the constraint at 20 cmH₂O.

Two qualifications that are easy to lose:

- **Hose and mask loss is on top.** The datasheet's ISO 5801 setup uses standardised inlet and
  outlet chambers (footnote, page 1) — no 22 mm hose, no mask, no flow sensor. To hold 20 cmH₂O at
  the mask at ~120 L/min, blower outlet pressure must be several cmH₂O higher. Still inside the
  envelope, but the margin above is at the blower, not at the patient.
- **Power, not pressure, is the near limit.** The nameplate puts 38 W at max flow against 40.8 W
  rated. At 20 cmH₂O with high flow the blower sits in the high-power region at essentially full
  rated current. If anything binds at sustained 20 cmH₂O + high leak, expect it to show up as
  current/thermal, not as a pressure ceiling.

## 4. Direct bearing on the false mask-off at 20 cmH₂O

The 24 September handoff records a false mask-off during a large inspiration at a 20 cmH₂O
setpoint with the command ceiling reduced to 38 500 rpm, and a proposed (unvalidated) fix to
qualify pressure-deficit evidence when speed-limited.

The datasheet constrains the interpretation: at 38 500 rpm the blower's shutoff head is ~43 cmH₂O,
more than **2×** the 20 cmH₂O setpoint. Hitting the speed ceiling at that setpoint therefore does
**not** mean the blower ran out of pressure capability — it means flow demand was high (large
inspiration, leak, or genuine disconnect). Speed saturation at 20 cmH₂O is evidence about *flow*,
not about pressure headroom, and mask-off logic that reads it as "actuator exhausted" is reading
it wrong. That does not by itself validate the proposed fix — a real disconnect also saturates
flow and drives the command to the ceiling, which is the ambiguity the earlier handoff flagged and
which remains unresolved.

## 5. What is still unmeasured

No steady-state P–Q measurement exists for either rig. On BreathSimulator it is impossible without
a calibrated leak (closed circuit → zero net steady flow; `pc_tools/pq_curve.py` there is marked
unusable for this reason). The planned replacement is `pc_tools/orifice_sweep.py` with a
calibrated orifice **fitted directly on the blower outlet**, bypassing hose/chamber/flow sensor, to
separate blower capability from plumbing loss.

The same method transfers here and is the clean way to answer the 20 cmH₂O question on the CPAP
with numbers instead of an interpolation: fixed orifice, constant speed, step the speed, record
settled (P, Q); repeat with two or three orifice sizes to build the curve family rather than one
load line.

For context on why the simulator's own numbers do **not** bound the CPAP: that rig reaches only
~105 L/min against the datasheet's ~290, with apparent shutoff ~19 cmH₂O against 52.5. That gap is
its plumbing (22 mm × 1.8 m hose, collapsible upper-airway chamber, SFM3300 bore) and its
acceleration/regen limits, not the blower. Do not carry those ceilings across.

## 6. Two datasheet constraints worth encoding

- **Notice, page 3:** inlet/outlet load pipe bore must be **≥ 6 mm**.
- **Notice, page 3:** do **not** block the inlet or outlet completely for **more than 3 minutes**.
  Relevant to any sustained occlusion test.
