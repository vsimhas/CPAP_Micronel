#!/usr/bin/env python3
"""
Bench re-calibration of the sqrt feed-forward model, with the noise-triggered
FF terms taken out of the way first.

Why this exists
---------------
The steady-state model in blower_ctrl.c is

    rpm = ff_k * sqrt(P_cmH2O) + ff_offset

with ff_k = 6141 and ff_offset = 2000 by default. Those were fitted around
8 cmH2O. Measured on the bench at 16 cmH2O the circuit actually needed about
29,600 rpm while the model predicts 26,564 - roughly 3,000 rpm low. The
integrator has to carry that gap, so a quarter of its budget is spent on
steady-state trim instead of rejecting breath disturbances, and at high
pressure the loop runs out of headroom and clips at the rpm ceiling.

This script measures the real curve and refits both coefficients.

It also applies step 1 of the bench plan first: raising the dip and rise FF
dP/dt thresholds so they stop firing on pressure noise. Measured at 16 cmH2O
with NO flow, dip FF was reaching its full 1500 rpm cap and rise FF -995 rpm,
purely on noise - which both wastes headroom and corrupts any measurement
taken while it is happening.

What it does
------------
  1. Raises dip_ff_thresh / rise_ff_thresh (skip with --no-thresholds).
  2. Steps CPAP through a list of pressures, holding each one.
  3. At each pressure, after a settle time, averages steady-state mech rpm
     and delivered pressure over a dwell window.
  4. Refits ff_k and ff_offset by least squares and prints the commands to
     apply them.

Points where the loop CLIPPED at the rpm ceiling are reported and excluded
from the fit - a clipped point tells you nothing about what the circuit
needs, only that the controller ran out of room.

Usage
-----
    python blower_recal.py --list
    python blower_recal.py --port COM13
    python blower_recal.py --port COM13 --pressures 6,8,10,12,14,16,18,20
    python blower_recal.py --port COM13 --dry-run          # print, send nothing
    python blower_recal.py --port COM13 --plot --csv recal.csv

IMPORTANT: this drives the blower through real therapy pressures. Run it on
the bench with the circuit sealed and nobody attached, and do not breathe into
it - the fit needs a static circuit. Therapy is always stopped on exit,
including on Ctrl-C.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required\n"
        "  pip install -r pc_tools/requirements.txt"
    ) from exc


# Firmware defaults, for the "before" column of the report.
DEFAULT_FF_K = 6141.0
DEFAULT_FF_OFFSET = 2000.0
DEFAULT_RPM_CAP = 36000   # g_safety_safe_rpm_max; was 30000 before 2026-09-22

# Plausibility bounds for one T line. The CDC stream drops bytes often enough
# that ~3% of lines arrive with merged fields; without these a single mangled
# row (mech_rpm = 2.9e11 was observed) destroys every average.
T_LIMITS = (
    (-20.0, 40.0),      # P_sensor cmH2O
    (-200.0, 200.0),    # Q_patient slm
    (-1.0, 40000.0),    # pid rpm command
    (-1.0, 40000.0),    # mech rpm
)


@dataclass
class Point:
    """One settled pressure step."""

    setpoint: float
    p_mean: float
    p_sd: float
    rpm_mech: float
    rpm_cmd: float
    q_mean: float
    q_sd: float
    n: int
    clipped_frac: float
    excluded: str = ""


@dataclass
class Link:
    """Line-oriented serial link to the device."""

    port: str
    baud: int = 115200
    dry_run: bool = False
    debug: bool = False
    _ser: "serial.Serial | None" = field(default=None, init=False, repr=False)
    _buf: bytes = field(default=b"", init=False, repr=False)

    def open(self) -> None:
        if self.dry_run:
            print(f"[dry-run] would open {self.port} @ {self.baud}")
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.2)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def send(self, command: str) -> None:
        if self.dry_run:
            print(f"[dry-run] > {command}")
            return
        assert self._ser is not None
        self._ser.write((command + "\r\n").encode("ascii", errors="ignore"))
        self._ser.flush()
        if self.debug:
            print(f"> {command}")
        # The firmware parses one command per line from a small buffer; a short
        # gap keeps a burst of settings from overrunning it.
        time.sleep(0.08)

    def read_lines(self, duration_s: float):
        """Yield decoded lines for duration_s seconds."""
        if self.dry_run:
            return
        assert self._ser is not None
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            chunk = self._ser.read(4096)
            if chunk:
                self._buf += chunk
                while b"\n" in self._buf:
                    raw, self._buf = self._buf.split(b"\n", 1)
                    yield raw.decode("utf-8", errors="replace").strip()
            else:
                time.sleep(0.005)


def parse_t_line(line: str):
    """Return (P_sensor, Q_patient, rpm_cmd, rpm_mech) or None."""
    if not line.startswith("T,"):
        return None
    f = line.split(",")
    if len(f) < 6:
        return None
    try:
        vals = (float(f[2]), float(f[3]), float(f[4]), float(f[5]))
    except ValueError:
        return None
    for v, (lo, hi) in zip(vals, T_LIMITS):
        if not (lo <= v <= hi):
            return None
    return vals


def collect(link: Link, seconds: float, rpm_cap: int):
    """Read T lines for `seconds` and return the parsed samples."""
    samples = []
    rejected = 0
    for line in link.read_lines(seconds):
        if not line.startswith("T,"):
            continue
        parsed = parse_t_line(line)
        if parsed is None:
            rejected += 1
        else:
            samples.append(parsed)
    return samples, rejected


def measure_point(link: Link, setpoint: float, args) -> Point:
    link.send(f"therapy set pressure {setpoint:.1f}")

    print(f"  {setpoint:5.1f} cmH2O : settling {args.settle:.0f}s ...", end="", flush=True)
    for _ in link.read_lines(args.settle):
        pass
    print(f" averaging {args.dwell:.0f}s ...", end="", flush=True)

    samples, rejected = collect(link, args.dwell, args.rpm_cap)
    if args.dry_run:
        print(" [dry-run]")
        return Point(setpoint, setpoint, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, "dry-run")
    if len(samples) < 20:
        print(f" only {len(samples)} samples")
        return Point(setpoint, 0, 0, 0, 0, 0, 0, len(samples), 0.0, "too few samples")

    press = [s[0] for s in samples]
    flow = [s[1] for s in samples]
    cmd = [s[2] for s in samples]
    mech = [s[3] for s in samples]
    clipped = sum(1 for c in cmd if c >= args.rpm_cap - 10) / len(cmd)

    pt = Point(
        setpoint=setpoint,
        p_mean=statistics.fmean(press),
        p_sd=statistics.pstdev(press),
        rpm_mech=statistics.fmean(mech),
        rpm_cmd=statistics.fmean(cmd),
        q_mean=statistics.fmean(flow),
        q_sd=statistics.pstdev(flow),
        n=len(samples),
        clipped_frac=clipped,
    )

    # A point is only usable if the loop had room to find its own answer and
    # the circuit was actually static while we measured it.
    if clipped > 0.02:
        pt.excluded = f"clipped {clipped*100:.0f}% at rpm cap"
    elif pt.q_sd > args.quiet_sd:
        pt.excluded = f"circuit not static (Q sd {pt.q_sd:.1f} slm)"
    elif abs(pt.p_mean - setpoint) > args.track_tol:
        pt.excluded = f"did not reach setpoint ({pt.p_mean:.2f})"

    note = f"  P={pt.p_mean:6.2f}  mech={pt.rpm_mech:7.0f}"
    if rejected:
        note += f"  ({rejected} corrupt lines)"
    if pt.excluded:
        note += f"  EXCLUDED: {pt.excluded}"
    print(note)
    return pt


def fit_sqrt_model(points):
    """Least-squares fit of rpm = k*sqrt(P) + offset. Returns (k, offset, r2)."""
    usable = [p for p in points if not p.excluded]
    if len(usable) < 2:
        return None

    xs = [math.sqrt(p.p_mean) for p in usable]
    ys = [p.rpm_mech for p in usable]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    k = sxy / sxx
    offset = my - k * mx

    ss_res = sum((y - (k * x + offset)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return k, offset, r2


def report(points, fit, args) -> None:
    print("\n" + "=" * 78)
    print("MEASURED STEADY STATE")
    print("=" * 78)
    print(f"{'set':>6} {'P meas':>8} {'P sd':>6} {'mech rpm':>9} {'cmd rpm':>8} "
          f"{'Q':>7} {'clip%':>6}  note")
    for p in points:
        print(f"{p.setpoint:6.1f} {p.p_mean:8.2f} {p.p_sd:6.3f} {p.rpm_mech:9.0f} "
              f"{p.rpm_cmd:8.0f} {p.q_mean:7.1f} {p.clipped_frac*100:6.1f}  {p.excluded}")

    odd_flow = [p for p in points if p.p_mean > 0 and p.q_mean < -args.flow_warn]
    if odd_flow:
        print()
        print("  NOTE: strongly negative patient flow with nobody attached:")
        for p in odd_flow:
            print(f"    {p.setpoint:5.1f} cmH2O -> Q_patient {p.q_mean:+.1f} slm")
        print("  Patient flow is total minus the vent and leak models, so with no")
        print("  patient it should sit near zero. A large negative value means those")
        print("  models are over-subtracting for this circuit. It does not affect the")
        print("  rpm fit below, but it does mean the leak/vent calibration is off.")

    print("\n" + "=" * 78)
    print("MODEL")
    print("=" * 78)
    print(f"  current : rpm = {args.ff_k:.0f} * sqrt(P) + {args.ff_offset:.0f}")
    if fit is None:
        print("\n  Not enough usable points to fit. Every point was excluded -")
        print("  see the notes above. If they clipped, you are out of rpm headroom")
        print("  at these pressures and no re-fit will change that; reduce the")
        print("  pressure range or raise the rpm ceiling first.")
        return

    k, offset, r2 = fit
    usable = [p for p in points if not p.excluded]
    print(f"  fitted  : rpm = {k:.0f} * sqrt(P) + {offset:.0f}    (R2 = {r2:.4f})")

    weak = len(usable) < args.min_points
    if weak:
        print()
        print("  " + "!" * 70)
        print(f"  ONLY {len(usable)} USABLE POINTS - this fit is not trustworthy.")
        print("  Two points define a line exactly, so R2 = 1.0000 means nothing here:")
        print("  there is no residual left to tell you whether the model is right.")
        print("  Worse, k and offset trade off perfectly against each other with two")
        print("  points, so you cannot tell a slope change from an offset change -")
        print("  which is exactly the distinction that matters (k scales every breath")
        print("  excursion; offset does not).")
        print("  Re-run with more pressures and fix whatever excluded the others.")
        print("  " + "!" * 70)

    print("\n  prediction error of each model, in rpm:")
    print(f"    {'set':>6} {'measured':>9} {'current':>9} {'err':>7} {'fitted':>9} {'err':>7}")
    for p in points:
        if p.excluded:
            continue
        cur = args.ff_k * math.sqrt(p.p_mean) + args.ff_offset
        new = k * math.sqrt(p.p_mean) + offset
        print(f"    {p.setpoint:6.1f} {p.rpm_mech:9.0f} {cur:9.0f} {cur-p.rpm_mech:+7.0f} "
              f"{new:9.0f} {new-p.rpm_mech:+7.0f}")

    print("\n  To apply:" if not weak else "\n  Would apply (NOT recommended yet):")
    print(f"    blower set ff_k {k:.0f}")
    print(f"    blower set ff_offset {offset:.0f}")
    print()
    print("  " + "-" * 70)
    print("  READ THIS BEFORE APPLYING THE OFFSET.")
    print("  This fit is of TOTAL commanded rpm against sqrt(P), so it suits")
    print("  steadyStateRpmForSetpoint(), which has no flow term. The physics")
    print("  feed-forward ADDS K*sqrt(hose + c_fan*Q^2) on top, and the sweep")
    print("  above already contained the circuit's vent flow, rising with")
    print("  pressure. Pasting this offset into ff_offset double-counts it.")
    print("  Measured on 2026-09-21: the offset this fit produced was WORSE at")
    print("  8 cmH2O than the value it replaced.")
    print()
    print("  For ff_offset, calibrate from the integrator instead: hold a")
    print("  pressure, wait two minutes, read g_blower_ctrl_i_term, and the")
    print("  offset that zeroes it is (offset in force + I). Do that at two")
    print("  pressures and take the midpoint.")
    print("  " + "-" * 70)
    print("\n  Then re-run this script to confirm the residuals are small, and")
    print("  re-check the integrator: freeing the steady-state gap should drop")
    print("  the mean i_term substantially (watch g_blower_ctrl_i_term).")

    if k > DEFAULT_FF_K * 1.25 or k < DEFAULT_FF_K * 0.75:
        print(f"\n  NOTE: fitted k ({k:.0f}) is far from the bench value "
              f"({DEFAULT_FF_K:.0f}).")
        print("  k is multiplicative - it scales every breath excursion and every")
        print("  k-derived rpm floor too. If the curve is really this different,")
        print("  prefer to carry the gap in the offset and re-check the circuit.")


def write_csv(points, path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        fh.write("setpoint_cmh2o,p_mean,p_sd,rpm_mech,rpm_cmd,q_mean,q_sd,"
                 "samples,clipped_frac,excluded\n")
        for p in points:
            fh.write(f"{p.setpoint},{p.p_mean:.3f},{p.p_sd:.4f},{p.rpm_mech:.0f},"
                     f"{p.rpm_cmd:.0f},{p.q_mean:.2f},{p.q_sd:.2f},{p.n},"
                     f"{p.clipped_frac:.4f},\"{p.excluded}\"\n")
    print(f"\n  wrote {path}")


def make_plot(points, fit, args) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib not installed - skipping plot)")
        return

    usable = [p for p in points if not p.excluded]
    excluded = [p for p in points if p.excluded]

    fig, ax = plt.subplots(figsize=(7, 5))
    if usable:
        ax.plot([math.sqrt(p.p_mean) for p in usable],
                [p.rpm_mech for p in usable], "o", label="measured (used)")
    if excluded:
        ax.plot([math.sqrt(p.p_mean) for p in excluded if p.p_mean > 0],
                [p.rpm_mech for p in excluded if p.p_mean > 0],
                "x", color="0.6", label="excluded")

    all_p = [p.p_mean for p in points if p.p_mean > 0]
    if all_p:
        lo, hi = min(all_p), max(all_p)
        xs = [math.sqrt(lo + (hi - lo) * i / 50.0) for i in range(51)]
        ax.plot(xs, [args.ff_k * x + args.ff_offset for x in xs],
                "--", label=f"current ({args.ff_k:.0f}, {args.ff_offset:.0f})")
        if fit:
            k, offset, _ = fit
            ax.plot(xs, [k * x + offset for x in xs],
                    "-", label=f"fitted ({k:.0f}, {offset:.0f})")

    ax.axhline(args.rpm_cap, color="r", ls=":", label=f"rpm cap {args.rpm_cap}")
    ax.set_xlabel("sqrt(pressure)  [sqrt(cmH2O)]")
    ax.set_ylabel("steady-state mech RPM")
    ax.set_title("Blower sqrt-model re-calibration")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Re-fit the blower sqrt feed-forward model on the bench.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage\n-----")[-1],
    )
    ap.add_argument("--port", help="Serial port, e.g. COM13")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="List serial ports and exit")
    ap.add_argument("--pressures", default="8,12,16,20",
                    help="Comma-separated setpoints (default 8,12,16,20)")
    ap.add_argument("--settle", type=float, default=15.0,
                    help="Seconds to discard after each pressure change (default 15)")
    ap.add_argument("--dwell", type=float, default=15.0,
                    help="Seconds to average at each pressure (default 15)")
    ap.add_argument("--rpm-cap", type=int, default=DEFAULT_RPM_CAP, dest="rpm_cap",
                    help=f"RPM ceiling used to detect clipping (default {DEFAULT_RPM_CAP})")
    ap.add_argument("--ff-k", type=float, default=DEFAULT_FF_K, dest="ff_k",
                    help="Current ff_k, for the comparison table")
    ap.add_argument("--ff-offset", type=float, default=DEFAULT_FF_OFFSET, dest="ff_offset",
                    help="Current ff_offset, for the comparison table")
    ap.add_argument("--no-thresholds", action="store_false", dest="thresholds",
                    help="Do not raise the dip/rise FF dP/dt thresholds first")
    ap.add_argument("--dip-thresh", type=float, default=5.0)
    ap.add_argument("--rise-thresh", type=float, default=5.0)
    ap.add_argument("--quiet-sd", type=float, default=3.0,
                    help="Max Q_patient sd (slm) for a point to count as static")
    ap.add_argument("--track-tol", type=float, default=1.0,
                    help="Max |P_measured - setpoint| for a point to be usable")
    ap.add_argument("--min-points", type=int, default=3,
                    help="Warn loudly below this many usable points (default 3)")
    ap.add_argument("--flow-warn", type=float, default=5.0,
                    help="Flag points whose mean Q_patient is below -this (slm)")
    ap.add_argument("--csv", type=Path, help="Write the measured points to CSV")
    ap.add_argument("--plot", action="store_true", help="Show the fit plot")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the commands without opening the port")
    ap.add_argument("--debug", action="store_true", help="Echo every command sent")
    ap.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    args = ap.parse_args()

    if args.list:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device}  {p.description}")
        return 0

    if not args.port and not args.dry_run:
        ap.error("--port is required (or use --list / --dry-run)")

    try:
        pressures = [float(x) for x in args.pressures.split(",") if x.strip()]
    except ValueError:
        ap.error("--pressures must be a comma-separated list of numbers")
    if not pressures:
        ap.error("--pressures is empty")

    total = len(pressures) * (args.settle + args.dwell)
    print(__doc__.split("Usage\n-----")[0].strip())
    print("\n" + "-" * 78)
    print(f"Port       : {args.port}")
    print(f"Pressures  : {', '.join(f'{p:g}' for p in pressures)} cmH2O")
    print(f"Timing     : {args.settle:.0f}s settle + {args.dwell:.0f}s dwell each "
          f"(~{total/60:.1f} min total)")
    print(f"Thresholds : {'dip/rise -> %.1f cmH2O/s' % args.dip_thresh if args.thresholds else 'left alone'}")
    print("-" * 78)
    print("\nThe blower WILL run at these pressures. Seal the circuit, leave it")
    print("unattended, and do not breathe into it - the fit needs a static circuit.")

    if not args.yes and not args.dry_run:
        try:
            if input("\nProceed? [y/N] ").strip().lower() not in ("y", "yes"):
                print("Aborted.")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            return 1

    link = Link(args.port or "-", args.baud, args.dry_run, args.debug)
    points = []
    started = False
    try:
        link.open()
        link.send("telem on")

        if args.thresholds:
            # Step 1: stop the dip/rise FF firing on pressure noise. Measured at
            # 16 cmH2O with no flow, dip reached its full 1500 rpm cap purely on
            # noise, which both wastes headroom and pollutes this measurement.
            print("\nRaising dip/rise FF thresholds ...")
            link.send(f"blower set dip_ff_thresh {args.dip_thresh:g}")
            link.send(f"blower set rise_ff_thresh {args.rise_thresh:g}")

        # Fixed CPAP, no ramp, no EPR - anything that moves the setpoint during
        # the dwell would be fitted as if it were the circuit.
        print("Configuring fixed CPAP (no ramp, no EPR) ...")
        link.send("therapy set mode cpap")
        link.send("therapy set ramp 0")
        link.send("therapy set epr 0")
        link.send(f"therapy set pressure {pressures[0]:.1f}")
        link.send("therapy start")
        started = True

        print("\nMeasuring:")
        for sp in pressures:
            points.append(measure_point(link, sp, args))

    except KeyboardInterrupt:
        print("\n\nInterrupted - stopping therapy.")
    finally:
        if started:
            link.send("therapy stop")
            print("\nTherapy stopped.")
        link.close()

    if args.dry_run:
        return 0

    fit = fit_sqrt_model(points)
    report(points, fit, args)
    if args.csv:
        write_csv(points, args.csv)
    if args.plot:
        make_plot(points, fit, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
