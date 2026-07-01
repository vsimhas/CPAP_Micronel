#!/usr/bin/env python3
"""
Live CPAP pressure / patient-flow plotter over USB VCP.

Expects CSV lines from firmware:
  T,<ms>,<pressure_cmh2o>,<patient_flow_slm>,<pid_rpm_cmd>,<mech_rpm>
  R,<ms>,<rpm_cmd>,<rpm_act>,<pressure_cmh2o>   — on manual blower speed change

Usage:
  pip install pyserial matplotlib
  python cpap_telemetry_plot.py --port COM13
  python cpap_telemetry_plot.py --port COM13 --monitor   # text-only check
  python cpap_telemetry_plot.py --list                   # show COM ports
"""

from __future__ import annotations

import argparse
import collections
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator
import serial
import serial.tools.list_ports

TELEM_RE = re.compile(
    rb"^T,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
    rb"(?:,(-?\d+))?(?:,(-?\d+))?\s*$"
)
BENCH_RE = re.compile(rb"^R,(\d+),(-?\d+),(-?\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$")
STATUS_RE = re.compile(
    rb"^S,(\d+),(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+),(\d+),(\d+)"
    rb"(?:,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))?"
    rb"(?:,(\d+),(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+))?\s*$"
)

APAP_STATE_NAMES = {
    0: "Idle",
    1: "Monitor",
    2: "NoBreath",
    3: "FOT",
    4: "FOT",
}


def format_fot_impedance(z: float) -> str:
    if z >= 900.0:
        return "HIGH (blocked)"
    if z <= 0.0:
        return "—"
    return f"{z:.2f} cmH2O/slm"


def format_fot_status(st: dict[str, float | int]) -> str:
    mode = int(st.get("mode", 0))
    if mode == 0:
        return "Airway check: n/a in CPAP mode"
    if "apap_st" not in st:
        return "Airway check: waiting for therapy status…"

    apap_st = int(st.get("apap_st", 0))
    osc = int(st.get("fot_osc", 0))
    p_pp = float(st.get("fot_p_pp", 0.0))
    q_pp = float(st.get("fot_q_pp", 0.0))
    z = float(st.get("fot_z", 0.0))
    osa = int(st.get("fot_osa", 0))
    q_thr = float(st.get("fot_q_thr", 0.0))
    treating = int(st.get("fot_treating", 0))
    state_name = APAP_STATE_NAMES.get(apap_st, str(apap_st))

    if not osc and apap_st < 2:
        return f"Airway check: {state_name} — FOT idle (breathing detected)"

    osa_label = "OBSTRUCTED" if osa else "patent"
    if treating:
        osa_label += " + raising P"
    return (
        f"Airway check [{state_name}]: "
        f"ΔP_pp={p_pp:.2f} cmH2O  ΔQ_pp={q_pp:.2f} slm  "
        f"Z_eff={format_fot_impedance(z)}  "
        f"(OSA if ΔQ_pp < {q_thr:.1f})  → {osa_label}"
    )


def configure_time_grid(ax: plt.Axes) -> None:
    """Major vertical grid every 5 s; thinner minor lines every 1 s."""
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.set_minor_locator(MultipleLocator(1))
    ax.grid(True, which="major", alpha=0.35)
    ax.grid(True, which="minor", axis="x", alpha=0.15, linewidth=0.6)


def list_serial_ports(verbose: bool = False) -> list[str]:
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(p.device)
        if verbose:
            print(f"{p.device}\t{p.description}")
    return ports


class TelemetryReader:
    def __init__(
        self,
        port: str,
        baud: int,
        window_s: float,
        debug: bool = False,
        enable_on_open: bool = True,
        record_path: str | None = None,
    ) -> None:
        self.port = port
        self.baud = baud
        self.window_s = window_s
        self.debug = debug
        self.enable_on_open = enable_on_open
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._t0_ms: int | None = None
        self._times: collections.deque[float] = collections.deque()
        self._pressure: collections.deque[float] = collections.deque()
        self._flow: collections.deque[float] = collections.deque()
        self._pid_rpm_cmd: collections.deque[int | None] = collections.deque()
        self._mech_rpm: collections.deque[int | None] = collections.deque()
        self._bench_times: collections.deque[float] = collections.deque()
        self._bench_rpm_cmd: collections.deque[int] = collections.deque()
        self._bench_rpm_act: collections.deque[int] = collections.deque()
        self._last_line = ""
        self._last_other = ""
        self._last_bench_line = ""
        self._last_ack = ""
        self._last_err = ""
        self._last_status: dict[str, float | int] = {}
        self._samples = 0
        self._bench_events = 0
        self._bytes_in = 0
        self._lines_in = 0
        self._other_lines = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ser: serial.Serial | None = None
        self._opened_at = 0.0
        self._record_path = record_path
        self._record_file = None
        self._echo_bench_lines = False

    def start(self) -> None:
        if self._record_path:
            self._record_file = open(self._record_path, "w", encoding="utf-8")
            self._record_file.write("# raw CPAP telemetry\n")
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._ser.dtr = True
        self._ser.rts = False
        self._opened_at = time.monotonic()
        time.sleep(0.4)
        self._ser.reset_input_buffer()
        if self.enable_on_open:
            self._ser.write(b"telem on\r\n")
            self._ser.flush()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        if self._record_file is not None:
            self._record_file.close()
            self._record_file = None

    def send_command(self, command: str) -> bool:
        line = command.strip()
        if not line:
            return False
        with self._write_lock:
            if self._ser is None or not self._ser.is_open:
                with self._lock:
                    self._last_err = "serial closed"
                return False
            self._ser.write((line + "\r\n").encode("ascii", errors="ignore"))
            self._ser.flush()
        return True

    def snapshot(
        self,
    ) -> tuple[
        list[float],
        list[float],
        list[float],
        list[int | None],
        list[int | None],
        list[float],
        list[int],
        list[int],
        str,
        int,
        int,
        int,
        str,
        float,
        int,
    ]:
        with self._lock:
            return (
                list(self._times),
                list(self._pressure),
                list(self._flow),
                list(self._pid_rpm_cmd),
                list(self._mech_rpm),
                list(self._bench_times),
                list(self._bench_rpm_cmd),
                list(self._bench_rpm_act),
                self._last_line,
                self._samples,
                self._bytes_in,
                self._lines_in,
                self._last_other,
                time.monotonic() - self._opened_at,
                self._bench_events,
            )

    def control_snapshot(self) -> tuple[str, str, dict[str, float | int]]:
        with self._lock:
            return self._last_ack, self._last_err, dict(self._last_status)

    def _time_s(self, ms: int) -> float:
        if self._t0_ms is None:
            self._t0_ms = ms
        return (ms - self._t0_ms) / 1000.0

    def _trim_window(self) -> None:
        while self._times and (self._times[-1] - self._times[0]) > self.window_s:
            self._times.popleft()
            self._pressure.popleft()
            self._flow.popleft()
            self._pid_rpm_cmd.popleft()
            self._mech_rpm.popleft()
        while self._bench_times and self._times and self._bench_times[0] < self._times[0]:
            self._bench_times.popleft()
            self._bench_rpm_cmd.popleft()
            self._bench_rpm_act.popleft()

    def _append_sample(
        self,
        ms: int,
        pressure: float,
        flow: float,
        pid_rpm_cmd: int | None,
        mech_rpm: int | None,
    ) -> None:
        t_s = self._time_s(ms)

        with self._lock:
            self._times.append(t_s)
            self._pressure.append(pressure)
            self._flow.append(flow)
            self._pid_rpm_cmd.append(pid_rpm_cmd)
            self._mech_rpm.append(mech_rpm)
            self._samples += 1
            parts = [f"T,{ms},{pressure:.2f},{flow:.2f}"]
            if pid_rpm_cmd is not None:
                parts.append(str(pid_rpm_cmd))
                if mech_rpm is not None:
                    parts.append(str(mech_rpm))
            self._last_line = ",".join(parts)
            self._trim_window()

    def _append_bench(self, ms: int, rpm_cmd: int, rpm_act: int, pressure: float) -> None:
        t_s = self._time_s(ms)

        with self._lock:
            self._bench_times.append(t_s)
            self._bench_rpm_cmd.append(rpm_cmd)
            self._bench_rpm_act.append(rpm_act)
            self._bench_events += 1
            bench_line = f"R,{ms},{rpm_cmd},{rpm_act},{pressure:.2f}"
            self._last_line = bench_line
            self._last_bench_line = bench_line
            self._trim_window()

    def _handle_line(self, line: bytes) -> None:
        with self._lock:
            self._lines_in += 1

        if self.debug:
            print(line.decode("utf-8", errors="replace"))

        if self._record_file is not None:
            self._record_file.write(line.decode("utf-8", errors="replace") + "\n")
            self._record_file.flush()

        m = TELEM_RE.match(line)
        if m:
            ms = int(m.group(1))
            pressure = float(m.group(2))
            flow = float(m.group(3))
            pid_rpm_cmd = int(m.group(4)) if m.group(4) is not None else None
            mech_rpm = int(m.group(5)) if m.group(5) is not None else None
            self._append_sample(ms, pressure, flow, pid_rpm_cmd, mech_rpm)
            return

        m = BENCH_RE.match(line)
        if m:
            ms = int(m.group(1))
            rpm_cmd = int(m.group(2))
            rpm_act = int(m.group(3))
            pressure = float(m.group(4))
            self._append_bench(ms, rpm_cmd, rpm_act, pressure)
            return

        m = STATUS_RE.match(line)
        if m:
            with self._lock:
                self._last_status = {
                    "ms": int(m.group(1)),
                    "state": int(m.group(2)),
                    "target": float(m.group(3)),
                    "setpoint": float(m.group(4)),
                    "measured": float(m.group(5)),
                    "leak": float(m.group(6)),
                    "epr": int(m.group(7)),
                    "maskoff": int(m.group(8)),
                    "fault": int(m.group(9)),
                }
                if m.group(10) is not None:
                    self._last_status["mode"] = int(m.group(10))
                    self._last_status["pressure_min"] = float(m.group(11))
                    self._last_status["pressure_max"] = float(m.group(12))
                if m.group(13) is not None:
                    self._last_status["apap_st"] = int(m.group(13))
                    self._last_status["fot_osc"] = int(m.group(14))
                    self._last_status["fot_p_pp"] = float(m.group(15))
                    self._last_status["fot_q_pp"] = float(m.group(16))
                    self._last_status["fot_z"] = float(m.group(17))
                    self._last_status["fot_osa"] = int(m.group(18))
                    self._last_status["fot_q_thr"] = float(m.group(19))
                    self._last_status["fot_treating"] = int(m.group(20))
                self._last_line = line.decode("utf-8", errors="replace")
            return

        if line.startswith(b"# ack "):
            with self._lock:
                self._last_ack = line.decode("utf-8", errors="replace")
                self._last_err = ""
                self._last_line = self._last_ack
            return

        if line.startswith(b"# err "):
            with self._lock:
                self._last_err = line.decode("utf-8", errors="replace")
                self._last_line = self._last_err
            return

        with self._lock:
            self._other_lines += 1
            text = line.decode("utf-8", errors="replace")
            if text:
                self._last_other = text

    def _run(self) -> None:
        assert self._ser is not None
        buf = bytearray()
        while not self._stop.is_set():
            chunk = self._ser.read(512)
            if not chunk:
                continue
            with self._lock:
                self._bytes_in += len(chunk)
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl]).strip(b"\r")
                del buf[: nl + 1]
                if line:
                    self._handle_line(line)


def run_monitor(reader: TelemetryReader, duration: float, record_path: str | None) -> int:
    reader._echo_bench_lines = True
    print(f"Monitoring {reader.port} for {duration:.0f}s (Ctrl+C to stop early)...")
    if record_path:
        print(f"Recording raw T/R lines to: {record_path}")
    print("Expect lines like: T,12345,8.12,3.45,17500,17420  or  R,12345,17500,17480,7.50")
    print("Or debug text like: [0.001] I [SYS] debug log online")
    print("-" * 60)
    t_end = time.monotonic() + duration
    try:
        while time.monotonic() < t_end:
            (
                _t,
                _p,
                _f,
                _rpm,
                _mech,
                _bt,
                _bc,
                _ba,
                last,
                samples,
                nbytes,
                nlines,
                other,
                elapsed,
                bench,
            ) = reader.snapshot()
            if reader._echo_bench_lines and reader._last_bench_line:
                print(f"\n{reader._last_bench_line}")
                reader._last_bench_line = ""
            status = (
                f"[{elapsed:5.1f}s] bytes={nbytes:6d} lines={nlines:4d} "
                f"telem={samples:4d} bench={bench:3d} other={reader._other_lines}"
            )
            if last:
                status += f"  last={last}"
            elif other:
                status += f"  last_other={other[:60]}"
            print(status, end="\r", flush=True)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print()

    (
        _t,
        _p,
        _f,
        _rpm,
        _mech,
        _bt,
        _bc,
        _ba,
        last,
        samples,
        nbytes,
        nlines,
        other,
        elapsed,
        bench,
    ) = reader.snapshot()
    print(
        f"Done in {elapsed:.1f}s: {nbytes} bytes, {nlines} lines, "
        f"{samples} telem, {bench} bench"
    )
    if nbytes == 0:
        print("\nNo data received. Check:")
        print("  1. Correct COM port? STM32 boards often expose TWO ports")
        print("     (ST-Link UART vs USB-CDC). Use --list for descriptions.")
        print("  2. Firmware rebuilt/flashed with telemetry_stream.c?")
        print("  3. Close other serial terminals (PuTTY, Keil debug, etc.)")
        print("  4. Replug USB after flashing")
        return 1
    if samples == 0 and reader._other_lines > 0:
        print("\nSerial works but no T,... lines yet.")
        print("  - g_telem_enabled should be 1 (default after latest firmware)")
        print("  - Sent 'telem on' on connect; try again after reset")
        if other:
            print(f"  - Last non-telem line: {other}")
        return 2
    if samples > 0:
        print(f"Telemetry OK. Last line: {last}")
        return 0
    return 2


def main() -> int:
    parser = argparse.ArgumentParser(description="CPAP live telemetry plotter")
    parser.add_argument("--port", help="Serial port (e.g. COM13)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--window",
        type=float,
        default=30.0,
        help="Rolling plot window in seconds (default 30)",
    )
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="Text-only monitor (no plot) to verify serial data",
    )
    parser.add_argument(
        "--monitor-time",
        type=float,
        default=15.0,
        help="Seconds for --monitor mode (default 15)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print every received line to the console",
    )
    parser.add_argument(
        "--no-enable",
        action="store_true",
        help="Do not send 'telem on' when opening the port",
    )
    parser.add_argument(
        "--record",
        metavar="FILE",
        help="Write raw T/R lines to FILE (works with --monitor or plot mode)",
    )
    args = parser.parse_args()

    if args.list:
        if not list_serial_ports(verbose=True):
            print("No serial ports found.")
        return 0

    port = args.port
    if port is None:
        ports = list_serial_ports()
        if not ports:
            print("No serial ports found. Use --port COMx", file=sys.stderr)
            return 1
        port = ports[0]
        print(f"Using first port: {port}")

    reader = TelemetryReader(
        port,
        args.baud,
        args.window,
        debug=args.debug,
        enable_on_open=not args.no_enable,
        record_path=args.record,
    )
    try:
        reader.start()
    except serial.SerialException as exc:
        print(f"Cannot open {port}: {exc}", file=sys.stderr)
        print("Close other programs using this COM port and try again.", file=sys.stderr)
        return 1

    if args.monitor:
        try:
            return run_monitor(reader, args.monitor_time, args.record)
        finally:
            reader.stop()

    root = tk.Tk()
    root.title(f"CPAP Telemetry and Therapy Control - {port}")
    root.geometry("1100x1020")
    root.minsize(1000, 900)

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True)

    plot_tab = ttk.Frame(notebook)
    control_tab = ttk.Frame(notebook, padding=12)
    foc_tab = ttk.Frame(notebook, padding=12)
    notebook.add(plot_tab, text="Plots")
    notebook.add(control_tab, text="Therapy Controls")
    notebook.add(foc_tab, text="FOC Tuning")

    fig = Figure(figsize=(10, 6), dpi=100)
    fig.subplots_adjust(bottom=0.12)
    ax_p = fig.add_subplot(2, 1, 1)
    ax_f = fig.add_subplot(2, 1, 2, sharex=ax_p)
    fig.suptitle(f"CPAP telemetry — {port}")
    ax_p.set_ylabel("Pressure at blower (cmH2O)")
    ax_p.set_ylim(0, 25)
    ax_p.set_xlim(0, args.window)
    configure_time_grid(ax_p)
    ax_rpm = ax_p.twinx()
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_ylim(0, 22000)
    ax_f.set_ylabel("Patient flow (slm)")
    ax_f.set_xlabel("Time (s)")
    ax_f.set_ylim(-30, 30)
    ax_f.set_xlim(0, args.window)
    configure_time_grid(ax_f)

    line_p, = ax_p.plot([], [], "b-", linewidth=1.2, label="P_sensor")
    line_pid_rpm, = ax_rpm.plot(
        [], [], color="darkorange", linewidth=1.0, drawstyle="steps-post", label="PID RPM cmd"
    )
    line_mech_rpm, = ax_rpm.plot(
        [], [], color="crimson", linewidth=1.2, label="RPM mech"
    )
    line_rpm_cmd, = ax_rpm.plot(
        [], [], "x", color="saddlebrown", markersize=4, linestyle="none", label="bench RPM cmd"
    )
    line_rpm_act, = ax_rpm.plot(
        [], [], "o", color="mediumpurple", markersize=4, linestyle="none", label="bench RPM act"
    )
    line_f, = ax_f.plot([], [], "g-", linewidth=1.2, label="Q_patient")
    ax_p.legend(loc="upper left")
    ax_rpm.legend(loc="upper right")
    ax_f.legend(loc="upper right")
    waiting = ax_p.text(
        0.5,
        0.5,
        "Waiting for T,... telemetry lines",
        transform=ax_p.transAxes,
        ha="center",
        va="center",
        fontsize=11,
        color="gray",
    )
    status = fig.text(0.01, 0.01, "", fontsize=8, family="monospace")
    fot_status = fig.text(0.01, 0.045, "", fontsize=9, family="monospace", color="#1a3a6b")

    status_poll_counter = [0]

    canvas = FigureCanvasTkAgg(fig, master=plot_tab)
    toolbar = NavigationToolbar2Tk(canvas, plot_tab, pack_toolbar=False)
    toolbar.update()
    toolbar.pack(side=tk.BOTTOM, fill=tk.X)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    cmd_status = tk.StringVar(value="Ready")
    ack_status = tk.StringVar(value="")
    live_status = tk.StringVar(value="No therapy status yet")
    mode_var = tk.StringVar(value="CPAP")
    pressure_var = tk.StringVar(value="8.0")
    apap_min_var = tk.StringVar(value="5.0")
    apap_max_var = tk.StringVar(value="20.0")
    ramp_var = tk.StringVar(value="0")
    epr_var = tk.BooleanVar(value=False)
    epr_type_var = tk.StringVar(value="fulltime")
    epr_level_var = tk.IntVar(value=1)
    maskoff_var = tk.BooleanVar(value=True)
    smart_start_var = tk.BooleanVar(value=False)
    smart_stop_var = tk.BooleanVar(value=False)
    tube_mode_var = tk.StringVar(value="Auto")
    tube_temp_var = tk.StringVar(value="27")
    climate_var = tk.StringVar(value="Auto")
    humidity_var = tk.StringVar(value="4")
    airplane_var = tk.BooleanVar(value=False)
    loop_enable_var = tk.BooleanVar(value=True)
    pid_enable_var = tk.BooleanVar(value=True)
    rpm_floor_var = tk.BooleanVar(value=True)
    manual_rpm_var = tk.StringVar(value="12000")
    manual_ramp_ms_var = tk.StringVar(value="5000")
    blower_vars: dict[str, tk.StringVar | tk.BooleanVar] = {
        "kp_low": tk.StringVar(value="131"),
        "ki_low": tk.StringVar(value="1796"),
        "kd_low": tk.StringVar(value="0.6"),
        "kp_mid": tk.StringVar(value="107"),
        "ki_mid": tk.StringVar(value="1467"),
        "kd_mid": tk.StringVar(value="0.5"),
        "kp_high": tk.StringVar(value="83"),
        "ki_high": tk.StringVar(value="1136"),
        "kd_high": tk.StringVar(value="0.4"),
        "ff_enable": tk.BooleanVar(value=True),
        "ff_k": tk.StringVar(value="6141"),
        "flow_ff_enable": tk.BooleanVar(value=True),
        "flow_ff_k": tk.StringVar(value="58"),
        "flow_ff_max": tk.StringVar(value="2200"),
        "flow_ff_deadband": tk.StringVar(value="3"),
        "flow_ff_rise": tk.StringVar(value="3500"),
        "flow_ff_fall": tk.StringVar(value="7000"),
        "over_p_gain": tk.StringVar(value="1.5"),
        "over_brake": tk.StringVar(value="6000"),
        "slew_down": tk.StringVar(value="6"),
        "slew_up": tk.StringVar(value="18"),
        "out_alpha": tk.StringVar(value="1.0"),
        "integ_deadband": tk.StringVar(value="1.2"),
        "integ_max": tk.StringVar(value="4000"),
        "meas_alpha": tk.StringVar(value="0.45"),
        "deriv_alpha": tk.StringVar(value="0.25"),
        "rpm_max": tk.StringVar(value="30000"),
    }
    foc_vars: dict[str, tk.StringVar] = {
        "speed_kp": tk.StringVar(value="2339"),
        "speed_ki": tk.StringVar(value="367"),
        "torque_kp": tk.StringVar(value="2168"),
        "torque_ki": tk.StringVar(value="806"),
        "flux_kp": tk.StringVar(value="2168"),
        "flux_ki": tk.StringVar(value="806"),
    }

    state_names = {
        0: "Idle",
        1: "Ramp",
        2: "Running",
        3: "Stopping",
        4: "Fault",
    }
    therapy_mode_names = {
        0: "CPAP",
        1: "Autoset",
        2: "Autoset for her",
    }
    therapy_mode_commands = {
        "CPAP": "cpap",
        "Autoset": "autoset",
        "Autoset for her": "autoset_her",
    }

    def send(command: str) -> None:
        if reader.send_command(command):
            cmd_status.set(f"Sent: {command}")
        else:
            cmd_status.set(f"Send failed: {command}")

    def ramp_minutes() -> int:
        text = ramp_var.get()
        if text.startswith("Auto"):
            return 5
        try:
            return int(text)
        except ValueError:
            return 0

    def apply_loop_options(request_status: bool = True) -> None:
        send(f"blower set loop_enable {1 if loop_enable_var.get() else 0}")
        send(f"blower set pid_enable {1 if pid_enable_var.get() else 0}")
        send(f"blower set rpm_floor_enable {1 if rpm_floor_var.get() else 0}")
        send(f"blower set ff_enable {1 if blower_vars['ff_enable'].get() else 0}")
        send(f"blower set flow_ff_enable {1 if blower_vars['flow_ff_enable'].get() else 0}")
        if request_status:
            send("therapy status")

    def manual_ramp_ms() -> int:
        try:
            ramp_ms = int(float(manual_ramp_ms_var.get()))
        except ValueError:
            cmd_status.set("Invalid manual ramp duration (ms)")
            return -1
        return max(0, min(65535, ramp_ms))

    def manual_target_rpm() -> int | None:
        try:
            rpm = int(float(manual_rpm_var.get()))
        except ValueError:
            cmd_status.set("Invalid manual target RPM")
            return None
        return max(0, min(45000, rpm))

    def start_manual_blower() -> None:
        rpm = manual_target_rpm()
        if rpm is None:
            return
        ramp_ms = manual_ramp_ms()
        if ramp_ms < 0:
            return
        send("therapy stop")
        send("blower set loop_enable 0")
        send(f"blower manual start {rpm} {ramp_ms}")

    def update_manual_blower_speed() -> None:
        rpm = manual_target_rpm()
        if rpm is None:
            return
        ramp_ms = manual_ramp_ms()
        if ramp_ms < 0:
            return
        send(f"blower manual speed {rpm} {ramp_ms}")

    def stop_manual_blower() -> None:
        send("blower manual stop")

    def apply_settings() -> None:
        mode = mode_var.get()
        if mode not in therapy_mode_commands:
            cmd_status.set("Invalid therapy mode")
            return
        send(f"therapy set mode {therapy_mode_commands[mode]}")
        try:
            if mode == "CPAP":
                pressure = float(pressure_var.get())
                send(f"therapy set pressure {pressure:.1f}")
            else:
                pmin = float(apap_min_var.get())
                pmax = float(apap_max_var.get())
                if pmin >= pmax:
                    cmd_status.set("Min pressure must be below max")
                    return
                send(f"therapy set pressure_min {pmin:.1f}")
                send(f"therapy set pressure_max {pmax:.1f}")
        except ValueError:
            cmd_status.set("Invalid numeric field")
            return
        send(f"therapy set ramp {ramp_minutes()}")
        send(f"therapy set epr {1 if epr_var.get() else 0}")
        if epr_var.get():
            send(f"therapy set epr_type {epr_type_var.get()}")
            send(f"therapy set epr_level {epr_level_var.get()}")
        send(f"therapy set maskoff {1 if maskoff_var.get() else 0}")
        apply_loop_options(request_status=False)
        apply_blower_tuning(request_status=False)
        apply_foc_tuning(request_status=False)
        send("therapy status")

    def apply_foc_tuning(request_status: bool = True) -> None:
        for name, var in foc_vars.items():
            value = var.get().strip()
            if not value:
                cmd_status.set(f"Missing FOC tuning value: {name}")
                return
            try:
                int(value)
            except ValueError:
                cmd_status.set(f"Invalid FOC tuning value: {name}")
                return
            send(f"foc set {name} {value}")
        if request_status:
            send("therapy status")

    def apply_blower_tuning(request_status: bool = True) -> None:
        for name, var in blower_vars.items():
            if isinstance(var, tk.BooleanVar):
                send(f"blower set {name} {1 if var.get() else 0}")
                continue
            value = var.get().strip()
            if not value:
                cmd_status.set(f"Missing blower tuning value: {name}")
                return
            try:
                float(value)
            except ValueError:
                cmd_status.set(f"Invalid blower tuning value: {name}")
                return
            send(f"blower set {name} {value}")
        if request_status:
            send("therapy status")

    def start_therapy() -> None:
        send("blower manual stop")
        apply_settings()
        send("therapy start")

    def stop_therapy() -> None:
        send("therapy stop")
        send("blower manual stop")

    def refresh_status() -> None:
        ack, err, st = reader.control_snapshot()
        if err:
            ack_status.set(err)
        elif ack:
            ack_status.set(ack)
        if st:
            state = state_names.get(int(st.get("state", -1)), str(st.get("state", "?")))
            mode_label = therapy_mode_names.get(int(st.get("mode", 0)), "CPAP")
            range_text = ""
            if int(st.get("mode", 0)) != 0:
                range_text = "  Range={:.1f}-{:.1f}".format(
                    float(st.get("pressure_min", 0.0)),
                    float(st.get("pressure_max", 0.0)),
                )
            live_status.set(
                "Mode={mode}{range_text}  State={state}  Target={target:.1f}  "
                "Setpoint={setpoint:.1f}  Measured={measured:.1f}  Leak={leak:.0f}  "
                "EPR={epr}  MaskOff={maskoff}  Fault={fault}".format(
                    mode=mode_label,
                    range_text=range_text,
                    state=state,
                    target=float(st.get("target", 0.0)),
                    setpoint=float(st.get("setpoint", 0.0)),
                    measured=float(st.get("measured", 0.0)),
                    leak=float(st.get("leak", 0.0)),
                    epr="On" if int(st.get("epr", 0)) else "Off",
                    maskoff="Active" if int(st.get("maskoff", 0)) else "No",
                    fault=int(st.get("fault", 0)),
                )
            )
        root.after(500, refresh_status)

    controls = ttk.LabelFrame(control_tab, text="Therapy", padding=10)
    controls.grid(row=0, column=0, sticky="nsew", padx=(0, 12), pady=(0, 12))
    options = ttk.LabelFrame(control_tab, text="Options", padding=10)
    options.grid(row=0, column=1, sticky="nsew", pady=(0, 12))
    loop_frame = ttk.LabelFrame(control_tab, text="Pressure Loop (CM7)", padding=10)
    loop_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 12))
    manual_frame = ttk.LabelFrame(control_tab, text="Manual Blower (open loop)", padding=10)
    manual_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 12))
    tuning = ttk.LabelFrame(control_tab, text="Blower Tuning", padding=10)
    tuning.grid(row=3, column=0, sticky="nsew", padx=(0, 6), pady=(0, 0))
    more = ttk.LabelFrame(control_tab, text="More / Status", padding=10)
    more.grid(row=3, column=1, sticky="nsew", padx=(6, 0), pady=(0, 0))
    control_tab.columnconfigure(0, weight=3)
    control_tab.columnconfigure(1, weight=2)
    control_tab.rowconfigure(3, weight=1)

    ttk.Label(controls, text="Mode").grid(row=0, column=0, sticky="w")
    ttk.Combobox(
        controls,
        textvariable=mode_var,
        values=["CPAP", "Autoset", "Autoset for her"],
        width=16,
        state="readonly",
    ).grid(row=0, column=1, sticky="w", padx=8, pady=4)

    cpap_pressure_lbl = ttk.Label(controls, text="CPAP Pressure (cmH2O)")
    cpap_pressure_sb = ttk.Spinbox(
        controls, from_=4.0, to=25.0, increment=0.5, textvariable=pressure_var, width=8
    )
    apap_min_lbl = ttk.Label(controls, text="Min Pressure (cmH2O)")
    apap_min_sb = ttk.Spinbox(
        controls, from_=4.0, to=25.0, increment=0.5, textvariable=apap_min_var, width=8
    )
    apap_max_lbl = ttk.Label(controls, text="Max Pressure (cmH2O)")
    apap_max_sb = ttk.Spinbox(
        controls, from_=4.0, to=25.0, increment=0.5, textvariable=apap_max_var, width=8
    )

    def update_mode_visibility(*_args: object) -> None:
        if mode_var.get() == "CPAP":
            cpap_pressure_lbl.grid(row=1, column=0, sticky="w")
            cpap_pressure_sb.grid(row=1, column=1, sticky="w", padx=8, pady=4)
            apap_min_lbl.grid_remove()
            apap_min_sb.grid_remove()
            apap_max_lbl.grid_remove()
            apap_max_sb.grid_remove()
        else:
            cpap_pressure_lbl.grid_remove()
            cpap_pressure_sb.grid_remove()
            apap_min_lbl.grid(row=1, column=0, sticky="w")
            apap_min_sb.grid(row=1, column=1, sticky="w", padx=8, pady=4)
            apap_max_lbl.grid(row=2, column=0, sticky="w")
            apap_max_sb.grid(row=2, column=1, sticky="w", padx=8, pady=4)

    mode_var.trace_add("write", update_mode_visibility)
    update_mode_visibility()

    ttk.Label(controls, text="Ramp Time (min)").grid(row=3, column=0, sticky="w")
    ttk.Combobox(
        controls,
        textvariable=ramp_var,
        values=["0", "Auto/5", "5", "10", "15", "20", "25", "30", "35", "40", "45"],
        width=8,
        state="readonly",
    ).grid(row=3, column=1, sticky="w", padx=8, pady=4)
    ttk.Checkbutton(controls, text="Pressure Relief (EPR)", variable=epr_var).grid(
        row=4, column=0, sticky="w", pady=4
    )

    epr_options = ttk.LabelFrame(controls, text="EPR Settings", padding=8)
    epr_options.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(0, 8))

    def update_epr_options_visibility(*_args: object) -> None:
        if epr_var.get():
            epr_options.grid()
        else:
            epr_options.grid_remove()

    epr_var.trace_add("write", update_epr_options_visibility)

    ttk.Label(epr_options, text="EPR Type").grid(row=0, column=0, sticky="w", pady=2)
    epr_type_frame = ttk.Frame(epr_options)
    epr_type_frame.grid(row=0, column=1, sticky="w", pady=2)
    ttk.Radiobutton(
        epr_type_frame, text="Full time", variable=epr_type_var, value="fulltime"
    ).pack(side=tk.LEFT, padx=(0, 12))
    ttk.Radiobutton(
        epr_type_frame, text="Ramp only", variable=epr_type_var, value="ramp"
    ).pack(side=tk.LEFT)

    ttk.Label(epr_options, text="EPR Level (cmH2O)").grid(row=1, column=0, sticky="w", pady=2)
    epr_level_frame = ttk.Frame(epr_options)
    epr_level_frame.grid(row=1, column=1, sticky="w", pady=2)
    for level in (1, 2, 3):
        ttk.Radiobutton(
            epr_level_frame, text=str(level), variable=epr_level_var, value=level
        ).pack(side=tk.LEFT, padx=(0, 10))

    update_epr_options_visibility()

    ttk.Checkbutton(controls, text="Mask-off low-speed recovery", variable=maskoff_var).grid(
        row=6, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Button(controls, text="Apply Settings", command=apply_settings).grid(row=7, column=0, sticky="ew", pady=8)
    ttk.Button(controls, text="Start Therapy", command=start_therapy).grid(row=7, column=1, sticky="ew", pady=8, padx=8)
    ttk.Button(controls, text="Stop Therapy", command=stop_therapy).grid(row=8, column=0, columnspan=2, sticky="ew")

    ttk.Checkbutton(options, text="SmartStart (UI placeholder)", variable=smart_start_var).grid(
        row=0, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Checkbutton(options, text="SmartStop (UI placeholder)", variable=smart_stop_var).grid(
        row=1, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Label(options, text="Tube Temperature").grid(row=2, column=0, sticky="w")
    ttk.Combobox(options, textvariable=tube_mode_var, values=["Off", "Auto", "Manual"], width=10, state="readonly").grid(
        row=2, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Label(options, text="Manual Temp (C)").grid(row=3, column=0, sticky="w")
    ttk.Spinbox(options, from_=16, to=30, increment=1, textvariable=tube_temp_var, width=8).grid(
        row=3, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Label(options, text="Climate Control").grid(row=4, column=0, sticky="w")
    ttk.Combobox(options, textvariable=climate_var, values=["Auto", "Manual"], width=10, state="readonly").grid(
        row=4, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Label(options, text="Humidity Level").grid(row=5, column=0, sticky="w")
    ttk.Combobox(options, textvariable=humidity_var, values=["0", "1", "2", "3", "4", "5", "6", "7", "8"], width=8, state="readonly").grid(
        row=5, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Checkbutton(options, text="Airplane Mode (UI placeholder)", variable=airplane_var).grid(
        row=6, column=0, columnspan=2, sticky="w", pady=4
    )

    ttk.Checkbutton(loop_frame, text="Outer loop enabled", variable=loop_enable_var).grid(
        row=0, column=0, sticky="w", padx=(0, 16), pady=2
    )
    ttk.Checkbutton(loop_frame, text="PID trim (P/I/D)", variable=pid_enable_var).grid(
        row=0, column=1, sticky="w", padx=(0, 16), pady=2
    )
    ttk.Checkbutton(loop_frame, text="Pressure FF (sqrt)", variable=blower_vars["ff_enable"]).grid(
        row=0, column=2, sticky="w", padx=(0, 16), pady=2
    )
    ttk.Checkbutton(loop_frame, text="Flow FF", variable=blower_vars["flow_ff_enable"]).grid(
        row=0, column=3, sticky="w", pady=2
    )
    ttk.Label(
        loop_frame,
        text="Disable PID to run FF-only; disable outer loop for open-loop manual blower.",
        wraplength=720,
    ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(4, 2))
    ttk.Checkbutton(
        loop_frame,
        text="RPM floor at sqrt(P) FF (EPR off)",
        variable=rpm_floor_var,
    ).grid(row=2, column=0, columnspan=2, sticky="w", pady=2)
    ttk.Button(loop_frame, text="Apply Loop Options", command=apply_loop_options).grid(
        row=3, column=0, columnspan=2, sticky="w"
    )

    ttk.Label(manual_frame, text="Target RPM").grid(row=0, column=0, sticky="w", pady=2)
    ttk.Spinbox(manual_frame, from_=0, to=45000, increment=500, textvariable=manual_rpm_var, width=10).grid(
        row=0, column=1, sticky="w", padx=8, pady=2
    )
    ttk.Label(manual_frame, text="Ramp (ms)").grid(row=0, column=2, sticky="w", pady=2)
    ttk.Spinbox(manual_frame, from_=0, to=65535, increment=500, textvariable=manual_ramp_ms_var, width=8).grid(
        row=0, column=3, sticky="w", padx=8, pady=2
    )
    ttk.Button(manual_frame, text="Start Manual", command=start_manual_blower).grid(
        row=1, column=0, sticky="ew", padx=(0, 8), pady=8
    )
    ttk.Button(manual_frame, text="Update Speed", command=update_manual_blower_speed).grid(
        row=1, column=1, sticky="ew", padx=(0, 8), pady=8
    )
    ttk.Button(manual_frame, text="Stop Manual", command=stop_manual_blower).grid(
        row=1, column=2, sticky="ew", pady=8
    )
    ttk.Label(
        manual_frame,
        text="Stops therapy, disables pressure PID, and ramps CM4 motor speed directly (ramp in ms).",
        wraplength=720,
    ).grid(row=2, column=0, columnspan=4, sticky="w")
    manual_frame.columnconfigure(0, weight=1)
    manual_frame.columnconfigure(1, weight=1)
    manual_frame.columnconfigure(2, weight=1)

    def add_entry(parent, row: int, col: int, label: str, key: str, width: int = 8) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(0, 4), pady=2)
        ttk.Entry(parent, textvariable=blower_vars[key], width=width).grid(
            row=row, column=col + 1, sticky="w", padx=(0, 12), pady=2
        )

    ttk.Label(tuning, text="PID Gains").grid(row=0, column=0, sticky="w", pady=(0, 4))
    add_entry(tuning, 1, 0, "LOW Kp", "kp_low")
    add_entry(tuning, 1, 2, "LOW Ki", "ki_low")
    add_entry(tuning, 1, 4, "LOW Kd", "kd_low")
    add_entry(tuning, 2, 0, "MID Kp", "kp_mid")
    add_entry(tuning, 2, 2, "MID Ki", "ki_mid")
    add_entry(tuning, 2, 4, "MID Kd", "kd_mid")
    add_entry(tuning, 3, 0, "HIGH Kp", "kp_high")
    add_entry(tuning, 3, 2, "HIGH Ki", "ki_high")
    add_entry(tuning, 3, 4, "HIGH Kd", "kd_high")

    ttk.Separator(tuning, orient=tk.HORIZONTAL).grid(row=4, column=0, columnspan=6, sticky="ew", pady=8)
    ttk.Label(
        tuning,
        text="FF toggles are in Pressure Loop above; gains below.",
    ).grid(row=5, column=0, columnspan=4, sticky="w", pady=(0, 4))
    add_entry(tuning, 6, 0, "sqrt FF K", "ff_k")
    add_entry(tuning, 7, 0, "flow FF K", "flow_ff_k")
    add_entry(tuning, 7, 2, "flow FF max", "flow_ff_max")
    add_entry(tuning, 8, 0, "flow deadband", "flow_ff_deadband")
    add_entry(tuning, 8, 2, "flow rise", "flow_ff_rise")
    add_entry(tuning, 8, 4, "flow fall", "flow_ff_fall")

    ttk.Separator(tuning, orient=tk.HORIZONTAL).grid(row=9, column=0, columnspan=6, sticky="ew", pady=8)
    add_entry(tuning, 10, 0, "over P gain", "over_p_gain")
    add_entry(tuning, 10, 2, "over brake", "over_brake")
    add_entry(tuning, 10, 4, "RPM max", "rpm_max")
    add_entry(tuning, 11, 0, "slew down", "slew_down")
    add_entry(tuning, 11, 2, "slew up", "slew_up")
    add_entry(tuning, 11, 4, "out alpha", "out_alpha")
    add_entry(tuning, 12, 0, "I deadband", "integ_deadband")
    add_entry(tuning, 12, 2, "I max rpm", "integ_max")
    add_entry(tuning, 12, 4, "meas alpha", "meas_alpha")
    add_entry(tuning, 13, 0, "D alpha", "deriv_alpha")
    ttk.Button(tuning, text="Apply Blower Tuning", command=apply_blower_tuning).grid(
        row=13, column=2, columnspan=2, sticky="ew", padx=8, pady=4
    )

    ttk.Button(more, text="Request Status", command=lambda: send("therapy status")).grid(
        row=0, column=0, sticky="ew"
    )
    ttk.Button(more, text="Mask Fit (LCD placeholder)", state=tk.DISABLED).grid(
        row=1, column=0, sticky="ew", pady=4
    )
    ttk.Button(more, text="Diagnostics (LCD placeholder)", state=tk.DISABLED).grid(
        row=2, column=0, sticky="ew", pady=4
    )
    ttk.Label(more, textvariable=cmd_status, wraplength=340).grid(
        row=3, column=0, sticky="w", pady=(10, 0)
    )
    ttk.Label(more, textvariable=ack_status, wraplength=340).grid(row=4, column=0, sticky="w")
    ttk.Label(more, textvariable=live_status, wraplength=340).grid(row=5, column=0, sticky="w")
    more.columnconfigure(0, weight=1)

    foc_frame = ttk.LabelFrame(foc_tab, text="CM4 FOC Inner Loop (Motor Control SDK)", padding=10)
    foc_frame.pack(fill=tk.BOTH, expand=True)
    ttk.Label(
        foc_frame,
        text=(
            "CM4 speed/torque/flux PI numerators from drive_parameters.h. "
            "Macros set compile-time defaults; USB commands apply them at runtime via IPC."
        ),
        wraplength=900,
    ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

    def add_foc_entry(row: int, label: str, key: str) -> None:
        ttk.Label(foc_frame, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(foc_frame, textvariable=foc_vars[key], width=10).grid(row=row, column=1, sticky="w", pady=4)

    ttk.Label(foc_frame, text="Speed loop (RPM -> Iq)").grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 2))
    add_foc_entry(2, "Speed Kp", "speed_kp")
    add_foc_entry(3, "Speed Ki", "speed_ki")
    ttk.Label(foc_frame, text="Torque loop (Iq)").grid(row=4, column=0, columnspan=2, sticky="w", pady=(12, 2))
    add_foc_entry(5, "Torque Kp", "torque_kp")
    add_foc_entry(6, "Torque Ki", "torque_ki")
    ttk.Label(foc_frame, text="Flux loop (Id)").grid(row=7, column=0, columnspan=2, sticky="w", pady=(12, 2))
    add_foc_entry(8, "Flux Kp", "flux_kp")
    add_foc_entry(9, "Flux Ki", "flux_ki")
    ttk.Button(foc_frame, text="Apply FOC Tuning", command=apply_foc_tuning).grid(
        row=10, column=0, columnspan=2, sticky="ew", pady=(16, 4)
    )

    def update(_frame: int):
        status_poll_counter[0] += 1
        if status_poll_counter[0] % 10 == 0:
            reader.send_command("therapy status")

        (
            times,
            pressure,
            flow,
            pid_rpm_cmd,
            mech_rpm,
            bench_times,
            bench_rpm_cmd,
            bench_rpm_act,
            last,
            n,
            nbytes,
            nlines,
            other,
            elapsed,
            bench,
        ) = reader.snapshot()
        if times:
            waiting.set_visible(False)
            line_p.set_data(times, pressure)
            line_f.set_data(times, flow)
            rpm_times = [t for t, rpm in zip(times, pid_rpm_cmd) if rpm is not None]
            rpm_values = [rpm for rpm in pid_rpm_cmd if rpm is not None]
            line_pid_rpm.set_data(rpm_times, rpm_values)
            mech_times = [t for t, rpm in zip(times, mech_rpm) if rpm is not None]
            mech_values = [rpm for rpm in mech_rpm if rpm is not None]
            line_mech_rpm.set_data(mech_times, mech_values)
            t_right = max(args.window, times[-1] + 0.5)
            t_left = max(0, times[-1] - args.window)
            ax_p.set_xlim(t_left, t_right)
            ax_f.set_xlim(t_left, t_right)
            if pressure:
                p_min, p_max = min(pressure), max(pressure)
                ax_p.set_ylim(max(0, p_min - 1), min(30, p_max + 2))
            if flow:
                f_min, f_max = min(flow), max(flow)
                margin = max(5.0, (f_max - f_min) * 0.15)
                ax_f.set_ylim(f_min - margin, f_max + margin)
            rpm_axis_values = list(rpm_values) + list(mech_values) + bench_rpm_cmd + bench_rpm_act
            if rpm_axis_values:
                rpm_min = min(rpm_axis_values)
                rpm_max = max(rpm_axis_values)
                margin = max(500.0, (rpm_max - rpm_min) * 0.1)
                ax_rpm.set_ylim(max(0, rpm_min - margin), rpm_max + margin)
        else:
            line_pid_rpm.set_data([], [])
            line_mech_rpm.set_data([], [])
            waiting.set_visible(True)
            hint = "no serial data yet" if nbytes == 0 else "serial OK, no T,... lines yet"
            waiting.set_text(
                f"Waiting for telemetry ({hint})\n"
                f"bytes={nbytes}  lines={nlines}\n"
                "Try: --monitor  or  --list for COM ports"
            )

        if bench_times:
            line_rpm_cmd.set_data(bench_times, bench_rpm_cmd)
            line_rpm_act.set_data(bench_times, bench_rpm_act)
        else:
            line_rpm_cmd.set_data([], [])
            line_rpm_act.set_data([], [])

        status.set_text(
            f"t={elapsed:4.0f}s  bytes={nbytes}  lines={nlines}  "
            f"telem={n}  bench={bench}  last={last or other or '—'}"
        )
        _, _, st = reader.control_snapshot()
        fot_status.set_text(format_fot_status(st))
        return line_p, line_pid_rpm, line_mech_rpm, line_rpm_cmd, line_rpm_act, line_f, waiting, status, fot_status

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )
    _ = ani

    print(f"Plot running on {port}. Sent 'telem on' to the device.")
    print("PID RPM cmd (orange steps) and measured RPM mech (crimson) overlay on pressure.")
    print("If the plot stays empty, run:  python cpap_telemetry_plot.py --port", port, "--monitor")
    print("Use the Therapy Controls tab to send start/stop and settings commands.")
    print("Use the FOC Tuning tab to adjust CM4 inner-loop PI gains before/during therapy.")
    print("Close the GUI window to exit.")

    refresh_status()
    send("therapy status")

    def on_close() -> None:
        root.quit()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
