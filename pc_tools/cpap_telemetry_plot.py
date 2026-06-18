#!/usr/bin/env python3
"""
Live CPAP pressure / patient-flow plotter over USB VCP.

Expects CSV lines from firmware:
  T,<ms>,<pressure_cmh2o at blower>,<patient_flow_slm>,<pid_rpm_cmd>
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

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports

TELEM_RE = re.compile(
    rb"^T,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)(?:,(-?\d+))?\s*$"
)
BENCH_RE = re.compile(rb"^R,(\d+),(-?\d+),(-?\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$")


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
        self._t0_ms: int | None = None
        self._times: collections.deque[float] = collections.deque()
        self._pressure: collections.deque[float] = collections.deque()
        self._flow: collections.deque[float] = collections.deque()
        self._pid_rpm_cmd: collections.deque[int | None] = collections.deque()
        self._bench_times: collections.deque[float] = collections.deque()
        self._bench_rpm_cmd: collections.deque[int] = collections.deque()
        self._bench_rpm_act: collections.deque[int] = collections.deque()
        self._last_line = ""
        self._last_other = ""
        self._last_bench_line = ""
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

    def snapshot(
        self,
    ) -> tuple[
        list[float],
        list[float],
        list[float],
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
        while self._bench_times and self._times and self._bench_times[0] < self._times[0]:
            self._bench_times.popleft()
            self._bench_rpm_cmd.popleft()
            self._bench_rpm_act.popleft()

    def _append_sample(self, ms: int, pressure: float, flow: float, pid_rpm_cmd: int | None) -> None:
        t_s = self._time_s(ms)

        with self._lock:
            self._times.append(t_s)
            self._pressure.append(pressure)
            self._flow.append(flow)
            self._pid_rpm_cmd.append(pid_rpm_cmd)
            self._samples += 1
            if pid_rpm_cmd is None:
                self._last_line = f"T,{ms},{pressure:.2f},{flow:.2f}"
            else:
                self._last_line = f"T,{ms},{pressure:.2f},{flow:.2f},{pid_rpm_cmd}"
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
            self._append_sample(ms, pressure, flow, pid_rpm_cmd)
            return

        m = BENCH_RE.match(line)
        if m:
            ms = int(m.group(1))
            rpm_cmd = int(m.group(2))
            rpm_act = int(m.group(3))
            pressure = float(m.group(4))
            self._append_bench(ms, rpm_cmd, rpm_act, pressure)
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
    print("Expect lines like: T,12345,8.12,3.45,17500  or  R,12345,17500,17480,7.50")
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

    fig, (ax_p, ax_f) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.suptitle(f"CPAP telemetry — {port}")
    ax_p.set_ylabel("Pressure at blower (cmH2O)")
    ax_p.set_ylim(0, 25)
    ax_p.set_xlim(0, args.window)
    ax_p.grid(True, alpha=0.3)
    ax_rpm = ax_p.twinx()
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_ylim(0, 22000)
    ax_f.set_ylabel("Patient flow (slm)")
    ax_f.set_xlabel("Time (s)")
    ax_f.set_ylim(-30, 30)
    ax_f.set_xlim(0, args.window)
    ax_f.grid(True, alpha=0.3)

    line_p, = ax_p.plot([], [], "b-", linewidth=1.2, label="P_sensor")
    line_pid_rpm, = ax_rpm.plot(
        [], [], color="darkorange", linewidth=1.0, drawstyle="steps-post", label="PID RPM cmd"
    )
    line_rpm_cmd, = ax_rpm.plot(
        [], [], "x", color="saddlebrown", markersize=4, linestyle="none", label="bench RPM cmd"
    )
    line_rpm_act, = ax_rpm.plot(
        [], [], "o", color="crimson", markersize=4, linestyle="none", label="RPM act"
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

    def update(_frame: int):
        (
            times,
            pressure,
            flow,
            pid_rpm_cmd,
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
            rpm_axis_values = list(rpm_values) + bench_rpm_cmd + bench_rpm_act
            if rpm_axis_values:
                rpm_min = min(rpm_axis_values)
                rpm_max = max(rpm_axis_values)
                margin = max(500.0, (rpm_max - rpm_min) * 0.1)
                ax_rpm.set_ylim(max(0, rpm_min - margin), rpm_max + margin)
        else:
            line_pid_rpm.set_data([], [])
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
        return line_p, line_pid_rpm, line_rpm_cmd, line_rpm_act, line_f, waiting, status

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )
    _ = ani

    print(f"Plot running on {port}. Sent 'telem on' to the device.")
    print("PID RPM command overlays continuously on the pressure graph.")
    print("If the plot stays empty, run:  python cpap_telemetry_plot.py --port", port, "--monitor")
    print("Close the plot window to exit.")
    try:
        plt.show()
    finally:
        reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
