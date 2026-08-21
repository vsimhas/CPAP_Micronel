"""Inspect a SomnaLink session EDF: header fields and decoded signal values.

Usage:
    python edf_inspect.py 26082100.EDF
    python edf_inspect.py 26082100.EDF --dump TargetP --seconds 5

Prints the raw header exactly as stored, then decodes each signal with the EDF
formula so firmware scaling can be verified independently of any viewer.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Signal:
    label: str
    transducer: str
    dimension: str
    phys_min: float
    phys_max: float
    dig_min: int
    dig_max: int
    prefilter: str
    samples_per_record: int

    @property
    def gain(self) -> float:
        span = self.dig_max - self.dig_min
        if span == 0:
            return 0.0
        return (self.phys_max - self.phys_min) / span

    def to_physical(self, raw: int) -> float:
        return self.phys_min + (raw - self.dig_min) * self.gain


@dataclass
class EdfFile:
    version: str
    patient: str
    recording: str
    start_date: str
    start_time: str
    header_bytes: int
    reserved: str
    n_records: int
    record_duration: float
    signals: list[Signal]
    raw_records: list[list[list[int]]]

    @property
    def bytes_per_record(self) -> int:
        return sum(s.samples_per_record for s in self.signals) * 2


def _ascii(blob: bytes) -> str:
    return blob.decode("ascii", errors="replace").strip()


def _num(blob: bytes, cast, field: str):
    text = _ascii(blob)
    try:
        return cast(text)
    except ValueError as exc:
        raise ValueError(f"header field {field!r} is not numeric: {text!r}") from exc


def read_edf(path: Path) -> EdfFile:
    data = path.read_bytes()
    if len(data) < 256:
        raise ValueError(f"file is only {len(data)} bytes; EDF needs at least 256")

    version = _ascii(data[0:8])
    patient = _ascii(data[8:88])
    recording = _ascii(data[88:168])
    start_date = _ascii(data[168:176])
    start_time = _ascii(data[176:184])
    header_bytes = _num(data[184:192], int, "header_bytes")
    reserved = _ascii(data[192:236])
    n_records = _num(data[236:244], int, "n_records")
    record_duration = _num(data[244:252], float, "record_duration")
    nsig = _num(data[252:256], int, "nsig")

    if header_bytes != 256 + 256 * nsig:
        print(
            f"WARNING: header_bytes={header_bytes} but {nsig} signals imply "
            f"{256 + 256 * nsig}",
            file=sys.stderr,
        )
    if len(data) < header_bytes:
        raise ValueError("file is truncated inside the header")

    def block(offset: int, width: int) -> list[bytes]:
        base = 256 + offset * nsig
        return [data[base + i * width : base + (i + 1) * width] for i in range(nsig)]

    labels = block(0, 16)
    transducers = block(16, 80)
    dimensions = block(16 + 80, 8)
    phys_mins = block(16 + 80 + 8, 8)
    phys_maxs = block(16 + 80 + 8 + 8, 8)
    dig_mins = block(16 + 80 + 8 + 8 + 8, 8)
    dig_maxs = block(16 + 80 + 8 + 8 + 8 + 8, 8)
    prefilters = block(16 + 80 + 8 + 8 + 8 + 8 + 8, 80)
    counts = block(16 + 80 + 8 + 8 + 8 + 8 + 8 + 80, 8)

    signals = [
        Signal(
            label=_ascii(labels[i]),
            transducer=_ascii(transducers[i]),
            dimension=_ascii(dimensions[i]),
            phys_min=_num(phys_mins[i], float, f"phys_min[{i}]"),
            phys_max=_num(phys_maxs[i], float, f"phys_max[{i}]"),
            dig_min=_num(dig_mins[i], int, f"dig_min[{i}]"),
            dig_max=_num(dig_maxs[i], int, f"dig_max[{i}]"),
            prefilter=_ascii(prefilters[i]),
            samples_per_record=_num(counts[i], int, f"samples[{i}]"),
        )
        for i in range(nsig)
    ]

    per_record = sum(s.samples_per_record for s in signals) * 2
    available = (len(data) - header_bytes) // per_record if per_record else 0
    if n_records < 0:
        print(
            f"WARNING: n_records is {n_records} (never finalized); using "
            f"{available} records found in the file",
            file=sys.stderr,
        )
        n_records = available
    elif n_records != available:
        print(
            f"WARNING: header says {n_records} records but the file holds "
            f"{available}",
            file=sys.stderr,
        )
        n_records = min(n_records, available)

    raw_records: list[list[list[int]]] = []
    pos = header_bytes
    for _ in range(n_records):
        record = []
        for sig in signals:
            count = sig.samples_per_record
            chunk = data[pos : pos + count * 2]
            record.append(list(struct.unpack(f"<{count}h", chunk)))
            pos += count * 2
        raw_records.append(record)

    return EdfFile(
        version=version,
        patient=patient,
        recording=recording,
        start_date=start_date,
        start_time=start_time,
        header_bytes=header_bytes,
        reserved=reserved,
        n_records=n_records,
        record_duration=record_duration,
        signals=signals,
        raw_records=raw_records,
    )


def summarize(edf: EdfFile, path: Path) -> None:
    size = path.stat().st_size
    print(f"File            : {path}")
    print(f"Size            : {size} bytes")
    print(f"Version         : {edf.version!r}")
    print(f"Patient         : {edf.patient!r}")
    print(f"Recording       : {edf.recording!r}")
    print(f"Start           : {edf.start_date} {edf.start_time}")
    print(f"Header bytes    : {edf.header_bytes}")
    print(f"Reserved        : {edf.reserved!r} ({'EDF+' if edf.reserved else 'classic EDF'})")
    print(f"Data records    : {edf.n_records} x {edf.record_duration} s")
    print(f"Bytes/record    : {edf.bytes_per_record}")
    expected = edf.header_bytes + edf.n_records * edf.bytes_per_record
    flag = "OK" if expected == size else f"MISMATCH (expected {expected})"
    print(f"Size check      : {flag}")
    print()

    header = (
        f"{'Signal':<10} {'Unit':<7} {'PhysMin':>9} {'PhysMax':>9} "
        f"{'DigMin':>7} {'DigMax':>7} {'Hz':>4} "
        f"{'min':>9} {'mean':>9} {'max':>9}"
    )
    print(header)
    print("-" * len(header))

    for idx, sig in enumerate(edf.signals):
        raws = [v for rec in edf.raw_records for v in rec[idx]]
        if raws:
            values = [sig.to_physical(v) for v in raws]
            lo, hi = min(values), max(values)
            mean = sum(values) / len(values)
            stats = f"{lo:9.3f} {mean:9.3f} {hi:9.3f}"
        else:
            stats = f"{'-':>9} {'-':>9} {'-':>9}"
        rate = sig.samples_per_record / edf.record_duration if edf.record_duration else 0
        print(
            f"{sig.label:<10} {sig.dimension:<7} {sig.phys_min:9.2f} "
            f"{sig.phys_max:9.2f} {sig.dig_min:7d} {sig.dig_max:7d} "
            f"{rate:4.0f} {stats}"
        )


def dump(edf: EdfFile, label: str, seconds: float) -> None:
    matches = [i for i, s in enumerate(edf.signals) if s.label.lower() == label.lower()]
    if not matches:
        available = ", ".join(s.label for s in edf.signals)
        raise SystemExit(f"no signal named {label!r}; available: {available}")

    idx = matches[0]
    sig = edf.signals[idx]
    limit = max(1, int(seconds / edf.record_duration)) if edf.record_duration else 1

    print()
    print(f"First {min(limit, edf.n_records)} record(s) of {sig.label} [{sig.dimension}]")
    print(f"gain = ({sig.phys_max} - {sig.phys_min}) / ({sig.dig_max} - {sig.dig_min})"
          f" = {sig.gain:.9f}")
    for r, record in enumerate(edf.raw_records[:limit]):
        t0 = r * edf.record_duration
        pairs = ", ".join(
            f"{raw}->{sig.to_physical(raw):.3f}" for raw in record[idx][:10]
        )
        more = " ..." if len(record[idx]) > 10 else ""
        print(f"  t={t0:6.1f}s  {pairs}{more}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="EDF file to inspect")
    parser.add_argument("--dump", metavar="LABEL", help="print raw->physical samples")
    parser.add_argument(
        "--seconds", type=float, default=2.0, help="seconds to dump (default 2)"
    )
    args = parser.parse_args()

    if not args.path.is_file():
        raise SystemExit(f"no such file: {args.path}")

    edf = read_edf(args.path)
    summarize(edf, args.path)
    if args.dump:
        dump(edf, args.dump, args.seconds)


if __name__ == "__main__":
    main()
