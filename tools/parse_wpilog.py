#!/usr/bin/env python3
"""
Parse WPILog (.wpilog) files from an AdvantageKit robot.
Outputs CSV for use by compare_signals.py and Claude Code debugging loops.

Usage:
  python tools/parse_wpilog.py <logfile.wpilog> [options]

Options:
  --catalog                List all signal names in the log
  --signals NAME [NAME...] Filter to signals whose path contains any of these strings
  --start SECONDS          Only include records at or after this timestamp
  --end SECONDS            Only include records at or before this timestamp
  --anomalies              Print detected anomalies (velocity drops, brownouts, CAN spikes)
  --out FILE               Write CSV to file instead of stdout

Install dependency once:
  pip install robotpy-wpiutil
"""

import sys
import argparse
import csv
import struct
from pathlib import Path


def load_reader():
    try:
        from wpiutil.log import DataLogReader
        return DataLogReader
    except ImportError:
        print("ERROR: robotpy-wpiutil not installed. Run: pip install robotpy-wpiutil", file=sys.stderr)
        sys.exit(1)


def read_log(path: str):
    DataLogReader = load_reader()

    # First pass: build entry id -> name/type map
    entries = {}
    reader = DataLogReader(path)
    for record in reader:
        if record.isStart():
            info = record.getStartData()
            entries[info.entry] = {"name": info.name, "type": info.type}

    # Second pass: collect data records
    rows = []
    entries2 = {}
    reader2 = load_reader()(path)
    for record in reader2:
        if record.isStart():
            info = record.getStartData()
            entries2[info.entry] = {"name": info.name, "type": info.type}
            continue
        if record.isControl():
            continue

        entry_id = record.getEntry()
        if entry_id not in entries2:
            continue

        name = entries2[entry_id]["name"]
        dtype = entries2[entry_id]["type"]
        ts = record.getTimestamp() / 1_000_000.0  # microseconds -> seconds

        value = decode_value(record, dtype)
        if value is not None:
            rows.append({"timestamp": ts, "signal": name, "type": dtype, "value": value})

    return rows, entries


def decode_value(record, dtype: str):
    try:
        if dtype == "double":
            return record.getDouble()
        elif dtype == "float":
            return record.getFloat()
        elif dtype == "int64":
            return record.getInteger()
        elif dtype == "boolean":
            return int(record.getBoolean())
        elif dtype == "string":
            return record.getString()
        elif dtype == "double[]":
            return str(record.getDoubleArray())
        elif dtype == "float[]":
            return str(record.getFloatArray())
        elif dtype == "int64[]":
            return str(record.getIntegerArray())
        elif dtype == "boolean[]":
            return str(record.getBooleanArray())
        elif dtype.startswith("struct:") or dtype.startswith("struct[]:"):
            # AKit logs WPILib units as structs; extract first double best-effort
            raw = record.getRaw()
            if len(raw) >= 8:
                return struct.unpack_from('<d', raw, 0)[0]
            return None
        else:
            try:
                raw = record.getRaw()
                if len(raw) >= 8:
                    return struct.unpack_from('<d', raw, 0)[0]
            except Exception:
                pass
            return None
    except Exception:
        return None


def catalog(rows, entries):
    names = {}
    for row in rows:
        sig = row["signal"]
        if sig not in names:
            names[sig] = {"type": row["type"], "count": 0, "min": None, "max": None}
        names[sig]["count"] += 1
        try:
            v = float(row["value"])
            if names[sig]["min"] is None or v < names[sig]["min"]:
                names[sig]["min"] = v
            if names[sig]["max"] is None or v > names[sig]["max"]:
                names[sig]["max"] = v
        except (ValueError, TypeError):
            pass

    print(f"{'Signal':<70} {'Type':<25} {'Samples':>8} {'Min':>12} {'Max':>12}")
    print("-" * 135)
    for name in sorted(names):
        info = names[name]
        mn = f"{info['min']:.4f}" if info["min"] is not None else "—"
        mx = f"{info['max']:.4f}" if info["max"] is not None else "—"
        print(f"{name:<70} {info['type']:<25} {info['count']:>8} {mn:>12} {mx:>12}")


def filter_rows(rows, signals=None, start=None, end=None):
    result = []
    for row in rows:
        if start is not None and row["timestamp"] < start:
            continue
        if end is not None and row["timestamp"] > end:
            continue
        if signals:
            if not any(s.lower() in row["signal"].lower() for s in signals):
                continue
        result.append(row)
    return result


def detect_anomalies(rows):
    from collections import defaultdict
    by_signal = defaultdict(list)
    for row in rows:
        by_signal[row["signal"]].append(row)

    anomalies = []

    # Velocity drop detection
    for sig, recs in by_signal.items():
        if "velocity" not in sig.lower() and "rpm" not in sig.lower():
            continue
        prev = None
        for rec in sorted(recs, key=lambda r: r["timestamp"]):
            try:
                v = float(rec["value"])
            except (ValueError, TypeError):
                continue
            if prev is not None:
                drop = prev - v
                if drop > 10 and prev > 5:
                    anomalies.append({
                        "type": "velocity_drop",
                        "signal": sig,
                        "time": rec["timestamp"],
                        "from": prev,
                        "to": v,
                        "drop": drop,
                    })
            prev = v

    # Brownout detection — only voltage signals, not current
    for sig, recs in by_signal.items():
        sig_lower = sig.lower()
        if "batteryvoltage" not in sig_lower and "busvoltage" not in sig_lower:
            continue
        for rec in recs:
            try:
                v = float(rec["value"])
            except (ValueError, TypeError):
                continue
            if v < 7.0:
                anomalies.append({"type": "brownout", "signal": sig, "time": rec["timestamp"], "voltage": v})

    # CAN utilization spike
    for sig, recs in by_signal.items():
        if "canutilization" not in sig.lower() and "canbus" not in sig.lower():
            continue
        for rec in recs:
            try:
                v = float(rec["value"])
            except (ValueError, TypeError):
                continue
            if v > 0.85:
                anomalies.append({"type": "can_spike", "signal": sig, "time": rec["timestamp"], "utilization": v})

    # Current spike
    for sig, recs in by_signal.items():
        if "statorcurrent" not in sig.lower() and "supplycurrent" not in sig.lower():
            continue
        for rec in recs:
            try:
                v = float(rec["value"])
            except (ValueError, TypeError):
                continue
            if abs(v) > 60:
                anomalies.append({"type": "current_spike", "signal": sig, "time": rec["timestamp"], "current_A": v})

    return anomalies


def write_csv(rows, out_file=None):
    output = out_file if out_file else sys.stdout
    writer = csv.DictWriter(output, fieldnames=["timestamp", "signal", "type", "value"])
    writer.writeheader()
    for row in sorted(rows, key=lambda r: r["timestamp"]):
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Parse WPILog files for FRC robot debugging")
    parser.add_argument("logfile", help="Path to .wpilog file")
    parser.add_argument("--catalog", action="store_true", help="List all signals")
    parser.add_argument("--signals", nargs="+", metavar="NAME", help="Filter signals by substring")
    parser.add_argument("--start", type=float, metavar="SEC", help="Start time in seconds")
    parser.add_argument("--end", type=float, metavar="SEC", help="End time in seconds")
    parser.add_argument("--anomalies", action="store_true", help="Detect anomalies")
    parser.add_argument("--out", metavar="FILE", help="Output CSV file path")
    args = parser.parse_args()

    if not Path(args.logfile).exists():
        print(f"ERROR: File not found: {args.logfile}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading {args.logfile}...", file=sys.stderr)
    rows, entries = read_log(args.logfile)
    print(f"Loaded {len(rows)} records from {len(entries)} signals.", file=sys.stderr)

    if args.catalog:
        catalog(rows, entries)
        return

    if args.anomalies:
        filtered = filter_rows(rows, signals=args.signals, start=args.start, end=args.end)
        anomalies = detect_anomalies(filtered)
        if not anomalies:
            print("No anomalies detected.")
        else:
            print(f"\n=== {len(anomalies)} anomalies detected ===\n")
            for a in sorted(anomalies, key=lambda x: x["time"]):
                t = a["time"]
                atype = a["type"]
                sig = a["signal"]
                if atype == "velocity_drop":
                    print(f"  [{t:.3f}s] VELOCITY DROP  {sig}: {a['from']:.2f} -> {a['to']:.2f}  (drop={a['drop']:.2f})")
                elif atype == "brownout":
                    print(f"  [{t:.3f}s] BROWNOUT       {sig}: {a['voltage']:.2f}V")
                elif atype == "can_spike":
                    print(f"  [{t:.3f}s] CAN SPIKE      {sig}: {a['utilization']*100:.1f}%")
                elif atype == "current_spike":
                    print(f"  [{t:.3f}s] CURRENT SPIKE  {sig}: {a['current_A']:.1f}A")
        return

    filtered = filter_rows(rows, signals=args.signals, start=args.start, end=args.end)

    if args.out:
        with open(args.out, "w", newline="") as f:
            write_csv(filtered, f)
        print(f"Wrote {len(filtered)} records to {args.out}", file=sys.stderr)
    else:
        write_csv(filtered)


if __name__ == "__main__":
    main()
