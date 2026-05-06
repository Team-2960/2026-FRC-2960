#!/usr/bin/env python3
"""
Current limit optimizer for FRC Team 2960.

Analyzes one or more WPILog files and recommends optimized supply/stator
current limits per subsystem, based on actual draw during the enabled
portions of the match(es).

Supply limits are capped at SUPPLY_HARD_CAP_A (80A by default), which is
the recommended ceiling for sustained Kraken X60 draw on the FRC battery.

Usage:
  python tools/current_analysis.py logs/file.wpilog
  python tools/current_analysis.py logs/q14.wpilog logs/q34.wpilog --enabled-only
  python tools/current_analysis.py logs/*.wpilog --enabled-only --json out.json
"""

import sys
import argparse
import csv
import json
import math
import subprocess
import tempfile
import glob
from pathlib import Path
from collections import defaultdict


SUPPLY_HARD_CAP_A = 80
BROWNOUT_THRESHOLD_V = 6.75


def parse_log_to_csv(log_path, signals):
    script = Path(__file__).parent / "parse_wpilog.py"
    out = tempfile.mktemp(suffix=".csv")
    cmd = [sys.executable, str(script), log_path, "--out", out, "--signals"] + signals
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"ERROR parsing {log_path}: {result.stderr}", file=sys.stderr)
        return None
    return out


def load_csv_grouped(path):
    by_signal = defaultdict(list)
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                ts = float(row["timestamp"])
                v = float(row["value"])
            except (ValueError, KeyError):
                continue
            by_signal[row["signal"]].append((ts, v))
    for sig in by_signal:
        by_signal[sig].sort()
    return by_signal


def find_enabled_windows(by_signal):
    enabled = by_signal.get("/DriverStation/Enabled", [])
    if not enabled:
        return []
    windows = []
    on = None
    for ts, v in enabled:
        if v >= 0.5 and on is None:
            on = ts
        elif v < 0.5 and on is not None:
            windows.append((on, ts))
            on = None
    if on is not None:
        windows.append((on, enabled[-1][0]))
    return windows


def in_windows(ts, windows):
    if not windows:
        return True
    for s, e in windows:
        if s <= ts <= e:
            return True
    return False


def stats(values):
    if not values:
        return None
    vals = sorted(values)
    n = len(vals)
    def pct(p):
        idx = min(n - 1, int(math.ceil(p * n)) - 1)
        return vals[max(0, idx)]
    mean = sum(vals) / n
    rms = math.sqrt(sum(v*v for v in vals) / n)
    return {"n": n, "min": vals[0], "max": vals[-1], "mean": mean,
            "rms": rms, "p50": pct(0.50), "p95": pct(0.95), "p99": pct(0.99)}


def integrate_charge(samples):
    """Trapezoidal integration of |current| over time. Returns A*s."""
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(samples)):
        t0, v0 = samples[i-1]
        t1, v1 = samples[i]
        dt = t1 - t0
        if dt <= 0 or dt > 1.0:
            continue
        total += 0.5 * (abs(v0) + abs(v1)) * dt
    return total


def subsystem_name(signal):
    parts = signal.strip("/").split("/")
    if len(parts) >= 3:
        return parts[1]
    return parts[0] if parts else signal


def expand_log_paths(args_logs):
    expanded = []
    for p in args_logs:
        if any(c in p for c in "*?["):
            expanded.extend(sorted(glob.glob(p)))
        else:
            expanded.append(p)
    return [p for p in expanded if Path(p).exists()]


def main():
    parser = argparse.ArgumentParser(description="Recommend motor current limits from WPILog data")
    parser.add_argument("logfiles", nargs="+", help="One or more .wpilog files (glob OK)")
    parser.add_argument("--enabled-only", action="store_true",
                        help="Restrict analysis to robot-enabled windows only")
    parser.add_argument("--json", metavar="FILE", help="Write JSON report")
    parser.add_argument("--supply-cap", type=int, default=SUPPLY_HARD_CAP_A,
                        help=f"Hard cap for supply current limit (default {SUPPLY_HARD_CAP_A}A)")
    args = parser.parse_args()

    log_paths = expand_log_paths(args.logfiles)
    if not log_paths:
        print("ERROR: no log files found", file=sys.stderr)
        sys.exit(1)

    print(f"Analyzing {len(log_paths)} log file(s):", file=sys.stderr)
    for p in log_paths:
        print(f"  - {p}", file=sys.stderr)

    signal_filters = ["StatorCurrent", "SupplyCurrent",
                      "DriverStation/Enabled", "BatteryVoltage"]

    # Aggregate samples across all logs
    per_subsystem = defaultdict(lambda: {"stator": [], "supply": []})
    all_battery_voltages = []
    total_enabled_time = 0.0
    log_summaries = []

    for log_path in log_paths:
        csv_path = parse_log_to_csv(log_path, signal_filters)
        if csv_path is None:
            continue
        by_signal = load_csv_grouped(csv_path)
        windows = find_enabled_windows(by_signal) if args.enabled_only else []
        if args.enabled_only and not windows:
            print(f"  WARN: no enabled windows in {log_path}, skipping", file=sys.stderr)
            continue

        log_min_v = None
        for sig, recs in by_signal.items():
            if windows:
                recs = [(t, v) for t, v in recs if in_windows(t, windows)]
            if sig == "/SystemStats/BatteryVoltage":
                vs = [v for _, v in recs]
                all_battery_voltages.extend(vs)
                if vs:
                    log_min_v = min(vs)
                continue
            if sig.endswith("/StatorCurrent"):
                kind = "stator"
            elif sig.endswith("/SupplyCurrent"):
                kind = "supply"
            else:
                continue
            sub = subsystem_name(sig)
            per_subsystem[sub][kind].append(recs)

        if windows:
            t = sum(e - s for s, e in windows)
            total_enabled_time += t
            log_summaries.append({"file": Path(log_path).name,
                                  "enabled_s": t, "min_v": log_min_v})

    # Battery report
    print()
    print("=" * 78)
    print("CURRENT LIMIT ANALYSIS — aggregated across all logs")
    print("=" * 78)
    print(f"Total enabled time analyzed: {total_enabled_time:.1f}s")
    if all_battery_voltages:
        bv = stats(all_battery_voltages)
        print(f"Battery: min {bv['min']:.2f}V, mean {bv['mean']:.2f}V, "
              f"p5 {sorted(all_battery_voltages)[len(all_battery_voltages)//20]:.2f}V")
        if bv["min"] < BROWNOUT_THRESHOLD_V:
            print(f"  !! BROWNOUT detected — battery dropped to {bv['min']:.2f}V "
                  f"(threshold {BROWNOUT_THRESHOLD_V}V)")
        elif bv["min"] < 9.0:
            print(f"  !! Battery sagged to {bv['min']:.2f}V — at risk of brownout")

    print()
    print("Per-match minimum battery voltage:")
    for s in log_summaries:
        flag = "  !! BROWNOUT" if (s["min_v"] is not None and s["min_v"] < BROWNOUT_THRESHOLD_V) \
               else "  !! risk" if (s["min_v"] is not None and s["min_v"] < 9.0) \
               else ""
        v = f"{s['min_v']:.2f}V" if s['min_v'] is not None else "—"
        print(f"  {s['file']:<55} {v}{flag}")

    print()
    print(f"{'Subsystem':<22} {'Type':<8} {'Peak':>7} {'p99':>7} {'p95':>7} "
          f"{'RMS':>7} {'Mean':>7} {'A*s':>9}")
    print("-" * 78)

    report = {}
    charge_breakdown = []
    total_charge = 0.0
    for sub in sorted(per_subsystem):
        report[sub] = {}
        for kind in ("stator", "supply"):
            recs_list = per_subsystem[sub][kind]
            if not recs_list:
                continue
            # Concatenate per-log windows; integrate each separately so the
            # gap between matches doesn't get integrated.
            per_log_charge = sum(integrate_charge(r) for r in recs_list)
            all_values = [abs(v) for r in recs_list for _, v in r]
            s = stats(all_values)
            if s:
                print(f"{sub:<22} {kind:<8} "
                      f"{s['max']:>7.1f} {s['p99']:>7.1f} {s['p95']:>7.1f} "
                      f"{s['rms']:>7.1f} {s['mean']:>7.1f} {per_log_charge:>9.1f}")
                report[sub][kind] = {**s, "charge_As": per_log_charge}
                if kind == "supply":
                    charge_breakdown.append((sub, per_log_charge))
                    total_charge += per_log_charge

    # Recommendations
    print()
    print("=" * 78)
    print(f"RECOMMENDED LIMITS (supply hard-capped at {args.supply_cap}A)")
    print("=" * 78)
    print(f"{'Subsystem':<22} {'StatorLimit':>12} {'SupplyLimit':>12}  Notes")
    print("-" * 78)

    recommendations = {}
    for sub in sorted(per_subsystem):
        stator = report.get(sub, {}).get("stator")
        supply = report.get(sub, {}).get("supply")
        notes = []

        stator_limit = None
        if stator and stator["p99"] > 0:
            stator_limit = int(math.ceil(stator["p99"] * 1.10 / 5) * 5)

        supply_limit = None
        if supply and supply["p95"] > 0:
            raw = math.ceil(supply["p95"] * 1.20 / 5) * 5
            supply_limit = min(args.supply_cap, int(raw))
            if raw > args.supply_cap:
                notes.append(f"data wanted {int(raw)}A, capped at {args.supply_cap}A")
            if supply["p99"] > args.supply_cap:
                notes.append(f"will throttle: p99 draw is {supply['p99']:.0f}A")

        recommendations[sub] = {"stator_A": stator_limit, "supply_A": supply_limit}
        st = f"{stator_limit}A" if stator_limit else "—"
        sp = f"{supply_limit}A" if supply_limit else "—"
        print(f"{sub:<22} {st:>12} {sp:>12}  {'; '.join(notes)}")

    # Battery budget
    print()
    print("=" * 78)
    print("BATTERY BUDGET (supply A*s, ranked) — visible signals only")
    print("=" * 78)
    if total_charge > 0:
        for sub, ch in sorted(charge_breakdown, key=lambda x: -x[1]):
            pct = ch / total_charge * 100
            bar = "#" * int(pct / 2)
            print(f"  {sub:<22} {ch:>9.1f} A*s  ({pct:>5.1f}%)  {bar}")
        print(f"  {'TOTAL':<22} {total_charge:>9.1f} A*s")
        print()
        print("  NOTE: drivetrain (8 motors), IntakeRoller, IntakeAngle, ShooterHood,")
        print("        and Climber don't log current — total robot draw is much higher.")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({
                "log_summaries": log_summaries,
                "subsystems": report,
                "recommendations": recommendations,
                "supply_cap_A": args.supply_cap,
                "total_visible_charge_As": total_charge,
            }, f, indent=2)
        print(f"\nJSON report written to {args.json}", file=sys.stderr)


if __name__ == "__main__":
    main()
