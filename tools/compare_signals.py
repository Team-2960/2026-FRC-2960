#!/usr/bin/env python3
"""
Compare two WPILog CSV files (before/after a code fix) and report PASS/FAIL.

Exit code 0 = PASS (anomaly resolved, no regressions)
Exit code 1 = FAIL (anomaly still present or regression introduced)

Usage:
  # Compare two pre-extracted CSV files:
  python tools/compare_signals.py --before /tmp/before.csv --after /tmp/after.csv \
      --target-signals ShooterWheel/Velocity --regression-signals Drivetrain

  # Compare two raw .wpilog files directly (runs parse internally):
  python tools/compare_signals.py --before-log logs/match7.wpilog --after-log logs/replay.wpilog \
      --target-signals ShooterWheel --regression-signals Drivetrain

  # Output JSON result for scripted use:
  python tools/compare_signals.py ... --output-json /tmp/result.json
"""

import sys
import argparse
import csv
import json
import subprocess
from collections import defaultdict
from pathlib import Path


VELOCITY_DROP_THRESHOLD = 10.0     # RPS or RPM drop that counts as an anomaly
REGRESSION_TOLERANCE_PCT = 0.15    # 15% change in a regression signal counts as a regression


def load_csv(path: str) -> list:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                row["timestamp"] = float(row["timestamp"])
                row["value_f"] = float(row["value"])
            except (ValueError, KeyError):
                row["value_f"] = None
            rows.append(row)
    return rows


def csv_from_log(log_path: str, signals: list) -> str:
    import tempfile
    script = Path(__file__).parent / "parse_wpilog.py"
    out = tempfile.mktemp(suffix=".csv")
    cmd = [sys.executable, str(script), log_path, "--out", out]
    if signals:
        cmd += ["--signals"] + signals
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"ERROR parsing log: {result.stderr}", file=sys.stderr)
        sys.exit(1)
    return out


def group_by_signal(rows: list) -> dict:
    d = defaultdict(list)
    for row in rows:
        d[row["signal"]].append(row)
    return d


def count_drops(recs: list) -> int:
    drops = 0
    prev = None
    for rec in sorted(recs, key=lambda r: r["timestamp"]):
        v = rec.get("value_f")
        if v is None:
            continue
        if prev is not None and (prev - v) > VELOCITY_DROP_THRESHOLD and prev > 5:
            drops += 1
        prev = v
    return drops


def average_value(recs: list):
    vals = [r["value_f"] for r in recs if r.get("value_f") is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def compare(before_csv, after_csv, target_signals, regression_signals, output_json):
    print("Loading before CSV...", file=sys.stderr)
    before_rows = load_csv(before_csv)
    print("Loading after CSV...", file=sys.stderr)
    after_rows = load_csv(after_csv)

    before_by_sig = group_by_signal(before_rows)
    after_by_sig = group_by_signal(after_rows)

    issues = []
    passes = []

    # Check target signals: anomalies should be gone or reduced
    print("\n=== Target Signal Comparison ===")
    for sig_filter in (target_signals or []):
        matched = [s for s in set(list(before_by_sig) + list(after_by_sig))
                   if sig_filter.lower() in s.lower()]
        for sig in sorted(matched):
            b_drops = count_drops(before_by_sig.get(sig, []))
            a_drops = count_drops(after_by_sig.get(sig, []))
            if b_drops == 0 and a_drops == 0:
                print(f"  {sig}: no drops before or after  ✓")
                passes.append(f"{sig}: no drops")
            elif a_drops < b_drops:
                pct = (b_drops - a_drops) / max(b_drops, 1) * 100
                print(f"  {sig}: drops {b_drops} -> {a_drops}  ({pct:.0f}% reduction)  ✓")
                passes.append(f"{sig}: drops reduced {b_drops}->{a_drops}")
            elif a_drops == b_drops and b_drops > 0:
                print(f"  {sig}: drops unchanged at {b_drops}  ✗")
                issues.append(f"{sig}: anomaly not resolved (still {a_drops} drops)")
            else:
                print(f"  {sig}: drops increased {b_drops} -> {a_drops}  ✗")
                issues.append(f"{sig}: anomaly worsened ({b_drops}->{a_drops} drops)")

    # Check regression signals: averages should not shift significantly
    print("\n=== Regression Check ===")
    for sig_filter in (regression_signals or []):
        matched = [s for s in before_by_sig if sig_filter.lower() in s.lower()]
        for sig in sorted(matched):
            b_avg = average_value(before_by_sig[sig])
            a_avg = average_value(after_by_sig.get(sig, []))
            if b_avg is None or a_avg is None or abs(b_avg) < 0.001:
                continue
            change_pct = abs(a_avg - b_avg) / abs(b_avg)
            if change_pct > REGRESSION_TOLERANCE_PCT:
                print(f"  {sig}: {b_avg:.4f} -> {a_avg:.4f} ({change_pct*100:.1f}% change)  ✗ REGRESSION")
                issues.append(f"Regression in {sig}: {b_avg:.4f} -> {a_avg:.4f}")
            else:
                print(f"  {sig}: {b_avg:.4f} -> {a_avg:.4f} ({change_pct*100:.1f}%)  ✓")
                passes.append(f"{sig}: stable")

    # Summary
    passed = len(issues) == 0
    print("\n" + "=" * 50)
    if passed:
        print("RESULT: PASS ✓")
    else:
        print("RESULT: FAIL ✗")
        for issue in issues:
            print(f"  - {issue}")

    if output_json:
        with open(output_json, "w") as f:
            json.dump({"passed": passed, "issues": issues, "passes": passes}, f, indent=2)
        print(f"\nResult written to {output_json}", file=sys.stderr)

    return passed


def main():
    parser = argparse.ArgumentParser(description="Compare before/after WPILog data for FRC debugging")
    parser.add_argument("--before", metavar="CSV")
    parser.add_argument("--after", metavar="CSV")
    parser.add_argument("--before-log", metavar="WPILOG")
    parser.add_argument("--after-log", metavar="WPILOG")
    parser.add_argument("--target-signals", nargs="+", metavar="NAME")
    parser.add_argument("--regression-signals", nargs="+", metavar="NAME")
    parser.add_argument("--output-json", metavar="FILE")
    args = parser.parse_args()

    all_signals = (args.target_signals or []) + (args.regression_signals or [])
    before_csv = args.before
    after_csv = args.after

    if args.before_log:
        print(f"Parsing before log: {args.before_log}", file=sys.stderr)
        before_csv = csv_from_log(args.before_log, all_signals)
    if args.after_log:
        print(f"Parsing after log: {args.after_log}", file=sys.stderr)
        after_csv = csv_from_log(args.after_log, all_signals)

    if not before_csv or not after_csv:
        parser.error("Provide either --before/--after CSV files or --before-log/--after-log .wpilog files")

    for path in [before_csv, after_csv]:
        if not Path(path).exists():
            print(f"ERROR: File not found: {path}", file=sys.stderr)
            sys.exit(1)

    passed = compare(before_csv, after_csv,
                     args.target_signals or [],
                     args.regression_signals or [],
                     args.output_json)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
