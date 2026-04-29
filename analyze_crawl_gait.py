#!/usr/bin/env python3
"""
Analyze crawl gait CSV log: IK jumps, body displacement, swing sequence, phase counts.
"""
import csv
import sys
from collections import defaultdict

CSV_PATH = "/home/cc/climbing_ws/test_logs/crawl_gait_20260429_202915.csv"

# Phase ID to name mapping (from swing_leg_controller.py)
PHASE_NAMES = {
    0: "SUPPORT",
    1: "TEST_LIFT_CLEARANCE",
    2: "TEST_PRESS_CONTACT",
    3: "DETACH_SLIDE",
    4: "TANGENTIAL_ALIGN",
    5: "PRELOAD_OR_COMPLIANT",  # observed combined label in CSV
    6: "COMPLIANT_SETTLE",
    7: "ATTACHED_HOLD",
}

LEGS = ["lf", "rf", "rr", "lr"]
# Columns per leg (0-indexed in cmd suffix order)
LEG_CMD_COLS = {}  # leg -> (col_cmd_x, col_cmd_y, col_cmd_z)
LEG_PHASE_COL = {}
LEG_SUPPORT_COL = {}

def parse_csv(path):
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            rows.append(row)
    print(f"Read {len(rows)} data rows (plus header).")
    return rows


def analyze_ik_jumps(rows):
    """
    Per leg: compute per-sample change in cmd_x, cmd_y, cmd_z.
    Flag any change > 0.01m as a "jump" (potential IK branch switch).
    """
    print("=" * 70)
    print("1. IK BRANCH SWITCHING DETECTION")
    print("=" * 70)
    total_jumps = 0
    leg_results = {}
    for leg in LEGS:
        jumps = []
        prev = None
        for idx, row in enumerate(rows):
            try:
                cur = (
                    float(row[f"{leg}_cmd_x"]),
                    float(row[f"{leg}_cmd_y"]),
                    float(row[f"{leg}_cmd_z"]),
                )
            except (ValueError, KeyError):
                continue
            if prev is not None:
                dx = abs(cur[0] - prev[0])
                dy = abs(cur[1] - prev[1])
                dz = abs(cur[2] - prev[2])
                max_delta = max(dx, dy, dz)
                if max_delta > 0.01:
                    jumps.append({
                        "row": idx,
                        "elapsed": float(row["elapsed_s"]),
                        "prev": prev,
                        "cur": cur,
                        "dx": dx,
                        "dy": dy,
                        "dz": dz,
                        "max_delta": max_delta,
                        "phase_before": int(float(row.get(f"{leg}_phase_id", -1))),
                    })
            prev = cur

        leg_results[leg] = jumps
        total_jumps += len(jumps)

        # Summarize
        max_mag = max((j["max_delta"] for j in jumps), default=0.0)
        print(f"\n  Leg {leg.upper()}: {len(jumps)} jumps detected "
              f"(max magnitude {max_mag*1000:.1f} mm)")
        if jumps:
            print(f"    Jumps at elapsed times (s):")
            for j in jumps:
                print(f"      t={j['elapsed']:.3f}s  "
                      f"drift=({j['dx']*1000:.1f}, {j['dy']*1000:.1f}, {j['dz']*1000:.1f}) mm  "
                      f"max={j['max_delta']*1000:.1f} mm  "
                      f"phase_before={j['phase_before']} ({PHASE_NAMES.get(j['phase_before'],'?')})")
    print(f"\n  TOTAL jumps across all legs: {total_jumps}")
    print(f"  Comparison (pre-fix): ~15 jumps across 4 legs, magnitudes up to 0.050 m.")
    return leg_results


def analyze_body_displacement(rows):
    """Body X displacement: start vs end, range."""
    print("\n" + "=" * 70)
    print("2. BODY DISPLACEMENT")
    print("=" * 70)
    body_x_vals = []
    for row in rows:
        try:
            body_x_vals.append(float(row["body_x"]))
        except (ValueError, KeyError):
            continue

    start_x = body_x_vals[0]
    end_x = body_x_vals[-1]
    min_x = min(body_x_vals)
    max_x = max(body_x_vals)
    displacement = end_x - start_x

    print(f"  body_x at start: {start_x*1000:.2f} mm")
    print(f"  body_x at end:   {end_x*1000:.2f} mm")
    span = (max_x - min_x) * 1000
    print(f"  body_x range:    [{min_x*1000:.2f}, {max_x*1000:.2f}] mm  (span={span:.2f} mm)")
    print(f"  Net displacement: {displacement*1000:.2f} mm ({displacement*1000:.2f} mm per ~{len(rows)/50:.0f}s of logging)")
    # Also check body_y, body_z
    body_y_vals = [float(r["body_y"]) for r in rows if "body_y" in r]
    body_z_vals = [float(r["body_z"]) for r in rows if "body_z" in r]
    if body_y_vals:
        print(f"  body_y range: [{min(body_y_vals)*1000:.2f}, {max(body_y_vals)*1000:.2f}] mm")
    if body_z_vals:
        print(f"  body_z range: [{min(body_z_vals)*1000:.2f}, {max(body_z_vals)*1000:.2f}] mm")


def analyze_swing_sequence(rows):
    """
    Determine swing order: which legs leave SUPPORT (phase_id=0) and in what order.
    Track phase_id transitions per leg.
    """
    print("\n" + "=" * 70)
    print("3. SWING SEQUENCE")
    print("=" * 70)

    # Track phase transitions per leg
    transitions = {leg: [] for leg in LEGS}
    prev_phase = {leg: None for leg in LEGS}

    for idx, row in enumerate(rows):
        elapsed = float(row["elapsed_s"])
        for leg in LEGS:
            try:
                phase_id = int(float(row[f"{leg}_phase_id"]))
            except (ValueError, KeyError):
                continue
            if prev_phase[leg] is not None and prev_phase[leg] != phase_id:
                transitions[leg].append({
                    "row": idx,
                    "elapsed": elapsed,
                    "from": prev_phase[leg],
                    "to": phase_id,
                })
            prev_phase[leg] = phase_id

    # Also track the actual swing cycle: when does each leg first enter a non-SUPPORT phase
    # and what order do they swing in?
    swing_cycles = []
    current_cycle_swing_start = {}

    for idx, row in enumerate(rows):
        elapsed = float(row["elapsed_s"])
        phases_this_sample = {}
        for leg in LEGS:
            try:
                phases_this_sample[leg] = int(float(row[f"{leg}_phase_id"]))
            except (ValueError, KeyError):
                phases_this_sample[leg] = -1

        # Check which legs are in non-SUPPORT this sample
        swinging = {leg: pid for leg, pid in phases_this_sample.items() if pid != 0}
        if swinging:
            for leg, pid in swinging.items():
                if leg not in current_cycle_swing_start:
                    current_cycle_swing_start[leg] = {
                        "start_time": elapsed,
                        "phase": pid,
                    }

        # Check if all swinging legs returned to SUPPORT
        if current_cycle_swing_start:
            all_returned = all(
                phases_this_sample.get(leg, 0) == 0
                for leg in current_cycle_swing_start
            )
            if all_returned:
                swing_cycles.append(current_cycle_swing_start)
                current_cycle_swing_start = {}

    # Print transitions per leg
    for leg in LEGS:
        if transitions[leg]:
            print(f"\n  Leg {leg.upper()} transitions ({len(transitions[leg])} total):")
            for t in transitions[leg][:10]:
                print(f"    t={t['elapsed']:.3f}s: {PHASE_NAMES.get(t['from'], '?')}({t['from']}) -> {PHASE_NAMES.get(t['to'], '?')}({t['to']})")
            if len(transitions[leg]) > 10:
                print(f"    ... ({len(transitions[leg]) - 10} more transitions)")
        else:
            print(f"\n  Leg {leg.upper()}: NO transitions (always SUPPORT)")

    # Print swing cycles
    print(f"\n  Swing cycles detected: {len(swing_cycles)}")
    for i, cycle in enumerate(swing_cycles):
        legs_ordered = sorted(cycle.keys(), key=lambda l: cycle[l]["start_time"])
        print(f"    Cycle {i+1}: swing order = {[l.upper() for l in legs_ordered]}")
        for leg in legs_ordered:
            info = cycle[leg]
            print(f"      {leg.upper()}: starts at t={info['start_time']:.3f}s, phase={info['phase']}")

    # If no cycles detected (all SUPPORT all the time), report that
    if not swing_cycles:
        print("  (no complete swing cycles detected - legs may be in SUPPORT throughout)")


def analyze_phase_counts(rows):
    """Count occurrences of each phase_id per leg."""
    print("\n" + "=" * 70)
    print("4. PHASE ANALYSIS")
    print("=" * 70)

    for leg in LEGS:
        counts = defaultdict(int)
        for row in rows:
            try:
                pid = int(float(row[f"{leg}_phase_id"]))
            except (ValueError, KeyError):
                continue
            counts[pid] += 1

        total = sum(counts.values())
        print(f"\n  Leg {leg.upper()}: {total} samples")
        for pid in sorted(counts.keys()):
            pct = counts[pid] / total * 100 if total else 0
            name = PHASE_NAMES.get(pid, "UNKNOWN")
            print(f"    Phase {pid} ({name}): {counts[pid]:5d} samples ({pct:5.1f}%)")

        # Check for DETACH_SLIDE specifically
        detach_count = counts.get(1, 0)  # User asked about DETACH_SLIDE(1) but let's check both 1 and 3
        detach_count_3 = counts.get(3, 0)
        test_lift_count = counts.get(1, 0)
        if detach_count_3 > 0:
            print(f"    *** DETACH_SLIDE (phase_id=3) entered {detach_count_3} times ***")
        elif test_lift_count > 0:
            print(f"    *** TEST_LIFT_CLEARANCE (phase_id=1) entered {test_lift_count} times ***")
        else:
            print(f"    DETACH_SLIDE (phase_id=3): NEVER entered (0 samples)")
            print(f"    TEST_LIFT_CLEARANCE (phase_id=1): NEVER entered (0 samples)")

    # Quick check: was the robot ever outside SUPPORT at all?
    non_support_counts = defaultdict(int)
    for row in rows:
        elapsed = float(row["elapsed_s"])
        for leg in LEGS:
            try:
                pid = int(float(row[f"{leg}_phase_id"]))
            except (ValueError, KeyError):
                continue
            if pid not in (0,):
                non_support_counts[leg] += 1

    print(f"\n  Summary - non-SUPPORT samples per leg:")
    for leg in LEGS:
        print(f"    {leg.upper()}: {non_support_counts.get(leg, 0)} samples outside SUPPORT")


def main():
    rows = parse_csv(CSV_PATH)
    if not rows:
        print("No data rows found. Exiting.")
        sys.exit(1)

    # 1. IK branch switching
    jumps = analyze_ik_jumps(rows)

    # 2. Body displacement
    analyze_body_displacement(rows)

    # 3. Swing sequence
    analyze_swing_sequence(rows)

    # 4. Phase analysis
    analyze_phase_counts(rows)

    print("\n" + "=" * 70)
    print("DONE")
    print("=" * 70)


if __name__ == "__main__":
    main()
