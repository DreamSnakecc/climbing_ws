#!/usr/bin/env python3
"""
Analyze crawl gait CSV: commanded vs actual foot positions.

Key insight: both cmd_xyz and ujc_xyz are in BODY frame.
- cmd_xyz: desired foot position relative to body CoM
- ujc_xyz: actual foot position from FK (also body frame)

There's a ~0.23m calibration offset (hip mounting radius + coxa) that is constant.
We subtract it to get meaningful tracking error.
"""
import csv
import math

csv_path = "test_logs/crawl_gait_20260507_143947.csv"
legs = ["lf", "rf", "rr", "lr"]

# ------------------------------------------------------------------ #
# Load data
# ------------------------------------------------------------------ #
rows = []
with open(csv_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

total = len(rows)
print(f"Samples: {total}, Duration: {float(rows[-1]['elapsed_s']):.1f}s")

# ------------------------------------------------------------------ #
# Calibration offset: use first few samples as baseline
# ------------------------------------------------------------------ #
# First sample values
sample0 = {}
for leg in legs:
    sample0[leg] = {
        "cmd_x": float(rows[0][f"{leg}_cmd_x"]),
        "cmd_y": float(rows[0][f"{leg}_cmd_y"]),
        "cmd_z": float(rows[0][f"{leg}_cmd_z"]),
        "ujc_x": float(rows[0][f"{leg}_ujc_x"]),
        "ujc_y": float(rows[0][f"{leg}_ujc_y"]),
        "ujc_z": float(rows[0][f"{leg}_ujc_z"]),
    }

print("\n--- INIT baseline (first sample, body already in CLIMB) ---")
print(f"{'Leg':>4} | {'cmd_x':>8} {'cmd_y':>8} {'cmd_z':>8} | "
      f"{'ujc_x':>8} {'ujc_y':>8} {'ujc_z':>8} | "
      f"{'offset_x':>8} {'offset_y':>8} {'offset_z':>8} {'offset_3d':>8}")
print("-" * 86)
for leg in legs:
    s = sample0[leg]
    dx = s["ujc_x"] - s["cmd_x"]
    dy = s["ujc_y"] - s["cmd_y"]
    dz = s["ujc_z"] - s["cmd_z"]
    d3d = math.sqrt(dx*dx + dy*dy + dz*dz)
    print(f"{leg:>4} | {s['cmd_x']:>8.4f} {s['cmd_y']:>8.4f} {s['cmd_z']:>8.4f} | "
          f"{s['ujc_x']:>8.4f} {s['ujc_y']:>8.4f} {s['ujc_z']:>8.4f} | "
          f"{dx:>8.3f} {dy:>8.3f} {dz:>8.3f} {d3d:>8.3f}")

# ------------------------------------------------------------------ #
# Phase transitions - identify full swing-support cycles per leg
# ------------------------------------------------------------------ #
print("\n--- Phase transitions per leg ---")
for leg in legs:
    prev_phase = None
    transitions = []
    for r in rows:
        phase = r[f"{leg}_phase"]
        if phase != prev_phase:
            if prev_phase is not None:
                transitions.append((len(transitions), prev_phase, float(r["elapsed_s"])))
            prev_phase = phase
    print(f"  {leg}: {len(transitions)} transitions, phases: {[t[1] for t in transitions[:10]]}")
    # Extract swing cycles
    swing_starts = []
    prev_support = None
    for r in rows:
        sup = float(r[f"{leg}_cmd_support"])
        if prev_support is not None and prev_support > 0.5 and sup < 0.5:
            swing_starts.append(float(r["elapsed_s"]))
        prev_support = sup
    print(f"  {leg}: {len(swing_starts)} swing cycles at times: {[f'{t:.1f}' for t in swing_starts]}")

# ------------------------------------------------------------------ #
# Tracking error during SWING (residual after calibration offset)
# ------------------------------------------------------------------ #
print("\n--- SWING PHASE: Residual tracking error (ujc - cmd - offset) ---")
print("  (offset subtracted per-leg to account for hip mounting calibration)")
for leg in legs:
    cal = sample0[leg]
    err_x, err_y, err_z, err_3d = [], [], [], []
    for r in rows:
        phase = r[f"{leg}_phase"]
        if phase != "LIFT_SWING":
            continue
        cmd_x = float(r[f"{leg}_cmd_x"])
        cmd_y = float(r[f"{leg}_cmd_y"])
        cmd_z = float(r[f"{leg}_cmd_z"])
        ujc_x = float(r[f"{leg}_ujc_x"])
        ujc_y = float(r[f"{leg}_ujc_y"])
        ujc_z = float(r[f"{leg}_ujc_z"])
        # Residual error: deviation from constant offset (ujc - cmd = constant if tracking perfectly)
        res_x = (ujc_x - cmd_x) - (cal["ujc_x"] - cal["cmd_x"])
        res_y = (ujc_y - cmd_y) - (cal["ujc_y"] - cal["cmd_y"])
        res_z = (ujc_z - cmd_z) - (cal["ujc_z"] - cal["cmd_z"])
        res_3d = math.sqrt(res_x*res_x + res_y*res_y + res_z*res_z)
        err_x.append(res_x); err_y.append(res_y); err_z.append(res_z); err_3d.append(res_3d)

    if err_3d:
        n = len(err_3d)
        rms_3d = math.sqrt(sum(v*v for v in err_3d)/n)
        max_3d = max(abs(v) for v in err_3d)
        rms_x = math.sqrt(sum(v*v for v in err_x)/n)
        rms_y = math.sqrt(sum(v*v for v in err_y)/n)
        rms_z = math.sqrt(sum(v*v for v in err_z)/n)
        print(f"  {leg}: n={n:4d} | dx_rms={rms_x*1000:6.1f}mm dy_rms={rms_y*1000:6.1f}mm "
              f"dz_rms={rms_z*1000:6.1f}mm | 3d_rms={rms_3d*1000:5.1f}mm 3d_max={max_3d*1000:5.1f}mm")
    else:
        print(f"  {leg}: no swing samples")

# ------------------------------------------------------------------ #
# SUPPORT phase: foot stability (should not move in world frame)
# ------------------------------------------------------------------ #
print("\n--- SUPPORT PHASE: Foot world-frame stability (slipping check) ---")
print("  For each support phase, check if ujc changes (indicating foot slip)")
for leg in legs:
    # Track ujc position during each continuous support period
    support_periods = []
    cur_start = None
    cur_first_ujc = None
    for r in rows:
        sup = float(r[f"{leg}_cmd_support"])
        if sup > 0.5:
            if cur_start is None:
                cur_start = float(r["elapsed_s"])
                cur_first_ujc = (float(r[f"{leg}_ujc_x"]), float(r[f"{leg}_ujc_y"]), float(r[f"{leg}_ujc_z"]))
        else:
            if cur_start is not None:
                support_periods.append((cur_start, float(r["elapsed_s"]), list(cur_first_ujc)))
                cur_start = None
                cur_first_ujc = None
    if cur_start is not None:
        support_periods.append((cur_start, float(rows[-1]["elapsed_s"]), list(cur_first_ujc)))

    if support_periods:
        max_drift = 0
        total_drift = 0
        count = 0
        for sp_start, sp_end, first_ujc in support_periods:
            # Find end of this support period
            for r in rows:
                t = float(r["elapsed_s"])
                sup = float(r[f"{leg}_cmd_support"])
                if t < sp_start or sup < 0.5:
                    continue
                if t > sp_end:
                    break
                # Compare current ujc to first ujc of support period
                dx = float(r[f"{leg}_ujc_x"]) - first_ujc[0]
                dy = float(r[f"{leg}_ujc_y"]) - first_ujc[1]
                dz = float(r[f"{leg}_ujc_z"]) - first_ujc[2]
                drift = math.sqrt(dx*dx + dy*dy + dz*dz)
                if drift > max_drift:
                    max_drift = drift
                total_drift += drift
                count += 1
        avg_drift = total_drift / count if count > 0 else 0
        print(f"  {leg}: {len(support_periods)} support periods | "
              f"avg_drift={avg_drift*1000:.1f}mm | max_drift={max_drift*1000:.1f}mm")
    else:
        print(f"  {leg}: no support phases")

# ------------------------------------------------------------------ #
# Body motion analysis
# ------------------------------------------------------------------ #
print("\n--- BODY MOTION ---")
body_x = [float(r["body_x"]) for r in rows]
body_y = [float(r["body_y"]) for r in rows]
body_z = [float(r["body_z"]) for r in rows]
body_vx = [float(r["body_vx"]) for r in rows]

t_start = float(rows[0]["elapsed_s"])
t_end = float(rows[-1]["elapsed_s"])
duration = t_end - t_start

dx = body_x[-1] - body_x[0]
dy = body_y[-1] - body_y[0]
dz = body_z[-1] - body_z[0]

avg_vx = sum(body_vx) / len(body_vx)
peak_vx = max(body_vx)
min_vx = min(body_vx)

print(f"  Body displacement: ({dx*1000:.1f}, {dy*1000:.1f}, {dz*1000:.1f}) mm over {duration:.1f}s")
print(f"  Avg velocity (from body_vx): {avg_vx*1000:.1f} mm/s")
print(f"  Peak body_vx: {peak_vx*1000:.1f} mm/s")
print(f"  Min body_vx:  {min_vx*1000:.1f} mm/s")
print(f"  Commanded velocity: 10 mm/s (from robot.yaml)")

# Estimate actual displacement from integrated body_vx
integrated_x = sum(body_vx) * (duration / len(body_vx))
print(f"  Integrated displacement: {integrated_x*1000:.1f} mm (from body_vx)")
print(f"  Actual displacement: {dx*1000:.1f} mm (from body_x)")

# ------------------------------------------------------------------ #
# Per-leg: cmd position changes during support (should track body motion)
# ------------------------------------------------------------------ #
print("\n--- SUPPORT PHASE: cmd position drift in body frame ---")
print("  During support, cmd position should change to keep foot on wall as body moves")
for leg in legs:
    support_cmd_x_start = []
    support_cmd_x_end = []
    prev_sup = False
    period_start = None
    period_start_cmd_x = None
    for r in rows:
        sup = float(r[f"{leg}_cmd_support"]) > 0.5
        if sup and not prev_sup:
            period_start = float(r["elapsed_s"])
            period_start_cmd_x = float(r[f"{leg}_cmd_x"])
        elif not sup and prev_sup and period_start is not None:
            support_cmd_x_start.append(period_start_cmd_x)
            support_cmd_x_end.append(float(r[f"{leg}_cmd_x"]))
            period_start = None
        prev_sup = sup
    if period_start is not None:
        support_cmd_x_start.append(period_start_cmd_x)
        support_cmd_x_end.append(float(rows[-1][f"{leg}_cmd_x"]))

    if support_cmd_x_start:
        avg_delta = sum(e - s for s, e in zip(support_cmd_x_start, support_cmd_x_end)) / len(support_cmd_x_start)
        print(f"  {leg}: {len(support_cmd_x_start)} support periods, "
              f"avg cmd_x change during support = {avg_delta*1000:.2f}mm")
    else:
        print(f"  {leg}: no support periods found")

# ------------------------------------------------------------------ #
# Summary
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("SUMMARY")
print("="*80)

# Body velocity analysis
low_vel_samples = sum(1 for v in body_vx if v < 0.005)
total_samples = len(body_vx)
print(f"1. Body velocity: avg={avg_vx*1000:.1f} mm/s (cmd=10 mm/s)")
print(f"   Body displacement during recording: {dx*1000:.1f}mm in {duration:.1f}s")
print(f"   Fraction of time body_vx < 5mm/s: {low_vel_samples/total_samples*100:.0f}%")

# Check if swing duration matches expectation
print(f"\n2. Gait: sequential, freq=0.25Hz, swing_ratio=0.25")
print(f"   Expected swing duration: {4*0.25/0.25:.1f}s per leg cycle")
print(f"   Expected swing time: {0.25/0.25:.1f}s per swing (at 0.25 Hz, swing_ratio=0.25)")

# Per-leg swing cycles from log output
print(f"\n3. Swing cycles completed: lf=4, rf=4, rr=4, lr=4 (from test log)")

# Check if support feet move (slip)
print(f"\n4. Maximum foot drift during support:")
print(f"   (Note: if robot is rigid and feet don't slip, drift should be near-zero)")
for leg in legs:
    max_drift = 0
    for r in rows:
        sup = float(r[f"{leg}_cmd_support"]) > 0.5
        if not sup:
            continue
        # Compare to first sample's ujc
        dx = float(r[f"{leg}_ujc_x"]) - float(rows[0][f"{leg}_ujc_x"])
        dy = float(r[f"{leg}_ujc_y"]) - float(rows[0][f"{leg}_ujc_y"])
        dz = float(r[f"{leg}_ujc_z"]) - float(rows[0][f"{leg}_ujc_z"])
        drift = math.sqrt(dx*dx + dy*dy + dz*dz)
        if drift > max_drift:
            max_drift = drift
    print(f"   {leg}: max drift from initial = {max_drift*1000:.1f}mm")
