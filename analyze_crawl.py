#!/usr/bin/env python3
"""Analyze commanded vs actual foot positions from crawl gait test CSV."""
import csv
import sys
from collections import defaultdict
import math

csv_path = "test_logs/crawl_gait_20260507_143947.csv"

# Check header to ensure column names match
with open(csv_path) as f:
    header = f.readline().strip().split(",")
    print(f"CSV columns: {len(header)}")
    # Verify key columns exist
    for leg in ["lf", "rf", "rr", "lr"]:
        assert f"{leg}_cmd_support" in header, f"Missing {leg}_cmd_support"
        assert f"{leg}_cmd_x" in header, f"Missing {leg}_cmd_x"

legs = ["lf", "rf", "rr", "lr"]
phases = ["SUPPORT", "LIFT_SWING", "ADMIT_OR_PRELOAD"]

# Per-leg stats
leg_errors = {leg: {"dx": [], "dy": [], "dz": [], "d3d": [], "phase": []} for leg in legs}
support_errors = {leg: {"dx": [], "dy": [], "dz": [], "d3d": []} for leg in legs}
swing_errors = {leg: {"dx": [], "dy": [], "dz": [], "d3d": []} for leg in legs}

body_x_vals = []
body_vx_vals = []
total_duration = 0

with open(csv_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t = float(row["elapsed_s"])
        total_duration = max(total_duration, t)
        body_x_vals.append(float(row["body_x"]))
        body_vx_vals.append(float(row["body_vx"]))

        for leg in legs:
            cmd_x = float(row[f"{leg}_cmd_x"])
            cmd_y = float(row[f"{leg}_cmd_y"])
            cmd_z = float(row[f"{leg}_cmd_z"])
            ujc_x = float(row[f"{leg}_ujc_x"])
            ujc_y = float(row[f"{leg}_ujc_y"])
            ujc_z = float(row[f"{leg}_ujc_z"])
            phase = row[f"{leg}_phase"]
            support_val = float(row[f"{leg}_cmd_support"])
            support = abs(support_val) > 0.5

            dx = ujc_x - cmd_x
            dy = ujc_y - cmd_y
            dz = ujc_z - cmd_z
            d3d = math.sqrt(dx*dx + dy*dy + dz*dz)

            leg_errors[leg]["dx"].append(dx)
            leg_errors[leg]["dy"].append(dy)
            leg_errors[leg]["dz"].append(dz)
            leg_errors[leg]["d3d"].append(d3d)
            leg_errors[leg]["phase"].append(phase)

            if support:
                support_errors[leg]["dx"].append(dx)
                support_errors[leg]["dy"].append(dy)
                support_errors[leg]["dz"].append(dz)
                support_errors[leg]["d3d"].append(d3d)
            else:
                swing_errors[leg]["dx"].append(dx)
                swing_errors[leg]["dy"].append(dy)
                swing_errors[leg]["dz"].append(dz)
                swing_errors[leg]["d3d"].append(d3d)

def stats(vals):
    if not vals:
        return {"mean": 0, "rms": 0, "max_abs": 0, "std": 0}
    n = len(vals)
    mean = sum(vals) / n
    rms = math.sqrt(sum(v*v for v in vals) / n)
    max_abs = max(abs(v) for v in vals)
    std = math.sqrt(sum((v - mean)**2 for v in vals) / n)
    return {"mean": mean, "rms": rms, "max_abs": max_abs, "std": std}

print("=" * 80)
print("CRAWL GAIT ANALYSIS: Commanded vs Actual Foot Positions")
print(f"File: {csv_path}")
print(f"Duration: {total_duration:.1f}s, Samples: {len(body_x_vals)}")
print(f"Body displacement: {body_x_vals[-1] - body_x_vals[0]:.4f}m ({(body_x_vals[-1] - body_x_vals[0])*1000:.1f}mm)")
print(f"Nominal body vel: {round(sum(body_vx_vals)/len(body_vx_vals), 4)} m/s")
print("=" * 80)

# ============ Section 1: All samples per leg ============
print("\n--- ALL SAMPLES: Per-leg XYZ error (UJC - CMD) ---")
print(f"{'Leg':>4} | {'dx_mean':>8} {'dx_rms':>8} {'dx_max':>8} | "
      f"{'dy_mean':>8} {'dy_rms':>8} {'dy_max':>8} | "
      f"{'dz_mean':>8} {'dz_rms':>8} {'dz_max':>8} | "
      f"{'3d_rms':>8} {'3d_max':>8}")
print("-" * 98)
for leg in legs:
    s = leg_errors[leg]
    dx_s = stats(s["dx"])
    dy_s = stats(s["dy"])
    dz_s = stats(s["dz"])
    d3d_s = stats(s["d3d"])
    print(f"{leg:>4} | "
          f"{dx_s['mean']:>8.3f} {dx_s['rms']:>8.3f} {dx_s['max_abs']:>8.3f} | "
          f"{dy_s['mean']:>8.3f} {dy_s['rms']:>8.3f} {dy_s['max_abs']:>8.3f} | "
          f"{dz_s['mean']:>8.3f} {dz_s['rms']:>8.3f} {dz_s['max_abs']:>8.3f} | "
          f"{d3d_s['rms']:>8.3f} {d3d_s['max_abs']:>8.3f}")

# ============ Section 2: Support phase errors ============
print("\n--- SUPPORT PHASE: Per-leg XYZ error ---")
print(f"{'Leg':>4} | {'dx_mean':>8} {'dx_rms':>8} {'dx_max':>8} | "
      f"{'dy_mean':>8} {'dy_rms':>8} {'dy_max':>8} | "
      f"{'dz_mean':>8} {'dz_rms':>8} {'dz_max':>8} | "
      f"{'3d_rms':>8} {'3d_max':>8}")
print("-" * 98)
for leg in legs:
    s = support_errors[leg]
    dx_s = stats(s["dx"])
    dy_s = stats(s["dy"])
    dz_s = stats(s["dz"])
    d3d_s = stats(s["d3d"])
    print(f"{leg:>4} | "
          f"{dx_s['mean']:>8.3f} {dx_s['rms']:>8.3f} {dx_s['max_abs']:>8.3f} | "
          f"{dy_s['mean']:>8.3f} {dy_s['rms']:>8.3f} {dy_s['max_abs']:>8.3f} | "
          f"{dz_s['mean']:>8.3f} {dz_s['rms']:>8.3f} {dz_s['max_abs']:>8.3f} | "
          f"{d3d_s['rms']:>8.3f} {d3d_s['max_abs']:>8.3f}")

# ============ Section 3: Swing phase errors ============
print("\n--- SWING PHASE: Per-leg XYZ error ---")
print(f"{'Leg':>4} | {'dx_mean':>8} {'dx_rms':>8} {'dx_max':>8} | "
      f"{'dy_mean':>8} {'dy_rms':>8} {'dy_max':>8} | "
      f"{'dz_mean':>8} {'dz_rms':>8} {'dz_max':>8} | "
      f"{'3d_rms':>8} {'3d_max':>8}")
print("-" * 98)
for leg in legs:
    s = swing_errors[leg]
    dx_s = stats(s["dx"])
    dy_s = stats(s["dy"])
    dz_s = stats(s["dz"])
    d3d_s = stats(s["d3d"])
    print(f"{leg:>4} | "
          f"{dx_s['mean']:>8.3f} {dx_s['rms']:>8.3f} {dx_s['max_abs']:>8.3f} | "
          f"{dy_s['mean']:>8.3f} {dy_s['rms']:>8.3f} {dy_s['max_abs']:>8.3f} | "
          f"{dz_s['mean']:>8.3f} {dz_s['rms']:>8.3f} {dz_s['max_abs']:>8.3f} | "
          f"{d3d_s['rms']:>8.3f} {d3d_s['max_abs']:>8.3f}")

# ============ Section 4: Nominal foot positions ============
print("\n--- NOMINAL COMMANDED FOOT POSITIONS (from INIT check in terminal) ---")
print(f"{'Leg':>4} | {'cmd_x':>8} {'cmd_y':>8} {'cmd_z':>8} | "
      f"{'ujc_x':>8} {'ujc_y':>8} {'ujc_z':>8}")
print("-" * 56)
# These came from INIT hold output (rows 24-27 of terminal output):
nominal = {
    "lf": {"cmd": (0.1007, 0.1007, -0.1000), "ujc": (0.3347, 0.3337, -0.1010)},
    "rf": {"cmd": (0.1007, -0.1007, -0.1000), "ujc": (0.3329, -0.3339, -0.1025)},
    "rr": {"cmd": (-0.1007, -0.1007, -0.1000), "ujc": (-0.3341, -0.3350, -0.1004)},
    "lr": {"cmd": (-0.1007, 0.1007, -0.1000), "ujc": (-0.3331, 0.3341, -0.1023)},
}
for leg in legs:
    cmd = nominal[leg]["cmd"]
    ujc = nominal[leg]["ujc"]
    dx = ujc[0] - cmd[0]
    dy = ujc[1] - cmd[1]
    dz = ujc[2] - cmd[2]
    d3d = math.sqrt(dx*dx + dy*dy + dz*dz)
    print(f"{leg:>4} | {cmd[0]:>8.4f} {cmd[1]:>8.4f} {cmd[2]:>8.4f} | "
          f"{ujc[0]:>8.4f} {ujc[1]:>8.4f} {ujc[2]:>8.4f} | "
          f"Δ={d3d*1000:5.1f}mm")

print("\n--- KEY FINDINGS ---")

# Determine max support drift
max_support_drift = 0
for leg in legs:
    d3d = stats(support_errors[leg]["d3d"])
    if d3d["max_abs"] > max_support_drift:
        max_support_drift = d3d["max_abs"]
        worst_leg_support = leg

# Determine max swing error
max_swing_error = 0
for leg in legs:
    d3d = stats(swing_errors[leg]["d3d"])
    if d3d["max_abs"] > max_swing_error:
        max_swing_error = d3d["max_abs"]
        worst_leg_swing = leg

print(f"1. Support-phase foot drift: max 3D error = {max_support_drift*1000:.1f}mm ({worst_leg_support})")
print(f"2. Swing-phase tracking error: max 3D error = {max_swing_error*1000:.1f}mm ({worst_leg_swing})")
print(f"3. Body displacement: {(body_x_vals[-1] - body_x_vals[0])*1000:.1f}mm over {total_duration:.1f}s")

# Check if feet are moving during support (body drag)
print(f"\n4. Foot drift during SUPPORT (should be ~0 if robot is rigid):")
for leg in legs:
    s = support_errors[leg]
    dx_drift = max(abs(v) for v in s["dx"]) if s["dx"] else 0
    dy_drift = max(abs(v) for v in s["dy"]) if s["dy"] else 0
    dz_drift = max(abs(v) for v in s["dz"]) if s["dz"] else 0
    print(f"   {leg}: dx_drift={dx_drift*1000:.1f}mm dy_drift={dy_drift*1000:.1f}mm dz_drift={dz_drift*1000:.1f}mm")
