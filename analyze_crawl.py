#!/usr/bin/env python3
"""Analyze crawl gait CSV log."""

import csv
import numpy as np
from collections import defaultdict

CSV_PATH = "/home/cc/climbing_ws/test_logs/crawl_gait_20260429_164359.csv"

LEGS = ["lf", "rf", "rr", "lr"]
PHASE_NAMES = {
    0: "SUPPORT", 1: "DETACH_SLIDE",
    2: "TANGENTIAL_ALIGN", 3: "PRELOAD_COMPRESS",
    4: "TANGENTIAL_ALIGN_wait", 5: "PRELOAD_OR_COMPLIANT",
    6: "COMPLIANT_SETTLE",
}

# Load data
rows = []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

print(f"Loaded {len(rows)} rows\n")

# Convert numeric fields
def float_or_nan(v):
    try:
        return float(v)
    except (ValueError, TypeError):
        return float("nan")

for r in rows:
    r["elapsed_s"] = float_or_nan(r["elapsed_s"])
    r["body_x"] = float_or_nan(r["body_x"])
    r["body_y"] = float_or_nan(r["body_y"])
    r["body_z"] = float_or_nan(r["body_z"])
    for leg in LEGS:
        r[f"{leg}_phase_id"] = int(float_or_nan(r[f"{leg}_phase_id"]))
        r[f"{leg}_cmd_x"] = float_or_nan(r[f"{leg}_cmd_x"])
        r[f"{leg}_cmd_y"] = float_or_nan(r[f"{leg}_cmd_y"])
        r[f"{leg}_cmd_z"] = float_or_nan(r[f"{leg}_cmd_z"])
        r[f"{leg}_cmd_support"] = int(float_or_nan(r[f"{leg}_cmd_support"]))
        r[f"{leg}_ujc_z"] = float_or_nan(r[f"{leg}_ujc_z"])

# ============================================================
# 1. Body displacement
# ============================================================
body_x_start = rows[0]["body_x"]
body_x_end = rows[-1]["body_x"]
body_y_start = rows[0]["body_y"]
body_y_end = rows[-1]["body_y"]

print("=" * 60)
print("1. BODY DISPLACEMENT")
print("=" * 60)
print(f"  body_x: {body_x_start:.4f} -> {body_x_end:.4f}  (Δ = {body_x_end - body_x_start:.4f} m)")
print(f"  body_y: {body_y_start:.4f} -> {body_y_end:.4f}  (Δ = {body_y_end - body_y_start:.4f} m)")
print(f"  Duration: {rows[-1]['elapsed_s'] - rows[0]['elapsed_s']:.2f} s")
# Also check body_x trajectory over time
bx_vals = np.array([r["body_x"] for r in rows])
by_vals = np.array([r["body_y"] for r in rows])
print(f"  body_x range: [{bx_vals.min():.4f}, {bx_vals.max():.4f}]  std: {bx_vals.std():.4f}")
print(f"  body_y range: [{by_vals.min():.4f}, {by_vals.max():.4f}]  std: {by_vals.std():.4f}")

# Check if body moved at all
if abs(bx_vals.max() - bx_vals.min()) < 0.01:
    print("  ⚠ body_x barely moved — body is effectively stationary")
print()

# ============================================================
# 2. Leg swing patterns — count phase_id=1 transitions
# ============================================================
print("=" * 60)
print("2. LEG SWING PATTERNS (DETACH_SLIDE = phase_id=1)")
print("=" * 60)

for leg in LEGS:
    pid_key = f"{leg}_phase_id"
    swing_starts = []
    prev = rows[0][pid_key]
    for i, r in enumerate(rows):
        curr = r[pid_key]
        # Transition INTO phase_id=1
        if prev != 1 and curr == 1:
            swing_starts.append((i, r["elapsed_s"]))
        prev = curr

    print(f"\n  {leg.upper()}: {len(swing_starts)} swing transitions detected")
    for idx, (row_idx, t) in enumerate(swing_starts):
        duration = 0
        # measure duration of this swing
        for j in range(row_idx, len(rows)):
            if rows[j][pid_key] != 1:
                duration = rows[j]["elapsed_s"] - t
                break
        else:
            duration = rows[-1]["elapsed_s"] - t
        print(f"    Swing #{idx+1}: t={t:.3f}s (row {row_idx}), duration={duration:.2f}s")

# Also count transitions into phase_id=0 (SUPPORT) — useful for counting full swing cycles
print("\n  --- SUPPORT (phase_id=0) entries (full cycle markers) ---")
for leg in LEGS:
    pid_key = f"{leg}_phase_id"
    support_entries = []
    prev = rows[0][pid_key]
    for i, r in enumerate(rows):
        curr = r[pid_key]
        if prev != 0 and curr == 0:
            support_entries.append((i, r["elapsed_s"]))
        prev = curr
    print(f"  {leg.upper()}: {len(support_entries)} SUPPORT entries")

print()

# ============================================================
# 3. Swing sequence analysis
# ============================================================
print("=" * 60)
print("3. SWING SEQUENCE")
print("=" * 60)

# For each row, determine which leg(s) are NOT in SUPPORT (phase_id != 0)
swing_events = []  # (elapsed_s, leg, phase_id)
for r in rows:
    t = r["elapsed_s"]
    for leg in LEGS:
        pid = r[f"{leg}_phase_id"]
        if pid != 0:
            swing_events.append((t, leg, pid))

# Group into contiguous blocks per leg
# For each leg, find contiguous non-zero phase_id blocks
print("  Chronological swing phases (non-SUPPORT):")
last_time = -1
for t, leg, pid in swing_events:
    if t - last_time < 0.01:
        continue  # skip duplicates within same time step
    print(f"    t={t:7.3f}s  {leg.upper()}  phase={pid} ({PHASE_NAMES.get(pid, '?')})")
    last_time = t

# Summarize the overall sequence
# For a proper sequential crawl gait, we expect: ALL_HOLD → one leg swings → ALL_HOLD → next leg swings
# So at any time, only ONE leg should be in non-SUPPORT phase
print("\n  --- Concurrent swing analysis ---")
for r in rows:
    t = r["elapsed_s"]
    non_support = [(leg, r[f"{leg}_phase_id"]) for leg in LEGS if r[f"{leg}_phase_id"] != 0]
    if len(non_support) > 1:
        phases_str = ", ".join(f"{leg}={pid}({PHASE_NAMES.get(pid,'?')})" for leg, pid in non_support)
        print(f"    t={t:7.3f}s  MULTIPLE legs non-SUPPORT: {phases_str}")

print()

# ============================================================
# 4. IK branch switching detection
# ============================================================
print("=" * 60)
print("4. IK BRANCH SWITCHING DETECTION")
print("=" * 60)

# Look for sudden jumps in cmd_x/y/z during SUPPORT (phase_id=0)
# Also look at cmd_support toggles
print("  --- cmd_support toggles during SUPPORT ---")
leg_pairs = [("lf", "rr"), ("rf", "lr")]  # diagonal pairs
for leg_a, leg_b in leg_pairs:
    print(f"\n  Diagonal pair: {leg_a.upper()} vs {leg_b.upper()}")
    prev_support = {leg_a: rows[0][f"{leg_a}_cmd_support"],
                    leg_b: rows[0][f"{leg_b}_cmd_support"]}
    toggle_events = {leg_a: [], leg_b: []}
    for i, r in enumerate(rows[1:], 1):
        for leg in (leg_a, leg_b):
            curr = r[f"{leg}_cmd_support"]
            if curr != prev_support[leg]:
                toggle_events[leg].append((i, r["elapsed_s"], curr))
                prev_support[leg] = curr
    for leg in (leg_a, leg_b):
        print(f"    {leg.upper()} cmd_support toggles: {len(toggle_events[leg])} — times: " +
              ", ".join(f"t={t:.2f}s" for _, t, _ in toggle_events[leg]))

print("\n  --- Sudden cmd jumps analysis (during SUPPORT, phase_id=0) ---")
# For each leg, during SUPPORT blocks, look at cmd_x, cmd_y, cmd_z differences between consecutive samples
leg_support_jumps = {leg: {"x": [], "y": [], "z": [], "times": []} for leg in LEGS}
for leg in LEGS:
    prev_vals = None
    prev_time = None
    for r in rows:
        if r[f"{leg}_phase_id"] == 0:
            t = r["elapsed_s"]
            vals = (r[f"{leg}_cmd_x"], r[f"{leg}_cmd_y"], r[f"{leg}_cmd_z"])
            if prev_vals is not None:
                dx = abs(vals[0] - prev_vals[0])
                dy = abs(vals[1] - prev_vals[1])
                dz = abs(vals[2] - prev_vals[2])
                dt = t - prev_time
                if dt > 0 and (dx > 0.01 or dy > 0.01 or dz > 0.01):
                    speed_x = dx / dt
                    speed_y = dy / dt
                    speed_z = dz / dt
                    if speed_x + speed_y + speed_z > 0.5:  # threshold for sudden jump
                        leg_support_jumps[leg]["x"].append(dx)
                        leg_support_jumps[leg]["y"].append(dy)
                        leg_support_jumps[leg]["z"].append(dz)
                        leg_support_jumps[leg]["times"].append((t, dx, dy, dz, dt))
            prev_vals = vals
            prev_time = t

for leg in LEGS:
    jumps = leg_support_jumps[leg]
    if jumps["times"]:
        print(f"\n  {leg.upper()}: {len(jumps['times'])} sudden jumps during SUPPORT")
        for t, dx, dy, dz, dt in jumps["times"][:5]:
            print(f"    t={t:.3f}s  Δx={dx:.4f} Δy={dy:.4f} Δz={dz:.4f}  (dt={dt:.3f}s)")
        if len(jumps["times"]) > 5:
            print(f"    ... and {len(jumps['times'])-5} more")
    else:
        print(f"\n  {leg.upper()}: No sudden jumps during SUPPORT")

# Cross-check: look at diagonal legs during same phase
print("\n  --- Cross-check: Diagonal leg cmd values correlation ---")
for leg_a, leg_b in leg_pairs:
    phase_key_a = f"{leg_a}_phase_id"
    phase_key_b = f"{leg_b}_phase_id"
    # Find times when BOTH diagonal legs are in SUPPORT
    support_times = [(r["elapsed_s"],
                      r[f"{leg_a}_cmd_x"], r[f"{leg_a}_cmd_y"], r[f"{leg_a}_cmd_z"],
                      r[f"{leg_b}_cmd_x"], r[f"{leg_b}_cmd_y"], r[f"{leg_b}_cmd_z"])
                     for r in rows if r[phase_key_a] == 0 and r[phase_key_b] == 0]
    if support_times:
        cmd_x_a = np.array([s[1] for s in support_times])
        cmd_x_b = np.array([s[4] for s in support_times])
        cmd_y_a = np.array([s[2] for s in support_times])
        cmd_y_b = np.array([s[5] for s in support_times])
        # Check if diagonal legs have mirrored cmd values
        x_diff = np.abs(cmd_x_a + cmd_x_b).mean()  # should be ~0 if mirrored
        y_diff = np.abs(cmd_y_a + cmd_y_b).mean()
        print(f"    {leg_a.upper()}-{leg_b.upper()}: mean |cmd_x_a + cmd_x_b| = {x_diff:.4f}  "
              f"mean |cmd_y_a + cmd_y_b| = {y_diff:.4f}")
        # Check for sudden jumps in both legs simultaneously (branch switch indicator)
        x_jumps_a = np.abs(np.diff(cmd_x_a))
        x_jumps_b = np.abs(np.diff(cmd_x_b))
        # Count simultaneous jumps
        both_jump = np.sum((x_jumps_a > 0.01) & (x_jumps_b > 0.01))
        print(f"    Simultaneous cmd_x jumps (>0.01): {both_jump} occurrences")

print()

# ============================================================
# 5. Swing trajectory smoothness
# ============================================================
print("=" * 60)
print("5. SWING TRAJECTORY SMOOTHNESS")
print("=" * 60)

for leg in LEGS:
    # Extract ALL non-SUPPORT blocks (swing phases)
    pid_key = f"{leg}_phase_id"
    cmd_x_key = f"{leg}_cmd_x"
    cmd_y_key = f"{leg}_cmd_y"
    cmd_z_key = f"{leg}_cmd_z"

    in_swing = False
    swing_blocks = []
    current_block = {"t": [], "x": [], "y": [], "z": []}
    for r in rows:
        t = r["elapsed_s"]
        if r[pid_key] != 0 and r[pid_key] != 5:  # exclude PRELOAD_OR_COMPLIANT and SUPPORT
            if not in_swing:
                in_swing = True
                current_block = {"t": [t], "x": [r[cmd_x_key]], "y": [r[cmd_y_key]], "z": [r[cmd_z_key]]}
            else:
                current_block["t"].append(t)
                current_block["x"].append(r[cmd_x_key])
                current_block["y"].append(r[cmd_y_key])
                current_block["z"].append(r[cmd_z_key])
        else:
            if in_swing and len(current_block["t"]) > 3:
                swing_blocks.append(current_block)
            in_swing = False

    if in_swing and len(current_block["t"]) > 3:
        swing_blocks.append(current_block)

    print(f"\n  {leg.upper()}: {len(swing_blocks)} swing trajectories")
    for bi, block in enumerate(swing_blocks[:4]):  # show first 4
        t_arr = np.array(block["t"])
        x_arr = np.array(block["x"])
        y_arr = np.array(block["y"])
        z_arr = np.array(block["z"])
        dur = t_arr[-1] - t_arr[0]
        n_pts = len(t_arr)

        # Compute second differences as smoothness metric
        if len(x_arr) >= 3:
            x_dd = np.abs(np.diff(x_arr, n=2))
            y_dd = np.abs(np.diff(y_arr, n=2))
            z_dd = np.abs(np.diff(z_arr, n=2))
            smoothness_x = np.mean(x_dd) if len(x_dd) > 0 else 0
            smoothness_y = np.mean(y_dd) if len(y_dd) > 0 else 0
            smoothness_z = np.mean(z_dd) if len(z_dd) > 0 else 0
            # Lower = smoother
            print(f"    Block #{bi+1}: {n_pts} pts, {dur:.2f}s, "
                  f"mean|d2x/dt2|={smoothness_x:.6f}, "
                  f"mean|d2y/dt2|={smoothness_y:.6f}, "
                  f"mean|d2z/dt2|={smoothness_z:.6f}")
        else:
            print(f"    Block #{bi+1}: {n_pts} pts, {dur:.2f}s (too few points for smoothness calc)")

        # Check for monotonic cmd_z (should go up then down in a smooth arc)
        z_diff = np.diff(z_arr)
        # Count sign changes in z direction
        sign_changes = np.sum(np.diff(np.sign(z_diff)) != 0) if len(z_diff) > 1 else 0
        print(f"           Z direction changes (sign changes in dz/dt): {sign_changes}")

print()

# ============================================================
# 6. Z tracking performance
# ============================================================
print("=" * 60)
print("6. Z TRACKING (ujc_z vs cmd_z)")
print("=" * 60)

for leg in LEGS:
    pid_key = f"{leg}_phase_id"
    cmd_z_key = f"{leg}_cmd_z"
    ujc_z_key = f"{leg}_ujc_z"

    # Analyze during swing phases (non-SUPPORT, non-PRELOAD_OR_COMPLIANT)
    swing_errors = []
    support_errors = []

    for r in rows:
        t = r["elapsed_s"]
        cmd_z = r[cmd_z_key]
        ujc_z = r[ujc_z_key]
        error = ujc_z - cmd_z  # actual - commanded

        if r[pid_key] == 1:  # DETACH_SLIDE
            swing_errors.append((t, cmd_z, ujc_z, error))
        elif r[pid_key] == 0:  # SUPPORT
            support_errors.append((t, cmd_z, ujc_z, error))

    print(f"\n  {leg.upper()}:")

    if swing_errors:
        err_vals = np.array([e[3] for e in swing_errors])
        cmd_vals = np.array([e[1] for e in swing_errors])
        ujc_vals = np.array([e[2] for e in swing_errors])
        print(f"    During DETACH_SLIDE (swing):")
        print(f"      cmd_z range: [{cmd_vals.min():.4f}, {cmd_vals.max():.4f}]")
        print(f"      ujc_z range: [{ujc_vals.min():.4f}, {ujc_vals.max():.4f}]")
        print(f"      Mean error (ujc_z - cmd_z): {np.mean(err_vals):.4f}")
        print(f"      RMSE: {np.sqrt(np.mean(err_vals**2)):.4f}")
        print(f"      Max abs error: {np.max(np.abs(err_vals)):.4f}")
        print(f"      Std error: {np.std(err_vals):.4f}")

        # Check for unusually large errors
        large_errors = [e for e in swing_errors if abs(e[3]) > 0.02]
        if large_errors:
            print(f"      ⚠ {len(large_errors)} samples with |error| > 0.02m")
            for t, cmd, ujc, err in large_errors[:5]:
                print(f"        t={t:.3f}s  cmd_z={cmd:.4f}  ujc_z={ujc:.4f}  error={err:.4f}")

    if support_errors:
        err_vals_s = np.array([e[3] for e in support_errors])
        print(f"    During SUPPORT:")
        print(f"      Mean error: {np.mean(err_vals_s):.4f}")
        print(f"      RMSE: {np.sqrt(np.mean(err_vals_s**2)):.4f}")
        print(f"      Max abs error: {np.max(np.abs(err_vals_s)):.4f}")

print("\n" + "=" * 60)
print("SUMMARY OF FINDINGS")
print("=" * 60)
print(f"1. Body displacement: Δx = {body_x_end - body_x_start:.4f}m, Δy = {body_y_end - body_y_start:.4f}m")
print(f"   body_x range over test: [{bx_vals.min():.4f}, {bx_vals.max():.4f}]")
print(f"   body_y range over test: [{by_vals.min():.4f}, {by_vals.max():.4f}]")

# Count total swing cycles
total_swings = {}
for leg in LEGS:
    pid_key = f"{leg}_phase_id"
    count = 0
    prev = rows[0][pid_key]
    for r in rows:
        if prev != 1 and r[pid_key] == 1:
            count += 1
        prev = r[pid_key]
    total_swings[leg] = count
print(f"2. Total detected swing cycles: {total_swings}")

# IK branch switching summary
ik_anomalies = sum(1 for leg in LEGS if leg_support_jumps[leg]["times"])
print(f"4. {'⚠ IK branch switching suspected' if ik_anomalies > 0 else 'No evidence of IK branch switching detected'}")

# Smoothness
print(f"5. Swing trajectories appear {'smooth' if all(len(leg_support_jumps[leg]['times']) < 3 for leg in LEGS) else 'to be checked — see above for details'}")

# Z tracking summary
all_rmse = []
for leg in LEGS:
    pid_key = f"{leg}_phase_id"
    cmd_z_key = f"{leg}_cmd_z"
    ujc_z_key = f"{leg}_ujc_z"
    errors = []
    for r in rows:
        if r[pid_key] == 1:
            errors.append(r[ujc_z_key] - r[cmd_z_key])
    if errors:
        rmse = np.sqrt(np.mean(np.array(errors)**2))
        all_rmse.append(rmse)
        print(f"6. {leg.upper()} Z tracking RMSE during swing: {rmse:.4f}m")
if all_rmse:
    print(f"   Average Z tracking RMSE across legs: {np.mean(all_rmse):.4f}m")
