#!/usr/bin/env python3
"""Deep-dive analysis of crawl gait CSV — phase_id distributions and adhesion."""

import csv
import numpy as np
from collections import Counter, defaultdict

CSV_PATH = "/home/cc/climbing_ws/test_logs/crawl_gait_20260429_164359.csv"
LEGS = ["lf", "rf", "rr", "lr"]

rows = []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

for r in rows:
    for k in r:
        if k not in ("wall_time", "ros_time", "mission_state", "phase_label", "mission_active"):
            try:
                r[k] = float(r[k])
            except:
                r[k] = float("nan")

print("=" * 70)
print("DEEP DIVE: ACTUAL PHASE ID DISTRIBUTION PER LEG")
print("=" * 70)
for leg in LEGS:
    pid_counts = Counter(r[f"{leg}_phase_id"] for r in rows)
    sorted_counts = sorted(pid_counts.items())
    print(f"  {leg.upper()}: {dict(sorted_counts)}")

print("\n" + "=" * 70)
print("PHASE TRANSITION SEQUENCE (chronological, all legs)")
print("=" * 70)
# Walk through rows and log whenever ANY leg's phase_id changes
last = {leg: rows[0][f"{leg}_phase_id"] for leg in LEGS}
for i, r in enumerate(rows):
    t = r["elapsed_s"]
    changes = []
    for leg in LEGS:
        curr = r[f"{leg}_phase_id"]
        if curr != last[leg]:
            changes.append((leg, last[leg], curr, t))
            last[leg] = curr
    for leg, old, new, t in changes:
        print(f"  t={t:7.3f}s  {leg.upper()}: {old} -> {new}")

print("\n" + "=" * 70)
print("ADHESION & MEASURED CONTACT STATUS ACROSS ALL LEGS (sample)")
print("=" * 70)
print("  Showing 1 row per distinct adhesion/contact state change")
prev_adhesion = {leg: None for leg in LEGS}
prev_contact = {leg: None for leg in LEGS}
for r in rows:
    t = r["elapsed_s"]
    changed = False
    for leg in LEGS:
        a = r[f"{leg}_adhesion"]
        c = r[f"{leg}_measured_contact"]
        if a != prev_adhesion[leg] or c != prev_contact[leg]:
            changed = True
        prev_adhesion[leg] = a
        prev_contact[leg] = c
    if changed:
        statuses = " | ".join(
            f"{leg.upper()}: adh={r[f'{leg}_adhesion']:.0f}, contact={r[f'{leg}_measured_contact']:.0f}, "
            f"att_rdy={r[f'{leg}_attachment_ready']:.0f}, fan_rpm={r[f'{leg}_fan_rpm']:.0f}"
            for leg in LEGS
        )
        print(f"  t={t:7.3f}s  {r['body_x']:+.4f}|{r['body_y']:+.4f}  {statuses}")

print("\n" + "=" * 70)
print("PERSISTENT ADHESION STATES (summary)")
print("=" * 70)
for leg in LEGS:
    adh_series = [r[f"{leg}_adhesion"] for r in rows]
    unique_vals = sorted(set(adh_series))
    pct_contact = np.mean([r[f"{leg}_measured_contact"] for r in rows]) * 100
    pct_att_rdy = np.mean([r[f"{leg}_attachment_ready"] for r in rows]) * 100
    print(f"  {leg.upper()}: adhesion values={unique_vals}, "
          f"contact={pct_contact:.0f}%, attach_ready={pct_att_rdy:.0f}%")

print("\n" + "=" * 70)
print("FAN CURRENT ANALYSIS")
print("=" * 70)
for leg in LEGS:
    currents = [r[f"{leg}_fan_current_a"] for r in rows]
    nonzero = [c for c in currents if abs(c) > 0.001]
    print(f"  {leg.upper()}: mean_current={np.mean(currents):.4f}A, "
          f"nonzero_samples={len(nonzero)}/{len(currents)}")
    if nonzero:
        print(f"         current range: [{min(nonzero):.4f}, {max(nonzero):.4f}]")

print("\n" + "=" * 70)
print("BODY VELOCITY ANALYSIS")
print("=" * 70)
vx_vals = np.array([r["body_vx"] for r in rows])
vy_vals = np.array([r["body_vy"] for r in rows])
vz_vals = np.array([r["body_vz"] for r in rows])
print(f"  body_vx: mean={vx_vals.mean():.5f} std={vx_vals.std():.5f} range=[{vx_vals.min():.5f}, {vx_vals.max():.5f}]")
print(f"  body_vy: mean={vy_vals.mean():.5f} std={vy_vals.std():.5f} range=[{vy_vals.min():.5f}, {vy_vals.max():.5f}]")
print(f"  body_vz: mean={vz_vals.mean():.5f} std={vz_vals.std():.5f} range=[{vz_vals.min():.5f}, {vz_vals.max():.5f}]")
# Count non-zero velocity samples
nz_vx = np.sum(np.abs(vx_vals) > 0.0001)
nz_vy = np.sum(np.abs(vy_vals) > 0.0001)
nz_vz = np.sum(np.abs(vz_vals) > 0.0001)
print(f"  Non-zero: vx={nz_vx}, vy={nz_vy}, vz={nz_vz} (out of {len(vx_vals)})")

print("\n" + "=" * 70)
print("GAP DETECTION (t > 1.0s between consecutive samples)")
print("=" * 70)
prev_t = rows[0]["elapsed_s"]
for i, r in enumerate(rows[1:], 1):
    dt = r["elapsed_s"] - prev_t
    if dt > 1.0:
        print(f"  Gap at row {i}: t={prev_t:.3f}s -> {r['elapsed_s']:.3f}s (dt={dt:.2f}s)")
    prev_t = r["elapsed_s"]

print("\n" + "=" * 70)
print("COMMAND TRAJECTORY SMOOTHNESS DURING NON-SUPPORT PHASES")
print("=" * 70)
for leg in LEGS:
    # Get data during TANGENTIAL_ALIGN_wait (4) and PRELOAD_OR_COMPLIANT (5)
    for phase_id, phase_name in [(4, "TANGENTIAL_ALIGN_wait"), (5, "PRELOAD_OR_COMPLIANT")]:
        mask = [r[f"{leg}_phase_id"] == phase_id for r in rows]
        if not any(mask):
            continue
        idxs = np.where(mask)[0]
        # Find contiguous blocks
        blocks = []
        start = idxs[0]
        for j in range(1, len(idxs)):
            if idxs[j] - idxs[j-1] > 1:
                blocks.append((start, idxs[j-1]))
                start = idxs[j]
        blocks.append((start, idxs[-1]))
        
        print(f"\n  {leg.upper()} phase={phase_id}({phase_name}): {len(blocks)} blocks")
        for bi, (s, e) in enumerate(blocks[:2]):
            t_vals = np.array([rows[k]["elapsed_s"] for k in range(s, e+1)])
            x_vals = np.array([rows[k][f"{leg}_cmd_x"] for k in range(s, e+1)])
            y_vals = np.array([rows[k][f"{leg}_cmd_y"] for k in range(s, e+1)])
            z_vals = np.array([rows[k][f"{leg}_cmd_z"] for k in range(s, e+1)])
            dur = t_vals[-1] - t_vals[0]
            n = len(t_vals)

            if n >= 3:
                dd2_x = np.mean(np.abs(np.diff(x_vals, n=2)))
                dd2_y = np.mean(np.abs(np.diff(y_vals, n=2)))
                dd2_z = np.mean(np.abs(np.diff(z_vals, n=2)))
                print(f"    Block #{bi+1}: {n} pts, {dur:.2f}s, "
                      f"cmd_x range=[{x_vals.min():.4f},{x_vals.max():.4f}], "
                      f"cmd_y range=[{y_vals.min():.4f},{y_vals.max():.4f}], "
                      f"cmd_z range=[{z_vals.min():.4f},{z_vals.max():.4f}]")
            else:
                print(f"    Block #{bi+1}: {n} pts, {dur:.2f}s (too few)")

print("\n" + "=" * 70)
print("SLEW RATE ANALYSIS (cmd changes per second during non-SUPPORT)")
print("=" * 70)
for leg in LEGS:
    non_support = [(r["elapsed_s"], r[f"{leg}_cmd_x"], r[f"{leg}_cmd_y"], r[f"{leg}_cmd_z"])
                   for r in rows if r[f"{leg}_phase_id"] != 0 and r[f"{leg}_phase_id"] != 5]
    if len(non_support) < 3:
        print(f"  {leg.upper()}: insufficient non-SUPPORT/non-PRELOAD data")
        continue
    t_arr = np.array([n[0] for n in non_support])
    x_arr = np.array([n[1] for n in non_support])
    y_arr = np.array([n[2] for n in non_support])
    z_arr = np.array([n[3] for n in non_support])

    # Compute slew rates (dx/dt per sample)
    dts = np.diff(t_arr)
    vx = np.abs(np.diff(x_arr)) / np.where(dts > 0, dts, 0.001)
    vy = np.abs(np.diff(y_arr)) / np.where(dts > 0, dts, 0.001)
    vz = np.abs(np.diff(z_arr)) / np.where(dts > 0, dts, 0.001)

    print(f"  {leg.upper()} (n={len(non_support)}):")
    print(f"    x slew rate: max={vx.max():.2f} m/s, mean={vx.mean():.3f} m/s")
    print(f"    y slew rate: max={vy.max():.2f} m/s, mean={vy.mean():.3f} m/s")
    print(f"    z slew rate: max={vz.max():.2f} m/s, mean={vz.mean():.3f} m/s")
