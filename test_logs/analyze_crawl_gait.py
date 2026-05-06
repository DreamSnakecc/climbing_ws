#!/usr/bin/env python3
"""
Deep analysis of crawl_gait log: body_x vs command_x for support legs.
Fix: swing cycle is SUPPORT → TANGENTIAL_ALIGN → PRELOAD_OR_COMPLIANT → SUPPORT
"""

import csv
import math

CSV_PATH = "/home/cc/climbing_ws/test_logs/crawl_gait_20260430_113450.csv"

rows = []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

print(f"Total rows: {len(rows)}")
print(f"Time span: {float(rows[0]['elapsed_s']):.4f}s to {float(rows[-1]['elapsed_s']):.4f}s "
      f"({float(rows[-1]['elapsed_s']) - float(rows[0]['elapsed_s']):.2f}s)")
print()

# ─── 0. Identify operating_center_x per leg ─────────────────────────────────
# The code: cmd_x = operating_center_x - body_x
# So operating_center_x = body_x + cmd_x (approx - before yaw transform)
# For front legs, expected ~0.1007; for rear legs, expected ~-0.1007
print("=" * 70)
print("0. BASELINE: operating_center_x = body_x + cmd_x (for each support period)")
print("=" * 70)
print()

legs_cmd = ["lf_cmd_x", "rf_cmd_x", "rr_cmd_x", "lr_cmd_x"]
legs_sup = ["lf_cmd_support", "rf_cmd_support", "rr_cmd_support", "lr_cmd_support"]
leg_labels = ["LF", "RF", "RR", "LR"]

for leg_label, cmd_col, sup_col in zip(leg_labels, legs_cmd, legs_sup):
    vals = []
    for r in rows:
        if int(r[sup_col]) == 1:
            vals.append(float(r["body_x"]) + float(r[cmd_col]))
    if vals:
        print(f"  {leg_label}:  mean operating_center = {sum(vals)/len(vals):.6f}  "
              f"(range [{min(vals):.6f}, {max(vals):.6f}], spread={max(vals)-min(vals):.6f})")
    else:
        print(f"  {leg_label}: no support rows")

print()

# ─── 1. body_x statistics ────────────────────────────────────────────────────
body_x_vals = [float(r["body_x"]) for r in rows]
elapsed = [float(r["elapsed_s"]) for r in rows]

bmin, bmax = min(body_x_vals), max(body_x_vals)
bmean = sum(body_x_vals) / len(body_x_vals)
print("=" * 70)
print("1. body_x OVER TIME")
print("=" * 70)
print(f"  min:  {bmin:.6f}")
print(f"  max:  {bmax:.6f}")
print(f"  mean: {bmean:.6f}")
print(f"  range: {bmax - bmin:.6f}")
print()

n = len(rows)
samples = [0, n // 4, n // 2, 3 * n // 4, n - 1]
print("  Samples:")
for idx in samples:
    print(f"    row {idx:4d}  elapsed={elapsed[idx]:8.4f}s  body_x={body_x_vals[idx]:.6f}")

print()

# ─── 2. body_x relationship with cmd_x for support legs ──────────────────────
print("=" * 70)
print("2. body_x vs support leg cmd_x (delta analysis, non-zero deltas only)")
print("=" * 70)
print()

for leg_name, cmd_col, sup_col in zip(leg_labels, legs_cmd, legs_sup):
    prev_body = None
    prev_cmd = None
    body_deltas = []
    cmd_deltas = []

    for r in rows:
        support = int(r[sup_col])
        if support != 1:
            prev_body = None
            prev_cmd = None
            continue
        b = float(r["body_x"])
        c = float(r[cmd_col])
        if prev_body is not None and prev_cmd is not None:
            db = b - prev_body
            dc = c - prev_cmd
            if db != 0.0 or dc != 0.0:
                body_deltas.append(db)
                cmd_deltas.append(dc)
        prev_body = b
        prev_cmd = c

    if body_deltas:
        mean_db = sum(body_deltas) / len(body_deltas)
        mean_dc = sum(cmd_deltas) / len(cmd_deltas)
        num = sum((d - mean_db) * (c - mean_dc) for d, c in zip(body_deltas, cmd_deltas))
        den = math.sqrt(sum((d - mean_db) ** 2 for d in body_deltas)) * math.sqrt(
            sum((c - mean_dc) ** 2 for c in cmd_deltas)
        )
        corr = num / den if den != 0 else 0.0
        opp_sign = sum(1 for d, c in zip(body_deltas, cmd_deltas) if d * c < 0)
        same_sign = sum(1 for d, c in zip(body_deltas, cmd_deltas) if d * c > 0)
        zero_one = sum(1 for d, c in zip(body_deltas, cmd_deltas) if d == 0 or c == 0)
        print(f"  {leg_name}:  {len(body_deltas)} support-rows with nonzero deltas")
        print(f"    corr(body_delta, cmd_delta): {corr:.4f}")
        print(f"    mean body_delta:  {mean_db:.8f}")
        print(f"    mean cmd_delta:   {mean_dc:.8f}")
        print(f"    opp sign: {opp_sign:3d} ({100*opp_sign/len(body_deltas):5.1f}%)  "
              f"same sign: {same_sign:3d} ({100*same_sign/len(body_deltas):5.1f}%)  "
              f"one zero: {zero_one:3d} ({100*zero_one/len(body_deltas):5.1f}%)")
    else:
        print(f"  {leg_name}: no support rows")

print()

# ─── 3. Per-swing-cycle analysis (full cycle) ────────────────────────────────
print("=" * 70)
print("3. PER-SWING-CYCLE ANALYSIS")
print("=" * 70)
print("  Full cycle: SUPPORT -> TANGENTIAL_ALIGN -> PRELOAD_OR_COMPLIANT -> SUPPORT")
print()

for leg_label, phase_col in zip(leg_labels, ["lf_phase", "rf_phase", "rr_phase", "lr_phase"]):
    print(f"  --- {leg_label} ---")
    prev_phase = None
    in_swing = False  # True from SUPPORT->TANGENTIAL_ALIGN until back to SUPPORT
    swing_start_idx = None
    swing_start_body = None
    swing_start_cmds = {}
    cycles = []

    for i, r in enumerate(rows):
        phase = r[phase_col]
        if prev_phase is None:
            prev_phase = phase
            continue

        # Swing start: SUPPORT -> TANGENTIAL_ALIGN
        if prev_phase == "SUPPORT" and phase == "TANGENTIAL_ALIGN":
            in_swing = True
            swing_start_idx = i
            swing_start_body = float(r["body_x"])
            swing_start_cmds = {
                l: float(r[f"{l.lower()}_cmd_x"]) for l in leg_labels
            }

        # Swing end: arriving at SUPPORT (from PRELOAD_OR_COMPLIANT)
        if in_swing and phase == "SUPPORT":
            end_body = float(r["body_x"])
            end_cmds = {
                l: float(r[f"{l.lower()}_cmd_x"]) for l in leg_labels
            }
            cmd_changes = {l: end_cmds[l] - swing_start_cmds[l] for l in leg_labels}
            cycles.append({
                "start_row": swing_start_idx,
                "end_row": i,
                "start_body_x": swing_start_body,
                "end_body_x": end_body,
                "body_x_change": end_body - swing_start_body,
                "cmd_changes": cmd_changes,
            })
            in_swing = False
            swing_start_idx = None

        prev_phase = phase

    print(f"    Found {len(cycles)} swing cycles")
    for cyc_idx, cyc in enumerate(cycles):
        print(f"\n    Cycle {cyc_idx}: rows {cyc['start_row']}-{cyc['end_row']}")
        print(f"      body_x: {cyc['start_body_x']:.6f} -> {cyc['end_body_x']:.6f}  "
              f"(Δ={cyc['body_x_change']:.6f})")
        for l in leg_labels:
            dc = cyc["cmd_changes"][l]
            print(f"        {l}_cmd_x Δ = {dc:+.6f}")
    print()

print()

# ─── 4. body_x + cmd_x invariant over time ───────────────────────────────────
print("=" * 70)
print("4. body_x + cmd_x INVARIANT (should be constant for each support leg)")
print("   Sampled every 20th row with color on support/non-support")
print("=" * 70)
print()

sample_step = 20
header = (f"{'row':>5} {'elapsed':>8} {'body_x':>9} "
          f"{'LF_sum':>9} {'RF_sum':>9} {'RR_sum':>9} {'LR_sum':>9}")
print(header)
print("-" * len(header))
for i in range(0, len(rows), sample_step):
    r = rows[i]
    b = float(r["body_x"])
    t = float(r["elapsed_s"])
    sums = []
    for leg in ["lf", "rf", "rr", "lr"]:
        s = int(r[f"{leg}_cmd_support"])
        val = b + float(r[f"{leg}_cmd_x"])
        marker = "*" if s == 1 else " "
        sums.append(f"{val:+9.6f}{marker}")
    print(f"{i:5d} {t:8.4f} {b:9.6f} "
          f"{sums[0]} {sums[1]} {sums[2]} {sums[3]}")

print("\n  (* = support leg)")
print()

# ─── 5. Focus on the LF swing at t≈36.7s (body_x drops to -0.0098) ──────────
print("=" * 70)
print("5. FOCUS: LF swing at t≈36.7s (body_x drops to -0.0098)")
print("=" * 70)
print()
print("  Around the LF lift-off at row 98 and replant at row 143:")
print()
print(f"{'row':>5} {'elapsed':>8} {'body_x':>9} {'LF_cmd':>9} {'LF_phase':>22} "
      f"{'RF_cmd':>9} {'RR_cmd':>9} {'LR_cmd':>9}")
for i in range(90, 150):
    r = rows[i]
    print(f"{i:5d} {float(r['elapsed_s']):8.4f} {float(r['body_x']):9.6f} "
          f"{float(r['lf_cmd_x']):+9.6f} {r['lf_phase']:>22} "
          f"{float(r['rf_cmd_x']):+9.6f} {float(r['rr_cmd_x']):+9.6f} {float(r['lr_cmd_x']):+9.6f}")

print()

# ─── 6. Focus on the LF swing at t≈41.8s (body_x drops to -0.0048) ──────────
print("=" * 70)
print("6. FOCUS: LF swing at t≈41.8s (body_x drops to -0.0048)")
print("=" * 70)
print()
print("  Around the LF lift-off at row 350 and replant at row 395:")
print()
print(f"{'row':>5} {'elapsed':>8} {'body_x':>9} {'LF_cmd':>9} {'LF_phase':>22} "
      f"{'RF_cmd':>9} {'RR_cmd':>9} {'LR_cmd':>9}")
for i in range(340, 405):
    r = rows[i]
    print(f"{i:5d} {float(r['elapsed_s']):8.4f} {float(r['body_x']):9.6f} "
          f"{float(r['lf_cmd_x']):+9.6f} {r['lf_phase']:>22} "
          f"{float(r['rf_cmd_x']):+9.6f} {float(r['rr_cmd_x']):+9.6f} {float(r['lr_cmd_x']):+9.6f}")

print()

# ─── 7. Summary: body_x at each leg's lift-off and replant ───────────────────
print("=" * 70)
print("7. body_x AT ALL LEG LIFT-OFFS AND REPLANTS (summary)")
print("=" * 70)
print()

prev_phases = {l: "" for l in ["lf", "rf", "rr", "lr"]}
events = []
for i, r in enumerate(rows):
    cur = {l: r[f"{l}_phase"] for l in ["lf", "rf", "rr", "lr"]}
    for l in cur:
        if cur[l] != prev_phases[l]:
            b = float(r["body_x"])
            t = float(r["elapsed_s"])
            events.append({
                "row": i, "time": t, "leg": l.upper(), "body_x": b,
                "from": prev_phases[l], "to": cur[l]
            })
    prev_phases = cur

# Group
print(f"  Lift-offs (SUPPORT -> TANGENTIAL_ALIGN):")
for ev in events:
    if ev["from"] == "SUPPORT" and ev["to"] == "TANGENTIAL_ALIGN":
        print(f"    row {ev['row']:4d}  t={ev['time']:8.4f}s  leg={ev['leg']}  body_x={ev['body_x']:+.6f}")

print()
print(f"  Replants (PRELOAD_OR_COMPLIANT -> SUPPORT):")
for ev in events:
    if ev["from"] == "PRELOAD_OR_COMPLIANT" and ev["to"] == "SUPPORT":
        print(f"    row {ev['row']:4d}  t={ev['time']:8.4f}s  leg={ev['leg']}  body_x={ev['body_x']:+.6f}")

# body_x at each swing phase
lift_off_body = [ev["body_x"] for ev in events
                 if ev["from"] == "SUPPORT" and ev["to"] == "TANGENTIAL_ALIGN"]
replant_body = [ev["body_x"] for ev in events
                if ev["from"] == "PRELOAD_OR_COMPLIANT" and ev["to"] == "SUPPORT"]

print()
if lift_off_body:
    print(f"  Lift-off body_x:  min={min(lift_off_body):.6f}  max={max(lift_off_body):.6f}  "
          f"mean={sum(lift_off_body)/len(lift_off_body):.6f}")
if replant_body:
    print(f"  Replant body_x:   min={min(replant_body):.6f}  max={max(replant_body):.6f}  "
          f"mean={sum(replant_body)/len(replant_body):.6f}")

print()

# ─── 8. Does body_x decrease when a leg is held in SUPPORT? ──────────────────
print("=" * 70)
print("8. body_x behavior DURING SUPPORT periods (non-swinging legs)")
print("   Checking: when body_x drops during LF swing, do support cmd_x compensate?")
print("=" * 70)
print()

# For LF swing from 98-143: body_x goes -0.0098 (drop of ~0.0246 from baseline)
# Check what RF, RR, LR cmd_x do
print("  Example: LF swing cycle (rows 98-143)")
print(f"    body_x start: {float(rows[98]['body_x']):.6f}, end: {float(rows[143]['body_x']):.6f}")
print(f"    body_x Δ: {float(rows[143]['body_x']) - float(rows[98]['body_x']):.6f}")
for leg in ["rf", "rr", "lr"]:
    start_cmd = float(rows[98][f"{leg}_cmd_x"])
    end_cmd = float(rows[143][f"{leg}_cmd_x"])
    start_sup = int(rows[98][f"{leg}_cmd_support"])
    end_sup = int(rows[143][f"{leg}_cmd_support"])
    print(f"    {leg.upper()}_cmd_x: {start_cmd:+.6f} -> {end_cmd:+.6f}  (Δ={end_cmd-start_cmd:+.6f})  "
          f"support: {start_sup}->{end_sup}")

print()
print("  Example: LF swing cycle (rows 350-395)")
print(f"    body_x start: {float(rows[350]['body_x']):.6f}, end: {float(rows[395]['body_x']):.6f}")
print(f"    body_x Δ: {float(rows[395]['body_x']) - float(rows[350]['body_x']):.6f}")
for leg in ["rf", "rr", "lr"]:
    start_cmd = float(rows[350][f"{leg}_cmd_x"])
    end_cmd = float(rows[395][f"{leg}_cmd_x"])
    start_sup = int(rows[350][f"{leg}_cmd_support"])
    end_sup = int(rows[395][f"{leg}_cmd_support"])
    print(f"    {leg.upper()}_cmd_x: {start_cmd:+.6f} -> {end_cmd:+.6f}  (Δ={end_cmd-start_cmd:+.6f})  "
          f"support: {start_sup}->{end_sup}")

print()
print("  Example: LF swing cycle (rows 902-946)")
print(f"    body_x start: {float(rows[902]['body_x']):.6f}, end: {float(rows[946]['body_x']):.6f}")
print(f"    body_x Δ: {float(rows[946]['body_x']) - float(rows[902]['body_x']):.6f}")
for leg in ["rf", "rr", "lr"]:
    start_cmd = float(rows[902][f"{leg}_cmd_x"])
    end_cmd = float(rows[946][f"{leg}_cmd_x"])
    start_sup = int(rows[902][f"{leg}_cmd_support"])
    end_sup = int(rows[946][f"{leg}_cmd_support"])
    print(f"    {leg.upper()}_cmd_x: {start_cmd:+.6f} -> {end_cmd:+.6f}  (Δ={end_cmd-start_cmd:+.6f})  "
          f"support: {start_sup}->{end_sup}")

print()

# ─── 9. Final: body_x trend by chunk ─────────────────────────────────────────
print("=" * 70)
print("9. body_x TREND PER SEGMENT (by 1-second chunks)")
print("=" * 70)
print()

chunk_size_s = 1.0
t0 = float(rows[0]["elapsed_s"])
chunks = []
current_chunk = []
for r in rows:
    t = float(r["elapsed_s"])
    chunk_idx = int((t - t0) / chunk_size_s)
    if chunk_idx >= len(chunks):
        chunks.append([])
    chunks[chunk_idx].append(float(r["body_x"]))

for ci, vals in enumerate(chunks):
    t_start = t0 + ci * chunk_size_s
    print(f"  [{t_start:6.2f}-{t_start+chunk_size_s:6.2f}s]  min={min(vals):+.6f}  "
          f"max={max(vals):+.6f}  mean={sum(vals)/len(vals):+.6f}")

print()
print(f"\n--- END OF ANALYSIS ---")
