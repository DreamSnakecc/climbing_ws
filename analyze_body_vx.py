#!/usr/bin/env python3
"""Analyze body velocity vs leg phases to understand sequential gait behavior."""
import math

csv_path = "test_logs/crawl_gait_20260507_143947.csv"
legs = ["lf", "rf", "rr", "lr"]

with open(csv_path) as f:
    raw = [line.strip().split(",") for line in f.readlines()]
data_rows = raw[1:]

def parse_row(values):
    r = {}
    r["elapsed_s"] = float(values[2])
    r["body_x"] = float(values[6])
    r["body_y"] = float(values[7])
    r["body_z"] = float(values[8])
    r["body_vx"] = float(values[9])
    for li, leg in enumerate(legs):
        off = 12 + li * 12
        r[f"{leg}_phase"] = values[off]
        r[f"{leg}_phase_id"] = int(values[off+1]) if values[off+1].lstrip('-').isdigit() else 0
        r[f"{leg}_cmd_x"] = float(values[off+2])
        r[f"{leg}_cmd_y"] = float(values[off+3])
        r[f"{leg}_cmd_z"] = float(values[off+4])
        r[f"{leg}_cmd_support"] = int(values[off+5])
        r[f"{leg}_ujc_x"] = float(values[off+6])
        r[f"{leg}_ujc_y"] = float(values[off+7])
        r[f"{leg}_ujc_z"] = float(values[off+8])
        r[f"{leg}_fan_rpm"] = float(values[off+10])
    return r

rdata = [parse_row(v) for v in data_rows]

print("=" * 90)
print("SEQUENTIAL GAIT BODY VELOCITY ANALYSIS")
print("=" * 90)

# 1. Total body_x, body_vx stats per phase
phase_stats = {}
for r in rdata:
    # Determine what phase(s) we're in (look at all legs - global gait phase)
    swing_legs = [leg for leg in legs if r[f"{leg}_phase"] == "LIFT_SWING"]
    admit_legs = [leg for leg in legs if r[f"{leg}_phase"] == "ADMIT_OR_PRELOAD"]
    support_legs = [leg for leg in legs if r[f"{leg}_cmd_support"] == 1]
    
    if len(swing_legs) == 1:
        label = f"SWING({swing_legs[0]})"
    elif len(admit_legs) == 1:
        label = f"ADMIT({admit_legs[0]})"
    elif all(r[f"{leg}_cmd_support"] == 1 for leg in legs):
        label = "ALL_SUPPORT"
    else:
        n_support = sum(1 for leg in legs if r[f"{leg}_cmd_support"] == 1)
        label = f"TRANSITION({n_support}sup)"
    
    if label not in phase_stats:
        phase_stats[label] = {"count": 0, "vx_sum": 0.0, "vx_vals": []}
    phase_stats[label]["count"] += 1
    phase_stats[label]["vx_sum"] += r["body_vx"]
    phase_stats[label]["vx_vals"].append(r["body_vx"])

# Print stats grouped by phase category
print(f"\n{'Phase Category':>30} | {'Samples':>8} | {'Time%':>8} | {'Avg Vx(mm/s)':>13} | {'Max Vx':>8}")
print("-" * 76)

for label in sorted(phase_stats.keys()):
    s = phase_stats[label]
    avg_vx = s["vx_sum"] / s["count"] * 1000
    max_vx = max(s["vx_vals"]) * 1000
    pct = s["count"] / len(rdata) * 100
    print(f"{label:>30} | {s['count']:>8} | {pct:>7.1f}% | {avg_vx:>10.1f}  | {max_vx:>7.1f}")

# 2. Per-leg: body_vx during each of their phases
print(f"\n{'='*90}")
print("PER-LEG BODY VELOCITY BY PHASE")
print(f"{'='*90}")
for leg in legs:
    phase_vx = {}
    for r in rdata:
        phase = r[f"{leg}_phase"]
        vx = r["body_vx"]
        if phase not in phase_vx:
            phase_vx[phase] = []
        phase_vx[phase].append(vx)
    
    print(f"\n  {leg}:")
    for phase in ["SUPPORT", "LIFT_SWING", "ADMIT_OR_PRELOAD"]:
        vals = phase_vx.get(phase, [])
        if vals:
            avg = sum(vals)/len(vals)*1000
            mx = max(vals)*1000
            mn = min(vals)*1000
            print(f"    {phase:>18}: n={len(vals):5d} | vx_avg={avg:6.1f} vx_max={mx:6.1f} vx_min={mn:6.1f} mm/s")

# 3. Detailed body_x vs body_vx over time (focus on one swing cycle)
print(f"\n{'='*90}")
print("DETAILED BODY ADVANCEMENT DURING ONE LF SWING CYCLE")
print(f"{'='*90}")

# Find a clean LF swing cycle
lf_swing_start = None
lf_swing_samples = []
for r in rdata:
    if r["lf_phase"] == "LIFT_SWING" and lf_swing_start is None:
        lf_swing_start = r["elapsed_s"]
    if lf_swing_start is not None:
        lf_swing_samples.append(r)
        if lf_swing_start is not None and r["lf_phase"] == "SUPPORT" and len(lf_swing_samples) > 5:
            break

if lf_swing_samples:
    start_t = lf_swing_samples[0]["elapsed_s"]
    init_body_x = lf_swing_samples[0]["body_x"]
    init_body_vx = lf_swing_samples[0]["body_vx"]
    
    print(f"\n  LF swing cycle: {lf_swing_samples[0]['elapsed_s']:.3f}s → {lf_swing_samples[-1]['elapsed_s']:.3f}s")
    print(f"{'t(s)':>8} | {'dt(s)':>8} | {'body_x(mm)':>12} | {'body_vx(mm/s)':>15} | {'dx(mm)':>8} | {'lf_phase':>18} | {'rf_phase':>18}")
    print("-" * 95)
    
    prev_x = init_body_x
    for i, r in enumerate(lf_swing_samples):
        t = r["elapsed_s"]
        bx = r["body_x"]
        bvx = r["body_vx"]
        dx = (bx - prev_x) * 1000
        prev_x = bx
        dt = t - start_t
        print(f"{t:>8.3f} | {dt:>8.3f} | {bx*1000:>10.3f}  | {bvx*1000:>12.1f}  | {dx:>7.2f} | {r['lf_phase']:>18} | {r['rf_phase']:>18}")
        if i > 50:  # limit output
            print(f"  ... ({len(lf_swing_samples)} total samples)")
            break
    
    # Measure the advancement
    swing_advancement_1 = (lf_swing_samples[-1]["body_x"] - lf_swing_samples[0]["body_x"]) * 1000
    print(f"\n  Total body advancement during this LF cycle: {swing_advancement_1:.2f} mm")

# 4. Compare to computed: should be velocity * time
print(f"\n{'='*90}")
print("KEY FINDING: SEQUENTIAL GAIT TIMING")
print(f"{'='*90}")

# Count all swing start events
swing_events = []
for leg in legs:
    prev_sup = 0
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if prev_sup == 0 and sup == 1:
            swing_events.append((r["elapsed_s"], f"{leg}_END"))
        elif prev_sup == 1 and sup == 0:
            swing_events.append((r["elapsed_s"], f"{leg}_START"))
        prev_sup = sup

swing_events.sort()

# Find all SWING_START events per leg
print(f"\n  Swing events (sorted):")
for t, evt in swing_events:
    print(f"    t={t:7.3f}s {evt}")

# Full cycle: body_x progression vs cumulative advancement
print(f"\n{'='*90}")
print("BODY DISPLACEMENT ANALYSIS")
print(f"{'='*90}")
body_x_first = rdata[0]["body_x"]
body_x_last = rdata[-1]["body_x"]
total_disp = (body_x_last - body_x_first) * 1000
total_time = rdata[-1]["elapsed_s"] - rdata[0]["elapsed_s"]

# Count total number of swing cycles (support_leg transitions)
total_swings = 0
for leg in legs:
    prev_sup = None
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if prev_sup == 1 and sup == 0:
            total_swings += 1
        prev_sup = sup

avg_vx = sum(r["body_vx"] for r in rdata) / len(rdata) * 1000

# Compute: how many seconds was body ACTUALLY moving
moving_samples = sum(1 for r in rdata if r["body_vx"] > 0.001)
moving_time = moving_samples / len(rdata) * total_time  # seconds

print(f"\n  Recording duration: {total_time:.1f}s")
print(f"  Body displacement: {total_disp:.1f}mm")
print(f"  Average body_vx: {avg_vx:.2f} mm/s")
print(f"  Nominal cmd velocity: 10 mm/s")
print(f"  Total swings recorded: {total_swings}")
print(f"  Body actively moving for: {moving_time:.1f}s ({moving_samples}/{len(rdata)} samples)")
print(f"  Effective duty cycle (moving/total): {moving_time/total_time*100:.1f}%")
print(f"  Theoretical duty cycle (full model): 4 * swing_time / cycle_time")
print()
print(f"  Average advancement per swing: {total_disp/total_swings:.1f}mm")
print(f"  Expected advancement per swing (10mm/s × avg_swing_duration):")

# Calculate advancement per swing from actual data
total_advancement_from_vx = sum(r["body_vx"] for r in rdata) * (total_time / len(rdata))
print(f"  Integrated advancement from body_vx: {total_advancement_from_vx*1000:.1f}mm")

# Per-swing advancement
for leg in legs:
    per_swing = []
    prev_sup = 0
    swing_start_x = None
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if prev_sup == 0 and sup == 1:
            # swing just ended
            if swing_start_x is not None:
                per_swing.append((r["body_x"] - swing_start_x) * 1000)
                swing_start_x = None
        elif prev_sup == 1 and sup == 0:
            swing_start_x = r["body_x"]
        prev_sup = sup
    if per_swing:
        print(f"  {leg}: per-swing advancement (mm): {', '.join(f'{v:.1f}' for v in per_swing)}")
        print(f"        avg={sum(per_swing)/len(per_swing):.1f}mm total={sum(per_swing):.1f}mm")
