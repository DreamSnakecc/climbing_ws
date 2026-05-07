#!/usr/bin/env python3
"""
Correct analysis of crawl gait CSV.

BUG FOUND: `_build_csv_columns` in test_crawl_gait.py has 13 columns per leg
(including `{leg}_adhesion`) but `_sample_row` only writes 12 values per leg.
This shifts all subsequent data by 4 columns.
We fix this by reading raw CSV and mapping correctly.

Actual column order per leg (12 values):
  phase, phase_id, cmd_x, cmd_y, cmd_z, cmd_support,
  ujc_x, ujc_y, ujc_z, attachment_ready, fan_rpm, fan_current
"""
import csv
import math

csv_path = "test_logs/crawl_gait_20260507_143947.csv"
legs = ["lf", "rf", "rr", "lr"]

# ------------------------------------------------------------------ #
# Read CSV correctly: skip header, parse raw values
# ------------------------------------------------------------------ #
with open(csv_path) as f:
    raw = [line.strip().split(",") for line in f.readlines()]

header = raw[0]
data_rows = raw[1:]

print(f"Header columns: {len(header)}")
print(f"Data rows: {len(data_rows)}, values per row: {len(data_rows[0])}")
print(f"Mismatch: header has {len(header)} cols but data has {len(data_rows[0])} values")
print(f"(Bug: _build_csv_columns writes {len(header)-len(data_rows[0])} extra columns)")

# Map: common(12) + leg(12 per leg) = 60 columns
# Build proper column index:
# [0:12]  = common/body
# [12:24] = lf  (12)
# [24:36] = rf  (12)
# [36:48] = rr  (12)
# [48:60] = lr  (12)

def parse_row(values):
    r = {}
    # common
    r["wall_time"] = float(values[0])
    r["ros_time"] = float(values[1])
    r["elapsed_s"] = float(values[2])
    r["mission_state"] = values[3]
    r["mission_active"] = int(values[4])
    r["phase_label"] = values[5]
    r["body_x"] = float(values[6])
    r["body_y"] = float(values[7])
    r["body_z"] = float(values[8])
    r["body_vx"] = float(values[9])
    r["body_vy"] = float(values[10])
    r["body_vz"] = float(values[11])

    for li, leg in enumerate(legs):
        off = 12 + li * 12
        r[f"{leg}_phase"] = values[off]
        r[f"{leg}_phase_id"] = int(values[off+1]) if values[off+1].lstrip('-').isdigit() else 0
        r[f"{leg}_cmd_x"] = float(values[off+2])
        r[f"{leg}_cmd_y"] = float(values[off+3])
        r[f"{leg}_cmd_z"] = float(values[off+4])
        r[f"{leg}_cmd_support"] = int(values[off+5]) if values[off+5].lstrip('-').isdigit() else 0
        r[f"{leg}_ujc_x"] = float(values[off+6])
        r[f"{leg}_ujc_y"] = float(values[off+7])
        r[f"{leg}_ujc_z"] = float(values[off+8])
        r[f"{leg}_attachment_ready"] = int(values[off+9]) if values[off+9].lstrip('-').isdigit() else 0
        r[f"{leg}_adhesion"] = float(values[off+10]) if values[off+10].lstrip('-').replace('.','').isdigit() else 0
        r[f"{leg}_fan_rpm"] = float(values[off+11])
        # fan_current_a is actually at off+12 in the header but doesn't exist in data
        # However, the values are: phase(1)+phase_id(1)+cmd(3)+support(1)+ujc(3)+attach(1)+adhesion(0)+fan_rpm(1)+fan_current(1) = 12
        # Wait, adhesion is NOT in the header for the data. Let me recount:
        # phase(1), phase_id(1), cmd_x(1), cmd_y(1), cmd_z(1), cmd_support(1),
        # ujc_x(1), ujc_y(1), ujc_z(1), attachment_ready(1), fan_rpm(1), fan_current(1) = 12

    # Actually the real data without adhesion:
    for li, leg in enumerate(legs):
        off = 12 + li * 12
        r[f"{leg}_phase"] = values[off]
        r[f"{leg}_phase_id"] = int(values[off+1])
        r[f"{leg}_cmd_x"] = float(values[off+2])
        r[f"{leg}_cmd_y"] = float(values[off+3])
        r[f"{leg}_cmd_z"] = float(values[off+4])
        r[f"{leg}_cmd_support"] = int(values[off+5])
        r[f"{leg}_ujc_x"] = float(values[off+6])
        r[f"{leg}_ujc_y"] = float(values[off+7])
        r[f"{leg}_ujc_z"] = float(values[off+8])
        r[f"{leg}_attachment_ready"] = int(values[off+9])
        r[f"{leg}_fan_rpm"] = float(values[off+10])
        r[f"{leg}_fan_current_a"] = float(values[off+11])
    return r

# Re-read
rdata = [parse_row(v) for v in data_rows]
total = len(rdata)

# ------------------------------------------------------------------ #
# Section 1: INIT baseline
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("SECTION 1: INIT BASELINE (first sample)")
print("="*80)
print(f"{'Leg':>4} | {'cmd_x':>8} {'cmd_y':>8} {'cmd_z':>8} | "
      f"{'ujc_x':>8} {'ujc_y':>8} {'ujc_z':>8} | "
      f"{'Δx':>8} {'Δy':>8} {'Δz':>8} {'Δ3d':>8}")
print("-" * 86)
for leg in legs:
    r = rdata[0]
    cx = r[f"{leg}_cmd_x"]; cy = r[f"{leg}_cmd_y"]; cz = r[f"{leg}_cmd_z"]
    ux = r[f"{leg}_ujc_x"]; uy = r[f"{leg}_ujc_y"]; uz = r[f"{leg}_ujc_z"]
    dx = ux-cx; dy = uy-cy; dz = uz-cz
    d3 = math.sqrt(dx*dx + dy*dy + dz*dz)
    print(f"{leg:>4} | {cx:>8.4f} {cy:>8.4f} {cz:>8.4f} | "
          f"{ux:>8.4f} {uy:>8.4f} {uz:>8.4f} | "
          f"{dx:>8.3f} {dy:>8.3f} {dz:>8.3f} {d3:>8.3f}")

print("\nNOTE: cmd is in BODY frame, ujc is FK output (also body frame at rest)")
print("The offset (Δ) is the static hip mounting geometry + FK zero offset.")

# ------------------------------------------------------------------ #
# Section 2: Body motion (X direction - crawl direction)
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("SECTION 2: BODY MOTION ANALYSIS")
print("="*80)
bx = [r["body_x"] for r in rdata]
by = [r["body_y"] for r in rdata]
bz = [r["body_z"] for r in rdata]
bvx = [r["body_vx"] for r in rdata]
ts = [r["elapsed_s"] for r in rdata]
dur = ts[-1] - ts[0]

dx = bx[-1] - bx[0]; dy = by[-1] - by[0]; dz = bz[-1] - bz[0]
avg_vx = sum(bvx)/len(bvx)
peak_vx = max(bvx)

# Commanded body velocity = 0.01 m/s = 10 mm/s
print(f"Duration: {dur:.1f}s")
print(f"Displacement: ({dx*1000:.1f}, {dy*1000:.1f}, {dz*1000:.1f}) mm")
print(f"Avg body_vx: {avg_vx*1000:.1f} mm/s  (cmd=10.0 mm/s)")
print(f"Peak body_vx: {peak_vx*1000:.1f} mm/s")

# Time when body moving vs stopped
moving_time = sum(1 for v in bvx if v >= 0.005) / len(bvx) * 100
print(f"Time body_vx >= 5mm/s: {moving_time:.0f}%")
print(f"Time body_vx < 5mm/s (stalled/lift): {100-moving_time:.0f}%")

# ------------------------------------------------------------------ #
# Section 3: Phase sequences per leg
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("SECTION 3: PHASE SEQUENCES & SWING CYCLES")
print("="*80)

for leg in legs:
    # Collect phase and support info
    prev_sup = None
    swing_starts = []
    support_starts = []
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if prev_sup is not None:
            if prev_sup == 1 and sup == 0:
                swing_starts.append((r["elapsed_s"], r[f"{leg}_phase"]))
            elif prev_sup == 0 and sup == 1:
                support_starts.append(r["elapsed_s"])
        prev_sup = sup

    # Count swing cycles
    swings = min(len(swing_starts), len(support_starts))
    swing_durations = []
    for i in range(swings):
        sd = support_starts[i] - swing_starts[i][0]
        swing_durations.append(sd)

    unique_phases = set(r[f"{leg}_phase"] for r in rdata)
    print(f"\n  {leg}: {swings} swing cycles, phases seen: {unique_phases}")

    if swing_starts:
        print(f"  Swing start times: {[f'{t:.1f}({p})' for t, p in swing_starts]}")
    if swing_durations:
        avg_sd = sum(swing_durations)/len(swing_durations)
        print(f"  Avg swing duration: {avg_sd:.2f}s (expected ~1.0s at 0.25Hz, swing_ratio=0.25)")

# ------------------------------------------------------------------ #
# Section 4: Tracking error analysis (corrected)
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("SECTION 4: LEG TRACKING ERROR (cmd vs actual in body frame)")
print("="*80)
print("During SUPPORT: foot stays on wall, body moves → cmd in body frame changes")
print("  Expected: cmd position drifts opposite to body motion")
print(f"  Body moved {dx*1000:.1f}mm in +X → cmd_x should drift -{dx*1000:.1f}mm per leg in body frame")
print()
print("During SWING: leg lifts off wall, cmd follows swing trajectory")
print("  Expected: actual foot (ujc) should follow cmd with small tracking error")
print()

for leg in legs:
    # SUPPORT: track cmd_x drift vs expected body motion
    support_cmd_start = []
    support_cmd_end = []
    prev_sup = 0
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if prev_sup == 0 and sup == 1:
            support_cmd_start.append(r[f"{leg}_cmd_x"])
        elif prev_sup == 1 and sup == 0:
            support_cmd_end.append(r[f"{leg}_cmd_x"])
        prev_sup = sup

    if support_cmd_start and support_cmd_end:
        # Real cmd_x drift per support period
        drifts = [e - s for s, e in zip(support_cmd_start, support_cmd_end)]
        avg_drift = sum(drifts)/len(drifts)
        print(f"  {leg} SUPPORT: cmd_x drifts avg={avg_drift*1000:.1f}mm per support period "
              f"(body moves ~{dx*1000:.1f}mm total = {dx*1000/4:.1f}mm per leg period)")

    # SWING: tracking error (residual after removing calibration offset)
    cal = rdata[0]
    cal_cx, cal_cy, cal_cz = cal[f"{leg}_cmd_x"], cal[f"{leg}_cmd_y"], cal[f"{leg}_cmd_z"]
    cal_ux, cal_uy, cal_uz = cal[f"{leg}_ujc_x"], cal[f"{leg}_ujc_y"], cal[f"{leg}_ujc_z"]
    cal_dx = cal_ux - cal_cx
    cal_dy = cal_uy - cal_cy
    cal_dz = cal_uz - cal_cz

    swing_errors = []
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if sup == 0:  # swing phase
            res_x = (r[f"{leg}_ujc_x"] - r[f"{leg}_cmd_x"]) - cal_dx
            res_y = (r[f"{leg}_ujc_y"] - r[f"{leg}_cmd_y"]) - cal_dy
            res_z = (r[f"{leg}_ujc_z"] - r[f"{leg}_cmd_z"]) - cal_dz
            swing_errors.append(math.sqrt(res_x*res_x + res_y*res_y + res_z*res_z))

    if swing_errors:
        rms = math.sqrt(sum(e*e for e in swing_errors)/len(swing_errors))
        mx = max(swing_errors)
        print(f"  {leg} SWING: 3D tracking residual rms={rms*1000:.1f}mm max={mx*1000:.1f}mm "
              f"(n={len(swing_errors)})")

# ------------------------------------------------------------------ #
# Section 5: Foot stability during support (slip detection)
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("SECTION 5: FOOT STABILITY DURING SUPPORT (slip detection)")
print("="*80)
print("Using body-frame ujc - should stay almost constant if foot grips wall")
print("(ujc in body frame = foot pos relative to body; body moves → ujc changes)")
print()

for leg in legs:
    supports = []
    cur_support = []
    prev_sup = 0
    for r in rdata:
        sup = r[f"{leg}_cmd_support"]
        if sup == 1:
            cur_support.append(r)
        elif prev_sup == 1 and cur_support:
            supports.append(cur_support)
            cur_support = []
        prev_sup = sup
    if cur_support:
        supports.append(cur_support)

    if supports:
        all_drifts = []
        for sp in supports:
            first_ujc = (sp[0][f"{leg}_ujc_x"], sp[0][f"{leg}_ujc_y"], sp[0][f"{leg}_ujc_z"])
            max_drift = 0
            for r in sp:
                dx = r[f"{leg}_ujc_x"] - first_ujc[0]
                dy = r[f"{leg}_ujc_y"] - first_ujc[1]
                dz = r[f"{leg}_ujc_z"] - first_ujc[2]
                drift = math.sqrt(dx*dx+dy*dy+dz*dz)
                if drift > max_drift:
                    max_drift = drift
            all_drifts.append(max_drift)
        avg_drift = sum(all_drifts)/len(all_drifts)
        max_drift = max(all_drifts)
        print(f"  {leg}: {len(supports)} support periods, "
              f"ujc drift avg={avg_drift*1000:.1f}mm max={max_drift*1000:.1f}mm")
        if max_drift > 0.010:
            print(f"        ⚠ Significant drift detected (foot slipping or robot compliance)")
    else:
        print(f"  {leg}: no support periods analyzed")

# ------------------------------------------------------------------ #
# KEY FINDINGS
# ------------------------------------------------------------------ #
print("\n" + "="*80)
print("KEY FINDINGS")
print("="*80)

print(f"""
1. BODY VELOCITY
   - Commanded: 10.0 mm/s
   - Measured avg: {avg_vx*1000:.1f} mm/s
   - Actual displacement: {dx*1000:.1f}mm over {dur:.1f}s
   - Body only moves at full speed {moving_time:.0f}% of the time

2. GAIT CYCLES (from test log output):
   - 4 swing cycles completed per leg in ~45s
   - Expected: 0.25 Hz sequential gait → each leg swings every 4s
     Expected cycles in 45s: ~11. But only 4 completed.

3. SWING DURATION:
   - Expected: 1.0s per swing (0.25Hz, swing_ratio=0.25)
   - But total time between support periods is >1s due to
     sequential gait (4 legs × 1s each = 4s per full cycle)

4. CSV DATA ISSUES:
   - Bug in test_crawl_gait.py: _build_csv_columns includes
     adhesion column per leg but _sample_row doesn't write it
   - This shifted ALL leg columns by 1 position per leg
   - Previous analysis tools reading with DictReader get
     completely incorrect data for all legs except LF

5. LEG TRACKING (from corrected data):
   - Static offset (hip geometry + FK zero) = ~0.23-0.35m per axis
   - Swing residual error = ~10-100mm (depends on leg and phase)
   - Support drift = should be zero if foot is locked on wall
""")

# Verify the CSV header bug by testing first row
print("VERIFYING CSV HEADER BUG:")
print(f"  Header has {len(header)} columns, data has {len(data_rows[0])} values")
leg_cols_header = [c for c in header if any(f"{leg}_" in c for leg in legs)]
print(f"  Leg columns in header: {len(leg_cols_header)} (expected 52 = 13×4)")
with open(csv_path) as f:
    header_line = f.readline().strip()
    first_data = f.readline().strip()
print(f"  Header last col: {header.split(',')[-1]}")
print(f"  Data last col value: {first_data.split(',')[-1]}")
print(f"  Data has {len(first_data.split(','))} values")
