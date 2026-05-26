#!/usr/bin/env python3
"""
Analyze a crawl-gait CSV log from the climbing robot.
Pure stdlib (csv + math only) — no pandas/numpy dependency.
"""

import csv
import math

CSV_PATH = "/home/cc/climbing_ws/test_logs/crawl_gait_20260430_113450.csv"
LEGS = ["lf", "rf", "rr", "lr"]
PHASE_MAP = {
    0: "SUPPORT",
    1: "LIFT_SWING",
    2: "PRELOAD",
    3: "ADMIT",
    4: "RELEASE_WAIT",
    5: "LIFT",
    6: "TRANSFER",
}
ZERO_TOL = 1e-6

# ---------------------------------------------------------------------------
# 1. Load
# ---------------------------------------------------------------------------
rows = []
with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    for r in reader:
        # Convert numeric fields
        out = {}
        for k, v in r.items():
            k = k.strip()
            v = v.strip()
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
        rows.append(out)

n = len(rows)
print(f"Loaded {n} rows, {len(rows[0])} columns")
print(f"Time span: {rows[0]['elapsed_s']:.2f}s → {rows[-1]['elapsed_s']:.2f}s")
print()

# Helper to get a column as floats
def col(name):
    return [r[name] for r in rows]

# Helper for column per leg
def leg_col(leg, field):
    return [r[f"{leg}_{field}"] for r in rows]

# ---------------------------------------------------------------------------
# 2. Body motion
# ---------------------------------------------------------------------------
print("=" * 72)
print("1. BODY MOTION")
print("=" * 72)
bx = col("body_x")
by = col("body_y")
bz = col("body_z")
bx_start, bx_end = bx[0], bx[-1]
bx_delta = bx_end - bx_start
bx_min, bx_max = min(bx), max(bx)

print(f"  body_x:  {bx_start:.4f} → {bx_end:.4f}  (\u0394 = {bx_delta:.4f} m)")
print(f"  body_x range: [{bx_min:.4f}, {bx_max:.4f}]")
print(f"  body_y range: [{min(by):.4f}, {max(by):.4f}]")
print(f"  body_z range: [{min(bz):.4f}, {max(bz):.4f}]")

if bx_delta > 0.001:
    print("  \u2713 body_x INCREASES \u2014 body is moving forward.")
elif bx_delta < -0.001:
    print("  \u2717 body_x DECREASES \u2014 body is moving backward!")
else:
    print("  ~ body_x is nearly constant \u2014 no meaningful forward motion.")

num_dec = sum(1 for i in range(1, n) if bx[i] - bx[i-1] < -1e-6)
print(f"  Number of small backward steps in body_x: {num_dec} / {n-1}")

cum_max = max(bx[i] - bx[0] for i in range(n))
print(f"  Max cumulative forward displacement: {cum_max:.4f} m")
print()

# ---------------------------------------------------------------------------
# 3. Support-leg cmd_x drift
# ---------------------------------------------------------------------------
print("=" * 72)
print("2. SUPPORT-LEG CMD_X DRIFT (should decrease to push body forward)")
print("=" * 72)

for leg in LEGS:
    supp = leg_col(leg, "cmd_support")
    cmd_x = leg_col(leg, "cmd_x")
    # gather values when cmd_support == 1
    vals = [cmd_x[i] for i in range(n) if supp[i] == 1.0]
    nsupp = len(vals)
    if nsupp == 0:
        print(f"  {leg.upper()}: never in support \u2014 skipping.")
        continue
    first, last = vals[0], vals[-1]
    delta = last - first
    vmin, vmax = min(vals), max(vals)
    print(f"  {leg.upper()}: support cmd_x {first:.4f} \u2192 {last:.4f}  (\u0394 = {delta:.4f} m)")
    print(f"           range [{vmin:.4f}, {vmax:.4f}] over {nsupp} support rows")
    if delta < -0.001:
        print(f"           \u2713 cmd_x DECREASES \u2014 leg is moving backward in body frame (good).")
    elif delta > 0.001:
        print(f"           \u2717 cmd_x INCREASES \u2014 leg is NOT moving backward (bad!).")
    else:
        print(f"           ~ cmd_x nearly constant \u2014 no meaningful drift.")
    print()

# ---------------------------------------------------------------------------
# 4. ujc tracking for support legs
# ---------------------------------------------------------------------------
print("=" * 72)
print("3. UJC TRACKING ERROR FOR SUPPORT LEGS (cmd vs measured)")
print("=" * 72)

for leg in LEGS:
    supp = leg_col(leg, "cmd_support")
    nsupp = sum(1 for v in supp if v == 1.0)
    if nsupp == 0:
        continue
    for axis in ["x", "y", "z"]:
        cmd = leg_col(leg, f"cmd_{axis}")
        ujc = leg_col(leg, f"ujc_{axis}")
        errs = [ujc[i] - cmd[i] for i in range(n) if supp[i] == 1.0]
        mean_e = sum(errs) / len(errs)
        std_e = math.sqrt(sum((e - mean_e)**2 for e in errs) / len(errs))
        max_abs = max(abs(e) for e in errs)
        print(f"  {leg.upper()}_{axis}: mean_error={mean_e:.4f}, std={std_e:.4f}, max|error|={max_abs:.4f}")
    print()

# ---------------------------------------------------------------------------
# 5. Swing counts per leg
# ---------------------------------------------------------------------------
print("=" * 72)
print("4. SWING COUNTS PER LEG")
print("=" * 72)

for leg in LEGS:
    pid = leg_col(leg, "phase_id")
    cnt = sum(1 for i in range(1, n) if pid[i] != 0.0 and pid[i-1] == 0.0)
    print(f"  {leg.upper()}: {cnt} swing cycles")
print()

# ---------------------------------------------------------------------------
# 6. Anomalies
# ---------------------------------------------------------------------------
print("=" * 72)
print("5. ANOMALIES")
print("=" * 72)

# 6a. Multiple legs swinging simultaneously
print("5a. Multiple legs out of support simultaneously:")
multi_rows = []
for i in range(n):
    swinging = [leg for leg in LEGS if rows[i][f"{leg}_phase_id"] != 0.0]
    if len(swinging) > 1:
        phases = {leg: PHASE_MAP.get(int(rows[i][f"{leg}_phase_id"]), "?") for leg in swinging}
        multi_rows.append((rows[i]["elapsed_s"], swinging, phases))

if multi_rows:
    print(f"    Found {len(multi_rows)} rows with >1 leg swinging:")
    for ts, legs_list, phases in multi_rows[:15]:  # cap output
        print(f"      elapsed={ts:.3f}s  legs: {legs_list}  phases: {phases}")
    if len(multi_rows) > 15:
        print(f"      ... and {len(multi_rows) - 15} more rows")
else:
    print("    None \u2014 at most one leg swings at a time \u2713")
print()

# 6b. Stuck legs
print("5b. Legs stuck in non-support phases (consecutive rows in same non-zero phase):")
for leg in LEGS:
    pid = leg_col(leg, "phase_id")
    elapsed = col("elapsed_s")
    i = 0
    while i < n:
        if pid[i] != 0.0:
            phase_start = i
            phase_val = pid[i]
            while i < n and pid[i] == phase_val:
                i += 1
            phase_end = i - 1
            run_len = phase_end - phase_start + 1
            if run_len >= 2:
                dur = elapsed[phase_end] - elapsed[phase_start]
                pname = PHASE_MAP.get(int(phase_val), "?")
                print(f"    {leg.upper()}: {pname} for {dur:.2f}s ({run_len} rows, "
                      f"elapsed {elapsed[phase_start]:.3f}\u2013{elapsed[phase_end]:.3f})")
        else:
            i += 1
print()

# ---------------------------------------------------------------------------
# 7. Forward body motion per swing cycle
# ---------------------------------------------------------------------------
print("=" * 72)
print("6. FORWARD BODY MOTION \u2014 DETAILED VIEW")
print("=" * 72)

swing_cycles = []
for leg in LEGS:
    pid = leg_col(leg, "phase_id")
    elapsed = col("elapsed_s")
    bx = col("body_x")
    # find transitions from SUPPORT(0) to non-zero
    i = 0
    while i < n:
        if pid[i] != 0.0 and (i == 0 or pid[i-1] == 0.0):
            t_start = elapsed[i]
            bx_start = bx[i]
            # find end of this swing (back to SUPPORT)
            j = i + 1
            while j < n and pid[j] != 0.0:
                j += 1
            if j < n:
                t_end = elapsed[j]
                bx_end = bx[j]
                swing_cycles.append((leg.upper(), t_start, t_end, bx_start, bx_end, bx_end - bx_start))
            i = j
        else:
            i += 1

if swing_cycles:
    swing_cycles.sort(key=lambda x: x[1])  # sort by t_start
    delta_sym = "\u0394"
    print(f"  {'Leg':>4s} {'Start(s)':>9s} {'End(s)':>9s} {'bx_start':>10s} {'bx_end':>10s} {f'{delta_sym}bx':>8s}")
    print("  " + "-" * 58)
    for s in swing_cycles:
        leg_name, ts, te, bxs, bxe, dx = s
        mark = "\u2713" if dx > 0 else ("\u2717" if dx < 0 else "~")
        print(f"  {leg_name:>4s} {ts:>9.3f} {te:>9.3f}  {bxs:>10.4f}  {bxe:>10.4f}  {dx:>+8.4f} {mark}")

    avg_dx = sum(s[5] for s in swing_cycles) / len(swing_cycles)
    pos = sum(1 for s in swing_cycles if s[5] > 0)
    neg = sum(1 for s in swing_cycles if s[5] < 0)
    print(f"\n  Average \u0394body_x per swing cycle: {avg_dx:.4f} m")
    print(f"  Cycles with forward motion: {pos}, backward: {neg}")
else:
    print("  No swing cycles detected!")
print()

# ---------------------------------------------------------------------------
# 8. Summary
# ---------------------------------------------------------------------------
print("=" * 72)
print("SUMMARY")
print("=" * 72)

dur = rows[-1]["elapsed_s"] - rows[0]["elapsed_s"]
print(f"  Total duration: {dur:.2f}s")

if bx_delta > 0.001:
    print(f"  \u2713 Body moves forward: \u0394body_x = {bx_delta:.4f} m")
elif bx_delta < -0.001:
    print(f"  \u2717 Body moves backward: \u0394body_x = {bx_delta:.4f} m")
else:
    print(f"  ~ Body stationary: \u0394body_x = {bx_delta:.4f} m")

print(f"\n  Support-leg cmd_x drift (should decrease):")
for leg in LEGS:
    supp = leg_col(leg, "cmd_support")
    cmd_x = leg_col(leg, "cmd_x")
    vals = [cmd_x[i] for i in range(n) if supp[i] == 1.0]
    if vals:
        d = vals[-1] - vals[0]
        mark = "\u2713 decreases" if d < -0.001 else ("\u2717 increases!" if d > 0.001 else "~ constant")
        print(f"    {leg.upper()}: \u0394 = {d:.4f} m ({mark})")

print(f"\n  Swing cycles per leg:")
for leg in LEGS:
    pid = leg_col(leg, "phase_id")
    cnt = sum(1 for i in range(1, n) if pid[i] != 0.0 and pid[i-1] == 0.0)
    print(f"    {leg.upper()}: {cnt}")

if multi_rows:
    print(f"\n  \u26a0  {len(multi_rows)} rows with >1 leg swinging (see anomaly section)")
else:
    print(f"\n  \u2713 No simultaneous multi-leg swings")

print(f"\n  Mean |tracking error| across support legs:")
for leg in LEGS:
    supp = leg_col(leg, "cmd_support")
    nsupp = sum(1 for v in supp if v == 1.0)
    if nsupp:
        for axis in ["x", "y", "z"]:
            cmd = leg_col(leg, f"cmd_{axis}")
            ujc = leg_col(leg, f"ujc_{axis}")
            errs = [abs(ujc[i] - cmd[i]) for i in range(n) if supp[i] == 1.0]
            avg_e = sum(errs) / len(errs)
            print(f"    {leg.upper()}_{axis}: {avg_e:.4f} m")

print()
