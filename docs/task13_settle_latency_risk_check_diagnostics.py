"""
Task 13: same risk-1/risk-2 check as task12, but against a Diagnostics_*.csv export instead
of a raw ImuSamples_*.csv. Diagnostics rows already carry the pipeline's OWN computed
GaitEvent_LeftStance/RightStance and MotionContext_State (one row per completed 100Hz frame,
no need to re-derive calibration/quaternion math or re-implement GaitEventDetector) — so stance
periods here reflect the actual GaitEventDetector.MinStanceFrames value in effect when this
file was recorded, not a re-simulation.

Only the settle-detection part (frames-to-quiet based on per-frame foot gyro magnitude) is
re-simulated in Python, exactly mirroring the "data-driven settle" design being evaluated.
"""
import sys, csv, math
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\Diagnostics_20260721_133630.csv"
FS_HZ = 100.0

MIN_STANCE_FLOOR = 2
SETTLE_QUIET_FRAMES = 3
MAX_STANCE_FOR_SETTLE = 20  # frames (200ms)

def pct(sorted_vals, p):
    n = len(sorted_vals)
    if n == 0: return float('nan')
    idx = min(n - 1, max(0, round(p * (n - 1))))
    return sorted_vals[idx]

def b(s):
    return s.strip() == 'True'

rows = []
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        rows.append({
            'pid': int(row['PacketId']),
            'left_stance': b(row['GaitEvent_LeftStance']),
            'right_stance': b(row['GaitEvent_RightStance']),
            'left_gyr': (float(row['Left_GyrX']), float(row['Left_GyrY']), float(row['Left_GyrZ'])),
            'right_gyr': (float(row['Right_GyrX']), float(row['Right_GyrY']), float(row['Right_GyrZ'])),
            'mc_state': row['MotionContext_State'].strip(),
            'stage': row['Stage'].strip(),
        })

print(f"Loaded {len(rows)} frames, {(rows[-1]['pid'] - rows[0]['pid']) / FS_HZ:.1f}s @ {FS_HZ:.0f}Hz nominal")
stages = defaultdict(int)
for r in rows: stages[r['stage']] += 1
print(f"Stage breakdown: {dict(stages)}\n")

def analyze_foot(stance_key, gyr_key, settle_gyro_thr):
    in_stance = False
    stance_len = 0
    quiet_streak = 0
    frames_to_settle = None
    records = []  # (duration_frames, frames_to_settle_or_None, mc_state_at_end)

    for r in rows:
        stance = r[stance_key]
        if stance and not in_stance:
            in_stance = True; stance_len = 0; quiet_streak = 0; frames_to_settle = None
        if stance:
            stance_len += 1
            g_mag = math.sqrt(sum(v * v for v in r[gyr_key]))
            quiet_streak = quiet_streak + 1 if g_mag < settle_gyro_thr else 0
            if frames_to_settle is None and stance_len >= MIN_STANCE_FLOOR and \
               (quiet_streak >= SETTLE_QUIET_FRAMES or stance_len >= MAX_STANCE_FOR_SETTLE):
                frames_to_settle = stance_len
        elif in_stance:
            records.append((stance_len, frames_to_settle, r['mc_state']))
            in_stance = False
    return records

def report(name, records):
    durations = sorted(dur for dur, settle, st in records)
    settle_vals = sorted(settle for dur, settle, st in records if settle is not None)
    n_total = len(records)
    n_dropped = sum(1 for dur, settle, st in records if settle is None)
    n_capped = sum(1 for dur, settle, st in records if settle is not None and settle >= MAX_STANCE_FOR_SETTLE)

    print(f"{name}: {n_total} stance periods total")
    if durations:
        print(f"  stance duration:  min={min(durations)} p10={pct(durations,.1)} median={pct(durations,.5)} "
              f"p90={pct(durations,.9)} max={max(durations)} frames ({min(durations)*10}-{max(durations)*10}ms)")
    if settle_vals:
        print(f"  frames-to-settle: min={min(settle_vals)} p10={pct(settle_vals,.1)} median={pct(settle_vals,.5)} "
              f"p90={pct(settle_vals,.9)} max={max(settle_vals)} frames ({min(settle_vals)*10}-{max(settle_vals)*10}ms)")
    print(f"  risk 1 (never settled before swing, silently dropped): {n_dropped}/{n_total} ({100*n_dropped/max(1,n_total):.1f}%)")
    print(f"  risk 2 (hit the {MAX_STANCE_FOR_SETTLE}-frame/{MAX_STANCE_FOR_SETTLE*10}ms cap): {n_capped}/{n_total} ({100*n_capped/max(1,n_total):.1f}%)")

    by_state_total = defaultdict(int)
    by_state_dropped = defaultdict(int)
    for dur, settle, st in records:
        by_state_total[st] += 1
        if settle is None:
            by_state_dropped[st] += 1
    print("  risk 1 breakdown by MotionState at stance-end:")
    for st in ('Straight', 'ReacquiringPd', 'Turning'):
        tot = by_state_total.get(st, 0)
        drop = by_state_dropped.get(st, 0)
        if tot:
            print(f"    {st:14s}: {drop}/{tot} dropped ({100*drop/tot:.1f}%)")

print("=" * 70)
for thr in (0.2, 0.3, 0.4):
    print(f"\n--- SettleGyroThreshold = {thr} rad/s ---")
    recL = analyze_foot('left_stance', 'left_gyr', thr)
    recR = analyze_foot('right_stance', 'right_gyr', thr)
    report("Left foot", recL)
    report("Right foot", recR)
