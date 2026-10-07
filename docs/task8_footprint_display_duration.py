"""
Task 8: EF stone-overlay footprint display duration analysis

Estimates a good value for FootOutDisplaySec (FootHudController.cs) from a real
recorded walk, by measuring how often each foot actually lands (stance onset).

Why this matters: in OnFpaUpdate's EF branch, every FPA emission for a foot does
    CancelInvoke(nameof(HideFootOutL)); Invoke(nameof(HideFootOutL), FootOutDisplaySec);
i.e. the hide-timer for that foot resets on every new step. If FootOutDisplaySec is
close to or longer than the real gap between consecutive steps of the SAME foot, the
timer keeps getting pushed back before it fires, so the footprint never actually
hides — this is the "一直在显示" (always showing) symptom the user reported with the
current 1.0s value. The fix is to measure the real per-foot step-to-step gap from an
actual walk and pick a duration comfortably shorter than it.

Detection mirrors GaitEventDetector.IsStance's two magnitude gates:
    accOk  = |freeAcc| < FreeAccStanceThreshold (2.5 m/s²)
    gyroOk = |gyro|     < GyroThreshold          (1.0 rad/s)
plus the same 5-frame debounce (MinStanceFrames). The pitch gate is skipped: it
needs a calibration reference quaternion, and this CSV is a mid-session capture
(PacketId doesn't start at the session's calibration stomp) so no calibration
reference is available here — the pitch gate mainly rejects a tilted-but-still foot
near a turn, it does not materially change *how often* a foot lands, which is all
this script measures.
"""
import sys, math, csv
from collections import Counter

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260817_162335.csv"
FS_HZ = 100.0  # sample rate — mirrors ImuFrameCollector.SampleRateHz

FREE_ACC_THR = 2.5   # m/s² — GaitEventDetector.FreeAccStanceThreshold
GYRO_THR     = 1.0   # rad/s — GaitEventDetector.GyroThreshold
MIN_STANCE_F = 5     # frames — GaitEventDetector.MinStanceFrames

# ── load ─────────────────────────────────────────────────────────────────────
frames = {}
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid = int(row['PacketId']); role = row['Role'].strip()
        if role not in ('Left', 'Right'):
            continue
        fa = (float(row['FreeAx']), float(row['FreeAy']), float(row['FreeAz']))
        g  = (float(row['Gx']), float(row['Gy']), float(row['Gz']))
        frames.setdefault(pid, {})[role] = (fa, g)

pids = sorted(frames.keys())
print(f"Loaded {len(pids)} distinct PacketId frames spanning "
      f"{(pids[-1]-pids[0])/FS_HZ:.1f}s @ {FS_HZ:.0f}Hz")

# ── stance detection per foot (mirrors GaitEventDetector.IsStance, minus pitch) ─
def stance_onsets(role):
    """Returns the list of PacketIds where this foot transitions swing->stance
    (debounced), i.e. approximate footfall/heel-strike times."""
    onsets = []
    stance_count = 0
    prev_confirmed = False
    for pid in pids:
        f = frames[pid].get(role)
        if f is None:
            continue
        fa, g = f
        fa_mag = math.sqrt(sum(v*v for v in fa))
        g_mag  = math.sqrt(sum(v*v for v in g))
        raw_stance = fa_mag < FREE_ACC_THR and g_mag < GYRO_THR
        stance_count = stance_count + 1 if raw_stance else 0
        confirmed = stance_count >= MIN_STANCE_F
        if confirmed and not prev_confirmed:
            onsets.append(pid)
        prev_confirmed = confirmed
    return onsets

l_onsets = stance_onsets('Left')
r_onsets = stance_onsets('Right')

# ── interval analysis: real gap (seconds) between consecutive footfalls ──────
def intervals_sec(pid_list):
    return [(b - a) / FS_HZ for a, b in zip(pid_list, pid_list[1:])]

def pct(sorted_vals, p):
    n = len(sorted_vals)
    if n == 0:
        return float('nan')
    idx = min(n - 1, max(0, round(p * (n - 1))))
    return sorted_vals[idx]

def report(name, iv):
    if not iv:
        print(f"{name}: no data")
        return None
    s = sorted(iv)
    n = len(s)
    print(f"{name}: n={n}  min={min(s):.3f}s  p10={pct(s,0.10):.3f}s  p25={pct(s,0.25):.3f}s  "
          f"median={pct(s,0.50):.3f}s  mean={sum(s)/n:.3f}s  max={max(s):.3f}s")
    return s

l_iv = intervals_sec(l_onsets)
r_iv = intervals_sec(r_onsets)
# interleave both feet by time to get the overall "some foot just landed" cadence
any_onsets = sorted(l_onsets + r_onsets)
any_iv = intervals_sec(any_onsets)

print(f"\nLeft  foot footfalls: {len(l_onsets)}   Right foot footfalls: {len(r_onsets)}\n")
l_sorted = report("Left  foot step-to-step gap (same-foot consecutive footfalls)", l_iv)
r_sorted = report("Right foot step-to-step gap (same-foot consecutive footfalls)", r_iv)
report("Any-foot gap (overall footfall cadence, either foot)", any_iv)

# ── recommendation ────────────────────────────────────────────────────────────
# Use the 10th percentile of the same-foot step-to-step gap (robust to a stray
# short/long interval) as the "fast walking" reference for that foot, then take
# only a fraction of it so the flash reliably finishes hiding before the next
# same-foot step lands even at that pace.
SAFETY_FRACTION     = 0.5    # display duration <= 50% of the tightest realistic per-foot gap
MIN_PERCEPTIBLE_SEC = 0.15   # below this a flash barely registers as visible feedback
MAX_SEC              = 0.5   # upper guard so a slow/sparse walk doesn't recommend something long

candidates = [pct(l_sorted, 0.10) if l_sorted else math.inf,
              pct(r_sorted, 0.10) if r_sorted else math.inf]
fastest_p10_gap = min(candidates)

if math.isinf(fastest_p10_gap):
    print("\nNot enough repeated footfalls to recommend a duration from this recording.")
else:
    recommended = min(MAX_SEC, max(MIN_PERCEPTIBLE_SEC, fastest_p10_gap * SAFETY_FRACTION))
    print(f"\nFastest per-foot step-to-step gap (p10, robust to outliers): {fastest_p10_gap:.3f}s")
    print(f"Recommended FootOutDisplaySec: ~{recommended:.2f}s "
          f"({SAFETY_FRACTION:.0%} of {fastest_p10_gap:.3f}s, clamped to [{MIN_PERCEPTIBLE_SEC}, {MAX_SEC}]s)")
    print("(current FootOutDisplaySec = 1.0s in FootHudController.cs — that is longer than "
          "most of the measured same-foot gaps above, so the footprint rarely finishes hiding "
          "before the next step re-triggers it — matches the '一直在显示' complaint.)")
