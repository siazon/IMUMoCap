"""
Task 19: find a CalibrationProcessor parameter set that meaningfully shortens its wall-clock
duration without materially degrading the resulting reference-quaternion precision, using the
REAL stillness/sway signature recorded at the start of
docs/ImuSamples_20261005_130206_from_episode3.csv (the confirmed real calibration episode from
task16/task17).

Key fact from CalibrationProcessor.cs:109-113: the collection buffer is NEVER cleared on a jitter
-- only the "consecutive still frames" counter needed to RESUME appending is reset. So for a FIXED
(StaticGyroThreshold, StaticRequiredFrames) pair, the sequence of appended quaternions (and the
frame index each was appended at) is fully determined and independent of StaticCollectFrames --
StaticCollectFrames only decides how many appended entries are required before declaring
Completed. One simulation pass per (gyro_thr, required_f) pair therefore answers every
StaticCollectFrames candidate by slicing the recorded sequence.

Precision metric: angular deviation (deg) between the median quaternion computed from a
candidate's sliced buffer and the CANONICAL calibration (current production defaults: 0.3 rad/s,
30 frames, 300 frames) -- i.e. "how many degrees would this shortcut shift the zero-point away
from what the real 3s/0.3-threshold calibration actually produced", per role (report the max of
Pelvis/Left/Right since all three feed FpaEngine/ProgressionDirEstimator and one bad sensor
dominates FPA error -- see live-vs-fpa / drift-coupling analysis earlier this session).

Usage: python task19_calibration_duration_tuning.py [path/to/ImuSamples_*.csv]
"""
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task17_baseline_vs_retention as t17

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20261005_130206_from_episode3.csv"
FS_HZ = t17.FS_HZ

BASELINE_GYRO_THR, BASELINE_REQUIRED_F, BASELINE_COLLECT_F = 0.3, 30, 300
PRODUCTION_TIMEOUT_F = t17.STATIC_TIMEOUT_F  # 1000 (10s) -- real app declares Failed past this
SCAN_F = 5000  # 50s of accepted frames -- generous window to cover even the slowest candidate


def q_angle_deg(qa, qb):
    """Angular distance (deg) between two quaternions, double-cover safe."""
    dot = max(-1.0, min(1.0, abs(sum(a * b for a, b in zip(qa, qb)))))
    return math.degrees(2 * math.acos(dot))


def simulate_buffer(raw, accepted_pids, gyro_thr, required_f):
    """Replay CalibrationProcessor's append logic over the first SCAN_F accepted frames.
    Returns {role: [(frame_idx, quat), ...]} in append order, plus a jitter-interruption count."""
    gyro_thr_sq = gyro_thr * gyro_thr
    consecutive = 0
    n_resets = 0
    seq = {'Pelvis': [], 'Left': [], 'Right': []}
    for i in range(min(SCAN_F, len(accepted_pids))):
        pid = accepted_pids[i]
        p, l, r = raw[pid]['Pelvis'], raw[pid]['Left'], raw[pid]['Right']
        is_static = (sum(v * v for v in p['g']) < gyro_thr_sq and
                     sum(v * v for v in l['g']) < gyro_thr_sq and
                     sum(v * v for v in r['g']) < gyro_thr_sq)
        if is_static:
            consecutive += 1
            if consecutive >= required_f:
                seq['Pelvis'].append((i, p['q']))
                seq['Left'].append((i, l['q']))
                seq['Right'].append((i, r['q']))
        else:
            if consecutive > 0:
                n_resets += 1
            consecutive = 0
    return seq, n_resets


def main():
    raw, accepted_pids = t17.load_and_gate(CSV_PATH)
    if not accepted_pids:
        print("No frames survived DataQualityGate -- aborting.")
        return

    canon_seq, canon_resets = simulate_buffer(raw, accepted_pids, BASELINE_GYRO_THR, BASELINE_REQUIRED_F)
    if len(canon_seq['Pelvis']) < BASELINE_COLLECT_F:
        print(f"Canonical config (0.3 rad/s, 30f, 300f) never completed within {SCAN_F} frames on "
              f"this recording -- cannot establish ground truth. Aborting.")
        return
    canon_ref = {role: t17.median_quaternion([q for _, q in canon_seq[role][:BASELINE_COLLECT_F]])
                 for role in ('Pelvis', 'Left', 'Right')}
    canon_frame_idx = canon_seq['Pelvis'][BASELINE_COLLECT_F - 1][0]
    canon_time_s = canon_frame_idx / FS_HZ
    canon_idxs = [idx for idx, _ in canon_seq['Pelvis'][:BASELINE_COLLECT_F]]
    canon_resets_in_window = sum(1 for a, b in zip(canon_idxs, canon_idxs[1:]) if b - a > 1)
    print(f"Canonical (production defaults 0.3 rad/s / 30f / 300f): completes at t={canon_time_s:.2f}s, "
          f"{canon_resets_in_window} still-streak interruption(s) during its own 3s collection "
          f"({canon_resets} total jitter events seen across the full {SCAN_F}-frame scan, for reference).")
    if canon_frame_idx > PRODUCTION_TIMEOUT_F:
        print(f"  *** exceeds the real 10s ({PRODUCTION_TIMEOUT_F}f) production timeout -- the app "
              f"would have declared Failed and required a retry. ***")
    print()

    seq_cache = {}

    def eval_combo(gyro_thr, required_f, collect_f):
        key = (gyro_thr, required_f)
        if key not in seq_cache:
            seq_cache[key] = simulate_buffer(raw, accepted_pids, gyro_thr, required_f)
        seq, n_resets = seq_cache[key]
        if len(seq['Pelvis']) < collect_f:
            return None
        frame_idx = seq['Pelvis'][collect_f - 1][0]
        max_dev = 0.0
        for role in ('Pelvis', 'Left', 'Right'):
            ref = t17.median_quaternion([q for _, q in seq[role][:collect_f]])
            max_dev = max(max_dev, q_angle_deg(ref, canon_ref[role]))
        # interruptions that actually occurred DURING this candidate's own collection window
        # (gap >1 between consecutive appended frame indices, up to collect_f), not n_resets
        # (which counts jitter across the whole SCAN_F scan regardless of where collection ended)
        idxs = [idx for idx, _ in seq['Pelvis'][:collect_f]]
        resets_in_window = sum(1 for a, b in zip(idxs, idxs[1:]) if b - a > 1)
        return frame_idx / FS_HZ, max_dev, resets_in_window, frame_idx

    print("=" * 84)
    print("Sensitivity: StaticCollectFrames alone (gyro_thr=0.3, required_f=30)")
    print("=" * 84)
    for cf in (300, 200, 150, 100, 50):
        r = eval_combo(BASELINE_GYRO_THR, BASELINE_REQUIRED_F, cf)
        if r is None:
            print(f"  collect_f={cf:4d}: never reached within {SCAN_F} frames")
            continue
        t_s, dev, nr, fidx = r
        flag = "  [TIMEOUT]" if fidx > PRODUCTION_TIMEOUT_F else ""
        print(f"  collect_f={cf:4d} ({cf/FS_HZ:.1f}s of data): completes t={t_s:5.2f}s  "
              f"max_dev_vs_canonical={dev:5.2f} deg  resets={nr}{flag}")

    print()
    print("=" * 84)
    print("Sensitivity: StaticRequiredFrames alone (gyro_thr=0.3, collect_f=300)")
    print("=" * 84)
    for rf in (30, 20, 15, 10, 5):
        r = eval_combo(BASELINE_GYRO_THR, rf, BASELINE_COLLECT_F)
        if r is None:
            print(f"  required_f={rf:3d}: never reached within {SCAN_F} frames")
            continue
        t_s, dev, nr, fidx = r
        flag = "  [TIMEOUT]" if fidx > PRODUCTION_TIMEOUT_F else ""
        print(f"  required_f={rf:3d} ({rf/FS_HZ:.2f}s pre-roll): completes t={t_s:5.2f}s  "
              f"max_dev_vs_canonical={dev:5.2f} deg  resets={nr}{flag}")

    print()
    print("=" * 84)
    print("Sensitivity: StaticGyroThreshold alone (required_f=30, collect_f=300)")
    print("=" * 84)
    for gt in (0.3, 0.35, 0.4, 0.5):
        r = eval_combo(gt, BASELINE_REQUIRED_F, BASELINE_COLLECT_F)
        if r is None:
            print(f"  gyro_thr={gt:.2f}: never reached within {SCAN_F} frames")
            continue
        t_s, dev, nr, fidx = r
        flag = "  [TIMEOUT]" if fidx > PRODUCTION_TIMEOUT_F else ""
        print(f"  gyro_thr={gt:.2f} rad/s: completes t={t_s:5.2f}s  "
              f"max_dev_vs_canonical={dev:5.2f} deg  resets={nr}{flag}")

    print()
    print("=" * 84)
    print("Combined candidates")
    print("=" * 84)
    candidates = [
        (0.3, 30, 300, "current production default"),
        (0.3, 20, 200, "shorter pre-roll + shorter collection, threshold unchanged"),
        (0.3, 15, 150, "more aggressive, threshold unchanged"),
        (0.3, 20, 150, "shorter collection, moderate pre-roll"),
        (0.3, 15, 100, "aggressive on both, threshold unchanged"),
        (0.4, 20, 200, "+ loosened threshold"),
    ]
    for gt, rf, cf, label in candidates:
        r = eval_combo(gt, rf, cf)
        if r is None:
            print(f"  gyro={gt:.2f} required_f={rf:3d} collect_f={cf:4d}  [{label}]: never reached")
            continue
        t_s, dev, nr, fidx = r
        flag = "  [TIMEOUT]" if fidx > PRODUCTION_TIMEOUT_F else ""
        print(f"  gyro={gt:.2f} required_f={rf:3d} collect_f={cf:4d}  [{label}]")
        print(f"      time={t_s:5.2f}s (delta {t_s-canon_time_s:+5.2f}s vs canonical)  "
              f"max_dev={dev:5.2f} deg  resets={nr}{flag}")


if __name__ == "__main__":
    main()
