"""
Task 20: why did calibration take a long time / keep failing to complete on
docs/ImuSamples_20261007_112940.csv?

Replays CalibrationProcessor.cs's exact FSM (current defaults: 0.3 rad/s gyro threshold, 20-frame
pre-roll, 200-frame collection, 1000-frame/10s timeout -- see task19_calibration_duration_tuning.py
for how these were picked) against the REAL recorded data, through the SAME DataQualityGate the
production pipeline applies first (CalibrationProcessor never even sees a gate-rejected bundle --
_staticTimeoutCounter only increments on frames that already passed the gate, so heavy gate
rejection silently stretches the real-world time-to-timeout well past 10s without showing up in the
frame-count timeout at all).

For every attempt (TriggerStart -> Completed or Failed-by-timeout), reports:
  - wall-clock duration, number of accepted frames consumed, number of stillness-streak resets
  - which role (Pelvis/Left/Right) broke the streak most often, and by how much over threshold
  - a chronological per-role gyro-magnitude trace so a persistently-noisy sensor (vs. occasional
    genuine movement) is visible at a glance

Usage: python task20_calibration_failure_diagnosis.py [path/to/ImuSamples_*.csv]
"""
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task17_baseline_vs_retention as t17

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20261007_112940.csv"
FS_HZ = t17.FS_HZ

# Current CalibrationProcessor.cs defaults (post task19 tuning)
GYRO_THR, REQUIRED_F, COLLECT_F, TIMEOUT_F = 0.3, 20, 200, 1000


def gyro_mag(sample):
    return math.sqrt(sum(v * v for v in sample['g']))


def main():
    raw, accepted_pids = t17.load_and_gate(CSV_PATH)
    total_pids = len({p for p, d in raw.items() if len(d) == 3})
    print(f"Recording spans {total_pids} complete bundles "
          f"({total_pids/FS_HZ:.1f}s wall-clock) -> {len(accepted_pids)} survived DataQualityGate "
          f"({100*len(accepted_pids)/total_pids:.1f}% acceptance).\n")

    gyro_thr_sq = GYRO_THR * GYRO_THR
    consecutive = 0
    timeout_counter = 0
    collected = 0
    attempt_start_i = 0
    attempt_num = 0
    break_counts = {'Pelvis': 0, 'Left': 0, 'Right': 0}
    break_margins = {'Pelvis': [], 'Left': [], 'Right': []}

    def report_attempt(end_i, outcome, n_resets):
        nonlocal attempt_num
        attempt_num += 1
        start_pid, end_pid = accepted_pids[attempt_start_i], accepted_pids[end_i]
        gated_frames = end_i - attempt_start_i + 1
        print(f"Attempt {attempt_num}: {outcome} -- gated frames consumed={gated_frames} "
              f"({gated_frames/FS_HZ:.2f}s of accepted data), stillness-streak resets={n_resets}, "
              f"spans accepted-stream t={attempt_start_i/FS_HZ:.1f}s..{end_i/FS_HZ:.1f}s "
              f"(pid {start_pid}..{end_pid})")

    i = 0
    n = len(accepted_pids)
    while i < n:
        pid = accepted_pids[i]
        p, l, r = raw[pid]['Pelvis'], raw[pid]['Left'], raw[pid]['Right']
        timeout_counter += 1
        if timeout_counter > TIMEOUT_F:
            report_attempt(i - 1, "FAILED (10s timeout)", sum(break_counts.values()))
            # CalibrationProcessor.Reset() isn't called automatically by production on Failed --
            # an operator/participant retry re-triggers TriggerStart(). Model that as a fresh
            # attempt starting right here so later attempts in the same file are still visible.
            timeout_counter = 0
            consecutive = 0
            collected = 0
            attempt_start_i = i
            break_counts = {'Pelvis': 0, 'Left': 0, 'Right': 0}
            break_margins = {'Pelvis': [], 'Left': [], 'Right': []}

        gmags = {'Pelvis': gyro_mag(p), 'Left': gyro_mag(l), 'Right': gyro_mag(r)}
        is_static = all(gmags[role] ** 2 < gyro_thr_sq for role in gmags)

        if is_static:
            consecutive += 1
            if consecutive >= REQUIRED_F:
                collected += 1
                if collected >= COLLECT_F:
                    report_attempt(i, "COMPLETED", sum(break_counts.values()))
                    timeout_counter = 0
                    consecutive = 0
                    collected = 0
                    attempt_start_i = i + 1
                    break_counts = {'Pelvis': 0, 'Left': 0, 'Right': 0}
                    break_margins = {'Pelvis': [], 'Left': [], 'Right': []}
        else:
            if consecutive > 0:
                worst_role = max(gmags, key=lambda rl: gmags[rl])
                break_counts[worst_role] += 1
                break_margins[worst_role].append(gmags[worst_role])
            consecutive = 0
        i += 1

    if attempt_start_i <= n - 1 and timeout_counter <= TIMEOUT_F:
        print(f"\n(Recording ends mid-attempt {attempt_num + 1}: "
              f"{(n - 1 - attempt_start_i)/FS_HZ:.1f}s of accepted data consumed without "
              f"reaching 200 collected frames or the 10s timeout -- i.e. the file just stops here.)")

    print(f"\nStillness-streak breaks by role across the whole file (which sensor most often had "
          f"the HIGHEST gyro magnitude at the moment a streak broke -- not necessarily the only one "
          f"over threshold, but the dominant one):")
    for role in ('Pelvis', 'Left', 'Right'):
        margins = break_margins[role]
        if not margins:
            print(f"  {role}: 0 breaks")
            continue
        avg = sum(margins) / len(margins)
        print(f"  {role}: {break_counts[role]} breaks, gyro-mag at break min={min(margins):.3f} "
              f"avg={avg:.3f} max={max(margins):.3f} rad/s (threshold={GYRO_THR:.2f})")

    # Chronological gyro-magnitude trace (10 bins) per role, over the WHOLE accepted stream --
    # reveals whether one sensor is persistently noisy/above-threshold vs. only briefly disturbed.
    print(f"\nChronological gyro-magnitude trend (10 bins over {len(accepted_pids)/FS_HZ:.1f}s of "
          f"accepted data) -- mean / max per bin, per role:")
    n_bins = 10
    bin_size = max(1, len(accepted_pids) // n_bins)
    for role in ('Pelvis', 'Left', 'Right'):
        print(f"  {role}:")
        for b in range(0, len(accepted_pids), bin_size):
            chunk = accepted_pids[b:b + bin_size]
            if not chunk:
                continue
            mags = [gyro_mag(raw[pid][role]) for pid in chunk]
            t_start, t_end = b / FS_HZ, (b + len(chunk)) / FS_HZ
            print(f"    t={t_start:5.1f}-{t_end:5.1f}s: mean={sum(mags)/len(mags):.3f}  "
                  f"max={max(mags):.3f} rad/s")


if __name__ == "__main__":
    main()
