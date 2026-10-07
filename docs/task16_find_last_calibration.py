"""
Task 16: find every standing-still episode in a raw ImuSamples_*.csv that is long enough to be a
calibration-quality period (3s of all-three-IMU stillness, mirrored frame-for-frame from
CalibrationProcessor.cs), then help tell which of those episodes are REAL "Start Calibration"
attempts versus unrelated pauses (rest, waiting, etc.) elsewhere in the same recording -- using
pelvis orientation as the signal: genuine calibration attempts face the same direction as each
other (episode [1] is taken as confirmed-calibration, per the user), unrelated pauses generally
don't.

Why episodes, not raw windows: a single still period easily produces several back-to-back
"calibration completions" once CalibrationProcessor's 3s collection window is satisfied
repeatedly while the person keeps standing still, so raw windows are first grouped into episodes
(gap <= CLUSTER_GAP_SEC apart = same episode) before orientation comparison or file-trimming are
even considered.

Mirrors CalibrationProcessor.cs exactly, including the one easy-to-miss detail: on a jitter frame
(not static), only the consecutive-static counter resets to 0 -- the collection buffer itself is
NOT cleared ("抖动时暂停采集但不清空缓冲区"), so brief jitter inside an otherwise-still period
doesn't throw away already-collected frames.

Usage:
  python task16_find_last_calibration.py [path/to/ImuSamples_*.csv]
      -- lists episodes with their heading-vs-episode-[1] comparison, writes nothing.
  python task16_find_last_calibration.py [path/to/ImuSamples_*.csv] <episode number>
      -- also writes <input>_from_episode<N>.csv, kept rows = PacketId >= that episode's start.
         Original file is never modified or deleted.
"""
import sys
import csv
import math
from collections import defaultdict

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20261005_130206.csv"
FS_HZ = 100.0

# ── CalibrationProcessor.cs defaults ─────────────────────────────────────────────────────────
STATIC_GYRO_THR = 0.3     # rad/s
STATIC_REQUIRED_F = 30    # 0.3s continuous stillness before collection starts
STATIC_COLLECT_F = 300    # 3s of collected (not necessarily contiguous-after-jitter) frames
STATIC_TIMEOUT_F = 1000   # 10s hard ceiling per attempt before it's abandoned and retried here


def is_static(p_g, l_g, r_g):
    thr_sq = STATIC_GYRO_THR * STATIC_GYRO_THR
    return (sum(v * v for v in p_g) < thr_sq
            and sum(v * v for v in l_g) < thr_sq
            and sum(v * v for v in r_g) < thr_sq)


def extract_yaw(q):
    """Same hardcoded-Z-axis yaw as FpaEngine.ExtractYaw / MainWindow.RawFootYawDeg."""
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def norm_deg(d):
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d


def find_all_calibrations(pids, frames):
    """Returns a list of (collect_start_idx, complete_idx) frame-index pairs, one per completed
    calibration, scanning the whole recording and auto-restarting the FSM after each completion
    or timeout (mirrors clicking "Start Calibration" again)."""
    results = []
    static_consecutive = 0
    timeout_counter = 0
    buf_len = 0          # we only need the COUNT for completion detection, not the quaternions
    collect_start_idx = None

    i = 0
    n = len(pids)
    while i < n:
        pid = pids[i]
        p, l, r = frames[pid]['Pelvis'], frames[pid]['Left'], frames[pid]['Right']

        timeout_counter += 1
        if timeout_counter > STATIC_TIMEOUT_F:
            # This attempt timed out (CalibrationState.Failed) -- reset and keep scanning,
            # same as an operator retrying "Start Calibration".
            static_consecutive = 0; timeout_counter = 0; buf_len = 0; collect_start_idx = None
            continue  # re-examine this same frame as the first frame of a fresh attempt

        if is_static(p['g'], l['g'], r['g']):
            static_consecutive += 1
            if static_consecutive >= STATIC_REQUIRED_F:
                if buf_len == 0:
                    collect_start_idx = i - STATIC_REQUIRED_F + 1  # stillness actually began here
                buf_len += 1
                if buf_len >= STATIC_COLLECT_F:
                    results.append((collect_start_idx, i))
                    static_consecutive = 0; timeout_counter = 0; buf_len = 0; collect_start_idx = None
        else:
            static_consecutive = 0  # buffer (buf_len) is deliberately NOT reset here

        i += 1

    return results


# Consecutive calibration-completion points closer together than this are treated as the same
# standing-still episode (jitter/re-triggering within one attempt), not separate episodes.
CLUSTER_GAP_SEC = 10.0

# How close an episode's mean pelvis heading has to be to episode [1]'s (confirmed calibration)
# to be flagged as "also facing the same way" -- participants don't stand perfectly still, so this
# is deliberately loose (not a tight sensor-mounting-accuracy threshold).
HEADING_MATCH_THRESHOLD_DEG = 20.0

# How much heading change right after leaving a still episode counts as "a turn" (rest-and-turn)
# versus "walked straight out" (calibration -> baseline). Loose, same reasoning as the above.
TURN_MATCH_THRESHOLD_DEG = 25.0


def cluster_calibrations(calibrations, pids, pids0):
    """Groups raw completion points into episodes by gap; returns one list of member
    (start_idx, complete_idx) windows per episode."""
    episodes = []
    cur_windows = []
    cur_prev_complete_idx = None
    for start_idx, complete_idx in calibrations:
        if not cur_windows:
            cur_windows = [(start_idx, complete_idx)]
        else:
            gap_sec = (pids[start_idx] - pids[cur_prev_complete_idx]) / FS_HZ
            if gap_sec <= CLUSTER_GAP_SEC:
                cur_windows.append((start_idx, complete_idx))
            else:
                episodes.append(cur_windows)
                cur_windows = [(start_idx, complete_idx)]
        cur_prev_complete_idx = complete_idx
    episodes.append(cur_windows)
    return episodes


def episode_pelvis_heading_deg(windows, pids, frames):
    """Circular mean of the pelvis's raw (hardcoded-Z) yaw across every frame in every still
    window belonging to this episode -- an orientation fingerprint to tell apart "this is another
    calibration, facing the same way" from "this is just an unrelated pause, facing elsewhere"."""
    cos_sum = sin_sum = 0.0
    n = 0
    for start_idx, complete_idx in windows:
        for i in range(start_idx, complete_idx + 1):
            yaw = extract_yaw(frames[pids[i]]['Pelvis']['q'])
            cos_sum += math.cos(yaw); sin_sum += math.sin(yaw); n += 1
    return math.degrees(math.atan2(sin_sum, cos_sum)), n


# How much walking (not standing) right before/after an episode to average into
# heading_before/heading_after -- a few seconds covers several gait cycles, smoothing out the
# per-step pelvis sway without reaching into a different activity.
TURN_WINDOW_SEC = 2.5


def heading_around(pids, frames, center_idx, direction, window_sec=TURN_WINDOW_SEC):
    """Circular mean pelvis yaw over `window_sec` seconds of frames immediately before
    (direction=-1) or after (direction=+1) center_idx -- NOT restricted to static frames, this is
    meant to capture the walking that leads into / resumes out of a still episode."""
    n_frames = int(window_sec * FS_HZ)
    cos_sum = sin_sum = 0.0
    n = 0
    if direction < 0:
        lo, hi = max(0, center_idx - n_frames), center_idx - 1
    else:
        lo, hi = center_idx + 1, min(len(pids) - 1, center_idx + n_frames)
    for i in range(lo, hi + 1):
        yaw = extract_yaw(frames[pids[i]]['Pelvis']['q'])
        cos_sum += math.cos(yaw); sin_sum += math.sin(yaw); n += 1
    if n == 0:
        return None, 0
    return math.degrees(math.atan2(sin_sum, cos_sum)), n


def count_kept_rows(cut_pid):
    kept = total = 0
    with open(CSV_PATH, encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            total += 1
            if int(row['PacketId']) >= cut_pid:
                kept += 1
    return kept, total


def write_from(cut_pid, fieldnames, suffix):
    out_path = (CSV_PATH[:-4] if CSV_PATH.lower().endswith(".csv") else CSV_PATH) + suffix
    kept = total = 0
    with open(CSV_PATH, encoding='utf-8-sig') as fin, \
         open(out_path, 'w', newline='', encoding='utf-8') as fout:
        reader = csv.DictReader(fin)
        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()
        for row in reader:
            total += 1
            if int(row['PacketId']) >= cut_pid:
                writer.writerow(row)
                kept += 1
    return out_path, kept, total


def main():
    frames = defaultdict(dict)
    with open(CSV_PATH, encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            pid = int(row['PacketId']); role = row['Role'].strip()
            if role not in ('Pelvis', 'Left', 'Right'):
                continue
            frames[pid][role] = {
                'g': (float(row['Gx']), float(row['Gy']), float(row['Gz'])),
                'q': (float(row['Qx']), float(row['Qy']), float(row['Qz']), float(row['Qw'])),
            }

    pids = sorted(p for p, d in frames.items() if len(d) == 3)
    pids0 = pids[0]
    total_sec = (pids[-1] - pids0) / FS_HZ
    print(f"Loaded {len(pids)} complete (Pelvis+Left+Right) frames, {total_sec:.1f}s @ {FS_HZ:.0f}Hz nominal\n")

    calibrations = find_all_calibrations(pids, frames)
    if not calibrations:
        print("No calibration-quality still period (3s, all three IMUs) found anywhere in this "
              "recording -- nothing to trim. Leaving the file untouched.")
        return

    episodes = cluster_calibrations(calibrations, pids, pids0)
    print(f"Found {len(calibrations)} raw still-window(s), grouped into {len(episodes)} standing episode(s) "
          f"(gap <= {CLUSTER_GAP_SEC:.0f}s = same episode):\n")

    # Heading is computed from each episode's FIRST window only, not an all-windows average --
    # a real CalibrationProcessor completes once, at the first qualifying 3s window (Process()
    # stops calling ProcessCollecting once State != CollectingStaticPose), so that's the only
    # moment that's actually comparable to another episode's calibration moment. Later sub-windows
    # in a multi-window episode can reflect the person having shifted stance while just continuing
    # to stand near the laptop (operating it, waiting, etc.) -- not a second calibration-relevant
    # orientation.
    headings = [episode_pelvis_heading_deg([windows[0]], pids, frames) for windows in episodes]
    ref_heading_deg, _ = headings[0]  # episode [1] is confirmed-calibration per the user

    candidates = []
    for k, windows in enumerate(episodes):
        start_idx, end_idx = windows[0][0], windows[-1][1]
        cut_pid = pids[start_idx]
        start_t = (cut_pid - pids0) / FS_HZ
        kept, total = count_kept_rows(cut_pid)
        candidates.append((cut_pid, start_t))
        heading_deg, n_frames = headings[k]
        diff_from_ep1 = abs(norm_deg(heading_deg - ref_heading_deg))

        # The real CalibrationProcessor only ever completes ONCE per "Start Calibration" click
        # (Process() stops calling ProcessCollecting once State != CollectingStaticPose) -- so if
        # this episode is calibration, the moment that matters is the FIRST window's completion,
        # not wherever the last jittery re-detection in this episode happens to end. Report both.
        first_end_idx = windows[0][1]
        before_deg, n_before = heading_around(pids, frames, start_idx, direction=-1)
        after_ep_deg, _ = heading_around(pids, frames, end_idx, direction=+1)
        after_first_deg, _ = heading_around(pids, frames, first_end_idx, direction=+1)
        turn_before = abs(norm_deg(heading_deg - before_deg)) if before_deg is not None else None
        turn_out_ep_end = abs(norm_deg(after_ep_deg - heading_deg)) if after_ep_deg is not None else None
        turn_out_1st_end = abs(norm_deg(after_first_deg - heading_deg)) if after_first_deg is not None else None

        # Combined verdict, per the user: a genuine 2nd calibration should (a) face the same way
        # as episode [1]'s first window, AND (b) walk straight out immediately after its own first
        # window completes (CalibrationProcessor fires once, at the first qualifying 3s window --
        # see note above). Either alone is noisy (e.g. two different rests can coincidentally face
        # similar directions, or a calibration's first-window-exit can still show some stance-
        # adjustment jitter) -- requiring both together is the actual ask.
        heading_matches = (k == 0) or diff_from_ep1 <= HEADING_MATCH_THRESHOLD_DEG
        walks_straight_out = (turn_out_1st_end is not None and turn_out_1st_end <= TURN_MATCH_THRESHOLD_DEG)
        is_calib_like = heading_matches and walks_straight_out
        if k == 0:
            tag = "reference (confirmed calibration)"
        elif is_calib_like:
            tag = "CALIBRATION-like: same heading as [1] AND walks straight out"
        elif walks_straight_out and not heading_matches:
            tag = "walks straight out, but DIFFERENT heading from [1] -- not a heading match"
        elif heading_matches and not walks_straight_out:
            tag = "same-ish heading as [1], but turns right after -- not a clean walk-out"
        else:
            tag = "different heading AND turns after -- likely a rest/pause"

        print(f"  [{k+1}] episode starts t={start_t:7.1f}s (pid={cut_pid}), "
              f"{len(windows)} still-window(s), {total_sec - start_t:6.1f}s / "
              f"{100*kept/total:5.1f}% kept if cut here")
        print(f"       pelvis heading (1st window) {heading_deg:7.1f}° (n={n_frames}), "
              f"{diff_from_ep1:5.1f}° from [1]'s heading")
        print(f"       turn into stop: {('%5.1f' % turn_before) if turn_before is not None else '  n/a'}°   "
              f"turn out @1st-window-end: {('%5.1f' % turn_out_1st_end) if turn_out_1st_end is not None else '  n/a'}°   "
              f"turn out @episode-end: {('%5.1f' % turn_out_ep_end) if turn_out_ep_end is not None else '  n/a'}°")
        print(f"       -> {tag}")

    if len(sys.argv) > 2:
        choice = int(sys.argv[2])
        cut_pid, start_t = candidates[choice - 1]
        out_path, kept, total = write_from(cut_pid, fieldnames, f"_from_episode{choice}.csv")
        print(f"\nCutting at episode [{choice}] (t={start_t:.1f}s, pid={cut_pid}).")
        print(f"Wrote {out_path}")
        print(f"  kept {kept}/{total} rows ({100*kept/total:.1f}%)")
        print(f"  original file left untouched: {CSV_PATH}")
    else:
        print("\nNo episode chosen — run again with the episode number as a 2nd argument to write "
              "the trimmed file, e.g.:")
        print(f"  python {sys.argv[0]} \"{CSV_PATH}\" <episode number>")


if __name__ == "__main__":
    main()
