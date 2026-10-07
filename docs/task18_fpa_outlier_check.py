"""
Task 18: diagnose why ImuSamples_20261005_143616.csv's right-foot FPA readings look unusually
large/noisy (Baseline R: mean=-12.64 deg, SD=11.69 deg -- vs the previous session's R SD=4.27 deg)
by dumping every settled step's raw angle alongside the gate context at the moment it settled
(MotionContext state/confidence, PD stability, gyro-at-settle) -- reuses task17's pipeline port
directly (same CalibrationProcessor/GaitEventDetector/MotionContextDetector/
ProgressionDirEstimator/FpaEngine replication, verified against current C# source) rather than
re-deriving it.

Usage: python task18_fpa_outlier_check.py [path/to/ImuSamples_*.csv] [outlier_threshold_deg]
"""
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task17_baseline_vs_retention as t17

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20261005_143616.csv"
OUTLIER_THRESHOLD_DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
FS_HZ = t17.FS_HZ


def main():
    raw, accepted_pids = t17.load_and_gate(CSV_PATH)
    if not accepted_pids:
        print("No frames survived DataQualityGate -- aborting."); return

    static_consecutive = timeout_counter = 0
    pelvis_buf, left_buf, right_buf = [], [], []
    calib_idx = None
    for i, pid in enumerate(accepted_pids):
        timeout_counter += 1
        if timeout_counter > t17.STATIC_TIMEOUT_F: break
        p, l, r = raw[pid]['Pelvis'], raw[pid]['Left'], raw[pid]['Right']
        is_static = (sum(v*v for v in p['g']) < t17.STATIC_GYRO_THR**2 and
                     sum(v*v for v in l['g']) < t17.STATIC_GYRO_THR**2 and
                     sum(v*v for v in r['g']) < t17.STATIC_GYRO_THR**2)
        if is_static:
            static_consecutive += 1
            if static_consecutive >= t17.STATIC_REQUIRED_F:
                pelvis_buf.append(p['q']); left_buf.append(l['q']); right_buf.append(r['q'])
                if len(pelvis_buf) >= t17.STATIC_COLLECT_F:
                    calib_idx = i
                    break
        else:
            static_consecutive = 0

    if calib_idx is None:
        print("Calibration never completed -- aborting."); return

    pelvis_ref = t17.median_quaternion(pelvis_buf)
    left_ref = t17.median_quaternion(left_buf)
    right_ref = t17.median_quaternion(right_buf)
    calib_pid = accepted_pids[calib_idx]
    print(f"Calibration at pid={calib_pid} (t={(calib_pid-accepted_pids[0])/FS_HZ:.2f}s)\n")

    gait = t17.GaitEventDetector()
    motion = t17.MotionContextDetector()
    pd = t17.ProgressionDirEstimator(pelvis_ref, left_ref, right_ref)
    left_sampler, right_sampler = t17.StanceSampler(), t17.StanceSampler()

    events = []  # (t_sec, foot, fpa_deg, mc_state, mc_conf, pd_stab, pd_dir_deg, settle_frames, gyro_at_settle)
    t0_pid = calib_pid

    for i in range(calib_idx, len(accepted_pids)):
        pid = accepted_pids[i]
        p, l, r = raw[pid]['Pelvis'], raw[pid]['Left'], raw[pid]['Right']

        left_stance, right_stance, walking_now = gait.detect(l, r, left_ref, right_ref)
        mc_state, mc_conf = motion.detect(p, walking_now)
        pd_dir, pd_valid, pd_stab = pd.update(p['q'], l['q'], r['q'], left_stance, right_stance, mc_state, motion)

        l_gmag = math.sqrt(sum(v*v for v in l['g']))
        r_gmag = math.sqrt(sum(v*v for v in r['g']))
        if left_stance:
            ly = t17.norm_rad(t17.extract_yaw_z(l['q']) - t17.extract_yaw_z(left_ref))
            left_sampler.add_frame(ly, l_gmag)
        else:
            left_sampler.mark_swing()
        if right_stance:
            ry = t17.norm_rad(t17.extract_yaw_z(r['q']) - t17.extract_yaw_z(right_ref))
            right_sampler.add_frame(ry, r_gmag)
        else:
            right_sampler.mark_swing()

        context_ok = (mc_state == 'Straight') and (mc_conf >= t17.CONTEXT_CONFIDENCE_THR)
        pd_ok = pd_valid and (pd_stab >= t17.PD_STABILITY_THR)
        t_sec = (pid - t0_pid) / FS_HZ
        if context_ok and pd_ok and (left_stance or right_stance):
            settled_l = left_sampler.try_settle()
            if settled_l is not None:
                fpa_l_deg = math.degrees(t17.norm_rad(settled_l - pd_dir))
                events.append((t_sec, 'L', fpa_l_deg, mc_state, mc_conf, pd_stab,
                               math.degrees(pd_dir), left_sampler.count, l_gmag))
            settled_r = right_sampler.try_settle()
            if settled_r is not None:
                fpa_r_deg = math.degrees(t17.norm_rad(settled_r - pd_dir))
                events.append((t_sec, 'R', fpa_r_deg, mc_state, mc_conf, pd_stab,
                               math.degrees(pd_dir), right_sampler.count, r_gmag))

    print(f"{len(events)} total settled steps.\n")

    print(f"{'t(s)':>7} {'foot':>4} {'fpa_deg':>8} {'mc_state':>14} {'mc_conf':>7} {'pd_stab':>7} "
          f"{'pd_dir':>7} {'settle_f':>8} {'gyro@settle':>11}")
    outliers = [e for e in events if abs(e[2]) >= OUTLIER_THRESHOLD_DEG]
    print(f"\n{len(outliers)} step(s) with |fpa| >= {OUTLIER_THRESHOLD_DEG:.0f} deg:")
    for t_sec, foot, fpa_deg, mc_state, mc_conf, pd_stab, pd_dir_deg, settle_f, gyro in outliers:
        print(f"{t_sec:7.1f} {foot:>4} {fpa_deg:8.1f} {mc_state:>14} {mc_conf:7.2f} {pd_stab:7.2f} "
              f"{pd_dir_deg:7.1f} {settle_f:8d} {gyro:11.3f}")

    # Full chronological R-foot trace (first 60 steps) -- the foot the user flagged -- so the
    # overall pattern (steady drift vs scattered spikes vs bimodal) is visible at a glance.
    r_events = [e for e in events if e[1] == 'R']
    print(f"\nFirst 60 right-foot steps (of {len(r_events)} total), chronological:")
    print(f"{'t(s)':>7} {'fpa_deg':>8} {'mc_state':>14} {'mc_conf':>7} {'pd_stab':>7} {'settle_f':>8}")
    for t_sec, foot, fpa_deg, mc_state, mc_conf, pd_stab, pd_dir_deg, settle_f, gyro in r_events[:60]:
        print(f"{t_sec:7.1f} {fpa_deg:8.1f} {mc_state:>14} {mc_conf:7.2f} {pd_stab:7.2f} {settle_f:8d}")


if __name__ == "__main__":
    main()
