"""
Task 21: replay a REAL multi-calibration recording through the C# pipeline logic and test whether the
direction-dependent FPA difference (out vs. back after a 180-degree turn) comes from how
ProgressionDirEstimator extracts heading.

Inputs
  ImuSamples_*.csv         raw bundles (docs/ImuSamples_20261012_160139.csv)
  Diagnostics_*.csv (live) the REAL app run (docs/Diagnostics_20261012_160143.csv). NOT the _161022 one:
                           that is an offline replay with a single auto-calibration and an empty Stage column.

Reuses (does not copy) the classes in task17_baseline_vs_retention.py, which were checked against the C# in
IMUMoCap/Pipeline/*.cs (DataQualityGate, CalibrationProcessor, GaitEventDetector, MotionContextDetector,
ProgressionDirEstimator, FpaEngine/StanceSampler constants and logic). What task17 cannot do, this adds:

  GaitPipeline.TriggerRecalibration() (GaitPipeline.cs:153-162) resets ONLY calibration + _pd + _fpa.
  _gait and _motion keep their state. While calibrating / AwaitingRecalibration the pipeline returns before
  gait/motion/PD/FPA (GaitPipeline.cs:189,193), so those frames are NOT fed to them.
  -> a calibration boundary rebuilds ProgressionDirEstimator + both StanceSamplers, keeps gait + motion.

Calibration start times are not logged. Each calibration's completion pid is the first Diagnostics row after
a PacketId gap; the start frame is searched so that CalibrationProcessor completes exactly there.

PD heading modes (swapped by replacing task17.relative_quat / task17.extract_heading, used only by PD):
  sensor : OLD C# (before ProgressionDirEstimator.WorldHeading; matches the live recording). heading about the sensor axis most aligned with gravity, of inv(ref)*q
  world  : twist about the world vertical axis of the world-frame delta q*inv(ref)

Usage: python task21_multicalib_pd_replay.py
"""
import sys, os, csv, math, statistics as st
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.argv = [sys.argv[0]]                       # task17 reads sys.argv[1] at import time
import task17_baseline_vs_retention as t17

DOCS = os.path.dirname(os.path.abspath(__file__))
IMU = os.path.join(DOCS, "ImuSamples_20261012_160139.csv")
DIAG = os.path.join(DOCS, "Diagnostics_20261012_160143.csv")


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def cmean_deg(v):
    return math.degrees(math.atan2(sum(math.sin(math.radians(x)) for x in v),
                                   sum(math.cos(math.radians(x)) for x in v)))


# ── PD heading modes ────────────────────────────────────────────────────────────────────────
def _orig_relative_quat(now, ref):             # old C# (the live recording was made with this): inv(ref) * q
    return t17.q_mul(t17.q_inv(ref), now)


def _orig_extract_heading(q, axis):            # old C# ExtractHeading: Euler angle about the detected vertical sensor axis
    x, y, z, w = q
    if axis == 0: return math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    if axis == 1: return math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def _world_delta(now, ref):                    # world-frame delta: q * inv(ref)
    return t17.q_mul(now, t17.q_inv(ref))


def _twist_z(q, axis):                         # rotation about world vertical (Z); axis arg unused
    x, y, z, w = q
    if w < 0:
        z, w = -z, -w
    return 2.0 * math.atan2(z, w)


def set_pd_mode(mode):
    if mode == "sensor":
        t17.relative_quat, t17.extract_heading = _orig_relative_quat, _orig_extract_heading
    elif mode == "world":
        t17.relative_quat, t17.extract_heading = _world_delta, _twist_z
    else:
        raise ValueError(mode)


# ── calibration reconstruction ──────────────────────────────────────────────────────────────
def simulate_calibration(raw, accepted, start_idx):
    """CalibrationProcessor.ProcessCollecting from accepted[start_idx]. Returns (end_idx, refs) or None."""
    thr2 = t17.STATIC_GYRO_THR ** 2
    consec = timeout = 0
    buf = {"Pelvis": [], "Left": [], "Right": []}
    for i in range(start_idx, len(accepted)):
        timeout += 1
        if timeout > t17.STATIC_TIMEOUT_F:
            return None
        b = raw[accepted[i]]
        if all(sum(v * v for v in b[r]["g"]) < thr2 for r in buf):
            consec += 1
            if consec >= t17.STATIC_REQUIRED_F:
                for r in buf:
                    buf[r].append(b[r]["q"])
                if len(buf["Pelvis"]) >= t17.STATIC_COLLECT_F:
                    return i, {r: t17.median_quaternion(buf[r]) for r in buf}
        else:
            consec = 0
    return None


def load_live(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    pids = [int(r["PacketId"]) for r in rows]
    comp = [pids[0]] + [b for a, b in zip(pids, pids[1:]) if b - a > 30]
    gaps = [(a + 1, b - 1) for a, b in zip(pids, pids[1:]) if b - a > 30]   # unprocessed ranges
    live = {}
    for r in rows:
        pid = int(r["PacketId"])
        l = float(r["FPA_Left_Deg"]) if r["FPA_Left_Deg"] != "" else None
        rr = float(r["FPA_Right_Deg"]) if r["FPA_Right_Deg"] != "" else None
        if l is not None or rr is not None:
            live[pid] = (l, rr, r["Stage"], float(r["PD_DirectionDeg"]))
    return pids, comp, gaps, live


def find_calibrations(raw, accepted, comp):
    idx_of = {p: i for i, p in enumerate(accepted)}
    cals = []
    lo = 0
    for k, c in enumerate(comp):
        found = None
        for s in range(lo, idx_of[c] + 1):
            res = simulate_calibration(raw, accepted, s)
            if res and accepted[res[0]] == c:
                found = (s, res[1]); break
        if not found:
            raise SystemExit(f"calibration #{k+1}: no start frame completes at pid {c}")
        cals.append({"pid": c, "start_pid": accepted[found[0]], "refs": found[1]})
        lo = idx_of[c] + 1
    return cals


# ── replay ──────────────────────────────────────────────────────────────────────────────────
def replay(raw, accepted, cals, gaps, mode):
    set_pd_mode(mode)
    skip = set()
    for a, b in gaps:
        skip.update(range(a, b + 1))
    first_comp = cals[0]["pid"]
    gait, motion = t17.GaitEventDetector(), t17.MotionContextDetector()
    events = []                                 # (pid, foot, fpa_deg, block, leg, pd_deg)
    k = -1
    pd = ls = rs = None
    leg = 0
    prev_state = "Straight"
    for pid in accepted:
        if pid < first_comp or pid in skip:
            continue
        if k + 1 < len(cals) and pid >= cals[k + 1]["pid"]:
            k += 1
            refs = cals[k]["refs"]
            pel_ref, l_ref, r_ref = refs["Pelvis"], refs["Left"], refs["Right"]
            pd = t17.ProgressionDirEstimator(pel_ref, l_ref, r_ref)     # TriggerRecalibration: _pd.Reset()
            ls, rs = t17.StanceSampler(), t17.StanceSampler()           # TriggerRecalibration: _fpa.Reset()
            leg += 1000                                                  # a new block never shares a leg id
        p, l, r = raw[pid]["Pelvis"], raw[pid]["Left"], raw[pid]["Right"]
        lst, rst, walking = gait.detect(l, r, l_ref, r_ref)
        state, conf = motion.detect(p, walking)
        if prev_state == "Turning" and state != "Turning":
            leg += 1
        prev_state = state
        pd_dir, pd_valid, pd_stab = pd.update(p["q"], l["q"], r["q"], lst, rst, state, motion)

        if lst:
            ls.add_frame(t17.norm_rad(t17.extract_yaw_z(l["q"]) - t17.extract_yaw_z(l_ref)), math.sqrt(sum(v * v for v in l["g"])))
        else:
            ls.mark_swing()
        if rst:
            rs.add_frame(t17.norm_rad(t17.extract_yaw_z(r["q"]) - t17.extract_yaw_z(r_ref)), math.sqrt(sum(v * v for v in r["g"])))
        else:
            rs.mark_swing()

        ctx_ok = state == "Straight" and conf >= t17.CONTEXT_CONFIDENCE_THR
        pd_ok = pd_valid and pd_stab >= t17.PD_STABILITY_THR
        if ctx_ok and pd_ok and (lst or rst):
            for foot, s in (("L", ls), ("R", rs)):
                settled = s.try_settle()
                if settled is not None:
                    events.append((pid, foot, math.degrees(t17.norm_rad(settled - pd_dir)), k, leg, math.degrees(pd_dir)))
    set_pd_mode("sensor")
    return events


# ── reports ─────────────────────────────────────────────────────────────────────────────────
STAGES = ["Baseline", "Training1", "Training2", "Training3", "Retention"]


def direction_table(events, label):
    print(f"\n── {label}: direction effect per block (A = odd legs, B = even legs; leg 1 of each block is A) ──")
    print(f"{'block':10} {'foot':>4} {'A mean':>8} {'B mean':>8} {'A-B':>7} {'nA':>4} {'nB':>4} | PD turn A->B (deg)")
    for k in range(len(STAGES)):
        ev = [e for e in events if e[3] == k]
        legs = sorted({e[4] for e in ev})
        parity = {lg: i % 2 for i, lg in enumerate(legs)}
        grp = {0: defaultdict(list), 1: defaultdict(list)}
        pdg = {0: [], 1: []}
        for pid, foot, fpa, _, lg, pdd in ev:
            grp[parity[lg]][foot].append(fpa); pdg[parity[lg]].append(pdd)
        turn = wrap(cmean_deg(pdg[1]) - cmean_deg(pdg[0])) if pdg[0] and pdg[1] else float("nan")
        for foot in "LR":
            a, b = grp[0][foot], grp[1][foot]
            if len(a) > 3 and len(b) > 3:
                print(f"{STAGES[k]:10} {foot:>4} {st.mean(a):8.2f} {st.mean(b):8.2f} {st.mean(a)-st.mean(b):+7.2f} {len(a):>4} {len(b):>4} | {turn:7.1f}" if foot == "L"
                      else f"{'':10} {foot:>4} {st.mean(a):8.2f} {st.mean(b):8.2f} {st.mean(a)-st.mean(b):+7.2f} {len(a):>4} {len(b):>4} |")


def block_stats(events, label):
    print(f"\n── {label}: block mean / SD / first-25 vs last-25 ──")
    print(f"{'block':10} {'foot':>4} {'n':>4} {'mean':>7} {'SD':>6} {'first25':>8} {'last25':>8}")
    for k in range(len(STAGES)):
        for foot in "LR":
            v = [e[2] for e in events if e[3] == k and e[1] == foot]
            if len(v) >= 30:
                print(f"{STAGES[k]:10} {foot:>4} {len(v):>4} {st.mean(v):7.2f} {st.stdev(v):6.2f} {st.mean(v[:25]):8.2f} {st.mean(v[-25:]):8.2f}")


def main():
    raw, accepted = t17.load_and_gate(IMU)
    pids, comp, gaps, live = load_live(DIAG)
    print(f"live calibration completion pids: {comp}")
    print(f"unprocessed (recalibration) pid ranges: {gaps}")
    cals = find_calibrations(raw, accepted, comp)
    for i, c in enumerate(cals, 1):
        print(f"  cal #{i}: start_pid={c['start_pid']} complete_pid={c['pid']}  ({(c['pid']-c['start_pid'])/100:.1f}s)")

    ev_s = replay(raw, accepted, cals, gaps, "sensor")
    ev_w = replay(raw, accepted, cals, gaps, "world")

    # 1) fidelity: python (C# as written) vs the REAL app, same pid, same foot
    print("\n── fidelity check: python 'sensor' mode vs LIVE app output (same pid) ──")
    ok = True
    for foot, ix in (("L", 0), ("R", 1)):
        d = [live[e[0]][ix] - e[2] for e in ev_s if e[1] == foot and e[0] in live and live[e[0]][ix] is not None]
        py_n = sum(1 for e in ev_s if e[1] == foot)
        live_n = sum(1 for v in live.values() if v[ix] is not None)
        print(f"  {foot}: python steps={py_n} live steps={live_n} matched={len(d)}  "
              + (f"diff mean={st.mean(d):+.3f} SD={st.stdev(d):.3f} max|d|={max(abs(x) for x in d):.3f}" if len(d) > 1 else "no match"))
        ok = ok and len(d) > 0.95 * live_n and len(d) > 1 and max(abs(x) for x in d) < 0.1
    print("  FIDELITY:", "OK -- python reproduces the live app" if ok else "NOT OK -- do not trust the comparison below")
    if not ok:
        return

    block_stats(ev_s, "sensor (C# as written)")
    direction_table(ev_s, "sensor (C# as written)")
    block_stats(ev_w, "world (twist about vertical)")
    direction_table(ev_w, "world (twist about vertical)")


if __name__ == "__main__":
    main()
