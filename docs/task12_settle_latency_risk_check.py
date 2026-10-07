"""
Task 12: empirically check, on a recorded walk, how likely the 3 risks flagged for the
"data-driven settle" (adaptive StanceSampler) design are to actually trigger during Straight
(non-turning) walking segments -- as a proxy for "flat hard floor, walking a straight line at
a casual pace."

For every stance period that occurs while MotionContextDetector reports Straight, measures:
  - stance duration (frames) -- risk 1: is it ever shorter than what's needed to "go quiet"?
  - frames-since-onset until gyro magnitude first stays < SETTLE_GYRO_THR for
    SETTLE_QUIET_FRAMES consecutive frames -- risk 2: does this ever approach/hit the
    MAX_STANCE_FRAMES_FOR_SETTLE cap?
"""
import sys, csv, math
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260922_121955.csv"
FS_HZ = 100.0

STATIC_GYRO_THR, STATIC_REQUIRED_F, STATIC_COLLECT_F, STATIC_TIMEOUT_F = 0.3, 30, 300, 1000
FREE_ACC_THR, GYRO_THR, FOOT_PITCH_THR, MIN_STANCE_F = 2.5, 1.0, 0.35, 2  # current (shrunk) MinStanceFrames=2
YAW_RATE_THR, DELTAQ_YAW_THR = 0.70, 0.17
TURNING_CONFIRM_F, STRAIGHT_CONFIRM_F, REACQ_TURNING_CONFIRM_F = 10, 20, 10

# Proposed data-driven settle parameters
MIN_STANCE_FLOOR = 2
SETTLE_GYRO_THR = 0.2      # rad/s
SETTLE_QUIET_FRAMES = 3
MAX_STANCE_FOR_SETTLE = 20  # frames (200ms)

def q_inv(q):
    x, y, z, w = q; n2 = x*x+y*y+z*z+w*w
    return (-x/n2, -y/n2, -z/n2, w/n2)

def vec3_transform(v, q):
    x, y, z, w = q; vx, vy, vz = v
    x2, y2, z2 = x+x, y+y, z+z
    wx2, wy2, wz2 = w*x2, w*y2, w*z2
    xx2, xy2, xz2 = x*x2, x*y2, x*z2
    yy2, yz2, zz2 = y*y2, y*z2, z*z2
    return (vx*(1-yy2-zz2)+vy*(xy2-wz2)+vz*(xz2+wy2),
            vx*(xy2+wz2)+vy*(1-xx2-zz2)+vz*(yz2-wx2),
            vx*(xz2-wy2)+vy*(yz2+wx2)+vz*(1-xx2-yy2))

def twist_z(q):
    _, _, z, w = q
    return 2.0 * math.atan2(z, w)

def q_norm(q):
    x, y, z, w = q; n = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n, y/n, z/n, w/n)

def median_quaternion(qs):
    xs, ys, zs, ws = [], [], [], []
    for (x, y, z, w) in qs:
        if w < 0: x, y, z, w = -x, -y, -z, -w
        xs.append(x); ys.append(y); zs.append(z); ws.append(w)
    xs.sort(); ys.sort(); zs.sort(); ws.sort()
    mid = len(qs) // 2
    return q_norm((xs[mid], ys[mid], zs[mid], ws[mid]))

frames = defaultdict(dict)
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid = int(row['PacketId']); role = row['Role'].strip()
        if role not in ('Pelvis', 'Left', 'Right'):
            continue
        q = (float(row['Qx']), float(row['Qy']), float(row['Qz']), float(row['Qw']))
        g = (float(row['Gx']), float(row['Gy']), float(row['Gz']))
        fa = (float(row['FreeAx']), float(row['FreeAy']), float(row['FreeAz']))
        dq = (float(row['DQx']), float(row['DQy']), float(row['DQz']), float(row['DQw']))
        frames[pid][role] = {'q': q, 'g': g, 'fa': fa, 'dq': dq}

pids = sorted(p for p, d in frames.items() if len(d) == 3)

static_consecutive = timeout_counter = 0
pelvis_buf, left_buf, right_buf = [], [], []
calib_idx = None
for i, pid in enumerate(pids):
    timeout_counter += 1
    if timeout_counter > STATIC_TIMEOUT_F: break
    p, l, r = frames[pid]['Pelvis'], frames[pid]['Left'], frames[pid]['Right']
    is_static = (sum(v*v for v in p['g']) < STATIC_GYRO_THR**2 and
                 sum(v*v for v in l['g']) < STATIC_GYRO_THR**2 and
                 sum(v*v for v in r['g']) < STATIC_GYRO_THR**2)
    if is_static:
        static_consecutive += 1
        if static_consecutive >= STATIC_REQUIRED_F:
            pelvis_buf.append(p['q']); left_buf.append(l['q']); right_buf.append(r['q'])
            if len(pelvis_buf) >= STATIC_COLLECT_F:
                calib_idx = i
                break
    else:
        static_consecutive = 0

left_ref = median_quaternion(left_buf)
right_ref = median_quaternion(right_buf)
calib_pid = pids[calib_idx]

def is_stance(sample, cal_ref):
    fa_mag = math.sqrt(sum(v*v for v in sample['fa']))
    g_mag = math.sqrt(sum(v*v for v in sample['g']))
    acc_ok = fa_mag < FREE_ACC_THR
    gyro_ok = g_mag < GYRO_THR
    pitch_ok = True
    if cal_ref is not None:
        g_ref = vec3_transform((0.0, 0.0, -1.0), q_inv(cal_ref))
        g_now = vec3_transform((0.0, 0.0, -1.0), q_inv(sample['q']))
        cos_tilt = max(-1.0, min(1.0, sum(a*b for a, b in zip(g_ref, g_now))))
        pitch_ok = math.acos(cos_tilt) < FOOT_PITCH_THR
    return acc_ok and gyro_ok and pitch_ok

# MotionContextDetector (same as task11) — only to exclude Turning/ReacquiringPd stances
mc_state = 'Straight'
mc_turning_frames = mc_straight_frames = mc_reacq_turning_frames = 0
mc_deltaq_yaw_accum = 0.0
def motion_context_detect(pelvis_sample):
    global mc_state, mc_turning_frames, mc_straight_frames, mc_reacq_turning_frames, mc_deltaq_yaw_accum
    wx, wy, wz = vec3_transform(pelvis_sample['g'], pelvis_sample['q'])
    pelvis_yaw_rate = abs(wz)
    mc_deltaq_yaw_accum += twist_z(pelvis_sample['dq'])
    turning_signal = pelvis_yaw_rate > YAW_RATE_THR or abs(mc_deltaq_yaw_accum) > DELTAQ_YAW_THR
    if mc_state == 'Straight':
        if turning_signal:
            mc_turning_frames += 1
            if mc_turning_frames >= TURNING_CONFIRM_F:
                mc_state = 'Turning'; mc_turning_frames = 0; mc_straight_frames = 0; mc_deltaq_yaw_accum = 0.0
                return 'Turning'
            return 'Straight'
        mc_turning_frames = 0; mc_deltaq_yaw_accum = 0.0
        return 'Straight'
    elif mc_state == 'Turning':
        mc_deltaq_yaw_accum = 0.0
        if not turning_signal:
            mc_straight_frames += 1
            if mc_straight_frames >= STRAIGHT_CONFIRM_F:
                mc_state = 'ReacquiringPd'; mc_straight_frames = 0; mc_reacq_turning_frames = 0
                return 'ReacquiringPd'
        else:
            mc_straight_frames = 0
        return 'Turning'
    else:
        mc_deltaq_yaw_accum = 0.0
        if turning_signal:
            mc_reacq_turning_frames += 1
            if mc_reacq_turning_frames >= REACQ_TURNING_CONFIRM_F:
                mc_state = 'Turning'; mc_straight_frames = 0; mc_reacq_turning_frames = 0
        else:
            mc_reacq_turning_frames = 0
        return mc_state

# Precompute the MotionContext timeline ONCE (shared by both feet) so it isn't accidentally
# re-driven twice per frame or left stale when analyzing the second foot.
mc_timeline = []
for i in range(calib_idx, len(pids)):
    mc_timeline.append(motion_context_detect(frames[pids[i]]['Pelvis']))

def analyze_foot(role, cal_ref):
    stance_count = 0
    in_stance = False
    stance_len = 0
    quiet_streak = 0
    frames_to_settle = None
    records = []  # one per ended stance: (duration_frames, frames_to_settle_or_None, mc_state_at_end)

    for idx, i in enumerate(range(calib_idx, len(pids))):
        pid = pids[i]
        f = frames[pid][role]
        state = mc_timeline[idx]

        raw = is_stance(f, cal_ref)
        stance_count = stance_count + 1 if raw else 0
        confirmed = stance_count >= MIN_STANCE_F

        if confirmed and not in_stance:
            in_stance = True; stance_len = 0; quiet_streak = 0; frames_to_settle = None
        if confirmed:
            stance_len += 1
            g_mag = math.sqrt(sum(v*v for v in f['g']))
            quiet_streak = quiet_streak + 1 if g_mag < SETTLE_GYRO_THR else 0
            if frames_to_settle is None and stance_len >= MIN_STANCE_FLOOR and \
               (quiet_streak >= SETTLE_QUIET_FRAMES or stance_len >= MAX_STANCE_FOR_SETTLE):
                frames_to_settle = stance_len
        elif in_stance:
            records.append((stance_len, frames_to_settle, state))
            in_stance = False
    return records

def pct(sorted_vals, p):
    n = len(sorted_vals)
    if n == 0: return float('nan')
    idx = min(n-1, max(0, round(p*(n-1))))
    return sorted_vals[idx]

def report(name, records):
    durations = sorted(dur for dur, settle, st in records)
    settle_vals = sorted(settle for dur, settle, st in records if settle is not None)
    n_total = len(records)
    n_dropped = sum(1 for dur, settle, st in records if settle is None)
    n_capped = sum(1 for dur, settle, st in records if settle is not None and settle >= MAX_STANCE_FOR_SETTLE)

    print(f"\n{name}: {n_total} stance periods total")
    if durations:
        print(f"  stance duration:      min={min(durations)} p10={pct(durations,.1)} median={pct(durations,.5)} "
              f"p90={pct(durations,.9)} max={max(durations)} frames ({min(durations)*10}-{max(durations)*10}ms)")
    if settle_vals:
        print(f"  frames-to-settle:     min={min(settle_vals)} p10={pct(settle_vals,.1)} median={pct(settle_vals,.5)} "
              f"p90={pct(settle_vals,.9)} max={max(settle_vals)} frames ({min(settle_vals)*10}-{max(settle_vals)*10}ms)")
    print(f"  risk 1 (never settled before swing, silently dropped): {n_dropped}/{n_total} ({100*n_dropped/n_total:.1f}%)")
    print(f"  risk 2 (hit the {MAX_STANCE_FOR_SETTLE}-frame/{MAX_STANCE_FOR_SETTLE*10}ms cap): {n_capped}/{n_total} ({100*n_capped/n_total:.1f}%)")

    # Break down risk 1 (dropped) by which MotionState the stance ended in — turning-induced
    # instability vs genuinely "normal-looking" walking should look very different here.
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

dL_records = analyze_foot('Left', left_ref)
dR_records = analyze_foot('Right', right_ref)

report("Left foot", dL_records)
report("Right foot", dR_records)

print("\nDropped (risk-1) cases, (duration_frames, mc_state) — short/marginal vs genuinely long-but-noisy:")
print("  Left: ", [(d, st) for d, s, st in dL_records if s is None])
print("  Right:", [(d, st) for d, s, st in dR_records if s is None])
