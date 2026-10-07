"""
Task 11: validate the PD-based live-angle fix (MainWindow.xaml.cs LiveFootYawDeg, using
row.PdDirectionDeg instead of a raw pelvis-quaternion twist) against the same recording
that showed the axis-swap fix (TwistX) was insufficient.

Faithfully mirrors, in order: GaitEventDetector.Detect (stance), MotionContextDetector.Detect
(Straight/Turning/ReacquiringPd), ProgressionDirEstimator.Update (PD fusion + stability +
ConfirmStraight gating) -- then computes LiveFootYawDeg = footTwistZ - PD.DirectionDeg exactly
as the C# fix now does, and compares it against the earlier single-axis-twist attempts.
"""
import sys, csv, math
from collections import defaultdict, deque

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260922_121955.csv"
FS_HZ = 100.0

STATIC_GYRO_THR, STATIC_REQUIRED_F, STATIC_COLLECT_F, STATIC_TIMEOUT_F = 0.3, 30, 300, 1000

# GaitEventDetector
FREE_ACC_THR, GYRO_THR, FOOT_PITCH_THR, MIN_STANCE_F = 2.5, 1.0, 0.35, 2
# MotionContextDetector
YAW_RATE_THR, DELTAQ_YAW_THR = 0.70, 0.17
TURNING_CONFIRM_F, STRAIGHT_CONFIRM_F, REACQ_TURNING_CONFIRM_F = 10, 20, 10
# ProgressionDirEstimator
PELVIS_W, LEFT_W, RIGHT_W = 0.6, 0.2, 0.2
STABILITY_WINDOW, STABILITY_THR, MIN_STEPS_VALID, PELVIS_AGREE_THR_DEG = 10, 0.85, 2, 15.0

def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    cx = ay*bz-az*by; cy = az*bx-ax*bz; cz = ax*by-ay*bx
    dot = ax*bx+ay*by+az*bz
    return (ax*bw+bx*aw+cx, ay*bw+by*aw+cy, az*bw+bz*aw+cz, aw*bw-dot)

def q_inv(q):
    x, y, z, w = q; n2 = x*x+y*y+z*z+w*w
    return (-x/n2, -y/n2, -z/n2, w/n2)

def q_norm(q):
    x, y, z, w = q; n = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n, y/n, z/n, w/n)

def relative_quat(now, ref):
    return q_mul(q_inv(ref), now)

def twist_z(q):
    _, _, z, w = q
    return 2.0 * math.atan2(z, w)

def vec3_transform(v, q):
    x, y, z, w = q; vx, vy, vz = v
    x2, y2, z2 = x+x, y+y, z+z
    wx2, wy2, wz2 = w*x2, w*y2, w*z2
    xx2, xy2, xz2 = x*x2, x*y2, x*z2
    yy2, yz2, zz2 = y*y2, y*z2, z*z2
    return (vx*(1-yy2-zz2)+vy*(xy2-wz2)+vz*(xz2+wy2),
            vx*(xy2+wz2)+vy*(1-xx2-zz2)+vz*(yz2-wx2),
            vx*(xz2-wy2)+vy*(yz2+wx2)+vz*(1-xx2-yy2))

def detect_heading_axis(sensor_ref):
    gx, gy, gz = vec3_transform((0.0, 0.0, -1.0), q_inv(sensor_ref))
    ax, ay, az = abs(gx), abs(gy), abs(gz)
    if ax >= ay and ax >= az: return 0
    if ay >= ax and ay >= az: return 1
    return 2

def extract_heading(q, axis):
    x, y, z, w = q
    if axis == 0: return math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    if axis == 1: return math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def norm_deg(deg):
    while deg > 180.0: deg -= 360.0
    while deg < -180.0: deg += 360.0
    return deg

def norm_rad(r):
    while r > math.pi: r -= 2*math.pi
    while r < -math.pi: r += 2*math.pi
    return r

def median_quaternion(qs):
    xs, ys, zs, ws = [], [], [], []
    for (x, y, z, w) in qs:
        if w < 0: x, y, z, w = -x, -y, -z, -w
        xs.append(x); ys.append(y); zs.append(z); ws.append(w)
    xs.sort(); ys.sort(); zs.sort(); ws.sort()
    mid = len(qs) // 2
    return q_norm((xs[mid], ys[mid], zs[mid], ws[mid]))

# ── load ─────────────────────────────────────────────────────────────────────
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

# ── calibration (same as before) ────────────────────────────────────────────
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

pelvis_ref = median_quaternion(pelvis_buf)
left_ref = median_quaternion(left_buf)
right_ref = median_quaternion(right_buf)
calib_pid = pids[calib_idx]
pelvis_axis = detect_heading_axis(pelvis_ref)
left_axis = detect_heading_axis(left_ref)
right_axis = detect_heading_axis(right_ref)
print(f"Calibration at pid={calib_pid} (t={(calib_pid-pids[0])/FS_HZ:.2f}s), "
      f"axes: pelvis={pelvis_axis} left={left_axis} right={right_axis}\n")

# ── GaitEventDetector state ─────────────────────────────────────────────────
left_stance_count = right_stance_count = 0

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

# ── MotionContextDetector state ─────────────────────────────────────────────
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
                return 'Turning', 1.0
            return 'Straight', 1.0 - mc_turning_frames / TURNING_CONFIRM_F
        mc_turning_frames = 0; mc_deltaq_yaw_accum = 0.0
        return 'Straight', 1.0
    elif mc_state == 'Turning':
        mc_deltaq_yaw_accum = 0.0
        if not turning_signal:
            mc_straight_frames += 1
            if mc_straight_frames >= STRAIGHT_CONFIRM_F:
                mc_state = 'ReacquiringPd'; mc_straight_frames = 0; mc_reacq_turning_frames = 0
                return 'ReacquiringPd', 0.0
        else:
            mc_straight_frames = 0
        return 'Turning', 1.0
    else:  # ReacquiringPd
        mc_deltaq_yaw_accum = 0.0
        if turning_signal:
            mc_reacq_turning_frames += 1
            if mc_reacq_turning_frames >= REACQ_TURNING_CONFIRM_F:
                mc_state = 'Turning'; mc_straight_frames = 0; mc_reacq_turning_frames = 0
        else:
            mc_reacq_turning_frames = 0
        return mc_state, 0.0

def confirm_straight():
    global mc_state
    if mc_state == 'ReacquiringPd':
        mc_state = 'Straight'

# ── ProgressionDirEstimator state ───────────────────────────────────────────
pd_current = 0.0
pd_has_estimate = False
pd_step_count = 0
pd_prev_context = 'Straight'
pd_history = deque()
pd_hist_cos = pd_hist_sin = 0.0
left_in_stance = right_in_stance = False
left_stance_cos = left_stance_sin = right_stance_cos = right_stance_sin = 0.0
left_stance_frames = right_stance_frames = 0
left_yaw_ready = right_yaw_ready = False
last_left_yaw = last_right_yaw = 0.0

def pd_stability():
    n = len(pd_history)
    if n < 2: return 0.0
    r_bar = math.sqrt(pd_hist_cos**2 + pd_hist_sin**2) / n
    return 1.0 / (1.0 + (1.0 - r_bar))

def pd_update(pel_q, left_q, right_q, left_stance, right_stance, mc_state_now):
    global pd_current, pd_has_estimate, pd_step_count, pd_prev_context
    global pd_hist_cos, pd_hist_sin, left_in_stance, right_in_stance
    global left_stance_cos, left_stance_sin, right_stance_cos, right_stance_sin
    global left_stance_frames, right_stance_frames, left_yaw_ready, right_yaw_ready
    global last_left_yaw, last_right_yaw

    if mc_state_now not in ('Straight', 'ReacquiringPd'):
        pd_prev_context = mc_state_now
        return pd_current, pd_has_estimate and pd_stability() >= STABILITY_THR, pd_stability()

    if mc_state_now == 'ReacquiringPd' and pd_prev_context == 'Turning':
        pd_history.clear(); pd_hist_cos = pd_hist_sin = 0.0
        pd_step_count = 0; pd_has_estimate = False
        left_yaw_ready = right_yaw_ready = False
        left_in_stance = right_in_stance = False
        left_stance_cos = left_stance_sin = right_stance_cos = right_stance_sin = 0.0
        left_stance_frames = right_stance_frames = 0
    pd_prev_context = mc_state_now

    pelvis_yaw = extract_heading(relative_quat(pel_q, pelvis_ref), pelvis_axis)

    # TrackStanceYaw
    if left_stance:
        if not left_in_stance:
            left_in_stance = True; left_stance_cos = left_stance_sin = 0.0; left_stance_frames = 0
        ly = extract_heading(relative_quat(left_q, left_ref), left_axis)
        left_stance_cos += math.cos(ly); left_stance_sin += math.sin(ly); left_stance_frames += 1
    elif left_in_stance:
        left_in_stance = False
        last_left_yaw = math.atan2(left_stance_sin, left_stance_cos) if left_stance_frames > 0 else last_left_yaw
        left_yaw_ready = True

    if right_stance:
        if not right_in_stance:
            right_in_stance = True; right_stance_cos = right_stance_sin = 0.0; right_stance_frames = 0
        ry = extract_heading(relative_quat(right_q, right_ref), right_axis)
        right_stance_cos += math.cos(ry); right_stance_sin += math.sin(ry); right_stance_frames += 1
    elif right_in_stance:
        right_in_stance = False
        last_right_yaw = math.atan2(right_stance_sin, right_stance_cos) if right_stance_frames > 0 else last_right_yaw
        right_yaw_ready = True

    if not (left_yaw_ready and right_yaw_ready):
        return pd_current, pd_has_estimate and pd_stability() >= STABILITY_THR, pd_stability()
    left_yaw_ready = right_yaw_ready = False

    pd_step_count += 1
    total_w = PELVIS_W + LEFT_W + RIGHT_W
    wx = (math.cos(pelvis_yaw)*PELVIS_W + math.cos(last_left_yaw)*LEFT_W + math.cos(last_right_yaw)*RIGHT_W) / total_w
    wy = (math.sin(pelvis_yaw)*PELVIS_W + math.sin(last_left_yaw)*LEFT_W + math.sin(last_right_yaw)*RIGHT_W) / total_w
    fused = math.atan2(wy, wx)

    pd_history.append(fused); pd_hist_cos += math.cos(fused); pd_hist_sin += math.sin(fused)
    if len(pd_history) > STABILITY_WINDOW:
        old = pd_history.popleft(); pd_hist_cos -= math.cos(old); pd_hist_sin -= math.sin(old)
    pd_current = fused; pd_has_estimate = True

    if mc_state_now == 'ReacquiringPd':
        stability = pd_stability()
        pelvis_agree_deg = abs(math.degrees(norm_rad(pd_current - pelvis_yaw)))
        if stability >= STABILITY_THR and pd_step_count >= MIN_STEPS_VALID and pelvis_agree_deg <= PELVIS_AGREE_THR_DEG:
            confirm_straight()

    return pd_current, pd_has_estimate and pd_stability() >= STABILITY_THR, pd_stability()

# ── main loop ────────────────────────────────────────────────────────────────
ts, mc_states, pd_dir_deg_series = [], [], []
angleL_pd, angleR_pd = [], []

for i in range(calib_idx, len(pids)):
    pid = pids[i]
    p, l, r = frames[pid]['Pelvis'], frames[pid]['Left'], frames[pid]['Right']

    left_stance_count = left_stance_count + 1 if is_stance(l, left_ref) else 0
    right_stance_count = right_stance_count + 1 if is_stance(r, right_ref) else 0
    left_stance = left_stance_count >= MIN_STANCE_F
    right_stance = right_stance_count >= MIN_STANCE_F

    state, conf = motion_context_detect(p)
    pd_dir, pd_valid, pd_stab = pd_update(p['q'], l['q'], r['q'], left_stance, right_stance, state)
    pd_deg = math.degrees(pd_dir)

    l_twist_deg = math.degrees(twist_z(relative_quat(l['q'], left_ref)))
    r_twist_deg = math.degrees(twist_z(relative_quat(r['q'], right_ref)))
    angleL_pd.append(norm_deg(l_twist_deg - pd_deg))
    angleR_pd.append(norm_deg(r_twist_deg - pd_deg))

    ts.append((pid - calib_pid) / FS_HZ)
    mc_states.append(state)
    pd_dir_deg_series.append(pd_deg)

n = len(ts)
print(f"{n} frames, {ts[-1]:.1f}s post-calibration\n")
print(f"{'t(s)':>7} {'MC_state':>14} {'PD_dir_deg':>11} {'angleL_pd':>10} {'angleR_pd':>10}")
step = int(0.5 * FS_HZ)
for i in range(0, n, step):
    print(f"{ts[i]:7.2f} {mc_states[i]:>14} {pd_dir_deg_series[i]:11.1f} {angleL_pd[i]:10.1f} {angleR_pd[i]:10.1f}")

TAIL_SEC = 5.0
tail_n = int(TAIL_SEC * FS_HZ)
def mean_std(vals):
    m = sum(vals)/len(vals)
    return m, math.sqrt(sum((x-m)**2 for x in vals)/len(vals))
print(f"\nFinal {TAIL_SEC:.0f}s:")
for label, series in (("angleL_pd", angleL_pd), ("angleR_pd", angleR_pd)):
    m, s = mean_std(series[-tail_n:])
    print(f"  {label:10s} mean={m:7.1f}  std={s:6.1f}")
