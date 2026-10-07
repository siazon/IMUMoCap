
import sys, csv, math
from collections import defaultdict, deque

CSV_PATH = sys.argv[1]
FS_HZ = 100.0

STATIC_GYRO_THR, STATIC_REQUIRED_F, STATIC_COLLECT_F, STATIC_TIMEOUT_F = 0.3, 30, 300, 1000
FREE_ACC_THR, GYRO_THR, FOOT_PITCH_THR, MIN_STANCE_F = 2.5, 1.0, 0.35, 2
WALKING_WINDOW_F = 300
YAW_RATE_THR, DELTAQ_YAW_THR = 0.70, 0.17
TURNING_CONFIRM_F, STRAIGHT_CONFIRM_F, REACQ_TURNING_CONFIRM_F = 10, 20, 10
PELVIS_W, LEFT_W, RIGHT_W = 0.6, 0.2, 0.2
STABILITY_WINDOW, STABILITY_THR, MIN_STEPS_VALID, PELVIS_AGREE_THR_DEG = 10, 0.85, 2, 15.0
CONTEXT_CONFIDENCE_THR, PD_STABILITY_THR = 0.7, 0.7
MIN_STANCE_FLOOR, SETTLE_GYRO_THR, SETTLE_QUIET_F, MAX_STANCE_FOR_SETTLE = 2, 0.35, 3, 20
TOLERANCE_W_MIN_DEG = 4.0
TARGET_K = 1.0
BASELINE_TARGET = 100
TRAINING_BLOCK_TARGET = 100
RETENTION_TARGET = 100
BLOCK_ALPHAS = [1.5, 1.0, 0.5]

def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    cx = ay*bz - az*by; cy = az*bx - ax*bz; cz = ax*by - ay*bx
    dot = ax*bx + ay*by + az*bz
    return (ax*bw+bx*aw+cx, ay*bw+by*aw+cy, az*bw+bz*aw+cz, aw*bw-dot)

def q_inv(q):
    x, y, z, w = q; n2 = x*x+y*y+z*z+w*w
    return (-x/n2, -y/n2, -z/n2, w/n2)

def q_norm(q):
    x, y, z, w = q; n = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n, y/n, z/n, w/n)

def relative_quat(now, ref):
    return q_mul(q_inv(ref), now)

def vec3_transform(v, q):
    x, y, z, w = q; vx, vy, vz = v
    x2, y2, z2 = x+x, y+y, z+z
    wx2, wy2, wz2 = w*x2, w*y2, w*z2
    xx2, xy2, xz2 = x*x2, x*y2, x*z2
    yy2, yz2, zz2 = y*y2, y*z2, z*z2
    return (vx*(1-yy2-zz2)+vy*(xy2-wz2)+vz*(xz2+wy2),
            vx*(xy2+wz2)+vy*(1-xx2-zz2)+vz*(yz2-wx2),
            vx*(xz2-wy2)+vy*(yz2+wx2)+vz*(1-xx2-yy2))

def extract_yaw_z(q):
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def detect_heading_axis(sensor_ref):
    gx, gy, gz = vec3_transform((0.0, 0.0, -1.0), q_inv(sensor_ref))
    ax, ay, az = abs(gx), abs(gy), abs(gz)
    if ax >= ay and ax >= az: return 0
    if ay >= ax and ay >= az: return 1
    return 2

def extract_heading(q, axis):
    x, y, z, w = q
    if axis == 0:
        return math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    if axis == 1:
        return math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

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

frames = defaultdict(dict)
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid = int(row['PacketId']); role = row['Role'].strip()
        if role not in ('Pelvis', 'Left', 'Right'):
            continue
        frames[pid][role] = {
            'q': (float(row['Qx']), float(row['Qy']), float(row['Qz']), float(row['Qw'])),
            'g': (float(row['Gx']), float(row['Gy']), float(row['Gz'])),
            'fa': (float(row['FreeAx']), float(row['FreeAy']), float(row['FreeAz'])),
            'dq': (float(row['DQx']), float(row['DQy']), float(row['DQz']), float(row['DQw'])),
        }
pids = sorted(p for p, d in frames.items() if len(d) == 3)
print(f"Loaded {len(pids)} complete frames, {(pids[-1]-pids[0])/FS_HZ:.1f}s @ {FS_HZ:.0f}Hz nominal")

static_consecutive = timeout_counter = 0
pelvis_buf, left_buf, right_buf = [], [], []
calib_idx = None
for i, pid in enumerate(pids):
    timeout_counter += 1
    if timeout_counter > STATIC_TIMEOUT_F:
        break
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

if calib_idx is None:
    print("Calibration never completed — aborting."); sys.exit(1)

pelvis_ref = median_quaternion(pelvis_buf)
left_ref = median_quaternion(left_buf)
right_ref = median_quaternion(right_buf)
calib_pid = pids[calib_idx]
pelvis_axis = detect_heading_axis(pelvis_ref)
left_axis = detect_heading_axis(left_ref)
right_axis = detect_heading_axis(right_ref)
print(f"Calibration at pid={calib_pid} (t={(calib_pid-pids[0])/FS_HZ:.2f}s)")

left_stance_count = right_stance_count = 0
left_stance_prev = right_stance_prev = False
left_transitions = right_transitions = 0
walking_frame_count = 0
is_walking = False

def is_stance_raw(sample, cal_ref):
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

def gait_detect(l_sample, r_sample):
    global left_stance_count, right_stance_count, left_stance_prev, right_stance_prev
    global left_transitions, right_transitions, walking_frame_count, is_walking
    left_stance_count = left_stance_count + 1 if is_stance_raw(l_sample, left_ref) else 0
    right_stance_count = right_stance_count + 1 if is_stance_raw(r_sample, right_ref) else 0
    left_confirmed = left_stance_count >= MIN_STANCE_F
    right_confirmed = right_stance_count >= MIN_STANCE_F
    if left_confirmed != left_stance_prev:
        left_transitions += 1; left_stance_prev = left_confirmed
    if right_confirmed != right_stance_prev:
        right_transitions += 1; right_stance_prev = right_confirmed
    walking_frame_count += 1
    if walking_frame_count >= WALKING_WINDOW_F:
        is_walking = left_transitions >= 2 and right_transitions >= 2
        walking_frame_count = 0
        left_transitions = right_transitions = 0
    return left_confirmed, right_confirmed, is_walking

mc_state = 'Straight'
mc_turning_f = mc_straight_f = mc_reacq_turning_f = 0
mc_deltaq_accum = 0.0

def motion_context_detect(pelvis_sample, gait_is_walking):
    global mc_state, mc_turning_f, mc_straight_f, mc_reacq_turning_f, mc_deltaq_accum
    _, _, wz = vec3_transform(pelvis_sample['g'], pelvis_sample['q'])
    pelvis_yaw_rate = abs(wz)
    dqx, dqy, dqz, dqw = pelvis_sample['dq']
    mc_deltaq_accum += 2.0 * math.atan2(dqz, dqw)
    turning_signal = pelvis_yaw_rate > YAW_RATE_THR or abs(mc_deltaq_accum) > DELTAQ_YAW_THR
    if mc_state == 'Straight':
        if turning_signal:
            mc_turning_f += 1
            if mc_turning_f >= TURNING_CONFIRM_F:
                mc_state = 'Turning'; mc_turning_f = mc_straight_f = 0; mc_deltaq_accum = 0.0
                return 'Turning', 1.0
            conf = 1.0 - mc_turning_f / TURNING_CONFIRM_F
        else:
            mc_turning_f = 0; mc_deltaq_accum = 0.0
            conf = 1.0
        state = 'Straight'
    elif mc_state == 'Turning':
        mc_deltaq_accum = 0.0
        if not turning_signal:
            mc_straight_f += 1
            if mc_straight_f >= STRAIGHT_CONFIRM_F:
                mc_state = 'ReacquiringPd'; mc_straight_f = 0; mc_reacq_turning_f = 0
                return 'ReacquiringPd', 0.0
        else:
            mc_straight_f = 0
        state, conf = 'Turning', 1.0
    else:
        mc_deltaq_accum = 0.0
        if turning_signal:
            mc_reacq_turning_f += 1
            if mc_reacq_turning_f >= REACQ_TURNING_CONFIRM_F:
                mc_state = 'Turning'; mc_straight_f = mc_reacq_turning_f = 0
        else:
            mc_reacq_turning_f = 0
        state, conf = mc_state, 0.0
    if not gait_is_walking and state == 'Straight':
        conf *= 0.5
    return state, conf

def confirm_straight():
    global mc_state
    if mc_state == 'ReacquiringPd':
        mc_state = 'Straight'

pd_current = 0.0
pd_has_estimate = False
pd_step_count = 0
pd_prev_context = 'Straight'
pd_history = deque()
pd_hist_cos = pd_hist_sin = 0.0
left_in_stance_pd = right_in_stance_pd = False
left_stance_cos = left_stance_sin = right_stance_cos = right_stance_sin = 0.0
left_stance_frames_pd = right_stance_frames_pd = 0
left_yaw_ready = right_yaw_ready = False
last_left_yaw = last_right_yaw = 0.0

def pd_stability():
    n = len(pd_history)
    if n < 2: return 0.0
    r_bar = math.sqrt(pd_hist_cos**2 + pd_hist_sin**2) / n
    return 1.0 / (1.0 + (1.0 - r_bar))

def pd_update(pel_q, left_q, right_q, left_stance, right_stance, mc_state_now):
    global pd_current, pd_has_estimate, pd_step_count, pd_prev_context
    global pd_hist_cos, pd_hist_sin, left_in_stance_pd, right_in_stance_pd
    global left_stance_cos, left_stance_sin, right_stance_cos, right_stance_sin
    global left_stance_frames_pd, right_stance_frames_pd, left_yaw_ready, right_yaw_ready
    global last_left_yaw, last_right_yaw
    if mc_state_now not in ('Straight', 'ReacquiringPd'):
        pd_prev_context = mc_state_now
        stab = pd_stability()
        return pd_current, pd_has_estimate and stab >= STABILITY_THR, stab
    if mc_state_now == 'ReacquiringPd' and pd_prev_context == 'Turning':
        pd_history.clear(); pd_hist_cos = pd_hist_sin = 0.0
        pd_step_count = 0; pd_has_estimate = False
        left_yaw_ready = right_yaw_ready = False
        left_in_stance_pd = right_in_stance_pd = False
        left_stance_cos = left_stance_sin = right_stance_cos = right_stance_sin = 0.0
        left_stance_frames_pd = right_stance_frames_pd = 0
    pd_prev_context = mc_state_now
    pelvis_yaw = extract_heading(relative_quat(pel_q, pelvis_ref), pelvis_axis)
    if left_stance:
        if not left_in_stance_pd:
            left_in_stance_pd = True; left_stance_cos = left_stance_sin = 0.0; left_stance_frames_pd = 0
        ly = extract_heading(relative_quat(left_q, left_ref), left_axis)
        left_stance_cos += math.cos(ly); left_stance_sin += math.sin(ly); left_stance_frames_pd += 1
    elif left_in_stance_pd:
        left_in_stance_pd = False
        last_left_yaw = math.atan2(left_stance_sin, left_stance_cos) if left_stance_frames_pd > 0 else last_left_yaw
        left_yaw_ready = True
    if right_stance:
        if not right_in_stance_pd:
            right_in_stance_pd = True; right_stance_cos = right_stance_sin = 0.0; right_stance_frames_pd = 0
        ry = extract_heading(relative_quat(right_q, right_ref), right_axis)
        right_stance_cos += math.cos(ry); right_stance_sin += math.sin(ry); right_stance_frames_pd += 1
    elif right_in_stance_pd:
        right_in_stance_pd = False
        last_right_yaw = math.atan2(right_stance_sin, right_stance_cos) if right_stance_frames_pd > 0 else last_right_yaw
        right_yaw_ready = True
    if not (left_yaw_ready and right_yaw_ready):
        stab = pd_stability()
        return pd_current, pd_has_estimate and stab >= STABILITY_THR, stab
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
    stab = pd_stability()
    if mc_state_now == 'ReacquiringPd':
        pelvis_agree_deg = abs(math.degrees(norm_rad(pd_current - pelvis_yaw)))
        if stab >= STABILITY_THR and pd_step_count >= MIN_STEPS_VALID and pelvis_agree_deg <= PELVIS_AGREE_THR_DEG:
            confirm_straight()
    return pd_current, pd_has_estimate and stab >= STABILITY_THR, stab

class StanceSampler:
    def __init__(self):
        self.cos_sum = self.sin_sum = 0.0
        self.count = 0
        self.in_stance = False
        self.emitted_this_stance = False
        self.quiet_streak = 0
    def add_frame(self, yaw, gyro_mag):
        if not self.in_stance:
            self.in_stance = True; self.emitted_this_stance = False
            self.cos_sum = self.sin_sum = 0.0; self.count = 0; self.quiet_streak = 0
        self.cos_sum += math.cos(yaw); self.sin_sum += math.sin(yaw); self.count += 1
        self.quiet_streak = self.quiet_streak + 1 if gyro_mag < SETTLE_GYRO_THR else 0
    def mark_swing(self):
        self.in_stance = False
    def has_settled(self):
        return self.count >= MIN_STANCE_FLOOR and \
               (self.quiet_streak >= SETTLE_QUIET_F or self.count >= MAX_STANCE_FOR_SETTLE)
    def try_settle(self):
        if not self.in_stance or self.emitted_this_stance or not self.has_settled():
            return None
        self.emitted_this_stance = True
        return math.atan2(self.sin_sum, self.cos_sum)

left_sampler, right_sampler = StanceSampler(), StanceSampler()
pelvis_ref_yaw = extract_yaw_z(pelvis_ref)
left_ref_yaw = extract_yaw_z(left_ref)
right_ref_yaw = extract_yaw_z(right_ref)

step_events = []  # (frame_idx, pid, foot, fpa_deg)

for frame_idx, i in enumerate(range(calib_idx, len(pids))):
    pid = pids[i]
    p, l, r = frames[pid]['Pelvis'], frames[pid]['Left'], frames[pid]['Right']
    left_stance, right_stance, walking_now = gait_detect(l, r)
    mc_state_now, mc_conf = motion_context_detect(p, walking_now)
    pd_dir, pd_valid, pd_stab = pd_update(p['q'], l['q'], r['q'], left_stance, right_stance, mc_state_now)

    if left_stance:
        ly = norm_rad(extract_yaw_z(l['q']) - left_ref_yaw)
        g_mag_l = math.sqrt(sum(v*v for v in l['g']))
        left_sampler.add_frame(ly, g_mag_l)
    else:
        left_sampler.mark_swing()
    if right_stance:
        ry = norm_rad(extract_yaw_z(r['q']) - right_ref_yaw)
        g_mag_r = math.sqrt(sum(v*v for v in r['g']))
        right_sampler.add_frame(ry, g_mag_r)
    else:
        right_sampler.mark_swing()

    context_ok = (mc_state_now == 'Straight') and (mc_conf >= CONTEXT_CONFIDENCE_THR)
    pd_ok = pd_valid and (pd_stab >= PD_STABILITY_THR)
    if context_ok and pd_ok and (left_stance or right_stance):
        settled_l = left_sampler.try_settle()
        if settled_l is not None:
            fpa_l_deg = math.degrees(norm_rad(settled_l - pd_dir))
            step_events.append((frame_idx, pid, 'L', fpa_l_deg))
        settled_r = right_sampler.try_settle()
        if settled_r is not None:
            fpa_r_deg = math.degrees(norm_rad(settled_r - pd_dir))
            step_events.append((frame_idx, pid, 'R', fpa_r_deg))

n_frames = frame_idx + 1
print(f"{n_frames} frames post-calibration ({n_frames/FS_HZ:.1f}s)")
print(f"Total settled fpa steps: L={sum(1 for e in step_events if e[2]=='L')}  R={sum(1 for e in step_events if e[2]=='R')}\n")

# ---- stage segmentation mirroring GaitPipeline / BaselineProcessor / ExperimentRecorder ----
def consume_block(events, start, target):
    cl = cr = 0
    i = start
    while i < len(events) and not (cl >= target and cr >= target):
        _, _, foot, _ = events[i]
        if foot == 'L': cl += 1
        else: cr += 1
        i += 1
    return events[start:i], i, cl, cr

def mean_sd(vals):
    n = len(vals)
    if n == 0: return float('nan'), float('nan')
    m = sum(vals) / n
    if n < 2: return m, 0.0
    sd = (sum((v-m)**2 for v in vals) / (n-1)) ** 0.5
    return m, sd

def compute_target(mean, sd):
    if mean > 10.0:
        return mean - TARGET_K * sd, 'ToeIn'
    return mean + TARGET_K * sd, 'ToeOut'

def tolerance(sd, alpha):
    return max(TOLERANCE_W_MIN_DEG, alpha * sd)

def block_stats(block_events, target, tol):
    errs = {'L': [], 'R': []}
    on = {'L': 0, 'R': 0}
    for _, _, foot, fpa in block_events:
        e = fpa - target
        errs[foot].append(e)
        if abs(e) <= tol: on[foot] += 1
    return errs, on

ptr = 0
report = []

baseline_block, ptr, bl, br = consume_block(step_events, ptr, BASELINE_TARGET)
fpaL_b = [e[3] for e in baseline_block if e[2]=='L']
fpaR_b = [e[3] for e in baseline_block if e[2]=='R']
meanL, sdL = mean_sd(fpaL_b)
meanR, sdR = mean_sd(fpaR_b)
targetL, dirL = compute_target(meanL, sdL)
targetR, dirR = compute_target(meanR, sdR)
t0 = baseline_block[0][1] if baseline_block else None
t1 = baseline_block[-1][1] if baseline_block else None
print("="*90)
print(f"BASELINE  (steps used: L={bl} R={br}, both must reach {BASELINE_TARGET})")
print(f"  time span: pid {t0} -> {t1}  ({(t1-t0)/FS_HZ:.1f}s)" if baseline_block else "  (insufficient data)")
print(f"  L: mu={meanL:6.2f} SD={sdL:5.2f} -> Target={targetL:6.2f} ({dirL})")
print(f"  R: mu={meanR:6.2f} SD={sdR:5.2f} -> Target={targetR:6.2f} ({dirR})")

stage_names = ["Training1", "Training2", "Training3"]
prev_end_pid = t1
for bi, alpha in enumerate(BLOCK_ALPHAS):
    block, ptr, cl, cr = consume_block(step_events, ptr, TRAINING_BLOCK_TARGET)
    if not block:
        print(f"\n{stage_names[bi]}: NO DATA (recording ended before this block completed)")
        continue
    bt0, bt1 = block[0][1], block[-1][1]
    gap_s = (bt0 - prev_end_pid) / FS_HZ if prev_end_pid is not None else float('nan')
    tolL = tolerance(sdL, alpha); tolR = tolerance(sdR, alpha)
    errs, on = block_stats(block, targetL, tolL)  # placeholder, fixed below per-foot
    errsL = [e[3]-targetL for e in block if e[2]=='L']
    errsR = [e[3]-targetR for e in block if e[2]=='R']
    onL = sum(1 for e in errsL if abs(e) <= tolL)
    onR = sum(1 for e in errsR if abs(e) <= tolR)
    print(f"\n{stage_names[bi]}  (alpha={alpha}, steps L={cl} R={cr}, tol L={tolL:.2f} R={tolR:.2f})")
    print(f"  gap since previous block end: {gap_s:.1f}s  [protocol expects ~0s before Training3 if bi==2, ~120s rest before this if bi==1]")
    print(f"  time span: {(bt1-bt0)/FS_HZ:.1f}s")
    print(f"  L: mean error={sum(errsL)/len(errsL) if errsL else float('nan'):+.2f} deg   on-target={onL}/{len(errsL)} ({100*onL/len(errsL) if errsL else 0:.0f}%)")
    print(f"  R: mean error={sum(errsR)/len(errsR) if errsR else float('nan'):+.2f} deg   on-target={onR}/{len(errsR)} ({100*onR/len(errsR) if errsR else 0:.0f}%)")
    prev_end_pid = bt1

retention_block, ptr, rl, rr = consume_block(step_events, ptr, RETENTION_TARGET)
if retention_block:
    rt0, rt1 = retention_block[0][1], retention_block[-1][1]
    gap_s = (rt0 - prev_end_pid) / FS_HZ if prev_end_pid is not None else float('nan')
    alpha_ret = BLOCK_ALPHAS[-1]
    tolL = tolerance(sdL, alpha_ret); tolR = tolerance(sdR, alpha_ret)
    errsL = [e[3]-targetL for e in retention_block if e[2]=='L']
    errsR = [e[3]-targetR for e in retention_block if e[2]=='R']
    onL = sum(1 for e in errsL if abs(e) <= tolL)
    onR = sum(1 for e in errsR if abs(e) <= tolR)
    print(f"\nRETENTION  (steps L={rl} R={rr}, feedback OFF)")
    print(f"  gap since Training3 end: {gap_s:.1f}s  [protocol expects ~300s rest+MC+NASA-TLX+IMI-PC]")
    print(f"  time span: {(rt1-rt0)/FS_HZ:.1f}s")
    print(f"  L: mean error={sum(errsL)/len(errsL) if errsL else float('nan'):+.2f} deg   on-target={onL}/{len(errsL)} ({100*onL/len(errsL) if errsL else 0:.0f}%)")
    print(f"  R: mean error={sum(errsR)/len(errsR) if errsR else float('nan'):+.2f} deg   on-target={onR}/{len(errsR)} ({100*onR/len(errsR) if errsR else 0:.0f}%)")
else:
    print("\nRETENTION: NO DATA (recording ended before retention completed)")

remaining = len(step_events) - ptr
print(f"\n(leftover unconsumed settled steps after last completed block: {remaining})")
