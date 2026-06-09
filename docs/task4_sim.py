"""
Task 4 Verification: ProgressionDirEstimator on ImuSamples_20260608_161118.csv
Simulates full pipeline: Calibration → GaitEvent → MotionContext → PD Estimator
Focus: does ConfirmStraight() get called after turns?
"""
import sys, math, csv
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')

CSV_PATH = r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260608_161118.csv"

# ── Parameters (mirrors C# defaults) ──────────────────────────────────────────
STATIC_GYRO_THRESHOLD = 0.3
STATIC_REQUIRED_FRAMES = 10
STATIC_COLLECT_FRAMES = 50
STOMP_ACC_THRESHOLD = 25.0
STOMP_MAX_FRAMES = 20

FREE_ACC_STANCE_THRESHOLD = 2.5
GYRO_STANCE_THRESHOLD = 1.0
FOOT_PITCH_THRESHOLD = 0.35
WALKING_WINDOW = 100
WALKING_MIN_TRANSITIONS = 2

YAW_RATE_THRESHOLD = 0.70
DELTA_Q_YAW_THRESHOLD = 0.17
TURNING_CONFIRM_FRAMES = 10
STRAIGHT_CONFIRM_FRAMES = 10
REACQ_TURNING_CONFIRM_FRAMES = 10

PELVIS_WEIGHT = 0.6
LEFT_WEIGHT = 0.2
RIGHT_WEIGHT = 0.2
STABILITY_WINDOW = 3
STABILITY_THRESHOLD = 0.65
MIN_STEPS_BEFORE_VALID = 2

# ── Math helpers ───────────────────────────────────────────────────────────────
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )

def quat_inv(q):
    x, y, z, w = q
    n2 = x*x + y*y + z*z + w*w
    return (-x/n2, -y/n2, -z/n2, w/n2)

def quat_calibrate(measured, ref):
    if ref is None: return measured
    return quat_mul(quat_inv(ref), measured)

def vec_transform(v, q):
    """Rotate vector v by quaternion q (q * v * q^-1), return (x,y,z)."""
    vx, vy, vz = v
    x, y, z, w = q
    # p = (vx, vy, vz, 0)
    # result = q * p * q_inv
    tx = w*vx + y*vz - z*vy
    ty = w*vy + z*vx - x*vz
    tz = w*vz + x*vy - y*vx
    tw = -x*vx - y*vy - z*vz
    rx = tw*(-x) + tx*w + ty*(-z) - tz*(-y)  # using q_inv
    ry = tw*(-y) - tx*(-z) + ty*w + tz*(-x)
    rz = tw*(-z) + tx*(-y) - ty*(-x) + tz*w
    # simpler: direct formula
    # q v q* where q = (x,y,z,w)
    rx = vx*(w*w+x*x-y*y-z*z) + vy*2*(x*y-w*z) + vz*2*(x*z+w*y)
    ry = vx*2*(x*y+w*z) + vy*(w*w-x*x+y*y-z*z) + vz*2*(y*z-w*x)
    rz = vx*2*(x*z-w*y) + vy*2*(y*z+w*x) + vz*(w*w-x*x-y*y+z*z)
    return (rx, ry, rz)

def extract_yaw(q):
    x, y, z, w = q
    denom = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(2.0*(w*z + x*y), denom), denom

def extract_yaw_from_dq(dq):
    x, y, z, w = dq
    return 2.0 * math.atan2(z, w)

def normalize_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def median_quat(samples):
    # Component-wise median, then normalize
    def med(vals): s=sorted(vals); n=len(vals); return (s[n//2-1]+s[n//2])/2 if n%2==0 else s[n//2]
    xs=[s[0] for s in samples]; ys=[s[1] for s in samples]
    zs=[s[2] for s in samples]; ws=[s[3] for s in samples]
    mx,my,mz,mw = med(xs),med(ys),med(zs),med(ws)
    n = math.sqrt(mx*mx+my*my+mz*mz+mw*mw)
    return (mx/n,my/n,mz/n,mw/n)

# ── Load CSV ───────────────────────────────────────────────────────────────────
frames = {}  # pid -> {Pelvis, Left, Right}
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid = int(row['PacketId'])
        role = row['Role'].strip()
        def fv(k): return float(row[k])
        d = {
            'q': (fv('Qx'), fv('Qy'), fv('Qz'), fv('Qw')),
            'g': (fv('Gx'), fv('Gy'), fv('Gz')),
            'fa': (fv('FreeAx'), fv('FreeAy'), fv('FreeAz')),
            'a': (fv('Ax'), fv('Ay'), fv('Az')),
            'dq': (fv('DQx'), fv('DQy'), fv('DQz'), fv('DQw')),
            'sw': int(row['StatusWord']),
        }
        if pid not in frames: frames[pid] = {}
        frames[pid][role] = d

pids = sorted(frames.keys())
print(f"Total packets: {len(pids)}, range {pids[0]}–{pids[-1]}")

# ── Calibration (Task 1) ───────────────────────────────────────────────────────
cal_profile = None  # {PelvisRef, LeftRef, RightRef}
cal_state = 'WaitingForStomp'  # WaitingForStomp / Collecting / Done
stomp_active = False; stomp_frames = 0
static_consecutive = 0
collect_buf = {'Pelvis': [], 'Left': [], 'Right': []}
collect_count = 0
cal_pid = None

for pid in pids:
    f = frames[pid]
    if 'Left' not in f: continue

    # Pre-gate: stomp detection on left foot
    if cal_state == 'WaitingForStomp':
        az = f['Left']['a'][2]
        if abs(az - 9.81) > STOMP_ACC_THRESHOLD:
            if not stomp_active:
                stomp_active = True; stomp_frames = 0
            stomp_frames += 1
            if stomp_frames > STOMP_MAX_FRAMES:
                stomp_active = False; stomp_frames = 0
        elif stomp_active:
            cal_state = 'Collecting'; stomp_active = False; stomp_frames = 0

    # Post-gate: static frame collection
    if cal_state == 'Collecting' and 'Pelvis' in f and 'Left' in f and 'Right' in f:
        pg, lg, rg = [math.sqrt(sum(v**2 for v in f[r]['g'])) for r in ['Pelvis','Left','Right']]
        is_static = pg < STATIC_GYRO_THRESHOLD and lg < STATIC_GYRO_THRESHOLD and rg < STATIC_GYRO_THRESHOLD
        if is_static:
            static_consecutive += 1
        else:
            static_consecutive = 0

        if static_consecutive >= STATIC_REQUIRED_FRAMES:
            collect_count += 1
            for role_key, role_name in [('Pelvis','Pelvis'),('Left','Left'),('Right','Right')]:
                collect_buf[role_name].append(f[role_key]['q'])
            if collect_count >= STATIC_COLLECT_FRAMES:
                cal_profile = {
                    'PelvisRef': median_quat(collect_buf['Pelvis']),
                    'LeftRef':   median_quat(collect_buf['Left']),
                    'RightRef':  median_quat(collect_buf['Right']),
                }
                cal_state = 'Done'; cal_pid = pid
                break

print(f"Calibration: {cal_state}, completed at pid={cal_pid}")
if cal_profile:
    p = cal_profile['PelvisRef']
    _, denom = extract_yaw(p)
    print(f"  PelvisRef yaw denom = {denom:.4f}  (|denom|<0.1 → gimbal-lock risk)")

# ── Stance detection helper ────────────────────────────────────────────────────
def is_stance(sample, cal_ref):
    fa = sample['fa']; g = sample['g']; q = sample['q']
    acc_ok  = math.sqrt(sum(v**2 for v in fa)) < FREE_ACC_STANCE_THRESHOLD
    gyro_ok = math.sqrt(sum(v**2 for v in g)) < GYRO_STANCE_THRESHOLD
    pitch_ok = True
    if cal_ref is not None:
        g_ref = vec_transform((0,0,-1), quat_inv(cal_ref))
        g_now = vec_transform((0,0,-1), quat_inv(q))
        dot = sum(a*b for a,b in zip(g_ref, g_now))
        dot = max(-1.0, min(1.0, dot))
        pitch_ok = math.acos(dot) < FOOT_PITCH_THRESHOLD
    return acc_ok and gyro_ok and pitch_ok

# ── Walking detection ──────────────────────────────────────────────────────────
left_stance_hist = deque(maxlen=WALKING_WINDOW)
right_stance_hist = deque(maxlen=WALKING_WINDOW)

def count_transitions(hist):
    count = 0
    prev = None
    for v in hist:
        if prev is not None and v != prev: count += 1
        prev = v
    return count

# ── Motion Context ─────────────────────────────────────────────────────────────
ctx_state = 'Straight'
turning_frames = 0; straight_frames = 0; reacq_turning_frames = 0
dq_yaw_accum = 0.0

def handle_straight(sig):
    global ctx_state, turning_frames, dq_yaw_accum
    if sig:
        turning_frames += 1
        if turning_frames >= TURNING_CONFIRM_FRAMES:
            ctx_state = 'Turning'; turning_frames = 0; dq_yaw_accum = 0.0
            return 'Turning', 1.0
        return 'Straight', 1.0 - turning_frames/TURNING_CONFIRM_FRAMES
    turning_frames = 0; dq_yaw_accum = 0.0
    return 'Straight', 1.0

def handle_turning(sig):
    global ctx_state, straight_frames, dq_yaw_accum
    dq_yaw_accum = 0.0
    if not sig:
        straight_frames += 1
        if straight_frames >= STRAIGHT_CONFIRM_FRAMES:
            ctx_state = 'ReacquiringPd'; straight_frames = 0
            return 'ReacquiringPd', 0.0
    else:
        straight_frames = 0
    return 'Turning', 1.0

def handle_reacquiring(sig):
    global ctx_state, reacq_turning_frames, dq_yaw_accum, straight_frames
    dq_yaw_accum = 0.0
    if sig:
        reacq_turning_frames += 1
        if reacq_turning_frames >= REACQ_TURNING_CONFIRM_FRAMES:
            ctx_state = 'Turning'; straight_frames = 0; reacq_turning_frames = 0
    else:
        reacq_turning_frames = 0
    return ctx_state, 0.0

def confirm_straight():
    global ctx_state
    if ctx_state == 'ReacquiringPd':
        ctx_state = 'Straight'

# ── PD Estimator state ─────────────────────────────────────────────────────────
pd_history = deque()
pd_sum_cos = 0.0; pd_sum_sin = 0.0
current_pd = 0.0; has_estimate = False; step_count = 0
prev_ctx = 'Straight'

left_in_stance = False; right_in_stance = False
l_cos = l_sin = r_cos = r_sin = 0.0
l_frames = r_frames = 0
left_yaw_ready = right_yaw_ready = False
last_left_yaw = last_right_yaw = 0.0

def compute_stability():
    n = len(pd_history)
    if n < 2: return 0.0
    r_bar = math.sqrt(pd_sum_cos**2 + pd_sum_sin**2) / n
    return 1.0 / (1.0 + (1.0 - r_bar))

def update_history(pd):
    global pd_sum_cos, pd_sum_sin
    pd_history.append(pd)
    pd_sum_cos += math.cos(pd); pd_sum_sin += math.sin(pd)
    if len(pd_history) > STABILITY_WINDOW:
        old = pd_history.popleft()
        pd_sum_cos -= math.cos(old); pd_sum_sin -= math.sin(old)

# ── Main simulation ────────────────────────────────────────────────────────────
print("\n── Task 4: PD Estimator (post-calibration frames only) ──")
print(f"{'pid':>6} {'ctx':>12} {'step':>5} {'stab':>6} {'PD_deg':>8} {'pelv_denom':>11} {'confirm':>8}")

confirm_calls = []  # (pid, step_count, stability)
state_log = []

for pid in pids:
    if cal_profile is None: continue
    if pid <= (cal_pid or 0): continue
    f = frames[pid]
    if 'Pelvis' not in f or 'Left' not in f or 'Right' not in f: continue

    pf = f['Pelvis']; lf = f['Left']; rf = f['Right']

    # Stance
    ls = is_stance(lf, cal_profile['LeftRef'])
    rs = is_stance(rf, cal_profile['RightRef'])
    left_stance_hist.append(ls); right_stance_hist.append(rs)
    is_walking = (count_transitions(left_stance_hist) >= WALKING_MIN_TRANSITIONS and
                  count_transitions(right_stance_hist) >= WALKING_MIN_TRANSITIONS)

    # Motion context
    pq = pf['q']
    yr_vec = vec_transform(pf['g'], pq)
    pelvis_yr = abs(yr_vec[2])
    dq = pf['dq']
    dq_yaw_accum += extract_yaw_from_dq(dq)
    turning_sig = pelvis_yr > YAW_RATE_THRESHOLD or abs(dq_yaw_accum) > DELTA_Q_YAW_THRESHOLD

    if ctx_state == 'Straight':    ctx, conf = handle_straight(turning_sig)
    elif ctx_state == 'Turning':   ctx, conf = handle_turning(turning_sig)
    else:                          ctx, conf = handle_reacquiring(turning_sig)

    # PD Estimator
    # On Turning→ReacquiringPd transition, flush history
    if ctx == 'ReacquiringPd' and prev_ctx == 'Turning':
        pd_history.clear(); pd_sum_cos = 0.0; pd_sum_sin = 0.0
        step_count = 0; has_estimate = False
        left_yaw_ready = right_yaw_ready = False
        left_in_stance = right_in_stance = False
        l_cos = l_sin = r_cos = r_sin = 0.0; l_frames = r_frames = 0

    prev_ctx = ctx

    if ctx in ('Straight', 'ReacquiringPd'):
        # Track stance yaw
        if ls and lf['q'] is not None:
            if not left_in_stance:
                left_in_stance = True; l_cos = 0.0; l_sin = 0.0; l_frames = 0
            ly, _ = extract_yaw(quat_calibrate(lf['q'], cal_profile['LeftRef']))
            l_cos += math.cos(ly); l_sin += math.sin(ly); l_frames += 1
        elif left_in_stance:
            left_in_stance = False
            last_left_yaw = math.atan2(l_sin, l_cos) if l_frames > 0 else last_left_yaw
            left_yaw_ready = True

        if rs and rf['q'] is not None:
            if not right_in_stance:
                right_in_stance = True; r_cos = 0.0; r_sin = 0.0; r_frames = 0
            ry, _ = extract_yaw(quat_calibrate(rf['q'], cal_profile['RightRef']))
            r_cos += math.cos(ry); r_sin += math.sin(ry); r_frames += 1
        elif right_in_stance:
            right_in_stance = False
            last_right_yaw = math.atan2(r_sin, r_cos) if r_frames > 0 else last_right_yaw
            right_yaw_ready = True

        if left_yaw_ready and right_yaw_ready:
            left_yaw_ready = right_yaw_ready = False
            step_count += 1

            pelvis_yaw, pelv_denom = extract_yaw(quat_calibrate(pq, cal_profile['PelvisRef']))
            total_w = PELVIS_WEIGHT + LEFT_WEIGHT + RIGHT_WEIGHT
            wx = (math.cos(pelvis_yaw)*PELVIS_WEIGHT + math.cos(last_left_yaw)*LEFT_WEIGHT + math.cos(last_right_yaw)*RIGHT_WEIGHT) / total_w
            wy = (math.sin(pelvis_yaw)*PELVIS_WEIGHT + math.sin(last_left_yaw)*LEFT_WEIGHT + math.sin(last_right_yaw)*RIGHT_WEIGHT) / total_w
            fused = math.atan2(wy, wx)
            update_history(fused)
            current_pd = fused; has_estimate = True

            stability = compute_stability()
            pd_deg = math.degrees(current_pd)

            confirmed = ''
            if ctx == 'ReacquiringPd' and stability >= STABILITY_THRESHOLD and step_count >= MIN_STEPS_BEFORE_VALID:
                confirm_straight()
                confirmed = '*** CONFIRM'
                confirm_calls.append((pid, step_count, stability))

            state_log.append((pid, ctx_state, step_count, stability, pd_deg, pelv_denom, confirmed))
            print(f"{pid:>6} {ctx_state:>12} {step_count:>5} {stability:>6.3f} {pd_deg:>8.2f}° {pelv_denom:>11.4f}  {confirmed}")

print("\n── Summary ─────────────────────────────────────────────────────────────")
if confirm_calls:
    for pid, sc, stab in confirm_calls:
        print(f"ConfirmStraight called at pid={pid}, step_count={sc}, stability={stab:.3f}")
else:
    print("ConfirmStraight was NEVER called")

# State distribution
from collections import Counter
# Recompute state counts from main loop for context
ctx_state2 = 'Straight'
turning_frames2 = 0; straight_frames2 = 0; reacq_turning_frames2 = 0; dq_yaw_accum2 = 0.0
state_counts = Counter()
for pid in pids:
    if cal_profile is None: continue
    if pid <= (cal_pid or 0): continue
    f = frames[pid]
    if 'Pelvis' not in f or 'Left' not in f or 'Right' not in f: continue
    pf = f['Pelvis']; pq2 = pf['q']
    yr_vec2 = vec_transform(pf['g'], pq2)
    yr2 = abs(yr_vec2[2])
    dq2 = pf['dq']
    dq_yaw_accum2 += extract_yaw_from_dq(dq2)
    sig2 = yr2 > YAW_RATE_THRESHOLD or abs(dq_yaw_accum2) > DELTA_Q_YAW_THRESHOLD

    if ctx_state2 == 'Straight':
        if sig2:
            turning_frames2 += 1
            if turning_frames2 >= TURNING_CONFIRM_FRAMES: ctx_state2='Turning'; turning_frames2=0; dq_yaw_accum2=0.0
        else: turning_frames2=0; dq_yaw_accum2=0.0
    elif ctx_state2 == 'Turning':
        dq_yaw_accum2=0.0
        if not sig2:
            straight_frames2 += 1
            if straight_frames2 >= STRAIGHT_CONFIRM_FRAMES: ctx_state2='ReacquiringPd'; straight_frames2=0; reacq_turning_frames2=0
        else: straight_frames2=0
    else:  # ReacquiringPd
        dq_yaw_accum2=0.0
        if sig2:
            reacq_turning_frames2 += 1
            if reacq_turning_frames2 >= REACQ_TURNING_CONFIRM_FRAMES:
                ctx_state2='Turning'; straight_frames2=0; reacq_turning_frames2=0
        else: reacq_turning_frames2=0
    state_counts[ctx_state2] += 1

total = sum(state_counts.values())
print("\nState distribution (all post-cal frames):")
for s, n in state_counts.most_common():
    print(f"  {s}: {n} ({100*n/total:.1f}%)")

# Pelvis denom analysis during post-turn walking
print("\nPelvis ExtractYaw denom statistics during Straight/ReacquiringPd steps:")
denoms = [(pid, pelv_denom) for pid, ctx2, sc, stab, pd_deg, pelv_denom, conf in state_log]
if denoms:
    ds = [abs(d) for _, d in denoms]
    print(f"  min|denom|={min(ds):.4f}, max|denom|={max(ds):.4f}, mean={sum(ds)/len(ds):.4f}")
    near_zero = sum(1 for d in ds if d < 0.1)
    print(f"  Steps with |denom|<0.1 (gimbal-lock risk): {near_zero}/{len(ds)}")
