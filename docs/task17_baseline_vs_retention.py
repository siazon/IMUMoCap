"""
Task 17: replay the cleaned recording (docs/ImuSamples_20261005_130206_from_episode3.csv, see
task16_find_last_calibration.py) through a faithful Python port of the FULL pipeline and compare
Retention against Baseline -- did the FPA angle move toward (and stay near) the personalized
Target after training, with feedback removed?

Every stage mirrors current C# source, checked line-by-line this session (not reused blindly from
earlier docs/task*.py scripts -- several of those turned out to encode now-outdated formulas):
  DataQualityGate.cs        -- StatusWord + acc/gyro spike rejection (per bundle, vs previous RAW
                                bundle regardless of accept/reject)
  CalibrationProcessor.cs   -- static-window median-quaternion calibration
  GaitEventDetector.cs      -- stance debounce + IsWalking
  MotionContextDetector.cs  -- Straight/Turning/ReacquiringPd + IsWalking confidence halving
  ProgressionDirEstimator.cs-- PD fusion, stability, ConfirmStraight
  FpaEngine.cs               -- data-driven settle -> FpaResult (own hardcoded-Z ExtractYaw,
                                NOT the axis-aware heading ProgressionDirEstimator uses internally)
  BaselineProcessor.cs + Models/BaselineProfile.cs -- Target/SD/Tolerance
  GaitPipeline.cs            -- block/stage orchestration: TrainingBlockAlphas={1.5,1.0,0.5},
                                100 valid steps/foot per stage (Baseline/Block1/2/3/Retention)

Stage boundaries are NOT in the raw CSV (no stage column) -- GaitPipeline only ever exposes
"how many valid steps this block has collected", not wall-clock stage transitions. So stages here
are inferred purely by COUNTING valid settled steps in chronological order: first 100/foot ->
Baseline; next 100/foot -> Training1 (alpha=1.5); next 100/foot -> Training2 (alpha=1.0); next
100/foot -> Training3 (alpha=0.5); next 100/foot -> Retention. This assumes the two ~120s rest
periods between blocks don't themselves generate 100 "valid" (Straight context, stable PD) steps
of their own -- a safe assumption since rests in this recording involve walking to a laptop and
turning (see task16's analysis), which should mostly fail the Straight-context gate. Flagged
again in the printed output; sanity-check the reported stage boundary times against your own
memory of the session.

Usage: python task17_baseline_vs_retention.py [path/to/cleaned_ImuSamples.csv]
"""
import sys
import csv
import math
from collections import deque, defaultdict

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20261005_130206_from_episode3.csv"
FS_HZ = 100.0

# ── DataQualityGate.cs ───────────────────────────────────────────────────────────────────────
STATUSWORD_ORIENTATION_VALID = 0x02
STATUSWORD_CLIPPING_DETECTED = 0x00080000
ACC_DELTA_THR = 100.0   # m/s^2
GYRO_DELTA_THR = 20.0   # rad/s

# ── CalibrationProcessor.cs ──────────────────────────────────────────────────────────────────
STATIC_GYRO_THR, STATIC_REQUIRED_F, STATIC_COLLECT_F, STATIC_TIMEOUT_F = 0.3, 30, 300, 1000

# ── GaitEventDetector.cs ─────────────────────────────────────────────────────────────────────
FREE_ACC_THR, GYRO_THR, FOOT_PITCH_THR, MIN_STANCE_F = 2.5, 1.0, 0.35, 2
WALKING_WINDOW_F = 300

# ── MotionContextDetector.cs ─────────────────────────────────────────────────────────────────
YAW_RATE_THR, DELTAQ_YAW_THR = 0.70, 0.17
TURNING_CONFIRM_F, STRAIGHT_CONFIRM_F, REACQ_TURNING_CONFIRM_F = 10, 20, 10

# ── ProgressionDirEstimator.cs ───────────────────────────────────────────────────────────────
PELVIS_W, LEFT_W, RIGHT_W = 0.6, 0.2, 0.2
STABILITY_WINDOW, STABILITY_THR, MIN_STEPS_VALID, PELVIS_AGREE_THR_DEG = 10, 0.85, 2, 15.0

# ── FpaEngine.cs ─────────────────────────────────────────────────────────────────────────────
CONTEXT_CONFIDENCE_THR, PD_STABILITY_THR = 0.7, 0.7
MIN_STANCE_FLOOR, SETTLE_GYRO_THR, SETTLE_QUIET_F, MAX_STANCE_FOR_SETTLE = 2, 0.35, 3, 20

# ── BaselineProfile.cs / GaitPipeline.cs ─────────────────────────────────────────────────────
TARGET_K, TOLERANCE_W_MIN_DEG = 1.0, 4.0
MIN_BASELINE_STEPS = 100
TRAINING_BLOCK_TARGET_STEPS = 100
TRAINING_BLOCK_ALPHAS = [1.5, 1.0, 0.5]
RETENTION_TARGET_STEPS = 100


# ── quaternion helpers (q as (x, y, z, w)) ──────────────────────────────────────────────────

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
    """FpaEngine.ExtractYaw: hardcoded-Z yaw of an ABSOLUTE quaternion."""
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def detect_heading_axis(sensor_ref):
    gx, gy, gz = vec3_transform((0.0, 0.0, -1.0), q_inv(sensor_ref))
    ax, ay, az = abs(gx), abs(gy), abs(gz)
    if ax >= ay and ax >= az: return 0
    if ay >= ax and ay >= az: return 1
    return 2

def extract_heading(q, axis):
    """ProgressionDirEstimator.ExtractHeading: axis-aware heading of a (relative) quaternion."""
    x, y, z, w = q
    if axis == 0: return math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    if axis == 1: return math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

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


# ── load + DataQualityGate ──────────────────────────────────────────────────────────────────

def load_and_gate(csv_path):
    raw = defaultdict(dict)
    with open(csv_path, encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            pid = int(row['PacketId']); role = row['Role'].strip()
            if role not in ('Pelvis', 'Left', 'Right'):
                continue
            raw[pid][role] = {
                'sw': int(row['StatusWord']),
                'q': (float(row['Qx']), float(row['Qy']), float(row['Qz']), float(row['Qw'])),
                'g': (float(row['Gx']), float(row['Gy']), float(row['Gz'])),
                'fa': (float(row['FreeAx']), float(row['FreeAy']), float(row['FreeAz'])),
                'a': (float(row['Ax']), float(row['Ay']), float(row['Az'])),
                'dq': (float(row['DQx']), float(row['DQy']), float(row['DQz']), float(row['DQw'])),
            }
    pids_all = sorted(p for p, d in raw.items() if len(d) == 3)

    def status_error(s):
        return ((s & STATUSWORD_ORIENTATION_VALID) == 0) or ((s & STATUSWORD_CLIPPING_DETECTED) != 0)

    def dist(a, b):
        return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))

    def spike(curr, prev):
        return dist(curr['a'], prev['a']) > ACC_DELTA_THR or dist(curr['g'], prev['g']) > GYRO_DELTA_THR

    accepted_pids = []
    prev = None  # dict with 'Pelvis'/'Left'/'Right' raw samples, updated every bundle (accept or reject)
    n_rejected_status = n_rejected_spike = 0
    for pid in pids_all:
        b = raw[pid]
        p, l, r = b['Pelvis'], b['Left'], b['Right']
        rejected = status_error(p['sw']) or status_error(l['sw']) or status_error(r['sw'])
        if rejected:
            n_rejected_status += 1
        elif prev is not None and (spike(p, prev['Pelvis']) or spike(l, prev['Left']) or spike(r, prev['Right'])):
            rejected = True
            n_rejected_spike += 1
        prev = b
        if not rejected:
            accepted_pids.append(pid)

    print(f"DataQualityGate: {len(pids_all)} complete bundles -> {len(accepted_pids)} accepted "
          f"({n_rejected_status} status-rejected, {n_rejected_spike} spike-rejected)")
    return raw, accepted_pids


# ── GaitEventDetector ───────────────────────────────────────────────────────────────────────

class GaitEventDetector:
    def __init__(self):
        self.left_count = self.right_count = 0
        self.left_prev = self.right_prev = False
        self.left_trans = self.right_trans = 0
        self.walk_count = 0
        self.is_walking = False

    @staticmethod
    def _is_stance_raw(sample, cal_ref):
        fa_mag = math.sqrt(sum(v*v for v in sample['fa']))
        g_mag = math.sqrt(sum(v*v for v in sample['g']))
        if fa_mag >= FREE_ACC_THR or g_mag >= GYRO_THR:
            return False
        g_ref = vec3_transform((0.0, 0.0, -1.0), q_inv(cal_ref))
        g_now = vec3_transform((0.0, 0.0, -1.0), q_inv(sample['q']))
        cos_tilt = max(-1.0, min(1.0, sum(a*b for a, b in zip(g_ref, g_now))))
        return math.acos(cos_tilt) < FOOT_PITCH_THR

    def detect(self, l_sample, r_sample, left_ref, right_ref):
        self.left_count = self.left_count + 1 if self._is_stance_raw(l_sample, left_ref) else 0
        self.right_count = self.right_count + 1 if self._is_stance_raw(r_sample, right_ref) else 0
        left_confirmed = self.left_count >= MIN_STANCE_F
        right_confirmed = self.right_count >= MIN_STANCE_F

        if left_confirmed != self.left_prev:
            self.left_trans += 1; self.left_prev = left_confirmed
        if right_confirmed != self.right_prev:
            self.right_trans += 1; self.right_prev = right_confirmed

        self.walk_count += 1
        if self.walk_count >= WALKING_WINDOW_F:
            self.is_walking = self.left_trans >= 2 and self.right_trans >= 2
            self.walk_count = 0
            self.left_trans = self.right_trans = 0

        return left_confirmed, right_confirmed, self.is_walking


# ── MotionContextDetector ───────────────────────────────────────────────────────────────────

class MotionContextDetector:
    def __init__(self):
        self.state = 'Straight'
        self.turning_f = self.straight_f = self.reacq_turning_f = 0
        self.deltaq_accum = 0.0

    def detect(self, pelvis_sample, gait_is_walking):
        _, _, wz = vec3_transform(pelvis_sample['g'], pelvis_sample['q'])
        pelvis_yaw_rate = abs(wz)
        dqx, dqy, dqz, dqw = pelvis_sample['dq']
        self.deltaq_accum += 2.0 * math.atan2(dqz, dqw)
        turning_signal = pelvis_yaw_rate > YAW_RATE_THR or abs(self.deltaq_accum) > DELTAQ_YAW_THR

        if self.state == 'Straight':
            if turning_signal:
                self.turning_f += 1
                if self.turning_f >= TURNING_CONFIRM_F:
                    self.state = 'Turning'; self.turning_f = self.straight_f = 0; self.deltaq_accum = 0.0
                    state, conf = 'Turning', 1.0
                else:
                    state, conf = 'Straight', 1.0 - self.turning_f / TURNING_CONFIRM_F
            else:
                self.turning_f = 0; self.deltaq_accum = 0.0
                state, conf = 'Straight', 1.0
        elif self.state == 'Turning':
            self.deltaq_accum = 0.0
            if not turning_signal:
                self.straight_f += 1
                if self.straight_f >= STRAIGHT_CONFIRM_F:
                    self.state = 'ReacquiringPd'; self.straight_f = self.reacq_turning_f = 0
                    state, conf = 'ReacquiringPd', 0.0
                else:
                    state, conf = 'Turning', 1.0
            else:
                self.straight_f = 0
                state, conf = 'Turning', 1.0
        else:  # ReacquiringPd
            self.deltaq_accum = 0.0
            if turning_signal:
                self.reacq_turning_f += 1
                if self.reacq_turning_f >= REACQ_TURNING_CONFIRM_F:
                    self.state = 'Turning'; self.straight_f = self.reacq_turning_f = 0
            else:
                self.reacq_turning_f = 0
            state, conf = self.state, 0.0

        if not gait_is_walking and state == 'Straight':
            conf *= 0.5
        return state, conf

    def confirm_straight(self):
        if self.state == 'ReacquiringPd':
            self.state = 'Straight'


# ── ProgressionDirEstimator ─────────────────────────────────────────────────────────────────

class ProgressionDirEstimator:
    def __init__(self, pelvis_ref, left_ref, right_ref):
        self.pelvis_ref, self.left_ref, self.right_ref = pelvis_ref, left_ref, right_ref
        self.pelvis_axis = detect_heading_axis(pelvis_ref)
        self.left_axis = detect_heading_axis(left_ref)
        self.right_axis = detect_heading_axis(right_ref)
        self.pd_current = 0.0
        self.has_estimate = False
        self.step_count = 0
        self.prev_context = 'Straight'
        self.history = deque()
        self.hist_cos = self.hist_sin = 0.0
        self.left_in_stance = self.right_in_stance = False
        self.left_cos = self.left_sin = self.right_cos = self.right_sin = 0.0
        self.left_frames = self.right_frames = 0
        self.left_ready = self.right_ready = False
        self.last_left_yaw = self.last_right_yaw = 0.0

    def stability(self):
        n = len(self.history)
        if n < 2: return 0.0
        r_bar = math.sqrt(self.hist_cos**2 + self.hist_sin**2) / n
        return 1.0 / (1.0 + (1.0 - r_bar))

    def update(self, pel_q, left_q, right_q, left_stance, right_stance, mc_state, mc_detector):
        if mc_state not in ('Straight', 'ReacquiringPd'):
            self.prev_context = mc_state
            stab = self.stability()
            return self.pd_current, self.has_estimate and stab >= STABILITY_THR, stab

        if mc_state == 'ReacquiringPd' and self.prev_context == 'Turning':
            self.history.clear(); self.hist_cos = self.hist_sin = 0.0
            self.step_count = 0; self.has_estimate = False
            self.left_ready = self.right_ready = False
            self.left_in_stance = self.right_in_stance = False
            self.left_cos = self.left_sin = self.right_cos = self.right_sin = 0.0
            self.left_frames = self.right_frames = 0
        self.prev_context = mc_state

        pelvis_yaw = extract_heading(relative_quat(pel_q, self.pelvis_ref), self.pelvis_axis)

        if left_stance:
            if not self.left_in_stance:
                self.left_in_stance = True; self.left_cos = self.left_sin = 0.0; self.left_frames = 0
            ly = extract_heading(relative_quat(left_q, self.left_ref), self.left_axis)
            self.left_cos += math.cos(ly); self.left_sin += math.sin(ly); self.left_frames += 1
        elif self.left_in_stance:
            self.left_in_stance = False
            self.last_left_yaw = math.atan2(self.left_sin, self.left_cos) if self.left_frames > 0 else self.last_left_yaw
            self.left_ready = True

        if right_stance:
            if not self.right_in_stance:
                self.right_in_stance = True; self.right_cos = self.right_sin = 0.0; self.right_frames = 0
            ry = extract_heading(relative_quat(right_q, self.right_ref), self.right_axis)
            self.right_cos += math.cos(ry); self.right_sin += math.sin(ry); self.right_frames += 1
        elif self.right_in_stance:
            self.right_in_stance = False
            self.last_right_yaw = math.atan2(self.right_sin, self.right_cos) if self.right_frames > 0 else self.last_right_yaw
            self.right_ready = True

        if not (self.left_ready and self.right_ready):
            stab = self.stability()
            return self.pd_current, self.has_estimate and stab >= STABILITY_THR, stab
        self.left_ready = self.right_ready = False

        self.step_count += 1
        total_w = PELVIS_W + LEFT_W + RIGHT_W
        wx = (math.cos(pelvis_yaw)*PELVIS_W + math.cos(self.last_left_yaw)*LEFT_W + math.cos(self.last_right_yaw)*RIGHT_W) / total_w
        wy = (math.sin(pelvis_yaw)*PELVIS_W + math.sin(self.last_left_yaw)*LEFT_W + math.sin(self.last_right_yaw)*RIGHT_W) / total_w
        fused = math.atan2(wy, wx)

        self.history.append(fused); self.hist_cos += math.cos(fused); self.hist_sin += math.sin(fused)
        if len(self.history) > STABILITY_WINDOW:
            old = self.history.popleft(); self.hist_cos -= math.cos(old); self.hist_sin -= math.sin(old)
        self.pd_current = fused; self.has_estimate = True

        stab = self.stability()
        if mc_state == 'ReacquiringPd':
            pelvis_agree_deg = abs(math.degrees(norm_rad(self.pd_current - pelvis_yaw)))
            if stab >= STABILITY_THR and self.step_count >= MIN_STEPS_VALID and pelvis_agree_deg <= PELVIS_AGREE_THR_DEG:
                mc_detector.confirm_straight()

        return self.pd_current, self.has_estimate and stab >= STABILITY_THR, stab


# ── FpaEngine.StanceSampler ─────────────────────────────────────────────────────────────────

class StanceSampler:
    def __init__(self):
        self.cos_sum = self.sin_sum = 0.0
        self.count = 0
        self.in_stance = False
        self.emitted = False
        self.quiet_streak = 0

    def add_frame(self, yaw, gyro_mag):
        if not self.in_stance:
            self.in_stance = True; self.emitted = False
            self.cos_sum = self.sin_sum = 0.0; self.count = 0; self.quiet_streak = 0
        self.cos_sum += math.cos(yaw); self.sin_sum += math.sin(yaw); self.count += 1
        self.quiet_streak = self.quiet_streak + 1 if gyro_mag < SETTLE_GYRO_THR else 0

    def mark_swing(self):
        self.in_stance = False

    def has_settled(self):
        return self.count >= MIN_STANCE_FLOOR and \
               (self.quiet_streak >= SETTLE_QUIET_F or self.count >= MAX_STANCE_FOR_SETTLE)

    def try_settle(self):
        if not self.in_stance or self.emitted or not self.has_settled():
            return None
        self.emitted = True
        return math.atan2(self.sin_sum, self.cos_sum)


# ── BaselineProfile ──────────────────────────────────────────────────────────────────────────

def compute_target(mean, sd):
    if mean > 10.0:
        return mean - TARGET_K * sd, 'ToeIn'
    return mean + TARGET_K * sd, 'ToeOut'

def tolerance(sd, alpha):
    return max(TOLERANCE_W_MIN_DEG, alpha * sd)


def main():
    raw, accepted_pids = load_and_gate(CSV_PATH)
    if not accepted_pids:
        print("No frames survived DataQualityGate -- aborting."); return

    # ── Calibration ─────────────────────────────────────────────────────────────────────────
    static_consecutive = timeout_counter = 0
    pelvis_buf, left_buf, right_buf = [], [], []
    calib_idx = None
    for i, pid in enumerate(accepted_pids):
        timeout_counter += 1
        if timeout_counter > STATIC_TIMEOUT_F: break
        p, l, r = raw[pid]['Pelvis'], raw[pid]['Left'], raw[pid]['Right']
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
        print("Calibration never completed on this recording -- aborting."); return

    pelvis_ref = median_quaternion(pelvis_buf)
    left_ref = median_quaternion(left_buf)
    right_ref = median_quaternion(right_buf)
    calib_pid = accepted_pids[calib_idx]
    print(f"Calibration completed at pid={calib_pid} "
          f"(t={(calib_pid-accepted_pids[0])/FS_HZ:.2f}s into accepted-frame stream)\n")

    gait = GaitEventDetector()
    motion = MotionContextDetector()
    pd = ProgressionDirEstimator(pelvis_ref, left_ref, right_ref)
    left_sampler, right_sampler = StanceSampler(), StanceSampler()

    # (frame_idx, foot, fpa_deg) in chronological order, post-calibration, gate-passed steps only
    step_events = []
    t0_pid = calib_pid

    for i in range(calib_idx, len(accepted_pids)):
        pid = accepted_pids[i]
        p, l, r = raw[pid]['Pelvis'], raw[pid]['Left'], raw[pid]['Right']

        left_stance, right_stance, walking_now = gait.detect(l, r, left_ref, right_ref)
        mc_state, mc_conf = motion.detect(p, walking_now)
        pd_dir, pd_valid, pd_stab = pd.update(p['q'], l['q'], r['q'], left_stance, right_stance, mc_state, motion)

        if left_stance:
            ly = norm_rad(extract_yaw_z(l['q']) - extract_yaw_z(left_ref))
            left_sampler.add_frame(ly, math.sqrt(sum(v*v for v in l['g'])))
        else:
            left_sampler.mark_swing()
        if right_stance:
            ry = norm_rad(extract_yaw_z(r['q']) - extract_yaw_z(right_ref))
            right_sampler.add_frame(ry, math.sqrt(sum(v*v for v in r['g'])))
        else:
            right_sampler.mark_swing()

        context_ok = (mc_state == 'Straight') and (mc_conf >= CONTEXT_CONFIDENCE_THR)
        pd_ok = pd_valid and (pd_stab >= PD_STABILITY_THR)
        if context_ok and pd_ok and (left_stance or right_stance):
            settled_l = left_sampler.try_settle()
            if settled_l is not None:
                fpa_l_deg = math.degrees(norm_rad(settled_l - pd_dir))
                step_events.append((i, 'L', fpa_l_deg, (pid - t0_pid) / FS_HZ))
            settled_r = right_sampler.try_settle()
            if settled_r is not None:
                fpa_r_deg = math.degrees(norm_rad(settled_r - pd_dir))
                step_events.append((i, 'R', fpa_r_deg, (pid - t0_pid) / FS_HZ))

    print(f"Settled steps after calibration: L={sum(1 for e in step_events if e[1]=='L')}  "
          f"R={sum(1 for e in step_events if e[1]=='R')}\n")

    # ── bucket into stages by valid-step count (see module docstring for the caveat) ─────────
    stage_targets = [
        ('Baseline', MIN_BASELINE_STEPS),
        ('Training1', TRAINING_BLOCK_TARGET_STEPS),
        ('Training2', TRAINING_BLOCK_TARGET_STEPS),
        ('Training3', TRAINING_BLOCK_TARGET_STEPS),
        ('Retention', RETENTION_TARGET_STEPS),
    ]
    stages = defaultdict(lambda: {'L': [], 'R': []})
    counts = {'L': 0, 'R': 0}
    stage_idx = {'L': 0, 'R': 0}
    stage_boundary_t = {}  # stage_name -> (t_start, t_end)
    stage_open_t = {name: None for name, _ in stage_targets}

    for frame_idx, foot, fpa_deg, t_sec in step_events:
        if stage_idx[foot] >= len(stage_targets):
            continue  # ran out of stages (not enough data for 500 steps/foot)
        name, target = stage_targets[stage_idx[foot]]
        if stage_open_t[name] is None:
            stage_open_t[name] = t_sec
        stages[name][foot].append(fpa_deg)
        counts[foot] += 1
        if len(stages[name][foot]) >= target:
            stage_boundary_t.setdefault(name, [None, None])
            stage_boundary_t[name][1] = t_sec  # last-updated end time for this stage/foot
            stage_idx[foot] += 1

    print("Stage boundaries inferred from step counts (elapsed seconds since calibration):")
    for name, _ in stage_targets:
        nl, nr = len(stages[name]['L']), len(stages[name]['R'])
        end_t = stage_boundary_t.get(name, [None, None])[1]
        status = f"complete (ends ~t={end_t:.1f}s)" if end_t is not None else "INCOMPLETE -- recording too short"
        print(f"  {name:10s}: L={nl:3d} steps, R={nr:3d} steps -- {status}")
    print()

    if len(stages['Baseline']['L']) < MIN_BASELINE_STEPS or len(stages['Baseline']['R']) < MIN_BASELINE_STEPS:
        print("Baseline never reached 100 valid steps/foot -- cannot compute Target. Stopping.")
        return

    def mean_sd(vals):
        m = sum(vals) / len(vals)
        sd = (sum((v-m)**2 for v in vals) / (len(vals)-1)) ** 0.5 if len(vals) > 1 else 0.0
        return m, sd

    targets, sds, dirs = {}, {}, {}
    print("=" * 78)
    print("BASELINE")
    print("=" * 78)
    for foot in ('L', 'R'):
        vals = stages['Baseline'][foot][:MIN_BASELINE_STEPS]
        m, sd = mean_sd(vals)
        target, direction = compute_target(m, sd)
        targets[foot], sds[foot], dirs[foot] = target, sd, direction
        print(f"  {foot}: mean={m:6.2f}°  SD={sd:5.2f}°  n={len(vals)}  ->  Target={target:6.2f}° ({direction})")

    print()
    for name in ('Training1', 'Training2', 'Training3'):
        if len(stages[name]['L']) < TRAINING_BLOCK_TARGET_STEPS or len(stages[name]['R']) < TRAINING_BLOCK_TARGET_STEPS:
            print(f"{name}: incomplete, skipping stats.\n")
            continue
        alpha = TRAINING_BLOCK_ALPHAS[int(name[-1]) - 1]
        print(f"{name} (tolerance alpha={alpha}):")
        for foot in ('L', 'R'):
            vals = stages[name][foot][:TRAINING_BLOCK_TARGET_STEPS]
            m, sd = mean_sd(vals)
            errors = [v - targets[foot] for v in vals]
            mae = sum(abs(e) for e in errors) / len(errors)
            tol = tolerance(sds[foot], alpha)
            on_target_pct = 100 * sum(1 for e in errors if abs(e) <= tol) / len(errors)
            print(f"  {foot}: mean={m:6.2f}°  SD={sd:5.2f}°  mean|error from target|={mae:5.2f}°  "
                  f"on-target(±{tol:.1f}°)={on_target_pct:5.1f}%  n={len(vals)}")
        print()

    print("=" * 78)
    print("RETENTION vs BASELINE")
    print("=" * 78)
    if len(stages['Retention']['L']) < RETENTION_TARGET_STEPS or len(stages['Retention']['R']) < RETENTION_TARGET_STEPS:
        print(f"Retention incomplete (L={len(stages['Retention']['L'])}, R={len(stages['Retention']['R'])} "
              f"of {RETENTION_TARGET_STEPS} needed) -- recording may be too short, or the step-count "
              "stage-bucketing assumption broke down (e.g. a rest period leaked valid steps into "
              "the wrong block). Reporting whatever was collected.")
    for foot in ('L', 'R'):
        vals = stages['Retention'][foot]
        if not vals:
            print(f"  {foot}: no Retention steps found."); continue
        m, sd = mean_sd(vals)
        errors = [v - targets[foot] for v in vals]
        mae = sum(abs(e) for e in errors) / len(errors)
        # |mean_baseline - target| == SD_baseline exactly, by construction (target = mean +/- 1*SD)
        # -- so this ratio directly says "how many baseline-SDs away from target is retention's
        # mean", with 1.0 being "no net change from baseline" and 0 being "landed exactly on target".
        baseline_gap_sd_units = abs(m - targets[foot]) / sds[foot] if sds[foot] > 0 else float('nan')
        print(f"  {foot}: Baseline mean={stages['Baseline'][foot] and mean_sd(stages['Baseline'][foot][:MIN_BASELINE_STEPS])[0]:.2f}°"
              f"  ->  Retention mean={m:6.2f}°  (SD={sd:5.2f}°, n={len(vals)})")
        print(f"       Target={targets[foot]:6.2f}°  Retention mean error from Target={m-targets[foot]:+6.2f}°  "
              f"mean|error|={mae:5.2f}°")
        print(f"       Retention mean is {baseline_gap_sd_units:4.2f}x the original Baseline-to-Target gap "
              f"(1.0 = no change from Baseline, 0 = landed exactly on Target, >1 = moved further away)")
        for alpha, label in ((1.5, 'Block1-loose'), (0.5, 'Block3-tight')):
            tol = tolerance(sds[foot], alpha)
            pct = 100 * sum(1 for e in errors if abs(e) <= tol) / len(errors)
            print(f"       on-target @ alpha={alpha} ({label}, ±{tol:.1f}°): {pct:5.1f}%")
        print()


if __name__ == "__main__":
    main()
