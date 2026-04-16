"""
Pipeline simulation against ImuSamples_20260414_165259.csv
Faithfully replicates C# pipeline logic in Python.
"""
import csv, math, sys
from collections import defaultdict, deque

CSV = "D:/SourceCode/IMUMoCap/docs/ImuSamples_20260414_165259.csv"
XSF_ORIENTATION_VALID = 0x02
XSF_CLIPPING_DETECTED = 0x00080000  # 524288

# ─── Quaternion / Vector helpers ─────────────────────────────────────────────
def vec3_len(v):  return math.sqrt(sum(x*x for x in v))
def vec3_dist(a, b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def q_len(q):
    return math.sqrt(sum(x*x for x in q))

def q_norm(q):
    l = q_len(q); return tuple(x/l for x in q) if l else (0,0,0,1)

def q_inv(q):                   # q = (x,y,z,w)
    x,y,z,w = q; l2 = sum(v*v for v in q)
    return (-x/l2,-y/l2,-z/l2, w/l2)

def q_mul(a, b):
    x1,y1,z1,w1 = a; x2,y2,z2,w2 = b
    return (w1*x2+x1*w2+y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2,
            w1*w2-x1*x2-y1*y2-z1*z2)

def extract_yaw(q):
    x,y,z,w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def extract_pitch(q):           # rotation around X axis
    x,y,z,w = q
    return math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))

def calib_q(q, ref):            # q_rel = Inv(ref)*q
    return q_mul(q_inv(ref), q) if ref else q

def norm_angle(r):
    while r >  math.pi: r -= 2*math.pi
    while r < -math.pi: r += 2*math.pi
    return r

def median_component(vals):
    s = sorted(vals); return s[len(s)//2]

def median_quat(qs):
    # same-hemisphere, then component-wise median, normalise
    qs2 = [(-x,-y,-z,-w) if w<0 else (x,y,z,w) for x,y,z,w in qs]
    xs,ys,zs,ws = zip(*qs2)
    q = (median_component(xs),median_component(ys),
         median_component(zs),median_component(ws))
    return q_norm(q)

# ─── Parse CSV ───────────────────────────────────────────────────────────────
def parse_csv(path):
    by_pkt = defaultdict(dict)
    with open(path, encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            pid  = int(r['PacketId'])
            role = r['Role']
            by_pkt[pid][role] = {
                'sw':  int(r['StatusWord']),
                'Q':   (float(r['Qx']),float(r['Qy']),float(r['Qz']),float(r['Qw'])),
                'G':   (float(r['Gx']),float(r['Gy']),float(r['Gz'])),
                'FA':  (float(r['FreeAx']),float(r['FreeAy']),float(r['FreeAz'])),
                'A':   (float(r['Ax']),float(r['Ay']),float(r['Az'])),
                'DQ':  (float(r['DQx']),float(r['DQy']),float(r['DQz']),float(r['DQw'])),
            }
    bundles = [(pid, by_pkt[pid]) for pid in sorted(by_pkt)
               if all(k in by_pkt[pid] for k in ('Pelvis','Left','Right'))]
    print(f"Bundles (complete 3-IMU): {len(bundles)}  "
          f"(incomplete: {sum(1 for pid in by_pkt if not all(k in by_pkt[pid] for k in ('Pelvis','Left','Right')))})")
    return bundles

# ─── Stage 1: DataQualityGate ────────────────────────────────────────────────
class Gate:
    def __init__(self):
        self.prev = None
        self.n_pass = self.n_sw = self.n_spike = 0
        self.sw_counts = defaultdict(int)

    def _status_err(self, f):
        sw = f['sw']
        self.sw_counts[sw] += 1
        if (sw & XSF_ORIENTATION_VALID) == 0: return True
        if (sw & XSF_CLIPPING_DETECTED) != 0: return True
        return False

    def eval(self, pid, bnd):
        p,l,r = bnd['Pelvis'],bnd['Left'],bnd['Right']
        if self._status_err(p) or self._status_err(l) or self._status_err(r):
            self.n_sw += 1; self.prev = bnd; return None
        if self.prev:
            for role in ('Pelvis','Left','Right'):
                pr = self.prev[role]; cr = bnd[role]
                if vec3_dist(cr['A'],pr['A']) > 100 or vec3_dist(cr['G'],pr['G']) > 20:
                    self.n_spike += 1; self.prev = bnd; return None
        self.prev = bnd; self.n_pass += 1
        return bnd

# ─── Stage 2: CalibrationProcessor ──────────────────────────────────────────
class CalibProc:
    def __init__(self):
        self.state      = 'WaitingForStart'
        self.peak       = False
        self.peak_cnt   = 0
        self.static_cnt = 0
        self.timeout    = 0
        self.buf        = {'Pelvis':[],'Left':[],'Right':[]}
        self.profile    = None
        self.stomp_pid  = None

    def pre_gate(self, pid, bnd):
        if self.state != 'WaitingForStart': return False
        vert = abs(bnd['Left']['A'][2] - 9.81)
        if not self.peak:
            if vert > 25: self.peak = True; self.peak_cnt = 1
        else:
            self.peak_cnt += 1
            if vert < 10:   # 25 * 0.4
                if self.peak_cnt <= 20:
                    self.state    = 'CollectingStaticPose'
                    self.stomp_pid = pid
                    self.peak = False; self.peak_cnt = 0
                    return True
                self.peak = False; self.peak_cnt = 0
            elif self.peak_cnt > 20:
                self.peak = False; self.peak_cnt = 0
        return False

    def process(self, bnd):
        if self.state != 'CollectingStaticPose': return False
        self.timeout += 1
        if self.timeout > 1000: self.state = 'Failed'; return True
        g_sq = 0.09   # 0.3^2
        is_static = all(
            sum(bnd[role]['G'][i]**2 for i in range(3)) < g_sq
            for role in ('Pelvis','Left','Right'))
        if is_static:
            self.static_cnt += 1
            if self.static_cnt >= 30:
                for role in ('Pelvis','Left','Right'):
                    self.buf[role].append(bnd[role]['Q'])
                if len(self.buf['Pelvis']) >= 300:
                    self.profile = {r: median_quat(self.buf[r]) for r in ('Pelvis','Left','Right')}
                    self.state = 'Completed'
                    return True
        else:
            self.static_cnt = 0
        return False

# ─── Stage 3: GaitEventDetector ──────────────────────────────────────────────
class GaitDetector:
    def __init__(self):
        self.lc = self.rc = 0
        self.l_prev = self.r_prev = False
        self.lt = self.rt = 0
        self.wf = 0

    def _is_stance(self, f, ref):
        fa_ok   = vec3_len(f['FA']) < 2.5
        gyro_ok = vec3_len(f['G'])  < 1.0
        pitch_ok = True
        if ref:
            qr = calib_q(f['Q'], ref)
            pitch_ok = abs(extract_pitch(qr)) < 0.175
        return fa_ok and gyro_ok and pitch_ok

    def detect(self, bnd, profile):
        lref = profile['Left']  if profile else None
        rref = profile['Right'] if profile else None
        l = self._is_stance(bnd['Left'],  lref)
        r = self._is_stance(bnd['Right'], rref)
        self.lc = self.lc+1 if l else 0
        self.rc = self.rc+1 if r else 0
        lc = self.lc >= 5; rc = self.rc >= 5
        if lc != self.l_prev: self.lt += 1; self.l_prev = lc
        if rc != self.r_prev: self.rt += 1; self.r_prev = rc
        self.wf += 1
        is_walk = False
        if self.wf >= 300:
            is_walk = self.lt >= 2 and self.rt >= 2
            self.wf = self.lt = self.rt = 0
        return {'L': lc, 'R': rc, 'walk': is_walk}

# ─── Stage 4: MotionContextDetector ──────────────────────────────────────────
class MotionCtx:
    def __init__(self):
        self.state  = 'Straight'
        self.t_cnt  = 0; self.s_cnt = 0
        self.dq_acc = 0.0
        self.h_win  = deque(); self.h_sum = 0.0
        self.last_y = None

    def detect(self, bnd, gait):
        yaw_rate = abs(bnd['Pelvis']['G'][1])
        dq = bnd['Pelvis']['DQ']
        self.dq_acc += 2.0 * math.atan2(dq[2], dq[3])
        yaw = extract_yaw(bnd['Pelvis']['Q'])
        if self.last_y is not None:
            d = norm_angle(yaw - self.last_y)
            self.h_win.append(d); self.h_sum += abs(d)
            if len(self.h_win) > 20: self.h_sum -= abs(self.h_win.popleft())
        self.last_y = yaw
        turning = (yaw_rate > 0.26 or self.h_sum > 0.17 or abs(self.dq_acc) > 0.17)
        if self.state == 'Straight':
            if turning:
                self.t_cnt += 1
                if self.t_cnt >= 10:
                    self.state = 'Turning'; self.t_cnt = 0; self.dq_acc = 0
                    return 'Turning', 1.0
                return 'Straight', 1.0 - self.t_cnt/10
            self.t_cnt = 0; self.dq_acc = 0
            return 'Straight', 1.0
        elif self.state == 'Turning':
            if not turning:
                self.s_cnt += 1
                if self.s_cnt >= 20:
                    self.state = 'ReacquiringPd'; self.s_cnt = 0; self.dq_acc = 0
                    return 'ReacquiringPd', 0.0
            else:
                self.s_cnt = 0
            return 'Turning', 1.0
        else:
            if turning: self.state = 'Turning'; self.s_cnt = 0
            return self.state, 0.0

    def confirm_straight(self):
        if self.state == 'ReacquiringPd': self.state = 'Straight'

# ─── Stage 5: ProgressionDirEstimator ────────────────────────────────────────
class PdEstimator:
    def __init__(self):
        self.pd = 0.0; self.valid = False; self.steps = 0
        self.hist = deque(); self.hs = 0.0; self.hss = 0.0  # hs=sumCos, hss=sumSin
        self.l_in = self.r_in = False
        self.l_yaw = self.r_yaw = 0.0
        self.l_frames = self.r_frames = 0
        self.l_ready = self.r_ready = False
        self.last_l = self.last_r = 0.0

    def update(self, bnd, gait, ctx, mc, profile):
        state, conf = ctx
        if state not in ('Straight','ReacquiringPd'): return self._build()
        pref = profile['Pelvis'] if profile else None
        lref = profile['Left']   if profile else None
        rref = profile['Right']  if profile else None
        py = extract_yaw(calib_q(bnd['Pelvis']['Q'], pref))
        self._track(bnd, gait, lref, rref)
        if not (self.l_ready and self.r_ready): return self._build()
        self.l_ready = self.r_ready = False
        self.steps += 1
        total = 1.0
        wx = (math.cos(py)*0.6 + math.cos(self.last_l)*0.2 + math.cos(self.last_r)*0.2) / total
        wy = (math.sin(py)*0.6 + math.sin(self.last_l)*0.2 + math.sin(self.last_r)*0.2) / total
        fused = math.atan2(wy, wx)
        self._hist(fused); self.pd = fused; self.valid = True
        if state == 'ReacquiringPd' and self._stab() >= 0.85 and self.steps >= 5:
            mc.confirm_straight()
        return self._build()

    def _track(self, bnd, gait, lref, rref):
        if gait['L']:
            if not self.l_in: self.l_in = True; self.l_yaw = 0; self.l_frames = 0
            self.l_yaw += extract_yaw(calib_q(bnd['Left']['Q'], lref)); self.l_frames += 1
        elif self.l_in:
            self.l_in = False
            if self.l_frames: self.last_l = self.l_yaw / self.l_frames
            self.l_ready = True
        if gait['R']:
            if not self.r_in: self.r_in = True; self.r_yaw = 0; self.r_frames = 0
            self.r_yaw += extract_yaw(calib_q(bnd['Right']['Q'], rref)); self.r_frames += 1
        elif self.r_in:
            self.r_in = False
            if self.r_frames: self.last_r = self.r_yaw / self.r_frames
            self.r_ready = True

    def _hist(self, v):
        self.hist.append(v)
        self.hs  += math.cos(v)   # sumCos
        self.hss += math.sin(v)   # sumSin
        if len(self.hist) > 10:
            o = self.hist.popleft()
            self.hs  -= math.cos(o)
            self.hss -= math.sin(o)

    def _stab(self):
        n = len(self.hist)
        if n < 2: return 0.0
        # 圆形方差：R_bar = |sum_cos + i*sum_sin| / n
        r_bar   = math.sqrt(self.hs**2 + self.hss**2) / n
        circ_var = 1.0 - r_bar
        return 1.0 / (1.0 + circ_var)

    def _build(self):
        s = self._stab()
        return {'dir': self.pd, 'valid': self.valid and s >= 0.85, 'stab': s}

# ─── Stage 6: FpaEngine ──────────────────────────────────────────────────────
class Sampler:
    def __init__(self):
        self.yaws = []; self.in_s = False; self.emitted = False
    def add(self, y):
        if not self.in_s: self.in_s = True; self.emitted = False; self.yaws = []
        self.yaws.append(y)
    def swing(self): self.in_s = False
    def settle(self, n):
        if not self.in_s or self.emitted or len(self.yaws) < n: return None
        self.emitted = True; return sum(self.yaws)/len(self.yaws)
    def reset(self): self.yaws = []; self.in_s = False; self.emitted = False

class FpaEngine:
    def __init__(self):
        self.ls = Sampler(); self.rs = Sampler()

    def process(self, bnd, gait, ctx, pd, profile):
        state, conf = ctx
        lref = profile['Left']  if profile else None
        rref = profile['Right'] if profile else None
        ly = extract_yaw(calib_q(bnd['Left']['Q'],  lref))
        ry = extract_yaw(calib_q(bnd['Right']['Q'], rref))
        if gait['L']: self.ls.add(ly)
        else:         self.ls.swing()
        if gait['R']: self.rs.add(ry)
        else:         self.rs.swing()
        if state != 'Straight' or conf < 0.7: return None
        if not pd['valid'] or pd['stab'] < 0.7: return None
        if not gait['L'] and not gait['R']:    return None
        fl = self.ls.settle(10); fr = self.rs.settle(10)
        if fl is None and fr is None: return None
        def deg(v):
            if v is None: return None
            return math.degrees(norm_angle(v - pd['dir']))
        return {'L': deg(fl), 'R': deg(fr)}

# ─── Main simulation ─────────────────────────────────────────────────────────
def run():
    print("=" * 65)
    print("  IMUMoCap Pipeline Simulation")
    print("=" * 65)

    bundles = parse_csv(CSV)
    N = len(bundles)

    gate    = Gate()
    cal     = CalibProc()
    gait    = GaitDetector()
    mctx    = MotionCtx()
    pd_est  = PdEstimator()
    fpa_eng = FpaEngine()

    # Counters / trackers
    gate_pass = 0; gate_fail_sw = 0; gate_fail_spike = 0
    cal_complete_pid = None; cal_fail = False
    gait_stats = {'L_stance':0,'R_stance':0,'both_swing':0,'both_stance':0}
    ctx_stats  = defaultdict(int)
    pd_valid_frames = 0
    fpa_results = []
    baseline_steps_l = []; baseline_steps_r = []
    in_baseline = False; in_training = False

    # Track stance durations
    l_dur = 0; r_dur = 0
    l_durs = []; r_durs = []

    # FPA gate reject reasons (per frame)
    fpa_gate_ctx = 0; fpa_gate_pd = 0; fpa_gate_both_swing = 0; fpa_gate_settle = 0

    # Pitch check analysis (pre-calibration we skip, post-calibration we track)
    pitch_l_vals = []; pitch_r_vals = []

    print(f"\n── Stage 1: Bundle stats ────────────────────────────────────")
    sw_dist = defaultdict(int)
    for pid, bnd in bundles:
        for role in ('Pelvis','Left','Right'):
            sw_dist[bnd[role]['sw']] += 1
    for sw, cnt in sorted(sw_dist.items()):
        flag_str = []
        if (sw & XSF_ORIENTATION_VALID) == 0: flag_str.append("OrientValid=0")
        if (sw & XSF_CLIPPING_DETECTED) != 0: flag_str.append("ClipDetected")
        gate_res = "PASS" if not flag_str else "REJECT"
        print(f"  SW=0x{sw:06X} ({sw:>8d}): {cnt:5d} frames  [{gate_res}: {', '.join(flag_str) or 'OK'}]")

    print(f"\n── Running full pipeline ({N} bundles) ──────────────────────")

    for i, (pid, bnd) in enumerate(bundles):
        # Pre-gate stomp detection
        if cal.state == 'WaitingForStart':
            if cal.pre_gate(pid, bnd):
                print(f"  [Frame {i:4d} / PktId {pid}] STOMP detected → CollectingStaticPose")

        # Gate
        vf = gate.eval(pid, bnd)
        if vf is None:
            if gate.n_sw > gate_fail_sw:      gate_fail_sw = gate.n_sw
            if gate.n_spike > gate_fail_spike: gate_fail_spike = gate.n_spike
            continue

        gate_pass += 1

        # Calibration
        if cal.state not in ('Completed','Failed'):
            changed = cal.process(bnd)
            if changed:
                if cal.state == 'Completed':
                    cal_complete_pid = pid
                    print(f"  [Frame {i:4d} / PktId {pid}] Calibration COMPLETED "
                          f"(timeout counter={cal.timeout})")
                elif cal.state == 'Failed':
                    cal_fail = True
                    print(f"  [Frame {i:4d} / PktId {pid}] Calibration FAILED (timeout)")
            if cal.state != 'Completed': continue

        profile = cal.profile

        # Post-cal: track pitch values
        if profile:
            qrl = calib_q(bnd['Left']['Q'],  profile['Left'])
            qrr = calib_q(bnd['Right']['Q'], profile['Right'])
            pitch_l_vals.append(abs(math.degrees(extract_pitch(qrl))))
            pitch_r_vals.append(abs(math.degrees(extract_pitch(qrr))))

        # Gait
        gait_ev = gait.detect(bnd, profile)
        if gait_ev['L'] and gait_ev['R']:  gait_stats['both_stance'] += 1
        elif not gait_ev['L'] and not gait_ev['R']: gait_stats['both_swing'] += 1
        if gait_ev['L']: gait_stats['L_stance'] += 1
        if gait_ev['R']: gait_stats['R_stance'] += 1

        # Stance duration tracking
        if gait_ev['L']: l_dur += 1
        else:
            if l_dur > 0: l_durs.append(l_dur)
            l_dur = 0
        if gait_ev['R']: r_dur += 1
        else:
            if r_dur > 0: r_durs.append(r_dur)
            r_dur = 0

        # Motion context
        ctx = mctx.detect(bnd, gait_ev)
        ctx_stats[ctx[0]] += 1

        # PD
        pd = pd_est.update(bnd, gait_ev, ctx, mctx, profile)
        if pd['valid']: pd_valid_frames += 1

        # FPA
        result = fpa_eng.process(bnd, gait_ev, ctx, pd, profile)
        if result:
            fpa_results.append(result)

    # ── Summary ─────────────────────────────────────────────────────────────
    print(f"\n── Gate results ─────────────────────────────────────────────")
    total = gate.n_pass + gate.n_sw + gate.n_spike
    print(f"  Total bundles:    {N}")
    print(f"  PASS:             {gate.n_pass}  ({gate.n_pass/N*100:.1f}%)")
    print(f"  SW reject:        {gate.n_sw}   ({gate.n_sw/N*100:.1f}%)")
    print(f"  Spike reject:     {gate.n_spike}    ({gate.n_spike/N*100:.1f}%)")

    print(f"\n── Calibration ──────────────────────────────────────────────")
    if cal.stomp_pid:
        print(f"  Stomp detected at PktId {cal.stomp_pid}")
    else:
        print(f"  STOMP NOT DETECTED — calibration never started!")
    print(f"  Final state:  {cal.state}")
    if cal.profile:
        print(f"  Static frames collected: {len(cal.buf['Pelvis'])} / 300 required")
        print(f"  LeftFootRef  yaw: {math.degrees(extract_yaw(cal.profile['Left'])):+.1f}°")
        print(f"  RightFootRef yaw: {math.degrees(extract_yaw(cal.profile['Right'])):+.1f}°")
        print(f"  PelvisRef    yaw: {math.degrees(extract_yaw(cal.profile['Pelvis'])):+.1f}°")
    else:
        print(f"  No CalibrationProfile produced.")

    post_cal = gate.n_pass - (cal.timeout if cal.profile else gate.n_pass)
    print(f"\n── Post-calibration frames ──────────────────────────────────")
    post_cal_frames = sum(1 for i,(pid,bnd) in enumerate(bundles)
                         if cal.stomp_pid and pid >= cal.stomp_pid and gate.eval.__func__ is not None)
    # Approximate: frames after calibration completed
    cal_frame_count = sum(1 for pid,_ in bundles if cal_complete_pid and pid >= cal_complete_pid)
    print(f"  Frames after calibration complete: ~{cal_frame_count}")

    print(f"\n── Gait detection (post-calibration) ───────────────────────")
    if cal_frame_count > 0:
        print(f"  Left  stance:   {gait_stats['L_stance']}  ({gait_stats['L_stance']/max(cal_frame_count,1)*100:.1f}%)")
        print(f"  Right stance:   {gait_stats['R_stance']}  ({gait_stats['R_stance']/max(cal_frame_count,1)*100:.1f}%)")
        print(f"  Both in swing:  {gait_stats['both_swing']} ({gait_stats['both_swing']/max(cal_frame_count,1)*100:.1f}%)")
        print(f"  Both in stance: {gait_stats['both_stance']}  ({gait_stats['both_stance']/max(cal_frame_count,1)*100:.1f}%)")
        if l_durs:
            print(f"  Left  stance duration:  mean={sum(l_durs)/len(l_durs):.1f} frames  "
                  f"min={min(l_durs)}  max={max(l_durs)}  count={len(l_durs)}")
        else:
            print(f"  Left  stance: NO STANCE PERIODS DETECTED")
        if r_durs:
            print(f"  Right stance duration:  mean={sum(r_durs)/len(r_durs):.1f} frames  "
                  f"min={min(r_durs)}  max={max(r_durs)}  count={len(r_durs)}")
        else:
            print(f"  Right stance: NO STANCE PERIODS DETECTED")

    print(f"\n── Pitch values (post-calibration, |pitch| °) ───────────────")
    if pitch_l_vals:
        import statistics
        print(f"  Left  foot:  mean={statistics.mean(pitch_l_vals):.1f}°  "
              f"median={statistics.median(pitch_l_vals):.1f}°  "
              f"max={max(pitch_l_vals):.1f}°  "
              f">10°: {sum(1 for v in pitch_l_vals if v>10)/len(pitch_l_vals)*100:.1f}%")
        print(f"  Right foot:  mean={statistics.mean(pitch_r_vals):.1f}°  "
              f"median={statistics.median(pitch_r_vals):.1f}°  "
              f"max={max(pitch_r_vals):.1f}°  "
              f">10°: {sum(1 for v in pitch_r_vals if v>10)/len(pitch_r_vals)*100:.1f}%")
    else:
        print("  (no post-calibration frames)")

    print(f"\n── Motion context ───────────────────────────────────────────")
    for state, cnt in sorted(ctx_stats.items()):
        print(f"  {state:16s}: {cnt:5d}  ({cnt/max(sum(ctx_stats.values()),1)*100:.1f}%)")

    print(f"\n── PD estimation ────────────────────────────────────────────")
    total_post = sum(ctx_stats.values())
    print(f"  PD valid frames:  {pd_valid_frames}  ({pd_valid_frames/max(total_post,1)*100:.1f}% of post-cal frames)")
    print(f"  PD steps counted: {pd_est.steps}")
    if pd_est.valid:
        print(f"  Final PD direction: {math.degrees(pd_est.pd):.1f}°  stability={pd_est._stab():.3f}")
    else:
        print(f"  PD never reached valid state (need {pd_est.steps} steps with stability ≥ 0.85)")

    print(f"\n── FPA output ───────────────────────────────────────────────")
    print(f"  FPA results produced: {len(fpa_results)}")
    if fpa_results:
        ls = [r['L'] for r in fpa_results if r['L'] is not None]
        rs = [r['R'] for r in fpa_results if r['R'] is not None]
        if ls:
            import statistics
            print(f"  Left  FPA:  mean={statistics.mean(ls):+.1f}°  "
                  f"sd={statistics.stdev(ls) if len(ls)>1 else 0:.1f}°  "
                  f"range=[{min(ls):+.1f}°, {max(ls):+.1f}°]  n={len(ls)}")
        if rs:
            import statistics
            print(f"  Right FPA:  mean={statistics.mean(rs):+.1f}°  "
                  f"sd={statistics.stdev(rs) if len(rs)>1 else 0:.1f}°  "
                  f"range=[{min(rs):+.1f}°, {max(rs):+.1f}°]  n={len(rs)}")
        print(f"\n  All FPA readings:")
        for j, r in enumerate(fpa_results):
            ls_str = f"L={r['L']:+.1f}°" if r['L'] is not None else "L=---"
            rs_str = f"R={r['R']:+.1f}°" if r['R'] is not None else "R=---"
            print(f"    #{j+1:2d}: {ls_str}  {rs_str}")
    else:
        # Diagnose WHY fpa is zero
        print(f"\n  DIAGNOSING: why no FPA output?")
        print(f"  Context frames Straight: {ctx_stats.get('Straight',0)}")
        print(f"  PD valid frames:         {pd_valid_frames}")
        print(f"  Stance L periods:        {len(l_durs)}")
        print(f"  Stance R periods:        {len(r_durs)}")
        if not cal.profile:
            print(f"  → Calibration never completed")
        elif not l_durs and not r_durs:
            print(f"  → Stance never detected (all pitch_ok failures?)")
        elif pd_valid_frames == 0:
            print(f"  → PD never became valid (not enough steps with stability≥0.85)")
        else:
            print(f"  → Gate or settle condition never met")

    print(f"\n── Bug / Issue Checks (after fixes) ────────────────────────")
    # Fix 1: circular statistics验证
    # 模拟方向接近±π场景下的稳定性
    pd_test = PdEstimator()
    for angle in [3.10, -3.12, 3.11, -3.13, 3.09, -3.12, 3.10, -3.11, 3.12, -3.10]:
        pd_test._hist(angle)
    stab_circular = pd_test._stab()
    # 旧线性方式（用于对比）
    lin_sum = sum([3.10, -3.12, 3.11, -3.13, 3.09, -3.12, 3.10, -3.11, 3.12, -3.10])
    lin_sq  = sum(v*v for v in [3.10, -3.12, 3.11, -3.13, 3.09, -3.12, 3.10, -3.11, 3.12, -3.10])
    n = 10; lin_mean = lin_sum/n; lin_var = max(0, lin_sq/n - lin_mean*lin_mean)
    stab_linear = 1.0/(1.0+lin_var)
    print(f"  [Fix 1] PD near ±180° stress test (directions alternating ±3.1 rad):")
    print(f"    Old (linear) stability = {stab_linear:.4f}  → {'WOULD FAIL (< 0.85)' if stab_linear<0.85 else 'pass'}")
    print(f"    New (circular) stability = {stab_circular:.4f} → {'pass' if stab_circular>=0.85 else 'FAIL'}")
    if pd_est.steps >= 2:
        print(f"  [Fix 1] Actual data: PD stability={pd_est._stab():.4f}  steps={pd_est.steps}")

    # Fix 2: baseline IsReady兜底逻辑
    min_steps = 20
    print(f"\n  [Fix 2] Baseline IsReady simulation:")
    cases = [(20, 20, "正常"), (20, 8, "右脚明显滞后"), (40, 5, "兜底触发（40≥2×20）"), (15, 15, "两脚均未达标")]
    for l_cnt, r_cnt, desc in cases:
        old_ready = l_cnt >= min_steps and r_cnt >= min_steps
        new_ready = (l_cnt >= min_steps and r_cnt >= min_steps) or max(l_cnt, r_cnt) >= min_steps*2
        changed = "★ 行为改变" if old_ready != new_ready else ""
        print(f"    L={l_cnt:2d} R={r_cnt:2d} ({desc}): 旧={old_ready} 新={new_ready} {changed}")

    # 2. Pitch gate selectivity
    if pitch_l_vals and pitch_r_vals:
        pct_l = sum(1 for v in pitch_l_vals if v > 10) / len(pitch_l_vals) * 100
        pct_r = sum(1 for v in pitch_r_vals if v > 10) / len(pitch_r_vals) * 100
        if pct_l > 60 or pct_r > 60:
            print(f"  [WARN] Pitch threshold 10° rejects >{max(pct_l,pct_r):.0f}% of frames")
            print(f"         Stance detection heavily blocked by pitch check")
        else:
            print(f"  [OK] Pitch gate: L rejects {pct_l:.1f}%, R rejects {pct_r:.1f}% of post-cal frames")

    # 3. Stomp detection
    if not cal.stomp_pid:
        print(f"  [FAIL] No stomp in this dataset → calibration never ran")
        print(f"         Max left vert acc in data: check below")
        max_va = max(abs(bnd['Left']['A'][2]-9.81) for _,bnd in bundles)
        print(f"         Max |Az-g| = {max_va:.2f} m/s² (threshold=25)")

    # 4. StatusWord distribution
    sw2 = sum(1 for _,bnd in bundles for r in ('Pelvis','Left','Right') if bnd[r]['sw']==2)
    sw0 = sum(1 for _,bnd in bundles for r in ('Pelvis','Left','Right') if bnd[r]['sw']==0)
    sw_clip = sum(1 for _,bnd in bundles for r in ('Pelvis','Left','Right') if bnd[r]['sw'] & XSF_CLIPPING_DETECTED)
    print(f"  [INFO] SW distribution: valid={sw2}, no-orient={sw0}, clipping={sw_clip}")

    # 5. Incomplete bundles
    by_pkt = defaultdict(dict)
    with open(CSV, encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            by_pkt[int(r['PacketId'])][r['Role']] = 1
    incomplete = sum(1 for pid in by_pkt
                     if not all(k in by_pkt[pid] for k in ('Pelvis','Left','Right')))
    if incomplete:
        print(f"  [WARN] {incomplete} PacketIds with missing IMU(s) → lost synchronization")
    else:
        print(f"  [OK] All PacketIds have all 3 IMUs")

    # 6. TryConsumeStep: requires BOTH feet's yaw to be ready simultaneously
    print(f"\n  [NOTE] PD TryConsumeStep requires both L+R yaw ready in same call.")
    print(f"         If one foot has a very short stance and transitions before the other,")
    print(f"         steps can be silently dropped.")

    print(f"\n{'='*65}")

if __name__ == '__main__':
    run()
