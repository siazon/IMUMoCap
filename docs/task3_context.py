"""
Task 3: MotionContext (Straight/Turning/ReacquiringPd) verification
Mirrors MotionContextDetector.cs exactly.
"""
import sys, math, csv

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260609_201647.csv"

# ── constants ──────────────────────────────────────────────────────────────────
STATIC_GYRO=0.3; STATIC_REQ=10; STATIC_COLLECT=50; STATIC_TIMEOUT=300
STOMP_THRESH=15.0; STOMP_MAX=20
XSF_ORIENT_VALID=0x02; XSF_CLIPPING=0x00080000

YAW_RATE_THR   = 0.70   # rad/s
DELTA_Q_THR    = 0.17   # rad
TURN_CONFIRM   = 10
STR_CONFIRM    = 10
REACQ_CONFIRM  = 10

FREE_ACC_THR=2.5; GYRO_THR=1.0; PITCH_THR=0.35; MIN_STANCE_F=5
WALK_WINDOW=100; WALK_MIN_TRANS=2

# ── helpers ───────────────────────────────────────────────────────────────────
def quat_inv(q):
    x,y,z,w=q; n2=x*x+y*y+z*z+w*w; return (-x/n2,-y/n2,-z/n2,w/n2)
def vec_transform(v,q):
    vx,vy,vz=v; x,y,z,w=q
    rx=vx*(w*w+x*x-y*y-z*z)+vy*2*(x*y-w*z)+vz*2*(x*z+w*y)
    ry=vx*2*(x*y+w*z)+vy*(w*w-x*x+y*y-z*z)+vz*2*(y*z-w*x)
    rz=vx*2*(x*z-w*y)+vy*2*(y*z+w*x)+vz*(w*w-x*x-y*y+z*z)
    return (rx,ry,rz)
def median_quat(s):
    def med(v): s2=sorted(v); n=len(v); return (s2[n//2-1]+s2[n//2])/2 if n%2==0 else s2[n//2]
    comps=list(zip(*s)); mx,my,mz,mw=[med(list(c)) for c in comps]
    n=math.sqrt(mx*mx+my*my+mz*mz+mw*mw); return (mx/n,my/n,mz/n,mw/n)
def gate_ok(f):
    if not all(r in f for r in ['Pelvis','Left','Right']): return False
    for r in ['Pelvis','Left','Right']:
        sw=f[r]['sw']
        if (sw&XSF_ORIENT_VALID)==0 or (sw&XSF_CLIPPING)!=0: return False
    return True

# ── load ─────────────────────────────────────────────────────────────────────
frames={}
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid=int(row['PacketId']); role=row['Role'].strip()
        def fv(k): return float(row[k])
        frames.setdefault(pid,{})[role]={
            'q':(fv('Qx'),fv('Qy'),fv('Qz'),fv('Qw')),
            'g':(fv('Gx'),fv('Gy'),fv('Gz')),
            'fa':(fv('FreeAx'),fv('FreeAy'),fv('FreeAz')),
            'a':(fv('Ax'),fv('Ay'),fv('Az')),
            'dq':(fv('DQx'),fv('DQy'),fv('DQz'),fv('DQw')),
            'sw':int(row['StatusWord']),
        }
pids=sorted(frames.keys())

# ── calibration ───────────────────────────────────────────────────────────────
cal=None; state='Wait'; static_n=0
stomp_peak=False; stomp_fc=0
buf={'Pelvis':[],'Left':[],'Right':[]}; cnt=0; cal_pid=None; timeout_counter=0
for pid in pids:
    f=frames[pid]
    if 'Left' not in f: continue
    if state=='Wait':
        va=abs(f['Left']['a'][2]-9.81)
        if not stomp_peak:
            if va>STOMP_THRESH: stomp_peak=True; stomp_fc=1
        else:
            stomp_fc+=1
            if va<STOMP_THRESH*0.4:
                if stomp_fc<=STOMP_MAX: state='Collect'
                stomp_peak=False; stomp_fc=0
            elif stomp_fc>STOMP_MAX: stomp_peak=False; stomp_fc=0
    if state=='Collect' and gate_ok(f):
        timeout_counter+=1
        if timeout_counter>STATIC_TIMEOUT: break
        pg,lg,rg=[math.sqrt(sum(v**2 for v in f[r]['g'])) for r in ['Pelvis','Left','Right']]
        ok2=pg<STATIC_GYRO and lg<STATIC_GYRO and rg<STATIC_GYRO
        static_n=static_n+1 if ok2 else 0
        if static_n>=STATIC_REQ:
            cnt+=1
            for r in ['Pelvis','Left','Right']: buf[r].append(f[r]['q'])
            if cnt>=STATIC_COLLECT:
                cal={r:median_quat(buf[r]) for r in ['Pelvis','Left','Right']}
                cal_pid=pid; break
if cal is None: print("Calibration failed"); sys.exit(1)
print(f"Calibration complete: pid={cal_pid}\n")

# ── stance + IsWalking (Task 2, reused) ───────────────────────────────────────
def is_stance_raw(f_sensor, cal_ref):
    fa_mag=math.sqrt(sum(v**2 for v in f_sensor['fa']))
    g_mag =math.sqrt(sum(v**2 for v in f_sensor['g']))
    if fa_mag>=FREE_ACC_THR or g_mag>=GYRO_THR: return False
    if cal_ref:
        g_ref=vec_transform((0,0,-1),quat_inv(cal_ref))
        g_now=vec_transform((0,0,-1),quat_inv(f_sensor['q']))
        dot=max(-1.0,min(1.0,sum(a*b for a,b in zip(g_ref,g_now))))
        if math.acos(dot)>=PITCH_THR: return False
    return True

l_cnt=0; r_cnt=0; l_prev=False; r_prev=False
l_trans=0; r_trans=0; walk_fc=0; is_walking=False

# ── motion context state machine ──────────────────────────────────────────────
ctx_state='Straight'; turn_f=0; str_f=0; reacq_f=0; dq_accum=0.0

def step_context(pq, pg, pdq, is_walking):
    global ctx_state, turn_f, str_f, reacq_f, dq_accum
    yr = abs(vec_transform(pg, pq)[2])
    dq_accum += 2*math.atan2(pdq[2], pdq[3])
    sig = yr > YAW_RATE_THR or abs(dq_accum) > DELTA_Q_THR

    if ctx_state == 'Straight':
        if sig:
            turn_f += 1
            if turn_f >= TURN_CONFIRM:
                ctx_state = 'Turning'; turn_f = 0; str_f = 0; dq_accum = 0.0
        else:
            turn_f = 0; dq_accum = 0.0
    elif ctx_state == 'Turning':
        dq_accum = 0.0
        if not sig:
            str_f += 1
            if str_f >= STR_CONFIRM:
                ctx_state = 'ReacquiringPd'; str_f = 0; reacq_f = 0
        else:
            str_f = 0
    else:  # ReacquiringPd
        dq_accum = 0.0
        if sig:
            reacq_f += 1
            if reacq_f >= REACQ_CONFIRM:
                ctx_state = 'Turning'; str_f = 0; reacq_f = 0
        else:
            reacq_f = 0

    conf = 1.0 if ctx_state == 'Straight' else 0.0
    if ctx_state == 'Straight' and not is_walking:
        conf *= 0.5
    return ctx_state, conf, yr

# ── main loop ─────────────────────────────────────────────────────────────────
transitions = []   # (pid, old_state, new_state)
prev_ctx = 'Straight'
state_durations = {'Straight': 0, 'Turning': 0, 'ReacquiringPd': 0}

for pid in pids:
    if pid <= cal_pid: continue
    f = frames[pid]
    if not all(r in f for r in ['Pelvis','Left','Right']): continue

    # stance / IsWalking
    ls_raw = is_stance_raw(f['Left'],  cal['Left'])
    rs_raw = is_stance_raw(f['Right'], cal['Right'])
    l_cnt = l_cnt+1 if ls_raw else 0
    r_cnt = r_cnt+1 if rs_raw else 0
    ls = l_cnt >= MIN_STANCE_F
    rs = r_cnt >= MIN_STANCE_F
    if ls != l_prev: l_trans += 1; l_prev = ls
    if rs != r_prev: r_trans += 1; r_prev = rs
    walk_fc += 1
    if walk_fc >= WALK_WINDOW:
        is_walking = l_trans >= WALK_MIN_TRANS and r_trans >= WALK_MIN_TRANS
        walk_fc = 0; l_trans = 0; r_trans = 0

    pf = f['Pelvis']
    ctx, conf, yr = step_context(pf['q'], pf['g'], pf['dq'], is_walking)

    state_durations[ctx] = state_durations.get(ctx, 0) + 1

    if ctx != prev_ctx:
        transitions.append((pid, prev_ctx, ctx, yr))
        prev_ctx = ctx

# ── output ────────────────────────────────────────────────────────────────────
total = sum(state_durations.values())
print(f"State distribution ({total} frames after calibration):")
for s, n in state_durations.items():
    print(f"  {s:<16}: {n:>5} frames  ({100*n/total:5.1f}%)")

print(f"\nState transitions:")
print(f"  {'pid':>7}  {'from':<16} -> {'to':<16}  pelvis_yr(rad/s)")
print("  " + "-"*60)
for pid, old, new, yr in transitions:
    print(f"  {pid:>7}  {old:<16} -> {new:<16}  {yr:.3f}")

print(f"\nTotal transitions: {len(transitions)}")
straight_false = sum(1 for _,old,new,_ in transitions if old=='Straight')
print(f"Straight->Turning entries: {straight_false}  (should equal number of actual turns)")
