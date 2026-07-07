"""
Task 4: PD estimator verification
Mirrors ProgressionDirEstimator.cs (axis detection, stance-yaw fusion, stability, ConfirmStraight)
"""
import sys, math, csv
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260619_195332.csv"

# ── constants ─────────────────────────────────────────────────────────────────
STATIC_GYRO=0.3; STATIC_REQ=10; STATIC_COLLECT=50; STATIC_TIMEOUT=300
STOMP_THRESH=15.0; STOMP_MAX=20
XSF_ORIENT_VALID=0x02; XSF_CLIPPING=0x00080000
FREE_ACC_THR=2.5; GYRO_THR=1.0; PITCH_THR=0.35; MIN_STANCE_F=5
WALK_WINDOW=300; WALK_MIN_TRANS=2
YAW_RATE_THR=0.70; DELTA_Q_THR=0.17
TURN_CONFIRM=10; STR_CONFIRM=10; REACQ_CONFIRM=10
PELVIS_W=0.6; LEFT_W=0.2; RIGHT_W=0.2
STAB_WINDOW=10; STAB_THR=0.85; MIN_STEPS=2

# ── helpers ───────────────────────────────────────────────────────────────────
def quat_inv(q):
    x,y,z,w=q; n2=x*x+y*y+z*z+w*w; return (-x/n2,-y/n2,-z/n2,w/n2)
def quat_mul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
def vec_transform(v,q):
    vx,vy,vz=v; x,y,z,w=q
    rx=vx*(w*w+x*x-y*y-z*z)+vy*2*(x*y-w*z)+vz*2*(x*z+w*y)
    ry=vx*2*(x*y+w*z)+vy*(w*w-x*x+y*y-z*z)+vz*2*(y*z-w*x)
    rz=vx*2*(x*z-w*y)+vy*2*(y*z+w*x)+vz*(w*w-x*x-y*y+z*z)
    return (rx,ry,rz)
def calibrate(q, ref): return quat_mul(quat_inv(ref), q)
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
def detect_axis(ref):
    gx,gy,gz=vec_transform((0,0,-1),quat_inv(ref))
    ax,ay,az=abs(gx),abs(gy),abs(gz)
    if ax>=ay and ax>=az: return 0
    if ay>=ax and ay>=az: return 1
    return 2
def extract_heading(q, axis):
    x,y,z,w=q
    if axis==0: return math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    if axis==1: return math.asin(max(-1.0,min(1.0, 2*(w*y-z*x))))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

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
cal=None; st='Wait'; static_n=0; stomp_peak=False; stomp_fc=0
buf={'Pelvis':[],'Left':[],'Right':[]}; cnt=0; cal_pid=None; tc=0
for pid in pids:
    f=frames[pid]
    if 'Left' not in f: continue
    if st=='Wait':
        va=abs(f['Left']['a'][2]-9.81)
        if not stomp_peak:
            if va>STOMP_THRESH: stomp_peak=True; stomp_fc=1
        else:
            stomp_fc+=1
            if va<STOMP_THRESH*0.4:
                if stomp_fc<=STOMP_MAX: st='Collect'
                stomp_peak=False; stomp_fc=0
            elif stomp_fc>STOMP_MAX: stomp_peak=False; stomp_fc=0
    if st=='Collect' and gate_ok(f):
        tc+=1
        if tc>STATIC_TIMEOUT: break
        pg,lg,rg=[math.sqrt(sum(v**2 for v in f[r]['g'])) for r in ['Pelvis','Left','Right']]
        ok=pg<STATIC_GYRO and lg<STATIC_GYRO and rg<STATIC_GYRO
        static_n=static_n+1 if ok else 0
        if static_n>=STATIC_REQ:
            cnt+=1
            for r in ['Pelvis','Left','Right']: buf[r].append(f[r]['q'])
            if cnt>=STATIC_COLLECT:
                cal={r:median_quat(buf[r]) for r in ['Pelvis','Left','Right']}
                cal_pid=pid; break
if cal is None: print("Calibration failed"); sys.exit(1)

p_axis = detect_axis(cal['Pelvis'])
l_axis = detect_axis(cal['Left'])
r_axis = detect_axis(cal['Right'])
print(f"Calibration pid={cal_pid}  axes: Pelvis={p_axis} Left={l_axis} Right={r_axis}\n")

# ── stance + IsWalking ────────────────────────────────────────────────────────
def is_stance_raw(fs, cal_ref):
    if math.sqrt(sum(v**2 for v in fs['fa']))>=FREE_ACC_THR: return False
    if math.sqrt(sum(v**2 for v in fs['g']))>=GYRO_THR: return False
    if cal_ref:
        g_ref=vec_transform((0,0,-1),quat_inv(cal_ref))
        g_now=vec_transform((0,0,-1),quat_inv(fs['q']))
        dot=max(-1.0,min(1.0,sum(a*b for a,b in zip(g_ref,g_now))))
        if math.acos(dot)>=PITCH_THR: return False
    return True

l_cnt=0; r_cnt=0; l_prev=False; r_prev=False
l_trans=0; r_trans=0; walk_fc=0; is_walking=False

# ── motion context ────────────────────────────────────────────────────────────
ctx='Straight'; turn_f=0; str_f=0; reacq_f=0; dq_acc=0.0
prev_ctx='Straight'

def step_ctx(pq,pg,pdq):
    global ctx,turn_f,str_f,reacq_f,dq_acc
    yr=abs(vec_transform(pg,pq)[2])
    dq_acc+=2*math.atan2(pdq[2],pdq[3])
    sig=yr>YAW_RATE_THR or abs(dq_acc)>DELTA_Q_THR
    if ctx=='Straight':
        if sig:
            turn_f+=1
            if turn_f>=TURN_CONFIRM: ctx='Turning'; turn_f=0; str_f=0; dq_acc=0.0
        else: turn_f=0; dq_acc=0.0
    elif ctx=='Turning':
        dq_acc=0.0
        if not sig:
            str_f+=1
            if str_f>=STR_CONFIRM: ctx='ReacquiringPd'; str_f=0; reacq_f=0
        else: str_f=0
    else:
        dq_acc=0.0
        if sig:
            reacq_f+=1
            if reacq_f>=REACQ_CONFIRM: ctx='Turning'; str_f=0; reacq_f=0
        else: reacq_f=0
    return ctx

# ── PD estimator (mirrors ProgressionDirEstimator.cs) ────────────────────────
cur_pd=0.0; has_est=False; step_n=0
pd_hist=deque(); pd_cos=0.0; pd_sin=0.0
li=False; ri=False
lcs=lss=rcs=rss=0.0; lf_n=rf_n=0
lyr=ryr=False; last_ly=last_ry=0.0

def compute_stability():
    n=len(pd_hist)
    if n<2: return 0.0
    r=math.sqrt(pd_cos**2+pd_sin**2)/n
    return 1.0/(1.0+(1.0-r))

def confirm_straight():
    global ctx
    if ctx=='ReacquiringPd': ctx='Straight'

def step_pd(pq,lq,rq,ls,rs,ctx_now):
    global cur_pd,has_est,step_n,prev_ctx
    global li,ri,lcs,lss,rcs,rss,lf_n,rf_n,lyr,ryr,last_ly,last_ry,pd_cos,pd_sin

    if ctx_now not in ('Straight','ReacquiringPd'):
        prev_ctx=ctx_now; return None

    # flush on Turning→ReacquiringPd
    if ctx_now=='ReacquiringPd' and prev_ctx=='Turning':
        pd_hist.clear(); pd_cos=0.0; pd_sin=0.0; step_n=0; has_est=False
        lyr=ryr=False; li=ri=False; lcs=lss=rcs=rss=0.0; lf_n=rf_n=0
    prev_ctx=ctx_now

    # track stance yaw
    if ls:
        if not li: li=True; lcs=0.0; lss=0.0; lf_n=0
        ly=extract_heading(calibrate(lq, cal['Left']), l_axis)
        lcs+=math.cos(ly); lss+=math.sin(ly); lf_n+=1
    elif li:
        li=False; last_ly=math.atan2(lss,lcs) if lf_n>0 else last_ly; lyr=True
    if rs:
        if not ri: ri=True; rcs=0.0; rss=0.0; rf_n=0
        ry=extract_heading(calibrate(rq, cal['Right']), r_axis)
        rcs+=math.cos(ry); rss+=math.sin(ry); rf_n+=1
    elif ri:
        ri=False; last_ry=math.atan2(rss,rcs) if rf_n>0 else last_ry; ryr=True

    if not (lyr and ryr): return None
    lyr=ryr=False; step_n+=1

    py=extract_heading(calibrate(pq, cal['Pelvis']), p_axis)
    tw=PELVIS_W+LEFT_W+RIGHT_W
    wx=(math.cos(py)*PELVIS_W+math.cos(last_ly)*LEFT_W+math.cos(last_ry)*RIGHT_W)/tw
    wy=(math.sin(py)*PELVIS_W+math.sin(last_ly)*LEFT_W+math.sin(last_ry)*RIGHT_W)/tw
    fused=math.atan2(wy,wx)

    pd_hist.append(fused); pd_cos+=math.cos(fused); pd_sin+=math.sin(fused)
    if len(pd_hist)>STAB_WINDOW:
        old=pd_hist.popleft(); pd_cos-=math.cos(old); pd_sin-=math.sin(old)

    cur_pd=fused; has_est=True
    stab=compute_stability()
    is_valid=has_est and stab>=STAB_THR

    if ctx_now=='ReacquiringPd' and stab>=STAB_THR and step_n>=MIN_STEPS:
        confirm_straight()

    return {'step':step_n,'ctx':ctx_now,'pelvis':math.degrees(py),
            'left':math.degrees(last_ly),'right':math.degrees(last_ry),
            'fused':math.degrees(fused),'stab':stab,'valid':is_valid}

# ── main loop ─────────────────────────────────────────────────────────────────
print(f"{'pid':>7} {'ctx':>14} {'step':>4} {'pelvis°':>8} {'left°':>7} {'right°':>7} {'fused°':>7} {'stab':>6} {'valid':>5}")
print("-"*72)

steps_pre_turn=[]; steps_post_turn=[]
turned=False

for pid in pids:
    if pid<=cal_pid: continue
    f=frames[pid]
    if not all(r in f for r in ['Pelvis','Left','Right']): continue

    ls_raw=is_stance_raw(f['Left'],cal['Left'])
    rs_raw=is_stance_raw(f['Right'],cal['Right'])
    l_cnt=l_cnt+1 if ls_raw else 0
    r_cnt=r_cnt+1 if rs_raw else 0
    ls=l_cnt>=MIN_STANCE_F; rs=r_cnt>=MIN_STANCE_F
    if ls!=l_prev: l_trans+=1; l_prev=ls
    if rs!=r_prev: r_trans+=1; r_prev=rs
    walk_fc+=1
    if walk_fc>=WALK_WINDOW:
        is_walking=l_trans>=WALK_MIN_TRANS and r_trans>=WALK_MIN_TRANS
        walk_fc=0; l_trans=0; r_trans=0

    pf=f['Pelvis']
    old_ctx=ctx
    ctx_now=step_ctx(pf['q'],pf['g'],pf['dq'])
    if old_ctx=='Turning' and ctx_now=='ReacquiringPd': turned=True

    res=step_pd(pf['q'],f['Left']['q'],f['Right']['q'],ls,rs,ctx_now)
    if res:
        r=res
        print(f"{pid:>7} {r['ctx']:>14} {r['step']:>4} {r['pelvis']:>+8.2f} {r['left']:>+7.2f} {r['right']:>+7.2f} {r['fused']:>+7.2f} {r['stab']:>6.3f} {'Y' if r['valid'] else 'N':>5}")
        if not turned: steps_pre_turn.append(r)
        else:          steps_post_turn.append(r)

# ── summary ───────────────────────────────────────────────────────────────────
print()
if len(steps_pre_turn)>=2:
    diffs=[abs(steps_pre_turn[i+1]['fused']-steps_pre_turn[i]['fused']) for i in range(len(steps_pre_turn)-1)]
    diffs=[d if d<=180 else 360-d for d in diffs]
    print(f"Pre-turn:  {len(steps_pre_turn)} steps, max step-to-step Δ={max(diffs):.1f}°, valid={all(s['valid'] for s in steps_pre_turn)}")
if len(steps_post_turn)>=2:
    diffs=[abs(steps_post_turn[i+1]['fused']-steps_post_turn[i]['fused']) for i in range(len(steps_post_turn)-1)]
    diffs=[d if d<=180 else 360-d for d in diffs]
    valid_post=[s for s in steps_post_turn if s['valid']]
    print(f"Post-turn: {len(steps_post_turn)} steps, max step-to-step Δ={max(diffs):.1f}°, first valid at step {valid_post[0]['step'] if valid_post else 'N/A'}")
