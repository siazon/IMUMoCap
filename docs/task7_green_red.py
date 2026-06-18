"""
Task 7: Per-step green/red display with updated tolerance formula max(4°, SD).
Baseline: first BASELINE_N steps per foot.
Training: all remaining steps, each shown as green (on-target) or red (off-target).
"""
import sys, math, csv
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260608_163541.csv"
BASELINE_N = 20

STATIC_GYRO=0.3; STATIC_REQ=10; STATIC_COLLECT=50; STATIC_TIMEOUT=300
STOMP_THRESH=15.0; STOMP_MAX=20
XSF_ORIENT_VALID=0x02; XSF_CLIPPING=0x00080000
FREE_ACC_THR=2.5; GYRO_THR=1.0; PITCH_THR=0.35; MIN_STANCE_F=5
WALK_WINDOW=100; WALK_MIN_TRANS=2
YAW_RATE_THR=0.70; DELTA_Q_THR=0.17; TURN_CONFIRM=10; STR_CONFIRM=10; REACQ_CONFIRM=10
PELVIS_W=0.6; LEFT_W=0.2; RIGHT_W=0.2
STAB_WINDOW=3; STAB_THR=0.65; MIN_STEPS=2
CTX_CONF_THR=0.7; PD_STAB_THR=0.7; MIN_SETTLE=5

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
def calibrate(q,ref): return quat_mul(quat_inv(ref),q)
def extract_yaw_z(q):
    x,y,z,w=q; return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def normalize_angle(a):
    while a>math.pi: a-=2*math.pi
    while a<-math.pi: a+=2*math.pi
    return a
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
    return 0 if ax>=ay and ax>=az else (1 if ay>=ax and ay>=az else 2)
def extract_heading(q,axis):
    x,y,z,w=q
    if axis==0: return math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    if axis==1: return math.asin(max(-1.0,min(1.0,2*(w*y-z*x))))
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

class StanceSampler:
    def __init__(self): self.yaws=[]; self.in_stance=False; self.emitted=False
    def add_frame(self,yaw):
        if not self.in_stance: self.in_stance=True; self.emitted=False; self.yaws=[]
        self.yaws.append(yaw)
    def mark_swing(self): self.in_stance=False
    def try_settle(self,min_f):
        if not self.in_stance or self.emitted: return None
        if len(self.yaws)<min_f: return None
        self.emitted=True; return sum(self.yaws)/len(self.yaws)

frames={}
with open(CSV_PATH,encoding='utf-8-sig') as f:
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

cal=None; cst='Wait'; static_n=0; stomp_peak=False; stomp_fc=0
buf={'Pelvis':[],'Left':[],'Right':[]}; cnt=0; cal_pid=None; tc=0
for pid in pids:
    f=frames[pid]
    if 'Left' not in f: continue
    if cst=='Wait':
        va=abs(f['Left']['a'][2]-9.81)
        if not stomp_peak:
            if va>STOMP_THRESH: stomp_peak=True; stomp_fc=1
        else:
            stomp_fc+=1
            if va<STOMP_THRESH*0.4:
                if stomp_fc<=STOMP_MAX: cst='Collect'
                stomp_peak=False; stomp_fc=0
            elif stomp_fc>STOMP_MAX: stomp_peak=False; stomp_fc=0
    if cst=='Collect' and gate_ok(f):
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
p_axis=detect_axis(cal['Pelvis']); l_axis=detect_axis(cal['Left']); r_axis=detect_axis(cal['Right'])

def is_stance_raw(fs,cal_ref):
    if math.sqrt(sum(v**2 for v in fs['fa']))>=FREE_ACC_THR: return False
    if math.sqrt(sum(v**2 for v in fs['g']))>=GYRO_THR: return False
    if cal_ref:
        g_ref=vec_transform((0,0,-1),quat_inv(cal_ref))
        g_now=vec_transform((0,0,-1),quat_inv(fs['q']))
        if math.acos(max(-1.0,min(1.0,sum(a*b for a,b in zip(g_ref,g_now)))))>=PITCH_THR: return False
    return True

l_cnt=0; r_cnt=0; l_prev=False; r_prev=False
l_trans=0; r_trans=0; walk_fc=0; is_walking=False
ctx='Straight'; turn_f=0; str_f=0; reacq_f=0; dq_acc=0.0; prev_ctx='Straight'

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
    conf=1.0 if ctx=='Straight' else 0.0
    if ctx=='Straight' and not is_walking: conf*=0.5
    return ctx, conf

cur_pd=0.0; has_est=False; step_n=0
pd_hist=deque(); pd_cos=0.0; pd_sin=0.0
li=False; ri=False; lcs=lss=rcs=rss=0.0; lf_n=rf_n=0
lyr=ryr=False; last_ly=last_ry=0.0

def pd_stability():
    n=len(pd_hist)
    if n<2: return 0.0
    r=math.sqrt(pd_cos**2+pd_sin**2)/n; return 1.0/(1.0+(1.0-r))

def confirm_straight():
    global ctx
    if ctx=='ReacquiringPd': ctx='Straight'

def step_pd(pq,lq,rq,ls,rs,ctx_now):
    global cur_pd,has_est,step_n,prev_ctx
    global li,ri,lcs,lss,rcs,rss,lf_n,rf_n,lyr,ryr,last_ly,last_ry,pd_cos,pd_sin
    if ctx_now not in ('Straight','ReacquiringPd'):
        prev_ctx=ctx_now; return cur_pd, has_est, pd_stability()
    if ctx_now=='ReacquiringPd' and prev_ctx=='Turning':
        pd_hist.clear(); pd_cos=0.0; pd_sin=0.0; step_n=0; has_est=False
        lyr=ryr=False; li=ri=False; lcs=lss=rcs=rss=0.0; lf_n=rf_n=0
    prev_ctx=ctx_now
    if ls:
        if not li: li=True; lcs=0.0; lss=0.0; lf_n=0
        ly=extract_heading(calibrate(lq,cal['Left']),l_axis)
        lcs+=math.cos(ly); lss+=math.sin(ly); lf_n+=1
    elif li:
        li=False; last_ly=math.atan2(lss,lcs) if lf_n>0 else last_ly; lyr=True
    if rs:
        if not ri: ri=True; rcs=0.0; rss=0.0; rf_n=0
        ry=extract_heading(calibrate(rq,cal['Right']),r_axis)
        rcs+=math.cos(ry); rss+=math.sin(ry); rf_n+=1
    elif ri:
        ri=False; last_ry=math.atan2(rss,rcs) if rf_n>0 else last_ry; ryr=True
    if lyr and ryr:
        lyr=ryr=False; step_n+=1
        py=extract_heading(calibrate(pq,cal['Pelvis']),p_axis)
        tw=PELVIS_W+LEFT_W+RIGHT_W
        fused=math.atan2(
            (math.sin(py)*PELVIS_W+math.sin(last_ly)*LEFT_W+math.sin(last_ry)*RIGHT_W)/tw,
            (math.cos(py)*PELVIS_W+math.cos(last_ly)*LEFT_W+math.cos(last_ry)*RIGHT_W)/tw)
        pd_hist.append(fused); pd_cos+=math.cos(fused); pd_sin+=math.sin(fused)
        if len(pd_hist)>STAB_WINDOW:
            old=pd_hist.popleft(); pd_cos-=math.cos(old); pd_sin-=math.sin(old)
        cur_pd=fused; has_est=True
        stab=pd_stability()
        if ctx_now=='ReacquiringPd' and stab>=STAB_THR and step_n>=MIN_STEPS:
            confirm_straight()
    return cur_pd, has_est, pd_stability()

l_sampler=StanceSampler(); r_sampler=StanceSampler()

def process_fpa(lq,rq,ls,rs,ctx_now,conf,pd_dir,pd_valid,pd_stab):
    if ls: l_sampler.add_frame(extract_yaw_z(calibrate(lq,cal['Left'])))
    else:  l_sampler.mark_swing()
    if rs: r_sampler.add_frame(extract_yaw_z(calibrate(rq,cal['Right'])))
    else:  r_sampler.mark_swing()
    ctx_ok = ctx_now=='Straight' and conf>=CTX_CONF_THR
    pd_ok  = pd_valid and pd_stab>=PD_STAB_THR
    if not ctx_ok or not pd_ok: return None
    if not ls and not rs: return None
    fpa_l_rad=l_sampler.try_settle(MIN_SETTLE)
    fpa_r_rad=r_sampler.try_settle(MIN_SETTLE)
    if fpa_l_rad is None and fpa_r_rad is None: return None
    fpa_l=math.degrees(normalize_angle(fpa_l_rad-pd_dir)) if fpa_l_rad is not None else float('nan')
    fpa_r=math.degrees(normalize_angle(fpa_r_rad-pd_dir)) if fpa_r_rad is not None else float('nan')
    return (fpa_l, fpa_r)

# ── collect all FPA steps ─────────────────────────────────────────────────────
all_fpa = []
for pid in pids:
    if pid<=cal_pid: continue
    f=frames[pid]
    if not all(r in f for r in ['Pelvis','Left','Right']): continue
    ls_raw=is_stance_raw(f['Left'],cal['Left']); rs_raw=is_stance_raw(f['Right'],cal['Right'])
    l_cnt=l_cnt+1 if ls_raw else 0; r_cnt=r_cnt+1 if rs_raw else 0
    ls=l_cnt>=MIN_STANCE_F; rs=r_cnt>=MIN_STANCE_F
    if ls!=l_prev: l_trans+=1; l_prev=ls
    if rs!=r_prev: r_trans+=1; r_prev=rs
    walk_fc+=1
    if walk_fc>=WALK_WINDOW:
        is_walking=l_trans>=WALK_MIN_TRANS and r_trans>=WALK_MIN_TRANS
        walk_fc=0; l_trans=0; r_trans=0
    pf=f['Pelvis']
    ctx_now,conf=step_ctx(pf['q'],pf['g'],pf['dq'])
    pd_dir,pd_valid,pd_stab=step_pd(pf['q'],f['Left']['q'],f['Right']['q'],ls,rs,ctx_now)
    pd_is_valid=pd_valid and pd_stab>=STAB_THR
    result=process_fpa(f['Left']['q'],f['Right']['q'],ls,rs,ctx_now,conf,pd_dir,pd_is_valid,pd_stab)
    if result is not None:
        all_fpa.append(result)

# ── separate per-foot lists ───────────────────────────────────────────────────
l_steps = [(i, fpa_l) for i,(fpa_l,_) in enumerate(all_fpa) if not math.isnan(fpa_l)]
r_steps = [(i, fpa_r) for i,(_,fpa_r) in enumerate(all_fpa) if not math.isnan(fpa_r)]

if len(l_steps)<BASELINE_N+1 or len(r_steps)<BASELINE_N+1:
    print("Not enough steps"); sys.exit(1)

def stats(vals):
    n=len(vals); mu=sum(vals)/n
    sd=math.sqrt(sum((v-mu)**2 for v in vals)/(n-1)) if n>1 else 0.0
    return mu, sd

bl_L_vals = [v for _,v in l_steps[:BASELINE_N]]
bl_R_vals = [v for _,v in r_steps[:BASELINE_N]]
mu_L, sd_L = stats(bl_L_vals)
mu_R, sd_R = stats(bl_R_vals)

def compute_target(mu):
    return (mu-5.0,'ToeIn') if mu>10.0 else (mu+5.0,'ToeOut')

t_L, dir_L = compute_target(mu_L)
t_R, dir_R = compute_target(mu_R)

# new formula: max(4°, SD)
tol_L = max(4.0, sd_L)
tol_R = max(4.0, sd_R)

# ── print baseline summary ────────────────────────────────────────────────────
print(f"── Baseline (first {BASELINE_N} steps per foot) ──────────────────────")
print(f"  Left:   μ={mu_L:+.2f}°  SD={sd_L:.2f}°  → target={t_L:+.2f}° ({dir_L})  tol=±{tol_L:.2f}°")
print(f"  Right:  μ={mu_R:+.2f}°  SD={sd_R:.2f}°  → target={t_R:+.2f}° ({dir_R})  tol=±{tol_R:.2f}°")
print(f"  Tolerance formula: max(4°, SD)  [was: clamp(SD,6°,10°)]")
print()

# ── per-step training output ──────────────────────────────────────────────────
train_L = l_steps[BASELINE_N:]
train_R = r_steps[BASELINE_N:]

# build a merged timeline: step_index -> (fpa_l or nan, fpa_r or nan)
# so we can print pairs where both feet appear at same "output event"
merged = {}
for seq,(i,v) in enumerate(train_L):
    merged.setdefault(i,{})[' L'] = (seq+1, v)
for seq,(i,v) in enumerate(train_R):
    merged.setdefault(i,{})[' R'] = (seq+1, v)

GREEN = 'O'   # on-target
RED   = 'X'   # off-target
DASH  = '-'   # foot not emitted this step

def label(fpa, target, tol):
    return GREEN if abs(fpa-target)<=tol else RED

print(f"── Per-step training results ────────────────────────────────────────────")
print(f"  {'#':>4}  {'Left FPA':>9}  {'err_L':>7}  {'L':>2}  │  {'Right FPA':>9}  {'err_R':>7}  {'R':>2}")
print("  " + "─"*58)

l_green=l_red=r_green=r_red=0
for idx in sorted(merged.keys()):
    d = merged[idx]
    if ' L' in d:
        lseq, lv = d[' L']
        lerr = lv - t_L
        llbl = label(lv, t_L, tol_L)
        l_str = f"{lv:+8.2f}°  {lerr:+7.2f}°  {llbl}"
        if llbl==GREEN: l_green+=1
        else: l_red+=1
    else:
        l_str = f"{'':>9}  {'':>7}  {DASH}"

    if ' R' in d:
        rseq, rv = d[' R']
        rerr = rv - t_R
        rlbl = label(rv, t_R, tol_R)
        r_str = f"{rv:+8.2f}°  {rerr:+7.2f}°  {rlbl}"
        if rlbl==GREEN: r_green+=1
        else: r_red+=1
    else:
        r_str = f"{'':>9}  {'':>7}  {DASH}"

    print(f"  {idx:>4}  {l_str}  │  {r_str}")

# ── summary ───────────────────────────────────────────────────────────────────
l_total = l_green+l_red
r_total = r_green+r_red
print()
print(f"── Summary ──────────────────────────────────────────────────────────────")
print(f"  Left:   {l_green:>2} green / {l_red:>2} red  ({100*l_green/l_total:.1f}%)  tol=±{tol_L:.2f}°  target={t_L:+.2f}°")
print(f"  Right:  {r_green:>2} green / {r_red:>2} red  ({100*r_green/r_total:.1f}%)  tol=±{tol_R:.2f}°  target={t_R:+.2f}°")
print(f"  Total:  {l_green+r_green:>2} green / {l_red+r_red:>2} red  ({100*(l_green+r_green)/(l_total+r_total):.1f}%)")
