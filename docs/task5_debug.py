"""
Task 5 Debug: trace raw yaw values at each PD step to find post-turn FPA anomaly
"""
import sys, math, csv
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')

CSV_PATH = r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260608_161118.csv"

STATIC_GYRO_THRESHOLD = 0.3; STATIC_REQUIRED_FRAMES = 10; STATIC_COLLECT_FRAMES = 50
STOMP_ACC_THRESHOLD = 25.0; STOMP_MAX_FRAMES = 20
FREE_ACC_STANCE_THRESHOLD = 2.5; GYRO_STANCE_THRESHOLD = 1.0; FOOT_PITCH_THRESHOLD = 0.35
WALKING_WINDOW = 100; WALKING_MIN_TRANSITIONS = 2
YAW_RATE_THRESHOLD = 0.70; DELTA_Q_YAW_THRESHOLD = 0.17
TURNING_CONFIRM_FRAMES = 10; STRAIGHT_CONFIRM_FRAMES = 10; REACQ_TURNING_CONFIRM_FRAMES = 10
PELVIS_WEIGHT = 0.6; LEFT_WEIGHT = 0.2; RIGHT_WEIGHT = 0.2
STABILITY_WINDOW = 3; STABILITY_THRESHOLD = 0.65; MIN_STEPS_BEFORE_VALID = 2

def quat_mul(a, b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
def quat_inv(q):
    x,y,z,w=q; n2=x*x+y*y+z*z+w*w; return (-x/n2,-y/n2,-z/n2,w/n2)
def quat_calibrate(m, r): return quat_mul(quat_inv(r), m) if r else m
def vec_transform(v, q):
    vx,vy,vz=v; x,y,z,w=q
    rx=vx*(w*w+x*x-y*y-z*z)+vy*2*(x*y-w*z)+vz*2*(x*z+w*y)
    ry=vx*2*(x*y+w*z)+vy*(w*w-x*x+y*y-z*z)+vz*2*(y*z-w*x)
    rz=vx*2*(x*z-w*y)+vy*2*(y*z+w*x)+vz*(w*w-x*x-y*y+z*z)
    return (rx,ry,rz)
def extract_yaw(q):
    x,y,z,w=q; denom=1-2*(y*y+z*z)
    return math.atan2(2*(w*z+x*y), denom), denom
def extract_yaw_from_dq(dq): x,y,z,w=dq; return 2*math.atan2(z,w)
def normalize_angle(a):
    while a>math.pi: a-=2*math.pi
    while a<-math.pi: a+=2*math.pi
    return a
def median_quat(s):
    def med(v): s2=sorted(v); n=len(v); return (s2[n//2-1]+s2[n//2])/2 if n%2==0 else s2[n//2]
    comps=list(zip(*s)); mx,my,mz,mw=[med(list(c)) for c in comps]
    n=math.sqrt(mx*mx+my*my+mz*mz+mw*mw); return (mx/n,my/n,mz/n,mw/n)

frames = {}
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid=int(row['PacketId']); role=row['Role'].strip()
        def fv(k): return float(row[k])
        d={'q':(fv('Qx'),fv('Qy'),fv('Qz'),fv('Qw')),'g':(fv('Gx'),fv('Gy'),fv('Gz')),
           'fa':(fv('FreeAx'),fv('FreeAy'),fv('FreeAz')),'a':(fv('Ax'),fv('Ay'),fv('Az')),
           'dq':(fv('DQx'),fv('DQy'),fv('DQz'),fv('DQw')),'sw':int(row['StatusWord'])}
        if pid not in frames: frames[pid]={}
        frames[pid][role]=d

pids=sorted(frames.keys())

# Calibration
cal_profile=None; cal_state='WaitingForStomp'; stomp_active=False; stomp_frames=0
static_consecutive=0; collect_buf={'Pelvis':[],'Left':[],'Right':[]}; collect_count=0; cal_pid=None

for pid in pids:
    f=frames[pid]
    if 'Left' not in f: continue
    if cal_state=='WaitingForStomp':
        az=f['Left']['a'][2]
        if abs(az-9.81)>STOMP_ACC_THRESHOLD:
            if not stomp_active: stomp_active=True; stomp_frames=0
            stomp_frames+=1
            if stomp_frames>STOMP_MAX_FRAMES: stomp_active=False; stomp_frames=0
        elif stomp_active: cal_state='Collecting'; stomp_active=False; stomp_frames=0
    if cal_state=='Collecting' and all(r in f for r in ['Pelvis','Left','Right']):
        pg,lg,rg=[math.sqrt(sum(v**2 for v in f[r]['g'])) for r in ['Pelvis','Left','Right']]
        is_static=pg<STATIC_GYRO_THRESHOLD and lg<STATIC_GYRO_THRESHOLD and rg<STATIC_GYRO_THRESHOLD
        if is_static: static_consecutive+=1
        else: static_consecutive=0
        if static_consecutive>=STATIC_REQUIRED_FRAMES:
            collect_count+=1
            for rn in ['Pelvis','Left','Right']: collect_buf[rn].append(f[rn]['q'])
            if collect_count>=STATIC_COLLECT_FRAMES:
                cal_profile={'PelvisRef':median_quat(collect_buf['Pelvis']),
                             'LeftRef':median_quat(collect_buf['Left']),
                             'RightRef':median_quat(collect_buf['Right'])}
                cal_state='Done'; cal_pid=pid; break

py0,d0 = extract_yaw(cal_profile['PelvisRef'])
ly0,ld0 = extract_yaw(cal_profile['LeftRef'])
ry0,rd0 = extract_yaw(cal_profile['RightRef'])
print(f"PelvisRef: yaw={math.degrees(py0):.1f}°, denom={d0:.4f}")
print(f"LeftRef:   yaw={math.degrees(ly0):.1f}°, denom={ld0:.4f}")
print(f"RightRef:  yaw={math.degrees(ry0):.1f}°, denom={rd0:.4f}")

# Stance helper
def is_stance(sample, cal_ref):
    fa=sample['fa']; g=sample['g']; q=sample['q']
    if math.sqrt(sum(v**2 for v in fa))>=FREE_ACC_STANCE_THRESHOLD: return False
    if math.sqrt(sum(v**2 for v in g))>=GYRO_STANCE_THRESHOLD: return False
    if cal_ref:
        g_ref=vec_transform((0,0,-1),quat_inv(cal_ref)); g_now=vec_transform((0,0,-1),quat_inv(q))
        dot=max(-1.0,min(1.0,sum(a*b for a,b in zip(g_ref,g_now))))
        if math.acos(dot)>=FOOT_PITCH_THRESHOLD: return False
    return True

# Motion context
ctx_state='Straight'; turn_f=0; str_f=0; reacq_f=0; dq_accum=0.0
def step_context(pq,pg,pdq):
    global ctx_state,turn_f,str_f,reacq_f,dq_accum
    yr=abs(vec_transform(pg,pq)[2]); dq_accum+=extract_yaw_from_dq(pdq)
    sig=yr>YAW_RATE_THRESHOLD or abs(dq_accum)>DELTA_Q_YAW_THRESHOLD; conf=1.0
    if ctx_state=='Straight':
        if sig:
            turn_f+=1
            if turn_f>=TURNING_CONFIRM_FRAMES: ctx_state='Turning'; turn_f=0; dq_accum=0.0
            else: conf=1.0-turn_f/TURNING_CONFIRM_FRAMES
        else: turn_f=0; dq_accum=0.0
    elif ctx_state=='Turning':
        dq_accum=0.0
        if not sig:
            str_f+=1
            if str_f>=STRAIGHT_CONFIRM_FRAMES: ctx_state='ReacquiringPd'; str_f=0; reacq_f=0; conf=0.0
        else: str_f=0
        conf=0.0 if ctx_state!='Straight' else 1.0
    else:
        dq_accum=0.0; conf=0.0
        if sig:
            reacq_f+=1
            if reacq_f>=REACQ_TURNING_CONFIRM_FRAMES: ctx_state='Turning'; str_f=0; reacq_f=0
        else: reacq_f=0
    return ctx_state, conf
def confirm_straight():
    global ctx_state
    if ctx_state=='ReacquiringPd': ctx_state='Straight'

# PD estimator
pd_hist=deque(); pd_cos=0.0; pd_sin=0.0
cur_pd=0.0; pd_valid=False; step_n=0; prev_ctx='Straight'
li=False; ri=False; lcs=lss=rcs=rss=0.0; lf_n=rf_n=0
lyr=ryr=False; last_ly=last_ry=0.0

def compute_stability():
    n=len(pd_hist)
    if n<2: return 0.0
    r=math.sqrt(pd_cos**2+pd_sin**2)/n; return 1.0/(1.0+(1.0-r))
def update_pd_history(pd):
    global pd_cos,pd_sin
    pd_hist.append(pd); pd_cos+=math.cos(pd); pd_sin+=math.sin(pd)
    if len(pd_hist)>STABILITY_WINDOW:
        old=pd_hist.popleft(); pd_cos-=math.cos(old); pd_sin-=math.sin(old)

def step_pd_debug(pq,lq,rq,ls,rs,ctx):
    global cur_pd,pd_valid,step_n,prev_ctx
    global li,ri,lcs,lss,rcs,rss,lf_n,rf_n,lyr,ryr,last_ly,last_ry,pd_cos,pd_sin
    if ctx not in ('Straight','ReacquiringPd'):
        prev_ctx=ctx; return None
    if ctx=='ReacquiringPd' and prev_ctx=='Turning':
        pd_hist.clear(); pd_cos=0.0; pd_sin=0.0; step_n=0; pd_valid=False
        lyr=ryr=False; li=ri=False; lcs=lss=rcs=rss=0.0; lf_n=rf_n=0
    prev_ctx=ctx
    if ls:
        if not li: li=True; lcs=0.0; lss=0.0; lf_n=0
        ly,_=extract_yaw(quat_calibrate(lq, cal_profile['LeftRef']))
        lcs+=math.cos(ly); lss+=math.sin(ly); lf_n+=1
    elif li:
        li=False; last_ly=math.atan2(lss,lcs) if lf_n>0 else last_ly; lyr=True
    if rs:
        if not ri: ri=True; rcs=0.0; rss=0.0; rf_n=0
        ry,_=extract_yaw(quat_calibrate(rq, cal_profile['RightRef']))
        rcs+=math.cos(ry); rss+=math.sin(ry); rf_n+=1
    elif ri:
        ri=False; last_ry=math.atan2(rss,rcs) if rf_n>0 else last_ry; ryr=True
    if lyr and ryr:
        lyr=ryr=False; step_n+=1
        py,pdenom=extract_yaw(quat_calibrate(pq, cal_profile['PelvisRef']))
        tw=PELVIS_WEIGHT+LEFT_WEIGHT+RIGHT_WEIGHT
        wx=(math.cos(py)*PELVIS_WEIGHT+math.cos(last_ly)*LEFT_WEIGHT+math.cos(last_ry)*RIGHT_WEIGHT)/tw
        wy=(math.sin(py)*PELVIS_WEIGHT+math.sin(last_ly)*LEFT_WEIGHT+math.sin(last_ry)*RIGHT_WEIGHT)/tw
        fused=math.atan2(wy,wx)
        update_pd_history(fused); cur_pd=fused; pd_valid=True
        stab=compute_stability()
        if ctx=='ReacquiringPd' and stab>=STABILITY_THRESHOLD and step_n>=MIN_STEPS_BEFORE_VALID:
            confirm_straight()
        return {'step':step_n,'ctx':ctx,'pelvis_yaw':math.degrees(py),'pelvis_denom':pdenom,
                'left_yaw':math.degrees(last_ly),'right_yaw':math.degrees(last_ry),
                'fused':math.degrees(fused),'stab':stab}
    return None

# Walking tracker
l_hist=deque(maxlen=WALKING_WINDOW); r_hist=deque(maxlen=WALKING_WINDOW)
def trans_count(h): return sum(1 for a,b in zip(list(h),list(h)[1:]) if a!=b)

print(f"\n{'pid':>6} {'ctx':>12} {'step':>4} {'pelv_yaw':>9} {'left_yaw':>9} {'rght_yaw':>9} {'fused':>9} {'stab':>6}")
print("-"*75)

for pid in pids:
    if not cal_profile or pid<=cal_pid: continue
    f=frames[pid]
    if not all(r in f for r in ['Pelvis','Left','Right']): continue
    pf=f['Pelvis']; lf=f['Left']; rf=f['Right']
    ls=is_stance(lf, cal_profile['LeftRef']); rs=is_stance(rf, cal_profile['RightRef'])
    l_hist.append(ls); r_hist.append(rs)
    ctx,conf=step_context(pf['q'], pf['g'], pf['dq'])
    result=step_pd_debug(pf['q'], lf['q'], rf['q'], ls, rs, ctx)
    if result:
        r=result
        print(f"{pid:>6} {r['ctx']:>12} {r['step']:>4} {r['pelvis_yaw']:>+9.2f}° {r['left_yaw']:>+9.2f}° {r['right_yaw']:>+9.2f}° {r['fused']:>+9.2f}° {r['stab']:>6.3f}")
