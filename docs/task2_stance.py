"""
Task 2: Stance/Swing detection + IsWalking verification
Mirrors GaitEventDetector.cs exactly (gravity-vector pitch check, 5-frame debounce, 100-frame window)
"""
import sys, math, csv
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260609_201647.csv"

# ── constants (mirrors C# defaults) ──────────────────────────────────────────
STATIC_GYRO      = 0.3;  STATIC_REQ = 10;  STATIC_COLLECT = 50
STATIC_TIMEOUT   = 300;  STOMP_THRESH = 15.0;  STOMP_MAX = 20
XSF_ORIENT_VALID = 0x02; XSF_CLIPPING = 0x00080000

FREE_ACC_THR   = 2.5    # m/s²
GYRO_THR       = 1.0    # rad/s
PITCH_THR      = 0.35   # rad (~20°)
MIN_STANCE_F   = 5      # debounce frames
WALK_WINDOW    = 100    # frames
WALK_MIN_TRANS = 2      # per foot per window

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
def median_quat(s):
    def med(v): s2=sorted(v); n=len(v); return (s2[n//2-1]+s2[n//2])/2 if n%2==0 else s2[n//2]
    comps=list(zip(*s)); mx,my,mz,mw=[med(list(c)) for c in comps]
    n=math.sqrt(mx*mx+my*my+mz*mz+mw*mw); return (mx/n,my/n,mz/n,mw/n)
def gate_ok(f):
    if not all(r in f for r in ['Pelvis','Left','Right']): return False
    for r in ['Pelvis','Left','Right']:
        sw=f[r]['sw']
        if (sw & XSF_ORIENT_VALID)==0 or (sw & XSF_CLIPPING)!=0: return False
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
            'sw':int(row['StatusWord']),
        }
pids=sorted(frames.keys())

# ── calibration (same as Task 1) ─────────────────────────────────────────────
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
        if timeout_counter>STATIC_TIMEOUT: print("TIMEOUT"); break
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
print(f"Calibration complete: pid={cal_pid}")
print()

# ── is_stance (mirrors C# GaitEventDetector.IsStance) ────────────────────────
def is_stance(role, f, cal_ref):
    fa_mag = math.sqrt(sum(v**2 for v in f['fa']))
    g_mag  = math.sqrt(sum(v**2 for v in f['g']))
    if fa_mag >= FREE_ACC_THR or g_mag >= GYRO_THR:
        return False, fa_mag, g_mag, None
    if cal_ref is not None:
        g_ref = vec_transform((0,0,-1), quat_inv(cal_ref))
        g_now = vec_transform((0,0,-1), quat_inv(f['q']))
        dot   = max(-1.0, min(1.0, sum(a*b for a,b in zip(g_ref, g_now))))
        tilt  = math.acos(dot)
        if tilt >= PITCH_THR:
            return False, fa_mag, g_mag, tilt
        return True, fa_mag, g_mag, tilt
    return True, fa_mag, g_mag, None

# ── Task 2 main loop ──────────────────────────────────────────────────────────
l_cnt=0; r_cnt=0; l_prev=False; r_prev=False
l_trans=0; r_trans=0; walk_fc=0; is_walking=False
walking_first_pid=None; walk_window_idx=0

# collect first 20 stance-event rows + walking trigger for display
events=[]
for pid in pids:
    if pid <= cal_pid: continue
    f=frames[pid]
    if not all(r in f for r in ['Left','Right']): continue

    ls_raw, lfa, lg, lt = is_stance('Left',  f['Left'],  cal['Left'])
    rs_raw, rfa, rg, rt = is_stance('Right', f['Right'], cal['Right'])

    l_cnt = l_cnt+1 if ls_raw else 0
    r_cnt = r_cnt+1 if rs_raw else 0
    ls = l_cnt >= MIN_STANCE_F
    rs = r_cnt >= MIN_STANCE_F

    if ls != l_prev: l_trans+=1; l_prev=ls
    if rs != r_prev: r_trans+=1; r_prev=rs

    walk_fc+=1
    if walk_fc >= WALK_WINDOW:
        prev_walking = is_walking
        is_walking = l_trans>=WALK_MIN_TRANS and r_trans>=WALK_MIN_TRANS
        if is_walking and not prev_walking and walking_first_pid is None:
            walking_first_pid = pid
        walk_window_idx+=1
        walk_fc=0; l_trans=0; r_trans=0

    transition = (ls != (l_cnt==MIN_STANCE_F and ls)) or True  # always log
    events.append((pid, ls, rs, is_walking,
                   lfa, lg, lt if lt is not None else float('nan'),
                   rfa, rg, rt if rt is not None else float('nan')))

# ── output ────────────────────────────────────────────────────────────────────
print(f"{'pid':>7}  {'LS':>3} {'RS':>3} {'Walk':>5}  {'lFA':>5} {'lGy':>5} {'lTlt°':>6}  {'rFA':>5} {'rGy':>5} {'rTlt°':>6}")
print("-"*75)

# print rows where stance changes + first 5 frames of each window
prev_ls=prev_rs=prev_wk=None
shown=0
for (pid,ls,rs,wk,lfa,lg,lt,rfa,rg,rt) in events:
    if ls!=prev_ls or rs!=prev_rs or wk!=prev_wk or shown<5:
        ltd=math.degrees(lt) if not math.isnan(lt) else float('nan')
        rtd=math.degrees(rt) if not math.isnan(rt) else float('nan')
        print(f"{pid:>7}  {'T' if ls else 'F':>3} {'T' if rs else 'F':>3} {'T' if wk else 'F':>5}  "
              f"{lfa:>5.2f} {lg:>5.2f} {ltd:>6.1f}  {rfa:>5.2f} {rg:>5.2f} {rtd:>6.1f}")
        prev_ls=ls; prev_rs=rs; prev_wk=wk; shown+=1
    if shown>=40: break

print()
if walking_first_pid:
    print(f"IsWalking -> True at pid={walking_first_pid}  ({walking_first_pid-cal_pid} frames after calibration)")
else:
    print("IsWalking never became True in this recording")
