"""
Layer B2: Axis detection + heading tracking across turns
Checks: after calibration, does each sensor's heading correctly follow 180° turns?
"""
import sys, math, csv
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260608_161118.csv"

STATIC_GYRO      = 0.3
STATIC_REQ       = 10
STATIC_COLLECT   = 50
STATIC_TIMEOUT   = 300          # C# StaticTimeoutFrames — counted in ValidFrames
STOMP_THRESH     = 15.0
STOMP_MAX        = 20
YAW_RATE_THR     = 0.70
XSF_ORIENT_VALID = 0x02
XSF_CLIPPING     = 0x00080000

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

def detect_axis(ref):
    gx,gy,gz=vec_transform((0,0,-1), quat_inv(ref))
    ax,ay,az=abs(gx),abs(gy),abs(gz)
    if ax>=ay and ax>=az: return 0, gx
    if ay>=ax and ay>=az: return 1, gy
    return 2, gz

def extract_heading(q, axis):
    x,y,z,w=q
    if axis==0: return math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    if axis==1: return math.asin(max(-1.0,min(1.0, 2*(w*y-z*x))))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

# ── Load ───────────────────────────────────────────────────────────────────────
frames={}
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid=int(row['PacketId']); role=row['Role'].strip()
        def fv(k): return float(row[k])
        frames.setdefault(pid,{})[role]={
            'q':(fv('Qx'),fv('Qy'),fv('Qz'),fv('Qw')),
            'g':(fv('Gx'),fv('Gy'),fv('Gz')),
            'a':(fv('Ax'),fv('Ay'),fv('Az')),
            'sw':int(row['StatusWord']),
        }
pids=sorted(frames.keys())

# ── Calibration ────────────────────────────────────────────────────────────────
def gate_ok(f):
    """Mirrors DataQualityGate: all 3 sensors present, StatusWord passes."""
    if not all(r in f for r in ['Pelvis','Left','Right']): return False
    for r in ['Pelvis','Left','Right']:
        sw=f[r]['sw']
        if (sw & XSF_ORIENT_VALID)==0 or (sw & XSF_CLIPPING)!=0: return False
    return True

cal=None; state='Wait'; static_n=0
stomp_peak=False; stomp_fc=0          # mirrors C# _stompPeakSeen / _stompFrameCount
buf={'Pelvis':[],'Left':[],'Right':[]}; cnt=0; cal_pid=None
timeout_counter=0                     # mirrors C# _staticTimeoutCounter (ValidFrame count)
for pid in pids:
    f=frames[pid]
    if 'Left' not in f: continue
    if state=='Wait':
        va = abs(f['Left']['a'][2]-9.81)
        if not stomp_peak:
            if va > STOMP_THRESH:
                stomp_peak=True; stomp_fc=1
        else:
            stomp_fc+=1
            if va < STOMP_THRESH*0.4:             # 40% hysteresis — mirrors C# line 75
                if stomp_fc <= STOMP_MAX:
                    state='Collect'
                stomp_peak=False; stomp_fc=0
            elif stomp_fc > STOMP_MAX:
                stomp_peak=False; stomp_fc=0
    if state=='Collect' and gate_ok(f):
        timeout_counter+=1
        if timeout_counter > STATIC_TIMEOUT:
            print(f"  TIMEOUT: no static window in {STATIC_TIMEOUT} valid frames after stomp")
            break
        pg,lg,rg=[math.sqrt(sum(v**2 for v in f[r]['g'])) for r in ['Pelvis','Left','Right']]
        ok=pg<STATIC_GYRO and lg<STATIC_GYRO and rg<STATIC_GYRO
        static_n = static_n+1 if ok else 0
        if static_n>=STATIC_REQ:
            cnt+=1
            for r in ['Pelvis','Left','Right']: buf[r].append(f[r]['q'])
            if cnt>=STATIC_COLLECT:
                cal={r:median_quat(buf[r]) for r in ['Pelvis','Left','Right']}
                cal_pid=pid; break

print(f"Calibration pid={cal_pid}")
if cal is None:
    if timeout_counter > STATIC_TIMEOUT:
        print(f"  FAIL: calibration timed out after {STATIC_TIMEOUT} valid frames")
        print("  Cause: person walked immediately after stomp (no post-stomp static standing)")
    else:
        max_stomp = max(
            (abs(frames[pid]['Left']['a'][2] - 9.81), pid)
            for pid in pids if 'Left' in frames[pid]
        )
        print(f"  FAIL: no stomp detected. Max |Az-9.81| = {max_stomp[0]:.2f} at pid={max_stomp[1]} (threshold={STOMP_THRESH})")
    sys.exit(1)
print()

# ── Axis detection ─────────────────────────────────────────────────────────────
roles = [('Pelvis','PelvisRef'), ('Left','LeftRef'), ('Right','RightRef')]
axis_map={0:'X → Roll',1:'Y → Pitch',2:'Z → Yaw'}
axes={}
print(f"{'Sensor':<8} {'Axis':>14}  {'g_dominant':>10}  {'g_vector (sensor frame)'}")
print("-"*62)
for name, _ in roles:
    ref=cal[name]
    ax, gdom = detect_axis(ref)
    gvec=vec_transform((0,0,-1), quat_inv(ref))
    axes[name]=ax
    print(f"{name:<8} {axis_map[ax]:>14}  {gdom:>+10.4f}  ({gvec[0]:+.3f}, {gvec[1]:+.3f}, {gvec[2]:+.3f})")

# ── Turn detection (yaw rate on pelvis) ───────────────────────────────────────
turn_pids=[]
in_turn=False; turn_start=0
for pid in pids:
    if not cal or pid<=cal_pid: continue
    f=frames[pid]
    if 'Pelvis' not in f: continue
    pq=f['Pelvis']['q']; pg=f['Pelvis']['g']
    yr=abs(vec_transform(pg,pq)[2])
    if yr>YAW_RATE_THR and not in_turn:
        in_turn=True; turn_start=pid
    elif yr<=YAW_RATE_THR and in_turn:
        in_turn=False
        if pid-turn_start >= 20:          # ignore transient gait peaks (<20 frames)
            turn_pids.append((turn_start,pid))

print(f"\nTurns detected (yaw rate > {YAW_RATE_THR} rad/s, duration >= 20 frames):")
for i,(ts,te) in enumerate(turn_pids):
    print(f"  Turn {i+1}: pid {ts}–{te}")

# ── Phase windows ──────────────────────────────────────────────────────────────
if len(turn_pids)>=2:
    t1s,t1e = turn_pids[0]
    t2s,t2e = turn_pids[1]
    phases = [
        ("Pre-turn",    cal_pid+1, t1s-1),
        ("Post-Turn-1", t1e+1,     t2s-1),
        ("Post-Turn-2", t2e+1,     pids[-1]),
    ]
else:
    phases=[]
    print("Not enough turns for phase analysis")

# ── Per-sensor heading per phase ──────────────────────────────────────────────
print()
for name, _ in roles:
    ref=cal[name]; ax=axes[name]
    print(f"── {name} (axis={axis_map[ax]}) ──────────────────────────────")
    for phase_name, p_start, p_end in phases:
        headings=[]
        for pid in pids:
            if pid<p_start or pid>p_end: continue
            f=frames[pid]
            if name not in f: continue
            q=f[name]['q']
            h=math.degrees(extract_heading(calibrate(q,ref), ax))
            headings.append(h)
        if not headings: print(f"  {phase_name}: no data"); continue
        # circular mean
        cx=sum(math.cos(math.radians(h)) for h in headings)/len(headings)
        cy=sum(math.sin(math.radians(h)) for h in headings)/len(headings)
        mean_h=math.degrees(math.atan2(cy,cx))
        std_h=math.degrees(math.sqrt(max(0, -2*math.log(math.sqrt(cx*cx+cy*cy)+1e-12))))
        print(f"  {phase_name:<14}: mean={mean_h:>+7.1f}°  std≈{std_h:>5.1f}°  n={len(headings)}")
    print()
