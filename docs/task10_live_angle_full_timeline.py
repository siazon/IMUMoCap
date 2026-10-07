"""
Task 10: full-timeline dump of the live-angle pipeline (post TwistX pelvis fix) over the
WHOLE recording, to explain why the AR footprint reportedly still points "right" after a
U-turn, goes back to correct after turning back to the calibration orientation, and goes
wrong again after another U-turn (i.e. an error that alternates with parity of 180 deg from
calibration, not a one-off transient).

Reuses the calibration + quaternion math from task9_live_angle_turn_repro.py, but instead of
looking at only the single biggest turn window, it:
  1. Builds the continuous UNWRAPPED pelvis heading (TwistX-based) for the whole walk.
  2. Segments the walk into "plateaus" (near-constant heading) separated by fast-turn events.
  3. For each plateau, reports the settled angleL/angleR (fixed formula) AND a "swing" metric
     -- how far the pelvis's local +X (the assumed up axis) has tilted away from its
     calibration-time direction -- to check whether the pure-yaw assumption behind TwistX
     actually holds during/after each turn, or whether real lean is corrupting the twist.
"""
import sys, csv, math
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260922_121955.csv"
FS_HZ = 100.0

STATIC_GYRO_THR, STATIC_REQUIRED_F, STATIC_COLLECT_F, STATIC_TIMEOUT_F = 0.3, 30, 300, 1000

def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    cx = ay * bz - az * by; cy = az * bx - ax * bz; cz = ax * by - ay * bx
    dot = ax * bx + ay * by + az * bz
    return (ax*bw+bx*aw+cx, ay*bw+by*aw+cy, az*bw+bz*aw+cz, aw*bw-dot)

def q_inv(q):
    x, y, z, w = q; n2 = x*x+y*y+z*z+w*w
    return (-x/n2, -y/n2, -z/n2, w/n2)

def q_norm(q):
    x, y, z, w = q; n = math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n, y/n, z/n, w/n)

def relative_quat(now, ref):
    return q_mul(q_inv(ref), now)

def twist_x(q):
    x, _, _, w = q
    return 2.0 * math.atan2(x, w)

def twist_z(q):
    _, _, z, w = q
    return 2.0 * math.atan2(z, w)

# Standard aerospace Euler yaw about a WORLD-fixed Z axis, computed directly on a
# sensor-to-global quaternion (NOT a calibration-relative composition). Robust to any
# amount of pitch/roll tilt (short of +-90 deg gimbal lock), unlike twist about a
# body-fixed axis established once at calibration -- tests whether Qx,Qy,Qz,Qw is
# already a proper world/global-referenced AHRS orientation.
def yaw_world_z(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def vec3_transform(v, q):
    x, y, z, w = q; vx, vy, vz = v
    x2, y2, z2 = x+x, y+y, z+z
    wx2, wy2, wz2 = w*x2, w*y2, w*z2
    xx2, xy2, xz2 = x*x2, x*y2, x*z2
    yy2, yz2, zz2 = y*y2, y*z2, z*z2
    return (vx*(1-yy2-zz2)+vy*(xy2-wz2)+vz*(xz2+wy2),
            vx*(xy2+wz2)+vy*(1-xx2-zz2)+vz*(yz2-wx2),
            vx*(xz2-wy2)+vy*(yz2+wx2)+vz*(1-xx2-yy2))

def norm_deg(deg):
    while deg > 180.0: deg -= 360.0
    while deg < -180.0: deg += 360.0
    return deg

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
        q = (float(row['Qx']), float(row['Qy']), float(row['Qz']), float(row['Qw']))
        g = (float(row['Gx']), float(row['Gy']), float(row['Gz']))
        frames[pid][role] = (q, g)

pids = sorted(p for p, d in frames.items() if len(d) == 3)

static_consecutive = timeout_counter = 0
pelvis_buf, left_buf, right_buf = [], [], []
calib_idx = None
for i, pid in enumerate(pids):
    timeout_counter += 1
    if timeout_counter > STATIC_TIMEOUT_F:
        break
    pel_q, pel_g = frames[pid]['Pelvis']; l_q, l_g = frames[pid]['Left']; r_q, r_g = frames[pid]['Right']
    is_static = (sum(v*v for v in pel_g) < STATIC_GYRO_THR**2 and
                 sum(v*v for v in l_g) < STATIC_GYRO_THR**2 and
                 sum(v*v for v in r_g) < STATIC_GYRO_THR**2)
    if is_static:
        static_consecutive += 1
        if static_consecutive >= STATIC_REQUIRED_F:
            pelvis_buf.append(pel_q); left_buf.append(l_q); right_buf.append(r_q)
            if len(pelvis_buf) >= STATIC_COLLECT_F:
                calib_idx = i
                break
    else:
        static_consecutive = 0

pelvis_ref = median_quaternion(pelvis_buf)
left_ref = median_quaternion(left_buf)
right_ref = median_quaternion(right_buf)
calib_pid = pids[calib_idx]
print(f"Calibration at pid={calib_pid} (t={(calib_pid-pids[0])/FS_HZ:.2f}s)\n")

# ── walk the rest, unwrap pelvis heading, compute swing (X-axis tilt from calibration) ──
ts, pel_hdg_raw, pel_hdg_unwrap, swing_deg = [], [], [], []
angleL_fixed, angleR_fixed = [], []
angleL_world, angleR_world = [], []
prev_raw = None
unwrap_offset = 0.0

pelvis_ref_yaw_world = yaw_world_z(pelvis_ref)
left_ref_yaw_world = yaw_world_z(left_ref)
right_ref_yaw_world = yaw_world_z(right_ref)

for i in range(calib_idx, len(pids)):
    pid = pids[i]
    pel_q, _ = frames[pid]['Pelvis']; l_q, _ = frames[pid]['Left']; r_q, _ = frames[pid]['Right']
    rel = relative_quat(pel_q, pelvis_ref)
    raw = math.degrees(twist_x(rel))
    if prev_raw is not None:
        d = raw - prev_raw
        if d > 180: unwrap_offset -= 360
        elif d < -180: unwrap_offset += 360
    prev_raw = raw
    unwrapped = raw + unwrap_offset

    # swing: how far local +X has tilted away from its calibration direction
    xr, _, _ = vec3_transform((1.0, 0.0, 0.0), rel)
    swing = math.degrees(math.acos(max(-1.0, min(1.0, xr))))

    l_naive = twist_z(relative_quat(l_q, left_ref))
    r_naive = twist_z(relative_quat(r_q, right_ref))
    pel_twist = twist_x(rel)
    angleL_fixed.append(norm_deg(math.degrees(l_naive) - math.degrees(pel_twist)))
    angleR_fixed.append(norm_deg(math.degrees(r_naive) - math.degrees(pel_twist)))

    # world-frame-yaw variant: relative-to-calibration yaw computed independently per
    # sensor via the world-Z Euler formula, no shared local-axis assumption at all.
    pel_yw = yaw_world_z(pel_q) - pelvis_ref_yaw_world
    l_yw = yaw_world_z(l_q) - left_ref_yaw_world
    r_yw = yaw_world_z(r_q) - right_ref_yaw_world
    angleL_world.append(norm_deg(math.degrees(l_yw - pel_yw)))
    angleR_world.append(norm_deg(math.degrees(r_yw - pel_yw)))

    ts.append((pid - calib_pid) / FS_HZ)
    pel_hdg_raw.append(raw)
    pel_hdg_unwrap.append(unwrapped)
    swing_deg.append(swing)

n = len(ts)
print(f"{n} frames, {ts[-1]:.1f}s post-calibration\n")

# ── print every 0.5s: unwrapped pelvis heading, swing, angleL/R(fixed) vs angleL/R(world) ──
print(f"{'t(s)':>7} {'pelvis_unwrap':>13} {'swing_deg':>10} {'angleL_fixed':>13} {'angleR_fixed':>13} {'angleL_world':>13} {'angleR_world':>13}")
step = int(0.5 * FS_HZ)
for i in range(0, n, step):
    print(f"{ts[i]:7.2f} {pel_hdg_unwrap[i]:13.1f} {swing_deg[i]:10.1f} {angleL_fixed[i]:13.1f} {angleR_fixed[i]:13.1f} {angleL_world[i]:13.1f} {angleR_world[i]:13.1f}")
