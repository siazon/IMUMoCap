"""
Task 9: reproduce the WS "live" angle stream (MainWindow.xaml.cs RawFootYawDeg / TwistZ)
on a recorded walk that includes a U-turn, to find why the AR footprint icon rotates
"neatly to the right" instead of pointing forward again after the participant turns around.

Mirrors, frame-for-frame, the C# path that actually feeds the AR "live" message
(angleL/angleR in MainWindow.xaml.cs):
    CalibrationProcessor.ProcessCollecting  -> CalibrationProfile (PelvisRef/LeftFootRef/RightFootRef)
    RawFootYawDeg = TwistZ(Inverse(footRef)*footNow) - TwistZ(Inverse(pelvisRef)*pelvisNow)
    TwistZ(q) = 2*atan2(q.z, q.w)                      # swing-twist about the SENSOR'S LOCAL Z axis

For comparison, also computes the axis-aware heading used by ProgressionDirEstimator
(DetectHeadingAxis + ExtractHeading), which auto-detects which local sensor axis is
actually vertical (aligned with gravity) per sensor, instead of assuming Z.
If a sensor's detected heading axis != Z, RawFootYawDeg's fixed-Z assumption is wrong,
and a swing-twist decomposition about the wrong axis is known to misbehave (jump in
roughly 90 deg steps) exactly for LARGE rotations like a U-turn -- which would explain
"steps forward render as a clean turn to the right" instead of a continuous ~0 heading.
"""
import sys, csv, math
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\SourceCode\IMUMoCap\docs\ImuSamples_20260922_121955.csv"
FS_HZ = 100.0

# ── CalibrationProcessor constants ──────────────────────────────────────────────
STATIC_GYRO_THR   = 0.3   # rad/s
STATIC_REQUIRED_F = 30
STATIC_COLLECT_F  = 300
STATIC_TIMEOUT_F  = 1000

# ── quaternion helpers (mirror System.Numerics.Quaternion / Vector3.Transform) ──
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    cx = ay * bz - az * by
    cy = az * bx - ax * bz
    cz = ax * by - ay * bx
    dot = ax * bx + ay * by + az * bz
    return (ax * bw + bx * aw + cx,
            ay * bw + by * aw + cy,
            az * bw + bz * aw + cz,
            aw * bw - dot)

def q_inv(q):
    x, y, z, w = q
    n2 = x * x + y * y + z * z + w * w
    return (-x / n2, -y / n2, -z / n2, w / n2)

def q_norm(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)

def relative_quat(now, ref):
    return q_mul(q_inv(ref), now)

def twist_z(q):
    _, _, z, w = q
    return 2.0 * math.atan2(z, w)

def twist_x(q):
    x, _, _, w = q
    return 2.0 * math.atan2(x, w)

def vec3_transform(v, q):
    x, y, z, w = q
    vx, vy, vz = v
    x2, y2, z2 = x + x, y + y, z + z
    wx2, wy2, wz2 = w * x2, w * y2, w * z2
    xx2, xy2, xz2 = x * x2, x * y2, x * z2
    yy2, yz2, zz2 = y * y2, y * z2, z * z2
    return (vx * (1 - yy2 - zz2) + vy * (xy2 - wz2) + vz * (xz2 + wy2),
            vx * (xy2 + wz2) + vy * (1 - xx2 - zz2) + vz * (yz2 - wx2),
            vx * (xz2 - wy2) + vy * (yz2 + wx2) + vz * (1 - xx2 - yy2))

def detect_heading_axis(sensor_ref, label=None):
    gx, gy, gz = vec3_transform((0.0, 0.0, -1.0), q_inv(sensor_ref))
    if label:
        print(f"  {label}: down-vector in sensor frame = ({gx:.3f}, {gy:.3f}, {gz:.3f})")
    ax, ay, az = abs(gx), abs(gy), abs(gz)
    if ax >= ay and ax >= az:
        return 0
    if ay >= ax and ay >= az:
        return 1
    return 2

def extract_heading(q, axis):
    x, y, z, w = q
    if axis == 0:
        return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    if axis == 1:
        return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def norm_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg

def median_quaternion(qs):
    xs, ys, zs, ws = [], [], [], []
    for (x, y, z, w) in qs:
        if w < 0:
            x, y, z, w = -x, -y, -z, -w
        xs.append(x); ys.append(y); zs.append(z); ws.append(w)
    xs.sort(); ys.sort(); zs.sort(); ws.sort()
    mid = len(qs) // 2
    return q_norm((xs[mid], ys[mid], zs[mid], ws[mid]))

# ── load CSV, group by PacketId ─────────────────────────────────────────────────
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
print(f"Loaded {len(pids)} complete (Pelvis+Left+Right) frames, "
      f"{(pids[-1] - pids[0]) / FS_HZ:.1f}s @ {FS_HZ:.0f}Hz\n")

# ── replicate CalibrationProcessor: trigger assumed at the very first frame ─────
static_consecutive = 0
timeout_counter = 0
pelvis_buf, left_buf, right_buf = [], [], []
calib_idx = None
for i, pid in enumerate(pids):
    timeout_counter += 1
    if timeout_counter > STATIC_TIMEOUT_F:
        print(f"Calibration FAILED to complete (timeout) starting from pid={pids[0]}")
        break
    pel_q, pel_g = frames[pid]['Pelvis']
    l_q, l_g = frames[pid]['Left']
    r_q, r_g = frames[pid]['Right']
    is_static = (sum(v * v for v in pel_g) < STATIC_GYRO_THR ** 2
                 and sum(v * v for v in l_g) < STATIC_GYRO_THR ** 2
                 and sum(v * v for v in r_g) < STATIC_GYRO_THR ** 2)
    if is_static:
        static_consecutive += 1
        if static_consecutive >= STATIC_REQUIRED_F:
            pelvis_buf.append(pel_q); left_buf.append(l_q); right_buf.append(r_q)
            if len(pelvis_buf) >= STATIC_COLLECT_F:
                calib_idx = i
                break
    else:
        static_consecutive = 0

if calib_idx is None:
    print("No static calibration window found at the start of this recording — aborting.")
    sys.exit(1)

pelvis_ref = median_quaternion(pelvis_buf)
left_ref = median_quaternion(left_buf)
right_ref = median_quaternion(right_buf)
calib_pid = pids[calib_idx]
print(f"Calibration completed at pid={calib_pid} (t={(calib_pid - pids[0]) / FS_HZ:.2f}s, frame #{calib_idx})\n")

print("Auto-detected heading axis per sensor (ProgressionDirEstimator.DetectHeadingAxis):")
pelvis_axis = detect_heading_axis(pelvis_ref, "Pelvis")
left_axis = detect_heading_axis(left_ref, "Left")
right_axis = detect_heading_axis(right_ref, "Right")
axis_name = {0: 'X(Roll)', 1: 'Y(Pitch)', 2: 'Z(Yaw)'}
print(f"  Pelvis: {axis_name[pelvis_axis]}   Left: {axis_name[left_axis]}   Right: {axis_name[right_axis]}")
print("  (RawFootYawDeg/TwistZ used by the AR 'live' stream ALWAYS assumes Z, regardless of this.)\n")

# ── walk through the rest of the recording, compute both versions ──────────────
naive_L, naive_R = [], []          # current program: TwistZ-about-Z (RawFootYawDeg)
fixed_L, fixed_R = [], []          # proposed fix: pelvis TwistX, feet TwistZ (hardcoded mounting)
corrected_L, corrected_R = [], []  # axis-aware equivalent (ProgressionDirEstimator-style)
pelvis_naive_hdg, pelvis_fixed_hdg, pelvis_corrected_hdg = [], [], []
ts = []

for i in range(calib_idx, len(pids)):
    pid = pids[i]
    pel_q, _ = frames[pid]['Pelvis']
    l_q, _ = frames[pid]['Left']
    r_q, _ = frames[pid]['Right']

    pel_naive = twist_z(relative_quat(pel_q, pelvis_ref))
    l_naive = twist_z(relative_quat(l_q, left_ref))
    r_naive = twist_z(relative_quat(r_q, right_ref))
    naive_L.append(norm_deg(math.degrees(l_naive - pel_naive)))
    naive_R.append(norm_deg(math.degrees(r_naive - pel_naive)))
    pelvis_naive_hdg.append(math.degrees(pel_naive))

    pel_fixed = twist_x(relative_quat(pel_q, pelvis_ref))
    fixed_L.append(norm_deg(math.degrees(l_naive - pel_fixed)))
    fixed_R.append(norm_deg(math.degrees(r_naive - pel_fixed)))
    pelvis_fixed_hdg.append(math.degrees(pel_fixed))

    pel_corr = extract_heading(relative_quat(pel_q, pelvis_ref), pelvis_axis)
    l_corr = extract_heading(relative_quat(l_q, left_ref), left_axis)
    r_corr = extract_heading(relative_quat(r_q, right_ref), right_axis)
    corrected_L.append(norm_deg(math.degrees(l_corr - pel_corr)))
    corrected_R.append(norm_deg(math.degrees(r_corr - pel_corr)))
    pelvis_corrected_hdg.append(math.degrees(pel_corr))

    ts.append((pid - calib_pid) / FS_HZ)

n = len(ts)
print(f"Post-calibration walk: {n} frames, {ts[-1]:.1f}s\n")

# ── locate the U-turn: window where the axis-aware pelvis heading changes fast ──
WIN = int(2.0 * FS_HZ)  # 2s window
best_i, best_delta = None, 0.0
for i in range(n - WIN):
    d = abs(norm_deg(pelvis_corrected_hdg[i + WIN] - pelvis_corrected_hdg[i]))
    if d > best_delta:
        best_delta = d
        best_i = i

if best_i is None or best_delta < 90.0:
    print("No clear >=90 deg pelvis turn found via the axis-aware heading; "
          "showing whole-recording summary instead.")
    lo, hi = 0, n - 1
else:
    lo = max(0, best_i - 50)
    hi = min(n - 1, best_i + WIN + 50)
    print(f"Largest turn found: {best_delta:.1f} deg over 2s, "
          f"around t={ts[best_i]:.1f}s..{ts[best_i+WIN]:.1f}s\n")

def show(label, i):
    print(f"  t={ts[i]:6.2f}s  pelvis(naive)={pelvis_naive_hdg[i]:7.1f}  pelvis(fixed)={pelvis_fixed_hdg[i]:7.1f}  pelvis(corr)={pelvis_corrected_hdg[i]:7.1f}"
          f"   |  angleL(naive)={naive_L[i]:7.1f}  angleL(fixed)={fixed_L[i]:7.1f}  angleL(corr)={corrected_L[i]:7.1f}")

print("Before turn:")
show("before", lo)
print("At/after turn (sampled every ~0.5s through the window):")
step = max(1, (hi - lo) // 12)
for i in range(lo, hi + 1, step):
    show("", i)
print("After turn (settled):")
show("after", hi)

# ── overall stats: how far does the naive 'live' angle drift from 0 vs corrected ─
def stats(vals):
    lo_v, hi_v = min(vals), max(vals)
    return lo_v, hi_v

nl_lo, nl_hi = stats(naive_L[hi-100:hi]) if hi > 100 else stats(naive_L)
cl_lo, cl_hi = stats(corrected_L[hi-100:hi]) if hi > 100 else stats(corrected_L)
print(f"\nLast 100 post-turn frames, live angleL(naive) range: [{nl_lo:.1f}, {nl_hi:.1f}]  "
      f"(should stay near 0 if still walking straight)")
print(f"Last 100 post-turn frames, angleL(corr)  range: [{cl_lo:.1f}, {cl_hi:.1f}]")

# ── final N seconds of the whole recording: is the signal settled near a constant? ─
def mean_std(vals):
    m = sum(vals) / len(vals)
    v = sum((x - m) ** 2 for x in vals) / len(vals)
    return m, math.sqrt(v)

TAIL_SEC = 5.0
tail_n = int(TAIL_SEC * FS_HZ)
if n > tail_n:
    print(f"\nFinal {TAIL_SEC:.0f}s of the recording (t={ts[-tail_n]:.1f}s..{ts[-1]:.1f}s):")
    for label, series in (("pelvis(naive)", pelvis_naive_hdg), ("pelvis(fixed)", pelvis_fixed_hdg), ("pelvis(corr)", pelvis_corrected_hdg),
                           ("angleL(naive)", naive_L), ("angleR(naive)", naive_R),
                           ("angleL(fixed)", fixed_L), ("angleR(fixed)", fixed_R),
                           ("angleL(corr)", corrected_L), ("angleR(corr)", corrected_R)):
        m, s = mean_std(series[-tail_n:])
        print(f"  {label:15s} mean={m:8.1f}  std={s:6.1f}")
