// IMUMoCap/Pipeline/MountingMonitor.cs
using System;
using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    public enum MountingLevel { Ok, Warn, Pause }

    /// <summary>A sensor's tilt moved away from its calibration tilt (role, degrees, how serious).</summary>
    public sealed record MountingAlert(ImuRole Role, float ShiftDeg, MountingLevel Level);

    /// <summary>
    /// IMU 佩戴检查,只用重力方向(俯仰/侧倾),看不到绕竖直轴的转动:
    /// - 校准时:Pelvis 的 X 轴、两脚的 Z 轴要朝上(录制数据里正常佩戴分别是 2–15° 和 17–32°)。
    /// - 实验中:行走时(脚只取静止帧)各传感器朝上方向的 5 s 滑动平均与校准时相比,偏离 &gt; 12° 警告、&gt; 20° 暂停;
    ///   每个级别每次校准后只报一次。正常佩戴的回放(Pl89 / Po9 / Ps2)里偏离多在 2–7°,偶尔到 10° 出头(所以警告用 12° 而不是 10°),
    ///   从未到 20°;2 s 窗口时转身等瞬时动作会冲到 13–17°,所以用 5 s。见 MountingReplayTests。
    /// </summary>
    public sealed class MountingMonitor
    {
        public const float PelvisMaxTiltDeg = 30f;   // pelvis X axis from vertical, at calibration
        public const float FootMaxTiltDeg   = 45f;   // foot Z axis from vertical, at calibration
        public const float WarnShiftDeg     = 12f;
        public const float PauseShiftDeg    = 20f;
        public const int   WindowFrames     = 500;   // contributing frames (5 s @ 100 Hz)

        private sealed class Track
        {
            public Vector3 Reference;
            public readonly Queue<Vector3> Window = new();
            public Vector3 Sum;
            public MountingLevel Reported;
        }

        private readonly Dictionary<ImuRole, Track> _tracks = new();

        /// <summary>World up (Z) expressed in the sensor frame; q is the sensor-to-world orientation.</summary>
        public static Vector3 UpInSensor(Quaternion q) => Vector3.Transform(Vector3.UnitZ, Quaternion.Conjugate(q));

        private static float AngleDeg(Vector3 a, Vector3 b)
            => MathF.Acos(Math.Clamp(Vector3.Dot(Vector3.Normalize(a), Vector3.Normalize(b)), -1f, 1f)) * 180f / MathF.PI;

        /// <summary>Null when every sensor is mounted the right way up, otherwise a message naming each wrong one.</summary>
        public static string? CheckCalibration(CalibrationProfile p)
        {
            var problems = new List<string>();
            float pelvis = AngleDeg(UpInSensor(p.PelvisRef), Vector3.UnitX);
            if (pelvis >= PelvisMaxTiltDeg) problems.Add($"Pelvis X axis {pelvis:F0}° from vertical (must be < {PelvisMaxTiltDeg:F0}°, X axis up)");
            foreach (var (name, q) in new[] { ("Left", p.LeftFootRef), ("Right", p.RightFootRef) })
            {
                float foot = AngleDeg(UpInSensor(q), Vector3.UnitZ);
                if (foot >= FootMaxTiltDeg) problems.Add($"{name} foot Z axis {foot:F0}° from vertical (must be < {FootMaxTiltDeg:F0}°, Z axis up)");
            }
            return problems.Count == 0 ? null : string.Join("; ", problems);
        }

        /// <summary>New calibration: its tilts become the reference and alerts can fire again.</summary>
        public void Start(CalibrationProfile p)
        {
            _tracks.Clear();
            _tracks[ImuRole.Pelvis] = new Track { Reference = UpInSensor(p.PelvisRef) };
            _tracks[ImuRole.Left]   = new Track { Reference = UpInSensor(p.LeftFootRef) };
            _tracks[ImuRole.Right]  = new Track { Reference = UpInSensor(p.RightFootRef) };
        }

        public void Reset() { _tracks.Clear(); _feet.Clear(); }

        // Own walking / foot-still detection: GaitEventDetector's stance needs the foot pitch within 20° of the
        // calibration, so a foot IMU that has tilted on the shoe would never be in stance and IsWalking would drop —
        // exactly when this monitor must keep watching. Same thresholds as stance, minus the pitch condition.
        public const float FootStillFreeAcc = 2.5f;   // m/s²
        public const float FootStillGyro    = 1.0f;   // rad/s
        public const int   WalkingWindow    = 300;    // frames: both feet switched still ↔ moving within 3 s
        private long _frame;
        private readonly Dictionary<ImuRole, (bool Still, long LastSwitch)> _feet = new();

        private static bool FootStill(ImuSampleFrame f)
            => f.FreeAcceleration.Length() < FootStillFreeAcc && f.RateOfTurn.Length() < FootStillGyro;

        private bool Walking(ImuRole role, bool still)
        {
            var s = _feet.GetValueOrDefault(role, (Still: still, LastSwitch: long.MinValue / 2));
            if (s.Still != still) s = (still, _frame);
            _feet[role] = s;
            return _frame - s.LastSwitch <= WalkingWindow;
        }

        /// <summary>One frame; returns the alerts raised by it (usually none).</summary>
        public IReadOnlyList<MountingAlert> Update(ImuSampleFrame pelvis, ImuSampleFrame left, ImuSampleFrame right)
        {
            var alerts = new List<MountingAlert>();
            _frame++;
            bool leftStill = FootStill(left), rightStill = FootStill(right);
            bool walking = Walking(ImuRole.Left, leftStill) & Walking(ImuRole.Right, rightStill);
            if (!walking || _tracks.Count == 0) return alerts;
            Add(ImuRole.Pelvis, pelvis.Quaternion, alerts);
            if (leftStill)  Add(ImuRole.Left, left.Quaternion, alerts);
            if (rightStill) Add(ImuRole.Right, right.Quaternion, alerts);
            return alerts;
        }

        private void Add(ImuRole role, Quaternion q, List<MountingAlert> alerts)
        {
            var t = _tracks[role];
            var up = UpInSensor(q);
            t.Window.Enqueue(up);
            t.Sum += up;
            if (t.Window.Count > WindowFrames) t.Sum -= t.Window.Dequeue();
            if (t.Window.Count < WindowFrames) return;

            float shift = AngleDeg(t.Sum, t.Reference);
            var level = shift > PauseShiftDeg ? MountingLevel.Pause : shift > WarnShiftDeg ? MountingLevel.Warn : MountingLevel.Ok;
            // Report each level once, in order (a jump straight past 20° still gives the warning first).
            for (var l = t.Reported + 1; l <= level; l++) alerts.Add(new MountingAlert(role, shift, l));
            if (level > t.Reported) t.Reported = level;
        }
    }
}
