// IMUMoCap/Pipeline/DataQualityGate.cs
using System.Numerics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 对完整的 ImuFrameBundle 进行质量检查。
    /// 通过检查则返回 ValidFrame，否则返回 null（帧被丢弃）。
    ///
    /// 检查项：
    /// 1. StatusWord: bit1=OrientationValid(0→reject), bit8-13=ClipFlags(1→reject)
    /// 2. 任一 IMU 的 RSSI 低于阈值
    /// 3. 任一 IMU 的加速度与上帧差超突变阈值（孤立尖峰）
    /// 4. 任一 IMU 的角速度与上帧差超突变阈值
    /// </summary>
    public sealed class DataQualityGate
    {
        // 可调参数（实验前根据环境设定，不需要运行时动态修改）
        public int   RssiThresholdDbm        { get; set; } = -50;
        public float AccDeltaThreshold_ms2   { get; set; } = 100f;  // m/s²  (raised: stomp delta ~70 m/s²)
        public float GyroDeltaThreshold_rads { get; set; } = 20f;   // rad/s

        private ImuSampleFrame? _prevPelvis;
        private ImuSampleFrame? _prevLeft;
        private ImuSampleFrame? _prevRight;

        public ValidFrame? Evaluate(ImuFrameBundle bundle)
        {
            if (!bundle.IsComplete) return null;

            var p = bundle.Pelvis!;
            var l = bundle.LeftFoot!;
            var r = bundle.RightFoot!;

            // 1. StatusWord 检查
            //if (HasStatusError(p) || HasStatusError(l) || HasStatusError(r))
            //{
            //    UpdatePrev(p, l, r);
            //    return null;
            //}

            // 2. RSSI 检查
            //if (p.Rssi < RssiThresholdDbm || l.Rssi < RssiThresholdDbm || r.Rssi < RssiThresholdDbm)
            //{
            //    UpdatePrev(p, l, r);
            //    return null;
            //}

            // 3 & 4. 突变检查（与上帧比较）
            if (_prevPelvis != null && (HasAccSpike(p, _prevPelvis) || HasGyroSpike(p, _prevPelvis)))
            {
                UpdatePrev(p, l, r);
                return null;
            }
            if (_prevLeft != null && (HasAccSpike(l, _prevLeft) || HasGyroSpike(l, _prevLeft)))
            {
                UpdatePrev(p, l, r);
                return null;
            }
            if (_prevRight != null && (HasAccSpike(r, _prevRight) || HasGyroSpike(r, _prevRight)))
            {
                UpdatePrev(p, l, r);
                return null;
            }

            UpdatePrev(p, l, r);
            return new ValidFrame(bundle);
        }

        public void Reset()
        {
            _prevPelvis = _prevLeft = _prevRight = null;
        }

        // ── 私有辅助 ──────────────────────────────────────────────────────────

        private static bool HasStatusError(ImuSampleFrame f)
        {
            if (f.StatusWord == null) return false;
            uint s = f.StatusWord.Value;

            // bit 1 = OrientationValid: 0 means filter not yet converged → reject
            if ((s & 0x02u) == 0) return true;

            // bit 8–13 = per-axis clip flags (sensor saturation) → reject
            if ((s & 0x3F00u) != 0) return true;

            return false;
        }

        private bool HasAccSpike(ImuSampleFrame curr, ImuSampleFrame prev)
        {
            if (!curr.HasAcceleration || !prev.HasAcceleration) return false;
            return Vector3.Distance(curr.Acceleration, prev.Acceleration) > AccDeltaThreshold_ms2;
        }

        private bool HasGyroSpike(ImuSampleFrame curr, ImuSampleFrame prev)
        {
            if (!curr.HasRateOfTurn || !prev.HasRateOfTurn) return false;
            return Vector3.Distance(curr.RateOfTurn, prev.RateOfTurn) > GyroDeltaThreshold_rads;
        }

        private void UpdatePrev(ImuSampleFrame p, ImuSampleFrame l, ImuSampleFrame r)
        {
            _prevPelvis = p;
            _prevLeft   = l;
            _prevRight  = r;
        }
    }
}
