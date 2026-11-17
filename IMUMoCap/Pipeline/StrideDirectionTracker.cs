// IMUMoCap/Pipeline/StrideDirectionTracker.cs
using System;
using System.Numerics;

namespace IMUMoCap.Pipeline
{
    /// <summary>一步(上一次着地结束 → 这一次着地开始)的水平位移。方向在世界系(与 free acceleration 同一坐标系)。</summary>
    public readonly record struct StrideResult(
        float DirectionRad, float LengthM, float SwingSec, float ResidualSpeed, int MissingPackets, long LandingPacketId);

    /// <summary>
    /// 步幅方向(docs/stride_fpa_design.md):摆动期对 free acceleration 水平分量二次积分(梯形),
    /// 两端速度强制为 0(ZUPT,线性去漂:位移减去 v_end·T/2),着地那一帧给出 StrideResult。
    /// 时间步长按 PacketId 差计算,丢包数单独统计(供门控 G2)。每只脚一个实例。
    /// </summary>
    public sealed class StrideDirectionTracker
    {
        /// <summary>PacketId 差换算成秒用。</summary>
        public int SampleRateHz { get; set; } = 100;

        private bool _hasAnchor, _inSwing;
        private long _prevPacketId;
        private Vector2 _prevAcc, _v, _p;
        private float _t;
        private int _missing;

        /// <summary>每个有效帧调用一次;只有摆动 → 着地的那一帧返回结果。</summary>
        public StrideResult? Update(long packetId, bool stance, Vector3 freeAcc)
        {
            var a = new Vector2(freeAcc.X, freeAcc.Y);
            StrideResult? result = null;

            if (_inSwing || (!stance && _hasAnchor))
            {
                if (!_inSwing) { _inSwing = true; _v = _p = Vector2.Zero; _t = 0f; _missing = 0; }
                long gap = packetId - _prevPacketId;
                float dt = gap / (float)SampleRateHz;
                _missing += (int)Math.Max(0, gap - 1);
                var vNew = _v + 0.5f * (_prevAcc + a) * dt;
                _p += 0.5f * (_v + vNew) * dt;
                _v = vNew;
                _t += dt;

                if (stance)
                {
                    var d = _p - _v * (_t / 2f);
                    result = new StrideResult(MathF.Atan2(d.Y, d.X), d.Length(), _t, _v.Length(), _missing, packetId);
                    _inSwing = false;
                }
            }

            if (stance) _hasAnchor = true;
            _prevPacketId = packetId;
            _prevAcc = a;
            return result;
        }

        public void Reset()
        {
            _hasAnchor = _inSwing = false;
            _v = _p = Vector2.Zero;
            _t = 0f;
            _missing = 0;
        }
    }
}
