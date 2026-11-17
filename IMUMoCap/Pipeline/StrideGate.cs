// IMUMoCap/Pipeline/StrideGate.cs
using System;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 步幅法 FPA 的门控(docs/stride_fpa_design.md 二、门控)。阈值为离线数据初值,需在回放测试中调整。
    /// 不使用 PD 有效性/稳定性,也不要求 PD 与骨盆一致。
    /// </summary>
    public static class StrideGate
    {
        public const float MinSwingSec = 0.25f, MaxSwingSec = 1.5f;   // G1
        public const int   MaxMissingPackets = 5;                     // G2(回放 11-12 16:38 调整:缺 3–5 帧的步方向噪声与 0–2 帧相同)
        public const float MinLengthM = 0.3f, MaxLengthM = 2.5f;      // G3
        public const float MaxResidualSpeed = 1.0f;                   // G4,m/s
        public const float MaxDirectionChangeDeg = 15f;               // G5 / G6
        public const float MaxAbsFpaDeg = 60f;                        // G8

        /// <param name="prevSameFootDirRad">同一只脚上一步的步幅方向;没有则为 null(视为不满足 G5)。</param>
        /// <param name="otherFootStraight">G6:另一只脚最近一步是否直走(按它自己的 G5)。每个传感器的航向参考各自独立
        /// (VRU 下任意且各自漂移,回放 11-12 16:38 时左右脚方向相差约 100°),所以不能直接比较两只脚的绝对方向。</param>
        /// <param name="turning">G7:MotionContext 判定转弯。</param>
        /// <param name="fpaDeg">本步步幅法 FPA;零点尚未估出时为 null,跳过 G8。</param>
        /// <returns>null = 通过;否则为排除原因。</returns>
        public static StepExclusionReason? Check(StrideResult s, float? prevSameFootDirRad, bool otherFootStraight,
                                                 bool turning, float? fpaDeg)
        {
            if (!IsIntegrationValid(s)) return StepExclusionReason.StrideInvalid;

            if (turning) return StepExclusionReason.Turning;

            if (!IsStraight(s, prevSameFootDirRad) || !otherFootStraight)
                return StepExclusionReason.StrideNotStraight;

            if (fpaDeg is float f && MathF.Abs(f) >= MaxAbsFpaDeg)
                return StepExclusionReason.StrideInvalid;

            return null;
        }

        /// <summary>G1–G4:积分本身可信(只有这样的步才作为下一步 G5/G6 的比较基准)。</summary>
        public static bool IsIntegrationValid(StrideResult s)
            => s.SwingSec >= MinSwingSec && s.SwingSec <= MaxSwingSec
               && s.MissingPackets <= MaxMissingPackets
               && s.LengthM >= MinLengthM && s.LengthM <= MaxLengthM
               && s.ResidualSpeed < MaxResidualSpeed;

        /// <summary>G5:积分可信,且与同一只脚上一步的方向差 &lt; MaxDirectionChangeDeg(没有上一步则不算直走)。</summary>
        public static bool IsStraight(StrideResult s, float? prevSameFootDirRad)
        {
            if (!IsIntegrationValid(s) || prevSameFootDirRad is not float prev) return false;
            float d = MathF.IEEERemainder(s.DirectionRad - prev, 2f * MathF.PI);
            return MathF.Abs(d) * 180f / MathF.PI < MaxDirectionChangeDeg;
        }
    }
}
