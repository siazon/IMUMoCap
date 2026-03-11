using System;
using System.Numerics;

namespace GaitTraining.Gait
{
    /// <summary>
    /// 从传感器→世界系 Quaternion 提取水平面 heading（rad，数学角）。
    /// <para>
    /// 世界系：ENU，Z-up。Heading = atan2(y, x)，从东轴逆时针，范围 (-π, π]。
    /// </para>
    /// <para>
    /// 传感器前向约定：
    ///   骨盆 → 传感器系 -Z (0, 0, -1)
    ///   左脚 → 传感器系 +X (1, 0,  0)
    ///   右脚 → 传感器系 +X (1, 0,  0)
    /// </para>
    /// </summary>
    public static class HeadingUtil
    {
        // 传感器系前向向量
        private static readonly Vector3 PelvisLocalForward = new(0f, 0f, -1f);
        private static readonly Vector3 FootLocalForward = new(1f, 0f, 0f);

        // ─────────────────────────────────────────────────────────
        //  公开接口
        // ─────────────────────────────────────────────────────────

        /// <summary>提取骨盆 heading（rad）。</summary>
        public static float PelvisHeading(Quaternion sensorToWorld)
            => ExtractHeading(sensorToWorld, PelvisLocalForward);

        /// <summary>提取脚部 heading（rad）。左右脚前向相同，均为 +X。</summary>
        public static float FootHeading(Quaternion sensorToWorld)
            => ExtractHeading(sensorToWorld, FootLocalForward);

        /// <summary>
        /// 圆域角度差 a - b，结果范围 (-π, π]。
        /// 用于 jump gate 和 yaw consistency 计算。
        /// </summary>
        public static float AngleDiff(float a, float b)
        {
            float d = a - b;
            // 折叠到 (-π, π]
            while (d > MathF.PI) d -= 2f * MathF.PI;
            while (d < -MathF.PI) d += 2f * MathF.PI;
            return d;
        }

        /// <summary>
        /// 圆域均值（circular mean）。输入为 heading 数组（rad），返回 (-π, π]。
        /// </summary>
        public static float CircularMean(ReadOnlySpan<float> headings)
        {
            if (headings.IsEmpty) return 0f;
            float sumSin = 0f, sumCos = 0f;
            foreach (float h in headings)
            {
                sumSin += MathF.Sin(h);
                sumCos += MathF.Cos(h);
            }
            return MathF.Atan2(sumSin / headings.Length, sumCos / headings.Length);
        }

        // ─────────────────────────────────────────────────────────
        //  内部实现
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 把传感器系前向向量旋转到世界系，投影水平面，取 atan2(y, x)。
        /// </summary>
        private static float ExtractHeading(Quaternion q, Vector3 localForward)
        {
            // System.Numerics：Vector3.Transform(v, q) = q * v * q^-1
            Vector3 world = Vector3.Transform(localForward, q);

            // 投影到水平面（忽略 Z），取数学角
            // ENU: x=East, y=North → atan2(y,x) 从东轴逆时针
            return MathF.Atan2(world.Y, world.X);
        }
    }
}