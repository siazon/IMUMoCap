using System;
using System.Diagnostics;
using System.Numerics;
using GaitTraining.Imu;

namespace GaitTraining.Gait
{
    // ─────────────────────────────────────────────────────────────
    //  配置
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 质量门控可调参数。所有阈值运行时可修改，下一帧立即生效。
    /// </summary>
    public sealed class QualityGateConfig
    {
        // ── 全局开关 ─────────────────────────────────────────────
        /// <summary>
        /// false = 所有门控跳过，IsValid 恒为 true。
        /// 调试时遇到无 step 输出，先关此开关确认步态检测本身是否正常。
        /// </summary>
        public bool EnableAll { get; set; } = false;

        // ── 独立开关 ─────────────────────────────────────────────
        /// <summary>Gate 1：heading 帧间突变检测。</summary>
        public bool EnableJumpGate { get; set; } = true;

        /// <summary>Gate 2：heading 变化与 gyro yaw rate 一致性检测。</summary>
        public bool EnableYawConsistency { get; set; } = true;

        /// <summary>Gate 3：mag 模长异常检测。</summary>
        public bool EnableMagNorm { get; set; } = true;

        // 在现有 EnableMagNorm 下面加
        /// <summary>
        /// 是否对脚部传感器做 mag norm 检查。
        /// 行走时脚部 mag 受金属鞋底/地面干扰波动大，建议保持 false。
        /// </summary>
        public bool EnableFootMagNorm { get; set; } = false;

        /// <summary>Gate 4：acc 模长异常检测（主要防硬件故障帧）。</summary>
        public bool EnableAccNorm { get; set; } = true;



        // ── Gate 1 阈值（heading jump） ──────────────────────────
        /// <summary>
        /// 骨盆 heading 帧间最大允许变化（°）。
        /// @100Hz，10° = 1000°/s，正常行走骨盆 yaw rate 约 30-60°/s（每帧 0.3-0.6°）。
        /// 调参：若骨盆 heading 频繁触发，先检查磁扰；可适当放宽到 15°。
        /// </summary>
        public float PelvisMaxHeadingJumpDeg { get; set; } = 10f;

        /// <summary>
        /// 脚部 heading 帧间最大允许变化（°）。
        /// 摆动期脚部 yaw rate 可达 200°/s（每帧 2°），默认 30° 留有充足余量。
        /// 调参：若 stance 内脚部 heading 频繁触发，可放宽到 45°。
        /// </summary>
        public float FootMaxHeadingJumpDeg { get; set; } = 30f;

        // ── Gate 2 阈值（yaw rate consistency） ──────────────────
        /// <summary>
        /// heading 帧间变化量与 gyro yaw rate 积分量的最大允许差值（°）。
        /// 差值大说明磁扰导致 heading 跳变但陀螺未感知。
        /// 调参：室内金属环境下可放宽到 20°；户外可收紧到 10°。
        /// </summary>
        public float MaxYawConsistencyDeg { get; set; } = 15f;

        /// <summary>IMU 采样率（Hz），用于 gyro 积分换算。</summary>
        public int SampleRateHz { get; set; } = 100;

        // ── Gate 3 阈值（mag norm，归一化单位） ──────────────────
        /// <summary>
        /// mag 模长正常范围下限（归一化）。
        /// 静止时应在 0.9-1.1；0.5 是保守下限，磁扰时可能降到 0.6-0.7。
        /// 调参：若大量帧被 mag gate 过滤，先用 Debug 打印 MagNorm 观察实际范围。
        /// </summary>
        public float MagNormMin { get; set; } = 0.5f;

        /// <summary>
        /// mag 模长正常范围上限（归一化）。
        /// 调参：同上，金属近场干扰可能导致 MagNorm > 1.3。
        /// </summary>
        public float MagNormMax { get; set; } = 1.5f;

        // ── Gate 4 阈值（acc norm） ───────────────────────────────
        /// <summary>
        /// acc 模长正常范围下限（m/s²）。低于此值疑似硬件故障或自由落体。
        /// 行走时 acc 在 0.5g-2g（4.9-19.6 m/s²）波动，1.0 是极保守下限。
        /// </summary>
        public float AccNormMin { get; set; } = 1.0f;

        /// <summary>
        /// acc 模长正常范围上限（m/s²）。高于此值疑似剧烈撞击或硬件故障。
        /// 正常行走地面冲击峰值约 3g（29.4 m/s²），25.0 留有余量。
        /// </summary>
        public float AccNormMax { get; set; } = 25.0f;
        
        // 新增配置参数
        /// <summary>
        /// 连续 invalid 帧超过此数量时，强制接受当前 heading 为新基准，解除死锁。
        /// 默认 10 帧 = 100ms。调参：越小越容易接受跳变；越大越保守。
        /// </summary>
        public int MaxInvalidStreakBeforeReset { get; set; } = 10;
    }

    // ─────────────────────────────────────────────────────────────
    //  QualityResult
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 单帧质量检查结果。下游模块检查 <see cref="IsValid"/> 决定是否处理该帧。
    /// 各 fail 字段可用于诊断具体触发原因。
    /// </summary>
    public readonly struct QualityResult
    {
        public readonly bool IsValid;

        // Gate 1 - Jump
        public readonly bool PelvisJumpFail;
        public readonly bool LeftJumpFail;
        public readonly bool RightJumpFail;

        // Gate 2 - Yaw Consistency
        public readonly bool PelvisYawConsistencyFail;
        public readonly bool LeftYawConsistencyFail;
        public readonly bool RightYawConsistencyFail;

        // Gate 3 - Mag Norm
        public readonly bool PelvisMagFail;
        public readonly bool LeftMagFail;
        public readonly bool RightMagFail;

        // Gate 4 - Acc Norm
        public readonly bool PelvisAccFail;
        public readonly bool LeftAccFail;
        public readonly bool RightAccFail;

        // 供下游读取的预计算 heading（rad），无论是否 valid 都填充
        public readonly float PelvisHeading;
        public readonly float LeftHeading;
        public readonly float RightHeading;

        public QualityResult(
            bool pelvisJump, bool leftJump, bool rightJump,
            bool pelvisYaw, bool leftYaw, bool rightYaw,
            bool pelvisMag, bool leftMag, bool rightMag,
            bool pelvisAcc, bool leftAcc, bool rightAcc,
            float pelvisHeading, float leftHeading, float rightHeading)
        {
            PelvisJumpFail = pelvisJump;
            LeftJumpFail = leftJump;
            RightJumpFail = rightJump;
            PelvisYawConsistencyFail = pelvisYaw;
            LeftYawConsistencyFail = leftYaw;
            RightYawConsistencyFail = rightYaw;
            PelvisMagFail = pelvisMag;
            LeftMagFail = leftMag;
            RightMagFail = rightMag;
            PelvisAccFail = pelvisAcc;
            LeftAccFail = leftAcc;
            RightAccFail = rightAcc;
            PelvisHeading = pelvisHeading;
            LeftHeading = leftHeading;
            RightHeading = rightHeading;

            IsValid = !(pelvisJump || leftJump || rightJump ||
                        pelvisYaw || leftYaw || rightYaw ||
                        pelvisMag || leftMag || rightMag ||
                        pelvisAcc || leftAcc || rightAcc);
        }

        /// <summary>全部通过的快捷构造（EnableAll=false 时使用）。</summary>
        public static QualityResult AllPass(float pelvisH, float leftH, float rightH)
            => new(false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   pelvisH, leftH, rightH);

        public override string ToString()
        {
            if (IsValid) return "Valid";
            return $"INVALID [" +
                   $"Jump={F(PelvisJumpFail)}P/{F(LeftJumpFail)}L/{F(RightJumpFail)}R " +
                   $"Yaw={F(PelvisYawConsistencyFail)}P/{F(LeftYawConsistencyFail)}L/{F(RightYawConsistencyFail)}R " +
                   $"Mag={F(PelvisMagFail)}P/{F(LeftMagFail)}L/{F(RightMagFail)}R " +
                   $"Acc={F(PelvisAccFail)}P/{F(LeftAccFail)}L/{F(RightAccFail)}R]";

            static string F(bool v) => v ? "X" : "o";
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  诊断
    // ─────────────────────────────────────────────────────────────

    public sealed class QualityGateDiagnostics
    {
        public long TotalFrames { get; internal set; }
        public long InvalidFrames { get; internal set; }
        public long JumpFailFrames { get; internal set; }
        public long YawConsistFailFrames { get; internal set; }
        public long MagFailFrames { get; internal set; }
        public long AccFailFrames { get; internal set; }

        public float InvalidRate => TotalFrames == 0 ? 0f
            : (float)InvalidFrames / TotalFrames;

        public override string ToString() =>
            $"Total={TotalFrames} Invalid={InvalidFrames}({InvalidRate:P1}) " +
            $"Jump={JumpFailFrames} Yaw={YawConsistFailFrames} " +
            $"Mag={MagFailFrames} Acc={AccFailFrames}";
    }

    // ─────────────────────────────────────────────────────────────
    //  QualityGate
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 全局质量门控。每帧调用 <see cref="Evaluate"/>，返回 <see cref="QualityResult"/>。
    /// <para>
    /// 上游：<see cref="ImuFrameAggregator"/> 输出的完整 <see cref="SyncedFrame"/>。
    /// 下游：stance 检测、标定、FPA 计算。
    /// </para>
    /// <para>线程假设：与其他模块相同，单线程调用。</para>
    /// </summary>
    public sealed class QualityGate
    {
        // ── 配置 ─────────────────────────────────────────────────
        public QualityGateConfig Config { get; set; }

        // ── 诊断 ─────────────────────────────────────────────────
        private readonly QualityGateDiagnostics _diag = new();
        public QualityGateDiagnostics GetDiagnostics() => _diag;

        // ── 上一帧 heading（用于 jump gate 和 yaw consistency） ──
        private float _prevPelvisHeading = float.NaN;
        private float _prevLeftHeading = float.NaN;
        private float _prevRightHeading = float.NaN;

        // 新增：连续 invalid 计数器
        private int _pelvisInvalidStreak;
        private int _leftInvalidStreak;
        private int _rightInvalidStreak;

        


        // ── 构造 ─────────────────────────────────────────────────
        public QualityGate(QualityGateConfig? config = null)
        {
            Config = config ?? new QualityGateConfig();
        }

        // ─────────────────────────────────────────────────────────
        //  主入口
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 评估一帧质量。无论结果如何，内部 heading 状态都会更新（valid 帧更新，invalid 帧不更新）。
        /// </summary>
        public QualityResult Evaluate(in SyncedFrame frame)
        {
            _diag.TotalFrames++;

            // 提取三颗 heading
            float pelvisH = HeadingUtil.PelvisHeading(frame.Pelvis.Orientation);
            float leftH = HeadingUtil.FootHeading(frame.Left.Orientation);
            float rightH = HeadingUtil.FootHeading(frame.Right.Orientation);

            // 全局开关关闭 → 直接通过，但仍更新 heading 状态
            if (!Config.EnableAll)
            {
                UpdatePrevHeadings(pelvisH, leftH, rightH);
                return QualityResult.AllPass(pelvisH, leftH, rightH);
            }

            // ── Gate 3：Mag Norm（不依赖前帧，先算） ─────────────
            bool pelvisMag = Config.EnableMagNorm && CheckMagFail(frame.Pelvis.MagNorm);
            bool leftMag = Config.EnableMagNorm && Config.EnableFootMagNorm && CheckMagFail(frame.Left.MagNorm);
            bool rightMag = Config.EnableMagNorm && Config.EnableFootMagNorm && CheckMagFail(frame.Right.MagNorm);

            // ── Gate 4：Acc Norm ──────────────────────────────────
            bool pelvisAcc = Config.EnableAccNorm && CheckAccFail(frame.Pelvis.AccMag);
            bool leftAcc = Config.EnableAccNorm && CheckAccFail(frame.Left.AccMag);
            bool rightAcc = Config.EnableAccNorm && CheckAccFail(frame.Right.AccMag);

            // ── Gate 1 & 2：需要前帧 heading ─────────────────────
            bool pelvisJump = false, leftJump = false, rightJump = false;
            bool pelvisYaw = false, leftYaw = false, rightYaw = false;

            bool hasPrev = !float.IsNaN(_prevPelvisHeading);
            if (hasPrev)
            {
                float dt = 1f / Config.SampleRateHz;   // 秒

                if (Config.EnableJumpGate)
                {
                    pelvisJump = CheckJumpFail(pelvisH, _prevPelvisHeading,
                                               Config.PelvisMaxHeadingJumpDeg);
                    leftJump = CheckJumpFail(leftH, _prevLeftHeading,
                                               Config.FootMaxHeadingJumpDeg);
                    rightJump = CheckJumpFail(rightH, _prevRightHeading,
                                               Config.FootMaxHeadingJumpDeg);

                    float leftDiffDeg = MathF.Abs(HeadingUtil.AngleDiff(leftH, _prevLeftHeading))
                        * (180f / MathF.PI);
                    float rightDiffDeg = MathF.Abs(HeadingUtil.AngleDiff(rightH, _prevRightHeading))
                                         * (180f / MathF.PI);

                    // 临时：打印触发时的实际差值
                    if (leftDiffDeg > Config.FootMaxHeadingJumpDeg ||
                        rightDiffDeg > Config.FootMaxHeadingJumpDeg)
                    {
                        Debug.WriteLine($"[JumpGate] pkt={frame.PacketId} " +
                                        $"L_diff={leftDiffDeg:F1}° R_diff={rightDiffDeg:F1}° " +
                                        $"prevLH={_prevLeftHeading * 180 / MathF.PI:F1}° " +
                                        $"currLH={leftH * 180 / MathF.PI:F1}°");
                    }
                }

                if (Config.EnableYawConsistency)
                {
                    // 骨盆：前向 -Z，对应世界系 yaw 方向的 gyro 分量
                    // 脚部：前向 +X
                    // 两种安装方式下，gyro 世界系 yaw rate 均需从传感器系转换。
                    // 简化做法：用传感器系 gyro 在世界系 Z 轴的投影作为 yaw rate 估计。
                    // q * (0,0,1) * q^-1 得到世界系 Z 轴在传感器系的表示，
                    // 与 gyro 点积即为 yaw rate（rad/s）。
                    float pelvisGyroYaw = ProjectGyroToWorldZ(frame.Pelvis);
                    float leftGyroYaw = ProjectGyroToWorldZ(frame.Left);
                    float rightGyroYaw = ProjectGyroToWorldZ(frame.Right);

                    pelvisYaw = CheckYawConsistencyFail(
                        pelvisH, _prevPelvisHeading, pelvisGyroYaw, dt);
                    leftYaw = CheckYawConsistencyFail(
                        leftH, _prevLeftHeading, leftGyroYaw, dt);
                    rightYaw = CheckYawConsistencyFail(
                        rightH, _prevRightHeading, rightGyroYaw, dt);
                }
            }

            // ── 汇总结果 ─────────────────────────────────────────
            var result = new QualityResult(
                pelvisJump, leftJump, rightJump,
                pelvisYaw, leftYaw, rightYaw,
                pelvisMag, leftMag, rightMag,
                pelvisAcc, leftAcc, rightAcc,
                pelvisH, leftH, rightH);

            // ── 更新诊断 ─────────────────────────────────────────
            if (!result.IsValid)
            {
                _diag.InvalidFrames++;
                if (pelvisJump || leftJump || rightJump) _diag.JumpFailFrames++;
                if (pelvisYaw || leftYaw || rightYaw) _diag.YawConsistFailFrames++;
                if (pelvisMag || leftMag || rightMag) _diag.MagFailFrames++;
                if (pelvisAcc || leftAcc || rightAcc) _diag.AccFailFrames++;

                Debug.WriteLine($"[QualityGate] pkt={frame.PacketId} {result}");
            }

            // ── 仅 valid 帧更新 heading 历史 ─────────────────────
            // invalid 帧不更新，防止一次磁扰后下一帧 jump gate 连锁误判
            if (result.IsValid)
                UpdatePrevHeadings(pelvisH, leftH, rightH);

            // 新逻辑：valid 帧正常更新；invalid 帧累计计数，超过阈值时强制更新
            if (result.IsValid)
            {
                UpdatePrevHeadings(pelvisH, leftH, rightH);
                _pelvisInvalidStreak = 0;
                _leftInvalidStreak = 0;
                _rightInvalidStreak = 0;
            }
            else
            {
                // 分别累计每个传感器的 invalid streak
                // 只有该传感器自己触发 jump/yaw 才累计，mag/acc 触发不算
                if (pelvisJump || pelvisYaw) _pelvisInvalidStreak++;
                else { _prevPelvisHeading = pelvisH; _pelvisInvalidStreak = 0; }

                if (leftJump || leftYaw) _leftInvalidStreak++;
                else { _prevLeftHeading = leftH; _leftInvalidStreak = 0; }

                if (rightJump || rightYaw) _rightInvalidStreak++;
                else { _prevRightHeading = rightH; _rightInvalidStreak = 0; }

                // 超过阈值：强制接受新 heading，解除死锁
                int maxStreak = Config.MaxInvalidStreakBeforeReset;

                if (_pelvisInvalidStreak >= maxStreak)
                {
                    _prevPelvisHeading = pelvisH;
                    _pelvisInvalidStreak = 0;
                    Debug.WriteLine($"[QualityGate] Pelvis heading reset forced at pkt={frame.PacketId} " +
                                    $"newH={pelvisH * 180f / MathF.PI:F1}°");
                }
                if (_leftInvalidStreak >= maxStreak)
                {
                    _prevLeftHeading = leftH;
                    _leftInvalidStreak = 0;
                    Debug.WriteLine($"[QualityGate] Left heading reset forced at pkt={frame.PacketId} " +
                                    $"newH={leftH * 180f / MathF.PI:F1}°");
                }
                if (_rightInvalidStreak >= maxStreak)
                {
                    _prevRightHeading = rightH;
                    _rightInvalidStreak = 0;
                    Debug.WriteLine($"[QualityGate] Right heading reset forced at pkt={frame.PacketId} " +
                                    $"newH={rightH * 180f / MathF.PI:F1}°");
                }
            }

            return result;
        }

        // ─────────────────────────────────────────────────────────
        //  门控判断
        // ─────────────────────────────────────────────────────────

        private bool CheckMagFail(float magNorm)
            => magNorm < Config.MagNormMin || magNorm > Config.MagNormMax;

        private bool CheckAccFail(float accNorm)
            => accNorm < Config.AccNormMin || accNorm > Config.AccNormMax;

        private bool CheckJumpFail(float curr, float prev, float maxDeg)
        {
            float diffDeg = MathF.Abs(HeadingUtil.AngleDiff(curr, prev))
                            * (180f / MathF.PI);
            return diffDeg > maxDeg;
        }

        private bool CheckYawConsistencyFail(
            float currH, float prevH, float gyroYawRad, float dt)
        {
            float headingDiffDeg = HeadingUtil.AngleDiff(currH, prevH)
                                   * (180f / MathF.PI);
            float gyroDiffDeg = gyroYawRad * dt * (180f / MathF.PI);
            float errorDeg = MathF.Abs(headingDiffDeg - gyroDiffDeg);
            return errorDeg > Config.MaxYawConsistencyDeg;
        }

        // ─────────────────────────────────────────────────────────
        //  Gyro Yaw Rate 投影到世界系 Z 轴
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 将传感器系 gyro 向量投影到世界系 Z 轴，得到 yaw rate（rad/s）。
        /// 公式：worldZ_in_sensor = q^-1 * (0,0,1)；yawRate = dot(gyro, worldZ_in_sensor)。
        /// 这等价于把 gyro 旋转到世界系后取 Z 分量，但避免了完整的向量旋转。
        /// </summary>
        private static float ProjectGyroToWorldZ(in ImuSample s)
        {
            // 世界系 Z 轴 (0,0,1) 在传感器系的表示：q^-1 旋转
            var qInv = Quaternion.Inverse(s.Orientation);
            var worldZInSensor = System.Numerics.Vector3.Transform(
                new System.Numerics.Vector3(0f, 0f, 1f), qInv);

            // dot product with gyro
            return System.Numerics.Vector3.Dot(s.Gyr, worldZInSensor);
        }

        // ─────────────────────────────────────────────────────────
        //  辅助
        // ─────────────────────────────────────────────────────────

        private void UpdatePrevHeadings(float p, float l, float r)
        {
            _prevPelvisHeading = p;
            _prevLeftHeading = l;
            _prevRightHeading = r;
        }

        // ─────────────────────────────────────────────────────────
        //  Reset
        // ─────────────────────────────────────────────────────────

        public void Reset()
        {
            _prevPelvisHeading = float.NaN;
            _prevLeftHeading = float.NaN;
            _prevRightHeading = float.NaN;
            _pelvisInvalidStreak = 0;
            _leftInvalidStreak = 0;
            _rightInvalidStreak = 0;
            _diag.TotalFrames = 0;
            _diag.InvalidFrames = 0;
            _diag.JumpFailFrames = 0;
            _diag.YawConsistFailFrames = 0;
            _diag.MagFailFrames = 0;
            _diag.AccFailFrames = 0;
            Debug.WriteLine("[QualityGate] Reset.");
        }
    }
}