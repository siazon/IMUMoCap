// IMUMoCap/Pipeline/FpaEngine.cs
using System;
using System.Collections.Generic;
using System.Diagnostics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 门控 + 落地稳定后结算，输出步级 FpaResult。
    ///
    /// 采集与输出分离：
    ///   采集：只要脚在 stance 就持续采集 yaw，gate 失败不丢弃已采集数据。
    ///   输出：数据驱动结算——落地满 MinStanceFramesFloor 帧后，一旦该脚角速度连续
    ///         SettleQuietFrames 帧低于 SettleGyroThreshold（真正静止，不是固定帧数瞎猜）
    ///         AND 当前帧 gate 全通过 → 立即输出 FPA；若一直不安静，到 MaxStanceFramesForSettle
    ///         硬上限强制结算，防止个别落地冲击大的步子卡住不输出。
    ///
    /// FPA 计算：
    ///   FPA = mean(foot_yaw，落地到结算这段帧) - PD.DirectionRad，转换为度
    ///   正值 = toe-out，负值 = toe-in
    /// </summary>
    public sealed class FpaEngine
    {
        // ── 门控阈值 ──────────────────────────────────────────────────────────
        public float ContextConfidenceThreshold { get; set; } = 0.7f;
        public float PdStabilityThreshold { get; set; } = 0.7f;

        // ── 结算参数（数据驱动）──────────────────────────────────────────────
        // 落地后至少攒够这么多帧才可能结算，防止单帧抖动/刚过 debounce 边界那一帧被当成"已安静"
        // 回放验证(2026-11-09 数据):floor=2 + 静止检测会让一次摆动里的短暂静止被当成新一步而重复输出;
        // 恢复为 10 帧下限且不要求连续静止(SettleQuietFrames=0),即 9fc4d57f 之前的"落地满 10 帧"规则。
        public int MinStanceFramesFloor { get; set; } = 10;
        // 角速度模长连续低于此值达到 SettleQuietFrames 帧，才认为脚真正静止（比 GaitEventDetector
        // 判定"算不算落地"用的 GyroThreshold=1.0 更紧）。0.35 是在两份真实录制（一份转身较多的
        // 测试走动、一份 188s 的 Baseline+Training 正式录制）上验证过的值：0.2 时正常走路(Straight)
        // 下仍有 4-8% 的落地因为一直没连续 3 帧低于阈值而被静默丢弃（docs/task12、task13），
        // 0.3-0.4 基本消除该风险，0.35 取中间留一点余量。
        public float SettleGyroThreshold { get; set; } = 0.35f; // rad/s
        public int SettleQuietFrames { get; set; } = 0;
        // 硬上限：即使一直不安静也不能无限等（落地冲击大/有回弹的步子），到点强制结算
        public int MaxStanceFramesForSettle { get; set; } = 20; // frames (200ms @ 100Hz)

        // ── BaselineProfile（Training 阶段设置，Baseline 阶段为 null）────────
        public BaselineProfile? Baseline { get; set; }

        // ── 步级结果事件：每个真正落地(stance)完成的周期，无论有没有输出 FpaResult ──
        // emitted=true 时 reason 为 null；emitted=false 时 reason 说明被门控排除的原因
        // （用于统计 Turning 等排除比例，供 QoE 分析用，不影响 FpaResult 本身的输出逻辑）
        public event Action<string, bool, StepExclusionReason?>? OnStepOutcome;

        // ── 步幅法(docs/stride_fpa_design.md)──────────────────────────────────
        /// <summary>
        /// true:反馈用步幅法 FPA(零点估出前仍用当前方法);false:反馈用当前方法,步幅法只记录。
        /// 两种方法的结果都写入 FpaResult.FpaPd_* / FpaStride_*。
        /// </summary>
        public bool UseStrideDirection { get; set; } = false;

        /// <summary>
        /// 零点(docs/stride_fpa_design.md 一):零点估计阶段(反馈仍用当前方法)同时估两种,都估出后每只脚定一次:
        ///   方案 B:φ_B = 校准时脚的航向 − 最初 N 个直线步的平均方向(返程按 180° 折回),只用脚自己的传感器;
        ///   方案 A:φ_A = 前 N 步"步幅法 raw − 当前方法 raw"的均值(骨盆参照)。
        /// |φ_A − φ_B| ≤ ZeroCrossCheckMaxDeg 用 B;否则说明校准时没有面朝行走路线(B 会整体偏几十度),退回 A。
        /// </summary>
        public const float ZeroCrossCheckMaxDeg = 15f;

        /// <summary>
        /// φ_B 估出后,方案 A 再过这么多个(积分可信的)步仍未估出(如开头 PD 一直无效 / 卡在 ReacquiringPd),
        /// 就不做交叉检查直接用 B,免得步幅法反馈一直等下去。
        /// </summary>
        public const int ZeroCrossCheckTimeoutStrides = 10;

        /// <summary>每只脚的零点定下时触发一次:(foot, 采用的 φ°, 该方案的圆周标准差°)。</summary>
        public event Action<string, float, float>? OnStrideZeroEstimated;

        /// <summary>每只脚的零点定下时触发一次:(foot, φ_A − φ_B °(超时未做交叉检查时为 NaN), 是否采用了方案 B)。</summary>
        public event Action<string, float, bool>? OnStrideZeroCrossCheck;

        /// <summary>
        /// 零点定下前这只脚的 MTw 滤波器重启:(foot, 说明)。校准时的航向属于旧滤波器坐标系,零点无法再估对,
        /// 这只脚停止输出,直到重新校准(其校准参考改变)后重新估计零点。
        /// </summary>
        public event Action<string, string>? OnStrideZeroInvalidated;

        /// <summary>
        /// MTw 滤波器重启(StatusWord OrientationValid 1 → 0,由 GaitPipeline 在 DataQualityGate 之前检测)。
        /// 零点定下后:无需处理(步幅法 FPA 只比较同一传感器同一步内的量,与坐标系无关)。零点定下前:
        ///   脚:这只脚零点作废、停止输出,等重新校准;
        ///   骨盆:PD(当前方法)不再可信 → 不再输出零点前的 PD 结果、不再收集方案 A,方案 B 一估出就直接采用(不做交叉检查)。
        /// 校准完成前(还没进入零点估计)忽略。
        /// </summary>
        public void NotifyFilterRestart(ImuRole role)
        {
            if (!_calibrated) return;
            if (role == ImuRole.Pelvis)
            {
                foreach (var (foot, sf) in new[] { ("L", _leftStride), ("R", _rightStride) })
                {
                    if (sf.PhiB != null || sf.NeedsRecalibration) continue;
                    sf.PdUnreliable = true;
                    DecideZero(foot, sf);
                }
                return;
            }
            var (f, s) = role == ImuRole.Left ? ("L", _leftStride) : ("R", _rightStride);
            if (s.PhiB != null || s.NeedsRecalibration) return;
            s.NeedsRecalibration = true;
            s.RecalibrationRef = s.LastFootRef;
            OnStrideZeroInvalidated?.Invoke(f, "filter restarted before the stride FPA zero was decided — recalibration needed");
        }

        private bool _calibrated;

        public int SampleRateHz
        {
            set { _leftStride.Tracker.SampleRateHz = value; _rightStride.Tracker.SampleRateHz = value; }
        }

        // ── 内部 stance 追踪（per foot）───────────────────────────────────────
        private readonly StanceSampler _leftSampler = new();
        private readonly StanceSampler _rightSampler = new();
        private readonly StrideFoot _leftStride = new();
        private readonly StrideFoot _rightStride = new();

        public FpaResult? Process(ValidFrame frame, GaitEvent gait,
                                  MotionContext context, PdEstimate pd,
                                  CalibrationProfile? calibration = null)
        {
            if (calibration != null) _calibrated = true;
            // a foot waiting for recalibration after a filter restart: a new calibration (changed reference) restarts its zero
            _leftStride.TrackCalibration(calibration?.LeftFootRef);
            _rightStride.TrackCalibration(calibration?.RightFootRef);

            // ── Step 0: 步幅积分(两只脚先各自给出本帧的着地结果,再更新"上一步方向"历史)──
            var strideL = _leftStride.Tracker.Update(frame.PacketId, gait.LeftStance,
                frame.LeftFoot.HasFreeAcceleration ? frame.LeftFoot.FreeAcceleration : System.Numerics.Vector3.Zero);
            var strideR = _rightStride.Tracker.Update(frame.PacketId, gait.RightStance,
                frame.RightFoot.HasFreeAcceleration ? frame.RightFoot.FreeAcceleration : System.Numerics.Vector3.Zero);
            _leftStride.OnLanding(strideL, _rightStride.LastStraight);
            _rightStride.OnLanding(strideR, _leftStride.LastStraight);
            if (!gait.LeftStance) _leftStride.Pending = null;
            if (!gait.RightStance) _rightStride.Pending = null;
            bool turningNow = context.State == ContextState.Turning;
            CollectEarlyDirection("L", strideL, _leftStride, calibration?.LeftFootRef, turningNow);
            CollectEarlyDirection("R", strideR, _rightStride, calibration?.RightFootRef, turningNow);

            // ── Step 1: 无条件更新 stance 状态（采集与 gate 无关）────────────────
            // stance→swing 的转换帧上，如果这一步从未成功输出过 FPA，视为一次排除。
            if (gait.LeftStance)
                _leftSampler.AddFrame(NormalizeAngle(
                    ExtractYaw(frame.LeftFoot.Quaternion) -
                    ExtractYaw(calibration?.LeftFootRef ?? System.Numerics.Quaternion.Identity)),
                    frame.LeftFoot.RateOfTurn.Length(), SettleGyroThreshold);
            else
            {
                var excludedL = _leftSampler.MarkSwing();
                if (excludedL.HasValue) OnStepOutcome?.Invoke("L", false, excludedL);
            }

            if (gait.RightStance)
                _rightSampler.AddFrame(NormalizeAngle(
                    ExtractYaw(frame.RightFoot.Quaternion) -
                    ExtractYaw(calibration?.RightFootRef ?? System.Numerics.Quaternion.Identity)),
                    frame.RightFoot.RateOfTurn.Length(), SettleGyroThreshold);
            else
            {
                var excludedR = _rightSampler.MarkSwing();
                if (excludedR.HasValue) OnStepOutcome?.Invoke("R", false, excludedR);
            }

            // ── Step 2: Gate 检查——仅影响输出，不影响采集 ──────────────────────
            bool contextOk = context.State == ContextState.Straight
                          && context.Confidence >= ContextConfidenceThreshold;
            bool pdOk = pd.IsValid && pd.Stability >= PdStabilityThreshold;

            // 门控失败原因（仅在 !contextOk 时才会被 NoteGate 采用；contextOk 但 !pdOk 时用 PdInvalid）
            StepExclusionReason blockReason = context.State switch
            {
                ContextState.Turning       => StepExclusionReason.Turning,
                ContextState.ReacquiringPd => StepExclusionReason.ReacquiringPd,
                _ /* Straight 但 confidence 不够，或 pd 不合格 */ =>
                    contextOk ? StepExclusionReason.PdInvalid : StepExclusionReason.LowConfidence,
            };
            bool pdGateOk = contextOk && pdOk;
            bool turning = context.State == ContextState.Turning;
            // ── Step 3: 每只脚各自门控 + 结算(脚已安静或触发硬上限)──────────────
            // 右脚取反：yaw 以逆时针为正，左脚外八 = 逆时针（+），右脚外八 = 顺时针（−）；
            // 取反后左右脚统一为"正 = toe-out，负 = toe-in"，使同一个 target 规则对两只脚含义一致。
            var footL = SettleFoot("L", +1f, _leftSampler, _leftStride, calibration?.LeftFootRef, pdGateOk, blockReason, turning, pd);
            var footR = SettleFoot("R", -1f, _rightSampler, _rightStride, calibration?.RightFootRef, pdGateOk, blockReason, turning, pd);

            if (footL == null && footR == null) return null;

            // 此时的 confidence/stability 即决定本次输出的原始质量
            bool marginal = (context.Confidence - ContextConfidenceThreshold < 0.1f)
                         || (pd.Stability - PdStabilityThreshold < 0.1f);
            string quality = marginal ? "Marginal" : "High";

            float fpaLDeg = footL?.Feedback ?? float.NaN;
            float fpaRDeg = footR?.Feedback ?? float.NaN;

            bool onTargetL = false, onTargetR = false;
            float errorL = float.NaN, errorR = float.NaN;

            if (Baseline != null)
            {
                if (!float.IsNaN(fpaLDeg))
                {
                    errorL = fpaLDeg - Baseline.Target_L;
                    onTargetL = MathF.Abs(errorL) <= Baseline.ToleranceL;
                }
                if (!float.IsNaN(fpaRDeg))
                {
                    errorR = fpaRDeg - Baseline.Target_R;
                    onTargetR = MathF.Abs(errorR) <= Baseline.ToleranceR;
                }
            }

            return new FpaResult
            {
                PacketId = frame.PacketId,
                Fpa_L = fpaLDeg,
                Fpa_R = fpaRDeg,
                FpaPd_L = footL?.Pd ?? float.NaN,
                FpaPd_R = footR?.Pd ?? float.NaN,
                FpaStride_L = footL?.Stride ?? float.NaN,
                FpaStride_R = footR?.Stride ?? float.NaN,
                OnTarget_L = onTargetL,
                OnTarget_R = onTargetR,
                Error_L = errorL,
                Error_R = errorR,
                Tolerance_L = Baseline?.ToleranceL ?? float.NaN,
                Tolerance_R = Baseline?.ToleranceR ?? float.NaN,
                ContextConfidence = context.Confidence,
                PdStability = pd.Stability,
                Quality = quality,
            };
        }

        /// <summary>
        /// 一只脚本帧的门控与结算。门控:步幅法模式且零点已知 → StrideGate;否则 → 当前方法的 PD 门控。
        /// 结算时两种方法各自算一次(各自门控不过则为 NaN),并在两者都可信时为零点估计提供一个样本。
        /// 返回 null 表示这只脚本帧没有结算。
        /// </summary>
        private (float Feedback, float Pd, float Stride)? SettleFoot(
            string foot, float sign, StanceSampler sampler, StrideFoot sf, System.Numerics.Quaternion? footRef,
            bool pdGateOk, StepExclusionReason pdBlockReason, bool turning, PdEstimate pd)
        {
            float yawRef = ExtractYaw(footRef ?? System.Numerics.Quaternion.Identity);
            bool strideFeedback = UseStrideDirection && sf.PhiB != null;

            StepExclusionReason? strideReason = StepExclusionReason.StrideInvalid;
            if (sf.Pending is StrideResult pending && sampler.PeekMean() is float meanNow)
                strideReason = StrideGate.Check(pending, sf.PrevSameDir, sf.OtherStraight, turning,
                    sf.PhiB != null ? StrideFpaDeg(sign, meanNow + yawRef - pending.DirectionRad, sf) : null);

            // Stride mode before the zero is decided: PD FPA is the feedback, but only for steps with a real stride (G1–G4),
            // so shuffling / tiny foot movements (Pf2 2026-11-17: 1 cm "strides") don't fill the baseline.
            // After a pelvis filter restart (PdUnreliable) the PD FPA isn't trusted any more; after a foot restart
            // (NeedsRecalibration) nothing for this foot is, until it's recalibrated.
            bool preZeroStrideOk = !UseStrideDirection
                || (sf.Pending is StrideResult p && StrideGate.IsIntegrationValid(p) && !sf.PdUnreliable);
            bool gateOk = !sf.NeedsRecalibration && (strideFeedback ? strideReason == null : pdGateOk && preZeroStrideOk);
            sampler.NoteGate(MinStanceFramesFloor, SettleQuietFrames, MaxStanceFramesForSettle, gateOk,
                             sf.NeedsRecalibration ? StepExclusionReason.StrideInvalid
                             : strideFeedback ? strideReason ?? pdBlockReason
                             : !preZeroStrideOk && pdGateOk ? StepExclusionReason.StrideInvalid : pdBlockReason);
            if (!gateOk) return null;

            float? settled = sampler.TrySettle(MinStanceFramesFloor, SettleQuietFrames, MaxStanceFramesForSettle);
            if (settled is not float rel) return null;
            OnStepOutcome?.Invoke(foot, true, null);

            float pdRaw = NormalizeAngle(rel - pd.DirectionRad);
            float fpaPd = pdGateOk ? sign * RadToDeg(pdRaw) : float.NaN;
            float fpaStride = float.NaN;
            if (sf.Pending is StrideResult s)
            {
                float strideRaw = NormalizeAngle(rel + yawRef - s.DirectionRad);   // ψ_abs − s
                // plan A zero (pelvis-referenced), kept only for the cross-check against plan B
                if (!sf.ZeroA.IsReady && pdGateOk && !sf.PdUnreliable && StrideGate.Check(s, sf.PrevSameDir, sf.OtherStraight, turning, null) == null
                    && sf.ZeroA.Add(NormalizeAngle(strideRaw - pdRaw)))
                    DecideZero(foot, sf);
                if (sf.PhiB != null)
                {
                    float f = StrideFpaDeg(sign, strideRaw, sf);
                    if (StrideGate.Check(s, sf.PrevSameDir, sf.OtherStraight, turning, f) == null) fpaStride = f;
                }
            }
            return (strideFeedback ? fpaStride : fpaPd, fpaPd, fpaStride);
        }

        private static float StrideFpaDeg(float sign, float strideRaw, StrideFoot sf)
            => sign * RadToDeg(NormalizeAngle(strideRaw - sf.PhiB!.Value));

        /// <summary>
        /// 方案 B 候选:着地时收集最初 N 个直线步(G1–G7)的方向,返程(与第一个相差 &gt; 90°)按 180° 折回;
        /// 满 N 个后 φ_B = 校准时脚的航向 − 平均方向。是否采用由 DecideZero 决定。
        /// </summary>
        private void CollectEarlyDirection(string foot, StrideResult? stride, StrideFoot sf,
                                           System.Numerics.Quaternion? footRef, bool turning)
        {
            if (sf.PhiB != null || sf.NeedsRecalibration || stride is not StrideResult s) return;
            if (sf.CandidateB is float b)
            {
                // waiting for plan A: after the timeout, use B without the cross-check
                if (StrideGate.IsIntegrationValid(s) && ++sf.StridesSinceB >= ZeroCrossCheckTimeoutStrides)
                {
                    sf.PhiB = b;
                    OnStrideZeroEstimated?.Invoke(foot, RadToDeg(b), sf.Early.SdDeg);
                    OnStrideZeroCrossCheck?.Invoke(foot, float.NaN, true);
                }
                return;
            }
            if (StrideGate.Check(s, sf.PrevSameDir, sf.OtherStraight, turning, null) != null) return;
            sf.EarlyAnchor ??= s.DirectionRad;
            float dir = MathF.Abs(NormalizeAngle(s.DirectionRad - sf.EarlyAnchor.Value)) > MathF.PI / 2
                ? NormalizeAngle(s.DirectionRad + MathF.PI) : s.DirectionRad;
            if (!sf.Early.Add(dir)) return;
            sf.CandidateB = NormalizeAngle(ExtractYaw(footRef ?? System.Numerics.Quaternion.Identity) - sf.Early.OffsetRad);
            DecideZero(foot, sf);
        }

        /// <summary>两种零点都估出后定下这只脚的零点(见 ZeroCrossCheckMaxDeg),之后步幅法 FPA 才输出。</summary>
        private void DecideZero(string foot, StrideFoot sf)
        {
            if (sf.PhiB != null || sf.CandidateB is not float b) return;
            if (sf.PdUnreliable)
            {
                // pelvis filter restarted before the zero: plan A can't be trusted, use B without the cross-check
                sf.PhiB = b;
                OnStrideZeroEstimated?.Invoke(foot, RadToDeg(b), sf.Early.SdDeg);
                OnStrideZeroCrossCheck?.Invoke(foot, float.NaN, true);
                return;
            }
            if (!sf.ZeroA.IsReady) return;
            float diff = RadToDeg(NormalizeAngle(sf.ZeroA.OffsetRad - b));
            bool useB = MathF.Abs(diff) <= ZeroCrossCheckMaxDeg;
            sf.PhiB = useB ? b : sf.ZeroA.OffsetRad;
            OnStrideZeroEstimated?.Invoke(foot, RadToDeg(sf.PhiB.Value), useB ? sf.Early.SdDeg : sf.ZeroA.SdDeg);
            OnStrideZeroCrossCheck?.Invoke(foot, diff, useB);
        }

        /// <summary>重校准 / 训练开始:清 stance 与步幅状态,零点 φ 保留(只在新 condition 时重估)。</summary>
        public void Reset()
        {
            _leftSampler.Reset();
            _rightSampler.Reset();
            _leftStride.Reset();
            _rightStride.Reset();
        }

        /// <summary>新 condition:除 Reset() 外,零点 φ 也重新估计。</summary>
        public void ResetCondition()
        {
            Reset();
            _leftStride.ResetZero();
            _rightStride.ResetZero();
            _calibrated = false;
        }

        // ── StrideFoot:一只脚的步幅法状态 ────────────────────────────────────
        private sealed class StrideFoot
        {
            public readonly StrideDirectionTracker Tracker = new();
            public readonly StrideZeroEstimator ZeroA = new();   // plan A (pelvis-referenced), cross-check only
            public readonly StrideZeroEstimator Early = new();   // plan B: mean of the first N straight stride directions
            public float? EarlyAnchor;                            // first collected direction (return trips folded to it)
            public float? CandidateB;                             // plan B zero once its N strides are in, rad
            public float? PhiB;                                   // zero in use (B, or A as fallback), rad; null until decided
            public int StridesSinceB;                             // strides since CandidateB, for ZeroCrossCheckTimeoutStrides
            public bool PdUnreliable;                             // pelvis filter restarted before the zero was decided
            public bool NeedsRecalibration;                       // this foot's filter restarted before the zero was decided
            public System.Numerics.Quaternion? RecalibrationRef;  // calibration ref at that moment (a different one = recalibrated)
            public System.Numerics.Quaternion? LastFootRef;

            public void TrackCalibration(System.Numerics.Quaternion? footRef)
            {
                LastFootRef = footRef;
                if (NeedsRecalibration && footRef is { } r && r != RecalibrationRef) ResetZero();
            }

            public void ResetZero()
            {
                ZeroA.Reset(); Early.Reset();
                EarlyAnchor = CandidateB = PhiB = null;
                StridesSinceB = 0;
                PdUnreliable = NeedsRecalibration = false;
                RecalibrationRef = null;
            }
            public StrideResult? Pending;          // 本次着地对应的这一步(着地时给出,抬脚时清除)
            public float? PrevSameDir;             // 着地时快照:本脚上一步的方向(G5)
            public bool OtherStraight;             // 着地时快照:另一只脚最近一步是否直走(G6)
            public float? LastDir;                 // 本脚最近一个积分可信(G1–G4)的步幅方向
            public bool LastStraight;              // 本脚最近一步是否直走(G5),供另一只脚的 G6

            public void OnLanding(StrideResult? stride, bool otherLastStraight)
            {
                if (stride is not StrideResult s) return;
                Pending = s;
                PrevSameDir = LastDir;
                OtherStraight = otherLastStraight;
                LastStraight = StrideGate.IsStraight(s, LastDir);
                if (StrideGate.IsIntegrationValid(s)) LastDir = s.DirectionRad;
            }

            public void Reset()
            {
                Tracker.Reset();
                Pending = null;
                PrevSameDir = LastDir = null;
                OtherStraight = LastStraight = false;
            }
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));

        private static float NormalizeAngle(float rad)
        {
            while (rad > MathF.PI) rad -= 2f * MathF.PI;
            while (rad < -MathF.PI) rad += 2f * MathF.PI;
            return rad;
        }

        private static float RadToDeg(float rad) => rad * (180f / MathF.PI);

        // ── StanceSampler（内嵌私有类）────────────────────────────────────────
        // 单脚状态机：检测 swing→stance 转换，落地后采集，直到脚真正安静下来（或触发硬上限）
        // 再输出一次 FPA。

        private sealed class StanceSampler
        {
            private float _cosSum = 0f;
            private float _sinSum = 0f;
            private int _count = 0;
            private bool _inStance = false;
            private bool _emittedThisStance = false;

            // ── 排除原因追踪：这一步是否已经达到过结算所需帧数，以及当时被什么原因挡住 ──
            private bool _reachedReady = false;
            private StepExclusionReason? _pendingReason = null;

            // ── 数据驱动结算：角速度连续低于阈值的帧数 ──────────────────────────
            private int _quietStreak = 0;

            public void AddFrame(float yaw, float gyroMagRadPerSec, float settleGyroThreshold)
            {
                if (!_inStance)
                {
                    _inStance = true;
                    _emittedThisStance = false;
                    _reachedReady = false;
                    _pendingReason = null;
                    _cosSum = 0f;
                    _sinSum = 0f;
                    _count = 0;
                    _quietStreak = 0;
                }
                _cosSum += MathF.Cos(yaw);
                _sinSum += MathF.Sin(yaw);
                _count++;
                _quietStreak = gyroMagRadPerSec < settleGyroThreshold ? _quietStreak + 1 : 0;
            }

            // 已达最小帧数下限，且（连续 settleQuietFrames 帧真正安静 或 触及硬上限）→ 可以结算。
            private bool HasSettled(int minFramesFloor, int settleQuietFrames, int maxFrames) =>
                _count >= minFramesFloor && (_quietStreak >= settleQuietFrames || _count >= maxFrames);

            /// <summary>每帧调用一次，记录门控状态；已安静但门控未过时，暂存排除原因。</summary>
            public void NoteGate(int minFramesFloor, int settleQuietFrames, int maxFrames,
                                  bool gateOk, StepExclusionReason reasonIfBlocked)
            {
                if (!_inStance || _emittedThisStance || !HasSettled(minFramesFloor, settleQuietFrames, maxFrames)) return;
                _reachedReady = true;
                _pendingReason = gateOk ? null : reasonIfBlocked;
            }

            /// <summary>
            /// 落地→抬脚的转换帧上调用。若这一步曾经达标却从未成功输出过 FPA，
            /// 返回最后一次记录的排除原因；否则返回 null（要么仍在摆动，要么已正常输出）。
            /// </summary>
            public StepExclusionReason? MarkSwing()
            {
                StepExclusionReason? excludedReason = null;
                if (_inStance && !_emittedThisStance && _reachedReady)
                    excludedReason = _pendingReason;

                _inStance = false;
                _reachedReady = false;
                _pendingReason = null;
                return excludedReason;
            }

            /// <summary>当前 stance 已采集部分的圆周均值(未结算也可查看);不在 stance 或无数据时为 null。</summary>
            public float? PeekMean() => _inStance && _count > 0 ? MathF.Atan2(_sinSum, _cosSum) : null;

            public float? TrySettle(int minFramesFloor, int settleQuietFrames, int maxFrames)
            {
                if (!_inStance || _emittedThisStance) return null;
                if (!HasSettled(minFramesFloor, settleQuietFrames, maxFrames)) return null;

                _emittedThisStance = true;
                return MathF.Atan2(_sinSum, _cosSum);
            }

            public void Reset()
            {
                _cosSum = _sinSum = 0f;
                _count = 0;
                _inStance = false;
                _emittedThisStance = false;
                _reachedReady = false;
                _pendingReason = null;
                _quietStreak = 0;
            }
        }
    }
}
