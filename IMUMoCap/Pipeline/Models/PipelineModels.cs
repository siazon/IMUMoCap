// IMUMoCap/Pipeline/Models/PipelineModels.cs
using System.Numerics;

namespace IMUMoCap.Pipeline.Models
{
    // ── ValidFrame ────────────────────────────────────────────────────────────
    // ImuFrameBundle 通过 DataQualityGate 后的包装，表示该帧数据可信
    public sealed class ValidFrame
    {
        public ValidFrame(ImuFrameBundle bundle) => Bundle = bundle;
        public ImuFrameBundle Bundle { get; }
        public long   PacketId  => Bundle.PacketId;
        public double TimeSec   => Bundle.TimeSec;
        public ImuSampleFrame Pelvis    => Bundle.Pelvis!;
        public ImuSampleFrame LeftFoot  => Bundle.LeftFoot!;
        public ImuSampleFrame RightFoot => Bundle.RightFoot!;
    }

    // ── GaitEvent ─────────────────────────────────────────────────────────────
    // 每帧的步态事件，由 GaitEventDetector 输出
    public sealed class GaitEvent
    {
        public bool LeftStance      { get; set; }
        public bool RightStance     { get; set; }
        public bool LeftSwing       { get; set; }
        public bool RightSwing      { get; set; }
        public bool IsWalking       { get; set; }
    }

    // ── MotionContext ─────────────────────────────────────────────────────────
    // 运动上下文，由 MotionContextDetector 输出
    public enum ContextState { Straight, Turning, ReacquiringPd }

    public sealed class MotionContext
    {
        public ContextState State      { get; set; }
        public float        Confidence { get; set; }  // 0–1
    }

    // ── PdEstimate ────────────────────────────────────────────────────────────
    // 行进方向估计，由 ProgressionDirEstimator 输出
    public sealed class PdEstimate
    {
        public float DirectionRad { get; set; }
        public bool  IsValid      { get; set; }
        public float Stability    { get; set; }  // 0–1，方差归一化倒数
    }

    // ── FpaResult ─────────────────────────────────────────────────────────────
    // 步级 FPA 结果，由 FpaEngine 输出（每步一个）
    public sealed class FpaResult
    {
        public long  PacketId    { get; set; }
        public float Fpa_L       { get; set; }  // 度，正值=toe-out（Fpa_R 已在 FpaEngine 取反，左右脚同一约定）
        public float Fpa_R       { get; set; }
        // Fpa_L/R 是用于反馈的值;下面两组分别是两种方法各自的结果(NaN = 该方法这一步没有结果),供记录对比。
        public float FpaPd_L     { get; set; } = float.NaN;  // 当前方法:脚偏航 − PD
        public float FpaPd_R     { get; set; } = float.NaN;
        public float FpaStride_L { get; set; } = float.NaN;  // 步幅法:脚偏航 − 本脚步幅方向 − φ
        public float FpaStride_R { get; set; } = float.NaN;
        public bool  OnTarget_L  { get; set; }
        public bool  OnTarget_R  { get; set; }
        public float Error_L     { get; set; }  // FPA - Target，带符号，供AR反馈
        public float Error_R     { get; set; }
        public float Tolerance_L { get; set; }  // NaN if no baseline
        public float Tolerance_R { get; set; }

        // ── 质量标签（gate 通过时的原始置信度，供事后按需筛选低质量 step）────
        public float  ContextConfidence { get; set; } // MotionContext.Confidence
        public float  PdStability       { get; set; } // PdEstimate.Stability
        public string Quality           { get; set; } = ""; // "High" | "Marginal"
    }

    // ── StepExclusionReason ───────────────────────────────────────────────────
    // 一个已完成的 stance 周期（真实落地）因门控未通过而没有输出 FpaResult 的原因
    // StrideInvalid / StrideNotStraight:步幅法门控(StrideGate G1–G4、G8 / G5–G6)
    public enum StepExclusionReason { Turning, ReacquiringPd, LowConfidence, PdInvalid, StrideInvalid, StrideNotStraight }

    // ── FrameQualityReport ────────────────────────────────────────────────────
    // 实验结束后的帧质量报告，由 ImuFrameCollector 生成
    public sealed class FrameQualityReport
    {
        public int TotalFrames        { get; set; }
        public int CompletedFrames    { get; set; }
        public int PelvisGapFrames    { get; set; }
        public int LeftFootGapFrames  { get; set; }
        public int RightFootGapFrames { get; set; }
        public float CompletionRate   => TotalFrames == 0 ? 0f :
            (float)CompletedFrames / TotalFrames;
    }
}
