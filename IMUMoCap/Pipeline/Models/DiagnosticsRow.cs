// IMUMoCap/Pipeline/Models/DiagnosticsRow.cs
using System;
using System.Numerics;

namespace IMUMoCap.Pipeline.Models
{
    public sealed class DiagnosticsRow
    {
        // ── Timing ────────────────────────────────────────────────────────────
        public long   Timestamp  { get; init; }  // DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        public long   PacketId   { get; init; }

        // ── Pelvis IMU ────────────────────────────────────────────────────────
        public Vector3    PelvisAcc  { get; init; }
        public Vector3    PelvisGyr  { get; init; }
        public Quaternion PelvisQuat { get; init; }

        // ── Left Foot IMU ─────────────────────────────────────────────────────
        public Vector3    LeftAcc    { get; init; }
        public Vector3    LeftGyr    { get; init; }
        public Quaternion LeftQuat   { get; init; }

        // ── Right Foot IMU ────────────────────────────────────────────────────
        public Vector3    RightAcc   { get; init; }
        public Vector3    RightGyr   { get; init; }
        public Quaternion RightQuat  { get; init; }

        // ── GaitEvent ─────────────────────────────────────────────────────────
        public bool LeftStance  { get; init; }
        public bool RightStance { get; init; }
        public bool IsWalking   { get; init; }

        // ── MotionContext ─────────────────────────────────────────────────────
        public ContextState MotionState      { get; init; }
        public float        MotionConfidence { get; init; }

        // ── PdEstimate ────────────────────────────────────────────────────────
        public float PdDirectionDeg { get; init; }
        public float PdStability    { get; init; }
        public bool  PdIsValid      { get; init; }

        // ── FpaResult (NaN when not available this frame) ─────────────────────
        public float FpaLeft_Deg       { get; init; }
        public float FpaLeft_Error     { get; init; }
        public bool  FpaLeft_OnTarget  { get; init; }
        public float FpaRight_Deg      { get; init; }
        public float FpaRight_Error    { get; init; }
        public bool  FpaRight_OnTarget { get; init; }

        // ── CSV header (matches property order above) ─────────────────────────
        public static string CsvHeader =>
            "Timestamp,PacketId," +
            "Pelvis_AccX,Pelvis_AccY,Pelvis_AccZ," +
            "Pelvis_GyrX,Pelvis_GyrY,Pelvis_GyrZ," +
            "Pelvis_QuatW,Pelvis_QuatX,Pelvis_QuatY,Pelvis_QuatZ," +
            "Left_AccX,Left_AccY,Left_AccZ," +
            "Left_GyrX,Left_GyrY,Left_GyrZ," +
            "Left_QuatW,Left_QuatX,Left_QuatY,Left_QuatZ," +
            "Right_AccX,Right_AccY,Right_AccZ," +
            "Right_GyrX,Right_GyrY,Right_GyrZ," +
            "Right_QuatW,Right_QuatX,Right_QuatY,Right_QuatZ," +
            "GaitEvent_LeftStance,GaitEvent_RightStance,GaitEvent_IsWalking," +
            "MotionContext_State,MotionContext_Confidence," +
            "PD_DirectionDeg,PD_Stability,PD_IsValid," +
            "FPA_Left_Deg,FPA_Left_Error,FPA_Left_OnTarget," +
            "FPA_Right_Deg,FPA_Right_Error,FPA_Right_OnTarget";

        public string ToCsvRow() =>
            $"{Timestamp},{PacketId}," +
            $"{PelvisAcc.X:F4},{PelvisAcc.Y:F4},{PelvisAcc.Z:F4}," +
            $"{PelvisGyr.X:F4},{PelvisGyr.Y:F4},{PelvisGyr.Z:F4}," +
            $"{PelvisQuat.W:F6},{PelvisQuat.X:F6},{PelvisQuat.Y:F6},{PelvisQuat.Z:F6}," +
            $"{LeftAcc.X:F4},{LeftAcc.Y:F4},{LeftAcc.Z:F4}," +
            $"{LeftGyr.X:F4},{LeftGyr.Y:F4},{LeftGyr.Z:F4}," +
            $"{LeftQuat.W:F6},{LeftQuat.X:F6},{LeftQuat.Y:F6},{LeftQuat.Z:F6}," +
            $"{RightAcc.X:F4},{RightAcc.Y:F4},{RightAcc.Z:F4}," +
            $"{RightGyr.X:F4},{RightGyr.Y:F4},{RightGyr.Z:F4}," +
            $"{RightQuat.W:F6},{RightQuat.X:F6},{RightQuat.Y:F6},{RightQuat.Z:F6}," +
            $"{LeftStance},{RightStance},{IsWalking}," +
            $"{MotionState},{MotionConfidence:F3}," +
            $"{PdDirectionDeg:F2},{PdStability:F3},{PdIsValid}," +
            $"{(float.IsNaN(FpaLeft_Deg)   ? "" : FpaLeft_Deg.ToString("F2"))}," +
            $"{(float.IsNaN(FpaLeft_Error)  ? "" : FpaLeft_Error.ToString("F2"))}," +
            $"{FpaLeft_OnTarget}," +
            $"{(float.IsNaN(FpaRight_Deg)  ? "" : FpaRight_Deg.ToString("F2"))}," +
            $"{(float.IsNaN(FpaRight_Error) ? "" : FpaRight_Error.ToString("F2"))}," +
            $"{FpaRight_OnTarget}";
    }
}
