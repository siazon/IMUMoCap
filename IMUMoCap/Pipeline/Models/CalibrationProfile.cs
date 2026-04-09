// IMUMoCap/Pipeline/Models/CalibrationProfile.cs
using System.Numerics;

namespace IMUMoCap.Pipeline.Models
{
    /// <summary>
    /// 校准结果：三个 IMU 各自在静立时的参考姿态四元数。
    /// 用于后续所有模块消除安装误差。
    /// </summary>
    public sealed class CalibrationProfile
    {
        public Quaternion PelvisRef    { get; init; }
        public Quaternion LeftFootRef  { get; init; }
        public Quaternion RightFootRef { get; init; }
    }
}
