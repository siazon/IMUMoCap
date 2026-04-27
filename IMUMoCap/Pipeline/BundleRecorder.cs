// IMUMoCap/Pipeline/BundleRecorder.cs
using System;
using System.IO;
using System.Text.Json;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 将 ImuFrameBundle 逐行序列化为 JSONL 文件，用于测试回放。
    /// 用法：pipeline.Recorder = new BundleRecorder(); pipeline.Recorder.Start("output.jsonl");
    /// </summary>
    public sealed class BundleRecorder : IDisposable
    {
        private StreamWriter? _writer;

        public void Start(string path)
        {
            Stop();
            _writer = new StreamWriter(path, append: false, System.Text.Encoding.UTF8);
        }

        public void Stop()
        {
            _writer?.Dispose();
            _writer = null;
        }

        public void Record(ImuFrameBundle bundle)
        {
            if (_writer == null) return;
            var dto = BundleDto.From(bundle);
            _writer.WriteLine(JsonSerializer.Serialize(dto));
        }

        public void Dispose() => Stop();
    }

    // ── DTO（序列化用，与 ImuSampleFrame 字段一一对应）─────────────────────────

    internal sealed class BundleDto
    {
        public long   PacketId { get; set; }
        public double TimeSec  { get; set; }
        public SampleDto[] Samples { get; set; } = Array.Empty<SampleDto>();

        public static BundleDto From(ImuFrameBundle b)
        {
            var samples = new System.Collections.Generic.List<SampleDto>();
            foreach (var (_, s) in b.Samples)
                samples.Add(SampleDto.From(s));
            return new BundleDto { PacketId = b.PacketId, TimeSec = b.TimeSec, Samples = samples.ToArray() };
        }

        public ImuFrameBundle ToBundle()
        {
            var bundle = new ImuFrameBundle(PacketId, TimeSec);
            foreach (var s in Samples)
                bundle.AddOrUpdate(s.ToFrame());
            return bundle;
        }
    }

    internal sealed class SampleDto
    {
        public string Role       { get; set; } = "";
        public uint?  StatusWord { get; set; }

        public bool     HasQ  { get; set; }
        public float[]? Q     { get; set; }   // [X, Y, Z, W]

        public bool     HasGyro { get; set; }
        public float[]? Gyro    { get; set; } // [X, Y, Z]

        public bool     HasAcc    { get; set; }
        public float[]? Acc       { get; set; }

        public bool     HasFreeAcc { get; set; }
        public float[]? FreeAcc    { get; set; }

        public bool     HasDeltaQ { get; set; }
        public float[]? DeltaQ    { get; set; }

        public static SampleDto From(ImuSampleFrame f) => new()
        {
            Role       = f.Role.ToString(),
            StatusWord = f.StatusWord,
            HasQ       = f.HasQuaternion,
            Q          = f.HasQuaternion ? new[] { f.Quaternion.X, f.Quaternion.Y, f.Quaternion.Z, f.Quaternion.W } : null,
            HasGyro    = f.HasRateOfTurn,
            Gyro       = f.HasRateOfTurn ? new[] { f.RateOfTurn.X, f.RateOfTurn.Y, f.RateOfTurn.Z } : null,
            HasAcc     = f.HasAcceleration,
            Acc        = f.HasAcceleration ? new[] { f.Acceleration.X, f.Acceleration.Y, f.Acceleration.Z } : null,
            HasFreeAcc = f.HasFreeAcceleration,
            FreeAcc    = f.HasFreeAcceleration ? new[] { f.FreeAcceleration.X, f.FreeAcceleration.Y, f.FreeAcceleration.Z } : null,
            HasDeltaQ  = f.HasDeltaQ,
            DeltaQ     = f.HasDeltaQ ? new[] { f.DeltaQ.X, f.DeltaQ.Y, f.DeltaQ.Z, f.DeltaQ.W } : null,
        };

        public ImuSampleFrame ToFrame()
        {
            var role = Enum.Parse<ImuRole>(Role);
            var frame = new ImuSampleFrame { Role = role, StatusWord = StatusWord };

            if (HasQ && Q != null)
            {
                frame.HasQuaternion = true;
                frame.Quaternion = new System.Numerics.Quaternion(Q[0], Q[1], Q[2], Q[3]);
            }
            if (HasGyro && Gyro != null)
            {
                frame.HasRateOfTurn = true;
                frame.RateOfTurn = new System.Numerics.Vector3(Gyro[0], Gyro[1], Gyro[2]);
            }
            if (HasAcc && Acc != null)
            {
                frame.HasAcceleration = true;
                frame.Acceleration = new System.Numerics.Vector3(Acc[0], Acc[1], Acc[2]);
            }
            if (HasFreeAcc && FreeAcc != null)
            {
                frame.HasFreeAcceleration = true;
                frame.FreeAcceleration = new System.Numerics.Vector3(FreeAcc[0], FreeAcc[1], FreeAcc[2]);
            }
            if (HasDeltaQ && DeltaQ != null)
            {
                frame.HasDeltaQ = true;
                frame.DeltaQ = new System.Numerics.Quaternion(DeltaQ[0], DeltaQ[1], DeltaQ[2], DeltaQ[3]);
            }
            return frame;
        }
    }
}
