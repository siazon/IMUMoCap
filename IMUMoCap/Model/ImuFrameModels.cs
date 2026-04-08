using System.Collections.Generic;
using System.Numerics;

namespace IMUMoCap
{
    public sealed class ImuSampleFrame
    {
        public long PacketId { get; set; }
        public ushort? PacketCounter { get; set; }
        public double TimeSec { get; set; }
        public uint DeviceId { get; set; }
        public ImuRole Role { get; set; }

        public uint? StatusWord { get; set; }
        public int? Rssi { get; set; }

        public bool HasQuaternion { get; set; }
        public Quaternion Quaternion { get; set; } = Quaternion.Identity;

        public bool HasRateOfTurn { get; set; }
        public Vector3 RateOfTurn { get; set; }

        public bool HasFreeAcceleration { get; set; }
        public Vector3 FreeAcceleration { get; set; }

        public bool HasAcceleration { get; set; }
        public Vector3 Acceleration { get; set; }

        public bool HasMagneticField { get; set; }
        public Vector3 MagneticField { get; set; }

        public bool HasDeltaQ { get; set; }
        public Quaternion DeltaQ { get; set; } = Quaternion.Identity;

        public bool HasDeltaV { get; set; }
        public Vector3 DeltaV { get; set; }
    }

    public sealed class ImuFrameBundle
    {
        private readonly Dictionary<ImuRole, ImuSampleFrame> _samples = new();

        public ImuFrameBundle(long packetId, double timeSec)
        {
            PacketId = packetId;
            TimeSec = timeSec;
        }

        public long PacketId { get; }
        public double TimeSec { get; }
        public IReadOnlyDictionary<ImuRole, ImuSampleFrame> Samples => _samples;

        public ImuSampleFrame? Pelvis => TryGet(ImuRole.Pelvis);
        public ImuSampleFrame? LeftFoot => TryGet(ImuRole.Left);
        public ImuSampleFrame? RightFoot => TryGet(ImuRole.Right);

        public bool IsComplete =>
            _samples.ContainsKey(ImuRole.Pelvis) &&
            _samples.ContainsKey(ImuRole.Left) &&
            _samples.ContainsKey(ImuRole.Right);

        public void AddOrUpdate(ImuSampleFrame sample)
        {
            _samples[sample.Role] = sample;
        }

        public ImuSampleFrame? TryGet(ImuRole role)
        {
            return _samples.TryGetValue(role, out var sample) ? sample : null;
        }
    }
}
