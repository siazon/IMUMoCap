using System.Collections.Generic;
using System.Linq;
using IMUMoCap.Pipeline.Models;
using XDA;

namespace IMUMoCap.Methods
{
    public sealed class ImuFrameCollector
    {
        private readonly Dictionary<long, ImuFrameBundle> _frameBundles = new();
        private long? _firstPacketId;
        private long _lastCompletedPacketId = -1;

        public int SampleRateHz { get; set; } = 100;
        public int MaxPendingPacketLag { get; set; } = 16;

        // ── Gap 统计字段 ──────────────────────────────────────────────────────
        private int _totalFramesSeen    = 0;
        private int _completedFrames    = 0;
        private int _pelvisGapFrames    = 0;
        private int _leftFootGapFrames  = 0;
        private int _rightFootGapFrames = 0;

        public (ImuSampleFrame Sample, ImuFrameBundle? CompletedBundle) Process(ImuRole sensor, uint deviceId, XsDataPacket packet)
        {
            var sample = BuildImuSample(sensor, deviceId, packet);
            var completedBundle = UpsertFrameBundle(sample);
            return (sample, completedBundle?.IsComplete == true ? completedBundle : null);
        }

        private ImuSampleFrame BuildImuSample(ImuRole sensor, uint deviceId, XsDataPacket packet)
        {
            long packetId = packet.packetId();
            _firstPacketId ??= packetId;

            var sample = new ImuSampleFrame
            {
                PacketId = packetId,
                PacketCounter = packet.containsPacketCounter() ? packet.packetCounter() : null,
                TimeSec = (packetId - _firstPacketId.Value) / (double)SampleRateHz,
                DeviceId = deviceId,
                Role = sensor,
                StatusWord = packet.containsStatus() ? packet.status() : null,
                Rssi = packet.containsRssi() ? packet.rssi() : null
            };

            if (packet.containsOrientation())
            {
                sample.HasQuaternion = true;
                sample.Quaternion = Utils.ToNumericsQuaternion(packet.orientationQuaternion());
            }

            if (packet.containsCalibratedGyroscopeData())
            {
                sample.HasRateOfTurn = true;
                sample.RateOfTurn = Utils.ToNumericsVector3(packet.calibratedGyroscopeData());
            }

            if (packet.containsFreeAcceleration())
            {
                sample.HasFreeAcceleration = true;
                sample.FreeAcceleration = Utils.ToNumericsVector3(packet.freeAcceleration());
            }

            if (packet.containsCalibratedAcceleration())
            {
                sample.HasAcceleration = true;
                sample.Acceleration = Utils.ToNumericsVector3(packet.calibratedAcceleration());
            }

            if (packet.containsCalibratedMagneticField())
            {
                sample.HasMagneticField = true;
                sample.MagneticField = Utils.ToNumericsVector3(packet.calibratedMagneticField());
            }

            if (packet.containsSdiData())
            {
                XsSdiData sdiData = packet.sdiData();

                if (packet.containsOrientationIncrement())
                {
                    sample.HasDeltaQ = true;
                    sample.DeltaQ = Utils.ToNumericsQuaternion(sdiData.orientationIncrement());
                }

                if (packet.containsVelocityIncrement())
                {
                    sample.HasDeltaV = true;
                    sample.DeltaV = Utils.ToNumericsVector3(sdiData.velocityIncrement());
                }
            }

            return sample;
        }

        private void CleanupStaleFrameBundles(long currentPacketId)
        {
            if (_frameBundles.Count == 0)
                return;

            var stalePacketIds = _frameBundles
                .Where(kvp => !kvp.Value.IsComplete && currentPacketId - kvp.Key > MaxPendingPacketLag)
                .Select(kvp => kvp.Key)
                .ToList();

            foreach (var stalePacketId in stalePacketIds)
            {
                var staleBundle = _frameBundles[stalePacketId];
                if (staleBundle.Pelvis    == null) _pelvisGapFrames++;
                if (staleBundle.LeftFoot  == null) _leftFootGapFrames++;
                if (staleBundle.RightFoot == null) _rightFootGapFrames++;
                _totalFramesSeen++;
                _frameBundles.Remove(stalePacketId);
            }
        }

        private ImuFrameBundle? UpsertFrameBundle(ImuSampleFrame sample)
        {
            if (sample.PacketId <= _lastCompletedPacketId)
                return null;

            if (!_frameBundles.TryGetValue(sample.PacketId, out var bundle))
            {
                bundle = new ImuFrameBundle(sample.PacketId, sample.TimeSec);
                _frameBundles[sample.PacketId] = bundle;
            }

            bundle.AddOrUpdate(sample);
            CleanupStaleFrameBundles(sample.PacketId);

            if (!bundle.IsComplete)
                return bundle;

            _frameBundles.Remove(sample.PacketId);
            _lastCompletedPacketId = sample.PacketId;
            _completedFrames++;
            return bundle;
        }

        public void Reset()
        {
            _frameBundles.Clear();
            _firstPacketId = null;
            _lastCompletedPacketId = -1;
            _totalFramesSeen = 0;
            _completedFrames = 0;
            _pelvisGapFrames = 0;
            _leftFootGapFrames = 0;
            _rightFootGapFrames = 0;
        }

        public FrameQualityReport GenerateReport()
        {
            return new FrameQualityReport
            {
                TotalFrames        = _totalFramesSeen + _completedFrames,
                CompletedFrames    = _completedFrames,
                PelvisGapFrames    = _pelvisGapFrames,
                LeftFootGapFrames  = _leftFootGapFrames,
                RightFootGapFrames = _rightFootGapFrames,
            };
        }
    }
}
