using System;
using System.Collections.Generic;
using System.Numerics;
using XDA;

public sealed class PacketBundle
{
    public long PacketId { get; init; }
    public (string deviceId, XsSdiData sdi, XsCalibratedData cal)? Pelvis;
    public (string deviceId, XsSdiData sdi, XsCalibratedData cal)? Left;
    public (string deviceId, XsSdiData sdi, XsCalibratedData cal)? Right;

    public bool IsComplete => Pelvis.HasValue && Left.HasValue && Right.HasValue;
}

public sealed class PacketAggregator
{
    private readonly string _pelvisId, _leftId, _rightId;
    private readonly Dictionary<long, PacketBundle> _buffer = new();
    private readonly int _maxKeep = 30; // 0.3s buffer @ 100Hz

    public PacketAggregator(string pelvisId, string leftId, string rightId)
    {
        _pelvisId = pelvisId; _leftId = leftId; _rightId = rightId;
    }

    public PacketBundle? Add(long packetId, string deviceId, XsSdiData sdi, XsCalibratedData cal)
    {
        if (!_buffer.TryGetValue(packetId, out var b))
        {
            b = new PacketBundle { PacketId = packetId };
            _buffer[packetId] = b;
        }

        if (deviceId == _pelvisId) b.Pelvis = (deviceId, sdi, cal);
        else if (deviceId == _leftId) b.Left = (deviceId, sdi, cal);
        else if (deviceId == _rightId) b.Right = (deviceId, sdi, cal);

        // write back (PacketBundle is class so not necessary, but keep pattern clear)
        _buffer[packetId] = b;

        if (b.IsComplete)
        {
            _buffer.Remove(packetId);
            TrimOld(packetId);
            return b;
        }
        return null;
    }

    private void TrimOld(long newest)
    {
        if (_buffer.Count <= _maxKeep) return;

        // remove the oldest ones (simple approach)
        var keys = new List<long>(_buffer.Keys);
        keys.Sort();
        int removeCount = Math.Max(0, keys.Count - _maxKeep);
        for (int i = 0; i < removeCount; i++)
            _buffer.Remove(keys[i]);
    }
}
