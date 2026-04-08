using IMUMoCap.Model;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;

namespace IMUMoCap.Services
{
    /// <summary>
    /// Thread-safe registry that maps hardware device IDs to pre-declared ImuViewModel slots.
    /// Slots are declared at startup; binding happens on first data packet from a device.
    /// </summary>
    public sealed class ImuSlotRegistry
    {
        private readonly List<ImuViewModel> _imus;
        private readonly ConcurrentDictionary<uint, ImuViewModel> _deviceMap = new();

        public ImuSlotRegistry(IEnumerable<ImuViewModel> imus)
            => _imus = imus.ToList();

        /// <summary>Read-only snapshot of all declared slots (order matches construction order).</summary>
        public IReadOnlyList<ImuViewModel> Imus => _imus;

        /// <summary>
        /// Returns the slot for <paramref name="deviceId"/>.
        /// On first call for a given ID, binds the matching pre-declared slot.
        /// Thread-safe: multiple concurrent callers for the same ID are benign.
        /// </summary>
        public ImuViewModel GetOrAssign(uint deviceId)
        {
            if (_deviceMap.TryGetValue(deviceId, out var cached))
                return cached;

            var vm = _imus.FirstOrDefault(a => a.DeviceId == deviceId)
                ?? throw new InvalidOperationException(
                    $"Unrecognized deviceId {deviceId:X8}. Add it to the Imus list in MainWindow constructor.");

            vm.BindDevice(deviceId);
            // GetOrAdd is atomic: if two threads race, both see the same winner
            return _deviceMap.GetOrAdd(deviceId, vm);
        }

        /// <summary>Returns the slot if already bound, null otherwise.</summary>
        public ImuViewModel? TryGet(uint deviceId)
            => _deviceMap.TryGetValue(deviceId, out var vm) ? vm : null;
    }
}
