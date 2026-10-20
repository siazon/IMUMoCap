using System.Text.Json.Serialization;

namespace IMUMoCap.Model
{
    /// <summary>
    /// One row of imu_sensors.json: a physical sensor's hardware ID and which role (if any)
    /// it's currently bound to. Loaded at startup into ImuSlotRegistry; rewritten on Save Params.
    /// </summary>
    public sealed class ImuSensorConfigEntry
    {
        public string SlotName { get; set; } = "";
        public string DeviceId { get; set; } = ""; // hex string, e.g. "0x00B43CAB"

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ImuRole Role { get; set; } = ImuRole.None;
    }
}
