using System.IO;
using System.Numerics;
using IMUMoCap.Methods;
using IMUMoCap.Model;

namespace IMUMoCap.Tests.Unit
{
    public class ImuSamplesCsvTests
    {
        [Fact]
        public void MagneticField_RoundTrips_AndOldFilesStillLoad()
        {
            var path = System.IO.Path.Combine(System.IO.Path.GetTempPath(), $"imu_{Guid.NewGuid():N}.csv");
            try
            {
                var csv = new CsvUtil();
                csv.WriteImuSamplesCsv(path, new[]
                {
                    new ImuSampleFrame { PacketId = 1, Role = ImuRole.Pelvis, HasMagneticField = true, MagneticField = new Vector3(0.1f, -0.2f, 0.3f) },
                    new ImuSampleFrame { PacketId = 2, Role = ImuRole.Left },
                });
                var back = csv.ReadImuSamplesCsv(path);
                Assert.True(back[0].HasMagneticField);
                Assert.Equal(-0.2f, back[0].MagneticField.Y, 3);
                Assert.False(back[1].HasMagneticField);

                // 旧格式:23 列、没有 Mag
                File.WriteAllLines(path, new[] { "header", "3,Right,2,,,,,,,,,,,,,,,,,,,,," });
                var old = csv.ReadImuSamplesCsv(path);
                Assert.Single(old);
                Assert.False(old[0].HasMagneticField);
            }
            finally { File.Delete(path); }
        }

        [Fact]
        public void ReceiveTime_RoundTripsWithSubMillisecondPrecision_AndFilesWithoutItStillLoad()
        {
            var path = System.IO.Path.Combine(System.IO.Path.GetTempPath(), $"imu_{Guid.NewGuid():N}.csv");
            try
            {
                var csv = new CsvUtil();
                csv.WriteImuSamplesCsv(path, new[]
                {
                    new ImuSampleFrame { PacketId = 1, Role = ImuRole.Pelvis, ReceivedUtcMs = 1794930173222.4567 },
                    new ImuSampleFrame { PacketId = 2, Role = ImuRole.Left },
                });
                var back = csv.ReadImuSamplesCsv(path);
                Assert.Equal(1794930173222.457, back[0].ReceivedUtcMs!.Value, 3);
                Assert.Null(back[1].ReceivedUtcMs);

                // 加列之前的格式:26 列(有 Mag,没有到达时间)
                File.WriteAllLines(path, new[] { "header", "3,Right,2,,,,,,,,,,,,,,,,,,,,,,," });
                var old = csv.ReadImuSamplesCsv(path);
                Assert.Single(old);
                Assert.Null(old[0].ReceivedUtcMs);
            }
            finally { File.Delete(path); }
        }
    }
}
