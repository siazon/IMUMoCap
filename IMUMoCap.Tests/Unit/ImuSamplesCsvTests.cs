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
    }
}
