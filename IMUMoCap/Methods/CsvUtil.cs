using IMUMoCap.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text;
using System.Linq;

namespace IMUMoCap.Methods
{
    public sealed class CsvUtil
    {
        private static string Csv(string? value)
        {
            if (string.IsNullOrEmpty(value))
                return "";

            if (!value.Contains(',') && !value.Contains('"') && !value.Contains('\n') && !value.Contains('\r'))
                return value;

            return "\"" + value.Replace("\"", "\"\"") + "\"";
        }

        private static string Csv(bool value) => value ? "1" : "0";
        private static string Csv(float value) => value.ToString("F4", CultureInfo.InvariantCulture);
        private static string Csv(double value) => value.ToString("F4", CultureInfo.InvariantCulture);
        private static string Csv(int? value) => value?.ToString(CultureInfo.InvariantCulture) ?? "";
        private static string Csv(uint? value) => value?.ToString(CultureInfo.InvariantCulture) ?? "";
        private static string Csv(ushort? value) => value?.ToString(CultureInfo.InvariantCulture) ?? "";
        private static string CsvVector(Vector3 value) => $"{Csv(value.X)},{Csv(value.Y)},{Csv(value.Z)}";
        private static string CsvQuaternion(Quaternion value) => $"{Csv(value.X)},{Csv(value.Y)},{Csv(value.Z)},{Csv(value.W)}";

        public List<ImuSampleFrame> ReadImuSamplesCsv(string filePath)
        {
            var samples = new List<ImuSampleFrame>();
            using var reader = new StreamReader(filePath, Encoding.UTF8);
            reader.ReadLine(); // skip header
            string? line;
            while ((line = reader.ReadLine()) != null)
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var c = line.Split(',');
                if (c.Length < 23) continue;

                var s = new ImuSampleFrame
                {
                    PacketId   = long.Parse(c[0], CultureInfo.InvariantCulture),
                    Role       = Enum.Parse<ImuRole>(c[1]),
                    StatusWord = string.IsNullOrEmpty(c[2]) ? null
                                 : uint.Parse(c[2], CultureInfo.InvariantCulture),
                };

                if (!string.IsNullOrEmpty(c[3]))
                {
                    s.HasQuaternion = true;
                    s.Quaternion = new Quaternion(
                        float.Parse(c[3], CultureInfo.InvariantCulture),
                        float.Parse(c[4], CultureInfo.InvariantCulture),
                        float.Parse(c[5], CultureInfo.InvariantCulture),
                        float.Parse(c[6], CultureInfo.InvariantCulture));
                }
                if (!string.IsNullOrEmpty(c[7]))
                {
                    s.HasRateOfTurn = true;
                    s.RateOfTurn = new Vector3(
                        float.Parse(c[7], CultureInfo.InvariantCulture),
                        float.Parse(c[8], CultureInfo.InvariantCulture),
                        float.Parse(c[9], CultureInfo.InvariantCulture));
                }
                if (!string.IsNullOrEmpty(c[10]))
                {
                    s.HasFreeAcceleration = true;
                    s.FreeAcceleration = new Vector3(
                        float.Parse(c[10], CultureInfo.InvariantCulture),
                        float.Parse(c[11], CultureInfo.InvariantCulture),
                        float.Parse(c[12], CultureInfo.InvariantCulture));
                }
                if (!string.IsNullOrEmpty(c[13]))
                {
                    s.HasAcceleration = true;
                    s.Acceleration = new Vector3(
                        float.Parse(c[13], CultureInfo.InvariantCulture),
                        float.Parse(c[14], CultureInfo.InvariantCulture),
                        float.Parse(c[15], CultureInfo.InvariantCulture));
                }
                if (!string.IsNullOrEmpty(c[16]))
                {
                    s.HasDeltaQ = true;
                    s.DeltaQ = new Quaternion(
                        float.Parse(c[16], CultureInfo.InvariantCulture),
                        float.Parse(c[17], CultureInfo.InvariantCulture),
                        float.Parse(c[18], CultureInfo.InvariantCulture),
                        float.Parse(c[19], CultureInfo.InvariantCulture));
                }
                if (!string.IsNullOrEmpty(c[20]))
                {
                    s.HasDeltaV = true;
                    s.DeltaV = new Vector3(
                        float.Parse(c[20], CultureInfo.InvariantCulture),
                        float.Parse(c[21], CultureInfo.InvariantCulture),
                        float.Parse(c[22], CultureInfo.InvariantCulture));
                }
                samples.Add(s);
            }
            return samples;
        }

        public static List<ImuFrameBundle> GroupIntoBundles(List<ImuSampleFrame> samples)
        {
            long firstId = samples.Count > 0 ? samples.Min(s => s.PacketId) : 0;
            var byId = new SortedDictionary<long, ImuFrameBundle>();
            foreach (var s in samples)
            {
                if (!byId.TryGetValue(s.PacketId, out var bundle))
                {
                    bundle = new ImuFrameBundle(s.PacketId, (s.PacketId - firstId) / 100.0);
                    byId[s.PacketId] = bundle;
                }
                bundle.AddOrUpdate(s);
            }
            // Include all bundles (complete and partial). Incomplete bundles will be used
            // for pre-gate stomp detection but rejected by DataQualityGate for full processing.
            return byId.Values.ToList();
        }

        public void WriteImuSamplesCsv(string filePath, IReadOnlyList<ImuSampleFrame> samples)
        {
            var csv = new StringBuilder();
            csv.AppendLine(
                "PacketId,Role,StatusWord," +
                "Qx,Qy,Qz,Qw," +
                "Gx,Gy,Gz," +
                "FreeAx,FreeAy,FreeAz," +
                "Ax,Ay,Az," +
                "DQx,DQy,DQz,DQw," +
                "DVx,DVy,DVz");

            foreach (var sample in samples)
            {
                csv.Append(sample.PacketId.ToString(CultureInfo.InvariantCulture)).Append(',')
                   .Append(sample.Role.ToString()).Append(',')
                   .Append(Csv(sample.StatusWord)).Append(',');

                if (sample.HasQuaternion) csv.Append(CsvQuaternion(sample.Quaternion));
                else csv.Append(",,,");
                csv.Append(',');

                if (sample.HasRateOfTurn) csv.Append(CsvVector(sample.RateOfTurn));
                else csv.Append(",,");
                csv.Append(',');

                if (sample.HasFreeAcceleration) csv.Append(CsvVector(sample.FreeAcceleration));
                else csv.Append(",,");
                csv.Append(',');

                if (sample.HasAcceleration) csv.Append(CsvVector(sample.Acceleration));
                else csv.Append(",,");
                csv.Append(',');

                if (sample.HasDeltaQ) csv.Append(CsvQuaternion(sample.DeltaQ));
                else csv.Append(",,,");
                csv.Append(',');

                if (sample.HasDeltaV) csv.Append(CsvVector(sample.DeltaV));
                else csv.Append(",,");

                csv.AppendLine();
            }

            File.WriteAllText(filePath, csv.ToString(), Encoding.UTF8);
        }

    }
}
