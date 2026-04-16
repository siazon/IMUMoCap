using IMUMoCap.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text;

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
