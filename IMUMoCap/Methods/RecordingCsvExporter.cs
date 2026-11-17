// IMUMoCap/Methods/RecordingCsvExporter.cs
using System.Collections.Generic;
using System.IO;
using System.Text;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Methods
{
    /// <summary>
    /// 一次录制的三个分析用 CSV(与手动 Save 按钮的文件格式、命名相同):
    /// ImuSamples_{stamp}.csv、Diagnostics_{stamp}.csv、Diagnostics_{stamp}_Events.csv(没有事件时不写)。
    /// 同一个 stamp 再保存一次会覆盖(Retention 结束先存一次,End Recording 再存完整版)。
    /// </summary>
    public static class RecordingCsvExporter
    {
        public sealed record Paths(string ImuSamples, string Diagnostics, string? Events);

        public static Paths Save(string dir, string stamp, IReadOnlyList<ImuSampleFrame> samples,
                                 IReadOnlyList<DiagnosticsRow> rows, ExperimentRecorder recorder)
        {
            Directory.CreateDirectory(dir);
            string imuPath = Path.Combine(dir, $"ImuSamples_{stamp}.csv");
            string diagPath = Path.Combine(dir, $"Diagnostics_{stamp}.csv");

            new CsvUtil().WriteImuSamplesCsv(imuPath, samples);

            using (var sw = new StreamWriter(diagPath, append: false, Encoding.UTF8))
            {
                sw.WriteLine(DiagnosticsRow.CsvHeader);
                foreach (var row in rows) sw.WriteLine(row.ToCsvRow());
            }

            string? eventsPath = null;
            if (recorder.Events.Count > 0)
            {
                eventsPath = Path.Combine(dir, $"Diagnostics_{stamp}_Events.csv");
                recorder.WriteEventsCsv(eventsPath);
            }
            return new Paths(imuPath, diagPath, eventsPath);
        }
    }
}
