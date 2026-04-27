// IMUMoCap.Tests/BundlePlayer.cs
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using IMUMoCap;
using IMUMoCap.Pipeline;

namespace IMUMoCap.Tests
{
    /// <summary>
    /// 从 JSONL 文件读取录制的 ImuFrameBundle，用于集成测试回放。
    /// BundleDto 是 internal，通过 InternalsVisibleTo 对测试项目可见。
    /// </summary>
    public static class BundlePlayer
    {
        public static IEnumerable<ImuFrameBundle> Load(string path)
        {
            foreach (var line in File.ReadLines(path))
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var dto = JsonSerializer.Deserialize<IMUMoCap.Pipeline.BundleDto>(line);
                if (dto != null)
                    yield return dto.ToBundle();
            }
        }
    }
}
