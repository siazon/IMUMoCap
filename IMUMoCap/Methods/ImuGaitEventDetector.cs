using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

public class ImuGaitEventDetector
{
    // ====== Sampling ======
    public double Fs = 100.0; // 100Hz

    // ====== Parameters (tune these) ======
    public double GravityEstWindowSec = 0.50; // only used when FreeAcc not present
    public double MinStepIntervalSec = 0.35;  // min distance between HS peaks
    public double HSThreshold = 1.2;          // HS threshold on dyn accel magnitude (m/s^2) - tune!
    public double TO_SearchStartSec = 0.10;   // start searching TO after HS
    public double PitchRateThreshold = 1.5;   // rad/s - tune!

    // ====== Header names (your latest file) ======

    /*
     如果你用的是 FreeAcc 来做 HS（代码会自动选择），通常冲击峰会更明显：
    HSThreshold 先从 1.0~2.0 (m/s²) 试
    MinStepIntervalSec 走得快可调到 0.25~0.30，走得慢可 0.4~0.5
    如果 HS 误检多：提高 HSThreshold 或增大 MinStepIntervalSec
    如果 HS 漏检：降低 HSThreshold
     */
    public string ColCounter = "PacketCounter";

    public string ColAccX = "Acc_X";
    public string ColAccY = "Acc_Y";
    public string ColAccZ = "Acc_Z";

    public string ColFreeE = "FreeAcc_E";
    public string ColFreeN = "FreeAcc_N";
    public string ColFreeU = "FreeAcc_U";

    public string ColQw = "Quat_q0";
    public string ColQx = "Quat_q1";
    public string ColQy = "Quat_q2";
    public string ColQz = "Quat_q3";

    public class Sample
    {
        public long Counter;
        public double T;

        // raw acc (maybe with gravity)
        public double Ax, Ay, Az;

        // free acc (gravity removed) if available
        public bool HasFreeAcc;
        public double Fx, Fy, Fz;

        // quat
        public double Qw, Qx, Qy, Qz;

        // derived
        public double DynAccMag;   // used for HS (either freeAccMag or de-trended accMag)
        public double Pitch;
        public double PitchRate;
    }

    public class GaitEvents
    {
        public List<int> HeelStrikes = new();
        public List<int> ToeOffs = new();
        public int StepCount => HeelStrikes.Count;
    }

    public List<Sample> LoadCsv(string path)
    {
        using var sr = new StreamReader(path);
        var headerLine = sr.ReadLine();
        if (headerLine == null) return new List<Sample>();

        var headers = headerLine.Split(',').Select(h => h.Trim()).ToArray();
        var idx = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < headers.Length; i++)
            if (!idx.ContainsKey(headers[i]))
                idx[headers[i]] = i;

        int GetIndexOrMinus1(string name) => idx.TryGetValue(name, out var i) ? i : -1;
        int GetIndexOrThrow(string name)
        {
            var i = GetIndexOrMinus1(name);
            if (i < 0) throw new InvalidOperationException($"CSV header missing required column: {name}");
            return i;
        }

        int iCounter = GetIndexOrThrow(ColCounter);

        // Detect acceleration source:
        // Prefer FreeAcc_* if present, else use Acc_*.
        int iFreeE = GetIndexOrMinus1(ColFreeE);
        int iFreeN = GetIndexOrMinus1(ColFreeN);
        int iFreeU = GetIndexOrMinus1(ColFreeU);
        bool hasFree = iFreeE >= 0 && iFreeN >= 0 && iFreeU >= 0;

        int iAx = GetIndexOrThrow(ColAccX);
        int iAy = GetIndexOrThrow(ColAccY);
        int iAz = GetIndexOrThrow(ColAccZ);

        int iQw = GetIndexOrThrow(ColQw);
        int iQx = GetIndexOrThrow(ColQx);
        int iQy = GetIndexOrThrow(ColQy);
        int iQz = GetIndexOrThrow(ColQz);

        long? counter0 = null;
        var list = new List<Sample>();

        string? line;
        while ((line = sr.ReadLine()) != null)
        {
            if (string.IsNullOrWhiteSpace(line)) continue;
            var parts = line.Split(',');
            if (parts.Length < headers.Length) continue;

            bool TryParseLong(int c, out long v) =>
                long.TryParse(parts[c].Trim(), NumberStyles.Any, CultureInfo.InvariantCulture, out v);

            bool TryParseDouble(int c, out double v)
            {
                var s = parts[c].Trim();
                if (string.IsNullOrEmpty(s)) { v = double.NaN; return false; }
                return double.TryParse(s, NumberStyles.Any, CultureInfo.InvariantCulture, out v);
            }

            if (!TryParseLong(iCounter, out var counter)) continue;

            if (!TryParseDouble(iAx, out var ax)) continue;
            if (!TryParseDouble(iAy, out var ay)) continue;
            if (!TryParseDouble(iAz, out var az)) continue;

            if (!TryParseDouble(iQw, out var qw)) continue;
            if (!TryParseDouble(iQx, out var qx)) continue;
            if (!TryParseDouble(iQy, out var qy)) continue;
            if (!TryParseDouble(iQz, out var qz)) continue;

            double fe = 0, fn = 0, fu = 0;
            if (hasFree)
            {
                // If any parse fails, treat as not available for this row
                if (!(TryParseDouble(iFreeE, out fe) && TryParseDouble(iFreeN, out fn) && TryParseDouble(iFreeU, out fu)))
                    hasFree = false;
            }

            if (counter0 == null) counter0 = counter;
            var t = (counter - counter0.Value) / Fs;

            list.Add(new Sample
            {
                Counter = counter,
                T = t,
                Ax = ax,
                Ay = ay,
                Az = az,
                HasFreeAcc = hasFree,
                Fx = fe,
                Fy = fn,
                Fz = fu,
                Qw = qw,
                Qx = qx,
                Qy = qy,
                Qz = qz
            });
        }

        return list;
    }

    public GaitEvents Detect(List<Sample> s)
    {
        var outEv = new GaitEvents();
        if (s == null || s.Count < 10) return outEv;

        // 1) Build dyn acceleration magnitude for HS detection
        if (s.All(x => x.HasFreeAcc))
        {
            // Use FreeAcc magnitude directly (best for HS impact)
            for (int i = 0; i < s.Count; i++)
                s[i].DynAccMag = Math.Sqrt(s[i].Fx * s[i].Fx + s[i].Fy * s[i].Fy + s[i].Fz * s[i].Fz);
        }
        else
        {
            // Fallback: use Acc magnitude then de-trend with moving average
            var accMag = new double[s.Count];
            for (int i = 0; i < s.Count; i++)
                accMag[i] = Math.Sqrt(s[i].Ax * s[i].Ax + s[i].Ay * s[i].Ay + s[i].Az * s[i].Az);

            int win = Math.Max(3, (int)Math.Round(GravityEstWindowSec * Fs));
            var mean = MovingAverage(accMag, win);

            for (int i = 0; i < s.Count; i++)
                s[i].DynAccMag = accMag[i] - mean[i];
        }

        // 2) Pitch & pitchRate (from quaternion)
        for (int i = 0; i < s.Count; i++)
            s[i].Pitch = QuaternionToPitch(s[i].Qw, s[i].Qx, s[i].Qy, s[i].Qz);

        for (int i = 1; i < s.Count; i++)
            s[i].PitchRate = (s[i].Pitch - s[i - 1].Pitch) * Fs;
        s[0].PitchRate = s[1].PitchRate;

        // 3) Heel-strike peaks on dyn acceleration
        int minDist = (int)Math.Round(MinStepIntervalSec * Fs);
        outEv.HeelStrikes = FindPeaks(s.Select(x => x.DynAccMag).ToArray(), HSThreshold, minDist);

        // 4) Toe-off: for each HS interval, find pitchRate positive peak after a short delay
        int searchOffset = (int)Math.Round(TO_SearchStartSec * Fs);
        for (int k = 0; k < outEv.HeelStrikes.Count - 1; k++)
        {
            int a = outEv.HeelStrikes[k];
            int b = outEv.HeelStrikes[k + 1];
            int start = Math.Min(a + searchOffset, b - 2);
            if (start >= b - 2) { outEv.ToeOffs.Add(start); continue; }

            int bestIdx = -1;
            double bestVal = double.NegativeInfinity;

            for (int i = Math.Max(start, 1); i < b - 1; i++)
            {
                double v = s[i].PitchRate;
                if (v > PitchRateThreshold &&
                    v > s[i - 1].PitchRate &&
                    v >= s[i + 1].PitchRate)
                {
                    if (v > bestVal) { bestVal = v; bestIdx = i; }
                }
            }

            // fallback: take max pitchRate in the interval
            if (bestIdx < 0)
            {
                for (int i = start; i < b; i++)
                {
                    if (s[i].PitchRate > bestVal)
                    {
                        bestVal = s[i].PitchRate;
                        bestIdx = i;
                    }
                }
            }

            outEv.ToeOffs.Add(bestIdx);
        }

        return outEv;
    }

    // ===== utils =====

    public static double[] MovingAverage(double[] x, int win)
    {
        int n = x.Length;
        var y = new double[n];
        int half = win / 2;

        for (int i = 0; i < n; i++)
        {
            int a = Math.Max(0, i - half);
            int b = Math.Min(n - 1, i + half);
            double sum = 0;
            int cnt = 0;
            for (int j = a; j <= b; j++) { sum += x[j]; cnt++; }
            y[i] = sum / Math.Max(1, cnt);
        }
        return y;
    }

    public static List<int> FindPeaks(double[] x, double threshold, int minDistance)
    {
        var peaks = new List<int>();
        int last = -999999;

        for (int i = 1; i < x.Length - 1; i++)
        {
            if (x[i] > threshold && x[i] > x[i - 1] && x[i] >= x[i + 1])
            {
                if (i - last >= minDistance)
                {
                    peaks.Add(i);
                    last = i;
                }
                else
                {
                    // if too close, keep the stronger peak
                    int lastIdx = peaks.Count - 1;
                    if (lastIdx >= 0 && x[i] > x[peaks[lastIdx]])
                    {
                        peaks[lastIdx] = i;
                        last = i;
                    }
                }
            }
        }
        return peaks;
    }

    // pitch = asin(2*(w*y - z*x))  (radians)
    public static double QuaternionToPitch(double w, double x, double y, double z)
    {
        double sinp = 2.0 * (w * y - z * x);
        sinp = Math.Max(-1.0, Math.Min(1.0, sinp));
        return Math.Asin(sinp);
    }
}


