using System;
using System.Collections.Generic;
using System.Numerics;
using XDA;

public sealed class FootStanceDetector
{
    // --- Config (tune if needed) ---
    private readonly float _fs;                 // sampling rate, e.g., 100
    private readonly int _winN;                 // window length (samples)
    private readonly float _gyroThEnter;        // rad/s
    private readonly float _gyroThExit;         // rad/s (higher -> hysteresis)
    private readonly float _accThEnter;         // m/s^2 (| |a|-g | < th)
    private readonly float _accThExit;          // m/s^2
    private readonly int _minEnterSamples;      // min consecutive "still" samples to enter stance
    private readonly int _minExitSamples;       // min consecutive "moving" samples to exit stance

    // --- State ---
    private readonly RingBuffer _gyroBuf;
    private readonly RingBuffer _accDevBuf;

    private bool _isStance;
    private int _stillCount;
    private int _moveCount;

    // Online estimate of gravity magnitude (helps when sensor scale slightly off)
    private float _gEst = 9.81f;
    private const float GAlpha = 0.01f; // low-pass update rate for g estimate during stance

    public FootStanceDetector(
        float fs = 100f,
        int windowMs = 200,               // typical 150~250ms
        float gyroEnterDeg = 30f,         // enter stance when gyro < 30 deg/s
        float gyroExitDeg = 60f,          // exit stance when gyro > 60 deg/s
        float accEnter = 1.5f,            // | |a|-g | < 1.5 m/s^2 enter
        float accExit = 2.5f,             // | |a|-g | > 2.5 m/s^2 exit
        int minEnterMs = 120,             // need ~120ms stable to enter
        int minExitMs = 80                // need ~80ms moving to exit
    )
    {
        _fs = fs;
        _winN = Math.Max(5, (int)MathF.Round(windowMs * fs / 1000f));

        _gyroThEnter = DegToRad(gyroEnterDeg);
        _gyroThExit = DegToRad(gyroExitDeg);

        _accThEnter = accEnter;
        _accThExit = accExit;

        _minEnterSamples = Math.Max(1, (int)MathF.Round(minEnterMs * fs / 1000f));
        _minExitSamples = Math.Max(1, (int)MathF.Round(minExitMs * fs / 1000f));

        _gyroBuf = new RingBuffer(_winN);
        _accDevBuf = new RingBuffer(_winN);
    }

    /// <summary>
    /// Update stance detector with calibrated accel (m/s^2) and gyro (rad/s).
    /// Returns current IsStance state.
    /// </summary>
    public bool Update(XsVector3 acc, XsVector3 gyr)
    {
        float gyroMag =(float) gyr.cartesianLength() ;                      // rad/s
        float accMag = (float)acc.cartesianLength();                       // m/s^2
        float accDev = MathF.Abs(accMag - _gEst);          // deviation from gravity magnitude

        _gyroBuf.Add(gyroMag);
        _accDevBuf.Add(accDev);

        // Window features: median or mean. Median is more robust to spikes.
        float gyroFeat = _gyroBuf.Median();
        float accFeat = _accDevBuf.Median();

        // Still / moving decisions with hysteresis thresholds
        bool stillNow = (gyroFeat < _gyroThEnter) && (accFeat < _accThEnter);
        bool moveNow = (gyroFeat > _gyroThExit) || (accFeat > _accThExit);

        if (!_isStance)
        {
            if (stillNow)
            {
                _stillCount++;
                if (_stillCount >= _minEnterSamples)
                {
                    _isStance = true;
                    _moveCount = 0;

                    // Update g estimate gently when entering stance
                    _gEst = Lerp(_gEst, accMag, 0.1f);
                }
            }
            else
            {
                _stillCount = 0;
            }
        }
        else
        {
            if (moveNow)
            {
                _moveCount++;
                if (_moveCount >= _minExitSamples)
                {
                    _isStance = false;
                    _stillCount = 0;
                }
            }
            else
            {
                _moveCount = 0;

                // While in stance (and not moving), keep tracking g slowly
                _gEst = Lerp(_gEst, accMag, GAlpha);
            }
        }

        return _isStance;
    }

    public bool IsStance => _isStance;

    // ---- helpers ----
    private static float DegToRad(float deg) => deg * MathF.PI / 180f;
    private static float Lerp(float a, float b, float t) => a + (b - a) * t;

    /// <summary>Simple ring buffer with median/mean.</summary>
    private sealed class RingBuffer
    {
        private readonly float[] _buf;
        private int _idx;
        private int _count;

        public RingBuffer(int capacity)
        {
            _buf = new float[capacity];
        }

        public void Add(float x)
        {
            _buf[_idx] = x;
            _idx = (_idx + 1) % _buf.Length;
            if (_count < _buf.Length) _count++;
        }

        public float Median()
        {
            if (_count == 0) return 0f;
            // Copy current values
            float[] tmp = new float[_count];
            for (int i = 0; i < _count; i++)
                tmp[i] = _buf[i];
            Array.Sort(tmp);
            int mid = tmp.Length / 2;
            return (tmp.Length % 2 == 1) ? tmp[mid] : 0.5f * (tmp[mid - 1] + tmp[mid]);
        }
    }
}
