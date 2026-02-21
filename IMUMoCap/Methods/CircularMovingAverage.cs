using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap.Methods
{
    public sealed class CircularMovingAverage
    {
        private readonly int _n;
        private readonly Queue<float> _buf = new();
        private double _sinSum, _cosSum;

        public CircularMovingAverage(int n)
        {
            _n = Math.Max(5, n);
        }

        public int Count => _buf.Count;

        public void Reset()
        {
            _buf.Clear();
            _sinSum = 0;
            _cosSum = 0;
        }

        public void Add(float angleRad)
        {
            _buf.Enqueue(angleRad);
            _sinSum += Math.Sin(angleRad);
            _cosSum += Math.Cos(angleRad);

            while (_buf.Count > _n)
            {
                float old = _buf.Dequeue();
                _sinSum -= Math.Sin(old);
                _cosSum -= Math.Cos(old);
            }
        }

        public float MeanRad
        {
            get
            {
                if (_buf.Count == 0) return 0f;
                return (float)Math.Atan2(_sinSum / _buf.Count, _cosSum / _buf.Count);
            }
        }
    }
}
