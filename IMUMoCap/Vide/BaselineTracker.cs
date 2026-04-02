using System;
using IMUMoCap.Methods;

namespace IMUMoCap
{
    internal sealed class BaselineTracker
    {
        private readonly int _requiredStepsPerFoot;
        private int _leftCount;
        private int _rightCount;
        private double _leftSum;
        private double _rightSum;

        public BaselineTracker(int requiredStepsPerFoot)
        {
            _requiredStepsPerFoot = Math.Max(1, requiredStepsPerFoot);
        }

        public int RequiredStepsPerFoot => _requiredStepsPerFoot;
        public int LeftCount => _leftCount;
        public int RightCount => _rightCount;
        public bool IsComplete => _leftCount >= _requiredStepsPerFoot &&
                                  _rightCount >= _requiredStepsPerFoot;

        public void Reset()
        {
            _leftCount = 0;
            _rightCount = 0;
            _leftSum = 0d;
            _rightSum = 0d;
        }

        public bool TryAddStep(GaitTraining.Gait.StepFpaResult result)
        {
            switch (result.Foot)
            {
                case ImuRole.Left:
                    if (_leftCount >= _requiredStepsPerFoot) return false;
                    _leftCount++;
                    _leftSum += result.FpaDeg;
                    return true;

                case ImuRole.Right:
                    if (_rightCount >= _requiredStepsPerFoot) return false;
                    _rightCount++;
                    _rightSum += result.FpaDeg;
                    return true;

                default:
                    return false;
            }
        }

        public bool TryGetMean(ImuRole foot, out double mean)
        {
            switch (foot)
            {
                case ImuRole.Left:
                    if (_leftCount == 0)
                    {
                        mean = 0d;
                        return false;
                    }
                    mean = _leftSum / _leftCount;
                    return true;

                case ImuRole.Right:
                    if (_rightCount == 0)
                    {
                        mean = 0d;
                        return false;
                    }
                    mean = _rightSum / _rightCount;
                    return true;

                default:
                    mean = 0d;
                    return false;
            }
        }

        public int GetCount(ImuRole foot)
        {
            return foot switch
            {
                ImuRole.Left => _leftCount,
                ImuRole.Right => _rightCount,
                _ => 0
            };
        }

     
    }
}
