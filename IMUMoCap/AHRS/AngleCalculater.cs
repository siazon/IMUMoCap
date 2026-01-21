using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using XDA;

namespace IMUMoCap.AHRS
{
    public class AngleCalculater
    {

        /// <summary>
        /// 
        /// </summary>
        /// <param name="array_qua">quaternion</param>
        /// <param name="_oricalibrated">角度xyz</param>
        public double GetXYAngle(double[] array_qua, double[] _oricalibrated) {
            QuaternionUtil quaternionUtil= new QuaternionUtil();
            XsQuaternion _qref1 = new XsQuaternion();
            _qref1.assign(array_qua[0], array_qua[1], array_qua[2], array_qua[3]);
            var _dcmQref1 = quaternionUtil._DCMqua(array_qua);
            double[,] _Euler1=new double[3,3];
            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];
            var _Tibia1 = quaternionUtil._EulerAnglesToMatrix(_Euler1, 1);
            var _result1 = quaternionUtil._MultiplyMatrix(_dcmQref1, _Tibia1);
            var _angle1 = quaternionUtil._toDeg(Math.Atan(_result1[0, 1] / _result1[0, 0]));
            return _angle1;
        }

        public double GetXZAngle(double[] array_qua, double[] _oricalibrated)
        {
            QuaternionUtil quaternionUtil = new QuaternionUtil();
            XsQuaternion _qref1 = new XsQuaternion();
            _qref1.assign(array_qua[0], array_qua[1], array_qua[2], array_qua[3]);
            var _dcmQref1 = quaternionUtil._DCMqua(array_qua);
            double[,] _Euler1 = new double[3, 3];
            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];
            var _Tibia1 = quaternionUtil._EulerAnglesToMatrix(_Euler1, 1);
            var _result1 = quaternionUtil._MultiplyMatrix(_dcmQref1, _Tibia1);
            var _angle1 = quaternionUtil._toDeg(Math.Atan(_result1[0, 2] / _result1[0, 0]));
            return _angle1;
        }
        public double GetYZAngle(double[] array_qua, double[] _oricalibrated)
        {
            QuaternionUtil quaternionUtil = new QuaternionUtil();
            XsQuaternion _qref1 = new XsQuaternion();
            _qref1.assign(array_qua[0], array_qua[1], array_qua[2], array_qua[3]);
            var _dcmQref1 = quaternionUtil._DCMqua(array_qua);
            double[,] _Euler1 = new double[3, 3];
            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];
            var _Tibia1 = quaternionUtil._EulerAnglesToMatrix(_Euler1, 1);
            var _result1 = quaternionUtil._MultiplyMatrix(_dcmQref1, _Tibia1);
            var _angle1 = quaternionUtil._toDeg(Math.Atan(_result1[1, 2] / _result1[2, 2]));
            return _angle1;
        }

        public double Dot(IMUData q1, IMUData q2)
        {
            return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
        }

        public double AngleBetweenQuaternions(IMUData q1, IMUData q2)
        {
            double dot = Dot(q1, q2);

            dot = Math.Clamp(dot, -1.0, 1.0);

            double angle = 2.0 * Math.Acos(dot);
            return angle;
        }
        public Vector3 QuaternionToEuler(Quaternion quaternion)
        {
            Vector3 euler = quaternion.ToEulerAngles();
            var q = quaternion;
            double sqw = q.W * q.W;
            double sqx = q.X * q.X;
            double sqy = q.Y * q.Y;
            double sqz = q.Z * q.Z;
            euler.X = (float)Math.Atan2(2f * q.X * q.W + 2f * q.Y * q.Z, 1 - 2f * (sqz + sqw));     // Yaw 
            euler.Y = (float)Math.Asin(2f * (q.X * q.Z - q.W * q.Y));                             // Pitch 
            euler.Z = (float)Math.Atan2(2f * q.X * q.Y + 2f * q.Z * q.W, 1 - 2f * (sqy + sqz));

            var _x = euler.X.ConvertRadiansToDegrees();
            var _y = euler.Y.ConvertRadiansToDegrees();
            var _z = euler.Z.ConvertRadiansToDegrees();
            return euler;
        }
        private XsEuler getEulerByXs(XsQuaternion q)
        {
            XsEuler xsEuler = new XsEuler(q);
            var x = xsEuler.x();
            var y = xsEuler.y();
            var z = xsEuler.z();
            return xsEuler;
        }
        public float[] ConvertToRotationMatrix(float[] Quaternion)
        {
            float num = 2f * Quaternion[0] * Quaternion[0] - 1f + 2f * Quaternion[1] * Quaternion[1];
            float num2 = 2f * (Quaternion[1] * Quaternion[2] + Quaternion[0] * Quaternion[3]);
            float num3 = 2f * (Quaternion[1] * Quaternion[3] - Quaternion[0] * Quaternion[2]);
            float num4 = 2f * (Quaternion[1] * Quaternion[2] - Quaternion[0] * Quaternion[3]);
            float num5 = 2f * Quaternion[0] * Quaternion[0] - 1f + 2f * Quaternion[2] * Quaternion[2];
            float num6 = 2f * (Quaternion[2] * Quaternion[3] + Quaternion[0] * Quaternion[1]);
            float num7 = 2f * (Quaternion[1] * Quaternion[3] + Quaternion[0] * Quaternion[2]);
            float num8 = 2f * (Quaternion[2] * Quaternion[3] - Quaternion[0] * Quaternion[1]);
            float num9 = 2f * Quaternion[0] * Quaternion[0] - 1f + 2f * Quaternion[3] * Quaternion[3];
            return new float[9] { num, num2, num3, num4, num5, num6, num7, num8, num9 };
        }
    }
    public static class AngleCalculaterExtr {
        public static double DegreesToRadians(this double degrees)
        {
            return degrees * (Math.PI / 180.0);
        }
    }
}
