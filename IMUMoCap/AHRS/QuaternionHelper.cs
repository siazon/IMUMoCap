using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap.AHRS
{
    public class QuaternionHelper
    {
 
        public double GetYawAngle(double[] array_qua, double[] _oricalibrated)
        {
            var _dcmQref1 = _DCMqua(array_qua);
            double[,] _Euler1 = new double[3, 3];
            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];
            var _Tibia1 = _EulerAnglesToMatrix(_Euler1, 1);
            var _result1 = _MultiplyMatrix(_dcmQref1, _Tibia1);
            var _angle1 = _toDeg(Math.Atan(_result1[0, 1] / _result1[0, 0]));
            return _angle1;
        }

        public double GetPitchAngle(double[] array_qua, double[] _oricalibrated)
        {
            var _dcmQref1 = _DCMqua(array_qua);
            double[,] _Euler1 = new double[3, 3];
            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];
            var _Tibia1 = _EulerAnglesToMatrix(_Euler1, 1);
            var _result1 = _MultiplyMatrix(_dcmQref1, _Tibia1);
            var _angle1 = _toDeg(Math.Atan(_result1[0, 2] / _result1[0, 0]));
            return _angle1;
        }
        public double GetRollAngle(double[] array_qua, double[] _oricalibrated)
        {
            var _dcmQref1 = _DCMqua(array_qua);
            double[,] _Euler1 = new double[3, 3];
            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];
            var _Tibia1 = _EulerAnglesToMatrix(_Euler1, 1);
            var _result1 = _MultiplyMatrix(_dcmQref1, _Tibia1);
            var _angle1 = _toDeg(Math.Atan(_result1[1, 2] / _result1[2, 2]));
            return _angle1;
        }
        public double[,] _DCMqua(double[] a)
        {
            double q0 = a[0], q1 = a[1], q2 = a[2], q3 = a[3];
            double[,] _dcmatrix = new double[3, 3] {
            {Math.Pow(q3+q0-q1-q2,2),2*(q0*q1 + q3*q2), 2*(q0*q2 - q3*q1) },
            {2*(q0*q1 - q3*q2),Math.Pow(q3-q0+q1-q2,2),2*(q1*q2 + q3*q0) },
            {2*(q0*q2 - q3*q1), 2*(q1*q2 - q3*q0),Math.Pow(q3-q0+q1+q2,2) }
            };
            return _dcmatrix;
        }
        public double[,] _EulerAnglesToMatrix(double[,] EulerAngle, int EulerOrder)
        {
            // Convert Euler Angles passed in a vector of Radians
            // into a rotation matrix. The individual Euler Angles are
            // processed in the order requested.
            //_toRad(EulerAngle[0, 0]);
            double[,] Mx = new double[3, 3];
            double Sx = Math.Sin(_toRad(EulerAngle[0, 0]));
            double Sy = Math.Sin(_toRad(EulerAngle[1, 0]));
            double Sz = Math.Sin(_toRad(EulerAngle[2, 0]));

            double Cx = Math.Cos(_toRad(EulerAngle[0, 0]));
            double Cy = Math.Cos(_toRad(EulerAngle[1, 0]));
            double Cz = Math.Cos(_toRad(EulerAngle[2, 0]));
            //1 ORDER_XYZ,
            //2 ORDER_YZX,
            //3 ORDER_ZXY,
            //4 ORDER_ZYX,
            //5 ORDER_YXZ,
            //6 ORDER_XZY

            switch (EulerOrder)
            {
                case 1:
                    Mx[0, 0] = Cy * Cz;
                    Mx[0, 1] = -Cy * Sz;
                    Mx[0, 2] = Sy;
                    Mx[1, 0] = Cz * Sx * Sy + Cx * Sz;
                    Mx[1, 1] = Cx * Cz - Sx * Sy * Sz;
                    Mx[1, 2] = -Cy * Sx;
                    Mx[2, 0] = -Cx * Cz * Sy + Sx * Sz;
                    Mx[2, 1] = Cz * Sx + Cx * Sy * Sz;
                    Mx[2, 2] = Cx * Cy;
                    break;

                case 2:
                    Mx[0, 0] = Cy * Cz;
                    Mx[0, 1] = Sx * Sy - Cx * Cy * Sz;
                    Mx[0, 2] = Cx * Sy + Cy * Sx * Sz;
                    Mx[1, 0] = Sz;
                    Mx[1, 1] = Cx * Cz;
                    Mx[1, 2] = -Cz * Sx;
                    Mx[2, 0] = -Cz * Sy;
                    Mx[2, 1] = Cy * Sx + Cx * Sy * Sz;
                    Mx[2, 2] = Cx * Cy - Sx * Sy * Sz;
                    break;

                case 3:
                    Mx[0, 0] = Cy * Cz - Sx * Sy * Sz;
                    Mx[0, 1] = -Cx * Sz;
                    Mx[0, 2] = Cz * Sy + Cy * Sx * Sz;
                    Mx[1, 0] = Cz * Sx * Sy + Cy * Sz;
                    Mx[1, 1] = Cx * Cz;
                    Mx[1, 2] = -Cy * Cz * Sx + Sy * Sz;
                    Mx[2, 0] = -Cx * Sy;
                    Mx[2, 1] = Sx;
                    Mx[2, 2] = Cx * Cy;
                    break;

                case 4:
                    Mx[0, 0] = Cy * Cz;
                    Mx[0, 1] = Cz * Sx * Sy - Cx * Sz;
                    Mx[0, 2] = Cx * Cz * Sy + Sx * Sz;
                    Mx[1, 0] = Cy * Sz;
                    Mx[1, 1] = Cx * Cz + Sx * Sy * Sz;
                    Mx[1, 2] = -Cz * Sx + Cx * Sy * Sz;
                    Mx[2, 0] = -Sy;
                    Mx[2, 1] = Cy * Sx;
                    Mx[2, 2] = Cx * Cy;
                    break;

                case 5:
                    Mx[0, 0] = Cy * Cz + Sx * Sy * Sz;
                    Mx[0, 1] = Cz * Sx * Sy - Cy * Sz;
                    Mx[0, 2] = Cx * Sy;
                    Mx[1, 0] = Cx * Sz;
                    Mx[1, 1] = Cx * Cz;
                    Mx[1, 2] = -Sx;
                    Mx[2, 0] = -Cz * Sy + Cy * Sx * Sz;
                    Mx[2, 1] = Cy * Cz * Sx + Sy * Sz;
                    Mx[2, 2] = Cx * Cy;
                    break;

                case 6:
                    Mx[0, 0] = Cy * Cz;
                    Mx[0, 1] = -Sz;
                    Mx[0, 2] = Cz * Sy;
                    Mx[1, 0] = Sx * Sy + Cx * Cy * Sz;
                    Mx[1, 1] = Cx * Cz;
                    Mx[1, 2] = -Cy * Sx + Cx * Sy * Sz;
                    Mx[2, 0] = -Cx * Sy + Cy * Sx * Sz;
                    Mx[2, 1] = Cz * Sx;
                    Mx[2, 2] = Cx * Cy + Sx * Sy * Sz;
                    break;
            }
            return Mx;
        }
        public double _toRad(double _degrees)
        {
            return (Math.PI / 180) * _degrees;
        }

        public double _toDeg(double _radians)
        {
            double degrees = (180 / Math.PI) * _radians;
            return degrees;
        }
        public double[,] _MultiplyMatrix(double[,] A, double[,] B)
        {
            int rA = A.GetLength(0);
            int cA = A.GetLength(1);
            int rB = B.GetLength(0);
            int cB = B.GetLength(1);
            double temp = 0;
            double[,] kHasil = new double[rA, cB];
            if (cA != rB)
            {
                Console.WriteLine("matrik can't be multiplied !!");
                return A;
            }
            else
            {
                for (int i = 0; i < rA; i++)
                {
                    for (int j = 0; j < cB; j++)
                    {
                        temp = 0;
                        for (int k = 0; k < cA; k++)
                        {
                            temp += A[i, k] * B[k, j];
                        }
                        kHasil[i, j] = temp;
                    }
                }
                return kHasil;
            }
        }
        public static double ToRad(double degrees)
        {
            return (Math.PI / 180.0) * degrees;
        }

        public  double[] EulerToQuaternion(double yaw, double pitch, double roll)
        {
            // Convert degrees to radians
            yaw = ToRad(yaw);
            pitch = ToRad(pitch);
            roll = ToRad(roll);

            // Calculate trigonometric values
            double cy = Math.Cos(yaw * 0.5);
            double sy = Math.Sin(yaw * 0.5);
            double cp = Math.Cos(pitch * 0.5);
            double sp = Math.Sin(pitch * 0.5);
            double cr = Math.Cos(roll * 0.5);
            double sr = Math.Sin(roll * 0.5);

            // Compute quaternion components
            double q0 = cr * cp * cy + sr * sp * sy;
            double q1 = sr * cp * cy - cr * sp * sy;
            double q2 = cr * sp * cy + sr * cp * sy;
            double q3 = cr * cp * sy - sr * sp * cy;

            return new double[] { q0, q1, q2, q3 };
        }
    }
}
