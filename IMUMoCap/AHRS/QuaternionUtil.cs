using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;
using XDA;

namespace IMUMoCap.AHRS
{
    public class QuaternionUtil
    {
        XsQuaternion _qref1, _qref2, _qref3, _qref4, _qref5, _qref6;
        double[,] _dcmQref1, _dcmQref2, _dcmQref3, _dcmQref4, _dcmQref5, _dcmQref6;
        double[,] _Euler1, _Euler2, _Euler3, _Euler4, _Euler5, _Euler6;
        double[,] _Tibia1, _Tibia2, _Femur1, _Femur2, _Sacrum, _Trunk;
        double[,] _result1, _result2, _result3, _result4, _result5, _result6;
        double _angle1, _angle2, _angle3, _angle4, _angle5, _angle6, _angle7, _angle1A, _angle2A;
        double[] array_qua, _oricalibrated, _accxyz, _quatcalibrated, array_ori;
        double c1, c2, c3, c4, c5, c6, c7;
        int counter;
        private void timer1_Tick(object sender, EventArgs e)
        {

            int i_ori = 0;
            int i_qua = 0;
            double[,] _dcmQref = new double[3, 1];
            string text = "";
            XsVector3 _datas;
            XsQuaternion _quaternion = new XsQuaternion(0, 0, 0, 0);

            _qref1.assign(array_qua[0], array_qua[1], array_qua[2], array_qua[3]); //Sensor 1
            _qref2.assign(array_qua[4], array_qua[5], array_qua[6], array_qua[7]); //Sensor 2
            _qref3.assign(array_qua[8], array_qua[9], array_qua[10], array_qua[11]); //Sensor 3
            _qref4.assign(array_qua[12], array_qua[13], array_qua[14], array_qua[15]); //Sensor 4
            _qref5.assign(array_qua[16], array_qua[17], array_qua[18], array_qua[19]); //Sensor 5
            _qref6.assign(array_qua[20], array_qua[21], array_qua[22], array_qua[23]); //Sensor 6

            _dcmQref1 = _DCMqua(_qref1);
            _dcmQref2 = _DCMqua(_qref2);
            _dcmQref3 = _DCMqua(_qref3);
            _dcmQref4 = _DCMqua(_qref4);
            _dcmQref5 = _DCMqua(_qref5);
            _dcmQref6 = _DCMqua(_qref6);

            _Euler1[0, 0] = _oricalibrated[0];
            _Euler1[1, 0] = _oricalibrated[1];
            _Euler1[2, 0] = _oricalibrated[2];

            _Euler2[0, 0] = _oricalibrated[3];
            _Euler2[1, 0] = _oricalibrated[4];
            _Euler2[2, 0] = _oricalibrated[5];

            _Euler3[0, 0] = _oricalibrated[6];
            _Euler3[1, 0] = _oricalibrated[7];
            _Euler3[2, 0] = _oricalibrated[8];

            _Euler4[0, 0] = _oricalibrated[9];
            _Euler4[1, 0] = _oricalibrated[10];
            _Euler4[2, 0] = _oricalibrated[11];

            _Euler5[0, 0] = _oricalibrated[12];
            _Euler5[1, 0] = _oricalibrated[13];
            _Euler5[2, 0] = _oricalibrated[14];

            _Euler6[0, 0] = _oricalibrated[15];
            _Euler6[1, 0] = _oricalibrated[16];
            _Euler6[2, 0] = _oricalibrated[17];

            _Tibia1 = _EulerAnglesToMatrix(_Euler1, 1);
            _Tibia2 = _EulerAnglesToMatrix(_Euler2, 1);
            _Femur1 = _EulerAnglesToMatrix(_Euler3, 1);
            _Femur2 = _EulerAnglesToMatrix(_Euler4, 1);
            _Sacrum = _EulerAnglesToMatrix(_Euler5, 1);
            _Trunk = _EulerAnglesToMatrix(_Euler6, 1);

            _result1 = _MultiplyMatrix(_dcmQref1, _Tibia1);
            _result2 = _MultiplyMatrix(_dcmQref2, _Tibia2);
            _result3 = _MultiplyMatrix(_dcmQref3, _Femur1);
            _result4 = _MultiplyMatrix(_dcmQref4, _Femur2);
            _result5 = _MultiplyMatrix(_dcmQref5, _Sacrum);
            _result6 = _MultiplyMatrix(_dcmQref6, _Trunk);

            // These are angles from the unit vectors
            _angle1 = _toDeg(Math.Atan(_result1[0, 1] / _result1[0, 0])); // L sensor 1 Tibia angle plane XY
            _angle2 = _toDeg(Math.Atan(_result2[0, 1] / _result2[0, 0])); // R sensor 2 Tibia angle plane XY

            _angle3 = _toDeg(Math.Atan(_result3[0, 2] / _result3[0, 0]) - Math.Atan(_result1[0, 2] / _result1[0, 0])); // L sensor 3 angle plane XZ
            _angle4 = _toDeg(Math.Atan(_result4[0, 2] / _result4[0, 0]) - Math.Atan(_result2[0, 2] / _result2[0, 0])); // L sensor 3 angle plane XZ

            _angle5 = _toDeg(Math.Atan(_result5[0, 2] / _result5[0, 0]) - Math.Atan(_result3[0, 2] / _result3[0, 0])); // M sensor 5 angle plane XZ
            _angle6 = _toDeg(Math.Atan(_result5[0, 2] / _result5[0, 0]) - Math.Atan(_result4[0, 2] / _result4[0, 0])); // M sensor 5 angle plane XZ

            _angle7 = _toDeg(Math.Atan(_result6[0, 2] / _result6[0, 0])); // Msensor 6 angle plane XZ

            _angle1A = _toDeg(Math.Atan(_result1[0, 2] / _result1[0, 0]));
            _angle2A = _toDeg(Math.Atan(_result2[0, 2] / _result2[0, 0]));

            _accxyz[6] = Math.Sqrt(Math.Pow(_accxyz[0], 2) + Math.Pow(_accxyz[1], 2) + Math.Pow(_accxyz[2], 2));
            _accxyz[7] = Math.Sqrt(Math.Pow(_accxyz[3], 2) + Math.Pow(_accxyz[4], 2) + Math.Pow(_accxyz[5], 2));

            Console.WriteLine("[{0}]", string.Join(", ", " Angle XY: " + _angle1A + _angle2A));
            //rtbData.Text = text;

            //_ltAngle.Text = String.Format("{0:0.00} °\n", _angle1);
            //_rtAngle.Text = String.Format("{0:0.00} °\n", _angle2);
            //_lkfAngle.Text = String.Format("{0:0.00} °\n", _angle3);
            //_rkfAngle.Text = String.Format("{0:0.00} °\n", _angle4);
            //_lhfAngle.Text = String.Format("{0:0.00} °\n", _angle5);
            //_rhfAngle.Text = String.Format("{0:0.00} °\n", _angle6);
            //_tlAngle.Text = String.Format("{0:0.00} °\n", _angle7);

            if (counter < 121)
            {
                //_" + _idtextBox.Text
                //+ "_" + _agetextBox.Text + "_" + _gendercomboBox.SelectedItem.ToString() + "_" + _feedbackcomboBox.SelectedItem.ToString() + "_" + _testType +
                //"
                using (StreamWriter writer = new StreamWriter("C:\\Users\\thiag\\OneDrive\\Documentos\\Visual Studio 2017\\TestAIT\\Participant.txt", true))
                {
                    TimeSpan t = DateTime.UtcNow - new DateTime(1970, 1, 1);
                    Int32 unixTimestamp = (Int32)(DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1))).TotalSeconds;
                    long secondsSinceEpoch = (long)t.TotalMilliseconds;
                    Console.WriteLine(secondsSinceEpoch);
                    Console.WriteLine(DateTime.Now.ToString("dd/MM/yyyy hh:mm:ss:fff"));

                    writer.WriteLine(DateTime.Now.ToString("dd/MM/yyyy hh:mm:ss:fff") + "," + unixTimestamp + "," + secondsSinceEpoch + ","
                    + _angle1 + "," + _angle2 + "," + _angle3 + "," + _angle4 + "," + _angle5 + "," + _angle6 + "," + _angle7 + ","
                    + Convert.ToInt32(c1) + "," + Convert.ToInt32(c2) + "," + Convert.ToInt32(c3) + "," + Convert.ToInt32(c4) + "," + Convert.ToInt32(c5) + ","
                    + Convert.ToInt32(c6) + ","
                    + _quatcalibrated[0] + "," + _quatcalibrated[1] + "," + _quatcalibrated[2] + "," + _quatcalibrated[3] + ","
                    + _quatcalibrated[4] + "," + _quatcalibrated[5] + "," + _quatcalibrated[6] + "," + _quatcalibrated[7] + ","
                    + _quatcalibrated[8] + "," + _quatcalibrated[9] + "," + _quatcalibrated[10] + "," + _quatcalibrated[11] + ","
                    + _quatcalibrated[12] + "," + _quatcalibrated[13] + "," + _quatcalibrated[14] + "," + _quatcalibrated[15] + ","
                    + _quatcalibrated[16] + "," + _quatcalibrated[17] + "," + _quatcalibrated[18] + "," + _quatcalibrated[19] + ","
                    + _quatcalibrated[20] + "," + _quatcalibrated[21] + "," + _quatcalibrated[22] + "," + _quatcalibrated[23] + ","
                    + _oricalibrated[0] + "," + _oricalibrated[1] + "," + _oricalibrated[2] + ","
                    + _oricalibrated[3] + "," + _oricalibrated[4] + "," + _oricalibrated[5] + ","
                    + _oricalibrated[6] + "," + _oricalibrated[7] + "," + _oricalibrated[8] + ","
                    + _oricalibrated[9] + "," + _oricalibrated[10] + "," + _oricalibrated[11] + ","
                    + _oricalibrated[12] + "," + _oricalibrated[13] + "," + _oricalibrated[14] + ","
                    + _oricalibrated[15] + "," + _oricalibrated[16] + "," + _oricalibrated[17] + ","
                    + array_qua[0] + "," + array_qua[1] + "," + array_qua[2] + "," + array_qua[3] + ","
                    + array_qua[4] + "," + array_qua[5] + "," + array_qua[6] + "," + array_qua[7] + ","
                    + array_qua[8] + "," + array_qua[9] + "," + array_qua[10] + "," + array_qua[11] + ","
                    + array_qua[12] + "," + array_qua[13] + "," + array_qua[14] + "," + array_qua[15] + ","
                    + array_qua[16] + "," + array_qua[17] + "," + array_qua[18] + "," + array_qua[19] + ","
                    + array_qua[20] + "," + array_qua[21] + "," + array_qua[22] + "," + array_qua[23] + ","
                    + array_ori[0] + "," + array_ori[1] + "," + array_ori[2] + ","
                    + array_ori[3] + "," + array_ori[4] + "," + array_ori[5] + ","
                    + array_ori[6] + "," + array_ori[7] + "," + array_ori[8] + ","
                    + array_ori[9] + "," + array_ori[10] + "," + array_ori[11] + ","
                    + array_ori[12] + "," + array_ori[13] + "," + array_ori[14] + "," + array_ori[15] + "," + array_ori[16] + "," + array_ori[17]);
                }
            }
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
        public double[,] _DCMqua(XsQuaternion a)
        {
            double q0 = a.w(), q1 = a.x(), q2 = a.y(), q3 = a.z();
            double[,] _dcmatrix = new double[3, 3]
            {
                {Math.Pow(q3,2) + Math.Pow(q0,2) - Math.Pow(q1,2) - Math.Pow(q2,2),2*(q0*q1 + q3*q2), 2*(q0*q2 - q3*q1)} ,
                /* initializers for row indexed by 0 */
                {2*(q0*q1 - q3*q2), Math.Pow(q3,2) - Math.Pow(q0,2) + Math.Pow(q1,2)- Math.Pow(q2,2), 2*(q1*q2 + q3*q0)} , 
                /* initializers for row indexed by 1 */
                {2*(q0*q2 - q3*q1), 2*(q1*q2 - q3*q0), Math.Pow(q3,2) -Math.Pow(q0,2) + Math.Pow(q1,2) + Math.Pow(q2,2)}
                /* initializers for row indexed by 2 */
            };
            return _dcmatrix;
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

        public XsQuaternion _Multiquaternion(XsQuaternion a, XsQuaternion b)
        {
            XsQuaternion c = new XsQuaternion(0, 0, 0, 0);
            c.assign(
            a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(), // 1
            a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(), // i
            a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(), // j
            a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w()); // k)

            return c;
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

        public double _DotProduct(double[,] vec1, double[,] vec2)
        {
            if (vec1 == null)
                return 0;

            if (vec2 == null)
                return 0;

            if (vec1.Length != vec2.Length)
                return 0;

            double tVal = 0;
            for (int x = 0; x < 3; x++)
            {
                tVal += vec1[x, 1] * vec2[x, 1];
            }

            return tVal;
        }
    }
}
