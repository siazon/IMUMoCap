using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace IMUMoCap.Methods
{
    public class IMUUIUpdater
    {
        public IMUUIUpdater()
        {
            Mt = Transpose3x3(M);
            Rcorr = RotationAxisAngle(new Vector3D(0, 1, 0), 180);
        }

        private readonly Matrix3D M = new Matrix3D(
      1, 0, 0, 0,
      0, 0, 1, 0,
      0, -1, 0, 0,
      0, 0, 0, 1);
        private Matrix3D Mt;
        private Matrix3D Rcorr;
        /// <summary>
        /// 把 IMU 四元数转换为可直接赋给 WPF 模型的 MatrixTransform3D。
        /// </summary>
        /// <param name="qImu">IMU 输出的 orientationQuaternion（注意 WPF 构造顺序是 x,y,z,w）</param>
        /// <param name="applyConjugate">
        /// 如果你发现方向整体相反（比如右转变左转、或你之前验证 Conjugate 才更对），设为 true。
        /// </param>
        public MatrixTransform3D CreateTransform(Quaternion qImu, bool applyConjugate = true)
        {
            qImu.Normalize();

            // 很多 Xsens 输出的四元数定义可能是 world->sensor，你渲染期望 sensor->world
            // 你之前测试 Conjugate 会改变对错，所以默认打开
            if (applyConjugate)
                qImu.Conjugate();

            // Rimu: IMU坐标系下的旋转矩阵
            var Rimu = QuaternionToMatrix3x3(qImu);

            // 换基：Rwpf = M * Rimu * M^T
            var Rwpf = Mul(Mul(M, Rimu), Mt);
            Rwpf = Mul(Rcorr, Rwpf);
            return new MatrixTransform3D(Rwpf);
        }
        private Matrix3D RotationAxisAngle(Vector3D axis, double degrees)
        {
            axis.Normalize();
            var rot = new AxisAngleRotation3D(axis, degrees);
            return new RotateTransform3D(rot).Value;
        }
        // --------- helpers ---------

        private Matrix3D QuaternionToMatrix3x3(Quaternion q)
        {
            // WPF Quaternion: (X,Y,Z,W)
            double x = q.X, y = q.Y, z = q.Z, w = q.W;

            double xx = x * x, yy = y * y, zz = z * z;
            double xy = x * y, xz = x * z, yz = y * z;
            double wx = w * x, wy = w * y, wz = w * z;

            // 3x3 旋转块 + 齐次行列
            return new Matrix3D(
                1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0,
                2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0,
                2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0,
                0, 0, 0, 1
            );
        }

        private Matrix3D Mul(Matrix3D a, Matrix3D b)
        {
            var r = a;
            r.Append(b); // r = a * b
            return r;
        }

        private Matrix3D Transpose3x3(Matrix3D m)
        {
            // 只转置 3x3 旋转部分；平移保持 0；齐次为 1
            return new Matrix3D(
                m.M11, m.M21, m.M31, 0,
                m.M12, m.M22, m.M32, 0,
                m.M13, m.M23, m.M33, 0,
                0, 0, 0, 1
            );
        }
    }
}
