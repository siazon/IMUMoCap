using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using XDA;

namespace IMUMoCap.Methods
{
    public  class Utils
    {
        public static Quaternion QForHeading(Quaternion qWs, bool useConjugate)
    => useConjugate ? Quaternion.Conjugate(qWs) : qWs;

        public static float HeadingENU_Consistent(Quaternion qWs, Vector3 sensorAxis, bool useConjugate)
        {
            var q = QForHeading(qWs, useConjugate);
            Vector3 fw = Vector3.Transform(sensorAxis, q);
            fw.Z = 0;
            if (fw.LengthSquared() < 1e-8f) return 0f;
            fw = Vector3.Normalize(fw);
            return MathF.Atan2(fw.Y, fw.X);
        }
        public static Quaternion Normalize(Quaternion q)
        {
            float n = MathF.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            if (n < 1e-12f) return Quaternion.Identity;
            float inv = 1f / n;
            return new Quaternion(q.X * inv, q.Y * inv, q.Z * inv, q.W * inv);
        }

        public static Quaternion ApplyRotationOffset(Quaternion source, Quaternion offset)
        {
            return Normalize(offset * source);
        }

        public static Vector3 RotateVector(Vector3 source, Quaternion rotation)
        {
            return Vector3.Transform(source, Normalize(rotation));
        }

        public static float WrapPi(float a)
        {
            while (a > MathF.PI) a -= 2f * MathF.PI;
            while (a < -MathF.PI) a += 2f * MathF.PI;
            return a;
        }
        public static Quaternion ToNumericsQuaternion(XsQuaternion inc)
        {
            // Adjust here if your SDK is WXYZ etc.
            return new Quaternion((float)inc.x(), (float)inc.y(), (float)inc.z(), (float)inc.w());
        }
        public static Vector3 ToNumericsVector3(XsVector v)
        {
           return new Vector3((float)v.value(0), (float)v.value(1), (float)v.value(2));
        }

        public static Vector3 ToNumericsVector3(XsVector3 v)
        {
            return new Vector3((float)v.value(0), (float)v.value(1), (float)v.value(2));
        }
     
    }
}
