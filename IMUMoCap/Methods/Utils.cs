using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

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
        public static float WrapPi(float a)
        {
            while (a > MathF.PI) a -= 2f * MathF.PI;
            while (a < -MathF.PI) a += 2f * MathF.PI;
            return a;
        }
    }
}
