using System;

namespace IMUMoCap.AHRS
{
    public static class AngleCalculaterExtr
    {
        public static double DegreesToRadians(this double degrees)
        {
            return degrees * (Math.PI / 180.0);
        }
    }
}
