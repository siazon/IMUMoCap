using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap.AHRS
{
    public class IMUData
    {
        public IMUData() { }
        public IMUData(double _x, double _y, double _z, double _w)
        {
            x = _x; y = _y; z = _z; w = _w;
        }
        public string PackageId { get; set; }

        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double w { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double aX { get; set; }
        public double aY { get; set; }
        public double aZ { get; set; }
        public double Angle { get; set; }
        public Querternion Q1 { get; set; }
        public Querternion Q2 { get; set; }
        public Querternion Q3 { get; set; }
        public Querternion Q4 { get; set; }
        public Querternion Q5 { get; set; }
        public Querternion Q6 { get; set; }
        public RecoredData[] RecoredDatas = new RecoredData[8];
    }
    public class RecoredData
    {
        public string PackageId { get; set; }
        public Querternion Querternion { get; set; }
        public Vector3 Accelerate { get; set; }
        public Vector3 Orientation { get; set; }
        public Vector3 AHRS { get; set; }
        public Vector3 MadgwickAHRS { get; set; }
    }

    public class Querternion
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double w { get; set; }
    }
}
