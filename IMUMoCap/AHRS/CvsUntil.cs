using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap.AHRS
{
    public class CvsUntil
    {
        public List<string[]> ReadCsv(string path, string fileName)
        {
            List<string[]> list = new List<string[]>();
            string line;
            StreamReader stream = Read(path, fileName);
            while ((line = stream.ReadLine()) != null)
            {
                list.Add(line.Split(','));
            }
            stream.Close();
            stream.Dispose();
            return list;
        }
        private StreamReader Read(string path, string fileName)
        {
            if (path == null)
                return null;
            path += fileName;
            if (!File.Exists(path))
                File.CreateText(path);
            return new StreamReader(path);
        }
        public void WriteCVS(string path, string fileName, List<RecoredData> data)
        {
            var csv = new StringBuilder();
            var header = string.Format("PacketId,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g)");
            csv.AppendLine(header);
            foreach (var item in data)
            {
                var first = item.PackageId;
                var second = item.ToString();
                var newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16}", item.PackageId,
                        item.quaternion.X, item.quaternion.Y, item.quaternion.Z, item.quaternion.W,
                    item.Accelerate.X, item.Accelerate.Y, item.Accelerate.Z,
                    item.Orientation.X, item.Orientation.Y, item.Orientation.Z,
                    item.MadgwickAHRS.X, item.MadgwickAHRS.Y, item.MadgwickAHRS.Z,
                    item.AHRS.X,item.AHRS.Y,item.AHRS.Z);
                csv.AppendLine(newLine);
            }

            string filePath = path + fileName;
            File.WriteAllText(filePath, csv.ToString());
        }
    }
}
