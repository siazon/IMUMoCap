using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap
{
    public class DeviceModel : INotifyPropertyChanged
    {
        private string _deviceName;

        public string DeviceName
        {
            get { return _deviceName; }
            set
            {
                _deviceName = value;
                OnPropertyChanged(nameof(DeviceName));
            }
        }
        private string _xsTime;

        public string XsTime
        {
            get { return _xsTime; }
            set
            {
                _xsTime = value;
                OnPropertyChanged(nameof(XsTime));
            }
        }
        private double x;

        public double X
        {
            get { return x; }
            set
            {
                x = value;
                OnPropertyChanged(nameof(X));
            }
        }
        private double y;

        public double Y
        {
            get { return y; }
            set
            {
                y = value;
                OnPropertyChanged(nameof(Y));
            }
        }
        private double z;

        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
        public double Z
        {
            get { return z; }
            set
            {
                z = value;
                OnPropertyChanged(nameof(Z));
            }
        }
        public string packetId { get; set; }

        private double angle;


        public double Angle
        {
            get { return angle; }
            set
            {
                angle = value;
                OnPropertyChanged(nameof(Angle));
            }
        }
        private double angleXZ;


        public double AngleXZ
        {
            get { return angleXZ; }
            set
            {
                angleXZ = value;
                OnPropertyChanged(nameof(AngleXZ));
            }
        }
        private double angleYZ;


        public double AngleYZ
        {
            get { return angleYZ; }
            set
            {
                angleYZ = value;
                OnPropertyChanged(nameof(AngleYZ));
            }
        }
    }
}
