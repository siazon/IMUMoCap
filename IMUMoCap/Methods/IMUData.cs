using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap.Methods
{
    public class IMUData
    {
        public DateTime Timestamp { get; set; }
        public double Qw { get; set; }  // 四元数 w
        public double Qx { get; set; }  // 四元数 x
        public double Qy { get; set; }  // 四元数 y
        public double Qz { get; set; }  // 四元数 z
    }

    public class GaitEvent
    {
        public DateTime Timestamp { get; set; }
        public GaitEventType EventType { get; set; }
        public double PitchAngle { get; set; }  // 俯仰角（绕X轴）
        public double RollAngle { get; set; }   // 滚转角（绕Y轴）
        public double YawAngle { get; set; }    // 偏航角（绕Z轴）
    }

    public enum GaitEventType
    {
        HeelStrike,  // 脚跟着地
        ToeOff       // 脚尖离地
    }
}
