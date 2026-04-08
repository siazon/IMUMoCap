using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IMUMoCap
{
   public enum States
    {
        DETECTING,
        CONNECTING,
        CONNECTED,
        ENABLED,
        OPERATIONAL,
        AWAIT_MEASUREMENT_START,
        MEASURING,
        AWAIT_RECORDING_START,
        RECORDING,
        FLUSHING
    };
    public enum TestState { Launching, Connected,  Calibrating, Calibrated,Baseline, Step };
    public enum ImuRole { Pelvis, Left, Right }

}
