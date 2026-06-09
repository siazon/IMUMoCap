using IMUMoCap.Model;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace IMUMoCap
{
    public partial class MainPageVM : INotifyPropertyChanged
    {
        #region

        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        private ObservableCollection<string> _devices = new ObservableCollection<string>();

        public ObservableCollection<string> Devices
        {
            get { return _devices; }
            set
            {
                _devices = value;

                OnPropertyChanged(nameof(Devices));
            }
        }

        private ObservableCollection<string> _logs = new ObservableCollection<string>();

        public ObservableCollection<string> Logs
        {
            get { return _logs; }
            set
            {
                _logs = value;

                OnPropertyChanged(nameof(Logs));
            }
        }
        private Transform3D _transform3D;

        public Transform3D Transform3D
        {
            get { return _transform3D; }
            set
            {
                _transform3D = value;
                OnPropertyChanged(nameof(Transform3D));
            }
        }

        private string _logList;

        public string LogList
        {
            get { return _logList; }
            set
            {
                _logList = value;
                OnPropertyChanged(nameof(LogList));
            }
        }
        private bool _isScrollToEnd = true;

        public bool IsScrollerToEnd
        {
            get { return _isScrollToEnd; }
            set
            {
                _isScrollToEnd = value;
                OnPropertyChanged(nameof(IsScrollerToEnd));
            }
        }


        private ObservableCollection<string> _UpdateRates = new ObservableCollection<string>();

        public ObservableCollection<string> UpdateRates
        {
            get { return _UpdateRates; }
            set
            {
                _UpdateRates = value;
                OnPropertyChanged(nameof(UpdateRates));
            }
        }
        private int _selectedRate;

        public int SelectedRate
        {
            get { return _selectedRate; }
            set
            {
                _selectedRate = value;
                OnPropertyChanged(nameof(SelectedRate));
            }
        }
        private ObservableCollection<string> _connectedMtws = new ObservableCollection<string>();

        public ObservableCollection<string> ConnectedMtws
        {
            get { return _connectedMtws; }
            set
            {
                _connectedMtws = value;
                OnPropertyChanged(nameof(ConnectedMtws));
            }
        }
        private string _error;

        public string Error
        {
            get { return _error; }
            set
            {
                _error = value;
                OnPropertyChanged(nameof(Error));
            }
        }


        private ObservableCollection<DeviceModel> _deviceModels = new ObservableCollection<DeviceModel>();

        public ObservableCollection<DeviceModel> DeviceModels
        {
            get { return _deviceModels; }
            set
            {
                _deviceModels = value;
                OnPropertyChanged(nameof(DeviceModels));
            }
        }


        private int _selectedMtw;

        public int SelectedMtw
        {
            get { return _selectedMtw; }
            set
            {
                _selectedMtw = value;
                OnPropertyChanged(nameof(SelectedMtw));
            }
        }

        private string _btnConnect;

        public string ConnectBtn
        {
            get { return _btnConnect; }
            set { _btnConnect = value; OnPropertyChanged(nameof(ConnectBtn)); }
        }

        private string _xsTime;

        public string XsTime
        {
            get { return _xsTime; }
            set { _xsTime = value; OnPropertyChanged(nameof(XsTime)); }
        }


        private States _state;


        public States DeviceState
        {
            get { return _state; }
            set
            {
                _state = value;
                OnPropertyChanged(nameof(DeviceState));
            }
        }

        private string _statusLabel;


        public string StatusLabel
        {
            get { return _statusLabel; }
            set
            {
                _statusLabel = value;
                OnPropertyChanged(nameof(StatusLabel));
            }
        }

        private string _CalibrationState = "Not calibrated";

        public string CalibrationState
        {
            get { return _CalibrationState; }
            set
            {
                _CalibrationState = value;
                OnPropertyChanged(nameof(CalibrationState));
            }
        }

        private float _ProgDirDeg;

        public float ProgDirDeg
        {
            get { return _ProgDirDeg; }
            set
            {
                _ProgDirDeg = value;
                OnPropertyChanged(nameof(ProgDirDeg));
            }
        }

        private int _PelvisRejectStreak;

        public int PelvisRejectStreak
        {
            get { return _PelvisRejectStreak; }
            set
            {
                _PelvisRejectStreak = value;
                OnPropertyChanged(nameof(PelvisRejectStreak));
            }
        }

        private string _StanceSummary;

        public string StanceSummary
        {
            get { return _StanceSummary; }
            set
            {
                _StanceSummary = value;
                OnPropertyChanged(nameof(StanceSummary));
            }
        }

        private float _LeftFpaDeg;

        public float LeftFpaDeg
        {
            get { return _LeftFpaDeg; }
            set
            {
                _LeftFpaDeg = value;
                OnPropertyChanged(nameof(LeftFpaDeg));
            }
        }

        private float _RightFpaDeg;

        public float RightFpaDeg
        {
            get { return _RightFpaDeg; }
            set
            {
                _RightFpaDeg = value;
                OnPropertyChanged(nameof(RightFpaDeg));
            }
        }


        private string _LeftFpaNote = "";
        public string LeftFpaNote
        {
            get { return _LeftFpaNote; }
            set { _LeftFpaNote = value; OnPropertyChanged(nameof(LeftFpaNote)); }
        }

        private string _RightFpaNote = "";
        public string RightFpaNote
        {
            get { return _RightFpaNote; }
            set { _RightFpaNote = value; OnPropertyChanged(nameof(RightFpaNote)); }
        }

        // ── FPA card target labels & background colors ────────────────────────

        internal static readonly Brush DefaultCardBg  = MakeBrush(0x11, 0x00, 0x00, 0x00);
        internal static readonly Brush OnTargetBg     = MakeBrush(0x55, 0x00, 0xCC, 0x44);
        internal static readonly Brush OffTargetBg    = MakeBrush(0x55, 0xCC, 0x22, 0x00);
        private static Brush MakeBrush(byte a, byte r, byte g, byte b)
        {
            var br = new SolidColorBrush(Color.FromArgb(a, r, g, b));
            br.Freeze();
            return br;
        }

        private string _leftFpaTarget = "";
        public string LeftFpaTarget
        {
            get => _leftFpaTarget;
            set { _leftFpaTarget = value; OnPropertyChanged(nameof(LeftFpaTarget)); }
        }

        private string _rightFpaTarget = "";
        public string RightFpaTarget
        {
            get => _rightFpaTarget;
            set { _rightFpaTarget = value; OnPropertyChanged(nameof(RightFpaTarget)); }
        }

        private Brush _leftFpaBackground = DefaultCardBg;
        public Brush LeftFpaBackground
        {
            get => _leftFpaBackground;
            set { _leftFpaBackground = value; OnPropertyChanged(nameof(LeftFpaBackground)); }
        }

        private Brush _rightFpaBackground = DefaultCardBg;
        public Brush RightFpaBackground
        {
            get => _rightFpaBackground;
            set { _rightFpaBackground = value; OnPropertyChanged(nameof(RightFpaBackground)); }
        }

        // ── Pipeline tunable parameters (bound to right-panel sliders) ────────

        private float _stompThreshold = 15f;
        public float StompThreshold
        {
            get { return _stompThreshold; }
            set { _stompThreshold = value; OnPropertyChanged(nameof(StompThreshold)); }
        }

        private float _staticGyroThreshold = 0.3f;
        public float StaticGyroThreshold
        {
            get { return _staticGyroThreshold; }
            set { _staticGyroThreshold = value; OnPropertyChanged(nameof(StaticGyroThreshold)); }
        }

        private float _stanceFreeAccThreshold = 2.5f;
        public float StanceFreeAccThreshold
        {
            get { return _stanceFreeAccThreshold; }
            set { _stanceFreeAccThreshold = value; OnPropertyChanged(nameof(StanceFreeAccThreshold)); }
        }

        private float _stanceGyroThreshold = 1.0f;
        public float StanceGyroThreshold
        {
            get { return _stanceGyroThreshold; }
            set { _stanceGyroThreshold = value; OnPropertyChanged(nameof(StanceGyroThreshold)); }
        }

        private float _pdConfidenceThreshold = 0.7f;
        public float PdConfidenceThreshold
        {
            get { return _pdConfidenceThreshold; }
            set { _pdConfidenceThreshold = value; OnPropertyChanged(nameof(PdConfidenceThreshold)); }
        }

        private float _pdStabilityThreshold = 0.7f;
        public float PdStabilityThreshold
        {
            get { return _pdStabilityThreshold; }
            set { _pdStabilityThreshold = value; OnPropertyChanged(nameof(PdStabilityThreshold)); }
        }

        private int _minBaselineSteps = 5;
        public int MinBaselineSteps
        {
            get { return _minBaselineSteps; }
            set { _minBaselineSteps = value; OnPropertyChanged(nameof(MinBaselineSteps)); }
        }

        // ── Panel visibility ──────────────────────────────────────────────────

        private bool _logPanelVisible = true;
        public bool LogPanelVisible
        {
            get { return _logPanelVisible; }
            set { _logPanelVisible = value; OnPropertyChanged(nameof(LogPanelVisible)); }
        }

        private bool _paramsPanelVisible = true;
        public bool ParamsPanelVisible
        {
            get { return _paramsPanelVisible; }
            set { _paramsPanelVisible = value; OnPropertyChanged(nameof(ParamsPanelVisible)); }
        }

        private bool _RotationByDegree = true;

        public bool RotationByDegree
        {
            get { return _RotationByDegree; }
            set
            {
                _RotationByDegree = value;
                OnPropertyChanged(nameof(RotationByDegree));
            }
        }

        //public ObservableCollection<ISeries> Series { get; set; } = new ObservableCollection<ISeries>();
        private IMUData _imuData;

        public IMUData ImuData
        {
            get { return _imuData; }
            set
            {
                _imuData = value;
                OnPropertyChanged(nameof(ImuData));
            }
        }

        public List<IMUData> datas { get; set; } = new List<IMUData>();
        #endregion

        public void DataReceived(RecordedData data, int dataSource)
        {
            var d = datas.FirstOrDefault(a => a.PackageId == data.PackageId);
            if (d == null)
            {
                IMUData imuData = new IMUData() { PackageId = data.PackageId };
                imuData.RecordedDatas[dataSource] = data;
                datas.Add(imuData);
            }
            else
            {
                d.RecordedDatas[dataSource] = data;
            }

        }
    }
}
