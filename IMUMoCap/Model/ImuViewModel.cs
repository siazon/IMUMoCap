using IMUMoCap;
using IMUMoCap.Methods;
using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Media.Media3D;

public class ImuViewModel : INotifyPropertyChanged
{
    public string SlotName { get; }              // UI显示用：IMU-1 / IMU-2 / IMU-3
    public uint DeviceId { get;  set; } = 0;  // 第一次绑定后填入
    public ImuRole   Role { get; set; }

    private Quaternion _quat = Quaternion.Identity;
    public Quaternion Quat
    {
        get => _quat;
        set { _quat = value; OnPropertyChanged(); }
    }
    private bool _isConnected;
    public bool IsConnected
    {
        get => _isConnected;
        set { _isConnected = value; OnPropertyChanged(); }
    }

    private DateTime _lastUpdateUtc;
    public DateTime LastUpdateUtc
    {
        get => _lastUpdateUtc;
        set { _lastUpdateUtc = value; OnPropertyChanged(); }
    }

    public ImuViewModel(string slotName) => SlotName = slotName;

    public void BindDevice(uint deviceId)
    {
        DeviceId = deviceId;
        IsConnected = true;
        OnPropertyChanged(nameof(DeviceId));
        OnPropertyChanged(nameof(IsConnected));
    }

    public event PropertyChangedEventHandler? PropertyChanged;
    private void OnPropertyChanged([CallerMemberName] string? name = null)
        => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
