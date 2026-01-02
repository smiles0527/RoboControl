using System.Windows.Threading;
using ControlWorkbench.Core.Collections;
using ControlWorkbench.Core.Units;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// ViewModel for the Telemetry Dashboard tab.
/// </summary>
public class TelemetryDashboardViewModel : ViewModelBase
{
    private const int RingBufferSize = 1000;
    private readonly DispatcherTimer _updateTimer;

    // Ring buffers for plot data
    public RingBuffer<double> TimeBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> GyroXBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> GyroYBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> GyroZBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> AccelXBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> AccelYBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> AccelZBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> RollBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> PitchBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> YawBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> GpsSpeedBuffer { get; } = new(RingBufferSize);
    public RingBuffer<double> GpsTimeBuffer { get; } = new(RingBufferSize);

    // Current values - Attitude
    private double _roll;
    private double _pitch;
    private double _yaw;
    private double _rollRate;
    private double _pitchRate;
    private double _yawRate;

    // Current values - IMU
    private double _accelMagnitude;
    private double _temperature;
    private double _gyroX;
    private double _gyroY;
    private double _gyroZ;
    private double _accelX;
    private double _accelY;
    private double _accelZ;

    // Current values - GPS
    private double _latitude;
    private double _longitude;
    private double _altitude;
    private double _gpsSpeed;
    private double _horizontalAccuracy;
    private double _verticalAccuracy;

    // Current values - State Estimate
    private double _stateX;
    private double _stateY;
    private double _stateYaw;
    private double _stateVx;
    private double _stateVy;
    private double _covarianceTrace;

    // Heartbeat
    private uint _uptimeMs;
    private byte _systemState;
    private bool _isArmed;

    private double _startTime;
    private bool _initialized;

    public TelemetryDashboardViewModel()
    {
        _updateTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(50) // 20 Hz UI update
        };
        _updateTimer.Tick += UpdateTimer_Tick;
        _updateTimer.Start();
    }

    // Event to trigger plot refresh
    public event EventHandler? PlotDataUpdated;

    // Attitude Properties (in degrees for display)
    public double RollDeg
    {
        get => _roll;
        private set => SetProperty(ref _roll, value);
    }

    public double PitchDeg
    {
        get => _pitch;
        private set => SetProperty(ref _pitch, value);
    }

    public double YawDeg
    {
        get => _yaw;
        private set => SetProperty(ref _yaw, value);
    }

    public double RollRateDegPerSec
    {
        get => _rollRate;
        private set => SetProperty(ref _rollRate, value);
    }

    public double PitchRateDegPerSec
    {
        get => _pitchRate;
        private set => SetProperty(ref _pitchRate, value);
    }

    public double YawRateDegPerSec
    {
        get => _yawRate;
        private set => SetProperty(ref _yawRate, value);
    }

    // IMU Properties
    public double GyroX
    {
        get => _gyroX;
        private set => SetProperty(ref _gyroX, value);
    }

    public double GyroY
    {
        get => _gyroY;
        private set => SetProperty(ref _gyroY, value);
    }

    public double GyroZ
    {
        get => _gyroZ;
        private set => SetProperty(ref _gyroZ, value);
    }

    public double AccelX
    {
        get => _accelX;
        private set => SetProperty(ref _accelX, value);
    }

    public double AccelY
    {
        get => _accelY;
        private set => SetProperty(ref _accelY, value);
    }

    public double AccelZ
    {
        get => _accelZ;
        private set => SetProperty(ref _accelZ, value);
    }

    public double AccelMagnitude
    {
        get => _accelMagnitude;
        private set => SetProperty(ref _accelMagnitude, value);
    }

    public double Temperature
    {
        get => _temperature;
        private set => SetProperty(ref _temperature, value);
    }

    // GPS Properties
    public double Latitude
    {
        get => _latitude;
        private set => SetProperty(ref _latitude, value);
    }

    public double Longitude
    {
        get => _longitude;
        private set => SetProperty(ref _longitude, value);
    }

    public double Altitude
    {
        get => _altitude;
        private set => SetProperty(ref _altitude, value);
    }

    public double GpsSpeed
    {
        get => _gpsSpeed;
        private set => SetProperty(ref _gpsSpeed, value);
    }

    public double HorizontalAccuracy
    {
        get => _horizontalAccuracy;
        private set => SetProperty(ref _horizontalAccuracy, value);
    }

    public double VerticalAccuracy
    {
        get => _verticalAccuracy;
        private set => SetProperty(ref _verticalAccuracy, value);
    }

    // State Estimate Properties
    public double StateX
    {
        get => _stateX;
        private set => SetProperty(ref _stateX, value);
    }

    public double StateY
    {
        get => _stateY;
        private set => SetProperty(ref _stateY, value);
    }

    public double StateYawDeg
    {
        get => _stateYaw;
        private set => SetProperty(ref _stateYaw, value);
    }

    public double StateVx
    {
        get => _stateVx;
        private set => SetProperty(ref _stateVx, value);
    }

    public double StateVy
    {
        get => _stateVy;
        private set => SetProperty(ref _stateVy, value);
    }

    public double CovarianceTrace
    {
        get => _covarianceTrace;
        private set => SetProperty(ref _covarianceTrace, value);
    }

    // Heartbeat Properties
    public uint UptimeMs
    {
        get => _uptimeMs;
        private set => SetProperty(ref _uptimeMs, value);
    }

    public byte SystemState
    {
        get => _systemState;
        private set => SetProperty(ref _systemState, value);
    }

    public bool IsArmed
    {
        get => _isArmed;
        private set => SetProperty(ref _isArmed, value);
    }

    public string UptimeFormatted => TimeSpan.FromMilliseconds(UptimeMs).ToString(@"hh\:mm\:ss");

    /// <summary>
    /// Processes an incoming message.
    /// </summary>
    public void ProcessMessage(IMessage message)
    {
        switch (message)
        {
            case HeartbeatMessage hb:
                ProcessHeartbeat(hb);
                break;
            case ImuRawMessage imu:
                ProcessImu(imu);
                break;
            case AttitudeMessage att:
                ProcessAttitude(att);
                break;
            case GpsMessage gps:
                ProcessGps(gps);
                break;
            case StateEstimate2DMessage se:
                ProcessStateEstimate(se);
                break;
        }
    }

    private void ProcessHeartbeat(HeartbeatMessage hb)
    {
        UptimeMs = hb.UptimeMs;
        SystemState = hb.SystemState;
        IsArmed = hb.Armed != 0;
        OnPropertyChanged(nameof(UptimeFormatted));
    }

    private void ProcessImu(ImuRawMessage imu)
    {
        double time = GetRelativeTime(imu.TimeUs);

        GyroX = Angle.FromRadians(imu.GyroX).Degrees;
        GyroY = Angle.FromRadians(imu.GyroY).Degrees;
        GyroZ = Angle.FromRadians(imu.GyroZ).Degrees;

        AccelX = imu.AccelX;
        AccelY = imu.AccelY;
        AccelZ = imu.AccelZ;
        AccelMagnitude = System.Math.Sqrt(imu.AccelX * imu.AccelX + imu.AccelY * imu.AccelY + imu.AccelZ * imu.AccelZ);
        Temperature = imu.Temperature;

        // Add to ring buffers
        TimeBuffer.Add(time);
        GyroXBuffer.Add(GyroX);
        GyroYBuffer.Add(GyroY);
        GyroZBuffer.Add(GyroZ);
        AccelXBuffer.Add(AccelX);
        AccelYBuffer.Add(AccelY);
        AccelZBuffer.Add(AccelZ);
    }

    private void ProcessAttitude(AttitudeMessage att)
    {
        RollDeg = Angle.FromRadians(att.Roll).Degrees;
        PitchDeg = Angle.FromRadians(att.Pitch).Degrees;
        YawDeg = Angle.FromRadians(att.Yaw).Degrees;
        RollRateDegPerSec = Angle.FromRadians(att.P).Degrees;
        PitchRateDegPerSec = Angle.FromRadians(att.Q).Degrees;
        YawRateDegPerSec = Angle.FromRadians(att.R).Degrees;

        double time = GetRelativeTime(att.TimeUs);
        RollBuffer.Add(RollDeg);
        PitchBuffer.Add(PitchDeg);
        YawBuffer.Add(YawDeg);
    }

    private void ProcessGps(GpsMessage gps)
    {
        Latitude = gps.Latitude;
        Longitude = gps.Longitude;
        Altitude = gps.Altitude;
        
        double vn = gps.VelocityNorth;
        double ve = gps.VelocityEast;
        GpsSpeed = System.Math.Sqrt(vn * vn + ve * ve);
        
        HorizontalAccuracy = gps.HorizontalAccuracy;
        VerticalAccuracy = gps.VerticalAccuracy;

        double time = GetRelativeTime(gps.TimeUs);
        GpsTimeBuffer.Add(time);
        GpsSpeedBuffer.Add(GpsSpeed);
    }

    private void ProcessStateEstimate(StateEstimate2DMessage se)
    {
        StateX = se.X;
        StateY = se.Y;
        StateYawDeg = Angle.FromRadians(se.Yaw).Degrees;
        StateVx = se.VelocityX;
        StateVy = se.VelocityY;
        CovarianceTrace = se.CovarianceXX + se.CovarianceYY + se.CovarianceYawYaw;
    }

    private double GetRelativeTime(long timeUs)
    {
        double time = timeUs / 1_000_000.0;
        if (!_initialized)
        {
            _startTime = time;
            _initialized = true;
        }
        return time - _startTime;
    }

    private void UpdateTimer_Tick(object? sender, EventArgs e)
    {
        PlotDataUpdated?.Invoke(this, EventArgs.Empty);
    }

    public void ClearBuffers()
    {
        TimeBuffer.Clear();
        GyroXBuffer.Clear();
        GyroYBuffer.Clear();
        GyroZBuffer.Clear();
        AccelXBuffer.Clear();
        AccelYBuffer.Clear();
        AccelZBuffer.Clear();
        RollBuffer.Clear();
        PitchBuffer.Clear();
        YawBuffer.Clear();
        GpsSpeedBuffer.Clear();
        GpsTimeBuffer.Clear();
        _initialized = false;
    }
}
