namespace ControlWorkbench.Core.Sensors;

/// <summary>
/// IMU (Inertial Measurement Unit) data structure.
/// </summary>
public record ImuData
{
    public DateTime Timestamp { get; init; }
    
    // Accelerometer (m/s²)
    public double AccelX { get; init; }
    public double AccelY { get; init; }
    public double AccelZ { get; init; }
    
    // Gyroscope (rad/s)
    public double GyroX { get; init; }
    public double GyroY { get; init; }
    public double GyroZ { get; init; }
    
    // Magnetometer (µT)
    public double? MagX { get; init; }
    public double? MagY { get; init; }
    public double? MagZ { get; init; }
    
    // Temperature (°C)
    public double? Temperature { get; init; }
}

/// <summary>
/// GPS/GNSS data structure.
/// </summary>
public record GpsData
{
    public DateTime Timestamp { get; init; }
    public double Latitude { get; init; }
    public double Longitude { get; init; }
    public double Altitude { get; init; }
    public double? GroundSpeed { get; init; }
    public double? Course { get; init; }
    public GpsFixType FixType { get; init; }
    public int SatellitesVisible { get; init; }
    public double? Hdop { get; init; }
    public double? Vdop { get; init; }
    public double? Pdop { get; init; }
    public double? HorizontalAccuracy { get; init; }
    public double? VerticalAccuracy { get; init; }
}

public enum GpsFixType
{
    NoFix = 0,
    Fix2D = 2,
    Fix3D = 3,
    DGps = 4,
    RtkFloat = 5,
    RtkFixed = 6
}

/// <summary>
/// Barometric pressure/altitude data.
/// </summary>
public record BarometerData
{
    public DateTime Timestamp { get; init; }
    public double Pressure { get; init; } // hPa
    public double Temperature { get; init; } // °C
    public double Altitude { get; init; } // m (computed from pressure)
}

/// <summary>
/// Optical flow sensor data.
/// </summary>
public record OpticalFlowData
{
    public DateTime Timestamp { get; init; }
    public double FlowX { get; init; } // rad/s
    public double FlowY { get; init; } // rad/s
    public double GroundDistance { get; init; } // m
    public int Quality { get; init; } // 0-255
}

/// <summary>
/// LiDAR point cloud data.
/// </summary>
public record LidarData
{
    public DateTime Timestamp { get; init; }
    public LidarPoint[] Points { get; init; } = Array.Empty<LidarPoint>();
    public int SequenceNumber { get; init; }
}

public record LidarPoint(float X, float Y, float Z, float Intensity);

/// <summary>
/// Ultrasonic/sonar range finder data.
/// </summary>
public record RangeFinderData
{
    public DateTime Timestamp { get; init; }
    public double Range { get; init; } // m
    public double MinRange { get; init; }
    public double MaxRange { get; init; }
    public double FieldOfView { get; init; } // rad
    public RangeFinderType Type { get; init; }
}

public enum RangeFinderType
{
    Ultrasonic,
    Infrared,
    Laser
}

/// <summary>
/// Camera/vision data.
/// </summary>
public record CameraData
{
    public DateTime Timestamp { get; init; }
    public int Width { get; init; }
    public int Height { get; init; }
    public string Encoding { get; init; } = "rgb8";
    public byte[] ImageData { get; init; } = Array.Empty<byte>();
    public int SequenceNumber { get; init; }
}

/// <summary>
/// Battery/power system data.
/// </summary>
public record BatteryData
{
    public DateTime Timestamp { get; init; }
    public double Voltage { get; init; } // V
    public double Current { get; init; } // A
    public double? StateOfCharge { get; init; } // 0-100%
    public double? Temperature { get; init; } // °C
    public double? Capacity { get; init; } // Ah
    public double? RemainingCapacity { get; init; } // Ah
    public int? CellCount { get; init; }
    public double[]? CellVoltages { get; init; }
    public BatteryHealth Health { get; init; }
}

public enum BatteryHealth
{
    Unknown,
    Good,
    Degraded,
    Replace,
    Critical
}

/// <summary>
/// Motor/actuator data.
/// </summary>
public record MotorData
{
    public int MotorId { get; init; }
    public DateTime Timestamp { get; init; }
    public double Position { get; init; } // rad or m
    public double Velocity { get; init; } // rad/s or m/s
    public double Current { get; init; } // A
    public double Temperature { get; init; } // °C
    public double TargetPosition { get; init; }
    public double TargetVelocity { get; init; }
    public MotorMode Mode { get; init; }
    public MotorStatus Status { get; init; }
}

public enum MotorMode
{
    Off,
    Position,
    Velocity,
    Torque,
    Current
}

public enum MotorStatus
{
    Ok,
    OverTemperature,
    OverCurrent,
    EncoderError,
    CommunicationError,
    Disabled
}

/// <summary>
/// RC/Radio receiver data.
/// </summary>
public record RcInputData
{
    public DateTime Timestamp { get; init; }
    public ushort[] Channels { get; init; } = Array.Empty<ushort>(); // 1000-2000 µs
    public int ChannelCount { get; init; }
    public int Rssi { get; init; } // Signal strength
    public bool IsValid { get; init; }
    public bool FailsafeActive { get; init; }
}

/// <summary>
/// Servo/PWM output data.
/// </summary>
public record ServoOutputData
{
    public DateTime Timestamp { get; init; }
    public ushort[] Outputs { get; init; } = Array.Empty<ushort>(); // 1000-2000 µs
    public int OutputCount { get; init; }
}

/// <summary>
/// Vehicle attitude (orientation).
/// </summary>
public record AttitudeData
{
    public DateTime Timestamp { get; init; }
    public double Roll { get; init; } // rad
    public double Pitch { get; init; } // rad
    public double Yaw { get; init; } // rad
    public double RollRate { get; init; } // rad/s
    public double PitchRate { get; init; } // rad/s
    public double YawRate { get; init; } // rad/s
    
    // Quaternion representation
    public double Qw { get; init; }
    public double Qx { get; init; }
    public double Qy { get; init; }
    public double Qz { get; init; }
}

/// <summary>
/// Vehicle position in local frame.
/// </summary>
public record LocalPositionData
{
    public DateTime Timestamp { get; init; }
    public double X { get; init; } // m (North)
    public double Y { get; init; } // m (East)
    public double Z { get; init; } // m (Down)
    public double Vx { get; init; } // m/s
    public double Vy { get; init; } // m/s
    public double Vz { get; init; } // m/s
    public string FrameId { get; init; } = "local_ned";
}

/// <summary>
/// Vehicle position in global frame.
/// </summary>
public record GlobalPositionData
{
    public DateTime Timestamp { get; init; }
    public double Latitude { get; init; } // deg
    public double Longitude { get; init; } // deg
    public double AltitudeMsl { get; init; } // m
    public double AltitudeRelative { get; init; } // m
    public double Heading { get; init; } // rad
    public double GroundSpeed { get; init; } // m/s
    public double VerticalSpeed { get; init; } // m/s
}

/// <summary>
/// Sensor fusion configuration.
/// </summary>
public class SensorFusionConfig
{
    public bool UseGps { get; set; } = true;
    public bool UseImu { get; set; } = true;
    public bool UseBarometer { get; set; } = true;
    public bool UseMagnetometer { get; set; } = true;
    public bool UseOpticalFlow { get; set; } = false;
    public bool UseRangeFinder { get; set; } = false;
    public bool UseVisionPosition { get; set; } = false;
    
    // Sensor weights for fusion
    public double GpsWeight { get; set; } = 1.0;
    public double ImuWeight { get; set; } = 1.0;
    public double BaroWeight { get; set; } = 1.0;
    public double MagWeight { get; set; } = 0.5;
    public double FlowWeight { get; set; } = 0.8;
    
    // Noise parameters
    public double AccelNoise { get; set; } = 0.1;
    public double GyroNoise { get; set; } = 0.01;
    public double GpsPositionNoise { get; set; } = 1.0;
    public double GpsVelocityNoise { get; set; } = 0.1;
    public double BaroNoise { get; set; } = 0.5;
}

/// <summary>
/// Sensor calibration data.
/// </summary>
public record SensorCalibration
{
    // Accelerometer
    public double[] AccelBias { get; init; } = new double[3];
    public double[,] AccelScale { get; init; } = new double[3, 3];
    
    // Gyroscope
    public double[] GyroBias { get; init; } = new double[3];
    public double[,] GyroScale { get; init; } = new double[3, 3];
    
    // Magnetometer
    public double[] MagBias { get; init; } = new double[3];
    public double[,] MagScale { get; init; } = new double[3, 3];
    
    // Barometer
    public double BaroOffset { get; init; }
    public double BaroScale { get; init; } = 1.0;
}

/// <summary>
/// Sensor health monitoring.
/// </summary>
public class SensorHealthMonitor
{
    private readonly Dictionary<string, SensorHealth> _sensorHealth = new();
    private readonly Dictionary<string, DateTime> _lastUpdate = new();
    private readonly TimeSpan _timeoutThreshold = TimeSpan.FromSeconds(1);

    public void UpdateSensor(string sensorName, bool isValid, double? confidence = null)
    {
        _lastUpdate[sensorName] = DateTime.UtcNow;
        _sensorHealth[sensorName] = new SensorHealth
        {
            IsOnline = true,
            IsValid = isValid,
            Confidence = confidence ?? 1.0,
            LastUpdate = DateTime.UtcNow
        };
    }

    public SensorHealth GetHealth(string sensorName)
    {
        if (!_sensorHealth.TryGetValue(sensorName, out var health))
        {
            return new SensorHealth { IsOnline = false, IsValid = false };
        }

        // Check for timeout
        if (_lastUpdate.TryGetValue(sensorName, out var lastUpdate))
        {
            if (DateTime.UtcNow - lastUpdate > _timeoutThreshold)
            {
                return health with { IsOnline = false, IsValid = false };
            }
        }

        return health;
    }

    public Dictionary<string, SensorHealth> GetAllHealth()
    {
        var result = new Dictionary<string, SensorHealth>();
        foreach (var sensor in _sensorHealth.Keys)
        {
            result[sensor] = GetHealth(sensor);
        }
        return result;
    }
}

public record SensorHealth
{
    public bool IsOnline { get; init; }
    public bool IsValid { get; init; }
    public double Confidence { get; init; }
    public DateTime LastUpdate { get; init; }
}
