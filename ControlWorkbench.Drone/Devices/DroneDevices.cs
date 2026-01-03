namespace ControlWorkbench.Drone.Devices;

/// <summary>
/// Core drone device abstractions for various flight controller platforms.
/// Supports ArduPilot, Betaflight, PX4, and custom builds.
/// </summary>
public interface IFlightController
{
    string Name { get; }
    FlightControllerType Type { get; }
    bool IsConnected { get; }
    FlightMode CurrentMode { get; }
    
    Task<bool> ConnectAsync(string connectionString, CancellationToken ct = default);
    Task DisconnectAsync();
    
    Task<bool> ArmAsync(CancellationToken ct = default);
    Task<bool> DisarmAsync(CancellationToken ct = default);
    
    Task SetFlightModeAsync(FlightMode mode, CancellationToken ct = default);
    Task SendRcChannelsAsync(RcChannels channels, CancellationToken ct = default);
    
    event Action<DroneTelemetry>? TelemetryReceived;
    event Action<DroneStatus>? StatusChanged;
    event Action<string>? LogReceived;
}

public enum FlightControllerType
{
    ArduPilot,      // ArduCopter, ArduPlane, ArduRover
    Betaflight,     // FPV racing drones
    PX4,            // Professional/research drones
    INAV,           // Navigation-focused
    DJI,            // DJI flight controllers
    Custom          // Custom flight controller
}

public enum FlightMode
{
    // Common modes
    Stabilize,
    AltHold,
    Loiter,
    Auto,
    Guided,
    Land,
    RTL,            // Return to Launch
    
    // ArduPilot specific
    PosHold,
    Acro,
    Sport,
    Drift,
    Flip,
    AutoTune,
    Brake,
    SmartRTL,
    
    // Betaflight specific
    Angle,
    Horizon,
    AirMode,
    
    // Advanced
    FollowMe,
    Circle,
    Waypoint,
    
    // Safety
    Disarmed,
    Emergency
}

/// <summary>
/// RC channel inputs (typically 16 channels).
/// </summary>
public class RcChannels
{
    // Standard mapping
    public int Roll { get; set; }       // Channel 1 (1000-2000)
    public int Pitch { get; set; }      // Channel 2 (1000-2000)
    public int Throttle { get; set; }   // Channel 3 (1000-2000)
    public int Yaw { get; set; }        // Channel 4 (1000-2000)
    
    // Aux channels
    public int[] Aux { get; } = new int[12];  // Channels 5-16
    
    public static RcChannels Neutral => new()
    {
        Roll = 1500,
        Pitch = 1500,
        Throttle = 1000,  // Throttle at minimum
        Yaw = 1500
    };
    
    public int[] ToArray()
    {
        var channels = new int[16];
        channels[0] = Roll;
        channels[1] = Pitch;
        channels[2] = Throttle;
        channels[3] = Yaw;
        for (int i = 0; i < 12; i++)
            channels[4 + i] = Aux[i];
        return channels;
    }
}

/// <summary>
/// Complete drone telemetry packet.
/// </summary>
public class DroneTelemetry
{
    public DateTimeOffset Timestamp { get; set; } = DateTimeOffset.UtcNow;
    
    // Attitude (degrees)
    public double Roll { get; set; }
    public double Pitch { get; set; }
    public double Yaw { get; set; }
    
    // Angular rates (degrees/second)
    public double RollRate { get; set; }
    public double PitchRate { get; set; }
    public double YawRate { get; set; }
    
    // Position (GPS)
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double AltitudeMSL { get; set; }     // Above sea level
    public double AltitudeAGL { get; set; }     // Above ground level
    public double AltitudeRelative { get; set; } // Above home
    
    // Velocity (m/s)
    public double VelocityNorth { get; set; }
    public double VelocityEast { get; set; }
    public double VelocityDown { get; set; }
    public double GroundSpeed { get; set; }
    public double AirSpeed { get; set; }
    public double ClimbRate { get; set; }
    
    // Heading
    public double Heading { get; set; }         // Magnetic
    public double HeadingTrue { get; set; }     // True north
    public double CourseOverGround { get; set; }
    
    // Battery
    public double BatteryVoltage { get; set; }
    public double BatteryCurrent { get; set; }
    public double BatteryRemaining { get; set; } // Percent
    public double BatteryConsumed { get; set; }  // mAh
    
    // GPS
    public int GpsSatellites { get; set; }
    public GpsFixType GpsFix { get; set; }
    public double GpsHdop { get; set; }
    public double GpsVdop { get; set; }
    
    // System
    public FlightMode Mode { get; set; }
    public bool Armed { get; set; }
    public bool Failsafe { get; set; }
    public double CpuLoad { get; set; }
    public double BoardVoltage { get; set; }
    
    // Motors
    public int[] MotorOutputs { get; set; } = new int[8];
    
    // RC inputs
    public RcChannels? RcInputs { get; set; }
    
    // Sensors
    public double BarometerAltitude { get; set; }
    public double RangefinderDistance { get; set; }
    public double OpticalFlowX { get; set; }
    public double OpticalFlowY { get; set; }
    
    // Home position
    public double HomeLatitude { get; set; }
    public double HomeLongitude { get; set; }
    public double DistanceToHome { get; set; }
    public double BearingToHome { get; set; }
    
    // Mission
    public int CurrentWaypoint { get; set; }
    public int TotalWaypoints { get; set; }
    public double DistanceToWaypoint { get; set; }
}

public enum GpsFixType
{
    NoFix = 0,
    Fix2D = 2,
    Fix3D = 3,
    DGPS = 4,
    RTKFloat = 5,
    RTKFixed = 6
}

/// <summary>
/// Drone status summary.
/// </summary>
public class DroneStatus
{
    public bool Connected { get; set; }
    public bool Armed { get; set; }
    public bool GpsOk { get; set; }
    public bool EkfOk { get; set; }
    public bool CompassOk { get; set; }
    public bool BarometerOk { get; set; }
    public bool AccelerometerOk { get; set; }
    public bool GyroOk { get; set; }
    public bool RcOk { get; set; }
    public bool BatteryOk { get; set; }
    public bool Failsafe { get; set; }
    
    public List<string> Errors { get; } = new();
    public List<string> Warnings { get; } = new();
    
    public bool ReadyToArm => GpsOk && EkfOk && CompassOk && BarometerOk && 
                               AccelerometerOk && GyroOk && RcOk && BatteryOk && 
                               !Failsafe && Errors.Count == 0;
}

/// <summary>
/// Common motor configurations.
/// </summary>
public enum FrameType
{
    // Multirotor
    QuadX,
    QuadPlus,
    QuadH,
    HexX,
    HexPlus,
    OctoX,
    OctoPlus,
    Y6,
    Tricopter,
    
    // Fixed wing
    Plane,
    FlyingWing,
    VTail,
    
    // Hybrid
    QuadPlane,
    Tiltrotor,
    
    // Ground
    Rover,
    Boat
}

/// <summary>
/// Motor/ESC configuration.
/// </summary>
public class MotorConfig
{
    public int Index { get; set; }
    public double PositionX { get; set; }   // Forward is positive
    public double PositionY { get; set; }   // Right is positive
    public double PositionZ { get; set; }   // Down is positive
    public bool Clockwise { get; set; }
    public double TiltAngle { get; set; }   // For tilt-rotors
    
    public int PwmMin { get; set; } = 1000;
    public int PwmMax { get; set; } = 2000;
    public int PwmIdle { get; set; } = 1100;
    
    public static MotorConfig[] CreateQuadX()
    {
        return new[]
        {
            new MotorConfig { Index = 0, PositionX = 1, PositionY = -1, Clockwise = true },   // Front Right
            new MotorConfig { Index = 1, PositionX = -1, PositionY = -1, Clockwise = false }, // Back Right
            new MotorConfig { Index = 2, PositionX = -1, PositionY = 1, Clockwise = true },   // Back Left
            new MotorConfig { Index = 3, PositionX = 1, PositionY = 1, Clockwise = false }    // Front Left
        };
    }
    
    public static MotorConfig[] CreateHexX()
    {
        double sin60 = System.Math.Sin(System.Math.PI / 3);
        double cos60 = System.Math.Cos(System.Math.PI / 3);
        return new[]
        {
            new MotorConfig { Index = 0, PositionX = 1, PositionY = 0, Clockwise = true },
            new MotorConfig { Index = 1, PositionX = cos60, PositionY = -sin60, Clockwise = false },
            new MotorConfig { Index = 2, PositionX = -cos60, PositionY = -sin60, Clockwise = true },
            new MotorConfig { Index = 3, PositionX = -1, PositionY = 0, Clockwise = false },
            new MotorConfig { Index = 4, PositionX = -cos60, PositionY = sin60, Clockwise = true },
            new MotorConfig { Index = 5, PositionX = cos60, PositionY = sin60, Clockwise = false }
        };
    }
}

/// <summary>
/// Battery configuration.
/// </summary>
public class BatteryConfig
{
    public int CellCount { get; set; } = 4;          // 4S, 6S, etc.
    public double CapacityMah { get; set; } = 5000;
    public double CellVoltageMin { get; set; } = 3.3;
    public double CellVoltageMax { get; set; } = 4.2;
    public double CellVoltageNominal { get; set; } = 3.7;
    
    public double TotalVoltageMin => CellCount * CellVoltageMin;
    public double TotalVoltageMax => CellCount * CellVoltageMax;
    public double TotalVoltageNominal => CellCount * CellVoltageNominal;
    
    public double CalculatePercentage(double voltage)
    {
        double cellVoltage = voltage / CellCount;
        return System.Math.Clamp((cellVoltage - CellVoltageMin) / (CellVoltageMax - CellVoltageMin) * 100, 0, 100);
    }
}

/// <summary>
/// Drone physical specifications.
/// </summary>
public class DroneSpecs
{
    public string Name { get; set; } = "Custom Drone";
    public FrameType Frame { get; set; } = FrameType.QuadX;
    
    // Physical dimensions
    public double WeightGrams { get; set; } = 1500;
    public double ArmLengthMm { get; set; } = 250;      // Motor to center
    public double PropDiameterInches { get; set; } = 5;
    public double PropPitchInches { get; set; } = 4.5;
    
    // Performance
    public double MaxThrustGrams { get; set; } = 4000;  // Total thrust at 100%
    public double HoverThrottlePercent { get; set; } = 50;
    public double MaxSpeedMs { get; set; } = 20;
    public double MaxClimbRateMs { get; set; } = 5;
    
    // Battery
    public BatteryConfig Battery { get; set; } = new();
    public double FlightTimeMinutes { get; set; } = 25;
    
    // Motors
    public MotorConfig[] Motors { get; set; } = MotorConfig.CreateQuadX();
    
    public double ThrustToWeightRatio => MaxThrustGrams / WeightGrams;
}

