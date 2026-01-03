namespace ControlWorkbench.VEX;

/// <summary>
/// VEX V5 protocol constants and definitions.
/// </summary>
public static class VexConstants
{
    // Protocol framing
    public const byte Preamble1 = 0xAA;
    public const byte Preamble2 = 0x55;
    public const byte Version = 0x01;
    public const int MaxPayloadSize = 255;
    public const int HeaderSize = 4; // Preamble(2) + Type(1) + Length(1)
    public const int ChecksumSize = 1;

    // Default communication settings
    public const int DefaultBaudRate = 115200;
    public const int DefaultTimeoutMs = 100;

    // V5 Brain ports
    public const int SmartPortCount = 21;
    public const int ThreeWirePortCount = 8;

    // Motor constants
    public const double MotorMaxRpm = 200.0; // 200 RPM cartridge
    public const double MotorMaxRpm100 = 100.0; // 100 RPM cartridge
    public const double MotorMaxRpm600 = 600.0; // 600 RPM cartridge
    public const double MotorEncoderTicksPerRev = 1800.0; // Encoder resolution
    public const double MotorMaxVoltage = 12000.0; // mV
    public const double MotorMaxCurrent = 2500.0; // mA

    // IMU constants
    public const double ImuMaxRotationRate = 1000.0; // deg/s

    // Timing
    public const int ControlLoopPeriodMs = 10; // 100 Hz default
    public const int TelemetryPeriodMs = 20; // 50 Hz telemetry
}

/// <summary>
/// VEX V5 message types for telemetry protocol.
/// </summary>
public enum VexMessageType : byte
{
    // System messages (0x00 - 0x0F)
    Heartbeat = 0x00,
    Acknowledge = 0x01,
    Error = 0x02,
    SystemStatus = 0x03,
    BatteryStatus = 0x04,
    CompetitionStatus = 0x05,

    // Sensor data (0x10 - 0x2F)
    Odometry = 0x10,
    ImuData = 0x11,
    MotorTelemetry = 0x12,
    EncoderData = 0x13,
    RotationSensor = 0x14,
    DistanceSensor = 0x15,
    OpticalSensor = 0x16,
    VisionSensor = 0x17,
    GpsSensor = 0x18,
    InertialSensor = 0x19,
    PotentiometerData = 0x1A,
    LineSensorData = 0x1B,
    BumperData = 0x1C,
    LimitSwitchData = 0x1D,

    // Control/Parameter messages (0x30 - 0x4F)
    SetParameter = 0x30,
    GetParameter = 0x31,
    ParameterValue = 0x32,
    ParameterList = 0x33,
    PidState = 0x34,
    MotionProfileState = 0x35,
    ControllerInput = 0x36,

    // Path/Autonomous messages (0x50 - 0x5F)
    PathWaypoint = 0x50,
    PathComplete = 0x51,
    PathProgress = 0x52,
    AutonomousState = 0x53,

    // Logging (0x60 - 0x6F)
    LogMessage = 0x60,
    DebugValue = 0x61,
    GraphData = 0x62,

    // Commands (0x70 - 0x7F)
    StartRecording = 0x70,
    StopRecording = 0x71,
    RequestSync = 0x72,
    ResetOdometry = 0x73,
    SetMotorTarget = 0x74,
    EmergencyStop = 0x7F
}

/// <summary>
/// V5 motor cartridge types.
/// </summary>
public enum MotorCartridge
{
    Red = 100,    // 100 RPM, high torque
    Green = 200,  // 200 RPM, standard
    Blue = 600    // 600 RPM, high speed
}

/// <summary>
/// V5 motor brake modes.
/// </summary>
public enum MotorBrakeMode
{
    Coast = 0,
    Brake = 1,
    Hold = 2
}

/// <summary>
/// Competition state.
/// </summary>
public enum CompetitionState
{
    Disabled = 0,
    Autonomous = 1,
    DriverControl = 2,
    SkillsAutonomous = 3,
    SkillsDriver = 4
}

/// <summary>
/// V5 smart port device types.
/// </summary>
public enum V5DeviceType
{
    None = 0,
    Motor = 2,
    Rotation = 4,
    Imu = 6,
    Distance = 7,
    Radio = 8,
    Vision = 11,
    Gps = 20,
    Optical = 29
}

/// <summary>
/// Controller button definitions.
/// </summary>
[Flags]
public enum ControllerButtons : ushort
{
    None = 0,
    L1 = 1 << 0,
    L2 = 1 << 1,
    R1 = 1 << 2,
    R2 = 1 << 3,
    Up = 1 << 4,
    Down = 1 << 5,
    Left = 1 << 6,
    Right = 1 << 7,
    X = 1 << 8,
    B = 1 << 9,
    Y = 1 << 10,
    A = 1 << 11
}
