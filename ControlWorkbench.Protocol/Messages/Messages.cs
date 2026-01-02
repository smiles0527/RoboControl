namespace ControlWorkbench.Protocol.Messages;

/// <summary>
/// Base interface for all protocol messages.
/// </summary>
public interface IMessage
{
    /// <summary>
    /// Gets the message type.
    /// </summary>
    MessageType Type { get; }

    /// <summary>
    /// Gets the payload size in bytes.
    /// </summary>
    int PayloadSize { get; }
}

/// <summary>
/// Heartbeat message containing system status.
/// Payload: uint32 uptime_ms, uint8 system_state, uint8 armed, uint16 reserved
/// </summary>
public sealed record HeartbeatMessage : IMessage
{
    public MessageType Type => MessageType.Heartbeat;
    public int PayloadSize => 8; // 4 + 1 + 1 + 2

    /// <summary>
    /// System uptime in milliseconds.
    /// </summary>
    public uint UptimeMs { get; init; }

    /// <summary>
    /// Current system state.
    /// </summary>
    public byte SystemState { get; init; }

    /// <summary>
    /// Whether the system is armed.
    /// </summary>
    public byte Armed { get; init; }

    /// <summary>
    /// Reserved bytes.
    /// </summary>
    public ushort Reserved { get; init; }
}

/// <summary>
/// Raw IMU data message.
/// Payload: int64 time_us, float gx/gy/gz (rad/s), float ax/ay/az (m/s²), float temperature_c
/// </summary>
public sealed record ImuRawMessage : IMessage
{
    public MessageType Type => MessageType.ImuRaw;
    public int PayloadSize => 36; // 8 + 4*7

    /// <summary>
    /// Timestamp in microseconds.
    /// </summary>
    public long TimeUs { get; init; }

    /// <summary>
    /// Gyroscope X-axis (rad/s).
    /// </summary>
    public float GyroX { get; init; }

    /// <summary>
    /// Gyroscope Y-axis (rad/s).
    /// </summary>
    public float GyroY { get; init; }

    /// <summary>
    /// Gyroscope Z-axis (rad/s).
    /// </summary>
    public float GyroZ { get; init; }

    /// <summary>
    /// Accelerometer X-axis (m/s²).
    /// </summary>
    public float AccelX { get; init; }

    /// <summary>
    /// Accelerometer Y-axis (m/s²).
    /// </summary>
    public float AccelY { get; init; }

    /// <summary>
    /// Accelerometer Z-axis (m/s²).
    /// </summary>
    public float AccelZ { get; init; }

    /// <summary>
    /// Temperature in Celsius.
    /// </summary>
    public float Temperature { get; init; }
}

/// <summary>
/// Attitude estimate message.
/// Payload: int64 time_us, float roll/pitch/yaw (rad), float p/q/r (rad/s)
/// </summary>
public sealed record AttitudeMessage : IMessage
{
    public MessageType Type => MessageType.Attitude;
    public int PayloadSize => 32; // 8 + 4*6

    /// <summary>
    /// Timestamp in microseconds.
    /// </summary>
    public long TimeUs { get; init; }

    /// <summary>
    /// Roll angle (rad).
    /// </summary>
    public float Roll { get; init; }

    /// <summary>
    /// Pitch angle (rad).
    /// </summary>
    public float Pitch { get; init; }

    /// <summary>
    /// Yaw angle (rad).
    /// </summary>
    public float Yaw { get; init; }

    /// <summary>
    /// Roll rate (rad/s).
    /// </summary>
    public float P { get; init; }

    /// <summary>
    /// Pitch rate (rad/s).
    /// </summary>
    public float Q { get; init; }

    /// <summary>
    /// Yaw rate (rad/s).
    /// </summary>
    public float R { get; init; }
}

/// <summary>
/// GPS data message.
/// Payload: int64 time_us, double lat_deg, double lon_deg, float alt_m, float vn/ve/vd, float hAcc_m, vAcc_m
/// </summary>
public sealed record GpsMessage : IMessage
{
    public MessageType Type => MessageType.Gps;
    public int PayloadSize => 48; // 8 + 8 + 8 + 4*6

    /// <summary>
    /// Timestamp in microseconds.
    /// </summary>
    public long TimeUs { get; init; }

    /// <summary>
    /// Latitude in degrees.
    /// </summary>
    public double Latitude { get; init; }

    /// <summary>
    /// Longitude in degrees.
    /// </summary>
    public double Longitude { get; init; }

    /// <summary>
    /// Altitude in meters.
    /// </summary>
    public float Altitude { get; init; }

    /// <summary>
    /// North velocity (m/s).
    /// </summary>
    public float VelocityNorth { get; init; }

    /// <summary>
    /// East velocity (m/s).
    /// </summary>
    public float VelocityEast { get; init; }

    /// <summary>
    /// Down velocity (m/s).
    /// </summary>
    public float VelocityDown { get; init; }

    /// <summary>
    /// Horizontal accuracy (m).
    /// </summary>
    public float HorizontalAccuracy { get; init; }

    /// <summary>
    /// Vertical accuracy (m).
    /// </summary>
    public float VerticalAccuracy { get; init; }
}

/// <summary>
/// 2D state estimate message.
/// Payload: int64 time_us, float x_m, y_m, yaw_rad, vx, vy, cov_xx, cov_xy, cov_yy, cov_yawyaw
/// </summary>
public sealed record StateEstimate2DMessage : IMessage
{
    public MessageType Type => MessageType.StateEstimate2D;
    public int PayloadSize => 44; // 8 + 4*9

    /// <summary>
    /// Timestamp in microseconds.
    /// </summary>
    public long TimeUs { get; init; }

    /// <summary>
    /// X position (m).
    /// </summary>
    public float X { get; init; }

    /// <summary>
    /// Y position (m).
    /// </summary>
    public float Y { get; init; }

    /// <summary>
    /// Yaw angle (rad).
    /// </summary>
    public float Yaw { get; init; }

    /// <summary>
    /// X velocity (m/s).
    /// </summary>
    public float VelocityX { get; init; }

    /// <summary>
    /// Y velocity (m/s).
    /// </summary>
    public float VelocityY { get; init; }

    /// <summary>
    /// Covariance XX.
    /// </summary>
    public float CovarianceXX { get; init; }

    /// <summary>
    /// Covariance XY.
    /// </summary>
    public float CovarianceXY { get; init; }

    /// <summary>
    /// Covariance YY.
    /// </summary>
    public float CovarianceYY { get; init; }

    /// <summary>
    /// Covariance YawYaw.
    /// </summary>
    public float CovarianceYawYaw { get; init; }
}

/// <summary>
/// Parameter value message.
/// Payload: char name[32], float value, uint8 type, uint8 flags, uint16 reserved
/// </summary>
public sealed record ParamValueMessage : IMessage
{
    public MessageType Type => MessageType.ParamValue;
    public int PayloadSize => 40; // 32 + 4 + 1 + 1 + 2

    /// <summary>
    /// Parameter name (ASCII, null-terminated).
    /// </summary>
    public string Name { get; init; } = string.Empty;

    /// <summary>
    /// Parameter value.
    /// </summary>
    public float Value { get; init; }

    /// <summary>
    /// Parameter type (0 = float).
    /// </summary>
    public byte ParamType { get; init; }

    /// <summary>
    /// Parameter flags.
    /// </summary>
    public byte Flags { get; init; }

    /// <summary>
    /// Reserved bytes.
    /// </summary>
    public ushort Reserved { get; init; }
}

/// <summary>
/// Parameter set command message.
/// Payload: char name[32], float value, uint8 type, uint8 reserved0, uint16 reserved1
/// </summary>
public sealed record ParamSetMessage : IMessage
{
    public MessageType Type => MessageType.ParamSet;
    public int PayloadSize => 40; // 32 + 4 + 1 + 1 + 2

    /// <summary>
    /// Parameter name (ASCII, null-terminated).
    /// </summary>
    public string Name { get; init; } = string.Empty;

    /// <summary>
    /// Parameter value to set.
    /// </summary>
    public float Value { get; init; }

    /// <summary>
    /// Parameter type (0 = float).
    /// </summary>
    public byte ParamType { get; init; }

    /// <summary>
    /// Reserved byte.
    /// </summary>
    public byte Reserved0 { get; init; }

    /// <summary>
    /// Reserved bytes.
    /// </summary>
    public ushort Reserved1 { get; init; }
}
