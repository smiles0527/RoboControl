namespace ControlWorkbench.Protocol;

/// <summary>
/// Defines the message types supported by the protocol.
/// </summary>
public enum MessageType : byte
{
    /// <summary>
    /// Heartbeat message containing system status.
    /// </summary>
    Heartbeat = 0x01,

    /// <summary>
    /// Raw IMU data (gyroscope and accelerometer).
    /// </summary>
    ImuRaw = 0x02,

    /// <summary>
    /// Attitude estimate (Euler angles and rates).
    /// </summary>
    Attitude = 0x03,

    /// <summary>
    /// GPS position and velocity data.
    /// </summary>
    Gps = 0x04,

    /// <summary>
    /// 2D state estimate with covariance.
    /// </summary>
    StateEstimate2D = 0x05,

    /// <summary>
    /// Parameter value notification.
    /// </summary>
    ParamValue = 0x10,

    /// <summary>
    /// Parameter set command.
    /// </summary>
    ParamSet = 0x11
}

/// <summary>
/// Protocol constants.
/// </summary>
public static class ProtocolConstants
{
    /// <summary>
    /// First byte of the preamble.
    /// </summary>
    public const byte Preamble1 = 0xAA;

    /// <summary>
    /// Second byte of the preamble.
    /// </summary>
    public const byte Preamble2 = 0x55;

    /// <summary>
    /// Current protocol version.
    /// </summary>
    public const byte Version = 0x01;

    /// <summary>
    /// Header size: Preamble(2) + Version(1) + MessageType(1) + PayloadLength(2)
    /// </summary>
    public const int HeaderSize = 6;

    /// <summary>
    /// CRC size in bytes.
    /// </summary>
    public const int CrcSize = 4;

    /// <summary>
    /// Maximum payload size.
    /// </summary>
    public const int MaxPayloadSize = 4096;

    /// <summary>
    /// Parameter name fixed size.
    /// </summary>
    public const int ParamNameSize = 32;
}
