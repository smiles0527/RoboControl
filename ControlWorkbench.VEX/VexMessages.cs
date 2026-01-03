using System.Buffers.Binary;

namespace ControlWorkbench.VEX;

/// <summary>
/// VEX V5 telemetry message parser and builder.
/// </summary>
public class VexMessageCodec
{
    private readonly byte[] _rxBuffer = new byte[512];
    private int _rxIndex = 0;
    private ParseState _state = ParseState.WaitingPreamble1;

    private enum ParseState
    {
        WaitingPreamble1,
        WaitingPreamble2,
        WaitingType,
        WaitingLength,
        ReadingPayload,
        WaitingChecksum
    }

    private VexMessageType _currentType;
    private int _currentLength;
    private int _payloadIndex;

    public event Action<VexMessage>? MessageReceived;

    /// <summary>
    /// Process incoming bytes from the V5 brain.
    /// </summary>
    public void ProcessBytes(ReadOnlySpan<byte> data)
    {
        foreach (byte b in data)
        {
            ProcessByte(b);
        }
    }

    private void ProcessByte(byte b)
    {
        switch (_state)
        {
            case ParseState.WaitingPreamble1:
                if (b == VexConstants.Preamble1)
                    _state = ParseState.WaitingPreamble2;
                break;

            case ParseState.WaitingPreamble2:
                if (b == VexConstants.Preamble2)
                    _state = ParseState.WaitingType;
                else
                    _state = ParseState.WaitingPreamble1;
                break;

            case ParseState.WaitingType:
                _currentType = (VexMessageType)b;
                _state = ParseState.WaitingLength;
                break;

            case ParseState.WaitingLength:
                _currentLength = b;
                _payloadIndex = 0;
                if (_currentLength == 0)
                    _state = ParseState.WaitingChecksum;
                else
                    _state = ParseState.ReadingPayload;
                break;

            case ParseState.ReadingPayload:
                _rxBuffer[_payloadIndex++] = b;
                if (_payloadIndex >= _currentLength)
                    _state = ParseState.WaitingChecksum;
                break;

            case ParseState.WaitingChecksum:
                byte expectedChecksum = ComputeChecksum(_currentType, _rxBuffer.AsSpan(0, _currentLength));
                if (b == expectedChecksum)
                {
                    var payload = new byte[_currentLength];
                    Array.Copy(_rxBuffer, payload, _currentLength);
                    MessageReceived?.Invoke(new VexMessage(_currentType, payload));
                }
                _state = ParseState.WaitingPreamble1;
                break;
        }
    }

    /// <summary>
    /// Build a message to send to the V5 brain.
    /// </summary>
    public static byte[] BuildMessage(VexMessageType type, ReadOnlySpan<byte> payload)
    {
        if (payload.Length > VexConstants.MaxPayloadSize)
            throw new ArgumentException($"Payload too large: {payload.Length} > {VexConstants.MaxPayloadSize}");

        var message = new byte[VexConstants.HeaderSize + payload.Length + VexConstants.ChecksumSize];
        message[0] = VexConstants.Preamble1;
        message[1] = VexConstants.Preamble2;
        message[2] = (byte)type;
        message[3] = (byte)payload.Length;
        payload.CopyTo(message.AsSpan(4));
        message[^1] = ComputeChecksum(type, payload);

        return message;
    }

    /// <summary>
    /// Build a message with no payload.
    /// </summary>
    public static byte[] BuildMessage(VexMessageType type)
    {
        return BuildMessage(type, ReadOnlySpan<byte>.Empty);
    }

    private static byte ComputeChecksum(VexMessageType type, ReadOnlySpan<byte> payload)
    {
        byte checksum = (byte)type;
        checksum ^= (byte)payload.Length;
        foreach (byte b in payload)
            checksum ^= b;
        return checksum;
    }
}

/// <summary>
/// Represents a parsed VEX message.
/// </summary>
public readonly struct VexMessage
{
    public VexMessageType Type { get; }
    public byte[] Payload { get; }
    public DateTime Timestamp { get; }

    public VexMessage(VexMessageType type, byte[] payload)
    {
        Type = type;
        Payload = payload;
        Timestamp = DateTime.UtcNow;
    }
}

/// <summary>
/// Telemetry data structures for VEX V5.
/// </summary>
public static class VexTelemetry
{
    /// <summary>
    /// Odometry data from the robot.
    /// </summary>
    public readonly record struct OdometryData(
        double X,           // inches
        double Y,           // inches
        double Theta,       // radians
        double VelocityX,   // inches/sec
        double VelocityY,   // inches/sec
        double AngularVelocity, // rad/sec
        uint Timestamp      // ms since start
    )
    {
        public static OdometryData Parse(ReadOnlySpan<byte> data)
        {
            return new OdometryData(
                BinaryPrimitives.ReadDoubleLittleEndian(data),
                BinaryPrimitives.ReadDoubleLittleEndian(data[8..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[16..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[24..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[32..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[40..]),
                BinaryPrimitives.ReadUInt32LittleEndian(data[48..])
            );
        }

        public byte[] ToBytes()
        {
            var data = new byte[52];
            BinaryPrimitives.WriteDoubleLittleEndian(data, X);
            BinaryPrimitives.WriteDoubleLittleEndian(data.AsSpan(8), Y);
            BinaryPrimitives.WriteDoubleLittleEndian(data.AsSpan(16), Theta);
            BinaryPrimitives.WriteDoubleLittleEndian(data.AsSpan(24), VelocityX);
            BinaryPrimitives.WriteDoubleLittleEndian(data.AsSpan(32), VelocityY);
            BinaryPrimitives.WriteDoubleLittleEndian(data.AsSpan(40), AngularVelocity);
            BinaryPrimitives.WriteUInt32LittleEndian(data.AsSpan(48), Timestamp);
            return data;
        }
    }

    /// <summary>
    /// IMU/Inertial sensor data.
    /// </summary>
    public readonly record struct ImuData(
        double Heading,     // degrees
        double Pitch,       // degrees
        double Roll,        // degrees
        double GyroX,       // deg/sec
        double GyroY,       // deg/sec
        double GyroZ,       // deg/sec
        double AccelX,      // g
        double AccelY,      // g
        double AccelZ       // g
    )
    {
        public static ImuData Parse(ReadOnlySpan<byte> data)
        {
            return new ImuData(
                BinaryPrimitives.ReadDoubleLittleEndian(data),
                BinaryPrimitives.ReadDoubleLittleEndian(data[8..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[16..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[24..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[32..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[40..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[48..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[56..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[64..])
            );
        }
    }

    /// <summary>
    /// Motor telemetry data.
    /// </summary>
    public readonly record struct MotorData(
        byte Port,
        double Position,        // degrees
        double Velocity,        // RPM
        double Current,         // mA
        double Voltage,         // mV
        double Temperature,     // Celsius
        double Power,           // Watts
        double Torque,          // Nm
        int TargetPosition,     // encoder ticks
        int TargetVelocity,     // RPM
        MotorBrakeMode BrakeMode,
        bool IsReversed,
        bool IsOverTemp,
        bool IsOverCurrent
    )
    {
        public static MotorData Parse(ReadOnlySpan<byte> data)
        {
            return new MotorData(
                data[0],
                BinaryPrimitives.ReadDoubleLittleEndian(data[1..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[9..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[17..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[25..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[33..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[41..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[49..]),
                BinaryPrimitives.ReadInt32LittleEndian(data[57..]),
                BinaryPrimitives.ReadInt32LittleEndian(data[61..]),
                (MotorBrakeMode)data[65],
                (data[66] & 0x01) != 0,
                (data[66] & 0x02) != 0,
                (data[66] & 0x04) != 0
            );
        }
    }

    /// <summary>
    /// PID controller state for debugging.
    /// </summary>
    public readonly record struct PidStateData(
        byte ControllerId,
        double Setpoint,
        double Measurement,
        double Error,
        double Integral,
        double Derivative,
        double Output,
        double Kp,
        double Ki,
        double Kd
    )
    {
        public static PidStateData Parse(ReadOnlySpan<byte> data)
        {
            return new PidStateData(
                data[0],
                BinaryPrimitives.ReadDoubleLittleEndian(data[1..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[9..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[17..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[25..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[33..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[41..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[49..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[57..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[65..])
            );
        }
    }

    /// <summary>
    /// Battery status data.
    /// </summary>
    public readonly record struct BatteryData(
        double Voltage,     // V
        double Current,     // A
        double Capacity,    // % remaining
        double Temperature  // Celsius
    )
    {
        public static BatteryData Parse(ReadOnlySpan<byte> data)
        {
            return new BatteryData(
                BinaryPrimitives.ReadDoubleLittleEndian(data),
                BinaryPrimitives.ReadDoubleLittleEndian(data[8..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[16..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[24..])
            );
        }
    }

    /// <summary>
    /// Controller input data.
    /// </summary>
    public readonly record struct ControllerData(
        sbyte LeftStickX,
        sbyte LeftStickY,
        sbyte RightStickX,
        sbyte RightStickY,
        ControllerButtons Buttons,
        ControllerButtons ButtonsPressed,  // Just pressed this frame
        ControllerButtons ButtonsReleased  // Just released this frame
    )
    {
        public static ControllerData Parse(ReadOnlySpan<byte> data)
        {
            return new ControllerData(
                (sbyte)data[0],
                (sbyte)data[1],
                (sbyte)data[2],
                (sbyte)data[3],
                (ControllerButtons)BinaryPrimitives.ReadUInt16LittleEndian(data[4..]),
                (ControllerButtons)BinaryPrimitives.ReadUInt16LittleEndian(data[6..]),
                (ControllerButtons)BinaryPrimitives.ReadUInt16LittleEndian(data[8..])
            );
        }
    }

    /// <summary>
    /// Rotation sensor data.
    /// </summary>
    public readonly record struct RotationData(
        byte Port,
        double Position,    // degrees
        double Velocity,    // deg/sec
        int RawPosition     // centidegrees
    )
    {
        public static RotationData Parse(ReadOnlySpan<byte> data)
        {
            return new RotationData(
                data[0],
                BinaryPrimitives.ReadDoubleLittleEndian(data[1..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[9..]),
                BinaryPrimitives.ReadInt32LittleEndian(data[17..])
            );
        }
    }

    /// <summary>
    /// Distance sensor data.
    /// </summary>
    public readonly record struct DistanceData(
        byte Port,
        double Distance,    // mm
        double Velocity,    // mm/sec (object approach rate)
        int Confidence,     // 0-100%
        int ObjectSize      // relative size
    )
    {
        public static DistanceData Parse(ReadOnlySpan<byte> data)
        {
            return new DistanceData(
                data[0],
                BinaryPrimitives.ReadDoubleLittleEndian(data[1..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[9..]),
                BinaryPrimitives.ReadInt32LittleEndian(data[17..]),
                BinaryPrimitives.ReadInt32LittleEndian(data[21..])
            );
        }
    }

    /// <summary>
    /// GPS sensor data.
    /// </summary>
    public readonly record struct GpsData(
        byte Port,
        double X,           // inches (field coordinates)
        double Y,           // inches
        double Heading,     // degrees
        double Pitch,       // degrees
        double Roll,        // degrees
        int Quality         // 0-100%
    )
    {
        public static GpsData Parse(ReadOnlySpan<byte> data)
        {
            return new GpsData(
                data[0],
                BinaryPrimitives.ReadDoubleLittleEndian(data[1..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[9..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[17..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[25..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[33..]),
                BinaryPrimitives.ReadInt32LittleEndian(data[41..])
            );
        }
    }

    /// <summary>
    /// Log message from robot.
    /// </summary>
    public readonly record struct LogMessage(
        LogLevel Level,
        string Message,
        uint Timestamp
    )
    {
        public static LogMessage Parse(ReadOnlySpan<byte> data)
        {
            var level = (LogLevel)data[0];
            var timestamp = BinaryPrimitives.ReadUInt32LittleEndian(data[1..]);
            var message = System.Text.Encoding.UTF8.GetString(data[5..]);
            return new LogMessage(level, message, timestamp);
        }
    }

    public enum LogLevel : byte
    {
        Debug = 0,
        Info = 1,
        Warning = 2,
        Error = 3
    }
}
