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
/// Parsed VEX message.
/// </summary>
public struct VexMessage
{
    public VexMessageType Type { get; }
    public byte[] Payload { get; }
    public DateTime ReceivedAt { get; }

    public VexMessage(VexMessageType type, byte[] payload)
    {
        Type = type;
        Payload = payload;
        ReceivedAt = DateTime.UtcNow;
    }

    /// <summary>
    /// Get the port number from the first byte of payload.
    /// </summary>
    public int Port => Payload.Length > 0 ? Payload[0] : 0;

    /// <summary>
    /// Get payload without the port byte.
    /// </summary>
    public ReadOnlySpan<byte> Data => Payload.Length > 1 ? Payload.AsSpan(1) : ReadOnlySpan<byte>.Empty;
}

/// <summary>
/// VEX telemetry data structures for parsing.
/// </summary>
public static class VexTelemetry
{
    /// <summary>
    /// Odometry data from robot.
    /// </summary>
    public readonly record struct OdometryData(
        double X,
        double Y,
        double Theta,
        double VelocityX,
        double VelocityY,
        double AngularVelocity,
        uint Timestamp
    )
    {
        public byte[] ToBytes()
        {
            var bytes = new byte[52];
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(0), X);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(8), Y);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(16), Theta);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(24), VelocityX);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(32), VelocityY);
            BinaryPrimitives.WriteDoubleLittleEndian(bytes.AsSpan(40), AngularVelocity);
            BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(48), Timestamp);
            return bytes;
        }

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
    }

    /// <summary>
    /// IMU sensor data.
    /// </summary>
    public readonly record struct ImuData(
        double Heading,
        double Pitch,
        double Roll,
        double GyroX,
        double GyroY,
        double GyroZ,
        double AccelX,
        double AccelY,
        double AccelZ
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
        double Position,
        double Velocity,
        double Current,
        double Voltage,
        double Temperature,
        double Power,
        double Torque
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
                BinaryPrimitives.ReadDoubleLittleEndian(data[49..])
            );
        }
    }

    /// <summary>
    /// Optical sensor (color) data.
    /// </summary>
    public readonly record struct OpticalData(
        byte Port,
        double Hue,
        double Saturation,
        double Brightness,
        byte Proximity
    )
    {
        public static OpticalData Parse(ReadOnlySpan<byte> data)
        {
            return new OpticalData(
                data[0],
                BinaryPrimitives.ReadDoubleLittleEndian(data[1..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[9..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[17..]),
                data[25]
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
        ControllerButtons ButtonsPressed,
        ControllerButtons ButtonsReleased
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
        double Position,
        double Velocity,
        int RawPosition
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
        double Distance,
        double Velocity,
        int Confidence,
        int ObjectSize
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
        double X,
        double Y,
        double Heading,
        double Pitch,
        double Roll,
        int Quality
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

    /// <summary>
    /// PID controller state data.
    /// </summary>
    public readonly record struct PidStateData(
        byte ControllerId,
        string Name,
        double Setpoint,
        double Measurement,
        double Error,
        double Output,
        double P,
        double I,
        double D,
        uint Timestamp
    )
    {
        public static PidStateData Parse(ReadOnlySpan<byte> data)
        {
            byte controllerId = data[0];
            int nameLen = data[1];
            string name = System.Text.Encoding.UTF8.GetString(data.Slice(2, nameLen));
            int offset = 2 + nameLen;

            return new PidStateData(
                controllerId,
                name,
                BinaryPrimitives.ReadDoubleLittleEndian(data[offset..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[(offset + 8)..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[(offset + 16)..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[(offset + 24)..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[(offset + 32)..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[(offset + 40)..]),
                BinaryPrimitives.ReadDoubleLittleEndian(data[(offset + 48)..]),
                BinaryPrimitives.ReadUInt32LittleEndian(data[(offset + 56)..])
            );
        }
    }

    /// <summary>
    /// Battery status data.
    /// </summary>
    public readonly record struct BatteryData(
        double Voltage,
        double Current,
        double Capacity,
        double Temperature
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

    public enum LogLevel : byte
    {
        Debug = 0,
        Info = 1,
        Warning = 2,
        Error = 3
    }
}
