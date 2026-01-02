using System.Buffers.Binary;
using System.IO.Hashing;
using System.Text;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Protocol;

/// <summary>
/// Encodes messages to the binary protocol format.
/// </summary>
public static class MessageEncoder
{
    /// <summary>
    /// Encodes a message to a byte array.
    /// </summary>
    public static byte[] Encode(IMessage message)
    {
        int payloadSize = message.PayloadSize;
        int totalSize = ProtocolConstants.HeaderSize + payloadSize + ProtocolConstants.CrcSize;
        byte[] buffer = new byte[totalSize];

        // Write header
        buffer[0] = ProtocolConstants.Preamble1;
        buffer[1] = ProtocolConstants.Preamble2;
        buffer[2] = ProtocolConstants.Version;
        buffer[3] = (byte)message.Type;
        BinaryPrimitives.WriteUInt16LittleEndian(buffer.AsSpan(4), (ushort)payloadSize);

        // Write payload
        Span<byte> payload = buffer.AsSpan(ProtocolConstants.HeaderSize, payloadSize);
        EncodePayload(message, payload);

        // Calculate and write CRC32 over header + payload
        uint crc = CalculateCrc32(buffer.AsSpan(0, ProtocolConstants.HeaderSize + payloadSize));
        BinaryPrimitives.WriteUInt32LittleEndian(buffer.AsSpan(ProtocolConstants.HeaderSize + payloadSize), crc);

        return buffer;
    }

    private static void EncodePayload(IMessage message, Span<byte> payload)
    {
        switch (message)
        {
            case HeartbeatMessage hb:
                EncodeHeartbeat(hb, payload);
                break;
            case ImuRawMessage imu:
                EncodeImuRaw(imu, payload);
                break;
            case AttitudeMessage att:
                EncodeAttitude(att, payload);
                break;
            case GpsMessage gps:
                EncodeGps(gps, payload);
                break;
            case StateEstimate2DMessage se:
                EncodeStateEstimate2D(se, payload);
                break;
            case ParamValueMessage pv:
                EncodeParamValue(pv, payload);
                break;
            case ParamSetMessage ps:
                EncodeParamSet(ps, payload);
                break;
            default:
                throw new ArgumentException($"Unknown message type: {message.GetType().Name}");
        }
    }

    private static void EncodeHeartbeat(HeartbeatMessage msg, Span<byte> payload)
    {
        BinaryPrimitives.WriteUInt32LittleEndian(payload, msg.UptimeMs);
        payload[4] = msg.SystemState;
        payload[5] = msg.Armed;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.Slice(6), msg.Reserved);
    }

    private static void EncodeImuRaw(ImuRawMessage msg, Span<byte> payload)
    {
        int offset = 0;
        BinaryPrimitives.WriteInt64LittleEndian(payload.Slice(offset), msg.TimeUs); offset += 8;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.GyroX); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.GyroY); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.GyroZ); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.AccelX); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.AccelY); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.AccelZ); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Temperature);
    }

    private static void EncodeAttitude(AttitudeMessage msg, Span<byte> payload)
    {
        int offset = 0;
        BinaryPrimitives.WriteInt64LittleEndian(payload.Slice(offset), msg.TimeUs); offset += 8;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Roll); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Pitch); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Yaw); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.P); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Q); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.R);
    }

    private static void EncodeGps(GpsMessage msg, Span<byte> payload)
    {
        int offset = 0;
        BinaryPrimitives.WriteInt64LittleEndian(payload.Slice(offset), msg.TimeUs); offset += 8;
        BinaryPrimitives.WriteDoubleLittleEndian(payload.Slice(offset), msg.Latitude); offset += 8;
        BinaryPrimitives.WriteDoubleLittleEndian(payload.Slice(offset), msg.Longitude); offset += 8;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Altitude); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.VelocityNorth); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.VelocityEast); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.VelocityDown); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.HorizontalAccuracy); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.VerticalAccuracy);
    }

    private static void EncodeStateEstimate2D(StateEstimate2DMessage msg, Span<byte> payload)
    {
        int offset = 0;
        BinaryPrimitives.WriteInt64LittleEndian(payload.Slice(offset), msg.TimeUs); offset += 8;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.X); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Y); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.Yaw); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.VelocityX); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.VelocityY); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.CovarianceXX); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.CovarianceXY); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.CovarianceYY); offset += 4;
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(offset), msg.CovarianceYawYaw);
    }

    private static void EncodeParamValue(ParamValueMessage msg, Span<byte> payload)
    {
        WriteFixedString(payload.Slice(0, ProtocolConstants.ParamNameSize), msg.Name);
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(32), msg.Value);
        payload[36] = msg.ParamType;
        payload[37] = msg.Flags;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.Slice(38), msg.Reserved);
    }

    private static void EncodeParamSet(ParamSetMessage msg, Span<byte> payload)
    {
        WriteFixedString(payload.Slice(0, ProtocolConstants.ParamNameSize), msg.Name);
        BinaryPrimitives.WriteSingleLittleEndian(payload.Slice(32), msg.Value);
        payload[36] = msg.ParamType;
        payload[37] = msg.Reserved0;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.Slice(38), msg.Reserved1);
    }

    private static void WriteFixedString(Span<byte> destination, string value)
    {
        destination.Clear();
        int length = Math.Min(value.Length, destination.Length - 1);
        Encoding.ASCII.GetBytes(value.AsSpan(0, length), destination);
    }

    private static uint CalculateCrc32(ReadOnlySpan<byte> data)
    {
        return Crc32.HashToUInt32(data);
    }
}
