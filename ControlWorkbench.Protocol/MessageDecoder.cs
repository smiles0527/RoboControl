using System.Buffers.Binary;
using System.IO.Hashing;
using System.Text;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Protocol;

/// <summary>
/// Result of a decode operation.
/// </summary>
public readonly struct DecodeResult
{
    /// <summary>
    /// The decoded message, if successful.
    /// </summary>
    public IMessage? Message { get; init; }

    /// <summary>
    /// Number of bytes consumed from the input.
    /// </summary>
    public int BytesConsumed { get; init; }

    /// <summary>
    /// Whether decoding was successful.
    /// </summary>
    public bool Success { get; init; }

    /// <summary>
    /// Error message if decoding failed.
    /// </summary>
    public string? Error { get; init; }

    public static DecodeResult SuccessResult(IMessage message, int bytesConsumed) =>
        new() { Message = message, BytesConsumed = bytesConsumed, Success = true };

    public static DecodeResult Failure(string error, int bytesConsumed = 0) =>
        new() { Error = error, BytesConsumed = bytesConsumed, Success = false };

    public static DecodeResult NeedMoreData =>
        new() { Success = false, BytesConsumed = 0 };
}

/// <summary>
/// Decoder statistics for monitoring protocol health.
/// </summary>
public class DecoderStatistics
{
    public long FramesDecoded { get; set; }
    public long CrcErrors { get; set; }
    public long FramingErrors { get; set; }
    public long BytesScanned { get; set; }
    public long UnknownMessageTypes { get; set; }
    public long PayloadErrors { get; set; }

    public void Reset()
    {
        FramesDecoded = 0;
        CrcErrors = 0;
        FramingErrors = 0;
        BytesScanned = 0;
        UnknownMessageTypes = 0;
        PayloadErrors = 0;
    }
}

/// <summary>
/// Decodes messages from the binary protocol format.
/// Handles resynchronization on preamble after errors.
/// </summary>
public class MessageDecoder
{
    private readonly byte[] _buffer;
    private int _bufferOffset;
    private int _bufferLength;

    /// <summary>
    /// Gets the decoder statistics.
    /// </summary>
    public DecoderStatistics Statistics { get; } = new();

    /// <summary>
    /// Creates a new decoder with the specified buffer size.
    /// </summary>
    public MessageDecoder(int bufferSize = 8192)
    {
        _buffer = new byte[bufferSize];
        _bufferOffset = 0;
        _bufferLength = 0;
    }

    /// <summary>
    /// Adds data to the internal buffer.
    /// </summary>
    public void AddData(ReadOnlySpan<byte> data)
    {
        // Compact buffer if needed
        if (_bufferOffset > 0 && _bufferOffset + _bufferLength + data.Length > _buffer.Length)
        {
            Buffer.BlockCopy(_buffer, _bufferOffset, _buffer, 0, _bufferLength);
            _bufferOffset = 0;
        }

        // Check if we have room
        if (_bufferLength + data.Length > _buffer.Length - _bufferOffset)
        {
            // Overflow - drop oldest data
            int toDrop = _bufferLength + data.Length - (_buffer.Length - _bufferOffset);
            _bufferOffset += toDrop;
            _bufferLength -= toDrop;
            Statistics.FramingErrors++;
        }

        data.CopyTo(_buffer.AsSpan(_bufferOffset + _bufferLength));
        _bufferLength += data.Length;
    }

    /// <summary>
    /// Tries to decode the next message from the buffer.
    /// </summary>
    public DecodeResult TryDecode()
    {
        while (_bufferLength >= ProtocolConstants.HeaderSize)
        {
            var span = _buffer.AsSpan(_bufferOffset, _bufferLength);

            // Find preamble
            int preambleIndex = FindPreamble(span);
            if (preambleIndex < 0)
            {
                // No preamble found, discard all but last byte
                int toDiscard = _bufferLength - 1;
                Statistics.BytesScanned += toDiscard;
                _bufferOffset += toDiscard;
                _bufferLength = 1;
                return DecodeResult.NeedMoreData;
            }

            if (preambleIndex > 0)
            {
                // Discard bytes before preamble
                Statistics.BytesScanned += preambleIndex;
                Statistics.FramingErrors++;
                _bufferOffset += preambleIndex;
                _bufferLength -= preambleIndex;
                span = _buffer.AsSpan(_bufferOffset, _bufferLength);
            }

            // Check if we have the full header
            if (_bufferLength < ProtocolConstants.HeaderSize)
            {
                return DecodeResult.NeedMoreData;
            }

            // Validate version
            byte version = span[2];
            if (version != ProtocolConstants.Version)
            {
                // Skip this preamble and continue searching
                Statistics.FramingErrors++;
                _bufferOffset += 2;
                _bufferLength -= 2;
                continue;
            }

            // Read payload length
            ushort payloadLength = BinaryPrimitives.ReadUInt16LittleEndian(span.Slice(4));
            if (payloadLength > ProtocolConstants.MaxPayloadSize)
            {
                // Invalid length, skip preamble
                Statistics.FramingErrors++;
                _bufferOffset += 2;
                _bufferLength -= 2;
                continue;
            }

            int totalFrameSize = ProtocolConstants.HeaderSize + payloadLength + ProtocolConstants.CrcSize;

            // Check if we have the full frame
            if (_bufferLength < totalFrameSize)
            {
                return DecodeResult.NeedMoreData;
            }

            // Validate CRC
            var frameData = span.Slice(0, ProtocolConstants.HeaderSize + payloadLength);
            uint expectedCrc = BinaryPrimitives.ReadUInt32LittleEndian(
                span.Slice(ProtocolConstants.HeaderSize + payloadLength));
            uint actualCrc = Crc32.HashToUInt32(frameData);

            if (expectedCrc != actualCrc)
            {
                Statistics.CrcErrors++;
                // Skip preamble and continue
                _bufferOffset += 2;
                _bufferLength -= 2;
                continue;
            }

            // Decode message
            MessageType msgType = (MessageType)span[3];
            var payload = span.Slice(ProtocolConstants.HeaderSize, payloadLength);

            IMessage? message = DecodePayload(msgType, payload);
            if (message == null)
            {
                Statistics.UnknownMessageTypes++;
                // Consume frame and continue
                _bufferOffset += totalFrameSize;
                _bufferLength -= totalFrameSize;
                Statistics.BytesScanned += totalFrameSize;
                continue;
            }

            // Success - consume the frame
            _bufferOffset += totalFrameSize;
            _bufferLength -= totalFrameSize;
            Statistics.BytesScanned += totalFrameSize;
            Statistics.FramesDecoded++;

            return DecodeResult.SuccessResult(message, totalFrameSize);
        }

        return DecodeResult.NeedMoreData;
    }

    /// <summary>
    /// Decodes all available messages from the buffer.
    /// </summary>
    public IEnumerable<IMessage> DecodeAll()
    {
        while (true)
        {
            var result = TryDecode();
            if (!result.Success || result.Message == null)
                break;
            yield return result.Message;
        }
    }

    /// <summary>
    /// Clears the internal buffer.
    /// </summary>
    public void Reset()
    {
        _bufferOffset = 0;
        _bufferLength = 0;
    }

    private static int FindPreamble(ReadOnlySpan<byte> data)
    {
        for (int i = 0; i < data.Length - 1; i++)
        {
            if (data[i] == ProtocolConstants.Preamble1 && data[i + 1] == ProtocolConstants.Preamble2)
            {
                return i;
            }
        }
        return -1;
    }

    private IMessage? DecodePayload(MessageType msgType, ReadOnlySpan<byte> payload)
    {
        try
        {
            return msgType switch
            {
                MessageType.Heartbeat => DecodeHeartbeat(payload),
                MessageType.ImuRaw => DecodeImuRaw(payload),
                MessageType.Attitude => DecodeAttitude(payload),
                MessageType.Gps => DecodeGps(payload),
                MessageType.StateEstimate2D => DecodeStateEstimate2D(payload),
                MessageType.ParamValue => DecodeParamValue(payload),
                MessageType.ParamSet => DecodeParamSet(payload),
                _ => null
            };
        }
        catch
        {
            Statistics.PayloadErrors++;
            return null;
        }
    }

    private static HeartbeatMessage DecodeHeartbeat(ReadOnlySpan<byte> payload)
    {
        return new HeartbeatMessage
        {
            UptimeMs = BinaryPrimitives.ReadUInt32LittleEndian(payload),
            SystemState = payload[4],
            Armed = payload[5],
            Reserved = BinaryPrimitives.ReadUInt16LittleEndian(payload.Slice(6))
        };
    }

    private static ImuRawMessage DecodeImuRaw(ReadOnlySpan<byte> payload)
    {
        int offset = 0;
        long timeUs = BinaryPrimitives.ReadInt64LittleEndian(payload.Slice(offset)); offset += 8;
        float gx = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float gy = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float gz = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float ax = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float ay = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float az = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float temp = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset));

        return new ImuRawMessage
        {
            TimeUs = timeUs,
            GyroX = gx, GyroY = gy, GyroZ = gz,
            AccelX = ax, AccelY = ay, AccelZ = az,
            Temperature = temp
        };
    }

    private static AttitudeMessage DecodeAttitude(ReadOnlySpan<byte> payload)
    {
        int offset = 0;
        long timeUs = BinaryPrimitives.ReadInt64LittleEndian(payload.Slice(offset)); offset += 8;
        float roll = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float pitch = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float yaw = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float p = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float q = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float r = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset));

        return new AttitudeMessage
        {
            TimeUs = timeUs,
            Roll = roll, Pitch = pitch, Yaw = yaw,
            P = p, Q = q, R = r
        };
    }

    private static GpsMessage DecodeGps(ReadOnlySpan<byte> payload)
    {
        int offset = 0;
        long timeUs = BinaryPrimitives.ReadInt64LittleEndian(payload.Slice(offset)); offset += 8;
        double lat = BinaryPrimitives.ReadDoubleLittleEndian(payload.Slice(offset)); offset += 8;
        double lon = BinaryPrimitives.ReadDoubleLittleEndian(payload.Slice(offset)); offset += 8;
        float alt = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float vn = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float ve = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float vd = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float hAcc = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float vAcc = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset));

        return new GpsMessage
        {
            TimeUs = timeUs,
            Latitude = lat, Longitude = lon,
            Altitude = alt,
            VelocityNorth = vn, VelocityEast = ve, VelocityDown = vd,
            HorizontalAccuracy = hAcc, VerticalAccuracy = vAcc
        };
    }

    private static StateEstimate2DMessage DecodeStateEstimate2D(ReadOnlySpan<byte> payload)
    {
        int offset = 0;
        long timeUs = BinaryPrimitives.ReadInt64LittleEndian(payload.Slice(offset)); offset += 8;
        float x = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float y = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float yaw = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float vx = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float vy = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float covXX = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float covXY = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float covYY = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset)); offset += 4;
        float covYawYaw = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(offset));

        return new StateEstimate2DMessage
        {
            TimeUs = timeUs,
            X = x, Y = y, Yaw = yaw,
            VelocityX = vx, VelocityY = vy,
            CovarianceXX = covXX, CovarianceXY = covXY, CovarianceYY = covYY, CovarianceYawYaw = covYawYaw
        };
    }

    private static ParamValueMessage DecodeParamValue(ReadOnlySpan<byte> payload)
    {
        string name = ReadFixedString(payload.Slice(0, ProtocolConstants.ParamNameSize));
        float value = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(32));
        byte paramType = payload[36];
        byte flags = payload[37];
        ushort reserved = BinaryPrimitives.ReadUInt16LittleEndian(payload.Slice(38));

        return new ParamValueMessage
        {
            Name = name,
            Value = value,
            ParamType = paramType,
            Flags = flags,
            Reserved = reserved
        };
    }

    private static ParamSetMessage DecodeParamSet(ReadOnlySpan<byte> payload)
    {
        string name = ReadFixedString(payload.Slice(0, ProtocolConstants.ParamNameSize));
        float value = BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(32));
        byte paramType = payload[36];
        byte reserved0 = payload[37];
        ushort reserved1 = BinaryPrimitives.ReadUInt16LittleEndian(payload.Slice(38));

        return new ParamSetMessage
        {
            Name = name,
            Value = value,
            ParamType = paramType,
            Reserved0 = reserved0,
            Reserved1 = reserved1
        };
    }

    private static string ReadFixedString(ReadOnlySpan<byte> data)
    {
        int nullIndex = data.IndexOf((byte)0);
        if (nullIndex >= 0)
        {
            data = data.Slice(0, nullIndex);
        }
        return Encoding.ASCII.GetString(data);
    }
}
