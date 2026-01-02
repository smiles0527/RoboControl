using ControlWorkbench.Protocol;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Tests;

public class ProtocolTests
{
    [Fact]
    public void Heartbeat_RoundTrip_Success()
    {
        var original = new HeartbeatMessage
        {
            UptimeMs = 123456,
            SystemState = 1,
            Armed = 1,
            Reserved = 0
        };

        byte[] encoded = MessageEncoder.Encode(original);
        
        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        Assert.NotNull(result.Message);
        
        var decoded = Assert.IsType<HeartbeatMessage>(result.Message);
        Assert.Equal(original.UptimeMs, decoded.UptimeMs);
        Assert.Equal(original.SystemState, decoded.SystemState);
        Assert.Equal(original.Armed, decoded.Armed);
    }

    [Fact]
    public void ImuRaw_RoundTrip_Success()
    {
        var original = new ImuRawMessage
        {
            TimeUs = 1000000,
            GyroX = 0.1f,
            GyroY = -0.2f,
            GyroZ = 0.05f,
            AccelX = 0.5f,
            AccelY = -0.3f,
            AccelZ = 9.81f,
            Temperature = 25.5f
        };

        byte[] encoded = MessageEncoder.Encode(original);
        
        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        var decoded = Assert.IsType<ImuRawMessage>(result.Message);
        
        Assert.Equal(original.TimeUs, decoded.TimeUs);
        Assert.Equal(original.GyroX, decoded.GyroX, 5);
        Assert.Equal(original.GyroY, decoded.GyroY, 5);
        Assert.Equal(original.GyroZ, decoded.GyroZ, 5);
        Assert.Equal(original.AccelX, decoded.AccelX, 5);
        Assert.Equal(original.AccelY, decoded.AccelY, 5);
        Assert.Equal(original.AccelZ, decoded.AccelZ, 5);
        Assert.Equal(original.Temperature, decoded.Temperature, 5);
    }

    [Fact]
    public void Attitude_RoundTrip_Success()
    {
        var original = new AttitudeMessage
        {
            TimeUs = 2000000,
            Roll = 0.1f,
            Pitch = -0.05f,
            Yaw = 1.57f,
            P = 0.01f,
            Q = -0.02f,
            R = 0.03f
        };

        byte[] encoded = MessageEncoder.Encode(original);
        
        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        var decoded = Assert.IsType<AttitudeMessage>(result.Message);
        
        Assert.Equal(original.TimeUs, decoded.TimeUs);
        Assert.Equal(original.Roll, decoded.Roll, 5);
        Assert.Equal(original.Pitch, decoded.Pitch, 5);
        Assert.Equal(original.Yaw, decoded.Yaw, 5);
    }

    [Fact]
    public void Gps_RoundTrip_Success()
    {
        var original = new GpsMessage
        {
            TimeUs = 3000000,
            Latitude = 47.6062,
            Longitude = -122.3321,
            Altitude = 100.5f,
            VelocityNorth = 1.2f,
            VelocityEast = -0.5f,
            VelocityDown = 0.1f,
            HorizontalAccuracy = 2.5f,
            VerticalAccuracy = 4.0f
        };

        byte[] encoded = MessageEncoder.Encode(original);
        
        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        var decoded = Assert.IsType<GpsMessage>(result.Message);
        
        Assert.Equal(original.Latitude, decoded.Latitude, 10);
        Assert.Equal(original.Longitude, decoded.Longitude, 10);
        Assert.Equal(original.Altitude, decoded.Altitude, 5);
    }

    [Fact]
    public void StateEstimate2D_RoundTrip_Success()
    {
        var original = new StateEstimate2DMessage
        {
            TimeUs = 4000000,
            X = 10.5f,
            Y = -5.3f,
            Yaw = 0.785f,
            VelocityX = 2.1f,
            VelocityY = -1.0f,
            CovarianceXX = 0.01f,
            CovarianceXY = 0.001f,
            CovarianceYY = 0.02f,
            CovarianceYawYaw = 0.005f
        };

        byte[] encoded = MessageEncoder.Encode(original);
        
        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        var decoded = Assert.IsType<StateEstimate2DMessage>(result.Message);
        
        Assert.Equal(original.X, decoded.X, 5);
        Assert.Equal(original.Y, decoded.Y, 5);
        Assert.Equal(original.Yaw, decoded.Yaw, 5);
    }

    [Fact]
    public void ParamValue_RoundTrip_Success()
    {
        var original = new ParamValueMessage
        {
            Name = "PID_RATE_P",
            Value = 0.15f,
            ParamType = 0,
            Flags = 1
        };

        byte[] encoded = MessageEncoder.Encode(original);
        
        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        var decoded = Assert.IsType<ParamValueMessage>(result.Message);
        
        Assert.Equal(original.Name, decoded.Name);
        Assert.Equal(original.Value, decoded.Value, 5);
        Assert.Equal(original.ParamType, decoded.ParamType);
    }

    [Fact]
    public void Decoder_CrcFailure_DetectedAndCounted()
    {
        var message = new HeartbeatMessage { UptimeMs = 1000 };
        byte[] encoded = MessageEncoder.Encode(message);
        
        // Corrupt the CRC
        encoded[^1] ^= 0xFF;

        var decoder = new MessageDecoder();
        decoder.AddData(encoded);
        var result = decoder.TryDecode();

        Assert.False(result.Success);
        Assert.True(decoder.Statistics.CrcErrors > 0 || decoder.Statistics.FramingErrors > 0);
    }

    [Fact]
    public void Decoder_PartialFrame_NeedMoreData()
    {
        var message = new HeartbeatMessage { UptimeMs = 1000 };
        byte[] encoded = MessageEncoder.Encode(message);
        
        // Only provide first half
        var decoder = new MessageDecoder();
        decoder.AddData(encoded.AsSpan(0, encoded.Length / 2));
        var result = decoder.TryDecode();

        Assert.False(result.Success);
        Assert.Equal(0, result.BytesConsumed);
    }

    [Fact]
    public void Decoder_MultipleFrames_DecodesAll()
    {
        var msg1 = new HeartbeatMessage { UptimeMs = 1000 };
        var msg2 = new ImuRawMessage { TimeUs = 2000, GyroX = 0.1f };
        
        byte[] encoded1 = MessageEncoder.Encode(msg1);
        byte[] encoded2 = MessageEncoder.Encode(msg2);
        
        byte[] combined = new byte[encoded1.Length + encoded2.Length];
        encoded1.CopyTo(combined, 0);
        encoded2.CopyTo(combined, encoded1.Length);

        var decoder = new MessageDecoder();
        decoder.AddData(combined);
        
        var messages = decoder.DecodeAll().ToList();
        
        Assert.Equal(2, messages.Count);
        Assert.IsType<HeartbeatMessage>(messages[0]);
        Assert.IsType<ImuRawMessage>(messages[1]);
    }

    [Fact]
    public void Decoder_ResyncAfterGarbage_FindsValidFrame()
    {
        var message = new HeartbeatMessage { UptimeMs = 5000 };
        byte[] encoded = MessageEncoder.Encode(message);
        
        // Add garbage before the valid frame
        byte[] garbage = [0x01, 0x02, 0x03, 0x04, 0x05];
        byte[] combined = new byte[garbage.Length + encoded.Length];
        garbage.CopyTo(combined, 0);
        encoded.CopyTo(combined, garbage.Length);

        var decoder = new MessageDecoder();
        decoder.AddData(combined);
        var result = decoder.TryDecode();

        Assert.True(result.Success);
        Assert.IsType<HeartbeatMessage>(result.Message);
        Assert.True(decoder.Statistics.FramingErrors > 0);
    }

    [Fact]
    public void Frame_CorrectSize()
    {
        var heartbeat = new HeartbeatMessage();
        byte[] encoded = MessageEncoder.Encode(heartbeat);
        
        // Header(6) + Payload(8) + CRC(4) = 18
        Assert.Equal(18, encoded.Length);
        
        // Check preamble
        Assert.Equal(0xAA, encoded[0]);
        Assert.Equal(0x55, encoded[1]);
        
        // Check version
        Assert.Equal(0x01, encoded[2]);
        
        // Check message type
        Assert.Equal((byte)MessageType.Heartbeat, encoded[3]);
    }
}
