using System.IO.Ports;

namespace ControlWorkbench.VEX.Communication;

/// <summary>
/// Real V5 Brain communication over USB serial.
/// Implements the VEX V5 serial protocol for telemetry and commands.
/// </summary>
public class V5SerialConnection : IDisposable
{
    private SerialPort? _port;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private readonly byte[] _rxBuffer = new byte[4096];
    private int _rxIndex = 0;

    public event Action<V5TelemetryPacket>? TelemetryReceived;
    public event Action<string>? LogReceived;
    public event Action<bool>? ConnectionChanged;

    public bool IsConnected => _port?.IsOpen ?? false;
    public string PortName { get; private set; } = "";

    // V5 Protocol constants
    private const byte PACKET_START = 0xC9;
    private const byte PACKET_END = 0xAA;

    /// <summary>
    /// Find available V5 Brain serial ports.
    /// </summary>
    public static string[] FindV5Ports()
    {
        var ports = new List<string>();
        
        foreach (var portName in SerialPort.GetPortNames())
        {
            try
            {
                // VEX V5 Brain typically shows up as specific VID/PID
                // On Windows, we can check the port description
                // For now, return all COM ports and let user select
                ports.Add(portName);
            }
            catch { }
        }

        return ports.ToArray();
    }

    /// <summary>
    /// Connect to V5 Brain on specified port.
    /// </summary>
    public async Task<bool> ConnectAsync(string portName, CancellationToken ct = default)
    {
        try
        {
            Disconnect();

            _port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 100,
                WriteTimeout = 100,
                DtrEnable = true,
                RtsEnable = true
            };

            _port.Open();
            PortName = portName;

            // Start receive task
            _cts = new CancellationTokenSource();
            _receiveTask = Task.Run(() => ReceiveLoop(_cts.Token), _cts.Token);

            // Send handshake
            await SendHandshakeAsync(ct);

            ConnectionChanged?.Invoke(true);
            return true;
        }
        catch (Exception ex)
        {
            LogReceived?.Invoke($"Connection failed: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// Disconnect from V5 Brain.
    /// </summary>
    public void Disconnect()
    {
        _cts?.Cancel();
        
        try
        {
            _port?.Close();
        }
        catch { }

        _port?.Dispose();
        _port = null;
        PortName = "";

        ConnectionChanged?.Invoke(false);
    }

    /// <summary>
    /// Send a command to the V5 Brain.
    /// </summary>
    public async Task SendCommandAsync(V5Command command, CancellationToken ct = default)
    {
        if (!IsConnected) throw new InvalidOperationException("Not connected");

        var packet = BuildPacket(command);
        await _port!.BaseStream.WriteAsync(packet, ct);
    }

    /// <summary>
    /// Request specific telemetry data.
    /// </summary>
    public async Task RequestTelemetryAsync(TelemetryType type, CancellationToken ct = default)
    {
        var command = new V5Command
        {
            Type = V5CommandType.RequestTelemetry,
            Data = new byte[] { (byte)type }
        };
        await SendCommandAsync(command, ct);
    }

    /// <summary>
    /// Send motor command (for testing, not during competition).
    /// </summary>
    public async Task SetMotorPowerAsync(int port, int power, CancellationToken ct = default)
    {
        var command = new V5Command
        {
            Type = V5CommandType.SetMotor,
            Data = new byte[] { (byte)port, (byte)(power + 128) }
        };
        await SendCommandAsync(command, ct);
    }

    private async Task SendHandshakeAsync(CancellationToken ct)
    {
        var handshake = new V5Command
        {
            Type = V5CommandType.Handshake,
            Data = new byte[] { 0x01, 0x00 } // Version 1.0
        };
        await SendCommandAsync(handshake, ct);
    }

    private byte[] BuildPacket(V5Command command)
    {
        var packet = new List<byte>
        {
            PACKET_START,
            (byte)command.Type,
            (byte)command.Data.Length
        };
        packet.AddRange(command.Data);

        // Simple checksum
        byte checksum = 0;
        foreach (var b in command.Data)
            checksum ^= b;
        packet.Add(checksum);
        packet.Add(PACKET_END);

        return packet.ToArray();
    }

    private void ReceiveLoop(CancellationToken ct)
    {
        var buffer = new byte[256];

        while (!ct.IsCancellationRequested && _port?.IsOpen == true)
        {
            try
            {
                int bytesRead = _port.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    ProcessReceivedData(buffer.AsSpan(0, bytesRead));
                }
            }
            catch (TimeoutException) { }
            catch (Exception ex)
            {
                if (!ct.IsCancellationRequested)
                {
                    LogReceived?.Invoke($"Receive error: {ex.Message}");
                }
            }
        }
    }

    private void ProcessReceivedData(ReadOnlySpan<byte> data)
    {
        foreach (var b in data)
        {
            _rxBuffer[_rxIndex++] = b;

            if (_rxIndex >= 4 && _rxBuffer[_rxIndex - 1] == PACKET_END)
            {
                // Try to parse packet
                if (TryParsePacket(_rxBuffer.AsSpan(0, _rxIndex), out var packet))
                {
                    TelemetryReceived?.Invoke(packet);
                }
                _rxIndex = 0;
            }

            if (_rxIndex >= _rxBuffer.Length)
            {
                _rxIndex = 0; // Buffer overflow, reset
            }
        }
    }

    private bool TryParsePacket(ReadOnlySpan<byte> data, out V5TelemetryPacket packet)
    {
        packet = new V5TelemetryPacket();

        if (data.Length < 5 || data[0] != PACKET_START || data[^1] != PACKET_END)
            return false;

        var type = (TelemetryType)data[1];
        int length = data[2];

        if (data.Length < length + 5)
            return false;

        var payload = data.Slice(3, length);

        // Verify checksum
        byte checksum = 0;
        foreach (var b in payload)
            checksum ^= b;

        if (checksum != data[3 + length])
            return false;

        packet.Type = type;
        packet.Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        packet.RawData = payload.ToArray();

        // Parse based on type
        ParseTelemetryPayload(packet, payload);

        return true;
    }

    private void ParseTelemetryPayload(V5TelemetryPacket packet, ReadOnlySpan<byte> payload)
    {
        switch (packet.Type)
        {
            case TelemetryType.Motors:
                ParseMotorTelemetry(packet, payload);
                break;
            case TelemetryType.Sensors:
                ParseSensorTelemetry(packet, payload);
                break;
            case TelemetryType.Position:
                ParsePositionTelemetry(packet, payload);
                break;
            case TelemetryType.Battery:
                ParseBatteryTelemetry(packet, payload);
                break;
        }
    }

    private void ParseMotorTelemetry(V5TelemetryPacket packet, ReadOnlySpan<byte> payload)
    {
        // Each motor: port(1) + power(1) + velocity(2) + position(4) + temp(1) + current(2)
        int offset = 0;
        while (offset + 11 <= payload.Length)
        {
            int port = payload[offset];
            int power = payload[offset + 1] - 128;
            int velocity = BitConverter.ToInt16(payload.Slice(offset + 2, 2));
            int position = BitConverter.ToInt32(payload.Slice(offset + 4, 4));
            int temp = payload[offset + 8];
            int current = BitConverter.ToInt16(payload.Slice(offset + 9, 2));

            packet.Motors[port] = new MotorTelemetry
            {
                Port = port,
                Power = power,
                Velocity = velocity,
                Position = position,
                Temperature = temp,
                Current = current
            };

            offset += 11;
        }
    }

    private void ParseSensorTelemetry(V5TelemetryPacket packet, ReadOnlySpan<byte> payload)
    {
        // Parse IMU, rotation sensors, etc.
        if (payload.Length >= 12)
        {
            packet.ImuHeading = BitConverter.ToSingle(payload.Slice(0, 4));
            packet.ImuPitch = BitConverter.ToSingle(payload.Slice(4, 4));
            packet.ImuRoll = BitConverter.ToSingle(payload.Slice(8, 4));
        }
    }

    private void ParsePositionTelemetry(V5TelemetryPacket packet, ReadOnlySpan<byte> payload)
    {
        // Odometry position: x(4) + y(4) + heading(4)
        if (payload.Length >= 12)
        {
            packet.OdometryX = BitConverter.ToSingle(payload.Slice(0, 4));
            packet.OdometryY = BitConverter.ToSingle(payload.Slice(4, 4));
            packet.OdometryHeading = BitConverter.ToSingle(payload.Slice(8, 4));
        }
    }

    private void ParseBatteryTelemetry(V5TelemetryPacket packet, ReadOnlySpan<byte> payload)
    {
        if (payload.Length >= 4)
        {
            packet.BatteryVoltage = BitConverter.ToUInt16(payload.Slice(0, 2)) / 1000.0f;
            packet.BatteryPercent = payload[2];
            packet.BatteryTemperature = payload[3];
        }
    }

    public void Dispose()
    {
        Disconnect();
    }
}

public class V5Command
{
    public V5CommandType Type { get; set; }
    public byte[] Data { get; set; } = Array.Empty<byte>();
}

public enum V5CommandType : byte
{
    Handshake = 0x01,
    RequestTelemetry = 0x02,
    SetMotor = 0x10,
    SetPneumatic = 0x11,
    SetParameter = 0x20,
    StartAuton = 0x30,
    StopMotors = 0x31
}

public enum TelemetryType : byte
{
    Motors = 0x01,
    Sensors = 0x02,
    Position = 0x03,
    Battery = 0x04,
    Controller = 0x05,
    Competition = 0x06,
    Custom = 0xFF
}

public class V5TelemetryPacket
{
    public TelemetryType Type { get; set; }
    public long Timestamp { get; set; }
    public byte[] RawData { get; set; } = Array.Empty<byte>();

    // Parsed data
    public Dictionary<int, MotorTelemetry> Motors { get; } = new();
    
    public float ImuHeading { get; set; }
    public float ImuPitch { get; set; }
    public float ImuRoll { get; set; }
    
    public float OdometryX { get; set; }
    public float OdometryY { get; set; }
    public float OdometryHeading { get; set; }
    
    public float BatteryVoltage { get; set; }
    public int BatteryPercent { get; set; }
    public int BatteryTemperature { get; set; }
}

public class MotorTelemetry
{
    public int Port { get; set; }
    public int Power { get; set; }
    public int Velocity { get; set; }
    public int Position { get; set; }
    public int Temperature { get; set; }
    public int Current { get; set; }
}
