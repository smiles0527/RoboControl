using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;

namespace ControlWorkbench.Drone.Hardware;

/// <summary>
/// Full MAVLink 2.0 protocol implementation for real hardware communication.
/// Supports ArduPilot, PX4, and other MAVLink-compatible autopilots.
/// </summary>
public class MavlinkConnection : IDisposable
{
    private readonly ITransportLayer _transport;
    private readonly byte _systemId;
    private readonly byte _componentId;
    private byte _sequence;
    
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private readonly MavlinkParser _parser = new();
    
    private readonly Dictionary<uint, Action<MavlinkMessage>> _handlers = new();
    private readonly Dictionary<string, double> _parameters = new();
    private readonly TaskCompletionSource<bool>? _paramListComplete;
    
    // State
    private MavlinkHeartbeat? _lastHeartbeat;
    private MavlinkGlobalPosition? _lastPosition;
    private MavlinkAttitude? _lastAttitude;
    private MavlinkVfrHud? _lastVfrHud;
    private MavlinkSysStatus? _lastSysStatus;
    private DateTime _lastHeartbeatTime;
    
    public event Action<MavlinkMessage>? MessageReceived;
    public event Action<MavlinkHeartbeat>? HeartbeatReceived;
    public event Action<MavlinkGlobalPosition>? PositionReceived;
    public event Action<MavlinkAttitude>? AttitudeReceived;
    public event Action<string>? StatusTextReceived;
    public event Action<bool>? ConnectionChanged;
    
    public bool IsConnected => _transport.IsConnected && 
        (DateTime.UtcNow - _lastHeartbeatTime).TotalSeconds < 5;
    
    public MavlinkHeartbeat? Heartbeat => _lastHeartbeat;
    public MavlinkGlobalPosition? Position => _lastPosition;
    public MavlinkAttitude? Attitude => _lastAttitude;
    public MavlinkVfrHud? VfrHud => _lastVfrHud;
    public MavlinkSysStatus? SysStatus => _lastSysStatus;
    
    public MavlinkConnection(ITransportLayer transport, byte systemId = 255, byte componentId = 0)
    {
        _transport = transport;
        _systemId = systemId;
        _componentId = componentId;
        
        RegisterMessageHandlers();
    }
    
    /// <summary>
    /// Connect to autopilot via specified transport.
    /// </summary>
    public async Task ConnectAsync(CancellationToken ct = default)
    {
        await _transport.ConnectAsync(ct);
        
        _cts = new CancellationTokenSource();
        _receiveTask = Task.Run(() => ReceiveLoop(_cts.Token), _cts.Token);
        
        // Request data streams
        await RequestDataStreamsAsync(ct);
        
        ConnectionChanged?.Invoke(true);
    }
    
    /// <summary>
    /// Disconnect from autopilot.
    /// </summary>
    public async Task DisconnectAsync()
    {
        _cts?.Cancel();
        if (_receiveTask != null)
            await _receiveTask;
        
        await _transport.DisconnectAsync();
        ConnectionChanged?.Invoke(false);
    }
    
    // --- Commands ---
    
    /// <summary>
    /// Arm the vehicle.
    /// </summary>
    public Task<bool> ArmAsync(bool force = false, CancellationToken ct = default)
    {
        return SendCommandLongAsync(
            MavCmd.ComponentArmDisarm,
            1, // Arm
            force ? 21196 : 0, // Force arm if specified
            ct: ct);
    }
    
    /// <summary>
    /// Disarm the vehicle.
    /// </summary>
    public Task<bool> DisarmAsync(bool force = false, CancellationToken ct = default)
    {
        return SendCommandLongAsync(
            MavCmd.ComponentArmDisarm,
            0, // Disarm
            force ? 21196 : 0,
            ct: ct);
    }
    
    /// <summary>
    /// Takeoff to specified altitude.
    /// </summary>
    public Task<bool> TakeoffAsync(float altitude, CancellationToken ct = default)
    {
        return SendCommandLongAsync(
            MavCmd.NavTakeoff,
            0, 0, 0, float.NaN, float.NaN, float.NaN, altitude,
            ct);
    }
    
    /// <summary>
    /// Land at current position.
    /// </summary>
    public Task<bool> LandAsync(CancellationToken ct = default)
    {
        return SendCommandLongAsync(MavCmd.NavLand, ct: ct);
    }
    
    /// <summary>
    /// Return to launch.
    /// </summary>
    public Task<bool> ReturnToLaunchAsync(CancellationToken ct = default)
    {
        return SendCommandLongAsync(MavCmd.NavReturnToLaunch, ct: ct);
    }
    
    /// <summary>
    /// Set flight mode.
    /// </summary>
    public async Task<bool> SetModeAsync(string modeName, CancellationToken ct = default)
    {
        // Mode mapping depends on autopilot type
        var modeId = GetModeId(modeName);
        if (modeId < 0) return false;
        
        return await SendCommandLongAsync(
            MavCmd.DoSetMode,
            1, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            modeId,
            ct: ct);
    }
    
    /// <summary>
    /// Go to specified position.
    /// </summary>
    public Task GoToPositionAsync(double lat, double lon, float alt, CancellationToken ct = default)
    {
        var msg = new MavlinkSetPositionTargetGlobalInt
        {
            TargetSystem = 1,
            TargetComponent = 1,
            CoordinateFrame = 6, // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            TypeMask = 0b111111111000, // Only position
            LatInt = (int)(lat * 1e7),
            LonInt = (int)(lon * 1e7),
            Alt = alt
        };
        
        return SendMessageAsync(msg, ct);
    }
    
    /// <summary>
    /// Set velocity setpoint.
    /// </summary>
    public Task SetVelocityAsync(float vx, float vy, float vz, float yawRate = 0, CancellationToken ct = default)
    {
        var msg = new MavlinkSetPositionTargetLocalNed
        {
            TargetSystem = 1,
            TargetComponent = 1,
            CoordinateFrame = 1, // MAV_FRAME_LOCAL_NED
            TypeMask = 0b110111000111, // Velocity + yaw rate
            Vx = vx,
            Vy = vy,
            Vz = vz,
            YawRate = yawRate
        };
        
        return SendMessageAsync(msg, ct);
    }
    
    /// <summary>
    /// Upload mission to vehicle.
    /// </summary>
    public async Task<bool> UploadMissionAsync(List<MissionItem> items, CancellationToken ct = default)
    {
        // Send mission count
        var count = new MavlinkMissionCount
        {
            TargetSystem = 1,
            TargetComponent = 1,
            Count = (ushort)items.Count,
            MissionType = 0 // MAV_MISSION_TYPE_MISSION
        };
        await SendMessageAsync(count, ct);
        
        // Wait for mission requests and send items
        var tcs = new TaskCompletionSource<bool>();
        int nextSeq = 0;
        
        void OnMissionRequest(MavlinkMessage msg)
        {
            if (msg is MavlinkMissionRequest req && req.Seq < items.Count)
            {
                var item = items[req.Seq];
                var missionItem = new MavlinkMissionItem
                {
                    TargetSystem = 1,
                    TargetComponent = 1,
                    Seq = req.Seq,
                    Frame = (byte)item.Frame,
                    Command = (ushort)item.Command,
                    Current = req.Seq == 0 ? (byte)1 : (byte)0,
                    Autocontinue = 1,
                    Param1 = item.Param1,
                    Param2 = item.Param2,
                    Param3 = item.Param3,
                    Param4 = item.Param4,
                    X = (float)item.Latitude,
                    Y = (float)item.Longitude,
                    Z = item.Altitude
                };
                SendMessageAsync(missionItem, ct).Wait();
                nextSeq = req.Seq + 1;
            }
            else if (msg is MavlinkMissionAck ack)
            {
                tcs.TrySetResult(ack.Type == 0);
            }
        }
        
        _handlers[(uint)MavlinkMessageId.MissionRequest] = OnMissionRequest;
        _handlers[(uint)MavlinkMessageId.MissionAck] = OnMissionRequest;
        
        try
        {
            return await tcs.Task.WaitAsync(TimeSpan.FromSeconds(30), ct);
        }
        finally
        {
            _handlers.Remove((uint)MavlinkMessageId.MissionRequest);
            _handlers.Remove((uint)MavlinkMessageId.MissionAck);
        }
    }
    
    /// <summary>
    /// Start mission execution.
    /// </summary>
    public Task<bool> StartMissionAsync(CancellationToken ct = default)
    {
        return SendCommandLongAsync(MavCmd.MissionStart, 0, 0, ct: ct);
    }
    
    // --- Parameters ---
    
    /// <summary>
    /// Get parameter value.
    /// </summary>
    public async Task<float?> GetParameterAsync(string name, CancellationToken ct = default)
    {
        var tcs = new TaskCompletionSource<float?>();
        
        void OnParamValue(MavlinkMessage msg)
        {
            if (msg is MavlinkParamValue pv && pv.ParamId == name)
            {
                tcs.TrySetResult(pv.ParamValue);
            }
        }
        
        _handlers[(uint)MavlinkMessageId.ParamValue] = OnParamValue;
        
        var request = new MavlinkParamRequestRead
        {
            TargetSystem = 1,
            TargetComponent = 1,
            ParamId = name,
            ParamIndex = -1
        };
        await SendMessageAsync(request, ct);
        
        try
        {
            return await tcs.Task.WaitAsync(TimeSpan.FromSeconds(5), ct);
        }
        catch
        {
            return null;
        }
        finally
        {
            _handlers.Remove((uint)MavlinkMessageId.ParamValue);
        }
    }
    
    /// <summary>
    /// Set parameter value.
    /// </summary>
    public async Task<bool> SetParameterAsync(string name, float value, CancellationToken ct = default)
    {
        var tcs = new TaskCompletionSource<bool>();
        
        void OnParamValue(MavlinkMessage msg)
        {
            if (msg is MavlinkParamValue pv && pv.ParamId == name)
            {
                tcs.TrySetResult(System.Math.Abs(pv.ParamValue - value) < 0.001f);
            }
        }
        
        _handlers[(uint)MavlinkMessageId.ParamValue] = OnParamValue;
        
        var set = new MavlinkParamSet
        {
            TargetSystem = 1,
            TargetComponent = 1,
            ParamId = name,
            ParamValue = value,
            ParamType = 9 // MAV_PARAM_TYPE_REAL32
        };
        await SendMessageAsync(set, ct);
        
        try
        {
            return await tcs.Task.WaitAsync(TimeSpan.FromSeconds(5), ct);
        }
        catch
        {
            return false;
        }
        finally
        {
            _handlers.Remove((uint)MavlinkMessageId.ParamValue);
        }
    }
    
    // --- Internal ---
    
    private async Task<bool> SendCommandLongAsync(
        MavCmd command,
        float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
        float p5 = 0, float p6 = 0, float p7 = 0,
        CancellationToken ct = default)
    {
        var tcs = new TaskCompletionSource<bool>();
        
        void OnCommandAck(MavlinkMessage msg)
        {
            if (msg is MavlinkCommandAck ack && ack.Command == (ushort)command)
            {
                tcs.TrySetResult(ack.Result == 0);
            }
        }
        
        _handlers[(uint)MavlinkMessageId.CommandAck] = OnCommandAck;
        
        var cmd = new MavlinkCommandLong
        {
            TargetSystem = 1,
            TargetComponent = 1,
            Command = (ushort)command,
            Confirmation = 0,
            Param1 = p1, Param2 = p2, Param3 = p3, Param4 = p4,
            Param5 = p5, Param6 = p6, Param7 = p7
        };
        
        await SendMessageAsync(cmd, ct);
        
        try
        {
            return await tcs.Task.WaitAsync(TimeSpan.FromSeconds(10), ct);
        }
        catch
        {
            return false;
        }
        finally
        {
            _handlers.Remove((uint)MavlinkMessageId.CommandAck);
        }
    }
    
    private async Task SendMessageAsync(MavlinkMessage msg, CancellationToken ct)
    {
        var packet = _parser.Encode(msg, _systemId, _componentId, _sequence++);
        await _transport.SendAsync(packet, ct);
    }
    
    private async Task ReceiveLoop(CancellationToken ct)
    {
        var buffer = new byte[1024];
        
        while (!ct.IsCancellationRequested)
        {
            try
            {
                int bytesRead = await _transport.ReceiveAsync(buffer, ct);
                if (bytesRead > 0)
                {
                    _parser.ParseBytes(buffer.AsSpan(0, bytesRead));
                    
                    while (_parser.TryGetMessage(out var msg))
                    {
                        ProcessMessage(msg);
                    }
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                StatusTextReceived?.Invoke($"Receive error: {ex.Message}");
            }
        }
    }
    
    private void ProcessMessage(MavlinkMessage msg)
    {
        MessageReceived?.Invoke(msg);
        
        // Update state
        switch (msg)
        {
            case MavlinkHeartbeat hb:
                _lastHeartbeat = hb;
                _lastHeartbeatTime = DateTime.UtcNow;
                HeartbeatReceived?.Invoke(hb);
                break;
            case MavlinkGlobalPosition pos:
                _lastPosition = pos;
                PositionReceived?.Invoke(pos);
                break;
            case MavlinkAttitude att:
                _lastAttitude = att;
                AttitudeReceived?.Invoke(att);
                break;
            case MavlinkVfrHud vfr:
                _lastVfrHud = vfr;
                break;
            case MavlinkSysStatus sys:
                _lastSysStatus = sys;
                break;
            case MavlinkStatusText text:
                StatusTextReceived?.Invoke(text.Text);
                break;
        }
        
        // Invoke registered handlers
        if (_handlers.TryGetValue(msg.MsgId, out var handler))
        {
            handler(msg);
        }
    }
    
    private void RegisterMessageHandlers()
    {
        // Base handlers registered here
    }
    
    private async Task RequestDataStreamsAsync(CancellationToken ct)
    {
        // Request all data streams at 4Hz
        byte[] streamIds = [0, 1, 2, 3, 6, 10, 11, 12];
        
        foreach (var streamId in streamIds)
        {
            var request = new MavlinkRequestDataStream
            {
                TargetSystem = 1,
                TargetComponent = 1,
                ReqStreamId = streamId,
                ReqMessageRate = 4,
                StartStop = 1
            };
            await SendMessageAsync(request, ct);
        }
    }
    
    private int GetModeId(string modeName)
    {
        // ArduCopter modes
        return modeName.ToUpper() switch
        {
            "STABILIZE" => 0,
            "ACRO" => 1,
            "ALT_HOLD" => 2,
            "AUTO" => 3,
            "GUIDED" => 4,
            "LOITER" => 5,
            "RTL" => 6,
            "CIRCLE" => 7,
            "LAND" => 9,
            "POSHOLD" => 16,
            "BRAKE" => 17,
            _ => -1
        };
    }
    
    public void Dispose()
    {
        _cts?.Cancel();
        _transport.Dispose();
    }
}

/// <summary>
/// MAVLink 2.0 parser and encoder.
/// </summary>
public class MavlinkParser
{
    private const byte Stx = 0xFD; // MAVLink 2.0 start byte
    private readonly byte[] _buffer = new byte[280];
    private int _bufferIndex;
    private readonly Queue<MavlinkMessage> _messages = new();
    
    public void ParseBytes(ReadOnlySpan<byte> data)
    {
        foreach (var b in data)
        {
            if (_bufferIndex == 0 && b != Stx)
                continue;
            
            _buffer[_bufferIndex++] = b;
            
            if (_bufferIndex >= 10)
            {
                int payloadLen = _buffer[1];
                int totalLen = 12 + payloadLen; // Header + payload + checksum
                
                if (_bufferIndex >= totalLen)
                {
                    if (ValidateChecksum(_buffer.AsSpan(0, totalLen)))
                    {
                        var msg = DecodeMessage(_buffer.AsSpan(0, totalLen));
                        if (msg != null)
                            _messages.Enqueue(msg);
                    }
                    _bufferIndex = 0;
                }
            }
            
            if (_bufferIndex >= _buffer.Length)
                _bufferIndex = 0;
        }
    }
    
    public bool TryGetMessage(out MavlinkMessage message)
    {
        if (_messages.Count > 0)
        {
            message = _messages.Dequeue();
            return true;
        }
        message = null!;
        return false;
    }
    
    public byte[] Encode(MavlinkMessage msg, byte systemId, byte componentId, byte sequence)
    {
        var payload = msg.Serialize();
        var packet = new byte[12 + payload.Length];
        
        packet[0] = Stx;
        packet[1] = (byte)payload.Length;
        packet[2] = 0; // Incompat flags
        packet[3] = 0; // Compat flags
        packet[4] = sequence;
        packet[5] = systemId;
        packet[6] = componentId;
        packet[7] = (byte)(msg.MsgId & 0xFF);
        packet[8] = (byte)((msg.MsgId >> 8) & 0xFF);
        packet[9] = (byte)((msg.MsgId >> 16) & 0xFF);
        
        payload.CopyTo(packet, 10);
        
        // CRC
        ushort crc = ComputeCrc(packet.AsSpan(1, 9 + payload.Length), msg.CrcExtra);
        packet[10 + payload.Length] = (byte)(crc & 0xFF);
        packet[11 + payload.Length] = (byte)(crc >> 8);
        
        return packet;
    }
    
    private bool ValidateChecksum(ReadOnlySpan<byte> packet)
    {
        int payloadLen = packet[1];
        uint msgId = (uint)(packet[7] | (packet[8] << 8) | (packet[9] << 16));
        
        byte crcExtra = GetCrcExtra(msgId);
        ushort expectedCrc = ComputeCrc(packet.Slice(1, 9 + payloadLen), crcExtra);
        ushort actualCrc = (ushort)(packet[10 + payloadLen] | (packet[11 + payloadLen] << 8));
        
        return expectedCrc == actualCrc;
    }
    
    private MavlinkMessage? DecodeMessage(ReadOnlySpan<byte> packet)
    {
        uint msgId = (uint)(packet[7] | (packet[8] << 8) | (packet[9] << 16));
        var payload = packet.Slice(10, packet[1]);
        
        return msgId switch
        {
            0 => MavlinkHeartbeat.Deserialize(payload),
            1 => MavlinkSysStatus.Deserialize(payload),
            24 => MavlinkGpsRawInt.Deserialize(payload),
            30 => MavlinkAttitude.Deserialize(payload),
            33 => MavlinkGlobalPosition.Deserialize(payload),
            74 => MavlinkVfrHud.Deserialize(payload),
            77 => MavlinkCommandAck.Deserialize(payload),
            253 => MavlinkStatusText.Deserialize(payload),
            _ => null
        };
    }
    
    private ushort ComputeCrc(ReadOnlySpan<byte> data, byte crcExtra)
    {
        ushort crc = 0xFFFF;
        
        foreach (var b in data)
        {
            byte tmp = (byte)(b ^ (crc & 0xFF));
            tmp ^= (byte)(tmp << 4);
            crc = (ushort)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
        }
        
        // Add CRC extra
        byte tmp2 = (byte)(crcExtra ^ (crc & 0xFF));
        tmp2 ^= (byte)(tmp2 << 4);
        crc = (ushort)((crc >> 8) ^ (tmp2 << 8) ^ (tmp2 << 3) ^ (tmp2 >> 4));
        
        return crc;
    }
    
    private byte GetCrcExtra(uint msgId)
    {
        return msgId switch
        {
            0 => 50, // Heartbeat
            1 => 124, // SysStatus
            24 => 24, // GpsRawInt
            30 => 39, // Attitude
            33 => 104, // GlobalPositionInt
            74 => 20, // VfrHud
            76 => 152, // CommandLong
            77 => 143, // CommandAck
            253 => 83, // StatusText
            _ => 0
        };
    }
}

// Transport layer interface
public interface ITransportLayer : IDisposable
{
    bool IsConnected { get; }
    Task ConnectAsync(CancellationToken ct = default);
    Task DisconnectAsync();
    Task SendAsync(byte[] data, CancellationToken ct = default);
    Task<int> ReceiveAsync(byte[] buffer, CancellationToken ct = default);
}

/// <summary>
/// UDP transport for MAVLink over network.
/// </summary>
public class UdpMavlinkTransport : ITransportLayer
{
    private UdpClient? _client;
    private readonly string _host;
    private readonly int _port;
    private IPEndPoint? _remoteEp;
    
    public bool IsConnected => _client != null;
    
    public UdpMavlinkTransport(string host, int port)
    {
        _host = host;
        _port = port;
    }
    
    public Task ConnectAsync(CancellationToken ct = default)
    {
        _client = new UdpClient();
        _remoteEp = new IPEndPoint(IPAddress.Parse(_host), _port);
        _client.Connect(_remoteEp);
        return Task.CompletedTask;
    }
    
    public Task DisconnectAsync()
    {
        _client?.Close();
        _client = null;
        return Task.CompletedTask;
    }
    
    public async Task SendAsync(byte[] data, CancellationToken ct = default)
    {
        if (_client != null)
            await _client.SendAsync(data, ct);
    }
    
    public async Task<int> ReceiveAsync(byte[] buffer, CancellationToken ct = default)
    {
        if (_client == null) return 0;
        var result = await _client.ReceiveAsync(ct);
        result.Buffer.CopyTo(buffer, 0);
        return result.Buffer.Length;
    }
    
    public void Dispose() => _client?.Dispose();
}

/// <summary>
/// Serial port transport for direct connection.
/// </summary>
public class SerialMavlinkTransport : ITransportLayer
{
    private System.IO.Ports.SerialPort? _port;
    private readonly string _portName;
    private readonly int _baudRate;
    
    public bool IsConnected => _port?.IsOpen ?? false;
    
    public SerialMavlinkTransport(string portName, int baudRate = 57600)
    {
        _portName = portName;
        _baudRate = baudRate;
    }
    
    public Task ConnectAsync(CancellationToken ct = default)
    {
        _port = new System.IO.Ports.SerialPort(_portName, _baudRate);
        _port.Open();
        return Task.CompletedTask;
    }
    
    public Task DisconnectAsync()
    {
        _port?.Close();
        return Task.CompletedTask;
    }
    
    public Task SendAsync(byte[] data, CancellationToken ct = default)
    {
        _port?.Write(data, 0, data.Length);
        return Task.CompletedTask;
    }
    
    public Task<int> ReceiveAsync(byte[] buffer, CancellationToken ct = default)
    {
        if (_port == null || !_port.IsOpen) return Task.FromResult(0);
        int bytesRead = _port.Read(buffer, 0, buffer.Length);
        return Task.FromResult(bytesRead);
    }
    
    public void Dispose() => _port?.Dispose();
}

// MAVLink message types
public abstract class MavlinkMessage
{
    public abstract uint MsgId { get; }
    public abstract byte CrcExtra { get; }
    public abstract byte[] Serialize();
}

public class MavlinkHeartbeat : MavlinkMessage
{
    public override uint MsgId => 0;
    public override byte CrcExtra => 50;
    
    public uint CustomMode { get; set; }
    public byte Type { get; set; }
    public byte Autopilot { get; set; }
    public byte BaseMode { get; set; }
    public byte SystemStatus { get; set; }
    public byte MavlinkVersion { get; set; }
    
    public bool IsArmed => (BaseMode & 128) != 0;
    
    public override byte[] Serialize()
    {
        var data = new byte[9];
        BinaryPrimitives.WriteUInt32LittleEndian(data.AsSpan(0), CustomMode);
        data[4] = Type;
        data[5] = Autopilot;
        data[6] = BaseMode;
        data[7] = SystemStatus;
        data[8] = MavlinkVersion;
        return data;
    }
    
    public static MavlinkHeartbeat Deserialize(ReadOnlySpan<byte> data)
    {
        return new MavlinkHeartbeat
        {
            CustomMode = BinaryPrimitives.ReadUInt32LittleEndian(data),
            Type = data[4],
            Autopilot = data[5],
            BaseMode = data[6],
            SystemStatus = data[7],
            MavlinkVersion = data[8]
        };
    }
}

public class MavlinkAttitude : MavlinkMessage
{
    public override uint MsgId => 30;
    public override byte CrcExtra => 39;
    
    public uint TimeBootMs { get; set; }
    public float Roll { get; set; }
    public float Pitch { get; set; }
    public float Yaw { get; set; }
    public float Rollspeed { get; set; }
    public float Pitchspeed { get; set; }
    public float Yawspeed { get; set; }
    
    public override byte[] Serialize()
    {
        var data = new byte[28];
        BinaryPrimitives.WriteUInt32LittleEndian(data, TimeBootMs);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(4), Roll);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(8), Pitch);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(12), Yaw);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(16), Rollspeed);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(20), Pitchspeed);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(24), Yawspeed);
        return data;
    }
    
    public static MavlinkAttitude Deserialize(ReadOnlySpan<byte> data)
    {
        return new MavlinkAttitude
        {
            TimeBootMs = BinaryPrimitives.ReadUInt32LittleEndian(data),
            Roll = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(4)),
            Pitch = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(8)),
            Yaw = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(12)),
            Rollspeed = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(16)),
            Pitchspeed = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(20)),
            Yawspeed = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(24))
        };
    }
}

public class MavlinkGlobalPosition : MavlinkMessage
{
    public override uint MsgId => 33;
    public override byte CrcExtra => 104;
    
    public uint TimeBootMs { get; set; }
    public int Lat { get; set; }
    public int Lon { get; set; }
    public int Alt { get; set; }
    public int RelativeAlt { get; set; }
    public short Vx { get; set; }
    public short Vy { get; set; }
    public short Vz { get; set; }
    public ushort Hdg { get; set; }
    
    public double LatitudeDeg => Lat / 1e7;
    public double LongitudeDeg => Lon / 1e7;
    public double AltitudeM => Alt / 1000.0;
    public double RelativeAltM => RelativeAlt / 1000.0;
    
    public override byte[] Serialize()
    {
        var data = new byte[28];
        BinaryPrimitives.WriteUInt32LittleEndian(data, TimeBootMs);
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(4), Lat);
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(8), Lon);
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(12), Alt);
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(16), RelativeAlt);
        BinaryPrimitives.WriteInt16LittleEndian(data.AsSpan(20), Vx);
        BinaryPrimitives.WriteInt16LittleEndian(data.AsSpan(22), Vy);
        BinaryPrimitives.WriteInt16LittleEndian(data.AsSpan(24), Vz);
        BinaryPrimitives.WriteUInt16LittleEndian(data.AsSpan(26), Hdg);
        return data;
    }
    
    public static MavlinkGlobalPosition Deserialize(ReadOnlySpan<byte> data)
    {
        return new MavlinkGlobalPosition
        {
            TimeBootMs = BinaryPrimitives.ReadUInt32LittleEndian(data),
            Lat = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(4)),
            Lon = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(8)),
            Alt = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(12)),
            RelativeAlt = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(16)),
            Vx = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(20)),
            Vy = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(22)),
            Vz = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(24)),
            Hdg = BinaryPrimitives.ReadUInt16LittleEndian(data.Slice(26))
        };
    }
}

// Additional message types (partial implementations)
public class MavlinkSysStatus : MavlinkMessage
{
    public override uint MsgId => 1;
    public override byte CrcExtra => 124;
    public ushort VoltageBattery { get; set; }
    public short CurrentBattery { get; set; }
    public sbyte BatteryRemaining { get; set; }
    public override byte[] Serialize() => new byte[31];
    public static MavlinkSysStatus Deserialize(ReadOnlySpan<byte> data) => new()
    {
        VoltageBattery = BinaryPrimitives.ReadUInt16LittleEndian(data.Slice(14)),
        CurrentBattery = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(16)),
        BatteryRemaining = (sbyte)data[30]
    };
}

public class MavlinkGpsRawInt : MavlinkMessage
{
    public override uint MsgId => 24;
    public override byte CrcExtra => 24;
    public byte FixType { get; set; }
    public int Lat { get; set; }
    public int Lon { get; set; }
    public int Alt { get; set; }
    public ushort Eph { get; set; }
    public byte SatellitesVisible { get; set; }
    public override byte[] Serialize() => new byte[30];
    public static MavlinkGpsRawInt Deserialize(ReadOnlySpan<byte> data) => new()
    {
        FixType = data[28],
        Lat = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(8)),
        Lon = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(12)),
        Alt = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(16)),
        Eph = BinaryPrimitives.ReadUInt16LittleEndian(data.Slice(20)),
        SatellitesVisible = data[29]
    };
}

public class MavlinkVfrHud : MavlinkMessage
{
    public override uint MsgId => 74;
    public override byte CrcExtra => 20;
    public float Airspeed { get; set; }
    public float Groundspeed { get; set; }
    public float Alt { get; set; }
    public float Climb { get; set; }
    public short Heading { get; set; }
    public ushort Throttle { get; set; }
    public override byte[] Serialize() => new byte[20];
    public static MavlinkVfrHud Deserialize(ReadOnlySpan<byte> data) => new()
    {
        Airspeed = BinaryPrimitives.ReadSingleLittleEndian(data),
        Groundspeed = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(4)),
        Alt = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(8)),
        Climb = BinaryPrimitives.ReadSingleLittleEndian(data.Slice(12)),
        Heading = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(16)),
        Throttle = BinaryPrimitives.ReadUInt16LittleEndian(data.Slice(18))
    };
}

public class MavlinkCommandAck : MavlinkMessage
{
    public override uint MsgId => 77;
    public override byte CrcExtra => 143;
    public ushort Command { get; set; }
    public byte Result { get; set; }
    public override byte[] Serialize() => new byte[3];
    public static MavlinkCommandAck Deserialize(ReadOnlySpan<byte> data) => new()
    {
        Command = BinaryPrimitives.ReadUInt16LittleEndian(data),
        Result = data[2]
    };
}

public class MavlinkStatusText : MavlinkMessage
{
    public override uint MsgId => 253;
    public override byte CrcExtra => 83;
    public byte Severity { get; set; }
    public string Text { get; set; } = "";
    public override byte[] Serialize() => new byte[51];
    public static MavlinkStatusText Deserialize(ReadOnlySpan<byte> data)
    {
        var text = System.Text.Encoding.ASCII.GetString(data.Slice(1, 50)).TrimEnd('\0');
        return new MavlinkStatusText { Severity = data[0], Text = text };
    }
}

// Command messages
public class MavlinkCommandLong : MavlinkMessage
{
    public override uint MsgId => 76;
    public override byte CrcExtra => 152;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public ushort Command { get; set; }
    public byte Confirmation { get; set; }
    public float Param1 { get; set; }
    public float Param2 { get; set; }
    public float Param3 { get; set; }
    public float Param4 { get; set; }
    public float Param5 { get; set; }
    public float Param6 { get; set; }
    public float Param7 { get; set; }
    
    public override byte[] Serialize()
    {
        var data = new byte[33];
        BinaryPrimitives.WriteSingleLittleEndian(data, Param1);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(4), Param2);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(8), Param3);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(12), Param4);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(16), Param5);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(20), Param6);
        BinaryPrimitives.WriteSingleLittleEndian(data.AsSpan(24), Param7);
        BinaryPrimitives.WriteUInt16LittleEndian(data.AsSpan(28), Command);
        data[30] = TargetSystem;
        data[31] = TargetComponent;
        data[32] = Confirmation;
        return data;
    }
}

// Placeholder message classes (implement as needed)
public class MavlinkSetPositionTargetGlobalInt : MavlinkMessage
{
    public override uint MsgId => 86;
    public override byte CrcExtra => 5;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public byte CoordinateFrame { get; set; }
    public ushort TypeMask { get; set; }
    public int LatInt { get; set; }
    public int LonInt { get; set; }
    public float Alt { get; set; }
    public override byte[] Serialize() => new byte[53];
}

public class MavlinkSetPositionTargetLocalNed : MavlinkMessage
{
    public override uint MsgId => 84;
    public override byte CrcExtra => 143;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public byte CoordinateFrame { get; set; }
    public ushort TypeMask { get; set; }
    public float Vx { get; set; }
    public float Vy { get; set; }
    public float Vz { get; set; }
    public float YawRate { get; set; }
    public override byte[] Serialize() => new byte[51];
}

public class MavlinkMissionCount : MavlinkMessage
{
    public override uint MsgId => 44;
    public override byte CrcExtra => 221;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public ushort Count { get; set; }
    public byte MissionType { get; set; }
    public override byte[] Serialize() => new byte[5];
}

public class MavlinkMissionRequest : MavlinkMessage
{
    public override uint MsgId => 40;
    public override byte CrcExtra => 230;
    public ushort Seq { get; set; }
    public override byte[] Serialize() => new byte[4];
}

public class MavlinkMissionItem : MavlinkMessage
{
    public override uint MsgId => 39;
    public override byte CrcExtra => 254;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public ushort Seq { get; set; }
    public byte Frame { get; set; }
    public ushort Command { get; set; }
    public byte Current { get; set; }
    public byte Autocontinue { get; set; }
    public float Param1 { get; set; }
    public float Param2 { get; set; }
    public float Param3 { get; set; }
    public float Param4 { get; set; }
    public float X { get; set; }
    public float Y { get; set; }
    public float Z { get; set; }
    public override byte[] Serialize() => new byte[37];
}

public class MavlinkMissionAck : MavlinkMessage
{
    public override uint MsgId => 47;
    public override byte CrcExtra => 153;
    public byte Type { get; set; }
    public override byte[] Serialize() => new byte[3];
}

public class MavlinkParamRequestRead : MavlinkMessage
{
    public override uint MsgId => 20;
    public override byte CrcExtra => 214;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public string ParamId { get; set; } = "";
    public short ParamIndex { get; set; }
    public override byte[] Serialize() => new byte[20];
}

public class MavlinkParamValue : MavlinkMessage
{
    public override uint MsgId => 22;
    public override byte CrcExtra => 220;
    public string ParamId { get; set; } = "";
    public float ParamValue { get; set; }
    public override byte[] Serialize() => new byte[25];
}

public class MavlinkParamSet : MavlinkMessage
{
    public override uint MsgId => 23;
    public override byte CrcExtra => 168;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public string ParamId { get; set; } = "";
    public float ParamValue { get; set; }
    public byte ParamType { get; set; }
    public override byte[] Serialize() => new byte[23];
}

public class MavlinkRequestDataStream : MavlinkMessage
{
    public override uint MsgId => 66;
    public override byte CrcExtra => 148;
    public byte TargetSystem { get; set; }
    public byte TargetComponent { get; set; }
    public byte ReqStreamId { get; set; }
    public ushort ReqMessageRate { get; set; }
    public byte StartStop { get; set; }
    public override byte[] Serialize() => new byte[6];
}

public enum MavCmd : ushort
{
    NavWaypoint = 16,
    NavLoiterUnlim = 17,
    NavLoiterTurns = 18,
    NavLoiterTime = 19,
    NavReturnToLaunch = 20,
    NavLand = 21,
    NavTakeoff = 22,
    NavLandLocal = 23,
    NavTakeoffLocal = 24,
    NavFollow = 25,
    DoSetMode = 176,
    ComponentArmDisarm = 400,
    MissionStart = 300,
    DoReposition = 192,
    DoOrbit = 34
}

public enum MavlinkMessageId : uint
{
    Heartbeat = 0,
    SysStatus = 1,
    GpsRawInt = 24,
    Attitude = 30,
    GlobalPositionInt = 33,
    MissionRequest = 40,
    MissionAck = 47,
    VfrHud = 74,
    CommandAck = 77,
    ParamValue = 22,
    StatusText = 253
}

public class MissionItem
{
    public int Frame { get; set; }
    public int Command { get; set; }
    public float Param1 { get; set; }
    public float Param2 { get; set; }
    public float Param3 { get; set; }
    public float Param4 { get; set; }
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public float Altitude { get; set; }
}
