namespace ControlWorkbench.Drone.Devices;

/// <summary>
/// MAVLink-based flight controller implementation.
/// Works with ArduPilot, PX4, and other MAVLink-compatible systems.
/// </summary>
public class MavlinkFlightController : IFlightController
{
    public string Name { get; }
    public FlightControllerType Type { get; }
    public bool IsConnected { get; private set; }
    public FlightMode CurrentMode { get; private set; } = FlightMode.Disarmed;
    
    private Stream? _stream;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    
    // MAVLink state
    private byte _systemId = 1;
    private byte _componentId = 1;
    private byte _targetSystem = 1;
    private byte _targetComponent = 1;
    private ushort _sequenceNumber = 0;
    
    // Heartbeat
    private DateTime _lastHeartbeat = DateTime.MinValue;
    private const int HeartbeatTimeoutMs = 3000;
    
    public event Action<DroneTelemetry>? TelemetryReceived;
    public event Action<DroneStatus>? StatusChanged;
    public event Action<string>? LogReceived;
    
    public MavlinkFlightController(string name = "MAVLink FC", FlightControllerType type = FlightControllerType.ArduPilot)
    {
        Name = name;
        Type = type;
    }
    
    public async Task<bool> ConnectAsync(string connectionString, CancellationToken ct = default)
    {
        try
        {
            // Parse connection string
            // Formats: 
            //   "serial:/dev/ttyUSB0:57600"
            //   "udp:14550"
            //   "tcp:127.0.0.1:5760"
            
            if (connectionString.StartsWith("serial:"))
            {
                var parts = connectionString.Substring(7).Split(':');
                var port = new System.IO.Ports.SerialPort(parts[0], int.Parse(parts[1]));
                port.Open();
                _stream = port.BaseStream;
            }
            else if (connectionString.StartsWith("udp:"))
            {
                int port = int.Parse(connectionString.Substring(4));
                var client = new System.Net.Sockets.UdpClient(port);
                // Note: UdpClient doesn't directly provide a Stream, would need wrapper
                LogReceived?.Invoke($"UDP connection on port {port}");
            }
            else if (connectionString.StartsWith("tcp:"))
            {
                var parts = connectionString.Substring(4).Split(':');
                var client = new System.Net.Sockets.TcpClient();
                await client.ConnectAsync(parts[0], int.Parse(parts[1]), ct);
                _stream = client.GetStream();
            }
            
            IsConnected = true;
            
            // Start receive loop
            _cts = new CancellationTokenSource();
            _receiveTask = Task.Run(() => ReceiveLoopAsync(_cts.Token), _cts.Token);
            
            // Request data streams
            await RequestDataStreamsAsync(ct);
            
            LogReceived?.Invoke($"Connected to {Name}");
            return true;
        }
        catch (Exception ex)
        {
            LogReceived?.Invoke($"Connection failed: {ex.Message}");
            return false;
        }
    }
    
    public async Task DisconnectAsync()
    {
        _cts?.Cancel();
        
        if (_receiveTask != null)
            await _receiveTask;
        
        _stream?.Close();
        _stream = null;
        IsConnected = false;
        
        LogReceived?.Invoke("Disconnected");
    }
    
    public async Task<bool> ArmAsync(CancellationToken ct = default)
    {
        // MAVLink command: MAV_CMD_COMPONENT_ARM_DISARM (400)
        await SendCommandLongAsync(400, 1, 0, 0, 0, 0, 0, 0, ct);
        LogReceived?.Invoke("Arm command sent");
        return true;
    }
    
    public async Task<bool> DisarmAsync(CancellationToken ct = default)
    {
        // MAVLink command: MAV_CMD_COMPONENT_ARM_DISARM (400)
        await SendCommandLongAsync(400, 0, 0, 0, 0, 0, 0, 0, ct);
        LogReceived?.Invoke("Disarm command sent");
        return true;
    }
    
    public async Task SetFlightModeAsync(FlightMode mode, CancellationToken ct = default)
    {
        int modeId = MapFlightModeToId(mode);
        
        // Set mode using MAV_CMD_DO_SET_MODE (176)
        await SendCommandLongAsync(176, 1, modeId, 0, 0, 0, 0, 0, ct);
        LogReceived?.Invoke($"Set mode to {mode}");
    }
    
    public async Task SendRcChannelsAsync(RcChannels channels, CancellationToken ct = default)
    {
        // RC_CHANNELS_OVERRIDE message
        var channelArray = channels.ToArray();
        var payload = new byte[18];
        
        // Target system and component
        payload[0] = _targetSystem;
        payload[1] = _targetComponent;
        
        // Channels 1-8 as uint16
        for (int i = 0; i < 8; i++)
        {
            var value = (ushort)channelArray[i];
            payload[2 + i * 2] = (byte)(value & 0xFF);
            payload[3 + i * 2] = (byte)(value >> 8);
        }
        
        await SendMavlinkMessageAsync(70, payload, ct); // RC_CHANNELS_OVERRIDE = 70
    }
    
    /// <summary>
    /// Send a waypoint mission to the flight controller.
    /// </summary>
    public async Task UploadMissionAsync(List<Waypoint> waypoints, CancellationToken ct = default)
    {
        LogReceived?.Invoke($"Uploading mission with {waypoints.Count} waypoints");
        
        // Send MISSION_COUNT
        var countPayload = new byte[4];
        countPayload[0] = _targetSystem;
        countPayload[1] = _targetComponent;
        BitConverter.GetBytes((ushort)waypoints.Count).CopyTo(countPayload, 2);
        await SendMavlinkMessageAsync(44, countPayload, ct); // MISSION_COUNT = 44
        
        // Wait for MISSION_REQUEST messages and send MISSION_ITEM for each
        // (Simplified - real implementation needs proper handshaking)
        for (int i = 0; i < waypoints.Count; i++)
        {
            var wp = waypoints[i];
            var payload = CreateMissionItemPayload(i, wp);
            await SendMavlinkMessageAsync(39, payload, ct); // MISSION_ITEM = 39
            await Task.Delay(50, ct);
        }
        
        LogReceived?.Invoke("Mission upload complete");
    }
    
    /// <summary>
    /// Start the uploaded mission.
    /// </summary>
    public async Task StartMissionAsync(CancellationToken ct = default)
    {
        await SetFlightModeAsync(FlightMode.Auto, ct);
        LogReceived?.Invoke("Mission started");
    }
    
    /// <summary>
    /// Guided mode: Go to a specific location.
    /// </summary>
    public async Task GotoLocationAsync(double lat, double lon, double alt, CancellationToken ct = default)
    {
        // SET_POSITION_TARGET_GLOBAL_INT
        var payload = new byte[53];
        
        uint timeBootMs = (uint)Environment.TickCount;
        BitConverter.GetBytes(timeBootMs).CopyTo(payload, 0);
        
        payload[4] = _targetSystem;
        payload[5] = _targetComponent;
        payload[6] = 6; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        
        // Type mask - only position
        ushort typeMask = 0b0000111111111000;
        BitConverter.GetBytes(typeMask).CopyTo(payload, 7);
        
        // Position
        int latInt = (int)(lat * 1e7);
        int lonInt = (int)(lon * 1e7);
        BitConverter.GetBytes(latInt).CopyTo(payload, 9);
        BitConverter.GetBytes(lonInt).CopyTo(payload, 13);
        BitConverter.GetBytes((float)alt).CopyTo(payload, 17);
        
        await SendMavlinkMessageAsync(86, payload, ct); // SET_POSITION_TARGET_GLOBAL_INT = 86
        LogReceived?.Invoke($"Goto: {lat:F6}, {lon:F6}, {alt:F1}m");
    }
    
    /// <summary>
    /// Return to launch position.
    /// </summary>
    public async Task ReturnToLaunchAsync(CancellationToken ct = default)
    {
        await SetFlightModeAsync(FlightMode.RTL, ct);
    }
    
    /// <summary>
    /// Land at current position.
    /// </summary>
    public async Task LandAsync(CancellationToken ct = default)
    {
        await SetFlightModeAsync(FlightMode.Land, ct);
    }
    
    /// <summary>
    /// Takeoff to specified altitude.
    /// </summary>
    public async Task TakeoffAsync(double altitudeMeters, CancellationToken ct = default)
    {
        // MAV_CMD_NAV_TAKEOFF (22)
        await SendCommandLongAsync(22, 0, 0, 0, 0, 0, 0, (float)altitudeMeters, ct);
        LogReceived?.Invoke($"Takeoff to {altitudeMeters}m");
    }
    
    private async Task ReceiveLoopAsync(CancellationToken ct)
    {
        var buffer = new byte[280]; // Max MAVLink v1 message
        var telemetry = new DroneTelemetry();
        var status = new DroneStatus();
        
        while (!ct.IsCancellationRequested && _stream != null)
        {
            try
            {
                int bytesRead = await _stream.ReadAsync(buffer, ct);
                if (bytesRead > 0)
                {
                    // Parse MAVLink messages
                    ParseMavlinkMessages(buffer.AsSpan(0, bytesRead), telemetry, status);
                }
            }
            catch (OperationCanceledException) { break; }
            catch (Exception ex)
            {
                LogReceived?.Invoke($"Receive error: {ex.Message}");
            }
        }
    }
    
    private void ParseMavlinkMessages(ReadOnlySpan<byte> data, DroneTelemetry telemetry, DroneStatus status)
    {
        // Find MAVLink v1 start (0xFE) or v2 start (0xFD)
        for (int i = 0; i < data.Length; i++)
        {
            if (data[i] == 0xFE && i + 8 <= data.Length)
            {
                // MAVLink v1
                int len = data[i + 1];
                if (i + 8 + len <= data.Length)
                {
                    byte msgId = data[i + 5];
                    var payload = data.Slice(i + 6, len);
                    ProcessMessage(msgId, payload, telemetry, status);
                    i += 7 + len; // Skip to next potential message
                }
            }
        }
    }
    
    private void ProcessMessage(byte msgId, ReadOnlySpan<byte> payload, DroneTelemetry telemetry, DroneStatus status)
    {
        switch (msgId)
        {
            case 0: // HEARTBEAT
                _lastHeartbeat = DateTime.UtcNow;
                status.Connected = true;
                status.Armed = (payload[6] & 0x80) != 0;
                telemetry.Armed = status.Armed;
                telemetry.Mode = MapIdToFlightMode(payload[0]); // custom_mode
                CurrentMode = telemetry.Mode;
                StatusChanged?.Invoke(status);
                break;
                
            case 1: // SYS_STATUS
                status.BatteryOk = payload[14] > 20; // Remaining percent
                break;
                
            case 30: // ATTITUDE
                telemetry.Roll = BitConverter.ToSingle(payload.Slice(4, 4)) * 180 / System.Math.PI;
                telemetry.Pitch = BitConverter.ToSingle(payload.Slice(8, 4)) * 180 / System.Math.PI;
                telemetry.Yaw = BitConverter.ToSingle(payload.Slice(12, 4)) * 180 / System.Math.PI;
                telemetry.RollRate = BitConverter.ToSingle(payload.Slice(16, 4)) * 180 / System.Math.PI;
                telemetry.PitchRate = BitConverter.ToSingle(payload.Slice(20, 4)) * 180 / System.Math.PI;
                telemetry.YawRate = BitConverter.ToSingle(payload.Slice(24, 4)) * 180 / System.Math.PI;
                TelemetryReceived?.Invoke(telemetry);
                break;
                
            case 33: // GLOBAL_POSITION_INT
                telemetry.Latitude = BitConverter.ToInt32(payload.Slice(4, 4)) / 1e7;
                telemetry.Longitude = BitConverter.ToInt32(payload.Slice(8, 4)) / 1e7;
                telemetry.AltitudeMSL = BitConverter.ToInt32(payload.Slice(12, 4)) / 1000.0;
                telemetry.AltitudeRelative = BitConverter.ToInt32(payload.Slice(16, 4)) / 1000.0;
                telemetry.VelocityNorth = BitConverter.ToInt16(payload.Slice(20, 2)) / 100.0;
                telemetry.VelocityEast = BitConverter.ToInt16(payload.Slice(22, 2)) / 100.0;
                telemetry.VelocityDown = BitConverter.ToInt16(payload.Slice(24, 2)) / 100.0;
                telemetry.Heading = BitConverter.ToUInt16(payload.Slice(26, 2)) / 100.0;
                TelemetryReceived?.Invoke(telemetry);
                break;
                
            case 24: // GPS_RAW_INT
                telemetry.GpsFix = (GpsFixType)payload[0];
                telemetry.GpsSatellites = payload[17];
                telemetry.GpsHdop = BitConverter.ToUInt16(payload.Slice(14, 2)) / 100.0;
                status.GpsOk = telemetry.GpsFix >= GpsFixType.Fix3D && telemetry.GpsSatellites >= 6;
                break;
                
            case 147: // BATTERY_STATUS
                telemetry.BatteryVoltage = BitConverter.ToUInt16(payload.Slice(10, 2)) / 1000.0;
                telemetry.BatteryCurrent = BitConverter.ToInt16(payload.Slice(12, 2)) / 100.0;
                telemetry.BatteryRemaining = payload[35];
                break;
                
            case 253: // STATUSTEXT
                string text = System.Text.Encoding.ASCII.GetString(payload.Slice(1, 50)).TrimEnd('\0');
                LogReceived?.Invoke($"[FC] {text}");
                break;
        }
    }
    
    private async Task SendMavlinkMessageAsync(byte msgId, byte[] payload, CancellationToken ct)
    {
        if (_stream == null) return;
        
        // MAVLink v1 packet
        var packet = new byte[8 + payload.Length];
        packet[0] = 0xFE; // Start
        packet[1] = (byte)payload.Length;
        packet[2] = (byte)(_sequenceNumber++ & 0xFF);
        packet[3] = _systemId;
        packet[4] = _componentId;
        packet[5] = msgId;
        payload.CopyTo(packet, 6);
        
        // CRC
        ushort crc = CalculateCrc(packet.AsSpan(1, 5 + payload.Length), msgId);
        packet[6 + payload.Length] = (byte)(crc & 0xFF);
        packet[7 + payload.Length] = (byte)(crc >> 8);
        
        await _stream.WriteAsync(packet, ct);
    }
    
    private async Task SendCommandLongAsync(ushort command, float p1, float p2, float p3, 
        float p4, float p5, float p6, float p7, CancellationToken ct)
    {
        var payload = new byte[33];
        
        payload[0] = _targetSystem;
        payload[1] = _targetComponent;
        BitConverter.GetBytes(command).CopyTo(payload, 2);
        payload[4] = 0; // Confirmation
        BitConverter.GetBytes(p1).CopyTo(payload, 5);
        BitConverter.GetBytes(p2).CopyTo(payload, 9);
        BitConverter.GetBytes(p3).CopyTo(payload, 13);
        BitConverter.GetBytes(p4).CopyTo(payload, 17);
        BitConverter.GetBytes(p5).CopyTo(payload, 21);
        BitConverter.GetBytes(p6).CopyTo(payload, 25);
        BitConverter.GetBytes(p7).CopyTo(payload, 29);
        
        await SendMavlinkMessageAsync(76, payload, ct); // COMMAND_LONG = 76
    }
    
    private async Task RequestDataStreamsAsync(CancellationToken ct)
    {
        // Request common data streams at 10Hz
        byte[] streams = { 0, 1, 2, 3, 6, 10, 11, 12 };
        foreach (var streamId in streams)
        {
            var payload = new byte[6];
            payload[0] = _targetSystem;
            payload[1] = _targetComponent;
            payload[2] = streamId;
            BitConverter.GetBytes((ushort)10).CopyTo(payload, 3); // 10Hz
            payload[5] = 1; // Start
            
            await SendMavlinkMessageAsync(66, payload, ct); // REQUEST_DATA_STREAM = 66
            await Task.Delay(10, ct);
        }
    }
    
    private byte[] CreateMissionItemPayload(int seq, Waypoint wp)
    {
        var payload = new byte[37];
        
        payload[0] = _targetSystem;
        payload[1] = _targetComponent;
        BitConverter.GetBytes((ushort)seq).CopyTo(payload, 2);
        payload[4] = 3; // MAV_FRAME_GLOBAL_RELATIVE_ALT
        BitConverter.GetBytes((ushort)wp.Command).CopyTo(payload, 5);
        payload[7] = 0; // current
        payload[8] = 1; // autocontinue
        BitConverter.GetBytes((float)wp.Param1).CopyTo(payload, 9);
        BitConverter.GetBytes((float)wp.Param2).CopyTo(payload, 13);
        BitConverter.GetBytes((float)wp.Param3).CopyTo(payload, 17);
        BitConverter.GetBytes((float)wp.Param4).CopyTo(payload, 21);
        BitConverter.GetBytes((float)wp.Latitude).CopyTo(payload, 25);
        BitConverter.GetBytes((float)wp.Longitude).CopyTo(payload, 29);
        BitConverter.GetBytes((float)wp.Altitude).CopyTo(payload, 33);
        
        return payload;
    }
    
    private int MapFlightModeToId(FlightMode mode)
    {
        return mode switch
        {
            FlightMode.Stabilize => 0,
            FlightMode.Acro => 1,
            FlightMode.AltHold => 2,
            FlightMode.Auto => 3,
            FlightMode.Guided => 4,
            FlightMode.Loiter => 5,
            FlightMode.RTL => 6,
            FlightMode.Circle => 7,
            FlightMode.Land => 9,
            FlightMode.Drift => 11,
            FlightMode.Sport => 13,
            FlightMode.PosHold => 16,
            FlightMode.Brake => 17,
            FlightMode.SmartRTL => 21,
            _ => 0
        };
    }
    
    private FlightMode MapIdToFlightMode(int id)
    {
        return id switch
        {
            0 => FlightMode.Stabilize,
            1 => FlightMode.Acro,
            2 => FlightMode.AltHold,
            3 => FlightMode.Auto,
            4 => FlightMode.Guided,
            5 => FlightMode.Loiter,
            6 => FlightMode.RTL,
            7 => FlightMode.Circle,
            9 => FlightMode.Land,
            11 => FlightMode.Drift,
            13 => FlightMode.Sport,
            16 => FlightMode.PosHold,
            17 => FlightMode.Brake,
            21 => FlightMode.SmartRTL,
            _ => FlightMode.Stabilize
        };
    }
    
    private ushort CalculateCrc(ReadOnlySpan<byte> data, byte msgId)
    {
        // X.25 CRC with MAVLink CRC extra byte
        ushort crc = 0xFFFF;
        foreach (byte b in data)
        {
            byte tmp = (byte)(b ^ (crc & 0xFF));
            tmp ^= (byte)(tmp << 4);
            crc = (ushort)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
        }
        // CRC extra byte (varies by message - simplified)
        return crc;
    }
}

/// <summary>
/// Mission waypoint command.
/// </summary>
public class Waypoint
{
    public int Sequence { get; set; }
    public WaypointCommand Command { get; set; } = WaypointCommand.Waypoint;
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }      // Meters
    public double Param1 { get; set; }        // Hold time / radius
    public double Param2 { get; set; }        // Acceptance radius
    public double Param3 { get; set; }        // Pass through (0) / orbit
    public double Param4 { get; set; }        // Yaw angle
    
    public static Waypoint CreateNavWaypoint(double lat, double lon, double alt, double holdSeconds = 0)
    {
        return new Waypoint
        {
            Command = WaypointCommand.Waypoint,
            Latitude = lat,
            Longitude = lon,
            Altitude = alt,
            Param1 = holdSeconds
        };
    }
    
    public static Waypoint CreateTakeoff(double alt)
    {
        return new Waypoint
        {
            Command = WaypointCommand.Takeoff,
            Altitude = alt
        };
    }
    
    public static Waypoint CreateLand()
    {
        return new Waypoint { Command = WaypointCommand.Land };
    }
    
    public static Waypoint CreateRTL()
    {
        return new Waypoint { Command = WaypointCommand.ReturnToLaunch };
    }
    
    public static Waypoint CreateLoiterTime(double lat, double lon, double alt, double seconds)
    {
        return new Waypoint
        {
            Command = WaypointCommand.LoiterTime,
            Latitude = lat,
            Longitude = lon,
            Altitude = alt,
            Param1 = seconds
        };
    }
}

public enum WaypointCommand
{
    Waypoint = 16,          // MAV_CMD_NAV_WAYPOINT
    Loiter = 17,            // MAV_CMD_NAV_LOITER_UNLIM
    LoiterTurns = 18,       // MAV_CMD_NAV_LOITER_TURNS
    LoiterTime = 19,        // MAV_CMD_NAV_LOITER_TIME
    ReturnToLaunch = 20,    // MAV_CMD_NAV_RETURN_TO_LAUNCH
    Land = 21,              // MAV_CMD_NAV_LAND
    Takeoff = 22,           // MAV_CMD_NAV_TAKEOFF
    
    // Actions
    DoSetServo = 183,       // MAV_CMD_DO_SET_SERVO
    DoSetRelay = 181,       // MAV_CMD_DO_SET_RELAY
    DoChangeSpeed = 178,    // MAV_CMD_DO_CHANGE_SPEED
    DoSetCamTriggDist = 206,// Camera trigger
    DoGimbalPitch = 1000,   // Point camera
    
    // Conditionals
    ConditionDelay = 112,   // MAV_CMD_CONDITION_DELAY
    ConditionYaw = 115      // MAV_CMD_CONDITION_YAW
}

