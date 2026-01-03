using System.IO.Ports;

namespace ControlWorkbench.VEX;

/// <summary>
/// Connection manager for V5 Brain communication.
/// </summary>
public class VexConnection : IDisposable
{
    private SerialPort? _serialPort;
    private readonly VexMessageCodec _codec;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private readonly object _lock = new();
    
    private bool _isConnected;
    private DateTime _lastHeartbeat;
    private readonly TimeSpan _heartbeatTimeout = TimeSpan.FromSeconds(2);

    public bool IsConnected => _isConnected && (DateTime.UtcNow - _lastHeartbeat) < _heartbeatTimeout;
    public string? PortName => _serialPort?.PortName;

    // Events
    public event Action<bool>? ConnectionChanged;
    public event Action<VexTelemetry.OdometryData>? OdometryReceived;
    public event Action<VexTelemetry.ImuData>? ImuReceived;
    public event Action<VexTelemetry.MotorData>? MotorReceived;
    public event Action<VexTelemetry.PidStateData>? PidStateReceived;
    public event Action<VexTelemetry.BatteryData>? BatteryReceived;
    public event Action<VexTelemetry.ControllerData>? ControllerReceived;
    public event Action<VexTelemetry.LogMessage>? LogReceived;
    public event Action<string, double>? ParameterReceived;
    public event Action<VexMessage>? RawMessageReceived;
    public event Action<string>? ErrorOccurred;

    public VexConnection()
    {
        _codec = new VexMessageCodec();
        _codec.MessageReceived += OnMessageReceived;
    }

    /// <summary>
    /// Get available COM ports that might be V5 brains.
    /// </summary>
    public static string[] GetAvailablePorts()
    {
        return SerialPort.GetPortNames();
    }

    /// <summary>
    /// Attempt to auto-detect and connect to a V5 brain.
    /// </summary>
    public async Task<bool> AutoConnectAsync(CancellationToken ct = default)
    {
        var ports = GetAvailablePorts();
        
        foreach (var port in ports)
        {
            try
            {
                if (await TryConnectAsync(port, ct))
                    return true;
            }
            catch
            {
                // Try next port
            }
        }

        return false;
    }

    /// <summary>
    /// Connect to a specific COM port.
    /// </summary>
    public async Task<bool> TryConnectAsync(string portName, CancellationToken ct = default)
    {
        Disconnect();

        try
        {
            _serialPort = new SerialPort(portName, VexConstants.DefaultBaudRate)
            {
                ReadTimeout = VexConstants.DefaultTimeoutMs,
                WriteTimeout = VexConstants.DefaultTimeoutMs,
                DtrEnable = true,
                RtsEnable = true
            };

            _serialPort.Open();
            _cts = new CancellationTokenSource();
            _receiveTask = ReceiveLoopAsync(_cts.Token);

            // Send heartbeat and wait for response
            SendMessage(VexMessageType.Heartbeat);
            
            // Wait for connection confirmation
            var timeout = DateTime.UtcNow.AddSeconds(2);
            while (DateTime.UtcNow < timeout && !ct.IsCancellationRequested)
            {
                if (_isConnected)
                    return true;
                await Task.Delay(50, ct);
            }

            // No response, disconnect
            Disconnect();
            return false;
        }
        catch (Exception ex)
        {
            ErrorOccurred?.Invoke($"Connection failed: {ex.Message}");
            Disconnect();
            return false;
        }
    }

    /// <summary>
    /// Disconnect from the V5 brain.
    /// </summary>
    public void Disconnect()
    {
        _cts?.Cancel();
        
        try
        {
            _receiveTask?.Wait(500);
        }
        catch { }

        lock (_lock)
        {
            _serialPort?.Close();
            _serialPort?.Dispose();
            _serialPort = null;
        }

        if (_isConnected)
        {
            _isConnected = false;
            ConnectionChanged?.Invoke(false);
        }
    }

    /// <summary>
    /// Send a message to the V5 brain.
    /// </summary>
    public void SendMessage(VexMessageType type, ReadOnlySpan<byte> payload = default)
    {
        lock (_lock)
        {
            if (_serialPort?.IsOpen != true)
                return;

            try
            {
                var message = VexMessageCodec.BuildMessage(type, payload);
                _serialPort.Write(message, 0, message.Length);
            }
            catch (Exception ex)
            {
                ErrorOccurred?.Invoke($"Send failed: {ex.Message}");
            }
        }
    }

    /// <summary>
    /// Send a parameter update to the robot.
    /// </summary>
    public void SetParameter(string name, double value)
    {
        var nameBytes = System.Text.Encoding.UTF8.GetBytes(name);
        var payload = new byte[nameBytes.Length + 1 + 8];
        nameBytes.CopyTo(payload, 0);
        payload[nameBytes.Length] = 0; // Null terminator
        BitConverter.GetBytes(value).CopyTo(payload, nameBytes.Length + 1);
        
        SendMessage(VexMessageType.SetParameter, payload);
    }

    /// <summary>
    /// Request a parameter value from the robot.
    /// </summary>
    public void GetParameter(string name)
    {
        var nameBytes = System.Text.Encoding.UTF8.GetBytes(name + "\0");
        SendMessage(VexMessageType.GetParameter, nameBytes);
    }

    /// <summary>
    /// Reset the robot's odometry.
    /// </summary>
    public void ResetOdometry(double x = 0, double y = 0, double theta = 0)
    {
        var payload = new byte[24];
        BitConverter.GetBytes(x).CopyTo(payload, 0);
        BitConverter.GetBytes(y).CopyTo(payload, 8);
        BitConverter.GetBytes(theta).CopyTo(payload, 16);
        SendMessage(VexMessageType.ResetOdometry, payload);
    }

    /// <summary>
    /// Send emergency stop command.
    /// </summary>
    public void EmergencyStop()
    {
        SendMessage(VexMessageType.EmergencyStop);
    }

    /// <summary>
    /// Start recording telemetry on the robot.
    /// </summary>
    public void StartRecording()
    {
        SendMessage(VexMessageType.StartRecording);
    }

    /// <summary>
    /// Stop recording telemetry on the robot.
    /// </summary>
    public void StopRecording()
    {
        SendMessage(VexMessageType.StopRecording);
    }

    private async Task ReceiveLoopAsync(CancellationToken ct)
    {
        var buffer = new byte[256];

        while (!ct.IsCancellationRequested)
        {
            try
            {
                if (_serialPort?.IsOpen != true)
                {
                    await Task.Delay(100, ct);
                    continue;
                }

                int bytesToRead = _serialPort.BytesToRead;
                if (bytesToRead > 0)
                {
                    int read = _serialPort.Read(buffer, 0, System.Math.Min(bytesToRead, buffer.Length));
                    _codec.ProcessBytes(buffer.AsSpan(0, read));
                }
                else
                {
                    await Task.Delay(5, ct);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                ErrorOccurred?.Invoke($"Receive error: {ex.Message}");
                await Task.Delay(100, ct);
            }
        }
    }

    private void OnMessageReceived(VexMessage message)
    {
        RawMessageReceived?.Invoke(message);

        switch (message.Type)
        {
            case VexMessageType.Heartbeat:
            case VexMessageType.Acknowledge:
                _lastHeartbeat = DateTime.UtcNow;
                if (!_isConnected)
                {
                    _isConnected = true;
                    ConnectionChanged?.Invoke(true);
                }
                break;

            case VexMessageType.Odometry:
                if (message.Payload.Length >= 52)
                {
                    var data = VexTelemetry.OdometryData.Parse(message.Payload);
                    OdometryReceived?.Invoke(data);
                }
                break;

            case VexMessageType.ImuData:
                if (message.Payload.Length >= 72)
                {
                    var data = VexTelemetry.ImuData.Parse(message.Payload);
                    ImuReceived?.Invoke(data);
                }
                break;

            case VexMessageType.MotorTelemetry:
                if (message.Payload.Length >= 67)
                {
                    var data = VexTelemetry.MotorData.Parse(message.Payload);
                    MotorReceived?.Invoke(data);
                }
                break;

            case VexMessageType.PidState:
                if (message.Payload.Length >= 73)
                {
                    var data = VexTelemetry.PidStateData.Parse(message.Payload);
                    PidStateReceived?.Invoke(data);
                }
                break;

            case VexMessageType.BatteryStatus:
                if (message.Payload.Length >= 32)
                {
                    var data = VexTelemetry.BatteryData.Parse(message.Payload);
                    BatteryReceived?.Invoke(data);
                }
                break;

            case VexMessageType.ControllerInput:
                if (message.Payload.Length >= 10)
                {
                    var data = VexTelemetry.ControllerData.Parse(message.Payload);
                    ControllerReceived?.Invoke(data);
                }
                break;

            case VexMessageType.ParameterValue:
                ParseParameterValue(message.Payload);
                break;

            case VexMessageType.LogMessage:
                if (message.Payload.Length >= 6)
                {
                    var log = VexTelemetry.LogMessage.Parse(message.Payload);
                    LogReceived?.Invoke(log);
                }
                break;

            case VexMessageType.Error:
                if (message.Payload.Length > 0)
                {
                    var errorMsg = System.Text.Encoding.UTF8.GetString(message.Payload);
                    ErrorOccurred?.Invoke($"Robot error: {errorMsg}");
                }
                break;
        }
    }

    private void ParseParameterValue(byte[] payload)
    {
        // Find null terminator
        int nullIndex = Array.IndexOf(payload, (byte)0);
        if (nullIndex < 0 || nullIndex + 9 > payload.Length)
            return;

        var name = System.Text.Encoding.UTF8.GetString(payload, 0, nullIndex);
        var value = BitConverter.ToDouble(payload, nullIndex + 1);
        ParameterReceived?.Invoke(name, value);
    }

    public void Dispose()
    {
        Disconnect();
        _cts?.Dispose();
    }
}

/// <summary>
/// Telemetry recorder for logging and playback.
/// </summary>
public class TelemetryRecorder
{
    private readonly List<TelemetryFrame> _frames = new();
    private readonly object _lock = new();
    private DateTime _startTime;
    private bool _isRecording;

    public bool IsRecording => _isRecording;
    public int FrameCount => _frames.Count;
    public TimeSpan Duration => _frames.Count > 0 ? 
        TimeSpan.FromMilliseconds(_frames[^1].TimestampMs) : TimeSpan.Zero;

    public void StartRecording()
    {
        lock (_lock)
        {
            _frames.Clear();
            _startTime = DateTime.UtcNow;
            _isRecording = true;
        }
    }

    public void StopRecording()
    {
        _isRecording = false;
    }

    public void RecordOdometry(VexTelemetry.OdometryData data)
    {
        if (!_isRecording) return;
        
        lock (_lock)
        {
            _frames.Add(new TelemetryFrame
            {
                TimestampMs = (int)(DateTime.UtcNow - _startTime).TotalMilliseconds,
                Type = TelemetryFrameType.Odometry,
                X = data.X,
                Y = data.Y,
                Theta = data.Theta,
                VelocityX = data.VelocityX,
                VelocityY = data.VelocityY,
                AngularVelocity = data.AngularVelocity
            });
        }
    }

    public void RecordMotor(VexTelemetry.MotorData data)
    {
        if (!_isRecording) return;

        lock (_lock)
        {
            _frames.Add(new TelemetryFrame
            {
                TimestampMs = (int)(DateTime.UtcNow - _startTime).TotalMilliseconds,
                Type = TelemetryFrameType.Motor,
                Port = data.Port,
                Position = data.Position,
                Velocity = data.Velocity,
                Current = data.Current,
                Temperature = data.Temperature
            });
        }
    }

    public void RecordPid(VexTelemetry.PidStateData data)
    {
        if (!_isRecording) return;

        lock (_lock)
        {
            _frames.Add(new TelemetryFrame
            {
                TimestampMs = (int)(DateTime.UtcNow - _startTime).TotalMilliseconds,
                Type = TelemetryFrameType.Pid,
                ControllerId = data.ControllerId,
                Setpoint = data.Setpoint,
                Measurement = data.Measurement,
                Error = data.Error,
                Output = data.Output
            });
        }
    }

    public IReadOnlyList<TelemetryFrame> GetFrames() => _frames.AsReadOnly();

    public IEnumerable<TelemetryFrame> GetFrames(TelemetryFrameType type) =>
        _frames.Where(f => f.Type == type);

    /// <summary>
    /// Export recording to CSV format.
    /// </summary>
    public string ExportToCsv(TelemetryFrameType type)
    {
        var sb = new System.Text.StringBuilder();
        var frames = GetFrames(type).ToList();

        switch (type)
        {
            case TelemetryFrameType.Odometry:
                sb.AppendLine("time_ms,x,y,theta,vel_x,vel_y,angular_vel");
                foreach (var f in frames)
                {
                    sb.AppendLine($"{f.TimestampMs},{f.X:F4},{f.Y:F4},{f.Theta:F4},{f.VelocityX:F4},{f.VelocityY:F4},{f.AngularVelocity:F4}");
                }
                break;

            case TelemetryFrameType.Motor:
                sb.AppendLine("time_ms,port,position,velocity,current,temperature");
                foreach (var f in frames)
                {
                    sb.AppendLine($"{f.TimestampMs},{f.Port},{f.Position:F2},{f.Velocity:F2},{f.Current:F2},{f.Temperature:F2}");
                }
                break;

            case TelemetryFrameType.Pid:
                sb.AppendLine("time_ms,controller_id,setpoint,measurement,error,output");
                foreach (var f in frames)
                {
                    sb.AppendLine($"{f.TimestampMs},{f.ControllerId},{f.Setpoint:F4},{f.Measurement:F4},{f.Error:F4},{f.Output:F4}");
                }
                break;
        }

        return sb.ToString();
    }

    public void Clear()
    {
        lock (_lock)
        {
            _frames.Clear();
        }
    }
}

public class TelemetryFrame
{
    public int TimestampMs { get; set; }
    public TelemetryFrameType Type { get; set; }

    // Odometry
    public double X { get; set; }
    public double Y { get; set; }
    public double Theta { get; set; }
    public double VelocityX { get; set; }
    public double VelocityY { get; set; }
    public double AngularVelocity { get; set; }

    // Motor
    public byte Port { get; set; }
    public double Position { get; set; }
    public double Velocity { get; set; }
    public double Current { get; set; }
    public double Temperature { get; set; }

    // PID
    public byte ControllerId { get; set; }
    public double Setpoint { get; set; }
    public double Measurement { get; set; }
    public double Error { get; set; }
    public double Output { get; set; }
}

public enum TelemetryFrameType
{
    Odometry,
    Motor,
    Imu,
    Pid,
    Controller,
    Battery,
    Sensor
}
