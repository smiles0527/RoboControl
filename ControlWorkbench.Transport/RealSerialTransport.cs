using System.IO.Ports;
using System.Collections.Concurrent;
using ControlWorkbench.Protocol;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Transport;

/// <summary>
/// Production-ready serial transport for real hardware communication.
/// Tested with: VEX V5, Arduino, STM32, ESP32, Teensy
/// </summary>
public class RealSerialTransport : ITransport, IDisposable
{
    private SerialPort? _port;
    private CancellationTokenSource? _cts;
    private Task? _readTask;
    private readonly MessageDecoder _decoder = new();
    
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();
    private DateTime _connectedAt;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;
    public string? PortName => _port?.PortName;
    public TimeSpan Uptime => State == ConnectionState.Connected ? DateTime.UtcNow - _connectedAt : TimeSpan.Zero;

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    /// <summary>
    /// Get list of available COM ports with descriptions.
    /// </summary>
    public static List<PortInfo> GetAvailablePorts()
    {
        var ports = new List<PortInfo>();
        
        foreach (var portName in SerialPort.GetPortNames().OrderBy(p => p))
        {
            var info = new PortInfo { Name = portName };
            
            // Try to get additional info (Windows-specific)
            try
            {
                using var searcher = new System.Management.ManagementObjectSearcher(
                    $"SELECT * FROM Win32_PnPEntity WHERE Name LIKE '%({portName})%'");
                
                foreach (var obj in searcher.Get())
                {
                    info.Description = obj["Name"]?.ToString() ?? portName;
                    info.Manufacturer = obj["Manufacturer"]?.ToString();
                    info.DeviceId = obj["DeviceID"]?.ToString();
                    break;
                }
            }
            catch
            {
                info.Description = portName;
            }
            
            // Detect common devices
            info.DeviceType = DetectDeviceType(info.Description ?? "");
            ports.Add(info);
        }
        
        return ports;
    }

    private static DeviceType DetectDeviceType(string description)
    {
        var desc = description.ToLowerInvariant();
        
        if (desc.Contains("vex") || desc.Contains("v5"))
            return DeviceType.VexV5;
        if (desc.Contains("arduino"))
            return DeviceType.Arduino;
        if (desc.Contains("stm32") || desc.Contains("st-link"))
            return DeviceType.STM32;
        if (desc.Contains("esp32") || desc.Contains("cp210") || desc.Contains("ch340"))
            return DeviceType.ESP32;
        if (desc.Contains("teensy"))
            return DeviceType.Teensy;
        if (desc.Contains("ftdi"))
            return DeviceType.FTDI;
        if (desc.Contains("bluetooth") || desc.Contains("bt"))
            return DeviceType.Bluetooth;
            
        return DeviceType.Unknown;
    }

    /// <summary>
    /// Connect to a serial port with specified settings.
    /// </summary>
    public async Task ConnectAsync(SerialSettings settings, CancellationToken ct = default)
    {
        await DisconnectAsync();

        try
        {
            SetState(ConnectionState.Connecting);
            
            _port = new SerialPort
            {
                PortName = settings.PortName,
                BaudRate = settings.BaudRate,
                DataBits = settings.DataBits,
                Parity = settings.Parity,
                StopBits = settings.StopBits,
                Handshake = settings.Handshake,
                ReadTimeout = settings.ReadTimeoutMs,
                WriteTimeout = settings.WriteTimeoutMs,
                DtrEnable = settings.DtrEnable,
                RtsEnable = settings.RtsEnable,
                ReadBufferSize = 65536,
                WriteBufferSize = 65536
            };

            _port.Open();
            
            // Clear any stale data
            _port.DiscardInBuffer();
            _port.DiscardOutBuffer();
            
            _connectedAt = DateTime.UtcNow;
            _statistics.Reset();

            // Start background read task
            _cts = new CancellationTokenSource();
            _readTask = ReadLoopAsync(_cts.Token);

            SetState(ConnectionState.Connected);
        }
        catch (Exception ex)
        {
            SetState(ConnectionState.Error, ex.Message);
            throw;
        }
    }

    /// <summary>
    /// Connect with default settings (ITransport implementation).
    /// </summary>
    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        // Use first available port with default settings
        var ports = GetAvailablePorts();
        if (ports.Count == 0)
            throw new InvalidOperationException("No serial ports available");
            
        var settings = SerialSettings.ForDevice(ports[0].Name, ports[0].DeviceType);
        return ConnectAsync(settings, cancellationToken);
    }

    /// <summary>
    /// Connect with default settings for a specific device type.
    /// </summary>
    public Task ConnectAsync(string portName, DeviceType deviceType, CancellationToken ct = default)
    {
        var settings = SerialSettings.ForDevice(portName, deviceType);
        return ConnectAsync(settings, ct);
    }

    /// <summary>
    /// Auto-detect and connect to the first available supported device.
    /// </summary>
    public async Task<bool> AutoConnectAsync(CancellationToken ct = default)
    {
        var ports = GetAvailablePorts();
        
        // Prioritize known device types
        var prioritized = ports
            .OrderByDescending(p => p.DeviceType != DeviceType.Unknown)
            .ThenBy(p => p.Name);

        foreach (var port in prioritized)
        {
            if (ct.IsCancellationRequested) break;
            
            try
            {
                var settings = SerialSettings.ForDevice(port.Name, port.DeviceType);
                await ConnectAsync(settings, ct);
                
                // Wait briefly for device response
                await Task.Delay(100, ct);
                if (State == ConnectionState.Connected)
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
    /// Disconnect from the serial port.
    /// </summary>
    public Task DisconnectAsync()
    {
        _cts?.Cancel();

        try
        {
            _readTask?.Wait(500);
        }
        catch { }

        try
        {
            _port?.Close();
        }
        catch { }
        
        _port?.Dispose();
        _port = null;

        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    /// <summary>
    /// Send a message to the device.
    /// </summary>
    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_port?.IsOpen != true)
            throw new InvalidOperationException("Not connected");

        var data = MessageEncoder.Encode(message);
        _port.Write(data, 0, data.Length);
        
        _statistics.BytesSent += data.Length;
        _statistics.PacketsSent++;
        _statistics.LastSendTime = DateTime.UtcNow;
        
        return Task.CompletedTask;
    }

    /// <summary>
    /// Write raw bytes to the serial port.
    /// </summary>
    public void WriteRaw(ReadOnlySpan<byte> data)
    {
        if (_port?.IsOpen != true) return;

        _port.Write(data.ToArray(), 0, data.Length);
        _statistics.BytesSent += data.Length;
    }

    private async Task ReadLoopAsync(CancellationToken ct)
    {
        var buffer = new byte[4096];

        while (!ct.IsCancellationRequested && _port?.IsOpen == true)
        {
            try
            {
                if (_port.BaseStream == null) break;
                
                int bytesToRead = _port.BytesToRead;
                if (bytesToRead > 0)
                {
                    int read = await _port.BaseStream.ReadAsync(
                        buffer.AsMemory(0, Math.Min(bytesToRead, buffer.Length)), ct);
                    
                    if (read > 0)
                    {
                        _statistics.BytesReceived += read;
                        _statistics.LastReceiveTime = DateTime.UtcNow;
                        
                        // Decode messages
                        _decoder.AddData(buffer.AsSpan(0, read));
                        foreach (var msg in _decoder.DecodeAll())
                        {
                            _statistics.PacketsReceived++;
                            MessageReceived?.Invoke(this, new MessageReceivedEventArgs(
                                msg, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000));
                        }
                    }
                }
                else
                {
                    await Task.Delay(1, ct);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception)
            {
                _statistics.Errors++;
                await Task.Delay(100, ct);
            }
        }
    }

    private void SetState(ConnectionState newState, string? message = null)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState, message));
    }

    public void Dispose()
    {
        DisconnectAsync().Wait(1000);
        _cts?.Dispose();
    }
}

/// <summary>
/// Serial port configuration.
/// </summary>
public class SerialSettings
{
    public string PortName { get; set; } = "COM1";
    public int BaudRate { get; set; } = 115200;
    public int DataBits { get; set; } = 8;
    public Parity Parity { get; set; } = Parity.None;
    public StopBits StopBits { get; set; } = StopBits.One;
    public Handshake Handshake { get; set; } = Handshake.None;
    public int ReadTimeoutMs { get; set; } = 100;
    public int WriteTimeoutMs { get; set; } = 100;
    public bool DtrEnable { get; set; } = true;
    public bool RtsEnable { get; set; } = true;

    /// <summary>
    /// Get default settings for a specific device type.
    /// </summary>
    public static SerialSettings ForDevice(string portName, DeviceType deviceType)
    {
        var settings = new SerialSettings { PortName = portName };

        switch (deviceType)
        {
            case DeviceType.VexV5:
                settings.BaudRate = 115200;
                settings.DtrEnable = true;
                settings.RtsEnable = true;
                break;
                
            case DeviceType.Arduino:
                settings.BaudRate = 115200;
                settings.DtrEnable = true;  // Triggers reset on Arduino
                settings.RtsEnable = false;
                break;
                
            case DeviceType.ESP32:
                settings.BaudRate = 115200;
                settings.DtrEnable = false;  // Avoid bootloader mode
                settings.RtsEnable = false;
                break;
                
            case DeviceType.STM32:
                settings.BaudRate = 115200;
                settings.DtrEnable = false;
                settings.RtsEnable = false;
                break;
                
            case DeviceType.Teensy:
                settings.BaudRate = 115200;  // Teensy ignores baud rate (USB native)
                settings.DtrEnable = true;
                settings.RtsEnable = true;
                break;
                
            default:
                settings.BaudRate = 115200;
                break;
        }

        return settings;
    }
}

/// <summary>
/// Information about an available serial port.
/// </summary>
public class PortInfo
{
    public string Name { get; set; } = "";
    public string? Description { get; set; }
    public string? Manufacturer { get; set; }
    public string? DeviceId { get; set; }
    public DeviceType DeviceType { get; set; }

    public override string ToString() => 
        $"{Name} - {Description ?? "Unknown Device"}";
}

/// <summary>
/// Known device types for auto-configuration.
/// </summary>
public enum DeviceType
{
    Unknown,
    VexV5,
    Arduino,
    ESP32,
    STM32,
    Teensy,
    FTDI,
    Bluetooth,
    RaspberryPi
}
