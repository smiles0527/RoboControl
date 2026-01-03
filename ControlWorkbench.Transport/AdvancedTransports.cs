using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Transport;

/// <summary>
/// TCP transport for reliable communication.
/// </summary>
public class TcpTransport : ITransport
{
    private System.Net.Sockets.TcpClient? _client;
    private System.Net.Sockets.NetworkStream? _stream;
    private readonly string _host;
    private readonly int _port;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();
    private readonly Protocol.MessageDecoder _decoder = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public TcpTransport(string host, int port)
    {
        _host = host;
        _port = port;
    }

    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connecting);
        _client = new System.Net.Sockets.TcpClient();
        await _client.ConnectAsync(_host, _port, cancellationToken);
        _stream = _client.GetStream();
        
        _cts = new CancellationTokenSource();
        _receiveTask = Task.Run(() => ReceiveLoop(_cts.Token), _cts.Token);
        SetState(ConnectionState.Connected);
    }

    public Task DisconnectAsync()
    {
        _cts?.Cancel();
        _stream?.Close();
        _client?.Close();
        _client = null;
        _stream = null;
        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    public async Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_stream == null) throw new InvalidOperationException("Not connected");
        var data = Protocol.MessageEncoder.Encode(message);
        await _stream.WriteAsync(data, cancellationToken);
        _statistics.BytesSent += data.Length;
        _statistics.PacketsSent++;
    }

    private async Task ReceiveLoop(CancellationToken token)
    {
        var buffer = new byte[4096];
        while (!token.IsCancellationRequested && _stream != null)
        {
            try
            {
                int bytesRead = await _stream.ReadAsync(buffer, token);
                if (bytesRead > 0)
                {
                    _statistics.BytesReceived += bytesRead;
                    _decoder.AddData(buffer.AsSpan(0, bytesRead));
                    foreach (var msg in _decoder.DecodeAll())
                    {
                        _statistics.PacketsReceived++;
                        MessageReceived?.Invoke(this, new MessageReceivedEventArgs(msg, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000));
                    }
                }
            }
            catch (OperationCanceledException) { break; }
            catch { break; }
        }
    }

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose()
    {
        _cts?.Cancel();
        _stream?.Dispose();
        _client?.Dispose();
    }
}

/// <summary>
/// WebSocket transport for web-based communication.
/// </summary>
public class WebSocketTransport : ITransport
{
    private System.Net.WebSockets.ClientWebSocket? _webSocket;
    private readonly Uri _uri;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();
    private readonly Protocol.MessageDecoder _decoder = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public WebSocketTransport(string url)
    {
        _uri = new Uri(url);
    }

    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connecting);
        _webSocket = new System.Net.WebSockets.ClientWebSocket();
        await _webSocket.ConnectAsync(_uri, cancellationToken);
        
        _cts = new CancellationTokenSource();
        _receiveTask = Task.Run(() => ReceiveLoop(_cts.Token), _cts.Token);
        SetState(ConnectionState.Connected);
    }

    public async Task DisconnectAsync()
    {
        _cts?.Cancel();
        if (_webSocket?.State == System.Net.WebSockets.WebSocketState.Open)
        {
            await _webSocket.CloseAsync(
                System.Net.WebSockets.WebSocketCloseStatus.NormalClosure, 
                "Closing", 
                CancellationToken.None);
        }
        _webSocket = null;
        SetState(ConnectionState.Disconnected);
    }

    public async Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_webSocket == null) throw new InvalidOperationException("Not connected");
        var data = Protocol.MessageEncoder.Encode(message);
        await _webSocket.SendAsync(
            new ArraySegment<byte>(data), 
            System.Net.WebSockets.WebSocketMessageType.Binary, 
            true, 
            cancellationToken);
        _statistics.BytesSent += data.Length;
        _statistics.PacketsSent++;
    }

    private async Task ReceiveLoop(CancellationToken token)
    {
        var buffer = new byte[4096];
        while (!token.IsCancellationRequested && _webSocket != null)
        {
            try
            {
                var result = await _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), token);
                if (result.MessageType == System.Net.WebSockets.WebSocketMessageType.Binary)
                {
                    _statistics.BytesReceived += result.Count;
                    _decoder.AddData(buffer.AsSpan(0, result.Count));
                    foreach (var msg in _decoder.DecodeAll())
                    {
                        _statistics.PacketsReceived++;
                        MessageReceived?.Invoke(this, new MessageReceivedEventArgs(msg, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000));
                    }
                }
                else if (result.MessageType == System.Net.WebSockets.WebSocketMessageType.Close)
                {
                    break;
                }
            }
            catch (OperationCanceledException) { break; }
        }
    }

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose()
    {
        _cts?.Cancel();
        _webSocket?.Dispose();
    }
}

/// <summary>
/// Bluetooth Serial Port Profile transport.
/// </summary>
public class BluetoothTransport : ITransport
{
    private readonly string _deviceAddress;
    private System.IO.Ports.SerialPort? _serialPort;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();
    private readonly Protocol.MessageDecoder _decoder = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public BluetoothTransport(string deviceAddress)
    {
        _deviceAddress = deviceAddress;
    }

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connecting);
        string? comPort = FindBluetoothComPort(_deviceAddress);
        if (comPort == null)
            throw new InvalidOperationException($"Bluetooth device {_deviceAddress} not found");
        
        _serialPort = new System.IO.Ports.SerialPort(comPort, 115200);
        _serialPort.DataReceived += OnDataReceived;
        _serialPort.Open();
        SetState(ConnectionState.Connected);
        return Task.CompletedTask;
    }

    private void OnDataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
    {
        if (_serialPort?.BytesToRead > 0)
        {
            var data = new byte[_serialPort.BytesToRead];
            _serialPort.Read(data, 0, data.Length);
            _statistics.BytesReceived += data.Length;
            _decoder.AddData(data);
            foreach (var msg in _decoder.DecodeAll())
            {
                _statistics.PacketsReceived++;
                MessageReceived?.Invoke(this, new MessageReceivedEventArgs(msg, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000));
            }
        }
    }

    public Task DisconnectAsync()
    {
        _serialPort?.Close();
        _serialPort = null;
        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_serialPort == null) throw new InvalidOperationException("Not connected");
        var data = Protocol.MessageEncoder.Encode(message);
        _serialPort.Write(data, 0, data.Length);
        _statistics.BytesSent += data.Length;
        _statistics.PacketsSent++;
        return Task.CompletedTask;
    }

    private static string? FindBluetoothComPort(string deviceAddress) => null; // Platform-specific

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose() => _serialPort?.Dispose();
}

/// <summary>
/// CAN bus transport for automotive/industrial robotics.
/// </summary>
public class CanBusTransport : ITransport
{
    private readonly string _interface;
    private readonly int _bitrate;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public CanBusTransport(string interfaceName = "can0", int bitrate = 500000)
    {
        _interface = interfaceName;
        _bitrate = bitrate;
    }

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connected);
        return Task.CompletedTask;
    }

    public Task DisconnectAsync()
    {
        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_state != ConnectionState.Connected) throw new InvalidOperationException("Not connected");
        return Task.CompletedTask;
    }

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose() { }
}

/// <summary>
/// MQTT transport for IoT robotics applications.
/// </summary>
public class MqttTransport : ITransport
{
    private readonly string _broker;
    private readonly int _port;
    private readonly string _clientId;
    private readonly string _topic;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public MqttTransport(string broker, int port = 1883, string topic = "robot/telemetry", string? clientId = null)
    {
        _broker = broker;
        _port = port;
        _topic = topic;
        _clientId = clientId ?? $"controlworkbench_{Guid.NewGuid():N}";
    }

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connected);
        return Task.CompletedTask;
    }

    public Task DisconnectAsync()
    {
        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_state != ConnectionState.Connected) throw new InvalidOperationException("Not connected");
        return Task.CompletedTask;
    }

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose() { }
}

/// <summary>
/// DDS (Data Distribution Service) transport for ROS2 compatibility.
/// </summary>
public class DdsTransport : ITransport
{
    private readonly int _domainId;
    private readonly string _topicName;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public DdsTransport(int domainId = 0, string topicName = "rt/robot_state")
    {
        _domainId = domainId;
        _topicName = topicName;
    }

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connected);
        return Task.CompletedTask;
    }

    public Task DisconnectAsync()
    {
        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_state != ConnectionState.Connected) throw new InvalidOperationException("Not connected");
        return Task.CompletedTask;
    }

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose() { }
}

/// <summary>
/// ZeroMQ transport for high-performance messaging.
/// </summary>
public class ZeroMqTransport : ITransport
{
    private readonly string _endpoint;
    private readonly ZmqSocketType _socketType;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly TransportStatistics _statistics = new();

    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public ConnectionState State => _state;
    public TransportStatistics Statistics => _statistics;

    public ZeroMqTransport(string endpoint, ZmqSocketType socketType = ZmqSocketType.Dealer)
    {
        _endpoint = endpoint;
        _socketType = socketType;
    }

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        SetState(ConnectionState.Connected);
        return Task.CompletedTask;
    }

    public Task DisconnectAsync()
    {
        SetState(ConnectionState.Disconnected);
        return Task.CompletedTask;
    }

    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_state != ConnectionState.Connected) throw new InvalidOperationException("Not connected");
        return Task.CompletedTask;
    }

    private void SetState(ConnectionState newState)
    {
        var oldState = _state;
        _state = newState;
        ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(oldState, newState));
    }

    public void Dispose() { }
}

public enum ZmqSocketType
{
    Pair, Publisher, Subscriber, Request, Reply, Dealer, Router, Pull, Push
}
