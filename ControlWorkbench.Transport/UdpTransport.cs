using System.Net;
using System.Net.Sockets;
using ControlWorkbench.Core.Time;
using ControlWorkbench.Protocol;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Transport;

/// <summary>
/// UDP transport implementation.
/// </summary>
public sealed class UdpTransport : ITransport
{
    private UdpClient? _client;
    private readonly MessageDecoder _decoder;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private ConnectionState _state = ConnectionState.Disconnected;
    private IPEndPoint? _remoteEndPoint;

    /// <summary>
    /// Gets or sets the local port to listen on.
    /// </summary>
    public int LocalPort { get; set; } = 14550;

    /// <summary>
    /// Gets or sets the remote host for sending (optional).
    /// </summary>
    public string? RemoteHost { get; set; }

    /// <summary>
    /// Gets or sets the remote port for sending (optional).
    /// </summary>
    public int RemotePort { get; set; } = 14551;

    /// <inheritdoc/>
    public ConnectionState State
    {
        get => _state;
        private set
        {
            if (_state != value)
            {
                var old = _state;
                _state = value;
                ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(old, value));
            }
        }
    }

    /// <inheritdoc/>
    public TransportStatistics Statistics { get; } = new();

    /// <inheritdoc/>
    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;

    /// <inheritdoc/>
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    public UdpTransport()
    {
        _decoder = new MessageDecoder();
    }

    /// <inheritdoc/>
    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        if (State == ConnectionState.Connected)
            throw new InvalidOperationException("Already connected.");

        try
        {
            State = ConnectionState.Connecting;
            Statistics.Reset();
            _decoder.Reset();

            _client = new UdpClient(LocalPort);

            if (!string.IsNullOrEmpty(RemoteHost))
            {
                _remoteEndPoint = new IPEndPoint(IPAddress.Parse(RemoteHost), RemotePort);
            }

            _cts = new CancellationTokenSource();
            _receiveTask = Task.Run(() => ReceiveLoop(_cts.Token), _cts.Token);

            State = ConnectionState.Connected;
        }
        catch (Exception ex)
        {
            State = ConnectionState.Error;
            throw new InvalidOperationException($"Failed to bind to port {LocalPort}: {ex.Message}", ex);
        }

        return Task.CompletedTask;
    }

    /// <inheritdoc/>
    public async Task DisconnectAsync()
    {
        if (State == ConnectionState.Disconnected)
            return;

        _cts?.Cancel();

        if (_receiveTask != null)
        {
            try
            {
                await _receiveTask.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                // Expected
            }
        }

        _client?.Close();
        _client?.Dispose();
        _client = null;

        _cts?.Dispose();
        _cts = null;
        _receiveTask = null;
        _remoteEndPoint = null;

        State = ConnectionState.Disconnected;
    }

    /// <inheritdoc/>
    public async Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_client == null)
            throw new InvalidOperationException("Not connected.");

        if (_remoteEndPoint == null)
            throw new InvalidOperationException("Remote endpoint not configured.");

        byte[] data = MessageEncoder.Encode(message);
        await _client.SendAsync(data, data.Length, _remoteEndPoint).ConfigureAwait(false);

        Statistics.BytesSent += data.Length;
        Statistics.PacketsSent++;
        Statistics.LastSendTime = DateTime.UtcNow;
    }

    private async Task ReceiveLoop(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested && _client != null)
        {
            try
            {
                var result = await _client.ReceiveAsync(cancellationToken).ConfigureAwait(false);
                long arrivalTime = HighResolutionTime.Now.Microseconds;

                // Update remote endpoint for responses if not set
                _remoteEndPoint ??= result.RemoteEndPoint;

                Statistics.BytesReceived += result.Buffer.Length;
                Statistics.LastReceiveTime = DateTime.UtcNow;

                _decoder.AddData(result.Buffer);

                foreach (var message in _decoder.DecodeAll())
                {
                    Statistics.PacketsReceived++;
                    MessageReceived?.Invoke(this, new MessageReceivedEventArgs(message, arrivalTime));
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (SocketException)
            {
                Statistics.Errors++;
                if (!cancellationToken.IsCancellationRequested)
                {
                    await Task.Delay(100, cancellationToken).ConfigureAwait(false);
                }
            }
        }
    }

    public void Dispose()
    {
        DisconnectAsync().GetAwaiter().GetResult();
    }
}
