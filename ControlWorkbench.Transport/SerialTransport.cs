using System.IO.Ports;
using ControlWorkbench.Core.Time;
using ControlWorkbench.Protocol;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Transport;

/// <summary>
/// Serial port transport implementation.
/// </summary>
public sealed class SerialTransport : ITransport
{
    private SerialPort? _port;
    private readonly MessageDecoder _decoder;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private ConnectionState _state = ConnectionState.Disconnected;

    /// <summary>
    /// Gets or sets the COM port name.
    /// </summary>
    public string PortName { get; set; } = "COM1";

    /// <summary>
    /// Gets or sets the baud rate.
    /// </summary>
    public int BaudRate { get; set; } = 115200;

    /// <summary>
    /// Gets the available COM ports.
    /// </summary>
    public static string[] GetAvailablePorts() => SerialPort.GetPortNames();

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

    public SerialTransport()
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

            _port = new SerialPort(PortName, BaudRate)
            {
                ReadTimeout = 100,
                WriteTimeout = 1000
            };
            _port.Open();

            _cts = new CancellationTokenSource();
            _receiveTask = Task.Run(() => ReceiveLoop(_cts.Token), _cts.Token);

            State = ConnectionState.Connected;
        }
        catch (Exception ex)
        {
            State = ConnectionState.Error;
            throw new InvalidOperationException($"Failed to connect to {PortName}: {ex.Message}", ex);
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

        _port?.Close();
        _port?.Dispose();
        _port = null;

        _cts?.Dispose();
        _cts = null;
        _receiveTask = null;

        State = ConnectionState.Disconnected;
    }

    /// <inheritdoc/>
    public async Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        if (_port == null || !_port.IsOpen)
            throw new InvalidOperationException("Not connected.");

        byte[] data = MessageEncoder.Encode(message);
        await _port.BaseStream.WriteAsync(data, cancellationToken).ConfigureAwait(false);

        Statistics.BytesSent += data.Length;
        Statistics.PacketsSent++;
        Statistics.LastSendTime = DateTime.UtcNow;
    }

    private async Task ReceiveLoop(CancellationToken cancellationToken)
    {
        byte[] buffer = new byte[1024];

        while (!cancellationToken.IsCancellationRequested && _port != null && _port.IsOpen)
        {
            try
            {
                int bytesRead = await _port.BaseStream.ReadAsync(buffer, cancellationToken).ConfigureAwait(false);
                if (bytesRead > 0)
                {
                    long arrivalTime = HighResolutionTime.Now.Microseconds;
                    Statistics.BytesReceived += bytesRead;
                    Statistics.LastReceiveTime = DateTime.UtcNow;

                    _decoder.AddData(buffer.AsSpan(0, bytesRead));

                    foreach (var message in _decoder.DecodeAll())
                    {
                        Statistics.PacketsReceived++;
                        MessageReceived?.Invoke(this, new MessageReceivedEventArgs(message, arrivalTime));
                    }
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception)
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
