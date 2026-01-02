using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Transport;

/// <summary>
/// Connection state enumeration.
/// </summary>
public enum ConnectionState
{
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Error
}

/// <summary>
/// Transport statistics.
/// </summary>
public class TransportStatistics
{
    public long BytesReceived { get; set; }
    public long BytesSent { get; set; }
    public long PacketsReceived { get; set; }
    public long PacketsSent { get; set; }
    public long Errors { get; set; }
    public DateTime? LastReceiveTime { get; set; }
    public DateTime? LastSendTime { get; set; }

    public double BytesPerSecondReceived { get; set; }
    public double PacketsPerSecondReceived { get; set; }

    public void Reset()
    {
        BytesReceived = 0;
        BytesSent = 0;
        PacketsReceived = 0;
        PacketsSent = 0;
        Errors = 0;
        LastReceiveTime = null;
        LastSendTime = null;
        BytesPerSecondReceived = 0;
        PacketsPerSecondReceived = 0;
    }
}

/// <summary>
/// Event arguments for received messages.
/// </summary>
public class MessageReceivedEventArgs : EventArgs
{
    public IMessage Message { get; }
    public long ArrivalTimestampUs { get; }

    public MessageReceivedEventArgs(IMessage message, long arrivalTimestampUs)
    {
        Message = message;
        ArrivalTimestampUs = arrivalTimestampUs;
    }
}

/// <summary>
/// Event arguments for connection state changes.
/// </summary>
public class ConnectionStateChangedEventArgs : EventArgs
{
    public ConnectionState OldState { get; }
    public ConnectionState NewState { get; }
    public string? Message { get; }

    public ConnectionStateChangedEventArgs(ConnectionState oldState, ConnectionState newState, string? message = null)
    {
        OldState = oldState;
        NewState = newState;
        Message = message;
    }
}

/// <summary>
/// Interface for transport implementations.
/// </summary>
public interface ITransport : IDisposable
{
    /// <summary>
    /// Gets the current connection state.
    /// </summary>
    ConnectionState State { get; }

    /// <summary>
    /// Gets the transport statistics.
    /// </summary>
    TransportStatistics Statistics { get; }

    /// <summary>
    /// Connects to the device.
    /// </summary>
    Task ConnectAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Disconnects from the device.
    /// </summary>
    Task DisconnectAsync();

    /// <summary>
    /// Sends a message to the device.
    /// </summary>
    Task SendAsync(IMessage message, CancellationToken cancellationToken = default);

    /// <summary>
    /// Event raised when a message is received.
    /// </summary>
    event EventHandler<MessageReceivedEventArgs>? MessageReceived;

    /// <summary>
    /// Event raised when the connection state changes.
    /// </summary>
    event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;
}
