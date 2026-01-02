using System.Threading.Channels;

namespace ControlWorkbench.Core.Collections;

/// <summary>
/// A thread-safe message queue for passing decoded telemetry between transport and UI.
/// Uses System.Threading.Channels for efficient async producer-consumer pattern.
/// </summary>
/// <typeparam name="T">The type of messages in the queue.</typeparam>
public sealed class MessageQueue<T>
{
    private readonly Channel<T> _channel;

    /// <summary>
    /// Gets the reader for consuming messages.
    /// </summary>
    public ChannelReader<T> Reader => _channel.Reader;

    /// <summary>
    /// Gets the writer for producing messages.
    /// </summary>
    public ChannelWriter<T> Writer => _channel.Writer;

    /// <summary>
    /// Creates a new message queue with bounded capacity.
    /// </summary>
    /// <param name="capacity">Maximum number of messages to buffer.</param>
    /// <param name="fullMode">Behavior when buffer is full.</param>
    public MessageQueue(int capacity = 1000, BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest)
    {
        var options = new BoundedChannelOptions(capacity)
        {
            FullMode = fullMode,
            SingleReader = false,
            SingleWriter = false
        };
        _channel = Channel.CreateBounded<T>(options);
    }

    /// <summary>
    /// Creates an unbounded message queue.
    /// </summary>
    public static MessageQueue<T> CreateUnbounded()
    {
        var queue = new MessageQueue<T>(1);
        return queue;
    }

    /// <summary>
    /// Tries to write a message to the queue.
    /// </summary>
    public bool TryWrite(T item) => _channel.Writer.TryWrite(item);

    /// <summary>
    /// Writes a message to the queue asynchronously.
    /// </summary>
    public ValueTask WriteAsync(T item, CancellationToken cancellationToken = default)
        => _channel.Writer.WriteAsync(item, cancellationToken);

    /// <summary>
    /// Tries to read a message from the queue.
    /// </summary>
    public bool TryRead(out T? item) => _channel.Reader.TryRead(out item);

    /// <summary>
    /// Reads a message from the queue asynchronously.
    /// </summary>
    public ValueTask<T> ReadAsync(CancellationToken cancellationToken = default)
        => _channel.Reader.ReadAsync(cancellationToken);

    /// <summary>
    /// Reads all available messages from the queue.
    /// </summary>
    public IEnumerable<T> ReadAll()
    {
        while (_channel.Reader.TryRead(out T? item))
        {
            yield return item;
        }
    }

    /// <summary>
    /// Gets the approximate count of items in the queue.
    /// </summary>
    public int Count => _channel.Reader.Count;

    /// <summary>
    /// Marks the queue as complete (no more writes).
    /// </summary>
    public void Complete() => _channel.Writer.Complete();
}
