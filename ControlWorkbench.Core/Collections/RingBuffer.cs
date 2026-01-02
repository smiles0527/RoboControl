namespace ControlWorkbench.Core.Collections;

/// <summary>
/// A thread-safe ring buffer for storing time-series data.
/// Optimized for live plotting where only the most recent N samples are needed.
/// </summary>
/// <typeparam name="T">The type of elements in the buffer.</typeparam>
public sealed class RingBuffer<T>
{
    private readonly T[] _buffer;
    private readonly object _lock = new();
    private int _head;
    private int _count;

    /// <summary>
    /// Gets the capacity of the buffer.
    /// </summary>
    public int Capacity => _buffer.Length;

    /// <summary>
    /// Gets the current number of elements in the buffer.
    /// </summary>
    public int Count
    {
        get { lock (_lock) return _count; }
    }

    /// <summary>
    /// Gets whether the buffer is full.
    /// </summary>
    public bool IsFull
    {
        get { lock (_lock) return _count == _buffer.Length; }
    }

    /// <summary>
    /// Creates a new ring buffer with the specified capacity.
    /// </summary>
    public RingBuffer(int capacity)
    {
        if (capacity <= 0)
            throw new ArgumentOutOfRangeException(nameof(capacity), "Capacity must be positive.");
        _buffer = new T[capacity];
        _head = 0;
        _count = 0;
    }

    /// <summary>
    /// Adds an element to the buffer, overwriting the oldest if full.
    /// </summary>
    public void Add(T item)
    {
        lock (_lock)
        {
            int index = (_head + _count) % _buffer.Length;
            if (_count == _buffer.Length)
            {
                // Buffer is full, overwrite oldest
                _buffer[_head] = item;
                _head = (_head + 1) % _buffer.Length;
            }
            else
            {
                _buffer[index] = item;
                _count++;
            }
        }
    }

    /// <summary>
    /// Copies all elements to an array in order from oldest to newest.
    /// </summary>
    public T[] ToArray()
    {
        lock (_lock)
        {
            T[] result = new T[_count];
            for (int i = 0; i < _count; i++)
            {
                result[i] = _buffer[(_head + i) % _buffer.Length];
            }
            return result;
        }
    }

    /// <summary>
    /// Copies elements to the provided arrays (useful for plotting X/Y data).
    /// </summary>
    public void CopyTo(T[] destination, int startIndex = 0)
    {
        lock (_lock)
        {
            for (int i = 0; i < _count && startIndex + i < destination.Length; i++)
            {
                destination[startIndex + i] = _buffer[(_head + i) % _buffer.Length];
            }
        }
    }

    /// <summary>
    /// Gets the element at the specified index (0 = oldest).
    /// </summary>
    public T this[int index]
    {
        get
        {
            lock (_lock)
            {
                if (index < 0 || index >= _count)
                    throw new ArgumentOutOfRangeException(nameof(index));
                return _buffer[(_head + index) % _buffer.Length];
            }
        }
    }

    /// <summary>
    /// Clears all elements from the buffer.
    /// </summary>
    public void Clear()
    {
        lock (_lock)
        {
            _head = 0;
            _count = 0;
            Array.Clear(_buffer);
        }
    }

    /// <summary>
    /// Gets the most recent element.
    /// </summary>
    public T? GetLatest()
    {
        lock (_lock)
        {
            if (_count == 0) return default;
            int index = (_head + _count - 1) % _buffer.Length;
            return _buffer[index];
        }
    }
}
