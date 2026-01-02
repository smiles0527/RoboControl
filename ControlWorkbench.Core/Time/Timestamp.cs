namespace ControlWorkbench.Core.Time;

/// <summary>
/// Represents a monotonic timestamp in microseconds.
/// Used for deterministic timing of telemetry messages.
/// </summary>
public readonly struct Timestamp : IEquatable<Timestamp>, IComparable<Timestamp>
{
    /// <summary>
    /// The timestamp value in microseconds.
    /// </summary>
    public long Microseconds { get; }

    /// <summary>
    /// The timestamp value in milliseconds.
    /// </summary>
    public double Milliseconds => Microseconds / 1000.0;

    /// <summary>
    /// The timestamp value in seconds.
    /// </summary>
    public double Seconds => Microseconds / 1_000_000.0;

    public Timestamp(long microseconds)
    {
        Microseconds = microseconds;
    }

    public static Timestamp FromMicroseconds(long us) => new(us);
    public static Timestamp FromMilliseconds(double ms) => new((long)(ms * 1000.0));
    public static Timestamp FromSeconds(double s) => new((long)(s * 1_000_000.0));

    public static Timestamp Zero => new(0);

    public static Timestamp operator +(Timestamp a, Timestamp b) => new(a.Microseconds + b.Microseconds);
    public static Timestamp operator -(Timestamp a, Timestamp b) => new(a.Microseconds - b.Microseconds);

    public static bool operator ==(Timestamp left, Timestamp right) => left.Equals(right);
    public static bool operator !=(Timestamp left, Timestamp right) => !left.Equals(right);
    public static bool operator <(Timestamp left, Timestamp right) => left.Microseconds < right.Microseconds;
    public static bool operator >(Timestamp left, Timestamp right) => left.Microseconds > right.Microseconds;
    public static bool operator <=(Timestamp left, Timestamp right) => left.Microseconds <= right.Microseconds;
    public static bool operator >=(Timestamp left, Timestamp right) => left.Microseconds >= right.Microseconds;

    public bool Equals(Timestamp other) => Microseconds == other.Microseconds;
    public override bool Equals(object? obj) => obj is Timestamp other && Equals(other);
    public override int GetHashCode() => Microseconds.GetHashCode();
    public int CompareTo(Timestamp other) => Microseconds.CompareTo(other.Microseconds);

    public override string ToString() => $"{Seconds:F6}s";
}

/// <summary>
/// Provides high-resolution time utilities.
/// </summary>
public static class HighResolutionTime
{
    private static readonly long StartTicks = System.Diagnostics.Stopwatch.GetTimestamp();
    private static readonly double TickFrequency = System.Diagnostics.Stopwatch.Frequency;

    /// <summary>
    /// Gets the current monotonic timestamp.
    /// </summary>
    public static Timestamp Now
    {
        get
        {
            long elapsedTicks = System.Diagnostics.Stopwatch.GetTimestamp() - StartTicks;
            long microseconds = (long)(elapsedTicks * 1_000_000.0 / TickFrequency);
            return Timestamp.FromMicroseconds(microseconds);
        }
    }

    /// <summary>
    /// Gets the elapsed time since application start in seconds.
    /// </summary>
    public static double ElapsedSeconds => Now.Seconds;
}
