namespace ControlWorkbench.Core.Logging;

/// <summary>
/// Interface for logging telemetry data.
/// </summary>
public interface ITelemetryLogger : IDisposable
{
    /// <summary>
    /// Logs a message with the given arrival timestamp.
    /// </summary>
    void Log<T>(T message, long arrivalTimestampUs) where T : class;

    /// <summary>
    /// Flushes any buffered data to disk.
    /// </summary>
    void Flush();

    /// <summary>
    /// Gets the base path for log files.
    /// </summary>
    string BasePath { get; }

    /// <summary>
    /// Gets whether the logger is currently active.
    /// </summary>
    bool IsLogging { get; }

    /// <summary>
    /// Starts a new logging session.
    /// </summary>
    void StartSession(string sessionName);

    /// <summary>
    /// Stops the current logging session.
    /// </summary>
    void StopSession();
}

/// <summary>
/// Represents a logged message with metadata.
/// </summary>
public record LoggedMessage(
    long ArrivalTimestampUs,
    string MessageType,
    byte[] RawData
);
