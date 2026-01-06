using System.Collections.Concurrent;
using System.Diagnostics;

namespace ControlWorkbench.Core.RealTime;

/// <summary>
/// Real-time Hardware Abstraction Layer for deterministic robot control.
/// Provides timing guarantees, priority management, and safe hardware access.
/// 
/// Features:
/// - Priority-based scheduling with deadline monitoring
/// - Lock-free data structures for sensor/actuator communication
/// - Watchdog timers for fault detection
/// - Hardware interface abstraction for multiple platforms
/// </summary>
public class RealTimeHardwareManager : IDisposable
{
    private readonly ConcurrentDictionary<string, IHardwareDevice> _devices = new();
    private readonly ConcurrentDictionary<string, ControlLoop> _controlLoops = new();
    private readonly PriorityScheduler _scheduler;
    private readonly WatchdogTimer _watchdog;
    private readonly TelemetryLogger _telemetry;
    
    private CancellationTokenSource? _cts;
    private Task? _mainLoop;
    
    public event Action<string, HardwareFault>? FaultDetected;
    public event Action<string, object>? DataReceived;
    
    public bool IsRunning => _mainLoop != null && !_mainLoop.IsCompleted;
    
    public RealTimeHardwareManager(RealTimeConfig? config = null)
    {
        config ??= new RealTimeConfig();
        
        _scheduler = new PriorityScheduler(config.MaxPriority);
        _watchdog = new WatchdogTimer(config.WatchdogTimeoutMs);
        _telemetry = new TelemetryLogger(config.TelemetryBufferSize);
        
        _watchdog.Timeout += OnWatchdogTimeout;
    }
    
    #region Device Management
    
    /// <summary>
    /// Register a hardware device.
    /// </summary>
    public void RegisterDevice<T>(string id, T device) where T : IHardwareDevice
    {
        if (!_devices.TryAdd(id, device))
            throw new InvalidOperationException($"Device {id} already registered");
        
        device.OnData += data => DataReceived?.Invoke(id, data);
        device.OnFault += fault => HandleFault(id, fault);
    }
    
    /// <summary>
    /// Get registered device.
    /// </summary>
    public T GetDevice<T>(string id) where T : IHardwareDevice
    {
        if (!_devices.TryGetValue(id, out var device))
            throw new KeyNotFoundException($"Device {id} not found");
        return (T)device;
    }
    
    /// <summary>
    /// Initialize all registered devices.
    /// </summary>
    public async Task InitializeDevicesAsync(CancellationToken ct = default)
    {
        var tasks = _devices.Values.Select(d => d.InitializeAsync(ct));
        await Task.WhenAll(tasks);
    }
    
    #endregion
    
    #region Control Loop Management
    
    /// <summary>
    /// Register a control loop with specified frequency and priority.
    /// </summary>
    public void RegisterControlLoop(
        string name,
        Action<ControlLoopContext> callback,
        double frequencyHz,
        int priority = 0,
        double deadlineMs = 0)
    {
        if (deadlineMs <= 0)
            deadlineMs = 1000.0 / frequencyHz * 0.8; // 80% of period
        
        var loop = new ControlLoop
        {
            Name = name,
            Callback = callback,
            PeriodMs = 1000.0 / frequencyHz,
            Priority = priority,
            DeadlineMs = deadlineMs,
            Statistics = new ControlLoopStatistics()
        };
        
        if (!_controlLoops.TryAdd(name, loop))
            throw new InvalidOperationException($"Control loop {name} already registered");
    }
    
    /// <summary>
    /// Start all control loops.
    /// </summary>
    public void Start()
    {
        if (IsRunning)
            throw new InvalidOperationException("Already running");
        
        _cts = new CancellationTokenSource();
        _watchdog.Start();
        
        // Start individual loop tasks
        foreach (var (name, loop) in _controlLoops)
        {
            loop.Task = Task.Factory.StartNew(
                () => RunControlLoop(loop, _cts.Token),
                _cts.Token,
                TaskCreationOptions.LongRunning,
                _scheduler.GetScheduler(loop.Priority)
            );
        }
        
        _mainLoop = Task.WhenAll(_controlLoops.Values.Select(l => l.Task!));
    }
    
    /// <summary>
    /// Stop all control loops.
    /// </summary>
    public async Task StopAsync()
    {
        if (!IsRunning) return;
        
        _cts?.Cancel();
        _watchdog.Stop();
        
        if (_mainLoop != null)
        {
            try
            {
                await _mainLoop;
            }
            catch (OperationCanceledException) { }
        }
        
        // Safe shutdown of devices
        foreach (var device in _devices.Values)
        {
            await device.SafeShutdownAsync();
        }
    }
    
    /// <summary>
    /// Get control loop statistics.
    /// </summary>
    public ControlLoopStatistics? GetStatistics(string name)
    {
        return _controlLoops.TryGetValue(name, out var loop) ? loop.Statistics : null;
    }
    
    #endregion
    
    #region Data Exchange
    
    /// <summary>
    /// Write command to actuator (lock-free).
    /// </summary>
    public void WriteCommand<T>(string deviceId, T command) where T : struct
    {
        if (_devices.TryGetValue(deviceId, out var device))
        {
            ((IActuatorDevice<T>)device).WriteCommand(command);
            _telemetry.LogCommand(deviceId, command);
        }
    }
    
    /// <summary>
    /// Read latest sensor data (lock-free).
    /// </summary>
    public T? ReadSensor<T>(string deviceId) where T : struct
    {
        if (_devices.TryGetValue(deviceId, out var device))
        {
            return ((ISensorDevice<T>)device).ReadLatest();
        }
        return null;
    }
    
    /// <summary>
    /// Subscribe to sensor data stream.
    /// </summary>
    public IDisposable SubscribeSensor<T>(string deviceId, Action<T> callback) where T : struct
    {
        if (_devices.TryGetValue(deviceId, out var device))
        {
            return ((ISensorDevice<T>)device).Subscribe(callback);
        }
        throw new KeyNotFoundException($"Device {deviceId} not found");
    }
    
    #endregion
    
    #region Internal
    
    private void RunControlLoop(ControlLoop loop, CancellationToken ct)
    {
        var stopwatch = Stopwatch.StartNew();
        var context = new ControlLoopContext
        {
            LoopName = loop.Name,
            Hardware = this
        };
        
        long nextTick = stopwatch.ElapsedMilliseconds;
        
        while (!ct.IsCancellationRequested)
        {
            long startTime = stopwatch.ElapsedMilliseconds;
            context.Timestamp = DateTime.UtcNow;
            context.DeltaTime = loop.PeriodMs / 1000.0;
            
            try
            {
                // Execute callback
                loop.Callback(context);
                
                // Feed watchdog
                _watchdog.Feed(loop.Name);
                
                // Update statistics
                long executionTime = stopwatch.ElapsedMilliseconds - startTime;
                UpdateStatistics(loop, executionTime);
                
                // Check deadline
                if (executionTime > loop.DeadlineMs)
                {
                    loop.Statistics!.DeadlineMisses++;
                    HandleFault(loop.Name, new HardwareFault
                    {
                        Type = FaultType.DeadlineMiss,
                        Message = $"Execution time {executionTime}ms exceeded deadline {loop.DeadlineMs}ms"
                    });
                }
            }
            catch (Exception ex)
            {
                HandleFault(loop.Name, new HardwareFault
                {
                    Type = FaultType.ExecutionError,
                    Message = ex.Message,
                    Exception = ex
                });
            }
            
            // Wait for next period
            nextTick += (long)loop.PeriodMs;
            long sleepTime = nextTick - stopwatch.ElapsedMilliseconds;
            
            if (sleepTime > 0)
            {
                SpinWait.SpinUntil(() => stopwatch.ElapsedMilliseconds >= nextTick || ct.IsCancellationRequested);
            }
            else
            {
                // Overrun - skip to next period
                nextTick = stopwatch.ElapsedMilliseconds;
                loop.Statistics!.Overruns++;
            }
        }
    }
    
    private void UpdateStatistics(ControlLoop loop, long executionTimeMs)
    {
        var stats = loop.Statistics!;
        stats.Iterations++;
        stats.LastExecutionTimeMs = executionTimeMs;
        stats.TotalExecutionTimeMs += executionTimeMs;
        stats.MaxExecutionTimeMs = System.Math.Max(stats.MaxExecutionTimeMs, executionTimeMs);
        stats.MinExecutionTimeMs = System.Math.Min(stats.MinExecutionTimeMs, executionTimeMs);
        
        // Running average
        double alpha = 0.1;
        stats.AvgExecutionTimeMs = (1 - alpha) * stats.AvgExecutionTimeMs + alpha * executionTimeMs;
    }
    
    private void HandleFault(string source, HardwareFault fault)
    {
        _telemetry.LogFault(source, fault);
        FaultDetected?.Invoke(source, fault);
        
        // Critical fault handling
        if (fault.Type == FaultType.Critical)
        {
            EmergencyStop();
        }
    }
    
    private void OnWatchdogTimeout(string source)
    {
        HandleFault(source, new HardwareFault
        {
            Type = FaultType.WatchdogTimeout,
            Message = $"Watchdog timeout for {source}"
        });
    }
    
    private void EmergencyStop()
    {
        // Immediately stop all actuators
        foreach (var device in _devices.Values)
        {
            if (device is IActuatorDevice actuator)
            {
                actuator.EmergencyStop();
            }
        }
    }
    
    public void Dispose()
    {
        StopAsync().Wait(1000);
        _cts?.Dispose();
        _watchdog.Dispose();
        
        foreach (var device in _devices.Values)
        {
            device.Dispose();
        }
    }
    
    #endregion
}

#region Hardware Interfaces

public interface IHardwareDevice : IDisposable
{
    string DeviceId { get; }
    bool IsConnected { get; }
    
    Task InitializeAsync(CancellationToken ct = default);
    Task SafeShutdownAsync();
    
    event Action<object>? OnData;
    event Action<HardwareFault>? OnFault;
}

public interface ISensorDevice<T> : IHardwareDevice where T : struct
{
    T? ReadLatest();
    IDisposable Subscribe(Action<T> callback);
}

public interface IActuatorDevice : IHardwareDevice
{
    void EmergencyStop();
}

public interface IActuatorDevice<T> : IActuatorDevice where T : struct
{
    void WriteCommand(T command);
}

#endregion

#region Concrete Devices

/// <summary>
/// IMU sensor with lock-free data access.
/// </summary>
public class ImuDevice : ISensorDevice<ImuData>
{
    private readonly string _port;
    private readonly int _baudRate;
    private readonly LockFreeRingBuffer<ImuData> _buffer;
    private readonly List<Action<ImuData>> _subscribers = new();
    
    private volatile bool _isConnected;
    private ImuData _latestData;
    
    public string DeviceId { get; }
    public bool IsConnected => _isConnected;
    
    public event Action<object>? OnData;
    public event Action<HardwareFault>? OnFault;
    
    public ImuDevice(string deviceId, string port, int baudRate = 921600)
    {
        DeviceId = deviceId;
        _port = port;
        _baudRate = baudRate;
        _buffer = new LockFreeRingBuffer<ImuData>(256);
    }
    
    public async Task InitializeAsync(CancellationToken ct = default)
    {
        // Initialize serial connection
        await Task.Delay(100, ct);
        _isConnected = true;
    }
    
    public Task SafeShutdownAsync()
    {
        _isConnected = false;
        return Task.CompletedTask;
    }
    
    public ImuData? ReadLatest()
    {
        return _buffer.TryPeek(out var data) ? data : null;
    }
    
    public IDisposable Subscribe(Action<ImuData> callback)
    {
        _subscribers.Add(callback);
        return new Subscription(() => _subscribers.Remove(callback));
    }
    
    internal void InjectData(ImuData data)
    {
        _latestData = data;
        _buffer.TryEnqueue(data);
        
        foreach (var sub in _subscribers)
            sub(data);
        
        OnData?.Invoke(data);
    }
    
    public void Dispose() { }
}

/// <summary>
/// Motor controller with command buffering.
/// </summary>
public class MotorController : IActuatorDevice<MotorCommand>
{
    private readonly LockFreeRingBuffer<MotorCommand> _commandBuffer;
    private volatile bool _isConnected;
    private volatile bool _emergencyStopped;
    
    public string DeviceId { get; }
    public bool IsConnected => _isConnected;
    
    public event Action<object>? OnData;
    public event Action<HardwareFault>? OnFault;
    
    public MotorController(string deviceId, int numMotors)
    {
        DeviceId = deviceId;
        _commandBuffer = new LockFreeRingBuffer<MotorCommand>(16);
    }
    
    public async Task InitializeAsync(CancellationToken ct = default)
    {
        await Task.Delay(100, ct);
        _isConnected = true;
    }
    
    public Task SafeShutdownAsync()
    {
        EmergencyStop();
        return Task.CompletedTask;
    }
    
    public void WriteCommand(MotorCommand command)
    {
        if (_emergencyStopped) return;
        _commandBuffer.TryEnqueue(command);
    }
    
    public void EmergencyStop()
    {
        _emergencyStopped = true;
        // Send zero commands to all motors
        WriteCommand(new MotorCommand { MotorSpeeds = [0, 0, 0, 0] });
    }
    
    public void Dispose() { }
}

#endregion

#region Data Structures

public struct ImuData
{
    public DateTime Timestamp;
    public float AccelX, AccelY, AccelZ;
    public float GyroX, GyroY, GyroZ;
    public float MagX, MagY, MagZ;
    public float Temperature;
}

public struct MotorCommand
{
    public float[] MotorSpeeds;
    public DateTime Timestamp;
}

public class HardwareFault
{
    public FaultType Type { get; set; }
    public string Message { get; set; } = "";
    public Exception? Exception { get; set; }
    public DateTime Timestamp { get; set; } = DateTime.UtcNow;
}

public enum FaultType
{
    None,
    Warning,
    DeadlineMiss,
    WatchdogTimeout,
    CommunicationError,
    ExecutionError,
    Critical
}

#endregion

#region Control Loop Infrastructure

public class ControlLoop
{
    public string Name { get; set; } = "";
    public Action<ControlLoopContext> Callback { get; set; } = _ => { };
    public double PeriodMs { get; set; }
    public int Priority { get; set; }
    public double DeadlineMs { get; set; }
    public ControlLoopStatistics? Statistics { get; set; }
    public Task? Task { get; set; }
}

public class ControlLoopContext
{
    public string LoopName { get; set; } = "";
    public DateTime Timestamp { get; set; }
    public double DeltaTime { get; set; }
    public RealTimeHardwareManager Hardware { get; set; } = null!;
}

public class ControlLoopStatistics
{
    public long Iterations { get; set; }
    public long LastExecutionTimeMs { get; set; }
    public long TotalExecutionTimeMs { get; set; }
    public long MaxExecutionTimeMs { get; set; }
    public long MinExecutionTimeMs { get; set; } = long.MaxValue;
    public double AvgExecutionTimeMs { get; set; }
    public int DeadlineMisses { get; set; }
    public int Overruns { get; set; }
    
    public double GetAverageExecutionTimeMs() => 
        Iterations > 0 ? (double)TotalExecutionTimeMs / Iterations : 0;
}

public class RealTimeConfig
{
    public int MaxPriority { get; set; } = 10;
    public int WatchdogTimeoutMs { get; set; } = 500;
    public int TelemetryBufferSize { get; set; } = 10000;
}

#endregion

#region Scheduling and Timing

/// <summary>
/// Priority-based task scheduler for control loops.
/// </summary>
public class PriorityScheduler
{
    private readonly TaskScheduler[] _schedulers;
    
    public PriorityScheduler(int maxPriority)
    {
        _schedulers = new TaskScheduler[maxPriority + 1];
        for (int i = 0; i <= maxPriority; i++)
        {
            _schedulers[i] = new LimitedConcurrencyScheduler(1);
        }
    }
    
    public TaskScheduler GetScheduler(int priority)
    {
        priority = System.Math.Clamp(priority, 0, _schedulers.Length - 1);
        return _schedulers[priority];
    }
}

/// <summary>
/// Limited concurrency task scheduler.
/// </summary>
public class LimitedConcurrencyScheduler : TaskScheduler
{
    private readonly int _maxConcurrency;
    private readonly LinkedList<Task> _tasks = new();
    private int _currentConcurrency;
    
    public LimitedConcurrencyScheduler(int maxConcurrency)
    {
        _maxConcurrency = maxConcurrency;
    }
    
    protected override IEnumerable<Task>? GetScheduledTasks()
    {
        lock (_tasks) return _tasks.ToList();
    }
    
    protected override void QueueTask(Task task)
    {
        lock (_tasks)
        {
            _tasks.AddLast(task);
            if (_currentConcurrency < _maxConcurrency)
            {
                _currentConcurrency++;
                ThreadPool.QueueUserWorkItem(_ => ProcessTasks());
            }
        }
    }
    
    protected override bool TryExecuteTaskInline(Task task, bool taskWasPreviouslyQueued)
    {
        if (_currentConcurrency >= _maxConcurrency) return false;
        return TryExecuteTask(task);
    }
    
    private void ProcessTasks()
    {
        while (true)
        {
            Task? task;
            lock (_tasks)
            {
                if (_tasks.Count == 0)
                {
                    _currentConcurrency--;
                    return;
                }
                task = _tasks.First!.Value;
                _tasks.RemoveFirst();
            }
            TryExecuteTask(task);
        }
    }
}

/// <summary>
/// Watchdog timer for detecting control loop failures.
/// </summary>
public class WatchdogTimer : IDisposable
{
    private readonly int _timeoutMs;
    private readonly ConcurrentDictionary<string, DateTime> _lastFeed = new();
    private Timer? _checkTimer;
    
    public event Action<string>? Timeout;
    
    public WatchdogTimer(int timeoutMs)
    {
        _timeoutMs = timeoutMs;
    }
    
    public void Start()
    {
        _checkTimer = new Timer(CheckTimeouts, null, 0, _timeoutMs / 2);
    }
    
    public void Stop()
    {
        _checkTimer?.Dispose();
        _checkTimer = null;
    }
    
    public void Feed(string source)
    {
        _lastFeed[source] = DateTime.UtcNow;
    }
    
    private void CheckTimeouts(object? state)
    {
        var now = DateTime.UtcNow;
        foreach (var (source, lastFeed) in _lastFeed)
        {
            if ((now - lastFeed).TotalMilliseconds > _timeoutMs)
            {
                Timeout?.Invoke(source);
            }
        }
    }
    
    public void Dispose() => _checkTimer?.Dispose();
}

#endregion

#region Lock-Free Data Structures

/// <summary>
/// Lock-free ring buffer for high-frequency sensor data.
/// </summary>
public class LockFreeRingBuffer<T>
{
    private readonly T[] _buffer;
    private readonly int _mask;
    private long _head;
    private long _tail;
    
    public LockFreeRingBuffer(int capacity)
    {
        // Round up to power of 2
        int size = 1;
        while (size < capacity) size <<= 1;
        
        _buffer = new T[size];
        _mask = size - 1;
    }
    
    public bool TryEnqueue(T item)
    {
        long tail = Interlocked.Read(ref _tail);
        long head = Interlocked.Read(ref _head);
        
        if (tail - head >= _buffer.Length)
            return false; // Full
        
        long slot = Interlocked.Increment(ref _tail) - 1;
        _buffer[slot & _mask] = item;
        return true;
    }
    
    public bool TryDequeue(out T item)
    {
        long head = Interlocked.Read(ref _head);
        long tail = Interlocked.Read(ref _tail);
        
        if (head >= tail)
        {
            item = default!;
            return false; // Empty
        }
        
        long slot = Interlocked.Increment(ref _head) - 1;
        item = _buffer[slot & _mask];
        return true;
    }
    
    public bool TryPeek(out T item)
    {
        long tail = Interlocked.Read(ref _tail);
        if (tail == 0)
        {
            item = default!;
            return false;
        }
        
        item = _buffer[(tail - 1) & _mask];
        return true;
    }
    
    public int Count => (int)(Interlocked.Read(ref _tail) - Interlocked.Read(ref _head));
}

#endregion

#region Telemetry

/// <summary>
/// High-performance telemetry logger.
/// </summary>
public class TelemetryLogger
{
    private readonly LockFreeRingBuffer<TelemetryEntry> _buffer;
    
    public TelemetryLogger(int bufferSize)
    {
        _buffer = new LockFreeRingBuffer<TelemetryEntry>(bufferSize);
    }
    
    public void LogCommand<T>(string device, T command)
    {
        _buffer.TryEnqueue(new TelemetryEntry
        {
            Timestamp = DateTime.UtcNow,
            Type = TelemetryType.Command,
            Source = device,
            Data = command!
        });
    }
    
    public void LogFault(string source, HardwareFault fault)
    {
        _buffer.TryEnqueue(new TelemetryEntry
        {
            Timestamp = DateTime.UtcNow,
            Type = TelemetryType.Fault,
            Source = source,
            Data = fault
        });
    }
    
    public IEnumerable<TelemetryEntry> Drain()
    {
        while (_buffer.TryDequeue(out var entry))
        {
            yield return entry;
        }
    }
}

public struct TelemetryEntry
{
    public DateTime Timestamp;
    public TelemetryType Type;
    public string Source;
    public object Data;
}

public enum TelemetryType
{
    Sensor,
    Command,
    State,
    Fault
}

#endregion

#region Utilities

public class Subscription : IDisposable
{
    private readonly Action _unsubscribe;
    
    public Subscription(Action unsubscribe)
    {
        _unsubscribe = unsubscribe;
    }
    
    public void Dispose() => _unsubscribe();
}

#endregion
