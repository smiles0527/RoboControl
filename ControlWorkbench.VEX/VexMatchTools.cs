namespace ControlWorkbench.VEX;

/// <summary>
/// Real-time data logger with circular buffer for match analysis.
/// </summary>
public class MatchLogger
{
    private readonly CircularBuffer<LogEntry> _buffer;
    private readonly string _logDirectory;
    private StreamWriter? _fileWriter;
    private bool _isLogging;
    private DateTime _matchStart;
    private string _matchName = "";

    public bool IsLogging => _isLogging;
    public int EntryCount => _buffer.Count;
    public string CurrentMatchName => _matchName;

    public MatchLogger(int bufferSize = 100000, string? logDirectory = null)
    {
        _buffer = new CircularBuffer<LogEntry>(bufferSize);
        _logDirectory = logDirectory ?? Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "ControlWorkbench", "Logs");
        
        Directory.CreateDirectory(_logDirectory);
    }

    /// <summary>
    /// Start logging a match.
    /// </summary>
    public void StartMatch(string matchName = "")
    {
        _matchName = string.IsNullOrEmpty(matchName) 
            ? $"Match_{DateTime.Now:yyyyMMdd_HHmmss}" 
            : matchName;
        
        _matchStart = DateTime.UtcNow;
        _buffer.Clear();

        var filePath = Path.Combine(_logDirectory, $"{_matchName}.csv");
        _fileWriter = new StreamWriter(filePath, false);
        _fileWriter.WriteLine("time_ms,category,name,value1,value2,value3,value4");
        
        _isLogging = true;
        Log("System", "MatchStart", 0);
    }

    /// <summary>
    /// Stop logging and close the file.
    /// </summary>
    public void StopMatch()
    {
        if (!_isLogging) return;

        Log("System", "MatchEnd", (DateTime.UtcNow - _matchStart).TotalMilliseconds);
        
        _fileWriter?.Flush();
        _fileWriter?.Close();
        _fileWriter = null;
        _isLogging = false;
    }

    /// <summary>
    /// Log a single value.
    /// </summary>
    public void Log(string category, string name, double value)
    {
        var entry = new LogEntry
        {
            TimeMs = (long)(DateTime.UtcNow - _matchStart).TotalMilliseconds,
            Category = category,
            Name = name,
            Value1 = value
        };

        _buffer.Add(entry);
        _fileWriter?.WriteLine($"{entry.TimeMs},{category},{name},{value:G6},,,");
    }

    /// <summary>
    /// Log odometry data.
    /// </summary>
    public void LogOdometry(double x, double y, double theta, double velocity = 0)
    {
        var entry = new LogEntry
        {
            TimeMs = (long)(DateTime.UtcNow - _matchStart).TotalMilliseconds,
            Category = "Odometry",
            Name = "Pose",
            Value1 = x,
            Value2 = y,
            Value3 = theta,
            Value4 = velocity
        };

        _buffer.Add(entry);
        _fileWriter?.WriteLine($"{entry.TimeMs},Odometry,Pose,{x:F4},{y:F4},{theta:F4},{velocity:F4}");
    }

    /// <summary>
    /// Log motor data.
    /// </summary>
    public void LogMotor(int port, double position, double velocity, double current, double temperature)
    {
        var entry = new LogEntry
        {
            TimeMs = (long)(DateTime.UtcNow - _matchStart).TotalMilliseconds,
            Category = "Motor",
            Name = $"Motor{port}",
            Value1 = position,
            Value2 = velocity,
            Value3 = current,
            Value4 = temperature
        };

        _buffer.Add(entry);
        _fileWriter?.WriteLine($"{entry.TimeMs},Motor,Motor{port},{position:F2},{velocity:F2},{current:F2},{temperature:F1}");
    }

    /// <summary>
    /// Log PID controller state.
    /// </summary>
    public void LogPid(string controllerName, double setpoint, double measurement, double output, double error)
    {
        var entry = new LogEntry
        {
            TimeMs = (long)(DateTime.UtcNow - _matchStart).TotalMilliseconds,
            Category = "PID",
            Name = controllerName,
            Value1 = setpoint,
            Value2 = measurement,
            Value3 = output,
            Value4 = error
        };

        _buffer.Add(entry);
        _fileWriter?.WriteLine($"{entry.TimeMs},PID,{controllerName},{setpoint:F4},{measurement:F4},{output:F4},{error:F4}");
    }

    /// <summary>
    /// Get recent entries for real-time display.
    /// </summary>
    public IEnumerable<LogEntry> GetRecentEntries(int count = 100)
    {
        return _buffer.TakeLast(count);
    }

    /// <summary>
    /// Get entries by category.
    /// </summary>
    public IEnumerable<LogEntry> GetByCategory(string category)
    {
        return _buffer.Where(e => e.Category == category);
    }

    /// <summary>
    /// Get all log files.
    /// </summary>
    public string[] GetLogFiles()
    {
        return Directory.GetFiles(_logDirectory, "*.csv")
            .OrderByDescending(f => File.GetCreationTime(f))
            .ToArray();
    }

    /// <summary>
    /// Load a previous log file for analysis.
    /// </summary>
    public List<LogEntry> LoadLogFile(string filePath)
    {
        var entries = new List<LogEntry>();
        
        foreach (var line in File.ReadLines(filePath).Skip(1))
        {
            var parts = line.Split(',');
            if (parts.Length >= 4)
            {
                entries.Add(new LogEntry
                {
                    TimeMs = long.TryParse(parts[0], out var t) ? t : 0,
                    Category = parts[1],
                    Name = parts[2],
                    Value1 = double.TryParse(parts[3], out var v1) ? v1 : 0,
                    Value2 = parts.Length > 4 && double.TryParse(parts[4], out var v2) ? v2 : 0,
                    Value3 = parts.Length > 5 && double.TryParse(parts[5], out var v3) ? v3 : 0,
                    Value4 = parts.Length > 6 && double.TryParse(parts[6], out var v4) ? v4 : 0
                });
            }
        }

        return entries;
    }
}

public class LogEntry
{
    public long TimeMs { get; set; }
    public string Category { get; set; } = "";
    public string Name { get; set; } = "";
    public double Value1 { get; set; }
    public double Value2 { get; set; }
    public double Value3 { get; set; }
    public double Value4 { get; set; }
}

/// <summary>
/// Simple circular buffer for efficient logging.
/// </summary>
public class CircularBuffer<T> : IEnumerable<T>
{
    private readonly T[] _buffer;
    private int _head;
    private int _tail;
    private int _count;

    public int Count => _count;
    public int Capacity => _buffer.Length;

    public CircularBuffer(int capacity)
    {
        _buffer = new T[capacity];
    }

    public void Add(T item)
    {
        _buffer[_head] = item;
        _head = (_head + 1) % _buffer.Length;

        if (_count < _buffer.Length)
        {
            _count++;
        }
        else
        {
            _tail = (_tail + 1) % _buffer.Length;
        }
    }

    public void Clear()
    {
        _head = 0;
        _tail = 0;
        _count = 0;
    }

    public IEnumerable<T> TakeLast(int count)
    {
        count = Math.Min(count, _count);
        int start = (_head - count + _buffer.Length) % _buffer.Length;
        
        for (int i = 0; i < count; i++)
        {
            yield return _buffer[(start + i) % _buffer.Length];
        }
    }

    public IEnumerator<T> GetEnumerator()
    {
        for (int i = 0; i < _count; i++)
        {
            yield return _buffer[(_tail + i) % _buffer.Length];
        }
    }

    System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();
}

/// <summary>
/// Match timer with autonomous and driver control phases.
/// </summary>
public class MatchTimer
{
    private DateTime _matchStart;
    private MatchPhase _currentPhase;
    private bool _isRunning;

    // Standard VRC timing
    public static readonly TimeSpan AutonomousDuration = TimeSpan.FromSeconds(15);
    public static readonly TimeSpan DriverControlDuration = TimeSpan.FromSeconds(105); // 1:45
    public static readonly TimeSpan SkillsAutonomousDuration = TimeSpan.FromSeconds(60);
    public static readonly TimeSpan SkillsDriverDuration = TimeSpan.FromSeconds(60);

    public MatchPhase CurrentPhase => _currentPhase;
    public bool IsRunning => _isRunning;
    public TimeSpan Elapsed => _isRunning ? DateTime.UtcNow - _matchStart : TimeSpan.Zero;

    public event Action<MatchPhase>? PhaseChanged;

    public void StartMatch(bool isSkills = false)
    {
        _matchStart = DateTime.UtcNow;
        _currentPhase = MatchPhase.Autonomous;
        _isRunning = true;
        PhaseChanged?.Invoke(_currentPhase);
    }

    public void Update()
    {
        if (!_isRunning) return;

        var elapsed = DateTime.UtcNow - _matchStart;
        var newPhase = _currentPhase;

        if (elapsed < AutonomousDuration)
        {
            newPhase = MatchPhase.Autonomous;
        }
        else if (elapsed < AutonomousDuration + DriverControlDuration)
        {
            newPhase = MatchPhase.DriverControl;
        }
        else
        {
            newPhase = MatchPhase.Ended;
            _isRunning = false;
        }

        if (newPhase != _currentPhase)
        {
            _currentPhase = newPhase;
            PhaseChanged?.Invoke(_currentPhase);
        }
    }

    public TimeSpan GetPhaseTimeRemaining()
    {
        var elapsed = Elapsed;

        return _currentPhase switch
        {
            MatchPhase.Autonomous => AutonomousDuration - elapsed,
            MatchPhase.DriverControl => (AutonomousDuration + DriverControlDuration) - elapsed,
            _ => TimeSpan.Zero
        };
    }

    public void Stop()
    {
        _isRunning = false;
        _currentPhase = MatchPhase.Ended;
    }
}

public enum MatchPhase
{
    NotStarted,
    Autonomous,
    DriverControl,
    Ended
}

/// <summary>
/// Battery health tracker across multiple matches.
/// </summary>
public class BatteryTracker
{
    private readonly List<BatteryReading> _readings = new();
    private readonly string _dataFile;

    public double? CurrentVoltage => _readings.LastOrDefault()?.Voltage;
    public double? CurrentCapacity => _readings.LastOrDefault()?.Capacity;
    public double? AverageVoltage => _readings.Count > 0 ? _readings.Average(r => r.Voltage) : null;

    public BatteryTracker(string? dataFile = null)
    {
        _dataFile = dataFile ?? Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "ControlWorkbench", "battery_history.csv");
        
        Directory.CreateDirectory(Path.GetDirectoryName(_dataFile)!);
        LoadHistory();
    }

    public void RecordReading(double voltage, double capacity, double temperature)
    {
        var reading = new BatteryReading
        {
            Timestamp = DateTime.UtcNow,
            Voltage = voltage,
            Capacity = capacity,
            Temperature = temperature
        };

        _readings.Add(reading);
        SaveReading(reading);

        // Check for warnings
        if (voltage < 7.0)
        {
            OnLowBatteryWarning?.Invoke(voltage);
        }
    }

    public event Action<double>? OnLowBatteryWarning;

    /// <summary>
    /// Get estimated matches remaining based on history.
    /// </summary>
    public int EstimateMatchesRemaining()
    {
        if (_readings.Count < 2) return -1;

        // Estimate based on voltage drop rate
        var recent = _readings.TakeLast(10).ToList();
        if (recent.Count < 2) return -1;

        double voltageDropPerMatch = (recent.First().Voltage - recent.Last().Voltage) / recent.Count;
        if (voltageDropPerMatch <= 0) return 99; // Battery not draining

        double voltageRemaining = CurrentVoltage!.Value - 7.0; // 7V minimum
        return (int)(voltageRemaining / voltageDropPerMatch);
    }

    private void LoadHistory()
    {
        if (!File.Exists(_dataFile)) return;

        foreach (var line in File.ReadLines(_dataFile).Skip(1))
        {
            var parts = line.Split(',');
            if (parts.Length >= 4 && DateTime.TryParse(parts[0], out var ts))
            {
                _readings.Add(new BatteryReading
                {
                    Timestamp = ts,
                    Voltage = double.Parse(parts[1]),
                    Capacity = double.Parse(parts[2]),
                    Temperature = double.Parse(parts[3])
                });
            }
        }
    }

    private void SaveReading(BatteryReading reading)
    {
        bool writeHeader = !File.Exists(_dataFile);
        
        using var writer = new StreamWriter(_dataFile, true);
        if (writeHeader)
        {
            writer.WriteLine("timestamp,voltage,capacity,temperature");
        }
        writer.WriteLine($"{reading.Timestamp:O},{reading.Voltage:F2},{reading.Capacity:F1},{reading.Temperature:F1}");
    }
}

public class BatteryReading
{
    public DateTime Timestamp { get; set; }
    public double Voltage { get; set; }
    public double Capacity { get; set; }
    public double Temperature { get; set; }
}
