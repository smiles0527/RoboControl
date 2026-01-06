using System.IO.Compression;
using System.Text.Json;
using ControlWorkbench.Drone.Devices;

namespace ControlWorkbench.Drone.Telemetry;

/// <summary>
/// Records and plays back drone telemetry data.
/// Supports multiple formats and real-time streaming.
/// </summary>
public class TelemetryRecorder : IDisposable
{
    private readonly List<TelemetryFrame> _frames = new();
    private StreamWriter? _streamWriter;
    private string? _outputPath;
    private bool _isRecording;
    private DateTimeOffset _recordingStart;
    private long _frameCount;
    
    public bool IsRecording => _isRecording;
    public long FrameCount => _frameCount;
    public TimeSpan Duration => _frames.Count > 0 
        ? _frames[^1].Timestamp - _frames[0].Timestamp 
        : TimeSpan.Zero;

    /// <summary>
    /// Starts recording telemetry to memory.
    /// </summary>
    public void StartRecording()
    {
        _frames.Clear();
        _recordingStart = DateTimeOffset.UtcNow;
        _frameCount = 0;
        _isRecording = true;
    }

    /// <summary>
    /// Starts recording telemetry to a file.
    /// </summary>
    public void StartRecording(string filePath)
    {
        _outputPath = filePath;
        _streamWriter = new StreamWriter(filePath);
        _recordingStart = DateTimeOffset.UtcNow;
        _frameCount = 0;
        _isRecording = true;
        
        // Write header
        _streamWriter.WriteLine("timestamp,lat,lon,alt,roll,pitch,yaw,rollRate,pitchRate,yawRate," +
                                "velN,velE,velD,groundSpeed,climbRate,heading,battV,battI,battRem," +
                                "gpsHdop,gpsSats,mode,armed,motor0,motor1,motor2,motor3");
    }

    /// <summary>
    /// Records a telemetry frame.
    /// </summary>
    public void RecordFrame(DroneTelemetry telemetry)
    {
        if (!_isRecording) return;
        
        var frame = new TelemetryFrame
        {
            Timestamp = telemetry.Timestamp,
            RelativeTime = telemetry.Timestamp - _recordingStart,
            Telemetry = telemetry
        };
        
        _frames.Add(frame);
        _frameCount++;
        
        // Write to file if streaming
        if (_streamWriter != null)
        {
            var t = telemetry;
            _streamWriter.WriteLine($"{t.Timestamp:O},{t.Latitude:F7},{t.Longitude:F7},{t.AltitudeRelative:F2}," +
                                    $"{t.Roll:F2},{t.Pitch:F2},{t.Yaw:F2}," +
                                    $"{t.RollRate:F2},{t.PitchRate:F2},{t.YawRate:F2}," +
                                    $"{t.VelocityNorth:F2},{t.VelocityEast:F2},{t.VelocityDown:F2}," +
                                    $"{t.GroundSpeed:F2},{t.ClimbRate:F2},{t.Heading:F1}," +
                                    $"{t.BatteryVoltage:F2},{t.BatteryCurrent:F1},{t.BatteryRemaining:F1}," +
                                    $"{t.GpsHdop:F1},{t.GpsSatellites},{(int)t.Mode},{(t.Armed ? 1 : 0)}," +
                                    $"{(t.MotorOutputs?.Length > 0 ? t.MotorOutputs[0] : 0)}," +
                                    $"{(t.MotorOutputs?.Length > 1 ? t.MotorOutputs[1] : 0)}," +
                                    $"{(t.MotorOutputs?.Length > 2 ? t.MotorOutputs[2] : 0)}," +
                                    $"{(t.MotorOutputs?.Length > 3 ? t.MotorOutputs[3] : 0)}");
        }
    }

    /// <summary>
    /// Stops recording.
    /// </summary>
    public void StopRecording()
    {
        _isRecording = false;
        _streamWriter?.Dispose();
        _streamWriter = null;
    }

    /// <summary>
    /// Gets all recorded frames.
    /// </summary>
    public IReadOnlyList<TelemetryFrame> GetFrames() => _frames;

    /// <summary>
    /// Saves recording to a compressed binary file.
    /// </summary>
    public void SaveToBinary(string filePath)
    {
        using var fileStream = File.Create(filePath);
        using var gzip = new GZipStream(fileStream, CompressionLevel.Optimal);
        using var writer = new BinaryWriter(gzip);
        
        // Header
        writer.Write("CWBTELEM"); // Magic
        writer.Write(1);          // Version
        writer.Write(_frames.Count);
        writer.Write(_recordingStart.ToUnixTimeMilliseconds());
        
        // Frames
        foreach (var frame in _frames)
        {
            var t = frame.Telemetry;
            writer.Write(frame.RelativeTime.TotalMilliseconds);
            writer.Write(t.Latitude);
            writer.Write(t.Longitude);
            writer.Write(t.AltitudeRelative);
            writer.Write(t.Roll);
            writer.Write(t.Pitch);
            writer.Write(t.Yaw);
            writer.Write(t.RollRate);
            writer.Write(t.PitchRate);
            writer.Write(t.YawRate);
            writer.Write(t.VelocityNorth);
            writer.Write(t.VelocityEast);
            writer.Write(t.VelocityDown);
            writer.Write(t.GroundSpeed);
            writer.Write(t.ClimbRate);
            writer.Write(t.Heading);
            writer.Write(t.BatteryVoltage);
            writer.Write(t.BatteryCurrent);
            writer.Write(t.BatteryRemaining);
            writer.Write(t.GpsHdop);
            writer.Write(t.GpsSatellites);
            writer.Write((int)t.Mode);
            writer.Write(t.Armed);
            
            writer.Write(t.MotorOutputs?.Length ?? 0);
            if (t.MotorOutputs != null)
            {
                foreach (var m in t.MotorOutputs)
                    writer.Write(m);
            }
        }
    }

    /// <summary>
    /// Loads recording from a compressed binary file.
    /// </summary>
    public static TelemetryRecorder LoadFromBinary(string filePath)
    {
        var recorder = new TelemetryRecorder();
        
        using var fileStream = File.OpenRead(filePath);
        using var gzip = new GZipStream(fileStream, CompressionMode.Decompress);
        using var reader = new BinaryReader(gzip);
        
        // Header
        var magic = reader.ReadString();
        if (magic != "CWBTELEM")
            throw new InvalidDataException("Invalid telemetry file");
        
        var version = reader.ReadInt32();
        var frameCount = reader.ReadInt32();
        var startMs = reader.ReadInt64();
        recorder._recordingStart = DateTimeOffset.FromUnixTimeMilliseconds(startMs);
        
        // Frames
        for (int i = 0; i < frameCount; i++)
        {
            var relativeMs = reader.ReadDouble();
            var telemetry = new DroneTelemetry
            {
                Timestamp = recorder._recordingStart + TimeSpan.FromMilliseconds(relativeMs),
                Latitude = reader.ReadDouble(),
                Longitude = reader.ReadDouble(),
                AltitudeRelative = reader.ReadDouble(),
                Roll = reader.ReadDouble(),
                Pitch = reader.ReadDouble(),
                Yaw = reader.ReadDouble(),
                RollRate = reader.ReadDouble(),
                PitchRate = reader.ReadDouble(),
                YawRate = reader.ReadDouble(),
                VelocityNorth = reader.ReadDouble(),
                VelocityEast = reader.ReadDouble(),
                VelocityDown = reader.ReadDouble(),
                GroundSpeed = reader.ReadDouble(),
                ClimbRate = reader.ReadDouble(),
                Heading = reader.ReadDouble(),
                BatteryVoltage = reader.ReadDouble(),
                BatteryCurrent = reader.ReadDouble(),
                BatteryRemaining = reader.ReadDouble(),
                GpsHdop = reader.ReadDouble(),
                GpsSatellites = reader.ReadInt32(),
                Mode = (FlightMode)reader.ReadInt32(),
                Armed = reader.ReadBoolean()
            };
            
            var motorCount = reader.ReadInt32();
            telemetry.MotorOutputs = new int[motorCount];
            for (int m = 0; m < motorCount; m++)
                telemetry.MotorOutputs[m] = reader.ReadInt32();
            
            recorder._frames.Add(new TelemetryFrame
            {
                Timestamp = telemetry.Timestamp,
                RelativeTime = TimeSpan.FromMilliseconds(relativeMs),
                Telemetry = telemetry
            });
        }
        
        return recorder;
    }

    /// <summary>
    /// Exports to GeoJSON for map visualization.
    /// </summary>
    public string ExportToGeoJson()
    {
        var coordinates = _frames
            .Where(f => f.Telemetry.Latitude != 0 && f.Telemetry.Longitude != 0)
            .Select(f => new[] { f.Telemetry.Longitude, f.Telemetry.Latitude, f.Telemetry.AltitudeRelative })
            .ToList();
        
        var geoJson = new
        {
            type = "FeatureCollection",
            features = new object[]
            {
                new
                {
                    type = "Feature",
                    properties = new
                    {
                        name = "Flight Path",
                        duration = Duration.TotalSeconds,
                        startTime = _recordingStart,
                        frames = _frames.Count
                    },
                    geometry = new
                    {
                        type = "LineString",
                        coordinates
                    }
                }
            }
        };
        
        return JsonSerializer.Serialize(geoJson, new JsonSerializerOptions { WriteIndented = true });
    }

    /// <summary>
    /// Exports to GPX format.
    /// </summary>
    public string ExportToGpx()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
        sb.AppendLine("<gpx version=\"1.1\" creator=\"ControlWorkbench\">");
        sb.AppendLine("  <trk>");
        sb.AppendLine("    <name>Flight Recording</name>");
        sb.AppendLine("    <trkseg>");
        
        foreach (var frame in _frames)
        {
            var t = frame.Telemetry;
            if (t.Latitude == 0 && t.Longitude == 0) continue;
            
            sb.AppendLine($"      <trkpt lat=\"{t.Latitude:F7}\" lon=\"{t.Longitude:F7}\">");
            sb.AppendLine($"        <ele>{t.AltitudeRelative:F1}</ele>");
            sb.AppendLine($"        <time>{t.Timestamp:O}</time>");
            sb.AppendLine($"        <speed>{t.GroundSpeed:F2}</speed>");
            sb.AppendLine("      </trkpt>");
        }
        
        sb.AppendLine("    </trkseg>");
        sb.AppendLine("  </trk>");
        sb.AppendLine("</gpx>");
        
        return sb.ToString();
    }

    public void Dispose()
    {
        StopRecording();
    }
}

/// <summary>
/// Single frame of telemetry data.
/// </summary>
public class TelemetryFrame
{
    public DateTimeOffset Timestamp { get; init; }
    public TimeSpan RelativeTime { get; init; }
    public required DroneTelemetry Telemetry { get; init; }
}

/// <summary>
/// Plays back recorded telemetry.
/// </summary>
public class TelemetryPlayer
{
    private readonly IReadOnlyList<TelemetryFrame> _frames;
    private int _currentIndex;
    private double _playbackSpeed = 1.0;
    private bool _isPlaying;
    private bool _loop;
    private TimeSpan _currentTime;
    
    public event Action<DroneTelemetry>? FramePlayed;
    public event Action? PlaybackEnded;
    
    public bool IsPlaying => _isPlaying;
    public TimeSpan CurrentTime => _currentTime;
    public TimeSpan Duration { get; }
    public double PlaybackSpeed 
    { 
        get => _playbackSpeed; 
        set => _playbackSpeed = System.Math.Clamp(value, 0.1, 10.0); 
    }
    public bool Loop { get => _loop; set => _loop = value; }
    public int CurrentFrameIndex => _currentIndex;
    public int TotalFrames => _frames.Count;

    public TelemetryPlayer(IReadOnlyList<TelemetryFrame> frames)
    {
        _frames = frames;
        Duration = frames.Count > 0 ? frames[^1].RelativeTime : TimeSpan.Zero;
    }

    public void Play()
    {
        _isPlaying = true;
    }

    public void Pause()
    {
        _isPlaying = false;
    }

    public void Stop()
    {
        _isPlaying = false;
        _currentIndex = 0;
        _currentTime = TimeSpan.Zero;
    }

    public void Seek(TimeSpan time)
    {
        _currentTime = time;
        _currentIndex = FindFrameAtTime(time);
    }

    public void SeekToFrame(int index)
    {
        if (index >= 0 && index < _frames.Count)
        {
            _currentIndex = index;
            _currentTime = _frames[index].RelativeTime;
        }
    }

    /// <summary>
    /// Updates playback - call this at regular intervals.
    /// </summary>
    public void Update(double deltaTime)
    {
        if (!_isPlaying || _frames.Count == 0)
            return;
        
        _currentTime += TimeSpan.FromSeconds(deltaTime * _playbackSpeed);
        
        // Find and play frames up to current time
        while (_currentIndex < _frames.Count && 
               _frames[_currentIndex].RelativeTime <= _currentTime)
        {
            FramePlayed?.Invoke(_frames[_currentIndex].Telemetry);
            _currentIndex++;
        }
        
        // Check for end
        if (_currentIndex >= _frames.Count)
        {
            if (_loop)
            {
                _currentIndex = 0;
                _currentTime = TimeSpan.Zero;
            }
            else
            {
                _isPlaying = false;
                PlaybackEnded?.Invoke();
            }
        }
    }

    /// <summary>
    /// Gets telemetry at a specific time with interpolation.
    /// </summary>
    public DroneTelemetry? GetTelemetryAtTime(TimeSpan time)
    {
        int index = FindFrameAtTime(time);
        if (index < 0 || index >= _frames.Count)
            return null;
        
        // Simple: return nearest frame
        // Could be enhanced with interpolation
        return _frames[index].Telemetry;
    }

    private int FindFrameAtTime(TimeSpan time)
    {
        // Binary search for frame
        int left = 0;
        int right = _frames.Count - 1;
        
        while (left < right)
        {
            int mid = (left + right) / 2;
            if (_frames[mid].RelativeTime < time)
                left = mid + 1;
            else
                right = mid;
        }
        
        return left;
    }
}

/// <summary>
/// Analyzes telemetry data for anomalies and statistics.
/// </summary>
public class TelemetryAnalyzer
{
    private readonly IReadOnlyList<TelemetryFrame> _frames;

    public TelemetryAnalyzer(IReadOnlyList<TelemetryFrame> frames)
    {
        _frames = frames;
    }

    /// <summary>
    /// Computes flight statistics.
    /// </summary>
    public FlightStatistics ComputeStatistics()
    {
        if (_frames.Count == 0)
            return new FlightStatistics();
        
        var stats = new FlightStatistics
        {
            Duration = _frames[^1].RelativeTime,
            FrameCount = _frames.Count
        };
        
        double totalDistance = 0;
        double maxAlt = double.MinValue;
        double minAlt = double.MaxValue;
        double maxSpeed = 0;
        double maxClimbRate = 0;
        double maxDescentRate = 0;
        double minBattery = 100;
        
        for (int i = 0; i < _frames.Count; i++)
        {
            var t = _frames[i].Telemetry;
            
            // Distance
            if (i > 0)
            {
                var prev = _frames[i - 1].Telemetry;
                double dist = HaversineDistance(prev.Latitude, prev.Longitude, t.Latitude, t.Longitude);
                if (dist < 1000) // Filter GPS jumps
                    totalDistance += dist;
            }
            
            // Altitude
            maxAlt = System.Math.Max(maxAlt, t.AltitudeRelative);
            minAlt = System.Math.Min(minAlt, t.AltitudeRelative);
            
            // Speed
            maxSpeed = System.Math.Max(maxSpeed, t.GroundSpeed);
            
            // Climb/descent
            if (t.ClimbRate > 0)
                maxClimbRate = System.Math.Max(maxClimbRate, t.ClimbRate);
            else
                maxDescentRate = System.Math.Max(maxDescentRate, -t.ClimbRate);
            
            // Battery
            minBattery = System.Math.Min(minBattery, t.BatteryRemaining);
        }
        
        stats.TotalDistanceMeters = totalDistance;
        stats.MaxAltitude = maxAlt;
        stats.MinAltitude = minAlt > 0 ? minAlt : 0;
        stats.MaxGroundSpeed = maxSpeed;
        stats.MaxClimbRate = maxClimbRate;
        stats.MaxDescentRate = maxDescentRate;
        stats.BatteryUsed = 100 - minBattery;
        
        // Compute averages
        stats.AverageAltitude = _frames.Average(f => f.Telemetry.AltitudeRelative);
        stats.AverageGroundSpeed = _frames.Average(f => f.Telemetry.GroundSpeed);
        
        return stats;
    }

    /// <summary>
    /// Detects anomalies in the flight.
    /// </summary>
    public List<FlightAnomaly> DetectAnomalies()
    {
        var anomalies = new List<FlightAnomaly>();
        
        for (int i = 1; i < _frames.Count; i++)
        {
            var prev = _frames[i - 1].Telemetry;
            var curr = _frames[i].Telemetry;
            var time = _frames[i].RelativeTime;
            
            // GPS jump
            double dist = HaversineDistance(prev.Latitude, prev.Longitude, curr.Latitude, curr.Longitude);
            double dt = (_frames[i].RelativeTime - _frames[i - 1].RelativeTime).TotalSeconds;
            if (dt > 0 && dist / dt > 50) // > 50 m/s jump
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.GpsJump,
                    Time = time,
                    Severity = AnomalySeverity.Warning,
                    Message = $"GPS jump detected: {dist:F0}m in {dt:F2}s"
                });
            }
            
            // Rapid descent
            if (curr.ClimbRate < -5)
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.RapidDescent,
                    Time = time,
                    Severity = curr.ClimbRate < -10 ? AnomalySeverity.Critical : AnomalySeverity.Warning,
                    Message = $"Rapid descent: {curr.ClimbRate:F1} m/s"
                });
            }
            
            // Low battery
            if (curr.BatteryRemaining < 20 && prev.BatteryRemaining >= 20)
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.LowBattery,
                    Time = time,
                    Severity = AnomalySeverity.Warning,
                    Message = $"Battery below 20%: {curr.BatteryRemaining:F0}%"
                });
            }
            
            if (curr.BatteryRemaining < 10 && prev.BatteryRemaining >= 10)
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.LowBattery,
                    Time = time,
                    Severity = AnomalySeverity.Critical,
                    Message = $"Battery critical: {curr.BatteryRemaining:F0}%"
                });
            }
            
            // High vibration (from angular rates)
            double rateSum = System.Math.Abs(curr.RollRate) + System.Math.Abs(curr.PitchRate);
            if (rateSum > 200)
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.HighVibration,
                    Time = time,
                    Severity = rateSum > 300 ? AnomalySeverity.Warning : AnomalySeverity.Info,
                    Message = $"High angular rates: {rateSum:F0} deg/s"
                });
            }
            
            // GPS quality drop
            if (curr.GpsSatellites < 6 && prev.GpsSatellites >= 6)
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.GpsQualityDrop,
                    Time = time,
                    Severity = AnomalySeverity.Warning,
                    Message = $"GPS satellites dropped to {curr.GpsSatellites}"
                });
            }
            
            // Failsafe
            if (curr.Failsafe && !prev.Failsafe)
            {
                anomalies.Add(new FlightAnomaly
                {
                    Type = AnomalyType.Failsafe,
                    Time = time,
                    Severity = AnomalySeverity.Critical,
                    Message = "Failsafe triggered"
                });
            }
        }
        
        return anomalies;
    }

    /// <summary>
    /// Generates a flight summary report.
    /// </summary>
    public string GenerateReport()
    {
        var stats = ComputeStatistics();
        var anomalies = DetectAnomalies();
        
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("???????????????????????????????????????????????????????????????");
        sb.AppendLine("                    FLIGHT ANALYSIS REPORT");
        sb.AppendLine("???????????????????????????????????????????????????????????????");
        sb.AppendLine();
        sb.AppendLine("FLIGHT STATISTICS");
        sb.AppendLine("?????????????????????????????????????????????????????????????");
        sb.AppendLine($"  Duration:           {stats.Duration:hh\\:mm\\:ss}");
        sb.AppendLine($"  Total Distance:     {stats.TotalDistanceMeters:F0} m ({stats.TotalDistanceMeters / 1000:F2} km)");
        sb.AppendLine($"  Max Altitude:       {stats.MaxAltitude:F1} m");
        sb.AppendLine($"  Avg Altitude:       {stats.AverageAltitude:F1} m");
        sb.AppendLine($"  Max Ground Speed:   {stats.MaxGroundSpeed:F1} m/s ({stats.MaxGroundSpeed * 3.6:F1} km/h)");
        sb.AppendLine($"  Avg Ground Speed:   {stats.AverageGroundSpeed:F1} m/s");
        sb.AppendLine($"  Max Climb Rate:     {stats.MaxClimbRate:F1} m/s");
        sb.AppendLine($"  Max Descent Rate:   {stats.MaxDescentRate:F1} m/s");
        sb.AppendLine($"  Battery Used:       {stats.BatteryUsed:F0}%");
        sb.AppendLine($"  Data Points:        {stats.FrameCount}");
        sb.AppendLine();
        
        if (anomalies.Count > 0)
        {
            sb.AppendLine("ANOMALIES DETECTED");
            sb.AppendLine("?????????????????????????????????????????????????????????????");
            foreach (var anomaly in anomalies.OrderBy(a => a.Time))
            {
                string severity = anomaly.Severity switch
                {
                    AnomalySeverity.Critical => "[CRITICAL]",
                    AnomalySeverity.Warning => "[WARNING] ",
                    _ => "[INFO]    "
                };
                sb.AppendLine($"  {anomaly.Time:mm\\:ss\\.ff} {severity} {anomaly.Message}");
            }
            sb.AppendLine();
        }
        else
        {
            sb.AppendLine("No anomalies detected - flight appears nominal.");
            sb.AppendLine();
        }
        
        sb.AppendLine("???????????????????????????????????????????????????????????????");
        
        return sb.ToString();
    }

    private static double HaversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371000;
        double dLat = (lat2 - lat1) * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1 * System.Math.PI / 180) * System.Math.Cos(lat2 * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        return R * 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
    }
}

public class FlightStatistics
{
    public TimeSpan Duration { get; set; }
    public int FrameCount { get; set; }
    public double TotalDistanceMeters { get; set; }
    public double MaxAltitude { get; set; }
    public double MinAltitude { get; set; }
    public double AverageAltitude { get; set; }
    public double MaxGroundSpeed { get; set; }
    public double AverageGroundSpeed { get; set; }
    public double MaxClimbRate { get; set; }
    public double MaxDescentRate { get; set; }
    public double BatteryUsed { get; set; }
}

public class FlightAnomaly
{
    public AnomalyType Type { get; set; }
    public TimeSpan Time { get; set; }
    public AnomalySeverity Severity { get; set; }
    public required string Message { get; set; }
}

public enum AnomalyType
{
    GpsJump,
    GpsQualityDrop,
    RapidDescent,
    LowBattery,
    HighVibration,
    Failsafe,
    MotorImbalance,
    CompassError,
    EkfError
}

public enum AnomalySeverity
{
    Info,
    Warning,
    Critical
}
