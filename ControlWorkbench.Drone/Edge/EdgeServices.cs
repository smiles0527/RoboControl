using System.Collections.Concurrent;
using System.Text.Json;
using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Drone.Fleet;
using FleetMission = ControlWorkbench.Drone.Fleet.Mission;

namespace ControlWorkbench.Drone.Edge;

/// <summary>
/// Local Mission Cache - Stores missions for offline execution.
/// </summary>
public class LocalMissionCache : IDisposable
{
    private readonly string _cachePath;
    private readonly ConcurrentDictionary<string, FleetMission> _missions;
    private readonly object _persistLock = new();
    private bool _isDisposed;

    public int Count => _missions.Count;

    public LocalMissionCache(string cachePath)
    {
        _cachePath = cachePath;
        _missions = new ConcurrentDictionary<string, FleetMission>();

        if (!Directory.Exists(cachePath))
            Directory.CreateDirectory(cachePath);

        LoadFromDisk();
    }

    public Task CacheAsync(FleetMission mission)
    {
        _missions[mission.Id] = mission;
        PersistToDisk();
        return Task.CompletedTask;
    }

    public Task<FleetMission?> GetAsync(string missionId)
    {
        return Task.FromResult(_missions.TryGetValue(missionId, out var mission) ? mission : null);
    }

    public Task<IReadOnlyList<FleetMission>> GetAllAsync()
    {
        return Task.FromResult<IReadOnlyList<FleetMission>>(_missions.Values.ToList());
    }

    public Task RemoveAsync(string missionId)
    {
        _missions.TryRemove(missionId, out _);
        PersistToDisk();
        return Task.CompletedTask;
    }

    private void LoadFromDisk()
    {
        var cacheFile = Path.Combine(_cachePath, "missions.json");
        if (File.Exists(cacheFile))
        {
            try
            {
                var json = File.ReadAllText(cacheFile);
                var missions = JsonSerializer.Deserialize<List<FleetMission>>(json);
                if (missions != null)
                {
                    foreach (var m in missions)
                        _missions[m.Id] = m;
                }
            }
            catch { /* Ignore corrupted cache */ }
        }
    }

    private void PersistToDisk()
    {
        lock (_persistLock)
        {
            try
            {
                var cacheFile = Path.Combine(_cachePath, "missions.json");
                var json = JsonSerializer.Serialize(_missions.Values.ToList());
                File.WriteAllText(cacheFile, json);
            }
            catch { /* Log but don't fail */ }
        }
    }

    public void Dispose()
    {
        if (_isDisposed) return;
        _isDisposed = true;
        PersistToDisk();
    }
}

/// <summary>
/// Telemetry Aggregator - Reduces telemetry volume for cloud upload.
/// </summary>
public class TelemetryAggregator
{
    private readonly AggregationConfiguration _config;
    private readonly ConcurrentDictionary<string, TelemetryBuffer> _buffers;
    private readonly ConcurrentQueue<AggregatedTelemetry> _pendingUploads;

    public TelemetryAggregator(AggregationConfiguration config)
    {
        _config = config;
        _buffers = new ConcurrentDictionary<string, TelemetryBuffer>();
        _pendingUploads = new ConcurrentQueue<AggregatedTelemetry>();
    }

    public AggregationResult Aggregate(string vehicleId, TelemetryFrame frame)
    {
        var buffer = _buffers.GetOrAdd(vehicleId, _ => new TelemetryBuffer());

        // Check if we should upload based on change thresholds
        bool shouldUpload = ShouldUpload(buffer, frame);

        buffer.Frames.Add(frame);
        buffer.LastFrame = frame;

        if (shouldUpload || buffer.Frames.Count >= _config.MaxBufferSize)
        {
            var aggregated = CreateAggregatedTelemetry(vehicleId, buffer);
            _pendingUploads.Enqueue(aggregated);
            buffer.Frames.Clear();
            buffer.LastUploadTime = DateTime.UtcNow;

            return new AggregationResult { ShouldUpload = true, Frame = aggregated };
        }

        return new AggregationResult { ShouldUpload = false };
    }

    private bool ShouldUpload(TelemetryBuffer buffer, TelemetryFrame frame)
    {
        if (buffer.LastFrame == null) return true;

        // Time-based
        if ((DateTime.UtcNow - buffer.LastUploadTime).TotalMilliseconds >= _config.MinUploadIntervalMs)
        {
            // Position change
            var posChange = (frame.Position - buffer.LastFrame.Position).L2Norm();
            if (posChange >= _config.PositionChangeThreshold) return true;

            // Velocity change
            var velChange = (frame.Velocity - buffer.LastFrame.Velocity).L2Norm();
            if (velChange >= _config.VelocityChangeThreshold) return true;

            // Attitude change
            var attChange = (frame.Attitude - buffer.LastFrame.Attitude).L2Norm() * 57.3; // rad to deg
            if (attChange >= _config.AttitudeChangeThreshold) return true;
        }

        return false;
    }

    private AggregatedTelemetry CreateAggregatedTelemetry(string vehicleId, TelemetryBuffer buffer)
    {
        var frames = buffer.Frames;
        return new AggregatedTelemetry
        {
            VehicleId = vehicleId,
            StartTime = frames.First().Timestamp,
            EndTime = frames.Last().Timestamp,
            SampleCount = frames.Count,
            RepresentativeFrame = frames.Last(),
            Summary = new TelemetrySummary
            {
                MinAltitude = frames.Min(f => f.Position[2]),
                MaxAltitude = frames.Max(f => f.Position[2]),
                AvgSpeed = frames.Average(f => f.Velocity.L2Norm()),
                MaxSpeed = frames.Max(f => f.Velocity.L2Norm()),
                MinBattery = frames.Min(f => f.BatteryPercent),
                DistanceTraveled = CalculateDistance(frames)
            }
        };
    }

    private double CalculateDistance(List<TelemetryFrame> frames)
    {
        double total = 0;
        for (int i = 1; i < frames.Count; i++)
        {
            total += (frames[i].Position - frames[i - 1].Position).L2Norm();
        }
        return total;
    }

    public IEnumerable<AggregatedTelemetry> GetPendingUploads()
    {
        var pending = new List<AggregatedTelemetry>();
        while (_pendingUploads.TryDequeue(out var item))
        {
            pending.Add(item);
        }
        return pending;
    }

    public void AcknowledgeUploads(IEnumerable<string> ids)
    {
        // Uploads confirmed, nothing to do as we already dequeued
    }

    private class TelemetryBuffer
    {
        public List<TelemetryFrame> Frames { get; } = new();
        public TelemetryFrame? LastFrame { get; set; }
        public DateTime LastUploadTime { get; set; } = DateTime.MinValue;
    }
}

public class AggregationResult
{
    public bool ShouldUpload { get; set; }
    public AggregatedTelemetry? Frame { get; set; }
}

/// <summary>
/// Local Decision Engine - Makes autonomous decisions without cloud.
/// </summary>
public class LocalDecisionEngine
{
    private readonly LocalDecisionConfiguration _config;

    public LocalDecisionEngine(LocalDecisionConfiguration config)
    {
        _config = config;
    }

    public LocalDecision Evaluate(VehicleEdgeState state, TelemetryFrame telemetry)
    {
        // Critical battery - must RTL
        if (telemetry.BatteryPercent <= _config.BatteryCriticalThreshold)
        {
            return new LocalDecision
            {
                Type = LocalDecisionType.ReturnToHome,
                RequiresAction = true,
                Reason = $"Critical battery: {telemetry.BatteryPercent:F1}%"
            };
        }

        // Low battery warning - consider RTL
        if (telemetry.BatteryPercent <= _config.BatteryWarningThreshold)
        {
            // Calculate if we can complete mission
            if (state.ActiveMission != null)
            {
                var remainingWaypoints = state.ActiveMission.Waypoints.Count - state.CurrentWaypointIndex;
                var estimatedTimeRemaining = remainingWaypoints * 60; // rough estimate: 1 min per waypoint

                // If we might not make it back, RTL now
                if (telemetry.BatteryPercent < 20 && remainingWaypoints > 3)
                {
                    return new LocalDecision
                    {
                        Type = LocalDecisionType.ReturnToHome,
                        RequiresAction = true,
                        Reason = $"Low battery with {remainingWaypoints} waypoints remaining"
                    };
                }
            }
        }

        // Distance from home check
        if (state.HomePosition != null)
        {
            var distanceFromHome = (telemetry.Position - state.HomePosition).L2Norm();
            if (distanceFromHome > _config.MaxDistanceFromHome)
            {
                return new LocalDecision
                {
                    Type = LocalDecisionType.ReturnToHome,
                    RequiresAction = true,
                    Reason = $"Max distance exceeded: {distanceFromHome:F0}m"
                };
            }
        }

        // GPS degraded - reduce speed
        if (telemetry.GpsFixType < 3 || telemetry.SatelliteCount < 6)
        {
            return new LocalDecision
            {
                Type = LocalDecisionType.AdjustSpeed,
                RequiresAction = true,
                Reason = "GPS degraded - reducing speed"
            };
        }

        // All good
        return new LocalDecision
        {
            Type = LocalDecisionType.Continue,
            RequiresAction = false
        };
    }
}

/// <summary>
/// Offline Mode Manager - Handles graceful degradation.
/// </summary>
public class OfflineModeManager
{
    private readonly OfflineConfiguration _config;

    public OfflineModeManager(OfflineConfiguration config)
    {
        _config = config;
    }

    public bool CanStartMissionOffline(FleetMission mission)
    {
        if (!_config.AllowOfflineMissionStart) return false;

        // Check mission duration
        if (mission.EstimatedDuration?.TotalMinutes > _config.MaxOfflineFlightMinutes)
            return false;

        return true;
    }
}

/// <summary>
/// Data Sync Manager - Handles cloud synchronization.
/// </summary>
public class DataSyncManager
{
    private readonly SyncConfiguration _config;
    private DateTime _lastConnectivityCheck;
    private bool _lastConnectivityResult = true;

    public DataSyncManager(SyncConfiguration config)
    {
        _config = config;
    }

    public bool IsCloudReachable()
    {
        // Cache connectivity check for 5 seconds
        if ((DateTime.UtcNow - _lastConnectivityCheck).TotalSeconds < 5)
            return _lastConnectivityResult;

        _lastConnectivityCheck = DateTime.UtcNow;

        try
        {
            // In production, this would ping the cloud endpoint
            // For now, simulate connectivity
            _lastConnectivityResult = true;
            return true;
        }
        catch
        {
            _lastConnectivityResult = false;
            return false;
        }
    }

    public Task<bool> UploadTelemetryAsync(List<AggregatedTelemetry> telemetry)
    {
        // In production, this would POST to cloud API
        return Task.FromResult(true);
    }

    public Task<bool> UploadActionsAsync(List<PendingCloudAction> actions)
    {
        // In production, this would POST to cloud API
        return Task.FromResult(true);
    }

    public Task<List<FleetMission>> FetchNewMissionsAsync()
    {
        // In production, this would GET from cloud API
        return Task.FromResult(new List<FleetMission>());
    }
}

/// <summary>
/// Edge Analytics - Local anomaly detection and metrics.
/// </summary>
public class EdgeAnalytics
{
    private readonly ConcurrentDictionary<string, VehicleAnalyticsState> _states;
    private int _totalAnomalies;
    private long _telemetryCount;
    private DateTime _startTime;

    public EdgeAnalytics()
    {
        _states = new ConcurrentDictionary<string, VehicleAnalyticsState>();
        _startTime = DateTime.UtcNow;
    }

    public List<EdgeAnomaly> DetectAnomalies(string vehicleId, TelemetryFrame frame)
    {
        var anomalies = new List<EdgeAnomaly>();
        var state = _states.GetOrAdd(vehicleId, _ => new VehicleAnalyticsState());

        // Battery drain rate anomaly
        if (state.LastBatteryPercent > 0)
        {
            var drainRate = (state.LastBatteryPercent - frame.BatteryPercent) /
                           (DateTime.UtcNow - state.LastUpdate).TotalMinutes;

            if (drainRate > 5) // More than 5% per minute is unusual
            {
                anomalies.Add(new EdgeAnomaly
                {
                    Type = AnomalyType.BatteryDrain,
                    Severity = drainRate > 10 ? AlertSeverity.Critical : AlertSeverity.Warning,
                    Message = $"Abnormal battery drain: {drainRate:F1}%/min",
                    Confidence = 0.85,
                    Metrics = new Dictionary<string, double> { ["drainRate"] = drainRate }
                });
            }
        }

        // Vibration/motor imbalance (from accelerometer variance)
        if (state.AccelHistory.Count >= 10)
        {
            var variance = CalculateVariance(state.AccelHistory);
            if (variance > 2.0) // High variance indicates vibration
            {
                anomalies.Add(new EdgeAnomaly
                {
                    Type = AnomalyType.VibrationExcessive,
                    Severity = variance > 5.0 ? AlertSeverity.Critical : AlertSeverity.Warning,
                    Message = $"Excessive vibration detected: {variance:F2}",
                    Confidence = 0.75,
                    Metrics = new Dictionary<string, double> { ["variance"] = variance }
                });
            }
        }

        // GPS drift detection
        if (frame.GpsHdop > 3.0)
        {
            anomalies.Add(new EdgeAnomaly
            {
                Type = AnomalyType.GpsDrift,
                Severity = frame.GpsHdop > 5.0 ? AlertSeverity.Warning : AlertSeverity.Info,
                Message = $"GPS accuracy degraded: HDOP {frame.GpsHdop:F1}",
                Confidence = 0.90
            });
        }

        // Temperature warning
        if (frame.BatteryTemperature > 50)
        {
            anomalies.Add(new EdgeAnomaly
            {
                Type = AnomalyType.TemperatureHigh,
                Severity = frame.BatteryTemperature > 60 ? AlertSeverity.Critical : AlertSeverity.Warning,
                Message = $"High battery temperature: {frame.BatteryTemperature:F1}°C",
                Confidence = 0.95
            });
        }

        // Update state
        state.LastBatteryPercent = frame.BatteryPercent;
        state.LastUpdate = DateTime.UtcNow;
        state.AccelHistory.Add(frame.Velocity.L2Norm());
        if (state.AccelHistory.Count > 100)
            state.AccelHistory.RemoveAt(0);

        _totalAnomalies += anomalies.Count;
        return anomalies;
    }

    public void RecordTelemetry(string vehicleId, TelemetryFrame frame)
    {
        Interlocked.Increment(ref _telemetryCount);
    }

    public double GetTelemetryRate()
    {
        var elapsed = (DateTime.UtcNow - _startTime).TotalSeconds;
        return elapsed > 0 ? _telemetryCount / elapsed : 0;
    }

    public int GetAnomalyCount() => _totalAnomalies;

    private double CalculateVariance(List<double> values)
    {
        if (values.Count == 0) return 0;
        var mean = values.Average();
        return values.Select(v => (v - mean) * (v - mean)).Average();
    }

    private class VehicleAnalyticsState
    {
        public double LastBatteryPercent { get; set; }
        public DateTime LastUpdate { get; set; } = DateTime.UtcNow;
        public List<double> AccelHistory { get; } = new();
    }
}
