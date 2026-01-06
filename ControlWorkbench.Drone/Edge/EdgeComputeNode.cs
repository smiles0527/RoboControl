using System.Collections.Concurrent;
using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Drone.Fleet;
using FleetMission = ControlWorkbench.Drone.Fleet.Mission;

namespace ControlWorkbench.Drone.Edge;

/// <summary>
/// Edge Computing Node - Local processing for low-latency drone operations.
/// Enables autonomous operation when cloud connectivity is unavailable.
/// Implements fog computing architecture with intelligent data filtering.
/// </summary>
public class EdgeComputeNode : IDisposable
{
    private readonly EdgeConfiguration _config;
    private readonly LocalMissionCache _missionCache;
    private readonly TelemetryAggregator _telemetryAggregator;
    private readonly LocalDecisionEngine _decisionEngine;
    private readonly OfflineModeManager _offlineManager;
    private readonly DataSyncManager _syncManager;
    private readonly EdgeAnalytics _analytics;

    private readonly ConcurrentDictionary<string, VehicleEdgeState> _vehicleStates;
    private readonly ConcurrentQueue<PendingCloudAction> _pendingActions;
    private readonly Timer _syncTimer;
    private readonly Timer _healthTimer;

    private bool _isOfflineMode;
    private DateTime _lastCloudSync;
    private bool _isDisposed;

    public event EventHandler<EdgeModeChangedEventArgs>? ModeChanged;
    public event EventHandler<EdgeAlertEventArgs>? AlertGenerated;

    public bool IsOfflineMode => _isOfflineMode;
    public DateTime LastCloudSync => _lastCloudSync;
    public int PendingActionsCount => _pendingActions.Count;

    public EdgeComputeNode(EdgeConfiguration config)
    {
        _config = config;
        _missionCache = new LocalMissionCache(config.CachePath);
        _telemetryAggregator = new TelemetryAggregator(config.AggregationConfig);
        _decisionEngine = new LocalDecisionEngine(config.DecisionConfig);
        _offlineManager = new OfflineModeManager(config.OfflineConfig);
        _syncManager = new DataSyncManager(config.SyncConfig);
        _analytics = new EdgeAnalytics();

        _vehicleStates = new ConcurrentDictionary<string, VehicleEdgeState>();
        _pendingActions = new ConcurrentQueue<PendingCloudAction>();

        _syncTimer = new Timer(SyncWithCloud, null,
            TimeSpan.FromSeconds(10), TimeSpan.FromSeconds(config.SyncIntervalSeconds));
        _healthTimer = new Timer(CheckConnectivity, null,
            TimeSpan.FromSeconds(5), TimeSpan.FromSeconds(5));

        _lastCloudSync = DateTime.UtcNow;
    }

    #region Telemetry Processing

    /// <summary>
    /// Process telemetry locally with edge filtering and aggregation.
    /// </summary>
    public EdgeTelemetryResult ProcessTelemetry(string vehicleId, TelemetryFrame frame)
    {
        var result = new EdgeTelemetryResult { VehicleId = vehicleId, Timestamp = frame.Timestamp };

        // Update local state
        var state = _vehicleStates.GetOrAdd(vehicleId, _ => new VehicleEdgeState { VehicleId = vehicleId });
        state.LastTelemetry = frame;
        state.LastUpdate = DateTime.UtcNow;
        state.IsConnected = true;

        // Run local anomaly detection
        var anomalies = _analytics.DetectAnomalies(vehicleId, frame);
        if (anomalies.Count > 0)
        {
            result.Anomalies = anomalies;
            foreach (var anomaly in anomalies)
            {
                AlertGenerated?.Invoke(this, new EdgeAlertEventArgs(vehicleId, anomaly));
            }
        }

        // Aggregate telemetry for cloud upload
        var aggregated = _telemetryAggregator.Aggregate(vehicleId, frame);
        result.ShouldUploadToCloud = aggregated.ShouldUpload;
        result.AggregatedFrame = aggregated.Frame;

        // Local decision making if needed
        if (state.ActiveMission != null)
        {
            var decision = _decisionEngine.Evaluate(state, frame);
            result.LocalDecision = decision;

            if (decision.RequiresAction)
            {
                ExecuteLocalAction(vehicleId, decision);
            }
        }

        // Update analytics
        _analytics.RecordTelemetry(vehicleId, frame);

        return result;
    }

    /// <summary>
    /// Get filtered telemetry stream for cloud upload.
    /// Reduces bandwidth by 80-90% through intelligent filtering.
    /// </summary>
    public IEnumerable<AggregatedTelemetry> GetPendingCloudTelemetry()
    {
        return _telemetryAggregator.GetPendingUploads();
    }

    /// <summary>
    /// Mark telemetry as successfully uploaded.
    /// </summary>
    public void AcknowledgeCloudUpload(IEnumerable<string> telemetryIds)
    {
        _telemetryAggregator.AcknowledgeUploads(telemetryIds);
    }

    #endregion

    #region Offline Mission Execution

    /// <summary>
    /// Cache a mission for offline execution.
    /// </summary>
    public async Task CacheMissionAsync(FleetMission mission)
    {
        await _missionCache.CacheAsync(mission);
    }

    /// <summary>
    /// Start a cached mission in offline mode.
    /// </summary>
    public async Task<OfflineMissionResult> StartOfflineMissionAsync(string missionId, string vehicleId)
    {
        var mission = await _missionCache.GetAsync(missionId);
        if (mission == null)
        {
            return new OfflineMissionResult
            {
                Success = false,
                Error = "Mission not found in local cache"
            };
        }

        // Validate vehicle is available
        if (!_vehicleStates.TryGetValue(vehicleId, out var state) || !state.IsConnected)
        {
            return new OfflineMissionResult
            {
                Success = false,
                Error = "Vehicle not connected to edge node"
            };
        }

        // Start mission locally
        state.ActiveMission = mission;
        state.MissionStartTime = DateTime.UtcNow;
        state.CurrentWaypointIndex = 0;

        // Queue for cloud sync when connectivity returns
        _pendingActions.Enqueue(new PendingCloudAction
        {
            Type = CloudActionType.MissionStarted,
            MissionId = missionId,
            VehicleId = vehicleId,
            Timestamp = DateTime.UtcNow,
            Data = new Dictionary<string, object>
            {
                ["startedOffline"] = true,
                ["edgeNodeId"] = _config.NodeId
            }
        });

        return new OfflineMissionResult
        {
            Success = true,
            MissionId = missionId,
            StartedAt = DateTime.UtcNow
        };
    }

    /// <summary>
    /// Get all cached missions available for offline execution.
    /// </summary>
    public async Task<IReadOnlyList<FleetMission>> GetCachedMissionsAsync()
    {
        return await _missionCache.GetAllAsync();
    }

    #endregion

    #region Local Decision Making

    /// <summary>
    /// Execute emergency procedures locally without cloud.
    /// </summary>
    public EmergencyResponse HandleEmergency(string vehicleId, EmergencyType type)
    {
        var response = new EmergencyResponse
        {
            VehicleId = vehicleId,
            EmergencyType = type,
            Timestamp = DateTime.UtcNow,
            HandledLocally = true
        };

        switch (type)
        {
            case EmergencyType.LowBattery:
                response.Action = EmergencyAction.ReturnToHome;
                response.Command = GenerateRTLCommand(vehicleId);
                break;

            case EmergencyType.LostLink:
                response.Action = EmergencyAction.HoldPosition;
                response.Command = GenerateHoldCommand(vehicleId);
                response.TimeoutSeconds = 60;
                response.FallbackAction = EmergencyAction.ReturnToHome;
                break;

            case EmergencyType.GeofenceViolation:
                response.Action = EmergencyAction.ReturnToSafeZone;
                response.Command = GenerateGeofenceRecoveryCommand(vehicleId);
                break;

            case EmergencyType.MotorFailure:
                response.Action = EmergencyAction.EmergencyLand;
                response.Command = GenerateEmergencyLandCommand(vehicleId);
                break;

            case EmergencyType.GpsLost:
                response.Action = EmergencyAction.AltitudeHold;
                response.Command = GenerateAltitudeHoldCommand(vehicleId);
                break;

            default:
                response.Action = EmergencyAction.HoldPosition;
                response.Command = GenerateHoldCommand(vehicleId);
                break;
        }

        // Queue for cloud notification
        _pendingActions.Enqueue(new PendingCloudAction
        {
            Type = CloudActionType.EmergencyHandled,
            VehicleId = vehicleId,
            Timestamp = DateTime.UtcNow,
            Data = new Dictionary<string, object>
            {
                ["emergencyType"] = type.ToString(),
                ["action"] = response.Action.ToString(),
                ["handledLocally"] = true
            }
        });

        AlertGenerated?.Invoke(this, new EdgeAlertEventArgs(vehicleId, new EdgeAnomaly
        {
            Type = AnomalyType.Emergency,
            Severity = AlertSeverity.Emergency,
            Message = $"Emergency {type} handled locally: {response.Action}"
        }));

        return response;
    }

    private void ExecuteLocalAction(string vehicleId, LocalDecision decision)
    {
        // Execute the decision locally
        // In production, this would send commands to the flight controller

        _pendingActions.Enqueue(new PendingCloudAction
        {
            Type = CloudActionType.LocalDecisionMade,
            VehicleId = vehicleId,
            Timestamp = DateTime.UtcNow,
            Data = new Dictionary<string, object>
            {
                ["decision"] = decision.Type.ToString(),
                ["reason"] = decision.Reason
            }
        });
    }

    #endregion

    #region Connectivity & Sync

    private void CheckConnectivity(object? state)
    {
        var wasOffline = _isOfflineMode;
        _isOfflineMode = !_syncManager.IsCloudReachable();

        if (wasOffline != _isOfflineMode)
        {
            ModeChanged?.Invoke(this, new EdgeModeChangedEventArgs(_isOfflineMode));

            if (!_isOfflineMode)
            {
                // Coming back online - trigger sync
                Task.Run(SyncPendingActionsAsync);
            }
        }
    }

    private async void SyncWithCloud(object? state)
    {
        if (_isOfflineMode) return;

        try
        {
            // Sync pending actions
            await SyncPendingActionsAsync();

            // Sync telemetry
            var pendingTelemetry = GetPendingCloudTelemetry().ToList();
            if (pendingTelemetry.Count > 0)
            {
                var uploaded = await _syncManager.UploadTelemetryAsync(pendingTelemetry);
                if (uploaded)
                {
                    AcknowledgeCloudUpload(pendingTelemetry.Select(t => t.Id));
                }
            }

            // Sync new missions to cache
            var newMissions = await _syncManager.FetchNewMissionsAsync();
            foreach (var mission in newMissions)
            {
                await CacheMissionAsync(mission);
            }

            _lastCloudSync = DateTime.UtcNow;
        }
        catch (Exception ex)
        {
            // Log error but don't crash
            Console.WriteLine($"Edge sync error: {ex.Message}");
        }
    }

    private async Task SyncPendingActionsAsync()
    {
        var batch = new List<PendingCloudAction>();

        while (_pendingActions.TryDequeue(out var action) && batch.Count < 100)
        {
            batch.Add(action);
        }

        if (batch.Count > 0)
        {
            var success = await _syncManager.UploadActionsAsync(batch);
            if (!success)
            {
                // Re-queue failed actions
                foreach (var action in batch)
                {
                    _pendingActions.Enqueue(action);
                }
            }
        }
    }

    #endregion

    #region Command Generation

    private VehicleCommand GenerateRTLCommand(string vehicleId)
    {
        return new VehicleCommand
        {
            VehicleId = vehicleId,
            Type = CommandType.ReturnToLaunch,
            Priority = CommandPriority.Emergency,
            Timestamp = DateTime.UtcNow
        };
    }

    private VehicleCommand GenerateHoldCommand(string vehicleId)
    {
        return new VehicleCommand
        {
            VehicleId = vehicleId,
            Type = CommandType.HoldPosition,
            Priority = CommandPriority.High,
            Timestamp = DateTime.UtcNow
        };
    }

    private VehicleCommand GenerateGeofenceRecoveryCommand(string vehicleId)
    {
        if (_vehicleStates.TryGetValue(vehicleId, out var state))
        {
            // Calculate vector back to safe zone
            var safePosition = state.LastKnownSafePosition ?? state.HomePosition;
            return new VehicleCommand
            {
                VehicleId = vehicleId,
                Type = CommandType.GoToPosition,
                Priority = CommandPriority.Emergency,
                TargetPosition = safePosition,
                Timestamp = DateTime.UtcNow
            };
        }

        return GenerateRTLCommand(vehicleId);
    }

    private VehicleCommand GenerateEmergencyLandCommand(string vehicleId)
    {
        return new VehicleCommand
        {
            VehicleId = vehicleId,
            Type = CommandType.EmergencyLand,
            Priority = CommandPriority.Emergency,
            Timestamp = DateTime.UtcNow
        };
    }

    private VehicleCommand GenerateAltitudeHoldCommand(string vehicleId)
    {
        return new VehicleCommand
        {
            VehicleId = vehicleId,
            Type = CommandType.AltitudeHold,
            Priority = CommandPriority.High,
            Timestamp = DateTime.UtcNow
        };
    }

    #endregion

    #region Analytics

    /// <summary>
    /// Get edge node performance metrics.
    /// </summary>
    public EdgeMetrics GetMetrics()
    {
        return new EdgeMetrics
        {
            NodeId = _config.NodeId,
            Timestamp = DateTime.UtcNow,
            IsOfflineMode = _isOfflineMode,
            ConnectedVehicles = _vehicleStates.Count(kv => kv.Value.IsConnected),
            PendingCloudActions = _pendingActions.Count,
            CachedMissions = _missionCache.Count,
            TelemetryProcessedPerSecond = _analytics.GetTelemetryRate(),
            AnomaliesDetected = _analytics.GetAnomalyCount(),
            LastCloudSync = _lastCloudSync,
            UptimeSeconds = (DateTime.UtcNow - _config.StartTime).TotalSeconds,
            MemoryUsageMb = GC.GetTotalMemory(false) / (1024.0 * 1024.0)
        };
    }

    #endregion

    public void Dispose()
    {
        if (_isDisposed) return;
        _isDisposed = true;

        _syncTimer.Dispose();
        _healthTimer.Dispose();
        _missionCache.Dispose();
    }
}

#region Edge Models

public class EdgeConfiguration
{
    public string NodeId { get; set; } = Guid.NewGuid().ToString();
    public string CachePath { get; set; } = "./edge_cache";
    public string CloudEndpoint { get; set; } = string.Empty;
    public int SyncIntervalSeconds { get; set; } = 10;
    public DateTime StartTime { get; set; } = DateTime.UtcNow;
    public AggregationConfiguration AggregationConfig { get; set; } = new();
    public LocalDecisionConfiguration DecisionConfig { get; set; } = new();
    public OfflineConfiguration OfflineConfig { get; set; } = new();
    public SyncConfiguration SyncConfig { get; set; } = new();
}

public class AggregationConfiguration
{
    public double PositionChangeThreshold { get; set; } = 0.5; // meters
    public double VelocityChangeThreshold { get; set; } = 0.2; // m/s
    public double AttitudeChangeThreshold { get; set; } = 2.0; // degrees
    public int MinUploadIntervalMs { get; set; } = 1000;
    public int MaxBufferSize { get; set; } = 1000;
}

public class LocalDecisionConfiguration
{
    public double BatteryWarningThreshold { get; set; } = 25;
    public double BatteryCriticalThreshold { get; set; } = 15;
    public double MaxDistanceFromHome { get; set; } = 5000; // meters
    public int GpsLostTimeoutSeconds { get; set; } = 30;
    public int LinkLostTimeoutSeconds { get; set; } = 60;
}

public class OfflineConfiguration
{
    public int MaxCachedMissions { get; set; } = 100;
    public int MaxOfflineFlightMinutes { get; set; } = 30;
    public bool AllowOfflineMissionStart { get; set; } = true;
}

public class SyncConfiguration
{
    public int MaxRetries { get; set; } = 3;
    public int TimeoutSeconds { get; set; } = 30;
    public int BatchSize { get; set; } = 100;
}

public class VehicleEdgeState
{
    public string VehicleId { get; set; } = string.Empty;
    public bool IsConnected { get; set; }
    public DateTime LastUpdate { get; set; }
    public TelemetryFrame? LastTelemetry { get; set; }
    public FleetMission? ActiveMission { get; set; }
    public DateTime? MissionStartTime { get; set; }
    public int CurrentWaypointIndex { get; set; }
    public Vector<double>? HomePosition { get; set; }
    public Vector<double>? LastKnownSafePosition { get; set; }
}

public class EdgeTelemetryResult
{
    public string VehicleId { get; set; } = string.Empty;
    public DateTime Timestamp { get; set; }
    public bool ShouldUploadToCloud { get; set; }
    public AggregatedTelemetry? AggregatedFrame { get; set; }
    public List<EdgeAnomaly> Anomalies { get; set; } = new();
    public LocalDecision? LocalDecision { get; set; }
}

public class AggregatedTelemetry
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string VehicleId { get; set; } = string.Empty;
    public DateTime StartTime { get; set; }
    public DateTime EndTime { get; set; }
    public int SampleCount { get; set; }
    public TelemetryFrame RepresentativeFrame { get; set; } = null!;
    public TelemetrySummary Summary { get; set; } = new();
}

public class TelemetrySummary
{
    public double MinAltitude { get; set; }
    public double MaxAltitude { get; set; }
    public double AvgSpeed { get; set; }
    public double MaxSpeed { get; set; }
    public double MinBattery { get; set; }
    public double DistanceTraveled { get; set; }
}

public class EdgeAnomaly
{
    public AnomalyType Type { get; set; }
    public AlertSeverity Severity { get; set; }
    public string Message { get; set; } = string.Empty;
    public double Confidence { get; set; }
    public Dictionary<string, double>? Metrics { get; set; }
}

public enum AnomalyType
{
    BatteryDrain,
    MotorImbalance,
    VibrationExcessive,
    GpsDrift,
    AttitudeInstability,
    TemperatureHigh,
    SignalDegraded,
    Emergency
}

public class LocalDecision
{
    public LocalDecisionType Type { get; set; }
    public bool RequiresAction { get; set; }
    public string Reason { get; set; } = string.Empty;
    public VehicleCommand? Command { get; set; }
}

public enum LocalDecisionType
{
    Continue,
    AdjustSpeed,
    AdjustAltitude,
    SkipWaypoint,
    HoldPosition,
    ReturnToHome,
    EmergencyLand
}

public class OfflineMissionResult
{
    public bool Success { get; set; }
    public string? MissionId { get; set; }
    public DateTime StartedAt { get; set; }
    public string? Error { get; set; }
}

public class PendingCloudAction
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public CloudActionType Type { get; set; }
    public string? VehicleId { get; set; }
    public string? MissionId { get; set; }
    public DateTime Timestamp { get; set; }
    public Dictionary<string, object> Data { get; set; } = new();
}

public enum CloudActionType
{
    TelemetryBatch,
    MissionStarted,
    MissionCompleted,
    MissionAborted,
    AlertGenerated,
    EmergencyHandled,
    LocalDecisionMade,
    VehicleConnected,
    VehicleDisconnected
}

public class EmergencyResponse
{
    public string VehicleId { get; set; } = string.Empty;
    public EmergencyType EmergencyType { get; set; }
    public EmergencyAction Action { get; set; }
    public VehicleCommand? Command { get; set; }
    public DateTime Timestamp { get; set; }
    public bool HandledLocally { get; set; }
    public int? TimeoutSeconds { get; set; }
    public EmergencyAction? FallbackAction { get; set; }
}

public enum EmergencyType
{
    LowBattery,
    CriticalBattery,
    LostLink,
    GeofenceViolation,
    MotorFailure,
    GpsLost,
    ImuFailure,
    ObstacleDetected,
    WeatherAlert
}

public enum EmergencyAction
{
    Continue,
    HoldPosition,
    AltitudeHold,
    ReturnToHome,
    ReturnToSafeZone,
    Land,
    EmergencyLand
}

public class VehicleCommand
{
    public string VehicleId { get; set; } = string.Empty;
    public CommandType Type { get; set; }
    public CommandPriority Priority { get; set; }
    public Vector<double>? TargetPosition { get; set; }
    public double? TargetAltitude { get; set; }
    public double? TargetSpeed { get; set; }
    public DateTime Timestamp { get; set; }
    public Dictionary<string, object>? Parameters { get; set; }
}

public enum CommandType
{
    Arm,
    Disarm,
    Takeoff,
    Land,
    EmergencyLand,
    HoldPosition,
    AltitudeHold,
    GoToPosition,
    ReturnToLaunch,
    StartMission,
    PauseMission,
    ResumeMission,
    AbortMission
}

public enum CommandPriority
{
    Low,
    Normal,
    High,
    Emergency
}

public class EdgeMetrics
{
    public string NodeId { get; set; } = string.Empty;
    public DateTime Timestamp { get; set; }
    public bool IsOfflineMode { get; set; }
    public int ConnectedVehicles { get; set; }
    public int PendingCloudActions { get; set; }
    public int CachedMissions { get; set; }
    public double TelemetryProcessedPerSecond { get; set; }
    public int AnomaliesDetected { get; set; }
    public DateTime LastCloudSync { get; set; }
    public double UptimeSeconds { get; set; }
    public double MemoryUsageMb { get; set; }
}

public class EdgeModeChangedEventArgs : EventArgs
{
    public bool IsOfflineMode { get; }
    public DateTime Timestamp { get; }

    public EdgeModeChangedEventArgs(bool isOfflineMode)
    {
        IsOfflineMode = isOfflineMode;
        Timestamp = DateTime.UtcNow;
    }
}

public class EdgeAlertEventArgs : EventArgs
{
    public string VehicleId { get; }
    public EdgeAnomaly Anomaly { get; }

    public EdgeAlertEventArgs(string vehicleId, EdgeAnomaly anomaly)
    {
        VehicleId = vehicleId;
        Anomaly = anomaly;
    }
}

#endregion
