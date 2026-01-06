using System.Collections.Concurrent;
using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Enterprise Fleet Management System for multi-drone operations.
/// Provides centralized control, monitoring, and coordination for drone fleets.
/// Designed for 2-100+ vehicle operations with real-time state tracking.
/// </summary>
public class FleetManager : IDisposable
{
    private readonly FleetDatabase _database;
    private readonly VehicleRegistry _registry;
    private readonly MissionScheduler _scheduler;
    private readonly FleetAnalytics _analytics;
    private readonly MaintenanceManager _maintenance;
    private readonly ComplianceManager _compliance;

    private readonly ConcurrentDictionary<string, VehicleState> _liveStates;
    private readonly ConcurrentDictionary<string, ActiveMission> _activeMissions;
    private readonly Timer _healthCheckTimer;
    private readonly Timer _telemetryAggregationTimer;

    private bool _isDisposed;

    public event EventHandler<VehicleAlertEventArgs>? VehicleAlert;
    public event EventHandler<MissionEventArgs>? MissionStatusChanged;
    public event EventHandler<FleetMetricsEventArgs>? MetricsUpdated;

    public FleetManager(FleetConfiguration config)
    {
        _database = new FleetDatabase(config.DatabasePath);
        _registry = new VehicleRegistry(_database);
        _scheduler = new MissionScheduler(_registry, config.SchedulerConfig);
        _analytics = new FleetAnalytics(_database);
        _maintenance = new MaintenanceManager(_database, _registry);
        _compliance = new ComplianceManager(_database, config.ComplianceConfig);

        _liveStates = new ConcurrentDictionary<string, VehicleState>();
        _activeMissions = new ConcurrentDictionary<string, ActiveMission>();

        _healthCheckTimer = new Timer(PerformHealthChecks, null,
            TimeSpan.FromSeconds(5), TimeSpan.FromSeconds(config.HealthCheckIntervalSeconds));
        _telemetryAggregationTimer = new Timer(AggregateTelemetry, null,
            TimeSpan.FromSeconds(30), TimeSpan.FromSeconds(config.TelemetryAggregationIntervalSeconds));
    }

    #region Vehicle Management

    /// <summary>
    /// Register a new vehicle in the fleet.
    /// </summary>
    public async Task<RegistrationResult> RegisterVehicleAsync(VehicleRegistration registration)
    {
        var validation = await _compliance.ValidateRegistrationAsync(registration);
        if (!validation.IsValid)
        {
            return new RegistrationResult
            {
                Success = false,
                Errors = validation.Errors
            };
        }

        var vehicle = await _registry.RegisterAsync(registration);
        await _database.SaveVehicleAsync(vehicle);
        await _compliance.LogEventAsync(new ComplianceEvent
        {
            Type = ComplianceEventType.VehicleRegistered,
            VehicleId = vehicle.Id,
            Timestamp = DateTime.UtcNow,
            Details = $"Vehicle {vehicle.Callsign} registered to fleet"
        });

        _liveStates[vehicle.Id] = new VehicleState
        {
            VehicleId = vehicle.Id,
            Status = VehicleStatus.Offline,
            LastUpdate = DateTime.UtcNow
        };

        return new RegistrationResult
        {
            Success = true,
            VehicleId = vehicle.Id,
            Vehicle = vehicle
        };
    }

    /// <summary>
    /// Remove a vehicle from the fleet.
    /// </summary>
    public async Task<bool> DecommissionVehicleAsync(string vehicleId, string reason)
    {
        if (_activeMissions.ContainsKey(vehicleId))
        {
            throw new InvalidOperationException($"Cannot decommission vehicle {vehicleId} with active mission");
        }

        var vehicle = await _registry.GetVehicleAsync(vehicleId);
        if (vehicle == null) return false;

        vehicle.Status = VehicleStatus.Decommissioned;
        vehicle.DecommissionedAt = DateTime.UtcNow;
        vehicle.DecommissionReason = reason;

        await _database.SaveVehicleAsync(vehicle);
        await _compliance.LogEventAsync(new ComplianceEvent
        {
            Type = ComplianceEventType.VehicleDecommissioned,
            VehicleId = vehicleId,
            Timestamp = DateTime.UtcNow,
            Details = reason
        });

        _liveStates.TryRemove(vehicleId, out _);
        return true;
    }

    /// <summary>
    /// Get all vehicles in the fleet.
    /// </summary>
    public async Task<IReadOnlyList<Vehicle>> GetAllVehiclesAsync()
    {
        return await _registry.GetAllVehiclesAsync();
    }

    /// <summary>
    /// Get vehicles by status.
    /// </summary>
    public async Task<IReadOnlyList<Vehicle>> GetVehiclesByStatusAsync(VehicleStatus status)
    {
        return await _registry.GetVehiclesByStatusAsync(status);
    }

    /// <summary>
    /// Get real-time state of a vehicle.
    /// </summary>
    public VehicleState? GetLiveState(string vehicleId)
    {
        return _liveStates.TryGetValue(vehicleId, out var state) ? state : null;
    }

    /// <summary>
    /// Get real-time states of all vehicles.
    /// </summary>
    public IReadOnlyDictionary<string, VehicleState> GetAllLiveStates()
    {
        return _liveStates;
    }

    #endregion

    #region Telemetry Processing

    /// <summary>
    /// Process incoming telemetry from a vehicle.
    /// </summary>
    public async Task ProcessTelemetryAsync(string vehicleId, TelemetryFrame telemetry)
    {
        if (!_liveStates.TryGetValue(vehicleId, out var state))
        {
            // Unknown vehicle - could be newly connected
            state = new VehicleState { VehicleId = vehicleId };
            _liveStates[vehicleId] = state;
        }

        // Update live state
        state.Position = telemetry.Position;
        state.Velocity = telemetry.Velocity;
        state.Attitude = telemetry.Attitude;
        state.BatteryPercent = telemetry.BatteryPercent;
        state.BatteryVoltage = telemetry.BatteryVoltage;
        state.GpsFixType = telemetry.GpsFixType;
        state.SatelliteCount = telemetry.SatelliteCount;
        state.SignalStrength = telemetry.SignalStrength;
        state.LastUpdate = DateTime.UtcNow;
        state.Status = DetermineVehicleStatus(telemetry);

        // Check for alerts
        var alerts = CheckForAlerts(vehicleId, state, telemetry);
        foreach (var alert in alerts)
        {
            VehicleAlert?.Invoke(this, new VehicleAlertEventArgs(vehicleId, alert));
            await _database.SaveAlertAsync(alert);
        }

        // Update mission progress if active
        if (_activeMissions.TryGetValue(vehicleId, out var mission))
        {
            UpdateMissionProgress(mission, telemetry);
        }

        // Store telemetry for analytics (sampled)
        if (ShouldStoreTelemetry(vehicleId))
        {
            await _database.StoreTelemetryAsync(vehicleId, telemetry);
        }
    }

    private VehicleStatus DetermineVehicleStatus(TelemetryFrame telemetry)
    {
        if (telemetry.IsArmed && telemetry.Altitude > 1.0)
            return VehicleStatus.Flying;
        if (telemetry.IsArmed)
            return VehicleStatus.Armed;
        if (telemetry.BatteryPercent < 10)
            return VehicleStatus.CriticalBattery;
        return VehicleStatus.Ready;
    }

    private List<VehicleAlert> CheckForAlerts(string vehicleId, VehicleState state, TelemetryFrame telemetry)
    {
        var alerts = new List<VehicleAlert>();

        if (telemetry.BatteryPercent < 20 && telemetry.BatteryPercent >= 10)
        {
            alerts.Add(new VehicleAlert
            {
                VehicleId = vehicleId,
                Severity = AlertSeverity.Warning,
                Type = AlertType.LowBattery,
                Message = $"Battery at {telemetry.BatteryPercent}%",
                Timestamp = DateTime.UtcNow
            });
        }
        else if (telemetry.BatteryPercent < 10)
        {
            alerts.Add(new VehicleAlert
            {
                VehicleId = vehicleId,
                Severity = AlertSeverity.Critical,
                Type = AlertType.CriticalBattery,
                Message = $"Critical battery at {telemetry.BatteryPercent}%",
                Timestamp = DateTime.UtcNow
            });
        }

        if (telemetry.GpsFixType < 3 && state.Status == VehicleStatus.Flying)
        {
            alerts.Add(new VehicleAlert
            {
                VehicleId = vehicleId,
                Severity = AlertSeverity.Warning,
                Type = AlertType.GpsDegraded,
                Message = $"GPS fix degraded: {telemetry.GpsFixType}",
                Timestamp = DateTime.UtcNow
            });
        }

        if (telemetry.SignalStrength < 30)
        {
            alerts.Add(new VehicleAlert
            {
                VehicleId = vehicleId,
                Severity = AlertSeverity.Warning,
                Type = AlertType.WeakSignal,
                Message = $"Weak signal: {telemetry.SignalStrength}%",
                Timestamp = DateTime.UtcNow
            });
        }

        return alerts;
    }

    private readonly ConcurrentDictionary<string, DateTime> _lastTelemetryStore = new();

    private bool ShouldStoreTelemetry(string vehicleId)
    {
        var now = DateTime.UtcNow;
        if (_lastTelemetryStore.TryGetValue(vehicleId, out var last))
        {
            if ((now - last).TotalSeconds < 1.0) return false;
        }
        _lastTelemetryStore[vehicleId] = now;
        return true;
    }

    #endregion

    #region Mission Management

    /// <summary>
    /// Create and schedule a new mission.
    /// </summary>
    public async Task<MissionResult> CreateMissionAsync(MissionRequest request)
    {
        // Validate mission
        var validation = await ValidateMissionAsync(request);
        if (!validation.IsValid)
        {
            return new MissionResult
            {
                Success = false,
                Errors = validation.Errors
            };
        }

        // Create mission
        var mission = new Mission
        {
            Id = Guid.NewGuid().ToString(),
            Name = request.Name,
            Type = request.Type,
            Priority = request.Priority,
            CreatedAt = DateTime.UtcNow,
            ScheduledStart = request.ScheduledStart,
            EstimatedDuration = request.EstimatedDuration,
            Waypoints = request.Waypoints,
            Parameters = request.Parameters,
            Status = MissionStatus.Scheduled,
            CreatedBy = request.OperatorId
        };

        // Assign vehicle if specified, or let scheduler pick
        if (!string.IsNullOrEmpty(request.VehicleId))
        {
            mission.AssignedVehicleId = request.VehicleId;
        }
        else
        {
            var assignment = await _scheduler.FindBestVehicleAsync(mission);
            if (assignment == null)
            {
                return new MissionResult
                {
                    Success = false,
                    Errors = new[] { "No suitable vehicle available for mission" }
                };
            }
            mission.AssignedVehicleId = assignment.VehicleId;
        }

        await _database.SaveMissionAsync(mission);
        await _compliance.LogEventAsync(new ComplianceEvent
        {
            Type = ComplianceEventType.MissionCreated,
            VehicleId = mission.AssignedVehicleId,
            MissionId = mission.Id,
            Timestamp = DateTime.UtcNow,
            Details = $"Mission {mission.Name} created"
        });

        return new MissionResult
        {
            Success = true,
            MissionId = mission.Id,
            Mission = mission
        };
    }

    /// <summary>
    /// Start a scheduled mission.
    /// </summary>
    public async Task<bool> StartMissionAsync(string missionId, string operatorId)
    {
        var mission = await _database.GetMissionAsync(missionId);
        if (mission == null) return false;

        // Authorization check
        var authorized = await _compliance.CheckMissionAuthorizationAsync(
            missionId, mission.AssignedVehicleId!, operatorId);
        if (!authorized.IsAuthorized)
        {
            throw new UnauthorizedAccessException(authorized.Reason);
        }

        // Verify vehicle is ready
        var vehicleState = GetLiveState(mission.AssignedVehicleId!);
        if (vehicleState == null || vehicleState.Status != VehicleStatus.Ready)
        {
            throw new InvalidOperationException($"Vehicle {mission.AssignedVehicleId} is not ready");
        }

        mission.Status = MissionStatus.InProgress;
        mission.StartedAt = DateTime.UtcNow;
        mission.StartedBy = operatorId;

        await _database.SaveMissionAsync(mission);

        _activeMissions[mission.AssignedVehicleId!] = new ActiveMission
        {
            Mission = mission,
            CurrentWaypointIndex = 0,
            StartTime = DateTime.UtcNow
        };

        await _compliance.LogEventAsync(new ComplianceEvent
        {
            Type = ComplianceEventType.MissionStarted,
            VehicleId = mission.AssignedVehicleId,
            MissionId = missionId,
            OperatorId = operatorId,
            Timestamp = DateTime.UtcNow
        });

        MissionStatusChanged?.Invoke(this, new MissionEventArgs(mission, MissionEventType.Started));
        return true;
    }

    /// <summary>
    /// Abort an active mission.
    /// </summary>
    public async Task<bool> AbortMissionAsync(string missionId, string operatorId, string reason)
    {
        var mission = await _database.GetMissionAsync(missionId);
        if (mission == null) return false;

        mission.Status = MissionStatus.Aborted;
        mission.CompletedAt = DateTime.UtcNow;
        mission.AbortReason = reason;

        await _database.SaveMissionAsync(mission);

        if (mission.AssignedVehicleId != null)
        {
            _activeMissions.TryRemove(mission.AssignedVehicleId, out _);
        }

        await _compliance.LogEventAsync(new ComplianceEvent
        {
            Type = ComplianceEventType.MissionAborted,
            VehicleId = mission.AssignedVehicleId,
            MissionId = missionId,
            OperatorId = operatorId,
            Timestamp = DateTime.UtcNow,
            Details = reason
        });

        MissionStatusChanged?.Invoke(this, new MissionEventArgs(mission, MissionEventType.Aborted));
        return true;
    }

    /// <summary>
    /// Get mission history for a vehicle.
    /// </summary>
    public async Task<IReadOnlyList<Mission>> GetMissionHistoryAsync(
        string? vehicleId = null,
        DateTime? fromDate = null,
        DateTime? toDate = null,
        int limit = 100)
    {
        return await _database.GetMissionsAsync(vehicleId, fromDate, toDate, limit);
    }

    private void UpdateMissionProgress(ActiveMission activeMission, TelemetryFrame telemetry)
    {
        var mission = activeMission.Mission;
        if (activeMission.CurrentWaypointIndex >= mission.Waypoints.Count)
        {
            // Mission complete
            CompleteMissionAsync(mission.Id).Wait();
            return;
        }

        var currentWaypoint = mission.Waypoints[activeMission.CurrentWaypointIndex];
        var distance = CalculateDistance(telemetry.Position, currentWaypoint.Position);

        if (distance < currentWaypoint.AcceptanceRadius)
        {
            activeMission.CurrentWaypointIndex++;
            activeMission.WaypointReachedTimes.Add(DateTime.UtcNow);
        }

        activeMission.DistanceToWaypoint = distance;
        activeMission.Progress = (double)activeMission.CurrentWaypointIndex / mission.Waypoints.Count;
    }

    private async Task CompleteMissionAsync(string missionId)
    {
        var mission = await _database.GetMissionAsync(missionId);
        if (mission == null) return;

        mission.Status = MissionStatus.Completed;
        mission.CompletedAt = DateTime.UtcNow;

        await _database.SaveMissionAsync(mission);

        if (mission.AssignedVehicleId != null)
        {
            _activeMissions.TryRemove(mission.AssignedVehicleId, out _);
        }

        await _compliance.LogEventAsync(new ComplianceEvent
        {
            Type = ComplianceEventType.MissionCompleted,
            VehicleId = mission.AssignedVehicleId,
            MissionId = missionId,
            Timestamp = DateTime.UtcNow
        });

        MissionStatusChanged?.Invoke(this, new MissionEventArgs(mission, MissionEventType.Completed));
    }

    private async Task<ValidationResult> ValidateMissionAsync(MissionRequest request)
    {
        var errors = new List<string>();

        if (string.IsNullOrEmpty(request.Name))
            errors.Add("Mission name is required");

        if (request.Waypoints == null || request.Waypoints.Count == 0)
            errors.Add("At least one waypoint is required");

        if (request.ScheduledStart < DateTime.UtcNow)
            errors.Add("Scheduled start must be in the future");

        // Check geofence compliance
        if (request.Waypoints != null)
        {
            foreach (var wp in request.Waypoints)
            {
                var geofenceCheck = await _compliance.CheckGeofenceAsync(wp.Position);
                if (!geofenceCheck.IsAllowed)
                {
                    errors.Add($"Waypoint at {wp.Position} violates geofence: {geofenceCheck.Reason}");
                }
            }
        }

        return new ValidationResult
        {
            IsValid = errors.Count == 0,
            Errors = errors.ToArray()
        };
    }

    private double CalculateDistance(Vector<double> a, Vector<double> b)
    {
        return (a - b).L2Norm();
    }

    #endregion

    #region Fleet Analytics

    /// <summary>
    /// Get real-time fleet metrics.
    /// </summary>
    public FleetMetrics GetFleetMetrics()
    {
        var states = _liveStates.Values.ToList();

        return new FleetMetrics
        {
            TotalVehicles = states.Count,
            VehiclesOnline = states.Count(s => s.Status != VehicleStatus.Offline),
            VehiclesFlying = states.Count(s => s.Status == VehicleStatus.Flying),
            VehiclesReady = states.Count(s => s.Status == VehicleStatus.Ready),
            VehiclesInMaintenance = states.Count(s => s.Status == VehicleStatus.Maintenance),
            ActiveMissions = _activeMissions.Count,
            AverageBattery = states.Where(s => s.BatteryPercent > 0).Select(s => s.BatteryPercent).DefaultIfEmpty(0).Average(),
            AverageSignalStrength = states.Where(s => s.SignalStrength > 0).Select(s => s.SignalStrength).DefaultIfEmpty(0).Average(),
            Timestamp = DateTime.UtcNow
        };
    }

    /// <summary>
    /// Get historical analytics.
    /// </summary>
    public async Task<FleetAnalyticsReport> GetAnalyticsReportAsync(
        DateTime fromDate,
        DateTime toDate,
        string? vehicleId = null)
    {
        return await _analytics.GenerateReportAsync(fromDate, toDate, vehicleId);
    }

    /// <summary>
    /// Get utilization statistics.
    /// </summary>
    public async Task<UtilizationStats> GetUtilizationStatsAsync(DateTime fromDate, DateTime toDate)
    {
        return await _analytics.GetUtilizationAsync(fromDate, toDate);
    }

    #endregion

    #region Maintenance

    /// <summary>
    /// Schedule maintenance for a vehicle.
    /// </summary>
    public async Task<MaintenanceTicket> ScheduleMaintenanceAsync(MaintenanceRequest request)
    {
        return await _maintenance.ScheduleAsync(request);
    }

    /// <summary>
    /// Get pending maintenance for fleet.
    /// </summary>
    public async Task<IReadOnlyList<MaintenanceTicket>> GetPendingMaintenanceAsync()
    {
        return await _maintenance.GetPendingAsync();
    }

    /// <summary>
    /// Complete a maintenance ticket.
    /// </summary>
    public async Task CompleteMaintenanceAsync(string ticketId, MaintenanceCompletion completion)
    {
        await _maintenance.CompleteAsync(ticketId, completion);
    }

    #endregion

    #region Background Tasks

    private void PerformHealthChecks(object? state)
    {
        var now = DateTime.UtcNow;
        var staleThreshold = TimeSpan.FromSeconds(30);

        foreach (var (vehicleId, vehicleState) in _liveStates)
        {
            if (now - vehicleState.LastUpdate > staleThreshold &&
                vehicleState.Status != VehicleStatus.Offline)
            {
                vehicleState.Status = VehicleStatus.Offline;
                VehicleAlert?.Invoke(this, new VehicleAlertEventArgs(vehicleId, new VehicleAlert
                {
                    VehicleId = vehicleId,
                    Severity = AlertSeverity.Warning,
                    Type = AlertType.ConnectionLost,
                    Message = "Vehicle connection lost",
                    Timestamp = now
                }));
            }
        }
    }

    private void AggregateTelemetry(object? state)
    {
        var metrics = GetFleetMetrics();
        MetricsUpdated?.Invoke(this, new FleetMetricsEventArgs(metrics));

        // Store aggregated metrics
        _database.StoreFleetMetricsAsync(metrics).Wait();
    }

    #endregion

    public void Dispose()
    {
        if (_isDisposed) return;
        _isDisposed = true;

        _healthCheckTimer.Dispose();
        _telemetryAggregationTimer.Dispose();
        _database.Dispose();
    }
}

#region Configuration

public class FleetConfiguration
{
    public string DatabasePath { get; set; } = "fleet.db";
    public int HealthCheckIntervalSeconds { get; set; } = 5;
    public int TelemetryAggregationIntervalSeconds { get; set; } = 30;
    public SchedulerConfiguration SchedulerConfig { get; set; } = new();
    public ComplianceConfiguration ComplianceConfig { get; set; } = new();
}

public class SchedulerConfiguration
{
    public int MaxConcurrentMissions { get; set; } = 10;
    public double MinBatteryForMission { get; set; } = 30;
    public TimeSpan MaxMissionDuration { get; set; } = TimeSpan.FromHours(1);
}

public class ComplianceConfiguration
{
    public bool RequirePilotCertification { get; set; } = true;
    public bool EnforceGeofencing { get; set; } = true;
    public bool RequirePreflightChecklist { get; set; } = true;
    public List<GeofenceZone> RestrictedZones { get; set; } = new();
}

#endregion

#region Events

public class VehicleAlertEventArgs : EventArgs
{
    public string VehicleId { get; }
    public VehicleAlert Alert { get; }

    public VehicleAlertEventArgs(string vehicleId, VehicleAlert alert)
    {
        VehicleId = vehicleId;
        Alert = alert;
    }
}

public class MissionEventArgs : EventArgs
{
    public Mission Mission { get; }
    public MissionEventType EventType { get; }

    public MissionEventArgs(Mission mission, MissionEventType eventType)
    {
        Mission = mission;
        EventType = eventType;
    }
}

public enum MissionEventType
{
    Created,
    Started,
    WaypointReached,
    Completed,
    Aborted,
    Failed
}

public class FleetMetricsEventArgs : EventArgs
{
    public FleetMetrics Metrics { get; }

    public FleetMetricsEventArgs(FleetMetrics metrics)
    {
        Metrics = metrics;
    }
}

#endregion
