using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Fleet;

#region Vehicle Models

/// <summary>
/// Complete vehicle record for fleet management.
/// </summary>
public class Vehicle
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string Callsign { get; set; } = string.Empty;
    public string SerialNumber { get; set; } = string.Empty;
    public string Model { get; set; } = string.Empty;
    public string Manufacturer { get; set; } = string.Empty;

    // Specifications
    public VehicleSpecs Specs { get; set; } = new();

    // Status
    public VehicleStatus Status { get; set; } = VehicleStatus.Offline;
    public DateTime RegisteredAt { get; set; } = DateTime.UtcNow;
    public DateTime? LastFlightAt { get; set; }
    public DateTime? LastMaintenanceAt { get; set; }
    public DateTime? DecommissionedAt { get; set; }
    public string? DecommissionReason { get; set; }

    // Flight Statistics
    public double TotalFlightHours { get; set; }
    public int TotalFlights { get; set; }
    public double TotalDistanceKm { get; set; }

    // Maintenance
    public double FlightHoursSinceLastMaintenance { get; set; }
    public int FlightsSinceLastMaintenance { get; set; }
    public MaintenanceSchedule MaintenanceSchedule { get; set; } = new();

    // Compliance
    public string? FaaRegistrationNumber { get; set; }
    public DateTime? FaaRegistrationExpiry { get; set; }
    public List<string> InstalledPayloads { get; set; } = new();
    public List<VehicleCertification> Certifications { get; set; } = new();

    // Configuration
    public Dictionary<string, object> Configuration { get; set; } = new();
    public string? FirmwareVersion { get; set; }
    public DateTime? FirmwareUpdatedAt { get; set; }
}

public class VehicleSpecs
{
    public VehicleType Type { get; set; } = VehicleType.Quadcopter;
    public double MaxTakeoffWeightKg { get; set; } = 2.5;
    public double EmptyWeightKg { get; set; } = 1.5;
    public double MaxPayloadKg { get; set; } = 1.0;
    public double MaxSpeedMs { get; set; } = 20.0;
    public double MaxAltitudeM { get; set; } = 120.0;
    public double MaxRangeKm { get; set; } = 5.0;
    public double MaxFlightTimeMinutes { get; set; } = 30.0;
    public double BatteryCapacityWh { get; set; } = 100.0;
    public int MotorCount { get; set; } = 4;
    public double PropellerDiameterInches { get; set; } = 10.0;
    public bool HasRedundantImu { get; set; } = true;
    public bool HasRedundantGps { get; set; } = true;
    public bool HasObstacleAvoidance { get; set; } = true;
    public List<string> SupportedPayloads { get; set; } = new();
}

public enum VehicleType
{
    Quadcopter,
    Hexacopter,
    Octocopter,
    FixedWing,
    VTOL,
    Helicopter
}

public enum VehicleStatus
{
    Offline,
    Connecting,
    Ready,
    Armed,
    Flying,
    Landing,
    Returning,
    CriticalBattery,
    Emergency,
    Maintenance,
    Decommissioned
}

public class VehicleState
{
    public string VehicleId { get; set; } = string.Empty;
    public VehicleStatus Status { get; set; }
    public DateTime LastUpdate { get; set; }

    // Position & Motion
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Attitude { get; set; } = Vector<double>.Build.Dense(3); // Roll, Pitch, Yaw

    // Health
    public double BatteryPercent { get; set; }
    public double BatteryVoltage { get; set; }
    public double BatteryTemperature { get; set; }

    // Connectivity
    public int GpsFixType { get; set; }
    public int SatelliteCount { get; set; }
    public double SignalStrength { get; set; }
    public double Latency { get; set; }

    // Sensors
    public bool ImuHealthy { get; set; } = true;
    public bool GpsHealthy { get; set; } = true;
    public bool CompassHealthy { get; set; } = true;
    public bool BarometerHealthy { get; set; } = true;

    // Current Mission
    public string? ActiveMissionId { get; set; }
    public int CurrentWaypointIndex { get; set; }
    public double MissionProgress { get; set; }
}

public class VehicleRegistration
{
    public string Callsign { get; set; } = string.Empty;
    public string SerialNumber { get; set; } = string.Empty;
    public string Model { get; set; } = string.Empty;
    public string Manufacturer { get; set; } = string.Empty;
    public VehicleSpecs Specs { get; set; } = new();
    public string? FaaRegistrationNumber { get; set; }
    public string RegisteredBy { get; set; } = string.Empty;
}

public class RegistrationResult
{
    public bool Success { get; set; }
    public string? VehicleId { get; set; }
    public Vehicle? Vehicle { get; set; }
    public string[] Errors { get; set; } = Array.Empty<string>();
}

public class VehicleCertification
{
    public string Type { get; set; } = string.Empty;
    public string Number { get; set; } = string.Empty;
    public DateTime IssuedAt { get; set; }
    public DateTime ExpiresAt { get; set; }
    public string IssuedBy { get; set; } = string.Empty;
}

#endregion

#region Mission Models

public class Mission
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string Name { get; set; } = string.Empty;
    public string? Description { get; set; }
    public MissionType Type { get; set; }
    public MissionPriority Priority { get; set; }
    public MissionStatus Status { get; set; }

    // Assignment
    public string? AssignedVehicleId { get; set; }
    public string? AssignedPilotId { get; set; }

    // Timing
    public DateTime CreatedAt { get; set; }
    public DateTime? ScheduledStart { get; set; }
    public DateTime? StartedAt { get; set; }
    public DateTime? CompletedAt { get; set; }
    public TimeSpan? EstimatedDuration { get; set; }
    public TimeSpan? ActualDuration => CompletedAt.HasValue && StartedAt.HasValue
        ? CompletedAt.Value - StartedAt.Value
        : null;

    // Route
    public List<MissionWaypoint> Waypoints { get; set; } = new();
    public double EstimatedDistanceKm { get; set; }
    public double ActualDistanceKm { get; set; }

    // Results
    public string? AbortReason { get; set; }
    public MissionResults? Results { get; set; }

    // Metadata
    public string CreatedBy { get; set; } = string.Empty;
    public string? StartedBy { get; set; }
    public Dictionary<string, object> Parameters { get; set; } = new();
    public List<string> Tags { get; set; } = new();
}

public enum MissionType
{
    Survey,
    Inspection,
    Delivery,
    Surveillance,
    Search,
    Mapping,
    Photography,
    Custom
}

public enum MissionPriority
{
    Low,
    Normal,
    High,
    Urgent,
    Emergency
}

public enum MissionStatus
{
    Draft,
    Scheduled,
    PreFlight,
    InProgress,
    Paused,
    Returning,
    Completed,
    Aborted,
    Failed
}

public class MissionWaypoint
{
    public int Index { get; set; }
    public string? Name { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public double Speed { get; set; } = 5.0;
    public double AcceptanceRadius { get; set; } = 2.0;
    public TimeSpan? HoldTime { get; set; }
    public WaypointAction Action { get; set; } = WaypointAction.None;
    public Dictionary<string, object>? ActionParameters { get; set; }
}

public enum WaypointAction
{
    None,
    TakePhoto,
    StartVideo,
    StopVideo,
    DropPayload,
    Scan,
    Hover,
    Orbit,
    Land,
    RTL
}

public class MissionRequest
{
    public string Name { get; set; } = string.Empty;
    public string? Description { get; set; }
    public MissionType Type { get; set; }
    public MissionPriority Priority { get; set; } = MissionPriority.Normal;
    public string? VehicleId { get; set; }
    public DateTime? ScheduledStart { get; set; }
    public TimeSpan? EstimatedDuration { get; set; }
    public List<MissionWaypoint> Waypoints { get; set; } = new();
    public Dictionary<string, object> Parameters { get; set; } = new();
    public string OperatorId { get; set; } = string.Empty;
}

public class MissionResult
{
    public bool Success { get; set; }
    public string? MissionId { get; set; }
    public Mission? Mission { get; set; }
    public string[] Errors { get; set; } = Array.Empty<string>();
}

public class MissionResults
{
    public bool Successful { get; set; }
    public int WaypointsCompleted { get; set; }
    public int WaypointsTotal { get; set; }
    public double DistanceFlownKm { get; set; }
    public TimeSpan FlightTime { get; set; }
    public double BatteryUsedPercent { get; set; }
    public int PhotosTaken { get; set; }
    public int VideosRecorded { get; set; }
    public List<string> DataFiles { get; set; } = new();
    public Dictionary<string, object> CustomMetrics { get; set; } = new();
}

public class ActiveMission
{
    public Mission Mission { get; set; } = null!;
    public DateTime StartTime { get; set; }
    public int CurrentWaypointIndex { get; set; }
    public double DistanceToWaypoint { get; set; }
    public double Progress { get; set; }
    public List<DateTime> WaypointReachedTimes { get; set; } = new();
}

#endregion

#region Telemetry Models

public class TelemetryFrame
{
    public DateTime Timestamp { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Attitude { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);

    // Battery
    public double BatteryPercent { get; set; }
    public double BatteryVoltage { get; set; }
    public double BatteryCurrent { get; set; }
    public double BatteryTemperature { get; set; }

    // GPS
    public int GpsFixType { get; set; }
    public int SatelliteCount { get; set; }
    public double GpsHdop { get; set; }
    public double GpsVdop { get; set; }

    // Link
    public double SignalStrength { get; set; }
    public double Latency { get; set; }
    public int PacketLoss { get; set; }

    // Status
    public bool IsArmed { get; set; }
    public double Altitude { get; set; }
    public double GroundSpeed { get; set; }
    public double Heading { get; set; }

    // Motors
    public double[] MotorOutputs { get; set; } = Array.Empty<double>();
    public double[] MotorRpms { get; set; } = Array.Empty<double>();
}

#endregion

#region Alert Models

public class VehicleAlert
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string VehicleId { get; set; } = string.Empty;
    public AlertSeverity Severity { get; set; }
    public AlertType Type { get; set; }
    public string Message { get; set; } = string.Empty;
    public DateTime Timestamp { get; set; }
    public bool Acknowledged { get; set; }
    public string? AcknowledgedBy { get; set; }
    public DateTime? AcknowledgedAt { get; set; }
    public Dictionary<string, object>? Context { get; set; }
}

public enum AlertSeverity
{
    Info,
    Warning,
    Critical,
    Emergency
}

public enum AlertType
{
    LowBattery,
    CriticalBattery,
    GpsDegraded,
    GpsLost,
    WeakSignal,
    ConnectionLost,
    SensorFailure,
    MotorAnomaly,
    GeofenceWarning,
    GeofenceViolation,
    ObstacleDetected,
    WindWarning,
    MaintenanceDue,
    FirmwareOutdated,
    UnauthorizedCommand,
    Custom
}

#endregion

#region Analytics Models

public class FleetMetrics
{
    public DateTime Timestamp { get; set; }
    public int TotalVehicles { get; set; }
    public int VehiclesOnline { get; set; }
    public int VehiclesFlying { get; set; }
    public int VehiclesReady { get; set; }
    public int VehiclesInMaintenance { get; set; }
    public int ActiveMissions { get; set; }
    public double AverageBattery { get; set; }
    public double AverageSignalStrength { get; set; }
}

public class FleetAnalyticsReport
{
    public DateTime FromDate { get; set; }
    public DateTime ToDate { get; set; }
    public string? VehicleId { get; set; }

    // Flight Statistics
    public int TotalFlights { get; set; }
    public double TotalFlightHours { get; set; }
    public double TotalDistanceKm { get; set; }
    public double AverageFlightDurationMinutes { get; set; }

    // Mission Statistics
    public int MissionsCompleted { get; set; }
    public int MissionsAborted { get; set; }
    public int MissionsFailed { get; set; }
    public double MissionSuccessRate { get; set; }

    // Reliability
    public int AlertsGenerated { get; set; }
    public int CriticalAlerts { get; set; }
    public double UptimePercent { get; set; }
    public double MtbfHours { get; set; } // Mean Time Between Failures

    // Resource Usage
    public double TotalEnergyUsedWh { get; set; }
    public double AverageBatteryUsagePerFlight { get; set; }

    // Cost Analysis
    public double EstimatedOperatingCost { get; set; }
    public double CostPerFlightHour { get; set; }
    public double CostPerKm { get; set; }

    public List<DailyFlightStats> DailyStats { get; set; } = new();
    public Dictionary<string, VehicleAnalytics> VehicleBreakdown { get; set; } = new();
}

public class DailyFlightStats
{
    public DateTime Date { get; set; }
    public int Flights { get; set; }
    public double FlightHours { get; set; }
    public double DistanceKm { get; set; }
    public int Missions { get; set; }
}

public class VehicleAnalytics
{
    public string VehicleId { get; set; } = string.Empty;
    public string Callsign { get; set; } = string.Empty;
    public int Flights { get; set; }
    public double FlightHours { get; set; }
    public double DistanceKm { get; set; }
    public double UtilizationPercent { get; set; }
    public int Alerts { get; set; }
}

public class UtilizationStats
{
    public DateTime FromDate { get; set; }
    public DateTime ToDate { get; set; }
    public double FleetUtilizationPercent { get; set; }
    public double PeakUtilizationPercent { get; set; }
    public DateTime PeakUtilizationTime { get; set; }
    public Dictionary<string, double> VehicleUtilization { get; set; } = new();
    public Dictionary<DayOfWeek, double> UtilizationByDayOfWeek { get; set; } = new();
    public Dictionary<int, double> UtilizationByHour { get; set; } = new();
}

#endregion

#region Maintenance Models

public class MaintenanceSchedule
{
    public double FlightHoursInterval { get; set; } = 50;
    public int FlightsInterval { get; set; } = 100;
    public int DaysInterval { get; set; } = 30;
}

public class MaintenanceRequest
{
    public string VehicleId { get; set; } = string.Empty;
    public MaintenanceType Type { get; set; }
    public MaintenancePriority Priority { get; set; }
    public string Description { get; set; } = string.Empty;
    public string RequestedBy { get; set; } = string.Empty;
    public DateTime? ScheduledFor { get; set; }
    public List<string> RequiredParts { get; set; } = new();
}

public class MaintenanceTicket
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string VehicleId { get; set; } = string.Empty;
    public MaintenanceType Type { get; set; }
    public MaintenancePriority Priority { get; set; }
    public MaintenanceStatus Status { get; set; }
    public string Description { get; set; } = string.Empty;
    public DateTime CreatedAt { get; set; }
    public DateTime? ScheduledFor { get; set; }
    public DateTime? StartedAt { get; set; }
    public DateTime? CompletedAt { get; set; }
    public string? AssignedTo { get; set; }
    public List<string> RequiredParts { get; set; } = new();
    public List<string> UsedParts { get; set; } = new();
    public string? Notes { get; set; }
    public double? LaborHours { get; set; }
    public double? Cost { get; set; }
}

public enum MaintenanceType
{
    Scheduled,
    Unscheduled,
    Preventive,
    Corrective,
    Inspection,
    Calibration,
    FirmwareUpdate,
    BatteryReplacement,
    MotorReplacement,
    PropellerReplacement
}

public enum MaintenancePriority
{
    Low,
    Medium,
    High,
    Critical
}

public enum MaintenanceStatus
{
    Requested,
    Scheduled,
    InProgress,
    AwaitingParts,
    Completed,
    Cancelled
}

public class MaintenanceCompletion
{
    public string CompletedBy { get; set; } = string.Empty;
    public List<string> PartsUsed { get; set; } = new();
    public double LaborHours { get; set; }
    public double PartsCost { get; set; }
    public double LaborCost { get; set; }
    public string Notes { get; set; } = string.Empty;
    public bool PassedInspection { get; set; }
    public List<string>? FollowUpRequired { get; set; }
}

#endregion

#region Compliance Models

public class ComplianceEvent
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public ComplianceEventType Type { get; set; }
    public string? VehicleId { get; set; }
    public string? MissionId { get; set; }
    public string? OperatorId { get; set; }
    public DateTime Timestamp { get; set; }
    public string? Details { get; set; }
    public Dictionary<string, object>? Metadata { get; set; }
}

public enum ComplianceEventType
{
    VehicleRegistered,
    VehicleDecommissioned,
    MissionCreated,
    MissionStarted,
    MissionCompleted,
    MissionAborted,
    MissionFailed,
    GeofenceViolation,
    AltitudeViolation,
    UnauthorizedFlight,
    PilotCertificationExpired,
    MaintenanceOverdue,
    EmergencyLanding,
    IncidentReported
}

public class GeofenceZone
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string Name { get; set; } = string.Empty;
    public GeofenceType Type { get; set; }
    public List<Vector<double>> Vertices { get; set; } = new();
    public double? MinAltitude { get; set; }
    public double? MaxAltitude { get; set; }
    public bool IsActive { get; set; } = true;
    public DateTime? ActiveFrom { get; set; }
    public DateTime? ActiveUntil { get; set; }
    public string? Reason { get; set; }
}

public enum GeofenceType
{
    NoFlyZone,
    RestrictedZone,
    AuthorizationRequired,
    AltitudeRestriction,
    OperationalBoundary
}

public class GeofenceCheckResult
{
    public bool IsAllowed { get; set; }
    public string? Reason { get; set; }
    public GeofenceZone? ViolatedZone { get; set; }
}

public class AuthorizationResult
{
    public bool IsAuthorized { get; set; }
    public string? Reason { get; set; }
    public List<string>? MissingRequirements { get; set; }
}

public class ValidationResult
{
    public bool IsValid { get; set; }
    public string[] Errors { get; set; } = Array.Empty<string>();
}

#endregion
