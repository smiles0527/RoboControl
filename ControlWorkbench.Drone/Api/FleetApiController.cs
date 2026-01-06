using System.Text.Json;
using ControlWorkbench.Drone.Fleet;
using FleetMission = ControlWorkbench.Drone.Fleet.Mission;

namespace ControlWorkbench.Drone.Api;

/// <summary>
/// Fleet API Controller - RESTful API for fleet management operations.
/// Designed for integration with external systems, mobile apps, and web dashboards.
/// </summary>
public class FleetApiController
{
    private readonly FleetManager _fleetManager;
    private readonly JsonSerializerOptions _jsonOptions;

    public FleetApiController(FleetManager fleetManager)
    {
        _fleetManager = fleetManager;
        _jsonOptions = new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            WriteIndented = true
        };
    }

    #region Vehicle Endpoints

    /// <summary>
    /// GET /api/vehicles - List all vehicles
    /// </summary>
    public async Task<ApiResponse<List<VehicleDto>>> GetVehiclesAsync(VehicleFilter? filter = null)
    {
        try
        {
            var vehicles = await _fleetManager.GetAllVehiclesAsync();

            if (filter != null)
            {
                if (filter.Status.HasValue)
                    vehicles = vehicles.Where(v => v.Status == filter.Status.Value).ToList();

                if (!string.IsNullOrEmpty(filter.Model))
                    vehicles = vehicles.Where(v => v.Model.Contains(filter.Model, StringComparison.OrdinalIgnoreCase)).ToList();
            }

            var dtos = vehicles.Select(MapToDto).ToList();

            return ApiResponse<List<VehicleDto>>.Ok(dtos);
        }
        catch (Exception ex)
        {
            return ApiResponse<List<VehicleDto>>.Error(ex.Message);
        }
    }

    /// <summary>
    /// GET /api/vehicles/{id} - Get vehicle details
    /// </summary>
    public async Task<ApiResponse<VehicleDetailDto>> GetVehicleAsync(string vehicleId)
    {
        try
        {
            var vehicles = await _fleetManager.GetAllVehiclesAsync();
            var vehicle = vehicles.FirstOrDefault(v => v.Id == vehicleId);

            if (vehicle == null)
                return ApiResponse<VehicleDetailDto>.NotFound("Vehicle not found");

            var liveState = _fleetManager.GetLiveState(vehicleId);
            var dto = MapToDetailDto(vehicle, liveState);

            return ApiResponse<VehicleDetailDto>.Ok(dto);
        }
        catch (Exception ex)
        {
            return ApiResponse<VehicleDetailDto>.Error(ex.Message);
        }
    }

    /// <summary>
    /// POST /api/vehicles - Register new vehicle
    /// </summary>
    public async Task<ApiResponse<VehicleDto>> RegisterVehicleAsync(VehicleRegistrationRequest request)
    {
        try
        {
            var registration = new VehicleRegistration
            {
                Callsign = request.Callsign,
                SerialNumber = request.SerialNumber,
                Model = request.Model,
                Manufacturer = request.Manufacturer,
                FaaRegistrationNumber = request.FaaRegistrationNumber,
                Specs = new VehicleSpecs
                {
                    Type = request.Type,
                    MaxTakeoffWeightKg = request.MaxTakeoffWeightKg,
                    MaxFlightTimeMinutes = request.MaxFlightTimeMinutes,
                    MaxSpeedMs = request.MaxSpeedMs,
                    MaxAltitudeM = request.MaxAltitudeM
                }
            };

            var result = await _fleetManager.RegisterVehicleAsync(registration);

            if (!result.Success)
                return ApiResponse<VehicleDto>.BadRequest(string.Join(", ", result.Errors));

            return ApiResponse<VehicleDto>.Created(MapToDto(result.Vehicle!));
        }
        catch (Exception ex)
        {
            return ApiResponse<VehicleDto>.Error(ex.Message);
        }
    }

    /// <summary>
    /// GET /api/vehicles/{id}/telemetry - Get live telemetry
    /// </summary>
    public ApiResponse<TelemetryDto> GetVehicleTelemetry(string vehicleId)
    {
        try
        {
            var state = _fleetManager.GetLiveState(vehicleId);

            if (state == null)
                return ApiResponse<TelemetryDto>.NotFound("Vehicle not connected");

            var dto = new TelemetryDto
            {
                VehicleId = vehicleId,
                Timestamp = state.LastUpdate,
                Position = new PositionDto
                {
                    Latitude = state.Position[0],
                    Longitude = state.Position[1],
                    Altitude = state.Position[2]
                },
                Velocity = new VelocityDto
                {
                    North = state.Velocity[0],
                    East = state.Velocity[1],
                    Down = state.Velocity[2],
                    GroundSpeed = System.Math.Sqrt(state.Velocity[0] * state.Velocity[0] + state.Velocity[1] * state.Velocity[1])
                },
                Attitude = new AttitudeDto
                {
                    Roll = state.Attitude[0] * 57.3,
                    Pitch = state.Attitude[1] * 57.3,
                    Yaw = state.Attitude[2] * 57.3
                },
                Battery = new BatteryDto
                {
                    Percent = state.BatteryPercent,
                    Voltage = state.BatteryVoltage
                },
                Gps = new GpsDto
                {
                    FixType = state.GpsFixType,
                    Satellites = state.SatelliteCount
                },
                Status = state.Status.ToString(),
                SignalStrength = state.SignalStrength
            };

            return ApiResponse<TelemetryDto>.Ok(dto);
        }
        catch (Exception ex)
        {
            return ApiResponse<TelemetryDto>.Error(ex.Message);
        }
    }

    #endregion

    #region Mission Endpoints

    /// <summary>
    /// GET /api/missions - List missions
    /// </summary>
    public async Task<ApiResponse<List<MissionDto>>> GetMissionsAsync(MissionFilter? filter = null)
    {
        try
        {
            var missions = await _fleetManager.GetMissionHistoryAsync(
                filter?.VehicleId,
                filter?.FromDate,
                filter?.ToDate,
                filter?.Limit ?? 100
            );

            var dtos = missions.Select(MapToDto).ToList();

            return ApiResponse<List<MissionDto>>.Ok(dtos);
        }
        catch (Exception ex)
        {
            return ApiResponse<List<MissionDto>>.Error(ex.Message);
        }
    }

    /// <summary>
    /// POST /api/missions - Create new mission
    /// </summary>
    public async Task<ApiResponse<MissionDto>> CreateMissionAsync(CreateMissionRequest request)
    {
        try
        {
            var missionRequest = new MissionRequest
            {
                Name = request.Name,
                Description = request.Description,
                Type = request.Type,
                Priority = request.Priority,
                VehicleId = request.VehicleId,
                ScheduledStart = request.ScheduledStart,
                OperatorId = request.OperatorId,
                Waypoints = request.Waypoints.Select(w => new MissionWaypoint
                {
                    Index = w.Index,
                    Name = w.Name,
                    Position = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(
                        new[] { w.Latitude, w.Longitude, w.Altitude }),
                    Speed = w.Speed,
                    AcceptanceRadius = w.AcceptanceRadius,
                    HoldTime = w.HoldTimeSeconds.HasValue ? TimeSpan.FromSeconds(w.HoldTimeSeconds.Value) : null,
                    Action = w.Action
                }).ToList()
            };

            var result = await _fleetManager.CreateMissionAsync(missionRequest);

            if (!result.Success)
                return ApiResponse<MissionDto>.BadRequest(string.Join(", ", result.Errors));

            return ApiResponse<MissionDto>.Created(MapToDto(result.Mission!));
        }
        catch (Exception ex)
        {
            return ApiResponse<MissionDto>.Error(ex.Message);
        }
    }

    /// <summary>
    /// POST /api/missions/{id}/start - Start a mission
    /// </summary>
    public async Task<ApiResponse<MissionStatusDto>> StartMissionAsync(string missionId, string operatorId)
    {
        try
        {
            var success = await _fleetManager.StartMissionAsync(missionId, operatorId);

            if (!success)
                return ApiResponse<MissionStatusDto>.BadRequest("Failed to start mission");

            return ApiResponse<MissionStatusDto>.Ok(new MissionStatusDto
            {
                MissionId = missionId,
                Status = "InProgress",
                StartedAt = DateTime.UtcNow
            });
        }
        catch (UnauthorizedAccessException ex)
        {
            return ApiResponse<MissionStatusDto>.Unauthorized(ex.Message);
        }
        catch (Exception ex)
        {
            return ApiResponse<MissionStatusDto>.Error(ex.Message);
        }
    }

    /// <summary>
    /// POST /api/missions/{id}/abort - Abort a mission
    /// </summary>
    public async Task<ApiResponse<MissionStatusDto>> AbortMissionAsync(string missionId, string operatorId, string reason)
    {
        try
        {
            var success = await _fleetManager.AbortMissionAsync(missionId, operatorId, reason);

            if (!success)
                return ApiResponse<MissionStatusDto>.BadRequest("Failed to abort mission");

            return ApiResponse<MissionStatusDto>.Ok(new MissionStatusDto
            {
                MissionId = missionId,
                Status = "Aborted",
                AbortedAt = DateTime.UtcNow,
                AbortReason = reason
            });
        }
        catch (Exception ex)
        {
            return ApiResponse<MissionStatusDto>.Error(ex.Message);
        }
    }

    #endregion

    #region Fleet Analytics Endpoints

    /// <summary>
    /// GET /api/fleet/metrics - Get real-time fleet metrics
    /// </summary>
    public ApiResponse<FleetMetricsDto> GetFleetMetrics()
    {
        try
        {
            var metrics = _fleetManager.GetFleetMetrics();
            var dto = new FleetMetricsDto
            {
                Timestamp = metrics.Timestamp,
                TotalVehicles = metrics.TotalVehicles,
                VehiclesOnline = metrics.VehiclesOnline,
                VehiclesFlying = metrics.VehiclesFlying,
                VehiclesReady = metrics.VehiclesReady,
                VehiclesInMaintenance = metrics.VehiclesInMaintenance,
                ActiveMissions = metrics.ActiveMissions,
                AverageBattery = metrics.AverageBattery,
                AverageSignalStrength = metrics.AverageSignalStrength
            };

            return ApiResponse<FleetMetricsDto>.Ok(dto);
        }
        catch (Exception ex)
        {
            return ApiResponse<FleetMetricsDto>.Error(ex.Message);
        }
    }

    /// <summary>
    /// GET /api/fleet/analytics - Get analytics report
    /// </summary>
    public async Task<ApiResponse<AnalyticsReportDto>> GetAnalyticsAsync(DateTime fromDate, DateTime toDate, string? vehicleId = null)
    {
        try
        {
            var report = await _fleetManager.GetAnalyticsReportAsync(fromDate, toDate, vehicleId);

            var dto = new AnalyticsReportDto
            {
                FromDate = report.FromDate,
                ToDate = report.ToDate,
                TotalFlights = report.TotalFlights,
                TotalFlightHours = report.TotalFlightHours,
                TotalDistanceKm = report.TotalDistanceKm,
                MissionsCompleted = report.MissionsCompleted,
                MissionsAborted = report.MissionsAborted,
                MissionSuccessRate = report.MissionSuccessRate,
                AlertsGenerated = report.AlertsGenerated,
                CriticalAlerts = report.CriticalAlerts,
                UptimePercent = report.UptimePercent,
                EstimatedOperatingCost = report.EstimatedOperatingCost,
                CostPerFlightHour = report.CostPerFlightHour
            };

            return ApiResponse<AnalyticsReportDto>.Ok(dto);
        }
        catch (Exception ex)
        {
            return ApiResponse<AnalyticsReportDto>.Error(ex.Message);
        }
    }

    /// <summary>
    /// GET /api/fleet/status - Get fleet overview
    /// </summary>
    public ApiResponse<FleetStatusDto> GetFleetStatus()
    {
        try
        {
            var states = _fleetManager.GetAllLiveStates();
            var metrics = _fleetManager.GetFleetMetrics();

            var dto = new FleetStatusDto
            {
                Timestamp = DateTime.UtcNow,
                Summary = new FleetSummaryDto
                {
                    Total = metrics.TotalVehicles,
                    Online = metrics.VehiclesOnline,
                    Flying = metrics.VehiclesFlying,
                    Ready = metrics.VehiclesReady,
                    Maintenance = metrics.VehiclesInMaintenance,
                    Offline = metrics.TotalVehicles - metrics.VehiclesOnline
                },
                Vehicles = states.Select(kv => new VehicleStatusDto
                {
                    VehicleId = kv.Key,
                    Status = kv.Value.Status.ToString(),
                    LastUpdate = kv.Value.LastUpdate,
                    BatteryPercent = kv.Value.BatteryPercent,
                    Position = kv.Value.Position != null ? new PositionDto
                    {
                        Latitude = kv.Value.Position[0],
                        Longitude = kv.Value.Position[1],
                        Altitude = kv.Value.Position[2]
                    } : null,
                    ActiveMissionId = kv.Value.ActiveMissionId
                }).ToList()
            };

            return ApiResponse<FleetStatusDto>.Ok(dto);
        }
        catch (Exception ex)
        {
            return ApiResponse<FleetStatusDto>.Error(ex.Message);
        }
    }

    #endregion

    #region Command Endpoints

    /// <summary>
    /// POST /api/vehicles/{id}/command - Send command to vehicle
    /// </summary>
    public ApiResponse<CommandResultDto> SendCommand(string vehicleId, VehicleCommandRequest request)
    {
        try
        {
            // Validate vehicle exists and is connected
            var state = _fleetManager.GetLiveState(vehicleId);
            if (state == null)
                return ApiResponse<CommandResultDto>.NotFound("Vehicle not connected");

            // In production, this would send to the vehicle via telemetry link
            var result = new CommandResultDto
            {
                CommandId = Guid.NewGuid().ToString(),
                VehicleId = vehicleId,
                CommandType = request.CommandType,
                Status = "Sent",
                SentAt = DateTime.UtcNow
            };

            return ApiResponse<CommandResultDto>.Ok(result);
        }
        catch (Exception ex)
        {
            return ApiResponse<CommandResultDto>.Error(ex.Message);
        }
    }

    #endregion

    #region Mapping Helpers

    private VehicleDto MapToDto(Vehicle v)
    {
        return new VehicleDto
        {
            Id = v.Id,
            Callsign = v.Callsign,
            SerialNumber = v.SerialNumber,
            Model = v.Model,
            Manufacturer = v.Manufacturer,
            Status = v.Status.ToString(),
            TotalFlightHours = v.TotalFlightHours,
            TotalFlights = v.TotalFlights,
            LastFlightAt = v.LastFlightAt
        };
    }

    private VehicleDetailDto MapToDetailDto(Vehicle v, VehicleState? state)
    {
        return new VehicleDetailDto
        {
            Id = v.Id,
            Callsign = v.Callsign,
            SerialNumber = v.SerialNumber,
            Model = v.Model,
            Manufacturer = v.Manufacturer,
            Status = v.Status.ToString(),
            Specs = new VehicleSpecsDto
            {
                Type = v.Specs.Type.ToString(),
                MaxTakeoffWeightKg = v.Specs.MaxTakeoffWeightKg,
                MaxFlightTimeMinutes = v.Specs.MaxFlightTimeMinutes,
                MaxSpeedMs = v.Specs.MaxSpeedMs,
                MaxAltitudeM = v.Specs.MaxAltitudeM
            },
            Statistics = new VehicleStatsDto
            {
                TotalFlightHours = v.TotalFlightHours,
                TotalFlights = v.TotalFlights,
                TotalDistanceKm = v.TotalDistanceKm,
                FlightHoursSinceLastMaintenance = v.FlightHoursSinceLastMaintenance
            },
            Registration = new RegistrationDto
            {
                FaaNumber = v.FaaRegistrationNumber,
                ExpiresAt = v.FaaRegistrationExpiry
            },
            IsOnline = state != null,
            CurrentBattery = state?.BatteryPercent,
            FirmwareVersion = v.FirmwareVersion
        };
    }

    private MissionDto MapToDto(FleetMission m)
    {
        return new MissionDto
        {
            Id = m.Id,
            Name = m.Name,
            Type = m.Type.ToString(),
            Priority = m.Priority.ToString(),
            Status = m.Status.ToString(),
            AssignedVehicleId = m.AssignedVehicleId,
            CreatedAt = m.CreatedAt,
            ScheduledStart = m.ScheduledStart,
            StartedAt = m.StartedAt,
            CompletedAt = m.CompletedAt,
            WaypointCount = m.Waypoints.Count,
            EstimatedDistanceKm = m.EstimatedDistanceKm
        };
    }

    #endregion
}

#region API Response Types

public class ApiResponse<T>
{
    public bool Success { get; set; }
    public int StatusCode { get; set; }
    public T? Data { get; set; }
    public string? ErrorMessage { get; set; }
    public DateTime Timestamp { get; set; } = DateTime.UtcNow;

    public static ApiResponse<T> Ok(T data) => new() { Success = true, StatusCode = 200, Data = data };
    public static ApiResponse<T> Created(T data) => new() { Success = true, StatusCode = 201, Data = data };
    public static ApiResponse<T> BadRequest(string error) => new() { Success = false, StatusCode = 400, ErrorMessage = error };
    public static ApiResponse<T> Unauthorized(string error) => new() { Success = false, StatusCode = 401, ErrorMessage = error };
    public static ApiResponse<T> NotFound(string error) => new() { Success = false, StatusCode = 404, ErrorMessage = error };
    public static ApiResponse<T> Error(string error) => new() { Success = false, StatusCode = 500, ErrorMessage = error };
}

#endregion

#region Request DTOs

public class VehicleFilter
{
    public VehicleStatus? Status { get; set; }
    public string? Model { get; set; }
}

public class VehicleRegistrationRequest
{
    public string Callsign { get; set; } = string.Empty;
    public string SerialNumber { get; set; } = string.Empty;
    public string Model { get; set; } = string.Empty;
    public string Manufacturer { get; set; } = string.Empty;
    public string? FaaRegistrationNumber { get; set; }
    public VehicleType Type { get; set; }
    public double MaxTakeoffWeightKg { get; set; }
    public double MaxFlightTimeMinutes { get; set; }
    public double MaxSpeedMs { get; set; }
    public double MaxAltitudeM { get; set; }
}

public class MissionFilter
{
    public string? VehicleId { get; set; }
    public DateTime? FromDate { get; set; }
    public DateTime? ToDate { get; set; }
    public int? Limit { get; set; }
}

public class CreateMissionRequest
{
    public string Name { get; set; } = string.Empty;
    public string? Description { get; set; }
    public MissionType Type { get; set; }
    public MissionPriority Priority { get; set; }
    public string? VehicleId { get; set; }
    public DateTime? ScheduledStart { get; set; }
    public string OperatorId { get; set; } = string.Empty;
    public List<WaypointRequest> Waypoints { get; set; } = new();
}

public class WaypointRequest
{
    public int Index { get; set; }
    public string? Name { get; set; }
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }
    public double Speed { get; set; } = 5.0;
    public double AcceptanceRadius { get; set; } = 2.0;
    public int? HoldTimeSeconds { get; set; }
    public WaypointAction Action { get; set; }
}

public class VehicleCommandRequest
{
    public string CommandType { get; set; } = string.Empty;
    public Dictionary<string, object>? Parameters { get; set; }
}

#endregion

#region Response DTOs

public class VehicleDto
{
    public string Id { get; set; } = string.Empty;
    public string Callsign { get; set; } = string.Empty;
    public string SerialNumber { get; set; } = string.Empty;
    public string Model { get; set; } = string.Empty;
    public string Manufacturer { get; set; } = string.Empty;
    public string Status { get; set; } = string.Empty;
    public double TotalFlightHours { get; set; }
    public int TotalFlights { get; set; }
    public DateTime? LastFlightAt { get; set; }
}

public class VehicleDetailDto : VehicleDto
{
    public VehicleSpecsDto Specs { get; set; } = new();
    public VehicleStatsDto Statistics { get; set; } = new();
    public RegistrationDto Registration { get; set; } = new();
    public bool IsOnline { get; set; }
    public double? CurrentBattery { get; set; }
    public string? FirmwareVersion { get; set; }
}

public class VehicleSpecsDto
{
    public string Type { get; set; } = string.Empty;
    public double MaxTakeoffWeightKg { get; set; }
    public double MaxFlightTimeMinutes { get; set; }
    public double MaxSpeedMs { get; set; }
    public double MaxAltitudeM { get; set; }
}

public class VehicleStatsDto
{
    public double TotalFlightHours { get; set; }
    public int TotalFlights { get; set; }
    public double TotalDistanceKm { get; set; }
    public double FlightHoursSinceLastMaintenance { get; set; }
}

public class RegistrationDto
{
    public string? FaaNumber { get; set; }
    public DateTime? ExpiresAt { get; set; }
}

public class TelemetryDto
{
    public string VehicleId { get; set; } = string.Empty;
    public DateTime Timestamp { get; set; }
    public PositionDto Position { get; set; } = new();
    public VelocityDto Velocity { get; set; } = new();
    public AttitudeDto Attitude { get; set; } = new();
    public BatteryDto Battery { get; set; } = new();
    public GpsDto Gps { get; set; } = new();
    public string Status { get; set; } = string.Empty;
    public double SignalStrength { get; set; }
}

public class PositionDto
{
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }
}

public class VelocityDto
{
    public double North { get; set; }
    public double East { get; set; }
    public double Down { get; set; }
    public double GroundSpeed { get; set; }
}

public class AttitudeDto
{
    public double Roll { get; set; }
    public double Pitch { get; set; }
    public double Yaw { get; set; }
}

public class BatteryDto
{
    public double Percent { get; set; }
    public double Voltage { get; set; }
}

public class GpsDto
{
    public int FixType { get; set; }
    public int Satellites { get; set; }
}

public class MissionDto
{
    public string Id { get; set; } = string.Empty;
    public string Name { get; set; } = string.Empty;
    public string Type { get; set; } = string.Empty;
    public string Priority { get; set; } = string.Empty;
    public string Status { get; set; } = string.Empty;
    public string? AssignedVehicleId { get; set; }
    public DateTime CreatedAt { get; set; }
    public DateTime? ScheduledStart { get; set; }
    public DateTime? StartedAt { get; set; }
    public DateTime? CompletedAt { get; set; }
    public int WaypointCount { get; set; }
    public double EstimatedDistanceKm { get; set; }
}

public class MissionStatusDto
{
    public string MissionId { get; set; } = string.Empty;
    public string Status { get; set; } = string.Empty;
    public DateTime? StartedAt { get; set; }
    public DateTime? AbortedAt { get; set; }
    public string? AbortReason { get; set; }
}

public class FleetMetricsDto
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

public class AnalyticsReportDto
{
    public DateTime FromDate { get; set; }
    public DateTime ToDate { get; set; }
    public int TotalFlights { get; set; }
    public double TotalFlightHours { get; set; }
    public double TotalDistanceKm { get; set; }
    public int MissionsCompleted { get; set; }
    public int MissionsAborted { get; set; }
    public double MissionSuccessRate { get; set; }
    public int AlertsGenerated { get; set; }
    public int CriticalAlerts { get; set; }
    public double UptimePercent { get; set; }
    public double EstimatedOperatingCost { get; set; }
    public double CostPerFlightHour { get; set; }
}

public class FleetStatusDto
{
    public DateTime Timestamp { get; set; }
    public FleetSummaryDto Summary { get; set; } = new();
    public List<VehicleStatusDto> Vehicles { get; set; } = new();
}

public class FleetSummaryDto
{
    public int Total { get; set; }
    public int Online { get; set; }
    public int Flying { get; set; }
    public int Ready { get; set; }
    public int Maintenance { get; set; }
    public int Offline { get; set; }
}

public class VehicleStatusDto
{
    public string VehicleId { get; set; } = string.Empty;
    public string Status { get; set; } = string.Empty;
    public DateTime LastUpdate { get; set; }
    public double BatteryPercent { get; set; }
    public PositionDto? Position { get; set; }
    public string? ActiveMissionId { get; set; }
}

public class CommandResultDto
{
    public string CommandId { get; set; } = string.Empty;
    public string VehicleId { get; set; } = string.Empty;
    public string CommandType { get; set; } = string.Empty;
    public string Status { get; set; } = string.Empty;
    public DateTime SentAt { get; set; }
}

#endregion
