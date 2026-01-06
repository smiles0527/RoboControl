using System.Collections.Concurrent;
using System.Text.Json;

namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Fleet Database - Persistent storage for fleet management data.
/// Uses JSON file storage by default, can be extended to SQL/NoSQL backends.
/// </summary>
public class FleetDatabase : IDisposable
{
    private readonly string _basePath;
    private readonly ConcurrentDictionary<string, Vehicle> _vehicles;
    private readonly ConcurrentDictionary<string, Mission> _missions;
    private readonly ConcurrentDictionary<string, MaintenanceTicket> _maintenanceTickets;
    private readonly ConcurrentDictionary<string, VehicleAlert> _alerts;
    private readonly ConcurrentDictionary<string, ComplianceEvent> _complianceEvents;
    private readonly ConcurrentDictionary<string, Operator> _operators;
    private readonly List<TelemetryRecord> _telemetryBuffer;
    private readonly List<FleetMetrics> _metricsHistory;
    private readonly object _telemetryLock = new();
    private readonly Timer _persistTimer;
    private bool _isDirty;
    private bool _isDisposed;

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        WriteIndented = true,
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase
    };

    public FleetDatabase(string basePath)
    {
        _basePath = basePath;
        _vehicles = new ConcurrentDictionary<string, Vehicle>();
        _missions = new ConcurrentDictionary<string, Mission>();
        _maintenanceTickets = new ConcurrentDictionary<string, MaintenanceTicket>();
        _alerts = new ConcurrentDictionary<string, VehicleAlert>();
        _complianceEvents = new ConcurrentDictionary<string, ComplianceEvent>();
        _operators = new ConcurrentDictionary<string, Operator>();
        _telemetryBuffer = new List<TelemetryRecord>();
        _metricsHistory = new List<FleetMetrics>();

        EnsureDirectoryExists();
        LoadData();

        _persistTimer = new Timer(PersistIfDirty, null,
            TimeSpan.FromSeconds(30), TimeSpan.FromSeconds(30));
    }

    #region Vehicle Operations

    public Task SaveVehicleAsync(Vehicle vehicle)
    {
        _vehicles[vehicle.Id] = vehicle;
        _isDirty = true;
        return Task.CompletedTask;
    }

    public Task<Vehicle?> GetVehicleAsync(string vehicleId)
    {
        return Task.FromResult(_vehicles.TryGetValue(vehicleId, out var vehicle) ? vehicle : null);
    }

    public Task<IReadOnlyList<Vehicle>> GetAllVehiclesAsync()
    {
        return Task.FromResult<IReadOnlyList<Vehicle>>(_vehicles.Values.ToList());
    }

    public Task DeleteVehicleAsync(string vehicleId)
    {
        _vehicles.TryRemove(vehicleId, out _);
        _isDirty = true;
        return Task.CompletedTask;
    }

    #endregion

    #region Mission Operations

    public Task SaveMissionAsync(Mission mission)
    {
        _missions[mission.Id] = mission;
        _isDirty = true;
        return Task.CompletedTask;
    }

    public Task<Mission?> GetMissionAsync(string missionId)
    {
        return Task.FromResult(_missions.TryGetValue(missionId, out var mission) ? mission : null);
    }

    public Task<IReadOnlyList<Mission>> GetMissionsAsync(
        string? vehicleId = null,
        DateTime? fromDate = null,
        DateTime? toDate = null,
        int limit = 100)
    {
        var query = _missions.Values.AsEnumerable();

        if (!string.IsNullOrEmpty(vehicleId))
            query = query.Where(m => m.AssignedVehicleId == vehicleId);

        if (fromDate.HasValue)
            query = query.Where(m => m.CreatedAt >= fromDate.Value);

        if (toDate.HasValue)
            query = query.Where(m => m.CreatedAt <= toDate.Value);

        var result = query
            .OrderByDescending(m => m.CreatedAt)
            .Take(limit)
            .ToList();

        return Task.FromResult<IReadOnlyList<Mission>>(result);
    }

    public Task<IReadOnlyList<Mission>> GetMissionsByStatusAsync(MissionStatus status)
    {
        var missions = _missions.Values.Where(m => m.Status == status).ToList();
        return Task.FromResult<IReadOnlyList<Mission>>(missions);
    }

    #endregion

    #region Maintenance Operations

    public Task SaveMaintenanceTicketAsync(MaintenanceTicket ticket)
    {
        _maintenanceTickets[ticket.Id] = ticket;
        _isDirty = true;
        return Task.CompletedTask;
    }

    public Task<MaintenanceTicket?> GetMaintenanceTicketAsync(string ticketId)
    {
        return Task.FromResult(_maintenanceTickets.TryGetValue(ticketId, out var ticket) ? ticket : null);
    }

    public Task<IReadOnlyList<MaintenanceTicket>> GetMaintenanceTicketsAsync(
        string? vehicleId = null,
        MaintenanceStatus? status = null)
    {
        var query = _maintenanceTickets.Values.AsEnumerable();

        if (!string.IsNullOrEmpty(vehicleId))
            query = query.Where(t => t.VehicleId == vehicleId);

        if (status.HasValue)
            query = query.Where(t => t.Status == status.Value);

        return Task.FromResult<IReadOnlyList<MaintenanceTicket>>(query.ToList());
    }

    #endregion

    #region Alert Operations

    public Task SaveAlertAsync(VehicleAlert alert)
    {
        _alerts[alert.Id] = alert;
        _isDirty = true;
        return Task.CompletedTask;
    }

    public Task<IReadOnlyList<VehicleAlert>> GetAlertsAsync(
        string? vehicleId = null,
        bool? acknowledged = null,
        DateTime? fromDate = null,
        int limit = 100)
    {
        var query = _alerts.Values.AsEnumerable();

        if (!string.IsNullOrEmpty(vehicleId))
            query = query.Where(a => a.VehicleId == vehicleId);

        if (acknowledged.HasValue)
            query = query.Where(a => a.Acknowledged == acknowledged.Value);

        if (fromDate.HasValue)
            query = query.Where(a => a.Timestamp >= fromDate.Value);

        var result = query
            .OrderByDescending(a => a.Timestamp)
            .Take(limit)
            .ToList();

        return Task.FromResult<IReadOnlyList<VehicleAlert>>(result);
    }

    #endregion

    #region Compliance Operations

    public Task SaveComplianceEventAsync(ComplianceEvent evt)
    {
        _complianceEvents[evt.Id] = evt;
        _isDirty = true;
        return Task.CompletedTask;
    }

    public Task<IReadOnlyList<ComplianceEvent>> GetComplianceEventsAsync(
        string? vehicleId = null,
        string? missionId = null,
        ComplianceEventType? type = null,
        DateTime? fromDate = null,
        DateTime? toDate = null,
        int limit = 1000)
    {
        var query = _complianceEvents.Values.AsEnumerable();

        if (!string.IsNullOrEmpty(vehicleId))
            query = query.Where(e => e.VehicleId == vehicleId);

        if (!string.IsNullOrEmpty(missionId))
            query = query.Where(e => e.MissionId == missionId);

        if (type.HasValue)
            query = query.Where(e => e.Type == type.Value);

        if (fromDate.HasValue)
            query = query.Where(e => e.Timestamp >= fromDate.Value);

        if (toDate.HasValue)
            query = query.Where(e => e.Timestamp <= toDate.Value);

        var result = query
            .OrderByDescending(e => e.Timestamp)
            .Take(limit)
            .ToList();

        return Task.FromResult<IReadOnlyList<ComplianceEvent>>(result);
    }

    #endregion

    #region Operator Operations

    public Task SaveOperatorAsync(Operator op)
    {
        _operators[op.Id] = op;
        _isDirty = true;
        return Task.CompletedTask;
    }

    public Task<Operator?> GetOperatorAsync(string operatorId)
    {
        return Task.FromResult(_operators.TryGetValue(operatorId, out var op) ? op : null);
    }

    public Task<IReadOnlyList<Operator>> GetAllOperatorsAsync()
    {
        return Task.FromResult<IReadOnlyList<Operator>>(_operators.Values.ToList());
    }

    #endregion

    #region Telemetry Operations

    public Task StoreTelemetryAsync(string vehicleId, TelemetryFrame frame)
    {
        lock (_telemetryLock)
        {
            _telemetryBuffer.Add(new TelemetryRecord
            {
                VehicleId = vehicleId,
                Timestamp = frame.Timestamp,
                Position = new double[] { frame.Position[0], frame.Position[1], frame.Position[2] },
                Velocity = new double[] { frame.Velocity[0], frame.Velocity[1], frame.Velocity[2] },
                Attitude = new double[] { frame.Attitude[0], frame.Attitude[1], frame.Attitude[2] },
                BatteryPercent = frame.BatteryPercent,
                BatteryVoltage = frame.BatteryVoltage,
                GpsFixType = frame.GpsFixType,
                SatelliteCount = frame.SatelliteCount,
                SignalStrength = frame.SignalStrength
            });

            // Keep buffer size reasonable
            if (_telemetryBuffer.Count > 100000)
            {
                FlushTelemetryBuffer();
            }
        }

        return Task.CompletedTask;
    }

    public Task<IReadOnlyList<TelemetryRecord>> GetTelemetryAsync(
        string vehicleId,
        DateTime fromTime,
        DateTime toTime,
        int maxRecords = 10000)
    {
        lock (_telemetryLock)
        {
            var records = _telemetryBuffer
                .Where(t => t.VehicleId == vehicleId && t.Timestamp >= fromTime && t.Timestamp <= toTime)
                .Take(maxRecords)
                .ToList();

            return Task.FromResult<IReadOnlyList<TelemetryRecord>>(records);
        }
    }

    private void FlushTelemetryBuffer()
    {
        // In production, this would write to a time-series database
        // For now, just trim old data
        var cutoff = DateTime.UtcNow.AddHours(-24);
        _telemetryBuffer.RemoveAll(t => t.Timestamp < cutoff);
    }

    #endregion

    #region Metrics Operations

    public Task StoreFleetMetricsAsync(FleetMetrics metrics)
    {
        _metricsHistory.Add(metrics);

        // Keep last 24 hours of metrics
        var cutoff = DateTime.UtcNow.AddHours(-24);
        _metricsHistory.RemoveAll(m => m.Timestamp < cutoff);

        return Task.CompletedTask;
    }

    public Task<IReadOnlyList<FleetMetrics>> GetMetricsHistoryAsync(DateTime fromTime, DateTime toTime)
    {
        var metrics = _metricsHistory
            .Where(m => m.Timestamp >= fromTime && m.Timestamp <= toTime)
            .OrderBy(m => m.Timestamp)
            .ToList();

        return Task.FromResult<IReadOnlyList<FleetMetrics>>(metrics);
    }

    #endregion

    #region Persistence

    private void EnsureDirectoryExists()
    {
        var dir = Path.GetDirectoryName(_basePath);
        if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
        {
            Directory.CreateDirectory(dir);
        }
    }

    private void LoadData()
    {
        try
        {
            var dataFile = Path.Combine(_basePath, "fleet_data.json");
            if (File.Exists(dataFile))
            {
                var json = File.ReadAllText(dataFile);
                var data = JsonSerializer.Deserialize<FleetData>(json, JsonOptions);

                if (data != null)
                {
                    foreach (var v in data.Vehicles ?? new())
                        _vehicles[v.Id] = v;
                    foreach (var m in data.Missions ?? new())
                        _missions[m.Id] = m;
                    foreach (var t in data.MaintenanceTickets ?? new())
                        _maintenanceTickets[t.Id] = t;
                    foreach (var a in data.Alerts ?? new())
                        _alerts[a.Id] = a;
                    foreach (var e in data.ComplianceEvents ?? new())
                        _complianceEvents[e.Id] = e;
                    foreach (var o in data.Operators ?? new())
                        _operators[o.Id] = o;
                }
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Warning: Failed to load fleet data: {ex.Message}");
        }
    }

    private void PersistIfDirty(object? state)
    {
        if (_isDirty && !_isDisposed)
        {
            PersistData();
        }
    }

    private void PersistData()
    {
        try
        {
            var data = new FleetData
            {
                Vehicles = _vehicles.Values.ToList(),
                Missions = _missions.Values.ToList(),
                MaintenanceTickets = _maintenanceTickets.Values.ToList(),
                Alerts = _alerts.Values.Take(10000).ToList(), // Limit stored alerts
                ComplianceEvents = _complianceEvents.Values.Take(50000).ToList(),
                Operators = _operators.Values.ToList(),
                LastUpdated = DateTime.UtcNow
            };

            var json = JsonSerializer.Serialize(data, JsonOptions);
            var dataFile = Path.Combine(_basePath, "fleet_data.json");

            // Write to temp file first, then rename for atomic operation
            var tempFile = dataFile + ".tmp";
            File.WriteAllText(tempFile, json);
            File.Move(tempFile, dataFile, overwrite: true);

            _isDirty = false;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Warning: Failed to persist fleet data: {ex.Message}");
        }
    }

    public void Dispose()
    {
        if (_isDisposed) return;
        _isDisposed = true;

        _persistTimer.Dispose();
        PersistData();
    }

    #endregion
}

#region Data Transfer Objects

internal class FleetData
{
    public List<Vehicle> Vehicles { get; set; } = new();
    public List<Mission> Missions { get; set; } = new();
    public List<MaintenanceTicket> MaintenanceTickets { get; set; } = new();
    public List<VehicleAlert> Alerts { get; set; } = new();
    public List<ComplianceEvent> ComplianceEvents { get; set; } = new();
    public List<Operator> Operators { get; set; } = new();
    public DateTime LastUpdated { get; set; }
}

public class TelemetryRecord
{
    public string VehicleId { get; set; } = string.Empty;
    public DateTime Timestamp { get; set; }
    public double[] Position { get; set; } = Array.Empty<double>();
    public double[] Velocity { get; set; } = Array.Empty<double>();
    public double[] Attitude { get; set; } = Array.Empty<double>();
    public double BatteryPercent { get; set; }
    public double BatteryVoltage { get; set; }
    public int GpsFixType { get; set; }
    public int SatelliteCount { get; set; }
    public double SignalStrength { get; set; }
}

#endregion
