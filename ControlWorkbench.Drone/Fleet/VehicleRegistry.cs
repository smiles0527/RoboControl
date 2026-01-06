using System.Collections.Concurrent;

namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Vehicle Registry - Central repository for all fleet vehicle information.
/// Provides fast lookup, filtering, and state management for vehicle assets.
/// </summary>
public class VehicleRegistry
{
    private readonly FleetDatabase _database;
    private readonly ConcurrentDictionary<string, Vehicle> _vehicleCache;
    private readonly ConcurrentDictionary<string, string> _callsignIndex;
    private readonly ConcurrentDictionary<string, string> _serialIndex;
    private DateTime _lastCacheRefresh;
    private readonly TimeSpan _cacheExpiry = TimeSpan.FromMinutes(5);

    public VehicleRegistry(FleetDatabase database)
    {
        _database = database;
        _vehicleCache = new ConcurrentDictionary<string, Vehicle>();
        _callsignIndex = new ConcurrentDictionary<string, string>();
        _serialIndex = new ConcurrentDictionary<string, string>();
        _lastCacheRefresh = DateTime.MinValue;
    }

    /// <summary>
    /// Register a new vehicle.
    /// </summary>
    public async Task<Vehicle> RegisterAsync(VehicleRegistration registration)
    {
        // Check for duplicates
        if (_serialIndex.ContainsKey(registration.SerialNumber))
        {
            throw new InvalidOperationException($"Vehicle with serial {registration.SerialNumber} already registered");
        }

        if (_callsignIndex.ContainsKey(registration.Callsign.ToUpperInvariant()))
        {
            throw new InvalidOperationException($"Callsign {registration.Callsign} already in use");
        }

        var vehicle = new Vehicle
        {
            Id = Guid.NewGuid().ToString(),
            Callsign = registration.Callsign.ToUpperInvariant(),
            SerialNumber = registration.SerialNumber,
            Model = registration.Model,
            Manufacturer = registration.Manufacturer,
            Specs = registration.Specs,
            FaaRegistrationNumber = registration.FaaRegistrationNumber,
            RegisteredAt = DateTime.UtcNow,
            Status = VehicleStatus.Offline
        };

        _vehicleCache[vehicle.Id] = vehicle;
        _callsignIndex[vehicle.Callsign] = vehicle.Id;
        _serialIndex[vehicle.SerialNumber] = vehicle.Id;

        return vehicle;
    }

    /// <summary>
    /// Get a vehicle by ID.
    /// </summary>
    public async Task<Vehicle?> GetVehicleAsync(string vehicleId)
    {
        await RefreshCacheIfNeededAsync();

        return _vehicleCache.TryGetValue(vehicleId, out var vehicle) ? vehicle : null;
    }

    /// <summary>
    /// Get a vehicle by callsign.
    /// </summary>
    public async Task<Vehicle?> GetVehicleByCallsignAsync(string callsign)
    {
        await RefreshCacheIfNeededAsync();

        if (_callsignIndex.TryGetValue(callsign.ToUpperInvariant(), out var vehicleId))
        {
            return await GetVehicleAsync(vehicleId);
        }
        return null;
    }

    /// <summary>
    /// Get a vehicle by serial number.
    /// </summary>
    public async Task<Vehicle?> GetVehicleBySerialAsync(string serialNumber)
    {
        await RefreshCacheIfNeededAsync();

        if (_serialIndex.TryGetValue(serialNumber, out var vehicleId))
        {
            return await GetVehicleAsync(vehicleId);
        }
        return null;
    }

    /// <summary>
    /// Get all vehicles in the fleet.
    /// </summary>
    public async Task<IReadOnlyList<Vehicle>> GetAllVehiclesAsync()
    {
        await RefreshCacheIfNeededAsync();
        return _vehicleCache.Values.Where(v => v.Status != VehicleStatus.Decommissioned).ToList();
    }

    /// <summary>
    /// Get vehicles by status.
    /// </summary>
    public async Task<IReadOnlyList<Vehicle>> GetVehiclesByStatusAsync(VehicleStatus status)
    {
        await RefreshCacheIfNeededAsync();
        return _vehicleCache.Values.Where(v => v.Status == status).ToList();
    }

    /// <summary>
    /// Get vehicles that are available for missions.
    /// </summary>
    public async Task<IReadOnlyList<Vehicle>> GetAvailableVehiclesAsync()
    {
        await RefreshCacheIfNeededAsync();
        return _vehicleCache.Values
            .Where(v => v.Status == VehicleStatus.Ready || v.Status == VehicleStatus.Offline)
            .Where(v => !IsMaintenanceDue(v))
            .ToList();
    }

    /// <summary>
    /// Get vehicles that need maintenance.
    /// </summary>
    public async Task<IReadOnlyList<Vehicle>> GetVehiclesNeedingMaintenanceAsync()
    {
        await RefreshCacheIfNeededAsync();
        return _vehicleCache.Values.Where(IsMaintenanceDue).ToList();
    }

    /// <summary>
    /// Update vehicle status.
    /// </summary>
    public async Task UpdateStatusAsync(string vehicleId, VehicleStatus status)
    {
        if (_vehicleCache.TryGetValue(vehicleId, out var vehicle))
        {
            vehicle.Status = status;
            await _database.SaveVehicleAsync(vehicle);
        }
    }

    /// <summary>
    /// Record a completed flight.
    /// </summary>
    public async Task RecordFlightAsync(string vehicleId, double flightHours, double distanceKm)
    {
        if (_vehicleCache.TryGetValue(vehicleId, out var vehicle))
        {
            vehicle.TotalFlights++;
            vehicle.TotalFlightHours += flightHours;
            vehicle.TotalDistanceKm += distanceKm;
            vehicle.FlightsSinceLastMaintenance++;
            vehicle.FlightHoursSinceLastMaintenance += flightHours;
            vehicle.LastFlightAt = DateTime.UtcNow;

            await _database.SaveVehicleAsync(vehicle);
        }
    }

    /// <summary>
    /// Record completed maintenance.
    /// </summary>
    public async Task RecordMaintenanceAsync(string vehicleId)
    {
        if (_vehicleCache.TryGetValue(vehicleId, out var vehicle))
        {
            vehicle.FlightsSinceLastMaintenance = 0;
            vehicle.FlightHoursSinceLastMaintenance = 0;
            vehicle.LastMaintenanceAt = DateTime.UtcNow;
            vehicle.Status = VehicleStatus.Ready;

            await _database.SaveVehicleAsync(vehicle);
        }
    }

    /// <summary>
    /// Update vehicle firmware version.
    /// </summary>
    public async Task UpdateFirmwareVersionAsync(string vehicleId, string version)
    {
        if (_vehicleCache.TryGetValue(vehicleId, out var vehicle))
        {
            vehicle.FirmwareVersion = version;
            vehicle.FirmwareUpdatedAt = DateTime.UtcNow;
            await _database.SaveVehicleAsync(vehicle);
        }
    }

    /// <summary>
    /// Check if a vehicle is due for maintenance.
    /// </summary>
    public bool IsMaintenanceDue(Vehicle vehicle)
    {
        var schedule = vehicle.MaintenanceSchedule;

        if (vehicle.FlightHoursSinceLastMaintenance >= schedule.FlightHoursInterval)
            return true;

        if (vehicle.FlightsSinceLastMaintenance >= schedule.FlightsInterval)
            return true;

        if (vehicle.LastMaintenanceAt.HasValue)
        {
            var daysSinceMaintenance = (DateTime.UtcNow - vehicle.LastMaintenanceAt.Value).TotalDays;
            if (daysSinceMaintenance >= schedule.DaysInterval)
                return true;
        }

        return false;
    }

    /// <summary>
    /// Get fleet statistics.
    /// </summary>
    public async Task<FleetStats> GetFleetStatsAsync()
    {
        await RefreshCacheIfNeededAsync();

        var vehicles = _vehicleCache.Values.Where(v => v.Status != VehicleStatus.Decommissioned).ToList();

        return new FleetStats
        {
            TotalVehicles = vehicles.Count,
            ActiveVehicles = vehicles.Count(v => v.Status != VehicleStatus.Maintenance && v.Status != VehicleStatus.Offline),
            InMaintenance = vehicles.Count(v => v.Status == VehicleStatus.Maintenance),
            NeedingMaintenance = vehicles.Count(IsMaintenanceDue),
            TotalFlightHours = vehicles.Sum(v => v.TotalFlightHours),
            TotalFlights = vehicles.Sum(v => v.TotalFlights),
            TotalDistanceKm = vehicles.Sum(v => v.TotalDistanceKm),
            AverageFlightHoursPerVehicle = vehicles.Count > 0 ? vehicles.Average(v => v.TotalFlightHours) : 0,
            VehiclesByType = vehicles.GroupBy(v => v.Specs.Type).ToDictionary(g => g.Key, g => g.Count()),
            VehiclesByStatus = vehicles.GroupBy(v => v.Status).ToDictionary(g => g.Key, g => g.Count())
        };
    }

    private async Task RefreshCacheIfNeededAsync()
    {
        if (DateTime.UtcNow - _lastCacheRefresh > _cacheExpiry)
        {
            await RefreshCacheAsync();
        }
    }

    private async Task RefreshCacheAsync()
    {
        var vehicles = await _database.GetAllVehiclesAsync();

        _vehicleCache.Clear();
        _callsignIndex.Clear();
        _serialIndex.Clear();

        foreach (var vehicle in vehicles)
        {
            _vehicleCache[vehicle.Id] = vehicle;
            _callsignIndex[vehicle.Callsign] = vehicle.Id;
            _serialIndex[vehicle.SerialNumber] = vehicle.Id;
        }

        _lastCacheRefresh = DateTime.UtcNow;
    }
}

public class FleetStats
{
    public int TotalVehicles { get; set; }
    public int ActiveVehicles { get; set; }
    public int InMaintenance { get; set; }
    public int NeedingMaintenance { get; set; }
    public double TotalFlightHours { get; set; }
    public int TotalFlights { get; set; }
    public double TotalDistanceKm { get; set; }
    public double AverageFlightHoursPerVehicle { get; set; }
    public Dictionary<VehicleType, int> VehiclesByType { get; set; } = new();
    public Dictionary<VehicleStatus, int> VehiclesByStatus { get; set; } = new();
}
