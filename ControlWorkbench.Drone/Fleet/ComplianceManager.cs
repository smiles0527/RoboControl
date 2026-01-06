using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Compliance Manager - Regulatory compliance and audit trail management.
/// Supports Part 107, EASA regulations, and enterprise policy enforcement.
/// </summary>
public class ComplianceManager
{
    private readonly FleetDatabase _database;
    private readonly ComplianceConfiguration _config;
    private readonly List<GeofenceZone> _geofences;

    public event EventHandler<ComplianceViolationEventArgs>? ViolationDetected;

    public ComplianceManager(FleetDatabase database, ComplianceConfiguration config)
    {
        _database = database;
        _config = config;
        _geofences = new List<GeofenceZone>(config.RestrictedZones);
    }

    #region Registration Validation

    /// <summary>
    /// Validate vehicle registration compliance.
    /// </summary>
    public async Task<ValidationResult> ValidateRegistrationAsync(VehicleRegistration registration)
    {
        var errors = new List<string>();

        if (string.IsNullOrWhiteSpace(registration.Callsign))
            errors.Add("Callsign is required");

        if (string.IsNullOrWhiteSpace(registration.SerialNumber))
            errors.Add("Serial number is required");

        if (registration.Specs.MaxTakeoffWeightKg > 25 && string.IsNullOrEmpty(registration.FaaRegistrationNumber))
            errors.Add("FAA registration required for aircraft over 25kg MTOW");

        if (registration.Specs.MaxTakeoffWeightKg > 55)
            errors.Add("Aircraft over 55kg requires special certification");

        return new ValidationResult
        {
            IsValid = errors.Count == 0,
            Errors = errors.ToArray()
        };
    }

    #endregion

    #region Mission Authorization

    /// <summary>
    /// Check if a mission is authorized.
    /// </summary>
    public async Task<AuthorizationResult> CheckMissionAuthorizationAsync(
        string missionId,
        string vehicleId,
        string operatorId)
    {
        var missingRequirements = new List<string>();

        // Check operator certification
        if (_config.RequirePilotCertification)
        {
            var op = await _database.GetOperatorAsync(operatorId);
            if (op == null)
            {
                return new AuthorizationResult
                {
                    IsAuthorized = false,
                    Reason = "Operator not found in system",
                    MissingRequirements = new List<string> { "Valid operator registration" }
                };
            }

            var pilotCert = op.Certifications.FirstOrDefault(c => c.Type == CertificationType.Part107);
            if (pilotCert == null)
            {
                missingRequirements.Add("Part 107 certification");
            }
            else if (pilotCert.ExpiresAt < DateTime.UtcNow)
            {
                missingRequirements.Add("Part 107 certification (expired)");
            }

            // Check operator is authorized for this vehicle
            if (!op.AuthorizedVehicles.Contains(vehicleId) && !op.AuthorizedVehicles.Contains("*"))
            {
                missingRequirements.Add("Vehicle authorization");
            }
        }

        // Check vehicle compliance
        var vehicle = await _database.GetVehicleAsync(vehicleId);
        if (vehicle != null)
        {
            if (vehicle.FaaRegistrationExpiry.HasValue && vehicle.FaaRegistrationExpiry < DateTime.UtcNow)
            {
                missingRequirements.Add("Valid FAA registration");
            }

            if (vehicle.Status == VehicleStatus.Maintenance)
            {
                missingRequirements.Add("Vehicle cleared from maintenance");
            }
        }

        // Check preflight checklist if required
        if (_config.RequirePreflightChecklist)
        {
            // In production, this would check a completed preflight form
            // For now, we'll assume it's done
        }

        if (missingRequirements.Count > 0)
        {
            return new AuthorizationResult
            {
                IsAuthorized = false,
                Reason = "Missing required authorizations",
                MissingRequirements = missingRequirements
            };
        }

        return new AuthorizationResult { IsAuthorized = true };
    }

    #endregion

    #region Geofencing

    /// <summary>
    /// Check if a position is within allowed airspace.
    /// </summary>
    public Task<GeofenceCheckResult> CheckGeofenceAsync(Vector<double> position)
    {
        if (!_config.EnforceGeofencing)
        {
            return Task.FromResult(new GeofenceCheckResult { IsAllowed = true });
        }

        foreach (var zone in _geofences.Where(z => z.IsActive))
        {
            // Check if within time window
            if (zone.ActiveFrom.HasValue && DateTime.UtcNow < zone.ActiveFrom.Value)
                continue;
            if (zone.ActiveUntil.HasValue && DateTime.UtcNow > zone.ActiveUntil.Value)
                continue;

            // Check altitude restrictions
            if (zone.MaxAltitude.HasValue && position[2] > zone.MaxAltitude.Value)
            {
                if (zone.Type == GeofenceType.AltitudeRestriction)
                {
                    return Task.FromResult(new GeofenceCheckResult
                    {
                        IsAllowed = false,
                        Reason = $"Altitude {position[2]:F0}m exceeds limit of {zone.MaxAltitude}m",
                        ViolatedZone = zone
                    });
                }
            }

            // Check if point is inside polygon
            if (zone.Vertices.Count >= 3 && IsPointInPolygon(position, zone.Vertices))
            {
                if (zone.Type == GeofenceType.NoFlyZone)
                {
                    return Task.FromResult(new GeofenceCheckResult
                    {
                        IsAllowed = false,
                        Reason = $"No-fly zone: {zone.Name}",
                        ViolatedZone = zone
                    });
                }
                else if (zone.Type == GeofenceType.RestrictedZone)
                {
                    return Task.FromResult(new GeofenceCheckResult
                    {
                        IsAllowed = false,
                        Reason = $"Restricted zone: {zone.Name}. {zone.Reason}",
                        ViolatedZone = zone
                    });
                }
            }
        }

        return Task.FromResult(new GeofenceCheckResult { IsAllowed = true });
    }

    /// <summary>
    /// Add a geofence zone.
    /// </summary>
    public void AddGeofence(GeofenceZone zone)
    {
        _geofences.Add(zone);
    }

    /// <summary>
    /// Remove a geofence zone.
    /// </summary>
    public bool RemoveGeofence(string zoneId)
    {
        var zone = _geofences.FirstOrDefault(z => z.Id == zoneId);
        if (zone != null)
        {
            return _geofences.Remove(zone);
        }
        return false;
    }

    /// <summary>
    /// Get all active geofences.
    /// </summary>
    public IReadOnlyList<GeofenceZone> GetActiveGeofences()
    {
        return _geofences.Where(z => z.IsActive).ToList();
    }

    private bool IsPointInPolygon(Vector<double> point, List<Vector<double>> polygon)
    {
        // Ray casting algorithm for point-in-polygon test
        int n = polygon.Count;
        bool inside = false;

        double x = point[0], y = point[1];

        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            double xi = polygon[i][0], yi = polygon[i][1];
            double xj = polygon[j][0], yj = polygon[j][1];

            if ((yi > y) != (yj > y) &&
                x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    #endregion

    #region Audit Logging

    /// <summary>
    /// Log a compliance event.
    /// </summary>
    public async Task LogEventAsync(ComplianceEvent evt)
    {
        evt.Id = Guid.NewGuid().ToString();
        await _database.SaveComplianceEventAsync(evt);

        // Check for violations
        if (IsViolation(evt.Type))
        {
            ViolationDetected?.Invoke(this, new ComplianceViolationEventArgs(evt));
        }
    }

    /// <summary>
    /// Get audit log for compliance reporting.
    /// </summary>
    public async Task<IReadOnlyList<ComplianceEvent>> GetAuditLogAsync(
        string? vehicleId = null,
        DateTime? fromDate = null,
        DateTime? toDate = null,
        ComplianceEventType? type = null)
    {
        return await _database.GetComplianceEventsAsync(vehicleId, null, type, fromDate, toDate, 10000);
    }

    /// <summary>
    /// Generate compliance report for regulatory submission.
    /// </summary>
    public async Task<ComplianceReport> GenerateReportAsync(DateTime fromDate, DateTime toDate)
    {
        var events = await _database.GetComplianceEventsAsync(null, null, null, fromDate, toDate, int.MaxValue);
        var vehicles = await _database.GetAllVehiclesAsync();
        var operators = await _database.GetAllOperatorsAsync();

        var report = new ComplianceReport
        {
            GeneratedAt = DateTime.UtcNow,
            ReportPeriod = (fromDate, toDate),

            // Summary statistics
            TotalFlights = events.Count(e => e.Type == ComplianceEventType.MissionStarted),
            TotalFlightHours = 0, // Would need to calculate from missions
            VehiclesOperated = events.Select(e => e.VehicleId).Where(id => id != null).Distinct().Count(),
            OperatorsActive = events.Select(e => e.OperatorId).Where(id => id != null).Distinct().Count(),

            // Incidents
            GeofenceViolations = events.Count(e => e.Type == ComplianceEventType.GeofenceViolation),
            AltitudeViolations = events.Count(e => e.Type == ComplianceEventType.AltitudeViolation),
            UnauthorizedFlights = events.Count(e => e.Type == ComplianceEventType.UnauthorizedFlight),
            EmergencyLandings = events.Count(e => e.Type == ComplianceEventType.EmergencyLanding),
            IncidentsReported = events.Count(e => e.Type == ComplianceEventType.IncidentReported),

            // Certifications
            VehiclesRegistered = vehicles.Count(v => !string.IsNullOrEmpty(v.FaaRegistrationNumber)),
            VehiclesWithExpiredRegistration = vehicles.Count(v =>
                v.FaaRegistrationExpiry.HasValue && v.FaaRegistrationExpiry < DateTime.UtcNow),
            OperatorsWithValidPart107 = operators.Count(o =>
                o.Certifications.Any(c => c.Type == CertificationType.Part107 && c.ExpiresAt > DateTime.UtcNow)),

            // Maintenance compliance
            MaintenanceCompletedOnTime = 0, // Would need maintenance data
            MaintenanceOverdue = 0
        };

        // Generate detailed event log
        report.SignificantEvents = events
            .Where(e => IsSignificantEvent(e.Type))
            .Select(e => new ComplianceEventSummary
            {
                Timestamp = e.Timestamp,
                Type = e.Type.ToString(),
                VehicleId = e.VehicleId,
                OperatorId = e.OperatorId,
                Details = e.Details
            })
            .ToList();

        return report;
    }

    private bool IsViolation(ComplianceEventType type)
    {
        return type is
            ComplianceEventType.GeofenceViolation or
            ComplianceEventType.AltitudeViolation or
            ComplianceEventType.UnauthorizedFlight or
            ComplianceEventType.PilotCertificationExpired or
            ComplianceEventType.MaintenanceOverdue;
    }

    private bool IsSignificantEvent(ComplianceEventType type)
    {
        return type is
            ComplianceEventType.GeofenceViolation or
            ComplianceEventType.AltitudeViolation or
            ComplianceEventType.UnauthorizedFlight or
            ComplianceEventType.EmergencyLanding or
            ComplianceEventType.IncidentReported or
            ComplianceEventType.VehicleRegistered or
            ComplianceEventType.VehicleDecommissioned;
    }

    #endregion
}

#region Compliance Models

public class ComplianceViolationEventArgs : EventArgs
{
    public ComplianceEvent Event { get; }

    public ComplianceViolationEventArgs(ComplianceEvent evt)
    {
        Event = evt;
    }
}

public class ComplianceReport
{
    public DateTime GeneratedAt { get; set; }
    public (DateTime from, DateTime to) ReportPeriod { get; set; }

    // Operations Summary
    public int TotalFlights { get; set; }
    public double TotalFlightHours { get; set; }
    public int VehiclesOperated { get; set; }
    public int OperatorsActive { get; set; }

    // Incidents & Violations
    public int GeofenceViolations { get; set; }
    public int AltitudeViolations { get; set; }
    public int UnauthorizedFlights { get; set; }
    public int EmergencyLandings { get; set; }
    public int IncidentsReported { get; set; }

    // Certification Status
    public int VehiclesRegistered { get; set; }
    public int VehiclesWithExpiredRegistration { get; set; }
    public int OperatorsWithValidPart107 { get; set; }

    // Maintenance Status
    public int MaintenanceCompletedOnTime { get; set; }
    public int MaintenanceOverdue { get; set; }

    // Detailed Events
    public List<ComplianceEventSummary> SignificantEvents { get; set; } = new();
}

public class ComplianceEventSummary
{
    public DateTime Timestamp { get; set; }
    public string Type { get; set; } = string.Empty;
    public string? VehicleId { get; set; }
    public string? OperatorId { get; set; }
    public string? Details { get; set; }
}

#endregion

#region Operator Models

public class Operator
{
    public string Id { get; set; } = Guid.NewGuid().ToString();
    public string Name { get; set; } = string.Empty;
    public string Email { get; set; } = string.Empty;
    public string Phone { get; set; } = string.Empty;

    // Role & Permissions
    public OperatorRole Role { get; set; } = OperatorRole.Pilot;
    public List<string> Permissions { get; set; } = new();

    // Certifications
    public List<OperatorCertification> Certifications { get; set; } = new();

    // Authorization
    public List<string> AuthorizedVehicles { get; set; } = new(); // Vehicle IDs or "*" for all
    public List<string> AuthorizedMissionTypes { get; set; } = new();

    // Status
    public OperatorStatus Status { get; set; } = OperatorStatus.Active;
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime? LastActiveAt { get; set; }

    // Statistics
    public int TotalFlights { get; set; }
    public double TotalFlightHours { get; set; }
}

public enum OperatorRole
{
    Pilot,
    Observer,
    Supervisor,
    FleetManager,
    Technician,
    Administrator
}

public enum OperatorStatus
{
    Active,
    Suspended,
    Inactive,
    CertificationExpired
}

public class OperatorCertification
{
    public CertificationType Type { get; set; }
    public string Number { get; set; } = string.Empty;
    public DateTime IssuedAt { get; set; }
    public DateTime ExpiresAt { get; set; }
    public string IssuedBy { get; set; } = string.Empty;
    public bool IsValid => ExpiresAt > DateTime.UtcNow;
}

public enum CertificationType
{
    Part107,
    Part107Waiver,
    Part61,
    MedicalCertificate,
    DroneSpecific,
    PayloadSpecific,
    BVLOS,
    NightOperations,
    OverPeople
}

#endregion
