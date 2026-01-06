namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Maintenance Manager - Predictive and scheduled maintenance tracking.
/// Integrates with AI anomaly detection for proactive maintenance.
/// </summary>
public class MaintenanceManager
{
    private readonly FleetDatabase _database;
    private readonly VehicleRegistry _registry;

    public event EventHandler<MaintenanceAlertEventArgs>? MaintenanceAlert;

    public MaintenanceManager(FleetDatabase database, VehicleRegistry registry)
    {
        _database = database;
        _registry = registry;
    }

    /// <summary>
    /// Schedule a new maintenance task.
    /// </summary>
    public async Task<MaintenanceTicket> ScheduleAsync(MaintenanceRequest request)
    {
        var vehicle = await _registry.GetVehicleAsync(request.VehicleId);
        if (vehicle == null)
            throw new ArgumentException($"Vehicle {request.VehicleId} not found");

        var ticket = new MaintenanceTicket
        {
            Id = Guid.NewGuid().ToString(),
            VehicleId = request.VehicleId,
            Type = request.Type,
            Priority = request.Priority,
            Description = request.Description,
            Status = MaintenanceStatus.Scheduled,
            CreatedAt = DateTime.UtcNow,
            ScheduledFor = request.ScheduledFor ?? DateTime.UtcNow.AddDays(1),
            RequiredParts = request.RequiredParts
        };

        await _database.SaveMaintenanceTicketAsync(ticket);

        // Update vehicle status if high priority
        if (request.Priority == MaintenancePriority.Critical)
        {
            await _registry.UpdateStatusAsync(request.VehicleId, VehicleStatus.Maintenance);
        }

        return ticket;
    }

    /// <summary>
    /// Get all pending maintenance tickets.
    /// </summary>
    public async Task<IReadOnlyList<MaintenanceTicket>> GetPendingAsync()
    {
        var tickets = await _database.GetMaintenanceTicketsAsync(null, null);
        return tickets
            .Where(t => t.Status == MaintenanceStatus.Scheduled ||
                       t.Status == MaintenanceStatus.Requested ||
                       t.Status == MaintenanceStatus.AwaitingParts)
            .OrderByDescending(t => t.Priority)
            .ThenBy(t => t.ScheduledFor)
            .ToList();
    }

    /// <summary>
    /// Start maintenance work on a ticket.
    /// </summary>
    public async Task StartMaintenanceAsync(string ticketId, string technicianId)
    {
        var ticket = await _database.GetMaintenanceTicketAsync(ticketId);
        if (ticket == null)
            throw new ArgumentException($"Ticket {ticketId} not found");

        ticket.Status = MaintenanceStatus.InProgress;
        ticket.StartedAt = DateTime.UtcNow;
        ticket.AssignedTo = technicianId;

        await _database.SaveMaintenanceTicketAsync(ticket);
        await _registry.UpdateStatusAsync(ticket.VehicleId, VehicleStatus.Maintenance);
    }

    /// <summary>
    /// Complete maintenance work.
    /// </summary>
    public async Task CompleteAsync(string ticketId, MaintenanceCompletion completion)
    {
        var ticket = await _database.GetMaintenanceTicketAsync(ticketId);
        if (ticket == null)
            throw new ArgumentException($"Ticket {ticketId} not found");

        ticket.Status = MaintenanceStatus.Completed;
        ticket.CompletedAt = DateTime.UtcNow;
        ticket.UsedParts = completion.PartsUsed;
        ticket.LaborHours = completion.LaborHours;
        ticket.Cost = completion.PartsCost + completion.LaborCost;
        ticket.Notes = completion.Notes;

        await _database.SaveMaintenanceTicketAsync(ticket);

        // Reset vehicle maintenance counters and return to ready status
        await _registry.RecordMaintenanceAsync(ticket.VehicleId);

        // Check for follow-up requirements
        if (completion.FollowUpRequired != null && completion.FollowUpRequired.Count > 0)
        {
            foreach (var followUp in completion.FollowUpRequired)
            {
                await ScheduleAsync(new MaintenanceRequest
                {
                    VehicleId = ticket.VehicleId,
                    Type = MaintenanceType.Unscheduled,
                    Priority = MaintenancePriority.Medium,
                    Description = followUp,
                    RequestedBy = completion.CompletedBy
                });
            }
        }
    }

    /// <summary>
    /// Check fleet for maintenance needs.
    /// </summary>
    public async Task<IReadOnlyList<MaintenanceRecommendation>> CheckMaintenanceNeedsAsync()
    {
        var recommendations = new List<MaintenanceRecommendation>();
        var vehicles = await _registry.GetAllVehiclesAsync();

        foreach (var vehicle in vehicles)
        {
            if (vehicle.Status == VehicleStatus.Decommissioned) continue;

            var schedule = vehicle.MaintenanceSchedule;

            // Flight hours check
            var flightHoursRatio = vehicle.FlightHoursSinceLastMaintenance / schedule.FlightHoursInterval;
            if (flightHoursRatio >= 0.8)
            {
                recommendations.Add(new MaintenanceRecommendation
                {
                    VehicleId = vehicle.Id,
                    Callsign = vehicle.Callsign,
                    Reason = MaintenanceReason.FlightHoursInterval,
                    Urgency = flightHoursRatio >= 1.0 ? MaintenancePriority.High : MaintenancePriority.Medium,
                    Message = $"Flight hours: {vehicle.FlightHoursSinceLastMaintenance:F1}/{schedule.FlightHoursInterval}",
                    RecommendedDate = flightHoursRatio >= 1.0 ? DateTime.UtcNow : DateTime.UtcNow.AddDays(7)
                });
            }

            // Flights count check
            var flightsRatio = (double)vehicle.FlightsSinceLastMaintenance / schedule.FlightsInterval;
            if (flightsRatio >= 0.8)
            {
                recommendations.Add(new MaintenanceRecommendation
                {
                    VehicleId = vehicle.Id,
                    Callsign = vehicle.Callsign,
                    Reason = MaintenanceReason.FlightsInterval,
                    Urgency = flightsRatio >= 1.0 ? MaintenancePriority.High : MaintenancePriority.Medium,
                    Message = $"Flights: {vehicle.FlightsSinceLastMaintenance}/{schedule.FlightsInterval}",
                    RecommendedDate = flightsRatio >= 1.0 ? DateTime.UtcNow : DateTime.UtcNow.AddDays(7)
                });
            }

            // Calendar check
            if (vehicle.LastMaintenanceAt.HasValue)
            {
                var daysSinceMaintenance = (DateTime.UtcNow - vehicle.LastMaintenanceAt.Value).TotalDays;
                var daysRatio = daysSinceMaintenance / schedule.DaysInterval;
                if (daysRatio >= 0.8)
                {
                    recommendations.Add(new MaintenanceRecommendation
                    {
                        VehicleId = vehicle.Id,
                        Callsign = vehicle.Callsign,
                        Reason = MaintenanceReason.CalendarInterval,
                        Urgency = daysRatio >= 1.0 ? MaintenancePriority.High : MaintenancePriority.Medium,
                        Message = $"Days since maintenance: {daysSinceMaintenance:F0}/{schedule.DaysInterval}",
                        RecommendedDate = daysRatio >= 1.0 ? DateTime.UtcNow : DateTime.UtcNow.AddDays(7)
                    });
                }
            }

            // Certification expiry check
            foreach (var cert in vehicle.Certifications)
            {
                var daysToExpiry = (cert.ExpiresAt - DateTime.UtcNow).TotalDays;
                if (daysToExpiry <= 30)
                {
                    recommendations.Add(new MaintenanceRecommendation
                    {
                        VehicleId = vehicle.Id,
                        Callsign = vehicle.Callsign,
                        Reason = MaintenanceReason.CertificationExpiring,
                        Urgency = daysToExpiry <= 7 ? MaintenancePriority.Critical : MaintenancePriority.High,
                        Message = $"Certification '{cert.Type}' expires in {daysToExpiry:F0} days",
                        RecommendedDate = DateTime.UtcNow
                    });
                }
            }
        }

        return recommendations.OrderByDescending(r => r.Urgency).ToList();
    }

    /// <summary>
    /// Integrate predictive maintenance alert from AI system.
    /// </summary>
    public async Task ProcessPredictiveAlertAsync(PredictiveMaintenanceAlert alert)
    {
        // Auto-create maintenance ticket for critical predictions
        if (alert.ConfidenceLevel >= 0.8 && alert.EstimatedDaysToFailure <= 7)
        {
            var ticket = await ScheduleAsync(new MaintenanceRequest
            {
                VehicleId = alert.VehicleId,
                Type = MaintenanceType.Preventive,
                Priority = alert.EstimatedDaysToFailure <= 1
                    ? MaintenancePriority.Critical
                    : MaintenancePriority.High,
                Description = $"Predictive: {alert.ComponentName} - {alert.FailureType}. " +
                             $"Confidence: {alert.ConfidenceLevel:P0}, Est. failure in {alert.EstimatedDaysToFailure:F1} days",
                RequestedBy = "AI-Predictive-System"
            });

            MaintenanceAlert?.Invoke(this, new MaintenanceAlertEventArgs(alert.VehicleId, ticket));
        }
    }

    /// <summary>
    /// Get maintenance history for a vehicle.
    /// </summary>
    public async Task<IReadOnlyList<MaintenanceTicket>> GetHistoryAsync(
        string vehicleId,
        DateTime? fromDate = null)
    {
        var tickets = await _database.GetMaintenanceTicketsAsync(vehicleId, MaintenanceStatus.Completed);

        if (fromDate.HasValue)
        {
            tickets = tickets.Where(t => t.CompletedAt >= fromDate.Value).ToList();
        }

        return tickets.OrderByDescending(t => t.CompletedAt).ToList();
    }

    /// <summary>
    /// Get maintenance cost summary.
    /// </summary>
    public async Task<MaintenanceCostSummary> GetCostSummaryAsync(DateTime fromDate, DateTime toDate)
    {
        var tickets = await _database.GetMaintenanceTicketsAsync(null, MaintenanceStatus.Completed);
        var completedInRange = tickets
            .Where(t => t.CompletedAt >= fromDate && t.CompletedAt <= toDate)
            .ToList();

        return new MaintenanceCostSummary
        {
            Period = (fromDate, toDate),
            TotalCost = completedInRange.Sum(t => t.Cost ?? 0),
            LaborCost = completedInRange.Sum(t => (t.LaborHours ?? 0) * 50), // Assumed hourly rate
            PartsCost = completedInRange.Sum(t => t.Cost ?? 0) - completedInRange.Sum(t => (t.LaborHours ?? 0) * 50),
            TicketsCompleted = completedInRange.Count,
            TotalLaborHours = completedInRange.Sum(t => t.LaborHours ?? 0),
            CostByType = completedInRange
                .GroupBy(t => t.Type)
                .ToDictionary(g => g.Key, g => g.Sum(t => t.Cost ?? 0)),
            CostByVehicle = completedInRange
                .GroupBy(t => t.VehicleId)
                .ToDictionary(g => g.Key, g => g.Sum(t => t.Cost ?? 0))
        };
    }
}

#region Maintenance Models

public class MaintenanceRecommendation
{
    public string VehicleId { get; set; } = string.Empty;
    public string Callsign { get; set; } = string.Empty;
    public MaintenanceReason Reason { get; set; }
    public MaintenancePriority Urgency { get; set; }
    public string Message { get; set; } = string.Empty;
    public DateTime RecommendedDate { get; set; }
}

public enum MaintenanceReason
{
    FlightHoursInterval,
    FlightsInterval,
    CalendarInterval,
    CertificationExpiring,
    AnomalyDetected,
    ComponentDegradation,
    OperatorRequest,
    IncidentFollowUp
}

public class PredictiveMaintenanceAlert
{
    public string VehicleId { get; set; } = string.Empty;
    public string ComponentName { get; set; } = string.Empty;
    public string FailureType { get; set; } = string.Empty;
    public double ConfidenceLevel { get; set; }
    public double EstimatedDaysToFailure { get; set; }
    public double RemainingUsefulLife { get; set; }
    public DateTime PredictionTime { get; set; }
    public Dictionary<string, double>? SupportingMetrics { get; set; }
}

public class MaintenanceCostSummary
{
    public (DateTime from, DateTime to) Period { get; set; }
    public double TotalCost { get; set; }
    public double LaborCost { get; set; }
    public double PartsCost { get; set; }
    public int TicketsCompleted { get; set; }
    public double TotalLaborHours { get; set; }
    public Dictionary<MaintenanceType, double> CostByType { get; set; } = new();
    public Dictionary<string, double> CostByVehicle { get; set; } = new();
}

public class MaintenanceAlertEventArgs : EventArgs
{
    public string VehicleId { get; }
    public MaintenanceTicket Ticket { get; }

    public MaintenanceAlertEventArgs(string vehicleId, MaintenanceTicket ticket)
    {
        VehicleId = vehicleId;
        Ticket = ticket;
    }
}

#endregion
