namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Fleet Analytics - Business intelligence and operational metrics.
/// Provides insights for fleet optimization, cost analysis, and performance tracking.
/// </summary>
public class FleetAnalytics
{
    private readonly FleetDatabase _database;

    public FleetAnalytics(FleetDatabase database)
    {
        _database = database;
    }

    /// <summary>
    /// Generate comprehensive analytics report.
    /// </summary>
    public async Task<FleetAnalyticsReport> GenerateReportAsync(
        DateTime fromDate,
        DateTime toDate,
        string? vehicleId = null)
    {
        var report = new FleetAnalyticsReport
        {
            FromDate = fromDate,
            ToDate = toDate,
            VehicleId = vehicleId
        };

        // Get missions in range
        var missions = await _database.GetMissionsAsync(vehicleId, fromDate, toDate, int.MaxValue);

        // Flight statistics
        var completedMissions = missions.Where(m => m.Status == MissionStatus.Completed).ToList();
        var abortedMissions = missions.Where(m => m.Status == MissionStatus.Aborted).ToList();
        var failedMissions = missions.Where(m => m.Status == MissionStatus.Failed).ToList();

        report.TotalFlights = missions.Count;
        report.MissionsCompleted = completedMissions.Count;
        report.MissionsAborted = abortedMissions.Count;
        report.MissionsFailed = failedMissions.Count;

        if (missions.Count > 0)
        {
            report.MissionSuccessRate = (double)completedMissions.Count / missions.Count * 100;
        }

        // Flight hours and distance
        foreach (var mission in completedMissions)
        {
            if (mission.ActualDuration.HasValue)
            {
                report.TotalFlightHours += mission.ActualDuration.Value.TotalHours;
            }
            report.TotalDistanceKm += mission.ActualDistanceKm;
        }

        if (completedMissions.Count > 0)
        {
            report.AverageFlightDurationMinutes = report.TotalFlightHours * 60 / completedMissions.Count;
        }

        // Alerts analysis
        var alerts = await _database.GetAlertsAsync(vehicleId, null, fromDate, int.MaxValue);
        report.AlertsGenerated = alerts.Count;
        report.CriticalAlerts = alerts.Count(a => a.Severity == AlertSeverity.Critical || a.Severity == AlertSeverity.Emergency);

        // Uptime calculation
        report.UptimePercent = await CalculateUptimeAsync(vehicleId, fromDate, toDate);

        // MTBF calculation
        report.MtbfHours = await CalculateMtbfAsync(vehicleId, fromDate, toDate);

        // Cost analysis (estimates)
        report.EstimatedOperatingCost = CalculateOperatingCost(report);
        if (report.TotalFlightHours > 0)
        {
            report.CostPerFlightHour = report.EstimatedOperatingCost / report.TotalFlightHours;
        }
        if (report.TotalDistanceKm > 0)
        {
            report.CostPerKm = report.EstimatedOperatingCost / report.TotalDistanceKm;
        }

        // Daily stats
        report.DailyStats = await GenerateDailyStatsAsync(missions, fromDate, toDate);

        // Vehicle breakdown
        if (string.IsNullOrEmpty(vehicleId))
        {
            report.VehicleBreakdown = await GenerateVehicleBreakdownAsync(missions);
        }

        return report;
    }

    /// <summary>
    /// Get fleet utilization statistics.
    /// </summary>
    public async Task<UtilizationStats> GetUtilizationAsync(DateTime fromDate, DateTime toDate)
    {
        var stats = new UtilizationStats
        {
            FromDate = fromDate,
            ToDate = toDate
        };

        var vehicles = await _database.GetAllVehiclesAsync();
        var missions = await _database.GetMissionsAsync(null, fromDate, toDate, int.MaxValue);
        var totalHours = (toDate - fromDate).TotalHours;

        // Per-vehicle utilization
        foreach (var vehicle in vehicles.Where(v => v.Status != VehicleStatus.Decommissioned))
        {
            var vehicleMissions = missions
                .Where(m => m.AssignedVehicleId == vehicle.Id && m.Status == MissionStatus.Completed)
                .ToList();

            var flightHours = vehicleMissions
                .Where(m => m.ActualDuration.HasValue)
                .Sum(m => m.ActualDuration!.Value.TotalHours);

            var utilization = totalHours > 0 ? (flightHours / totalHours) * 100 : 0;
            stats.VehicleUtilization[vehicle.Id] = utilization;
        }

        // Fleet average
        if (stats.VehicleUtilization.Count > 0)
        {
            stats.FleetUtilizationPercent = stats.VehicleUtilization.Values.Average();
            stats.PeakUtilizationPercent = stats.VehicleUtilization.Values.Max();
        }

        // Utilization by day of week
        var missionsByDay = missions
            .Where(m => m.StartedAt.HasValue)
            .GroupBy(m => m.StartedAt!.Value.DayOfWeek);

        foreach (var group in missionsByDay)
        {
            var dayFlightHours = group
                .Where(m => m.ActualDuration.HasValue)
                .Sum(m => m.ActualDuration!.Value.TotalHours);
            stats.UtilizationByDayOfWeek[group.Key] = dayFlightHours;
        }

        // Utilization by hour
        var missionsByHour = missions
            .Where(m => m.StartedAt.HasValue)
            .GroupBy(m => m.StartedAt!.Value.Hour);

        foreach (var group in missionsByHour)
        {
            stats.UtilizationByHour[group.Key] = group.Count();
        }

        return stats;
    }

    /// <summary>
    /// Get real-time fleet KPIs.
    /// </summary>
    public async Task<FleetKpis> GetKpisAsync()
    {
        var now = DateTime.UtcNow;
        var todayStart = now.Date;
        var weekStart = now.AddDays(-(int)now.DayOfWeek);
        var monthStart = new DateTime(now.Year, now.Month, 1);

        var vehicles = await _database.GetAllVehiclesAsync();
        var activeVehicles = vehicles.Where(v => v.Status != VehicleStatus.Decommissioned).ToList();

        var todayMissions = await _database.GetMissionsAsync(null, todayStart, now, 1000);
        var weekMissions = await _database.GetMissionsAsync(null, weekStart, now, 1000);
        var monthMissions = await _database.GetMissionsAsync(null, monthStart, now, 1000);

        return new FleetKpis
        {
            Timestamp = now,

            // Fleet Status
            TotalVehicles = activeVehicles.Count,
            VehiclesOperational = activeVehicles.Count(v =>
                v.Status != VehicleStatus.Maintenance && v.Status != VehicleStatus.Offline),
            VehiclesInMaintenance = activeVehicles.Count(v => v.Status == VehicleStatus.Maintenance),

            // Today's Activity
            FlightsToday = todayMissions.Count(m => m.Status == MissionStatus.Completed),
            FlightHoursToday = todayMissions
                .Where(m => m.Status == MissionStatus.Completed && m.ActualDuration.HasValue)
                .Sum(m => m.ActualDuration!.Value.TotalHours),

            // Weekly Performance
            FlightsThisWeek = weekMissions.Count(m => m.Status == MissionStatus.Completed),
            MissionSuccessRateWeek = weekMissions.Count > 0
                ? (double)weekMissions.Count(m => m.Status == MissionStatus.Completed) / weekMissions.Count * 100
                : 100,

            // Monthly Totals
            FlightsThisMonth = monthMissions.Count(m => m.Status == MissionStatus.Completed),
            FlightHoursThisMonth = monthMissions
                .Where(m => m.Status == MissionStatus.Completed && m.ActualDuration.HasValue)
                .Sum(m => m.ActualDuration!.Value.TotalHours),
            DistanceThisMonthKm = monthMissions
                .Where(m => m.Status == MissionStatus.Completed)
                .Sum(m => m.ActualDistanceKm),

            // Fleet Totals
            TotalFleetFlightHours = activeVehicles.Sum(v => v.TotalFlightHours),
            TotalFleetFlights = activeVehicles.Sum(v => v.TotalFlights),
            TotalFleetDistanceKm = activeVehicles.Sum(v => v.TotalDistanceKm)
        };
    }

    /// <summary>
    /// Get trending alerts for the fleet.
    /// </summary>
    public async Task<AlertTrends> GetAlertTrendsAsync(int days = 7)
    {
        var fromDate = DateTime.UtcNow.AddDays(-days);
        var alerts = await _database.GetAlertsAsync(null, null, fromDate, 10000);

        var trends = new AlertTrends
        {
            Period = TimeSpan.FromDays(days),
            TotalAlerts = alerts.Count
        };

        // Group by type
        trends.AlertsByType = alerts
            .GroupBy(a => a.Type)
            .ToDictionary(g => g.Key, g => g.Count());

        // Group by severity
        trends.AlertsBySeverity = alerts
            .GroupBy(a => a.Severity)
            .ToDictionary(g => g.Key, g => g.Count());

        // Group by vehicle
        trends.AlertsByVehicle = alerts
            .GroupBy(a => a.VehicleId)
            .ToDictionary(g => g.Key, g => g.Count());

        // Daily trend
        trends.DailyTrend = alerts
            .GroupBy(a => a.Timestamp.Date)
            .OrderBy(g => g.Key)
            .Select(g => new DailyAlertCount { Date = g.Key, Count = g.Count() })
            .ToList();

        // Find most common alert
        if (trends.AlertsByType.Count > 0)
        {
            trends.MostCommonAlert = trends.AlertsByType.OrderByDescending(kv => kv.Value).First().Key;
        }

        return trends;
    }

    private async Task<double> CalculateUptimeAsync(string? vehicleId, DateTime fromDate, DateTime toDate)
    {
        // Calculate uptime based on time not in maintenance or offline
        var totalHours = (toDate - fromDate).TotalHours;

        // Get maintenance periods
        var maintenanceTickets = await _database.GetMaintenanceTicketsAsync(vehicleId, MaintenanceStatus.Completed);
        var maintenanceHours = maintenanceTickets
            .Where(t => t.StartedAt >= fromDate && t.CompletedAt <= toDate)
            .Sum(t => t.LaborHours ?? 0);

        return totalHours > 0 ? ((totalHours - maintenanceHours) / totalHours) * 100 : 100;
    }

    private async Task<double> CalculateMtbfAsync(string? vehicleId, DateTime fromDate, DateTime toDate)
    {
        var alerts = await _database.GetAlertsAsync(vehicleId, null, fromDate, int.MaxValue);
        var failures = alerts.Where(a =>
            a.Severity == AlertSeverity.Critical ||
            a.Severity == AlertSeverity.Emergency).ToList();

        if (failures.Count == 0) return double.MaxValue;

        var totalHours = (toDate - fromDate).TotalHours;
        return totalHours / failures.Count;
    }

    private double CalculateOperatingCost(FleetAnalyticsReport report)
    {
        // Simplified cost model - in production this would use real cost data
        const double costPerFlightHour = 50.0; // Typical for small commercial drones
        const double costPerKm = 0.10;
        const double maintenanceCostPerFlight = 5.0;

        return (report.TotalFlightHours * costPerFlightHour) +
               (report.TotalDistanceKm * costPerKm) +
               (report.TotalFlights * maintenanceCostPerFlight);
    }

    private async Task<List<DailyFlightStats>> GenerateDailyStatsAsync(
        IEnumerable<Mission> missions,
        DateTime fromDate,
        DateTime toDate)
    {
        var stats = new List<DailyFlightStats>();
        var date = fromDate.Date;

        while (date <= toDate.Date)
        {
            var dayMissions = missions
                .Where(m => m.StartedAt?.Date == date || m.CreatedAt.Date == date)
                .ToList();

            stats.Add(new DailyFlightStats
            {
                Date = date,
                Flights = dayMissions.Count(m => m.Status == MissionStatus.Completed),
                FlightHours = dayMissions
                    .Where(m => m.ActualDuration.HasValue)
                    .Sum(m => m.ActualDuration!.Value.TotalHours),
                DistanceKm = dayMissions.Sum(m => m.ActualDistanceKm),
                Missions = dayMissions.Count
            });

            date = date.AddDays(1);
        }

        return stats;
    }

    private async Task<Dictionary<string, VehicleAnalytics>> GenerateVehicleBreakdownAsync(
        IEnumerable<Mission> missions)
    {
        var breakdown = new Dictionary<string, VehicleAnalytics>();
        var vehicles = await _database.GetAllVehiclesAsync();

        foreach (var vehicle in vehicles.Where(v => v.Status != VehicleStatus.Decommissioned))
        {
            var vehicleMissions = missions
                .Where(m => m.AssignedVehicleId == vehicle.Id && m.Status == MissionStatus.Completed)
                .ToList();

            var alerts = await _database.GetAlertsAsync(vehicle.Id, null, DateTime.UtcNow.AddDays(-30), 1000);

            breakdown[vehicle.Id] = new VehicleAnalytics
            {
                VehicleId = vehicle.Id,
                Callsign = vehicle.Callsign,
                Flights = vehicleMissions.Count,
                FlightHours = vehicleMissions
                    .Where(m => m.ActualDuration.HasValue)
                    .Sum(m => m.ActualDuration!.Value.TotalHours),
                DistanceKm = vehicleMissions.Sum(m => m.ActualDistanceKm),
                Alerts = alerts.Count
            };
        }

        return breakdown;
    }
}

#region Analytics Models

public class FleetKpis
{
    public DateTime Timestamp { get; set; }

    // Fleet Status
    public int TotalVehicles { get; set; }
    public int VehiclesOperational { get; set; }
    public int VehiclesInMaintenance { get; set; }

    // Today
    public int FlightsToday { get; set; }
    public double FlightHoursToday { get; set; }

    // This Week
    public int FlightsThisWeek { get; set; }
    public double MissionSuccessRateWeek { get; set; }

    // This Month
    public int FlightsThisMonth { get; set; }
    public double FlightHoursThisMonth { get; set; }
    public double DistanceThisMonthKm { get; set; }

    // All Time
    public double TotalFleetFlightHours { get; set; }
    public int TotalFleetFlights { get; set; }
    public double TotalFleetDistanceKm { get; set; }
}

public class AlertTrends
{
    public TimeSpan Period { get; set; }
    public int TotalAlerts { get; set; }
    public AlertType? MostCommonAlert { get; set; }
    public Dictionary<AlertType, int> AlertsByType { get; set; } = new();
    public Dictionary<AlertSeverity, int> AlertsBySeverity { get; set; } = new();
    public Dictionary<string, int> AlertsByVehicle { get; set; } = new();
    public List<DailyAlertCount> DailyTrend { get; set; } = new();
}

public class DailyAlertCount
{
    public DateTime Date { get; set; }
    public int Count { get; set; }
}

#endregion
