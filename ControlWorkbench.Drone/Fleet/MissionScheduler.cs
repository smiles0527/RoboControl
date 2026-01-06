using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Fleet;

/// <summary>
/// Mission Scheduler - Intelligent mission assignment and scheduling.
/// Optimizes vehicle selection, timing, and resource utilization.
/// </summary>
public class MissionScheduler
{
    private readonly VehicleRegistry _registry;
    private readonly SchedulerConfiguration _config;
    private readonly List<ScheduledMission> _scheduledMissions;
    private readonly object _scheduleLock = new();

    public MissionScheduler(VehicleRegistry registry, SchedulerConfiguration config)
    {
        _registry = registry;
        _config = config;
        _scheduledMissions = new List<ScheduledMission>();
    }

    /// <summary>
    /// Find the best vehicle for a mission.
    /// </summary>
    public async Task<VehicleAssignment?> FindBestVehicleAsync(Mission mission)
    {
        var availableVehicles = await _registry.GetAvailableVehiclesAsync();

        if (availableVehicles.Count == 0)
            return null;

        var candidates = new List<VehicleScore>();

        foreach (var vehicle in availableVehicles)
        {
            var score = await ScoreVehicleForMissionAsync(vehicle, mission);
            if (score.IsEligible)
            {
                candidates.Add(score);
            }
        }

        if (candidates.Count == 0)
            return null;

        // Sort by score descending
        candidates = candidates.OrderByDescending(c => c.TotalScore).ToList();
        var best = candidates.First();

        return new VehicleAssignment
        {
            VehicleId = best.Vehicle.Id,
            Callsign = best.Vehicle.Callsign,
            Score = best.TotalScore,
            Reasoning = best.Reasoning
        };
    }

    /// <summary>
    /// Score a vehicle for mission suitability.
    /// </summary>
    private async Task<VehicleScore> ScoreVehicleForMissionAsync(Vehicle vehicle, Mission mission)
    {
        var score = new VehicleScore { Vehicle = vehicle };
        var reasons = new List<string>();

        // Check basic eligibility
        if (vehicle.Status == VehicleStatus.Maintenance ||
            vehicle.Status == VehicleStatus.Decommissioned ||
            vehicle.Status == VehicleStatus.Flying)
        {
            score.IsEligible = false;
            score.Reasoning = "Vehicle not available";
            return score;
        }

        // Calculate mission requirements
        var totalDistance = CalculateTotalDistance(mission.Waypoints);
        var estimatedFlightTime = EstimateFlightTime(totalDistance, mission.Waypoints);

        // Check if vehicle can complete mission
        if (estimatedFlightTime.TotalMinutes > vehicle.Specs.MaxFlightTimeMinutes * 0.8) // 80% safety margin
        {
            score.IsEligible = false;
            score.Reasoning = "Mission exceeds vehicle endurance";
            return score;
        }

        if (totalDistance > vehicle.Specs.MaxRangeKm)
        {
            score.IsEligible = false;
            score.Reasoning = "Mission exceeds vehicle range";
            return score;
        }

        // Check altitude requirements
        var maxAltitude = mission.Waypoints.Max(w => w.Position[2]);
        if (maxAltitude > vehicle.Specs.MaxAltitudeM)
        {
            score.IsEligible = false;
            score.Reasoning = "Mission exceeds vehicle altitude capability";
            return score;
        }

        score.IsEligible = true;

        // Score components (0-100 each)

        // Endurance margin score
        var enduranceMargin = 1.0 - (estimatedFlightTime.TotalMinutes / vehicle.Specs.MaxFlightTimeMinutes);
        score.EnduranceScore = System.Math.Max(0, enduranceMargin * 100);
        reasons.Add($"Endurance margin: {enduranceMargin:P0}");

        // Maintenance health score
        var maintenanceRatio = vehicle.FlightHoursSinceLastMaintenance / vehicle.MaintenanceSchedule.FlightHoursInterval;
        score.MaintenanceScore = System.Math.Max(0, (1.0 - maintenanceRatio) * 100);
        reasons.Add($"Maintenance health: {1.0 - maintenanceRatio:P0}");

        // Experience score (prefer vehicles with more flight time for complex missions)
        var experienceScore = System.Math.Min(100, vehicle.TotalFlightHours * 2);
        score.ExperienceScore = experienceScore;

        // Proximity score (would need current position - assume all equal for now)
        score.ProximityScore = 50;

        // Capability match score
        score.CapabilityScore = CalculateCapabilityMatch(vehicle, mission);

        // Availability score (check scheduling conflicts)
        var hasConflict = await CheckSchedulingConflictAsync(vehicle.Id, mission.ScheduledStart, estimatedFlightTime);
        score.AvailabilityScore = hasConflict ? 0 : 100;

        // Calculate weighted total
        score.TotalScore =
            score.EnduranceScore * 0.25 +
            score.MaintenanceScore * 0.20 +
            score.ExperienceScore * 0.10 +
            score.ProximityScore * 0.15 +
            score.CapabilityScore * 0.15 +
            score.AvailabilityScore * 0.15;

        score.Reasoning = string.Join("; ", reasons);

        return score;
    }

    /// <summary>
    /// Schedule a mission for a specific time slot.
    /// </summary>
    public async Task<ScheduleResult> ScheduleMissionAsync(Mission mission, DateTime startTime)
    {
        var estimatedDuration = mission.EstimatedDuration ?? TimeSpan.FromMinutes(30);

        // Check for conflicts
        if (!string.IsNullOrEmpty(mission.AssignedVehicleId))
        {
            var hasConflict = await CheckSchedulingConflictAsync(
                mission.AssignedVehicleId, startTime, estimatedDuration);

            if (hasConflict)
            {
                return new ScheduleResult
                {
                    Success = false,
                    Error = "Vehicle has conflicting scheduled mission"
                };
            }
        }

        lock (_scheduleLock)
        {
            _scheduledMissions.Add(new ScheduledMission
            {
                MissionId = mission.Id,
                VehicleId = mission.AssignedVehicleId,
                ScheduledStart = startTime,
                EstimatedEnd = startTime.Add(estimatedDuration)
            });
        }

        return new ScheduleResult
        {
            Success = true,
            ScheduledStart = startTime,
            EstimatedEnd = startTime.Add(estimatedDuration)
        };
    }

    /// <summary>
    /// Get upcoming scheduled missions.
    /// </summary>
    public IReadOnlyList<ScheduledMission> GetUpcomingMissions(string? vehicleId = null)
    {
        lock (_scheduleLock)
        {
            var query = _scheduledMissions
                .Where(m => m.ScheduledStart > DateTime.UtcNow);

            if (!string.IsNullOrEmpty(vehicleId))
                query = query.Where(m => m.VehicleId == vehicleId);

            return query.OrderBy(m => m.ScheduledStart).ToList();
        }
    }

    /// <summary>
    /// Find optimal time slots for a mission.
    /// </summary>
    public async Task<IReadOnlyList<TimeSlot>> FindAvailableTimeSlotsAsync(
        string vehicleId,
        TimeSpan missionDuration,
        DateTime fromTime,
        DateTime toTime,
        int maxSlots = 5)
    {
        var slots = new List<TimeSlot>();

        lock (_scheduleLock)
        {
            var vehicleMissions = _scheduledMissions
                .Where(m => m.VehicleId == vehicleId)
                .Where(m => m.ScheduledStart >= fromTime && m.ScheduledStart <= toTime)
                .OrderBy(m => m.ScheduledStart)
                .ToList();

            var currentTime = fromTime;

            foreach (var scheduled in vehicleMissions)
            {
                // Gap before this mission
                var gap = scheduled.ScheduledStart - currentTime;
                if (gap >= missionDuration)
                {
                    slots.Add(new TimeSlot
                    {
                        Start = currentTime,
                        End = currentTime.Add(missionDuration),
                        Available = true
                    });

                    if (slots.Count >= maxSlots) break;
                }

                currentTime = scheduled.EstimatedEnd;
            }

            // Check remaining time after last mission
            if (slots.Count < maxSlots && toTime - currentTime >= missionDuration)
            {
                slots.Add(new TimeSlot
                {
                    Start = currentTime,
                    End = currentTime.Add(missionDuration),
                    Available = true
                });
            }
        }

        return slots;
    }

    /// <summary>
    /// Optimize fleet schedule for a set of missions.
    /// Uses greedy assignment with priority ordering.
    /// </summary>
    public async Task<FleetSchedule> OptimizeScheduleAsync(
        IEnumerable<Mission> pendingMissions,
        DateTime scheduleWindow)
    {
        var schedule = new FleetSchedule { GeneratedAt = DateTime.UtcNow };
        var sortedMissions = pendingMissions
            .OrderByDescending(m => m.Priority)
            .ThenBy(m => m.ScheduledStart)
            .ToList();

        foreach (var mission in sortedMissions)
        {
            var assignment = await FindBestVehicleAsync(mission);
            if (assignment != null)
            {
                var startTime = mission.ScheduledStart ?? DateTime.UtcNow.AddMinutes(15);
                var result = await ScheduleMissionAsync(mission, startTime);

                if (result.Success)
                {
                    schedule.Assignments.Add(new MissionAssignment
                    {
                        MissionId = mission.Id,
                        MissionName = mission.Name,
                        VehicleId = assignment.VehicleId,
                        VehicleCallsign = assignment.Callsign,
                        ScheduledStart = result.ScheduledStart!.Value,
                        EstimatedEnd = result.EstimatedEnd!.Value,
                        Score = assignment.Score
                    });
                }
                else
                {
                    schedule.UnassignedMissions.Add(new UnassignedMission
                    {
                        MissionId = mission.Id,
                        MissionName = mission.Name,
                        Reason = result.Error ?? "Unknown error"
                    });
                }
            }
            else
            {
                schedule.UnassignedMissions.Add(new UnassignedMission
                {
                    MissionId = mission.Id,
                    MissionName = mission.Name,
                    Reason = "No suitable vehicle available"
                });
            }
        }

        return schedule;
    }

    private async Task<bool> CheckSchedulingConflictAsync(
        string vehicleId,
        DateTime? startTime,
        TimeSpan duration)
    {
        if (!startTime.HasValue) return false;

        var endTime = startTime.Value.Add(duration);

        lock (_scheduleLock)
        {
            return _scheduledMissions.Any(m =>
                m.VehicleId == vehicleId &&
                m.ScheduledStart < endTime &&
                m.EstimatedEnd > startTime.Value);
        }
    }

    private double CalculateTotalDistance(List<MissionWaypoint> waypoints)
    {
        if (waypoints.Count < 2) return 0;

        double total = 0;
        for (int i = 1; i < waypoints.Count; i++)
        {
            total += (waypoints[i].Position - waypoints[i - 1].Position).L2Norm();
        }

        return total / 1000.0; // Convert to km
    }

    private TimeSpan EstimateFlightTime(double distanceKm, List<MissionWaypoint> waypoints)
    {
        // Estimate based on average speed and hold times
        var avgSpeedKmh = waypoints.Average(w => w.Speed) * 3.6; // m/s to km/h
        var flightMinutes = (distanceKm / avgSpeedKmh) * 60;

        // Add hold times
        var holdMinutes = waypoints
            .Where(w => w.HoldTime.HasValue)
            .Sum(w => w.HoldTime!.Value.TotalMinutes);

        // Add takeoff/landing time
        flightMinutes += 2; // 1 min takeoff + 1 min landing

        return TimeSpan.FromMinutes(flightMinutes + holdMinutes);
    }

    private double CalculateCapabilityMatch(Vehicle vehicle, Mission mission)
    {
        // Check if vehicle has required payloads/capabilities
        // This would be more sophisticated in production
        return 80; // Default good match
    }
}

#region Scheduler Models

public class VehicleAssignment
{
    public string VehicleId { get; set; } = string.Empty;
    public string Callsign { get; set; } = string.Empty;
    public double Score { get; set; }
    public string Reasoning { get; set; } = string.Empty;
}

public class VehicleScore
{
    public Vehicle Vehicle { get; set; } = null!;
    public bool IsEligible { get; set; }
    public double EnduranceScore { get; set; }
    public double MaintenanceScore { get; set; }
    public double ExperienceScore { get; set; }
    public double ProximityScore { get; set; }
    public double CapabilityScore { get; set; }
    public double AvailabilityScore { get; set; }
    public double TotalScore { get; set; }
    public string Reasoning { get; set; } = string.Empty;
}

public class ScheduledMission
{
    public string MissionId { get; set; } = string.Empty;
    public string? VehicleId { get; set; }
    public DateTime ScheduledStart { get; set; }
    public DateTime EstimatedEnd { get; set; }
}

public class ScheduleResult
{
    public bool Success { get; set; }
    public DateTime? ScheduledStart { get; set; }
    public DateTime? EstimatedEnd { get; set; }
    public string? Error { get; set; }
}

public class TimeSlot
{
    public DateTime Start { get; set; }
    public DateTime End { get; set; }
    public bool Available { get; set; }
}

public class FleetSchedule
{
    public DateTime GeneratedAt { get; set; }
    public List<MissionAssignment> Assignments { get; set; } = new();
    public List<UnassignedMission> UnassignedMissions { get; set; } = new();
}

public class MissionAssignment
{
    public string MissionId { get; set; } = string.Empty;
    public string MissionName { get; set; } = string.Empty;
    public string VehicleId { get; set; } = string.Empty;
    public string VehicleCallsign { get; set; } = string.Empty;
    public DateTime ScheduledStart { get; set; }
    public DateTime EstimatedEnd { get; set; }
    public double Score { get; set; }
}

public class UnassignedMission
{
    public string MissionId { get; set; } = string.Empty;
    public string MissionName { get; set; } = string.Empty;
    public string Reason { get; set; } = string.Empty;
}

#endregion
