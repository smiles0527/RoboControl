namespace ControlWorkbench.Drone.Mission;

/// <summary>
/// Visual mission planner for drones.
/// Creates waypoint missions with support for various actions.
/// </summary>
public class MissionPlanner
{
    public List<MissionItem> Items { get; } = new();
    public MissionSettings Settings { get; set; } = new();
    
    // Home/launch position
    public GeoPoint? HomePosition { get; set; }
    
    public event Action<MissionStatistics>? StatisticsUpdated;
    
    /// <summary>
    /// Add a navigation waypoint.
    /// </summary>
    public MissionItem AddWaypoint(double lat, double lon, double altMeters, double speedMs = 5)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.Waypoint,
            Location = new GeoPoint(lat, lon, altMeters),
            Speed = speedMs
        };
        Items.Add(item);
        UpdateStatistics();
        return item;
    }
    
    /// <summary>
    /// Add takeoff command.
    /// </summary>
    public MissionItem AddTakeoff(double altMeters)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.Takeoff,
            Location = new GeoPoint(HomePosition?.Latitude ?? 0, HomePosition?.Longitude ?? 0, altMeters)
        };
        Items.Add(item);
        UpdateStatistics();
        return item;
    }
    
    /// <summary>
    /// Add land command.
    /// </summary>
    public MissionItem AddLand(double? lat = null, double? lon = null)
    {
        var lastPos = Items.LastOrDefault()?.Location ?? HomePosition;
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.Land,
            Location = new GeoPoint(lat ?? lastPos?.Latitude ?? 0, lon ?? lastPos?.Longitude ?? 0, 0)
        };
        Items.Add(item);
        UpdateStatistics();
        return item;
    }
    
    /// <summary>
    /// Add return to launch command.
    /// </summary>
    public MissionItem AddRTL()
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.ReturnToLaunch
        };
        Items.Add(item);
        UpdateStatistics();
        return item;
    }
    
    /// <summary>
    /// Add loiter (hover) at position.
    /// </summary>
    public MissionItem AddLoiter(double lat, double lon, double altMeters, double durationSeconds)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.Loiter,
            Location = new GeoPoint(lat, lon, altMeters),
            HoldTime = durationSeconds
        };
        Items.Add(item);
        UpdateStatistics();
        return item;
    }
    
    /// <summary>
    /// Add circle/orbit around a point.
    /// </summary>
    public MissionItem AddOrbit(double lat, double lon, double altMeters, double radiusMeters, int turns = 1)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.Orbit,
            Location = new GeoPoint(lat, lon, altMeters),
            Radius = radiusMeters,
            Turns = turns
        };
        Items.Add(item);
        UpdateStatistics();
        return item;
    }
    
    /// <summary>
    /// Add survey/grid pattern.
    /// </summary>
    public void AddSurveyGrid(GeoPoint corner1, GeoPoint corner2, double altMeters, double lineSpacingMeters, double heading = 0)
    {
        var grid = GenerateSurveyGrid(corner1, corner2, lineSpacingMeters, heading);
        
        foreach (var point in grid)
        {
            AddWaypoint(point.Latitude, point.Longitude, altMeters);
        }
    }
    
    /// <summary>
    /// Add camera trigger action.
    /// </summary>
    public MissionItem AddCameraTrigger(CameraTriggerType type, double param = 0)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.CameraTrigger,
            CameraTrigger = type,
            CameraParam = param
        };
        Items.Add(item);
        return item;
    }
    
    /// <summary>
    /// Add gimbal control.
    /// </summary>
    public MissionItem AddGimbalPitch(double pitchDegrees)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.GimbalControl,
            GimbalPitch = pitchDegrees
        };
        Items.Add(item);
        return item;
    }
    
    /// <summary>
    /// Add region of interest (camera always points here).
    /// </summary>
    public MissionItem AddROI(double lat, double lon, double altMeters)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.RegionOfInterest,
            Location = new GeoPoint(lat, lon, altMeters)
        };
        Items.Add(item);
        return item;
    }
    
    /// <summary>
    /// Add speed change command.
    /// </summary>
    public MissionItem AddSpeedChange(double speedMs)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.ChangeSpeed,
            Speed = speedMs
        };
        Items.Add(item);
        return item;
    }
    
    /// <summary>
    /// Add altitude change.
    /// </summary>
    public MissionItem AddAltitudeChange(double altMeters)
    {
        var item = new MissionItem
        {
            Sequence = Items.Count,
            Type = MissionItemType.ChangeAltitude,
            Location = new GeoPoint(0, 0, altMeters)
        };
        Items.Add(item);
        return item;
    }
    
    /// <summary>
    /// Remove a mission item.
    /// </summary>
    public void RemoveItem(int index)
    {
        if (index >= 0 && index < Items.Count)
        {
            Items.RemoveAt(index);
            ResequenceItems();
            UpdateStatistics();
        }
    }
    
    /// <summary>
    /// Move item to new position.
    /// </summary>
    public void MoveItem(int fromIndex, int toIndex)
    {
        if (fromIndex >= 0 && fromIndex < Items.Count && toIndex >= 0 && toIndex < Items.Count)
        {
            var item = Items[fromIndex];
            Items.RemoveAt(fromIndex);
            Items.Insert(toIndex, item);
            ResequenceItems();
            UpdateStatistics();
        }
    }
    
    /// <summary>
    /// Clear all items.
    /// </summary>
    public void Clear()
    {
        Items.Clear();
        UpdateStatistics();
    }
    
    /// <summary>
    /// Calculate mission statistics.
    /// </summary>
    public MissionStatistics CalculateStatistics()
    {
        var stats = new MissionStatistics();
        
        var navItems = Items.Where(i => i.Location != null && 
            (i.Type == MissionItemType.Waypoint || i.Type == MissionItemType.Takeoff || 
             i.Type == MissionItemType.Loiter || i.Type == MissionItemType.Orbit)).ToList();
        
        if (navItems.Count < 2)
        {
            return stats;
        }
        
        stats.WaypointCount = navItems.Count;
        
        // Calculate distances
        for (int i = 1; i < navItems.Count; i++)
        {
            double dist = CalculateDistance(navItems[i - 1].Location!, navItems[i].Location!);
            stats.TotalDistanceMeters += dist;
            
            // Altitude changes
            double altChange = navItems[i].Location!.Altitude - navItems[i - 1].Location!.Altitude;
            if (altChange > 0) stats.TotalClimbMeters += altChange;
            else stats.TotalDescentMeters += System.Math.Abs(altChange);
        }
        
        // Estimate time
        double avgSpeed = Settings.DefaultSpeed;
        stats.EstimatedTimeSeconds = stats.TotalDistanceMeters / avgSpeed;
        
        // Add hold times
        foreach (var item in Items.Where(i => i.HoldTime > 0))
        {
            stats.EstimatedTimeSeconds += item.HoldTime;
        }
        
        // Estimate battery usage (very rough)
        double avgCurrentAmps = 15; // Cruise current
        double flightTimeHours = stats.EstimatedTimeSeconds / 3600;
        stats.EstimatedBatteryUsagePercent = (avgCurrentAmps * flightTimeHours * 1000 / Settings.BatteryCapacityMah) * 100;
        
        // Bounding box
        if (navItems.Any())
        {
            stats.MinLatitude = navItems.Min(i => i.Location!.Latitude);
            stats.MaxLatitude = navItems.Max(i => i.Location!.Latitude);
            stats.MinLongitude = navItems.Min(i => i.Location!.Longitude);
            stats.MaxLongitude = navItems.Max(i => i.Location!.Longitude);
            stats.MaxAltitude = navItems.Max(i => i.Location!.Altitude);
        }
        
        return stats;
    }
    
    /// <summary>
    /// Export to MAVLink waypoint format.
    /// </summary>
    public List<Devices.Waypoint> ToMavlinkWaypoints()
    {
        var waypoints = new List<Devices.Waypoint>();
        int seq = 0;
        
        foreach (var item in Items)
        {
            var wp = item.Type switch
            {
                MissionItemType.Takeoff => new Devices.Waypoint
                {
                    Sequence = seq++,
                    Command = Devices.WaypointCommand.Takeoff,
                    Altitude = item.Location?.Altitude ?? Settings.DefaultAltitude
                },
                MissionItemType.Waypoint => new Devices.Waypoint
                {
                    Sequence = seq++,
                    Command = Devices.WaypointCommand.Waypoint,
                    Latitude = item.Location!.Latitude,
                    Longitude = item.Location.Longitude,
                    Altitude = item.Location.Altitude,
                    Param1 = item.HoldTime
                },
                MissionItemType.Loiter => new Devices.Waypoint
                {
                    Sequence = seq++,
                    Command = Devices.WaypointCommand.LoiterTime,
                    Latitude = item.Location!.Latitude,
                    Longitude = item.Location.Longitude,
                    Altitude = item.Location.Altitude,
                    Param1 = item.HoldTime
                },
                MissionItemType.Land => new Devices.Waypoint
                {
                    Sequence = seq++,
                    Command = Devices.WaypointCommand.Land,
                    Latitude = item.Location?.Latitude ?? 0,
                    Longitude = item.Location?.Longitude ?? 0
                },
                MissionItemType.ReturnToLaunch => new Devices.Waypoint
                {
                    Sequence = seq++,
                    Command = Devices.WaypointCommand.ReturnToLaunch
                },
                _ => null
            };
            
            if (wp != null)
                waypoints.Add(wp);
        }
        
        return waypoints;
    }
    
    /// <summary>
    /// Export to QGroundControl plan format (JSON).
    /// </summary>
    public string ToQgcPlanJson()
    {
        var plan = new
        {
            fileType = "Plan",
            geoFence = new { circles = Array.Empty<object>(), polygons = Array.Empty<object>() },
            groundStation = "ControlWorkbench",
            mission = new
            {
                cruiseSpeed = Settings.DefaultSpeed,
                firmwareType = 3, // ArduPilot
                hoverSpeed = Settings.DefaultSpeed,
                items = Items.Select((item, i) => new
                {
                    autoContinue = true,
                    command = GetQgcCommand(item.Type),
                    frame = 3, // GLOBAL_RELATIVE_ALT
                    @params = GetQgcParams(item),
                    type = "SimpleItem"
                }),
                plannedHomePosition = HomePosition != null ? new[] { HomePosition.Latitude, HomePosition.Longitude, HomePosition.Altitude } : null
            },
            version = 1
        };
        
        return System.Text.Json.JsonSerializer.Serialize(plan, new System.Text.Json.JsonSerializerOptions 
        { 
            WriteIndented = true 
        });
    }
    
    private int GetQgcCommand(MissionItemType type) => type switch
    {
        MissionItemType.Takeoff => 22,
        MissionItemType.Waypoint => 16,
        MissionItemType.Loiter => 19,
        MissionItemType.Orbit => 18,
        MissionItemType.Land => 21,
        MissionItemType.ReturnToLaunch => 20,
        MissionItemType.ChangeSpeed => 178,
        MissionItemType.CameraTrigger => 206,
        _ => 16
    };
    
    private double[] GetQgcParams(MissionItem item)
    {
        var p = new double[7];
        p[0] = item.HoldTime;
        p[1] = item.Radius;
        p[4] = item.Location?.Latitude ?? 0;
        p[5] = item.Location?.Longitude ?? 0;
        p[6] = item.Location?.Altitude ?? 0;
        return p;
    }
    
    private void ResequenceItems()
    {
        for (int i = 0; i < Items.Count; i++)
        {
            Items[i].Sequence = i;
        }
    }
    
    private void UpdateStatistics()
    {
        var stats = CalculateStatistics();
        StatisticsUpdated?.Invoke(stats);
    }
    
    private List<GeoPoint> GenerateSurveyGrid(GeoPoint corner1, GeoPoint corner2, double spacing, double heading)
    {
        var points = new List<GeoPoint>();
        
        // Convert to local coordinates
        double latDiff = corner2.Latitude - corner1.Latitude;
        double lonDiff = corner2.Longitude - corner1.Longitude;
        double latScale = 111320; // meters per degree
        double lonScale = 111320 * System.Math.Cos((corner1.Latitude + corner2.Latitude) / 2 * System.Math.PI / 180);
        
        double width = System.Math.Abs(lonDiff) * lonScale;
        double height = System.Math.Abs(latDiff) * latScale;
        
        int numLines = (int)System.Math.Ceiling(width / spacing);
        bool forward = true;
        
        for (int i = 0; i <= numLines; i++)
        {
            double x = i * spacing;
            double lonOffset = x / lonScale;
            double lon = corner1.Longitude + lonOffset;
            
            if (forward)
            {
                points.Add(new GeoPoint(corner1.Latitude, lon, corner1.Altitude));
                points.Add(new GeoPoint(corner2.Latitude, lon, corner1.Altitude));
            }
            else
            {
                points.Add(new GeoPoint(corner2.Latitude, lon, corner1.Altitude));
                points.Add(new GeoPoint(corner1.Latitude, lon, corner1.Altitude));
            }
            forward = !forward;
        }
        
        return points;
    }
    
    private double CalculateDistance(GeoPoint p1, GeoPoint p2)
    {
        // Haversine formula
        double R = 6371000; // Earth radius in meters
        double lat1 = p1.Latitude * System.Math.PI / 180;
        double lat2 = p2.Latitude * System.Math.PI / 180;
        double dLat = (p2.Latitude - p1.Latitude) * System.Math.PI / 180;
        double dLon = (p2.Longitude - p1.Longitude) * System.Math.PI / 180;
        
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1) * System.Math.Cos(lat2) * System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        double c = 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
        
        double horizontalDist = R * c;
        double altDiff = p2.Altitude - p1.Altitude;
        
        return System.Math.Sqrt(horizontalDist * horizontalDist + altDiff * altDiff);
    }
}

public class MissionItem
{
    public int Sequence { get; set; }
    public MissionItemType Type { get; set; }
    public GeoPoint? Location { get; set; }
    
    public double Speed { get; set; } = 5;
    public double HoldTime { get; set; } = 0;
    public double Radius { get; set; } = 0;
    public int Turns { get; set; } = 1;
    public double YawDegrees { get; set; } = double.NaN;
    
    public CameraTriggerType CameraTrigger { get; set; }
    public double CameraParam { get; set; }
    public double GimbalPitch { get; set; }
    
    public string Label { get; set; } = "";
    public string Notes { get; set; } = "";
}

public enum MissionItemType
{
    Takeoff,
    Waypoint,
    Land,
    ReturnToLaunch,
    Loiter,
    Orbit,
    Spline,
    
    ChangeSpeed,
    ChangeAltitude,
    
    CameraTrigger,
    GimbalControl,
    RegionOfInterest,
    
    Delay,
    SetServo,
    SetRelay,
    
    ConditionYaw,
    ConditionDelay
}

public enum CameraTriggerType
{
    TriggerOnce,
    TriggerByDistance,
    TriggerByTime,
    TriggerStart,
    TriggerStop,
    RecordVideoStart,
    RecordVideoStop
}

public class GeoPoint
{
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }
    
    public GeoPoint() { }
    
    public GeoPoint(double lat, double lon, double alt = 0)
    {
        Latitude = lat;
        Longitude = lon;
        Altitude = alt;
    }
    
    public override string ToString() => $"{Latitude:F6}, {Longitude:F6} @ {Altitude:F1}m";
}

public class MissionSettings
{
    public double DefaultAltitude { get; set; } = 50;       // meters
    public double DefaultSpeed { get; set; } = 5;           // m/s
    public double MaxAltitude { get; set; } = 120;          // meters
    public double MaxDistance { get; set; } = 5000;         // meters from home
    public double BatteryCapacityMah { get; set; } = 5000;
    public double LandingAltitude { get; set; } = 0;
    public bool AutoContinue { get; set; } = true;
}

public class MissionStatistics
{
    public int WaypointCount { get; set; }
    public double TotalDistanceMeters { get; set; }
    public double TotalClimbMeters { get; set; }
    public double TotalDescentMeters { get; set; }
    public double EstimatedTimeSeconds { get; set; }
    public double EstimatedBatteryUsagePercent { get; set; }
    
    public double MinLatitude { get; set; }
    public double MaxLatitude { get; set; }
    public double MinLongitude { get; set; }
    public double MaxLongitude { get; set; }
    public double MaxAltitude { get; set; }
    
    public TimeSpan EstimatedTime => TimeSpan.FromSeconds(EstimatedTimeSeconds);
    
    public override string ToString() => 
        $"Waypoints: {WaypointCount}, Distance: {TotalDistanceMeters:F0}m, Time: {EstimatedTime:mm\\:ss}";
}

