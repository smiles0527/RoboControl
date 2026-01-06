using ControlWorkbench.Drone.Mission;

namespace ControlWorkbench.Drone.Survey;

/// <summary>
/// Generates survey patterns for mapping, inspection, and photogrammetry missions.
/// </summary>
public class SurveyPatternGenerator
{
    /// <summary>
    /// Generates a lawn-mower grid pattern for area coverage.
    /// </summary>
    public static List<GeoPoint> GenerateGridPattern(
        GeoPoint corner1,
        GeoPoint corner2,
        double altitude,
        double lineSpacing,
        double headingDegrees = 0,
        bool crossHatch = false)
    {
        var points = new List<GeoPoint>();
        
        // Calculate bounds and dimensions
        double minLat = System.Math.Min(corner1.Latitude, corner2.Latitude);
        double maxLat = System.Math.Max(corner1.Latitude, corner2.Latitude);
        double minLon = System.Math.Min(corner1.Longitude, corner2.Longitude);
        double maxLon = System.Math.Max(corner1.Longitude, corner2.Longitude);
        
        // Convert to local coordinates (meters)
        double latScale = 111320.0;
        double lonScale = 111320.0 * System.Math.Cos((minLat + maxLat) / 2 * System.Math.PI / 180);
        
        double width = (maxLon - minLon) * lonScale;
        double height = (maxLat - minLat) * latScale;
        
        // Rotate pattern by heading
        double headingRad = headingDegrees * System.Math.PI / 180;
        
        // Number of lines
        int numLines = (int)System.Math.Ceiling(width / lineSpacing) + 1;
        bool forward = true;
        
        for (int i = 0; i < numLines; i++)
        {
            double x = i * lineSpacing;
            
            if (forward)
            {
                // Start of line
                var (lat1, lon1) = LocalToGeo(x, 0, minLat, minLon, headingRad, latScale, lonScale);
                // End of line
                var (lat2, lon2) = LocalToGeo(x, height, minLat, minLon, headingRad, latScale, lonScale);
                
                points.Add(new GeoPoint(lat1, lon1, altitude));
                points.Add(new GeoPoint(lat2, lon2, altitude));
            }
            else
            {
                var (lat1, lon1) = LocalToGeo(x, height, minLat, minLon, headingRad, latScale, lonScale);
                var (lat2, lon2) = LocalToGeo(x, 0, minLat, minLon, headingRad, latScale, lonScale);
                
                points.Add(new GeoPoint(lat1, lon1, altitude));
                points.Add(new GeoPoint(lat2, lon2, altitude));
            }
            
            forward = !forward;
        }
        
        // Cross-hatch: add perpendicular lines
        if (crossHatch)
        {
            int numCrossLines = (int)System.Math.Ceiling(height / lineSpacing) + 1;
            forward = true;
            
            for (int i = 0; i < numCrossLines; i++)
            {
                double y = i * lineSpacing;
                
                if (forward)
                {
                    var (lat1, lon1) = LocalToGeo(0, y, minLat, minLon, headingRad, latScale, lonScale);
                    var (lat2, lon2) = LocalToGeo(width, y, minLat, minLon, headingRad, latScale, lonScale);
                    
                    points.Add(new GeoPoint(lat1, lon1, altitude));
                    points.Add(new GeoPoint(lat2, lon2, altitude));
                }
                else
                {
                    var (lat1, lon1) = LocalToGeo(width, y, minLat, minLon, headingRad, latScale, lonScale);
                    var (lat2, lon2) = LocalToGeo(0, y, minLat, minLon, headingRad, latScale, lonScale);
                    
                    points.Add(new GeoPoint(lat1, lon1, altitude));
                    points.Add(new GeoPoint(lat2, lon2, altitude));
                }
                
                forward = !forward;
            }
        }
        
        return points;
    }

    /// <summary>
    /// Generates a spiral pattern centered on a point.
    /// </summary>
    public static List<GeoPoint> GenerateSpiralPattern(
        GeoPoint center,
        double altitude,
        double startRadius,
        double endRadius,
        double radialSpacing,
        int pointsPerRevolution = 36)
    {
        var points = new List<GeoPoint>();
        
        double totalRevolutions = (endRadius - startRadius) / radialSpacing;
        int totalPoints = (int)(totalRevolutions * pointsPerRevolution);
        
        double lonScale = 111320.0 * System.Math.Cos(center.Latitude * System.Math.PI / 180);
        double latScale = 111320.0;
        
        for (int i = 0; i < totalPoints; i++)
        {
            double t = (double)i / totalPoints;
            double angle = t * totalRevolutions * 2 * System.Math.PI;
            double radius = startRadius + t * (endRadius - startRadius);
            
            double x = radius * System.Math.Cos(angle);
            double y = radius * System.Math.Sin(angle);
            
            double lat = center.Latitude + y / latScale;
            double lon = center.Longitude + x / lonScale;
            
            points.Add(new GeoPoint(lat, lon, altitude));
        }
        
        return points;
    }

    /// <summary>
    /// Generates an orbit pattern around a point of interest.
    /// </summary>
    public static List<GeoPoint> GenerateOrbitPattern(
        GeoPoint center,
        double altitude,
        double radius,
        int numPoints,
        bool clockwise = true,
        double startAngleDegrees = 0)
    {
        var points = new List<GeoPoint>();
        
        double lonScale = 111320.0 * System.Math.Cos(center.Latitude * System.Math.PI / 180);
        double latScale = 111320.0;
        
        double startAngle = startAngleDegrees * System.Math.PI / 180;
        double direction = clockwise ? -1 : 1;
        
        for (int i = 0; i < numPoints; i++)
        {
            double angle = startAngle + direction * (2 * System.Math.PI * i / numPoints);
            
            double x = radius * System.Math.Cos(angle);
            double y = radius * System.Math.Sin(angle);
            
            double lat = center.Latitude + y / latScale;
            double lon = center.Longitude + x / lonScale;
            
            points.Add(new GeoPoint(lat, lon, altitude));
        }
        
        // Close the loop
        points.Add(new GeoPoint(points[0].Latitude, points[0].Longitude, altitude));
        
        return points;
    }

    /// <summary>
    /// Generates a corridor/linear inspection pattern.
    /// </summary>
    public static List<GeoPoint> GenerateCorridorPattern(
        List<GeoPoint> centerline,
        double altitude,
        double corridorWidth,
        int passesPerSegment = 3)
    {
        if (centerline.Count < 2)
            return [];
        
        var points = new List<GeoPoint>();
        
        double lonScale = 111320.0 * System.Math.Cos(centerline[0].Latitude * System.Math.PI / 180);
        double latScale = 111320.0;
        
        for (int seg = 0; seg < centerline.Count - 1; seg++)
        {
            var p1 = centerline[seg];
            var p2 = centerline[seg + 1];
            
            // Direction along segment
            double dx = (p2.Longitude - p1.Longitude) * lonScale;
            double dy = (p2.Latitude - p1.Latitude) * latScale;
            double length = System.Math.Sqrt(dx * dx + dy * dy);
            
            // Perpendicular direction
            double perpX = -dy / length;
            double perpY = dx / length;
            
            // Generate passes
            for (int pass = 0; pass < passesPerSegment; pass++)
            {
                double offset = (pass - (passesPerSegment - 1) / 2.0) * (corridorWidth / (passesPerSegment - 1));
                if (passesPerSegment == 1) offset = 0;
                
                double offsetLat = perpY * offset / latScale;
                double offsetLon = perpX * offset / lonScale;
                
                if (pass % 2 == 0)
                {
                    points.Add(new GeoPoint(p1.Latitude + offsetLat, p1.Longitude + offsetLon, altitude));
                    points.Add(new GeoPoint(p2.Latitude + offsetLat, p2.Longitude + offsetLon, altitude));
                }
                else
                {
                    points.Add(new GeoPoint(p2.Latitude + offsetLat, p2.Longitude + offsetLon, altitude));
                    points.Add(new GeoPoint(p1.Latitude + offsetLat, p1.Longitude + offsetLon, altitude));
                }
            }
        }
        
        return points;
    }

    /// <summary>
    /// Generates a facade/vertical structure inspection pattern.
    /// </summary>
    public static List<GeoPoint> GenerateFacadePattern(
        GeoPoint basePoint1,
        GeoPoint basePoint2,
        double standoffDistance,
        double minAltitude,
        double maxAltitude,
        double verticalSpacing,
        double horizontalSpacing)
    {
        var points = new List<GeoPoint>();
        
        double lonScale = 111320.0 * System.Math.Cos(basePoint1.Latitude * System.Math.PI / 180);
        double latScale = 111320.0;
        
        // Direction along facade
        double dx = (basePoint2.Longitude - basePoint1.Longitude) * lonScale;
        double dy = (basePoint2.Latitude - basePoint1.Latitude) * latScale;
        double facadeLength = System.Math.Sqrt(dx * dx + dy * dy);
        
        // Normalize
        dx /= facadeLength;
        dy /= facadeLength;
        
        // Perpendicular (outward from facade)
        double perpX = -dy;
        double perpY = dx;
        
        // Number of vertical and horizontal passes
        int numVertical = (int)System.Math.Ceiling((maxAltitude - minAltitude) / verticalSpacing) + 1;
        int numHorizontal = (int)System.Math.Ceiling(facadeLength / horizontalSpacing) + 1;
        
        bool goingUp = true;
        
        for (int h = 0; h < numHorizontal; h++)
        {
            double alongFacade = h * horizontalSpacing;
            
            // Position along facade
            double baseLat = basePoint1.Latitude + dy * alongFacade / latScale;
            double baseLon = basePoint1.Longitude + dx * alongFacade / lonScale;
            
            // Offset by standoff distance
            double flyLat = baseLat + perpY * standoffDistance / latScale;
            double flyLon = baseLon + perpX * standoffDistance / lonScale;
            
            // Vertical passes
            if (goingUp)
            {
                for (int v = 0; v < numVertical; v++)
                {
                    double alt = minAltitude + v * verticalSpacing;
                    points.Add(new GeoPoint(flyLat, flyLon, alt));
                }
            }
            else
            {
                for (int v = numVertical - 1; v >= 0; v--)
                {
                    double alt = minAltitude + v * verticalSpacing;
                    points.Add(new GeoPoint(flyLat, flyLon, alt));
                }
            }
            
            goingUp = !goingUp;
        }
        
        return points;
    }

    /// <summary>
    /// Generates a photogrammetry double-grid pattern for 3D reconstruction.
    /// </summary>
    public static List<GeoPoint> GeneratePhotogrammetryPattern(
        GeoPoint corner1,
        GeoPoint corner2,
        double altitude,
        double gsd,          // Ground Sample Distance in meters/pixel
        double cameraFocalLength,  // mm
        double sensorWidth,   // mm
        double frontOverlap = 0.75,
        double sideOverlap = 0.65)
    {
        // Calculate coverage per image
        double imageWidthGround = altitude * sensorWidth / cameraFocalLength;
        
        // Calculate spacing
        double frontSpacing = imageWidthGround * (1 - frontOverlap);
        double sideSpacing = imageWidthGround * (1 - sideOverlap);
        
        // Generate main grid (0 degrees)
        var mainGrid = GenerateGridPattern(corner1, corner2, altitude, sideSpacing, 0, false);
        
        // Generate cross grid (90 degrees)
        var crossGrid = GenerateGridPattern(corner1, corner2, altitude, sideSpacing, 90, false);
        
        // Combine
        var combined = new List<GeoPoint>();
        combined.AddRange(mainGrid);
        combined.AddRange(crossGrid);
        
        return combined;
    }

    /// <summary>
    /// Optimizes waypoint order using Nearest Neighbor heuristic.
    /// </summary>
    public static List<GeoPoint> OptimizeWaypointOrder(List<GeoPoint> waypoints, GeoPoint start)
    {
        if (waypoints.Count <= 1)
            return waypoints;
        
        var optimized = new List<GeoPoint>();
        var remaining = new List<GeoPoint>(waypoints);
        var current = start;
        
        while (remaining.Count > 0)
        {
            // Find nearest
            int nearestIdx = 0;
            double nearestDist = double.MaxValue;
            
            for (int i = 0; i < remaining.Count; i++)
            {
                double dist = HaversineDistance(current, remaining[i]);
                if (dist < nearestDist)
                {
                    nearestDist = dist;
                    nearestIdx = i;
                }
            }
            
            optimized.Add(remaining[nearestIdx]);
            current = remaining[nearestIdx];
            remaining.RemoveAt(nearestIdx);
        }
        
        return optimized;
    }

    /// <summary>
    /// Applies terrain following by adjusting altitudes based on elevation data.
    /// </summary>
    public static List<GeoPoint> ApplyTerrainFollowing(
        List<GeoPoint> waypoints,
        Func<double, double, double> getTerrainElevation,
        double aglHeight)
    {
        return waypoints.Select(wp =>
        {
            double terrainElev = getTerrainElevation(wp.Latitude, wp.Longitude);
            return new GeoPoint(wp.Latitude, wp.Longitude, terrainElev + aglHeight);
        }).ToList();
    }

    /// <summary>
    /// Calculates camera trigger positions for photogrammetry.
    /// </summary>
    public static List<(GeoPoint position, double yaw, double pitch)> CalculateCameraTriggerPositions(
        List<GeoPoint> flightPath,
        double triggerDistance,
        GeoPoint? roiPoint = null)
    {
        var triggers = new List<(GeoPoint, double, double)>();
        
        double accumulatedDistance = 0;
        
        for (int i = 0; i < flightPath.Count; i++)
        {
            bool shouldTrigger = false;
            
            if (i == 0)
            {
                shouldTrigger = true;
            }
            else
            {
                double segmentDist = HaversineDistance(flightPath[i - 1], flightPath[i]);
                accumulatedDistance += segmentDist;
                
                if (accumulatedDistance >= triggerDistance)
                {
                    shouldTrigger = true;
                    accumulatedDistance = 0;
                }
            }
            
            if (shouldTrigger)
            {
                var pos = flightPath[i];
                
                double yaw = 0;
                double pitch = -90; // Nadir
                
                if (roiPoint != null)
                {
                    // Point camera at ROI
                    yaw = CalculateBearing(pos, roiPoint);
                    double horizontalDist = HaversineDistance(pos, roiPoint);
                    double verticalDist = pos.Altitude - roiPoint.Altitude;
                    pitch = -System.Math.Atan2(verticalDist, horizontalDist) * 180 / System.Math.PI;
                }
                else if (i < flightPath.Count - 1)
                {
                    // Point along flight direction
                    yaw = CalculateBearing(pos, flightPath[i + 1]);
                }
                
                triggers.Add((pos, yaw, pitch));
            }
        }
        
        return triggers;
    }

    /// <summary>
    /// Estimates flight time for a mission.
    /// </summary>
    public static TimeSpan EstimateFlightTime(
        List<GeoPoint> waypoints,
        double cruiseSpeed,
        double climbRate,
        double descentRate,
        double turnRadius = 0)
    {
        double totalTime = 0;
        
        for (int i = 1; i < waypoints.Count; i++)
        {
            var p1 = waypoints[i - 1];
            var p2 = waypoints[i];
            
            double horizontalDist = HaversineDistance(p1, p2);
            double verticalDist = p2.Altitude - p1.Altitude;
            
            // Horizontal time
            double horizontalTime = horizontalDist / cruiseSpeed;
            
            // Vertical time
            double verticalTime = 0;
            if (verticalDist > 0)
                verticalTime = verticalDist / climbRate;
            else if (verticalDist < 0)
                verticalTime = -verticalDist / descentRate;
            
            // Total segment time (taking the longer of horizontal/vertical)
            double segmentTime = System.Math.Max(horizontalTime, verticalTime);
            
            // Add turn time if applicable
            if (turnRadius > 0 && i < waypoints.Count - 1)
            {
                // Estimate turn angle
                double bearing1 = CalculateBearing(p1, p2);
                double bearing2 = CalculateBearing(p2, waypoints[i + 1]);
                double turnAngle = System.Math.Abs(bearing2 - bearing1);
                if (turnAngle > 180) turnAngle = 360 - turnAngle;
                
                double turnArcLength = turnRadius * turnAngle * System.Math.PI / 180;
                segmentTime += turnArcLength / cruiseSpeed;
            }
            
            totalTime += segmentTime;
        }
        
        return TimeSpan.FromSeconds(totalTime);
    }

    /// <summary>
    /// Estimates battery consumption for a mission.
    /// </summary>
    public static double EstimateBatteryConsumption(
        List<GeoPoint> waypoints,
        double cruiseSpeed,
        DroneSpecs specs)
    {
        double totalEnergy = 0; // Wh
        
        for (int i = 1; i < waypoints.Count; i++)
        {
            var p1 = waypoints[i - 1];
            var p2 = waypoints[i];
            
            double horizontalDist = HaversineDistance(p1, p2);
            double verticalDist = p2.Altitude - p1.Altitude;
            double flightTime = horizontalDist / cruiseSpeed; // seconds
            
            // Power consumption model
            double power;
            if (verticalDist > 0)
            {
                // Climbing
                power = specs.CruisePowerWatts * 1.3 + specs.DroneWeight * 9.81 * verticalDist / flightTime;
            }
            else if (verticalDist < 0)
            {
                // Descending
                power = specs.CruisePowerWatts * 0.7;
            }
            else
            {
                // Level flight
                power = specs.CruisePowerWatts;
            }
            
            totalEnergy += power * flightTime / 3600; // Convert to Wh
        }
        
        return (totalEnergy / specs.BatteryCapacityWh) * 100; // Percentage
    }

    private static (double lat, double lon) LocalToGeo(
        double localX, double localY,
        double baseLat, double baseLon,
        double headingRad,
        double latScale, double lonScale)
    {
        // Rotate by heading
        double rx = localX * System.Math.Cos(headingRad) - localY * System.Math.Sin(headingRad);
        double ry = localX * System.Math.Sin(headingRad) + localY * System.Math.Cos(headingRad);
        
        double lat = baseLat + ry / latScale;
        double lon = baseLon + rx / lonScale;
        
        return (lat, lon);
    }

    private static double HaversineDistance(GeoPoint p1, GeoPoint p2)
    {
        const double R = 6371000; // Earth radius in meters
        
        double dLat = (p2.Latitude - p1.Latitude) * System.Math.PI / 180;
        double dLon = (p2.Longitude - p1.Longitude) * System.Math.PI / 180;
        
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(p1.Latitude * System.Math.PI / 180) * 
                   System.Math.Cos(p2.Latitude * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        
        return R * 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
    }

    private static double CalculateBearing(GeoPoint from, GeoPoint to)
    {
        double dLon = (to.Longitude - from.Longitude) * System.Math.PI / 180;
        double lat1 = from.Latitude * System.Math.PI / 180;
        double lat2 = to.Latitude * System.Math.PI / 180;
        
        double x = System.Math.Sin(dLon) * System.Math.Cos(lat2);
        double y = System.Math.Cos(lat1) * System.Math.Sin(lat2) - 
                   System.Math.Sin(lat1) * System.Math.Cos(lat2) * System.Math.Cos(dLon);
        
        double bearing = System.Math.Atan2(x, y) * 180 / System.Math.PI;
        return (bearing + 360) % 360;
    }
}

/// <summary>
/// Drone specifications for mission planning calculations.
/// </summary>
public class DroneSpecs
{
    public double DroneWeight { get; set; } = 1.5;           // kg
    public double MaxPayload { get; set; } = 0.5;            // kg
    public double MaxSpeed { get; set; } = 15;               // m/s
    public double CruiseSpeed { get; set; } = 8;             // m/s
    public double MaxClimbRate { get; set; } = 5;            // m/s
    public double MaxDescentRate { get; set; } = 3;          // m/s
    public double BatteryCapacityWh { get; set; } = 99;      // Wh (typical 6S 5000mAh)
    public double CruisePowerWatts { get; set; } = 300;      // Watts at cruise
    public double HoverPowerWatts { get; set; } = 350;       // Watts hovering
    public double WindResistance { get; set; } = 10;         // m/s max wind
    public double TurnRadius { get; set; } = 5;              // meters
    public double MinAltitude { get; set; } = 2;             // meters AGL
    public double MaxAltitude { get; set; } = 120;           // meters AGL (regulatory)
    
    public static DroneSpecs DJIMavic3 => new()
    {
        DroneWeight = 0.895,
        MaxPayload = 0,
        MaxSpeed = 21,
        CruiseSpeed = 12,
        MaxClimbRate = 8,
        MaxDescentRate = 6,
        BatteryCapacityWh = 77,
        CruisePowerWatts = 200,
        HoverPowerWatts = 250,
        WindResistance = 12,
        TurnRadius = 3
    };
    
    public static DroneSpecs CustomQuad5Inch => new()
    {
        DroneWeight = 0.65,
        MaxPayload = 0.2,
        MaxSpeed = 40,
        CruiseSpeed = 15,
        MaxClimbRate = 15,
        MaxDescentRate = 10,
        BatteryCapacityWh = 22, // 4S 1500mAh
        CruisePowerWatts = 150,
        HoverPowerWatts = 180,
        WindResistance = 15,
        TurnRadius = 2
    };
    
    public static DroneSpecs InspectionHex => new()
    {
        DroneWeight = 3.5,
        MaxPayload = 1.5,
        MaxSpeed = 12,
        CruiseSpeed = 6,
        MaxClimbRate = 4,
        MaxDescentRate = 2,
        BatteryCapacityWh = 266, // 6S 10000mAh x2
        CruisePowerWatts = 450,
        HoverPowerWatts = 550,
        WindResistance = 8,
        TurnRadius = 8
    };
}

/// <summary>
/// GeoJSON exporter for mission visualization.
/// </summary>
public static class GeoJsonExporter
{
    public static string ExportMission(
        List<GeoPoint> waypoints,
        GeoPoint? homePosition,
        string name = "Mission")
    {
        var features = new List<object>();
        
        // Add home marker
        if (homePosition != null)
        {
            features.Add(new
            {
                type = "Feature",
                properties = new { name = "Home", markerType = "home" },
                geometry = new
                {
                    type = "Point",
                    coordinates = new[] { homePosition.Longitude, homePosition.Latitude, homePosition.Altitude }
                }
            });
        }
        
        // Add waypoint markers
        for (int i = 0; i < waypoints.Count; i++)
        {
            var wp = waypoints[i];
            features.Add(new
            {
                type = "Feature",
                properties = new { name = $"WP{i + 1}", sequence = i + 1, altitude = wp.Altitude },
                geometry = new
                {
                    type = "Point",
                    coordinates = new[] { wp.Longitude, wp.Latitude, wp.Altitude }
                }
            });
        }
        
        // Add flight path line
        if (waypoints.Count >= 2)
        {
            features.Add(new
            {
                type = "Feature",
                properties = new { name = "Flight Path" },
                geometry = new
                {
                    type = "LineString",
                    coordinates = waypoints.Select(wp => new[] { wp.Longitude, wp.Latitude, wp.Altitude }).ToArray()
                }
            });
        }
        
        var geoJson = new
        {
            type = "FeatureCollection",
            name,
            features
        };
        
        return System.Text.Json.JsonSerializer.Serialize(geoJson, new System.Text.Json.JsonSerializerOptions
        {
            WriteIndented = true
        });
    }
    
    public static string ExportSurveyArea(
        GeoPoint corner1,
        GeoPoint corner2,
        List<GeoPoint> flightPath,
        string name = "Survey")
    {
        var features = new List<object>();
        
        // Survey boundary polygon
        features.Add(new
        {
            type = "Feature",
            properties = new { name = "Survey Area", type = "boundary" },
            geometry = new
            {
                type = "Polygon",
                coordinates = new[]
                {
                    new[]
                    {
                        new[] { corner1.Longitude, corner1.Latitude },
                        new[] { corner2.Longitude, corner1.Latitude },
                        new[] { corner2.Longitude, corner2.Latitude },
                        new[] { corner1.Longitude, corner2.Latitude },
                        new[] { corner1.Longitude, corner1.Latitude }
                    }
                }
            }
        });
        
        // Flight path
        features.Add(new
        {
            type = "Feature",
            properties = new { name = "Flight Path", type = "path" },
            geometry = new
            {
                type = "LineString",
                coordinates = flightPath.Select(wp => new[] { wp.Longitude, wp.Latitude, wp.Altitude }).ToArray()
            }
        });
        
        var geoJson = new
        {
            type = "FeatureCollection",
            name,
            features
        };
        
        return System.Text.Json.JsonSerializer.Serialize(geoJson, new System.Text.Json.JsonSerializerOptions
        {
            WriteIndented = true
        });
    }
}
