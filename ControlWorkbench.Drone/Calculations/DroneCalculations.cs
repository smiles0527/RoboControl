namespace ControlWorkbench.Drone.Calculations;

/// <summary>
/// Drone performance calculations and motor mixing.
/// </summary>
public static class DroneCalculations
{
    /// <summary>
    /// Calculate estimated hover throttle percentage.
    /// </summary>
    public static double CalculateHoverThrottle(double totalWeightGrams, double maxThrustGrams)
    {
        // At hover, thrust = weight
        // Hover throttle ? sqrt(weight / maxThrust) for most motors
        // (Due to non-linear thrust curve)
        return System.Math.Sqrt(totalWeightGrams / maxThrustGrams) * 100;
    }
    
    /// <summary>
    /// Calculate thrust to weight ratio.
    /// </summary>
    public static double CalculateThrustToWeight(double totalWeightGrams, double maxThrustGrams)
    {
        return maxThrustGrams / totalWeightGrams;
    }
    
    /// <summary>
    /// Estimate flight time based on battery and power consumption.
    /// </summary>
    public static double EstimateFlightTimeMinutes(
        double batteryCapacityMah,
        double avgPowerWatts,
        double batteryVoltage,
        double safetyMarginPercent = 20)
    {
        // Available capacity after safety margin
        double usableCapacity = batteryCapacityMah * (1 - safetyMarginPercent / 100);
        
        // Average current draw
        double avgCurrentAmps = avgPowerWatts / batteryVoltage;
        
        // Flight time in minutes
        return usableCapacity / (avgCurrentAmps * 1000) * 60;
    }
    
    /// <summary>
    /// Calculate motor RPM from KV rating and voltage.
    /// </summary>
    public static double CalculateMotorRpm(int kvRating, double voltage, double loadFactor = 0.85)
    {
        // RPM = KV * Voltage * load factor (typically 85% under prop load)
        return kvRating * voltage * loadFactor;
    }
    
    /// <summary>
    /// Estimate propeller tip speed (important for efficiency).
    /// </summary>
    public static (double speedMs, double mach) CalculatePropTipSpeed(
        double rpm, double propDiameterInches)
    {
        double diameterMeters = propDiameterInches * 0.0254;
        double circumference = System.Math.PI * diameterMeters;
        double rps = rpm / 60;
        double tipSpeedMs = circumference * rps;
        double mach = tipSpeedMs / 343; // Speed of sound at sea level
        
        return (tipSpeedMs, mach);
    }
    
    /// <summary>
    /// Estimate thrust per motor using simple thrust model.
    /// </summary>
    public static double EstimateMotorThrust(
        double rpm,
        double propDiameterInches,
        double propPitchInches,
        double airDensity = 1.225)
    {
        // Simplified thrust model: T = Ct * ? * n² * D?
        // Where Ct is thrust coefficient (varies with prop design)
        double Ct = 0.1; // Typical for quadcopter props
        double diameterM = propDiameterInches * 0.0254;
        double rps = rpm / 60;
        
        double thrust = Ct * airDensity * System.Math.Pow(rps, 2) * System.Math.Pow(diameterM, 4);
        return thrust * 1000 / 9.81; // Convert to grams
    }
    
    /// <summary>
    /// Calculate power draw from current and voltage.
    /// </summary>
    public static double CalculatePowerWatts(double currentAmps, double voltage)
    {
        return currentAmps * voltage;
    }
    
    /// <summary>
    /// Estimate range based on speed and flight time.
    /// </summary>
    public static double EstimateRangeKm(double speedMs, double flightTimeMinutes)
    {
        return speedMs * flightTimeMinutes * 60 / 1000;
    }
}

/// <summary>
/// Motor mixing for different frame configurations.
/// </summary>
public class MotorMixer
{
    private readonly double[,] _mixTable;
    private readonly int _motorCount;
    
    public MotorMixer(FrameConfiguration config)
    {
        _mixTable = GetMixTable(config);
        _motorCount = _mixTable.GetLength(0);
    }
    
    /// <summary>
    /// Calculate motor outputs from control inputs.
    /// </summary>
    /// <param name="throttle">0-1 range</param>
    /// <param name="roll">-1 to 1 range</param>
    /// <param name="pitch">-1 to 1 range</param>
    /// <param name="yaw">-1 to 1 range</param>
    /// <returns>Motor outputs 0-1 for each motor</returns>
    public double[] Mix(double throttle, double roll, double pitch, double yaw)
    {
        var outputs = new double[_motorCount];
        
        for (int i = 0; i < _motorCount; i++)
        {
            outputs[i] = throttle 
                + roll * _mixTable[i, 0] 
                + pitch * _mixTable[i, 1] 
                + yaw * _mixTable[i, 2];
            
            outputs[i] = System.Math.Clamp(outputs[i], 0, 1);
        }
        
        return outputs;
    }
    
    /// <summary>
    /// Apply airmode mixing (maintains control authority at low throttle).
    /// </summary>
    public double[] MixWithAirmode(double throttle, double roll, double pitch, double yaw, double minThrottle = 0.1)
    {
        var outputs = Mix(throttle, roll, pitch, yaw);
        
        // Find min and max
        double min = outputs.Min();
        double max = outputs.Max();
        
        // Boost throttle if needed to maintain control
        if (min < minThrottle)
        {
            double boost = minThrottle - min;
            for (int i = 0; i < outputs.Length; i++)
            {
                outputs[i] += boost;
            }
        }
        
        // Scale down if exceeding max
        if (max > 1)
        {
            double scale = (1 - minThrottle) / (max - minThrottle);
            for (int i = 0; i < outputs.Length; i++)
            {
                outputs[i] = minThrottle + (outputs[i] - minThrottle) * scale;
            }
        }
        
        return outputs;
    }
    
    private static double[,] GetMixTable(FrameConfiguration config)
    {
        return config switch
        {
            FrameConfiguration.QuadX => new double[,]
            {
                // Motor: Roll, Pitch, Yaw
                { -1, +1, +1 },  // Front Right (CW)
                { -1, -1, -1 },  // Back Right (CCW)
                { +1, -1, +1 },  // Back Left (CW)
                { +1, +1, -1 }   // Front Left (CCW)
            },
            FrameConfiguration.QuadPlus => new double[,]
            {
                { 0, +1, +1 },   // Front (CW)
                { -1, 0, -1 },   // Right (CCW)
                { 0, -1, +1 },   // Back (CW)
                { +1, 0, -1 }    // Left (CCW)
            },
            FrameConfiguration.HexX => new double[,]
            {
                { -0.5, +1, +1 },    // Front Right (CW)
                { -1, 0, -1 },       // Right (CCW)
                { -0.5, -1, +1 },    // Back Right (CW)
                { +0.5, -1, -1 },    // Back Left (CCW)
                { +1, 0, +1 },       // Left (CW)
                { +0.5, +1, -1 }     // Front Left (CCW)
            },
            FrameConfiguration.OctoX => new double[,]
            {
                { -0.41, +1, +1 },   // Motor 1
                { -1, +0.41, -1 },
                { -1, -0.41, +1 },
                { -0.41, -1, -1 },
                { +0.41, -1, +1 },
                { +1, -0.41, -1 },
                { +1, +0.41, +1 },
                { +0.41, +1, -1 }
            },
            _ => new double[,]
            {
                { -1, +1, +1 },
                { -1, -1, -1 },
                { +1, -1, +1 },
                { +1, +1, -1 }
            }
        };
    }
}

public enum FrameConfiguration
{
    QuadX,
    QuadPlus,
    QuadH,
    HexX,
    HexPlus,
    OctoX,
    OctoPlus,
    Y6,
    Tricopter
}

/// <summary>
/// Geofence calculations.
/// </summary>
public static class GeofenceCalculator
{
    /// <summary>
    /// Check if point is inside circular geofence.
    /// </summary>
    public static bool IsInsideCircle(
        double lat, double lon,
        double centerLat, double centerLon, double radiusMeters)
    {
        double distance = CalculateDistance(lat, lon, centerLat, centerLon);
        return distance <= radiusMeters;
    }
    
    /// <summary>
    /// Check if point is inside polygon geofence.
    /// </summary>
    public static bool IsInsidePolygon(
        double lat, double lon,
        List<(double lat, double lon)> polygon)
    {
        // Ray casting algorithm
        bool inside = false;
        int j = polygon.Count - 1;
        
        for (int i = 0; i < polygon.Count; i++)
        {
            if ((polygon[i].lon > lon) != (polygon[j].lon > lon) &&
                lat < (polygon[j].lat - polygon[i].lat) * (lon - polygon[i].lon) / 
                      (polygon[j].lon - polygon[i].lon) + polygon[i].lat)
            {
                inside = !inside;
            }
            j = i;
        }
        
        return inside;
    }
    
    /// <summary>
    /// Calculate distance between two GPS coordinates (Haversine).
    /// </summary>
    public static double CalculateDistance(
        double lat1, double lon1,
        double lat2, double lon2)
    {
        const double R = 6371000; // Earth radius in meters
        
        double lat1Rad = lat1 * System.Math.PI / 180;
        double lat2Rad = lat2 * System.Math.PI / 180;
        double dLat = (lat2 - lat1) * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1Rad) * System.Math.Cos(lat2Rad) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        double c = 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
        
        return R * c;
    }
    
    /// <summary>
    /// Calculate bearing from one point to another.
    /// </summary>
    public static double CalculateBearing(
        double lat1, double lon1,
        double lat2, double lon2)
    {
        double lat1Rad = lat1 * System.Math.PI / 180;
        double lat2Rad = lat2 * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        
        double y = System.Math.Sin(dLon) * System.Math.Cos(lat2Rad);
        double x = System.Math.Cos(lat1Rad) * System.Math.Sin(lat2Rad) -
                   System.Math.Sin(lat1Rad) * System.Math.Cos(lat2Rad) * System.Math.Cos(dLon);
        
        double bearing = System.Math.Atan2(y, x) * 180 / System.Math.PI;
        return (bearing + 360) % 360;
    }
    
    /// <summary>
    /// Calculate destination point given start, bearing, and distance.
    /// </summary>
    public static (double lat, double lon) CalculateDestination(
        double lat, double lon,
        double bearingDegrees, double distanceMeters)
    {
        const double R = 6371000;
        
        double latRad = lat * System.Math.PI / 180;
        double lonRad = lon * System.Math.PI / 180;
        double bearing = bearingDegrees * System.Math.PI / 180;
        double d = distanceMeters / R;
        
        double lat2 = System.Math.Asin(System.Math.Sin(latRad) * System.Math.Cos(d) +
                                 System.Math.Cos(latRad) * System.Math.Sin(d) * System.Math.Cos(bearing));
        double lon2 = lonRad + System.Math.Atan2(
            System.Math.Sin(bearing) * System.Math.Sin(d) * System.Math.Cos(latRad),
            System.Math.Cos(d) - System.Math.Sin(latRad) * System.Math.Sin(lat2));
        
        return (lat2 * 180 / System.Math.PI, lon2 * 180 / System.Math.PI);
    }
}

/// <summary>
/// Coordinate system conversions.
/// </summary>
public static class CoordinateConversions
{
    /// <summary>
    /// Convert GPS to local ENU (East-North-Up) coordinates.
    /// </summary>
    public static (double east, double north, double up) GpsToEnu(
        double lat, double lon, double alt,
        double refLat, double refLon, double refAlt)
    {
        // Convert to ECEF first
        var (x1, y1, z1) = GpsToEcef(lat, lon, alt);
        var (x0, y0, z0) = GpsToEcef(refLat, refLon, refAlt);
        
        // Delta ECEF
        double dx = x1 - x0;
        double dy = y1 - y0;
        double dz = z1 - z0;
        
        // Rotation matrix
        double latRad = refLat * System.Math.PI / 180;
        double lonRad = refLon * System.Math.PI / 180;
        
        double sinLat = System.Math.Sin(latRad);
        double cosLat = System.Math.Cos(latRad);
        double sinLon = System.Math.Sin(lonRad);
        double cosLon = System.Math.Cos(lonRad);
        
        double east = -sinLon * dx + cosLon * dy;
        double north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
        double up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;
        
        return (east, north, up);
    }
    
    /// <summary>
    /// Convert GPS to ECEF coordinates.
    /// </summary>
    public static (double x, double y, double z) GpsToEcef(
        double lat, double lon, double alt)
    {
        const double a = 6378137.0;          // WGS84 semi-major axis
        const double f = 1 / 298.257223563;  // WGS84 flattening
        double e2 = 2 * f - f * f;           // Eccentricity squared
        
        double latRad = lat * System.Math.PI / 180;
        double lonRad = lon * System.Math.PI / 180;
        
        double sinLat = System.Math.Sin(latRad);
        double cosLat = System.Math.Cos(latRad);
        double sinLon = System.Math.Sin(lonRad);
        double cosLon = System.Math.Cos(lonRad);
        
        double N = a / System.Math.Sqrt(1 - e2 * sinLat * sinLat);
        
        double x = (N + alt) * cosLat * cosLon;
        double y = (N + alt) * cosLat * sinLon;
        double z = (N * (1 - e2) + alt) * sinLat;
        
        return (x, y, z);
    }
    
    /// <summary>
    /// Convert body rates to Euler angle rates.
    /// </summary>
    public static (double rollRate, double pitchRate, double yawRate) BodyToEulerRates(
        double p, double q, double r,  // Body rates (rad/s)
        double roll, double pitch)      // Current attitude (rad)
    {
        double sinRoll = System.Math.Sin(roll);
        double cosRoll = System.Math.Cos(roll);
        double cosPitch = System.Math.Cos(pitch);
        double tanPitch = System.Math.Tan(pitch);
        
        double rollRate = p + sinRoll * tanPitch * q + cosRoll * tanPitch * r;
        double pitchRate = cosRoll * q - sinRoll * r;
        double yawRate = (sinRoll / cosPitch) * q + (cosRoll / cosPitch) * r;
        
        return (rollRate, pitchRate, yawRate);
    }
}

/// <summary>
/// Battery calculations and management.
/// </summary>
public static class BatteryCalculations
{
    /// <summary>
    /// Estimate remaining flight time from current consumption.
    /// </summary>
    public static double EstimateRemainingTimeMinutes(
        double remainingCapacityMah,
        double currentAmps)
    {
        if (currentAmps <= 0) return double.PositiveInfinity;
        return remainingCapacityMah / (currentAmps * 1000) * 60;
    }
    
    /// <summary>
    /// Calculate battery percentage from voltage (LiPo).
    /// </summary>
    public static double VoltageToPercent(double voltage, int cellCount)
    {
        double cellVoltage = voltage / cellCount;
        
        // LiPo voltage curve approximation
        if (cellVoltage >= 4.2) return 100;
        if (cellVoltage <= 3.3) return 0;
        
        // Piece-wise linear approximation
        if (cellVoltage >= 4.0)
            return 90 + (cellVoltage - 4.0) / 0.2 * 10;
        if (cellVoltage >= 3.8)
            return 40 + (cellVoltage - 3.8) / 0.2 * 50;
        if (cellVoltage >= 3.6)
            return 15 + (cellVoltage - 3.6) / 0.2 * 25;
        
        return (cellVoltage - 3.3) / 0.3 * 15;
    }
    
    /// <summary>
    /// Check if battery is safe to fly.
    /// </summary>
    public static (bool safe, string reason) CheckBatterySafety(
        double voltage, int cellCount,
        double tempCelsius,
        bool puffy)
    {
        double cellVoltage = voltage / cellCount;
        
        if (puffy)
            return (false, "Battery is puffy - dispose safely");
        
        if (cellVoltage < 3.5)
            return (false, $"Cell voltage too low ({cellVoltage:F2}V)");
        
        if (cellVoltage > 4.25)
            return (false, $"Cell voltage too high ({cellVoltage:F2}V) - possible overcharge");
        
        if (tempCelsius < 10)
            return (false, $"Battery too cold ({tempCelsius:F0}°C) - warm before flight");
        
        if (tempCelsius > 50)
            return (false, $"Battery too hot ({tempCelsius:F0}°C) - let cool down");
        
        return (true, "Battery OK");
    }
}

