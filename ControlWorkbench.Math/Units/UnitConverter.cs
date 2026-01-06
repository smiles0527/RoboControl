using SysMath = System.Math;

namespace ControlWorkbench.Math.Units;

/// <summary>
/// Unit conversion utilities for robotics and control applications.
/// </summary>
public static class UnitConverter
{
    // Angle conversions
    public static double DegToRad(double degrees) => degrees * SysMath.PI / 180.0;
    public static double RadToDeg(double radians) => radians * 180.0 / SysMath.PI;
    public static double RpmToRadPerSec(double rpm) => rpm * 2.0 * SysMath.PI / 60.0;
    public static double RadPerSecToRpm(double radPerSec) => radPerSec * 60.0 / (2.0 * SysMath.PI);
    
    // Angular rate conversions
    public static double DegPerSecToRadPerSec(double degPerSec) => DegToRad(degPerSec);
    public static double RadPerSecToDegPerSec(double radPerSec) => RadToDeg(radPerSec);
    
    // Length conversions
    public static double MetersToFeet(double meters) => meters * 3.28084;
    public static double FeetToMeters(double feet) => feet / 3.28084;
    public static double MetersToInches(double meters) => meters * 39.3701;
    public static double InchesToMeters(double inches) => inches / 39.3701;
    public static double MillimetersToInches(double mm) => mm / 25.4;
    public static double InchesToMillimeters(double inches) => inches * 25.4;
    
    // Velocity conversions
    public static double MpsToKph(double mps) => mps * 3.6;
    public static double KphToMps(double kph) => kph / 3.6;
    public static double MpsToMph(double mps) => mps * 2.23694;
    public static double MphToMps(double mph) => mph / 2.23694;
    public static double KnotsToMps(double knots) => knots * 0.514444;
    public static double MpsToKnots(double mps) => mps / 0.514444;
    
    // Acceleration conversions
    public static double GToMps2(double g) => g * 9.80665;
    public static double Mps2ToG(double mps2) => mps2 / 9.80665;
    
    // Angular acceleration
    public static double DegPerSec2ToRadPerSec2(double degPerSec2) => DegToRad(degPerSec2);
    public static double RadPerSec2ToDegPerSec2(double radPerSec2) => RadToDeg(radPerSec2);
    
    // Pressure conversions
    public static double PaToHPa(double pa) => pa / 100.0;
    public static double HPaToPa(double hPa) => hPa * 100.0;
    public static double PaToBar(double pa) => pa / 100000.0;
    public static double BarToPa(double bar) => bar * 100000.0;
    public static double PsiToPa(double psi) => psi * 6894.757;
    public static double PaToPsi(double pa) => pa / 6894.757;
    
    // Temperature conversions
    public static double CelsiusToFahrenheit(double c) => c * 9.0 / 5.0 + 32.0;
    public static double FahrenheitToCelsius(double f) => (f - 32.0) * 5.0 / 9.0;
    public static double CelsiusToKelvin(double c) => c + 273.15;
    public static double KelvinToCelsius(double k) => k - 273.15;
    
    // Frequency/Time conversions
    public static double HzToRadPerSec(double hz) => hz * 2.0 * SysMath.PI;
    public static double RadPerSecToHz(double radPerSec) => radPerSec / (2.0 * SysMath.PI);
    public static double FrequencyToPeriod(double hz) => 1.0 / hz;
    public static double PeriodToFrequency(double period) => 1.0 / period;
    
    // Mass conversions
    public static double KgToLbs(double kg) => kg * 2.20462;
    public static double LbsToKg(double lbs) => lbs / 2.20462;
    
    // Torque conversions  
    public static double NmToFtLbs(double nm) => nm * 0.737562;
    public static double FtLbsToNm(double ftLbs) => ftLbs / 0.737562;
    public static double NmToOzIn(double nm) => nm * 141.612;
    public static double OzInToNm(double ozIn) => ozIn / 141.612;
    
    // Power conversions
    public static double WattsToHorsepower(double watts) => watts / 745.7;
    public static double HorsepowerToWatts(double hp) => hp * 745.7;
    
    // Magnetic field conversions
    public static double GaussToTesla(double gauss) => gauss * 1e-4;
    public static double TeslaToGauss(double tesla) => tesla * 1e4;
    public static double GaussToMicroTesla(double gauss) => gauss * 100.0;
    public static double MicroTeslaToGauss(double uT) => uT / 100.0;
}

/// <summary>
/// Robotics-specific calculations and formulas.
/// </summary>
public static class RoboticsFormulas
{
    /// <summary>
    /// Calculate motor torque from electrical parameters.
    /// </summary>
    public static double MotorTorque(double current, double kt) => current * kt;
    
    /// <summary>
    /// Calculate motor back-EMF voltage.
    /// </summary>
    public static double BackEmf(double angularVelocity, double ke) => angularVelocity * ke;
    
    /// <summary>
    /// Calculate motor terminal voltage.
    /// V = IR + Ke*?
    /// </summary>
    public static double MotorVoltage(double current, double resistance, double angularVelocity, double ke)
        => current * resistance + BackEmf(angularVelocity, ke);
    
    /// <summary>
    /// Calculate motor angular velocity from terminal voltage.
    /// ? = (V - IR) / Ke
    /// </summary>
    public static double MotorSpeed(double voltage, double current, double resistance, double ke)
        => (voltage - current * resistance) / ke;
    
    /// <summary>
    /// Calculate no-load motor speed.
    /// </summary>
    public static double NoLoadSpeed(double voltage, double ke) => voltage / ke;
    
    /// <summary>
    /// Calculate stall torque.
    /// </summary>
    public static double StallTorque(double voltage, double resistance, double kt)
        => voltage / resistance * kt;
    
    /// <summary>
    /// Calculate motor power output.
    /// </summary>
    public static double MotorPower(double torque, double angularVelocity) => torque * angularVelocity;
    
    /// <summary>
    /// Calculate gear ratio from input and output values.
    /// </summary>
    public static double GearRatio(double inputTeeth, double outputTeeth) => outputTeeth / inputTeeth;
    
    /// <summary>
    /// Apply gear ratio to torque (output = input * ratio).
    /// </summary>
    public static double GearTorque(double inputTorque, double ratio, double efficiency = 1.0)
        => inputTorque * ratio * efficiency;
    
    /// <summary>
    /// Apply gear ratio to speed (output = input / ratio).
    /// </summary>
    public static double GearSpeed(double inputSpeed, double ratio) => inputSpeed / ratio;
    
    /// <summary>
    /// Calculate wheel linear velocity from angular velocity.
    /// </summary>
    public static double WheelLinearVelocity(double angularVelocity, double radius)
        => angularVelocity * radius;
    
    /// <summary>
    /// Calculate wheel angular velocity from linear velocity.
    /// </summary>
    public static double WheelAngularVelocity(double linearVelocity, double radius)
        => linearVelocity / radius;
    
    /// <summary>
    /// Calculate required motor torque for acceleration.
    /// ? = I*? + friction
    /// </summary>
    public static double AccelerationTorque(
        double inertia, 
        double angularAcceleration, 
        double frictionTorque = 0)
        => inertia * angularAcceleration + frictionTorque;
    
    /// <summary>
    /// Calculate payload capacity for a multi-rotor.
    /// </summary>
    public static double MulticopterPayload(
        int numMotors,
        double maxThrustPerMotor,
        double vehicleMass,
        double thrustMargin = 0.5) // 50% margin for control authority
        => numMotors * maxThrustPerMotor * (1 - thrustMargin) - vehicleMass;
    
    /// <summary>
    /// Calculate multicopter hover throttle percentage.
    /// </summary>
    public static double HoverThrottle(double totalMass, int numMotors, double maxThrustPerMotor)
    {
        double requiredThrust = totalMass * 9.81;
        double maxThrust = numMotors * maxThrustPerMotor;
        return requiredThrust / maxThrust * 100.0;
    }
    
    /// <summary>
    /// Calculate estimated flight time for a battery-powered vehicle.
    /// </summary>
    public static double FlightTime(
        double batteryCapacityMah,
        double averageCurrentA,
        double safetyMargin = 0.8) // Don't drain below 20%
        => batteryCapacityMah / 1000.0 * safetyMargin / averageCurrentA * 60.0; // minutes
    
    /// <summary>
    /// Calculate battery C-rate.
    /// </summary>
    public static double CRate(double currentA, double capacityMah) => currentA / (capacityMah / 1000.0);
    
    /// <summary>
    /// Calculate range for ground vehicle.
    /// </summary>
    public static double GroundVehicleRange(
        double batteryCapacityWh,
        double averagePowerW,
        double averageSpeedMps)
    {
        double runtime = batteryCapacityWh / averagePowerW; // hours
        return runtime * averageSpeedMps * 3600.0; // meters
    }
}

/// <summary>
/// GPS and navigation calculations.
/// </summary>
public static class NavigationFormulas
{
    private const double EarthRadiusM = 6371000.0;
    private const double EarthRadiusKm = 6371.0;
    
    /// <summary>
    /// Calculate Haversine distance between two GPS coordinates.
    /// </summary>
    public static double HaversineDistance(
        double lat1Deg, double lon1Deg,
        double lat2Deg, double lon2Deg)
    {
        double lat1 = UnitConverter.DegToRad(lat1Deg);
        double lat2 = UnitConverter.DegToRad(lat2Deg);
        double dLat = UnitConverter.DegToRad(lat2Deg - lat1Deg);
        double dLon = UnitConverter.DegToRad(lon2Deg - lon1Deg);
        
        double a = SysMath.Sin(dLat / 2) * SysMath.Sin(dLat / 2) +
                   SysMath.Cos(lat1) * SysMath.Cos(lat2) *
                   SysMath.Sin(dLon / 2) * SysMath.Sin(dLon / 2);
        double c = 2 * SysMath.Atan2(SysMath.Sqrt(a), SysMath.Sqrt(1 - a));
        
        return EarthRadiusM * c;
    }
    
    /// <summary>
    /// Calculate bearing from one GPS coordinate to another.
    /// </summary>
    public static double Bearing(
        double lat1Deg, double lon1Deg,
        double lat2Deg, double lon2Deg)
    {
        double lat1 = UnitConverter.DegToRad(lat1Deg);
        double lat2 = UnitConverter.DegToRad(lat2Deg);
        double dLon = UnitConverter.DegToRad(lon2Deg - lon1Deg);
        
        double y = SysMath.Sin(dLon) * SysMath.Cos(lat2);
        double x = SysMath.Cos(lat1) * SysMath.Sin(lat2) -
                   SysMath.Sin(lat1) * SysMath.Cos(lat2) * SysMath.Cos(dLon);
        
        double bearing = SysMath.Atan2(y, x);
        return (UnitConverter.RadToDeg(bearing) + 360) % 360;
    }
    
    /// <summary>
    /// Calculate destination point given start, bearing, and distance.
    /// </summary>
    public static (double Lat, double Lon) DestinationPoint(
        double lat1Deg, double lon1Deg,
        double bearingDeg, double distanceM)
    {
        double lat1 = UnitConverter.DegToRad(lat1Deg);
        double lon1 = UnitConverter.DegToRad(lon1Deg);
        double bearing = UnitConverter.DegToRad(bearingDeg);
        double d = distanceM / EarthRadiusM;
        
        double lat2 = SysMath.Asin(
            SysMath.Sin(lat1) * SysMath.Cos(d) +
            SysMath.Cos(lat1) * SysMath.Sin(d) * SysMath.Cos(bearing));
        
        double lon2 = lon1 + SysMath.Atan2(
            SysMath.Sin(bearing) * SysMath.Sin(d) * SysMath.Cos(lat1),
            SysMath.Cos(d) - SysMath.Sin(lat1) * SysMath.Sin(lat2));
        
        return (UnitConverter.RadToDeg(lat2), UnitConverter.RadToDeg(lon2));
    }
    
    /// <summary>
    /// Convert GPS to local ENU (East-North-Up) coordinates.
    /// </summary>
    public static (double E, double N, double U) GpsToEnu(
        double latDeg, double lonDeg, double altM,
        double refLatDeg, double refLonDeg, double refAltM)
    {
        // Convert to ECEF first
        var (x1, y1, z1) = GpsToEcef(latDeg, lonDeg, altM);
        var (x0, y0, z0) = GpsToEcef(refLatDeg, refLonDeg, refAltM);
        
        double dx = x1 - x0;
        double dy = y1 - y0;
        double dz = z1 - z0;
        
        double lat = UnitConverter.DegToRad(refLatDeg);
        double lon = UnitConverter.DegToRad(refLonDeg);
        
        double e = -SysMath.Sin(lon) * dx + SysMath.Cos(lon) * dy;
        double n = -SysMath.Sin(lat) * SysMath.Cos(lon) * dx -
                    SysMath.Sin(lat) * SysMath.Sin(lon) * dy +
                    SysMath.Cos(lat) * dz;
        double u = SysMath.Cos(lat) * SysMath.Cos(lon) * dx +
                   SysMath.Cos(lat) * SysMath.Sin(lon) * dy +
                   SysMath.Sin(lat) * dz;
        
        return (e, n, u);
    }
    
    /// <summary>
    /// Convert GPS to ECEF (Earth-Centered Earth-Fixed) coordinates.
    /// </summary>
    public static (double X, double Y, double Z) GpsToEcef(
        double latDeg, double lonDeg, double altM)
    {
        const double a = 6378137.0; // WGS84 semi-major axis
        const double e2 = 0.00669437999014; // WGS84 first eccentricity squared
        
        double lat = UnitConverter.DegToRad(latDeg);
        double lon = UnitConverter.DegToRad(lonDeg);
        
        double sinLat = SysMath.Sin(lat);
        double cosLat = SysMath.Cos(lat);
        double sinLon = SysMath.Sin(lon);
        double cosLon = SysMath.Cos(lon);
        
        double N = a / SysMath.Sqrt(1 - e2 * sinLat * sinLat);
        
        double x = (N + altM) * cosLat * cosLon;
        double y = (N + altM) * cosLat * sinLon;
        double z = (N * (1 - e2) + altM) * sinLat;
        
        return (x, y, z);
    }
    
    /// <summary>
    /// Calculate magnetic declination at a location (simplified model).
    /// For accurate values, use WMM or IGRF model.
    /// </summary>
    public static double MagneticDeclination(double latDeg, double lonDeg, int year = 2024)
    {
        // Simplified dipole model - for real applications use WMM
        // This is just an approximation for the continental US
        double declination = -0.1 * lonDeg + 0.05 * latDeg;
        return declination;
    }
    
    /// <summary>
    /// Apply magnetic declination to convert magnetic heading to true heading.
    /// </summary>
    public static double MagneticToTrue(double magneticHeadingDeg, double declinationDeg)
        => (magneticHeadingDeg + declinationDeg + 360) % 360;
    
    /// <summary>
    /// Apply magnetic declination to convert true heading to magnetic heading.
    /// </summary>
    public static double TrueToMagnetic(double trueHeadingDeg, double declinationDeg)
        => (trueHeadingDeg - declinationDeg + 360) % 360;
    
    /// <summary>
    /// Calculate ground speed and track from velocity components.
    /// </summary>
    public static (double Speed, double Track) VelocityToSpeedTrack(double vNorth, double vEast)
    {
        double speed = SysMath.Sqrt(vNorth * vNorth + vEast * vEast);
        double track = SysMath.Atan2(vEast, vNorth);
        track = (UnitConverter.RadToDeg(track) + 360) % 360;
        return (speed, track);
    }
    
    /// <summary>
    /// Calculate velocity components from speed and track.
    /// </summary>
    public static (double VNorth, double VEast) SpeedTrackToVelocity(double speed, double trackDeg)
    {
        double track = UnitConverter.DegToRad(trackDeg);
        return (speed * SysMath.Cos(track), speed * SysMath.Sin(track));
    }
}

/// <summary>
/// Attitude and rotation utilities.
/// </summary>
public static class AttitudeFormulas
{
    /// <summary>
    /// Calculate roll angle from accelerometer (stationary).
    /// </summary>
    public static double RollFromAccel(double ay, double az)
        => SysMath.Atan2(ay, az);
    
    /// <summary>
    /// Calculate pitch angle from accelerometer (stationary).
    /// </summary>
    public static double PitchFromAccel(double ax, double ay, double az)
        => SysMath.Atan2(-ax, SysMath.Sqrt(ay * ay + az * az));
    
    /// <summary>
    /// Calculate yaw/heading from magnetometer (level).
    /// </summary>
    public static double YawFromMag(double mx, double my)
        => SysMath.Atan2(-my, mx);
    
    /// <summary>
    /// Calculate heading with tilt compensation.
    /// </summary>
    public static double TiltCompensatedHeading(
        double mx, double my, double mz,
        double rollRad, double pitchRad)
    {
        double cosRoll = SysMath.Cos(rollRad);
        double sinRoll = SysMath.Sin(rollRad);
        double cosPitch = SysMath.Cos(pitchRad);
        double sinPitch = SysMath.Sin(pitchRad);
        
        double Xh = mx * cosPitch + mz * sinPitch;
        double Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
        
        double heading = SysMath.Atan2(-Yh, Xh);
        return (UnitConverter.RadToDeg(heading) + 360) % 360;
    }
    
    /// <summary>
    /// Normalize angle to [-?, ?] range.
    /// </summary>
    public static double NormalizeAngleRad(double angle)
    {
        while (angle > SysMath.PI) angle -= 2 * SysMath.PI;
        while (angle < -SysMath.PI) angle += 2 * SysMath.PI;
        return angle;
    }
    
    /// <summary>
    /// Normalize angle to [0, 360) range.
    /// </summary>
    public static double NormalizeAngleDeg(double angle)
    {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }
    
    /// <summary>
    /// Calculate shortest angular difference (signed).
    /// </summary>
    public static double AngularDifferenceRad(double target, double current)
    {
        double diff = target - current;
        return NormalizeAngleRad(diff);
    }
    
    /// <summary>
    /// Calculate shortest angular difference in degrees (signed).
    /// </summary>
    public static double AngularDifferenceDeg(double target, double current)
    {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }
    
    /// <summary>
    /// Convert Euler angles (ZYX convention) to quaternion.
    /// </summary>
    public static (double W, double X, double Y, double Z) EulerToQuaternion(
        double rollRad, double pitchRad, double yawRad)
    {
        double cr = SysMath.Cos(rollRad / 2);
        double sr = SysMath.Sin(rollRad / 2);
        double cp = SysMath.Cos(pitchRad / 2);
        double sp = SysMath.Sin(pitchRad / 2);
        double cy = SysMath.Cos(yawRad / 2);
        double sy = SysMath.Sin(yawRad / 2);
        
        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;
        
        return (w, x, y, z);
    }
    
    /// <summary>
    /// Convert quaternion to Euler angles (ZYX convention).
    /// </summary>
    public static (double Roll, double Pitch, double Yaw) QuaternionToEuler(
        double w, double x, double y, double z)
    {
        // Roll (x-axis rotation)
        double sinrCosp = 2 * (w * x + y * z);
        double cosrCosp = 1 - 2 * (x * x + y * y);
        double roll = SysMath.Atan2(sinrCosp, cosrCosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        double pitch;
        if (SysMath.Abs(sinp) >= 1)
            pitch = SysMath.CopySign(SysMath.PI / 2, sinp); // Gimbal lock
        else
            pitch = SysMath.Asin(sinp);
        
        // Yaw (z-axis rotation)
        double sinyCosp = 2 * (w * z + x * y);
        double cosyCosp = 1 - 2 * (y * y + z * z);
        double yaw = SysMath.Atan2(sinyCosp, cosyCosp);
        
        return (roll, pitch, yaw);
    }
    
    /// <summary>
    /// Normalize quaternion to unit length.
    /// </summary>
    public static (double W, double X, double Y, double Z) NormalizeQuaternion(
        double w, double x, double y, double z)
    {
        double norm = SysMath.Sqrt(w * w + x * x + y * y + z * z);
        return (w / norm, x / norm, y / norm, z / norm);
    }
}
