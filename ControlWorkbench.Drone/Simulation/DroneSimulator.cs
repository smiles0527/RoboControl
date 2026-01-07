using ControlWorkbench.Drone.Devices;

namespace ControlWorkbench.Drone.Simulation;

/// <summary>
/// Comprehensive multirotor drone physics simulation.
/// Models aerodynamics, motor dynamics, battery, and sensors.
/// </summary>
public class DroneSimulator
{
    // State
    public DroneState State { get; private set; } = new();
    public DroneSpecs Specs { get; set; } = new();
    
    // Environment
    public WindModel Wind { get; set; } = new();
    public double AirDensity { get; set; } = 1.225;  // kg/m³ at sea level
    public double Gravity { get; set; } = 9.81;      // m/s²
    
    // Controller outputs
    private double[] _motorCommands = new double[8];
    
    // Simulation
    private readonly Random _random = new();
    private double _time = 0;
    
    // Smoothed telemetry values for filtering noise
    private DroneTelemetry _smoothedTelemetry = new();
    private bool _firstTelemetry = true;
    private const double SmoothingFactor = 0.15; // Lower = smoother, higher = more responsive
    
    public event Action<DroneState>? StateUpdated;
    public event Action<string>? LogMessage;
    
    public void Reset(double latitude = 0, double longitude = 0, double altitude = 0, double heading = 0)
    {
        State = new DroneState
        {
            Latitude = latitude,
            Longitude = longitude,
            Altitude = altitude,
            Yaw = heading * System.Math.PI / 180,
            BatteryVoltage = Specs.Battery.TotalVoltageMax,
            BatteryRemaining = 100
        };
        _time = 0;
        _motorCommands = new double[Specs.Motors.Length];
    }
    
    /// <summary>
    /// Set motor throttle commands (0-1 for each motor).
    /// </summary>
    public void SetMotorCommands(double[] commands)
    {
        for (int i = 0; i < System.Math.Min(commands.Length, _motorCommands.Length); i++)
        {
            _motorCommands[i] = System.Math.Clamp(commands[i], 0, 1);
        }
    }
    
    /// <summary>
    /// Set attitude and throttle commands (for attitude controllers).
    /// </summary>
    public void SetAttitudeCommand(double roll, double pitch, double yaw, double throttle)
    {
        // Simple mixer for quad-X
        double r = roll;
        double p = pitch;
        double y = yaw;
        double t = throttle;
        
        if (Specs.Motors.Length >= 4)
        {
            // Quad-X mixing
            _motorCommands[0] = System.Math.Clamp(t - r + p + y, 0, 1); // Front Right
            _motorCommands[1] = System.Math.Clamp(t - r - p - y, 0, 1); // Back Right
            _motorCommands[2] = System.Math.Clamp(t + r - p + y, 0, 1); // Back Left
            _motorCommands[3] = System.Math.Clamp(t + r + p - y, 0, 1); // Front Left
        }
    }
    
    /// <summary>
    /// Step simulation forward by dt seconds.
    /// </summary>
    public void Step(double dt)
    {
        _time += dt;
        
        // Calculate forces and moments
        var (forces, moments) = CalculateForcesAndMoments();
        
        // Add gravity
        forces.Z += Specs.WeightGrams / 1000 * Gravity;
        
        // Add wind
        var windForce = Wind.CalculateForce(State.Altitude, Specs);
        forces.X += windForce.X;
        forces.Y += windForce.Y;
        forces.Z += windForce.Z;
        
        // Calculate accelerations
        double mass = Specs.WeightGrams / 1000; // kg
        double Ixx = 0.01; // kg·m² (approximate)
        double Iyy = 0.01;
        double Izz = 0.02;
        
        // Linear acceleration in body frame
        double axBody = forces.X / mass;
        double ayBody = forces.Y / mass;
        double azBody = forces.Z / mass;
        
        // Angular acceleration
        double pDot = moments.X / Ixx;
        double qDot = moments.Y / Iyy;
        double rDot = moments.Z / Izz;
        
        // Update angular velocities
        State.RollRate += pDot * dt;
        State.PitchRate += qDot * dt;
        State.YawRate += rDot * dt;
        
        // Apply damping
        State.RollRate *= 0.99;
        State.PitchRate *= 0.99;
        State.YawRate *= 0.99;
        
        // Update attitudes
        State.Roll += State.RollRate * dt;
        State.Pitch += State.PitchRate * dt;
        State.Yaw += State.YawRate * dt;
        
        // Normalize yaw
        while (State.Yaw > System.Math.PI) State.Yaw -= 2 * System.Math.PI;
        while (State.Yaw < -System.Math.PI) State.Yaw += 2 * System.Math.PI;
        
        // Rotate body accelerations to world frame
        var (axWorld, ayWorld, azWorld) = RotateBodyToWorld(axBody, ayBody, azBody, State.Roll, State.Pitch, State.Yaw);
        
        // Update velocities (NED frame)
        State.VelocityNorth += axWorld * dt;
        State.VelocityEast += ayWorld * dt;
        State.VelocityDown += (azWorld + Gravity) * dt;
        
        // Apply drag
        double speed = System.Math.Sqrt(State.VelocityNorth * State.VelocityNorth + 
                                  State.VelocityEast * State.VelocityEast + 
                                  State.VelocityDown * State.VelocityDown);
        double dragCoeff = 0.01;
        if (speed > 0.1)
        {
            double dragForce = dragCoeff * speed * speed;
            State.VelocityNorth -= dragForce * State.VelocityNorth / speed * dt / mass;
            State.VelocityEast -= dragForce * State.VelocityEast / speed * dt / mass;
            State.VelocityDown -= dragForce * State.VelocityDown / speed * dt / mass;
        }
        
        // Update position
        // Convert velocity to lat/lon change (approximate)
        double metersPerDegreeLat = 111320;
        double metersPerDegreeLon = 111320 * System.Math.Cos(State.Latitude * System.Math.PI / 180);
        
        State.Latitude += State.VelocityNorth * dt / metersPerDegreeLat;
        State.Longitude += State.VelocityEast * dt / metersPerDegreeLon;
        State.Altitude -= State.VelocityDown * dt;
        
        // Ground collision
        if (State.Altitude < 0)
        {
            State.Altitude = 0;
            State.VelocityDown = 0;
            State.VelocityNorth *= 0.5;
            State.VelocityEast *= 0.5;
        }
        
        // Update battery
        UpdateBattery(dt);
        
        // Update derived values
        State.GroundSpeed = System.Math.Sqrt(State.VelocityNorth * State.VelocityNorth + 
                                       State.VelocityEast * State.VelocityEast);
        State.ClimbRate = -State.VelocityDown;
        State.Heading = State.Yaw * 180 / System.Math.PI;
        if (State.Heading < 0) State.Heading += 360;
        
        State.Timestamp = DateTimeOffset.UtcNow;
        StateUpdated?.Invoke(State);
    }
    
    private (Vector3 forces, Vector3 moments) CalculateForcesAndMoments()
    {
        var forces = new Vector3();
        var moments = new Vector3();
        
        double totalThrust = 0;
        double thrustPerMotor = (Specs.MaxThrustGrams / 1000 * Gravity) / Specs.Motors.Length;
        
        for (int i = 0; i < Specs.Motors.Length && i < _motorCommands.Length; i++)
        {
            var motor = Specs.Motors[i];
            double thrust = _motorCommands[i] * thrustPerMotor;
            totalThrust += thrust;
            
            // Position in meters (convert from normalized)
            double armLength = Specs.ArmLengthMm / 1000;
            double posX = motor.PositionX * armLength;
            double posY = motor.PositionY * armLength;
            
            // Thrust is negative Z (up in NED)
            forces.Z -= thrust;
            
            // Moments from thrust
            moments.X += thrust * posY;  // Roll moment
            moments.Y -= thrust * posX;  // Pitch moment
            
            // Yaw moment from motor torque reaction
            double motorTorque = thrust * 0.01; // Approximate torque coefficient
            if (motor.Clockwise)
                moments.Z -= motorTorque;
            else
                moments.Z += motorTorque;
        }
        
        // Store motor RPM estimate
        State.MotorOutputs = _motorCommands.Select(c => (int)(c * 100)).ToArray();
        
        return (forces, moments);
    }
    
    private (double x, double y, double z) RotateBodyToWorld(double xb, double yb, double zb, double roll, double pitch, double yaw)
    {
        double cr = System.Math.Cos(roll);
        double sr = System.Math.Sin(roll);
        double cp = System.Math.Cos(pitch);
        double sp = System.Math.Sin(pitch);
        double cy = System.Math.Cos(yaw);
        double sy = System.Math.Sin(yaw);
        
        double x = xb * (cp * cy) + yb * (sr * sp * cy - cr * sy) + zb * (cr * sp * cy + sr * sy);
        double y = xb * (cp * sy) + yb * (sr * sp * sy + cr * cy) + zb * (cr * sp * sy - sr * cy);
        double z = xb * (-sp) + yb * (sr * cp) + zb * (cr * cp);
        
        return (x, y, z);
    }
    
    private void UpdateBattery(double dt)
    {
        // Estimate current draw based on motor commands
        double totalThrottle = _motorCommands.Sum();
        double maxCurrentPerMotor = 30; // Amps at full throttle
        double idleCurrent = 1; // Electronics
        
        double current = idleCurrent + (totalThrottle / Specs.Motors.Length) * maxCurrentPerMotor * Specs.Motors.Length;
        
        // mAh consumed
        double mahConsumed = current * dt / 3.6;
        State.BatteryConsumed += mahConsumed;
        
        // Update remaining
        State.BatteryRemaining = System.Math.Max(0, 100 - (State.BatteryConsumed / Specs.Battery.CapacityMah * 100));
        
        // Voltage sag under load
        double sagPerAmp = 0.01; // Volts per amp
        State.BatteryCurrent = current;
        State.BatteryVoltage = Specs.Battery.TotalVoltageMax * (State.BatteryRemaining / 100) - current * sagPerAmp;
    }
    
    /// <summary>
    /// Add sensor noise to telemetry readings.
    /// </summary>
    public DroneTelemetry GetNoisyTelemetry()
    {
        var rawTelem = new DroneTelemetry
        {
            Timestamp = State.Timestamp,
            
            // Attitude with gyro noise (reduced noise levels)
            Roll = State.Roll * 180 / System.Math.PI + GaussianNoise(0.05),
            Pitch = State.Pitch * 180 / System.Math.PI + GaussianNoise(0.05),
            Yaw = State.Yaw * 180 / System.Math.PI + GaussianNoise(0.1),
            
            RollRate = State.RollRate * 180 / System.Math.PI + GaussianNoise(0.2),
            PitchRate = State.PitchRate * 180 / System.Math.PI + GaussianNoise(0.2),
            YawRate = State.YawRate * 180 / System.Math.PI + GaussianNoise(0.2),
            
            // GPS with typical noise
            Latitude = State.Latitude + GaussianNoise(0.0000005),
            Longitude = State.Longitude + GaussianNoise(0.0000005),
            AltitudeMSL = State.Altitude + GaussianNoise(0.2),
            AltitudeRelative = State.Altitude + GaussianNoise(0.1),
            
            // Velocity with GPS noise
            VelocityNorth = State.VelocityNorth + GaussianNoise(0.05),
            VelocityEast = State.VelocityEast + GaussianNoise(0.05),
            VelocityDown = State.VelocityDown + GaussianNoise(0.05),
            GroundSpeed = State.GroundSpeed + GaussianNoise(0.05),
            ClimbRate = State.ClimbRate + GaussianNoise(0.05),
            
            Heading = State.Heading + GaussianNoise(0.5),
            
            // Battery (very low noise - ADC readings are stable)
            BatteryVoltage = State.BatteryVoltage + GaussianNoise(0.005),
            BatteryCurrent = State.BatteryCurrent + GaussianNoise(0.05),
            BatteryRemaining = State.BatteryRemaining,
            BatteryConsumed = State.BatteryConsumed,
            
            // Simulated sensors
            GpsSatellites = 12 + (int)GaussianNoise(1),
            GpsFix = GpsFixType.Fix3D,
            GpsHdop = 1.0 + System.Math.Abs(GaussianNoise(0.1)),
            
            Armed = State.Armed,
            Mode = State.Mode,
            
            MotorOutputs = State.MotorOutputs.ToArray()
        };
        
        // Apply low-pass filter to smooth values
        if (_firstTelemetry)
        {
            _smoothedTelemetry = rawTelem;
            _firstTelemetry = false;
        }
        else
        {
            _smoothedTelemetry = SmoothTelemetry(_smoothedTelemetry, rawTelem, SmoothingFactor);
        }
        
        return _smoothedTelemetry;
    }
    
    /// <summary>
    /// Apply exponential moving average filter to smooth telemetry.
    /// </summary>
    private static DroneTelemetry SmoothTelemetry(DroneTelemetry previous, DroneTelemetry current, double alpha)
    {
        return new DroneTelemetry
        {
            Timestamp = current.Timestamp,
            Roll = Lerp(previous.Roll, current.Roll, alpha),
            Pitch = Lerp(previous.Pitch, current.Pitch, alpha),
            Yaw = LerpAngle(previous.Yaw, current.Yaw, alpha),
            RollRate = Lerp(previous.RollRate, current.RollRate, alpha),
            PitchRate = Lerp(previous.PitchRate, current.PitchRate, alpha),
            YawRate = Lerp(previous.YawRate, current.YawRate, alpha),
            Latitude = Lerp(previous.Latitude, current.Latitude, alpha),
            Longitude = Lerp(previous.Longitude, current.Longitude, alpha),
            AltitudeMSL = Lerp(previous.AltitudeMSL, current.AltitudeMSL, alpha),
            AltitudeRelative = Lerp(previous.AltitudeRelative, current.AltitudeRelative, alpha),
            VelocityNorth = Lerp(previous.VelocityNorth, current.VelocityNorth, alpha),
            VelocityEast = Lerp(previous.VelocityEast, current.VelocityEast, alpha),
            VelocityDown = Lerp(previous.VelocityDown, current.VelocityDown, alpha),
            GroundSpeed = Lerp(previous.GroundSpeed, current.GroundSpeed, alpha),
            ClimbRate = Lerp(previous.ClimbRate, current.ClimbRate, alpha),
            Heading = LerpAngle(previous.Heading, current.Heading, alpha),
            BatteryVoltage = Lerp(previous.BatteryVoltage, current.BatteryVoltage, alpha),
            BatteryCurrent = Lerp(previous.BatteryCurrent, current.BatteryCurrent, alpha),
            BatteryRemaining = current.BatteryRemaining,
            BatteryConsumed = current.BatteryConsumed,
            GpsSatellites = current.GpsSatellites,
            GpsFix = current.GpsFix,
            GpsHdop = Lerp(previous.GpsHdop, current.GpsHdop, alpha),
            GpsVdop = Lerp(previous.GpsVdop, current.GpsVdop, alpha),
            Armed = current.Armed,
            Mode = current.Mode,
            MotorOutputs = current.MotorOutputs
        };
    }
    
    private static double Lerp(double a, double b, double t) => a + (b - a) * t;
    
    private static double LerpAngle(double a, double b, double t)
    {
        double diff = b - a;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return a + diff * t;
    }

    private double GaussianNoise(double stdDev)
    {
        double u1 = 1.0 - _random.NextDouble();
        double u2 = 1.0 - _random.NextDouble();
        return stdDev * System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

public class DroneState
{
    public DateTimeOffset Timestamp { get; set; }
    
    // Attitude (radians)
    public double Roll { get; set; }
    public double Pitch { get; set; }
    public double Yaw { get; set; }
    
    // Angular rates (rad/s)
    public double RollRate { get; set; }
    public double PitchRate { get; set; }
    public double YawRate { get; set; }
    
    // Position
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }
    
    // Velocity (NED, m/s)
    public double VelocityNorth { get; set; }
    public double VelocityEast { get; set; }
    public double VelocityDown { get; set; }
    
    // Derived
    public double GroundSpeed { get; set; }
    public double ClimbRate { get; set; }
    public double Heading { get; set; }
    
    // Battery
    public double BatteryVoltage { get; set; }
    public double BatteryCurrent { get; set; }
    public double BatteryRemaining { get; set; }
    public double BatteryConsumed { get; set; }
    
    // Status
    public bool Armed { get; set; }
    public FlightMode Mode { get; set; }
    
    // Motors
    public int[] MotorOutputs { get; set; } = new int[8];
}

public struct Vector3
{
    public double X;
    public double Y;
    public double Z;
    
    public Vector3(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }
}

/// <summary>
/// Wind model for simulation.
/// </summary>
public class WindModel
{
    public double BaseSpeedMs { get; set; } = 0;
    public double DirectionDegrees { get; set; } = 0;
    public double GustStrength { get; set; } = 0;
    public double GustPeriodSeconds { get; set; } = 5;
    public double TurbulenceIntensity { get; set; } = 0;
    
    private readonly Random _random = new();
    private double _gustPhase = 0;
    
    public Vector3 CalculateForce(double altitude, DroneSpecs specs)
    {
        double altitudeFactor = 1 + System.Math.Min(altitude / 100, 1);
        double speed = BaseSpeedMs * altitudeFactor;
        
        _gustPhase += 0.01;
        double gust = GustStrength * (0.5 + 0.5 * System.Math.Sin(_gustPhase * 2 * System.Math.PI / GustPeriodSeconds));
        speed += gust;
        
        speed += TurbulenceIntensity * (_random.NextDouble() - 0.5) * 2;
        
        double dirRad = DirectionDegrees * System.Math.PI / 180;
        double dragArea = 0.01;
        double dragForce = 0.5 * 1.225 * speed * speed * dragArea;
        
        return new Vector3(
            -dragForce * System.Math.Cos(dirRad),
            -dragForce * System.Math.Sin(dirRad),
            0
        );
    }
}
