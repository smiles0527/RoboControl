namespace ControlWorkbench.VEX.Simulation;

/// <summary>
/// Full robot simulation for testing autonomous routines without hardware.
/// Simulates physics, sensors, and motor behavior.
/// </summary>
public class RobotSimulator
{
    // Robot physical properties
    public double WheelDiameter { get; set; } = 4.0;        // inches
    public double TrackWidth { get; set; } = 12.0;          // inches
    public double RobotMass { get; set; } = 15.0;           // lbs
    public double WheelbaseLength { get; set; } = 12.0;     // inches

    // Current state
    public double X { get; private set; }                    // inches
    public double Y { get; private set; }                    // inches
    public double Heading { get; private set; }              // radians
    public double LinearVelocity { get; private set; }       // in/sec
    public double AngularVelocity { get; private set; }      // rad/sec

    // Motor simulation
    private readonly SimulatedMotor[] _leftMotors;
    private readonly SimulatedMotor[] _rightMotors;
    private readonly List<SimulatedSensor> _sensors = new();

    // Physics
    public double FrictionCoefficient { get; set; } = 0.8;
    public double MotorEfficiency { get; set; } = 0.85;
    public double MaxMotorRpm { get; set; } = 600;           // Blue cartridge
    public double MaxMotorTorque { get; set; } = 0.5;        // Nm

    // Simulation settings
    public double TimeStep { get; set; } = 0.01;             // 10ms (100Hz)
    public double SimulationTime { get; private set; }
    public bool IsRunning { get; private set; }

    // Field
    public FieldEnvironment Field { get; } = new();

    // Events
    public event Action<double>? OnTick;
    public event Action<string>? OnLog;
    public event Action<CollisionInfo>? OnCollision;

    public RobotSimulator(int leftMotorCount = 3, int rightMotorCount = 3)
    {
        _leftMotors = new SimulatedMotor[leftMotorCount];
        _rightMotors = new SimulatedMotor[rightMotorCount];

        for (int i = 0; i < leftMotorCount; i++)
            _leftMotors[i] = new SimulatedMotor { MaxRpm = MaxMotorRpm };
        for (int i = 0; i < rightMotorCount; i++)
            _rightMotors[i] = new SimulatedMotor { MaxRpm = MaxMotorRpm };
    }

    /// <summary>
    /// Reset simulation to starting position.
    /// </summary>
    public void Reset(double x = 0, double y = 0, double heading = 0)
    {
        X = x;
        Y = y;
        Heading = heading;
        LinearVelocity = 0;
        AngularVelocity = 0;
        SimulationTime = 0;

        foreach (var motor in _leftMotors) motor.Reset();
        foreach (var motor in _rightMotors) motor.Reset();
        foreach (var sensor in _sensors) sensor.Reset();
    }

    /// <summary>
    /// Set motor power (-127 to 127).
    /// </summary>
    public void SetMotorPower(int leftPower, int rightPower)
    {
        foreach (var motor in _leftMotors)
            motor.SetPower(leftPower);
        foreach (var motor in _rightMotors)
            motor.SetPower(rightPower);
    }

    /// <summary>
    /// Step the simulation forward.
    /// </summary>
    public void Step()
    {
        // Update motors
        foreach (var motor in _leftMotors) motor.Update(TimeStep);
        foreach (var motor in _rightMotors) motor.Update(TimeStep);

        // Calculate wheel velocities (in/sec)
        double leftVelocity = GetAverageMotorVelocity(_leftMotors);
        double rightVelocity = GetAverageMotorVelocity(_rightMotors);

        // Differential drive kinematics
        LinearVelocity = (leftVelocity + rightVelocity) / 2.0;
        AngularVelocity = (rightVelocity - leftVelocity) / TrackWidth;

        // Update position using Runge-Kutta integration
        double newHeading = Heading + AngularVelocity * TimeStep;
        double avgHeading = (Heading + newHeading) / 2.0;

        double dx = LinearVelocity * Math.Cos(avgHeading) * TimeStep;
        double dy = LinearVelocity * Math.Sin(avgHeading) * TimeStep;

        double newX = X + dx;
        double newY = Y + dy;

        // Check field boundaries
        var collision = Field.CheckCollision(newX, newY, 9.0); // 9" robot radius
        if (collision.HasCollision)
        {
            OnCollision?.Invoke(collision);
            // Bounce back slightly
            newX = X - dx * 0.1;
            newY = Y - dy * 0.1;
            LinearVelocity *= -0.3; // Lose energy on collision
        }

        X = newX;
        Y = newY;
        Heading = NormalizeAngle(newHeading);

        // Update sensors
        foreach (var sensor in _sensors)
        {
            sensor.Update(X, Y, Heading, Field);
        }

        SimulationTime += TimeStep;
        OnTick?.Invoke(SimulationTime);
    }

    /// <summary>
    /// Run simulation for specified duration.
    /// </summary>
    public async Task RunAsync(double durationSeconds, Action<RobotSimulator> controlLoop, CancellationToken ct = default)
    {
        IsRunning = true;
        double endTime = SimulationTime + durationSeconds;

        while (SimulationTime < endTime && !ct.IsCancellationRequested)
        {
            controlLoop(this);
            Step();
            await Task.Delay((int)(TimeStep * 1000), ct);
        }

        IsRunning = false;
    }

    /// <summary>
    /// Run simulation at maximum speed (no delays).
    /// </summary>
    public void RunFast(double durationSeconds, Action<RobotSimulator> controlLoop)
    {
        double endTime = SimulationTime + durationSeconds;
        while (SimulationTime < endTime)
        {
            controlLoop(this);
            Step();
        }
    }

    /// <summary>
    /// Add a simulated sensor.
    /// </summary>
    public T AddSensor<T>(T sensor) where T : SimulatedSensor
    {
        _sensors.Add(sensor);
        return sensor;
    }

    /// <summary>
    /// Get current motor positions (degrees).
    /// </summary>
    public (double left, double right) GetEncoderPositions()
    {
        double left = _leftMotors.Average(m => m.Position);
        double right = _rightMotors.Average(m => m.Position);
        return (left, right);
    }

    /// <summary>
    /// Get current motor velocities (RPM).
    /// </summary>
    public (double left, double right) GetEncoderVelocities()
    {
        double left = _leftMotors.Average(m => m.Velocity);
        double right = _rightMotors.Average(m => m.Velocity);
        return (left, right);
    }

    private double GetAverageMotorVelocity(SimulatedMotor[] motors)
    {
        // Convert RPM to in/sec
        double avgRpm = motors.Average(m => m.Velocity);
        return avgRpm / 60.0 * Math.PI * WheelDiameter;
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

/// <summary>
/// Simulated V5 Smart Motor.
/// </summary>
public class SimulatedMotor
{
    public double MaxRpm { get; set; } = 600;
    public double MaxTorque { get; set; } = 0.5;
    public double Inertia { get; set; } = 0.001;
    public double Friction { get; set; } = 0.01;

    public double Position { get; private set; }       // degrees
    public double Velocity { get; private set; }       // RPM
    public double Current { get; private set; }        // mA
    public double Temperature { get; private set; }    // °C

    private double _targetPower;
    private double _thermalEnergy;

    public void SetPower(int power)
    {
        _targetPower = Math.Clamp(power, -127, 127) / 127.0;
    }

    public void Update(double dt)
    {
        // Motor dynamics (simplified DC motor model)
        double targetVelocity = _targetPower * MaxRpm;
        double acceleration = (targetVelocity - Velocity) * 10 - Friction * Velocity;
        
        Velocity += acceleration * dt;
        Velocity = Math.Clamp(Velocity, -MaxRpm, MaxRpm);

        Position += Velocity * 6.0 * dt; // RPM to deg/sec

        // Current estimation (proportional to torque)
        Current = Math.Abs(_targetPower) * 2500; // Max 2.5A

        // Thermal simulation
        double powerDissipated = Current * Current * 0.00001; // I²R losses
        _thermalEnergy += powerDissipated * dt;
        _thermalEnergy -= (_thermalEnergy - 25) * 0.01 * dt; // Cooling
        Temperature = 25 + _thermalEnergy * 0.1;
    }

    public void Reset()
    {
        Position = 0;
        Velocity = 0;
        Current = 0;
        Temperature = 25;
        _targetPower = 0;
        _thermalEnergy = 0;
    }
}

/// <summary>
/// Base class for simulated sensors.
/// </summary>
public abstract class SimulatedSensor
{
    public string Name { get; set; } = "";
    public double NoiseLevel { get; set; } = 0.01;

    protected Random Random { get; } = new();

    public abstract void Update(double x, double y, double heading, FieldEnvironment field);
    public abstract void Reset();

    protected double AddNoise(double value)
    {
        return value + (Random.NextDouble() - 0.5) * 2 * NoiseLevel * value;
    }
}

/// <summary>
/// Simulated IMU sensor.
/// </summary>
public class SimulatedImu : SimulatedSensor
{
    public double Heading { get; private set; }
    public double HeadingRate { get; private set; }
    public double Pitch { get; private set; }
    public double Roll { get; private set; }

    private double _previousHeading;
    private double _drift;
    public double DriftRate { get; set; } = 0.001; // rad/sec drift

    public override void Update(double x, double y, double heading, FieldEnvironment field)
    {
        _drift += DriftRate * 0.01; // Accumulate drift
        Heading = AddNoise(heading * 180 / Math.PI + _drift);
        HeadingRate = AddNoise((heading - _previousHeading) / 0.01 * 180 / Math.PI);
        _previousHeading = heading;

        // Simulate slight tilt from acceleration
        Pitch = AddNoise(0);
        Roll = AddNoise(0);
    }

    public override void Reset()
    {
        Heading = 0;
        HeadingRate = 0;
        _previousHeading = 0;
        _drift = 0;
    }
}

/// <summary>
/// Simulated distance sensor.
/// </summary>
public class SimulatedDistanceSensor : SimulatedSensor
{
    public double Distance { get; private set; }        // mm
    public double MaxRange { get; set; } = 2000;        // mm
    public double OffsetX { get; set; }                 // Sensor offset from robot center
    public double OffsetY { get; set; }
    public double AngleOffset { get; set; }             // Sensor angle relative to robot

    public override void Update(double x, double y, double heading, FieldEnvironment field)
    {
        // Calculate sensor position
        double sensorX = x + OffsetX * Math.Cos(heading) - OffsetY * Math.Sin(heading);
        double sensorY = y + OffsetX * Math.Sin(heading) + OffsetY * Math.Cos(heading);
        double sensorAngle = heading + AngleOffset;

        // Ray cast to find distance
        Distance = AddNoise(field.RayCast(sensorX, sensorY, sensorAngle, MaxRange) * 25.4);
    }

    public override void Reset()
    {
        Distance = MaxRange;
    }
}

/// <summary>
/// Simulated optical sensor.
/// </summary>
public class SimulatedOpticalSensor : SimulatedSensor
{
    public double Hue { get; private set; }
    public double Saturation { get; private set; }
    public double Brightness { get; private set; }
    public int Proximity { get; private set; }

    public double OffsetX { get; set; }
    public double OffsetY { get; set; }

    public override void Update(double x, double y, double heading, FieldEnvironment field)
    {
        double sensorX = x + OffsetX * Math.Cos(heading) - OffsetY * Math.Sin(heading);
        double sensorY = y + OffsetX * Math.Sin(heading) + OffsetY * Math.Cos(heading);

        // Check for game objects under sensor
        var obj = field.GetObjectAt(sensorX, sensorY);
        if (obj != null)
        {
            Hue = obj.Color.Hue;
            Saturation = obj.Color.Saturation;
            Brightness = obj.Color.Brightness;
            Proximity = 200;
        }
        else
        {
            Hue = 0;
            Saturation = 0;
            Brightness = 20; // Floor
            Proximity = 0;
        }
    }

    public override void Reset()
    {
        Hue = 0;
        Saturation = 0;
        Brightness = 20;
        Proximity = 0;
    }
}

/// <summary>
/// VRC field environment with walls and game objects.
/// </summary>
public class FieldEnvironment
{
    public double Width { get; } = 144;    // 12 feet
    public double Height { get; } = 144;

    public List<FieldWall> Walls { get; } = new();
    public List<GameObject> Objects { get; } = new();
    public List<ScoringZone> ScoringZones { get; } = new();

    public FieldEnvironment()
    {
        // Add field perimeter walls
        Walls.Add(new FieldWall(0, 0, 144, 0));       // Bottom
        Walls.Add(new FieldWall(144, 0, 144, 144));   // Right
        Walls.Add(new FieldWall(144, 144, 0, 144));   // Top
        Walls.Add(new FieldWall(0, 144, 0, 0));       // Left
    }

    /// <summary>
    /// Set up field for a specific VRC game.
    /// </summary>
    public void SetupHighStakes()
    {
        // Clear existing objects
        Objects.Clear();
        ScoringZones.Clear();

        // Add stakes (simplified as scoring zones)
        ScoringZones.Add(new ScoringZone("Alliance Stake Red", 12, 72, 6));
        ScoringZones.Add(new ScoringZone("Alliance Stake Blue", 132, 72, 6));
        ScoringZones.Add(new ScoringZone("Neutral Stake 1", 72, 24, 4));
        ScoringZones.Add(new ScoringZone("Neutral Stake 2", 72, 120, 4));
        ScoringZones.Add(new ScoringZone("Wall Stake 1", 36, 0, 4));
        ScoringZones.Add(new ScoringZone("Wall Stake 2", 108, 0, 4));
        ScoringZones.Add(new ScoringZone("Wall Stake 3", 36, 144, 4));
        ScoringZones.Add(new ScoringZone("Wall Stake 4", 108, 144, 4));

        // Add rings
        var ringPositions = new[] {
            (36, 36), (72, 36), (108, 36),
            (36, 72), (72, 72), (108, 72),
            (36, 108), (72, 108), (108, 108)
        };

        for (int i = 0; i < ringPositions.Length; i++)
        {
            var (x, y) = ringPositions[i];
            bool isRed = i % 2 == 0;
            Objects.Add(new GameObject($"Ring{i + 1}", x, y, 
                isRed ? GameObjectType.RedRing : GameObjectType.BlueRing));
        }

        // Add mobile goals
        Objects.Add(new GameObject("MobileGoalRed", 24, 24, GameObjectType.MobileGoal));
        Objects.Add(new GameObject("MobileGoalBlue", 120, 120, GameObjectType.MobileGoal));
    }

    public CollisionInfo CheckCollision(double x, double y, double robotRadius)
    {
        // Check field boundaries
        if (x - robotRadius < 0 || x + robotRadius > Width ||
            y - robotRadius < 0 || y + robotRadius > Height)
        {
            return new CollisionInfo { HasCollision = true, Type = "Wall" };
        }

        // Check walls
        foreach (var wall in Walls)
        {
            if (wall.DistanceTo(x, y) < robotRadius)
            {
                return new CollisionInfo { HasCollision = true, Type = "Wall" };
            }
        }

        return new CollisionInfo { HasCollision = false };
    }

    public double RayCast(double x, double y, double angle, double maxDistance)
    {
        double dx = Math.Cos(angle);
        double dy = Math.Sin(angle);

        double minDist = maxDistance;

        // Check walls
        foreach (var wall in Walls)
        {
            double? dist = wall.RayIntersect(x, y, dx, dy);
            if (dist.HasValue && dist.Value < minDist)
                minDist = dist.Value;
        }

        // Check field boundaries
        double[] boundaryDists = {
            (0 - x) / dx, (Width - x) / dx,
            (0 - y) / dy, (Height - y) / dy
        };

        foreach (double dist in boundaryDists)
        {
            if (dist > 0 && dist < minDist)
            {
                double hitX = x + dx * dist;
                double hitY = y + dy * dist;
                if (hitX >= 0 && hitX <= Width && hitY >= 0 && hitY <= Height)
                    minDist = dist;
            }
        }

        return minDist;
    }

    public GameObject? GetObjectAt(double x, double y)
    {
        return Objects.FirstOrDefault(o => 
            Math.Sqrt(Math.Pow(o.X - x, 2) + Math.Pow(o.Y - y, 2)) < o.Radius);
    }
}

public class FieldWall
{
    public double X1 { get; }
    public double Y1 { get; }
    public double X2 { get; }
    public double Y2 { get; }

    public FieldWall(double x1, double y1, double x2, double y2)
    {
        X1 = x1; Y1 = y1; X2 = x2; Y2 = y2;
    }

    public double DistanceTo(double px, double py)
    {
        double dx = X2 - X1;
        double dy = Y2 - Y1;
        double t = Math.Max(0, Math.Min(1, ((px - X1) * dx + (py - Y1) * dy) / (dx * dx + dy * dy)));
        double nearestX = X1 + t * dx;
        double nearestY = Y1 + t * dy;
        return Math.Sqrt(Math.Pow(px - nearestX, 2) + Math.Pow(py - nearestY, 2));
    }

    public double? RayIntersect(double ox, double oy, double dx, double dy)
    {
        double x1 = X1, y1 = Y1, x2 = X2, y2 = Y2;
        double x3 = ox, y3 = oy, x4 = ox + dx * 1000, y4 = oy + dy * 1000;

        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.Abs(denom) < 0.0001) return null;

        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

        if (t >= 0 && t <= 1 && u >= 0)
            return u * Math.Sqrt(dx * dx + dy * dy) * 1000;

        return null;
    }
}

public class GameObject
{
    public string Name { get; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Radius { get; set; } = 3.0;
    public GameObjectType Type { get; }
    public ColorInfo Color { get; }
    public bool IsPickedUp { get; set; }

    public GameObject(string name, double x, double y, GameObjectType type)
    {
        Name = name;
        X = x;
        Y = y;
        Type = type;
        Color = type switch
        {
            GameObjectType.RedRing => new ColorInfo(0, 90, 80),
            GameObjectType.BlueRing => new ColorInfo(220, 90, 80),
            GameObjectType.MobileGoal => new ColorInfo(45, 70, 60),
            _ => new ColorInfo(0, 0, 50)
        };
    }
}

public enum GameObjectType
{
    RedRing,
    BlueRing,
    MobileGoal,
    Triball,
    Disc
}

public class ScoringZone
{
    public string Name { get; }
    public double X { get; }
    public double Y { get; }
    public double Radius { get; }
    public List<GameObject> ScoredObjects { get; } = new();

    public ScoringZone(string name, double x, double y, double radius)
    {
        Name = name; X = x; Y = y; Radius = radius;
    }
}

public record struct ColorInfo(double Hue, double Saturation, double Brightness);

public class CollisionInfo
{
    public bool HasCollision { get; set; }
    public string Type { get; set; } = "";
    public double X { get; set; }
    public double Y { get; set; }
}
