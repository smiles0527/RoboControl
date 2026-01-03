namespace ControlWorkbench.VEX.Devices;

/// <summary>
/// Manages all VEX V5 devices on the robot.
/// Provides easy access and automatic updates.
/// </summary>
public class V5Robot
{
    // Device collections
    public Dictionary<int, V5Motor> Motors { get; } = new();
    public Dictionary<int, V5RotationSensor> RotationSensors { get; } = new();
    public Dictionary<int, V5DistanceSensor> DistanceSensors { get; } = new();
    public Dictionary<int, V5OpticalSensor> OpticalSensors { get; } = new();
    public Dictionary<int, V5VisionSensor> VisionSensors { get; } = new();
    public Dictionary<int, V5GpsSensor> GpsSensors { get; } = new();
    public Dictionary<int, V5InertialSensor> InertialSensors { get; } = new();

    // Special devices (usually one of each)
    public V5InertialSensor? Imu { get; private set; }
    public V5GpsSensor? Gps { get; private set; }
    public V5Controller Controller { get; } = new();
    public V5Controller PartnerController { get; } = new();
    public V5Battery Battery { get; } = new();
    public V5Competition Competition { get; } = new();

    // 3-Wire devices
    public Dictionary<char, V5Pneumatic> Pneumatics { get; } = new();
    public Dictionary<char, V5Servo> Servos { get; } = new();
    public V5ThreeWireExpander? ThreeWireExpander { get; private set; }

    // Robot name/team
    public string TeamNumber { get; set; } = "";
    public string RobotName { get; set; } = "V5 Robot";

    // Connection state
    public bool IsConnected { get; private set; }
    public DateTime LastUpdate { get; private set; }

    // Events
    public event Action<string>? LogMessage;
    public event Action<V5Device>? DeviceUpdated;
    public event Action<bool>? ConnectionChanged;

    /// <summary>
    /// Add a motor on the specified port.
    /// </summary>
    public V5Motor AddMotor(int port, string? name = null, MotorCartridge cartridge = MotorCartridge.Green, bool reversed = false)
    {
        var motor = new V5Motor(port, name)
        {
            Cartridge = cartridge,
            IsReversed = reversed
        };
        Motors[port] = motor;
        Log($"Added motor on port {port}: {motor.Name}");
        return motor;
    }

    /// <summary>
    /// Add a rotation sensor on the specified port.
    /// </summary>
    public V5RotationSensor AddRotationSensor(int port, string? name = null, bool reversed = false)
    {
        var sensor = new V5RotationSensor(port, name) { IsReversed = reversed };
        RotationSensors[port] = sensor;
        Log($"Added rotation sensor on port {port}: {sensor.Name}");
        return sensor;
    }

    /// <summary>
    /// Add a distance sensor on the specified port.
    /// </summary>
    public V5DistanceSensor AddDistanceSensor(int port, string? name = null)
    {
        var sensor = new V5DistanceSensor(port, name);
        DistanceSensors[port] = sensor;
        Log($"Added distance sensor on port {port}: {sensor.Name}");
        return sensor;
    }

    /// <summary>
    /// Add an optical (color) sensor on the specified port.
    /// </summary>
    public V5OpticalSensor AddOpticalSensor(int port, string? name = null)
    {
        var sensor = new V5OpticalSensor(port, name);
        OpticalSensors[port] = sensor;
        Log($"Added optical sensor on port {port}: {sensor.Name}");
        return sensor;
    }

    /// <summary>
    /// Add a vision sensor on the specified port.
    /// </summary>
    public V5VisionSensor AddVisionSensor(int port, string? name = null)
    {
        var sensor = new V5VisionSensor(port, name);
        VisionSensors[port] = sensor;
        Log($"Added vision sensor on port {port}: {sensor.Name}");
        return sensor;
    }

    /// <summary>
    /// Add an inertial sensor (IMU) on the specified port.
    /// </summary>
    public V5InertialSensor AddInertialSensor(int port, string? name = null, bool setPrimary = true)
    {
        var sensor = new V5InertialSensor(port, name);
        InertialSensors[port] = sensor;
        if (setPrimary) Imu = sensor;
        Log($"Added inertial sensor on port {port}: {sensor.Name}");
        return sensor;
    }

    /// <summary>
    /// Add a GPS sensor on the specified port.
    /// </summary>
    public V5GpsSensor AddGpsSensor(int port, string? name = null, bool setPrimary = true)
    {
        var sensor = new V5GpsSensor(port, name);
        GpsSensors[port] = sensor;
        if (setPrimary) Gps = sensor;
        Log($"Added GPS sensor on port {port}: {sensor.Name}");
        return sensor;
    }

    /// <summary>
    /// Add a pneumatic solenoid on the specified 3-wire port.
    /// </summary>
    public V5Pneumatic AddPneumatic(char port, string? name = null)
    {
        var pneumatic = new V5Pneumatic(port, name);
        Pneumatics[port] = pneumatic;
        Log($"Added pneumatic on port {port}: {pneumatic.Name}");
        return pneumatic;
    }

    /// <summary>
    /// Add a servo on the specified 3-wire port.
    /// </summary>
    public V5Servo AddServo(char port, string? name = null)
    {
        var servo = new V5Servo(port, name);
        Servos[port] = servo;
        Log($"Added servo on port {port}: {servo.Name}");
        return servo;
    }

    /// <summary>
    /// Add a 3-wire expander on the specified smart port.
    /// </summary>
    public V5ThreeWireExpander AddThreeWireExpander(int port, string? name = null)
    {
        ThreeWireExpander = new V5ThreeWireExpander(port, name);
        Log($"Added 3-wire expander on port {port}");
        return ThreeWireExpander;
    }

    /// <summary>
    /// Get a motor by port number.
    /// </summary>
    public V5Motor? GetMotor(int port) =>
        Motors.TryGetValue(port, out var motor) ? motor : null;

    /// <summary>
    /// Get all motors in a group (by name prefix).
    /// </summary>
    public IEnumerable<V5Motor> GetMotorGroup(string namePrefix) =>
        Motors.Values.Where(m => m.Name.StartsWith(namePrefix, StringComparison.OrdinalIgnoreCase));

    /// <summary>
    /// Update a device from received telemetry data.
    /// </summary>
    public void UpdateDevice(VexMessageType messageType, int port, byte[] data)
    {
        V5Device? device = null;

        switch (messageType)
        {
            case VexMessageType.MotorTelemetry:
                if (Motors.TryGetValue(port, out var motor))
                {
                    motor.Update(data);
                    device = motor;
                }
                break;

            case VexMessageType.RotationSensor:
                if (RotationSensors.TryGetValue(port, out var rotation))
                {
                    rotation.Update(data);
                    device = rotation;
                }
                break;

            case VexMessageType.DistanceSensor:
                if (DistanceSensors.TryGetValue(port, out var distance))
                {
                    distance.Update(data);
                    device = distance;
                }
                break;

            case VexMessageType.OpticalSensor:
                if (OpticalSensors.TryGetValue(port, out var optical))
                {
                    optical.Update(data);
                    device = optical;
                }
                break;

            case VexMessageType.VisionSensor:
                if (VisionSensors.TryGetValue(port, out var vision))
                {
                    vision.Update(data);
                    device = vision;
                }
                break;

            case VexMessageType.InertialSensor:
            case VexMessageType.ImuData:
                if (InertialSensors.TryGetValue(port, out var imu))
                {
                    imu.Update(data);
                    device = imu;
                }
                break;

            case VexMessageType.GpsSensor:
                if (GpsSensors.TryGetValue(port, out var gps))
                {
                    gps.Update(data);
                    device = gps;
                }
                break;

            case VexMessageType.ControllerInput:
                Controller.Update(data);
                break;

            case VexMessageType.BatteryStatus:
                Battery.Update(data);
                break;

            case VexMessageType.CompetitionStatus:
                Competition.Update(data);
                break;
        }

        if (device != null)
        {
            DeviceUpdated?.Invoke(device);
        }

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Get a summary of all connected devices.
    /// </summary>
    public string GetDeviceSummary()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"=== {RobotName} ({TeamNumber}) ===");
        sb.AppendLine($"Connected: {IsConnected}");
        sb.AppendLine();

        if (Motors.Any())
        {
            sb.AppendLine("Motors:");
            foreach (var motor in Motors.Values)
                sb.AppendLine($"  Port {motor.Port}: {motor}");
        }

        if (InertialSensors.Any())
        {
            sb.AppendLine("Inertial Sensors:");
            foreach (var sensor in InertialSensors.Values)
                sb.AppendLine($"  Port {sensor.Port}: {sensor}");
        }

        if (RotationSensors.Any())
        {
            sb.AppendLine("Rotation Sensors:");
            foreach (var sensor in RotationSensors.Values)
                sb.AppendLine($"  Port {sensor.Port}: {sensor}");
        }

        if (DistanceSensors.Any())
        {
            sb.AppendLine("Distance Sensors:");
            foreach (var sensor in DistanceSensors.Values)
                sb.AppendLine($"  Port {sensor.Port}: {sensor}");
        }

        if (OpticalSensors.Any())
        {
            sb.AppendLine("Optical Sensors:");
            foreach (var sensor in OpticalSensors.Values)
                sb.AppendLine($"  Port {sensor.Port}: {sensor}");
        }

        if (VisionSensors.Any())
        {
            sb.AppendLine("Vision Sensors:");
            foreach (var sensor in VisionSensors.Values)
                sb.AppendLine($"  Port {sensor.Port}: {sensor}");
        }

        if (GpsSensors.Any())
        {
            sb.AppendLine("GPS Sensors:");
            foreach (var sensor in GpsSensors.Values)
                sb.AppendLine($"  Port {sensor.Port}: {sensor}");
        }

        if (Pneumatics.Any())
        {
            sb.AppendLine("Pneumatics:");
            foreach (var pneumatic in Pneumatics.Values)
                sb.AppendLine($"  Port {pneumatic.Port}: {pneumatic}");
        }

        sb.AppendLine();
        sb.AppendLine($"Battery: {Battery}");
        sb.AppendLine($"Competition: {Competition}");

        return sb.ToString();
    }

    /// <summary>
    /// Check for any issues with the robot.
    /// </summary>
    public List<RobotWarning> GetWarnings()
    {
        var warnings = new List<RobotWarning>();

        // Check battery
        if (Battery.Status == BatteryStatus.Critical)
            warnings.Add(new RobotWarning(WarningLevel.Critical, "Battery critically low!"));
        else if (Battery.Status == BatteryStatus.Low)
            warnings.Add(new RobotWarning(WarningLevel.Warning, "Battery low - consider charging"));

        // Check motors
        foreach (var motor in Motors.Values)
        {
            if (motor.IsOverTemperature)
                warnings.Add(new RobotWarning(WarningLevel.Critical, $"{motor.Name} overheating ({motor.Temperature:F0}°C)"));
            else if (motor.Temperature > 50)
                warnings.Add(new RobotWarning(WarningLevel.Warning, $"{motor.Name} getting hot ({motor.Temperature:F0}°C)"));

            if (motor.IsStalled)
                warnings.Add(new RobotWarning(WarningLevel.Warning, $"{motor.Name} may be stalled"));
        }

        // Check sensor connections
        foreach (var sensor in InertialSensors.Values)
        {
            if (!sensor.IsConnected)
                warnings.Add(new RobotWarning(WarningLevel.Error, $"{sensor.Name} not connected"));
        }

        return warnings;
    }

    private void Log(string message) => LogMessage?.Invoke(message);
}

public class RobotWarning
{
    public WarningLevel Level { get; }
    public string Message { get; }
    public DateTime Timestamp { get; }

    public RobotWarning(WarningLevel level, string message)
    {
        Level = level;
        Message = message;
        Timestamp = DateTime.UtcNow;
    }

    public override string ToString() => $"[{Level}] {Message}";
}

public enum WarningLevel
{
    Info,
    Warning,
    Error,
    Critical
}

/// <summary>
/// Pre-configured robot templates for common setups.
/// </summary>
public static class RobotTemplates
{
    /// <summary>
    /// Create a standard tank drive robot configuration.
    /// </summary>
    public static V5Robot CreateTankDrive(
        int[] leftPorts,
        int[] rightPorts,
        int imuPort = 0,
        bool hasGps = false,
        int gpsPort = 0)
    {
        var robot = new V5Robot { RobotName = "Tank Drive Robot" };

        // Add left motors
        for (int i = 0; i < leftPorts.Length; i++)
        {
            robot.AddMotor(leftPorts[i], $"LeftMotor{i + 1}", MotorCartridge.Blue, reversed: true);
        }

        // Add right motors
        for (int i = 0; i < rightPorts.Length; i++)
        {
            robot.AddMotor(rightPorts[i], $"RightMotor{i + 1}", MotorCartridge.Blue, reversed: false);
        }

        // Add IMU
        if (imuPort > 0)
        {
            robot.AddInertialSensor(imuPort, "IMU");
        }

        // Add GPS
        if (hasGps && gpsPort > 0)
        {
            robot.AddGpsSensor(gpsPort, "GPS");
        }

        return robot;
    }

    /// <summary>
    /// Create a 6-motor drivetrain configuration.
    /// </summary>
    public static V5Robot Create6MotorDrive()
    {
        return CreateTankDrive(
            leftPorts: [1, 2, 3],
            rightPorts: [4, 5, 6],
            imuPort: 10
        );
    }

    /// <summary>
    /// Create a robot with intake and lift mechanisms.
    /// </summary>
    public static V5Robot CreateWithMechanisms(
        int[] leftDrivePorts,
        int[] rightDrivePorts,
        int[] intakePorts,
        int[] liftPorts,
        int imuPort = 0)
    {
        var robot = CreateTankDrive(leftDrivePorts, rightDrivePorts, imuPort);
        robot.RobotName = "Competition Robot";

        // Add intake motors
        foreach (var port in intakePorts)
        {
            robot.AddMotor(port, $"Intake{port}", MotorCartridge.Green);
        }

        // Add lift motors
        foreach (var port in liftPorts)
        {
            robot.AddMotor(port, $"Lift{port}", MotorCartridge.Red);
        }

        return robot;
    }

    /// <summary>
    /// Create a robot with full sensor suite for autonomous.
    /// </summary>
    public static V5Robot CreateAutonomousSuite(
        int[] leftDrivePorts,
        int[] rightDrivePorts,
        int imuPort,
        int leftEncoderPort,
        int rightEncoderPort,
        int frontDistancePort = 0,
        int colorSensorPort = 0)
    {
        var robot = CreateTankDrive(leftDrivePorts, rightDrivePorts, imuPort);
        robot.RobotName = "Autonomous Robot";

        // Add tracking wheel encoders
        robot.AddRotationSensor(leftEncoderPort, "LeftEncoder", reversed: true);
        robot.AddRotationSensor(rightEncoderPort, "RightEncoder", reversed: false);

        // Add distance sensor
        if (frontDistancePort > 0)
        {
            robot.AddDistanceSensor(frontDistancePort, "FrontDistance");
        }

        // Add color sensor
        if (colorSensorPort > 0)
        {
            robot.AddOpticalSensor(colorSensorPort, "ColorSensor");
        }

        return robot;
    }
}
