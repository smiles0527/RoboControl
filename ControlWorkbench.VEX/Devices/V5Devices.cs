namespace ControlWorkbench.VEX.Devices;

/// <summary>
/// Base class for all VEX V5 devices.
/// </summary>
public abstract class V5Device
{
    public int Port { get; }
    public string Name { get; set; }
    public bool IsConnected { get; protected set; }
    public DateTime LastUpdate { get; protected set; }

    protected V5Device(int port, string? name = null)
    {
        Port = port;
        Name = name ?? $"{GetType().Name}_{port}";
    }

    public abstract void Update(byte[] data);
}

/// <summary>
/// V5 Smart Motor with full telemetry and control.
/// </summary>
public class V5Motor : V5Device
{
    // Current state
    public double Position { get; private set; }           // degrees
    public double Velocity { get; private set; }           // RPM
    public double Current { get; private set; }            // mA
    public double Voltage { get; private set; }            // mV
    public double Temperature { get; private set; }        // Celsius
    public double Power { get; private set; }              // Watts
    public double Torque { get; private set; }             // Nm
    public double Efficiency { get; private set; }         // %

    // Target/command state
    public double TargetPosition { get; set; }
    public double TargetVelocity { get; set; }
    public int TargetPower { get; set; }                   // -127 to 127

    // Configuration
    public MotorCartridge Cartridge { get; set; } = MotorCartridge.Green;
    public MotorBrakeMode BrakeMode { get; set; } = MotorBrakeMode.Coast;
    public bool IsReversed { get; set; }
    public double GearRatio { get; set; } = 1.0;

    // Status flags
    public bool IsOverTemperature => Temperature > 55;
    public bool IsOverCurrent => Current > 2500;
    public bool IsStalled => System.Math.Abs(Velocity) < 5 && System.Math.Abs(Current) > 1000;

    /// <summary>
    /// Maximum RPM based on cartridge type.
    /// </summary>
    public double MaxRpm => Cartridge switch
    {
        MotorCartridge.Red => 100,
        MotorCartridge.Green => 200,
        MotorCartridge.Blue => 600,
        _ => 200
    };

    public V5Motor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        if (data.Length < 57) return;

        Position = BitConverter.ToDouble(data, 0);
        Velocity = BitConverter.ToDouble(data, 8);
        Current = BitConverter.ToDouble(data, 16);
        Voltage = BitConverter.ToDouble(data, 24);
        Temperature = BitConverter.ToDouble(data, 32);
        Power = BitConverter.ToDouble(data, 40);
        Torque = BitConverter.ToDouble(data, 48);
        
        // Calculate efficiency
        if (Power > 0 && Voltage > 0 && Current > 0)
        {
            double inputPower = (Voltage / 1000.0) * (Current / 1000.0);
            Efficiency = (Power / inputPower) * 100;
        }

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Get position in rotations instead of degrees.
    /// </summary>
    public double PositionRotations => Position / 360.0;

    /// <summary>
    /// Get position accounting for gear ratio.
    /// </summary>
    public double OutputPosition => Position / GearRatio;

    /// <summary>
    /// Get velocity accounting for gear ratio.
    /// </summary>
    public double OutputVelocity => Velocity / GearRatio;

    /// <summary>
    /// Estimated time to overheat at current temperature rise rate.
    /// </summary>
    public TimeSpan? EstimatedTimeToOverheat(double tempRiseRatePerMinute)
    {
        if (tempRiseRatePerMinute <= 0) return null;
        double degreesToOverheat = 55 - Temperature;
        if (degreesToOverheat <= 0) return TimeSpan.Zero;
        return TimeSpan.FromMinutes(degreesToOverheat / tempRiseRatePerMinute);
    }

    public override string ToString() =>
        $"{Name}: {Velocity:F1} RPM, {Temperature:F1}°C, {Current:F0}mA";
}

/// <summary>
/// V5 Inertial Sensor (IMU) with heading, acceleration, and rotation rates.
/// </summary>
public class V5InertialSensor : V5Device
{
    // Orientation (degrees)
    public double Heading { get; private set; }
    public double Pitch { get; private set; }
    public double Roll { get; private set; }

    // Rotation rates (degrees/second)
    public double GyroX { get; private set; }
    public double GyroY { get; private set; }
    public double GyroZ { get; private set; }

    // Acceleration (g)
    public double AccelX { get; private set; }
    public double AccelY { get; private set; }
    public double AccelZ { get; private set; }

    // Calibration state
    public bool IsCalibrating { get; private set; }
    public bool IsCalibrated { get; private set; }

    // Heading utilities
    private double _headingOffset = 0;

    public V5InertialSensor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        if (data.Length < 72) return;

        Heading = BitConverter.ToDouble(data, 0);
        Pitch = BitConverter.ToDouble(data, 8);
        Roll = BitConverter.ToDouble(data, 16);
        GyroX = BitConverter.ToDouble(data, 24);
        GyroY = BitConverter.ToDouble(data, 32);
        GyroZ = BitConverter.ToDouble(data, 40);
        AccelX = BitConverter.ToDouble(data, 48);
        AccelY = BitConverter.ToDouble(data, 56);
        AccelZ = BitConverter.ToDouble(data, 64);

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Get heading with offset applied and normalized to 0-360.
    /// </summary>
    public double HeadingNormalized
    {
        get
        {
            double h = (Heading - _headingOffset) % 360;
            return h < 0 ? h + 360 : h;
        }
    }

    /// <summary>
    /// Get heading in radians.
    /// </summary>
    public double HeadingRadians => HeadingNormalized * System.Math.PI / 180.0;

    /// <summary>
    /// Reset heading to zero at current position.
    /// </summary>
    public void ResetHeading()
    {
        _headingOffset = Heading;
    }

    /// <summary>
    /// Set heading to a specific value.
    /// </summary>
    public void SetHeading(double newHeading)
    {
        _headingOffset = Heading - newHeading;
    }

    /// <summary>
    /// Get total rotation rate magnitude.
    /// </summary>
    public double RotationRate => System.Math.Sqrt(GyroX * GyroX + GyroY * GyroY + GyroZ * GyroZ);

    /// <summary>
    /// Check if robot is tilted beyond threshold.
    /// </summary>
    public bool IsTilted(double thresholdDegrees = 15) =>
        System.Math.Abs(Pitch) > thresholdDegrees || System.Math.Abs(Roll) > thresholdDegrees;

    /// <summary>
    /// Check if robot is approximately level.
    /// </summary>
    public bool IsLevel(double toleranceDegrees = 5) =>
        System.Math.Abs(Pitch) < toleranceDegrees && System.Math.Abs(Roll) < toleranceDegrees;

    public override string ToString() =>
        $"{Name}: Heading={HeadingNormalized:F1}°, Pitch={Pitch:F1}°, Roll={Roll:F1}°";
}

/// <summary>
/// V5 Rotation Sensor for precise position and velocity measurement.
/// </summary>
public class V5RotationSensor : V5Device
{
    public double Position { get; private set; }           // degrees
    public double Velocity { get; private set; }           // degrees/second
    public int RawPosition { get; private set; }           // centidegrees

    public bool IsReversed { get; set; }
    private double _positionOffset = 0;

    public V5RotationSensor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        if (data.Length < 21) return;

        Position = BitConverter.ToDouble(data, 1) * (IsReversed ? -1 : 1);
        Velocity = BitConverter.ToDouble(data, 9) * (IsReversed ? -1 : 1);
        RawPosition = BitConverter.ToInt32(data, 17);

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Position with offset applied.
    /// </summary>
    public double PositionWithOffset => Position - _positionOffset;

    /// <summary>
    /// Position in rotations.
    /// </summary>
    public double Rotations => PositionWithOffset / 360.0;

    /// <summary>
    /// Velocity in RPM.
    /// </summary>
    public double VelocityRpm => Velocity / 6.0;  // deg/s to RPM

    /// <summary>
    /// Reset position to zero.
    /// </summary>
    public void ResetPosition()
    {
        _positionOffset = Position;
    }

    /// <summary>
    /// Set position to a specific value.
    /// </summary>
    public void SetPosition(double newPosition)
    {
        _positionOffset = Position - newPosition;
    }

    public override string ToString() =>
        $"{Name}: {PositionWithOffset:F1}° ({VelocityRpm:F1} RPM)";
}

/// <summary>
/// V5 Optical Sensor for color detection and object sensing.
/// </summary>
public class V5OpticalSensor : V5Device
{
    // Color detection
    public double Hue { get; private set; }                // 0-360
    public double Saturation { get; private set; }         // 0-100%
    public double Brightness { get; private set; }         // 0-100%

    // Object detection
    public int Proximity { get; private set; }             // 0-255
    public bool IsObjectDetected => Proximity > ProximityThreshold;

    // Light settings
    public int LedBrightness { get; set; } = 100;          // 0-100%
    public int ProximityThreshold { get; set; } = 50;

    // Gesture detection (if supported)
    public GestureType LastGesture { get; private set; }

    public V5OpticalSensor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        if (data.Length < 25) return;

        Hue = BitConverter.ToDouble(data, 0);
        Saturation = BitConverter.ToDouble(data, 8);
        Brightness = BitConverter.ToDouble(data, 16);
        Proximity = data[24];

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Detect if the color is approximately red.
    /// </summary>
    public bool IsRed => (Hue < 30 || Hue > 330) && Saturation > 50;

    /// <summary>
    /// Detect if the color is approximately blue.
    /// </summary>
    public bool IsBlue => Hue > 180 && Hue < 260 && Saturation > 50;

    /// <summary>
    /// Detect if the color is approximately green.
    /// </summary>
    public bool IsGreen => Hue > 80 && Hue < 160 && Saturation > 50;

    /// <summary>
    /// Detect if the color is approximately yellow.
    /// </summary>
    public bool IsYellow => Hue > 40 && Hue < 80 && Saturation > 50;

    /// <summary>
    /// Get the detected color as an enum.
    /// </summary>
    public DetectedColor GetColor()
    {
        if (Saturation < 20) return Brightness > 50 ? DetectedColor.White : DetectedColor.Black;
        if (IsRed) return DetectedColor.Red;
        if (IsBlue) return DetectedColor.Blue;
        if (IsGreen) return DetectedColor.Green;
        if (IsYellow) return DetectedColor.Yellow;
        if (Hue > 260 && Hue < 330) return DetectedColor.Purple;
        if (Hue > 10 && Hue < 40) return DetectedColor.Orange;
        return DetectedColor.Unknown;
    }

    /// <summary>
    /// Check if detected color matches a target hue within tolerance.
    /// </summary>
    public bool IsColor(double targetHue, double tolerance = 20)
    {
        double diff = System.Math.Abs(Hue - targetHue);
        if (diff > 180) diff = 360 - diff;
        return diff < tolerance && Saturation > 30;
    }

    public override string ToString() =>
        $"{Name}: {GetColor()} (H:{Hue:F0} S:{Saturation:F0}% B:{Brightness:F0}%)";
}

public enum DetectedColor
{
    Unknown,
    Red,
    Orange,
    Yellow,
    Green,
    Blue,
    Purple,
    White,
    Black
}

public enum GestureType
{
    None,
    Up,
    Down,
    Left,
    Right
}

/// <summary>
/// V5 Distance Sensor for range measurement.
/// </summary>
public class V5DistanceSensor : V5Device
{
    public double Distance { get; private set; }           // mm
    public double Velocity { get; private set; }           // mm/s (approach rate)
    public int Confidence { get; private set; }            // 0-100%
    public int ObjectSize { get; private set; }            // relative size

    public double MinRange { get; } = 20;                  // mm
    public double MaxRange { get; } = 2000;                // mm

    public V5DistanceSensor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        if (data.Length < 25) return;

        Distance = BitConverter.ToDouble(data, 1);
        Velocity = BitConverter.ToDouble(data, 9);
        Confidence = BitConverter.ToInt32(data, 17);
        ObjectSize = BitConverter.ToInt32(data, 21);

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Distance in inches.
    /// </summary>
    public double DistanceInches => Distance / 25.4;

    /// <summary>
    /// Distance in centimeters.
    /// </summary>
    public double DistanceCm => Distance / 10.0;

    /// <summary>
    /// Check if object is detected within range.
    /// </summary>
    public bool IsObjectDetected => Distance > MinRange && Distance < MaxRange && Confidence > 50;

    /// <summary>
    /// Check if object is within a specific distance.
    /// </summary>
    public bool IsObjectWithin(double distanceMm) => IsObjectDetected && Distance < distanceMm;

    /// <summary>
    /// Check if object is approaching (negative velocity = approaching).
    /// </summary>
    public bool IsObjectApproaching => Velocity < -10;

    /// <summary>
    /// Estimate time to contact based on distance and velocity.
    /// </summary>
    public TimeSpan? TimeToContact
    {
        get
        {
            if (!IsObjectApproaching || Velocity >= 0) return null;
            return TimeSpan.FromSeconds(Distance / System.Math.Abs(Velocity));
        }
    }

    public override string ToString() =>
        $"{Name}: {DistanceInches:F1}\" ({Confidence}% confidence)";
}

/// <summary>
/// V5 Vision Sensor for object/color blob detection.
/// </summary>
public class V5VisionSensor : V5Device
{
    public List<VisionObject> Objects { get; } = new();
    public int ObjectCount => Objects.Count;

    // Signature definitions (set up in VEX brain or code)
    public Dictionary<int, string> SignatureNames { get; } = new()
    {
        { 1, "Signature 1" },
        { 2, "Signature 2" },
        { 3, "Signature 3" },
        { 4, "Signature 4" },
        { 5, "Signature 5" },
        { 6, "Signature 6" },
        { 7, "Signature 7" }
    };

    public int CameraWidth { get; } = 316;
    public int CameraHeight { get; } = 212;
    public int CameraCenterX => CameraWidth / 2;
    public int CameraCenterY => CameraHeight / 2;

    public V5VisionSensor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        Objects.Clear();
        
        if (data.Length < 2) return;
        int count = data[0];
        int offset = 1;

        for (int i = 0; i < count && offset + 12 <= data.Length; i++)
        {
            Objects.Add(new VisionObject
            {
                Signature = data[offset],
                X = BitConverter.ToInt16(data, offset + 1),
                Y = BitConverter.ToInt16(data, offset + 3),
                Width = BitConverter.ToInt16(data, offset + 5),
                Height = BitConverter.ToInt16(data, offset + 7),
                Angle = BitConverter.ToInt16(data, offset + 9)
            });
            offset += 11;
        }

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Get the largest object detected.
    /// </summary>
    public VisionObject? LargestObject =>
        Objects.OrderByDescending(o => o.Area).FirstOrDefault();

    /// <summary>
    /// Get objects by signature ID.
    /// </summary>
    public IEnumerable<VisionObject> GetBySignature(int signature) =>
        Objects.Where(o => o.Signature == signature);

    /// <summary>
    /// Get the largest object of a specific signature.
    /// </summary>
    public VisionObject? GetLargestBySignature(int signature) =>
        GetBySignature(signature).OrderByDescending(o => o.Area).FirstOrDefault();

    /// <summary>
    /// Check if any object of a signature is detected.
    /// </summary>
    public bool HasObject(int signature) =>
        Objects.Any(o => o.Signature == signature);

    /// <summary>
    /// Get horizontal offset from center (-1 to 1) for the largest object.
    /// Useful for aiming/tracking.
    /// </summary>
    public double GetAimOffset(int signature)
    {
        var obj = GetLargestBySignature(signature);
        if (obj == null) return 0;
        return (obj.CenterX - CameraCenterX) / (double)CameraCenterX;
    }

    public override string ToString() =>
        $"{Name}: {ObjectCount} objects detected";
}

public class VisionObject
{
    public int Signature { get; set; }
    public int X { get; set; }           // Top-left X
    public int Y { get; set; }           // Top-left Y
    public int Width { get; set; }
    public int Height { get; set; }
    public int Angle { get; set; }       // For color codes

    public int CenterX => X + Width / 2;
    public int CenterY => Y + Height / 2;
    public int Area => Width * Height;

    public override string ToString() =>
        $"Sig{Signature}: ({CenterX},{CenterY}) {Width}x{Height}";
}

/// <summary>
/// V5 GPS Sensor for field-relative positioning.
/// </summary>
public class V5GpsSensor : V5Device
{
    public double X { get; private set; }                  // inches from field center
    public double Y { get; private set; }                  // inches from field center
    public double Heading { get; private set; }            // degrees
    public double Pitch { get; private set; }              // degrees
    public double Roll { get; private set; }               // degrees
    public int Quality { get; private set; }               // 0-100%

    // Field dimensions (VRC standard)
    public static double FieldWidth { get; } = 144;        // inches (12 feet)
    public static double FieldHeight { get; } = 144;       // inches (12 feet)

    public V5GpsSensor(int port, string? name = null) : base(port, name) { }

    public override void Update(byte[] data)
    {
        if (data.Length < 45) return;

        X = BitConverter.ToDouble(data, 1);
        Y = BitConverter.ToDouble(data, 9);
        Heading = BitConverter.ToDouble(data, 17);
        Pitch = BitConverter.ToDouble(data, 25);
        Roll = BitConverter.ToDouble(data, 33);
        Quality = BitConverter.ToInt32(data, 41);

        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    /// <summary>
    /// Check if GPS fix is reliable.
    /// </summary>
    public bool HasGoodFix => Quality > 80;

    /// <summary>
    /// Heading in radians.
    /// </summary>
    public double HeadingRadians => Heading * System.Math.PI / 180.0;

    /// <summary>
    /// Distance from field center.
    /// </summary>
    public double DistanceFromCenter => System.Math.Sqrt(X * X + Y * Y);

    /// <summary>
    /// Check if robot is in a specific quadrant.
    /// </summary>
    public FieldQuadrant GetQuadrant()
    {
        if (X >= 0 && Y >= 0) return FieldQuadrant.NorthEast;
        if (X < 0 && Y >= 0) return FieldQuadrant.NorthWest;
        if (X < 0 && Y < 0) return FieldQuadrant.SouthWest;
        return FieldQuadrant.SouthEast;
    }

    /// <summary>
    /// Calculate distance to a target point.
    /// </summary>
    public double DistanceTo(double targetX, double targetY)
    {
        double dx = targetX - X;
        double dy = targetY - Y;
        return System.Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>
    /// Calculate heading to a target point.
    /// </summary>
    public double HeadingTo(double targetX, double targetY)
    {
        double dx = targetX - X;
        double dy = targetY - Y;
        double heading = System.Math.Atan2(dx, dy) * 180.0 / System.Math.PI;
        return heading < 0 ? heading + 360 : heading;
    }

    public override string ToString() =>
        $"{Name}: ({X:F1}, {Y:F1}) @ {Heading:F1}° [{Quality}%]";
}

public enum FieldQuadrant
{
    NorthEast,
    NorthWest,
    SouthWest,
    SouthEast
}

/// <summary>
/// V5 3-Wire Expander port for legacy sensors.
/// </summary>
public class V5ThreeWireExpander : V5Device
{
    public ThreeWirePort[] Ports { get; } = new ThreeWirePort[8];

    public V5ThreeWireExpander(int port, string? name = null) : base(port, name)
    {
        for (int i = 0; i < 8; i++)
        {
            Ports[i] = new ThreeWirePort((char)('A' + i));
        }
    }

    public ThreeWirePort this[char portLetter] => Ports[portLetter - 'A'];
    public ThreeWirePort this[int index] => Ports[index];

    public override void Update(byte[] data)
    {
        // Update individual port values
        for (int i = 0; i < 8 && i * 2 + 1 < data.Length; i++)
        {
            Ports[i].RawValue = BitConverter.ToInt16(data, i * 2);
        }
        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }
}

public class ThreeWirePort
{
    public char Letter { get; }
    public ThreeWireDeviceType DeviceType { get; set; } = ThreeWireDeviceType.None;
    public short RawValue { get; set; }

    public ThreeWirePort(char letter)
    {
        Letter = letter;
    }

    /// <summary>
    /// Value as percentage (for potentiometer, line sensor).
    /// </summary>
    public double Percentage => RawValue / 4095.0 * 100.0;

    /// <summary>
    /// For digital inputs.
    /// </summary>
    public bool IsHigh => RawValue > 2048;

    /// <summary>
    /// For bumper/limit switches.
    /// </summary>
    public bool IsPressed => IsHigh;
}

public enum ThreeWireDeviceType
{
    None,
    DigitalIn,
    DigitalOut,
    AnalogIn,
    AnalogOut,
    Potentiometer,
    LineSensor,
    LightSensor,
    Bumper,
    LimitSwitch,
    Encoder,
    Ultrasonic,
    Servo,
    Motor393
}
