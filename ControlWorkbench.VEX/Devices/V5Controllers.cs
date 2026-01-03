namespace ControlWorkbench.VEX.Devices;

/// <summary>
/// V5 Controller (Joystick) for driver input.
/// </summary>
public class V5Controller
{
    // Analog sticks (-127 to 127)
    public int LeftStickX { get; private set; }
    public int LeftStickY { get; private set; }
    public int RightStickX { get; private set; }
    public int RightStickY { get; private set; }

    // Deadband (values below this are treated as 0)
    public int Deadband { get; set; } = 5;

    // Buttons - current state
    public bool L1 { get; private set; }
    public bool L2 { get; private set; }
    public bool R1 { get; private set; }
    public bool R2 { get; private set; }
    public bool Up { get; private set; }
    public bool Down { get; private set; }
    public bool Left { get; private set; }
    public bool Right { get; private set; }
    public bool A { get; private set; }
    public bool B { get; private set; }
    public bool X { get; private set; }
    public bool Y { get; private set; }

    // Button events - just pressed this frame
    public bool L1Pressed { get; private set; }
    public bool L2Pressed { get; private set; }
    public bool R1Pressed { get; private set; }
    public bool R2Pressed { get; private set; }
    public bool UpPressed { get; private set; }
    public bool DownPressed { get; private set; }
    public bool LeftPressed { get; private set; }
    public bool RightPressed { get; private set; }
    public bool APressed { get; private set; }
    public bool BPressed { get; private set; }
    public bool XPressed { get; private set; }
    public bool YPressed { get; private set; }

    // Button events - just released this frame
    public bool L1Released { get; private set; }
    public bool L2Released { get; private set; }
    public bool R1Released { get; private set; }
    public bool R2Released { get; private set; }

    // Previous state for edge detection
    private ushort _previousButtons;

    // Connection
    public bool IsConnected { get; private set; }
    public DateTime LastUpdate { get; private set; }

    public void Update(byte[] data)
    {
        if (data.Length < 10) return;

        LeftStickX = ApplyDeadband((sbyte)data[0]);
        LeftStickY = ApplyDeadband((sbyte)data[1]);
        RightStickX = ApplyDeadband((sbyte)data[2]);
        RightStickY = ApplyDeadband((sbyte)data[3]);

        ushort buttons = BitConverter.ToUInt16(data, 4);
        ushort pressed = BitConverter.ToUInt16(data, 6);
        ushort released = BitConverter.ToUInt16(data, 8);

        // Update button states
        L1 = (buttons & 0x0001) != 0;
        L2 = (buttons & 0x0002) != 0;
        R1 = (buttons & 0x0004) != 0;
        R2 = (buttons & 0x0008) != 0;
        Up = (buttons & 0x0010) != 0;
        Down = (buttons & 0x0020) != 0;
        Left = (buttons & 0x0040) != 0;
        Right = (buttons & 0x0080) != 0;
        X = (buttons & 0x0100) != 0;
        B = (buttons & 0x0200) != 0;
        Y = (buttons & 0x0400) != 0;
        A = (buttons & 0x0800) != 0;

        // Update pressed events
        L1Pressed = (pressed & 0x0001) != 0;
        L2Pressed = (pressed & 0x0002) != 0;
        R1Pressed = (pressed & 0x0004) != 0;
        R2Pressed = (pressed & 0x0008) != 0;
        UpPressed = (pressed & 0x0010) != 0;
        DownPressed = (pressed & 0x0020) != 0;
        LeftPressed = (pressed & 0x0040) != 0;
        RightPressed = (pressed & 0x0080) != 0;
        XPressed = (pressed & 0x0100) != 0;
        BPressed = (pressed & 0x0200) != 0;
        YPressed = (pressed & 0x0400) != 0;
        APressed = (pressed & 0x0800) != 0;

        // Update released events
        L1Released = (released & 0x0001) != 0;
        L2Released = (released & 0x0002) != 0;
        R1Released = (released & 0x0004) != 0;
        R2Released = (released & 0x0008) != 0;

        _previousButtons = buttons;
        LastUpdate = DateTime.UtcNow;
        IsConnected = true;
    }

    private int ApplyDeadband(int value)
    {
        return System.Math.Abs(value) < Deadband ? 0 : value;
    }

    /// <summary>
    /// Get left stick as normalized vector (-1 to 1).
    /// </summary>
    public (double x, double y) LeftStickNormalized =>
        (LeftStickX / 127.0, LeftStickY / 127.0);

    /// <summary>
    /// Get right stick as normalized vector (-1 to 1).
    /// </summary>
    public (double x, double y) RightStickNormalized =>
        (RightStickX / 127.0, RightStickY / 127.0);

    /// <summary>
    /// Get left stick magnitude (0 to 1).
    /// </summary>
    public double LeftStickMagnitude
    {
        get
        {
            var (x, y) = LeftStickNormalized;
            return System.Math.Min(1.0, System.Math.Sqrt(x * x + y * y));
        }
    }

    /// <summary>
    /// Get left stick angle in degrees (0 = up, 90 = right).
    /// </summary>
    public double LeftStickAngle
    {
        get
        {
            var angle = System.Math.Atan2(LeftStickX, LeftStickY) * 180.0 / System.Math.PI;
            return angle < 0 ? angle + 360 : angle;
        }
    }

    /// <summary>
    /// Check if any button is currently pressed.
    /// </summary>
    public bool AnyButtonPressed =>
        L1 || L2 || R1 || R2 || Up || Down || Left || Right || A || B || X || Y;

    /// <summary>
    /// Get tank drive values from sticks.
    /// </summary>
    public (int left, int right) GetTankDrive() =>
        (LeftStickY, RightStickY);

    /// <summary>
    /// Get arcade drive values from sticks.
    /// </summary>
    public (int left, int right) GetArcadeDrive(bool singleStick = false)
    {
        int forward = singleStick ? LeftStickY : LeftStickY;
        int turn = singleStick ? LeftStickX : RightStickX;

        return (forward + turn, forward - turn);
    }

    /// <summary>
    /// Get curvature drive values (like Mario Kart).
    /// </summary>
    public (int left, int right) GetCurvatureDrive()
    {
        double speed = LeftStickY / 127.0;
        double turn = RightStickX / 127.0;

        // Apply curvature (turn rate depends on speed)
        double curvature = turn * System.Math.Abs(speed);

        double left = speed + curvature;
        double right = speed - curvature;

        // Normalize if exceeds limits
        double maxMag = System.Math.Max(System.Math.Abs(left), System.Math.Abs(right));
        if (maxMag > 1.0)
        {
            left /= maxMag;
            right /= maxMag;
        }

        return ((int)(left * 127), (int)(right * 127));
    }

    public override string ToString() =>
        $"Controller: L({LeftStickX},{LeftStickY}) R({RightStickX},{RightStickY})";
}

/// <summary>
/// V5 Battery status and health monitoring.
/// </summary>
public class V5Battery
{
    public double Voltage { get; private set; }            // Volts
    public double Current { get; private set; }            // Amps
    public double Capacity { get; private set; }           // Percentage
    public double Temperature { get; private set; }        // Celsius

    public DateTime LastUpdate { get; private set; }

    // Thresholds
    public double LowVoltageThreshold { get; set; } = 7.2;
    public double CriticalVoltageThreshold { get; set; } = 6.8;

    public void Update(byte[] data)
    {
        if (data.Length < 32) return;

        Voltage = BitConverter.ToDouble(data, 0);
        Current = BitConverter.ToDouble(data, 8);
        Capacity = BitConverter.ToDouble(data, 16);
        Temperature = BitConverter.ToDouble(data, 24);

        LastUpdate = DateTime.UtcNow;
    }

    /// <summary>
    /// Battery health status.
    /// </summary>
    public BatteryStatus Status
    {
        get
        {
            if (Voltage < CriticalVoltageThreshold) return BatteryStatus.Critical;
            if (Voltage < LowVoltageThreshold) return BatteryStatus.Low;
            if (Capacity > 80) return BatteryStatus.Good;
            if (Capacity > 50) return BatteryStatus.Moderate;
            return BatteryStatus.Low;
        }
    }

    /// <summary>
    /// Estimated matches remaining based on capacity.
    /// </summary>
    public int EstimatedMatchesRemaining => (int)(Capacity / 15);  // ~15% per match

    /// <summary>
    /// Power being consumed.
    /// </summary>
    public double PowerWatts => Voltage * Current;

    /// <summary>
    /// Check if battery needs charging.
    /// </summary>
    public bool NeedsCharging => Capacity < 30 || Voltage < LowVoltageThreshold;

    public override string ToString() =>
        $"Battery: {Voltage:F2}V {Capacity:F0}% ({Status})";
}

public enum BatteryStatus
{
    Good,
    Moderate,
    Low,
    Critical
}

/// <summary>
/// Competition state and timing.
/// </summary>
public class V5Competition
{
    public bool IsEnabled { get; private set; }
    public bool IsAutonomous { get; private set; }
    public bool IsDriverControl { get; private set; }
    public bool IsConnected { get; private set; }

    public CompetitionPhase Phase { get; private set; }

    public void Update(byte[] data)
    {
        if (data.Length < 1) return;

        byte status = data[0];
        IsEnabled = (status & 0x01) != 0;
        IsAutonomous = (status & 0x02) != 0;
        IsConnected = (status & 0x04) != 0;

        IsDriverControl = IsEnabled && !IsAutonomous;

        if (!IsEnabled)
            Phase = CompetitionPhase.Disabled;
        else if (IsAutonomous)
            Phase = CompetitionPhase.Autonomous;
        else
            Phase = CompetitionPhase.DriverControl;
    }

    public override string ToString() =>
        $"Competition: {Phase} (Connected: {IsConnected})";
}

public enum CompetitionPhase
{
    Disabled,
    Autonomous,
    DriverControl
}

/// <summary>
/// Pneumatic solenoid control.
/// </summary>
public class V5Pneumatic
{
    public char Port { get; }
    public string Name { get; set; }
    public bool IsExtended { get; private set; }

    public V5Pneumatic(char port, string? name = null)
    {
        Port = port;
        Name = name ?? $"Pneumatic_{port}";
    }

    /// <summary>
    /// Extend the piston.
    /// </summary>
    public void Extend()
    {
        IsExtended = true;
    }

    /// <summary>
    /// Retract the piston.
    /// </summary>
    public void Retract()
    {
        IsExtended = false;
    }

    /// <summary>
    /// Toggle the piston state.
    /// </summary>
    public void Toggle()
    {
        IsExtended = !IsExtended;
    }

    /// <summary>
    /// Set piston state.
    /// </summary>
    public void Set(bool extended)
    {
        IsExtended = extended;
    }

    public override string ToString() =>
        $"{Name}: {(IsExtended ? "Extended" : "Retracted")}";
}

/// <summary>
/// Servo motor control (for 3-wire servos).
/// </summary>
public class V5Servo
{
    public char Port { get; }
    public string Name { get; set; }
    
    private int _position = 127;  // 0-255

    public V5Servo(char port, string? name = null)
    {
        Port = port;
        Name = name ?? $"Servo_{port}";
    }

    /// <summary>
    /// Servo position (0-255).
    /// </summary>
    public int Position
    {
        get => _position;
        set => _position = System.Math.Clamp(value, 0, 255);
    }

    /// <summary>
    /// Servo position as percentage (0-100).
    /// </summary>
    public double PositionPercent
    {
        get => Position / 255.0 * 100;
        set => Position = (int)(value / 100 * 255);
    }

    /// <summary>
    /// Servo position as angle (0-180 degrees).
    /// </summary>
    public double Angle
    {
        get => Position / 255.0 * 180;
        set => Position = (int)(value / 180 * 255);
    }

    /// <summary>
    /// Set servo to center position.
    /// </summary>
    public void Center() => Position = 127;

    /// <summary>
    /// Set servo to minimum position.
    /// </summary>
    public void Min() => Position = 0;

    /// <summary>
    /// Set servo to maximum position.
    /// </summary>
    public void Max() => Position = 255;

    public override string ToString() =>
        $"{Name}: {Angle:F0}°";
}
