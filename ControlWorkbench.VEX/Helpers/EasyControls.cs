namespace ControlWorkbench.VEX.Helpers;

/// <summary>
/// Easy-to-use drivetrain control for beginners.
/// Handles common drive modes and motor management.
/// </summary>
public class Drivetrain
{
    private readonly List<int> _leftMotorPorts = new();
    private readonly List<int> _rightMotorPorts = new();
    
    public double WheelDiameter { get; set; } = 4.0;           // inches
    public double TrackWidth { get; set; } = 12.0;             // inches
    public double GearRatio { get; set; } = 1.0;               // motor:wheel
    public MotorCartridge Cartridge { get; set; } = MotorCartridge.Blue;

    public DriveMode Mode { get; set; } = DriveMode.Tank;
    public int MaxSpeed { get; set; } = 127;
    public int Deadband { get; set; } = 5;

    // Current state
    public int LeftPower { get; private set; }
    public int RightPower { get; private set; }
    public bool IsMoving => LeftPower != 0 || RightPower != 0;

    // Calculated properties
    public double WheelCircumference => WheelDiameter * System.Math.PI;
    public double MaxRpm => Cartridge switch
    {
        MotorCartridge.Red => 100,
        MotorCartridge.Green => 200,
        MotorCartridge.Blue => 600,
        _ => 200
    } / GearRatio;
    public double MaxVelocity => MaxRpm / 60.0 * WheelCircumference;  // inches/sec

    /// <summary>
    /// Configure drivetrain with motor ports.
    /// </summary>
    public void Configure(int[] leftPorts, int[] rightPorts)
    {
        _leftMotorPorts.Clear();
        _leftMotorPorts.AddRange(leftPorts);
        _rightMotorPorts.Clear();
        _rightMotorPorts.AddRange(rightPorts);
    }

    /// <summary>
    /// Get motor commands from controller input.
    /// </summary>
    public (int left, int right) GetDriveOutput(int leftY, int leftX, int rightY, int rightX)
    {
        // Apply deadband
        leftY = ApplyDeadband(leftY);
        leftX = ApplyDeadband(leftX);
        rightY = ApplyDeadband(rightY);
        rightX = ApplyDeadband(rightX);

        int left, right;

        switch (Mode)
        {
            case DriveMode.Tank:
                left = leftY;
                right = rightY;
                break;

            case DriveMode.Arcade:
                left = leftY + rightX;
                right = leftY - rightX;
                break;

            case DriveMode.ArcadeSingleStick:
                left = leftY + leftX;
                right = leftY - leftX;
                break;

            case DriveMode.Curvature:
                double speed = leftY / 127.0;
                double turn = rightX / 127.0 * System.Math.Abs(speed);
                left = (int)((speed + turn) * 127);
                right = (int)((speed - turn) * 127);
                break;

            default:
                left = 0;
                right = 0;
                break;
        }

        // Clamp to max speed
        left = Clamp(left, -MaxSpeed, MaxSpeed);
        right = Clamp(right, -MaxSpeed, MaxSpeed);

        LeftPower = left;
        RightPower = right;

        return (left, right);
    }

    /// <summary>
    /// Set motor power directly.
    /// </summary>
    public void SetPower(int left, int right)
    {
        LeftPower = Clamp(left, -127, 127);
        RightPower = Clamp(right, -127, 127);
    }

    /// <summary>
    /// Stop all motors.
    /// </summary>
    public void Stop()
    {
        LeftPower = 0;
        RightPower = 0;
    }

    /// <summary>
    /// Calculate distance to drive for a given number of rotations.
    /// </summary>
    public double RotationsToDistance(double rotations) =>
        rotations * WheelCircumference / GearRatio;

    /// <summary>
    /// Calculate rotations needed for a given distance.
    /// </summary>
    public double DistanceToRotations(double distance) =>
        distance / WheelCircumference * GearRatio;

    /// <summary>
    /// Calculate degrees needed for a given distance.
    /// </summary>
    public double DistanceToDegrees(double distance) =>
        DistanceToRotations(distance) * 360;

    /// <summary>
    /// Calculate arc length for a turn.
    /// </summary>
    public double TurnToDegrees(double turnAngleDegrees)
    {
        double arcLength = (turnAngleDegrees / 360.0) * (System.Math.PI * TrackWidth);
        return DistanceToDegrees(arcLength);
    }

    private int ApplyDeadband(int value) =>
        System.Math.Abs(value) < Deadband ? 0 : value;

    private int Clamp(int value, int min, int max) =>
        System.Math.Max(min, System.Math.Min(max, value));
}

public enum DriveMode
{
    Tank,
    Arcade,
    ArcadeSingleStick,
    Curvature
}

/// <summary>
/// Simple PID controller for beginners.
/// Pre-tuned defaults for common VEX applications.
/// </summary>
public class EasyPid
{
    public double Kp { get; set; }
    public double Ki { get; set; }
    public double Kd { get; set; }

    private double _integral;
    private double _previousError;
    private double _previousMeasurement;
    private DateTime _lastTime;
    private bool _initialized;

    public double Output { get; private set; }
    public double Error { get; private set; }
    public double Target { get; private set; }

    // Limits
    public double OutputMin { get; set; } = -127;
    public double OutputMax { get; set; } = 127;
    public double IntegralMax { get; set; } = 1000;

    // Options
    public bool UseDerivativeOnMeasurement { get; set; } = true;

    /// <summary>
    /// Create PID with custom gains.
    /// </summary>
    public EasyPid(double kp, double ki = 0, double kd = 0)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    /// <summary>
    /// Create PID for a specific application with pre-tuned gains.
    /// </summary>
    public static EasyPid ForDriveDistance() => new(2.0, 0.01, 0.1);
    public static EasyPid ForTurning() => new(1.5, 0.005, 0.2);
    public static EasyPid ForLiftPosition() => new(1.0, 0.02, 0.05);
    public static EasyPid ForFlywheel() => new(0.5, 0.1, 0.01);
    public static EasyPid ForLineFollowing() => new(0.8, 0.0, 0.3);

    /// <summary>
    /// Compute PID output.
    /// </summary>
    public double Compute(double target, double measurement)
    {
        Target = target;
        Error = target - measurement;

        // Calculate dt
        var now = DateTime.UtcNow;
        double dt = _initialized ? (now - _lastTime).TotalSeconds : 0.01;
        _lastTime = now;

        if (!_initialized)
        {
            _initialized = true;
            _previousError = Error;
            _previousMeasurement = measurement;
            return 0;
        }

        // P term
        double pTerm = Kp * Error;

        // I term with anti-windup
        _integral += Error * dt;
        _integral = Clamp(_integral, -IntegralMax, IntegralMax);
        double iTerm = Ki * _integral;

        // D term
        double derivative;
        if (UseDerivativeOnMeasurement)
        {
            derivative = -(measurement - _previousMeasurement) / dt;
            _previousMeasurement = measurement;
        }
        else
        {
            derivative = (Error - _previousError) / dt;
            _previousError = Error;
        }
        double dTerm = Kd * derivative;

        // Total output
        Output = Clamp(pTerm + iTerm + dTerm, OutputMin, OutputMax);
        return Output;
    }

    /// <summary>
    /// Check if target is reached (within tolerance).
    /// </summary>
    public bool IsSettled(double tolerance = 1.0) =>
        System.Math.Abs(Error) < tolerance;

    /// <summary>
    /// Reset the controller.
    /// </summary>
    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _previousMeasurement = 0;
        _initialized = false;
        Output = 0;
        Error = 0;
    }

    private double Clamp(double value, double min, double max) =>
        System.Math.Max(min, System.Math.Min(max, value));
}

/// <summary>
/// Simple motion profiling for smooth movements.
/// </summary>
public class EasyMotion
{
    public double MaxVelocity { get; set; } = 60;          // units/sec
    public double MaxAcceleration { get; set; } = 120;     // units/sec^2

    /// <summary>
    /// Calculate velocity for a given distance remaining.
    /// Uses triangular profile for deceleration.
    /// </summary>
    public double GetVelocity(double distanceRemaining, double currentVelocity = 0)
    {
        double direction = System.Math.Sign(distanceRemaining);
        double distance = System.Math.Abs(distanceRemaining);

        // Velocity to stop in remaining distance
        double stoppingVelocity = System.Math.Sqrt(2 * MaxAcceleration * distance);

        // Take minimum of max velocity and stopping velocity
        double targetVelocity = System.Math.Min(MaxVelocity, stoppingVelocity);

        return targetVelocity * direction;
    }

    /// <summary>
    /// Calculate motor power for smooth movement.
    /// </summary>
    public int GetPower(double distanceRemaining, double maxPower = 127)
    {
        double velocity = GetVelocity(distanceRemaining);
        double power = (velocity / MaxVelocity) * maxPower;
        return (int)System.Math.Clamp(power, -maxPower, maxPower);
    }

    /// <summary>
    /// Check if movement is complete.
    /// </summary>
    public bool IsComplete(double distanceRemaining, double tolerance = 0.5) =>
        System.Math.Abs(distanceRemaining) < tolerance;
}

/// <summary>
/// Color matching helper for game object detection.
/// </summary>
public class ColorMatcher
{
    private readonly List<ColorSignature> _signatures = new();

    /// <summary>
    /// Add a color signature to match against.
    /// </summary>
    public void AddColor(string name, double hue, double hueTolerance = 20, double minSaturation = 30)
    {
        _signatures.Add(new ColorSignature
        {
            Name = name,
            Hue = hue,
            HueTolerance = hueTolerance,
            MinSaturation = minSaturation
        });
    }

    /// <summary>
    /// Add standard VRC game colors.
    /// </summary>
    public void AddVrcColors()
    {
        AddColor("Red", 0, 30, 50);      // Red rings/discs
        AddColor("Blue", 220, 40, 50);   // Blue rings/discs
        AddColor("Yellow", 60, 20, 50);  // Yellow elements
        AddColor("Green", 120, 30, 40);  // Green elements
    }

    /// <summary>
    /// Match a color reading to known signatures.
    /// </summary>
    public string? Match(double hue, double saturation)
    {
        foreach (var sig in _signatures)
        {
            if (saturation < sig.MinSaturation) continue;

            double diff = System.Math.Abs(hue - sig.Hue);
            if (diff > 180) diff = 360 - diff;

            if (diff < sig.HueTolerance)
                return sig.Name;
        }
        return null;
    }

    /// <summary>
    /// Check if a specific color is detected.
    /// </summary>
    public bool IsColor(string name, double hue, double saturation)
    {
        var match = Match(hue, saturation);
        return match?.Equals(name, StringComparison.OrdinalIgnoreCase) == true;
    }

    private class ColorSignature
    {
        public string Name { get; set; } = "";
        public double Hue { get; set; }
        public double HueTolerance { get; set; }
        public double MinSaturation { get; set; }
    }
}

/// <summary>
/// Simple state machine for autonomous routines.
/// </summary>
public class AutonStateMachine
{
    private readonly List<AutonState> _states = new();
    private int _currentIndex = -1;
    private DateTime _stateStartTime;

    public string CurrentStateName => _currentIndex >= 0 && _currentIndex < _states.Count
        ? _states[_currentIndex].Name
        : "Not Started";
    public bool IsComplete => _currentIndex >= _states.Count;
    public TimeSpan StateElapsed => DateTime.UtcNow - _stateStartTime;

    public event Action<string>? StateChanged;

    /// <summary>
    /// Add a state to the sequence.
    /// </summary>
    public AutonStateMachine AddState(string name, Action action, Func<bool> isComplete, double timeoutSeconds = 5.0)
    {
        _states.Add(new AutonState
        {
            Name = name,
            Action = action,
            IsComplete = isComplete,
            Timeout = TimeSpan.FromSeconds(timeoutSeconds)
        });
        return this;
    }

    /// <summary>
    /// Add a timed state (runs for specified duration).
    /// </summary>
    public AutonStateMachine AddTimedState(string name, Action action, double durationSeconds)
    {
        var timeout = TimeSpan.FromSeconds(durationSeconds);
        return AddState(name, action, () => StateElapsed >= timeout, durationSeconds + 1);
    }

    /// <summary>
    /// Add a wait/delay state.
    /// </summary>
    public AutonStateMachine AddDelay(double seconds)
    {
        return AddTimedState($"Wait {seconds}s", () => { }, seconds);
    }

    /// <summary>
    /// Start the autonomous sequence.
    /// </summary>
    public void Start()
    {
        _currentIndex = 0;
        _stateStartTime = DateTime.UtcNow;
        StateChanged?.Invoke(CurrentStateName);
    }

    /// <summary>
    /// Update the state machine (call every loop iteration).
    /// </summary>
    public void Update()
    {
        if (IsComplete || _currentIndex < 0) return;

        var state = _states[_currentIndex];

        // Run the action
        state.Action();

        // Check for completion or timeout
        if (state.IsComplete() || StateElapsed > state.Timeout)
        {
            _currentIndex++;
            _stateStartTime = DateTime.UtcNow;
            StateChanged?.Invoke(CurrentStateName);
        }
    }

    /// <summary>
    /// Reset to start.
    /// </summary>
    public void Reset()
    {
        _currentIndex = -1;
    }

    private class AutonState
    {
        public string Name { get; set; } = "";
        public Action Action { get; set; } = () => { };
        public Func<bool> IsComplete { get; set; } = () => true;
        public TimeSpan Timeout { get; set; }
    }
}

/// <summary>
/// Slew rate limiter for smooth motor control.
/// Prevents jerky movements and reduces wear.
/// </summary>
public class SlewRateLimiter
{
    public double SlewRate { get; set; }  // units per second
    private double _currentValue;
    private DateTime _lastUpdate;
    private bool _initialized;

    /// <summary>
    /// Create a slew rate limiter.
    /// </summary>
    /// <param name="slewRate">Maximum change per second</param>
    public SlewRateLimiter(double slewRate = 500)
    {
        SlewRate = slewRate;
    }

    /// <summary>
    /// Get the rate-limited output.
    /// </summary>
    public double Calculate(double target)
    {
        var now = DateTime.UtcNow;
        
        if (!_initialized)
        {
            _initialized = true;
            _currentValue = target;
            _lastUpdate = now;
            return target;
        }

        double dt = (now - _lastUpdate).TotalSeconds;
        _lastUpdate = now;

        double maxChange = SlewRate * dt;
        double change = target - _currentValue;
        change = System.Math.Clamp(change, -maxChange, maxChange);
        
        _currentValue += change;
        return _currentValue;
    }

    /// <summary>
    /// Reset the limiter.
    /// </summary>
    public void Reset()
    {
        _initialized = false;
        _currentValue = 0;
    }
}

/// <summary>
/// Toggle switch helper for button control.
/// </summary>
public class Toggle
{
    private bool _state;
    private bool _previousButton;

    public bool State => _state;

    /// <summary>
    /// Update the toggle with button state.
    /// </summary>
    public bool Update(bool buttonPressed)
    {
        // Detect rising edge
        if (buttonPressed && !_previousButton)
        {
            _state = !_state;
        }
        _previousButton = buttonPressed;
        return _state;
    }

    /// <summary>
    /// Set the state directly.
    /// </summary>
    public void Set(bool state) => _state = state;

    /// <summary>
    /// Toggle the state.
    /// </summary>
    public void Flip() => _state = !_state;

    /// <summary>
    /// Reset to off.
    /// </summary>
    public void Reset()
    {
        _state = false;
        _previousButton = false;
    }
}
