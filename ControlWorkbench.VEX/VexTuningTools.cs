using System.Diagnostics;

namespace ControlWorkbench.VEX;

/// <summary>
/// Real-time PID tuning helper with step response analysis.
/// Actually useful for tuning VEX robots.
/// </summary>
public class PidTuningSession
{
    private readonly List<TuningDataPoint> _data = new();
    private readonly Stopwatch _stopwatch = new();
    private double _setpoint;
    private bool _isRecording;

    public bool IsRecording => _isRecording;
    public int DataPointCount => _data.Count;
    public double CurrentSetpoint => _setpoint;

    // Computed metrics
    public double? RiseTime { get; private set; }
    public double? SettlingTime { get; private set; }
    public double? Overshoot { get; private set; }
    public double? SteadyStateError { get; private set; }
    public double? PeakTime { get; private set; }

    /// <summary>
    /// Start recording a step response.
    /// </summary>
    public void StartStepResponse(double targetSetpoint)
    {
        _data.Clear();
        _setpoint = targetSetpoint;
        _stopwatch.Restart();
        _isRecording = true;
        
        RiseTime = null;
        SettlingTime = null;
        Overshoot = null;
        SteadyStateError = null;
        PeakTime = null;
    }

    /// <summary>
    /// Record a data point during the step response.
    /// </summary>
    public void RecordDataPoint(double measurement, double output)
    {
        if (!_isRecording) return;

        _data.Add(new TuningDataPoint
        {
            TimeMs = _stopwatch.ElapsedMilliseconds,
            Setpoint = _setpoint,
            Measurement = measurement,
            Output = output,
            Error = _setpoint - measurement
        });
    }

    /// <summary>
    /// Stop recording and compute metrics.
    /// </summary>
    public StepResponseMetrics StopAndAnalyze()
    {
        _isRecording = false;
        _stopwatch.Stop();

        if (_data.Count < 10)
        {
            return new StepResponseMetrics { IsValid = false };
        }

        var metrics = AnalyzeStepResponse();
        
        RiseTime = metrics.RiseTimeMs;
        SettlingTime = metrics.SettlingTimeMs;
        Overshoot = metrics.OvershootPercent;
        SteadyStateError = metrics.SteadyStateError;
        PeakTime = metrics.PeakTimeMs;

        return metrics;
    }

    private StepResponseMetrics AnalyzeStepResponse()
    {
        var metrics = new StepResponseMetrics { IsValid = true };

        double initialValue = _data[0].Measurement;
        double targetValue = _setpoint;
        double stepSize = targetValue - initialValue;
        
        if (Math.Abs(stepSize) < 0.001)
        {
            metrics.IsValid = false;
            return metrics;
        }

        double peakValue = initialValue;
        double peakTime = 0;
        bool foundRise10 = false, foundRise90 = false;
        double rise10Time = 0, rise90Time = 0;

        // Analyze the response
        foreach (var point in _data)
        {
            double normalizedResponse = (point.Measurement - initialValue) / stepSize;

            // Rise time (10% to 90%)
            if (!foundRise10 && normalizedResponse >= 0.1)
            {
                rise10Time = point.TimeMs;
                foundRise10 = true;
            }
            if (!foundRise90 && normalizedResponse >= 0.9)
            {
                rise90Time = point.TimeMs;
                foundRise90 = true;
            }

            // Peak detection
            if (stepSize > 0 && point.Measurement > peakValue)
            {
                peakValue = point.Measurement;
                peakTime = point.TimeMs;
            }
            else if (stepSize < 0 && point.Measurement < peakValue)
            {
                peakValue = point.Measurement;
                peakTime = point.TimeMs;
            }
        }

        // Rise time
        if (foundRise10 && foundRise90)
        {
            metrics.RiseTimeMs = rise90Time - rise10Time;
        }

        // Peak time and overshoot
        metrics.PeakTimeMs = peakTime;
        double overshoot = (peakValue - targetValue) / stepSize;
        metrics.OvershootPercent = Math.Max(0, overshoot * 100);

        // Settling time (within 2% of target)
        double tolerance = Math.Abs(stepSize) * 0.02;
        for (int i = _data.Count - 1; i >= 0; i--)
        {
            if (Math.Abs(_data[i].Measurement - targetValue) > tolerance)
            {
                if (i < _data.Count - 1)
                {
                    metrics.SettlingTimeMs = _data[i + 1].TimeMs;
                }
                break;
            }
        }

        // Steady-state error (average of last 10%)
        int lastPoints = Math.Max(5, _data.Count / 10);
        double sumError = 0;
        for (int i = _data.Count - lastPoints; i < _data.Count; i++)
        {
            sumError += _data[i].Error;
        }
        metrics.SteadyStateError = sumError / lastPoints;

        return metrics;
    }

    /// <summary>
    /// Get suggested gain adjustments based on response.
    /// </summary>
    public GainSuggestions GetSuggestions(double currentKp, double currentKi, double currentKd)
    {
        var suggestions = new GainSuggestions();

        if (Overshoot > 20) // Too much overshoot
        {
            suggestions.KpMultiplier = 0.8;
            suggestions.KdMultiplier = 1.3;
            suggestions.Message = "High overshoot: Reduce Kp, increase Kd";
        }
        else if (Overshoot < 5 && RiseTime > 500) // Too slow, no overshoot
        {
            suggestions.KpMultiplier = 1.3;
            suggestions.KiMultiplier = 1.2;
            suggestions.Message = "Slow response: Increase Kp and Ki";
        }
        else if (SteadyStateError > 1) // Steady-state error
        {
            suggestions.KiMultiplier = 1.5;
            suggestions.Message = "Steady-state error: Increase Ki";
        }
        else if (SettlingTime > 1000) // Takes too long to settle
        {
            suggestions.KdMultiplier = 1.2;
            suggestions.Message = "Slow settling: Increase Kd";
        }
        else
        {
            suggestions.Message = "Response looks good!";
        }

        suggestions.SuggestedKp = currentKp * suggestions.KpMultiplier;
        suggestions.SuggestedKi = currentKi * suggestions.KiMultiplier;
        suggestions.SuggestedKd = currentKd * suggestions.KdMultiplier;

        return suggestions;
    }

    /// <summary>
    /// Export data to CSV for external analysis.
    /// </summary>
    public string ExportToCsv()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("time_ms,setpoint,measurement,error,output");
        
        foreach (var point in _data)
        {
            sb.AppendLine($"{point.TimeMs},{point.Setpoint:F4},{point.Measurement:F4},{point.Error:F4},{point.Output:F4}");
        }
        
        return sb.ToString();
    }

    /// <summary>
    /// Get the recorded data for plotting.
    /// </summary>
    public IReadOnlyList<TuningDataPoint> GetData() => _data.AsReadOnly();
}

public class TuningDataPoint
{
    public long TimeMs { get; set; }
    public double Setpoint { get; set; }
    public double Measurement { get; set; }
    public double Error { get; set; }
    public double Output { get; set; }
}

public class StepResponseMetrics
{
    public bool IsValid { get; set; }
    public double? RiseTimeMs { get; set; }
    public double? SettlingTimeMs { get; set; }
    public double? OvershootPercent { get; set; }
    public double? SteadyStateError { get; set; }
    public double? PeakTimeMs { get; set; }

    public override string ToString()
    {
        if (!IsValid) return "Invalid response data";
        
        return $"Rise: {RiseTimeMs:F0}ms, Settle: {SettlingTimeMs:F0}ms, " +
               $"Overshoot: {OvershootPercent:F1}%, SS Error: {SteadyStateError:F3}";
    }
}

public class GainSuggestions
{
    public double KpMultiplier { get; set; } = 1.0;
    public double KiMultiplier { get; set; } = 1.0;
    public double KdMultiplier { get; set; } = 1.0;
    public double SuggestedKp { get; set; }
    public double SuggestedKi { get; set; }
    public double SuggestedKd { get; set; }
    public string Message { get; set; } = "";
}

/// <summary>
/// Motor characterization for feedforward tuning.
/// Run this to determine kS (static friction), kV (velocity), kA (acceleration).
/// </summary>
public class MotorCharacterization
{
    private readonly List<CharacterizationPoint> _data = new();
    private bool _isRunning;

    public bool IsRunning => _isRunning;

    /// <summary>
    /// Record a data point during characterization.
    /// </summary>
    public void RecordPoint(double voltage, double velocity, double acceleration)
    {
        _data.Add(new CharacterizationPoint
        {
            Voltage = voltage,
            Velocity = velocity,
            Acceleration = acceleration
        });
    }

    /// <summary>
    /// Analyze the data to compute feedforward gains.
    /// Uses linear regression: V = kS * sign(v) + kV * v + kA * a
    /// </summary>
    public FeedforwardGains ComputeGains()
    {
        if (_data.Count < 20)
        {
            return new FeedforwardGains { IsValid = false };
        }

        // Separate forward and reverse motion
        var forwardData = _data.Where(p => p.Velocity > 0.1).ToList();
        var reverseData = _data.Where(p => p.Velocity < -0.1).ToList();

        // Simple linear regression for kV (velocity constant)
        // V = kS + kV * velocity (ignoring acceleration for simplicity)
        double sumV = 0, sumVel = 0, sumVVel = 0, sumVel2 = 0;
        int n = forwardData.Count;

        foreach (var point in forwardData)
        {
            sumV += point.Voltage;
            sumVel += point.Velocity;
            sumVVel += point.Voltage * point.Velocity;
            sumVel2 += point.Velocity * point.Velocity;
        }

        double kV = (n * sumVVel - sumV * sumVel) / (n * sumVel2 - sumVel * sumVel);
        double kS = (sumV - kV * sumVel) / n;

        // Compute kA from acceleration data
        var accelData = _data.Where(p => Math.Abs(p.Acceleration) > 0.5).ToList();
        double kA = 0;
        
        if (accelData.Count > 10)
        {
            double sumResidual = 0, sumAccel2 = 0;
            foreach (var point in accelData)
            {
                double predicted = kS * Math.Sign(point.Velocity) + kV * point.Velocity;
                double residual = point.Voltage - predicted;
                sumResidual += residual * point.Acceleration;
                sumAccel2 += point.Acceleration * point.Acceleration;
            }
            kA = sumResidual / sumAccel2;
        }

        return new FeedforwardGains
        {
            IsValid = true,
            kS = Math.Abs(kS),
            kV = kV,
            kA = Math.Max(0, kA),
            DataPoints = _data.Count
        };
    }

    public void Clear() => _data.Clear();
}

public class CharacterizationPoint
{
    public double Voltage { get; set; }
    public double Velocity { get; set; }
    public double Acceleration { get; set; }
}

public class FeedforwardGains
{
    public bool IsValid { get; set; }
    public double kS { get; set; }  // Static friction (volts)
    public double kV { get; set; }  // Velocity gain (volts / (units/sec))
    public double kA { get; set; }  // Acceleration gain (volts / (units/sec^2))
    public int DataPoints { get; set; }

    public override string ToString()
    {
        if (!IsValid) return "Invalid characterization data";
        return $"kS={kS:F4}, kV={kV:F4}, kA={kA:F6} ({DataPoints} points)";
    }

    /// <summary>
    /// Compute feedforward voltage for desired velocity and acceleration.
    /// </summary>
    public double ComputeVoltage(double velocity, double acceleration = 0)
    {
        return kS * Math.Sign(velocity) + kV * velocity + kA * acceleration;
    }
}

/// <summary>
/// Autonomous routine recorder and playback.
/// Records driver actions for later autonomous playback.
/// </summary>
public class AutonomousRecorder
{
    private readonly List<RecordedAction> _actions = new();
    private readonly Stopwatch _stopwatch = new();
    private bool _isRecording;

    public bool IsRecording => _isRecording;
    public int ActionCount => _actions.Count;
    public TimeSpan Duration => _actions.Count > 0 
        ? TimeSpan.FromMilliseconds(_actions[^1].TimeMs) 
        : TimeSpan.Zero;

    public void StartRecording()
    {
        _actions.Clear();
        _stopwatch.Restart();
        _isRecording = true;
    }

    public void StopRecording()
    {
        _stopwatch.Stop();
        _isRecording = false;
    }

    public void RecordDriveAction(double leftPower, double rightPower)
    {
        if (!_isRecording) return;

        _actions.Add(new RecordedAction
        {
            TimeMs = _stopwatch.ElapsedMilliseconds,
            Type = ActionType.Drive,
            Value1 = leftPower,
            Value2 = rightPower
        });
    }

    public void RecordMotorAction(int motorPort, double power)
    {
        if (!_isRecording) return;

        _actions.Add(new RecordedAction
        {
            TimeMs = _stopwatch.ElapsedMilliseconds,
            Type = ActionType.Motor,
            Port = motorPort,
            Value1 = power
        });
    }

    public void RecordPneumaticAction(int solenoidPort, bool state)
    {
        if (!_isRecording) return;

        _actions.Add(new RecordedAction
        {
            TimeMs = _stopwatch.ElapsedMilliseconds,
            Type = ActionType.Pneumatic,
            Port = solenoidPort,
            Value1 = state ? 1 : 0
        });
    }

    public void RecordPose(double x, double y, double theta)
    {
        if (!_isRecording) return;

        _actions.Add(new RecordedAction
        {
            TimeMs = _stopwatch.ElapsedMilliseconds,
            Type = ActionType.Pose,
            Value1 = x,
            Value2 = y,
            Value3 = theta
        });
    }

    /// <summary>
    /// Export to PROS C++ code.
    /// </summary>
    public string ExportToProsCpp()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("// Recorded autonomous routine");
        sb.AppendLine("// Generated by ControlWorkbench");
        sb.AppendLine();
        sb.AppendLine("void recorded_autonomous() {");
        
        long lastTime = 0;
        foreach (var action in _actions)
        {
            long delay = action.TimeMs - lastTime;
            if (delay > 5)
            {
                sb.AppendLine($"    pros::delay({delay});");
            }

            switch (action.Type)
            {
                case ActionType.Drive:
                    sb.AppendLine($"    left_motors.move({action.Value1:F0});");
                    sb.AppendLine($"    right_motors.move({action.Value2:F0});");
                    break;
                case ActionType.Motor:
                    sb.AppendLine($"    motor_{action.Port}.move({action.Value1:F0});");
                    break;
                case ActionType.Pneumatic:
                    sb.AppendLine($"    solenoid_{action.Port}.set_value({(action.Value1 > 0 ? "true" : "false")});");
                    break;
            }

            lastTime = action.TimeMs;
        }

        sb.AppendLine("    ");
        sb.AppendLine("    // Stop all motors");
        sb.AppendLine("    left_motors.move(0);");
        sb.AppendLine("    right_motors.move(0);");
        sb.AppendLine("}");

        return sb.ToString();
    }

    /// <summary>
    /// Get playback iterator for real-time playback.
    /// </summary>
    public IEnumerable<RecordedAction> GetPlaybackActions()
    {
        return _actions.OrderBy(a => a.TimeMs);
    }
}

public class RecordedAction
{
    public long TimeMs { get; set; }
    public ActionType Type { get; set; }
    public int Port { get; set; }
    public double Value1 { get; set; }
    public double Value2 { get; set; }
    public double Value3 { get; set; }
}

public enum ActionType
{
    Drive,
    Motor,
    Pneumatic,
    Pose,
    Wait
}
