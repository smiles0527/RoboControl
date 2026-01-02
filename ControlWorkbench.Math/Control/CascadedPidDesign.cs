namespace ControlWorkbench.Math.Control;

/// <summary>
/// Cascaded PID controller helper.
/// Provides guidance for tuning cascaded loops where outer loops should be slower than inner loops.
/// </summary>
public class CascadedPidDesign
{
    /// <summary>
    /// Inner loop bandwidth (rad/s).
    /// </summary>
    public double InnerLoopBandwidth { get; set; } = 10.0;

    /// <summary>
    /// Outer loop bandwidth ratio (outer/inner). Should be < 1.
    /// Default is 0.2 (outer loop 5x slower than inner).
    /// </summary>
    public double OuterLoopBandwidthRatio { get; set; } = 0.2;

    /// <summary>
    /// Gets the outer loop bandwidth (rad/s).
    /// </summary>
    public double OuterLoopBandwidth => InnerLoopBandwidth * OuterLoopBandwidthRatio;

    /// <summary>
    /// Sample time (seconds).
    /// </summary>
    public double SampleTime { get; set; } = 0.01;

    /// <summary>
    /// Inner loop type.
    /// </summary>
    public CascadeLoopType InnerLoopType { get; set; } = CascadeLoopType.Rate;

    /// <summary>
    /// Outer loop type.
    /// </summary>
    public CascadeLoopType OuterLoopType { get; set; } = CascadeLoopType.Angle;

    /// <summary>
    /// Validates the cascade design.
    /// </summary>
    public (bool IsValid, string Message) Validate()
    {
        if (InnerLoopBandwidth <= 0)
            return (false, "Inner loop bandwidth must be positive.");
        if (OuterLoopBandwidthRatio <= 0 || OuterLoopBandwidthRatio >= 1)
            return (false, "Outer/inner bandwidth ratio must be between 0 and 1.");
        if (SampleTime <= 0)
            return (false, "Sample time must be positive.");

        double nyquist = System.Math.PI / SampleTime;
        if (InnerLoopBandwidth > 0.5 * nyquist)
            return (false, $"Inner loop bandwidth ({InnerLoopBandwidth:F2} rad/s) exceeds recommended limit (0.5 * Nyquist = {0.5 * nyquist:F2} rad/s).");

        double minRatio = 3.0;
        if (1.0 / OuterLoopBandwidthRatio < minRatio)
            return (false, $"Inner/outer bandwidth ratio ({1.0 / OuterLoopBandwidthRatio:F1}) should be at least {minRatio} for good cascade separation.");

        return (true, "Cascade design is valid.");
    }

    /// <summary>
    /// Computes recommended PID gains for a first-order plant.
    /// For G(s) = K / (tau*s + 1), designs PI to achieve target bandwidth.
    /// </summary>
    public static (double Kp, double Ki) DesignPiForFirstOrder(double plantGain, double plantTau, double targetBandwidth)
    {
        // For PI control of first-order plant, using IMC tuning:
        // Kp = tau / (K * lambda) where lambda = 1/bandwidth
        // Ki = Kp / tau
        double lambda = 1.0 / targetBandwidth;
        double kp = plantTau / (plantGain * lambda);
        double ki = kp / plantTau;
        return (kp, ki);
    }

    /// <summary>
    /// Computes recommended PID gains for a second-order plant.
    /// For G(s) = K / (s² + 2*zeta*wn*s + wn²), places closed-loop poles at desired locations.
    /// </summary>
    public static (double Kp, double Ki, double Kd) DesignPidForSecondOrder(
        double plantGain, 
        double plantWn, 
        double plantZeta,
        double targetWn,
        double targetZeta)
    {
        // This is a simplified pole placement for second-order systems
        // More rigorous methods would use state-space design
        
        double kp = (targetWn * targetWn - plantWn * plantWn) / plantGain;
        double ki = 0.5 * targetWn * kp; // Approximate integral gain
        double kd = (2 * targetZeta * targetWn - 2 * plantZeta * plantWn) / plantGain;

        return (kp, ki, kd);
    }

    /// <summary>
    /// Designs the inner loop PID controller.
    /// </summary>
    public PidController DesignInnerLoop(double plantGain = 1.0, double plantTau = 0.1)
    {
        var (kp, ki) = DesignPiForFirstOrder(plantGain, plantTau, InnerLoopBandwidth);
        
        var pid = new PidController(kp, ki, 0);
        pid.DerivativeFilterTf = 0.1 * SampleTime;
        
        // Set reasonable output limits based on loop type
        switch (InnerLoopType)
        {
            case CascadeLoopType.Rate:
                pid.SetOutputLimits(1.0); // Normalized
                break;
            case CascadeLoopType.Angle:
                pid.SetOutputLimits(5.0); // rad/s command
                break;
            case CascadeLoopType.Velocity:
                pid.SetOutputLimits(10.0); // m/s² acceleration
                break;
            case CascadeLoopType.Position:
                pid.SetOutputLimits(5.0); // m/s velocity command
                break;
        }

        return pid;
    }

    /// <summary>
    /// Designs the outer loop PID controller.
    /// </summary>
    public PidController DesignOuterLoop(double plantGain = 1.0, double plantTau = 0.5)
    {
        var (kp, ki) = DesignPiForFirstOrder(plantGain, plantTau, OuterLoopBandwidth);
        
        var pid = new PidController(kp, ki, 0);
        pid.DerivativeFilterTf = 0.1 * SampleTime * 5; // Slower filter for outer loop
        
        // Set reasonable output limits based on loop type
        switch (OuterLoopType)
        {
            case CascadeLoopType.Angle:
                pid.SetOutputLimits(5.0); // rad/s rate command
                break;
            case CascadeLoopType.Velocity:
                pid.SetOutputLimits(10.0); // m/s² or angle command
                break;
            case CascadeLoopType.Position:
                pid.SetOutputLimits(5.0); // m/s velocity command
                break;
            default:
                pid.SetOutputLimits(1.0);
                break;
        }

        return pid;
    }

    /// <summary>
    /// Gets recommended bandwidth ranges based on cascade configuration.
    /// </summary>
    public string GetRecommendations()
    {
        var validation = Validate();
        var sb = new System.Text.StringBuilder();

        sb.AppendLine("=== Cascaded PID Design Recommendations ===");
        sb.AppendLine();
        sb.AppendLine($"Inner Loop ({InnerLoopType}):");
        sb.AppendLine($"  Bandwidth: {InnerLoopBandwidth:F2} rad/s ({InnerLoopBandwidth / (2 * System.Math.PI):F2} Hz)");
        sb.AppendLine($"  Response time: ~{3.0 / InnerLoopBandwidth:F3} s (3/?)");
        sb.AppendLine();
        sb.AppendLine($"Outer Loop ({OuterLoopType}):");
        sb.AppendLine($"  Bandwidth: {OuterLoopBandwidth:F2} rad/s ({OuterLoopBandwidth / (2 * System.Math.PI):F2} Hz)");
        sb.AppendLine($"  Response time: ~{3.0 / OuterLoopBandwidth:F3} s (3/?)");
        sb.AppendLine($"  Bandwidth ratio: {OuterLoopBandwidthRatio:F2} (inner is {1.0 / OuterLoopBandwidthRatio:F1}x faster)");
        sb.AppendLine();
        sb.AppendLine($"Sample Rate: {1.0 / SampleTime:F1} Hz");
        sb.AppendLine($"Nyquist: {0.5 / SampleTime:F1} Hz ({System.Math.PI / SampleTime:F1} rad/s)");
        sb.AppendLine();

        if (!validation.IsValid)
        {
            sb.AppendLine($"? Warning: {validation.Message}");
        }
        else
        {
            sb.AppendLine("? Design parameters are valid.");
        }

        return sb.ToString();
    }
}

/// <summary>
/// Types of cascade loops.
/// </summary>
public enum CascadeLoopType
{
    Rate,       // Angular rate (innermost typically)
    Angle,      // Attitude angle
    Velocity,   // Linear velocity
    Position    // Position (outermost typically)
}
