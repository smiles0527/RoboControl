namespace ControlWorkbench.Math.Metrics;

/// <summary>
/// Step response metrics.
/// </summary>
public record StepResponseMetrics
{
    /// <summary>
    /// Rise time (10% to 90% of final value) in seconds.
    /// </summary>
    public double RiseTime { get; init; }

    /// <summary>
    /// Overshoot percentage.
    /// </summary>
    public double OvershootPercent { get; init; }

    /// <summary>
    /// Settling time (time to stay within 2% of final value) in seconds.
    /// </summary>
    public double SettlingTime { get; init; }

    /// <summary>
    /// Steady-state error (difference from target at the end).
    /// </summary>
    public double SteadyStateError { get; init; }

    /// <summary>
    /// Peak value achieved.
    /// </summary>
    public double PeakValue { get; init; }

    /// <summary>
    /// Time at which peak occurred.
    /// </summary>
    public double PeakTime { get; init; }

    /// <summary>
    /// Final value at end of data.
    /// </summary>
    public double FinalValue { get; init; }

    /// <summary>
    /// Whether the response is stable (settled).
    /// </summary>
    public bool IsStable { get; init; }
}

/// <summary>
/// Computes step response metrics from time-series data.
/// </summary>
public static class StepResponseAnalyzer
{
    /// <summary>
    /// Analyzes step response metrics from time and signal data.
    /// </summary>
    /// <param name="times">Time values in seconds.</param>
    /// <param name="values">Signal values.</param>
    /// <param name="stepTarget">The target value (step command).</param>
    /// <param name="initialValue">Initial value before step (default 0).</param>
    /// <param name="settlingBandPercent">Settling band percentage (default 2%).</param>
    public static StepResponseMetrics Analyze(
        double[] times,
        double[] values,
        double stepTarget,
        double initialValue = 0,
        double settlingBandPercent = 2.0)
    {
        if (times.Length != values.Length)
            throw new ArgumentException("Times and values must have the same length.");
        if (times.Length < 2)
            throw new ArgumentException("Need at least 2 data points.");

        double stepSize = stepTarget - initialValue;
        if (System.Math.Abs(stepSize) < 1e-10)
        {
            return new StepResponseMetrics
            {
                RiseTime = 0,
                OvershootPercent = 0,
                SettlingTime = 0,
                SteadyStateError = 0,
                PeakValue = initialValue,
                PeakTime = 0,
                FinalValue = initialValue,
                IsStable = true
            };
        }

        // Normalize values relative to step
        double[] normalized = new double[values.Length];
        for (int i = 0; i < values.Length; i++)
        {
            normalized[i] = (values[i] - initialValue) / stepSize;
        }

        // Find rise time (10% to 90%)
        double riseTime = ComputeRiseTime(times, normalized, 0.1, 0.9);

        // Find peak value and time
        double peakNorm = normalized[0];
        int peakIndex = 0;
        for (int i = 1; i < normalized.Length; i++)
        {
            if (stepSize > 0 && normalized[i] > peakNorm ||
                stepSize < 0 && normalized[i] < peakNorm)
            {
                peakNorm = normalized[i];
                peakIndex = i;
            }
        }
        double peakValue = values[peakIndex];
        double peakTime = times[peakIndex];

        // Compute overshoot
        double overshoot = 0;
        if (stepSize > 0 && peakNorm > 1.0)
            overshoot = (peakNorm - 1.0) * 100.0;
        else if (stepSize < 0 && peakNorm < 1.0)
            overshoot = (1.0 - peakNorm) * 100.0;

        // Find settling time (within band around final value)
        double finalNorm = normalized[^1];
        double finalValue = values[^1];
        double band = settlingBandPercent / 100.0;
        double settlingTime = ComputeSettlingTime(times, normalized, finalNorm, band);

        // Steady-state error
        double steadyStateError = stepTarget - finalValue;

        // Check stability (is it within settling band at the end?)
        bool isStable = System.Math.Abs(normalized[^1] - 1.0) <= band;

        return new StepResponseMetrics
        {
            RiseTime = riseTime,
            OvershootPercent = overshoot,
            SettlingTime = settlingTime,
            SteadyStateError = steadyStateError,
            PeakValue = peakValue,
            PeakTime = peakTime,
            FinalValue = finalValue,
            IsStable = isStable
        };
    }

    private static double ComputeRiseTime(double[] times, double[] normalized, double low, double high)
    {
        double tLow = double.NaN;
        double tHigh = double.NaN;

        for (int i = 1; i < normalized.Length; i++)
        {
            if (double.IsNaN(tLow) && normalized[i] >= low && normalized[i - 1] < low)
            {
                // Linear interpolation
                double t = Interpolate(normalized[i - 1], normalized[i], times[i - 1], times[i], low);
                tLow = t;
            }
            if (double.IsNaN(tHigh) && normalized[i] >= high && normalized[i - 1] < high)
            {
                double t = Interpolate(normalized[i - 1], normalized[i], times[i - 1], times[i], high);
                tHigh = t;
                break;
            }
        }

        if (double.IsNaN(tLow) || double.IsNaN(tHigh))
            return double.NaN;

        return tHigh - tLow;
    }

    private static double ComputeSettlingTime(double[] times, double[] normalized, double finalValue, double band)
    {
        // Find the last time the signal was outside the settling band
        double settlingTime = times[^1];

        for (int i = normalized.Length - 1; i >= 0; i--)
        {
            if (System.Math.Abs(normalized[i] - finalValue) > band)
            {
                if (i < normalized.Length - 1)
                    settlingTime = times[i + 1];
                else
                    settlingTime = times[i];
                break;
            }
            settlingTime = times[i];
        }

        return settlingTime;
    }

    private static double Interpolate(double y1, double y2, double t1, double t2, double y)
    {
        if (System.Math.Abs(y2 - y1) < 1e-10)
            return t1;
        return t1 + (y - y1) * (t2 - t1) / (y2 - y1);
    }

    /// <summary>
    /// Formats metrics as a readable string.
    /// </summary>
    public static string FormatMetrics(StepResponseMetrics metrics)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("=== Step Response Metrics ===");
        sb.AppendLine($"Rise Time (10-90%): {FormatTime(metrics.RiseTime)}");
        sb.AppendLine($"Overshoot: {metrics.OvershootPercent:F1}%");
        sb.AppendLine($"Peak Value: {metrics.PeakValue:F4} at t={FormatTime(metrics.PeakTime)}");
        sb.AppendLine($"Settling Time (2%): {FormatTime(metrics.SettlingTime)}");
        sb.AppendLine($"Steady-State Error: {metrics.SteadyStateError:F4}");
        sb.AppendLine($"Final Value: {metrics.FinalValue:F4}");
        sb.AppendLine($"Stable: {(metrics.IsStable ? "Yes" : "No")}");
        return sb.ToString();
    }

    private static string FormatTime(double seconds)
    {
        if (double.IsNaN(seconds))
            return "N/A";
        if (seconds < 0.001)
            return $"{seconds * 1e6:F1} µs";
        if (seconds < 1)
            return $"{seconds * 1000:F1} ms";
        return $"{seconds:F3} s";
    }
}
