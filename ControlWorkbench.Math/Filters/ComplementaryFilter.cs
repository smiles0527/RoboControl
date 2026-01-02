namespace ControlWorkbench.Math.Filters;

/// <summary>
/// First-order complementary filter for fusing high-frequency and low-frequency signals.
/// Commonly used for attitude estimation from gyro (high freq) and accelerometer/magnetometer (low freq).
/// </summary>
public class ComplementaryFilter
{
    private double _alpha;
    private double _state;
    private bool _initialized;

    /// <summary>
    /// Gets or sets the filter coefficient alpha (0..1).
    /// Higher alpha gives more weight to the high-frequency (gyro) signal.
    /// </summary>
    public double Alpha
    {
        get => _alpha;
        set
        {
            if (value < 0 || value > 1)
                throw new ArgumentOutOfRangeException(nameof(value), "Alpha must be between 0 and 1.");
            _alpha = value;
        }
    }

    /// <summary>
    /// Gets the current filtered state.
    /// </summary>
    public double State => _state;

    /// <summary>
    /// Gets whether the filter has been initialized.
    /// </summary>
    public bool IsInitialized => _initialized;

    /// <summary>
    /// Creates a complementary filter with the specified alpha.
    /// </summary>
    /// <param name="alpha">Filter coefficient (0..1). Higher = more gyro weight.</param>
    public ComplementaryFilter(double alpha = 0.98)
    {
        Alpha = alpha;
        _state = 0;
        _initialized = false;
    }

    /// <summary>
    /// Creates a complementary filter from cutoff frequency and sample time.
    /// alpha = tau / (tau + dt) where tau = 1 / (2*pi*fc)
    /// </summary>
    /// <param name="cutoffFrequencyHz">Low-pass cutoff frequency for the reference signal.</param>
    /// <param name="sampleTimeSeconds">Sample period (dt).</param>
    public static ComplementaryFilter FromCutoffFrequency(double cutoffFrequencyHz, double sampleTimeSeconds)
    {
        if (cutoffFrequencyHz <= 0)
            throw new ArgumentOutOfRangeException(nameof(cutoffFrequencyHz), "Cutoff frequency must be positive.");
        if (sampleTimeSeconds <= 0)
            throw new ArgumentOutOfRangeException(nameof(sampleTimeSeconds), "Sample time must be positive.");

        double tau = 1.0 / (2.0 * System.Math.PI * cutoffFrequencyHz);
        double alpha = tau / (tau + sampleTimeSeconds);
        return new ComplementaryFilter(alpha);
    }

    /// <summary>
    /// Computes alpha from cutoff frequency and sample time.
    /// </summary>
    public static double ComputeAlpha(double cutoffFrequencyHz, double sampleTimeSeconds)
    {
        double tau = 1.0 / (2.0 * System.Math.PI * cutoffFrequencyHz);
        return tau / (tau + sampleTimeSeconds);
    }

    /// <summary>
    /// Computes the cutoff frequency from alpha and sample time.
    /// </summary>
    public static double ComputeCutoffFrequency(double alpha, double sampleTimeSeconds)
    {
        if (alpha <= 0 || alpha >= 1)
            throw new ArgumentOutOfRangeException(nameof(alpha), "Alpha must be between 0 and 1 (exclusive).");
        
        // tau = alpha * dt / (1 - alpha)
        // fc = 1 / (2 * pi * tau)
        double tau = alpha * sampleTimeSeconds / (1.0 - alpha);
        return 1.0 / (2.0 * System.Math.PI * tau);
    }

    /// <summary>
    /// Resets the filter state.
    /// </summary>
    public void Reset()
    {
        _state = 0;
        _initialized = false;
    }

    /// <summary>
    /// Initializes the filter with a specific state.
    /// </summary>
    public void Initialize(double initialState)
    {
        _state = initialState;
        _initialized = true;
    }

    /// <summary>
    /// Updates the filter with gyro rate and reference angle.
    /// output = alpha * (prev + gyro*dt) + (1-alpha) * reference
    /// </summary>
    /// <param name="gyroRate">Gyroscope angular rate (rad/s).</param>
    /// <param name="referenceAngle">Reference angle from accelerometer/magnetometer (rad).</param>
    /// <param name="dt">Time step (seconds).</param>
    /// <returns>Filtered angle estimate.</returns>
    public double Update(double gyroRate, double referenceAngle, double dt)
    {
        if (!_initialized)
        {
            _state = referenceAngle;
            _initialized = true;
            return _state;
        }

        // Complementary filter equation
        _state = _alpha * (_state + gyroRate * dt) + (1.0 - _alpha) * referenceAngle;
        return _state;
    }

    /// <summary>
    /// Updates the filter with only gyro rate (high-pass portion only).
    /// Useful when reference measurement is not available.
    /// </summary>
    public double UpdateGyroOnly(double gyroRate, double dt)
    {
        if (!_initialized)
            throw new InvalidOperationException("Filter not initialized. Call Initialize() or Update() first.");
        
        _state += gyroRate * dt;
        return _state;
    }
}

/// <summary>
/// Design parameters for a complementary filter.
/// </summary>
public class ComplementaryFilterDesign
{
    /// <summary>
    /// Sample time in seconds.
    /// </summary>
    public double SampleTime { get; set; }

    /// <summary>
    /// Cutoff frequency in Hz.
    /// </summary>
    public double CutoffFrequencyHz { get; set; }

    /// <summary>
    /// Computed alpha value.
    /// </summary>
    public double Alpha => ComplementaryFilter.ComputeAlpha(CutoffFrequencyHz, SampleTime);

    /// <summary>
    /// Time constant tau in seconds.
    /// </summary>
    public double TimeConstant => 1.0 / (2.0 * System.Math.PI * CutoffFrequencyHz);

    /// <summary>
    /// Nyquist frequency for the given sample rate.
    /// </summary>
    public double NyquistFrequency => 0.5 / SampleTime;

    /// <summary>
    /// Creates a default design.
    /// </summary>
    public ComplementaryFilterDesign()
    {
        SampleTime = 0.01; // 100 Hz
        CutoffFrequencyHz = 0.1; // 0.1 Hz cutoff
    }

    /// <summary>
    /// Validates the design parameters.
    /// </summary>
    public (bool IsValid, string Message) Validate()
    {
        if (SampleTime <= 0)
            return (false, "Sample time must be positive.");
        if (CutoffFrequencyHz <= 0)
            return (false, "Cutoff frequency must be positive.");
        if (CutoffFrequencyHz >= NyquistFrequency)
            return (false, $"Cutoff frequency ({CutoffFrequencyHz:F2} Hz) must be less than Nyquist frequency ({NyquistFrequency:F2} Hz).");
        return (true, "Design is valid.");
    }

    /// <summary>
    /// Creates a filter from this design.
    /// </summary>
    public ComplementaryFilter CreateFilter()
    {
        return ComplementaryFilter.FromCutoffFrequency(CutoffFrequencyHz, SampleTime);
    }
}
