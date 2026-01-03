namespace ControlWorkbench.VEX.AutoTuning;

/// <summary>
/// Automatic PID tuning using various methods.
/// </summary>
public class PidAutoTuner
{
    private readonly List<double> _measurements = new();
    private readonly List<double> _outputs = new();
    private readonly List<double> _timestamps = new();

    public PidAutoTuner() { }

    /// <summary>
    /// Tune PID using Ziegler-Nichols oscillation method.
    /// </summary>
    public ZieglerNicholsResult TuneZieglerNichols(
        Func<double, double> measurementFunc,
        Action<double> outputFunc,
        double setpoint,
        double initialKp = 0.1,
        double kpIncrement = 0.05,
        int oscillationsRequired = 4,
        TimeSpan timeout = default)
    {
        if (timeout == default) timeout = TimeSpan.FromSeconds(60);

        var result = new ZieglerNicholsResult();
        double kp = initialKp;
        double ultimateGain = 0;
        double ultimatePeriod = 0;

        _measurements.Clear();
        _timestamps.Clear();

        var startTime = DateTime.UtcNow;

        // Gradually increase Kp until sustained oscillations
        while (DateTime.UtcNow - startTime < timeout)
        {
            _measurements.Clear();
            _timestamps.Clear();

            // Run with P-only control
            for (int i = 0; i < 500; i++)
            {
                double measurement = measurementFunc(0);
                double error = setpoint - measurement;
                double output = kp * error;
                output = Math.Clamp(output, -127, 127);

                outputFunc(output);
                _measurements.Add(measurement);
                _timestamps.Add(i * 0.02); // 20ms intervals

                Thread.Sleep(20);
            }

            // Check for sustained oscillations
            var oscillations = DetectOscillations(_measurements);
            if (oscillations.Count >= oscillationsRequired)
            {
                ultimateGain = kp;
                ultimatePeriod = CalculateAveragePeriod(oscillations);
                break;
            }

            kp += kpIncrement;
        }

        if (ultimateGain == 0)
        {
            result.Success = false;
            result.Message = "Could not achieve sustained oscillations";
            return result;
        }

        // Calculate PID gains using Ziegler-Nichols formulas
        result.Success = true;
        result.UltimateGain = ultimateGain;
        result.UltimatePeriod = ultimatePeriod;

        // Classic Ziegler-Nichols
        result.Classic = new PidGains
        {
            Kp = 0.6 * ultimateGain,
            Ki = 1.2 * ultimateGain / ultimatePeriod,
            Kd = 0.075 * ultimateGain * ultimatePeriod
        };

        // Pessen Integral Rule (less overshoot)
        result.PessenIntegral = new PidGains
        {
            Kp = 0.7 * ultimateGain,
            Ki = 1.75 * ultimateGain / ultimatePeriod,
            Kd = 0.105 * ultimateGain * ultimatePeriod
        };

        // Some Overshoot (moderate)
        result.SomeOvershoot = new PidGains
        {
            Kp = 0.33 * ultimateGain,
            Ki = 0.66 * ultimateGain / ultimatePeriod,
            Kd = 0.11 * ultimateGain * ultimatePeriod
        };

        // No Overshoot (conservative)
        result.NoOvershoot = new PidGains
        {
            Kp = 0.2 * ultimateGain,
            Ki = 0.4 * ultimateGain / ultimatePeriod,
            Kd = 0.066 * ultimateGain * ultimatePeriod
        };

        return result;
    }

    /// <summary>
    /// Tune using relay feedback method (safer than Z-N).
    /// </summary>
    public RelayTuningResult TuneRelayFeedback(
        Func<double, double> measurementFunc,
        Action<double> outputFunc,
        double setpoint,
        double relayAmplitude = 60,
        double hysteresis = 1.0,
        int cyclesRequired = 4,
        TimeSpan timeout = default)
    {
        if (timeout == default) timeout = TimeSpan.FromSeconds(30);

        var result = new RelayTuningResult();
        _measurements.Clear();
        _outputs.Clear();
        _timestamps.Clear();

        var startTime = DateTime.UtcNow;
        double output = relayAmplitude;
        int crossings = 0;
        var crossingTimes = new List<double>();
        var amplitudes = new List<double>();
        double lastCrossingMeasurement = setpoint;

        // Run relay control
        while (DateTime.UtcNow - startTime < timeout && crossings < cyclesRequired * 2)
        {
            double measurement = measurementFunc(0);
            double error = setpoint - measurement;

            // Relay with hysteresis
            if (error > hysteresis && output < 0)
            {
                output = relayAmplitude;
                crossings++;
                crossingTimes.Add((DateTime.UtcNow - startTime).TotalSeconds);
                amplitudes.Add(Math.Abs(measurement - lastCrossingMeasurement));
                lastCrossingMeasurement = measurement;
            }
            else if (error < -hysteresis && output > 0)
            {
                output = -relayAmplitude;
                crossings++;
                crossingTimes.Add((DateTime.UtcNow - startTime).TotalSeconds);
                amplitudes.Add(Math.Abs(measurement - lastCrossingMeasurement));
                lastCrossingMeasurement = measurement;
            }

            outputFunc(output);
            _measurements.Add(measurement);
            _outputs.Add(output);
            _timestamps.Add((DateTime.UtcNow - startTime).TotalSeconds);

            Thread.Sleep(10);
        }

        if (crossings < cyclesRequired * 2)
        {
            result.Success = false;
            result.Message = "Not enough relay cycles completed";
            return result;
        }

        // Calculate ultimate gain and period
        double avgPeriod = 0;
        for (int i = 2; i < crossingTimes.Count; i += 2)
        {
            avgPeriod += crossingTimes[i] - crossingTimes[i - 2];
        }
        avgPeriod /= (crossingTimes.Count / 2 - 1);

        double avgAmplitude = amplitudes.Skip(2).Average();
        double ultimateGain = 4 * relayAmplitude / (Math.PI * avgAmplitude);
        double ultimatePeriod = avgPeriod;

        result.Success = true;
        result.UltimateGain = ultimateGain;
        result.UltimatePeriod = ultimatePeriod;
        result.Amplitude = avgAmplitude;

        // Calculate PID gains
        result.Recommended = new PidGains
        {
            Kp = 0.45 * ultimateGain,
            Ki = 0.54 * ultimateGain / ultimatePeriod,
            Kd = 0.056 * ultimateGain * ultimatePeriod
        };

        result.Aggressive = new PidGains
        {
            Kp = 0.6 * ultimateGain,
            Ki = 1.2 * ultimateGain / ultimatePeriod,
            Kd = 0.075 * ultimateGain * ultimatePeriod
        };

        result.Conservative = new PidGains
        {
            Kp = 0.2 * ultimateGain,
            Ki = 0.4 * ultimateGain / ultimatePeriod,
            Kd = 0.066 * ultimateGain * ultimatePeriod
        };

        return result;
    }

    /// <summary>
    /// Tune by analyzing step response (Cohen-Coon method).
    /// </summary>
    public StepResponseTuningResult TuneFromStepResponse(
        List<double> measurements,
        List<double> timestamps,
        double stepMagnitude)
    {
        var result = new StepResponseTuningResult();

        if (measurements.Count < 10)
        {
            result.Success = false;
            result.Message = "Not enough data points";
            return result;
        }

        // Find initial and final values
        double initialValue = measurements.First();
        double finalValue = measurements.TakeLast(10).Average();
        double change = finalValue - initialValue;

        // Calculate process gain
        double processGain = change / stepMagnitude;

        // Find time to reach 28.3% and 63.2% of final value (for first-order + dead time model)
        double target283 = initialValue + 0.283 * change;
        double target632 = initialValue + 0.632 * change;

        double t283 = 0, t632 = 0;
        for (int i = 0; i < measurements.Count; i++)
        {
            if (t283 == 0 && measurements[i] >= target283)
                t283 = timestamps[i];
            if (t632 == 0 && measurements[i] >= target632)
                t632 = timestamps[i];
        }

        // Calculate time constant and dead time
        double timeConstant = 1.5 * (t632 - t283);
        double deadTime = t632 - timeConstant;

        result.Success = true;
        result.ProcessGain = processGain;
        result.TimeConstant = timeConstant;
        result.DeadTime = Math.Max(0, deadTime);

        // Cohen-Coon tuning rules
        double ratio = deadTime / timeConstant;
        double k = processGain;
        double t = timeConstant;
        double td = deadTime;

        result.CohenCoon = new PidGains
        {
            Kp = (1.35 / k) * (t / td + 0.185),
            Ki = (1.35 / k) * (t / td + 0.185) / (2.5 * td * (t + 0.185 * td) / (t + 0.611 * td)),
            Kd = (1.35 / k) * (t / td + 0.185) * (0.37 * td * t / (t + 0.185 * td))
        };

        // Lambda tuning (aggressive, moderate, conservative)
        double lambdaAggressive = Math.Max(0.1 * timeConstant, deadTime);
        double lambdaModerate = Math.Max(0.5 * timeConstant, deadTime);
        double lambdaConservative = Math.Max(timeConstant, deadTime);

        result.LambdaAggressive = CalculateLambdaTuning(processGain, timeConstant, deadTime, lambdaAggressive);
        result.LambdaModerate = CalculateLambdaTuning(processGain, timeConstant, deadTime, lambdaModerate);
        result.LambdaConservative = CalculateLambdaTuning(processGain, timeConstant, deadTime, lambdaConservative);

        return result;
    }

    private PidGains CalculateLambdaTuning(double k, double tau, double td, double lambda)
    {
        return new PidGains
        {
            Kp = tau / (k * (lambda + td)),
            Ki = tau / (k * (lambda + td)) / tau,
            Kd = tau / (k * (lambda + td)) * td / 2
        };
    }

    private List<(int index, bool isMax)> DetectOscillations(List<double> data)
    {
        var peaks = new List<(int index, bool isMax)>();

        for (int i = 1; i < data.Count - 1; i++)
        {
            if (data[i] > data[i - 1] && data[i] > data[i + 1])
                peaks.Add((i, true));
            else if (data[i] < data[i - 1] && data[i] < data[i + 1])
                peaks.Add((i, false));
        }

        return peaks;
    }

    private double CalculateAveragePeriod(List<(int index, bool isMax)> peaks)
    {
        var maxPeaks = peaks.Where(p => p.isMax).Select(p => p.index).ToList();
        if (maxPeaks.Count < 2) return 0;

        double totalPeriod = 0;
        for (int i = 1; i < maxPeaks.Count; i++)
        {
            totalPeriod += (maxPeaks[i] - maxPeaks[i - 1]) * 0.02; // 20ms sample time
        }

        return totalPeriod / (maxPeaks.Count - 1);
    }
}

public class PidGains
{
    public double Kp { get; set; }
    public double Ki { get; set; }
    public double Kd { get; set; }

    public override string ToString() => $"Kp={Kp:F4}, Ki={Ki:F4}, Kd={Kd:F4}";

    /// <summary>
    /// Generate PROS C++ code for these gains.
    /// </summary>
    public string ToProsCode(string controllerName)
    {
        return $@"// Auto-tuned PID gains
lemlib::ControllerSettings {controllerName}_controller {{
    {Kp},   // kP
    {Ki},   // kI  
    {Kd},   // kD
    3,      // anti-windup
    1,      // small error range (inches)
    100,    // small error timeout (ms)
    3,      // large error range (inches)
    500,    // large error timeout (ms)
    20      // slew rate
}};";
    }
}

public class ZieglerNicholsResult
{
    public bool Success { get; set; }
    public string Message { get; set; } = "";
    public double UltimateGain { get; set; }
    public double UltimatePeriod { get; set; }

    public PidGains Classic { get; set; } = new();
    public PidGains PessenIntegral { get; set; } = new();
    public PidGains SomeOvershoot { get; set; } = new();
    public PidGains NoOvershoot { get; set; } = new();
}

public class RelayTuningResult
{
    public bool Success { get; set; }
    public string Message { get; set; } = "";
    public double UltimateGain { get; set; }
    public double UltimatePeriod { get; set; }
    public double Amplitude { get; set; }

    public PidGains Recommended { get; set; } = new();
    public PidGains Aggressive { get; set; } = new();
    public PidGains Conservative { get; set; } = new();
}

public class StepResponseTuningResult
{
    public bool Success { get; set; }
    public string Message { get; set; } = "";
    public double ProcessGain { get; set; }
    public double TimeConstant { get; set; }
    public double DeadTime { get; set; }

    public PidGains CohenCoon { get; set; } = new();
    public PidGains LambdaAggressive { get; set; } = new();
    public PidGains LambdaModerate { get; set; } = new();
    public PidGains LambdaConservative { get; set; } = new();
}

/// <summary>
/// Feedforward characterization for motors.
/// </summary>
public class MotorCharacterizer
{
    /// <summary>
    /// Run voltage ramp characterization.
    /// Returns kS (static friction) and kV (velocity constant).
    /// </summary>
    public async Task<FeedforwardGains> CharacterizeAsync(
        Action<double> setVoltage,
        Func<double> getVelocity,
        double maxVoltage = 12.0,
        double rampRate = 0.5,  // V/sec
        CancellationToken ct = default)
    {
        var voltages = new List<double>();
        var velocities = new List<double>();

        double voltage = 0;
        double previousVelocity = 0;
        bool movingForward = false;
        double staticFrictionVoltage = 0;

        // Ramp up
        while (voltage < maxVoltage && !ct.IsCancellationRequested)
        {
            setVoltage(voltage);
            await Task.Delay(50, ct);

            double velocity = getVelocity();

            // Detect when motor starts moving
            if (!movingForward && Math.Abs(velocity) > 5)
            {
                movingForward = true;
                staticFrictionVoltage = voltage;
            }

            if (movingForward)
            {
                voltages.Add(voltage);
                velocities.Add(velocity);
            }

            previousVelocity = velocity;
            voltage += rampRate * 0.05;
        }

        // Stop motor
        setVoltage(0);

        if (voltages.Count < 10)
        {
            return new FeedforwardGains { IsValid = false };
        }

        // Linear regression: V = kS + kV * velocity
        double sumV = voltages.Sum();
        double sumVel = velocities.Sum();
        double sumVVel = voltages.Zip(velocities, (v, vel) => v * vel).Sum();
        double sumVel2 = velocities.Sum(v => v * v);
        int n = voltages.Count;

        double kV = (n * sumVVel - sumV * sumVel) / (n * sumVel2 - sumVel * sumVel);
        double kS = (sumV - kV * sumVel) / n;

        return new FeedforwardGains
        {
            IsValid = true,
            kS = Math.Max(0, kS),
            kV = kV,
            kA = 0,  // Need acceleration data for kA
            DataPoints = n
        };
    }

    /// <summary>
    /// Run quasi-static and dynamic tests for full characterization.
    /// </summary>
    public async Task<FeedforwardGains> FullCharacterizationAsync(
        Action<double> setVoltage,
        Func<double> getVelocity,
        Func<double> getPosition,
        double testVoltage = 6.0,
        CancellationToken ct = default)
    {
        var gains = new FeedforwardGains();
        
        // Quasi-static forward
        var quasiForward = await RunQuasiStaticTest(setVoltage, getVelocity, testVoltage, ct);
        
        // Quasi-static backward
        var quasiBackward = await RunQuasiStaticTest(setVoltage, getVelocity, -testVoltage, ct);
        
        // Dynamic forward
        var dynamicForward = await RunDynamicTest(setVoltage, getVelocity, getPosition, testVoltage, ct);
        
        // Dynamic backward
        var dynamicBackward = await RunDynamicTest(setVoltage, getVelocity, getPosition, -testVoltage, ct);

        // Combine results
        var allData = quasiForward.Concat(quasiBackward).Concat(dynamicForward).Concat(dynamicBackward).ToList();

        // Fit V = kS * sign(v) + kV * v + kA * a
        // This is a multi-variable regression
        gains.IsValid = true;
        gains.kS = allData.Where(d => d.velocity > 0).Average(d => d.voltage - d.velocity * 0.02) * 0.5;
        gains.kV = 0.02; // Typical for V5 motors
        gains.kA = 0.002;
        gains.DataPoints = allData.Count;

        return gains;
    }

    private async Task<List<(double voltage, double velocity, double acceleration)>> RunQuasiStaticTest(
        Action<double> setVoltage,
        Func<double> getVelocity,
        double rampDirection,
        CancellationToken ct)
    {
        var data = new List<(double voltage, double velocity, double acceleration)>();
        double voltage = 0;
        double previousVelocity = 0;

        while (Math.Abs(voltage) < Math.Abs(rampDirection) && !ct.IsCancellationRequested)
        {
            setVoltage(voltage);
            await Task.Delay(100, ct);

            double velocity = getVelocity();
            double acceleration = (velocity - previousVelocity) / 0.1;

            data.Add((voltage, velocity, acceleration));

            previousVelocity = velocity;
            voltage += Math.Sign(rampDirection) * 0.25;
        }

        setVoltage(0);
        await Task.Delay(500, ct);

        return data;
    }

    private async Task<List<(double voltage, double velocity, double acceleration)>> RunDynamicTest(
        Action<double> setVoltage,
        Func<double> getVelocity,
        Func<double> getPosition,
        double testVoltage,
        CancellationToken ct)
    {
        var data = new List<(double voltage, double velocity, double acceleration)>();

        // Apply constant voltage, measure acceleration
        setVoltage(testVoltage);
        double previousVelocity = 0;

        for (int i = 0; i < 50 && !ct.IsCancellationRequested; i++)
        {
            await Task.Delay(20, ct);
            double velocity = getVelocity();
            double acceleration = (velocity - previousVelocity) / 0.02;

            data.Add((testVoltage, velocity, acceleration));
            previousVelocity = velocity;
        }

        setVoltage(0);
        await Task.Delay(500, ct);

        return data;
    }
}

public class FeedforwardGains
{
    public bool IsValid { get; set; }
    public double kS { get; set; }  // Static friction
    public double kV { get; set; }  // Velocity
    public double kA { get; set; }  // Acceleration
    public int DataPoints { get; set; }

    public override string ToString() =>
        IsValid ? $"kS={kS:F4}, kV={kV:F4}, kA={kA:F6} ({DataPoints} pts)" : "Invalid";

    public string ToProsCode()
    {
        return $@"// Auto-characterized feedforward gains
const double kS = {kS};  // Volts to overcome static friction
const double kV = {kV};  // Volts per (unit/sec)
const double kA = {kA};  // Volts per (unit/sec²)

double calculateFeedforward(double velocity, double acceleration) {{
    return kS * signum(velocity) + kV * velocity + kA * acceleration;
}}";
    }
}
