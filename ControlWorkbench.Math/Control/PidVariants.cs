namespace ControlWorkbench.Math.Control;

/// <summary>
/// Proportional-Integral-Derivative (PID) Controller with anti-windup.
/// Industry standard controller with extensive features.
/// </summary>
public class ExtendedPidController
{
    private double _kp;
    private double _ki;
    private double _kd;
    private double _integral;
    private double _previousError;
    private double _previousMeasurement;
    private double _outputMin;
    private double _outputMax;
    private double _integralMin;
    private double _integralMax;
    private bool _useDerivativeOnMeasurement;
    private double _derivativeFilterCoeff;
    private double _filteredDerivative;

    public ExtendedPidController(
        double kp, double ki, double kd,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue,
        bool useDerivativeOnMeasurement = true,
        double derivativeFilterCoeff = 0.1)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _outputMin = outputMin;
        _outputMax = outputMax;
        _integralMin = outputMin;
        _integralMax = outputMax;
        _useDerivativeOnMeasurement = useDerivativeOnMeasurement;
        _derivativeFilterCoeff = derivativeFilterCoeff;
        Reset();
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // Proportional term
        double pTerm = _kp * error;
        
        // Integral term with anti-windup
        _integral += error * dt;
        _integral = System.Math.Clamp(_integral, _integralMin / _ki, _integralMax / _ki);
        double iTerm = _ki * _integral;
        
        // Derivative term
        double derivative;
        if (_useDerivativeOnMeasurement)
        {
            // Derivative on measurement to avoid derivative kick
            derivative = -(measurement - _previousMeasurement) / dt;
            _previousMeasurement = measurement;
        }
        else
        {
            derivative = (error - _previousError) / dt;
            _previousError = error;
        }
        
        // Low-pass filter on derivative
        _filteredDerivative = _derivativeFilterCoeff * derivative + 
            (1 - _derivativeFilterCoeff) * _filteredDerivative;
        double dTerm = _kd * _filteredDerivative;
        
        // Total output with saturation
        double output = pTerm + iTerm + dTerm;
        output = System.Math.Clamp(output, _outputMin, _outputMax);
        
        // Anti-windup: back-calculate integral if saturated
        if (_ki > 0)
        {
            double unsaturated = pTerm + iTerm + dTerm;
            if (unsaturated != output)
            {
                _integral -= (unsaturated - output) / _ki * 0.5;
            }
        }
        
        return output;
    }

    public void SetGains(double kp, double ki, double kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    public void SetOutputLimits(double min, double max)
    {
        _outputMin = min;
        _outputMax = max;
        _integralMin = min;
        _integralMax = max;
    }

    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _previousMeasurement = 0;
        _filteredDerivative = 0;
    }

    public (double kp, double ki, double kd) Gains => (_kp, _ki, _kd);
}

/// <summary>
/// PID with automatic setpoint weighting.
/// Reduces overshoot while maintaining disturbance rejection.
/// </summary>
public class PidWithSetpointWeighting
{
    private readonly PidController _pid;
    private readonly double _b; // Setpoint weight for proportional
    private readonly double _c; // Setpoint weight for derivative

    public PidWithSetpointWeighting(
        double kp, double ki, double kd,
        double b = 1.0, double c = 0.0,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _b = b;
        _c = c;
        _pid = new PidController(kp, ki, kd)
        {
            OutputMin = outputMin,
            OutputMax = outputMax
        };
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        // Weighted setpoint for proportional and derivative
        double pSetpoint = _b * setpoint + (1 - _b) * measurement;
        return _pid.Compute(pSetpoint, measurement, dt);
    }

    public void Reset() => _pid.Reset();
}

/// <summary>
/// PI-D Controller (Proportional-Integral with Derivative on measurement only).
/// Classic industrial form.
/// </summary>
public class PiDController
{
    private double _kp;
    private double _ki;
    private double _kd;
    private double _integral;
    private double _previousMeasurement;
    private double _outputMin;
    private double _outputMax;

    public PiDController(
        double kp, double ki, double kd,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _outputMin = outputMin;
        _outputMax = outputMax;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // P on error
        double pTerm = _kp * error;
        
        // I on error
        _integral += error * dt;
        double iTerm = _ki * _integral;
        
        // D on measurement only
        double derivative = -(measurement - _previousMeasurement) / dt;
        _previousMeasurement = measurement;
        double dTerm = _kd * derivative;
        
        double output = pTerm + iTerm + dTerm;
        return System.Math.Clamp(output, _outputMin, _outputMax);
    }

    public void Reset()
    {
        _integral = 0;
        _previousMeasurement = 0;
    }
}

/// <summary>
/// I-PD Controller (Integral with Proportional-Derivative on measurement).
/// Smoothest setpoint response, no overshoot.
/// </summary>
public class IPdController
{
    private double _kp;
    private double _ki;
    private double _kd;
    private double _integral;
    private double _previousMeasurement;
    private double _outputMin;
    private double _outputMax;

    public IPdController(
        double kp, double ki, double kd,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _outputMin = outputMin;
        _outputMax = outputMax;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // I on error
        _integral += error * dt;
        double iTerm = _ki * _integral;
        
        // P on measurement only
        double pTerm = -_kp * measurement;
        
        // D on measurement only
        double derivative = -(measurement - _previousMeasurement) / dt;
        _previousMeasurement = measurement;
        double dTerm = _kd * derivative;
        
        double output = pTerm + iTerm + dTerm;
        return System.Math.Clamp(output, _outputMin, _outputMax);
    }

    public void Reset()
    {
        _integral = 0;
        _previousMeasurement = 0;
    }
}

/// <summary>
/// Velocity-form PID (Incremental PID).
/// Outputs change in control signal, not absolute value.
/// Better for bumpless transfer and windup prevention.
/// </summary>
public class VelocityPidController
{
    private double _kp;
    private double _ki;
    private double _kd;
    private double _previousError;
    private double _previousError2;
    private double _previousOutput;
    private double _outputMin;
    private double _outputMax;

    public VelocityPidController(
        double kp, double ki, double kd,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _outputMin = outputMin;
        _outputMax = outputMax;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // Velocity form: ?u = Kp(e[k] - e[k-1]) + Ki*e[k]*dt + Kd*(e[k] - 2*e[k-1] + e[k-2])/dt
        double deltaP = _kp * (error - _previousError);
        double deltaI = _ki * error * dt;
        double deltaD = _kd * (error - 2 * _previousError + _previousError2) / dt;
        
        double deltaOutput = deltaP + deltaI + deltaD;
        double output = _previousOutput + deltaOutput;
        output = System.Math.Clamp(output, _outputMin, _outputMax);
        
        // Store history
        _previousError2 = _previousError;
        _previousError = error;
        _previousOutput = output;
        
        return output;
    }

    public void Reset()
    {
        _previousError = 0;
        _previousError2 = 0;
        _previousOutput = 0;
    }
}

/// <summary>
/// Parallel PID (Standard/ISA form).
/// u = Kp * (e + 1/Ti * ?e dt + Td * de/dt)
/// </summary>
public class ParallelPidController
{
    private double _kp;
    private double _ti; // Integral time
    private double _td; // Derivative time
    private double _integral;
    private double _previousError;
    private double _outputMin;
    private double _outputMax;

    public ParallelPidController(
        double kp, double ti, double td,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _kp = kp;
        _ti = ti;
        _td = td;
        _outputMin = outputMin;
        _outputMax = outputMax;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // Integral
        if (_ti > 0)
            _integral += error * dt / _ti;
        
        // Derivative
        double derivative = _td * (error - _previousError) / dt;
        _previousError = error;
        
        double output = _kp * (error + _integral + derivative);
        return System.Math.Clamp(output, _outputMin, _outputMax);
    }

    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
    }

    // Convert to parallel gains
    public (double kp, double ki, double kd) ToParallelGains() =>
        (_kp, _ti > 0 ? _kp / _ti : 0, _kp * _td);
}

/// <summary>
/// PID with feedforward control.
/// Improves tracking by anticipating setpoint changes.
/// </summary>
public class PidWithFeedforward
{
    private readonly PidController _pid;
    private readonly double _kff; // Feedforward gain
    private double _previousSetpoint;

    public PidWithFeedforward(
        double kp, double ki, double kd, double kff,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _pid = new PidController(kp, ki, kd)
        {
            OutputMin = outputMin,
            OutputMax = outputMax
        };
        _kff = kff;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        // Feedforward on setpoint velocity
        double setpointVelocity = (setpoint - _previousSetpoint) / dt;
        _previousSetpoint = setpoint;
        
        double feedforward = _kff * setpointVelocity;
        double feedback = _pid.Compute(setpoint, measurement, dt);
        
        return feedback + feedforward;
    }

    public void Reset()
    {
        _pid.Reset();
        _previousSetpoint = 0;
    }
}

/// <summary>
/// Cascaded PID Controller.
/// Outer loop sets setpoint for inner loop.
/// </summary>
public class CascadedPidController
{
    private readonly PidController _outerLoop;
    private readonly PidController _innerLoop;
    private readonly double _innerLoopRatio;

    public CascadedPidController(
        double outerKp, double outerKi, double outerKd,
        double innerKp, double innerKi, double innerKd,
        double innerOutputMin, double innerOutputMax,
        int innerLoopRatio = 1)
    {
        _outerLoop = new PidController(outerKp, outerKi, outerKd)
        {
            OutputMin = innerOutputMin,
            OutputMax = innerOutputMax
        };
        _innerLoop = new PidController(innerKp, innerKi, innerKd);
        _innerLoopRatio = innerLoopRatio;
    }

    public double Compute(
        double outerSetpoint, double outerMeasurement,
        double innerMeasurement, double dt)
    {
        // Outer loop runs at slower rate
        double innerSetpoint = _outerLoop.Compute(outerSetpoint, outerMeasurement, dt * _innerLoopRatio);
        
        // Inner loop
        return _innerLoop.Compute(innerSetpoint, innerMeasurement, dt);
    }

    public void Reset()
    {
        _outerLoop.Reset();
        _innerLoop.Reset();
    }
}

/// <summary>
/// PID Auto-tuner using relay feedback method (Åström-Hägglund).
/// </summary>
public class PidAutoTuner
{
    private readonly double _relayAmplitude;
    private readonly double _hysteresis;
    private double _output;
    private double _previousError;
    private readonly List<double> _peakTimes;
    private readonly List<double> _peakValues;
    private double _time;
    private bool _tuningComplete;
    private double _ultimateGain;
    private double _ultimatePeriod;

    public PidAutoTuner(double relayAmplitude = 1.0, double hysteresis = 0.05)
    {
        _relayAmplitude = relayAmplitude;
        _hysteresis = hysteresis;
        _peakTimes = new List<double>();
        _peakValues = new List<double>();
        _time = 0;
        _tuningComplete = false;
    }

    /// <summary>
    /// Run the relay experiment. Returns relay output.
    /// </summary>
    public double RunExperiment(double setpoint, double measurement, double dt)
    {
        _time += dt;
        double error = setpoint - measurement;
        
        // Relay with hysteresis
        if (error > _hysteresis)
            _output = _relayAmplitude;
        else if (error < -_hysteresis)
            _output = -_relayAmplitude;
        // else maintain previous output
        
        // Detect peaks
        if (_previousError > 0 && error <= 0) // Positive to negative crossing
        {
            _peakTimes.Add(_time);
            _peakValues.Add(measurement);
        }
        _previousError = error;
        
        // Check if we have enough oscillations
        if (_peakTimes.Count >= 5 && !_tuningComplete)
        {
            CalculateUltimateParameters();
        }
        
        return _output;
    }

    private void CalculateUltimateParameters()
    {
        // Average period from peaks
        double totalPeriod = 0;
        for (int i = 1; i < _peakTimes.Count; i++)
        {
            totalPeriod += (_peakTimes[i] - _peakTimes[i - 1]) * 2; // Full period
        }
        _ultimatePeriod = totalPeriod / (_peakTimes.Count - 1);
        
        // Amplitude from relay describing function
        double amplitude = (_peakValues.Max() - _peakValues.Min()) / 2;
        _ultimateGain = 4 * _relayAmplitude / (System.Math.PI * amplitude);
        
        _tuningComplete = true;
    }

    /// <summary>
    /// Get tuned PID gains using Ziegler-Nichols rules.
    /// </summary>
    public (double kp, double ki, double kd) GetZieglerNicholsGains()
    {
        if (!_tuningComplete)
            throw new InvalidOperationException("Tuning not complete");
        
        double kp = 0.6 * _ultimateGain;
        double ki = 2 * kp / _ultimatePeriod;
        double kd = kp * _ultimatePeriod / 8;
        
        return (kp, ki, kd);
    }

    /// <summary>
    /// Get tuned PID gains using Tyreus-Luyben rules (less aggressive).
    /// </summary>
    public (double kp, double ki, double kd) GetTyreusLuybenGains()
    {
        if (!_tuningComplete)
            throw new InvalidOperationException("Tuning not complete");
        
        double kp = _ultimateGain / 3.2;
        double ki = kp / (2.2 * _ultimatePeriod);
        double kd = kp * _ultimatePeriod / 6.3;
        
        return (kp, ki, kd);
    }

    /// <summary>
    /// Get tuned PID gains using Cohen-Coon rules.
    /// </summary>
    public (double kp, double ki, double kd) GetCohenCoonGains()
    {
        if (!_tuningComplete)
            throw new InvalidOperationException("Tuning not complete");
        
        // Approximate using ultimate parameters
        double kp = 0.45 * _ultimateGain;
        double ki = kp / (_ultimatePeriod * 0.83);
        double kd = kp * _ultimatePeriod * 0.125;
        
        return (kp, ki, kd);
    }

    public bool IsTuningComplete => _tuningComplete;
    public double UltimateGain => _ultimateGain;
    public double UltimatePeriod => _ultimatePeriod;
}

/// <summary>
/// Model Reference Adaptive PID Controller (MRAC-PID).
/// Adjusts gains to match reference model behavior.
/// </summary>
public class MracPidController
{
    private double _kp;
    private double _ki;
    private double _kd;
    private double _integral;
    private double _previousError;
    private double _previousMeasurement;
    
    // Adaptation parameters
    private readonly double _gammaP;
    private readonly double _gammaI;
    private readonly double _gammaD;
    private readonly double _referenceModelTau;
    private double _referenceOutput;
    private double _outputMin;
    private double _outputMax;

    public MracPidController(
        double initialKp, double initialKi, double initialKd,
        double gammaP = 0.01, double gammaI = 0.001, double gammaD = 0.001,
        double referenceModelTau = 1.0,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _kp = initialKp;
        _ki = initialKi;
        _kd = initialKd;
        _gammaP = gammaP;
        _gammaI = gammaI;
        _gammaD = gammaD;
        _referenceModelTau = referenceModelTau;
        _outputMin = outputMin;
        _outputMax = outputMax;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // Reference model: first-order response
        double refError = setpoint - _referenceOutput;
        _referenceOutput += refError * dt / _referenceModelTau;
        
        // Tracking error
        double trackingError = measurement - _referenceOutput;
        
        // Adapt gains using MIT rule
        _kp -= _gammaP * trackingError * error * dt;
        _ki -= _gammaI * trackingError * _integral * dt;
        _kd -= _gammaD * trackingError * (error - _previousError) / dt * dt;
        
        // Ensure positive gains
        _kp = System.Math.Max(0.01, _kp);
        _ki = System.Math.Max(0, _ki);
        _kd = System.Math.Max(0, _kd);
        
        // Standard PID computation
        _integral += error * dt;
        double derivative = -(measurement - _previousMeasurement) / dt;
        _previousError = error;
        _previousMeasurement = measurement;
        
        double output = _kp * error + _ki * _integral + _kd * derivative;
        return System.Math.Clamp(output, _outputMin, _outputMax);
    }

    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _previousMeasurement = 0;
        _referenceOutput = 0;
    }

    public (double kp, double ki, double kd) CurrentGains => (_kp, _ki, _kd);
}

/// <summary>
/// Fractional-Order PID Controller (PI^? D^?).
/// Uses fractional calculus for improved control.
/// </summary>
public class FractionalPidController
{
    private readonly double _kp;
    private readonly double _ki;
    private readonly double _kd;
    private readonly double _lambda; // Fractional order of integral (0 < ? < 2)
    private readonly double _mu; // Fractional order of derivative (0 < ? < 2)
    private readonly int _memoryLength;
    private readonly double[] _errorHistory;
    private readonly double[] _glCoeffsIntegral;
    private readonly double[] _glCoeffsDerivative;
    private int _historyIndex;
    private double _outputMin;
    private double _outputMax;

    public FractionalPidController(
        double kp, double ki, double kd,
        double lambda = 1.0, double mu = 1.0,
        int memoryLength = 100,
        double outputMin = double.MinValue,
        double outputMax = double.MaxValue)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _lambda = lambda;
        _mu = mu;
        _memoryLength = memoryLength;
        _outputMin = outputMin;
        _outputMax = outputMax;
        
        _errorHistory = new double[memoryLength];
        _glCoeffsIntegral = ComputeGrunwaldLetnikovCoeffs(lambda, memoryLength);
        _glCoeffsDerivative = ComputeGrunwaldLetnikovCoeffs(-mu, memoryLength);
        _historyIndex = 0;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;
        
        // Store error in circular buffer
        _errorHistory[_historyIndex] = error;
        
        // Compute fractional integral using Grünwald-Letnikov
        double fractionalIntegral = 0;
        for (int j = 0; j < _memoryLength; j++)
        {
            int idx = (_historyIndex - j + _memoryLength) % _memoryLength;
            fractionalIntegral += _glCoeffsIntegral[j] * _errorHistory[idx];
        }
        fractionalIntegral *= System.Math.Pow(dt, _lambda);
        
        // Compute fractional derivative using Grünwald-Letnikov
        double fractionalDerivative = 0;
        for (int j = 0; j < _memoryLength; j++)
        {
            int idx = (_historyIndex - j + _memoryLength) % _memoryLength;
            fractionalDerivative += _glCoeffsDerivative[j] * _errorHistory[idx];
        }
        fractionalDerivative *= System.Math.Pow(dt, -_mu);
        
        // Advance history index
        _historyIndex = (_historyIndex + 1) % _memoryLength;
        
        // PID output
        double output = _kp * error + _ki * fractionalIntegral + _kd * fractionalDerivative;
        return System.Math.Clamp(output, _outputMin, _outputMax);
    }

    private static double[] ComputeGrunwaldLetnikovCoeffs(double alpha, int length)
    {
        var coeffs = new double[length];
        coeffs[0] = 1;
        for (int j = 1; j < length; j++)
        {
            coeffs[j] = coeffs[j - 1] * (1 - (alpha + 1) / j);
        }
        return coeffs;
    }

    public void Reset()
    {
        Array.Clear(_errorHistory, 0, _memoryLength);
        _historyIndex = 0;
    }
}

/// <summary>
/// Dead-time compensator (Smith Predictor) with PID.
/// For systems with significant pure delay.
/// </summary>
public class SmithPredictorPid
{
    private readonly PidController _pid;
    private readonly double _processGain;
    private readonly double _processTimeConstant;
    private readonly double _deadTime;
    private readonly double _sampleTime;
    private readonly Queue<double> _delayBuffer;
    private double _modelOutput;
    private double _delayedModelOutput;

    public SmithPredictorPid(
        double kp, double ki, double kd,
        double processGain, double processTimeConstant, double deadTime,
        double sampleTime)
    {
        _pid = new PidController(kp, ki, kd);
        _processGain = processGain;
        _processTimeConstant = processTimeConstant;
        _deadTime = deadTime;
        _sampleTime = sampleTime;
        
        int delaySteps = (int)(deadTime / sampleTime);
        _delayBuffer = new Queue<double>(delaySteps + 1);
        for (int i = 0; i <= delaySteps; i++)
        {
            _delayBuffer.Enqueue(0);
        }
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        // Compute predicted output (model without delay)
        double alpha = dt / (_processTimeConstant + dt);
        double u = _pid.Compute(setpoint, measurement + _modelOutput - _delayedModelOutput, dt);
        
        _modelOutput = (1 - alpha) * _modelOutput + alpha * _processGain * u;
        
        // Delay the model output
        _delayBuffer.Enqueue(_modelOutput);
        _delayedModelOutput = _delayBuffer.Dequeue();
        
        return u;
    }

    public void Reset()
    {
        _pid.Reset();
        _modelOutput = 0;
        _delayedModelOutput = 0;
        _delayBuffer.Clear();
        int delaySteps = (int)(_deadTime / _sampleTime);
        for (int i = 0; i <= delaySteps; i++)
        {
            _delayBuffer.Enqueue(0);
        }
    }
}
