namespace ControlWorkbench.Math.Control;

/// <summary>
/// Discrete PID controller with anti-windup and derivative filtering.
/// </summary>
public class PidController
{
    private double _integral;
    private double _previousError;
    private double _previousDerivativeFiltered;
    private bool _initialized;

    /// <summary>
    /// Proportional gain.
    /// </summary>
    public double Kp { get; set; }

    /// <summary>
    /// Integral gain.
    /// </summary>
    public double Ki { get; set; }

    /// <summary>
    /// Derivative gain.
    /// </summary>
    public double Kd { get; set; }

    /// <summary>
    /// Output minimum limit.
    /// </summary>
    public double OutputMin { get; set; } = double.NegativeInfinity;

    /// <summary>
    /// Output maximum limit.
    /// </summary>
    public double OutputMax { get; set; } = double.PositiveInfinity;

    /// <summary>
    /// Integrator minimum limit (for anti-windup).
    /// </summary>
    public double IntegratorMin { get; set; } = double.NegativeInfinity;

    /// <summary>
    /// Integrator maximum limit (for anti-windup).
    /// </summary>
    public double IntegratorMax { get; set; } = double.PositiveInfinity;

    /// <summary>
    /// Derivative filter time constant (seconds).
    /// Set to 0 to disable filtering. Typical value: 0.1 * Td where Td = Kd/Kp.
    /// </summary>
    public double DerivativeFilterTf { get; set; } = 0;

    /// <summary>
    /// Back-calculation gain for anti-windup.
    /// Set to 0 to use clamping only. Typical value: 1/Ti = Kp/Ki.
    /// </summary>
    public double AntiWindupKb { get; set; } = 0;

    /// <summary>
    /// Gets the current integrator value.
    /// </summary>
    public double IntegratorValue => _integral;

    /// <summary>
    /// Gets the last computed derivative (filtered).
    /// </summary>
    public double LastDerivative => _previousDerivativeFiltered;

    /// <summary>
    /// Creates a PID controller with the specified gains.
    /// </summary>
    public PidController(double kp = 1.0, double ki = 0.0, double kd = 0.0)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        Reset();
    }

    /// <summary>
    /// Resets the controller state.
    /// </summary>
    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _previousDerivativeFiltered = 0;
        _initialized = false;
    }

    /// <summary>
    /// Computes the control output.
    /// </summary>
    /// <param name="setpoint">Desired value.</param>
    /// <param name="measurement">Measured value.</param>
    /// <param name="dt">Time step (seconds).</param>
    /// <returns>Control output.</returns>
    public double Compute(double setpoint, double measurement, double dt)
    {
        if (dt <= 0)
            throw new ArgumentOutOfRangeException(nameof(dt), "Time step must be positive.");

        double error = setpoint - measurement;

        // Proportional term
        double pTerm = Kp * error;

        // Integral term with trapezoidal (Tustin) integration
        if (!_initialized)
        {
            _previousError = error;
            _initialized = true;
        }

        double integralIncrement = Ki * (error + _previousError) * 0.5 * dt;
        double integralCandidate = _integral + integralIncrement;

        // Apply integrator limits (clamping anti-windup)
        integralCandidate = System.Math.Clamp(integralCandidate, IntegratorMin, IntegratorMax);
        double iTerm = integralCandidate;

        // Derivative term with optional filtering
        double derivative = (error - _previousError) / dt;
        double derivativeFiltered;

        if (DerivativeFilterTf > 0)
        {
            // First-order low-pass filter on derivative
            // alpha = dt / (Tf + dt)
            double alpha = dt / (DerivativeFilterTf + dt);
            derivativeFiltered = alpha * derivative + (1 - alpha) * _previousDerivativeFiltered;
        }
        else
        {
            derivativeFiltered = derivative;
        }

        double dTerm = Kd * derivativeFiltered;

        // Compute output before saturation
        double outputUnsat = pTerm + iTerm + dTerm;

        // Apply output limits
        double output = System.Math.Clamp(outputUnsat, OutputMin, OutputMax);

        // Back-calculation anti-windup
        if (AntiWindupKb > 0 && outputUnsat != output)
        {
            double saturationError = output - outputUnsat;
            integralCandidate += AntiWindupKb * saturationError * dt;
            integralCandidate = System.Math.Clamp(integralCandidate, IntegratorMin, IntegratorMax);
        }

        // Update state
        _integral = integralCandidate;
        _previousError = error;
        _previousDerivativeFiltered = derivativeFiltered;

        return output;
    }

    /// <summary>
    /// Computes the control output using derivative on measurement (avoid derivative kick).
    /// </summary>
    public double ComputeWithDerivativeOnMeasurement(double setpoint, double measurement, double previousMeasurement, double dt)
    {
        if (dt <= 0)
            throw new ArgumentOutOfRangeException(nameof(dt), "Time step must be positive.");

        double error = setpoint - measurement;

        // Proportional term
        double pTerm = Kp * error;

        // Integral term
        if (!_initialized)
        {
            _previousError = error;
            _initialized = true;
        }

        double integralIncrement = Ki * (error + _previousError) * 0.5 * dt;
        double integralCandidate = _integral + integralIncrement;
        integralCandidate = System.Math.Clamp(integralCandidate, IntegratorMin, IntegratorMax);
        double iTerm = integralCandidate;

        // Derivative on measurement (negative to oppose error)
        double derivative = -(measurement - previousMeasurement) / dt;
        double derivativeFiltered;

        if (DerivativeFilterTf > 0)
        {
            double alpha = dt / (DerivativeFilterTf + dt);
            derivativeFiltered = alpha * derivative + (1 - alpha) * _previousDerivativeFiltered;
        }
        else
        {
            derivativeFiltered = derivative;
        }

        double dTerm = Kd * derivativeFiltered;

        double outputUnsat = pTerm + iTerm + dTerm;
        double output = System.Math.Clamp(outputUnsat, OutputMin, OutputMax);

        if (AntiWindupKb > 0 && outputUnsat != output)
        {
            double saturationError = output - outputUnsat;
            integralCandidate += AntiWindupKb * saturationError * dt;
            integralCandidate = System.Math.Clamp(integralCandidate, IntegratorMin, IntegratorMax);
        }

        _integral = integralCandidate;
        _previousError = error;
        _previousDerivativeFiltered = derivativeFiltered;

        return output;
    }

    /// <summary>
    /// Sets symmetric output limits.
    /// </summary>
    public void SetOutputLimits(double limit)
    {
        OutputMin = -limit;
        OutputMax = limit;
    }

    /// <summary>
    /// Sets symmetric integrator limits.
    /// </summary>
    public void SetIntegratorLimits(double limit)
    {
        IntegratorMin = -limit;
        IntegratorMax = limit;
    }
}
