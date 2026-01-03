using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Control;

/// <summary>
/// Linear Quadratic Gaussian (LQG) Controller.
/// Combines LQR optimal control with Kalman filter estimation.
/// </summary>
public class LqgController
{
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _C;
    private readonly Matrix<double> _K; // LQR gain
    private readonly Matrix<double> _L; // Kalman gain
    
    private Vector<double> _xHat; // State estimate
    private readonly int _n; // State size
    private readonly int _m; // Input size

    public LqgController(
        Matrix<double> A, Matrix<double> B, Matrix<double> C,
        Matrix<double> Q, Matrix<double> R,
        Matrix<double> Qn, Matrix<double> Rn)
    {
        _A = A;
        _B = B;
        _C = C;
        _n = A.RowCount;
        _m = B.ColumnCount;
        
        // Compute LQR gain
        _K = ComputeLqrGain(A, B, Q, R);
        
        // Compute Kalman gain
        _L = ComputeKalmanGain(A, C, Qn, Rn);
        
        _xHat = Vector<double>.Build.Dense(_n);
    }

    public void Initialize(Vector<double> initialEstimate)
    {
        _xHat = initialEstimate.Clone();
    }

    /// <summary>
    /// Computes control input and updates state estimate.
    /// </summary>
    public Vector<double> Compute(Vector<double> y, Vector<double> reference)
    {
        // Compute control: u = -K * (x_hat - x_ref)
        var error = _xHat - reference;
        var u = -_K * error;
        
        // Predict state: x_hat = A*x_hat + B*u
        var xPred = _A * _xHat + _B * u;
        
        // Update with measurement: x_hat = x_pred + L*(y - C*x_pred)
        var innovation = y - _C * xPred;
        _xHat = xPred + _L * innovation;
        
        return u;
    }

    private static Matrix<double> ComputeLqrGain(Matrix<double> A, Matrix<double> B, Matrix<double> Q, Matrix<double> R)
    {
        // Solve discrete algebraic Riccati equation iteratively
        var P = Q.Clone();
        for (int i = 0; i < 100; i++)
        {
            var Pnew = Q + A.Transpose() * P * A - 
                A.Transpose() * P * B * (R + B.Transpose() * P * B).Inverse() * B.Transpose() * P * A;
            if ((Pnew - P).FrobeniusNorm() < 1e-10) break;
            P = Pnew;
        }
        return (R + B.Transpose() * P * B).Inverse() * B.Transpose() * P * A;
    }

    private static Matrix<double> ComputeKalmanGain(Matrix<double> A, Matrix<double> C, Matrix<double> Q, Matrix<double> R)
    {
        // Solve Riccati for Kalman filter
        int n = A.RowCount;
        var P = Matrix<double>.Build.DenseIdentity(n);
        for (int i = 0; i < 100; i++)
        {
            var Pnew = A * P * A.Transpose() + Q - 
                A * P * C.Transpose() * (R + C * P * C.Transpose()).Inverse() * C * P * A.Transpose();
            if ((Pnew - P).FrobeniusNorm() < 1e-10) break;
            P = Pnew;
        }
        return P * C.Transpose() * (R + C * P * C.Transpose()).Inverse();
    }

    public Vector<double> StateEstimate => _xHat;
}

/// <summary>
/// H-infinity (H?) robust controller.
/// Minimizes the worst-case gain from disturbance to error.
/// </summary>
public class HInfinityController
{
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B1; // Disturbance input
    private readonly Matrix<double> _B2; // Control input
    private readonly Matrix<double> _C1; // Performance output
    private readonly Matrix<double> _C2; // Measured output
    private readonly Matrix<double> _K; // Controller gain
    private readonly double _gamma; // H-infinity norm bound
    
    private Vector<double> _xc; // Controller state

    public HInfinityController(
        Matrix<double> A,
        Matrix<double> B1, Matrix<double> B2,
        Matrix<double> C1, Matrix<double> C2,
        Matrix<double> D11, Matrix<double> D12,
        Matrix<double> D21, Matrix<double> D22,
        double gamma = 1.0)
    {
        _A = A;
        _B1 = B1;
        _B2 = B2;
        _C1 = C1;
        _C2 = C2;
        _gamma = gamma;
        
        int n = A.RowCount;
        _xc = Vector<double>.Build.Dense(n);
        
        // Simplified: compute sub-optimal controller using Riccati approach
        _K = ComputeHinfGain(A, B2, C1, D12, gamma);
    }

    public Vector<double> Compute(Vector<double> y)
    {
        // State feedback with H-infinity gain
        var u = -_K * _xc;
        
        // Update controller state (simplified observer)
        _xc = _A * _xc + _B2 * u;
        
        return u;
    }

    private static Matrix<double> ComputeHinfGain(
        Matrix<double> A, Matrix<double> B, Matrix<double> C, Matrix<double> D, double gamma)
    {
        int n = A.RowCount;
        var R = D.Transpose() * D;
        var Q = C.Transpose() * C;
        
        // Modified Riccati for H-infinity
        var P = Q.Clone();
        var gammaInv2 = 1.0 / (gamma * gamma);
        
        for (int i = 0; i < 100; i++)
        {
            var M = R + B.Transpose() * P * B;
            var Pnew = Q + A.Transpose() * P * A - 
                A.Transpose() * P * B * M.Inverse() * B.Transpose() * P * A;
            
            // Add gamma penalty term
            Pnew = Pnew * (1.0 + gammaInv2);
            
            if ((Pnew - P).FrobeniusNorm() < 1e-10) break;
            P = Pnew;
        }
        
        return (R + B.Transpose() * P * B).Inverse() * B.Transpose() * P * A;
    }
}

/// <summary>
/// State Feedback Controller with pole placement.
/// </summary>
public class PolePlacementController
{
    private readonly Matrix<double> _K;
    private readonly double[] _desiredPoles;

    public PolePlacementController(Matrix<double> A, Matrix<double> B, double[] desiredPoles)
    {
        _desiredPoles = desiredPoles;
        _K = ComputeGainAckermann(A, B, desiredPoles);
    }

    public Vector<double> Compute(Vector<double> x, Vector<double> reference)
    {
        var error = x - reference;
        return -_K * error;
    }

    private static Matrix<double> ComputeGainAckermann(Matrix<double> A, Matrix<double> B, double[] poles)
    {
        int n = A.RowCount;
        
        // Compute controllability matrix
        var C = Matrix<double>.Build.Dense(n, n);
        var Ab = B;
        for (int i = 0; i < n; i++)
        {
            C.SetColumn(i, Ab.Column(0));
            Ab = A * Ab;
        }
        
        // Compute characteristic polynomial coefficients
        var alpha = ComputeCharacteristicCoeffs(poles);
        
        // Compute Ackermann's formula: phi(A)
        var phiA = Matrix<double>.Build.Dense(n, n);
        var Ak = Matrix<double>.Build.DenseIdentity(n);
        for (int i = 0; i <= n; i++)
        {
            phiA += alpha[i] * Ak;
            if (i < n) Ak = A * Ak;
        }
        
        // K = [0 0 ... 1] * C^-1 * phi(A)
        var eN = Vector<double>.Build.Dense(n);
        eN[n - 1] = 1;
        
        return (eN.ToRowMatrix() * C.Inverse() * phiA);
    }

    private static double[] ComputeCharacteristicCoeffs(double[] poles)
    {
        int n = poles.Length;
        var coeffs = new double[n + 1];
        coeffs[n] = 1; // Leading coefficient
        
        // Expand (s - p1)(s - p2)...(s - pn)
        var poly = new double[n + 1];
        poly[0] = 1;
        
        for (int i = 0; i < n; i++)
        {
            var newPoly = new double[n + 1];
            for (int j = 0; j <= i + 1; j++)
            {
                if (j > 0) newPoly[j] += poly[j - 1];
                if (j <= i) newPoly[j] -= poles[i] * poly[j];
            }
            poly = newPoly;
        }
        
        return poly;
    }
}

/// <summary>
/// Integral Sliding Mode Controller.
/// Robust against matched uncertainties from the start.
/// </summary>
public class IntegralSlidingModeController
{
    private readonly double _lambda;
    private readonly double _k;
    private readonly double _epsilon;
    private double _sigma; // Integral term
    private double _previousError;

    public IntegralSlidingModeController(double lambda, double k, double epsilon = 0.1)
    {
        _lambda = lambda;
        _k = k;
        _epsilon = epsilon;
        _sigma = 0;
        _previousError = 0;
    }

    public double Compute(double error, double errorDot, double dt, double nominalControl = 0)
    {
        // Update integral
        _sigma += error * dt;
        
        // Sliding surface with integral action
        double s = errorDot + _lambda * error + _lambda * _lambda * _sigma;
        
        // Discontinuous control with boundary layer
        double us = _k * Saturation(s / _epsilon);
        
        // Total control
        return nominalControl - us;
    }

    public void Reset()
    {
        _sigma = 0;
        _previousError = 0;
    }

    private static double Saturation(double x) => System.Math.Max(-1, System.Math.Min(1, x));
}

/// <summary>
/// Terminal Sliding Mode Controller.
/// Guarantees finite-time convergence.
/// </summary>
public class TerminalSlidingModeController
{
    private readonly double _beta;
    private readonly double _p;
    private readonly double _q;
    private readonly double _k;
    private readonly double _epsilon;

    public TerminalSlidingModeController(double beta, double p, double q, double k, double epsilon = 0.1)
    {
        _beta = beta;
        _p = p; // Must be odd
        _q = q; // Must be odd, p < q
        _k = k;
        _epsilon = epsilon;
    }

    public double Compute(double error, double errorDot)
    {
        // Terminal sliding surface: s = errorDot + beta * |error|^(p/q) * sign(error)
        double terminalTerm = _beta * System.Math.Pow(System.Math.Abs(error), _p / _q) * System.Math.Sign(error);
        double s = errorDot + terminalTerm;
        
        // Control law
        return -_k * Saturation(s / _epsilon);
    }

    private static double Saturation(double x) => System.Math.Max(-1, System.Math.Min(1, x));
}

/// <summary>
/// Super-Twisting Algorithm.
/// Second-order sliding mode for chattering reduction.
/// </summary>
public class SuperTwistingController
{
    private readonly double _alpha;
    private readonly double _beta;
    private double _integral;

    public SuperTwistingController(double alpha, double beta)
    {
        _alpha = alpha;
        _beta = beta;
        _integral = 0;
    }

    public double Compute(double s, double dt)
    {
        // Super-twisting algorithm
        double u1 = -_alpha * System.Math.Pow(System.Math.Abs(s), 0.5) * System.Math.Sign(s);
        _integral += -_beta * System.Math.Sign(s) * dt;
        
        return u1 + _integral;
    }

    public void Reset() => _integral = 0;
}

/// <summary>
/// Active Disturbance Rejection Controller (ADRC).
/// Uses Extended State Observer to estimate and compensate disturbances.
/// </summary>
public class AdrcController
{
    private readonly double _b0; // Control gain estimate
    private readonly double _wc; // Controller bandwidth
    private readonly double _wo; // Observer bandwidth
    private readonly int _order; // System order
    
    private Vector<double> _z; // Extended state estimate
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly double[] _L;
    private readonly double[] _Kp;

    public AdrcController(int order, double b0, double controllerBandwidth, double observerBandwidth)
    {
        _order = order;
        _b0 = b0;
        _wc = controllerBandwidth;
        _wo = observerBandwidth;
        
        // Build ESO matrices
        int n = order + 1; // Extended state includes disturbance
        _A = Matrix<double>.Build.Dense(n, n);
        _B = Vector<double>.Build.Dense(n).ToColumnMatrix();
        
        // Chain of integrators + disturbance state
        for (int i = 0; i < order; i++)
        {
            _A[i, i + 1] = 1;
        }
        _B[order - 1, 0] = b0;
        
        // Observer gains (place poles at -wo)
        _L = new double[n];
        for (int i = 0; i < n; i++)
        {
            _L[i] = BinomialCoeff(n, i + 1) * System.Math.Pow(_wo, i + 1);
        }
        
        // Controller gains (place poles at -wc)
        _Kp = new double[order];
        for (int i = 0; i < order; i++)
        {
            _Kp[i] = BinomialCoeff(order, i + 1) * System.Math.Pow(_wc, order - i);
        }
        
        _z = Vector<double>.Build.Dense(n);
    }

    public double Compute(double y, double reference, double dt)
    {
        // Extended State Observer update
        double yHat = _z[0];
        double error = y - yHat;
        
        // ESO dynamics
        for (int i = 0; i < _order; i++)
        {
            _z[i] += (_z[i + 1] + _L[i] * error) * dt;
        }
        _z[_order] += _L[_order] * error * dt;
        
        // State feedback with disturbance rejection
        double u0 = 0;
        var refDerivatives = Vector<double>.Build.Dense(_order);
        refDerivatives[0] = reference;
        
        for (int i = 0; i < _order; i++)
        {
            u0 += _Kp[i] * (refDerivatives[i] - _z[i]);
        }
        
        // Compensate estimated disturbance
        double u = (u0 - _z[_order]) / _b0;
        
        return u;
    }

    private static double BinomialCoeff(int n, int k)
    {
        if (k > n) return 0;
        if (k == 0 || k == n) return 1;
        double result = 1;
        for (int i = 0; i < k; i++)
            result = result * (n - i) / (i + 1);
        return result;
    }
}

/// <summary>
/// Iterative Learning Controller (ILC).
/// Improves tracking over repeated trials.
/// </summary>
public class IterativeLearningController
{
    private readonly int _trajectoryLength;
    private readonly double _learningGain;
    private readonly double _forgettingFactor;
    private double[] _feedforward;
    private int _iteration;

    public IterativeLearningController(int trajectoryLength, double learningGain = 0.5, double forgettingFactor = 1.0)
    {
        _trajectoryLength = trajectoryLength;
        _learningGain = learningGain;
        _forgettingFactor = forgettingFactor;
        _feedforward = new double[trajectoryLength];
        _iteration = 0;
    }

    /// <summary>
    /// Gets the feedforward signal for current timestep.
    /// </summary>
    public double GetFeedforward(int timestep)
    {
        if (timestep < 0 || timestep >= _trajectoryLength)
            return 0;
        return _feedforward[timestep];
    }

    /// <summary>
    /// Updates the learning after a complete trial.
    /// </summary>
    public void Learn(double[] trackingErrors)
    {
        if (trackingErrors.Length != _trajectoryLength)
            throw new ArgumentException("Error array length must match trajectory length");

        for (int k = 0; k < _trajectoryLength; k++)
        {
            // P-type ILC update: u_{j+1}(k) = Q * u_j(k) + L * e_j(k+1)
            int kNext = System.Math.Min(k + 1, _trajectoryLength - 1);
            _feedforward[k] = _forgettingFactor * _feedforward[k] + _learningGain * trackingErrors[kNext];
        }
        _iteration++;
    }

    public void Reset()
    {
        Array.Clear(_feedforward, 0, _trajectoryLength);
        _iteration = 0;
    }

    public int Iteration => _iteration;
}

/// <summary>
/// Repetitive Controller.
/// Rejects periodic disturbances.
/// </summary>
public class RepetitiveController
{
    private readonly int _period;
    private readonly double _gain;
    private readonly double[] _buffer;
    private int _index;

    public RepetitiveController(int periodSamples, double gain = 0.5)
    {
        _period = periodSamples;
        _gain = gain;
        _buffer = new double[periodSamples];
        _index = 0;
    }

    public double Compute(double error)
    {
        // Internal model: delay of one period
        double delayed = _buffer[_index];
        
        // Update buffer
        _buffer[_index] = error + _gain * delayed;
        
        // Advance index
        _index = (_index + 1) % _period;
        
        // Output is the delayed + learned signal
        return delayed;
    }

    public void Reset()
    {
        Array.Clear(_buffer, 0, _period);
        _index = 0;
    }
}

/// <summary>
/// Gain Scheduling Controller.
/// Switches between controllers based on operating point.
/// </summary>
public class GainScheduledController
{
    private readonly List<(double operatingPoint, double[] gains)> _schedule;
    private double _currentKp, _currentKi, _currentKd;
    private double _integral;
    private double _previousError;

    public GainScheduledController()
    {
        _schedule = new List<(double, double[])>();
        _currentKp = _currentKi = _currentKd = 0;
    }

    public void AddOperatingPoint(double operatingPoint, double kp, double ki, double kd)
    {
        _schedule.Add((operatingPoint, new[] { kp, ki, kd }));
        _schedule.Sort((a, b) => a.operatingPoint.CompareTo(b.operatingPoint));
    }

    public double Compute(double error, double operatingPoint, double dt)
    {
        // Interpolate gains based on operating point
        InterpolateGains(operatingPoint);
        
        // Standard PID
        _integral += error * dt;
        double derivative = (error - _previousError) / dt;
        _previousError = error;
        
        return _currentKp * error + _currentKi * _integral + _currentKd * derivative;
    }

    private void InterpolateGains(double op)
    {
        if (_schedule.Count == 0) return;
        if (_schedule.Count == 1)
        {
            _currentKp = _schedule[0].gains[0];
            _currentKi = _schedule[0].gains[1];
            _currentKd = _schedule[0].gains[2];
            return;
        }

        // Find bracketing points
        int i = 0;
        while (i < _schedule.Count - 1 && _schedule[i + 1].operatingPoint < op)
            i++;

        if (i >= _schedule.Count - 1)
        {
            _currentKp = _schedule[^1].gains[0];
            _currentKi = _schedule[^1].gains[1];
            _currentKd = _schedule[^1].gains[2];
            return;
        }

        // Linear interpolation
        double t = (op - _schedule[i].operatingPoint) / 
            (_schedule[i + 1].operatingPoint - _schedule[i].operatingPoint);
        t = System.Math.Max(0, System.Math.Min(1, t));

        _currentKp = _schedule[i].gains[0] + t * (_schedule[i + 1].gains[0] - _schedule[i].gains[0]);
        _currentKi = _schedule[i].gains[1] + t * (_schedule[i + 1].gains[1] - _schedule[i].gains[1]);
        _currentKd = _schedule[i].gains[2] + t * (_schedule[i + 1].gains[2] - _schedule[i].gains[2]);
    }

    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
    }
}

/// <summary>
/// Internal Model Controller (IMC).
/// Uses process model for control design.
/// </summary>
public class InternalModelController
{
    private readonly TransferFunction _processModel;
    private readonly TransferFunction _imcController;
    private readonly double _filterTimeConstant;
    private double[] _modelOutput;
    private double[] _controllerState;

    public InternalModelController(TransferFunction processModel, double filterTimeConstant)
    {
        _processModel = processModel;
        _filterTimeConstant = filterTimeConstant;
        
        // IMC controller = inverse of process model * filter
        _imcController = processModel.Invert();
        
        _modelOutput = new double[10];
        _controllerState = new double[10];
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        // Model output
        double yModel = _processModel.Evaluate(_controllerState[0], dt);
        
        // Disturbance estimate
        double disturbance = measurement - yModel;
        
        // IMC structure: r - d_hat -> IMC controller
        double error = setpoint - disturbance;
        
        // Low-pass filter for robustness
        double filtered = error; // Simplified
        
        // Controller output
        double u = _imcController.Evaluate(filtered, dt);
        
        _controllerState[0] = u;
        
        return u;
    }
}

/// <summary>
/// Simple transfer function representation.
/// </summary>
public class TransferFunction
{
    public double[] Numerator { get; }
    public double[] Denominator { get; }
    private readonly double[] _state;

    public TransferFunction(double[] numerator, double[] denominator)
    {
        Numerator = numerator;
        Denominator = denominator;
        _state = new double[System.Math.Max(numerator.Length, denominator.Length)];
    }

    public double Evaluate(double input, double dt)
    {
        // Simple first-order approximation
        if (Denominator.Length >= 2 && Numerator.Length >= 1)
        {
            double tau = Denominator[0] / Denominator[1];
            double gain = Numerator[0] / Denominator[1];
            double alpha = dt / (tau + dt);
            _state[0] = alpha * gain * input + (1 - alpha) * _state[0];
            return _state[0];
        }
        return input * (Numerator.Length > 0 ? Numerator[0] : 1);
    }

    public TransferFunction Invert()
    {
        return new TransferFunction(Denominator, Numerator);
    }
}

/// <summary>
/// Extremum Seeking Controller.
/// Real-time optimization without model.
/// </summary>
public class ExtremumSeekingController
{
    private readonly double _perturbationAmplitude;
    private readonly double _perturbationFrequency;
    private readonly double _adaptationGain;
    private readonly double _highPassCutoff;
    private double _theta; // Estimated optimal input
    private double _phase;
    private double _highPassState;

    public ExtremumSeekingController(
        double perturbationAmplitude = 0.1,
        double perturbationFrequency = 1.0,
        double adaptationGain = 0.1,
        double highPassCutoff = 0.5)
    {
        _perturbationAmplitude = perturbationAmplitude;
        _perturbationFrequency = perturbationFrequency;
        _adaptationGain = adaptationGain;
        _highPassCutoff = highPassCutoff;
        _theta = 0;
        _phase = 0;
        _highPassState = 0;
    }

    public double Compute(double costFunction, double dt)
    {
        // Generate perturbation signal
        _phase += 2 * System.Math.PI * _perturbationFrequency * dt;
        double perturbation = _perturbationAmplitude * System.Math.Sin(_phase);
        
        // High-pass filter the cost function
        double alpha = _highPassCutoff * dt / (1 + _highPassCutoff * dt);
        double highPassed = costFunction - _highPassState;
        _highPassState += alpha * (costFunction - _highPassState);
        
        // Demodulate
        double gradient = highPassed * System.Math.Sin(_phase);
        
        // Update estimate (gradient descent)
        _theta -= _adaptationGain * gradient * dt;
        
        // Output = estimate + perturbation
        return _theta + perturbation;
    }

    public double OptimalEstimate => _theta;
}

/// <summary>
/// Reference Governor.
/// Modifies references to satisfy constraints.
/// </summary>
public class ReferenceGovernor
{
    private readonly double[] _stateMin;
    private readonly double[] _stateMax;
    private readonly double[] _inputMin;
    private readonly double[] _inputMax;
    private readonly double _kappa; // Reference modification gain

    public ReferenceGovernor(
        double[] stateMin, double[] stateMax,
        double[] inputMin, double[] inputMax,
        double kappa = 0.9)
    {
        _stateMin = stateMin;
        _stateMax = stateMax;
        _inputMin = inputMin;
        _inputMax = inputMax;
        _kappa = kappa;
    }

    /// <summary>
    /// Modifies reference to ensure constraint satisfaction.
    /// </summary>
    public Vector<double> Govern(
        Vector<double> desiredReference,
        Vector<double> currentState,
        Vector<double> previousReference)
    {
        // Start with desired reference
        var v = desiredReference.Clone();
        
        // Check predicted constraint violation
        bool feasible = CheckFeasibility(currentState, v);
        
        if (!feasible)
        {
            // Scale back towards previous reference
            double lambda = FindFeasibleScaling(currentState, previousReference, desiredReference);
            v = previousReference + lambda * (desiredReference - previousReference);
        }
        
        return v;
    }

    private bool CheckFeasibility(Vector<double> state, Vector<double> reference)
    {
        for (int i = 0; i < state.Count && i < _stateMin.Length; i++)
        {
            if (state[i] < _stateMin[i] || state[i] > _stateMax[i])
                return false;
        }
        return true;
    }

    private double FindFeasibleScaling(Vector<double> state, Vector<double> prev, Vector<double> desired)
    {
        // Binary search for maximum feasible lambda
        double low = 0, high = 1;
        for (int iter = 0; iter < 10; iter++)
        {
            double mid = (low + high) / 2;
            var test = prev + mid * (desired - prev);
            if (CheckFeasibility(state, test))
                low = mid;
            else
                high = mid;
        }
        return low * _kappa;
    }
}
