using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.StateEstimation;

/// <summary>
/// Unscented Kalman Filter (UKF) for nonlinear state estimation.
/// Uses sigma points to capture the probability distribution.
/// </summary>
public class UnscentedKalmanFilter
{
    private readonly int _n; // State dimension
    private readonly double _alpha;
    private readonly double _beta;
    private readonly double _kappa;
    private readonly double _lambda;
    
    private Vector<double> _x; // State estimate
    private Matrix<double> _P; // State covariance
    private readonly Matrix<double> _Q; // Process noise
    private readonly Matrix<double> _R; // Measurement noise
    
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f; // State transition
    private readonly Func<Vector<double>, Vector<double>> _h; // Measurement function

    public UnscentedKalmanFilter(
        int stateSize,
        int measurementSize,
        Func<Vector<double>, Vector<double>, Vector<double>> stateTransition,
        Func<Vector<double>, Vector<double>> measurementFunction,
        Matrix<double> processNoise,
        Matrix<double> measurementNoise,
        double alpha = 0.001,
        double beta = 2.0,
        double kappa = 0.0)
    {
        _n = stateSize;
        _alpha = alpha;
        _beta = beta;
        _kappa = kappa;
        _lambda = alpha * alpha * (_n + kappa) - _n;
        
        _f = stateTransition;
        _h = measurementFunction;
        _Q = processNoise;
        _R = measurementNoise;
        
        _x = Vector<double>.Build.Dense(_n);
        _P = Matrix<double>.Build.DenseIdentity(_n);
    }

    public void Initialize(Vector<double> initialState, Matrix<double> initialCovariance)
    {
        _x = initialState.Clone();
        _P = initialCovariance.Clone();
    }

    /// <summary>
    /// Prediction step.
    /// </summary>
    public void Predict(Vector<double> u)
    {
        // Generate sigma points
        var (sigmaPoints, Wm, Wc) = GenerateSigmaPoints(_x, _P);
        
        // Propagate through state transition
        var propagatedPoints = new Vector<double>[2 * _n + 1];
        for (int i = 0; i <= 2 * _n; i++)
        {
            propagatedPoints[i] = _f(sigmaPoints[i], u);
        }
        
        // Compute predicted mean
        _x = Vector<double>.Build.Dense(_n);
        for (int i = 0; i <= 2 * _n; i++)
        {
            _x += Wm[i] * propagatedPoints[i];
        }
        
        // Compute predicted covariance
        _P = _Q.Clone();
        for (int i = 0; i <= 2 * _n; i++)
        {
            var diff = propagatedPoints[i] - _x;
            _P += Wc[i] * diff.OuterProduct(diff);
        }
    }

    /// <summary>
    /// Update step with measurement.
    /// </summary>
    public void Update(Vector<double> z)
    {
        int m = z.Count;
        
        // Generate sigma points
        var (sigmaPoints, Wm, Wc) = GenerateSigmaPoints(_x, _P);
        
        // Propagate through measurement function
        var measuredPoints = new Vector<double>[2 * _n + 1];
        for (int i = 0; i <= 2 * _n; i++)
        {
            measuredPoints[i] = _h(sigmaPoints[i]);
        }
        
        // Predicted measurement mean
        var zPred = Vector<double>.Build.Dense(m);
        for (int i = 0; i <= 2 * _n; i++)
        {
            zPred += Wm[i] * measuredPoints[i];
        }
        
        // Innovation covariance
        var Pzz = _R.Clone();
        for (int i = 0; i <= 2 * _n; i++)
        {
            var diff = measuredPoints[i] - zPred;
            Pzz += Wc[i] * diff.OuterProduct(diff);
        }
        
        // Cross covariance
        var Pxz = Matrix<double>.Build.Dense(_n, m);
        for (int i = 0; i <= 2 * _n; i++)
        {
            var xDiff = sigmaPoints[i] - _x;
            var zDiff = measuredPoints[i] - zPred;
            Pxz += Wc[i] * xDiff.OuterProduct(zDiff);
        }
        
        // Kalman gain
        var K = Pxz * Pzz.Inverse();
        
        // Update state and covariance
        var innovation = z - zPred;
        _x = _x + K * innovation;
        _P = _P - K * Pzz * K.Transpose();
    }

    private (Vector<double>[] points, double[] Wm, double[] Wc) GenerateSigmaPoints(Vector<double> x, Matrix<double> P)
    {
        int numPoints = 2 * _n + 1;
        var points = new Vector<double>[numPoints];
        var Wm = new double[numPoints];
        var Wc = new double[numPoints];
        
        // Weights
        Wm[0] = _lambda / (_n + _lambda);
        Wc[0] = Wm[0] + (1 - _alpha * _alpha + _beta);
        for (int i = 1; i < numPoints; i++)
        {
            Wm[i] = 1.0 / (2 * (_n + _lambda));
            Wc[i] = Wm[i];
        }
        
        // Matrix square root
        var sqrtP = (((_n + _lambda) * P)).Cholesky().Factor;
        
        // Sigma points
        points[0] = x.Clone();
        for (int i = 0; i < _n; i++)
        {
            var col = sqrtP.Column(i);
            points[i + 1] = x + col;
            points[i + 1 + _n] = x - col;
        }
        
        return (points, Wm, Wc);
    }

    public Vector<double> State => _x;
    public Matrix<double> Covariance => _P;
}

/// <summary>
/// Particle Filter for highly nonlinear/non-Gaussian estimation.
/// </summary>
public class ParticleFilter
{
    private readonly int _numParticles;
    private readonly int _stateSize;
    private Vector<double>[] _particles;
    private double[] _weights;
    private readonly Random _rng;
    
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _motionModel;
    private readonly Func<Vector<double>, Vector<double>, double> _measurementLikelihood;
    private readonly double[] _processNoise;

    public ParticleFilter(
        int numParticles,
        int stateSize,
        Func<Vector<double>, Vector<double>, Vector<double>> motionModel,
        Func<Vector<double>, Vector<double>, double> measurementLikelihood,
        double[] processNoise)
    {
        _numParticles = numParticles;
        _stateSize = stateSize;
        _motionModel = motionModel;
        _measurementLikelihood = measurementLikelihood;
        _processNoise = processNoise;
        _rng = new Random();
        
        _particles = new Vector<double>[numParticles];
        _weights = new double[numParticles];
        
        for (int i = 0; i < numParticles; i++)
        {
            _particles[i] = Vector<double>.Build.Dense(stateSize);
            _weights[i] = 1.0 / numParticles;
        }
    }

    /// <summary>
    /// Initialize particles around initial estimate.
    /// </summary>
    public void Initialize(Vector<double> initialState, double[] initialSpread)
    {
        for (int i = 0; i < _numParticles; i++)
        {
            _particles[i] = Vector<double>.Build.Dense(_stateSize);
            for (int j = 0; j < _stateSize; j++)
            {
                _particles[i][j] = initialState[j] + SampleGaussian() * initialSpread[j];
            }
            _weights[i] = 1.0 / _numParticles;
        }
    }

    /// <summary>
    /// Prediction step - propagate particles through motion model.
    /// </summary>
    public void Predict(Vector<double> u)
    {
        for (int i = 0; i < _numParticles; i++)
        {
            // Apply motion model
            _particles[i] = _motionModel(_particles[i], u);
            
            // Add process noise
            for (int j = 0; j < _stateSize; j++)
            {
                _particles[i][j] += SampleGaussian() * _processNoise[j];
            }
        }
    }

    /// <summary>
    /// Update step - weight particles by measurement likelihood.
    /// </summary>
    public void Update(Vector<double> z)
    {
        // Compute weights
        double sumWeights = 0;
        for (int i = 0; i < _numParticles; i++)
        {
            _weights[i] = _measurementLikelihood(_particles[i], z);
            sumWeights += _weights[i];
        }
        
        // Normalize weights
        if (sumWeights > 0)
        {
            for (int i = 0; i < _numParticles; i++)
            {
                _weights[i] /= sumWeights;
            }
        }
        
        // Resample if effective sample size is too low
        double nEff = 1.0 / _weights.Sum(w => w * w);
        if (nEff < _numParticles / 2)
        {
            Resample();
        }
    }

    private void Resample()
    {
        var newParticles = new Vector<double>[_numParticles];
        var cumSum = new double[_numParticles];
        
        cumSum[0] = _weights[0];
        for (int i = 1; i < _numParticles; i++)
        {
            cumSum[i] = cumSum[i - 1] + _weights[i];
        }
        
        // Systematic resampling
        double step = 1.0 / _numParticles;
        double u = _rng.NextDouble() * step;
        
        int j = 0;
        for (int i = 0; i < _numParticles; i++)
        {
            while (j < _numParticles - 1 && u > cumSum[j])
            {
                j++;
            }
            newParticles[i] = _particles[j].Clone();
            u += step;
        }
        
        _particles = newParticles;
        for (int i = 0; i < _numParticles; i++)
        {
            _weights[i] = 1.0 / _numParticles;
        }
    }

    /// <summary>
    /// Get the weighted mean estimate.
    /// </summary>
    public Vector<double> GetEstimate()
    {
        var estimate = Vector<double>.Build.Dense(_stateSize);
        for (int i = 0; i < _numParticles; i++)
        {
            estimate += _weights[i] * _particles[i];
        }
        return estimate;
    }

    /// <summary>
    /// Get the weighted covariance.
    /// </summary>
    public Matrix<double> GetCovariance()
    {
        var mean = GetEstimate();
        var cov = Matrix<double>.Build.Dense(_stateSize, _stateSize);
        
        for (int i = 0; i < _numParticles; i++)
        {
            var diff = _particles[i] - mean;
            cov += _weights[i] * diff.OuterProduct(diff);
        }
        
        return cov;
    }

    private double SampleGaussian()
    {
        double u1 = 1.0 - _rng.NextDouble();
        double u2 = 1.0 - _rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

/// <summary>
/// Information Filter - dual of Kalman Filter using information form.
/// More efficient for multi-sensor fusion.
/// </summary>
public class InformationFilter
{
    private Vector<double> _y; // Information vector (Y = P^-1 * x)
    private Matrix<double> _Y; // Information matrix (Y = P^-1)
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _H;
    private readonly Matrix<double> _R;

    public InformationFilter(
        Matrix<double> A,
        Matrix<double> B,
        Matrix<double> Q,
        Matrix<double> H,
        Matrix<double> R)
    {
        _A = A;
        _B = B;
        _Q = Q;
        _H = H;
        _R = R;
        
        int n = A.RowCount;
        _y = Vector<double>.Build.Dense(n);
        _Y = Matrix<double>.Build.Dense(n, n); // Start with zero information
    }

    public void Initialize(Vector<double> state, Matrix<double> covariance)
    {
        _Y = covariance.Inverse();
        _y = _Y * state;
    }

    public void Predict(Vector<double> u)
    {
        // Convert to state space
        var P = _Y.Inverse();
        var x = P * _y;
        
        // Prediction in state space
        var xPred = _A * x + _B * u;
        var PPred = _A * P * _A.Transpose() + _Q;
        
        // Convert back to information form
        _Y = PPred.Inverse();
        _y = _Y * xPred;
    }

    public void Update(Vector<double> z)
    {
        var RInv = _R.Inverse();
        var i = _H.Transpose() * RInv * z; // Information contribution
        var I = _H.Transpose() * RInv * _H; // Information matrix contribution
        
        // Simple additive update
        _y = _y + i;
        _Y = _Y + I;
    }

    /// <summary>
    /// Fuse information from multiple sensors.
    /// </summary>
    public void Fuse(Vector<double>[] measurements, Matrix<double>[] measurementMatrices, Matrix<double>[] noiseCovariances)
    {
        for (int i = 0; i < measurements.Length; i++)
        {
            var H = measurementMatrices[i];
            var R = noiseCovariances[i];
            var z = measurements[i];
            
            var RInv = R.Inverse();
            _y = _y + H.Transpose() * RInv * z;
            _Y = _Y + H.Transpose() * RInv * H;
        }
    }

    public Vector<double> State => _Y.Inverse() * _y;
    public Matrix<double> Covariance => _Y.Inverse();
}

/// <summary>
/// Moving Horizon Estimator - optimization-based state estimation.
/// </summary>
public class MovingHorizonEstimator
{
    private readonly int _horizon;
    private readonly int _stateSize;
    private readonly int _measurementSize;
    private readonly Queue<Vector<double>> _measurementHistory;
    private readonly Queue<Vector<double>> _inputHistory;
    
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Vector<double>> _h;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;

    public MovingHorizonEstimator(
        int horizon,
        int stateSize,
        int measurementSize,
        Func<Vector<double>, Vector<double>, Vector<double>> stateTransition,
        Func<Vector<double>, Vector<double>> measurementFunction,
        Matrix<double> processNoise,
        Matrix<double> measurementNoise)
    {
        _horizon = horizon;
        _stateSize = stateSize;
        _measurementSize = measurementSize;
        _f = stateTransition;
        _h = measurementFunction;
        _Q = processNoise;
        _R = measurementNoise;
        
        _measurementHistory = new Queue<Vector<double>>();
        _inputHistory = new Queue<Vector<double>>();
    }

    /// <summary>
    /// Process new measurement and estimate state.
    /// </summary>
    public Vector<double> Estimate(Vector<double> z, Vector<double> u, Vector<double> xPrior)
    {
        _measurementHistory.Enqueue(z);
        _inputHistory.Enqueue(u);
        
        // Maintain horizon length
        while (_measurementHistory.Count > _horizon)
        {
            _measurementHistory.Dequeue();
            _inputHistory.Dequeue();
        }
        
        // Solve optimization problem (simplified - uses iterative refinement)
        var measurements = _measurementHistory.ToArray();
        var inputs = _inputHistory.ToArray();
        
        return SolveOptimization(xPrior, measurements, inputs);
    }

    private Vector<double> SolveOptimization(Vector<double> xPrior, Vector<double>[] measurements, Vector<double>[] inputs)
    {
        int N = measurements.Length;
        var x = new Vector<double>[N + 1];
        x[0] = xPrior.Clone();
        
        // Forward simulation
        for (int k = 0; k < N; k++)
        {
            x[k + 1] = _f(x[k], inputs[k]);
        }
        
        // Gauss-Newton iterations
        for (int iter = 0; iter < 5; iter++)
        {
            // Compute residuals and Jacobians
            var totalGradient = Vector<double>.Build.Dense(_stateSize);
            var totalHessian = Matrix<double>.Build.Dense(_stateSize, _stateSize);
            
            for (int k = 0; k < N; k++)
            {
                var zPred = _h(x[k + 1]);
                var residual = measurements[k] - zPred;
                
                // Simplified gradient update
                var H = NumericalJacobian(v => _h(v), x[k + 1]);
                totalGradient += H.Transpose() * _R.Inverse() * residual;
                totalHessian += H.Transpose() * _R.Inverse() * H;
            }
            
            // Update initial state estimate
            if (totalHessian.Determinant() != 0)
            {
                var dx = totalHessian.Solve(totalGradient);
                x[0] += 0.5 * dx; // Damped update
                
                // Re-simulate
                for (int k = 0; k < N; k++)
                {
                    x[k + 1] = _f(x[k], inputs[k]);
                }
            }
        }
        
        return x[N]; // Return latest state estimate
    }

    private Matrix<double> NumericalJacobian(Func<Vector<double>, Vector<double>> f, Vector<double> x)
    {
        double eps = 1e-6;
        var y0 = f(x);
        int m = y0.Count;
        int n = x.Count;
        
        var J = Matrix<double>.Build.Dense(m, n);
        
        for (int j = 0; j < n; j++)
        {
            var xPlus = x.Clone();
            xPlus[j] += eps;
            var yPlus = f(xPlus);
            
            for (int i = 0; i < m; i++)
            {
                J[i, j] = (yPlus[i] - y0[i]) / eps;
            }
        }
        
        return J;
    }
}
 