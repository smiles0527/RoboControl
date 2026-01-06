using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.StateEstimation;

/// <summary>
/// Square Root Kalman Filter.
/// Numerically more stable than standard Kalman filter.
/// </summary>
public class SquareRootKalmanFilter
{
    private readonly int _n; // State size
    private readonly int _m; // Measurement size
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _H;
    private readonly Matrix<double> _sqrtQ; // Square root of process noise
    private readonly Matrix<double> _sqrtR; // Square root of measurement noise
    
    private Vector<double> _x; // State estimate
    private Matrix<double> _S; // Square root of covariance (P = S * S')

    public SquareRootKalmanFilter(
        Matrix<double> A, Matrix<double> B, Matrix<double> H,
        Matrix<double> Q, Matrix<double> R)
    {
        _A = A;
        _B = B;
        _H = H;
        _n = A.RowCount;
        _m = H.RowCount;
        
        // Compute square roots via Cholesky
        _sqrtQ = Q.Cholesky().Factor;
        _sqrtR = R.Cholesky().Factor;
        
        _x = Vector<double>.Build.Dense(_n);
        _S = Matrix<double>.Build.DenseIdentity(_n);
    }

    public void Initialize(Vector<double> state, Matrix<double> covariance)
    {
        _x = state.Clone();
        _S = covariance.Cholesky().Factor;
    }

    public void Predict(Vector<double> u)
    {
        // Predict state
        _x = _A * _x + _B * u;
        
        // Update square root covariance using QR decomposition
        // Construct compound matrix [A*S, sqrt(Q)]
        var compound = Matrix<double>.Build.Dense(_n, 2 * _n);
        compound.SetSubMatrix(0, 0, _A * _S);
        compound.SetSubMatrix(0, _n, _sqrtQ);
        
        // QR decomposition to get new S
        var qr = compound.Transpose().QR();
        _S = qr.R.SubMatrix(0, _n, 0, _n).Transpose();
    }

    public void Update(Vector<double> z)
    {
        // Innovation
        var zPred = _H * _x;
        var innovation = z - zPred;
        
        // Compound matrix for measurement update
        var HS = _H * _S;
        var compound = Matrix<double>.Build.Dense(_m + _n, _m);
        compound.SetSubMatrix(0, 0, _sqrtR);
        compound.SetSubMatrix(_m, 0, HS.Transpose());
        
        // QR decomposition
        var qr = compound.QR();
        var sqrtSy = qr.R.SubMatrix(0, _m, 0, _m);
        
        // Kalman gain
        var K = _S * HS.Transpose() * sqrtSy.Transpose().Inverse() * sqrtSy.Inverse();
        
        // Update state
        _x = _x + K * innovation;
        
        // Update covariance square root
        _S = _S - K * _H * _S;
        
        // Ensure positive definiteness via Cholesky
        var P = _S * _S.Transpose();
        _S = P.Cholesky().Factor;
    }

    public Vector<double> State => _x;
    public Matrix<double> Covariance => _S * _S.Transpose();
}

/// <summary>
/// Interacting Multiple Model (IMM) Estimator.
/// Handles mode transitions in hybrid systems.
/// </summary>
public class ImmEstimator
{
    private readonly int _numModels;
    private readonly int _stateSize;
    private readonly ExtendedKalmanFilter[] _filters;
    private readonly Matrix<double> _transitionMatrix;
    private double[] _modeProbabilities;

    public ImmEstimator(
        int stateSize,
        Func<Vector<double>, Vector<double>, Vector<double>>[] stateTransitions,
        Func<Vector<double>, Vector<double>>[] measurementFunctions,
        Matrix<double>[] processNoises,
        Matrix<double>[] measurementNoises,
        Matrix<double> modeTransitionProbabilities)
    {
        _numModels = stateTransitions.Length;
        _stateSize = stateSize;
        _transitionMatrix = modeTransitionProbabilities;
        _modeProbabilities = new double[_numModels];
        
        // Initialize with uniform probabilities
        for (int i = 0; i < _numModels; i++)
        {
            _modeProbabilities[i] = 1.0 / _numModels;
        }
        
        // Create filters for each model
        _filters = new ExtendedKalmanFilter[_numModels];
        for (int i = 0; i < _numModels; i++)
        {
            _filters[i] = new ExtendedKalmanFilter(
                stateSize,
                measurementNoises[i].RowCount,
                stateTransitions[i],
                measurementFunctions[i],
                processNoises[i],
                measurementNoises[i]);
        }
    }

    public void Initialize(Vector<double> initialState, Matrix<double> initialCovariance)
    {
        foreach (var filter in _filters)
        {
            filter.Initialize(initialState, initialCovariance);
        }
    }

    public void Predict(Vector<double> u)
    {
        // Compute mixing probabilities
        var cBar = Vector<double>.Build.Dense(_numModels);
        for (int j = 0; j < _numModels; j++)
        {
            for (int i = 0; i < _numModels; i++)
            {
                cBar[j] += _transitionMatrix[i, j] * _modeProbabilities[i];
            }
        }
        
        var mixingProbs = Matrix<double>.Build.Dense(_numModels, _numModels);
        for (int i = 0; i < _numModels; i++)
        {
            for (int j = 0; j < _numModels; j++)
            {
                mixingProbs[i, j] = _transitionMatrix[i, j] * _modeProbabilities[i] / cBar[j];
            }
        }
        
        // Compute mixed initial conditions
        var mixedStates = new Vector<double>[_numModels];
        var mixedCovariances = new Matrix<double>[_numModels];
        
        for (int j = 0; j < _numModels; j++)
        {
            mixedStates[j] = Vector<double>.Build.Dense(_stateSize);
            for (int i = 0; i < _numModels; i++)
            {
                mixedStates[j] += mixingProbs[i, j] * _filters[i].State;
            }
            
            mixedCovariances[j] = Matrix<double>.Build.Dense(_stateSize, _stateSize);
            for (int i = 0; i < _numModels; i++)
            {
                var diff = _filters[i].State - mixedStates[j];
                mixedCovariances[j] += mixingProbs[i, j] * 
                    (_filters[i].Covariance + diff.OuterProduct(diff));
            }
        }
        
        // Reinitialize and predict each filter
        for (int j = 0; j < _numModels; j++)
        {
            _filters[j].Initialize(mixedStates[j], mixedCovariances[j]);
            _filters[j].Predict(u);
        }
        
        _modeProbabilities = cBar.ToArray();
    }

    public void Update(Vector<double> z)
    {
        var likelihoods = new double[_numModels];
        double totalLikelihood = 0;
        
        // Update each filter and compute likelihood
        for (int j = 0; j < _numModels; j++)
        {
            // Store pre-update values
            var xPred = _filters[j].State.Clone();
            
            _filters[j].Update(z);
            
            // Compute likelihood (simplified Gaussian)
            var innovation = z - _filters[j].MeasurementFunction(_filters[j].State);
            double logLikelihood = -0.5 * innovation.DotProduct(innovation);
            likelihoods[j] = System.Math.Exp(logLikelihood);
            
            totalLikelihood += likelihoods[j] * _modeProbabilities[j];
        }
        
        // Update mode probabilities
        for (int j = 0; j < _numModels; j++)
        {
            _modeProbabilities[j] = likelihoods[j] * _modeProbabilities[j] / totalLikelihood;
        }
    }

    /// <summary>
    /// Gets the combined state estimate.
    /// </summary>
    public Vector<double> GetEstimate()
    {
        var estimate = Vector<double>.Build.Dense(_stateSize);
        for (int j = 0; j < _numModels; j++)
        {
            estimate += _modeProbabilities[j] * _filters[j].State;
        }
        return estimate;
    }

    /// <summary>
    /// Gets the combined covariance.
    /// </summary>
    public Matrix<double> GetCovariance()
    {
        var mean = GetEstimate();
        var cov = Matrix<double>.Build.Dense(_stateSize, _stateSize);
        
        for (int j = 0; j < _numModels; j++)
        {
            var diff = _filters[j].State - mean;
            cov += _modeProbabilities[j] * 
                (_filters[j].Covariance + diff.OuterProduct(diff));
        }
        
        return cov;
    }

    public double[] ModeProbabilities => _modeProbabilities;
    public int MostLikelyMode => Array.IndexOf(_modeProbabilities, _modeProbabilities.Max());
}

/// <summary>
/// Extended Kalman Filter reference for IMM.
/// </summary>
public class ExtendedKalmanFilter
{
    private readonly int _n;
    private readonly int _m;
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Vector<double>> _h;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    
    private Vector<double> _x;
    private Matrix<double> _P;

    public Func<Vector<double>, Vector<double>> MeasurementFunction => _h;

    public ExtendedKalmanFilter(
        int stateSize, int measurementSize,
        Func<Vector<double>, Vector<double>, Vector<double>> stateTransition,
        Func<Vector<double>, Vector<double>> measurementFunction,
        Matrix<double> processNoise,
        Matrix<double> measurementNoise)
    {
        _n = stateSize;
        _m = measurementSize;
        _f = stateTransition;
        _h = measurementFunction;
        _Q = processNoise;
        _R = measurementNoise;
        
        _x = Vector<double>.Build.Dense(_n);
        _P = Matrix<double>.Build.DenseIdentity(_n);
    }

    public void Initialize(Vector<double> state, Matrix<double> covariance)
    {
        _x = state.Clone();
        _P = covariance.Clone();
    }

    public void Predict(Vector<double> u)
    {
        // Compute Jacobian of f
        var F = NumericalJacobian(x => _f(x, u), _x);
        
        // Predict
        _x = _f(_x, u);
        _P = F * _P * F.Transpose() + _Q;
    }

    public void Update(Vector<double> z)
    {
        // Compute Jacobian of h
        var H = NumericalJacobian(_h, _x);
        
        // Innovation
        var y = z - _h(_x);
        var S = H * _P * H.Transpose() + _R;
        var K = _P * H.Transpose() * S.Inverse();
        
        // Update
        _x = _x + K * y;
        _P = (Matrix<double>.Build.DenseIdentity(_n) - K * H) * _P;
    }

    private Matrix<double> NumericalJacobian(Func<Vector<double>, Vector<double>> f, Vector<double> x)
    {
        double eps = 1e-7;
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

    public Vector<double> State => _x;
    public Matrix<double> Covariance => _P;
}

/// <summary>
/// Schmidt-Kalman Filter (Consider Filter).
/// Accounts for uncertainties in parameters without estimating them.
/// </summary>
public class SchmidtKalmanFilter
{
    private readonly int _n; // State size
    private readonly int _p; // Parameter size (considered, not estimated)
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _H;
    private readonly Matrix<double> _Ac; // State-parameter coupling
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    private readonly Matrix<double> _Qc; // Parameter uncertainty
    
    private Vector<double> _x;
    private Matrix<double> _P;
    private Matrix<double> _Pxc; // Cross-covariance

    public SchmidtKalmanFilter(
        Matrix<double> A, Matrix<double> B, Matrix<double> H,
        Matrix<double> Ac,
        Matrix<double> Q, Matrix<double> R,
        Matrix<double> parameterUncertainty)
    {
        _A = A;
        _B = B;
        _H = H;
        _Ac = Ac;
        _Q = Q;
        _R = R;
        _Qc = parameterUncertainty;
        
        _n = A.RowCount;
        _p = Ac.ColumnCount;
        
        _x = Vector<double>.Build.Dense(_n);
        _P = Matrix<double>.Build.DenseIdentity(_n);
        _Pxc = Matrix<double>.Build.Dense(_n, _p);
    }

    public void Initialize(Vector<double> state, Matrix<double> covariance)
    {
        _x = state.Clone();
        _P = covariance.Clone();
    }

    public void Predict(Vector<double> u)
    {
        // State prediction
        _x = _A * _x + _B * u;
        
        // Covariance prediction with parameter uncertainty
        _P = _A * _P * _A.Transpose() + _Q + 
             _Ac * _Qc * _Ac.Transpose();
        
        // Cross-covariance
        _Pxc = _A * _Pxc + _Ac * _Qc;
    }

    public void Update(Vector<double> z)
    {
        // Innovation
        var y = z - _H * _x;
        
        // Innovation covariance (includes parameter uncertainty effect)
        var S = _H * _P * _H.Transpose() + _R;
        
        // Kalman gain
        var K = _P * _H.Transpose() * S.Inverse();
        
        // State update
        _x = _x + K * y;
        
        // Covariance update (Joseph form for stability)
        var I_KH = Matrix<double>.Build.DenseIdentity(_n) - K * _H;
        _P = I_KH * _P * I_KH.Transpose() + K * _R * K.Transpose();
        
        // Update cross-covariance
        _Pxc = I_KH * _Pxc;
    }

    public Vector<double> State => _x;
    public Matrix<double> Covariance => _P;
}

/// <summary>
/// Rao-Blackwellized Particle Filter.
/// Combines particle filter with Kalman filter for efficiency.
/// </summary>
public class RaoBlackwellizedParticleFilter
{
    private readonly int _numParticles;
    private readonly int _nonlinearStateSize;
    private readonly int _linearStateSize;
    private readonly Random _rng;
    
    private Vector<double>[] _nonlinearParticles;
    private Vector<double>[] _linearMeans;
    private Matrix<double>[] _linearCovariances;
    private double[] _weights;
    
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _nonlinearDynamics;
    private readonly Matrix<double> _A; // Linear dynamics
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _H;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    private readonly double[] _nonlinearNoise;

    public RaoBlackwellizedParticleFilter(
        int numParticles,
        int nonlinearStateSize,
        int linearStateSize,
        Func<Vector<double>, Vector<double>, Vector<double>> nonlinearDynamics,
        Matrix<double> linearA, Matrix<double> linearB, Matrix<double> linearH,
        Matrix<double> linearQ, Matrix<double> linearR,
        double[] nonlinearProcessNoise)
    {
        _numParticles = numParticles;
        _nonlinearStateSize = nonlinearStateSize;
        _linearStateSize = linearStateSize;
        _nonlinearDynamics = nonlinearDynamics;
        _A = linearA;
        _B = linearB;
        _H = linearH;
        _Q = linearQ;
        _R = linearR;
        _nonlinearNoise = nonlinearProcessNoise;
        _rng = new Random();
        
        _nonlinearParticles = new Vector<double>[numParticles];
        _linearMeans = new Vector<double>[numParticles];
        _linearCovariances = new Matrix<double>[numParticles];
        _weights = new double[numParticles];
        
        for (int i = 0; i < numParticles; i++)
        {
            _nonlinearParticles[i] = Vector<double>.Build.Dense(nonlinearStateSize);
            _linearMeans[i] = Vector<double>.Build.Dense(linearStateSize);
            _linearCovariances[i] = Matrix<double>.Build.DenseIdentity(linearStateSize);
            _weights[i] = 1.0 / numParticles;
        }
    }

    public void Predict(Vector<double> u)
    {
        for (int i = 0; i < _numParticles; i++)
        {
            // Propagate nonlinear states with noise
            _nonlinearParticles[i] = _nonlinearDynamics(_nonlinearParticles[i], u);
            for (int j = 0; j < _nonlinearStateSize; j++)
            {
                _nonlinearParticles[i][j] += SampleGaussian() * _nonlinearNoise[j];
            }
            
            // Kalman prediction for linear states
            _linearMeans[i] = _A * _linearMeans[i] + _B * u;
            _linearCovariances[i] = _A * _linearCovariances[i] * _A.Transpose() + _Q;
        }
    }

    public void Update(Vector<double> z)
    {
        double sumWeights = 0;
        
        for (int i = 0; i < _numParticles; i++)
        {
            // Kalman update for linear states
            var zPred = _H * _linearMeans[i];
            var S = _H * _linearCovariances[i] * _H.Transpose() + _R;
            var K = _linearCovariances[i] * _H.Transpose() * S.Inverse();
            
            var innovation = z - zPred;
            _linearMeans[i] = _linearMeans[i] + K * innovation;
            _linearCovariances[i] = (Matrix<double>.Build.DenseIdentity(_linearStateSize) - K * _H) * _linearCovariances[i];
            
            // Weight based on innovation likelihood
            double logLikelihood = -0.5 * innovation.DotProduct(S.Inverse() * innovation);
            _weights[i] *= System.Math.Exp(logLikelihood);
            sumWeights += _weights[i];
        }
        
        // Normalize weights
        for (int i = 0; i < _numParticles; i++)
        {
            _weights[i] /= sumWeights;
        }
        
        // Resample if needed
        double nEff = 1.0 / _weights.Sum(w => w * w);
        if (nEff < _numParticles / 2)
        {
            Resample();
        }
    }

    private void Resample()
    {
        var cumSum = new double[_numParticles];
        cumSum[0] = _weights[0];
        for (int i = 1; i < _numParticles; i++)
            cumSum[i] = cumSum[i - 1] + _weights[i];
        
        var newNonlinear = new Vector<double>[_numParticles];
        var newMeans = new Vector<double>[_numParticles];
        var newCovs = new Matrix<double>[_numParticles];
        
        double step = 1.0 / _numParticles;
        double u = _rng.NextDouble() * step;
        int j = 0;
        
        for (int i = 0; i < _numParticles; i++)
        {
            while (j < _numParticles - 1 && u > cumSum[j]) j++;
            newNonlinear[i] = _nonlinearParticles[j].Clone();
            newMeans[i] = _linearMeans[j].Clone();
            newCovs[i] = _linearCovariances[j].Clone();
            u += step;
        }
        
        _nonlinearParticles = newNonlinear;
        _linearMeans = newMeans;
        _linearCovariances = newCovs;
        
        for (int i = 0; i < _numParticles; i++)
            _weights[i] = 1.0 / _numParticles;
    }

    public (Vector<double> nonlinear, Vector<double> linear) GetEstimate()
    {
        var nonlinearEst = Vector<double>.Build.Dense(_nonlinearStateSize);
        var linearEst = Vector<double>.Build.Dense(_linearStateSize);
        
        for (int i = 0; i < _numParticles; i++)
        {
            nonlinearEst += _weights[i] * _nonlinearParticles[i];
            linearEst += _weights[i] * _linearMeans[i];
        }
        
        return (nonlinearEst, linearEst);
    }

    private double SampleGaussian()
    {
        double u1 = 1.0 - _rng.NextDouble();
        double u2 = 1.0 - _rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

/// <summary>
/// Ensemble Kalman Filter (EnKF).
/// Uses ensemble of states for covariance estimation.
/// </summary>
public class EnsembleKalmanFilter
{
    private readonly int _ensembleSize;
    private readonly int _stateSize;
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Vector<double>> _h;
    private readonly double[] _processNoise;
    private readonly Matrix<double> _R;
    private readonly Random _rng;
    
    private Vector<double>[] _ensemble;

    public EnsembleKalmanFilter(
        int ensembleSize,
        int stateSize,
        Func<Vector<double>, Vector<double>, Vector<double>> stateTransition,
        Func<Vector<double>, Vector<double>> measurementFunction,
        double[] processNoise,
        Matrix<double> measurementNoise)
    {
        _ensembleSize = ensembleSize;
        _stateSize = stateSize;
        _f = stateTransition;
        _h = measurementFunction;
        _processNoise = processNoise;
        _R = measurementNoise;
        _rng = new Random();
        
        _ensemble = new Vector<double>[_ensembleSize];
        for (int i = 0; i < ensembleSize; i++)
        {
            _ensemble[i] = Vector<double>.Build.Dense(stateSize);
        }
    }

    public void Initialize(Vector<double> mean, Matrix<double> covariance)
    {
        var sqrtP = covariance.Cholesky().Factor;
        
        for (int i = 0; i < _ensembleSize; i++)
        {
            var perturbation = Vector<double>.Build.Dense(_stateSize);
            for (int j = 0; j < _stateSize; j++)
            {
                perturbation[j] = SampleGaussian();
            }
            _ensemble[i] = mean + sqrtP * perturbation;
        }
    }

    public void Predict(Vector<double> u)
    {
        for (int i = 0; i < _ensembleSize; i++)
        {
            _ensemble[i] = _f(_ensemble[i], u);
            
            // Add process noise
            for (int j = 0; j < _stateSize; j++)
            {
                _ensemble[i][j] += SampleGaussian() * _processNoise[j];
            }
        }
    }

    public void Update(Vector<double> z)
    {
        int m = z.Count;
        
        // Compute ensemble mean
        var xMean = Vector<double>.Build.Dense(_stateSize);
        for (int i = 0; i < _ensembleSize; i++)
        {
            xMean += _ensemble[i];
        }
        xMean /= _ensembleSize;
        
        // Compute measurement predictions
        var hEnsemble = new Vector<double>[_ensembleSize];
        var hMean = Vector<double>.Build.Dense(m);
        for (int i = 0; i < _ensembleSize; i++)
        {
            hEnsemble[i] = _h(_ensemble[i]);
            hMean += hEnsemble[i];
        }
        hMean /= _ensembleSize;
        
        // Compute anomalies
        var X = Matrix<double>.Build.Dense(_stateSize, _ensembleSize);
        var Y = Matrix<double>.Build.Dense(m, _ensembleSize);
        for (int i = 0; i < _ensembleSize; i++)
        {
            X.SetColumn(i, _ensemble[i] - xMean);
            Y.SetColumn(i, hEnsemble[i] - hMean);
        }
        
        // Sample covariances
        var Pxy = X * Y.Transpose() / (_ensembleSize - 1);
        var Pyy = Y * Y.Transpose() / (_ensembleSize - 1) + _R;
        
        // Kalman gain
        var K = Pxy * Pyy.Inverse();
        
        // Update each ensemble member
        for (int i = 0; i < _ensembleSize; i++)
        {
            // Perturbed observation
            var zPerturbed = z.Clone();
            var sqrtR = _R.Cholesky().Factor;
            var noise = Vector<double>.Build.Dense(m);
            for (int j = 0; j < m; j++)
            {
                noise[j] = SampleGaussian();
            }
            zPerturbed += sqrtR * noise;
            
            // Update
            _ensemble[i] = _ensemble[i] + K * (zPerturbed - hEnsemble[i]);
        }
    }

    public Vector<double> GetMean()
    {
        var mean = Vector<double>.Build.Dense(_stateSize);
        for (int i = 0; i < _ensembleSize; i++)
        {
            mean += _ensemble[i];
        }
        return mean / _ensembleSize;
    }

    public Matrix<double> GetCovariance()
    {
        var mean = GetMean();
        var cov = Matrix<double>.Build.Dense(_stateSize, _stateSize);
        
        for (int i = 0; i < _ensembleSize; i++)
        {
            var diff = _ensemble[i] - mean;
            cov += diff.OuterProduct(diff);
        }
        
        return cov / (_ensembleSize - 1);
    }

    private double SampleGaussian()
    {
        double u1 = 1.0 - _rng.NextDouble();
        double u2 = 1.0 - _rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}
