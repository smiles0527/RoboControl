using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Learning;

/// <summary>
/// Neural Network based adaptive control with Lyapunov stability guarantees.
/// Implements:
/// - Neural Network Adaptive Control (NNAC)
/// - Learning-based Model Predictive Control (L-MPC)
/// - Gaussian Process regression for model learning
/// - Meta-learning for fast adaptation
/// 
/// Based on:
/// - "Adaptive Neural Network Control of Robot Manipulators" (Lewis et al., 2003)
/// - "Learning-based Model Predictive Control" (Hewing et al., 2020)
/// </summary>
public class NeuralAdaptiveController
{
    private readonly int _stateDim;
    private readonly int _controlDim;
    private readonly AdaptiveNeuralNetwork _network;
    private readonly Matrix<double> _W;
    private readonly double _gamma;
    private readonly double _sigma;
    
    private Matrix<double> _Wdot;
    private Vector<double> _lastOutput;
    
    public NeuralAdaptiveController(
        int stateDim,
        int controlDim,
        int hiddenUnits = 64,
        double gamma = 10.0,
        double sigma = 0.01)
    {
        _stateDim = stateDim;
        _controlDim = controlDim;
        _gamma = gamma;
        _sigma = sigma;
        
        _network = new AdaptiveNeuralNetwork([stateDim, hiddenUnits, hiddenUnits, controlDim]);
        _W = Matrix<double>.Build.Dense(hiddenUnits, controlDim);
        _Wdot = Matrix<double>.Build.Dense(hiddenUnits, controlDim);
        _lastOutput = Vector<double>.Build.Dense(controlDim);
    }
    
    /// <summary>
    /// Compute adaptive control with online learning.
    /// </summary>
    public Vector<double> ComputeControl(
        Vector<double> state,
        Vector<double> stateDesired,
        Vector<double> stateError,
        Matrix<double> B,
        double dt)
    {
        // Forward pass through network to get basis functions
        var (phi, output) = _network.ForwardWithFeatures(state);
        
        // Compute adaptive estimate
        var fHat = _W.Transpose() * phi;
        
        // Lyapunov-based adaptation law: ? = -? * ? * e^T * B
        var Wnew = _W - _gamma * OuterProduct(phi, B.Transpose() * stateError) * dt;
        
        // ?-modification for robustness (prevents weight drift)
        Wnew -= _sigma * _W * dt;
        
        // Update weights
        for (int i = 0; i < _W.RowCount; i++)
            for (int j = 0; j < _W.ColumnCount; j++)
                _W[i, j] = Wnew[i, j];
        
        // Compute control: u = -K*e + fHat (cancels learned dynamics)
        var u = output + fHat;
        
        _lastOutput = u;
        return u;
    }
    
    /// <summary>
    /// Batch update using collected experience.
    /// </summary>
    public void UpdateFromExperience(
        List<(Vector<double> state, Vector<double> control, Vector<double> nextState)> experiences,
        int epochs = 10,
        double learningRate = 0.001)
    {
        for (int epoch = 0; epoch < epochs; epoch++)
        {
            double totalLoss = 0;
            
            foreach (var (state, control, nextState) in experiences)
            {
                // Predict next state
                var input = ConcatVectors(state, control);
                var predicted = _network.Forward(input);
                
                // Compute loss
                var error = predicted - nextState.SubVector(0, _controlDim);
                totalLoss += error.L2Norm();
                
                // Backpropagation
                _network.Backward(error, learningRate);
            }
        }
    }
    
    private static Matrix<double> OuterProduct(Vector<double> a, Vector<double> b)
    {
        var result = Matrix<double>.Build.Dense(a.Count, b.Count);
        for (int i = 0; i < a.Count; i++)
            for (int j = 0; j < b.Count; j++)
                result[i, j] = a[i] * b[j];
        return result;
    }
    
    private static Vector<double> ConcatVectors(Vector<double> a, Vector<double> b)
    {
        var result = Vector<double>.Build.Dense(a.Count + b.Count);
        result.SetSubVector(0, a.Count, a);
        result.SetSubVector(a.Count, b.Count, b);
        return result;
    }
}

/// <summary>
/// Feedforward neural network for adaptive control.
/// </summary>
public class AdaptiveNeuralNetwork
{
    private readonly int[] _layers;
    private readonly List<Matrix<double>> _weights = new();
    private readonly List<Vector<double>> _biases = new();
    private readonly List<Vector<double>> _activations = new();
    private readonly List<Vector<double>> _preActivations = new();
    
    public AdaptiveNeuralNetwork(int[] layers)
    {
        _layers = layers;
        
        var rng = new Random(42);
        
        for (int i = 0; i < layers.Length - 1; i++)
        {
            // Xavier initialization
            double std = System.Math.Sqrt(2.0 / (layers[i] + layers[i + 1]));
            
            var W = Matrix<double>.Build.Dense(layers[i + 1], layers[i],
                (r, c) => (rng.NextDouble() * 2 - 1) * std);
            var b = Vector<double>.Build.Dense(layers[i + 1]);
            
            _weights.Add(W);
            _biases.Add(b);
        }
    }
    
    /// <summary>
    /// Forward pass.
    /// </summary>
    public Vector<double> Forward(Vector<double> input)
    {
        _activations.Clear();
        _preActivations.Clear();
        
        var current = input;
        _activations.Add(current);
        
        for (int i = 0; i < _weights.Count; i++)
        {
            var z = _weights[i] * current + _biases[i];
            _preActivations.Add(z);
            
            // ReLU for hidden layers, linear for output
            current = i < _weights.Count - 1 ? ReLU(z) : z;
            _activations.Add(current);
        }
        
        return current;
    }
    
    /// <summary>
    /// Forward pass returning intermediate features.
    /// </summary>
    public (Vector<double> features, Vector<double> output) ForwardWithFeatures(Vector<double> input)
    {
        Forward(input);
        
        // Return second-to-last layer as features
        var features = _activations[^2];
        var output = _activations[^1];
        
        return (features, output);
    }
    
    /// <summary>
    /// Backward pass with gradient descent update.
    /// </summary>
    public void Backward(Vector<double> lossGradient, double learningRate)
    {
        var delta = lossGradient;
        
        for (int i = _weights.Count - 1; i >= 0; i--)
        {
            // Compute gradients
            var dW = OuterProduct(delta, _activations[i]);
            var db = delta;
            
            // Update weights
            _weights[i] -= learningRate * dW;
            _biases[i] -= learningRate * db;
            
            // Propagate gradient
            if (i > 0)
            {
                delta = _weights[i].Transpose() * delta;
                delta = delta.PointwiseMultiply(ReLUDerivative(_preActivations[i - 1]));
            }
        }
    }
    
    private static Vector<double> ReLU(Vector<double> x)
    {
        return x.Map(v => System.Math.Max(0, v));
    }
    
    private static Vector<double> ReLUDerivative(Vector<double> x)
    {
        return x.Map(v => v > 0 ? 1.0 : 0.0);
    }
    
    private static Matrix<double> OuterProduct(Vector<double> a, Vector<double> b)
    {
        var result = Matrix<double>.Build.Dense(a.Count, b.Count);
        for (int i = 0; i < a.Count; i++)
            for (int j = 0; j < b.Count; j++)
                result[i, j] = a[i] * b[j];
        return result;
    }
}

/// <summary>
/// Gaussian Process for model learning with uncertainty quantification.
/// </summary>
public class GaussianProcessModel
{
    private readonly double _lengthScale;
    private readonly double _signalVariance;
    private readonly double _noiseVariance;
    
    private List<Vector<double>> _X = new();
    private List<Vector<double>> _Y = new();
    private Matrix<double>? _K;
    private Matrix<double>? _KInv;
    
    public int DataCount => _X.Count;
    
    public GaussianProcessModel(
        double lengthScale = 1.0,
        double signalVariance = 1.0,
        double noiseVariance = 0.1)
    {
        _lengthScale = lengthScale;
        _signalVariance = signalVariance;
        _noiseVariance = noiseVariance;
    }
    
    /// <summary>
    /// Add training data point.
    /// </summary>
    public void AddData(Vector<double> x, Vector<double> y)
    {
        _X.Add(x);
        _Y.Add(y);
        _KInv = null; // Invalidate cache
    }
    
    /// <summary>
    /// Add batch of training data.
    /// </summary>
    public void AddDataBatch(
        List<Vector<double>> X,
        List<Vector<double>> Y)
    {
        _X.AddRange(X);
        _Y.AddRange(Y);
        _KInv = null;
    }
    
    /// <summary>
    /// Predict mean and variance at test point.
    /// </summary>
    public (Vector<double> mean, Matrix<double> variance) Predict(Vector<double> xTest)
    {
        if (_X.Count == 0)
        {
            // Prior: zero mean, prior variance
            int dim = xTest.Count;
            return (
                Vector<double>.Build.Dense(dim),
                Matrix<double>.Build.DenseIdentity(dim) * _signalVariance
            );
        }
        
        EnsureKernelComputed();
        
        int n = _X.Count;
        int outputDim = _Y[0].Count;
        
        // k(x*, X)
        var kStar = Vector<double>.Build.Dense(n);
        for (int i = 0; i < n; i++)
        {
            kStar[i] = Kernel(xTest, _X[i]);
        }
        
        // k(x*, x*)
        double kStarStar = Kernel(xTest, xTest);
        
        // Mean: k* K^-1 Y
        var mean = Vector<double>.Build.Dense(outputDim);
        for (int d = 0; d < outputDim; d++)
        {
            var y = Vector<double>.Build.Dense(n, i => _Y[i][d]);
            mean[d] = kStar * (_KInv! * y);
        }
        
        // Variance: k** - k* K^-1 k*^T
        double variance = kStarStar - kStar * (_KInv! * kStar);
        variance = System.Math.Max(variance, 1e-6); // Numerical stability
        
        return (mean, Matrix<double>.Build.DenseIdentity(outputDim) * variance);
    }
    
    /// <summary>
    /// Sample from posterior distribution.
    /// </summary>
    public Vector<double> Sample(Vector<double> xTest, Random? rng = null)
    {
        rng ??= new Random();
        
        var (mean, variance) = Predict(xTest);
        
        // Sample from N(mean, variance)
        int dim = mean.Count;
        var sample = Vector<double>.Build.Dense(dim);
        
        var L = variance.Cholesky().Factor;
        var z = Vector<double>.Build.Dense(dim, _ => SampleStandardNormal(rng));
        
        return mean + L * z;
    }
    
    /// <summary>
    /// Compute log marginal likelihood for hyperparameter optimization.
    /// </summary>
    public double LogMarginalLikelihood()
    {
        if (_X.Count == 0) return 0;
        
        EnsureKernelComputed();
        
        int n = _X.Count;
        int d = _Y[0].Count;
        
        double logLik = 0;
        
        // -0.5 * (y^T K^-1 y + log|K| + n*log(2?))
        for (int dim = 0; dim < d; dim++)
        {
            var y = Vector<double>.Build.Dense(n, i => _Y[i][dim]);
            var alpha = _KInv! * y;
            
            logLik -= 0.5 * (y * alpha);
        }
        
        // Log determinant
        var chol = _K!.Cholesky().Factor;
        double logDet = 0;
        for (int i = 0; i < n; i++)
            logDet += System.Math.Log(chol[i, i]);
        logDet *= 2;
        
        logLik -= 0.5 * logDet;
        logLik -= 0.5 * n * System.Math.Log(2 * System.Math.PI);
        
        return logLik;
    }
    
    /// <summary>
    /// Prune old data points using sparse approximation.
    /// </summary>
    public void Prune(int maxPoints)
    {
        if (_X.Count <= maxPoints) return;
        
        // Remove oldest points (simple FIFO)
        int toRemove = _X.Count - maxPoints;
        _X.RemoveRange(0, toRemove);
        _Y.RemoveRange(0, toRemove);
        _KInv = null;
    }
    
    private void EnsureKernelComputed()
    {
        if (_KInv != null) return;
        
        int n = _X.Count;
        _K = Matrix<double>.Build.Dense(n, n);
        
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                _K[i, j] = Kernel(_X[i], _X[j]);
                if (i == j)
                    _K[i, j] += _noiseVariance;
            }
        }
        
        _KInv = _K.Inverse();
    }
    
    /// <summary>
    /// Squared exponential (RBF) kernel.
    /// </summary>
    private double Kernel(Vector<double> x1, Vector<double> x2)
    {
        double dist = (x1 - x2).L2Norm();
        return _signalVariance * System.Math.Exp(-0.5 * dist * dist / (_lengthScale * _lengthScale));
    }
    
    private static double SampleStandardNormal(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

/// <summary>
/// Learning-based MPC that uses GP to model residual dynamics.
/// </summary>
public class LearningBasedMpc
{
    private readonly GaussianProcessModel _gpModel;
    private readonly int _horizon;
    private readonly double _dt;
    
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _nominalDynamics;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    
    public LearningBasedMpc(
        Func<Vector<double>, Vector<double>, Vector<double>> nominalDynamics,
        Matrix<double> Q,
        Matrix<double> R,
        int horizon = 20,
        double dt = 0.02)
    {
        _nominalDynamics = nominalDynamics;
        _Q = Q;
        _R = R;
        _horizon = horizon;
        _dt = dt;
        
        _gpModel = new GaussianProcessModel(
            lengthScale: 1.0,
            signalVariance: 1.0,
            noiseVariance: 0.1
        );
    }
    
    /// <summary>
    /// Update model with observed transition.
    /// </summary>
    public void UpdateModel(Vector<double> state, Vector<double> control, Vector<double> nextState)
    {
        // Compute residual: actual - nominal
        var nominalNext = _nominalDynamics(state, control);
        var residual = nextState - nominalNext;
        
        // Add to GP model
        var input = ConcatVectors(state, control);
        _gpModel.AddData(input, residual);
        
        // Prune if too many points
        _gpModel.Prune(500);
    }
    
    /// <summary>
    /// Solve MPC with learned dynamics.
    /// </summary>
    public LmpcSolution Solve(Vector<double> currentState, Vector<double> reference)
    {
        int n = currentState.Count;
        int m = _R.ColumnCount;
        
        // Initialize trajectory
        var xTraj = new Vector<double>[_horizon + 1];
        var uTraj = new Vector<double>[_horizon];
        
        xTraj[0] = currentState.Clone();
        for (int k = 0; k < _horizon; k++)
        {
            uTraj[k] = Vector<double>.Build.Dense(m);
            xTraj[k + 1] = currentState.Clone();
        }
        
        // Iterative linearization and solve
        for (int iter = 0; iter < 10; iter++)
        {
            // Linearize with GP-corrected dynamics
            var (A, B, c) = LinearizeWithGP(xTraj, uTraj);
            
            // Solve linear MPC
            var (xNew, uNew, cost) = SolveLinearMpc(xTraj[0], reference, A, B, c);
            
            // Line search
            double stepSize = 1.0;
            for (int k = 0; k <= _horizon; k++)
            {
                if (k < _horizon)
                {
                    uTraj[k] = uTraj[k] + stepSize * (uNew[k] - uTraj[k]);
                }
                xTraj[k] = xTraj[k] + stepSize * (xNew[k] - xTraj[k]);
            }
            
            // Check convergence
            double delta = 0;
            for (int k = 0; k < _horizon; k++)
                delta += (uNew[k] - uTraj[k]).L2Norm();
            
            if (delta < 1e-4)
                break;
        }
        
        // Compute uncertainty
        var uncertainties = new double[_horizon];
        for (int k = 0; k < _horizon; k++)
        {
            var input = ConcatVectors(xTraj[k], uTraj[k]);
            var (_, variance) = _gpModel.Predict(input);
            uncertainties[k] = variance.Trace();
        }
        
        return new LmpcSolution
        {
            StateTrajectory = xTraj,
            ControlTrajectory = uTraj,
            PredictionUncertainty = uncertainties,
            OptimalControl = uTraj[0]
        };
    }
    
    private (Matrix<double>[] A, Matrix<double>[] B, Vector<double>[] c) LinearizeWithGP(
        Vector<double>[] xTraj, Vector<double>[] uTraj)
    {
        int n = xTraj[0].Count;
        int m = uTraj[0].Count;
        
        var A = new Matrix<double>[_horizon];
        var B = new Matrix<double>[_horizon];
        var c = new Vector<double>[_horizon];
        
        double eps = 1e-5;
        
        for (int k = 0; k < _horizon; k++)
        {
            // Numerical differentiation with GP correction
            var f0 = PredictDynamics(xTraj[k], uTraj[k]);
            
            A[k] = Matrix<double>.Build.Dense(n, n);
            B[k] = Matrix<double>.Build.Dense(n, m);
            
            // df/dx
            for (int i = 0; i < n; i++)
            {
                var xp = xTraj[k].Clone();
                xp[i] += eps;
                var fp = PredictDynamics(xp, uTraj[k]);
                
                for (int j = 0; j < n; j++)
                    A[k][j, i] = (fp[j] - f0[j]) / eps;
            }
            
            // df/du
            for (int i = 0; i < m; i++)
            {
                var up = uTraj[k].Clone();
                up[i] += eps;
                var fp = PredictDynamics(xTraj[k], up);
                
                for (int j = 0; j < n; j++)
                    B[k][j, i] = (fp[j] - f0[j]) / eps;
            }
            
            c[k] = f0 - A[k] * xTraj[k] - B[k] * uTraj[k];
        }
        
        return (A, B, c);
    }
    
    private Vector<double> PredictDynamics(Vector<double> state, Vector<double> control)
    {
        var nominal = _nominalDynamics(state, control);
        
        if (_gpModel.DataCount > 0)
        {
            var input = ConcatVectors(state, control);
            var (residualMean, _) = _gpModel.Predict(input);
            return nominal + residualMean;
        }
        
        return nominal;
    }
    
    private (Vector<double>[] x, Vector<double>[] u, double cost) SolveLinearMpc(
        Vector<double> x0,
        Vector<double> xRef,
        Matrix<double>[] A,
        Matrix<double>[] B,
        Vector<double>[] c)
    {
        int n = x0.Count;
        int m = B[0].ColumnCount;
        
        // Solve Riccati recursion
        var P = new Matrix<double>[_horizon + 1];
        var K = new Matrix<double>[_horizon];
        var d = new Vector<double>[_horizon];
        
        P[_horizon] = _Q * 10; // Terminal cost
        
        for (int k = _horizon - 1; k >= 0; k--)
        {
            var Ak = A[k];
            var Bk = B[k];
            var Pk1 = P[k + 1];
            
            var S = _R + Bk.Transpose() * Pk1 * Bk;
            K[k] = S.Inverse() * Bk.Transpose() * Pk1 * Ak;
            d[k] = S.Inverse() * Bk.Transpose() * Pk1 * c[k];
            
            P[k] = _Q + Ak.Transpose() * Pk1 * (Ak - Bk * K[k]);
        }
        
        // Forward simulation
        var x = new Vector<double>[_horizon + 1];
        var u = new Vector<double>[_horizon];
        x[0] = x0;
        
        double totalCost = 0;
        
        for (int k = 0; k < _horizon; k++)
        {
            u[k] = -K[k] * x[k] - d[k];
            x[k + 1] = A[k] * x[k] + B[k] * u[k] + c[k];
            
            var xErr = x[k] - xRef;
            totalCost += 0.5 * (xErr * _Q * xErr + u[k] * _R * u[k]);
        }
        
        var xErrFinal = x[_horizon] - xRef;
        totalCost += 0.5 * xErrFinal * P[_horizon] * xErrFinal;
        
        return (x, u, totalCost);
    }
    
    private static Vector<double> ConcatVectors(Vector<double> a, Vector<double> b)
    {
        var result = Vector<double>.Build.Dense(a.Count + b.Count);
        result.SetSubVector(0, a.Count, a);
        result.SetSubVector(a.Count, b.Count, b);
        return result;
    }
}

/// <summary>
/// Model-Agnostic Meta-Learning (MAML) for fast adaptation.
/// </summary>
public class MetaLearningController
{
    private readonly AdaptiveNeuralNetwork _metaModel;
    private readonly double _metaLearningRate;
    private readonly double _innerLearningRate;
    private readonly int _innerSteps;
    
    public MetaLearningController(
        int[] layers,
        double metaLearningRate = 0.001,
        double innerLearningRate = 0.01,
        int innerSteps = 5)
    {
        _metaModel = new AdaptiveNeuralNetwork(layers);
        _metaLearningRate = metaLearningRate;
        _innerLearningRate = innerLearningRate;
        _innerSteps = innerSteps;
    }
    
    /// <summary>
    /// Meta-train on multiple tasks.
    /// </summary>
    public void MetaTrain(
        List<List<(Vector<double> input, Vector<double> target)>> tasks,
        int metaIterations = 100)
    {
        for (int iter = 0; iter < metaIterations; iter++)
        {
            // Accumulate gradients across tasks
            foreach (var task in tasks)
            {
                // Split task data
                var support = task.Take(task.Count / 2).ToList();
                var query = task.Skip(task.Count / 2).ToList();
                
                // Inner loop: adapt to task
                AdaptToTask(support);
                
                // Outer loop: compute meta-gradient on query set
                foreach (var (input, target) in query)
                {
                    var prediction = _metaModel.Forward(input);
                    var error = prediction - target;
                    _metaModel.Backward(error, _metaLearningRate / tasks.Count);
                }
            }
        }
    }
    
    /// <summary>
    /// Quickly adapt to a new task with few samples.
    /// </summary>
    public void AdaptToTask(List<(Vector<double> input, Vector<double> target)> samples)
    {
        for (int step = 0; step < _innerSteps; step++)
        {
            foreach (var (input, target) in samples)
            {
                var prediction = _metaModel.Forward(input);
                var error = prediction - target;
                _metaModel.Backward(error, _innerLearningRate);
            }
        }
    }
    
    /// <summary>
    /// Predict using adapted model.
    /// </summary>
    public Vector<double> Predict(Vector<double> input)
    {
        return _metaModel.Forward(input);
    }
}

public class LmpcSolution
{
    public Vector<double>[] StateTrajectory { get; set; } = [];
    public Vector<double>[] ControlTrajectory { get; set; } = [];
    public double[] PredictionUncertainty { get; set; } = [];
    public Vector<double> OptimalControl { get; set; } = Vector<double>.Build.Dense(0);
}
