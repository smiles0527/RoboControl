using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Control;

/// <summary>
/// Model Predictive Controller (MPC) implementation.
/// Solves constrained optimal control problems over a receding horizon.
/// </summary>
public class ModelPredictiveController
{
    private readonly int _horizonLength;
    private readonly int _stateSize;
    private readonly int _inputSize;
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    private readonly Matrix<double> _Qf;
    
    private Vector<double>? _uMin;
    private Vector<double>? _uMax;
    private Vector<double>? _xMin;
    private Vector<double>? _xMax;
    private Vector<double>? _duMax;

    /// <summary>
    /// Creates a new MPC controller.
    /// </summary>
    /// <param name="A">State transition matrix</param>
    /// <param name="B">Input matrix</param>
    /// <param name="Q">State cost matrix</param>
    /// <param name="R">Input cost matrix</param>
    /// <param name="horizonLength">Prediction horizon length</param>
    /// <param name="Qf">Terminal state cost (optional, defaults to Q)</param>
    public ModelPredictiveController(
        Matrix<double> A, 
        Matrix<double> B, 
        Matrix<double> Q, 
        Matrix<double> R, 
        int horizonLength,
        Matrix<double>? Qf = null)
    {
        _A = A;
        _B = B;
        _Q = Q;
        _R = R;
        _horizonLength = horizonLength;
        _stateSize = A.RowCount;
        _inputSize = B.ColumnCount;
        _Qf = Qf ?? Q;
    }

    /// <summary>
    /// Sets input constraints.
    /// </summary>
    public void SetInputConstraints(Vector<double> uMin, Vector<double> uMax)
    {
        _uMin = uMin;
        _uMax = uMax;
    }

    /// <summary>
    /// Sets state constraints.
    /// </summary>
    public void SetStateConstraints(Vector<double> xMin, Vector<double> xMax)
    {
        _xMin = xMin;
        _xMax = xMax;
    }

    /// <summary>
    /// Sets input rate constraints (delta u).
    /// </summary>
    public void SetInputRateConstraints(Vector<double> duMax)
    {
        _duMax = duMax;
    }

    /// <summary>
    /// Computes the optimal control sequence.
    /// </summary>
    /// <param name="x0">Current state</param>
    /// <param name="xRef">Reference trajectory (can be single point or full horizon)</param>
    /// <param name="uPrev">Previous control input (for rate constraints)</param>
    /// <returns>Optimal control sequence</returns>
    public Vector<double>[] Solve(Vector<double> x0, Vector<double>[] xRef, Vector<double>? uPrev = null)
    {
        // Build prediction matrices
        var (Sx, Su) = BuildPredictionMatrices();
        
        // Build QP matrices
        var H = BuildHessian(Su);
        var f = BuildGradient(x0, xRef, Sx, Su);
        
        // Solve QP (simplified - unconstrained solution)
        var u = SolveUnconstrainedQP(H, f);
        
        // Apply constraints via clamping (simplified approach)
        u = ApplyConstraints(u, uPrev);
        
        // Convert to array of vectors
        return ExtractControlSequence(u);
    }

    /// <summary>
    /// Computes the first optimal control input (receding horizon).
    /// </summary>
    public Vector<double> ComputeControl(Vector<double> x0, Vector<double> xRef, Vector<double>? uPrev = null)
    {
        var xRefArray = new Vector<double>[_horizonLength];
        for (int i = 0; i < _horizonLength; i++)
            xRefArray[i] = xRef;
            
        var uSequence = Solve(x0, xRefArray, uPrev);
        return uSequence[0];
    }

    private (Matrix<double> Sx, Matrix<double> Su) BuildPredictionMatrices()
    {
        int n = _stateSize;
        int m = _inputSize;
        int N = _horizonLength;
        
        // Sx: maps x0 to predicted states
        var Sx = Matrix<double>.Build.Dense(n * N, n);
        var Ak = Matrix<double>.Build.DenseIdentity(n);
        
        for (int k = 0; k < N; k++)
        {
            Ak = _A * Ak;
            Sx.SetSubMatrix(k * n, 0, Ak);
        }
        
        // Su: maps control sequence to predicted states
        var Su = Matrix<double>.Build.Dense(n * N, m * N);
        
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                var power = i - j;
                var Apower = power == 0 
                    ? Matrix<double>.Build.DenseIdentity(n) 
                    : _A.Power(power);
                Su.SetSubMatrix(i * n, j * m, Apower * _B);
            }
        }
        
        return (Sx, Su);
    }

    private Matrix<double> BuildHessian(Matrix<double> Su)
    {
        int n = _stateSize;
        int m = _inputSize;
        int N = _horizonLength;
        
        // Build block diagonal Q matrix
        var Qbar = Matrix<double>.Build.Dense(n * N, n * N);
        for (int k = 0; k < N - 1; k++)
        {
            Qbar.SetSubMatrix(k * n, k * n, _Q);
        }
        Qbar.SetSubMatrix((N - 1) * n, (N - 1) * n, _Qf);
        
        // Build block diagonal R matrix
        var Rbar = Matrix<double>.Build.Dense(m * N, m * N);
        for (int k = 0; k < N; k++)
        {
            Rbar.SetSubMatrix(k * m, k * m, _R);
        }
        
        // H = Su' * Qbar * Su + Rbar
        return Su.Transpose() * Qbar * Su + Rbar;
    }

    private Vector<double> BuildGradient(Vector<double> x0, Vector<double>[] xRef, Matrix<double> Sx, Matrix<double> Su)
    {
        int n = _stateSize;
        int N = _horizonLength;
        
        // Build reference vector
        var xRefVec = Vector<double>.Build.Dense(n * N);
        for (int k = 0; k < N; k++)
        {
            var idx = System.Math.Min(k, xRef.Length - 1);
            xRefVec.SetSubVector(k * n, n, xRef[idx]);
        }
        
        // Build block diagonal Q matrix
        var Qbar = Matrix<double>.Build.Dense(n * N, n * N);
        for (int k = 0; k < N - 1; k++)
        {
            Qbar.SetSubMatrix(k * n, k * n, _Q);
        }
        Qbar.SetSubMatrix((N - 1) * n, (N - 1) * n, _Qf);
        
        // f = Su' * Qbar * (Sx * x0 - xRef)
        return Su.Transpose() * Qbar * (Sx * x0 - xRefVec);
    }

    private Vector<double> SolveUnconstrainedQP(Matrix<double> H, Vector<double> f)
    {
        // Unconstrained: u* = -H^-1 * f
        return -H.Solve(f);
    }

    private Vector<double> ApplyConstraints(Vector<double> u, Vector<double>? uPrev)
    {
        int m = _inputSize;
        int N = _horizonLength;
        
        var result = u.Clone();
        
        for (int k = 0; k < N; k++)
        {
            for (int i = 0; i < m; i++)
            {
                int idx = k * m + i;
                
                // Input magnitude constraints
                if (_uMin != null && _uMax != null)
                {
                    result[idx] = System.Math.Max(_uMin[i], System.Math.Min(_uMax[i], result[idx]));
                }
                
                // Input rate constraints (only if we have previous input)
                if (_duMax != null && (uPrev != null || k > 0))
                {
                    double prevVal = k == 0 && uPrev != null ? uPrev[i] : result[(k - 1) * m + i];
                    double du = result[idx] - prevVal;
                    if (System.Math.Abs(du) > _duMax[i])
                    {
                        result[idx] = prevVal + System.Math.Sign(du) * _duMax[i];
                    }
                }
            }
        }
        
        return result;
    }

    private Vector<double>[] ExtractControlSequence(Vector<double> u)
    {
        int m = _inputSize;
        var result = new Vector<double>[_horizonLength];
        
        for (int k = 0; k < _horizonLength; k++)
        {
            result[k] = u.SubVector(k * m, m);
        }
        
        return result;
    }
}

/// <summary>
/// Adaptive controller that adjusts gains based on system performance.
/// </summary>
public class AdaptiveController
{
    private readonly double _gamma; // Adaptation rate
    private Vector<double> _theta; // Estimated parameters
    private readonly int _order; // System order
    private readonly double _lambda; // Forgetting factor
    private Matrix<double> _P; // Covariance matrix

    public AdaptiveController(int order, double gamma = 0.1, double lambda = 0.99)
    {
        _order = order;
        _gamma = gamma;
        _lambda = lambda;
        _theta = Vector<double>.Build.Dense(order);
        _P = Matrix<double>.Build.DenseIdentity(order) * 1000; // Large initial covariance
    }

    /// <summary>
    /// Updates the parameter estimates using recursive least squares.
    /// </summary>
    /// <param name="phi">Regressor vector</param>
    /// <param name="y">Measured output</param>
    public void Update(Vector<double> phi, double y)
    {
        // Prediction error
        double yHat = phi * _theta;
        double e = y - yHat;
        
        // Kalman gain
        var Pphi = _P * phi;
        double denom = _lambda + phi * Pphi;
        var K = Pphi / denom;
        
        // Update estimates
        _theta = _theta + K * e;
        
        // Update covariance
        _P = (_P - K.OuterProduct(Pphi)) / _lambda;
    }

    /// <summary>
    /// Computes control input using current parameter estimates.
    /// </summary>
    public double ComputeControl(Vector<double> phi, double reference)
    {
        double yHat = phi * _theta;
        return _gamma * (reference - yHat);
    }

    public Vector<double> Parameters => _theta;
}

/// <summary>
/// Sliding Mode Controller for robust control.
/// </summary>
public class SlidingModeController
{
    private readonly double _lambda; // Sliding surface slope
    private readonly double _eta; // Reaching law gain
    private readonly double _phi; // Boundary layer thickness

    public SlidingModeController(double lambda, double eta, double phi = 0.1)
    {
        _lambda = lambda;
        _eta = eta;
        _phi = phi;
    }

    /// <summary>
    /// Computes control using sliding mode approach.
    /// </summary>
    /// <param name="error">Position error</param>
    /// <param name="errorDot">Velocity error</param>
    /// <param name="equivalent">Equivalent control (feedforward)</param>
    /// <returns>Control signal</returns>
    public double Compute(double error, double errorDot, double equivalent = 0)
    {
        // Sliding surface
        double s = errorDot + _lambda * error;
        
        // Switching function with boundary layer (continuous approximation)
        double sat = Saturation(s / _phi);
        
        // Control law: u = u_eq - eta * sat(s/phi)
        return equivalent - _eta * sat;
    }

    private static double Saturation(double x)
    {
        if (x > 1) return 1;
        if (x < -1) return -1;
        return x;
    }
}

/// <summary>
/// Backstepping controller for nonlinear systems.
/// </summary>
public class BacksteppingController
{
    private readonly double[] _k; // Gains for each step
    private readonly int _order; // System order

    public BacksteppingController(double[] gains)
    {
        _k = gains;
        _order = gains.Length;
    }

    /// <summary>
    /// Computes control for a second-order system.
    /// </summary>
    public double ComputeSecondOrder(double x1, double x1d, double x2, double x2d, double x1dDot, double x2dDot)
    {
        // Error coordinates
        double z1 = x1 - x1d;
        double z2 = x2 - x2d;
        
        // Virtual control
        double alpha1 = -_k[0] * z1 + x1dDot;
        double alphaDot = -_k[0] * (x2 - x1dDot); // Simplified derivative
        
        // Actual control
        return -_k[1] * z2 - z1 + alphaDot + x2dDot;
    }
}

/// <summary>
/// Fuzzy Logic Controller.
/// </summary>
public class FuzzyController
{
    private readonly List<FuzzyRule> _rules = new();
    private readonly Dictionary<string, FuzzyVariable> _inputs = new();
    private readonly Dictionary<string, FuzzyVariable> _outputs = new();

    public void AddInput(string name, double min, double max, string[] membershipNames, double[][] membershipParams)
    {
        var variable = new FuzzyVariable(name, min, max);
        for (int i = 0; i < membershipNames.Length; i++)
        {
            variable.AddMembership(membershipNames[i], membershipParams[i]);
        }
        _inputs[name] = variable;
    }

    public void AddOutput(string name, double min, double max, string[] membershipNames, double[][] membershipParams)
    {
        var variable = new FuzzyVariable(name, min, max);
        for (int i = 0; i < membershipNames.Length; i++)
        {
            variable.AddMembership(membershipNames[i], membershipParams[i]);
        }
        _outputs[name] = variable;
    }

    public void AddRule(string[] inputMemberships, string outputMembership)
    {
        _rules.Add(new FuzzyRule(inputMemberships, outputMembership));
    }

    public double Compute(Dictionary<string, double> inputs)
    {
        // Fuzzification
        var fuzzified = new Dictionary<string, Dictionary<string, double>>();
        foreach (var (name, value) in inputs)
        {
            if (_inputs.TryGetValue(name, out var variable))
            {
                fuzzified[name] = variable.Fuzzify(value);
            }
        }
        
        // Rule evaluation
        var ruleOutputs = new List<(double weight, double centroid)>();
        foreach (var rule in _rules)
        {
            double weight = EvaluateRule(rule, fuzzified);
            if (weight > 0)
            {
                var output = _outputs.Values.First();
                double centroid = output.GetCentroid(rule.OutputMembership);
                ruleOutputs.Add((weight, centroid));
            }
        }
        
        // Defuzzification (centroid method)
        if (ruleOutputs.Count == 0) return 0;
        
        double sumWeighted = ruleOutputs.Sum(r => r.weight * r.centroid);
        double sumWeights = ruleOutputs.Sum(r => r.weight);
        
        return sumWeighted / sumWeights;
    }

    private double EvaluateRule(FuzzyRule rule, Dictionary<string, Dictionary<string, double>> fuzzified)
    {
        double minDegree = 1.0;
        int idx = 0;
        
        foreach (var (inputName, _) in _inputs)
        {
            if (idx < rule.InputMemberships.Length && fuzzified.TryGetValue(inputName, out var memberships))
            {
                var membershipName = rule.InputMemberships[idx];
                if (memberships.TryGetValue(membershipName, out var degree))
                {
                    minDegree = System.Math.Min(minDegree, degree);
                }
            }
            idx++;
        }
        
        return minDegree;
    }

    private class FuzzyVariable
    {
        public string Name { get; }
        public double Min { get; }
        public double Max { get; }
        private readonly Dictionary<string, double[]> _memberships = new();

        public FuzzyVariable(string name, double min, double max)
        {
            Name = name;
            Min = min;
            Max = max;
        }

        public void AddMembership(string name, double[] parameters)
        {
            _memberships[name] = parameters;
        }

        public Dictionary<string, double> Fuzzify(double value)
        {
            var result = new Dictionary<string, double>();
            foreach (var (name, parameters) in _memberships)
            {
                result[name] = TriangularMembership(value, parameters);
            }
            return result;
        }

        public double GetCentroid(string membershipName)
        {
            if (_memberships.TryGetValue(membershipName, out var parameters))
            {
                return (parameters[0] + parameters[1] + parameters[2]) / 3;
            }
            return (Min + Max) / 2;
        }

        private static double TriangularMembership(double x, double[] parameters)
        {
            double a = parameters[0], b = parameters[1], c = parameters[2];
            if (x <= a || x >= c) return 0;
            if (x < b) return (x - a) / (b - a);
            return (c - x) / (c - b);
        }
    }

    private record FuzzyRule(string[] InputMemberships, string OutputMembership);
}

/// <summary>
/// Neural Network based controller.
/// </summary>
public class NeuralNetworkController
{
    private readonly int[] _layerSizes;
    private readonly double[][] _weights;
    private readonly double[][] _biases;
    private readonly double _learningRate;

    public NeuralNetworkController(int[] layerSizes, double learningRate = 0.01)
    {
        _layerSizes = layerSizes;
        _learningRate = learningRate;
        
        // Initialize weights and biases
        var random = new Random(42);
        _weights = new double[layerSizes.Length - 1][];
        _biases = new double[layerSizes.Length - 1][];
        
        for (int i = 0; i < layerSizes.Length - 1; i++)
        {
            int inputSize = layerSizes[i];
            int outputSize = layerSizes[i + 1];
            
            _weights[i] = new double[inputSize * outputSize];
            _biases[i] = new double[outputSize];
            
            // Xavier initialization
            double scale = System.Math.Sqrt(2.0 / (inputSize + outputSize));
            for (int j = 0; j < _weights[i].Length; j++)
            {
                _weights[i][j] = (random.NextDouble() * 2 - 1) * scale;
            }
        }
    }

    /// <summary>
    /// Forward pass through the network.
    /// </summary>
    public double[] Forward(double[] input)
    {
        var current = input;
        
        for (int layer = 0; layer < _weights.Length; layer++)
        {
            int inputSize = _layerSizes[layer];
            int outputSize = _layerSizes[layer + 1];
            var output = new double[outputSize];
            
            for (int j = 0; j < outputSize; j++)
            {
                double sum = _biases[layer][j];
                for (int i = 0; i < inputSize; i++)
                {
                    sum += current[i] * _weights[layer][i * outputSize + j];
                }
                
                // Activation (tanh for hidden layers, linear for output)
                output[j] = layer < _weights.Length - 1 
                    ? System.Math.Tanh(sum) 
                    : sum;
            }
            
            current = output;
        }
        
        return current;
    }

    /// <summary>
    /// Computes control signal.
    /// </summary>
    public double ComputeControl(double[] state, double[] reference)
    {
        var input = state.Concat(reference).ToArray();
        var output = Forward(input);
        return output[0];
    }
}
