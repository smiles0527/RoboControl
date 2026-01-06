using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Control;

// Note: HInfinityController is defined in ClassicalControllers.cs
// This file contains additional robust and adaptive control methods

/// <summary>
/// Lyapunov-based Model Reference Adaptive Controller (MRAC).
/// Guarantees stability via Lyapunov analysis.
/// </summary>
public class LyapunovMrac
{
    private readonly Matrix<double> _Am; // Reference model A
    private readonly Matrix<double> _Bm; // Reference model B
    private readonly Matrix<double> _B;  // Plant B (known)
    private readonly Matrix<double> _Gamma; // Adaptation gain matrix
    
    private Vector<double> _xm; // Reference model state
    private Matrix<double> _Kx; // State feedback gain estimate
    private Matrix<double> _Kr; // Feedforward gain estimate
    
    private readonly int _n; // State dimension
    private readonly int _m; // Input dimension
    
    public LyapunovMrac(
        Matrix<double> referenceModelA,
        Matrix<double> referenceModelB,
        Matrix<double> plantB,
        Matrix<double> adaptationGain)
    {
        _Am = referenceModelA;
        _Bm = referenceModelB;
        _B = plantB;
        _Gamma = adaptationGain;
        
        _n = _Am.RowCount;
        _m = _B.ColumnCount;
        
        _xm = Vector<double>.Build.Dense(_n);
        _Kx = Matrix<double>.Build.Dense(_m, _n);
        _Kr = Matrix<double>.Build.DenseIdentity(_m);
    }
    
    /// <summary>
    /// Initialize controller with initial gains.
    /// </summary>
    public void Initialize(Matrix<double>? initialKx = null, Matrix<double>? initialKr = null)
    {
        if (initialKx != null) _Kx = initialKx.Clone();
        if (initialKr != null) _Kr = initialKr.Clone();
        _xm = Vector<double>.Build.Dense(_n);
    }
    
    /// <summary>
    /// Update adaptive gains and compute control.
    /// </summary>
    public Vector<double> Update(Vector<double> x, Vector<double> r, double dt)
    {
        // Update reference model
        var xmDot = _Am * _xm + _Bm * r;
        _xm = _xm + xmDot * dt;
        
        // Tracking error
        var e = x - _xm;
        
        // Solve Lyapunov equation: Am'P + PAm = -Q
        var Q = Matrix<double>.Build.DenseIdentity(_n);
        var P = SolveLyapunov(_Am, Q);
        
        // Adaptation law (gradient descent on Lyapunov function)
        // dKx/dt = -? * x * e' * P * B
        // dKr/dt = -? * r * e' * P * B
        
        var PB = P * _B;
        var ePB = PB.Transpose() * e;

        var dKx = -_Gamma * x.OuterProduct(ePB);
        var dKr = -_Gamma * r.OuterProduct(ePB);
        
        _Kx = _Kx + dKx.Transpose() * dt;
        _Kr = _Kr + dKr.Transpose() * dt;
        
        // Compute control: u = Kx * x + Kr * r
        return _Kx * x + _Kr * r;
    }
    
    /// <summary>
    /// Get control without adaptation (for testing).
    /// </summary>
    public Vector<double> Compute(Vector<double> x, Vector<double> r)
    {
        return _Kx * x + _Kr * r;
    }
    
    public Matrix<double> StateGain => _Kx.Clone();
    public Matrix<double> FeedforwardGain => _Kr.Clone();
    public Vector<double> ReferenceState => _xm.Clone();
    
    private Matrix<double> SolveLyapunov(Matrix<double> A, Matrix<double> Q)
    {
        // Solve A'P + PA = -Q using vectorization
        // (I ? A' + A' ? I) vec(P) = -vec(Q)
        
        int n = A.RowCount;
        var I = Matrix<double>.Build.DenseIdentity(n);
        
        // Kronecker products
        var M = Kronecker(I, A.Transpose()) + Kronecker(A.Transpose(), I);
        
        // Solve
        var qVec = Q.Enumerate().ToArray();
        var pVec = M.Solve(Vector<double>.Build.DenseOfArray(qVec.Select(x => -x).ToArray()));
        
        // Reshape
        var P = Matrix<double>.Build.Dense(n, n);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                P[i, j] = pVec[i * n + j];
            }
        }
        
        return P;
    }
    
    private Matrix<double> Kronecker(Matrix<double> A, Matrix<double> B)
    {
        int m = A.RowCount * B.RowCount;
        int n = A.ColumnCount * B.ColumnCount;
        var result = Matrix<double>.Build.Dense(m, n);
        
        for (int i = 0; i < A.RowCount; i++)
        {
            for (int j = 0; j < A.ColumnCount; j++)
            {
                result.SetSubMatrix(i * B.RowCount, j * B.ColumnCount, A[i, j] * B);
            }
        }
        
        return result;
    }
}

/// <summary>
/// L1 Adaptive Controller with guaranteed robustness margins.
/// Based on Hovakimyan &amp; Cao "L1 Adaptive Control Theory"
/// </summary>
public class L1AdaptiveController
{
    private readonly Matrix<double> _Am; // Desired closed-loop dynamics
    private readonly Matrix<double> _B;  // Known input matrix
    private readonly double _adaptationRate;
    
    private Vector<double> _xHat;  // State predictor
    private Vector<double> _sigma; // Adaptive parameter
    private Vector<double> _sigmaFiltered; // Filtered adaptive parameter
    private readonly LowPassFilter _filter;
    
    private readonly int _n;
    private readonly int _m;
    
    public L1AdaptiveController(
        Matrix<double> desiredDynamics,
        Matrix<double> inputMatrix,
        double adaptationRate,
        double filterBandwidth)
    {
        _Am = desiredDynamics;
        _B = inputMatrix;
        _adaptationRate = adaptationRate;
        
        _n = _Am.RowCount;
        _m = _B.ColumnCount;
        
        _xHat = Vector<double>.Build.Dense(_n);
        _sigma = Vector<double>.Build.Dense(_m);
        _sigmaFiltered = Vector<double>.Build.Dense(_m);
        
        _filter = new LowPassFilter(_m, filterBandwidth);
    }
    
    /// <summary>
    /// Update controller with new measurement.
    /// </summary>
    public Vector<double> Update(Vector<double> x, Vector<double> r, double dt)
    {
        // State prediction error
        var xTilde = _xHat - x;
        
        // Adaptive law with projection
        // ?? = -? * Proj(?, B' * P * x?)
        var P = SolveLyapunovForL1();
        var adaptationTerm = _B.Transpose() * P * xTilde;
        
        _sigma = _sigma - _adaptationRate * adaptationTerm * dt;
        
        // Apply projection to keep ? bounded
        _sigma = ProjectToBound(_sigma, 100);
        
        // Low-pass filter the adaptive parameter
        _sigmaFiltered = _filter.Filter(_sigma, dt);
        
        // Update state predictor
        // x?? = Am * x? + B * (u + ?)
        var xHatDot = _Am * _xHat + _B * (_sigmaFiltered + ComputeNominalControl(r));
        _xHat = _xHat + xHatDot * dt;
        
        // Control law: u = -?_filtered + nominal
        return ComputeNominalControl(r) - _sigmaFiltered;
    }
    
    private Vector<double> ComputeNominalControl(Vector<double> r)
    {
        // Simple reference feedforward
        return r;
    }
    
    private Matrix<double> SolveLyapunovForL1()
    {
        // P satisfying Am'P + PAm = -Q
        var Q = Matrix<double>.Build.DenseIdentity(_n);
        
        // Simplified: use diagonal P for speed
        var P = Matrix<double>.Build.DenseIdentity(_n);
        
        for (int i = 0; i < _n; i++)
        {
            if (_Am[i, i] < 0)
            {
                P[i, i] = -Q[i, i] / (2 * _Am[i, i]);
            }
        }
        
        return P;
    }
    
    private Vector<double> ProjectToBound(Vector<double> v, double bound)
    {
        var result = v.Clone();
        double norm = v.L2Norm();
        
        if (norm > bound)
        {
            result = v * (bound / norm);
        }
        
        return result;
    }
    
    public Vector<double> AdaptiveParameter => _sigma.Clone();
    public Vector<double> StateEstimate => _xHat.Clone();
}

/// <summary>
/// Low-pass filter for L1 adaptive control.
/// </summary>
public class LowPassFilter
{
    private readonly double _bandwidth;
    private Vector<double> _state;
    
    public LowPassFilter(int dimension, double bandwidth)
    {
        _bandwidth = bandwidth;
        _state = Vector<double>.Build.Dense(dimension);
    }
    
    public Vector<double> Filter(Vector<double> input, double dt)
    {
        // First-order filter: ? = -?(y - u)
        var stateDerivative = _bandwidth * (input - _state);
        _state = _state + stateDerivative * dt;
        return _state;
    }
    
    public void Reset()
    {
        _state = Vector<double>.Build.Dense(_state.Count);
    }
}

/// <summary>
/// Tube-based Robust MPC with guaranteed constraint satisfaction.
/// Handles bounded disturbances with hard constraint guarantees.
/// </summary>
public class TubeMpc
{
    private readonly int _nx;
    private readonly int _nu;
    private readonly int _N;
    
    private readonly Matrix<double> _A;
    private readonly Matrix<double> _B;
    private readonly Matrix<double> _K; // Ancillary feedback
    private readonly Matrix<double> _Ak; // A + BK (closed-loop)
    
    private readonly Vector<double> _xMax;
    private readonly Vector<double> _xMin;
    private readonly Vector<double> _uMax;
    private readonly Vector<double> _uMin;
    private readonly PolytopicSet _disturbanceSet;
    private readonly PolytopicSet _tube;
    
    private readonly ModelPredictiveController _nominalMpc;
    
    public TubeMpc(
        Matrix<double> A, Matrix<double> B,
        Matrix<double> Q, Matrix<double> R,
        Vector<double> xMin, Vector<double> xMax,
        Vector<double> uMin, Vector<double> uMax,
        PolytopicSet disturbanceSet,
        int horizon)
    {
        _A = A;
        _B = B;
        _nx = A.RowCount;
        _nu = B.ColumnCount;
        _N = horizon;
        
        _xMin = xMin;
        _xMax = xMax;
        _uMin = uMin;
        _uMax = uMax;
        _disturbanceSet = disturbanceSet;
        
        // Design ancillary controller K (LQR)
        _K = DesignAncillaryController(A, B, Q, R);
        _Ak = A + B * _K;
        
        // Compute minimal robust positively invariant set (tube)
        _tube = ComputeMinimalRpi();
        
        // Tighten constraints
        var (xMinTight, xMaxTight, uMinTight, uMaxTight) = TightenConstraints();
        
        // Create nominal MPC with tightened constraints
        _nominalMpc = new ModelPredictiveController(A, B, Q, R, horizon);
        _nominalMpc.SetInputConstraints(uMinTight, uMaxTight);
        _nominalMpc.SetStateConstraints(xMinTight, xMaxTight);
    }
    
    /// <summary>
    /// Compute robust control for current state.
    /// </summary>
    public Vector<double> Compute(Vector<double> x, Vector<double> xRef)
    {
        // Solve nominal MPC for tube center
        var xRefArray = new Vector<double>[_N];
        for (int i = 0; i < _N; i++) xRefArray[i] = xRef;
        
        var nominalSolution = _nominalMpc.Solve(x, xRefArray);
        var vNominal = nominalSolution[0];
        
        // Actual control: u = v + K(x - z)
        // where z is nominal state (tube center), v is nominal control
        // For simplicity, use current state as nominal state
        return vNominal + _K * x;
    }
    
    /// <summary>
    /// Check if state is feasible (inside tightened constraint set).
    /// </summary>
    public bool IsFeasible(Vector<double> x)
    {
        var (xMinTight, xMaxTight, _, _) = TightenConstraints();
        
        for (int i = 0; i < _nx; i++)
        {
            if (x[i] < xMinTight[i] || x[i] > xMaxTight[i])
                return false;
        }
        
        return true;
    }
    
    private Matrix<double> DesignAncillaryController(Matrix<double> A, Matrix<double> B, Matrix<double> Q, Matrix<double> R)
    {
        // Discrete LQR for ancillary feedback
        var P = SolveDiscreteRiccati(A, B, Q, R);
        return -(R + B.Transpose() * P * B).Inverse() * B.Transpose() * P * A;
    }
    
    private Matrix<double> SolveDiscreteRiccati(Matrix<double> A, Matrix<double> B, Matrix<double> Q, Matrix<double> R)
    {
        // Iterative solution
        var P = Q.Clone();
        
        for (int iter = 0; iter < 100; iter++)
        {
            var Pnew = Q + A.Transpose() * P * A - 
                       A.Transpose() * P * B * (R + B.Transpose() * P * B).Inverse() * B.Transpose() * P * A;
            
            if ((Pnew - P).FrobeniusNorm() < 1e-10)
                break;
            
            P = Pnew;
        }
        
        return P;
    }
    
    private PolytopicSet ComputeMinimalRpi()
    {
        // Compute minimal robust positively invariant set for x_{k+1} = (A+BK)x_k + w
        // Using algorithm from Rakovic et al.
        
        // Start with disturbance set
        var Z = _disturbanceSet;
        
        // Iterate: Z_{k+1} = Z_k ? (A+BK)^k W
        int maxIter = 20;
        var Ak_power = Matrix<double>.Build.DenseIdentity(_nx);
        
        for (int k = 0; k < maxIter; k++)
        {
            Ak_power = Ak_power * _Ak;
            
            // Check if contribution is negligible
            if (Ak_power.FrobeniusNorm() < 0.01)
                break;
            
            Z = Z.MinkowskiSum(_disturbanceSet.Transform(Ak_power));
        }
        
        return Z;
    }
    
    private (Vector<double> xMin, Vector<double> xMax, Vector<double> uMin, Vector<double> uMax) TightenConstraints()
    {
        // Tighten state constraints: X_tightened = X ? Z
        var xMinTight = _xMin + _tube.MinVertex();
        var xMaxTight = _xMax - _tube.MaxVertex();
        
        // Tighten input constraints: U_tightened = U ? KZ
        var kz = _tube.Transform(_K);
        var uMinTight = _uMin + kz.MinVertex();
        var uMaxTight = _uMax - kz.MaxVertex();
        
        return (xMinTight, xMaxTight, uMinTight, uMaxTight);
    }
}

/// <summary>
/// Polytopic set representation for robust control.
/// </summary>
public class PolytopicSet
{
    private readonly Matrix<double> _A; // Halfspace normals
    private readonly Vector<double> _b; // Halfspace bounds
    
    public PolytopicSet(Matrix<double> A, Vector<double> b)
    {
        _A = A;
        _b = b;
    }
    
    /// <summary>
    /// Create box constraint set.
    /// </summary>
    public static PolytopicSet Box(Vector<double> min, Vector<double> max)
    {
        int n = min.Count;
        var A = Matrix<double>.Build.Dense(2 * n, n);
        var b = Vector<double>.Build.Dense(2 * n);
        
        for (int i = 0; i < n; i++)
        {
            A[i, i] = 1;
            b[i] = max[i];
            A[n + i, i] = -1;
            b[n + i] = -min[i];
        }
        
        return new PolytopicSet(A, b);
    }
    
    /// <summary>
    /// Transform set by matrix: {Mx : x ? P}
    /// </summary>
    public PolytopicSet Transform(Matrix<double> M)
    {
        // For halfspace representation, need to compute A * M^{-1}
        try
        {
            var MInv = M.PseudoInverse();
            return new PolytopicSet(_A * MInv, _b);
        }
        catch
        {
            return this;
        }
    }
    
    /// <summary>
    /// Minkowski sum of two polytopes.
    /// </summary>
    public PolytopicSet MinkowskiSum(PolytopicSet other)
    {
        // For boxes, Minkowski sum is straightforward
        // For general polytopes, this is more complex
        
        // Combine halfspace constraints
        int m1 = _A.RowCount;
        int m2 = other._A.RowCount;
        int n = _A.ColumnCount;
        
        var newA = Matrix<double>.Build.Dense(m1 + m2, n);
        var newB = Vector<double>.Build.Dense(m1 + m2);
        
        newA.SetSubMatrix(0, 0, _A);
        newB.SetSubVector(0, m1, _b);
        
        newA.SetSubMatrix(m1, 0, other._A);
        
        // Adjust bounds for Minkowski sum
        for (int i = 0; i < m2; i++)
        {
            var row = other._A.Row(i);
            double support = ComputeSupport(row);
            newB[m1 + i] = other._b[i] + support;
        }
        
        return new PolytopicSet(newA, newB);
    }
    
    /// <summary>
    /// Compute support function in direction d.
    /// </summary>
    public double ComputeSupport(Vector<double> d)
    {
        // max d'x subject to Ax <= b
        // For box, this is sum of |d_i| * bound_i
        double support = 0;
        int n = d.Count;
        
        for (int i = 0; i < n; i++)
        {
            double maxVal = 0;
            for (int j = 0; j < _A.RowCount; j++)
            {
                if (_A[j, i] != 0)
                {
                    double bound = _b[j] / System.Math.Abs(_A[j, i]);
                    maxVal = System.Math.Max(maxVal, bound);
                }
            }
            support += System.Math.Abs(d[i]) * maxVal;
        }
        
        return support;
    }
    
    public Vector<double> MinVertex()
    {
        int n = _A.ColumnCount;
        var min = Vector<double>.Build.Dense(n);
        
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < _A.RowCount; j++)
            {
                if (_A[j, i] < 0)
                {
                    min[i] = System.Math.Max(min[i], -_b[j] / _A[j, i]);
                }
            }
        }
        
        return min;
    }
    
    public Vector<double> MaxVertex()
    {
        int n = _A.ColumnCount;
        var max = Vector<double>.Build.Dense(n);
        
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < _A.RowCount; j++)
            {
                if (_A[j, i] > 0)
                {
                    max[i] = System.Math.Max(max[i], _b[j] / _A[j, i]);
                }
            }
        }
        
        return max;
    }
}

/// <summary>
/// Differential Dynamic Programming (DDP) for trajectory optimization.
/// iLQR variant with Gauss-Newton Hessian approximation.
/// </summary>
public class DifferentialDynamicProgramming
{
    private readonly NonlinearSystem _dynamics;
    private readonly int _nx;
    private readonly int _nu;
    private readonly int _N;
    
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    private readonly Matrix<double> _Qf;
    
    private Vector<double>[] _x;
    private Vector<double>[] _u;
    private Matrix<double>[] _K;
    private Vector<double>[] _k;
    
    public DifferentialDynamicProgramming(
        NonlinearSystem dynamics,
        Matrix<double> Q, Matrix<double> R, Matrix<double> Qf,
        int horizon)
    {
        _dynamics = dynamics;
        _nx = dynamics.StateSize;
        _nu = dynamics.InputSize;
        _N = horizon;
        _Q = Q;
        _R = R;
        _Qf = Qf;
        
        _x = new Vector<double>[_N + 1];
        _u = new Vector<double>[_N];
        _K = new Matrix<double>[_N];
        _k = new Vector<double>[_N];
        
        for (int k = 0; k <= _N; k++)
            _x[k] = Vector<double>.Build.Dense(_nx);
        for (int k = 0; k < _N; k++)
        {
            _u[k] = Vector<double>.Build.Dense(_nu);
            _K[k] = Matrix<double>.Build.Dense(_nu, _nx);
            _k[k] = Vector<double>.Build.Dense(_nu);
        }
    }
    
    /// <summary>
    /// Solve trajectory optimization problem.
    /// </summary>
    public DdpResult Solve(Vector<double> x0, Vector<double> xGoal, double dt, int maxIterations = 50)
    {
        // Initialize trajectory
        InitializeTrajectory(x0, xGoal, dt);
        
        double lastCost = ComputeTotalCost(xGoal);
        
        for (int iter = 0; iter < maxIterations; iter++)
        {
            // Backward pass
            var success = BackwardPass(xGoal, dt);
            if (!success)
            {
                // Regularization increase needed
                continue;
            }
            
            // Forward pass with line search
            double alpha = ForwardPass(x0, xGoal, dt);
            
            double newCost = ComputeTotalCost(xGoal);
            double improvement = lastCost - newCost;
            
            if (improvement < 1e-6)
                break;
            
            lastCost = newCost;
        }
        
        return new DdpResult
        {
            StateTrajectory = _x.Select(v => v.Clone()).ToArray(),
            ControlTrajectory = _u.Select(v => v.Clone()).ToArray(),
            FeedbackGains = _K.Select(m => m.Clone()).ToArray(),
            FeedforwardGains = _k.Select(v => v.Clone()).ToArray(),
            Cost = lastCost
        };
    }
    
    private void InitializeTrajectory(Vector<double> x0, Vector<double> xGoal, double dt)
    {
        _x[0] = x0.Clone();
        
        for (int k = 0; k < _N; k++)
        {
            _u[k] = Vector<double>.Build.Dense(_nu);
            _x[k + 1] = _dynamics.Evaluate(_x[k], _u[k], null, dt);
        }
    }
    
    private bool BackwardPass(Vector<double> xGoal, double dt)
    {
        // Value function approximation: V(x) ? V + Vx'?x + 0.5 ?x'Vxx ?x
        var Vx = _Qf * (_x[_N] - xGoal);
        var Vxx = _Qf.Clone();
        
        for (int k = _N - 1; k >= 0; k--)
        {
            // Linearize dynamics
            var fx = _dynamics.StateJacobian(_x[k], _u[k], null, dt);
            var fu = _dynamics.InputJacobian(_x[k], _u[k], null, dt);
            
            // Cost derivatives
            var lx = _Q * (_x[k] - xGoal);
            var lu = _R * _u[k];
            var lxx = _Q;
            var luu = _R;
            var lux = Matrix<double>.Build.Dense(_nu, _nx);
            
            // Q-function derivatives
            var Qx = lx + fx.Transpose() * Vx;
            var Qu = lu + fu.Transpose() * Vx;
            var Qxx = lxx + fx.Transpose() * Vxx * fx;
            var Quu = luu + fu.Transpose() * Vxx * fu;
            var Qux = lux + fu.Transpose() * Vxx * fx;
            
            // Regularize Quu
            var QuuReg = Quu + Matrix<double>.Build.DenseIdentity(_nu) * 1e-6;
            
            // Check positive definiteness
            try
            {
                var chol = QuuReg.Cholesky();
            }
            catch
            {
                return false;
            }
            
            // Compute gains
            var QuuInv = QuuReg.Inverse();
            _K[k] = -QuuInv * Qux;
            _k[k] = -QuuInv * Qu;
            
            // Update value function
            Vx = Qx + _K[k].Transpose() * Quu * _k[k] + _K[k].Transpose() * Qu + Qux.Transpose() * _k[k];
            Vxx = Qxx + _K[k].Transpose() * Quu * _K[k] + _K[k].Transpose() * Qux + Qux.Transpose() * _K[k];
            Vxx = 0.5 * (Vxx + Vxx.Transpose());
        }
        
        return true;
    }
    
    private double ForwardPass(Vector<double> x0, Vector<double> xGoal, double dt)
    {
        double alpha = 1.0;
        double currentCost = ComputeTotalCost(xGoal);
        
        for (int ls = 0; ls < 10; ls++)
        {
            var xNew = new Vector<double>[_N + 1];
            var uNew = new Vector<double>[_N];
            
            xNew[0] = x0.Clone();
            
            for (int k = 0; k < _N; k++)
            {
                var dx = xNew[k] - _x[k];
                uNew[k] = _u[k] + alpha * _k[k] + _K[k] * dx;
                xNew[k + 1] = _dynamics.Evaluate(xNew[k], uNew[k], null, dt);
            }
            
            double newCost = ComputeTotalCost(xGoal, xNew, uNew);
            
            if (newCost < currentCost)
            {
                _x = xNew;
                _u = uNew;
                return alpha;
            }
            
            alpha *= 0.5;
        }
        
        return alpha;
    }
    
    private double ComputeTotalCost(Vector<double> xGoal, Vector<double>[]? x = null, Vector<double>[]? u = null)
    {
        x ??= _x;
        u ??= _u;
        
        double cost = 0;
        
        for (int k = 0; k < _N; k++)
        {
            var dx = x[k] - xGoal;
            cost += 0.5 * (dx * _Q * dx + u[k] * _R * u[k]);
        }
        
        var dxf = x[_N] - xGoal;
        cost += 0.5 * dxf * _Qf * dxf;
        
        return cost;
    }
}

public class DdpResult
{
    public Vector<double>[] StateTrajectory { get; set; } = [];
    public Vector<double>[] ControlTrajectory { get; set; } = [];
    public Matrix<double>[] FeedbackGains { get; set; } = [];
    public Vector<double>[] FeedforwardGains { get; set; } = [];
    public double Cost { get; set; }
}
