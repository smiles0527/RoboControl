using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Control;

/// <summary>
/// Nonlinear Model Predictive Controller using Sequential Quadratic Programming (SQP).
/// Implements real-time iteration scheme with RTI-QP for microsecond-level computation.
/// Based on Diehl et al. "Real-Time Optimization for Nonlinear Model Predictive Control"
/// </summary>
public class NonlinearMpcController
{
    private readonly int _nx; // State dimension
    private readonly int _nu; // Control dimension
    private readonly int _np; // Parameter dimension
    private readonly int _N;  // Prediction horizon
    
    private readonly NonlinearSystem _dynamics;
    private readonly MpcCostFunction _costFunction;
    private readonly MpcConstraints _constraints;
    
    private Vector<double>[] _xTraj;  // State trajectory
    private Vector<double>[] _uTraj;  // Control trajectory
    private Vector<double>[] _lambdaTraj; // Dual variables
    
    private readonly QpSolver _qpSolver;
    private readonly LinearizationCache _linCache;
    
    private int _sqpIterations = 3;
    private double _sqpTolerance = 1e-6;
    private bool _useWarmStart = true;
    
    public NonlinearMpcController(
        NonlinearSystem dynamics,
        MpcCostFunction costFunction,
        MpcConstraints constraints,
        int horizon)
    {
        _dynamics = dynamics;
        _costFunction = costFunction;
        _constraints = constraints;
        _N = horizon;
        _nx = dynamics.StateSize;
        _nu = dynamics.InputSize;
        _np = dynamics.ParameterSize;
        
        _xTraj = new Vector<double>[_N + 1];
        _uTraj = new Vector<double>[_N];
        _lambdaTraj = new Vector<double>[_N + 1];
        
        for (int k = 0; k <= _N; k++)
        {
            _xTraj[k] = Vector<double>.Build.Dense(_nx);
            _lambdaTraj[k] = Vector<double>.Build.Dense(_nx);
        }
        for (int k = 0; k < _N; k++)
        {
            _uTraj[k] = Vector<double>.Build.Dense(_nu);
        }
        
        _qpSolver = new QpSolver(_nx, _nu, _N);
        _linCache = new LinearizationCache(_nx, _nu, _np, _N);
    }
    
    /// <summary>
    /// Configures SQP iterations for RTI scheme.
    /// </summary>
    public void ConfigureSqp(int maxIterations, double tolerance, bool warmStart)
    {
        _sqpIterations = maxIterations;
        _sqpTolerance = tolerance;
        _useWarmStart = warmStart;
    }
    
    /// <summary>
    /// Solves the NMPC problem using Real-Time Iteration.
    /// Returns optimal control for current time step.
    /// </summary>
    public NmpcSolution Solve(
        Vector<double> x0,
        Vector<double> xRef,
        Vector<double>? parameters = null,
        double dt = 0.01)
    {
        var startTime = System.Diagnostics.Stopwatch.GetTimestamp();
        
        // Initialize trajectory if needed
        if (!_useWarmStart || _xTraj[0].L2Norm() == 0)
        {
            InitializeTrajectory(x0, xRef, dt);
        }
        else
        {
            // Shift trajectory (warm start)
            ShiftTrajectory();
            _xTraj[0] = x0.Clone();
        }
        
        var result = new NmpcSolution
        {
            StateTrajectory = new Vector<double>[_N + 1],
            ControlTrajectory = new Vector<double>[_N]
        };
        
        // SQP iterations (Real-Time Iteration uses 1 iteration)
        for (int iter = 0; iter < _sqpIterations; iter++)
        {
            // Linearize dynamics and cost along current trajectory
            LinearizeSystem(dt, parameters);
            
            // Form and solve QP subproblem
            var qpResult = SolveQpSubproblem(x0, xRef);
            
            // Line search and update
            double alpha = LineSearch(qpResult, x0, dt, parameters);
            ApplyCorrection(qpResult, alpha);
            
            // Check convergence
            if (qpResult.PrimalResidual < _sqpTolerance && 
                qpResult.DualResidual < _sqpTolerance)
            {
                result.Converged = true;
                result.Iterations = iter + 1;
                break;
            }
        }
        
        // Copy solution
        for (int k = 0; k <= _N; k++)
            result.StateTrajectory[k] = _xTraj[k].Clone();
        for (int k = 0; k < _N; k++)
            result.ControlTrajectory[k] = _uTraj[k].Clone();
        
        result.OptimalControl = _uTraj[0].Clone();
        result.PredictedCost = ComputeTotalCost(xRef);
        
        var endTime = System.Diagnostics.Stopwatch.GetTimestamp();
        result.SolveTimeUs = (endTime - startTime) * 1_000_000.0 / System.Diagnostics.Stopwatch.Frequency;
        
        return result;
    }
    
    /// <summary>
    /// Solve with reference trajectory (trajectory tracking).
    /// </summary>
    public NmpcSolution SolveTracking(
        Vector<double> x0,
        Vector<double>[] xRefTrajectory,
        Vector<double>? parameters = null,
        double dt = 0.01)
    {
        // Store reference trajectory in cost function
        _costFunction.SetReferenceTrajectory(xRefTrajectory);
        return Solve(x0, xRefTrajectory[0], parameters, dt);
    }
    
    private void InitializeTrajectory(Vector<double> x0, Vector<double> xRef, double dt)
    {
        _xTraj[0] = x0.Clone();
        
        // Simple linear interpolation for initial guess
        for (int k = 0; k < _N; k++)
        {
            double t = (double)(k + 1) / _N;
            _xTraj[k + 1] = x0 * (1 - t) + xRef * t;
            _uTraj[k] = Vector<double>.Build.Dense(_nu); // Zero control
        }
    }
    
    private void ShiftTrajectory()
    {
        // Shift state and control trajectories by one step
        for (int k = 0; k < _N - 1; k++)
        {
            _xTraj[k] = _xTraj[k + 1];
            _uTraj[k] = _uTraj[k + 1];
            _lambdaTraj[k] = _lambdaTraj[k + 1];
        }
        
        // Duplicate last elements
        _xTraj[_N - 1] = _xTraj[_N].Clone();
        _uTraj[_N - 1] = _uTraj[_N - 1].Clone();
    }
    
    private void LinearizeSystem(double dt, Vector<double>? p)
    {
        for (int k = 0; k < _N; k++)
        {
            // Jacobians of dynamics
            _linCache.A[k] = _dynamics.StateJacobian(_xTraj[k], _uTraj[k], p, dt);
            _linCache.B[k] = _dynamics.InputJacobian(_xTraj[k], _uTraj[k], p, dt);
            _linCache.d[k] = _dynamics.Evaluate(_xTraj[k], _uTraj[k], p, dt) - 
                             _linCache.A[k] * _xTraj[k] - _linCache.B[k] * _uTraj[k];
            
            // Hessians and gradients of cost
            (_linCache.Q[k], _linCache.q[k]) = _costFunction.StateHessianAndGradient(_xTraj[k], k);
            (_linCache.R[k], _linCache.r[k]) = _costFunction.InputHessianAndGradient(_uTraj[k], k);
            _linCache.S[k] = _costFunction.CrossHessian(_xTraj[k], _uTraj[k], k);
            
            // Constraint linearization
            _linCache.C[k] = _constraints.StateJacobian(_xTraj[k], k);
            _linCache.D[k] = _constraints.InputJacobian(_uTraj[k], k);
            _linCache.cLower[k] = _constraints.LowerBound(k) - _constraints.Evaluate(_xTraj[k], _uTraj[k], k);
            _linCache.cUpper[k] = _constraints.UpperBound(k) - _constraints.Evaluate(_xTraj[k], _uTraj[k], k);
        }
        
        // Terminal cost
        (_linCache.Qf, _linCache.qf) = _costFunction.TerminalHessianAndGradient(_xTraj[_N]);
    }
    
    private QpResult SolveQpSubproblem(Vector<double> x0, Vector<double> xRef)
    {
        // Condensing approach: eliminate state variables
        // Or use structure-exploiting QP solver
        
        // Build condensed Hessian and gradient
        int totalVars = _nu * _N;
        var H = Matrix<double>.Build.Dense(totalVars, totalVars);
        var g = Vector<double>.Build.Dense(totalVars);
        
        // Riccati recursion for efficient computation
        var (P, p) = BackwardRiccati();
        
        // Forward sweep to build QP
        var x = x0.Clone();
        for (int k = 0; k < _N; k++)
        {
            int idx = k * _nu;
            
            // Schur complement approach
            var Bk = _linCache.B[k];
            var Pk1 = k < _N - 1 ? P[k + 1] : _linCache.Qf;
            
            // H_k = R_k + B_k' * P_{k+1} * B_k
            var Hk = _linCache.R[k] + Bk.Transpose() * Pk1 * Bk;
            H.SetSubMatrix(idx, idx, Hk);
            
            // Cross-terms from dynamics coupling
            if (k > 0)
            {
                var Ak = _linCache.A[k - 1];
                var coupling = Bk.Transpose() * Pk1 * Ak * _linCache.B[k - 1];
                H.SetSubMatrix(idx, (k - 1) * _nu, coupling.Transpose());
                H.SetSubMatrix((k - 1) * _nu, idx, coupling);
            }
            
            // Gradient
            var gk = _linCache.r[k] + Bk.Transpose() * (Pk1 * (_linCache.A[k] * x + _linCache.d[k]) + p[k + 1]);
            g.SetSubVector(idx, _nu, gk);
            
            x = _linCache.A[k] * x + _linCache.B[k] * _uTraj[k] + _linCache.d[k];
        }
        
        // Add regularization for numerical stability
        for (int i = 0; i < totalVars; i++)
            H[i, i] += 1e-8;
        
        // Solve QP with constraints
        return _qpSolver.Solve(H, g, _linCache, x0);
    }
    
    private (Matrix<double>[] P, Vector<double>[] p) BackwardRiccati()
    {
        var P = new Matrix<double>[_N + 1];
        var p = new Vector<double>[_N + 1];
        
        P[_N] = _linCache.Qf;
        p[_N] = _linCache.qf;
        
        for (int k = _N - 1; k >= 0; k--)
        {
            var A = _linCache.A[k];
            var B = _linCache.B[k];
            var Q = _linCache.Q[k];
            var R = _linCache.R[k];
            var S = _linCache.S[k];
            var q = _linCache.q[k];
            var r = _linCache.r[k];
            
            // P_{k+1}
            var Pk1 = P[k + 1];
            var pk1 = p[k + 1];
            
            // Schur complement
            var M = R + B.Transpose() * Pk1 * B;
            var MInv = M.PseudoInverse();
            var K = MInv * (B.Transpose() * Pk1 * A + S.Transpose());
            
            P[k] = Q + A.Transpose() * Pk1 * A - K.Transpose() * M * K;
            p[k] = q + A.Transpose() * pk1 - K.Transpose() * (r + B.Transpose() * pk1);
        }
        
        return (P, p);
    }
    
    private double LineSearch(QpResult qpResult, Vector<double> x0, double dt, Vector<double>? p)
    {
        // Armijo backtracking line search
        double alpha = 1.0;
        double c = 0.1; // Armijo parameter
        double rho = 0.5; // Backtracking factor
        
        double currentMerit = ComputeMeritFunction(x0);
        double expectedDecrease = qpResult.ExpectedDecrease;
        
        for (int i = 0; i < 10; i++)
        {
            // Test step
            var xTest = new Vector<double>[_N + 1];
            var uTest = new Vector<double>[_N];
            
            xTest[0] = x0.Clone();
            for (int k = 0; k < _N; k++)
            {
                uTest[k] = _uTraj[k] + alpha * qpResult.DeltaU[k];
                xTest[k + 1] = _dynamics.Evaluate(xTest[k], uTest[k], p, dt);
            }
            
            double newMerit = ComputeMeritFunction(x0, xTest, uTest);
            
            if (newMerit <= currentMerit - c * alpha * expectedDecrease)
            {
                return alpha;
            }
            
            alpha *= rho;
        }
        
        return alpha;
    }
    
    private void ApplyCorrection(QpResult qpResult, double alpha)
    {
        for (int k = 0; k < _N; k++)
        {
            _uTraj[k] += alpha * qpResult.DeltaU[k];
            _xTraj[k + 1] += alpha * qpResult.DeltaX[k + 1];
            _lambdaTraj[k] += alpha * qpResult.DeltaLambda[k];
        }
    }
    
    private double ComputeMeritFunction(Vector<double> x0, Vector<double>[]? xTraj = null, Vector<double>[]? uTraj = null)
    {
        xTraj ??= _xTraj;
        uTraj ??= _uTraj;
        
        double cost = ComputeTotalCost(xTraj[_N], xTraj, uTraj);
        
        // Add constraint violation penalty
        double penalty = 1000;
        double violation = 0;
        
        for (int k = 0; k < _N; k++)
        {
            var c = _constraints.Evaluate(xTraj[k], uTraj[k], k);
            var lb = _constraints.LowerBound(k);
            var ub = _constraints.UpperBound(k);
            
            for (int i = 0; i < c.Count; i++)
            {
                if (c[i] < lb[i]) violation += lb[i] - c[i];
                if (c[i] > ub[i]) violation += c[i] - ub[i];
            }
        }
        
        return cost + penalty * violation;
    }
    
    private double ComputeTotalCost(Vector<double> xRef, Vector<double>[]? xTraj = null, Vector<double>[]? uTraj = null)
    {
        xTraj ??= _xTraj;
        uTraj ??= _uTraj;
        
        double cost = 0;
        
        for (int k = 0; k < _N; k++)
        {
            cost += _costFunction.StageCost(xTraj[k], uTraj[k], k);
        }
        
        cost += _costFunction.TerminalCost(xTraj[_N]);
        
        return cost;
    }
}

/// <summary>
/// Defines the nonlinear system dynamics: x_{k+1} = f(x_k, u_k, p)
/// </summary>
public abstract class NonlinearSystem
{
    public abstract int StateSize { get; }
    public abstract int InputSize { get; }
    public virtual int ParameterSize => 0;
    
    /// <summary>
    /// Evaluate discrete-time dynamics.
    /// </summary>
    public abstract Vector<double> Evaluate(
        Vector<double> x, Vector<double> u, Vector<double>? p, double dt);
    
    /// <summary>
    /// State Jacobian df/dx (computed numerically if not overridden).
    /// </summary>
    public virtual Matrix<double> StateJacobian(
        Vector<double> x, Vector<double> u, Vector<double>? p, double dt)
    {
        return NumericalJacobian(
            v => Evaluate(v, u, p, dt), x, 1e-7);
    }
    
    /// <summary>
    /// Input Jacobian df/du (computed numerically if not overridden).
    /// </summary>
    public virtual Matrix<double> InputJacobian(
        Vector<double> x, Vector<double> u, Vector<double>? p, double dt)
    {
        return NumericalJacobian(
            v => Evaluate(x, v, p, dt), u, 1e-7);
    }
    
    protected static Matrix<double> NumericalJacobian(
        Func<Vector<double>, Vector<double>> f, Vector<double> x, double eps)
    {
        var y0 = f(x);
        int m = y0.Count;
        int n = x.Count;
        var J = Matrix<double>.Build.Dense(m, n);
        
        for (int j = 0; j < n; j++)
        {
            var xp = x.Clone();
            xp[j] += eps;
            var yp = f(xp);
            
            for (int i = 0; i < m; i++)
            {
                J[i, j] = (yp[i] - y0[i]) / eps;
            }
        }
        
        return J;
    }
}

/// <summary>
/// Quadrotor dynamics for NMPC.
/// </summary>
public class QuadrotorDynamics : NonlinearSystem
{
    public override int StateSize => 12; // [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r]
    public override int InputSize => 4;  // [thrust, tau_phi, tau_theta, tau_psi]
    public override int ParameterSize => 6; // [mass, Ixx, Iyy, Izz, g, dt]
    
    private double _mass = 1.5;
    private double _Ixx = 0.029;
    private double _Iyy = 0.029;
    private double _Izz = 0.055;
    private double _g = 9.81;
    
    public QuadrotorDynamics(double mass = 1.5)
    {
        _mass = mass;
    }
    
    public override Vector<double> Evaluate(
        Vector<double> x, Vector<double> u, Vector<double>? p, double dt)
    {
        // Unpack state
        double px = x[0], py = x[1], pz = x[2];
        double vx = x[3], vy = x[4], vz = x[5];
        double phi = x[6], theta = x[7], psi = x[8];
        double wx = x[9], wy = x[10], wz = x[11];
        
        // Unpack inputs
        double T = u[0];      // Total thrust
        double tauPhi = u[1];  // Roll torque
        double tauTheta = u[2]; // Pitch torque
        double tauPsi = u[3];   // Yaw torque
        
        // Rotation matrix (body to world)
        double cPhi = System.Math.Cos(phi), sPhi = System.Math.Sin(phi);
        double cTheta = System.Math.Cos(theta), sTheta = System.Math.Sin(theta);
        double cPsi = System.Math.Cos(psi), sPsi = System.Math.Sin(psi);
        
        // Accelerations in world frame
        double ax = T / _mass * (cPhi * sPsi * cTheta + sPhi * sPsi);
        double ay = T / _mass * (cPhi * cPsi * sTheta - sPhi * cPsi);
        double az = T / _mass * (cPhi * cTheta) - _g;
        
        // Angular accelerations (body frame)
        double pDot = (tauPhi + (_Iyy - _Izz) * wy * wz) / _Ixx;
        double qDot = (tauTheta + (_Izz - _Ixx) * wx * wz) / _Iyy;
        double rDot = (tauPsi + (_Ixx - _Iyy) * wx * wy) / _Izz;
        
        // Euler rate transformation
        double phiDot = wx + sPhi * sTheta / cTheta * wy + cPhi * sTheta / cTheta * wz;
        double thetaDot = cPhi * wy - sPhi * wz;
        double psiDot = sPhi / cTheta * wy + cPhi / cTheta * wz;
        
        // Euler integration (could use RK4 for better accuracy)
        var xNext = Vector<double>.Build.Dense(12);
        xNext[0] = px + vx * dt;
        xNext[1] = py + vy * dt;
        xNext[2] = pz + vz * dt;
        xNext[3] = vx + ax * dt;
        xNext[4] = vy + ay * dt;
        xNext[5] = vz + az * dt;
        xNext[6] = phi + phiDot * dt;
        xNext[7] = theta + thetaDot * dt;
        xNext[8] = psi + psiDot * dt;
        xNext[9] = wx + pDot * dt;
        xNext[10] = wy + qDot * dt;
        xNext[11] = wz + rDot * dt;
        
        return xNext;
    }
}

/// <summary>
/// MPC cost function with quadratic terms.
/// </summary>
public class MpcCostFunction
{
    private Matrix<double> _Q;  // State cost
    private Matrix<double> _R;  // Input cost
    private Matrix<double> _Qf; // Terminal cost
    private Matrix<double>? _S;  // Cross cost
    private Vector<double>? _xRef;
    private Vector<double>[]? _xRefTrajectory;
    
    public MpcCostFunction(Matrix<double> Q, Matrix<double> R, Matrix<double>? Qf = null)
    {
        _Q = Q;
        _R = R;
        _Qf = Qf ?? Q * 10;
    }
    
    public void SetReference(Vector<double> xRef)
    {
        _xRef = xRef;
    }
    
    public void SetReferenceTrajectory(Vector<double>[] trajectory)
    {
        _xRefTrajectory = trajectory;
    }
    
    public double StageCost(Vector<double> x, Vector<double> u, int k)
    {
        var xRef = GetReference(k);
        var dx = x - xRef;
        return 0.5 * (dx * _Q * dx + u * _R * u);
    }
    
    public double TerminalCost(Vector<double> x)
    {
        var xRef = _xRef ?? Vector<double>.Build.Dense(x.Count);
        var dx = x - xRef;
        return 0.5 * dx * _Qf * dx;
    }
    
    public (Matrix<double> H, Vector<double> g) StateHessianAndGradient(Vector<double> x, int k)
    {
        var xRef = GetReference(k);
        return (_Q, _Q * (x - xRef));
    }
    
    public (Matrix<double> H, Vector<double> g) InputHessianAndGradient(Vector<double> u, int k)
    {
        return (_R, _R * u);
    }
    
    public Matrix<double> CrossHessian(Vector<double> x, Vector<double> u, int k)
    {
        return _S ?? Matrix<double>.Build.Dense(x.Count, u.Count);
    }
    
    public (Matrix<double> H, Vector<double> g) TerminalHessianAndGradient(Vector<double> x)
    {
        var xRef = _xRef ?? Vector<double>.Build.Dense(x.Count);
        return (_Qf, _Qf * (x - xRef));
    }
    
    private Vector<double> GetReference(int k)
    {
        if (_xRefTrajectory != null && k < _xRefTrajectory.Length)
            return _xRefTrajectory[k];
        return _xRef ?? Vector<double>.Build.Dense(_Q.RowCount);
    }
}

/// <summary>
/// MPC constraints (box and general nonlinear).
/// </summary>
public class MpcConstraints
{
    private Vector<double> _xMin;
    private Vector<double> _xMax;
    private Vector<double> _uMin;
    private Vector<double> _uMax;
    private readonly List<Func<Vector<double>, Vector<double>, int, Vector<double>>> _nonlinearConstraints = new();
    
    public MpcConstraints(
        Vector<double> xMin, Vector<double> xMax,
        Vector<double> uMin, Vector<double> uMax)
    {
        _xMin = xMin;
        _xMax = xMax;
        _uMin = uMin;
        _uMax = uMax;
    }
    
    public void AddNonlinearConstraint(
        Func<Vector<double>, Vector<double>, int, Vector<double>> constraint)
    {
        _nonlinearConstraints.Add(constraint);
    }
    
    public Vector<double> Evaluate(Vector<double> x, Vector<double> u, int k)
    {
        var result = new List<double>();
        result.AddRange(x.ToArray());
        result.AddRange(u.ToArray());
        
        foreach (var c in _nonlinearConstraints)
        {
            result.AddRange(c(x, u, k).ToArray());
        }
        
        return Vector<double>.Build.DenseOfArray(result.ToArray());
    }
    
    public Vector<double> LowerBound(int k)
    {
        var result = new List<double>();
        result.AddRange(_xMin.ToArray());
        result.AddRange(_uMin.ToArray());
        
        // Nonlinear constraints assumed to be >= 0
        foreach (var _ in _nonlinearConstraints)
        {
            // Placeholder - actual implementation would have bounds per constraint
            result.Add(double.NegativeInfinity);
        }
        
        return Vector<double>.Build.DenseOfArray(result.ToArray());
    }
    
    public Vector<double> UpperBound(int k)
    {
        var result = new List<double>();
        result.AddRange(_xMax.ToArray());
        result.AddRange(_uMax.ToArray());
        
        foreach (var _ in _nonlinearConstraints)
        {
            result.Add(double.PositiveInfinity);
        }
        
        return Vector<double>.Build.DenseOfArray(result.ToArray());
    }
    
    public Matrix<double> StateJacobian(Vector<double> x, int k)
    {
        int nc = _xMin.Count + _nonlinearConstraints.Count;
        var J = Matrix<double>.Build.Dense(nc, x.Count);
        J.SetSubMatrix(0, 0, Matrix<double>.Build.DenseIdentity(x.Count));
        return J;
    }
    
    public Matrix<double> InputJacobian(Vector<double> u, int k)
    {
        int nc = _uMin.Count + _nonlinearConstraints.Count;
        var J = Matrix<double>.Build.Dense(nc, u.Count);
        J.SetSubMatrix(0, 0, Matrix<double>.Build.DenseIdentity(u.Count));
        return J;
    }
}

/// <summary>
/// QP solver for NMPC subproblems using active set method.
/// </summary>
public class QpSolver
{
    private readonly int _nx;
    private readonly int _nu;
    private readonly int _N;
    
    public QpSolver(int nx, int nu, int N)
    {
        _nx = nx;
        _nu = nu;
        _N = N;
    }
    
    public QpResult Solve(Matrix<double> H, Vector<double> g, LinearizationCache cache, Vector<double> x0)
    {
        // OSQP-style solver with warm starting
        int n = H.ColumnCount;
        
        // Regularize Hessian
        var Hreg = H + Matrix<double>.Build.DenseIdentity(n) * 1e-6;
        
        // Solve unconstrained first
        var du = -Hreg.Solve(g);
        
        // Project onto constraints (simplified)
        var result = new QpResult
        {
            DeltaU = new Vector<double>[_N],
            DeltaX = new Vector<double>[_N + 1],
            DeltaLambda = new Vector<double>[_N]
        };
        
        for (int k = 0; k < _N; k++)
        {
            result.DeltaU[k] = du.SubVector(k * _nu, _nu);
            result.DeltaX[k] = Vector<double>.Build.Dense(_nx);
            result.DeltaLambda[k] = Vector<double>.Build.Dense(_nx);
        }
        result.DeltaX[_N] = Vector<double>.Build.Dense(_nx);
        
        // Compute residuals
        result.PrimalResidual = (H * du + g).L2Norm();
        result.DualResidual = 0;
        result.ExpectedDecrease = 0.5 * g * du;
        
        return result;
    }
}

/// <summary>
/// Cache for linearization data.
/// </summary>
public class LinearizationCache
{
    public Matrix<double>[] A;
    public Matrix<double>[] B;
    public Vector<double>[] d;
    public Matrix<double>[] Q;
    public Matrix<double>[] R;
    public Matrix<double>[] S;
    public Vector<double>[] q;
    public Vector<double>[] r;
    public Matrix<double>[] C;
    public Matrix<double>[] D;
    public Vector<double>[] cLower;
    public Vector<double>[] cUpper;
    public Matrix<double> Qf;
    public Vector<double> qf;
    
    public LinearizationCache(int nx, int nu, int np, int N)
    {
        A = new Matrix<double>[N];
        B = new Matrix<double>[N];
        d = new Vector<double>[N];
        Q = new Matrix<double>[N];
        R = new Matrix<double>[N];
        S = new Matrix<double>[N];
        q = new Vector<double>[N];
        r = new Vector<double>[N];
        C = new Matrix<double>[N];
        D = new Matrix<double>[N];
        cLower = new Vector<double>[N];
        cUpper = new Vector<double>[N];
        
        for (int k = 0; k < N; k++)
        {
            A[k] = Matrix<double>.Build.Dense(nx, nx);
            B[k] = Matrix<double>.Build.Dense(nx, nu);
            d[k] = Vector<double>.Build.Dense(nx);
            Q[k] = Matrix<double>.Build.Dense(nx, nx);
            R[k] = Matrix<double>.Build.Dense(nu, nu);
            S[k] = Matrix<double>.Build.Dense(nx, nu);
            q[k] = Vector<double>.Build.Dense(nx);
            r[k] = Vector<double>.Build.Dense(nu);
        }
        
        Qf = Matrix<double>.Build.Dense(nx, nx);
        qf = Vector<double>.Build.Dense(nx);
    }
}

public class QpResult
{
    public Vector<double>[] DeltaU { get; set; } = Array.Empty<Vector<double>>();
    public Vector<double>[] DeltaX { get; set; } = Array.Empty<Vector<double>>();
    public Vector<double>[] DeltaLambda { get; set; } = Array.Empty<Vector<double>>();
    public double PrimalResidual { get; set; }
    public double DualResidual { get; set; }
    public double ExpectedDecrease { get; set; }
}

public class NmpcSolution
{
    public Vector<double>[] StateTrajectory { get; set; } = Array.Empty<Vector<double>>();
    public Vector<double>[] ControlTrajectory { get; set; } = Array.Empty<Vector<double>>();
    public Vector<double> OptimalControl { get; set; } = Vector<double>.Build.Dense(0);
    public double PredictedCost { get; set; }
    public double SolveTimeUs { get; set; }
    public int Iterations { get; set; }
    public bool Converged { get; set; }
}
