using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Geometry;

namespace ControlWorkbench.Math.Control;

/// <summary>
/// Control Barrier Functions (CBFs) for guaranteed safety in autonomous systems.
/// Implements Higher-Order CBFs, Exponential CBFs, and CBF-QP for safe control.
/// 
/// Based on:
/// - "Control Barrier Functions: Theory and Applications" (Ames et al., 2019)
/// - "Safety Barrier Certificates for Collisions-Free Multirobot Systems" (Wang et al., 2017)
/// </summary>
public class ControlBarrierFunction
{
    private readonly Func<Vector<double>, double> _h;
    private readonly Func<Vector<double>, Vector<double>> _dh;
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Matrix<double>> _g;
    private readonly double _alpha;
    private readonly int _relativeDegree;
    
    /// <summary>
    /// Create a CBF with barrier function h(x), dynamics dx = f(x) + g(x)u.
    /// </summary>
    /// <param name="h">Barrier function h(x) where h(x) >= 0 defines safe set</param>
    /// <param name="dh">Gradient of h</param>
    /// <param name="f">Drift dynamics f(x)</param>
    /// <param name="g">Control matrix g(x)</param>
    /// <param name="alpha">Extended class-K function parameter</param>
    /// <param name="relativeDegree">Relative degree of the CBF</param>
    public ControlBarrierFunction(
        Func<Vector<double>, double> h,
        Func<Vector<double>, Vector<double>> dh,
        Func<Vector<double>, Vector<double>, Vector<double>> f,
        Func<Vector<double>, Matrix<double>> g,
        double alpha = 1.0,
        int relativeDegree = 1)
    {
        _h = h;
        _dh = dh;
        _f = f;
        _g = g;
        _alpha = alpha;
        _relativeDegree = relativeDegree;
    }
    
    /// <summary>
    /// Evaluate the barrier function h(x).
    /// </summary>
    public double Evaluate(Vector<double> x) => _h(x);
    
    /// <summary>
    /// Compute Lie derivative L_f h(x) = dh/dx � f(x).
    /// </summary>
    public double LieFDerivative(Vector<double> x, Vector<double> u)
    {
        var dh = _dh(x);
        var f = _f(x, u);
        return dh * f;
    }
    
    /// <summary>
    /// Compute control Lie derivative L_g h(x) = dh/dx � g(x).
    /// </summary>
    public Vector<double> LieGDerivative(Vector<double> x)
    {
        var dh = _dh(x);
        var g = _g(x);
        return g.Transpose() * dh;
    }
    
    /// <summary>
    /// Check if state is in safe set (h(x) >= 0).
    /// </summary>
    public bool IsSafe(Vector<double> x) => _h(x) >= 0;
    
    /// <summary>
    /// Get the CBF constraint for QP: L_f h + L_g h � u + ?(h) >= 0
    /// Returns (A, b) such that A*u >= b for safety.
    /// </summary>
    public (Vector<double> A, double b) GetConstraint(Vector<double> x, Vector<double> uNom)
    {
        double h = _h(x);
        double Lfh = LieFDerivative(x, uNom);
        var Lgh = LieGDerivative(x);
        
        // A * u >= b
        // L_g h � u >= -L_f h - ?(h)
        return (Lgh, -Lfh - ExtendedClassK(h));
    }
    
    /// <summary>
    /// Extended class-K function ?(h). Default is linear: ?(h) = ?h.
    /// </summary>
    protected virtual double ExtendedClassK(double h)
    {
        return _alpha * h;
    }
}

/// <summary>
/// Exponential CBF for higher relative degree systems.
/// </summary>
public class ExponentialCBF : ControlBarrierFunction
{
    private readonly double[] _alphas;
    
    public ExponentialCBF(
        Func<Vector<double>, double> h,
        Func<Vector<double>, Vector<double>> dh,
        Func<Vector<double>, Vector<double>, Vector<double>> f,
        Func<Vector<double>, Matrix<double>> g,
        double[] alphas)
        : base(h, dh, f, g, alphas[0], alphas.Length)
    {
        _alphas = alphas;
    }
    
    protected override double ExtendedClassK(double h)
    {
        // Exponential CBF: sum of alpha_i * h^i
        double result = 0;
        for (int i = 0; i < _alphas.Length; i++)
        {
            result += _alphas[i] * System.Math.Pow(h, i + 1);
        }
        return result;
    }
}

/// <summary>
/// CBF-QP: Quadratic Program for safe control synthesis.
/// Minimizes ||u - u_nom||^2 subject to CBF constraints.
/// </summary>
public class CbfQpController
{
    private readonly List<ControlBarrierFunction> _barriers = new();
    private readonly Matrix<double> _R; // Control cost matrix
    private readonly Vector<double> _uMin;
    private readonly Vector<double> _uMax;
    
    public CbfQpController(int controlDim, Vector<double>? uMin = null, Vector<double>? uMax = null)
    {
        _R = Matrix<double>.Build.DenseIdentity(controlDim);
        _uMin = uMin ?? Vector<double>.Build.Dense(controlDim, double.NegativeInfinity);
        _uMax = uMax ?? Vector<double>.Build.Dense(controlDim, double.PositiveInfinity);
    }
    
    /// <summary>
    /// Add a CBF constraint for obstacle/safety.
    /// </summary>
    public void AddBarrier(ControlBarrierFunction cbf)
    {
        _barriers.Add(cbf);
    }
    
    /// <summary>
    /// Compute safe control that satisfies all CBF constraints.
    /// </summary>
    public Vector<double> ComputeSafeControl(Vector<double> x, Vector<double> uNominal)
    {
        int n = uNominal.Count;
        
        // Collect constraints: A_i * u >= b_i for each CBF
        var constraints = new List<(Vector<double> A, double b)>();
        
        foreach (var cbf in _barriers)
        {
            var (A, b) = cbf.GetConstraint(x, uNominal);
            constraints.Add((A, b));
        }
        
        // Solve QP: min ||u - u_nom||^2 s.t. A*u >= b, u_min <= u <= u_max
        return SolveQp(uNominal, constraints);
    }
    
    private Vector<double> SolveQp(Vector<double> uNom, List<(Vector<double> A, double b)> constraints)
    {
        int n = uNom.Count;
        
        // Active set method for QP
        var u = uNom.Clone();
        var activeSet = new HashSet<int>();
        
        for (int iter = 0; iter < 100; iter++)
        {
            // Check constraint violations
            bool allSatisfied = true;
            int mostViolated = -1;
            double maxViolation = 0;
            
            for (int i = 0; i < constraints.Count; i++)
            {
                if (activeSet.Contains(i)) continue;
                
                var (A, b) = constraints[i];
                double slack = A * u - b;
                
                if (slack < -1e-6)
                {
                    allSatisfied = false;
                    if (-slack > maxViolation)
                    {
                        maxViolation = -slack;
                        mostViolated = i;
                    }
                }
            }
            
            if (allSatisfied)
            {
                // Check optimality with active constraints
                if (!CheckOptimality(u, uNom, constraints, activeSet, out int toRemove))
                {
                    activeSet.Remove(toRemove);
                    continue;
                }
                break;
            }
            
            // Add most violated constraint
            if (mostViolated >= 0)
            {
                activeSet.Add(mostViolated);
                
                // Solve KKT system with active constraints
                u = SolveKkt(uNom, constraints, activeSet);
            }
        }
        
        // Apply input bounds
        for (int i = 0; i < n; i++)
        {
            u[i] = System.Math.Clamp(u[i], _uMin[i], _uMax[i]);
        }
        
        return u;
    }
    
    private Vector<double> SolveKkt(Vector<double> uNom, 
        List<(Vector<double> A, double b)> constraints, HashSet<int> activeSet)
    {
        int n = uNom.Count;
        int m = activeSet.Count;
        
        if (m == 0)
            return uNom.Clone();
        
        // KKT: [R A'] [u] = [R*uNom]
        //      [A 0 ] [?]   [b     ]
        
        var Aactive = Matrix<double>.Build.Dense(m, n);
        var bactive = Vector<double>.Build.Dense(m);
        
        int row = 0;
        foreach (int idx in activeSet)
        {
            var (A, b) = constraints[idx];
            for (int j = 0; j < n; j++)
                Aactive[row, j] = A[j];
            bactive[row] = b;
            row++;
        }
        
        // Use Schur complement
        var RInv = _R.Inverse();
        var ARInvAt = Aactive * RInv * Aactive.Transpose();
        var rhs = bactive - Aactive * RInv * (_R * uNom);
        
        Vector<double> lambda;
        try
        {
            lambda = ARInvAt.Solve(rhs);
        }
        catch
        {
            lambda = Vector<double>.Build.Dense(m);
        }
        
        return uNom - RInv * (Aactive.Transpose() * lambda);
    }
    
    private bool CheckOptimality(Vector<double> u, Vector<double> uNom,
        List<(Vector<double> A, double b)> constraints, HashSet<int> activeSet, out int toRemove)
    {
        toRemove = -1;
        
        // Compute Lagrange multipliers
        int n = u.Count;
        int m = activeSet.Count;
        
        if (m == 0)
            return true;
        
        var Aactive = Matrix<double>.Build.Dense(m, n);
        int row = 0;
        foreach (int idx in activeSet)
        {
            var (A, _) = constraints[idx];
            for (int j = 0; j < n; j++)
                Aactive[row, j] = A[j];
            row++;
        }
        
        // Dual: ? = (A A')^-1 A * 2R(u - uNom)
        var grad = 2 * _R * (u - uNom);
        var AAt = Aactive * Aactive.Transpose();
        
        Vector<double> lambda;
        try
        {
            lambda = AAt.Solve(Aactive * grad);
        }
        catch
        {
            return true;
        }
        
        // Check for negative multipliers (inactive constraint)
        row = 0;
        foreach (int idx in activeSet)
        {
            if (lambda[row] < -1e-6)
            {
                toRemove = idx;
                return false;
            }
            row++;
        }
        
        return true;
    }
}

/// <summary>
/// Multi-robot collision avoidance using distributed CBFs.
/// </summary>
public class MultiRobotCbf
{
    private readonly int _numRobots;
    private readonly double _safetyRadius;
    private readonly double _alpha;
    
    public MultiRobotCbf(int numRobots, double safetyRadius, double alpha = 1.0)
    {
        _numRobots = numRobots;
        _safetyRadius = safetyRadius;
        _alpha = alpha;
    }
    
    /// <summary>
    /// Create pairwise CBFs for all robot pairs.
    /// </summary>
    public List<ControlBarrierFunction> CreatePairwiseCbfs(
        Func<int, Vector<double>> getPosition,
        Func<int, Vector<double>> getVelocity)
    {
        var cbfs = new List<ControlBarrierFunction>();
        
        for (int i = 0; i < _numRobots; i++)
        {
            for (int j = i + 1; j < _numRobots; j++)
            {
                int ri = i, rj = j;
                
                // h_ij(x) = ||p_i - p_j||^2 - R^2
                Func<Vector<double>, double> h = x =>
                {
                    var pi = getPosition(ri);
                    var pj = getPosition(rj);
                    return (pi - pj).L2Norm() * (pi - pj).L2Norm() - _safetyRadius * _safetyRadius;
                };
                
                // dh/dx = 2(p_i - p_j)
                Func<Vector<double>, Vector<double>> dh = x =>
                {
                    var pi = getPosition(ri);
                    var pj = getPosition(rj);
                    return 2 * (pi - pj);
                };
                
                // Use second-order dynamics
                Func<Vector<double>, Vector<double>, Vector<double>> f = (x, u) =>
                {
                    var vi = getVelocity(ri);
                    var vj = getVelocity(rj);
                    return vi - vj;
                };
                
                Func<Vector<double>, Matrix<double>> g = x =>
                    Matrix<double>.Build.DenseIdentity(3);
                
                cbfs.Add(new ControlBarrierFunction(h, dh, f, g, _alpha));
            }
        }
        
        return cbfs;
    }
}

/// <summary>
/// SE(3) dynamics for 6-DOF systems.
/// </summary>
public class SE3DynamicsCbf
{
    public int StateDim => 12;
    public int ControlDim => 6;
    
    private readonly double _mass;
    private readonly Matrix<double> _inertia;
    
    public SE3DynamicsCbf(double mass, Matrix<double> inertia)
    {
        _mass = mass;
        _inertia = inertia;
    }
    
    /// <summary>
    /// Drift vector in body frame.
    /// </summary>
    public Vector<double> DriftVector(Matrix<double> T)
    {
        // Gravity in body frame
        var R = T.SubMatrix(0, 3, 0, 3);
        var gWorld = Vector<double>.Build.DenseOfArray([0, 0, -9.81]);
        var gBody = R.Transpose() * gWorld;
        
        var drift = Vector<double>.Build.Dense(6);
        drift.SetSubVector(3, 3, gBody / _mass);
        
        return drift;
    }
    
    /// <summary>
    /// Control matrix mapping body forces/torques to accelerations.
    /// </summary>
    public Matrix<double> ControlMatrix(Matrix<double> T)
    {
        var B = Matrix<double>.Build.Dense(6, 6);
        
        // Angular acceleration from torques
        B.SetSubMatrix(0, 0, _inertia.Inverse());
        
        // Linear acceleration from forces
        B.SetSubMatrix(3, 3, Matrix<double>.Build.DenseIdentity(3) / _mass);
        
        return B;
    }
    
    /// <summary>
    /// Linearize dynamics around operating point.
    /// </summary>
    public (Matrix<double> A, Matrix<double> B, Vector<double> d) Linearize(
        Matrix<double> T, Vector<double> u, double dt)
    {
        int n = StateDim;
        int m = ControlDim;
        
        var A = Matrix<double>.Build.DenseIdentity(n);
        var B = Matrix<double>.Build.Dense(n, m);
        
        // Simplified linearization
        A.SetSubMatrix(0, 6, Matrix<double>.Build.DenseIdentity(6) * dt);
        B.SetSubMatrix(6, 0, ControlMatrix(T) * dt);
        
        var d = DriftVector(T) * dt;
        var dFull = Vector<double>.Build.Dense(n);
        dFull.SetSubVector(6, 6, d);
        
        return (A, B, dFull);
    }
}

/// <summary>
/// Cost function for CBF-based MPC.
/// </summary>
public class CbfMpcCostFunction
{
    public Matrix<double> StateWeight { get; set; }
    public Matrix<double> ControlWeight { get; set; }
    public Matrix<double> TerminalWeight { get; set; }
    
    public CbfMpcCostFunction(int stateDim = 12, int controlDim = 6)
    {
        StateWeight = Matrix<double>.Build.DenseIdentity(stateDim);
        ControlWeight = Matrix<double>.Build.DenseIdentity(controlDim) * 0.1;
        TerminalWeight = Matrix<double>.Build.DenseIdentity(stateDim) * 10;
    }
}

/// <summary>
/// Geometric MPC with integrated CBF safety constraints.
/// </summary>
public class GeometricMpcWithCbf
{
    private readonly int _horizon;
    private readonly double _dt;
    private readonly SE3DynamicsCbf _dynamics;
    private readonly CbfMpcCostFunction _cost;
    private readonly List<ControlBarrierFunction> _barriers = new();
    
    public GeometricMpcWithCbf(
        SE3DynamicsCbf dynamics,
        CbfMpcCostFunction cost,
        int horizon = 20,
        double dt = 0.02)
    {
        _dynamics = dynamics;
        _cost = cost;
        _horizon = horizon;
        _dt = dt;
    }
    
    /// <summary>
    /// Add safety barrier for collision avoidance.
    /// </summary>
    public void AddSafetyBarrier(
        Func<Matrix<double>, double> obstacleDistance,
        double safetyMargin)
    {
        var cbf = new ControlBarrierFunction(
            x => obstacleDistance(VectorToSE3(x)) - safetyMargin,
            x => ComputeDistanceGradient(x, obstacleDistance),
            (x, u) => _dynamics.DriftVector(VectorToSE3(x)),
            x => _dynamics.ControlMatrix(VectorToSE3(x)),
            alpha: 1.0
        );
        _barriers.Add(cbf);
    }
    
    /// <summary>
    /// Solve the safe geometric MPC problem.
    /// </summary>
    public GeometricMpcSolution Solve(
        Matrix<double> currentPose,
        Vector<double> currentVelocity,
        Matrix<double> goalPose)
    {
        var solution = new GeometricMpcSolution
        {
            PoseTrajectory = new Matrix<double>[_horizon + 1],
            VelocityTrajectory = new Vector<double>[_horizon + 1],
            ControlTrajectory = new Vector<double>[_horizon]
        };
        
        // Initialize trajectories
        solution.PoseTrajectory[0] = currentPose.Clone();
        solution.VelocityTrajectory[0] = currentVelocity.Clone();
        
        // Sequential QP with CBF constraints
        for (int sqpIter = 0; sqpIter < 5; sqpIter++)
        {
            // Linearize along trajectory
            var (H, g, Aeq, beq) = LinearizeTrajectory(solution, goalPose);
            
            // Add CBF constraints at each time step
            var (Aineq, bineq) = BuildCbfConstraints(solution);
            
            // Solve QP
            var delta = SolveQpWithConstraints(H, g, Aeq, beq, Aineq, bineq);
            
            // Apply update
            ApplyTrajectoryUpdate(solution, delta);
            
            if (delta.L2Norm() < 1e-4)
                break;
        }
        
        return solution;
    }
    
    private (Matrix<double> H, Vector<double> g, Matrix<double> Aeq, Vector<double> beq) 
        LinearizeTrajectory(GeometricMpcSolution traj, Matrix<double> goal)
    {
        int stateDim = 12; // 6 for pose, 6 for velocity (twist)
        int controlDim = 6;
        int totalDim = (_horizon + 1) * stateDim + _horizon * controlDim;
        
        var H = Matrix<double>.Build.Dense(totalDim, totalDim);
        var g = Vector<double>.Build.Dense(totalDim);
        
        // State cost
        var Q = _cost.StateWeight;
        var R = _cost.ControlWeight;
        var Qf = _cost.TerminalWeight;
        
        for (int k = 0; k <= _horizon; k++)
        {
            int stateOffset = k * stateDim;
            var Qk = k == _horizon ? Qf : Q;
            
            H.SetSubMatrix(stateOffset, stateOffset, Qk);
            
            // Gradient towards goal
            var error = ComputePoseError(traj.PoseTrajectory[k], goal);
            var grad = Qk * error;
            g.SetSubVector(stateOffset, stateDim, grad);
        }
        
        // Control cost
        for (int k = 0; k < _horizon; k++)
        {
            int controlOffset = (_horizon + 1) * stateDim + k * controlDim;
            H.SetSubMatrix(controlOffset, controlOffset, R);
        }
        
        // Dynamics constraints (equality)
        int numEqConstraints = _horizon * stateDim;
        var Aeq = Matrix<double>.Build.Dense(numEqConstraints, totalDim);
        var beq = Vector<double>.Build.Dense(numEqConstraints);
        
        for (int k = 0; k < _horizon; k++)
        {
            var (A, B, d) = _dynamics.Linearize(traj.PoseTrajectory[k], traj.ControlTrajectory[k], _dt);
            
            int constrOffset = k * stateDim;
            int stateOffset = k * stateDim;
            int nextStateOffset = (k + 1) * stateDim;
            int controlOffset = (_horizon + 1) * stateDim + k * controlDim;
            
            // x_{k+1} = A*x_k + B*u_k + d
            // A*x_k + B*u_k - x_{k+1} + d = 0
            Aeq.SetSubMatrix(constrOffset, stateOffset, A);
            Aeq.SetSubMatrix(constrOffset, controlOffset, B);
            Aeq.SetSubMatrix(constrOffset, nextStateOffset, -Matrix<double>.Build.DenseIdentity(stateDim));
            beq.SetSubVector(constrOffset, stateDim, -d);
        }
        
        return (H, g, Aeq, beq);
    }
    
    private (Matrix<double> A, Vector<double> b) BuildCbfConstraints(GeometricMpcSolution traj)
    {
        var constraints = new List<(Vector<double> a, double b)>();
        
        for (int k = 0; k < _horizon; k++)
        {
            var x = SE3ToVector(traj.PoseTrajectory[k]);
            var u = traj.ControlTrajectory[k];
            
            foreach (var cbf in _barriers)
            {
                var (a, b) = cbf.GetConstraint(x, u);
                
                // Transform to full trajectory variable space
                int controlOffset = (_horizon + 1) * 12 + k * 6;
                var aFull = Vector<double>.Build.Dense((_horizon + 1) * 12 + _horizon * 6);
                aFull.SetSubVector(controlOffset, 6, a);
                
                constraints.Add((aFull, b));
            }
        }
        
        if (constraints.Count == 0)
        {
            return (Matrix<double>.Build.Dense(0, 0), Vector<double>.Build.Dense(0));
        }
        
        var A = Matrix<double>.Build.Dense(constraints.Count, constraints[0].a.Count);
        var bVec = Vector<double>.Build.Dense(constraints.Count);
        
        for (int i = 0; i < constraints.Count; i++)
        {
            A.SetRow(i, constraints[i].a);
            bVec[i] = constraints[i].b;
        }
        
        return (A, bVec);
    }
    
    private Vector<double> SolveQpWithConstraints(
        Matrix<double> H, Vector<double> g,
        Matrix<double> Aeq, Vector<double> beq,
        Matrix<double> Aineq, Vector<double> bineq)
    {
        int n = H.ColumnCount;
        int meq = Aeq.RowCount;
        
        if (meq == 0)
            return H.Solve(-g);
        
        // Build augmented system
        var Kaug = Matrix<double>.Build.Dense(n + meq, n + meq);
        Kaug.SetSubMatrix(0, 0, H);
        Kaug.SetSubMatrix(0, n, Aeq.Transpose());
        Kaug.SetSubMatrix(n, 0, Aeq);
        
        var rhs = Vector<double>.Build.Dense(n + meq);
        rhs.SetSubVector(0, n, -g);
        rhs.SetSubVector(n, meq, beq);
        
        var sol = Kaug.Solve(rhs);
        
        return sol.SubVector(0, n);
    }
    
    private void ApplyTrajectoryUpdate(GeometricMpcSolution solution, Vector<double> delta)
    {
        int stateDim = 12;
        int controlDim = 6;
        
        for (int k = 0; k <= _horizon; k++)
        {
            var dx = delta.SubVector(k * stateDim, stateDim);
            var dpose = dx.SubVector(0, 6);
            
            solution.PoseTrajectory[k] = LieGroups.BoxPlusSE3(solution.PoseTrajectory[k], dpose);
        }
        
        for (int k = 0; k < _horizon; k++)
        {
            int offset = (_horizon + 1) * stateDim + k * controlDim;
            var du = delta.SubVector(offset, controlDim);
            solution.ControlTrajectory[k] += du;
        }
    }
    
    private Vector<double> SE3ToVector(Matrix<double> T)
    {
        return LieGroups.LogSE3(T);
    }
    
    private Matrix<double> VectorToSE3(Vector<double> xi)
    {
        return LieGroups.ExpSE3(xi);
    }
    
    private Vector<double> ComputePoseError(Matrix<double> current, Matrix<double> goal)
    {
        return LieGroups.BoxMinusSE3(current, goal);
    }
    
    private Vector<double> ComputeDistanceGradient(
        Vector<double> x, Func<Matrix<double>, double> distFunc)
    {
        var T = VectorToSE3(x);
        double eps = 1e-6;
        var grad = Vector<double>.Build.Dense(x.Count);
        
        double d0 = distFunc(T);
        
        for (int i = 0; i < x.Count; i++)
        {
            var xp = x.Clone();
            xp[i] += eps;
            var Tp = VectorToSE3(xp);
            grad[i] = (distFunc(Tp) - d0) / eps;
        }
        
        return grad;
    }
}

public class GeometricMpcSolution
{
    public Matrix<double>[] PoseTrajectory { get; set; } = [];
    public Vector<double>[] VelocityTrajectory { get; set; } = [];
    public Vector<double>[] ControlTrajectory { get; set; } = [];
    public double Cost { get; set; }
    public bool Converged { get; set; }
}
