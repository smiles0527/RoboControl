using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Geometry;

namespace ControlWorkbench.Drone.Trajectory;

/// <summary>
/// Differential Flatness-based Trajectory Generation for Quadrotors.
/// Generates dynamically feasible trajectories with guaranteed smoothness.
/// 
/// Based on:
/// - "Minimum Snap Trajectory Generation and Control for Quadrotors" (Mellinger & Kumar, 2011)
/// - "Polynomial Trajectory Planning for Aggressive Quadrotor Flight" (Richter et al., 2016)
/// </summary>
public class DifferentialFlatnessTrajectory
{
    private readonly int _order;        // Polynomial order
    private readonly int _derivative;   // Derivative to minimize (4 = snap)
    private readonly double[] _times;   // Segment times
    
    private Matrix<double>? _coeffsX;
    private Matrix<double>? _coeffsY;
    private Matrix<double>? _coeffsZ;
    private Matrix<double>? _coeffsYaw;
    
    public int SegmentCount => _times.Length - 1;
    public double TotalTime => _times[^1];
    
    /// <summary>
    /// Create trajectory generator with specified polynomial order.
    /// </summary>
    /// <param name="order">Polynomial order (typically 7-11)</param>
    /// <param name="derivative">Derivative to minimize (3=jerk, 4=snap)</param>
    public DifferentialFlatnessTrajectory(int order = 7, int derivative = 4)
    {
        _order = order;
        _derivative = derivative;
        _times = [];
    }
    
    /// <summary>
    /// Generate minimum-snap trajectory through waypoints.
    /// </summary>
    public void GenerateTrajectory(
        List<Waypoint3D> waypoints,
        double[]? segmentTimes = null,
        double maxVelocity = 5.0,
        double maxAcceleration = 10.0)
    {
        int n = waypoints.Count;
        if (n < 2)
            throw new ArgumentException("Need at least 2 waypoints");
        
        int numSegments = n - 1;
        
        // Compute segment times if not provided
        segmentTimes ??= ComputeSegmentTimes(waypoints, maxVelocity, maxAcceleration);
        
        // Convert to cumulative times
        var times = new double[n];
        times[0] = 0;
        for (int i = 0; i < numSegments; i++)
            times[i + 1] = times[i] + segmentTimes[i];
        
        // Extract waypoint positions
        var posX = waypoints.Select(w => w.Position[0]).ToArray();
        var posY = waypoints.Select(w => w.Position[1]).ToArray();
        var posZ = waypoints.Select(w => w.Position[2]).ToArray();
        var yaw = waypoints.Select(w => w.Yaw).ToArray();
        
        // Generate minimum-snap polynomials
        _coeffsX = SolveMinimumDerivative(times, posX, waypoints, 0);
        _coeffsY = SolveMinimumDerivative(times, posY, waypoints, 1);
        _coeffsZ = SolveMinimumDerivative(times, posZ, waypoints, 2);
        _coeffsYaw = SolveMinimumYaw(times, yaw, waypoints);
    }
    
    /// <summary>
    /// Evaluate trajectory at time t.
    /// </summary>
    public TrajectoryState Evaluate(double t)
    {
        if (_coeffsX == null)
            throw new InvalidOperationException("Trajectory not generated");
        
        t = System.Math.Clamp(t, 0, TotalTime);
        
        // Find segment
        int seg = 0;
        for (int i = 0; i < SegmentCount; i++)
        {
            if (t >= _times[i] && t <= _times[i + 1])
            {
                seg = i;
                break;
            }
        }
        
        double tau = t - _times[seg];
        
        // Evaluate position, velocity, acceleration, jerk
        var state = new TrajectoryState();
        
        state.Position = Vector<double>.Build.DenseOfArray([
            EvaluatePolynomial(_coeffsX!.Row(seg), tau, 0),
            EvaluatePolynomial(_coeffsY!.Row(seg), tau, 0),
            EvaluatePolynomial(_coeffsZ!.Row(seg), tau, 0)
        ]);
        
        state.Velocity = Vector<double>.Build.DenseOfArray([
            EvaluatePolynomial(_coeffsX.Row(seg), tau, 1),
            EvaluatePolynomial(_coeffsY.Row(seg), tau, 1),
            EvaluatePolynomial(_coeffsZ.Row(seg), tau, 1)
        ]);
        
        state.Acceleration = Vector<double>.Build.DenseOfArray([
            EvaluatePolynomial(_coeffsX.Row(seg), tau, 2),
            EvaluatePolynomial(_coeffsY.Row(seg), tau, 2),
            EvaluatePolynomial(_coeffsZ.Row(seg), tau, 2)
        ]);
        
        state.Jerk = Vector<double>.Build.DenseOfArray([
            EvaluatePolynomial(_coeffsX.Row(seg), tau, 3),
            EvaluatePolynomial(_coeffsY.Row(seg), tau, 3),
            EvaluatePolynomial(_coeffsZ.Row(seg), tau, 3)
        ]);
        
        state.Snap = Vector<double>.Build.DenseOfArray([
            EvaluatePolynomial(_coeffsX.Row(seg), tau, 4),
            EvaluatePolynomial(_coeffsY.Row(seg), tau, 4),
            EvaluatePolynomial(_coeffsZ.Row(seg), tau, 4)
        ]);
        
        state.Yaw = EvaluatePolynomial(_coeffsYaw!.Row(seg), tau, 0);
        state.YawRate = EvaluatePolynomial(_coeffsYaw.Row(seg), tau, 1);
        
        // Compute flat outputs ? full state
        ComputeFullState(state);
        
        state.Time = t;
        return state;
    }
    
    /// <summary>
    /// Sample trajectory at uniform time intervals.
    /// </summary>
    public List<TrajectoryState> Sample(double dt)
    {
        var samples = new List<TrajectoryState>();
        
        for (double t = 0; t <= TotalTime; t += dt)
        {
            samples.Add(Evaluate(t));
        }
        
        return samples;
    }
    
    /// <summary>
    /// Compute full quadrotor state from flat outputs using differential flatness.
    /// </summary>
    private void ComputeFullState(TrajectoryState state)
    {
        const double g = 9.81;
        
        // Thrust direction from acceleration
        var zBody = state.Acceleration + Vector<double>.Build.DenseOfArray([0, 0, g]);
        double thrust = zBody.L2Norm();
        zBody = zBody.Normalize(2);
        
        state.Thrust = thrust;
        
        // Compute orientation from thrust direction and yaw
        var xC = Vector<double>.Build.DenseOfArray([
            System.Math.Cos(state.Yaw),
            System.Math.Sin(state.Yaw),
            0
        ]);
        
        var yBody = Cross(zBody, xC).Normalize(2);
        var xBody = Cross(yBody, zBody);
        
        // Rotation matrix
        state.Rotation = Matrix<double>.Build.DenseOfColumnVectors(xBody, yBody, zBody);
        state.Quaternion = LieGroups.MatrixToQuaternion(state.Rotation);
        
        // Angular velocity from jerk
        var h = (state.Jerk - (zBody * state.Jerk) * zBody) / thrust;
        double p = -h * yBody;
        double q = h * xBody;
        
        // Yaw rate contribution
        double r = state.YawRate * zBody[2];
        
        state.AngularVelocity = Vector<double>.Build.DenseOfArray([p, q, r]);
        
        // Angular acceleration from snap (for feedforward)
        var hDot = (state.Snap - (zBody * state.Snap) * zBody - 2 * (zBody * state.Jerk) * h) / thrust;
        double pDot = -hDot * yBody - h * Cross(state.AngularVelocity, yBody);
        double qDot = hDot * xBody + h * Cross(state.AngularVelocity, xBody);
        
        state.AngularAcceleration = Vector<double>.Build.DenseOfArray([pDot, qDot, 0]);
    }
    
    private Matrix<double> SolveMinimumDerivative(
        double[] times, double[] positions, List<Waypoint3D> waypoints, int dim)
    {
        int n = positions.Length;
        int numSegments = n - 1;
        int numCoeffs = _order + 1;
        
        // Total unknowns: numSegments * numCoeffs
        int totalUnknowns = numSegments * numCoeffs;
        
        // Build cost matrix (minimize integral of derivative^2)
        var Q = BuildCostMatrix(times);
        
        // Build constraint matrices
        var (Aeq, beq) = BuildConstraintMatrices(times, positions, waypoints, dim);
        
        // Solve constrained QP: min 0.5*x'Qx s.t. Aeq*x = beq
        var coeffs = SolveConstrainedQP(Q, Aeq, beq);
        
        // Reshape to matrix (segments × coefficients)
        var result = Matrix<double>.Build.Dense(numSegments, numCoeffs);
        for (int s = 0; s < numSegments; s++)
        {
            for (int c = 0; c < numCoeffs; c++)
            {
                result[s, c] = coeffs[s * numCoeffs + c];
            }
        }
        
        return result;
    }
    
    private Matrix<double> SolveMinimumYaw(
        double[] times, double[] yaw, List<Waypoint3D> waypoints)
    {
        // Yaw is typically lower order (5th or 7th)
        int yawOrder = System.Math.Min(_order, 5);
        int n = yaw.Length;
        int numSegments = n - 1;
        int numCoeffs = yawOrder + 1;
        
        // Simple interpolation for yaw
        var result = Matrix<double>.Build.Dense(numSegments, _order + 1);
        
        for (int s = 0; s < numSegments; s++)
        {
            double T = times[s + 1] - times[s];
            double y0 = yaw[s];
            double y1 = yaw[s + 1];
            
            // Unwrap yaw angle
            while (y1 - y0 > System.Math.PI) y1 -= 2 * System.Math.PI;
            while (y1 - y0 < -System.Math.PI) y1 += 2 * System.Math.PI;
            
            // Linear interpolation (could be improved with minimum-jerk)
            result[s, 0] = y0;
            result[s, 1] = (y1 - y0) / T;
        }
        
        return result;
    }
    
    private Matrix<double> BuildCostMatrix(double[] times)
    {
        int numSegments = times.Length - 1;
        int numCoeffs = _order + 1;
        int totalSize = numSegments * numCoeffs;
        
        var Q = Matrix<double>.Build.Dense(totalSize, totalSize);
        
        for (int s = 0; s < numSegments; s++)
        {
            double T = times[s + 1] - times[s];
            
            // Cost matrix for minimizing integral of derivative^2
            for (int i = _derivative; i <= _order; i++)
            {
                for (int j = _derivative; j <= _order; j++)
                {
                    double coeff = Factorial(i) / Factorial(i - _derivative) *
                                   Factorial(j) / Factorial(j - _derivative);
                    int power = i + j - 2 * _derivative + 1;
                    double integral = coeff * System.Math.Pow(T, power) / power;
                    
                    int row = s * numCoeffs + i;
                    int col = s * numCoeffs + j;
                    Q[row, col] = integral;
                }
            }
        }
        
        return Q;
    }
    
    private (Matrix<double> Aeq, Vector<double> beq) BuildConstraintMatrices(
        double[] times, double[] positions, List<Waypoint3D> waypoints, int dim)
    {
        int n = positions.Length;
        int numSegments = n - 1;
        int numCoeffs = _order + 1;
        
        var constraints = new List<(Vector<double> a, double b)>();
        
        // Position constraints at waypoints
        for (int i = 0; i < n; i++)
        {
            if (i == 0)
            {
                // Start of first segment
                var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
                a[0] = 1; // c0
                constraints.Add((a, positions[i]));
            }
            else if (i == n - 1)
            {
                // End of last segment
                double T = times[i] - times[i - 1];
                var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
                for (int c = 0; c <= _order; c++)
                {
                    a[(numSegments - 1) * numCoeffs + c] = System.Math.Pow(T, c);
                }
                constraints.Add((a, positions[i]));
            }
            else
            {
                // Interior waypoint: end of segment i-1 = start of segment i = position
                double T = times[i] - times[i - 1];
                
                // End of segment i-1
                var a1 = Vector<double>.Build.Dense(numSegments * numCoeffs);
                for (int c = 0; c <= _order; c++)
                {
                    a1[(i - 1) * numCoeffs + c] = System.Math.Pow(T, c);
                }
                constraints.Add((a1, positions[i]));
                
                // Start of segment i
                var a2 = Vector<double>.Build.Dense(numSegments * numCoeffs);
                a2[i * numCoeffs + 0] = 1;
                constraints.Add((a2, positions[i]));
            }
        }
        
        // Continuity constraints at interior waypoints (velocity, acceleration, jerk, snap)
        for (int i = 1; i < n - 1; i++)
        {
            double T = times[i] - times[i - 1];
            
            for (int d = 1; d <= System.Math.Min(_derivative, _order - 1); d++)
            {
                var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
                
                // End of segment i-1
                for (int c = d; c <= _order; c++)
                {
                    double coeff = Factorial(c) / Factorial(c - d) * System.Math.Pow(T, c - d);
                    a[(i - 1) * numCoeffs + c] = coeff;
                }
                
                // Start of segment i (negative)
                a[i * numCoeffs + d] = -Factorial(d);
                
                constraints.Add((a, 0));
            }
        }
        
        // Endpoint constraints (velocity = 0, acceleration = 0 at start/end)
        // Start velocity
        {
            var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
            a[1] = 1; // c1 (derivative of c0 + c1*t + ...)
            var vel = waypoints[0].Velocity;
            constraints.Add((a, vel != null ? vel[dim] : 0));
        }
        
        // Start acceleration
        {
            var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
            a[2] = 2; // 2*c2
            var acc = waypoints[0].Acceleration;
            constraints.Add((a, acc != null ? acc[dim] : 0));
        }
        
        // End velocity
        {
            double T = times[^1] - times[^2];
            var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
            for (int c = 1; c <= _order; c++)
            {
                a[(numSegments - 1) * numCoeffs + c] = c * System.Math.Pow(T, c - 1);
            }
            var vel = waypoints[^1].Velocity;
            constraints.Add((a, vel != null ? vel[dim] : 0));
        }
        
        // End acceleration
        {
            double T = times[^1] - times[^2];
            var a = Vector<double>.Build.Dense(numSegments * numCoeffs);
            for (int c = 2; c <= _order; c++)
            {
                a[(numSegments - 1) * numCoeffs + c] = c * (c - 1) * System.Math.Pow(T, c - 2);
            }
            var acc = waypoints[^1].Acceleration;
            constraints.Add((a, acc != null ? acc[dim] : 0));
        }
        
        // Convert to matrix form
        var Aeq = Matrix<double>.Build.Dense(constraints.Count, numSegments * numCoeffs);
        var beq = Vector<double>.Build.Dense(constraints.Count);
        
        for (int i = 0; i < constraints.Count; i++)
        {
            Aeq.SetRow(i, constraints[i].a);
            beq[i] = constraints[i].b;
        }
        
        return (Aeq, beq);
    }
    
    private Vector<double> SolveConstrainedQP(Matrix<double> Q, Matrix<double> Aeq, Vector<double> beq)
    {
        int n = Q.ColumnCount;
        int m = Aeq.RowCount;
        
        // Regularize Q
        Q = Q + Matrix<double>.Build.DenseIdentity(n) * 1e-6;
        
        // KKT system: [Q A'] [x]   [0  ]
        //             [A 0 ] [?] = [beq]
        
        var KKT = Matrix<double>.Build.Dense(n + m, n + m);
        KKT.SetSubMatrix(0, 0, Q);
        KKT.SetSubMatrix(0, n, Aeq.Transpose());
        KKT.SetSubMatrix(n, 0, Aeq);
        
        var rhs = Vector<double>.Build.Dense(n + m);
        rhs.SetSubVector(n, m, beq);
        
        var solution = KKT.Solve(rhs);
        
        return solution.SubVector(0, n);
    }
    
    private double EvaluatePolynomial(Vector<double> coeffs, double t, int derivative)
    {
        double result = 0;
        
        for (int i = derivative; i < coeffs.Count; i++)
        {
            double coeff = coeffs[i] * Factorial(i) / Factorial(i - derivative);
            result += coeff * System.Math.Pow(t, i - derivative);
        }
        
        return result;
    }
    
    private double[] ComputeSegmentTimes(List<Waypoint3D> waypoints, double maxVel, double maxAcc)
    {
        int n = waypoints.Count - 1;
        var times = new double[n];
        
        for (int i = 0; i < n; i++)
        {
            double dist = (waypoints[i + 1].Position - waypoints[i].Position).L2Norm();
            
            // Time = max(distance/velocity, sqrt(distance/acceleration))
            double tVel = dist / maxVel;
            double tAcc = System.Math.Sqrt(2 * dist / maxAcc);
            times[i] = System.Math.Max(tVel, tAcc);
            times[i] = System.Math.Max(times[i], 0.1); // Minimum time
        }
        
        return times;
    }
    
    private static double Factorial(int n)
    {
        if (n <= 0) return 1;
        double result = 1;
        for (int i = 2; i <= n; i++)
            result *= i;
        return result;
    }
    
    private static Vector<double> Cross(Vector<double> a, Vector<double> b)
    {
        return Vector<double>.Build.DenseOfArray([
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ]);
    }
}

/// <summary>
/// Bezier curve-based trajectory with guaranteed collision avoidance.
/// Uses convex hull property for safe corridor generation.
/// </summary>
public class BezierTrajectory
{
    private readonly int _degree;
    private List<Vector<double>[]> _segments = new();
    private double[] _segmentTimes = [];
    
    public BezierTrajectory(int degree = 7)
    {
        _degree = degree;
    }
    
    /// <summary>
    /// Generate trajectory through safe flight corridors.
    /// </summary>
    public void GenerateThroughCorridors(
        List<SafeFlightCorridor> corridors,
        Vector<double> startPos,
        Vector<double> endPos,
        Vector<double>? startVel = null,
        Vector<double>? endVel = null)
    {
        int numSegments = corridors.Count;
        _segments = new List<Vector<double>[]>(numSegments);
        _segmentTimes = new double[numSegments];
        
        // Compute segment times based on corridor sizes
        for (int i = 0; i < numSegments; i++)
        {
            _segmentTimes[i] = corridors[i].SuggestedTime;
        }
        
        // Optimize control points to stay within corridors
        for (int seg = 0; seg < numSegments; seg++)
        {
            var controlPoints = new Vector<double>[_degree + 1];
            var corridor = corridors[seg];
            
            // Initialize control points along corridor center
            for (int i = 0; i <= _degree; i++)
            {
                double t = (double)i / _degree;
                controlPoints[i] = (1 - t) * corridor.Entry + t * corridor.Exit;
            }
            
            // Constrain endpoints
            if (seg == 0)
                controlPoints[0] = startPos;
            if (seg == numSegments - 1)
                controlPoints[_degree] = endPos;
            
            // Optimize control points to minimize snap while staying in corridor
            controlPoints = OptimizeControlPoints(controlPoints, corridor);
            
            _segments.Add(controlPoints);
        }
        
        // Ensure C2 continuity between segments
        EnforceContinuity();
    }
    
    /// <summary>
    /// Evaluate trajectory at time t.
    /// </summary>
    public (Vector<double> position, Vector<double> velocity, Vector<double> acceleration) Evaluate(double t)
    {
        // Find segment
        double cumTime = 0;
        int seg = 0;
        double localT = 0;
        
        for (int i = 0; i < _segmentTimes.Length; i++)
        {
            if (t <= cumTime + _segmentTimes[i])
            {
                seg = i;
                localT = (t - cumTime) / _segmentTimes[i];
                break;
            }
            cumTime += _segmentTimes[i];
        }
        
        localT = System.Math.Clamp(localT, 0, 1);
        
        var points = _segments[seg];
        double T = _segmentTimes[seg];
        
        // De Casteljau's algorithm for position
        var pos = DeCasteljau(points, localT);
        
        // Velocity: derivative of Bezier curve
        var velPoints = new Vector<double>[_degree];
        for (int i = 0; i < _degree; i++)
        {
            velPoints[i] = _degree * (points[i + 1] - points[i]) / T;
        }
        var vel = DeCasteljau(velPoints, localT);
        
        // Acceleration: second derivative
        var accPoints = new Vector<double>[_degree - 1];
        for (int i = 0; i < _degree - 1; i++)
        {
            accPoints[i] = (_degree - 1) * (velPoints[i + 1] - velPoints[i]) / T;
        }
        var acc = DeCasteljau(accPoints, localT);
        
        return (pos, vel, acc);
    }
    
    /// <summary>
    /// Check if entire trajectory is collision-free.
    /// Uses convex hull property of Bezier curves.
    /// </summary>
    public bool IsCollisionFree(List<SafeFlightCorridor> corridors)
    {
        for (int seg = 0; seg < _segments.Count; seg++)
        {
            var points = _segments[seg];
            var corridor = corridors[seg];
            
            // All control points must be inside corridor
            foreach (var point in points)
            {
                if (!corridor.Contains(point))
                    return false;
            }
        }
        
        return true;
    }
    
    private Vector<double>[] OptimizeControlPoints(Vector<double>[] initial, SafeFlightCorridor corridor)
    {
        var points = initial.Select(p => p.Clone()).ToArray();
        
        // Gradient descent to minimize snap while staying in corridor
        for (int iter = 0; iter < 100; iter++)
        {
            // Compute snap cost gradient
            var gradients = new Vector<double>[_degree + 1];
            for (int i = 0; i <= _degree; i++)
            {
                gradients[i] = Vector<double>.Build.Dense(3);
            }
            
            // Snap = fourth derivative, affects control points
            for (int i = 2; i <= _degree - 2; i++)
            {
                var snap = points[i - 2] - 4 * points[i - 1] + 6 * points[i] - 4 * points[i + 1] + points[i + 2];
                gradients[i] += 6 * snap;
            }
            
            // Update with projection onto corridor
            for (int i = 1; i < _degree; i++) // Don't move endpoints
            {
                points[i] -= 0.1 * gradients[i];
                points[i] = corridor.Project(points[i]);
            }
        }
        
        return points;
    }
    
    private void EnforceContinuity()
    {
        for (int seg = 0; seg < _segments.Count - 1; seg++)
        {
            var current = _segments[seg];
            var next = _segments[seg + 1];
            double T1 = _segmentTimes[seg];
            double T2 = _segmentTimes[seg + 1];
            
            // C0: position continuity
            next[0] = current[_degree];
            
            // C1: velocity continuity
            var vel1 = _degree * (current[_degree] - current[_degree - 1]) / T1;
            next[1] = next[0] + vel1 * T2 / _degree;
            
            // C2: acceleration continuity
            var acc1 = _degree * (_degree - 1) * (current[_degree] - 2 * current[_degree - 1] + current[_degree - 2]) / (T1 * T1);
            next[2] = 2 * next[1] - next[0] + acc1 * T2 * T2 / (_degree * (_degree - 1));
        }
    }
    
    private Vector<double> DeCasteljau(Vector<double>[] points, double t)
    {
        int n = points.Length;
        var work = points.Select(p => p.Clone()).ToArray();
        
        for (int r = 1; r < n; r++)
        {
            for (int i = 0; i < n - r; i++)
            {
                work[i] = (1 - t) * work[i] + t * work[i + 1];
            }
        }
        
        return work[0];
    }
}

/// <summary>
/// Safe flight corridor for collision avoidance.
/// </summary>
public class SafeFlightCorridor
{
    public Vector<double> Entry { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Exit { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Center { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> HalfExtents { get; set; } = Vector<double>.Build.Dense(3);
    public double SuggestedTime { get; set; } = 1.0;
    
    public bool Contains(Vector<double> point)
    {
        var local = point - Center;
        return System.Math.Abs(local[0]) <= HalfExtents[0] &&
               System.Math.Abs(local[1]) <= HalfExtents[1] &&
               System.Math.Abs(local[2]) <= HalfExtents[2];
    }
    
    public Vector<double> Project(Vector<double> point)
    {
        var projected = point.Clone();
        for (int i = 0; i < 3; i++)
        {
            double local = point[i] - Center[i];
            projected[i] = Center[i] + System.Math.Clamp(local, -HalfExtents[i], HalfExtents[i]);
        }
        return projected;
    }
}

/// <summary>
/// 3D waypoint with optional velocity/acceleration constraints.
/// </summary>
public class Waypoint3D
{
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double>? Velocity { get; set; }
    public Vector<double>? Acceleration { get; set; }
    public double Yaw { get; set; }
    public double? YawRate { get; set; }
    
    public Waypoint3D() { }
    
    public Waypoint3D(double x, double y, double z, double yaw = 0)
    {
        Position = Vector<double>.Build.DenseOfArray([x, y, z]);
        Yaw = yaw;
    }
}

/// <summary>
/// Full trajectory state including orientation and angular rates.
/// </summary>
public class TrajectoryState
{
    public double Time { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Acceleration { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Jerk { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Snap { get; set; } = Vector<double>.Build.Dense(3);
    
    public double Yaw { get; set; }
    public double YawRate { get; set; }
    
    public Matrix<double> Rotation { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    public Quaternion Quaternion { get; set; } = Quaternion.Identity;
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> AngularAcceleration { get; set; } = Vector<double>.Build.Dense(3);
    
    public double Thrust { get; set; }
}

/// <summary>
/// Time-optimal trajectory generation with velocity/acceleration limits.
/// </summary>
public class TimeOptimalTrajectory
{
    private readonly double _maxVelocity;
    private readonly double _maxAcceleration;
    private readonly double _maxJerk;
    
    public TimeOptimalTrajectory(
        double maxVelocity = 5.0,
        double maxAcceleration = 10.0,
        double maxJerk = 50.0)
    {
        _maxVelocity = maxVelocity;
        _maxAcceleration = maxAcceleration;
        _maxJerk = maxJerk;
    }
    
    /// <summary>
    /// Compute time-optimal trajectory using time allocation optimization.
    /// </summary>
    public (double[] times, DifferentialFlatnessTrajectory trajectory) Optimize(
        List<Waypoint3D> waypoints,
        int maxIterations = 50)
    {
        int n = waypoints.Count - 1;
        var times = new double[n];
        
        // Initial time allocation
        for (int i = 0; i < n; i++)
        {
            double dist = (waypoints[i + 1].Position - waypoints[i].Position).L2Norm();
            times[i] = dist / _maxVelocity;
        }
        
        // Gradient descent on total time
        for (int iter = 0; iter < maxIterations; iter++)
        {
            var traj = new DifferentialFlatnessTrajectory();
            traj.GenerateTrajectory(waypoints, times, _maxVelocity, _maxAcceleration);
            
            // Check constraints
            var (maxVel, maxAcc, maxJerk) = ComputeMaxDerivatives(traj);
            
            // Adjust times based on constraint violations
            bool feasible = true;
            for (int i = 0; i < n; i++)
            {
                double scaleFactor = 1.0;
                
                if (maxVel[i] > _maxVelocity)
                {
                    scaleFactor = System.Math.Max(scaleFactor, maxVel[i] / _maxVelocity);
                    feasible = false;
                }
                if (maxAcc[i] > _maxAcceleration)
                {
                    scaleFactor = System.Math.Max(scaleFactor, System.Math.Sqrt(maxAcc[i] / _maxAcceleration));
                    feasible = false;
                }
                if (maxJerk[i] > _maxJerk)
                {
                    scaleFactor = System.Math.Max(scaleFactor, System.Math.Pow(maxJerk[i] / _maxJerk, 1.0 / 3));
                    feasible = false;
                }
                
                times[i] *= scaleFactor;
            }
            
            if (feasible)
            {
                // Try to reduce times
                for (int i = 0; i < n; i++)
                {
                    times[i] *= 0.95;
                }
            }
        }
        
        var finalTraj = new DifferentialFlatnessTrajectory();
        finalTraj.GenerateTrajectory(waypoints, times, _maxVelocity, _maxAcceleration);
        
        return (times, finalTraj);
    }
    
    private (double[] maxVel, double[] maxAcc, double[] maxJerk) ComputeMaxDerivatives(
        DifferentialFlatnessTrajectory traj)
    {
        int n = traj.SegmentCount;
        var maxVel = new double[n];
        var maxAcc = new double[n];
        var maxJerk = new double[n];
        
        var samples = traj.Sample(0.01);
        double cumTime = 0;
        int seg = 0;
        
        foreach (var state in samples)
        {
            if (state.Time > cumTime + traj.TotalTime / n && seg < n - 1)
            {
                cumTime += traj.TotalTime / n;
                seg++;
            }
            
            maxVel[seg] = System.Math.Max(maxVel[seg], state.Velocity.L2Norm());
            maxAcc[seg] = System.Math.Max(maxAcc[seg], state.Acceleration.L2Norm());
            maxJerk[seg] = System.Math.Max(maxJerk[seg], state.Jerk.L2Norm());
        }
        
        return (maxVel, maxAcc, maxJerk);
    }
}
