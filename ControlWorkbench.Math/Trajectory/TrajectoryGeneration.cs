using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Trajectory;

/// <summary>
/// Trajectory planning and generation utilities.
/// </summary>
public static class TrajectoryPlanner
{
    /// <summary>
    /// Generates a minimum-jerk trajectory between two points.
    /// </summary>
    public static (double[] positions, double[] velocities, double[] accelerations) MinimumJerk(
        double start, double end, double duration, int numPoints)
    {
        var positions = new double[numPoints];
        var velocities = new double[numPoints];
        var accelerations = new double[numPoints];
        
        double dt = duration / (numPoints - 1);
        
        for (int i = 0; i < numPoints; i++)
        {
            double t = i * dt;
            double tau = t / duration;
            double tau2 = tau * tau;
            double tau3 = tau2 * tau;
            double tau4 = tau3 * tau;
            double tau5 = tau4 * tau;
            
            // Position: x(t) = x0 + (xf - x0) * (10*tau^3 - 15*tau^4 + 6*tau^5)
            double s = 10 * tau3 - 15 * tau4 + 6 * tau5;
            positions[i] = start + (end - start) * s;
            
            // Velocity
            double sDot = (30 * tau2 - 60 * tau3 + 30 * tau4) / duration;
            velocities[i] = (end - start) * sDot;
            
            // Acceleration
            double sDDot = (60 * tau - 180 * tau2 + 120 * tau3) / (duration * duration);
            accelerations[i] = (end - start) * sDDot;
        }
        
        return (positions, velocities, accelerations);
    }

    /// <summary>
    /// Generates a cubic polynomial trajectory.
    /// </summary>
    public static double[] CubicPolynomial(
        double q0, double qf, double v0, double vf, double duration, int numPoints)
    {
        // Solve for coefficients
        // q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        double T = duration;
        double T2 = T * T;
        double T3 = T2 * T;
        
        double a0 = q0;
        double a1 = v0;
        double a2 = (3 * (qf - q0) - (2 * v0 + vf) * T) / T2;
        double a3 = (2 * (q0 - qf) + (v0 + vf) * T) / T3;
        
        var result = new double[numPoints];
        double dt = duration / (numPoints - 1);
        
        for (int i = 0; i < numPoints; i++)
        {
            double t = i * dt;
            result[i] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
        }
        
        return result;
    }

    /// <summary>
    /// Generates a quintic polynomial trajectory.
    /// </summary>
    public static (double[] pos, double[] vel, double[] acc) QuinticPolynomial(
        double q0, double qf, double v0, double vf, double a0, double af, 
        double duration, int numPoints)
    {
        double T = duration;
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;
        
        // Solve the system for coefficients
        double c0 = q0;
        double c1 = v0;
        double c2 = a0 / 2;
        double c3 = (20 * qf - 20 * q0 - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) / (2 * T3);
        double c4 = (30 * q0 - 30 * qf + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) / (2 * T4);
        double c5 = (12 * qf - 12 * q0 - (6 * vf + 6 * v0) * T - (a0 - af) * T2) / (2 * T5);
        
        var pos = new double[numPoints];
        var vel = new double[numPoints];
        var acc = new double[numPoints];
        double dt = duration / (numPoints - 1);
        
        for (int i = 0; i < numPoints; i++)
        {
            double t = i * dt;
            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;
            
            pos[i] = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
            vel[i] = c1 + 2 * c2 * t + 3 * c3 * t2 + 4 * c4 * t3 + 5 * c5 * t4;
            acc[i] = 2 * c2 + 6 * c3 * t + 12 * c4 * t2 + 20 * c5 * t3;
        }
        
        return (pos, vel, acc);
    }

    /// <summary>
    /// Generates a trapezoidal velocity profile trajectory.
    /// </summary>
    public static (double[] pos, double[] vel) TrapezoidalVelocity(
        double q0, double qf, double vMax, double aMax, double dt)
    {
        double distance = System.Math.Abs(qf - q0);
        double sign = System.Math.Sign(qf - q0);
        
        // Time to accelerate/decelerate
        double ta = vMax / aMax;
        
        // Distance covered during acceleration
        double da = 0.5 * aMax * ta * ta;
        
        double T, tc;
        
        if (2 * da >= distance)
        {
            // Triangular profile (can't reach vMax)
            ta = System.Math.Sqrt(distance / aMax);
            tc = 0;
            T = 2 * ta;
        }
        else
        {
            // Trapezoidal profile
            double dc = distance - 2 * da;
            tc = dc / vMax;
            T = 2 * ta + tc;
        }
        
        int numPoints = (int)(T / dt) + 1;
        var pos = new double[numPoints];
        var vel = new double[numPoints];
        
        for (int i = 0; i < numPoints; i++)
        {
            double t = i * dt;
            
            if (t <= ta)
            {
                // Acceleration phase
                vel[i] = sign * aMax * t;
                pos[i] = q0 + sign * 0.5 * aMax * t * t;
            }
            else if (t <= ta + tc)
            {
                // Constant velocity phase
                vel[i] = sign * vMax;
                pos[i] = q0 + sign * (da + vMax * (t - ta));
            }
            else if (t <= T)
            {
                // Deceleration phase
                double td = t - ta - tc;
                vel[i] = sign * (vMax - aMax * td);
                pos[i] = q0 + sign * (da + vMax * tc + vMax * td - 0.5 * aMax * td * td);
            }
            else
            {
                vel[i] = 0;
                pos[i] = qf;
            }
        }
        
        return (pos, vel);
    }

    /// <summary>
    /// S-curve (7-segment) velocity profile for smooth motion.
    /// </summary>
    public static (double[] pos, double[] vel, double[] acc, double[] jerk) SCurveProfile(
        double distance, double vMax, double aMax, double jMax, double dt)
    {
        // Time for jerk phase
        double tj = aMax / jMax;
        
        // Time for constant acceleration
        double ta = vMax / aMax - tj;
        if (ta < 0) ta = 0;
        
        // Simplified calculation
        double T = 2 * tj + 2 * ta + distance / vMax;
        int numPoints = (int)(T / dt) + 1;
        
        var pos = new double[numPoints];
        var vel = new double[numPoints];
        var acc = new double[numPoints];
        var jerk = new double[numPoints];
        
        // Simplified implementation - integrate the jerk profile
        double currentPos = 0;
        double currentVel = 0;
        double currentAcc = 0;
        
        for (int i = 0; i < numPoints; i++)
        {
            double t = i * dt;
            double phase = t / T;
            
            // Simplified S-curve approximation
            double s = 0.5 * (1 - System.Math.Cos(System.Math.PI * phase));
            
            pos[i] = s * distance;
            vel[i] = (System.Math.PI / T) * System.Math.Sin(System.Math.PI * phase) * distance / 2;
            acc[i] = (System.Math.PI * System.Math.PI / (T * T)) * System.Math.Cos(System.Math.PI * phase) * distance / 2;
            jerk[i] = -(System.Math.PI * System.Math.PI * System.Math.PI / (T * T * T)) * System.Math.Sin(System.Math.PI * phase) * distance / 2;
        }
        
        return (pos, vel, acc, jerk);
    }
}

/// <summary>
/// Bezier curve trajectory generation.
/// </summary>
public class BezierTrajectory
{
    private readonly Vector<double>[] _controlPoints;
    
    public BezierTrajectory(Vector<double>[] controlPoints)
    {
        _controlPoints = controlPoints;
    }
    
    /// <summary>
    /// Evaluates the Bezier curve at parameter t in [0, 1].
    /// </summary>
    public Vector<double> Evaluate(double t)
    {
        int n = _controlPoints.Length - 1;
        var result = Vector<double>.Build.Dense(_controlPoints[0].Count);
        
        for (int i = 0; i <= n; i++)
        {
            double b = BinomialCoefficient(n, i) * System.Math.Pow(1 - t, n - i) * System.Math.Pow(t, i);
            result += b * _controlPoints[i];
        }
        
        return result;
    }
    
    /// <summary>
    /// Evaluates the first derivative at parameter t.
    /// </summary>
    public Vector<double> EvaluateDerivative(double t)
    {
        int n = _controlPoints.Length - 1;
        var result = Vector<double>.Build.Dense(_controlPoints[0].Count);
        
        for (int i = 0; i < n; i++)
        {
            double b = BinomialCoefficient(n - 1, i) * System.Math.Pow(1 - t, n - 1 - i) * System.Math.Pow(t, i);
            result += n * b * (_controlPoints[i + 1] - _controlPoints[i]);
        }
        
        return result;
    }
    
    /// <summary>
    /// Generates trajectory points along the curve.
    /// </summary>
    public Vector<double>[] GenerateTrajectory(int numPoints)
    {
        var points = new Vector<double>[numPoints];
        for (int i = 0; i < numPoints; i++)
        {
            double t = (double)i / (numPoints - 1);
            points[i] = Evaluate(t);
        }
        return points;
    }
    
    private static double BinomialCoefficient(int n, int k)
    {
        if (k > n) return 0;
        if (k == 0 || k == n) return 1;
        
        double result = 1;
        for (int i = 0; i < k; i++)
        {
            result *= (n - i) / (double)(i + 1);
        }
        return result;
    }
}

/// <summary>
/// B-spline trajectory generation.
/// </summary>
public class BSplineTrajectory
{
    private readonly Vector<double>[] _controlPoints;
    private readonly double[] _knots;
    private readonly int _degree;
    
    public BSplineTrajectory(Vector<double>[] controlPoints, int degree)
    {
        _controlPoints = controlPoints;
        _degree = degree;
        _knots = GenerateUniformKnots(controlPoints.Length, degree);
    }
    
    private static double[] GenerateUniformKnots(int numControlPoints, int degree)
    {
        int numKnots = numControlPoints + degree + 1;
        var knots = new double[numKnots];
        
        for (int i = 0; i < numKnots; i++)
        {
            if (i <= degree)
                knots[i] = 0;
            else if (i >= numKnots - degree - 1)
                knots[i] = 1;
            else
                knots[i] = (double)(i - degree) / (numControlPoints - degree);
        }
        
        return knots;
    }
    
    /// <summary>
    /// Evaluates the B-spline at parameter t.
    /// </summary>
    public Vector<double> Evaluate(double t)
    {
        int n = _controlPoints.Length;
        var result = Vector<double>.Build.Dense(_controlPoints[0].Count);
        
        for (int i = 0; i < n; i++)
        {
            double b = BasisFunction(i, _degree, t);
            result += b * _controlPoints[i];
        }
        
        return result;
    }
    
    private double BasisFunction(int i, int p, double t)
    {
        if (p == 0)
        {
            return (t >= _knots[i] && t < _knots[i + 1]) ? 1.0 : 0.0;
        }
        
        double left = 0, right = 0;
        
        double denom1 = _knots[i + p] - _knots[i];
        if (denom1 > 0)
        {
            left = (t - _knots[i]) / denom1 * BasisFunction(i, p - 1, t);
        }
        
        double denom2 = _knots[i + p + 1] - _knots[i + 1];
        if (denom2 > 0)
        {
            right = (_knots[i + p + 1] - t) / denom2 * BasisFunction(i + 1, p - 1, t);
        }
        
        return left + right;
    }
    
    public Vector<double>[] GenerateTrajectory(int numPoints)
    {
        var points = new Vector<double>[numPoints];
        for (int i = 0; i < numPoints; i++)
        {
            double t = (double)i / (numPoints - 1) * 0.999; // Avoid exact 1.0
            points[i] = Evaluate(t);
        }
        return points;
    }
}

/// <summary>
/// Dubins path planner for car-like vehicles.
/// </summary>
public class DubinsPathPlanner
{
    private readonly double _turningRadius;
    
    public DubinsPathPlanner(double turningRadius)
    {
        _turningRadius = turningRadius;
    }
    
    /// <summary>
    /// Computes the shortest Dubins path between two poses.
    /// </summary>
    /// <param name="start">(x, y, theta)</param>
    /// <param name="goal">(x, y, theta)</param>
    /// <returns>Path type and parameters</returns>
    public DubinsPath ComputePath(Vector<double> start, Vector<double> goal)
    {
        double x0 = start[0], y0 = start[1], theta0 = start[2];
        double x1 = goal[0], y1 = goal[1], theta1 = goal[2];
        
        // Normalize to start at origin
        double dx = x1 - x0;
        double dy = y1 - y0;
        double d = System.Math.Sqrt(dx * dx + dy * dy) / _turningRadius;
        double phi = System.Math.Atan2(dy, dx);
        double alpha = NormalizeAngle(theta0 - phi);
        double beta = NormalizeAngle(theta1 - phi);
        
        // Try all six Dubins path types
        var paths = new List<DubinsPath>
        {
            ComputeLSL(alpha, beta, d),
            ComputeRSR(alpha, beta, d),
            ComputeLSR(alpha, beta, d),
            ComputeRSL(alpha, beta, d),
            ComputeLRL(alpha, beta, d),
            ComputeRLR(alpha, beta, d)
        };
        
        // Return shortest valid path
        return paths.Where(p => p.IsValid).OrderBy(p => p.TotalLength).FirstOrDefault() 
            ?? new DubinsPath { IsValid = false };
    }
    
    private DubinsPath ComputeLSL(double alpha, double beta, double d)
    {
        double tmp = 2 + d * d - 2 * System.Math.Cos(alpha - beta) + 2 * d * (System.Math.Sin(alpha) - System.Math.Sin(beta));
        if (tmp < 0) return new DubinsPath { IsValid = false };
        
        double p = System.Math.Sqrt(tmp);
        double t = NormalizeAngle(-alpha + System.Math.Atan2(System.Math.Cos(beta) - System.Math.Cos(alpha), d + System.Math.Sin(alpha) - System.Math.Sin(beta)));
        double q = NormalizeAngle(beta - System.Math.Atan2(System.Math.Cos(beta) - System.Math.Cos(alpha), d + System.Math.Sin(alpha) - System.Math.Sin(beta)));
        
        return new DubinsPath
        {
            Type = DubinsPathType.LSL,
            Segment1 = t * _turningRadius,
            Segment2 = p * _turningRadius,
            Segment3 = q * _turningRadius,
            IsValid = true
        };
    }
    
    private DubinsPath ComputeRSR(double alpha, double beta, double d)
    {
        double tmp = 2 + d * d - 2 * System.Math.Cos(alpha - beta) + 2 * d * (System.Math.Sin(beta) - System.Math.Sin(alpha));
        if (tmp < 0) return new DubinsPath { IsValid = false };
        
        double p = System.Math.Sqrt(tmp);
        double t = NormalizeAngle(alpha - System.Math.Atan2(System.Math.Cos(alpha) - System.Math.Cos(beta), d - System.Math.Sin(alpha) + System.Math.Sin(beta)));
        double q = NormalizeAngle(-beta + System.Math.Atan2(System.Math.Cos(alpha) - System.Math.Cos(beta), d - System.Math.Sin(alpha) + System.Math.Sin(beta)));
        
        return new DubinsPath
        {
            Type = DubinsPathType.RSR,
            Segment1 = t * _turningRadius,
            Segment2 = p * _turningRadius,
            Segment3 = q * _turningRadius,
            IsValid = true
        };
    }
    
    private DubinsPath ComputeLSR(double alpha, double beta, double d)
    {
        double tmp = -2 + d * d + 2 * System.Math.Cos(alpha - beta) + 2 * d * (System.Math.Sin(alpha) + System.Math.Sin(beta));
        if (tmp < 0) return new DubinsPath { IsValid = false };
        
        double p = System.Math.Sqrt(tmp);
        double t = NormalizeAngle(-alpha + System.Math.Atan2(-System.Math.Cos(alpha) - System.Math.Cos(beta), d + System.Math.Sin(alpha) + System.Math.Sin(beta)) - System.Math.Atan2(-2, p));
        double q = NormalizeAngle(-beta + System.Math.Atan2(-System.Math.Cos(alpha) - System.Math.Cos(beta), d + System.Math.Sin(alpha) + System.Math.Sin(beta)) - System.Math.Atan2(-2, p));
        
        return new DubinsPath
        {
            Type = DubinsPathType.LSR,
            Segment1 = t * _turningRadius,
            Segment2 = p * _turningRadius,
            Segment3 = q * _turningRadius,
            IsValid = true
        };
    }
    
    private DubinsPath ComputeRSL(double alpha, double beta, double d)
    {
        double tmp = -2 + d * d + 2 * System.Math.Cos(alpha - beta) - 2 * d * (System.Math.Sin(alpha) + System.Math.Sin(beta));
        if (tmp < 0) return new DubinsPath { IsValid = false };
        
        double p = System.Math.Sqrt(tmp);
        double t = NormalizeAngle(alpha - System.Math.Atan2(System.Math.Cos(alpha) + System.Math.Cos(beta), d - System.Math.Sin(alpha) - System.Math.Sin(beta)) + System.Math.Atan2(2, p));
        double q = NormalizeAngle(beta - System.Math.Atan2(System.Math.Cos(alpha) + System.Math.Cos(beta), d - System.Math.Sin(alpha) - System.Math.Sin(beta)) + System.Math.Atan2(2, p));
        
        return new DubinsPath
        {
            Type = DubinsPathType.RSL,
            Segment1 = t * _turningRadius,
            Segment2 = p * _turningRadius,
            Segment3 = q * _turningRadius,
            IsValid = true
        };
    }
    
    private DubinsPath ComputeLRL(double alpha, double beta, double d)
    {
        double tmp = (6 - d * d + 2 * System.Math.Cos(alpha - beta) + 2 * d * (System.Math.Sin(alpha) - System.Math.Sin(beta))) / 8;
        if (System.Math.Abs(tmp) > 1) return new DubinsPath { IsValid = false };
        
        double p = NormalizeAngle(2 * System.Math.PI - System.Math.Acos(tmp));
        double t = NormalizeAngle(-alpha - System.Math.Atan2(System.Math.Cos(alpha) - System.Math.Cos(beta), d + System.Math.Sin(alpha) - System.Math.Sin(beta)) + p / 2);
        double q = NormalizeAngle(NormalizeAngle(beta) - alpha - t + NormalizeAngle(p));
        
        return new DubinsPath
        {
            Type = DubinsPathType.LRL,
            Segment1 = t * _turningRadius,
            Segment2 = p * _turningRadius,
            Segment3 = q * _turningRadius,
            IsValid = true
        };
    }
    
    private DubinsPath ComputeRLR(double alpha, double beta, double d)
    {
        double tmp = (6 - d * d + 2 * System.Math.Cos(alpha - beta) + 2 * d * (System.Math.Sin(beta) - System.Math.Sin(alpha))) / 8;
        if (System.Math.Abs(tmp) > 1) return new DubinsPath { IsValid = false };
        
        double p = NormalizeAngle(2 * System.Math.PI - System.Math.Acos(tmp));
        double t = NormalizeAngle(alpha - System.Math.Atan2(System.Math.Cos(alpha) - System.Math.Cos(beta), d - System.Math.Sin(alpha) + System.Math.Sin(beta)) + p / 2);
        double q = NormalizeAngle(alpha - beta - t + NormalizeAngle(p));
        
        return new DubinsPath
        {
            Type = DubinsPathType.RLR,
            Segment1 = t * _turningRadius,
            Segment2 = p * _turningRadius,
            Segment3 = q * _turningRadius,
            IsValid = true
        };
    }
    
    private static double NormalizeAngle(double angle)
    {
        while (angle > System.Math.PI) angle -= 2 * System.Math.PI;
        while (angle < -System.Math.PI) angle += 2 * System.Math.PI;
        return angle;
    }
}

public enum DubinsPathType { LSL, RSR, LSR, RSL, LRL, RLR }

public class DubinsPath
{
    public DubinsPathType Type { get; set; }
    public double Segment1 { get; set; }
    public double Segment2 { get; set; }
    public double Segment3 { get; set; }
    public bool IsValid { get; set; }
    public double TotalLength => Segment1 + Segment2 + Segment3;
}

/// <summary>
/// RRT (Rapidly-exploring Random Trees) path planner.
/// </summary>
public class RRTPlanner
{
    private readonly Random _rng = new();
    private readonly double[] _bounds;
    private readonly Func<Vector<double>, Vector<double>, bool> _collisionCheck;
    private readonly double _stepSize;
    private readonly int _maxIterations;

    public RRTPlanner(
        double[] bounds,
        Func<Vector<double>, Vector<double>, bool> collisionCheck,
        double stepSize = 0.5,
        int maxIterations = 5000)
    {
        _bounds = bounds;
        _collisionCheck = collisionCheck;
        _stepSize = stepSize;
        _maxIterations = maxIterations;
    }

    /// <summary>
    /// Plans a path from start to goal.
    /// </summary>
    public List<Vector<double>>? Plan(Vector<double> start, Vector<double> goal, double goalTolerance = 0.5)
    {
        var nodes = new List<RRTNode> { new(start, -1) };
        
        for (int i = 0; i < _maxIterations; i++)
        {
            // Sample random point (with goal bias)
            var sample = _rng.NextDouble() < 0.1 ? goal : RandomSample();
            
            // Find nearest node
            int nearestIdx = FindNearest(nodes, sample);
            var nearest = nodes[nearestIdx];
            
            // Steer towards sample
            var newPoint = Steer(nearest.Position, sample);
            
            // Check collision
            if (!_collisionCheck(nearest.Position, newPoint))
            {
                nodes.Add(new RRTNode(newPoint, nearestIdx));
                
                // Check if goal reached
                if ((newPoint - goal).L2Norm() < goalTolerance)
                {
                    return ReconstructPath(nodes, nodes.Count - 1);
                }
            }
        }
        
        return null; // No path found
    }

    private Vector<double> RandomSample()
    {
        int dim = _bounds.Length / 2;
        var sample = Vector<double>.Build.Dense(dim);
        for (int i = 0; i < dim; i++)
        {
            double min = _bounds[i * 2];
            double max = _bounds[i * 2 + 1];
            sample[i] = min + _rng.NextDouble() * (max - min);
        }
        return sample;
    }

    private int FindNearest(List<RRTNode> nodes, Vector<double> point)
    {
        int nearest = 0;
        double minDist = double.MaxValue;
        
        for (int i = 0; i < nodes.Count; i++)
        {
            double dist = (nodes[i].Position - point).L2Norm();
            if (dist < minDist)
            {
                minDist = dist;
                nearest = i;
            }
        }
        
        return nearest;
    }

    private Vector<double> Steer(Vector<double> from, Vector<double> to)
    {
        var direction = to - from;
        double distance = direction.L2Norm();
        
        if (distance <= _stepSize)
            return to;
        
        return from + (direction / distance) * _stepSize;
    }

    private List<Vector<double>> ReconstructPath(List<RRTNode> nodes, int goalIdx)
    {
        var path = new List<Vector<double>>();
        int current = goalIdx;
        
        while (current >= 0)
        {
            path.Add(nodes[current].Position);
            current = nodes[current].ParentIdx;
        }
        
        path.Reverse();
        return path;
    }

    private record RRTNode(Vector<double> Position, int ParentIdx);
}
