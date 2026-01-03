namespace ControlWorkbench.VEX;

/// <summary>
/// Pure pursuit path following controller for VEX robots.
/// </summary>
public class PurePursuitController
{
    private readonly double _lookaheadDistance;
    private readonly double _trackWidth;
    private List<Waypoint> _path = new();
    private int _currentWaypointIndex = 0;
    private bool _isFinished = false;
    private bool _isReversed = false;

    public double LookaheadDistance => _lookaheadDistance;
    public int CurrentWaypointIndex => _currentWaypointIndex;
    public bool IsFinished => _isFinished;
    public Waypoint? TargetPoint { get; private set; }

    public PurePursuitController(double lookaheadDistance = 15.0, double trackWidth = 12.0)
    {
        _lookaheadDistance = lookaheadDistance;
        _trackWidth = trackWidth;
    }

    /// <summary>
    /// Set the path to follow.
    /// </summary>
    public void SetPath(IEnumerable<Waypoint> waypoints, bool reversed = false)
    {
        _path = waypoints.ToList();
        _currentWaypointIndex = 0;
        _isFinished = false;
        _isReversed = reversed;
    }

    /// <summary>
    /// Compute left and right wheel velocities to follow the path.
    /// </summary>
    /// <param name="x">Current x position (inches)</param>
    /// <param name="y">Current y position (inches)</param>
    /// <param name="theta">Current heading (radians)</param>
    /// <param name="maxVelocity">Maximum velocity (0-127 or RPM)</param>
    /// <returns>Left and right wheel velocities</returns>
    public (double left, double right) Compute(double x, double y, double theta, double maxVelocity)
    {
        if (_path.Count == 0 || _isFinished)
        {
            return (0, 0);
        }

        // Find lookahead point
        var lookahead = FindLookaheadPoint(x, y);
        if (lookahead == null)
        {
            _isFinished = true;
            return (0, 0);
        }

        TargetPoint = lookahead.Value;

        // Calculate curvature to lookahead point
        double dx = lookahead.Value.X - x;
        double dy = lookahead.Value.Y - y;

        // Transform to robot frame
        double cos = System.Math.Cos(-theta);
        double sin = System.Math.Sin(-theta);
        double localX = dx * cos - dy * sin;
        double localY = dx * sin + dy * cos;

        // Handle reversed driving
        if (_isReversed)
        {
            localY = -localY;
        }

        // Calculate curvature: k = 2x / L^2
        double L = System.Math.Sqrt(localX * localX + localY * localY);
        double curvature = (L > 0.001) ? 2.0 * localX / (L * L) : 0;

        // Get velocity from waypoint or use max
        double velocity = lookahead.Value.Velocity > 0 ? lookahead.Value.Velocity : maxVelocity;
        if (_isReversed) velocity = -velocity;

        // Convert to differential drive
        double left = velocity * (1 - curvature * _trackWidth / 2);
        double right = velocity * (1 + curvature * _trackWidth / 2);

        return (left, right);
    }

    private Waypoint? FindLookaheadPoint(double x, double y)
    {
        // Find the closest point on path that's at least lookahead distance away
        double closestDist = double.MaxValue;
        int closestIndex = _currentWaypointIndex;

        // First, find closest waypoint to update current index
        for (int i = _currentWaypointIndex; i < _path.Count; i++)
        {
            double dist = Distance(x, y, _path[i].X, _path[i].Y);
            if (dist < closestDist)
            {
                closestDist = dist;
                closestIndex = i;
            }
        }
        _currentWaypointIndex = closestIndex;

        // Now find lookahead point by interpolating along path segments
        for (int i = _currentWaypointIndex; i < _path.Count - 1; i++)
        {
            var p1 = _path[i];
            var p2 = _path[i + 1];

            var intersections = CircleLineIntersection(x, y, _lookaheadDistance, p1, p2);
            
            foreach (var intersection in intersections)
            {
                // Prefer points further along the path
                double t = ParameterOnSegment(p1, p2, intersection);
                if (t >= 0 && t <= 1)
                {
                    // Interpolate velocity
                    double velocity = p1.Velocity + t * (p2.Velocity - p1.Velocity);
                    return new Waypoint(intersection.x, intersection.y, velocity);
                }
            }
        }

        // If no intersection found, return last waypoint if close enough
        var lastPoint = _path[^1];
        double distToEnd = Distance(x, y, lastPoint.X, lastPoint.Y);
        if (distToEnd < _lookaheadDistance * 2)
        {
            if (distToEnd < 2.0) // Within 2 inches of end
            {
                return null; // Signal completion
            }
            return lastPoint;
        }

        return _path[System.Math.Min(_currentWaypointIndex + 1, _path.Count - 1)];
    }

    private List<(double x, double y)> CircleLineIntersection(
        double cx, double cy, double r, Waypoint p1, Waypoint p2)
    {
        var result = new List<(double, double)>();

        double dx = p2.X - p1.X;
        double dy = p2.Y - p1.Y;
        double fx = p1.X - cx;
        double fy = p1.Y - cy;

        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
            return result;

        discriminant = System.Math.Sqrt(discriminant);

        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        if (t1 >= 0 && t1 <= 1)
            result.Add((p1.X + t1 * dx, p1.Y + t1 * dy));
        if (t2 >= 0 && t2 <= 1 && System.Math.Abs(t2 - t1) > 0.001)
            result.Add((p1.X + t2 * dx, p1.Y + t2 * dy));

        return result;
    }

    private double ParameterOnSegment(Waypoint p1, Waypoint p2, (double x, double y) point)
    {
        double dx = p2.X - p1.X;
        double dy = p2.Y - p1.Y;
        double len2 = dx * dx + dy * dy;
        if (len2 < 0.001) return 0;

        return ((point.x - p1.X) * dx + (point.y - p1.Y) * dy) / len2;
    }

    private double Distance(double x1, double y1, double x2, double y2)
    {
        return System.Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    /// <summary>
    /// Reset the controller for a new path run.
    /// </summary>
    public void Reset()
    {
        _currentWaypointIndex = 0;
        _isFinished = false;
        TargetPoint = null;
    }
}

/// <summary>
/// Ramsete controller for smooth trajectory following.
/// </summary>
public class RamseteController
{
    private readonly double _b;
    private readonly double _zeta;
    private readonly double _trackWidth;

    public RamseteController(double b = 2.0, double zeta = 0.7, double trackWidth = 12.0)
    {
        _b = b;
        _zeta = zeta;
        _trackWidth = trackWidth;
    }

    /// <summary>
    /// Compute wheel velocities to track a trajectory state.
    /// </summary>
    public (double left, double right) Compute(
        double x, double y, double theta,
        double desiredX, double desiredY, double desiredTheta,
        double desiredVelocity, double desiredAngularVelocity)
    {
        // Transform error to robot frame
        double cos = System.Math.Cos(theta);
        double sin = System.Math.Sin(theta);
        
        double ex = cos * (desiredX - x) + sin * (desiredY - y);
        double ey = -sin * (desiredX - x) + cos * (desiredY - y);
        double etheta = NormalizeAngle(desiredTheta - theta);

        // Ramsete gains
        double k = 2.0 * _zeta * System.Math.Sqrt(desiredAngularVelocity * desiredAngularVelocity + 
                                                    _b * desiredVelocity * desiredVelocity);

        // Compute velocity commands
        double v = desiredVelocity * System.Math.Cos(etheta) + k * ex;
        double omega = desiredAngularVelocity + k * etheta + 
                       _b * desiredVelocity * Sinc(etheta) * ey;

        // Convert to differential drive
        double left = v - omega * _trackWidth / 2;
        double right = v + omega * _trackWidth / 2;

        return (left, right);
    }

    private double Sinc(double x)
    {
        if (System.Math.Abs(x) < 1e-6)
            return 1.0;
        return System.Math.Sin(x) / x;
    }

    private double NormalizeAngle(double angle)
    {
        while (angle > System.Math.PI) angle -= 2 * System.Math.PI;
        while (angle < -System.Math.PI) angle += 2 * System.Math.PI;
        return angle;
    }
}

/// <summary>
/// Motion profiled point-to-point movement.
/// </summary>
public class MotionProfiledMovement
{
    private readonly double _maxVelocity;
    private readonly double _maxAcceleration;
    private readonly double _maxDeceleration;
    private readonly double _trackWidth;

    public MotionProfiledMovement(
        double maxVelocity = 60.0,      // inches/sec
        double maxAcceleration = 80.0,   // inches/sec^2
        double maxDeceleration = 100.0,  // inches/sec^2
        double trackWidth = 12.0)
    {
        _maxVelocity = maxVelocity;
        _maxAcceleration = maxAcceleration;
        _maxDeceleration = maxDeceleration;
        _trackWidth = trackWidth;
    }

    /// <summary>
    /// Generate a motion profile for driving straight.
    /// </summary>
    public MotionProfile GenerateStraightProfile(double distance)
    {
        return GenerateTrapezoidalProfile(System.Math.Abs(distance), 
            System.Math.Sign(distance) * _maxVelocity);
    }

    /// <summary>
    /// Generate a motion profile for turning in place.
    /// </summary>
    public MotionProfile GenerateTurnProfile(double angleRadians)
    {
        double arcLength = System.Math.Abs(angleRadians) * _trackWidth / 2;
        double maxAngularVel = _maxVelocity / (_trackWidth / 2);
        
        var profile = GenerateTrapezoidalProfile(arcLength, 
            System.Math.Sign(angleRadians) * maxAngularVel);
        profile.IsTurn = true;
        return profile;
    }

    /// <summary>
    /// Generate a motion profile for arc movement.
    /// </summary>
    public MotionProfile GenerateArcProfile(double radius, double angleRadians)
    {
        double arcLength = System.Math.Abs(radius * angleRadians);
        var profile = GenerateTrapezoidalProfile(arcLength, _maxVelocity);
        profile.Radius = radius;
        profile.IsArc = true;
        return profile;
    }

    private MotionProfile GenerateTrapezoidalProfile(double distance, double targetVelocity)
    {
        double absVel = System.Math.Abs(targetVelocity);
        double sign = System.Math.Sign(targetVelocity);
        
        // Calculate acceleration and deceleration distances
        double accelDist = (absVel * absVel) / (2 * _maxAcceleration);
        double decelDist = (absVel * absVel) / (2 * _maxDeceleration);
        
        // Check if triangular profile needed
        if (accelDist + decelDist > distance)
        {
            // Triangular profile - can't reach max velocity
            double peakVel = System.Math.Sqrt(2 * distance * _maxAcceleration * _maxDeceleration / 
                (_maxAcceleration + _maxDeceleration));
            accelDist = (peakVel * peakVel) / (2 * _maxAcceleration);
            decelDist = distance - accelDist;
            absVel = peakVel;
        }

        double cruiseDist = distance - accelDist - decelDist;
        
        double accelTime = absVel / _maxAcceleration;
        double cruiseTime = cruiseDist / absVel;
        double decelTime = absVel / _maxDeceleration;
        double totalTime = accelTime + cruiseTime + decelTime;

        return new MotionProfile
        {
            TotalDistance = distance,
            MaxVelocity = sign * absVel,
            AccelDistance = accelDist,
            CruiseDistance = cruiseDist,
            DecelDistance = decelDist,
            AccelTime = accelTime,
            CruiseTime = cruiseTime,
            DecelTime = decelTime,
            TotalTime = totalTime
        };
    }

    /// <summary>
    /// Sample the motion profile at a given time.
    /// </summary>
    public (double position, double velocity) SampleProfile(MotionProfile profile, double time)
    {
        if (time < 0) return (0, 0);
        if (time >= profile.TotalTime) return (profile.TotalDistance, 0);

        double sign = System.Math.Sign(profile.MaxVelocity);
        double absVel = System.Math.Abs(profile.MaxVelocity);

        if (time < profile.AccelTime)
        {
            // Acceleration phase
            double velocity = _maxAcceleration * time;
            double position = 0.5 * _maxAcceleration * time * time;
            return (sign * position, sign * velocity);
        }
        else if (time < profile.AccelTime + profile.CruiseTime)
        {
            // Cruise phase
            double t = time - profile.AccelTime;
            double position = profile.AccelDistance + absVel * t;
            return (sign * position, sign * absVel);
        }
        else
        {
            // Deceleration phase
            double t = time - profile.AccelTime - profile.CruiseTime;
            double velocity = absVel - _maxDeceleration * t;
            double position = profile.AccelDistance + profile.CruiseDistance + 
                              absVel * t - 0.5 * _maxDeceleration * t * t;
            return (sign * position, sign * velocity);
        }
    }
}

/// <summary>
/// Motion profile data.
/// </summary>
public class MotionProfile
{
    public double TotalDistance { get; set; }
    public double MaxVelocity { get; set; }
    public double AccelDistance { get; set; }
    public double CruiseDistance { get; set; }
    public double DecelDistance { get; set; }
    public double AccelTime { get; set; }
    public double CruiseTime { get; set; }
    public double DecelTime { get; set; }
    public double TotalTime { get; set; }
    public bool IsTurn { get; set; }
    public bool IsArc { get; set; }
    public double Radius { get; set; }
}

/// <summary>
/// Waypoint for path following.
/// </summary>
public readonly struct Waypoint
{
    public double X { get; }
    public double Y { get; }
    public double Velocity { get; }
    public double Heading { get; }

    public Waypoint(double x, double y, double velocity = 0, double heading = double.NaN)
    {
        X = x;
        Y = y;
        Velocity = velocity;
        Heading = heading;
    }

    public double DistanceTo(Waypoint other) =>
        System.Math.Sqrt((X - other.X) * (X - other.X) + (Y - other.Y) * (Y - other.Y));

    public double AngleTo(Waypoint other) =>
        System.Math.Atan2(other.X - X, other.Y - Y);
}

/// <summary>
/// Path generation utilities.
/// </summary>
public static class PathGenerator
{
    /// <summary>
    /// Generate smooth path from sparse waypoints using cubic spline interpolation.
    /// </summary>
    public static List<Waypoint> GenerateSmoothPath(
        IList<Waypoint> controlPoints, 
        double spacing = 2.0,
        double maxVelocity = 60.0)
    {
        if (controlPoints.Count < 2)
            return controlPoints.ToList();

        var result = new List<Waypoint>();

        for (int i = 0; i < controlPoints.Count - 1; i++)
        {
            var p0 = i > 0 ? controlPoints[i - 1] : controlPoints[i];
            var p1 = controlPoints[i];
            var p2 = controlPoints[i + 1];
            var p3 = i < controlPoints.Count - 2 ? controlPoints[i + 2] : controlPoints[i + 1];

            double segmentLength = p1.DistanceTo(p2);
            int numPoints = System.Math.Max(2, (int)(segmentLength / spacing));

            for (int j = 0; j < numPoints; j++)
            {
                double t = (double)j / numPoints;
                var point = CatmullRomInterpolate(p0, p1, p2, p3, t);
                result.Add(point);
            }
        }

        // Add last point
        result.Add(controlPoints[^1]);

        // Compute velocities based on curvature
        ComputeVelocities(result, maxVelocity);

        return result;
    }

    private static Waypoint CatmullRomInterpolate(
        Waypoint p0, Waypoint p1, Waypoint p2, Waypoint p3, double t)
    {
        double t2 = t * t;
        double t3 = t2 * t;

        double x = 0.5 * ((2 * p1.X) +
            (-p0.X + p2.X) * t +
            (2 * p0.X - 5 * p1.X + 4 * p2.X - p3.X) * t2 +
            (-p0.X + 3 * p1.X - 3 * p2.X + p3.X) * t3);

        double y = 0.5 * ((2 * p1.Y) +
            (-p0.Y + p2.Y) * t +
            (2 * p0.Y - 5 * p1.Y + 4 * p2.Y - p3.Y) * t2 +
            (-p0.Y + 3 * p1.Y - 3 * p2.Y + p3.Y) * t3);

        return new Waypoint(x, y);
    }

    private static void ComputeVelocities(List<Waypoint> path, double maxVelocity)
    {
        if (path.Count < 3) return;

        // Compute curvature at each point
        var curvatures = new double[path.Count];
        for (int i = 1; i < path.Count - 1; i++)
        {
            curvatures[i] = ComputeCurvature(path[i - 1], path[i], path[i + 1]);
        }

        // Forward pass - respect max velocity and curvature
        var velocities = new double[path.Count];
        velocities[0] = 0;
        for (int i = 1; i < path.Count; i++)
        {
            double curvatureLimit = curvatures[i] > 0.001 ? 
                System.Math.Sqrt(maxVelocity / (curvatures[i] * 10)) : maxVelocity;
            velocities[i] = System.Math.Min(maxVelocity, curvatureLimit);
        }
        velocities[^1] = 0;

        // Backward pass - respect deceleration limits
        double maxDecel = 100.0; // inches/sec^2
        for (int i = path.Count - 2; i >= 0; i--)
        {
            double dist = path[i].DistanceTo(path[i + 1]);
            double maxVelFromDecel = System.Math.Sqrt(velocities[i + 1] * velocities[i + 1] + 
                                                       2 * maxDecel * dist);
            velocities[i] = System.Math.Min(velocities[i], maxVelFromDecel);
        }

        // Update waypoints with velocities
        for (int i = 0; i < path.Count; i++)
        {
            var wp = path[i];
            path[i] = new Waypoint(wp.X, wp.Y, velocities[i], wp.Heading);
        }
    }

    private static double ComputeCurvature(Waypoint p1, Waypoint p2, Waypoint p3)
    {
        double x1 = p1.X, y1 = p1.Y;
        double x2 = p2.X, y2 = p2.Y;
        double x3 = p3.X, y3 = p3.Y;

        double k = 2 * System.Math.Abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));
        double d1 = System.Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        double d2 = System.Math.Sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
        double d3 = System.Math.Sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));

        double denom = d1 * d2 * d3;
        if (denom < 0.001) return 0;
        
        return k / denom;
    }

    /// <summary>
    /// Inject additional points at a specified spacing.
    /// </summary>
    public static List<Waypoint> InjectPoints(IList<Waypoint> path, double spacing)
    {
        var result = new List<Waypoint>();

        for (int i = 0; i < path.Count - 1; i++)
        {
            var p1 = path[i];
            var p2 = path[i + 1];
            double dist = p1.DistanceTo(p2);
            int numPoints = (int)(dist / spacing);

            result.Add(p1);
            for (int j = 1; j < numPoints; j++)
            {
                double t = (double)j / numPoints;
                double x = p1.X + t * (p2.X - p1.X);
                double y = p1.Y + t * (p2.Y - p1.Y);
                double vel = p1.Velocity + t * (p2.Velocity - p1.Velocity);
                result.Add(new Waypoint(x, y, vel));
            }
        }
        result.Add(path[^1]);

        return result;
    }
}
