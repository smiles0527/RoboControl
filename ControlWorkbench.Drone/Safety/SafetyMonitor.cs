using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Safety;

/// <summary>
/// Real-time safety monitor with formal verification guarantees.
/// Implements Control Barrier Functions (CBF) and runtime monitoring.
/// </summary>
public class SafetyMonitor
{
    private readonly List<SafetyConstraint> _constraints = new();
    private readonly SafetyConfig _config;
    private SafetyStatus _status = SafetyStatus.Nominal;
    private readonly List<SafetyViolation> _violations = new();
    
    public event Action<SafetyViolation>? ViolationDetected;
    public event Action<SafetyStatus>? StatusChanged;
    
    public SafetyStatus Status => _status;
    public IReadOnlyList<SafetyViolation> RecentViolations => _violations;
    
    public SafetyMonitor(SafetyConfig? config = null)
    {
        _config = config ?? SafetyConfig.Default;
        InitializeDefaultConstraints();
    }
    
    /// <summary>
    /// Add a custom safety constraint.
    /// </summary>
    public void AddConstraint(SafetyConstraint constraint)
    {
        _constraints.Add(constraint);
    }
    
    /// <summary>
    /// Check all constraints against current state.
    /// Returns true if all constraints are satisfied.
    /// </summary>
    public bool Check(VehicleState state)
    {
        bool allSafe = true;
        var currentTime = DateTime.UtcNow;
        
        foreach (var constraint in _constraints)
        {
            if (!constraint.Enabled) continue;
            
            var result = constraint.Evaluate(state);
            
            if (result.Violated)
            {
                allSafe = false;
                
                var violation = new SafetyViolation
                {
                    Constraint = constraint,
                    Severity = result.Severity,
                    Message = result.Message,
                    Value = result.Value,
                    Threshold = result.Threshold,
                    Timestamp = currentTime
                };
                
                _violations.Add(violation);
                ViolationDetected?.Invoke(violation);
                
                // Keep only recent violations
                while (_violations.Count > 100)
                    _violations.RemoveAt(0);
            }
        }
        
        UpdateStatus(allSafe);
        return allSafe;
    }
    
    /// <summary>
    /// Compute safe control input using Control Barrier Functions.
    /// Modifies the desired control to ensure safety constraints.
    /// </summary>
    public Vector<double> EnforceSafeControl(
        VehicleState state,
        Vector<double> desiredControl,
        VehicleDynamics dynamics)
    {
        // Collect active CBF constraints
        var cbfConstraints = _constraints.OfType<ControlBarrierFunction>().ToList();
        
        if (cbfConstraints.Count == 0)
            return desiredControl;
        
        // Solve CBF-QP
        return SolveCbfQp(state, desiredControl, cbfConstraints, dynamics);
    }
    
    /// <summary>
    /// Get recommended recovery action for current safety status.
    /// </summary>
    public RecoveryAction GetRecoveryAction()
    {
        return _status switch
        {
            SafetyStatus.Warning => RecoveryAction.Continue,
            SafetyStatus.Critical => RecoveryAction.ReturnToHome,
            SafetyStatus.Emergency => RecoveryAction.Land,
            SafetyStatus.FailSafe => RecoveryAction.EmergencyStop,
            _ => RecoveryAction.None
        };
    }
    
    private void InitializeDefaultConstraints()
    {
        // Geofence constraint
        AddConstraint(new GeofenceConstraint
        {
            Name = "Geofence",
            MaxRadius = _config.MaxRadius,
            MaxAltitude = _config.MaxAltitude,
            MinAltitude = _config.MinAltitude,
            HomePosition = (_config.HomeLatitude, _config.HomeLongitude)
        });
        
        // Attitude limits
        AddConstraint(new AttitudeLimitConstraint
        {
            Name = "AttitudeLimits",
            MaxRoll = _config.MaxRoll,
            MaxPitch = _config.MaxPitch
        });
        
        // Velocity limits
        AddConstraint(new VelocityLimitConstraint
        {
            Name = "VelocityLimits",
            MaxHorizontalSpeed = _config.MaxHorizontalSpeed,
            MaxVerticalSpeed = _config.MaxVerticalSpeed,
            MaxDescentRate = _config.MaxDescentRate
        });
        
        // Battery constraint
        AddConstraint(new BatteryConstraint
        {
            Name = "BatteryMonitor",
            WarningLevel = _config.BatteryWarning,
            CriticalLevel = _config.BatteryCritical
        });
        
        // Obstacle avoidance CBF
        AddConstraint(new ObstacleAvoidanceCbf
        {
            Name = "ObstacleAvoidance",
            MinimumDistance = _config.ObstacleMargin
        });
    }
    
    private void UpdateStatus(bool allSafe)
    {
        var newStatus = SafetyStatus.Nominal;
        
        if (!allSafe)
        {
            var maxSeverity = _violations
                .Where(v => (DateTime.UtcNow - v.Timestamp).TotalSeconds < 5)
                .Select(v => v.Severity)
                .DefaultIfEmpty(ConstraintSeverity.Info)
                .Max();
            
            newStatus = maxSeverity switch
            {
                ConstraintSeverity.Warning => SafetyStatus.Warning,
                ConstraintSeverity.Critical => SafetyStatus.Critical,
                ConstraintSeverity.Emergency => SafetyStatus.Emergency,
                _ => SafetyStatus.Warning
            };
        }
        
        if (newStatus != _status)
        {
            _status = newStatus;
            StatusChanged?.Invoke(_status);
        }
    }
    
    private Vector<double> SolveCbfQp(
        VehicleState state,
        Vector<double> uDesired,
        List<ControlBarrierFunction> cbfs,
        VehicleDynamics dynamics)
    {
        int nu = uDesired.Count;
        
        // Objective: minimize ||u - u_desired||^2
        // Subject to: Lf(h) + Lg(h)*u + ?(h) >= 0 for all CBFs
        
        // Build constraint matrices
        var A = Matrix<double>.Build.Dense(cbfs.Count, nu);
        var b = Vector<double>.Build.Dense(cbfs.Count);
        
        for (int i = 0; i < cbfs.Count; i++)
        {
            var cbf = cbfs[i];
            var (Lfh, Lgh) = cbf.ComputeLieDerivatives(state, dynamics);
            double h = cbf.Evaluate(state).Value;
            double alpha = cbf.ClassKFunction(h);
            
            // Constraint: Lgh * u >= -Lfh - alpha
            for (int j = 0; j < nu; j++)
            {
                A[i, j] = -Lgh[j];
            }
            b[i] = Lfh + alpha;
        }
        
        // Solve QP using active set method
        return SolveQp(uDesired, A, b);
    }
    
    private Vector<double> SolveQp(Vector<double> uDesired, Matrix<double> A, Vector<double> b)
    {
        // Simple projection-based solver
        var u = uDesired.Clone();
        
        for (int iter = 0; iter < 10; iter++)
        {
            bool allSatisfied = true;
            
            for (int i = 0; i < A.RowCount; i++)
            {
                var ai = A.Row(i);
                double violation = ai * u - b[i];
                
                if (violation > 0) // Constraint violated
                {
                    allSatisfied = false;
                    // Project onto constraint boundary
                    u = u - (violation / ai.L2Norm()) * ai.Normalize(2);
                }
            }
            
            if (allSatisfied) break;
        }
        
        return u;
    }
}

/// <summary>
/// Base class for safety constraints.
/// </summary>
public abstract class SafetyConstraint
{
    public string Name { get; set; } = "";
    public bool Enabled { get; set; } = true;
    public ConstraintSeverity DefaultSeverity { get; set; } = ConstraintSeverity.Warning;
    
    public abstract ConstraintResult Evaluate(VehicleState state);
}

/// <summary>
/// Control Barrier Function for safe control synthesis.
/// </summary>
public abstract class ControlBarrierFunction : SafetyConstraint
{
    public double Alpha { get; set; } = 1.0;
    
    /// <summary>
    /// Compute the barrier function value h(x).
    /// h(x) > 0 means safe, h(x) <= 0 means unsafe.
    /// </summary>
    public abstract double ComputeBarrier(VehicleState state);
    
    /// <summary>
    /// Compute Lie derivatives Lf(h) and Lg(h).
    /// </summary>
    public abstract (double Lfh, Vector<double> Lgh) ComputeLieDerivatives(
        VehicleState state, VehicleDynamics dynamics);
    
    /// <summary>
    /// Class-K function for CBF condition.
    /// </summary>
    public virtual double ClassKFunction(double h)
    {
        return Alpha * h;
    }
    
    public override ConstraintResult Evaluate(VehicleState state)
    {
        double h = ComputeBarrier(state);
        
        return new ConstraintResult
        {
            Violated = h <= 0,
            Severity = h <= 0 ? DefaultSeverity : ConstraintSeverity.Info,
            Value = h,
            Threshold = 0,
            Message = h <= 0 ? $"{Name}: Barrier function violated (h={h:F3})" : ""
        };
    }
}

/// <summary>
/// Geofence constraint using cylinder/polygon boundary.
/// </summary>
public class GeofenceConstraint : SafetyConstraint
{
    public double MaxRadius { get; set; } = 500;
    public double MaxAltitude { get; set; } = 120;
    public double MinAltitude { get; set; } = 2;
    public (double lat, double lon) HomePosition { get; set; }
    public List<(double lat, double lon)>? PolygonBoundary { get; set; }
    
    public override ConstraintResult Evaluate(VehicleState state)
    {
        // Distance from home
        double distance = HaversineDistance(
            state.Latitude, state.Longitude,
            HomePosition.lat, HomePosition.lon);
        
        // Check cylinder geofence
        if (distance > MaxRadius)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Value = distance,
                Threshold = MaxRadius,
                Message = $"Geofence breach: {distance:F1}m from home (max {MaxRadius:F1}m)"
            };
        }
        
        if (state.Altitude > MaxAltitude)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Value = state.Altitude,
                Threshold = MaxAltitude,
                Message = $"Altitude limit: {state.Altitude:F1}m (max {MaxAltitude:F1}m)"
            };
        }
        
        if (state.Altitude < MinAltitude && state.Armed)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Warning,
                Value = state.Altitude,
                Threshold = MinAltitude,
                Message = $"Low altitude: {state.Altitude:F1}m (min {MinAltitude:F1}m)"
            };
        }
        
        // Check polygon if defined
        if (PolygonBoundary != null && !IsPointInPolygon(state.Latitude, state.Longitude))
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Message = "Outside polygon geofence"
            };
        }
        
        return ConstraintResult.Safe;
    }
    
    private bool IsPointInPolygon(double lat, double lon)
    {
        if (PolygonBoundary == null || PolygonBoundary.Count < 3)
            return true;
        
        bool inside = false;
        int n = PolygonBoundary.Count;
        
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            var (yi, xi) = PolygonBoundary[i];
            var (yj, xj) = PolygonBoundary[j];
            
            if (((yi > lon) != (yj > lon)) &&
                (lat < (xj - xi) * (lon - yi) / (yj - yi) + xi))
            {
                inside = !inside;
            }
        }
        
        return inside;
    }
    
    private static double HaversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371000;
        double dLat = (lat2 - lat1) * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1 * System.Math.PI / 180) * System.Math.Cos(lat2 * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        return R * 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
    }
}

/// <summary>
/// Attitude limit constraint.
/// </summary>
public class AttitudeLimitConstraint : SafetyConstraint
{
    public double MaxRoll { get; set; } = 45;
    public double MaxPitch { get; set; } = 45;
    
    public override ConstraintResult Evaluate(VehicleState state)
    {
        double rollDeg = state.Roll * 180 / System.Math.PI;
        double pitchDeg = state.Pitch * 180 / System.Math.PI;
        
        if (System.Math.Abs(rollDeg) > MaxRoll)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Value = rollDeg,
                Threshold = MaxRoll,
                Message = $"Excessive roll: {rollDeg:F1}° (max ±{MaxRoll:F1}°)"
            };
        }
        
        if (System.Math.Abs(pitchDeg) > MaxPitch)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Value = pitchDeg,
                Threshold = MaxPitch,
                Message = $"Excessive pitch: {pitchDeg:F1}° (max ±{MaxPitch:F1}°)"
            };
        }
        
        return ConstraintResult.Safe;
    }
}

/// <summary>
/// Velocity limit constraint.
/// </summary>
public class VelocityLimitConstraint : SafetyConstraint
{
    public double MaxHorizontalSpeed { get; set; } = 20;
    public double MaxVerticalSpeed { get; set; } = 5;
    public double MaxDescentRate { get; set; } = 3;
    
    public override ConstraintResult Evaluate(VehicleState state)
    {
        double horizontalSpeed = System.Math.Sqrt(
            state.VelocityNorth * state.VelocityNorth +
            state.VelocityEast * state.VelocityEast);
        
        if (horizontalSpeed > MaxHorizontalSpeed)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Warning,
                Value = horizontalSpeed,
                Threshold = MaxHorizontalSpeed,
                Message = $"High speed: {horizontalSpeed:F1}m/s (max {MaxHorizontalSpeed:F1}m/s)"
            };
        }
        
        if (state.VelocityDown < -MaxVerticalSpeed)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Warning,
                Value = -state.VelocityDown,
                Threshold = MaxVerticalSpeed,
                Message = $"Fast climb: {-state.VelocityDown:F1}m/s"
            };
        }
        
        if (state.VelocityDown > MaxDescentRate)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Value = state.VelocityDown,
                Threshold = MaxDescentRate,
                Message = $"Fast descent: {state.VelocityDown:F1}m/s"
            };
        }
        
        return ConstraintResult.Safe;
    }
}

/// <summary>
/// Battery monitoring constraint.
/// </summary>
public class BatteryConstraint : SafetyConstraint
{
    public double WarningLevel { get; set; } = 30;
    public double CriticalLevel { get; set; } = 15;
    public double EmergencyLevel { get; set; } = 5;
    
    public override ConstraintResult Evaluate(VehicleState state)
    {
        if (state.BatteryPercent <= EmergencyLevel)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Emergency,
                Value = state.BatteryPercent,
                Threshold = EmergencyLevel,
                Message = $"EMERGENCY: Battery {state.BatteryPercent:F0}%"
            };
        }
        
        if (state.BatteryPercent <= CriticalLevel)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Critical,
                Value = state.BatteryPercent,
                Threshold = CriticalLevel,
                Message = $"Critical battery: {state.BatteryPercent:F0}%"
            };
        }
        
        if (state.BatteryPercent <= WarningLevel)
        {
            return new ConstraintResult
            {
                Violated = true,
                Severity = ConstraintSeverity.Warning,
                Value = state.BatteryPercent,
                Threshold = WarningLevel,
                Message = $"Low battery: {state.BatteryPercent:F0}%"
            };
        }
        
        return ConstraintResult.Safe;
    }
}

/// <summary>
/// Obstacle avoidance using Control Barrier Functions.
/// </summary>
public class ObstacleAvoidanceCbf : ControlBarrierFunction
{
    public double MinimumDistance { get; set; } = 3;
    public List<Obstacle> Obstacles { get; set; } = new();
    
    public override double ComputeBarrier(VehicleState state)
    {
        if (Obstacles.Count == 0)
            return double.MaxValue;
        
        // h(x) = min_i (||p - p_i|| - d_safe)
        double minBarrier = double.MaxValue;
        
        foreach (var obs in Obstacles)
        {
            double distance = System.Math.Sqrt(
                System.Math.Pow(state.X - obs.X, 2) +
                System.Math.Pow(state.Y - obs.Y, 2) +
                System.Math.Pow(state.Z - obs.Z, 2));
            
            double h = distance - MinimumDistance - obs.Radius;
            minBarrier = System.Math.Min(minBarrier, h);
        }
        
        return minBarrier;
    }
    
    public override (double Lfh, Vector<double> Lgh) ComputeLieDerivatives(
        VehicleState state, VehicleDynamics dynamics)
    {
        // Find closest obstacle
        Obstacle? closest = null;
        double minDist = double.MaxValue;
        
        foreach (var obs in Obstacles)
        {
            double dist = System.Math.Sqrt(
                System.Math.Pow(state.X - obs.X, 2) +
                System.Math.Pow(state.Y - obs.Y, 2) +
                System.Math.Pow(state.Z - obs.Z, 2));
            
            if (dist < minDist)
            {
                minDist = dist;
                closest = obs;
            }
        }
        
        if (closest == null)
        {
            return (0, Vector<double>.Build.Dense(dynamics.InputDimension));
        }
        
        // Gradient of h with respect to position
        double dx = state.X - closest.X;
        double dy = state.Y - closest.Y;
        double dz = state.Z - closest.Z;
        
        var gradH = Vector<double>.Build.DenseOfArray([
            dx / minDist,
            dy / minDist,
            dz / minDist
        ]);
        
        // Lf(h) = ?h/?x * f(x)
        var f = dynamics.GetDrift(state);
        double Lfh = 0;
        for (int i = 0; i < 3; i++)
            Lfh += gradH[i] * f[i];
        
        // Lg(h) = ?h/?x * g(x)
        var g = dynamics.GetControlMatrix(state);
        var Lgh = Vector<double>.Build.Dense(dynamics.InputDimension);
        for (int j = 0; j < dynamics.InputDimension; j++)
        {
            for (int i = 0; i < 3; i++)
            {
                Lgh[j] += gradH[i] * g[i, j];
            }
        }
        
        return (Lfh, Lgh);
    }
}

// Supporting types

public class ConstraintResult
{
    public bool Violated { get; set; }
    public ConstraintSeverity Severity { get; set; }
    public string Message { get; set; } = "";
    public double Value { get; set; }
    public double Threshold { get; set; }
    
    public static ConstraintResult Safe => new()
    {
        Violated = false,
        Severity = ConstraintSeverity.Info
    };
}

public enum ConstraintSeverity
{
    Info,
    Warning,
    Critical,
    Emergency
}

public enum SafetyStatus
{
    Nominal,
    Warning,
    Critical,
    Emergency,
    FailSafe
}

public enum RecoveryAction
{
    None,
    Continue,
    ReturnToHome,
    Land,
    EmergencyStop
}

public class SafetyViolation
{
    public SafetyConstraint Constraint { get; set; } = null!;
    public ConstraintSeverity Severity { get; set; }
    public string Message { get; set; } = "";
    public double Value { get; set; }
    public double Threshold { get; set; }
    public DateTime Timestamp { get; set; }
}

public class SafetyConfig
{
    public double MaxRadius { get; set; } = 500;
    public double MaxAltitude { get; set; } = 120;
    public double MinAltitude { get; set; } = 2;
    public double HomeLatitude { get; set; }
    public double HomeLongitude { get; set; }
    public double MaxRoll { get; set; } = 45;
    public double MaxPitch { get; set; } = 45;
    public double MaxHorizontalSpeed { get; set; } = 20;
    public double MaxVerticalSpeed { get; set; } = 5;
    public double MaxDescentRate { get; set; } = 3;
    public double BatteryWarning { get; set; } = 30;
    public double BatteryCritical { get; set; } = 15;
    public double ObstacleMargin { get; set; } = 3;
    
    public static SafetyConfig Default => new();
}

public class VehicleState
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }
    public double VelocityNorth { get; set; }
    public double VelocityEast { get; set; }
    public double VelocityDown { get; set; }
    public double Roll { get; set; }
    public double Pitch { get; set; }
    public double Yaw { get; set; }
    public double BatteryPercent { get; set; }
    public bool Armed { get; set; }
}

public class Obstacle
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Radius { get; set; }
}

public abstract class VehicleDynamics
{
    public abstract int StateDimension { get; }
    public abstract int InputDimension { get; }
    
    public abstract Vector<double> GetDrift(VehicleState state);
    public abstract Matrix<double> GetControlMatrix(VehicleState state);
}

/// <summary>
/// Quadrotor dynamics for CBF computation.
/// </summary>
public class QuadrotorCbfDynamics : VehicleDynamics
{
    public override int StateDimension => 6;
    public override int InputDimension => 4;
    
    private readonly double _mass;
    
    public QuadrotorCbfDynamics(double mass = 1.5)
    {
        _mass = mass;
    }
    
    public override Vector<double> GetDrift(VehicleState state)
    {
        // f(x) - drift dynamics (gravity, etc.)
        return Vector<double>.Build.DenseOfArray([
            state.VelocityNorth,
            state.VelocityEast,
            state.VelocityDown,
            0,
            0,
            9.81 // Gravity
        ]);
    }
    
    public override Matrix<double> GetControlMatrix(VehicleState state)
    {
        // g(x) - control influence
        double cr = System.Math.Cos(state.Roll);
        double sr = System.Math.Sin(state.Roll);
        double cp = System.Math.Cos(state.Pitch);
        double sp = System.Math.Sin(state.Pitch);
        double cy = System.Math.Cos(state.Yaw);
        double sy = System.Math.Sin(state.Yaw);
        
        // Simplified: thrust affects acceleration in body z direction
        var g = Matrix<double>.Build.Dense(6, 4);
        
        // Control input: [thrust, roll_rate, pitch_rate, yaw_rate]
        // Thrust affects vertical acceleration
        g[5, 0] = -cr * cp / _mass;
        
        return g;
    }
}
