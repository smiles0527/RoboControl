using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Models;

/// <summary>
/// Measurement model type enumeration.
/// </summary>
public enum MeasurementModelType
{
    GpsPosition2D,
    YawMeasurement,
    RangeBearing
}

/// <summary>
/// Result of a measurement model evaluation.
/// </summary>
public class MeasurementModelResult
{
    /// <summary>
    /// Expected measurement h(x).
    /// </summary>
    public required Vector<double> ExpectedMeasurement { get; init; }

    /// <summary>
    /// Measurement Jacobian H (dh/dx).
    /// </summary>
    public required Matrix<double> H { get; init; }
}

/// <summary>
/// Interface for measurement models.
/// </summary>
public interface IMeasurementModel
{
    /// <summary>
    /// Gets the measurement dimension.
    /// </summary>
    int MeasurementDimension { get; }

    /// <summary>
    /// Gets the expected state dimension.
    /// </summary>
    int ExpectedStateDimension { get; }

    /// <summary>
    /// Gets the measurement variable names.
    /// </summary>
    string[] MeasurementNames { get; }

    /// <summary>
    /// Evaluates the measurement model at the given state.
    /// </summary>
    MeasurementModelResult Evaluate(Vector<double> state, object? parameters = null);

    /// <summary>
    /// Gets the default measurement noise covariance R.
    /// </summary>
    Matrix<double> DefaultMeasurementNoise { get; }
}

/// <summary>
/// GPS position measurement model.
/// Measures [px, py] directly from state.
/// Works with states where first two elements are px, py.
/// </summary>
public class GpsPosition2DModel : IMeasurementModel
{
    public int MeasurementDimension => 2;
    public int ExpectedStateDimension => 2; // At least 2 (px, py)

    public string[] MeasurementNames => ["px_gps (m)", "py_gps (m)"];

    public Matrix<double> DefaultMeasurementNoise =>
        MatrixUtilities.Diagonal(4.0, 4.0); // 2m std dev

    public MeasurementModelResult Evaluate(Vector<double> state, object? parameters = null)
    {
        if (state.Count < 2)
            throw new ArgumentException("State must have at least 2 elements.");

        // h(x) = [px, py]
        var expectedMeasurement = Vector<double>.Build.Dense([state[0], state[1]]);

        // H = [[1, 0, 0, ...], [0, 1, 0, ...]]
        var H = Matrix<double>.Build.Dense(2, state.Count);
        H[0, 0] = 1;
        H[1, 1] = 1;

        return new MeasurementModelResult
        {
            ExpectedMeasurement = expectedMeasurement,
            H = H
        };
    }
}

/// <summary>
/// Yaw measurement model (e.g., from magnetometer).
/// Works with states where yaw is at a specific index.
/// </summary>
public class YawMeasurementModel : IMeasurementModel
{
    /// <summary>
    /// Index of yaw in the state vector.
    /// </summary>
    public int YawIndex { get; set; } = 2;

    public int MeasurementDimension => 1;
    public int ExpectedStateDimension => 3; // At least 3 for unicycle

    public string[] MeasurementNames => ["yaw_mag (rad)"];

    public Matrix<double> DefaultMeasurementNoise =>
        MatrixUtilities.Diagonal(0.01); // ~5.7 deg std dev

    public MeasurementModelResult Evaluate(Vector<double> state, object? parameters = null)
    {
        if (state.Count <= YawIndex)
            throw new ArgumentException($"State must have at least {YawIndex + 1} elements.");

        // h(x) = [yaw]
        var expectedMeasurement = Vector<double>.Build.Dense([state[YawIndex]]);

        // H = [0, 0, 1, 0, ...]
        var H = Matrix<double>.Build.Dense(1, state.Count);
        H[0, YawIndex] = 1;

        return new MeasurementModelResult
        {
            ExpectedMeasurement = expectedMeasurement,
            H = H
        };
    }
}

/// <summary>
/// Parameters for range-bearing measurement model.
/// </summary>
public record RangeBearingParameters(double BeaconX, double BeaconY);

/// <summary>
/// Range-bearing measurement to a known beacon.
/// State must have [px, py, theta] at indices 0, 1, 2.
/// Measures [range, bearing] where:
///   range = sqrt((px-bx)² + (py-by)²)
///   bearing = atan2(py-by, px-bx) - theta
/// </summary>
public class RangeBearingModel : IMeasurementModel
{
    public int MeasurementDimension => 2;
    public int ExpectedStateDimension => 3;

    public string[] MeasurementNames => ["range (m)", "bearing (rad)"];

    public Matrix<double> DefaultMeasurementNoise =>
        MatrixUtilities.Diagonal(0.25, 0.01); // 0.5m, ~5.7 deg

    public MeasurementModelResult Evaluate(Vector<double> state, object? parameters = null)
    {
        if (state.Count < 3)
            throw new ArgumentException("State must have at least 3 elements [px, py, theta].");

        if (parameters is not RangeBearingParameters rbParams)
            throw new ArgumentException("RangeBearingParameters required.");

        double px = state[0];
        double py = state[1];
        double theta = state[2];
        double bx = rbParams.BeaconX;
        double by = rbParams.BeaconY;

        double dx = px - bx;
        double dy = py - by;
        double range = System.Math.Sqrt(dx * dx + dy * dy);
        double bearing = System.Math.Atan2(dy, dx) - theta;

        // Normalize bearing to [-pi, pi]
        while (bearing > System.Math.PI) bearing -= 2 * System.Math.PI;
        while (bearing < -System.Math.PI) bearing += 2 * System.Math.PI;

        var expectedMeasurement = Vector<double>.Build.Dense([range, bearing]);

        // Jacobian H
        // dr/dpx = dx/range, dr/dpy = dy/range, dr/dtheta = 0
        // db/dpx = -dy/range², db/dpy = dx/range², db/dtheta = -1
        double r2 = range * range;
        if (r2 < 1e-10) r2 = 1e-10; // Avoid division by zero

        var H = Matrix<double>.Build.Dense(2, state.Count);
        H[0, 0] = dx / range;
        H[0, 1] = dy / range;
        H[0, 2] = 0;

        H[1, 0] = -dy / r2;
        H[1, 1] = dx / r2;
        H[1, 2] = -1;

        return new MeasurementModelResult
        {
            ExpectedMeasurement = expectedMeasurement,
            H = H
        };
    }
}

/// <summary>
/// Factory for creating measurement models.
/// </summary>
public static class MeasurementModelFactory
{
    public static IMeasurementModel Create(MeasurementModelType type) => type switch
    {
        MeasurementModelType.GpsPosition2D => new GpsPosition2DModel(),
        MeasurementModelType.YawMeasurement => new YawMeasurementModel(),
        MeasurementModelType.RangeBearing => new RangeBearingModel(),
        _ => throw new ArgumentException($"Unknown measurement model type: {type}")
    };
}
