using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Models;

/// <summary>
/// Motion model type enumeration.
/// </summary>
public enum MotionModelType
{
    Unicycle2D,
    ConstantVelocity2D,
    YawOnlyStrapdown
}

/// <summary>
/// Result of a motion model prediction.
/// </summary>
public class MotionModelResult
{
    /// <summary>
    /// Predicted state.
    /// </summary>
    public required Vector<double> PredictedState { get; init; }

    /// <summary>
    /// State transition Jacobian F (df/dx).
    /// </summary>
    public required Matrix<double> F { get; init; }

    /// <summary>
    /// Input Jacobian G (df/du or process noise matrix).
    /// </summary>
    public required Matrix<double> G { get; init; }
}

/// <summary>
/// Interface for motion models.
/// </summary>
public interface IMotionModel
{
    /// <summary>
    /// Gets the state dimension.
    /// </summary>
    int StateDimension { get; }

    /// <summary>
    /// Gets the input dimension.
    /// </summary>
    int InputDimension { get; }

    /// <summary>
    /// Gets the process noise dimension.
    /// </summary>
    int ProcessNoiseDimension { get; }

    /// <summary>
    /// Gets the state variable names.
    /// </summary>
    string[] StateNames { get; }

    /// <summary>
    /// Gets the input variable names.
    /// </summary>
    string[] InputNames { get; }

    /// <summary>
    /// Predicts the next state given current state, input, and time step.
    /// </summary>
    MotionModelResult Predict(Vector<double> state, Vector<double> input, double dt);

    /// <summary>
    /// Gets the default initial state.
    /// </summary>
    Vector<double> DefaultInitialState { get; }

    /// <summary>
    /// Gets the default process noise covariance Q.
    /// </summary>
    Matrix<double> DefaultProcessNoise { get; }
}

/// <summary>
/// Unicycle 2D motion model.
/// State: [px, py, theta]
/// Input: [v, omega]
/// f(x,u,dt) = [px + v*cos(theta)*dt, py + v*sin(theta)*dt, theta + omega*dt]
/// </summary>
public class Unicycle2DModel : IMotionModel
{
    public int StateDimension => 3;
    public int InputDimension => 2;
    public int ProcessNoiseDimension => 2;

    public string[] StateNames => ["px (m)", "py (m)", "theta (rad)"];
    public string[] InputNames => ["v (m/s)", "omega (rad/s)"];

    public Vector<double> DefaultInitialState => 
        Vector<double>.Build.Dense([0, 0, 0]);

    public Matrix<double> DefaultProcessNoise => 
        MatrixUtilities.Diagonal(0.01, 0.01); // velocity, omega noise

    public MotionModelResult Predict(Vector<double> state, Vector<double> input, double dt)
    {
        if (state.Count != 3)
            throw new ArgumentException("State must have 3 elements [px, py, theta].");
        if (input.Count != 2)
            throw new ArgumentException("Input must have 2 elements [v, omega].");

        double px = state[0];
        double py = state[1];
        double theta = state[2];
        double v = input[0];
        double omega = input[1];

        double cosTheta = System.Math.Cos(theta);
        double sinTheta = System.Math.Sin(theta);

        // Predicted state
        var predictedState = Vector<double>.Build.Dense([
            px + v * cosTheta * dt,
            py + v * sinTheta * dt,
            theta + omega * dt
        ]);

        // State transition Jacobian F = df/dx
        // F = [[1, 0, -v*sin(theta)*dt],
        //      [0, 1,  v*cos(theta)*dt],
        //      [0, 0, 1]]
        var F = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1, 0, -v * sinTheta * dt },
            { 0, 1,  v * cosTheta * dt },
            { 0, 0, 1 }
        });

        // Input Jacobian G = df/du
        // G = [[cos(theta)*dt, 0],
        //      [sin(theta)*dt, 0],
        //      [0, dt]]
        var G = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { cosTheta * dt, 0 },
            { sinTheta * dt, 0 },
            { 0, dt }
        });

        return new MotionModelResult
        {
            PredictedState = predictedState,
            F = F,
            G = G
        };
    }
}

/// <summary>
/// 2D constant velocity motion model.
/// State: [px, py, vx, vy]
/// Input: [ax, ay] (acceleration as process noise)
/// </summary>
public class ConstantVelocity2DModel : IMotionModel
{
    public int StateDimension => 4;
    public int InputDimension => 2;
    public int ProcessNoiseDimension => 2;

    public string[] StateNames => ["px (m)", "py (m)", "vx (m/s)", "vy (m/s)"];
    public string[] InputNames => ["ax (m/s²)", "ay (m/s²)"];

    public Vector<double> DefaultInitialState => 
        Vector<double>.Build.Dense([0, 0, 0, 0]);

    public Matrix<double> DefaultProcessNoise => 
        MatrixUtilities.Diagonal(0.1, 0.1); // acceleration noise

    public MotionModelResult Predict(Vector<double> state, Vector<double> input, double dt)
    {
        if (state.Count != 4)
            throw new ArgumentException("State must have 4 elements [px, py, vx, vy].");
        if (input.Count != 2)
            throw new ArgumentException("Input must have 2 elements [ax, ay].");

        double px = state[0];
        double py = state[1];
        double vx = state[2];
        double vy = state[3];
        double ax = input[0];
        double ay = input[1];

        // Predicted state with acceleration
        var predictedState = Vector<double>.Build.Dense([
            px + vx * dt + 0.5 * ax * dt * dt,
            py + vy * dt + 0.5 * ay * dt * dt,
            vx + ax * dt,
            vy + ay * dt
        ]);

        // State transition Jacobian F
        var F = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1, 0, dt, 0 },
            { 0, 1, 0, dt },
            { 0, 0, 1, 0 },
            { 0, 0, 0, 1 }
        });

        // Input Jacobian G (df/du)
        var G = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0.5 * dt * dt, 0 },
            { 0, 0.5 * dt * dt },
            { dt, 0 },
            { 0, dt }
        });

        return new MotionModelResult
        {
            PredictedState = predictedState,
            F = F,
            G = G
        };
    }
}

/// <summary>
/// Yaw-only strapdown model with gyro bias estimation.
/// State: [yaw, bias_g]
/// Input: [gyro_z] (measured gyro rate)
/// yaw_{k+1} = yaw_k + (gyro - bias_g) * dt
/// bias random walk
/// </summary>
public class YawOnlyStrapdownModel : IMotionModel
{
    public int StateDimension => 2;
    public int InputDimension => 1;
    public int ProcessNoiseDimension => 2;

    public string[] StateNames => ["yaw (rad)", "bias_g (rad/s)"];
    public string[] InputNames => ["gyro_z (rad/s)"];

    public Vector<double> DefaultInitialState => 
        Vector<double>.Build.Dense([0, 0]);

    public Matrix<double> DefaultProcessNoise => 
        MatrixUtilities.Diagonal(0.001, 0.0001); // yaw noise, bias random walk

    public MotionModelResult Predict(Vector<double> state, Vector<double> input, double dt)
    {
        if (state.Count != 2)
            throw new ArgumentException("State must have 2 elements [yaw, bias_g].");
        if (input.Count != 1)
            throw new ArgumentException("Input must have 1 element [gyro_z].");

        double yaw = state[0];
        double biasG = state[1];
        double gyroZ = input[0];

        // Predicted state
        var predictedState = Vector<double>.Build.Dense([
            yaw + (gyroZ - biasG) * dt,
            biasG // bias random walk (constant + noise)
        ]);

        // State transition Jacobian F
        var F = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1, -dt },
            { 0, 1 }
        });

        // Process noise Jacobian G
        // Noise enters yaw through dt and bias through sqrt(dt)
        var G = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { dt, 0 },
            { 0, 1 }
        });

        return new MotionModelResult
        {
            PredictedState = predictedState,
            F = F,
            G = G
        };
    }
}

/// <summary>
/// Factory for creating motion models.
/// </summary>
public static class MotionModelFactory
{
    public static IMotionModel Create(MotionModelType type) => type switch
    {
        MotionModelType.Unicycle2D => new Unicycle2DModel(),
        MotionModelType.ConstantVelocity2D => new ConstantVelocity2DModel(),
        MotionModelType.YawOnlyStrapdown => new YawOnlyStrapdownModel(),
        _ => throw new ArgumentException($"Unknown motion model type: {type}")
    };
}
