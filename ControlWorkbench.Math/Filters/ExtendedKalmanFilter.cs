using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Models;

namespace ControlWorkbench.Math.Filters;

/// <summary>
/// Result of an EKF prediction step.
/// </summary>
public class EkfPredictionResult
{
    /// <summary>
    /// Predicted state x??.
    /// </summary>
    public required Vector<double> PredictedState { get; init; }

    /// <summary>
    /// Predicted covariance P?.
    /// </summary>
    public required Matrix<double> PredictedCovariance { get; init; }

    /// <summary>
    /// State transition Jacobian F.
    /// </summary>
    public required Matrix<double> F { get; init; }

    /// <summary>
    /// Process noise Jacobian G.
    /// </summary>
    public required Matrix<double> G { get; init; }
}

/// <summary>
/// Result of an EKF update step.
/// </summary>
public class EkfUpdateResult
{
    /// <summary>
    /// Updated state x??.
    /// </summary>
    public required Vector<double> UpdatedState { get; init; }

    /// <summary>
    /// Updated covariance P?.
    /// </summary>
    public required Matrix<double> UpdatedCovariance { get; init; }

    /// <summary>
    /// Innovation (measurement residual) y = z - h(x??).
    /// </summary>
    public required Vector<double> Innovation { get; init; }

    /// <summary>
    /// Innovation covariance S = H*P?*H' + R.
    /// </summary>
    public required Matrix<double> InnovationCovariance { get; init; }

    /// <summary>
    /// Kalman gain K = P?*H'*S?¹.
    /// </summary>
    public required Matrix<double> KalmanGain { get; init; }

    /// <summary>
    /// Measurement Jacobian H.
    /// </summary>
    public required Matrix<double> H { get; init; }
}

/// <summary>
/// Extended Kalman Filter implementation.
/// </summary>
public class ExtendedKalmanFilter
{
    private Vector<double> _state;
    private Matrix<double> _covariance;
    private readonly IMotionModel _motionModel;

    /// <summary>
    /// Gets the current state estimate.
    /// </summary>
    public Vector<double> State => _state;

    /// <summary>
    /// Gets the current covariance matrix.
    /// </summary>
    public Matrix<double> Covariance => _covariance;

    /// <summary>
    /// Gets the motion model.
    /// </summary>
    public IMotionModel MotionModel => _motionModel;

    /// <summary>
    /// Creates a new EKF with the specified motion model.
    /// </summary>
    public ExtendedKalmanFilter(IMotionModel motionModel)
    {
        _motionModel = motionModel;
        _state = motionModel.DefaultInitialState.Clone();
        _covariance = MatrixUtilities.Identity(motionModel.StateDimension);
    }

    /// <summary>
    /// Creates a new EKF with initial state and covariance.
    /// </summary>
    public ExtendedKalmanFilter(IMotionModel motionModel, Vector<double> initialState, Matrix<double> initialCovariance)
    {
        _motionModel = motionModel;
        
        if (initialState.Count != motionModel.StateDimension)
            throw new ArgumentException($"Initial state dimension mismatch. Expected {motionModel.StateDimension}, got {initialState.Count}.");
        
        if (initialCovariance.RowCount != motionModel.StateDimension || initialCovariance.ColumnCount != motionModel.StateDimension)
            throw new ArgumentException($"Initial covariance dimension mismatch. Expected {motionModel.StateDimension}x{motionModel.StateDimension}.");

        _state = initialState.Clone();
        _covariance = initialCovariance.Clone();
    }

    /// <summary>
    /// Resets the filter to the specified state and covariance.
    /// </summary>
    public void Reset(Vector<double> state, Matrix<double> covariance)
    {
        if (state.Count != _motionModel.StateDimension)
            throw new ArgumentException($"State dimension mismatch. Expected {_motionModel.StateDimension}, got {state.Count}.");
        
        if (covariance.RowCount != _motionModel.StateDimension || covariance.ColumnCount != _motionModel.StateDimension)
            throw new ArgumentException($"Covariance dimension mismatch. Expected {_motionModel.StateDimension}x{_motionModel.StateDimension}.");

        _state = state.Clone();
        _covariance = covariance.Clone();
    }

    /// <summary>
    /// Performs the EKF prediction step.
    /// P? = F*P*F' + G*Q*G'
    /// </summary>
    public EkfPredictionResult Predict(Vector<double> input, double dt, Matrix<double> processNoise)
    {
        // Get motion model prediction and Jacobians
        var result = _motionModel.Predict(_state, input, dt);

        // Compute predicted covariance
        // P? = F * P * F' + G * Q * G'
        var F = result.F;
        var G = result.G;
        var P = _covariance;

        var predictedCovariance = F * P * F.Transpose() + G * processNoise * G.Transpose();

        // Force symmetry to prevent numerical drift
        predictedCovariance = MatrixUtilities.ForceSymmetric(predictedCovariance);

        // Update internal state
        _state = result.PredictedState;
        _covariance = predictedCovariance;

        return new EkfPredictionResult
        {
            PredictedState = _state.Clone(),
            PredictedCovariance = _covariance.Clone(),
            F = F,
            G = G
        };
    }

    /// <summary>
    /// Performs the EKF update step with a measurement.
    /// </summary>
    public EkfUpdateResult Update(
        IMeasurementModel measurementModel, 
        Vector<double> measurement, 
        Matrix<double> measurementNoise,
        object? measurementParameters = null)
    {
        // Evaluate measurement model
        var measResult = measurementModel.Evaluate(_state, measurementParameters);
        var H = measResult.H;
        var expectedMeasurement = measResult.ExpectedMeasurement;

        // Innovation
        var innovation = measurement - expectedMeasurement;

        // Innovation covariance: S = H * P * H' + R
        var S = H * _covariance * H.Transpose() + measurementNoise;

        // Kalman gain: K = P * H' * S?¹
        var K = _covariance * H.Transpose() * S.Inverse();

        // State update: x? = x? + K * y
        var updatedState = _state + K * innovation;

        // Covariance update using Joseph form for numerical stability:
        // P? = (I - K*H) * P * (I - K*H)' + K * R * K'
        var I = MatrixUtilities.Identity(_motionModel.StateDimension);
        var IminusKH = I - K * H;
        var updatedCovariance = IminusKH * _covariance * IminusKH.Transpose() + K * measurementNoise * K.Transpose();

        // Force symmetry
        updatedCovariance = MatrixUtilities.ForceSymmetric(updatedCovariance);

        // Update internal state
        _state = updatedState;
        _covariance = updatedCovariance;

        return new EkfUpdateResult
        {
            UpdatedState = _state.Clone(),
            UpdatedCovariance = _covariance.Clone(),
            Innovation = innovation,
            InnovationCovariance = S,
            KalmanGain = K,
            H = H
        };
    }

    /// <summary>
    /// Performs a single prediction step without updating internal state (for analysis).
    /// </summary>
    public static EkfPredictionResult ComputePrediction(
        IMotionModel motionModel,
        Vector<double> state,
        Matrix<double> covariance,
        Vector<double> input,
        double dt,
        Matrix<double> processNoise)
    {
        var result = motionModel.Predict(state, input, dt);
        var F = result.F;
        var G = result.G;

        var predictedCovariance = F * covariance * F.Transpose() + G * processNoise * G.Transpose();
        predictedCovariance = MatrixUtilities.ForceSymmetric(predictedCovariance);

        return new EkfPredictionResult
        {
            PredictedState = result.PredictedState,
            PredictedCovariance = predictedCovariance,
            F = F,
            G = G
        };
    }

    /// <summary>
    /// Performs a single update step without updating internal state (for analysis).
    /// </summary>
    public static EkfUpdateResult ComputeUpdate(
        IMeasurementModel measurementModel,
        Vector<double> state,
        Matrix<double> covariance,
        Vector<double> measurement,
        Matrix<double> measurementNoise,
        object? measurementParameters = null)
    {
        var measResult = measurementModel.Evaluate(state, measurementParameters);
        var H = measResult.H;
        var expectedMeasurement = measResult.ExpectedMeasurement;

        var innovation = measurement - expectedMeasurement;
        var S = H * covariance * H.Transpose() + measurementNoise;
        var K = covariance * H.Transpose() * S.Inverse();
        var updatedState = state + K * innovation;

        var I = MatrixUtilities.Identity(state.Count);
        var IminusKH = I - K * H;
        var updatedCovariance = IminusKH * covariance * IminusKH.Transpose() + K * measurementNoise * K.Transpose();
        updatedCovariance = MatrixUtilities.ForceSymmetric(updatedCovariance);

        return new EkfUpdateResult
        {
            UpdatedState = updatedState,
            UpdatedCovariance = updatedCovariance,
            Innovation = innovation,
            InnovationCovariance = S,
            KalmanGain = K,
            H = H
        };
    }
}
