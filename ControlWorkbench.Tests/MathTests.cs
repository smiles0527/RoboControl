using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math;
using ControlWorkbench.Math.Filters;
using ControlWorkbench.Math.Jacobians;
using ControlWorkbench.Math.Models;
using System;

namespace ControlWorkbench.Tests;

public class MathTests
{
    [Fact]
    public void Unicycle2D_Jacobian_MatchesNumerical()
    {
        var model = new Unicycle2DModel();
        var state = Vector<double>.Build.Dense([1.0, 2.0, 0.5]);
        var input = Vector<double>.Build.Dense([1.5, 0.2]);
        double dt = 0.1;

        var result = model.Predict(state, input, dt);
        var analyticF = result.F;

        Vector<double> f(Vector<double> x) => model.Predict(x, input, dt).PredictedState;
        var numericalF = NumericalJacobian.Compute(f, state);

        double maxDiff = NumericalJacobian.MaxAbsoluteDifference(analyticF, numericalF);
        Assert.True(maxDiff < 1e-5, $"Max difference: {maxDiff}");
    }

    [Fact]
    public void ConstantVelocity2D_Jacobian_MatchesNumerical()
    {
        var model = new ConstantVelocity2DModel();
        var state = Vector<double>.Build.Dense([5.0, -3.0, 1.0, 0.5]);
        var input = Vector<double>.Build.Dense([0.1, -0.2]);
        double dt = 0.05;

        var result = model.Predict(state, input, dt);
        var analyticF = result.F;

        Vector<double> f(Vector<double> x) => model.Predict(x, input, dt).PredictedState;
        var numericalF = NumericalJacobian.Compute(f, state);

        double maxDiff = NumericalJacobian.MaxAbsoluteDifference(analyticF, numericalF);
        Assert.True(maxDiff < 1e-5, $"Max difference: {maxDiff}");
    }

    [Fact]
    public void YawStrapdown_Jacobian_MatchesNumerical()
    {
        var model = new YawOnlyStrapdownModel();
        var state = Vector<double>.Build.Dense([0.3, 0.01]);
        var input = Vector<double>.Build.Dense([0.15]);
        double dt = 0.01;

        var result = model.Predict(state, input, dt);
        var analyticF = result.F;

        Vector<double> f(Vector<double> x) => model.Predict(x, input, dt).PredictedState;
        var numericalF = NumericalJacobian.Compute(f, state);

        double maxDiff = NumericalJacobian.MaxAbsoluteDifference(analyticF, numericalF);
        Assert.True(maxDiff < 1e-5, $"Max difference: {maxDiff}");
    }

    [Fact]
    public void GpsPosition_MeasurementJacobian_MatchesNumerical()
    {
        var model = new GpsPosition2DModel();
        var state = Vector<double>.Build.Dense([10.0, 5.0, 0.5]);

        var result = model.Evaluate(state);
        var analyticH = result.H;

        Vector<double> h(Vector<double> x) => model.Evaluate(x).ExpectedMeasurement;
        var numericalH = NumericalJacobian.Compute(h, state);

        double maxDiff = NumericalJacobian.MaxAbsoluteDifference(analyticH, numericalH);
        Assert.True(maxDiff < 1e-5, $"Max difference: {maxDiff}");
    }

    [Fact]
    public void RangeBearing_MeasurementJacobian_MatchesNumerical()
    {
        var model = new RangeBearingModel();
        var state = Vector<double>.Build.Dense([5.0, 3.0, 0.2]);
        var beaconParams = new RangeBearingParameters(10.0, 8.0);

        var result = model.Evaluate(state, beaconParams);
        var analyticH = result.H;

        Vector<double> h(Vector<double> x) => model.Evaluate(x, beaconParams).ExpectedMeasurement;
        var numericalH = NumericalJacobian.Compute(h, state);

        double maxDiff = NumericalJacobian.MaxAbsoluteDifference(analyticH, numericalH);
        Assert.True(maxDiff < 1e-4, $"Max difference: {maxDiff}");
    }

    [Fact]
    public void EKF_CovarianceStaysSymmetric()
    {
        var motionModel = new Unicycle2DModel();
        var ekf = new ExtendedKalmanFilter(motionModel);
        
        var input = Vector<double>.Build.Dense([1.0, 0.1]);
        var Q = MatrixUtilities.Diagonal(0.01, 0.01);

        for (int i = 0; i < 100; i++)
        {
            ekf.Predict(input, 0.1, Q);
        }

        Assert.True(MatrixUtilities.IsSymmetric(ekf.Covariance, 1e-10));
    }

    [Fact]
    public void EKF_PredictionUpdate_StateChanges()
    {
        var motionModel = new Unicycle2DModel();
        var initialState = Vector<double>.Build.Dense([0, 0, 0]);
        var initialP = MatrixUtilities.Diagonal(1.0, 1.0, 0.1);
        var ekf = new ExtendedKalmanFilter(motionModel, initialState, initialP);
        
        var input = Vector<double>.Build.Dense([1.0, 0.0]); // Move forward at 1 m/s
        var Q = MatrixUtilities.Diagonal(0.01, 0.001);

        ekf.Predict(input, 1.0, Q);

        // After 1 second at 1 m/s, x should be ~1
        Assert.True(ekf.State[0] > 0.9);
        Assert.True(System.Math.Abs(ekf.State[1]) < 0.1);
    }

    [Fact]
    public void ComplementaryFilter_ConvergesToReference()
    {
        var filter = ComplementaryFilter.FromCutoffFrequency(0.5, 0.01);
        
        double gyroRate = 0;
        double referenceAngle = 1.0;

        // Run for 10 seconds
        for (int i = 0; i < 1000; i++)
        {
            filter.Update(gyroRate, referenceAngle, 0.01);
        }

        // Should converge to reference angle
        Assert.True(System.Math.Abs(filter.State - referenceAngle) < 0.01);
    }

    [Fact]
    public void ComplementaryFilter_AlphaCalculation()
    {
        double fc = 0.5; // Hz
        double dt = 0.01; // s
        double alpha = ComplementaryFilter.ComputeAlpha(fc, dt);

        // alpha = tau / (tau + dt) where tau = 1/(2*pi*fc)
        double tau = 1.0 / (2 * System.Math.PI * fc);
        double expectedAlpha = tau / (tau + dt);

        Assert.Equal(expectedAlpha, alpha, 6);
    }

    [Fact]
    public void MatrixUtilities_ForceSymmetric_Works()
    {
        var asymmetric = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1.0, 0.1 },
            { 0.2, 2.0 }
        });

        var symmetric = MatrixUtilities.ForceSymmetric(asymmetric);

        Assert.True(MatrixUtilities.IsSymmetric(symmetric));
        Assert.Equal(0.15, symmetric[0, 1], 10);
        Assert.Equal(0.15, symmetric[1, 0], 10);
    }

    [Fact]
    public void MatrixUtilities_Diagonal_CreatesCorrectMatrix()
    {
        var diag = MatrixUtilities.Diagonal(1.0, 2.0, 3.0);

        Assert.Equal(3, diag.RowCount);
        Assert.Equal(3, diag.ColumnCount);
        Assert.Equal(1.0, diag[0, 0]);
        Assert.Equal(2.0, diag[1, 1]);
        Assert.Equal(3.0, diag[2, 2]);
        Assert.Equal(0.0, diag[0, 1]);
    }
}
