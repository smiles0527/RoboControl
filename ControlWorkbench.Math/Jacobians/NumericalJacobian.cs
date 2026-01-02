using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Jacobians;

/// <summary>
/// Provides numerical Jacobian computation using finite differences.
/// Useful for verifying analytic Jacobians.
/// </summary>
public static class NumericalJacobian
{
    /// <summary>
    /// Computes the Jacobian of a vector function using central differences.
    /// </summary>
    /// <param name="f">The function f: R^n -> R^m</param>
    /// <param name="x">The point at which to evaluate the Jacobian</param>
    /// <param name="epsilon">The perturbation size (default 1e-7)</param>
    /// <returns>The m x n Jacobian matrix</returns>
    public static Matrix<double> Compute(
        Func<Vector<double>, Vector<double>> f,
        Vector<double> x,
        double epsilon = 1e-7)
    {
        Vector<double> fx = f(x);
        int m = fx.Count;
        int n = x.Count;

        var jacobian = Matrix<double>.Build.Dense(m, n);

        for (int j = 0; j < n; j++)
        {
            var xPlus = x.Clone();
            var xMinus = x.Clone();
            xPlus[j] += epsilon;
            xMinus[j] -= epsilon;

            var fPlus = f(xPlus);
            var fMinus = f(xMinus);

            for (int i = 0; i < m; i++)
            {
                jacobian[i, j] = (fPlus[i] - fMinus[i]) / (2.0 * epsilon);
            }
        }

        return jacobian;
    }

    /// <summary>
    /// Computes the maximum absolute difference between two matrices.
    /// </summary>
    public static double MaxAbsoluteDifference(Matrix<double> a, Matrix<double> b)
    {
        if (a.RowCount != b.RowCount || a.ColumnCount != b.ColumnCount)
            throw new ArgumentException("Matrices must have the same dimensions.");

        double maxDiff = 0;
        for (int i = 0; i < a.RowCount; i++)
        {
            for (int j = 0; j < a.ColumnCount; j++)
            {
                double diff = System.Math.Abs(a[i, j] - b[i, j]);
                if (diff > maxDiff) maxDiff = diff;
            }
        }
        return maxDiff;
    }

    /// <summary>
    /// Validates an analytic Jacobian against a numerical one.
    /// </summary>
    /// <returns>True if the difference is within tolerance.</returns>
    public static bool Validate(
        Matrix<double> analyticJacobian,
        Func<Vector<double>, Vector<double>> f,
        Vector<double> x,
        double tolerance = 1e-5)
    {
        var numericalJacobian = Compute(f, x);
        double maxDiff = MaxAbsoluteDifference(analyticJacobian, numericalJacobian);
        return maxDiff <= tolerance;
    }
}
