using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math;

/// <summary>
/// Matrix utility functions for control and estimation.
/// Uses MathNet.Numerics for underlying operations.
/// All matrices use double precision for mathematical accuracy.
/// </summary>
public static class MatrixUtilities
{
    /// <summary>
    /// Creates a matrix from a 2D array.
    /// </summary>
    public static Matrix<double> FromArray(double[,] data)
    {
        return Matrix<double>.Build.DenseOfArray(data);
    }

    /// <summary>
    /// Creates a diagonal matrix from diagonal values.
    /// </summary>
    public static Matrix<double> Diagonal(params double[] diagonalValues)
    {
        return Matrix<double>.Build.DenseOfDiagonalArray(diagonalValues);
    }

    /// <summary>
    /// Creates an identity matrix of the specified size.
    /// </summary>
    public static Matrix<double> Identity(int size)
    {
        return Matrix<double>.Build.DenseIdentity(size);
    }

    /// <summary>
    /// Creates a zero matrix of the specified size.
    /// </summary>
    public static Matrix<double> Zeros(int rows, int columns)
    {
        return Matrix<double>.Build.Dense(rows, columns);
    }

    /// <summary>
    /// Creates a vector from array.
    /// </summary>
    public static Vector<double> Vector(params double[] values)
    {
        return MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(values);
    }

    /// <summary>
    /// Checks if a matrix is symmetric within tolerance.
    /// </summary>
    public static bool IsSymmetric(Matrix<double> matrix, double tolerance = 1e-10)
    {
        if (matrix.RowCount != matrix.ColumnCount) return false;
        
        for (int i = 0; i < matrix.RowCount; i++)
        {
            for (int j = i + 1; j < matrix.ColumnCount; j++)
            {
                if (System.Math.Abs(matrix[i, j] - matrix[j, i]) > tolerance)
                    return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Forces a matrix to be symmetric by averaging with its transpose.
    /// </summary>
    public static Matrix<double> ForceSymmetric(Matrix<double> matrix)
    {
        return (matrix + matrix.Transpose()) * 0.5;
    }

    /// <summary>
    /// Checks if a matrix is positive semi-definite (all eigenvalues >= 0).
    /// </summary>
    public static bool IsPositiveSemiDefinite(Matrix<double> matrix, double tolerance = -1e-10)
    {
        if (matrix.RowCount != matrix.ColumnCount) return false;
        
        var evd = matrix.Evd();
        foreach (var eigenvalue in evd.EigenValues)
        {
            if (eigenvalue.Real < tolerance)
                return false;
        }
        return true;
    }

    /// <summary>
    /// Computes the trace of a matrix.
    /// </summary>
    public static double Trace(Matrix<double> matrix)
    {
        return matrix.Trace();
    }

    /// <summary>
    /// Exports a matrix to a 2D array.
    /// </summary>
    public static double[,] ToArray(Matrix<double> matrix)
    {
        return matrix.ToArray();
    }

    /// <summary>
    /// Formats a matrix as a string for display.
    /// </summary>
    public static string Format(Matrix<double> matrix, string numberFormat = "F4")
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"[{matrix.RowCount}x{matrix.ColumnCount}]");
        for (int i = 0; i < matrix.RowCount; i++)
        {
            sb.Append("[ ");
            for (int j = 0; j < matrix.ColumnCount; j++)
            {
                sb.Append(matrix[i, j].ToString(numberFormat).PadLeft(10));
                if (j < matrix.ColumnCount - 1) sb.Append(", ");
            }
            sb.AppendLine(" ]");
        }
        return sb.ToString();
    }

    /// <summary>
    /// Formats a vector as a string for display.
    /// </summary>
    public static string Format(Vector<double> vector, string numberFormat = "F4")
    {
        var sb = new System.Text.StringBuilder();
        sb.Append("[ ");
        for (int i = 0; i < vector.Count; i++)
        {
            sb.Append(vector[i].ToString(numberFormat).PadLeft(10));
            if (i < vector.Count - 1) sb.Append(", ");
        }
        sb.Append(" ]");
        return sb.ToString();
    }
}
