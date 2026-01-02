using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Control;

/// <summary>
/// Result of LQR design.
/// </summary>
public class LqrResult
{
    /// <summary>
    /// State feedback gain K.
    /// </summary>
    public required Matrix<double> K { get; init; }

    /// <summary>
    /// Solution to the Riccati equation P.
    /// </summary>
    public required Matrix<double> P { get; init; }

    /// <summary>
    /// Closed-loop eigenvalues (A - B*K).
    /// </summary>
    public required System.Numerics.Complex[] ClosedLoopPoles { get; init; }

    /// <summary>
    /// Whether the design converged successfully.
    /// </summary>
    public bool Success { get; init; }

    /// <summary>
    /// Error message if design failed.
    /// </summary>
    public string? ErrorMessage { get; init; }
}

/// <summary>
/// LQR (Linear Quadratic Regulator) design.
/// Computes optimal state feedback gain K that minimizes J = integral(x'Qx + u'Ru)dt.
/// </summary>
public static class LqrDesign
{
    /// <summary>
    /// Solves the continuous-time algebraic Riccati equation (CARE) using gradient descent.
    /// A'P + PA - PBR^-1B'P + Q = 0
    /// </summary>
    public static LqrResult SolveContinuous(
        Matrix<double> A,
        Matrix<double> B,
        Matrix<double> Q,
        Matrix<double> R,
        int maxIterations = 10000,
        double tolerance = 1e-9)
    {
        ValidateDimensions(A, B, Q, R);

        int n = A.RowCount;
        int m = B.ColumnCount;

        try
        {
            var Rinv = R.Inverse();
            var BRinvBt = B * Rinv * B.Transpose();
            var At = A.Transpose();
            
            // Initialize P = Q
            var P = Q.Clone();
            
            // Ensure P is positive definite to start
            for (int i = 0; i < n; i++)
            {
                P[i, i] = System.Math.Max(P[i, i], 0.1);
            }

            double prevResidualNorm = double.MaxValue;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // Compute Riccati residual: res = A'P + PA - PBR^-1B'P + Q
                var residual = At * P + P * A - P * BRinvBt * P + Q;
                
                double residualNorm = residual.FrobeniusNorm();
                
                if (residualNorm < tolerance)
                {
                    return ComputeResult(A, B, P, Rinv, true);
                }
                
                // Adaptive step size
                double alpha = 0.005;
                if (residualNorm > prevResidualNorm)
                {
                    alpha *= 0.5;
                }
                
                // Gradient step: P = P - alpha * residual
                P = P - residual * alpha;
                
                // Ensure symmetry
                P = (P + P.Transpose()) * 0.5;
                
                // Clamp diagonal to prevent negative values
                for (int i = 0; i < n; i++)
                {
                    if (P[i, i] < 1e-6)
                    {
                        P[i, i] = 1e-6;
                    }
                }
                
                prevResidualNorm = residualNorm;
            }

            // Check if result is stable even if not converged
            var result = ComputeResult(A, B, P, Rinv, false);
            if (result.ClosedLoopPoles.All(p => p.Real < 0))
            {
                return new LqrResult
                {
                    K = result.K,
                    P = result.P,
                    ClosedLoopPoles = result.ClosedLoopPoles,
                    Success = true
                };
            }

            return new LqrResult
            {
                K = Matrix<double>.Build.Dense(m, n),
                P = P,
                ClosedLoopPoles = [],
                Success = false,
                ErrorMessage = $"CARE solver did not converge within {maxIterations} iterations."
            };
        }
        catch (Exception ex)
        {
            return new LqrResult
            {
                K = Matrix<double>.Build.Dense(m, n),
                P = Matrix<double>.Build.Dense(n, n),
                ClosedLoopPoles = [],
                Success = false,
                ErrorMessage = $"CARE solver failed: {ex.Message}"
            };
        }
    }

    private static LqrResult ComputeResult(Matrix<double> A, Matrix<double> B, Matrix<double> P, Matrix<double> Rinv, bool success)
    {
        var K = Rinv * B.Transpose() * P;
        var Acl = A - B * K;
        var evd = Acl.Evd();
        var poles = evd.EigenValues.ToArray();

        return new LqrResult
        {
            K = K,
            P = P,
            ClosedLoopPoles = poles,
            Success = success
        };
    }

    /// <summary>
    /// Solves the discrete-time algebraic Riccati equation (DARE).
    /// A'PA - P - A'PB(R + B'PB)^-1B'PA + Q = 0
    /// </summary>
    public static LqrResult SolveDiscrete(
        Matrix<double> A,
        Matrix<double> B,
        Matrix<double> Q,
        Matrix<double> R,
        int maxIterations = 1000,
        double tolerance = 1e-9)
    {
        ValidateDimensions(A, B, Q, R);

        int n = A.RowCount;
        int m = B.ColumnCount;

        try
        {
            var P = Q.Clone();
            var At = A.Transpose();
            var Bt = B.Transpose();

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // DARE iteration:
                // P_new = A'PA - A'PB(R + B'PB)^-1B'PA + Q
                var AtP = At * P;
                var AtPA = AtP * A;
                var AtPB = AtP * B;
                var BtPB = Bt * P * B;
                var temp = (R + BtPB).Inverse();
                var Pnew = AtPA - AtPB * temp * AtPB.Transpose() + Q;

                // Force symmetry
                Pnew = (Pnew + Pnew.Transpose()) * 0.5;

                double diff = (Pnew - P).FrobeniusNorm();
                P = Pnew;

                if (diff < tolerance)
                {
                    // Compute gain K = (R + B'PB)^-1 B' P A
                    var BtPBpR = Bt * P * B + R;
                    var K = BtPBpR.Inverse() * Bt * P * A;

                    // Compute closed-loop poles of (A - B*K)
                    var Acl = A - B * K;
                    var evd = Acl.Evd();
                    var poles = evd.EigenValues.ToArray();

                    return new LqrResult
                    {
                        K = K,
                        P = P,
                        ClosedLoopPoles = poles,
                        Success = true
                    };
                }
            }

            return new LqrResult
            {
                K = Matrix<double>.Build.Dense(m, n),
                P = P,
                ClosedLoopPoles = [],
                Success = false,
                ErrorMessage = $"DARE solver did not converge within {maxIterations} iterations."
            };
        }
        catch (Exception ex)
        {
            return new LqrResult
            {
                K = Matrix<double>.Build.Dense(m, n),
                P = Matrix<double>.Build.Dense(n, n),
                ClosedLoopPoles = [],
                Success = false,
                ErrorMessage = $"DARE solver failed: {ex.Message}"
            };
        }
    }

    private static void ValidateDimensions(Matrix<double> A, Matrix<double> B, Matrix<double> Q, Matrix<double> R)
    {
        int n = A.RowCount;
        int m = B.ColumnCount;

        if (A.ColumnCount != n)
            throw new ArgumentException($"A must be square. Got {A.RowCount}x{A.ColumnCount}.");
        
        if (B.RowCount != n)
            throw new ArgumentException($"B must have {n} rows to match A. Got {B.RowCount}.");
        
        if (Q.RowCount != n || Q.ColumnCount != n)
            throw new ArgumentException($"Q must be {n}x{n} to match A. Got {Q.RowCount}x{Q.ColumnCount}.");
        
        if (R.RowCount != m || R.ColumnCount != m)
            throw new ArgumentException($"R must be {m}x{m} to match B columns. Got {R.RowCount}x{R.ColumnCount}.");
    }

    /// <summary>
    /// Simulates the step response of a linear closed-loop system.
    /// </summary>
    public static (double[] Time, double[,] States) SimulateStepResponse(
        Matrix<double> A,
        Matrix<double> B,
        Matrix<double> K,
        Vector<double> targetState,
        double duration,
        double dt)
    {
        int n = A.RowCount;
        int steps = (int)(duration / dt) + 1;
        
        var times = new double[steps];
        var states = new double[steps, n];
        
        var Acl = A - B * K;
        var x = Vector<double>.Build.Dense(n);
        
        // Using reference tracking: u = -K(x - x_ref) 
        // Closed loop becomes: dx/dt = A*x + B*(-K*x + K*x_ref) = (A-BK)*x + BK*x_ref
        var BKxref = B * K * targetState;
        
        for (int i = 0; i < steps; i++)
        {
            times[i] = i * dt;
            for (int j = 0; j < n; j++)
            {
                states[i, j] = x[j];
            }
            
            // Euler integration
            var dx = Acl * x + BKxref;
            x = x + dx * dt;
        }
        
        return (times, states);
    }
}
