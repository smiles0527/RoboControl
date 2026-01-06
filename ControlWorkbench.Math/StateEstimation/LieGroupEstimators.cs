using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.StateEstimation;

// Note: CubatureKalmanFilter is also defined in AdvancedEstimators.cs
// This file contains Lie group estimators and related filters

/// <summary>
/// Invariant Extended Kalman Filter (InEKF) on Lie Groups.
/// Provides globally consistent state estimation for robotic systems.
/// Based on Barrau &amp; Bonnabel "The Invariant Extended Kalman Filter as an Observer"
/// </summary>
public class InvariantEkf
{
    private readonly int _dimState;
    private readonly int _dimNoise;
    private Matrix<double> _X;      // State on SE_k(3)
    private Matrix<double> _P;      // Right-invariant error covariance
    private readonly Matrix<double> _Q; // Process noise
    private readonly InvariantEkfType _type;
    
    public InvariantEkf(
        int stateMatrixDim,
        Matrix<double> processNoise,
        InvariantEkfType type = InvariantEkfType.RightInvariant)
    {
        _dimState = stateMatrixDim;
        _Q = processNoise;
        _dimNoise = processNoise.RowCount;
        _type = type;
        
        _X = Matrix<double>.Build.DenseIdentity(_dimState);
        _P = Matrix<double>.Build.DenseIdentity(ComputeErrorDimension()) * 0.01;
    }
    
    /// <summary>
    /// Initialize state estimate.
    /// </summary>
    public void Initialize(Matrix<double> initialState, Matrix<double>? initialCovariance = null)
    {
        _X = initialState.Clone();
        if (initialCovariance != null)
            _P = initialCovariance.Clone();
    }
    
    /// <summary>
    /// Prediction step using group exponential map.
    /// </summary>
    public void Predict(Vector<double> u, double dt)
    {
        // Compute discrete-time velocity
        var xi = ComputeVelocity(u, dt);
        
        // State update: X_{k+1} = X_k * exp(xi)
        var expXi = SE3Exponential(xi);
        _X = _X * expXi;
        
        // Covariance propagation using adjoint
        var F = ComputeProcessJacobian(u, dt);
        var G = ComputeNoiseJacobian(dt);
        
        _P = F * _P * F.Transpose() + G * _Q * G.Transpose();
    }
    
    /// <summary>
    /// Update with position measurement (GPS-like).
    /// </summary>
    public void UpdatePosition(Vector<double> position, Matrix<double> R)
    {
        var p = ExtractPosition(_X);
        var innovation = position - p;
        
        int errDim = _P.RowCount;
        var H = Matrix<double>.Build.Dense(3, errDim);
        H[0, 0] = 1; H[1, 1] = 1; H[2, 2] = 1;
        
        UpdateInvariant(innovation, H, R);
    }
    
    /// <summary>
    /// Update with rotation measurement.
    /// </summary>
    public void UpdateOrientation(Matrix<double> measuredRotation, Matrix<double> R)
    {
        var currentRot = ExtractRotation(_X);
        var rotError = currentRot.Transpose() * measuredRotation;
        var logError = SO3Logarithm(rotError);
        
        int errDim = _P.RowCount;
        var H = Matrix<double>.Build.Dense(3, errDim);
        H.SetSubMatrix(0, 3, Matrix<double>.Build.DenseIdentity(3));
        
        UpdateInvariant(logError, H, R);
    }
    
    /// <summary>
    /// Update with velocity measurement.
    /// </summary>
    public void UpdateVelocity(Vector<double> velocity, Matrix<double> R)
    {
        var v = ExtractVelocity(_X);
        var innovation = velocity - v;
        
        int errDim = _P.RowCount;
        var H = Matrix<double>.Build.Dense(3, errDim);
        for (int i = 0; i < 3; i++)
            H[i, 6 + i] = 1;
        
        UpdateInvariant(innovation, H, R);
    }
    
    private void UpdateInvariant(Vector<double> innovation, Matrix<double> H, Matrix<double> R)
    {
        var S = H * _P * H.Transpose() + R;
        var K = _P * H.Transpose() * S.Inverse();
        var delta = K * innovation;
        
        if (_type == InvariantEkfType.RightInvariant)
        {
            var correction = SE3Exponential(delta);
            _X = correction * _X;
        }
        else
        {
            var correction = SE3Exponential(delta);
            _X = _X * correction;
        }
        
        var IKH = Matrix<double>.Build.DenseIdentity(_P.RowCount) - K * H;
        _P = IKH * _P * IKH.Transpose() + K * R * K.Transpose();
        _P = 0.5 * (_P + _P.Transpose());
    }
    
    public Matrix<double> State => _X.Clone();
    public Matrix<double> Covariance => _P.Clone();
    public Vector<double> Position => ExtractPosition(_X);
    public Matrix<double> Rotation => ExtractRotation(_X);
    public Vector<double> Velocity => ExtractVelocity(_X);
    
    private int ComputeErrorDimension() => 9;
    
    private Vector<double> ComputeVelocity(Vector<double> u, double dt)
    {
        var xi = Vector<double>.Build.Dense(9);
        xi[0] = _X[0, 3] * dt;
        xi[1] = _X[1, 3] * dt;
        xi[2] = _X[2, 3] * dt;
        xi[3] = u[0] * dt;
        xi[4] = u[1] * dt;
        xi[5] = u[2] * dt;
        
        var R = ExtractRotation(_X);
        var aWorld = R * Vector<double>.Build.DenseOfArray([u[3], u[4], u[5]]);
        aWorld[2] -= 9.81;
        
        xi[6] = aWorld[0] * dt;
        xi[7] = aWorld[1] * dt;
        xi[8] = aWorld[2] * dt;
        
        return xi;
    }
    
    private Matrix<double> ComputeProcessJacobian(Vector<double> u, double dt)
    {
        int dim = _P.RowCount;
        var F = Matrix<double>.Build.DenseIdentity(dim);
        F[0, 6] = dt; F[1, 7] = dt; F[2, 8] = dt;
        return F;
    }
    
    private Matrix<double> ComputeNoiseJacobian(double dt)
    {
        return Matrix<double>.Build.DenseIdentity(_P.RowCount) * dt;
    }
    
    private Matrix<double> SE3Exponential(Vector<double> xi)
    {
        var result = Matrix<double>.Build.DenseIdentity(_dimState);
        
        if (xi.Count >= 6)
        {
            var omega = Vector<double>.Build.DenseOfArray([xi[3], xi[4], xi[5]]);
            var theta = omega.L2Norm();
            
            Matrix<double> R;
            if (theta < 1e-10)
                R = Matrix<double>.Build.DenseIdentity(3) + SkewSymmetric(omega);
            else
            {
                var axis = omega / theta;
                R = RodriguesRotation(axis, theta);
            }
            
            result.SetSubMatrix(0, 0, R);
            
            var V = LeftJacobian(omega);
            var rho = Vector<double>.Build.DenseOfArray([xi[0], xi[1], xi[2]]);
            var t = V * rho;
            
            result[0, 3] = t[0]; result[1, 3] = t[1]; result[2, 3] = t[2];
            
            if (xi.Count >= 9 && _dimState >= 5)
            {
                var nu = Vector<double>.Build.DenseOfArray([xi[6], xi[7], xi[8]]);
                var vel = V * nu;
                result[0, 4] = vel[0]; result[1, 4] = vel[1]; result[2, 4] = vel[2];
            }
        }
        
        return result;
    }
    
    private Vector<double> SO3Logarithm(Matrix<double> R)
    {
        double trace = R[0, 0] + R[1, 1] + R[2, 2];
        double theta = System.Math.Acos(System.Math.Clamp((trace - 1) / 2, -1, 1));
        
        if (theta < 1e-10)
            return Vector<double>.Build.Dense(3);
        
        double factor = theta / (2 * System.Math.Sin(theta));
        return Vector<double>.Build.DenseOfArray([
            factor * (R[2, 1] - R[1, 2]),
            factor * (R[0, 2] - R[2, 0]),
            factor * (R[1, 0] - R[0, 1])
        ]);
    }
    
    private Matrix<double> RodriguesRotation(Vector<double> axis, double theta)
    {
        var K = SkewSymmetric(axis);
        return Matrix<double>.Build.DenseIdentity(3) + 
               System.Math.Sin(theta) * K + 
               (1 - System.Math.Cos(theta)) * K * K;
    }
    
    private Matrix<double> SkewSymmetric(Vector<double> v)
    {
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0, -v[2], v[1] },
            { v[2], 0, -v[0] },
            { -v[1], v[0], 0 }
        });
    }
    
    private Matrix<double> LeftJacobian(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        if (theta < 1e-10)
            return Matrix<double>.Build.DenseIdentity(3);
        
        var axis = omega / theta;
        var K = SkewSymmetric(axis);
        
        return Matrix<double>.Build.DenseIdentity(3) +
               ((1 - System.Math.Cos(theta)) / theta) * K +
               ((theta - System.Math.Sin(theta)) / theta) * K * K;
    }
    
    private Vector<double> ExtractPosition(Matrix<double> X) => 
        Vector<double>.Build.DenseOfArray([X[0, 3], X[1, 3], X[2, 3]]);
    
    private Matrix<double> ExtractRotation(Matrix<double> X) => X.SubMatrix(0, 3, 0, 3);
    
    private Vector<double> ExtractVelocity(Matrix<double> X) => 
        X.ColumnCount > 4 
            ? Vector<double>.Build.DenseOfArray([X[0, 4], X[1, 4], X[2, 4]])
            : Vector<double>.Build.Dense(3);
}

public enum InvariantEkfType { LeftInvariant, RightInvariant }

/// <summary>
/// Continuous-Discrete Extended Kalman Filter with higher-order integration.
/// Uses RK4 for state propagation with continuous dynamics.
/// </summary>
public class ContinuousDiscreteEkf
{
    private readonly int _n;
    private Vector<double> _x;
    private Matrix<double> _P;
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Matrix<double>> _dfdx;
    private readonly Matrix<double> _Qc;
    
    public ContinuousDiscreteEkf(
        int stateSize,
        Func<Vector<double>, Vector<double>, Vector<double>> continuousDynamics,
        Func<Vector<double>, Matrix<double>> stateJacobian,
        Matrix<double> continuousProcessNoise)
    {
        _n = stateSize;
        _f = continuousDynamics;
        _dfdx = stateJacobian;
        _Qc = continuousProcessNoise;
        
        _x = Vector<double>.Build.Dense(_n);
        _P = Matrix<double>.Build.DenseIdentity(_n) * 0.1;
    }
    
    public void Initialize(Vector<double> x0, Matrix<double>? P0 = null)
    {
        _x = x0.Clone();
        if (P0 != null) _P = P0.Clone();
    }
    
    /// <summary>
    /// Propagate state using RK4 and covariance using matrix exponential.
    /// </summary>
    public void Predict(Vector<double> u, double dt, int substeps = 4)
    {
        double h = dt / substeps;
        
        for (int i = 0; i < substeps; i++)
        {
            // RK4 for state
            var k1 = _f(_x, u);
            var k2 = _f(_x + 0.5 * h * k1, u);
            var k3 = _f(_x + 0.5 * h * k2, u);
            var k4 = _f(_x + h * k3, u);
            
            _x = _x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
            
            // Covariance propagation using Van Loan method
            var F = _dfdx(_x);
            var Phi = ComputeStateTransitionMatrix(F, h);
            var Qd = ComputeDiscreteProcessNoise(F, h);
            
            _P = Phi * _P * Phi.Transpose() + Qd;
        }
    }
    
    public void Update(Vector<double> z, Func<Vector<double>, Vector<double>> h, 
                       Func<Vector<double>, Matrix<double>> dhdx, Matrix<double> R)
    {
        var zPred = h(_x);
        var H = dhdx(_x);
        var innovation = z - zPred;
        
        var S = H * _P * H.Transpose() + R;
        var K = _P * H.Transpose() * S.Inverse();
        
        _x = _x + K * innovation;
        
        var IKH = Matrix<double>.Build.DenseIdentity(_n) - K * H;
        _P = IKH * _P * IKH.Transpose() + K * R * K.Transpose();
        _P = 0.5 * (_P + _P.Transpose());
    }
    
    public Vector<double> State => _x.Clone();
    public Matrix<double> Covariance => _P.Clone();
    
    private Matrix<double> ComputeStateTransitionMatrix(Matrix<double> F, double dt)
    {
        // Pade approximation of matrix exponential
        int order = 6;
        var I = Matrix<double>.Build.DenseIdentity(_n);
        var Fdt = F * dt;
        
        var expF = I.Clone();
        var Fpower = I.Clone();
        double factorial = 1;
        
        for (int i = 1; i <= order; i++)
        {
            factorial *= i;
            Fpower = Fpower * Fdt;
            expF = expF + Fpower / factorial;
        }
        
        return expF;
    }
    
    private Matrix<double> ComputeDiscreteProcessNoise(Matrix<double> F, double dt)
    {
        // First-order approximation: Qd ? Q_c * dt
        // Van Loan method for more accuracy
        var M = Matrix<double>.Build.Dense(2 * _n, 2 * _n);
        
        M.SetSubMatrix(0, 0, -F);
        M.SetSubMatrix(0, _n, _Qc);
        M.SetSubMatrix(_n, _n, F.Transpose());
        
        var expM = ComputeStateTransitionMatrix(M, dt);
        var Phi = expM.SubMatrix(_n, _n, _n, _n).Transpose();
        var PhiQd = Phi * expM.SubMatrix(0, _n, _n, _n);
        
        return PhiQd;
    }
}

/// <summary>
/// Cubature Kalman Filter - more accurate than UKF for high-dimensional systems.
/// Uses spherical-radial cubature rule.
/// </summary>
public class CubatureKalmanFilter
{
    private readonly int _n;
    private readonly int _numCubaturePoints;
    private Vector<double> _x;
    private Matrix<double> _P;
    private readonly Matrix<double> _Q;
    private readonly Matrix<double> _R;
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Vector<double>> _h;
    
    public CubatureKalmanFilter(
        int stateSize,
        int measurementSize,
        Func<Vector<double>, Vector<double>, Vector<double>> stateTransition,
        Func<Vector<double>, Vector<double>> measurementFunction,
        Matrix<double> processNoise,
        Matrix<double> measurementNoise)
    {
        _n = stateSize;
        _numCubaturePoints = 2 * _n;
        _f = stateTransition;
        _h = measurementFunction;
        _Q = processNoise;
        _R = measurementNoise;
        
        _x = Vector<double>.Build.Dense(_n);
        _P = Matrix<double>.Build.DenseIdentity(_n);
    }
    
    public void Initialize(Vector<double> x0, Matrix<double>? P0 = null)
    {
        _x = x0.Clone();
        if (P0 != null) _P = P0.Clone();
    }
    
    public void Predict(Vector<double> u)
    {
        // Generate cubature points
        var (points, weight) = GenerateCubaturePoints(_x, _P);
        
        // Propagate through dynamics
        var propagated = new Vector<double>[_numCubaturePoints];
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            propagated[i] = _f(points[i], u);
        }
        
        // Compute predicted mean
        _x = Vector<double>.Build.Dense(_n);
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            _x += weight * propagated[i];
        }
        
        // Compute predicted covariance
        _P = _Q.Clone();
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            var diff = propagated[i] - _x;
            _P += weight * diff.OuterProduct(diff);
        }
    }
    
    public void Update(Vector<double> z)
    {
        int m = z.Count;
        
        // Generate cubature points from predicted state
        var (points, weight) = GenerateCubaturePoints(_x, _P);
        
        // Transform through measurement function
        var zPoints = new Vector<double>[_numCubaturePoints];
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            zPoints[i] = _h(points[i]);
        }
        
        // Predicted measurement mean
        var zPred = Vector<double>.Build.Dense(m);
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            zPred += weight * zPoints[i];
        }
        
        // Innovation covariance
        var Pzz = _R.Clone();
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            var diff = zPoints[i] - zPred;
            Pzz += weight * diff.OuterProduct(diff);
        }
        
        // Cross covariance
        var Pxz = Matrix<double>.Build.Dense(_n, m);
        for (int i = 0; i < _numCubaturePoints; i++)
        {
            var xDiff = points[i] - _x;
            var zDiff = zPoints[i] - zPred;
            Pxz += weight * xDiff.OuterProduct(zDiff);
        }
        
        // Kalman gain and update
        var K = Pxz * Pzz.Inverse();
        _x = _x + K * (z - zPred);
        _P = _P - K * Pzz * K.Transpose();
        _P = 0.5 * (_P + _P.Transpose());
    }
    
    public Vector<double> State => _x.Clone();
    public Matrix<double> Covariance => _P.Clone();
    
    private (Vector<double>[] points, double weight) GenerateCubaturePoints(Vector<double> x, Matrix<double> P)
    {
        var points = new Vector<double>[_numCubaturePoints];
        double weight = 1.0 / _numCubaturePoints;
        
        // Cholesky decomposition
        var sqrtP = (P * _n).Cholesky().Factor;
        
        // Generate cubature points
        for (int i = 0; i < _n; i++)
        {
            points[i] = x + sqrtP.Column(i);
            points[i + _n] = x - sqrtP.Column(i);
        }
        
        return (points, weight);
    }
}

/// <summary>
/// Square-Root Unscented Kalman Filter for improved numerical stability.
/// Maintains Cholesky factor of covariance directly.
/// </summary>
public class SquareRootUkf
{
    private readonly int _n;
    private readonly double _alpha;
    private readonly double _beta;
    private readonly double _kappa;
    private readonly double _lambda;
    
    private Vector<double> _x;
    private Matrix<double> _S; // Cholesky factor: P = S * S'
    private readonly Matrix<double> _sqrtQ;
    private readonly Matrix<double> _sqrtR;
    
    private readonly Func<Vector<double>, Vector<double>, Vector<double>> _f;
    private readonly Func<Vector<double>, Vector<double>> _h;
    
    public SquareRootUkf(
        int stateSize,
        int measurementSize,
        Func<Vector<double>, Vector<double>, Vector<double>> stateTransition,
        Func<Vector<double>, Vector<double>> measurementFunction,
        Matrix<double> processNoise,
        Matrix<double> measurementNoise,
        double alpha = 0.001,
        double beta = 2.0,
        double kappa = 0.0)
    {
        _n = stateSize;
        _alpha = alpha;
        _beta = beta;
        _kappa = kappa;
        _lambda = alpha * alpha * (_n + kappa) - _n;
        
        _f = stateTransition;
        _h = measurementFunction;
        _sqrtQ = processNoise.Cholesky().Factor;
        _sqrtR = measurementNoise.Cholesky().Factor;
        
        _x = Vector<double>.Build.Dense(_n);
        _S = Matrix<double>.Build.DenseIdentity(_n);
    }
    
    public void Initialize(Vector<double> x0, Matrix<double>? S0 = null)
    {
        _x = x0.Clone();
        if (S0 != null) _S = S0.Clone();
    }
    
    public void Predict(Vector<double> u)
    {
        // Generate sigma points
        var (sigmaPoints, Wm, Wc) = GenerateSigmaPoints(_x, _S);
        
        // Propagate sigma points
        var propagated = new Vector<double>[2 * _n + 1];
        for (int i = 0; i <= 2 * _n; i++)
        {
            propagated[i] = _f(sigmaPoints[i], u);
        }
        
        // Compute predicted mean
        _x = Vector<double>.Build.Dense(_n);
        for (int i = 0; i <= 2 * _n; i++)
        {
            _x += Wm[i] * propagated[i];
        }
        
        // QR decomposition for square root update
        var chi = Matrix<double>.Build.Dense(_n, 2 * _n + _sqrtQ.ColumnCount);
        
        for (int i = 1; i <= 2 * _n; i++)
        {
            var diff = propagated[i] - _x;
            chi.SetColumn(i - 1, System.Math.Sqrt(System.Math.Abs(Wc[i])) * diff);
        }
        
        for (int i = 0; i < _sqrtQ.ColumnCount; i++)
        {
            chi.SetColumn(2 * _n + i, _sqrtQ.Column(i));
        }
        
        // QR decomposition
        _S = CholeskyDowndate(chi.Transpose().QR().R.Transpose(), 
                             System.Math.Sqrt(System.Math.Abs(Wc[0])) * (propagated[0] - _x), 
                             Wc[0] < 0);
    }
    
    public void Update(Vector<double> z)
    {
        int m = z.Count;
        
        // Generate sigma points
        var (sigmaPoints, Wm, Wc) = GenerateSigmaPoints(_x, _S);
        
        // Transform through measurement
        var zPoints = new Vector<double>[2 * _n + 1];
        for (int i = 0; i <= 2 * _n; i++)
        {
            zPoints[i] = _h(sigmaPoints[i]);
        }
        
        // Predicted measurement
        var zPred = Vector<double>.Build.Dense(m);
        for (int i = 0; i <= 2 * _n; i++)
        {
            zPred += Wm[i] * zPoints[i];
        }
        
        // Square root of innovation covariance
        var Zchi = Matrix<double>.Build.Dense(m, 2 * _n + _sqrtR.ColumnCount);
        
        for (int i = 1; i <= 2 * _n; i++)
        {
            var diff = zPoints[i] - zPred;
            Zchi.SetColumn(i - 1, System.Math.Sqrt(System.Math.Abs(Wc[i])) * diff);
        }
        
        for (int i = 0; i < _sqrtR.ColumnCount; i++)
        {
            Zchi.SetColumn(2 * _n + i, _sqrtR.Column(i));
        }
        
        var Sz = Zchi.Transpose().QR().R.Transpose().SubMatrix(0, m, 0, m);
        
        // Cross covariance
        var Pxz = Matrix<double>.Build.Dense(_n, m);
        for (int i = 0; i <= 2 * _n; i++)
        {
            var xDiff = sigmaPoints[i] - _x;
            var zDiff = zPoints[i] - zPred;
            Pxz += Wc[i] * xDiff.OuterProduct(zDiff);
        }
        
        // Kalman gain using triangular solve
        var K = Pxz * Sz.Transpose().Inverse() * Sz.Inverse();
        
        // Update
        _x = _x + K * (z - zPred);
        
        // Square root covariance update
        var U = K * Sz;
        for (int i = 0; i < m; i++)
        {
            _S = CholeskyDowndate(_S, U.Column(i), false);
        }
    }
    
    public Vector<double> State => _x.Clone();
    public Matrix<double> Covariance => _S * _S.Transpose();
    public Matrix<double> CovarianceSqrt => _S.Clone();
    
    private (Vector<double>[] points, double[] Wm, double[] Wc) GenerateSigmaPoints(Vector<double> x, Matrix<double> S)
    {
        int numPoints = 2 * _n + 1;
        var points = new Vector<double>[numPoints];
        var Wm = new double[numPoints];
        var Wc = new double[numPoints];
        
        Wm[0] = _lambda / (_n + _lambda);
        Wc[0] = Wm[0] + (1 - _alpha * _alpha + _beta);
        
        for (int i = 1; i < numPoints; i++)
        {
            Wm[i] = 1.0 / (2 * (_n + _lambda));
            Wc[i] = Wm[i];
        }
        
        double gamma = System.Math.Sqrt(_n + _lambda);
        
        points[0] = x.Clone();
        for (int i = 0; i < _n; i++)
        {
            var col = gamma * S.Column(i);
            points[i + 1] = x + col;
            points[i + 1 + _n] = x - col;
        }
        
        return (points, Wm, Wc);
    }
    
    private Matrix<double> CholeskyDowndate(Matrix<double> S, Vector<double> v, bool subtract)
    {
        var result = S.Clone();
        int n = v.Count;
        var x = v.Clone();
        
        double sign = subtract ? -1 : 1;
        
        for (int k = 0; k < n; k++)
        {
            double r = System.Math.Sqrt(result[k, k] * result[k, k] + sign * x[k] * x[k]);
            double c = r / result[k, k];
            double s = x[k] / result[k, k];
            
            result[k, k] = r;
            
            for (int j = k + 1; j < n; j++)
            {
                result[k, j] = (result[k, j] + sign * s * x[j]) / c;
                x[j] = c * x[j] - s * result[k, j];
            }
        }
        
        return result;
    }
}
