using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Geometry;

/// <summary>
/// Lie Group implementations for SE(3), SO(3), SE(2), and related structures.
/// Provides manifold operations essential for robotics state estimation and control.
/// Based on "A micro Lie theory for state estimation in robotics" (Solà et al., 2021)
/// and "Lie Groups for 2D and 3D Transformations" (Blanco, 2010).
/// </summary>
public static class LieGroups
{
    private const double Epsilon = 1e-10;
    
    #region SO(3) - Special Orthogonal Group (3D Rotations)
    
    /// <summary>
    /// Exponential map so(3) ? SO(3). Maps angular velocity vector to rotation matrix.
    /// Uses Rodrigues' formula with Taylor expansion for small angles.
    /// </summary>
    public static Matrix<double> ExpSO3(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        
        if (theta < Epsilon)
        {
            // First-order Taylor expansion: R ? I + [?]×
            return Matrix<double>.Build.DenseIdentity(3) + Skew3(omega);
        }
        
        var axis = omega / theta;
        var K = Skew3(axis);
        
        // Rodrigues' formula: R = I + sin(?)K + (1-cos(?))K²
        return Matrix<double>.Build.DenseIdentity(3) +
               System.Math.Sin(theta) * K +
               (1 - System.Math.Cos(theta)) * K * K;
    }
    
    /// <summary>
    /// Logarithmic map SO(3) ? so(3). Inverse of exponential map.
    /// Returns the angular velocity vector (rotation axis × angle).
    /// </summary>
    public static Vector<double> LogSO3(Matrix<double> R)
    {
        double trace = R.Trace();
        double cosTheta = (trace - 1) / 2;
        cosTheta = System.Math.Clamp(cosTheta, -1, 1);
        double theta = System.Math.Acos(cosTheta);
        
        if (theta < Epsilon)
        {
            // Small angle: ? ? vee(R - I)
            return Vee3(R - Matrix<double>.Build.DenseIdentity(3));
        }
        
        if (System.Math.Abs(theta - System.Math.PI) < Epsilon)
        {
            // ? ? ?: use eigenvector method
            // R = I + 2*axis*axis^T, so axis = sqrt((diag(R)+1)/2)
            var diag = Vector<double>.Build.DenseOfArray([R[0, 0], R[1, 1], R[2, 2]]);
            var axis = ((diag + Vector<double>.Build.Dense(3, 1)) / 2).PointwiseSqrt();
            
            // Determine signs from off-diagonal elements
            if (R[0, 1] < 0) axis[1] = -axis[1];
            if (R[0, 2] < 0) axis[2] = -axis[2];
            
            return axis * theta;
        }
        
        // General case: ? = ?/(2sin(?)) * vee(R - R^T)
        var omega = Vee3(R - R.Transpose()) * theta / (2 * System.Math.Sin(theta));
        return omega;
    }
    
    /// <summary>
    /// Left Jacobian of SO(3). Maps perturbations in tangent space.
    /// J_l(?) = I + (1-cos(?))/?² [?]× + (?-sin(?))/?³ [?]×²
    /// </summary>
    public static Matrix<double> LeftJacobianSO3(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        
        if (theta < Epsilon)
        {
            return Matrix<double>.Build.DenseIdentity(3) + 0.5 * Skew3(omega);
        }
        
        var K = Skew3(omega / theta);
        double theta2 = theta * theta;
        
        return Matrix<double>.Build.DenseIdentity(3) +
               (1 - System.Math.Cos(theta)) / theta2 * Skew3(omega) +
               (theta - System.Math.Sin(theta)) / (theta2 * theta) * Skew3(omega) * Skew3(omega);
    }
    
    /// <summary>
    /// Inverse of left Jacobian of SO(3).
    /// </summary>
    public static Matrix<double> LeftJacobianInverseSO3(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        
        if (theta < Epsilon)
        {
            return Matrix<double>.Build.DenseIdentity(3) - 0.5 * Skew3(omega);
        }
        
        var K = Skew3(omega / theta);
        double halfTheta = theta / 2;
        double cotHalf = 1.0 / System.Math.Tan(halfTheta);
        
        return Matrix<double>.Build.DenseIdentity(3) -
               0.5 * Skew3(omega) +
               (1 - halfTheta * cotHalf) * K * K;
    }
    
    /// <summary>
    /// Right Jacobian of SO(3). J_r(?) = J_l(-?)
    /// </summary>
    public static Matrix<double> RightJacobianSO3(Vector<double> omega)
    {
        return LeftJacobianSO3(-omega);
    }
    
    /// <summary>
    /// Adjoint representation of SO(3). For R ? SO(3), Ad_R = R.
    /// </summary>
    public static Matrix<double> AdjointSO3(Matrix<double> R) => R;
    
    #endregion
    
    #region SE(3) - Special Euclidean Group (Rigid Body Transformations)
    
    /// <summary>
    /// Exponential map se(3) ? SE(3). Maps twist vector [?; v] to transformation matrix.
    /// </summary>
    public static Matrix<double> ExpSE3(Vector<double> xi)
    {
        var omega = xi.SubVector(0, 3);
        var v = xi.SubVector(3, 3);
        
        var R = ExpSO3(omega);
        var V = LeftJacobianSO3(omega);
        var t = V * v;
        
        var T = Matrix<double>.Build.DenseIdentity(4);
        T.SetSubMatrix(0, 0, R);
        T[0, 3] = t[0];
        T[1, 3] = t[1];
        T[2, 3] = t[2];
        
        return T;
    }
    
    /// <summary>
    /// Logarithmic map SE(3) ? se(3). Returns twist vector [?; v].
    /// </summary>
    public static Vector<double> LogSE3(Matrix<double> T)
    {
        var R = T.SubMatrix(0, 3, 0, 3);
        var t = Vector<double>.Build.DenseOfArray([T[0, 3], T[1, 3], T[2, 3]]);
        
        var omega = LogSO3(R);
        var VInv = LeftJacobianInverseSO3(omega);
        var v = VInv * t;
        
        var xi = Vector<double>.Build.Dense(6);
        xi.SetSubVector(0, 3, omega);
        xi.SetSubVector(3, 3, v);
        return xi;
    }
    
    /// <summary>
    /// Left Jacobian of SE(3). 6×6 matrix mapping tangent space perturbations.
    /// </summary>
    public static Matrix<double> LeftJacobianSE3(Vector<double> xi)
    {
        var omega = xi.SubVector(0, 3);
        var v = xi.SubVector(3, 3);
        
        var Jl_omega = LeftJacobianSO3(omega);
        var Q = ComputeQMatrix(omega, v);
        
        var J = Matrix<double>.Build.Dense(6, 6);
        J.SetSubMatrix(0, 0, Jl_omega);
        J.SetSubMatrix(3, 0, Q);
        J.SetSubMatrix(3, 3, Jl_omega);
        
        return J;
    }
    
    /// <summary>
    /// Adjoint representation of SE(3). 6×6 matrix.
    /// </summary>
    public static Matrix<double> AdjointSE3(Matrix<double> T)
    {
        var R = T.SubMatrix(0, 3, 0, 3);
        var t = Vector<double>.Build.DenseOfArray([T[0, 3], T[1, 3], T[2, 3]]);
        var tSkew = Skew3(t);
        
        var Ad = Matrix<double>.Build.Dense(6, 6);
        Ad.SetSubMatrix(0, 0, R);
        Ad.SetSubMatrix(3, 0, tSkew * R);
        Ad.SetSubMatrix(3, 3, R);
        
        return Ad;
    }
    
    /// <summary>
    /// Inverse of SE(3) transformation matrix.
    /// </summary>
    public static Matrix<double> InverseSE3(Matrix<double> T)
    {
        var R = T.SubMatrix(0, 3, 0, 3);
        var t = Vector<double>.Build.DenseOfArray([T[0, 3], T[1, 3], T[2, 3]]);
        
        var RInv = R.Transpose();
        var tInv = -RInv * t;
        
        var TInv = Matrix<double>.Build.DenseIdentity(4);
        TInv.SetSubMatrix(0, 0, RInv);
        TInv[0, 3] = tInv[0];
        TInv[1, 3] = tInv[1];
        TInv[2, 3] = tInv[2];
        
        return TInv;
    }
    
    /// <summary>
    /// Compose two SE(3) transformations: T1 ? T2 = T1 * T2
    /// </summary>
    public static Matrix<double> ComposeSE3(Matrix<double> T1, Matrix<double> T2)
    {
        return T1 * T2;
    }
    
    /// <summary>
    /// Relative transformation: T1?¹ * T2 (T2 in T1's frame)
    /// </summary>
    public static Matrix<double> RelativeSE3(Matrix<double> T1, Matrix<double> T2)
    {
        return InverseSE3(T1) * T2;
    }
    
    /// <summary>
    /// Box-plus operator: T ? ? = T * Exp(?)
    /// </summary>
    public static Matrix<double> BoxPlusSE3(Matrix<double> T, Vector<double> delta)
    {
        return T * ExpSE3(delta);
    }
    
    /// <summary>
    /// Box-minus operator: T1 ? T2 = Log(T2?¹ * T1)
    /// </summary>
    public static Vector<double> BoxMinusSE3(Matrix<double> T1, Matrix<double> T2)
    {
        return LogSE3(InverseSE3(T2) * T1);
    }
    
    #endregion
    
    #region Quaternion Operations (Unit Quaternions ? SO(3))
    
    /// <summary>
    /// Exponential map: angular velocity ? unit quaternion
    /// </summary>
    public static Quaternion ExpQuaternion(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        
        if (theta < Epsilon)
        {
            return new Quaternion(1, omega[0] / 2, omega[1] / 2, omega[2] / 2).Normalized();
        }
        
        double halfTheta = theta / 2;
        double sinHalf = System.Math.Sin(halfTheta) / theta;
        
        return new Quaternion(
            System.Math.Cos(halfTheta),
            omega[0] * sinHalf,
            omega[1] * sinHalf,
            omega[2] * sinHalf
        );
    }
    
    /// <summary>
    /// Logarithmic map: unit quaternion ? angular velocity
    /// </summary>
    public static Vector<double> LogQuaternion(Quaternion q)
    {
        q = q.W < 0 ? new Quaternion(-q.W, -q.X, -q.Y, -q.Z) : q;
        
        var vec = Vector<double>.Build.DenseOfArray([q.X, q.Y, q.Z]);
        double vecNorm = vec.L2Norm();
        
        if (vecNorm < Epsilon)
        {
            return 2 * vec;
        }
        
        double theta = 2 * System.Math.Atan2(vecNorm, q.W);
        return vec * (theta / vecNorm);
    }
    
    /// <summary>
    /// Convert rotation matrix to quaternion.
    /// </summary>
    public static Quaternion MatrixToQuaternion(Matrix<double> R)
    {
        double trace = R.Trace();
        double w, x, y, z;
        
        if (trace > 0)
        {
            double s = 0.5 / System.Math.Sqrt(trace + 1);
            w = 0.25 / s;
            x = (R[2, 1] - R[1, 2]) * s;
            y = (R[0, 2] - R[2, 0]) * s;
            z = (R[1, 0] - R[0, 1]) * s;
        }
        else if (R[0, 0] > R[1, 1] && R[0, 0] > R[2, 2])
        {
            double s = 2 * System.Math.Sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]);
            w = (R[2, 1] - R[1, 2]) / s;
            x = 0.25 * s;
            y = (R[0, 1] + R[1, 0]) / s;
            z = (R[0, 2] + R[2, 0]) / s;
        }
        else if (R[1, 1] > R[2, 2])
        {
            double s = 2 * System.Math.Sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]);
            w = (R[0, 2] - R[2, 0]) / s;
            x = (R[0, 1] + R[1, 0]) / s;
            y = 0.25 * s;
            z = (R[1, 2] + R[2, 1]) / s;
        }
        else
        {
            double s = 2 * System.Math.Sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]);
            w = (R[1, 0] - R[0, 1]) / s;
            x = (R[0, 2] + R[2, 0]) / s;
            y = (R[1, 2] + R[2, 1]) / s;
            z = 0.25 * s;
        }
        
        return new Quaternion(w, x, y, z).Normalized();
    }
    
    /// <summary>
    /// Convert quaternion to rotation matrix.
    /// </summary>
    public static Matrix<double> QuaternionToMatrix(Quaternion q)
    {
        double w = q.W, x = q.X, y = q.Y, z = q.Z;
        
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y) },
            { 2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x) },
            { 2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y) }
        });
    }
    
    #endregion
    
    #region SE(2) - Planar Rigid Body Transformations
    
    /// <summary>
    /// Exponential map se(2) ? SE(2). xi = [?, vx, vy]
    /// </summary>
    public static Matrix<double> ExpSE2(Vector<double> xi)
    {
        double theta = xi[0];
        double vx = xi[1], vy = xi[2];
        
        Matrix<double> V;
        if (System.Math.Abs(theta) < Epsilon)
        {
            V = Matrix<double>.Build.DenseIdentity(2);
        }
        else
        {
            double s = System.Math.Sin(theta) / theta;
            double c = (1 - System.Math.Cos(theta)) / theta;
            V = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { s, -c },
                { c, s }
            });
        }
        
        var v = Vector<double>.Build.DenseOfArray([vx, vy]);
        var t = V * v;
        
        double cosT = System.Math.Cos(theta), sinT = System.Math.Sin(theta);
        
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { cosT, -sinT, t[0] },
            { sinT, cosT, t[1] },
            { 0, 0, 1 }
        });
    }
    
    /// <summary>
    /// Logarithmic map SE(2) ? se(2).
    /// </summary>
    public static Vector<double> LogSE2(Matrix<double> T)
    {
        double theta = System.Math.Atan2(T[1, 0], T[0, 0]);
        double tx = T[0, 2], ty = T[1, 2];
        
        Matrix<double> VInv;
        if (System.Math.Abs(theta) < Epsilon)
        {
            VInv = Matrix<double>.Build.DenseIdentity(2);
        }
        else
        {
            double halfTheta = theta / 2;
            double cotHalf = System.Math.Cos(halfTheta) / System.Math.Sin(halfTheta);
            VInv = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { halfTheta * cotHalf, halfTheta },
                { -halfTheta, halfTheta * cotHalf }
            });
        }
        
        var t = Vector<double>.Build.DenseOfArray([tx, ty]);
        var v = VInv * t;
        
        return Vector<double>.Build.DenseOfArray([theta, v[0], v[1]]);
    }
    
    #endregion
    
    #region Uncertainty Propagation on Manifolds
    
    /// <summary>
    /// Propagate Gaussian uncertainty through SE(3) composition.
    /// Given T1 ~ N(?1, ?1) and T2 ~ N(?2, ?2), compute ? and ? of T1 ? T2.
    /// </summary>
    public static (Matrix<double> mean, Matrix<double> covariance) PropagateUncertaintySE3(
        Matrix<double> T1, Matrix<double> Sigma1,
        Matrix<double> T2, Matrix<double> Sigma2)
    {
        var Tmean = ComposeSE3(T1, T2);
        
        // First-order uncertainty propagation using adjoint
        var Ad1 = AdjointSE3(T1);
        var Sigma = Sigma1 + Ad1 * Sigma2 * Ad1.Transpose();
        
        return (Tmean, Sigma);
    }
    
    /// <summary>
    /// Compute the Mahalanobis distance between two SE(3) poses.
    /// </summary>
    public static double MahalanobisDistanceSE3(
        Matrix<double> T1, Matrix<double> T2, Matrix<double> Sigma)
    {
        var delta = BoxMinusSE3(T1, T2);
        var SigmaInv = Sigma.PseudoInverse();
        return System.Math.Sqrt((delta * SigmaInv * delta));
    }
    
    /// <summary>
    /// Sample from a Gaussian distribution on SE(3).
    /// </summary>
    public static Matrix<double> SampleSE3(Matrix<double> mean, Matrix<double> Sigma, Random? rng = null)
    {
        rng ??= new Random();
        
        // Cholesky decomposition
        var L = Sigma.Cholesky().Factor;
        
        // Sample standard normal
        var z = Vector<double>.Build.Dense(6, _ => SampleStandardNormal(rng));
        
        // Transform to desired distribution
        var delta = L * z;
        
        return BoxPlusSE3(mean, delta);
    }
    
    private static double SampleStandardNormal(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
    
    #endregion
    
    #region Helper Functions
    
    /// <summary>
    /// Skew-symmetric matrix from 3-vector: [v]× such that [v]× w = v × w
    /// </summary>
    public static Matrix<double> Skew3(Vector<double> v)
    {
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0, -v[2], v[1] },
            { v[2], 0, -v[0] },
            { -v[1], v[0], 0 }
        });
    }
    
    /// <summary>
    /// Inverse of skew-symmetric: extracts vector from skew matrix.
    /// </summary>
    public static Vector<double> Vee3(Matrix<double> S)
    {
        return Vector<double>.Build.DenseOfArray([
            S[2, 1],
            S[0, 2],
            S[1, 0]
        ]);
    }
    
    /// <summary>
    /// Q matrix for SE(3) Jacobian computation.
    /// </summary>
    private static Matrix<double> ComputeQMatrix(Vector<double> omega, Vector<double> v)
    {
        double theta = omega.L2Norm();
        
        if (theta < Epsilon)
        {
            return 0.5 * Skew3(v);
        }
        
        var wSkew = Skew3(omega);
        var vSkew = Skew3(v);
        double theta2 = theta * theta;
        double theta3 = theta2 * theta;
        double theta4 = theta3 * theta;
        double theta5 = theta4 * theta;
        
        double s = System.Math.Sin(theta);
        double c = System.Math.Cos(theta);
        
        var wvSkew = wSkew * vSkew;
        var vwSkew = vSkew * wSkew;
        var wwvSkew = wSkew * wSkew * vSkew;
        
        double c1 = (theta - s) / theta3;
        double c2 = (theta2 + 2 * c - 2) / (2 * theta4);
        double c3 = (2 * theta - 3 * s + theta * c) / (2 * theta5);
        
        return 0.5 * vSkew +
               c1 * (wvSkew + vwSkew + wSkew * vwSkew) +
               c2 * (wSkew * wvSkew + vwSkew * wSkew - 3 * wSkew * vwSkew) +
               c3 * (wwvSkew * wSkew + wSkew * wwvSkew);
    }
    
    #endregion
}

/// <summary>
/// Unit quaternion representing 3D rotation.
/// </summary>
public readonly struct Quaternion
{
    public double W { get; }
    public double X { get; }
    public double Y { get; }
    public double Z { get; }
    
    public Quaternion(double w, double x, double y, double z)
    {
        W = w; X = x; Y = y; Z = z;
    }
    
    public double Norm => System.Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
    
    public Quaternion Normalized()
    {
        double n = Norm;
        return new Quaternion(W / n, X / n, Y / n, Z / n);
    }
    
    public Quaternion Conjugate => new(W, -X, -Y, -Z);
    
    public Quaternion Inverse => new Quaternion(W, -X, -Y, -Z).Normalized();
    
    public static Quaternion operator *(Quaternion a, Quaternion b)
    {
        return new Quaternion(
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W
        );
    }
    
    public Vector<double> Rotate(Vector<double> v)
    {
        var p = new Quaternion(0, v[0], v[1], v[2]);
        var result = this * p * Conjugate;
        return Vector<double>.Build.DenseOfArray([result.X, result.Y, result.Z]);
    }
    
    /// <summary>
    /// Spherical linear interpolation between quaternions.
    /// </summary>
    public static Quaternion Slerp(Quaternion q1, Quaternion q2, double t)
    {
        double dot = q1.W * q2.W + q1.X * q2.X + q1.Y * q2.Y + q1.Z * q2.Z;
        
        if (dot < 0)
        {
            q2 = new Quaternion(-q2.W, -q2.X, -q2.Y, -q2.Z);
            dot = -dot;
        }
        
        if (dot > 0.9995)
        {
            // Linear interpolation for nearly identical quaternions
            return new Quaternion(
                q1.W + t * (q2.W - q1.W),
                q1.X + t * (q2.X - q1.X),
                q1.Y + t * (q2.Y - q1.Y),
                q1.Z + t * (q2.Z - q1.Z)
            ).Normalized();
        }
        
        double theta0 = System.Math.Acos(dot);
        double theta = theta0 * t;
        double sinTheta = System.Math.Sin(theta);
        double sinTheta0 = System.Math.Sin(theta0);
        
        double s0 = System.Math.Cos(theta) - dot * sinTheta / sinTheta0;
        double s1 = sinTheta / sinTheta0;
        
        return new Quaternion(
            s0 * q1.W + s1 * q2.W,
            s0 * q1.X + s1 * q2.X,
            s0 * q1.Y + s1 * q2.Y,
            s0 * q1.Z + s1 * q2.Z
        );
    }
    
    public static Quaternion Identity => new(1, 0, 0, 0);
    
    public static Quaternion FromAxisAngle(Vector<double> axis, double angle)
    {
        double halfAngle = angle / 2;
        double s = System.Math.Sin(halfAngle);
        return new Quaternion(System.Math.Cos(halfAngle), axis[0] * s, axis[1] * s, axis[2] * s);
    }
    
    public static Quaternion FromEuler(double roll, double pitch, double yaw)
    {
        double cr = System.Math.Cos(roll / 2), sr = System.Math.Sin(roll / 2);
        double cp = System.Math.Cos(pitch / 2), sp = System.Math.Sin(pitch / 2);
        double cy = System.Math.Cos(yaw / 2), sy = System.Math.Sin(yaw / 2);
        
        return new Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }
    
    public (double roll, double pitch, double yaw) ToEuler()
    {
        double sinr_cosp = 2 * (W * X + Y * Z);
        double cosr_cosp = 1 - 2 * (X * X + Y * Y);
        double roll = System.Math.Atan2(sinr_cosp, cosr_cosp);
        
        double sinp = 2 * (W * Y - Z * X);
        double pitch = System.Math.Abs(sinp) >= 1 
            ? System.Math.CopySign(System.Math.PI / 2, sinp) 
            : System.Math.Asin(sinp);
        
        double siny_cosp = 2 * (W * Z + X * Y);
        double cosy_cosp = 1 - 2 * (Y * Y + Z * Z);
        double yaw = System.Math.Atan2(siny_cosp, cosy_cosp);
        
        return (roll, pitch, yaw);
    }
}

/// <summary>
/// Dual quaternion for unified representation of rigid body transformations.
/// Combines rotation and translation in a single algebraic structure.
/// </summary>
public readonly struct DualQuaternion
{
    public Quaternion Real { get; }
    public Quaternion Dual { get; }
    
    public DualQuaternion(Quaternion real, Quaternion dual)
    {
        Real = real;
        Dual = dual;
    }
    
    public static DualQuaternion FromTransform(Quaternion rotation, Vector<double> translation)
    {
        var t = new Quaternion(0, translation[0] / 2, translation[1] / 2, translation[2] / 2);
        return new DualQuaternion(rotation, t * rotation);
    }
    
    public static DualQuaternion FromSE3(Matrix<double> T)
    {
        var R = T.SubMatrix(0, 3, 0, 3);
        var t = Vector<double>.Build.DenseOfArray([T[0, 3], T[1, 3], T[2, 3]]);
        var q = LieGroups.MatrixToQuaternion(R);
        return FromTransform(q, t);
    }
    
    public Matrix<double> ToSE3()
    {
        var R = LieGroups.QuaternionToMatrix(Real);
        var tQuat = Dual * Real.Conjugate;
        var t = Vector<double>.Build.DenseOfArray([2 * tQuat.X, 2 * tQuat.Y, 2 * tQuat.Z]);
        
        var T = Matrix<double>.Build.DenseIdentity(4);
        T.SetSubMatrix(0, 0, R);
        T[0, 3] = t[0];
        T[1, 3] = t[1];
        T[2, 3] = t[2];
        return T;
    }
    
    public DualQuaternion Conjugate => new(Real.Conjugate, Dual.Conjugate);
    
    public static DualQuaternion operator *(DualQuaternion a, DualQuaternion b)
    {
        return new DualQuaternion(
            a.Real * b.Real,
            new Quaternion(
                a.Real.W * b.Dual.W - a.Real.X * b.Dual.X - a.Real.Y * b.Dual.Y - a.Real.Z * b.Dual.Z +
                a.Dual.W * b.Real.W - a.Dual.X * b.Real.X - a.Dual.Y * b.Real.Y - a.Dual.Z * b.Real.Z,
                a.Real.W * b.Dual.X + a.Real.X * b.Dual.W + a.Real.Y * b.Dual.Z - a.Real.Z * b.Dual.Y +
                a.Dual.W * b.Real.X + a.Dual.X * b.Real.W + a.Dual.Y * b.Real.Z - a.Dual.Z * b.Real.Y,
                a.Real.W * b.Dual.Y - a.Real.X * b.Dual.Z + a.Real.Y * b.Dual.W + a.Real.Z * b.Dual.X +
                a.Dual.W * b.Real.Y - a.Dual.X * b.Real.Z + a.Dual.Y * b.Real.W + a.Dual.Z * b.Real.X,
                a.Real.W * b.Dual.Z + a.Real.X * b.Dual.Y - a.Real.Y * b.Dual.X + a.Real.Z * b.Dual.W +
                a.Dual.W * b.Real.Z + a.Dual.X * b.Real.Y - a.Dual.Y * b.Real.X + a.Dual.Z * b.Real.W
            )
        );
    }
    
    /// <summary>
    /// Screw linear interpolation (ScLERP) for smooth motion interpolation.
    /// </summary>
    public static DualQuaternion ScLerp(DualQuaternion dq1, DualQuaternion dq2, double t)
    {
        // Compute relative transformation
        var dqRel = dq1.Conjugate * dq2;
        
        // Compute screw parameters
        var (axis, angle, pitch, moment) = dqRel.ToScrew();
        
        // Interpolate
        double tAngle = t * angle;
        double tPitch = t * pitch;
        
        // Reconstruct interpolated dual quaternion
        var dqInterp = FromScrew(axis, tAngle, tPitch, moment);
        
        return dq1 * dqInterp;
    }
    
    public (Vector<double> axis, double angle, double pitch, Vector<double> moment) ToScrew()
    {
        // Extract rotation angle
        double angle = 2 * System.Math.Acos(Real.W);
        
        if (System.Math.Abs(angle) < 1e-10)
        {
            // Pure translation
            var t = Vector<double>.Build.DenseOfArray([2 * Dual.X, 2 * Dual.Y, 2 * Dual.Z]);
            return (t.Normalize(2), 0, t.L2Norm(), Vector<double>.Build.Dense(3));
        }
        
        var axis = Vector<double>.Build.DenseOfArray([Real.X, Real.Y, Real.Z]).Normalize(2);
        double pitch = -2 * Dual.W / System.Math.Sin(angle / 2);
        
        var moment = Vector<double>.Build.DenseOfArray([
            Dual.X - Real.X * pitch / 2,
            Dual.Y - Real.Y * pitch / 2,
            Dual.Z - Real.Z * pitch / 2
        ]) / System.Math.Sin(angle / 2);
        
        return (axis, angle, pitch, moment);
    }
    
    public static DualQuaternion FromScrew(Vector<double> axis, double angle, double pitch, Vector<double> moment)
    {
        double halfAngle = angle / 2;
        double sinHalf = System.Math.Sin(halfAngle);
        double cosHalf = System.Math.Cos(halfAngle);
        
        var real = new Quaternion(cosHalf, axis[0] * sinHalf, axis[1] * sinHalf, axis[2] * sinHalf);
        var dual = new Quaternion(
            -pitch / 2 * sinHalf,
            moment[0] * sinHalf + axis[0] * pitch / 2 * cosHalf,
            moment[1] * sinHalf + axis[1] * pitch / 2 * cosHalf,
            moment[2] * sinHalf + axis[2] * pitch / 2 * cosHalf
        );
        
        return new DualQuaternion(real, dual);
    }
    
    public static DualQuaternion Identity => new(Quaternion.Identity, new Quaternion(0, 0, 0, 0));
}
