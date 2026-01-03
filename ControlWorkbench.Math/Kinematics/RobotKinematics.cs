using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Kinematics;

/// <summary>
/// Forward and inverse kinematics for robotic manipulators.
/// </summary>
public class RobotKinematics
{
    private readonly double[,] _dhParameters; // n x 4 matrix: [a, alpha, d, theta]
    private readonly int _jointCount;
    private readonly JointType[] _jointTypes;

    public enum JointType { Revolute, Prismatic }

    /// <summary>
    /// Creates a robot kinematics solver using DH parameters.
    /// </summary>
    /// <param name="dhParameters">DH parameters [a, alpha, d, theta] for each joint</param>
    /// <param name="jointTypes">Type of each joint (revolute or prismatic)</param>
    public RobotKinematics(double[,] dhParameters, JointType[]? jointTypes = null)
    {
        _dhParameters = dhParameters;
        _jointCount = dhParameters.GetLength(0);
        _jointTypes = jointTypes ?? Enumerable.Repeat(JointType.Revolute, _jointCount).ToArray();
    }

    /// <summary>
    /// Computes the transformation matrix from base to end-effector.
    /// </summary>
    public Matrix<double> ForwardKinematics(double[] jointAngles)
    {
        if (jointAngles.Length != _jointCount)
            throw new ArgumentException("Joint angles count must match DH parameters");

        var T = Matrix<double>.Build.DenseIdentity(4);

        for (int i = 0; i < _jointCount; i++)
        {
            double a = _dhParameters[i, 0];
            double alpha = _dhParameters[i, 1];
            double d = _dhParameters[i, 2];
            double theta = _dhParameters[i, 3];

            // Apply joint variable
            if (_jointTypes[i] == JointType.Revolute)
                theta += jointAngles[i];
            else
                d += jointAngles[i];

            // DH transformation matrix
            var Ti = DHMatrix(a, alpha, d, theta);
            T = T * Ti;
        }

        return T;
    }

    /// <summary>
    /// Computes the Jacobian matrix at the current configuration.
    /// </summary>
    public Matrix<double> Jacobian(double[] jointAngles)
    {
        var J = Matrix<double>.Build.Dense(6, _jointCount);
        var T = Matrix<double>.Build.DenseIdentity(4);
        var transformations = new Matrix<double>[_jointCount + 1];
        transformations[0] = T.Clone();

        // Compute all transformations
        for (int i = 0; i < _jointCount; i++)
        {
            double a = _dhParameters[i, 0];
            double alpha = _dhParameters[i, 1];
            double d = _dhParameters[i, 2];
            double theta = _dhParameters[i, 3];

            if (_jointTypes[i] == JointType.Revolute)
                theta += jointAngles[i];
            else
                d += jointAngles[i];

            T = T * DHMatrix(a, alpha, d, theta);
            transformations[i + 1] = T.Clone();
        }

        // End-effector position
        var pe = T.Column(3).SubVector(0, 3);

        // Compute Jacobian columns
        for (int i = 0; i < _jointCount; i++)
        {
            var Ti = transformations[i];
            var zi = Ti.Column(2).SubVector(0, 3); // z-axis of frame i
            var pi = Ti.Column(3).SubVector(0, 3); // origin of frame i

            if (_jointTypes[i] == JointType.Revolute)
            {
                // Linear velocity: z × (pe - pi)
                var Jv = CrossProduct(zi, pe - pi);
                J.SetSubMatrix(0, i, Jv.ToColumnMatrix());
                
                // Angular velocity: z
                J.SetSubMatrix(3, i, zi.ToColumnMatrix());
            }
            else
            {
                // Prismatic: linear velocity is z, angular is 0
                J.SetSubMatrix(0, i, zi.ToColumnMatrix());
            }
        }

        return J;
    }

    /// <summary>
    /// Solves inverse kinematics using iterative Jacobian method.
    /// </summary>
    public double[]? InverseKinematics(
        Matrix<double> targetPose,
        double[] initialGuess,
        double tolerance = 1e-6,
        int maxIterations = 100)
    {
        var q = Vector<double>.Build.DenseOfArray(initialGuess);
        var targetPos = targetPose.Column(3).SubVector(0, 3);
        var targetRot = targetPose.SubMatrix(0, 3, 0, 3);

        for (int iter = 0; iter < maxIterations; iter++)
        {
            var currentPose = ForwardKinematics(q.ToArray());
            var currentPos = currentPose.Column(3).SubVector(0, 3);
            var currentRot = currentPose.SubMatrix(0, 3, 0, 3);

            // Position error
            var posError = targetPos - currentPos;

            // Orientation error (using angle-axis representation)
            var rotError = RotationError(currentRot, targetRot);

            // Combined error
            var error = Vector<double>.Build.Dense(6);
            error.SetSubVector(0, 3, posError);
            error.SetSubVector(3, 3, rotError);

            if (error.L2Norm() < tolerance)
                return q.ToArray();

            // Jacobian
            var J = Jacobian(q.ToArray());

            // Damped least squares (Levenberg-Marquardt)
            double lambda = 0.01;
            var JJt = J * J.Transpose();
            var dampedInverse = J.Transpose() * (JJt + lambda * Matrix<double>.Build.DenseIdentity(6)).Inverse();

            // Update
            var dq = dampedInverse * error;
            q = q + dq;
        }

        return null; // Did not converge
    }

    /// <summary>
    /// Computes inverse kinematics analytically for 6-DOF robots (if available).
    /// </summary>
    public double[][]? AnalyticalIK(Matrix<double> targetPose)
    {
        // This would contain analytical solutions for specific robot configurations
        // (e.g., Puma 560, UR5, KUKA)
        // Returns all possible solutions
        throw new NotImplementedException("Analytical IK requires robot-specific implementation");
    }

    private static Matrix<double> DHMatrix(double a, double alpha, double d, double theta)
    {
        double ct = System.Math.Cos(theta);
        double st = System.Math.Sin(theta);
        double ca = System.Math.Cos(alpha);
        double sa = System.Math.Sin(alpha);

        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { ct, -st * ca,  st * sa, a * ct },
            { st,  ct * ca, -ct * sa, a * st },
            { 0,   sa,       ca,      d },
            { 0,   0,        0,       1 }
        });
    }

    private static Vector<double> CrossProduct(Vector<double> a, Vector<double> b)
    {
        return Vector<double>.Build.DenseOfArray(new[]
        {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        });
    }

    private static Vector<double> RotationError(Matrix<double> R1, Matrix<double> R2)
    {
        var Re = R1.Transpose() * R2;
        // Extract angle-axis from rotation matrix
        double angle = System.Math.Acos((Re.Trace() - 1) / 2);
        
        if (System.Math.Abs(angle) < 1e-10)
            return Vector<double>.Build.Dense(3);

        double k = angle / (2 * System.Math.Sin(angle));
        return Vector<double>.Build.DenseOfArray(new[]
        {
            k * (Re[2, 1] - Re[1, 2]),
            k * (Re[0, 2] - Re[2, 0]),
            k * (Re[1, 0] - Re[0, 1])
        });
    }
}

/// <summary>
/// Dynamics computation for robotic systems.
/// </summary>
public class RobotDynamics
{
    private readonly int _n; // Number of joints
    private readonly double[] _masses;
    private readonly Matrix<double>[] _inertias;
    private readonly Vector<double>[] _comPositions;
    private readonly double[,] _dhParameters;

    public RobotDynamics(
        double[,] dhParameters,
        double[] masses,
        Matrix<double>[] inertias,
        Vector<double>[] comPositions)
    {
        _dhParameters = dhParameters;
        _n = dhParameters.GetLength(0);
        _masses = masses;
        _inertias = inertias;
        _comPositions = comPositions;
    }

    /// <summary>
    /// Computes the mass matrix M(q).
    /// </summary>
    public Matrix<double> MassMatrix(double[] q)
    {
        var M = Matrix<double>.Build.Dense(_n, _n);
        
        // Using composite rigid body algorithm
        // Simplified implementation - real implementation would use recursive Newton-Euler
        for (int i = 0; i < _n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                double mij = ComputeMassElement(q, i, j);
                M[i, j] = mij;
                M[j, i] = mij; // Symmetric
            }
        }
        
        return M;
    }

    /// <summary>
    /// Computes the Coriolis/centrifugal matrix C(q, qdot).
    /// </summary>
    public Matrix<double> CoriolisMatrix(double[] q, double[] qDot)
    {
        var C = Matrix<double>.Build.Dense(_n, _n);
        
        // Using Christoffel symbols
        double eps = 1e-6;
        
        for (int i = 0; i < _n; i++)
        {
            for (int j = 0; j < _n; j++)
            {
                double cij = 0;
                for (int k = 0; k < _n; k++)
                {
                    // Christoffel symbol c_ijk
                    var qPlus = (double[])q.Clone();
                    var qMinus = (double[])q.Clone();
                    
                    qPlus[k] += eps;
                    qMinus[k] -= eps;
                    
                    var MPlus = MassMatrix(qPlus);
                    var MMinus = MassMatrix(qMinus);
                    
                    double dMij_dqk = (MPlus[i, j] - MMinus[i, j]) / (2 * eps);
                    double dMik_dqj = (MPlus[i, k] - MMinus[i, k]) / (2 * eps);
                    double dMjk_dqi = (MPlus[j, k] - MMinus[j, k]) / (2 * eps);
                    
                    double christoffel = 0.5 * (dMij_dqk + dMik_dqj - dMjk_dqi);
                    cij += christoffel * qDot[k];
                }
                C[i, j] = cij;
            }
        }
        
        return C;
    }

    /// <summary>
    /// Computes the gravity vector G(q).
    /// </summary>
    public Vector<double> GravityVector(double[] q, double[] gravity = null!)
    {
        gravity ??= new[] { 0, 0, -9.81 };
        var G = Vector<double>.Build.Dense(_n);
        
        // Compute potential energy gradient
        double eps = 1e-6;
        
        for (int i = 0; i < _n; i++)
        {
            var qPlus = (double[])q.Clone();
            var qMinus = (double[])q.Clone();
            qPlus[i] += eps;
            qMinus[i] -= eps;
            
            double Vplus = ComputePotentialEnergy(qPlus, gravity);
            double Vminus = ComputePotentialEnergy(qMinus, gravity);
            
            G[i] = (Vplus - Vminus) / (2 * eps);
        }
        
        return G;
    }

    /// <summary>
    /// Computes inverse dynamics: ? = M(q)q? + C(q,q?)q? + G(q)
    /// </summary>
    public Vector<double> InverseDynamics(double[] q, double[] qDot, double[] qDDot)
    {
        var M = MassMatrix(q);
        var C = CoriolisMatrix(q, qDot);
        var G = GravityVector(q);
        
        var qDotVec = Vector<double>.Build.DenseOfArray(qDot);
        var qDDotVec = Vector<double>.Build.DenseOfArray(qDDot);
        
        return M * qDDotVec + C * qDotVec + G;
    }

    /// <summary>
    /// Computes forward dynamics: q? = M?¹(? - C(q,q?)q? - G(q))
    /// </summary>
    public Vector<double> ForwardDynamics(double[] q, double[] qDot, double[] tau)
    {
        var M = MassMatrix(q);
        var C = CoriolisMatrix(q, qDot);
        var G = GravityVector(q);
        
        var qDotVec = Vector<double>.Build.DenseOfArray(qDot);
        var tauVec = Vector<double>.Build.DenseOfArray(tau);
        
        return M.Inverse() * (tauVec - C * qDotVec - G);
    }

    private double ComputeMassElement(double[] q, int i, int j)
    {
        // Simplified - real implementation uses link Jacobians
        return _masses[System.Math.Max(i, j)] * 0.1; // Placeholder
    }

    private double ComputePotentialEnergy(double[] q, double[] gravity)
    {
        double V = 0;
        var g = Vector<double>.Build.DenseOfArray(gravity);
        
        // Sum potential energy of each link
        var kinematics = new RobotKinematics(_dhParameters, null);
        
        for (int i = 0; i < _n; i++)
        {
            var T = kinematics.ForwardKinematics(q.Take(i + 1).Concat(new double[_n - i - 1]).ToArray());
            var position = T.Column(3).SubVector(0, 3);
            V -= _masses[i] * g.DotProduct(position);
        }
        
        return V;
    }
}

/// <summary>
/// Quaternion operations for rotation representation.
/// </summary>
public static class QuaternionMath
{
    /// <summary>
    /// Creates a quaternion from Euler angles (roll, pitch, yaw).
    /// </summary>
    public static double[] FromEuler(double roll, double pitch, double yaw)
    {
        double cr = System.Math.Cos(roll / 2);
        double sr = System.Math.Sin(roll / 2);
        double cp = System.Math.Cos(pitch / 2);
        double sp = System.Math.Sin(pitch / 2);
        double cy = System.Math.Cos(yaw / 2);
        double sy = System.Math.Sin(yaw / 2);

        return new[]
        {
            cr * cp * cy + sr * sp * sy, // w
            sr * cp * cy - cr * sp * sy, // x
            cr * sp * cy + sr * cp * sy, // y
            cr * cp * sy - sr * sp * cy  // z
        };
    }

    /// <summary>
    /// Converts quaternion to Euler angles.
    /// </summary>
    public static (double roll, double pitch, double yaw) ToEuler(double[] q)
    {
        double w = q[0], x = q[1], y = q[2], z = q[3];

        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        double roll = System.Math.Atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        double pitch = System.Math.Abs(sinp) >= 1
            ? System.Math.CopySign(System.Math.PI / 2, sinp)
            : System.Math.Asin(sinp);

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = System.Math.Atan2(siny_cosp, cosy_cosp);

        return (roll, pitch, yaw);
    }

    /// <summary>
    /// Multiplies two quaternions.
    /// </summary>
    public static double[] Multiply(double[] q1, double[] q2)
    {
        return new[]
        {
            q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
            q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
            q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
            q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
        };
    }

    /// <summary>
    /// Quaternion conjugate (inverse for unit quaternions).
    /// </summary>
    public static double[] Conjugate(double[] q)
    {
        return new[] { q[0], -q[1], -q[2], -q[3] };
    }

    /// <summary>
    /// Normalizes a quaternion.
    /// </summary>
    public static double[] Normalize(double[] q)
    {
        double norm = System.Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        return new[] { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
    }

    /// <summary>
    /// Spherical linear interpolation (SLERP).
    /// </summary>
    public static double[] Slerp(double[] q1, double[] q2, double t)
    {
        double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

        // If dot < 0, negate one quaternion
        if (dot < 0)
        {
            q2 = new[] { -q2[0], -q2[1], -q2[2], -q2[3] };
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            // Linear interpolation for very close quaternions
            var result = new double[]
            {
                q1[0] + t * (q2[0] - q1[0]),
                q1[1] + t * (q2[1] - q1[1]),
                q1[2] + t * (q2[2] - q1[2]),
                q1[3] + t * (q2[3] - q1[3])
            };
            return Normalize(result);
        }

        double theta0 = System.Math.Acos(dot);
        double theta = theta0 * t;
        double sinTheta = System.Math.Sin(theta);
        double sinTheta0 = System.Math.Sin(theta0);

        double s1 = System.Math.Cos(theta) - dot * sinTheta / sinTheta0;
        double s2 = sinTheta / sinTheta0;

        return new[]
        {
            s1 * q1[0] + s2 * q2[0],
            s1 * q1[1] + s2 * q2[1],
            s1 * q1[2] + s2 * q2[2],
            s1 * q1[3] + s2 * q2[3]
        };
    }

    /// <summary>
    /// Converts quaternion to rotation matrix.
    /// </summary>
    public static Matrix<double> ToRotationMatrix(double[] q)
    {
        double w = q[0], x = q[1], y = q[2], z = q[3];

        return Matrix<double>.Build.DenseOfArray(new[,]
        {
            { 1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y) },
            { 2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x) },
            { 2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y) }
        });
    }

    /// <summary>
    /// Creates quaternion from rotation matrix.
    /// </summary>
    public static double[] FromRotationMatrix(Matrix<double> R)
    {
        double trace = R[0, 0] + R[1, 1] + R[2, 2];

        if (trace > 0)
        {
            double s = 0.5 / System.Math.Sqrt(trace + 1.0);
            return new[]
            {
                0.25 / s,
                (R[2, 1] - R[1, 2]) * s,
                (R[0, 2] - R[2, 0]) * s,
                (R[1, 0] - R[0, 1]) * s
            };
        }
        else if (R[0, 0] > R[1, 1] && R[0, 0] > R[2, 2])
        {
            double s = 2.0 * System.Math.Sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]);
            return new[]
            {
                (R[2, 1] - R[1, 2]) / s,
                0.25 * s,
                (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s
            };
        }
        else if (R[1, 1] > R[2, 2])
        {
            double s = 2.0 * System.Math.Sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]);
            return new[]
            {
                (R[0, 2] - R[2, 0]) / s,
                (R[0, 1] + R[1, 0]) / s,
                0.25 * s,
                (R[1, 2] + R[2, 1]) / s
            };
        }
        else
        {
            double s = 2.0 * System.Math.Sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]);
            return new[]
            {
                (R[1, 0] - R[0, 1]) / s,
                (R[0, 2] + R[2, 0]) / s,
                (R[1, 2] + R[2, 1]) / s,
                0.25 * s
            };
        }
    }
}
