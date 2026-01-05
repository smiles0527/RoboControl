using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Navigation;

/// <summary>
/// Pose in 3D space with position, orientation, and covariance.
/// Compatible with ArduPilot/PX4 conventions (NED frame).
/// </summary>
public class Pose3D
{
    /// <summary>
    /// Position in NED frame [North, East, Down] in meters.
    /// </summary>
    public Vector<double> Position { get; set; }
    
    /// <summary>
    /// Orientation quaternion [w, x, y, z] (Hamilton convention).
    /// </summary>
    public Quaternion Orientation { get; set; }
    
    /// <summary>
    /// Linear velocity in body frame [Vx, Vy, Vz] in m/s.
    /// </summary>
    public Vector<double> Velocity { get; set; }
    
    /// <summary>
    /// Angular velocity in body frame [p, q, r] in rad/s.
    /// </summary>
    public Vector<double> AngularVelocity { get; set; }
    
    /// <summary>
    /// 6x6 position+orientation covariance matrix.
    /// </summary>
    public Matrix<double> Covariance { get; set; }
    
    /// <summary>
    /// Timestamp in microseconds since epoch (matching MAVLink convention).
    /// </summary>
    public long TimestampUs { get; set; }
    
    /// <summary>
    /// Reference frame identifier.
    /// </summary>
    public ReferenceFrame Frame { get; set; }

    public Pose3D()
    {
        Position = Vector<double>.Build.Dense(3);
        Orientation = Quaternion.Identity;
        Velocity = Vector<double>.Build.Dense(3);
        AngularVelocity = Vector<double>.Build.Dense(3);
        Covariance = Matrix<double>.Build.DenseIdentity(6);
        Frame = ReferenceFrame.LocalNed;
    }

    /// <summary>
    /// Gets Euler angles [roll, pitch, yaw] in radians.
    /// </summary>
    public (double Roll, double Pitch, double Yaw) GetEulerAngles()
    {
        return Orientation.ToEulerAngles();
    }

    /// <summary>
    /// Gets the rotation matrix from body to NED frame.
    /// </summary>
    public Matrix<double> GetRotationMatrix()
    {
        return Orientation.ToRotationMatrix();
    }

    /// <summary>
    /// Transforms a vector from body frame to NED frame.
    /// </summary>
    public Vector<double> TransformToNed(Vector<double> bodyVector)
    {
        return GetRotationMatrix() * bodyVector;
    }

    /// <summary>
    /// Transforms a vector from NED frame to body frame.
    /// </summary>
    public Vector<double> TransformToBody(Vector<double> nedVector)
    {
        return GetRotationMatrix().Transpose() * nedVector;
    }

    /// <summary>
    /// Computes the relative pose to another pose.
    /// </summary>
    public Pose3D RelativeTo(Pose3D other)
    {
        var relativePosition = GetRotationMatrix().Transpose() * (other.Position - Position);
        var relativeOrientation = Orientation.Inverse() * other.Orientation;
        
        return new Pose3D
        {
            Position = relativePosition,
            Orientation = relativeOrientation,
            Velocity = other.Velocity - Velocity,
            AngularVelocity = other.AngularVelocity - AngularVelocity,
            TimestampUs = other.TimestampUs,
            Frame = ReferenceFrame.Body
        };
    }
}

/// <summary>
/// Reference frame identifiers (matching PX4/MAVLink conventions).
/// </summary>
public enum ReferenceFrame
{
    /// <summary>Local NED (North-East-Down) frame.</summary>
    LocalNed = 0,
    /// <summary>Local ENU (East-North-Up) frame.</summary>
    LocalEnu = 1,
    /// <summary>Global WGS84 with altitude above ellipsoid.</summary>
    GlobalWgs84 = 2,
    /// <summary>Global WGS84 with altitude above mean sea level.</summary>
    GlobalMsl = 3,
    /// <summary>Body frame (FRD: Forward-Right-Down).</summary>
    Body = 4,
    /// <summary>Vision/MoCap frame.</summary>
    Vision = 5,
    /// <summary>ECEF (Earth-Centered Earth-Fixed).</summary>
    Ecef = 6
}

/// <summary>
/// Quaternion representation for 3D rotations.
/// Uses Hamilton convention (w, x, y, z).
/// </summary>
public struct Quaternion
{
    public double W { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }

    public Quaternion(double w, double x, double y, double z)
    {
        W = w;
        X = x;
        Y = y;
        Z = z;
    }

    public static Quaternion Identity => new(1, 0, 0, 0);

    /// <summary>
    /// Creates quaternion from Euler angles [roll, pitch, yaw] in radians.
    /// Uses aerospace convention (ZYX rotation order).
    /// </summary>
    public static Quaternion FromEulerAngles(double roll, double pitch, double yaw)
    {
        double cr = System.Math.Cos(roll * 0.5);
        double sr = System.Math.Sin(roll * 0.5);
        double cp = System.Math.Cos(pitch * 0.5);
        double sp = System.Math.Sin(pitch * 0.5);
        double cy = System.Math.Cos(yaw * 0.5);
        double sy = System.Math.Sin(yaw * 0.5);

        return new Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }

    /// <summary>
    /// Creates quaternion from axis-angle representation.
    /// </summary>
    public static Quaternion FromAxisAngle(Vector<double> axis, double angle)
    {
        var normalizedAxis = axis.Normalize(2);
        double halfAngle = angle * 0.5;
        double sinHalf = System.Math.Sin(halfAngle);
        
        return new Quaternion(
            System.Math.Cos(halfAngle),
            normalizedAxis[0] * sinHalf,
            normalizedAxis[1] * sinHalf,
            normalizedAxis[2] * sinHalf
        );
    }

    /// <summary>
    /// Creates quaternion from rotation matrix.
    /// </summary>
    public static Quaternion FromRotationMatrix(Matrix<double> R)
    {
        double trace = R[0, 0] + R[1, 1] + R[2, 2];
        double w, x, y, z;

        if (trace > 0)
        {
            double s = 0.5 / System.Math.Sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (R[2, 1] - R[1, 2]) * s;
            y = (R[0, 2] - R[2, 0]) * s;
            z = (R[1, 0] - R[0, 1]) * s;
        }
        else if (R[0, 0] > R[1, 1] && R[0, 0] > R[2, 2])
        {
            double s = 2.0 * System.Math.Sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]);
            w = (R[2, 1] - R[1, 2]) / s;
            x = 0.25 * s;
            y = (R[0, 1] + R[1, 0]) / s;
            z = (R[0, 2] + R[2, 0]) / s;
        }
        else if (R[1, 1] > R[2, 2])
        {
            double s = 2.0 * System.Math.Sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]);
            w = (R[0, 2] - R[2, 0]) / s;
            x = (R[0, 1] + R[1, 0]) / s;
            y = 0.25 * s;
            z = (R[1, 2] + R[2, 1]) / s;
        }
        else
        {
            double s = 2.0 * System.Math.Sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]);
            w = (R[1, 0] - R[0, 1]) / s;
            x = (R[0, 2] + R[2, 0]) / s;
            y = (R[1, 2] + R[2, 1]) / s;
            z = 0.25 * s;
        }

        return new Quaternion(w, x, y, z).Normalized();
    }

    /// <summary>
    /// Converts quaternion to Euler angles [roll, pitch, yaw] in radians.
    /// </summary>
    public readonly (double Roll, double Pitch, double Yaw) ToEulerAngles()
    {
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (W * X + Y * Z);
        double cosr_cosp = 1 - 2 * (X * X + Y * Y);
        double roll = System.Math.Atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (W * Y - Z * X);
        double pitch = System.Math.Abs(sinp) >= 1
            ? System.Math.CopySign(System.Math.PI / 2, sinp)
            : System.Math.Asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (W * Z + X * Y);
        double cosy_cosp = 1 - 2 * (Y * Y + Z * Z);
        double yaw = System.Math.Atan2(siny_cosp, cosy_cosp);

        return (roll, pitch, yaw);
    }

    /// <summary>
    /// Converts quaternion to 3x3 rotation matrix.
    /// </summary>
    public readonly Matrix<double> ToRotationMatrix()
    {
        var q = Normalized();
        double w2 = q.W * q.W, x2 = q.X * q.X, y2 = q.Y * q.Y, z2 = q.Z * q.Z;
        double xy = q.X * q.Y, xz = q.X * q.Z, yz = q.Y * q.Z;
        double wx = q.W * q.X, wy = q.W * q.Y, wz = q.W * q.Z;

        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { w2 + x2 - y2 - z2, 2 * (xy - wz), 2 * (xz + wy) },
            { 2 * (xy + wz), w2 - x2 + y2 - z2, 2 * (yz - wx) },
            { 2 * (xz - wy), 2 * (yz + wx), w2 - x2 - y2 + z2 }
        });
    }

    public readonly double Norm() => System.Math.Sqrt(W * W + X * X + Y * Y + Z * Z);

    public readonly Quaternion Normalized()
    {
        double n = Norm();
        return n > 1e-10 ? new Quaternion(W / n, X / n, Y / n, Z / n) : Identity;
    }

    public readonly Quaternion Inverse() => new(W, -X, -Y, -Z);

    public static Quaternion operator *(Quaternion a, Quaternion b)
    {
        return new Quaternion(
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W
        );
    }

    /// <summary>
    /// Rotates a 3D vector by this quaternion.
    /// </summary>
    public readonly Vector<double> Rotate(Vector<double> v)
    {
        var qv = new Quaternion(0, v[0], v[1], v[2]);
        var result = this * qv * Inverse();
        return Vector<double>.Build.Dense([result.X, result.Y, result.Z]);
    }

    /// <summary>
    /// Spherical linear interpolation between two quaternions.
    /// </summary>
    public static Quaternion Slerp(Quaternion a, Quaternion b, double t)
    {
        double dot = a.W * b.W + a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        if (dot < 0)
        {
            b = new Quaternion(-b.W, -b.X, -b.Y, -b.Z);
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            return new Quaternion(
                a.W + t * (b.W - a.W),
                a.X + t * (b.X - a.X),
                a.Y + t * (b.Y - a.Y),
                a.Z + t * (b.Z - a.Z)
            ).Normalized();
        }

        double theta0 = System.Math.Acos(dot);
        double theta = theta0 * t;
        double sinTheta = System.Math.Sin(theta);
        double sinTheta0 = System.Math.Sin(theta0);

        double s0 = System.Math.Cos(theta) - dot * sinTheta / sinTheta0;
        double s1 = sinTheta / sinTheta0;

        return new Quaternion(
            s0 * a.W + s1 * b.W,
            s0 * a.X + s1 * b.X,
            s0 * a.Y + s1 * b.Y,
            s0 * a.Z + s1 * b.Z
        );
    }

    /// <summary>
    /// Computes the angular difference between two quaternions in radians.
    /// </summary>
    public static double AngleBetween(Quaternion a, Quaternion b)
    {
        var diff = a.Inverse() * b;
        return 2.0 * System.Math.Acos(System.Math.Clamp(System.Math.Abs(diff.W), 0, 1));
    }
}

/// <summary>
/// Multi-sensor pose estimator using Extended Kalman Filter.
/// Fuses IMU, GPS, barometer, magnetometer, and optionally vision/lidar.
/// Based on ArduPilot/PX4 EKF2/EKF3 architecture.
/// </summary>
public class MultiSensorPoseEstimator
{
    // State vector: [pn, pe, pd, vn, ve, vd, q0, q1, q2, q3, bax, bay, baz, bgx, bgy, bgz]
    // pn,pe,pd: NED position (m)
    // vn,ve,vd: NED velocity (m/s)
    // q0-q3: quaternion orientation
    // bax-baz: accelerometer bias (m/s²)
    // bgx-bgz: gyroscope bias (rad/s)
    private const int StateSize = 16;
    
    private Vector<double> _state;
    private Matrix<double> _covariance;
    private long _lastPredictTimeUs;
    
    // Process noise parameters
    private readonly PoseEstimatorConfig _config;
    
    // Sensor health tracking
    private readonly Dictionary<SensorType, SensorHealth> _sensorHealth = new();
    
    // Innovation test gates
    private readonly Dictionary<SensorType, double> _innovationGates = new();

    public MultiSensorPoseEstimator(PoseEstimatorConfig? config = null)
    {
        _config = config ?? PoseEstimatorConfig.Default;
        _state = Vector<double>.Build.Dense(StateSize);
        _state[6] = 1.0; // q0 = 1 (identity quaternion)
        _covariance = Matrix<double>.Build.DenseIdentity(StateSize);
        
        InitializeSensorHealth();
        InitializeInnovationGates();
    }

    /// <summary>
    /// Initializes the estimator with known position and orientation.
    /// </summary>
    public void Initialize(
        Vector<double>? position = null,
        Quaternion? orientation = null,
        Vector<double>? velocity = null,
        long timestampUs = 0)
    {
        _state = Vector<double>.Build.Dense(StateSize);
        
        if (position != null)
        {
            _state[0] = position[0];
            _state[1] = position[1];
            _state[2] = position[2];
        }
        
        if (velocity != null)
        {
            _state[3] = velocity[0];
            _state[4] = velocity[1];
            _state[5] = velocity[2];
        }
        
        var q = orientation ?? Quaternion.Identity;
        _state[6] = q.W;
        _state[7] = q.X;
        _state[8] = q.Y;
        _state[9] = q.Z;
        
        // Initialize covariance
        _covariance = Matrix<double>.Build.DenseIdentity(StateSize);
        _covariance.SetSubMatrix(0, 0, Matrix<double>.Build.DenseIdentity(3) * _config.InitialPositionVariance);
        _covariance.SetSubMatrix(3, 3, Matrix<double>.Build.DenseIdentity(3) * _config.InitialVelocityVariance);
        _covariance.SetSubMatrix(6, 6, Matrix<double>.Build.DenseIdentity(4) * _config.InitialAttitudeVariance);
        _covariance.SetSubMatrix(10, 10, Matrix<double>.Build.DenseIdentity(3) * _config.InitialAccelBiasVariance);
        _covariance.SetSubMatrix(13, 13, Matrix<double>.Build.DenseIdentity(3) * _config.InitialGyroBiasVariance);
        
        _lastPredictTimeUs = timestampUs;
    }

    /// <summary>
    /// Prediction step using IMU measurements.
    /// </summary>
    public void PredictImu(ImuMeasurement imu)
    {
        if (_lastPredictTimeUs == 0)
        {
            _lastPredictTimeUs = imu.TimestampUs;
            return;
        }
        
        double dt = (imu.TimestampUs - _lastPredictTimeUs) * 1e-6;
        if (dt <= 0 || dt > 1.0) // Invalid dt
        {
            _lastPredictTimeUs = imu.TimestampUs;
            return;
        }
        
        // Get current orientation
        var q = new Quaternion(_state[6], _state[7], _state[8], _state[9]);
        var R = q.ToRotationMatrix();
        
        // Remove estimated biases from IMU
        var accelCorrected = Vector<double>.Build.Dense([
            imu.AccelX - _state[10],
            imu.AccelY - _state[11],
            imu.AccelZ - _state[12]
        ]);
        
        var gyroCorrected = Vector<double>.Build.Dense([
            imu.GyroX - _state[13],
            imu.GyroY - _state[14],
            imu.GyroZ - _state[15]
        ]);
        
        // Transform acceleration to NED and remove gravity
        var accelNed = R * accelCorrected;
        accelNed[2] += _config.Gravity; // Add gravity (positive down in NED)
        
        // State prediction
        // Position: p = p + v*dt + 0.5*a*dt²
        _state[0] += _state[3] * dt + 0.5 * accelNed[0] * dt * dt;
        _state[1] += _state[4] * dt + 0.5 * accelNed[1] * dt * dt;
        _state[2] += _state[5] * dt + 0.5 * accelNed[2] * dt * dt;
        
        // Velocity: v = v + a*dt
        _state[3] += accelNed[0] * dt;
        _state[4] += accelNed[1] * dt;
        _state[5] += accelNed[2] * dt;
        
        // Quaternion update via omega integration
        double p = gyroCorrected[0];
        double qRate = gyroCorrected[1];
        double r = gyroCorrected[2];
        
        var omega = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0, -p, -qRate, -r },
            { p, 0, r, -qRate },
            { qRate, -r, 0, p },
            { r, qRate, -p, 0 }
        });
        
        var qVec = Vector<double>.Build.Dense([_state[6], _state[7], _state[8], _state[9]]);
        qVec = qVec + 0.5 * dt * omega * qVec;
        
        // Normalize quaternion
        double qNorm = qVec.L2Norm();
        _state[6] = qVec[0] / qNorm;
        _state[7] = qVec[1] / qNorm;
        _state[8] = qVec[2] / qNorm;
        _state[9] = qVec[3] / qNorm;
        
        // Covariance prediction
        var F = ComputeStateTransitionJacobian(dt, accelCorrected, gyroCorrected, R);
        var Q = ComputeProcessNoise(dt);
        
        _covariance = F * _covariance * F.Transpose() + Q;
        
        // Force symmetry
        _covariance = (_covariance + _covariance.Transpose()) * 0.5;
        
        _lastPredictTimeUs = imu.TimestampUs;
    }

    /// <summary>
    /// Updates state with GPS measurement.
    /// </summary>
    public bool UpdateGps(GpsMeasurement gps)
    {
        if (!_sensorHealth[SensorType.Gps].IsHealthy)
            return false;
        
        // Measurement: [pn, pe, pd, vn, ve, vd]
        var z = Vector<double>.Build.Dense([
            gps.PositionNorth,
            gps.PositionEast,
            gps.PositionDown,
            gps.VelocityNorth,
            gps.VelocityEast,
            gps.VelocityDown
        ]);
        
        // Measurement matrix
        var H = Matrix<double>.Build.Dense(6, StateSize);
        H[0, 0] = 1; H[1, 1] = 1; H[2, 2] = 1;
        H[3, 3] = 1; H[4, 4] = 1; H[5, 5] = 1;
        
        // Measurement noise
        var R = Matrix<double>.Build.DenseOfDiagonalArray(new double[]
        {
            gps.PositionVariance, gps.PositionVariance, gps.PositionVariance * 2,
            gps.VelocityVariance, gps.VelocityVariance, gps.VelocityVariance * 2
        });
        
        // Predicted measurement
        var zPred = H * _state;
        
        // Innovation
        var innovation = z - zPred;
        
        // Innovation gate test
        if (!PassesInnovationTest(innovation, H, R, SensorType.Gps))
        {
            _sensorHealth[SensorType.Gps].ConsecutiveFails++;
            return false;
        }
        
        // Kalman update
        var S = H * _covariance * H.Transpose() + R;
        var K = _covariance * H.Transpose() * S.Inverse();
        
        _state += K * innovation;
        
        // Joseph form covariance update
        var I = Matrix<double>.Build.DenseIdentity(StateSize);
        var IKH = I - K * H;
        _covariance = IKH * _covariance * IKH.Transpose() + K * R * K.Transpose();
        
        // Normalize quaternion
        NormalizeQuaternion();
        
        _sensorHealth[SensorType.Gps].ConsecutiveFails = 0;
        _sensorHealth[SensorType.Gps].LastUpdateUs = gps.TimestampUs;
        
        return true;
    }

    /// <summary>
    /// Updates state with barometer measurement.
    /// </summary>
    public bool UpdateBarometer(BarometerMeasurement baro)
    {
        if (!_sensorHealth[SensorType.Barometer].IsHealthy)
            return false;
        
        // Measurement: altitude (Down position)
        var z = Vector<double>.Build.Dense([baro.AltitudeMsl]);
        
        // Measurement matrix (negative because altitude is positive up, but state is NED)
        var H = Matrix<double>.Build.Dense(1, StateSize);
        H[0, 2] = -1;
        
        // Measurement noise
        var R = Matrix<double>.Build.Dense(1, 1);
        R[0, 0] = baro.Variance;
        
        // Predicted measurement
        var zPred = H * _state;
        
        // Innovation
        var innovation = z - zPred;
        
        if (!PassesInnovationTest(innovation, H, R, SensorType.Barometer))
        {
            _sensorHealth[SensorType.Barometer].ConsecutiveFails++;
            return false;
        }
        
        // Kalman update
        var S = H * _covariance * H.Transpose() + R;
        var K = _covariance * H.Transpose() * S.Inverse();
        
        _state += K * innovation;
        
        var I = Matrix<double>.Build.DenseIdentity(StateSize);
        var IKH = I - K * H;
        _covariance = IKH * _covariance * IKH.Transpose() + K * R * K.Transpose();
        
        NormalizeQuaternion();
        
        _sensorHealth[SensorType.Barometer].ConsecutiveFails = 0;
        _sensorHealth[SensorType.Barometer].LastUpdateUs = baro.TimestampUs;
        
        return true;
    }

    /// <summary>
    /// Updates state with magnetometer measurement (heading only).
    /// </summary>
    public bool UpdateMagnetometer(MagnetometerMeasurement mag)
    {
        if (!_sensorHealth[SensorType.Magnetometer].IsHealthy)
            return false;
        
        var q = new Quaternion(_state[6], _state[7], _state[8], _state[9]);
        var R_nb = q.ToRotationMatrix();
        
        // Transform magnetic field to body frame
        var magBody = Vector<double>.Build.Dense([mag.MagX, mag.MagY, mag.MagZ]);
        var magNed = R_nb * magBody;
        
        // Compute heading from magnetometer
        double measuredYaw = System.Math.Atan2(magNed[1], magNed[0]);
        
        // Get current yaw from state
        var (_, _, predictedYaw) = q.ToEulerAngles();
        
        // Innovation (heading only)
        double yawInnovation = NormalizeAngle(measuredYaw - predictedYaw);
        
        // Simplified heading update using small angle approximation
        var z = Vector<double>.Build.Dense([yawInnovation]);
        
        // Approximate H for heading (affects q0,q1,q2,q3)
        var H = ComputeHeadingMeasurementJacobian(q);
        
        var Rmag = Matrix<double>.Build.Dense(1, 1);
        Rmag[0, 0] = mag.Variance;
        
        if (!PassesInnovationTest(z, H, Rmag, SensorType.Magnetometer))
        {
            _sensorHealth[SensorType.Magnetometer].ConsecutiveFails++;
            return false;
        }
        
        var S = H * _covariance * H.Transpose() + Rmag;
        var K = _covariance * H.Transpose() * S.Inverse();
        
        _state += K * z;
        
        var I = Matrix<double>.Build.DenseIdentity(StateSize);
        var IKH = I - K * H;
        _covariance = IKH * _covariance * IKH.Transpose() + K * Rmag * K.Transpose();
        
        NormalizeQuaternion();
        
        _sensorHealth[SensorType.Magnetometer].ConsecutiveFails = 0;
        _sensorHealth[SensorType.Magnetometer].LastUpdateUs = mag.TimestampUs;
        
        return true;
    }

    /// <summary>
    /// Updates state with external vision/MoCap pose.
    /// </summary>
    public bool UpdateVision(VisionPoseMeasurement vision)
    {
        if (!_sensorHealth[SensorType.Vision].IsHealthy)
            return false;
        
        // Full 6-DOF update
        var z = Vector<double>.Build.Dense(7);
        z[0] = vision.PositionX;
        z[1] = vision.PositionY;
        z[2] = vision.PositionZ;
        z[3] = vision.QuatW;
        z[4] = vision.QuatX;
        z[5] = vision.QuatY;
        z[6] = vision.QuatZ;
        
        // Measurement matrix for position and quaternion
        var H = Matrix<double>.Build.Dense(7, StateSize);
        H[0, 0] = 1; H[1, 1] = 1; H[2, 2] = 1;
        H[3, 6] = 1; H[4, 7] = 1; H[5, 8] = 1; H[6, 9] = 1;
        
        // Measurement noise
        var R = Matrix<double>.Build.DenseOfDiagonalArray(new[]
        {
            vision.PositionVariance, vision.PositionVariance, vision.PositionVariance,
            vision.OrientationVariance, vision.OrientationVariance, 
            vision.OrientationVariance, vision.OrientationVariance
        });
        
        var zPred = H * _state;
        var innovation = z - zPred;
        
        // Handle quaternion sign ambiguity
        if (innovation[3] * _state[6] + innovation[4] * _state[7] + 
            innovation[5] * _state[8] + innovation[6] * _state[9] < 0)
        {
            for (int i = 3; i < 7; i++)
                z[i] = -z[i];
            innovation = z - zPred;
        }
        
        if (!PassesInnovationTest(innovation, H, R, SensorType.Vision))
        {
            _sensorHealth[SensorType.Vision].ConsecutiveFails++;
            return false;
        }
        
        var S = H * _covariance * H.Transpose() + R;
        var K = _covariance * H.Transpose() * S.Inverse();
        
        _state += K * innovation;
        
        var I = Matrix<double>.Build.DenseIdentity(StateSize);
        var IKH = I - K * H;
        _covariance = IKH * _covariance * IKH.Transpose() + K * R * K.Transpose();
        
        NormalizeQuaternion();
        
        _sensorHealth[SensorType.Vision].ConsecutiveFails = 0;
        _sensorHealth[SensorType.Vision].LastUpdateUs = vision.TimestampUs;
        
        return true;
    }

    /// <summary>
    /// Gets the current pose estimate.
    /// </summary>
    public Pose3D GetPose()
    {
        return new Pose3D
        {
            Position = Vector<double>.Build.Dense([_state[0], _state[1], _state[2]]),
            Orientation = new Quaternion(_state[6], _state[7], _state[8], _state[9]),
            Velocity = Vector<double>.Build.Dense([_state[3], _state[4], _state[5]]),
            AngularVelocity = Vector<double>.Build.Dense(3), // Not directly estimated
            Covariance = _covariance.SubMatrix(0, 6, 0, 6),
            TimestampUs = _lastPredictTimeUs,
            Frame = ReferenceFrame.LocalNed
        };
    }

    /// <summary>
    /// Gets the estimated IMU biases.
    /// </summary>
    public (Vector<double> AccelBias, Vector<double> GyroBias) GetBiases()
    {
        return (
            Vector<double>.Build.Dense([_state[10], _state[11], _state[12]]),
            Vector<double>.Build.Dense([_state[13], _state[14], _state[15]])
        );
    }

    /// <summary>
    /// Gets the health status of all sensors.
    /// </summary>
    public Dictionary<SensorType, SensorHealth> GetSensorHealth()
    {
        return new Dictionary<SensorType, SensorHealth>(_sensorHealth);
    }

    private Matrix<double> ComputeStateTransitionJacobian(
        double dt, Vector<double> accel, Vector<double> gyro, Matrix<double> R)
    {
        var F = Matrix<double>.Build.DenseIdentity(StateSize);
        
        // Position-velocity coupling
        F[0, 3] = dt; F[1, 4] = dt; F[2, 5] = dt;
        
        // Velocity-attitude coupling (simplified)
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                F[3 + i, 6 + j] = 2 * R[i, j] * accel.L2Norm() * dt;
            }
        }
        
        // Velocity-accel bias coupling
        F[3, 10] = -R[0, 0] * dt; F[3, 11] = -R[0, 1] * dt; F[3, 12] = -R[0, 2] * dt;
        F[4, 10] = -R[1, 0] * dt; F[4, 11] = -R[1, 1] * dt; F[4, 12] = -R[1, 2] * dt;
        F[5, 10] = -R[2, 0] * dt; F[5, 11] = -R[2, 1] * dt; F[5, 12] = -R[2, 2] * dt;
        
        // Quaternion-gyro bias coupling
        F[6, 13] = 0.5 * dt * _state[7]; F[6, 14] = 0.5 * dt * _state[8]; F[6, 15] = 0.5 * dt * _state[9];
        F[7, 13] = -0.5 * dt * _state[6]; F[7, 14] = 0.5 * dt * _state[9]; F[7, 15] = -0.5 * dt * _state[8];
        F[8, 13] = -0.5 * dt * _state[9]; F[8, 14] = -0.5 * dt * _state[6]; F[8, 15] = 0.5 * dt * _state[7];
        F[9, 13] = 0.5 * dt * _state[8]; F[9, 14] = -0.5 * dt * _state[7]; F[9, 15] = -0.5 * dt * _state[6];
        
        return F;
    }

    private Matrix<double> ComputeProcessNoise(double dt)
    {
        var Q = Matrix<double>.Build.Dense(StateSize, StateSize);
        
        // Position noise (integrated velocity noise)
        double posVar = _config.AccelNoiseSpectralDensity * dt * dt * dt / 3;
        Q[0, 0] = posVar; Q[1, 1] = posVar; Q[2, 2] = posVar;
        
        // Velocity noise (integrated acceleration noise)
        double velVar = _config.AccelNoiseSpectralDensity * dt;
        Q[3, 3] = velVar; Q[4, 4] = velVar; Q[5, 5] = velVar;
        
        // Quaternion noise (integrated gyro noise)
        double attVar = _config.GyroNoiseSpectralDensity * dt;
        Q[6, 6] = attVar * 0.25; Q[7, 7] = attVar; Q[8, 8] = attVar; Q[9, 9] = attVar;
        
        // Accel bias random walk
        double accelBiasVar = _config.AccelBiasRandomWalk * dt;
        Q[10, 10] = accelBiasVar; Q[11, 11] = accelBiasVar; Q[12, 12] = accelBiasVar;
        
        // Gyro bias random walk
        double gyroBiasVar = _config.GyroBiasRandomWalk * dt;
        Q[13, 13] = gyroBiasVar; Q[14, 14] = gyroBiasVar; Q[15, 15] = gyroBiasVar;
        
        return Q;
    }

    private Matrix<double> ComputeHeadingMeasurementJacobian(Quaternion q)
    {
        var H = Matrix<double>.Build.Dense(1, StateSize);
        
        // Partial derivatives of yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2² + q3²))
        double q0 = q.W, q1 = q.X, q2 = q.Y, q3 = q.Z;
        double num = 2 * (q0 * q3 + q1 * q2);
        double den = 1 - 2 * (q2 * q2 + q3 * q3);
        double denom2 = num * num + den * den;
        
        H[0, 6] = 2 * q3 * den / denom2;
        H[0, 7] = 2 * q2 * den / denom2;
        H[0, 8] = (2 * q1 * den + 4 * q2 * num) / denom2;
        H[0, 9] = (2 * q0 * den + 4 * q3 * num) / denom2;
        
        return H;
    }

    private bool PassesInnovationTest(Vector<double> innovation, Matrix<double> H, Matrix<double> R, SensorType sensor)
    {
        var S = H * _covariance * H.Transpose() + R;
        double nis = innovation.DotProduct(S.Inverse() * innovation);
        double gate = _innovationGates[sensor];
        
        return nis < gate;
    }

    private void NormalizeQuaternion()
    {
        double norm = System.Math.Sqrt(_state[6] * _state[6] + _state[7] * _state[7] + 
                                       _state[8] * _state[8] + _state[9] * _state[9]);
        if (norm > 1e-10)
        {
            _state[6] /= norm;
            _state[7] /= norm;
            _state[8] /= norm;
            _state[9] /= norm;
        }
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > System.Math.PI) angle -= 2 * System.Math.PI;
        while (angle < -System.Math.PI) angle += 2 * System.Math.PI;
        return angle;
    }

    private void InitializeSensorHealth()
    {
        foreach (SensorType type in Enum.GetValues<SensorType>())
        {
            _sensorHealth[type] = new SensorHealth();
        }
    }

    private void InitializeInnovationGates()
    {
        _innovationGates[SensorType.Gps] = 25.0; // Chi-squared 6-DOF 95%
        _innovationGates[SensorType.Barometer] = 5.0;
        _innovationGates[SensorType.Magnetometer] = 5.0;
        _innovationGates[SensorType.Vision] = 30.0;
        _innovationGates[SensorType.Lidar] = 10.0;
        _innovationGates[SensorType.OpticalFlow] = 10.0;
    }
}

/// <summary>
/// Sensor types for the pose estimator.
/// </summary>
public enum SensorType
{
    Imu,
    Gps,
    Barometer,
    Magnetometer,
    Vision,
    Lidar,
    OpticalFlow
}

/// <summary>
/// Sensor health tracking.
/// </summary>
public class SensorHealth
{
    public bool IsHealthy { get; set; } = true;
    public int ConsecutiveFails { get; set; }
    public long LastUpdateUs { get; set; }
    public double LastInnovation { get; set; }
    
    public bool CheckTimeout(long currentUs, long timeoutUs)
    {
        if (LastUpdateUs == 0) return false;
        return (currentUs - LastUpdateUs) > timeoutUs;
    }
}

/// <summary>
/// Configuration for pose estimator.
/// </summary>
public class PoseEstimatorConfig
{
    public double Gravity { get; set; } = 9.80665;
    public double AccelNoiseSpectralDensity { get; set; } = 0.004;
    public double GyroNoiseSpectralDensity { get; set; } = 0.001;
    public double AccelBiasRandomWalk { get; set; } = 0.0001;
    public double GyroBiasRandomWalk { get; set; } = 0.00001;
    public double InitialPositionVariance { get; set; } = 10.0;
    public double InitialVelocityVariance { get; set; } = 1.0;
    public double InitialAttitudeVariance { get; set; } = 0.1;
    public double InitialAccelBiasVariance { get; set; } = 0.1;
    public double InitialGyroBiasVariance { get; set; } = 0.01;
    
    public static PoseEstimatorConfig Default => new();
}

#region Sensor Measurements

/// <summary>
/// IMU measurement data.
/// </summary>
public record ImuMeasurement
{
    public long TimestampUs { get; init; }
    public double AccelX { get; init; } // m/s²
    public double AccelY { get; init; }
    public double AccelZ { get; init; }
    public double GyroX { get; init; } // rad/s
    public double GyroY { get; init; }
    public double GyroZ { get; init; }
}

/// <summary>
/// GPS measurement data in local NED frame.
/// </summary>
public record GpsMeasurement
{
    public long TimestampUs { get; init; }
    public double PositionNorth { get; init; } // m
    public double PositionEast { get; init; }
    public double PositionDown { get; init; }
    public double VelocityNorth { get; init; } // m/s
    public double VelocityEast { get; init; }
    public double VelocityDown { get; init; }
    public double PositionVariance { get; init; }
    public double VelocityVariance { get; init; }
    public int SatelliteCount { get; init; }
    public GpsFixType FixType { get; init; }
}

/// <summary>
/// GPS fix type (matching MAVLink GPS_FIX_TYPE).
/// </summary>
public enum GpsFixType
{
    NoGps = 0,
    NoFix = 1,
    Fix2D = 2,
    Fix3D = 3,
    DGps = 4,
    RtkFloat = 5,
    RtkFixed = 6,
    Static = 7,
    Ppp = 8
}

/// <summary>
/// Barometer measurement data.
/// </summary>
public record BarometerMeasurement
{
    public long TimestampUs { get; init; }
    public double AltitudeMsl { get; init; } // m
    public double PressurePa { get; init; }
    public double TemperatureC { get; init; }
    public double Variance { get; init; }
}

/// <summary>
/// Magnetometer measurement data.
/// </summary>
public record MagnetometerMeasurement
{
    public long TimestampUs { get; init; }
    public double MagX { get; init; } // Gauss or milliGauss
    public double MagY { get; init; }
    public double MagZ { get; init; }
    public double Variance { get; init; }
}

/// <summary>
/// External vision/MoCap pose measurement.
/// </summary>
public record VisionPoseMeasurement
{
    public long TimestampUs { get; init; }
    public double PositionX { get; init; } // m
    public double PositionY { get; init; }
    public double PositionZ { get; init; }
    public double QuatW { get; init; }
    public double QuatX { get; init; }
    public double QuatY { get; init; }
    public double QuatZ { get; init; }
    public double PositionVariance { get; init; }
    public double OrientationVariance { get; init; }
    public bool ResetRequired { get; init; }
}

#endregion
