using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.SLAM;

/// <summary>
/// Visual-Inertial Odometry using direct photometric and IMU fusion.
/// Implements a keyframe-based approach similar to ORB-SLAM3/VINS-Mono.
/// </summary>
public class VisualInertialOdometry
{
    private readonly VioConfig _config;
    private readonly ImuPreintegrator _imuPreintegrator;
    private readonly FeatureTracker _featureTracker;
    private readonly SlidingWindowOptimizer _optimizer;
    
    private readonly List<Keyframe> _keyframes = new();
    private readonly List<MapPoint> _mapPoints = new();
    private Keyframe? _currentKeyframe;
    private VioState _state;
    
    private Matrix<double> _biasGyro;
    private Matrix<double> _biasAccel;
    private bool _initialized;
    
    public event Action<VioState>? StateUpdated;
    public event Action<Keyframe>? KeyframeAdded;
    
    public VioState State => _state;
    public bool IsInitialized => _initialized;
    public int KeyframeCount => _keyframes.Count;
    public int MapPointCount => _mapPoints.Count;
    
    public VisualInertialOdometry(VioConfig? config = null)
    {
        _config = config ?? VioConfig.Default;
        _imuPreintegrator = new ImuPreintegrator(_config);
        _featureTracker = new FeatureTracker(_config);
        _optimizer = new SlidingWindowOptimizer(_config);
        
        _biasGyro = Matrix<double>.Build.Dense(3, 1);
        _biasAccel = Matrix<double>.Build.Dense(3, 1);
        _state = new VioState();
    }
    
    /// <summary>
    /// Process IMU measurement.
    /// </summary>
    public void ProcessImu(ImuMeasurement imu)
    {
        if (!_initialized)
        {
            // Collect IMU data for initialization
            _imuPreintegrator.AddMeasurement(imu);
            return;
        }
        
        _imuPreintegrator.AddMeasurement(imu);
        
        // Propagate state using IMU
        PropagateImu(imu);
    }
    
    /// <summary>
    /// Process camera image.
    /// </summary>
    public void ProcessImage(VioImage image)
    {
        // Track features from previous frame
        var features = _featureTracker.Track(image);
        
        if (!_initialized)
        {
            TryInitialize(features);
            return;
        }
        
        // Check if we need a new keyframe
        if (ShouldCreateKeyframe(features))
        {
            CreateKeyframe(image, features);
        }
        
        // Triangulate new map points
        TriangulatePoints(features);
        
        // Sliding window optimization
        if (_keyframes.Count >= _config.WindowSize)
        {
            OptimizeWindow();
        }
        
        // Update state
        UpdateState();
        StateUpdated?.Invoke(_state);
    }
    
    /// <summary>
    /// Process depth image (RGB-D mode).
    /// </summary>
    public void ProcessDepth(float[,] depthImage)
    {
        if (_currentKeyframe == null) return;
        
        // Add depth to tracked features
        foreach (var feature in _currentKeyframe.Features)
        {
            int x = (int)feature.Pixel.X;
            int y = (int)feature.Pixel.Y;
            
            if (x >= 0 && x < depthImage.GetLength(0) &&
                y >= 0 && y < depthImage.GetLength(1))
            {
                float depth = depthImage[x, y];
                if (depth > 0 && depth < _config.MaxDepth)
                {
                    feature.Depth = depth;
                    feature.HasDepth = true;
                }
            }
        }
    }
    
    /// <summary>
    /// Get current pose as transformation matrix.
    /// </summary>
    public Matrix<double> GetPose()
    {
        var T = Matrix<double>.Build.DenseIdentity(4);
        T.SetSubMatrix(0, 0, _state.Rotation);
        T[0, 3] = _state.Position[0];
        T[1, 3] = _state.Position[1];
        T[2, 3] = _state.Position[2];
        return T;
    }
    
    /// <summary>
    /// Get all keyframe poses for visualization.
    /// </summary>
    public List<Matrix<double>> GetKeyframePoses()
    {
        return _keyframes.Select(kf => kf.Pose).ToList();
    }
    
    /// <summary>
    /// Get all map points for visualization.
    /// </summary>
    public List<Vector<double>> GetMapPoints()
    {
        return _mapPoints.Where(mp => mp.IsValid).Select(mp => mp.Position).ToList();
    }
    
    private void TryInitialize(List<TrackedFeature> features)
    {
        if (_imuPreintegrator.MeasurementCount < _config.InitImuCount)
            return;
        
        // Estimate initial gravity direction and bias
        var (gravity, gyroBias) = EstimateGravityAndBias();
        
        _biasGyro = gyroBias;
        _state.Rotation = AlignToGravity(gravity);
        _state.Position = Vector<double>.Build.Dense(3);
        _state.Velocity = Vector<double>.Build.Dense(3);
        
        // Create first keyframe
        CreateKeyframe(null!, features);
        
        _initialized = true;
    }
    
    private void PropagateImu(ImuMeasurement imu)
    {
        double dt = imu.Dt;
        
        // Correct measurements with bias
        var gyro = Vector<double>.Build.DenseOfArray([imu.Gx, imu.Gy, imu.Gz]) - _biasGyro.Column(0);
        var accel = Vector<double>.Build.DenseOfArray([imu.Ax, imu.Ay, imu.Az]) - _biasAccel.Column(0);
        
        // Rotate acceleration to world frame and remove gravity
        var accelWorld = _state.Rotation * accel;
        accelWorld[2] += 9.81;
        
        // Update velocity and position
        _state.Velocity += accelWorld * dt;
        _state.Position += _state.Velocity * dt + 0.5 * accelWorld * dt * dt;
        
        // Update rotation
        var omega = gyro * dt;
        _state.Rotation = _state.Rotation * ExpSO3(omega);
    }
    
    private bool ShouldCreateKeyframe(List<TrackedFeature> features)
    {
        if (_currentKeyframe == null)
            return true;
        
        // Keyframe selection criteria
        int trackedCount = features.Count(f => f.Age > 0);
        int newFeatures = features.Count(f => f.Age == 0);
        
        // Too few tracked features
        if (trackedCount < _config.MinTrackedFeatures)
            return true;
        
        // Enough new features detected
        if (newFeatures > _config.MinNewFeatures)
            return true;
        
        // Motion threshold
        double translation = (_state.Position - _currentKeyframe.Position).L2Norm();
        if (translation > _config.KeyframeTranslationThreshold)
            return true;
        
        // Rotation threshold
        var relRot = _currentKeyframe.Rotation.Transpose() * _state.Rotation;
        double angle = System.Math.Acos(System.Math.Clamp((relRot.Trace() - 1) / 2, -1, 1));
        if (angle > _config.KeyframeRotationThreshold)
            return true;
        
        return false;
    }
    
    private void CreateKeyframe(VioImage image, List<TrackedFeature> features)
    {
        var keyframe = new Keyframe
        {
            Id = _keyframes.Count,
            Timestamp = image?.Timestamp ?? DateTime.UtcNow,
            Position = _state.Position.Clone(),
            Rotation = _state.Rotation.Clone(),
            Velocity = _state.Velocity.Clone(),
            BiasGyro = _biasGyro.Clone(),
            BiasAccel = _biasAccel.Clone(),
            Features = features,
            ImuPreintegration = _imuPreintegrator.GetPreintegration()
        };
        
        keyframe.Pose = Matrix<double>.Build.DenseIdentity(4);
        keyframe.Pose.SetSubMatrix(0, 0, keyframe.Rotation);
        keyframe.Pose[0, 3] = keyframe.Position[0];
        keyframe.Pose[1, 3] = keyframe.Position[1];
        keyframe.Pose[2, 3] = keyframe.Position[2];
        
        _keyframes.Add(keyframe);
        _currentKeyframe = keyframe;
        
        // Reset IMU preintegration
        _imuPreintegrator.Reset();
        
        KeyframeAdded?.Invoke(keyframe);
        
        // Marginalize old keyframes
        while (_keyframes.Count > _config.MaxKeyframes)
        {
            MarginalizeOldestKeyframe();
        }
    }
    
    private void TriangulatePoints(List<TrackedFeature> features)
    {
        foreach (var feature in features)
        {
            if (feature.MapPoint != null)
                continue;
            
            if (feature.HasDepth)
            {
                // Direct depth triangulation
                var point3D = _currentKeyframe!.Rotation * 
                    BackProject(feature.Pixel, feature.Depth) + 
                    _currentKeyframe.Position;
                
                var mapPoint = new MapPoint
                {
                    Id = _mapPoints.Count,
                    Position = point3D,
                    IsValid = true
                };
                
                _mapPoints.Add(mapPoint);
                feature.MapPoint = mapPoint;
            }
            else if (feature.Age >= 2)
            {
                // Multi-view triangulation
                TryTriangulate(feature);
            }
        }
    }
    
    private void TryTriangulate(TrackedFeature feature)
    {
        // Find the same feature in previous keyframes
        var observations = new List<(Matrix<double> pose, Vector<double> bearing)>();
        
        foreach (var kf in _keyframes.TakeLast(_config.TriangulationFrames))
        {
            var match = kf.Features.FirstOrDefault(f => f.Id == feature.Id);
            if (match != null)
            {
                observations.Add((kf.Pose, GetBearing(match.Pixel)));
            }
        }
        
        if (observations.Count < 2)
            return;
        
        // Linear triangulation
        var point = LinearTriangulate(observations);
        
        if (point != null && IsValidTriangulation(point, observations))
        {
            var mapPoint = new MapPoint
            {
                Id = _mapPoints.Count,
                Position = point,
                IsValid = true
            };
            
            _mapPoints.Add(mapPoint);
            feature.MapPoint = mapPoint;
        }
    }
    
    private Vector<double>? LinearTriangulate(List<(Matrix<double> pose, Vector<double> bearing)> observations)
    {
        // DLT triangulation
        int n = observations.Count;
        var A = Matrix<double>.Build.Dense(n * 2, 4);
        
        for (int i = 0; i < n; i++)
        {
            var (P, b) = observations[i];
            
            A[2 * i, 0] = b[0] * P[2, 0] - P[0, 0];
            A[2 * i, 1] = b[0] * P[2, 1] - P[0, 1];
            A[2 * i, 2] = b[0] * P[2, 2] - P[0, 2];
            A[2 * i, 3] = b[0] * P[2, 3] - P[0, 3];
            
            A[2 * i + 1, 0] = b[1] * P[2, 0] - P[1, 0];
            A[2 * i + 1, 1] = b[1] * P[2, 1] - P[1, 1];
            A[2 * i + 1, 2] = b[1] * P[2, 2] - P[1, 2];
            A[2 * i + 1, 3] = b[1] * P[2, 3] - P[1, 3];
        }
        
        var svd = A.Svd();
        var X = svd.VT.Row(3);
        
        if (System.Math.Abs(X[3]) < 1e-10)
            return null;
        
        return Vector<double>.Build.DenseOfArray([
            X[0] / X[3],
            X[1] / X[3],
            X[2] / X[3]
        ]);
    }
    
    private bool IsValidTriangulation(Vector<double> point, List<(Matrix<double> pose, Vector<double> bearing)> observations)
    {
        foreach (var (pose, bearing) in observations)
        {
            // Check if point is in front of camera
            var R = pose.SubMatrix(0, 3, 0, 3);
            var t = Vector<double>.Build.DenseOfArray([pose[0, 3], pose[1, 3], pose[2, 3]]);
            var pointCam = R.Transpose() * (point - t);
            
            if (pointCam[2] < 0)
                return false;
            
            // Check reprojection error
            var projected = pointCam / pointCam[2];
            double error = (projected - bearing).L2Norm();
            
            if (error > _config.MaxReprojectionError)
                return false;
        }
        
        return true;
    }
    
    private void OptimizeWindow()
    {
        // Collect keyframes in window
        var windowKeyframes = _keyframes.TakeLast(_config.WindowSize).ToList();
        
        // Collect map points visible in window
        var windowMapPoints = new HashSet<MapPoint>();
        foreach (var kf in windowKeyframes)
        {
            foreach (var f in kf.Features)
            {
                if (f.MapPoint != null)
                    windowMapPoints.Add(f.MapPoint);
            }
        }
        
        // Build and solve optimization problem
        _optimizer.Optimize(windowKeyframes, windowMapPoints.ToList());
        
        // Update state from latest keyframe
        var latest = windowKeyframes.Last();
        _state.Position = latest.Position.Clone();
        _state.Rotation = latest.Rotation.Clone();
        _state.Velocity = latest.Velocity.Clone();
        _biasGyro = latest.BiasGyro.Clone();
        _biasAccel = latest.BiasAccel.Clone();
    }
    
    private void MarginalizeOldestKeyframe()
    {
        if (_keyframes.Count <= _config.WindowSize)
            return;
        
        var oldest = _keyframes[0];
        
        // Remove features only observed in this keyframe
        foreach (var f in oldest.Features)
        {
            if (f.MapPoint != null)
            {
                f.MapPoint.ObservationCount--;
                if (f.MapPoint.ObservationCount <= 0)
                {
                    f.MapPoint.IsValid = false;
                }
            }
        }
        
        _keyframes.RemoveAt(0);
    }
    
    private void UpdateState()
    {
        // Propagate from last keyframe using IMU preintegration
        if (_currentKeyframe != null && _imuPreintegrator.MeasurementCount > 0)
        {
            var preint = _imuPreintegrator.GetPreintegration();
            
            // Apply preintegration
            _state.Rotation = _currentKeyframe.Rotation * preint.DeltaR;
            _state.Velocity = _currentKeyframe.Velocity + 
                Vector<double>.Build.DenseOfArray([0, 0, -9.81]) * preint.Dt +
                _currentKeyframe.Rotation * preint.DeltaV;
            _state.Position = _currentKeyframe.Position + 
                _currentKeyframe.Velocity * preint.Dt +
                0.5 * Vector<double>.Build.DenseOfArray([0, 0, -9.81]) * preint.Dt * preint.Dt +
                _currentKeyframe.Rotation * preint.DeltaP;
        }
    }
    
    private (Vector<double> gravity, Matrix<double> bias) EstimateGravityAndBias()
    {
        var measurements = _imuPreintegrator.GetMeasurements();
        
        // Average acceleration gives gravity direction (in sensor frame)
        var avgAccel = Vector<double>.Build.Dense(3);
        var avgGyro = Vector<double>.Build.Dense(3);
        
        foreach (var m in measurements)
        {
            avgAccel[0] += m.Ax;
            avgAccel[1] += m.Ay;
            avgAccel[2] += m.Az;
            avgGyro[0] += m.Gx;
            avgGyro[1] += m.Gy;
            avgGyro[2] += m.Gz;
        }
        
        avgAccel /= measurements.Count;
        avgGyro /= measurements.Count;
        
        var gravity = avgAccel.Normalize(2) * 9.81;
        var gyroBias = Matrix<double>.Build.DenseOfColumnVectors(avgGyro);
        
        return (gravity, gyroBias);
    }
    
    private Matrix<double> AlignToGravity(Vector<double> gravity)
    {
        // Rotate so that measured gravity aligns with [0, 0, -1]
        var gNorm = gravity.Normalize(2);
        var zWorld = Vector<double>.Build.DenseOfArray([0, 0, -1]);
        
        var axis = CrossProduct(gNorm, zWorld);
        double angle = System.Math.Acos(gNorm * zWorld);
        
        if (axis.L2Norm() < 1e-6)
            return Matrix<double>.Build.DenseIdentity(3);
        
        return ExpSO3(axis.Normalize(2) * angle);
    }
    
    private Vector<double> BackProject(Vector2D pixel, double depth)
    {
        double fx = _config.Fx, fy = _config.Fy;
        double cx = _config.Cx, cy = _config.Cy;
        
        double x = (pixel.X - cx) / fx * depth;
        double y = (pixel.Y - cy) / fy * depth;
        
        return Vector<double>.Build.DenseOfArray([x, y, depth]);
    }
    
    private Vector<double> GetBearing(Vector2D pixel)
    {
        double fx = _config.Fx, fy = _config.Fy;
        double cx = _config.Cx, cy = _config.Cy;
        
        var bearing = Vector<double>.Build.DenseOfArray([
            (pixel.X - cx) / fx,
            (pixel.Y - cy) / fy,
            1
        ]);
        
        return bearing.Normalize(2);
    }
    
    private static Matrix<double> ExpSO3(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        
        if (theta < 1e-10)
            return Matrix<double>.Build.DenseIdentity(3);
        
        var axis = omega / theta;
        var K = SkewSymmetric(axis);
        
        return Matrix<double>.Build.DenseIdentity(3) +
               System.Math.Sin(theta) * K +
               (1 - System.Math.Cos(theta)) * K * K;
    }
    
    private static Matrix<double> SkewSymmetric(Vector<double> v)
    {
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0, -v[2], v[1] },
            { v[2], 0, -v[0] },
            { -v[1], v[0], 0 }
        });
    }
    
    private static Vector<double> CrossProduct(Vector<double> a, Vector<double> b)
    {
        return Vector<double>.Build.DenseOfArray([
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ]);
    }
}

/// <summary>
/// IMU preintegration for efficient optimization.
/// </summary>
public class ImuPreintegrator
{
    private readonly VioConfig _config;
    private readonly List<ImuMeasurement> _measurements = new();
    private ImuPreintegration _preintegration;
    
    public int MeasurementCount => _measurements.Count;
    
    public ImuPreintegrator(VioConfig config)
    {
        _config = config;
        Reset();
    }
    
    public void AddMeasurement(ImuMeasurement imu)
    {
        _measurements.Add(imu);
        Integrate(imu);
    }
    
    public void Reset()
    {
        _measurements.Clear();
        _preintegration = new ImuPreintegration
        {
            DeltaR = Matrix<double>.Build.DenseIdentity(3),
            DeltaV = Vector<double>.Build.Dense(3),
            DeltaP = Vector<double>.Build.Dense(3),
            Dt = 0
        };
    }
    
    public ImuPreintegration GetPreintegration() => _preintegration;
    
    public List<ImuMeasurement> GetMeasurements() => _measurements.ToList();
    
    private void Integrate(ImuMeasurement imu)
    {
        double dt = imu.Dt;
        
        var omega = Vector<double>.Build.DenseOfArray([imu.Gx, imu.Gy, imu.Gz]);
        var accel = Vector<double>.Build.DenseOfArray([imu.Ax, imu.Ay, imu.Az]);
        
        // Rotation increment
        var dR = ExpSO3(omega * dt);
        
        // Velocity and position increments in body frame
        var dV = accel * dt;
        var dP = accel * dt * dt * 0.5;
        
        // Update preintegration
        _preintegration.DeltaP += _preintegration.DeltaV * dt + _preintegration.DeltaR * dP;
        _preintegration.DeltaV += _preintegration.DeltaR * dV;
        _preintegration.DeltaR = _preintegration.DeltaR * dR;
        _preintegration.Dt += dt;
    }
    
    private static Matrix<double> ExpSO3(Vector<double> omega)
    {
        double theta = omega.L2Norm();
        if (theta < 1e-10)
            return Matrix<double>.Build.DenseIdentity(3);
        
        var axis = omega / theta;
        var K = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 0, -axis[2], axis[1] },
            { axis[2], 0, -axis[0] },
            { -axis[1], axis[0], 0 }
        });
        
        return Matrix<double>.Build.DenseIdentity(3) +
               System.Math.Sin(theta) * K +
               (1 - System.Math.Cos(theta)) * K * K;
    }
}

/// <summary>
/// Feature tracker using optical flow.
/// </summary>
public class FeatureTracker
{
    private readonly VioConfig _config;
    private readonly List<TrackedFeature> _features = new();
    private int _nextFeatureId;
    private VioImage? _prevImage;
    
    public FeatureTracker(VioConfig config)
    {
        _config = config;
    }
    
    public List<TrackedFeature> Track(VioImage image)
    {
        var tracked = new List<TrackedFeature>();
        
        if (_prevImage != null)
        {
            // Track existing features using Lucas-Kanade
            foreach (var feature in _features)
            {
                var newPos = TrackPoint(_prevImage, image, feature.Pixel);
                
                if (newPos != null)
                {
                    feature.Pixel = newPos.Value;
                    feature.Age++;
                    tracked.Add(feature);
                }
            }
        }
        
        // Detect new features if needed
        if (tracked.Count < _config.MaxFeatures)
        {
            var newFeatures = DetectFeatures(image, tracked, _config.MaxFeatures - tracked.Count);
            tracked.AddRange(newFeatures);
        }
        
        _features.Clear();
        _features.AddRange(tracked);
        _prevImage = image;
        
        return tracked;
    }
    
    private Vector2D? TrackPoint(VioImage prev, VioImage curr, Vector2D point)
    {
        // Simplified Lucas-Kanade optical flow
        // In real implementation, use OpenCV or similar
        
        int x = (int)point.X;
        int y = (int)point.Y;
        int winSize = 21;
        int halfWin = winSize / 2;
        
        if (x < halfWin || x >= prev.Width - halfWin ||
            y < halfWin || y >= prev.Height - halfWin)
            return null;
        
        // Compute image gradients and optical flow
        // Placeholder - actual implementation would use proper image processing
        
        // Simulate tracking with small displacement
        double dx = 0, dy = 0;
        
        var newPoint = new Vector2D(point.X + dx, point.Y + dy);
        
        if (newPoint.X < 0 || newPoint.X >= curr.Width ||
            newPoint.Y < 0 || newPoint.Y >= curr.Height)
            return null;
        
        return newPoint;
    }
    
    private List<TrackedFeature> DetectFeatures(VioImage image, List<TrackedFeature> existing, int count)
    {
        var newFeatures = new List<TrackedFeature>();
        
        // Placeholder: detect corners using FAST or similar
        // Actual implementation would use proper feature detection
        
        var rng = new Random();
        
        for (int i = 0; i < count; i++)
        {
            var feature = new TrackedFeature
            {
                Id = _nextFeatureId++,
                Pixel = new Vector2D(rng.NextDouble() * image.Width, rng.NextDouble() * image.Height),
                Age = 0
            };
            newFeatures.Add(feature);
        }
        
        return newFeatures;
    }
}

/// <summary>
/// Sliding window bundle adjustment optimizer.
/// </summary>
public class SlidingWindowOptimizer
{
    private readonly VioConfig _config;
    
    public SlidingWindowOptimizer(VioConfig config)
    {
        _config = config;
    }
    
    public void Optimize(List<Keyframe> keyframes, List<MapPoint> mapPoints)
    {
        // Gauss-Newton optimization
        for (int iter = 0; iter < _config.OptimizationIterations; iter++)
        {
            var (H, b) = BuildSystem(keyframes, mapPoints);
            
            // Solve normal equations
            var delta = H.PseudoInverse() * b;
            
            // Apply update
            ApplyUpdate(keyframes, mapPoints, delta);
            
            if (delta.L2Norm() < 1e-6)
                break;
        }
    }
    
    private (Matrix<double> H, Vector<double> b) BuildSystem(List<Keyframe> keyframes, List<MapPoint> mapPoints)
    {
        // Dimension: 15 per keyframe (pose, velocity, biases) + 3 per map point
        int kfDim = 15;
        int mpDim = 3;
        int totalDim = keyframes.Count * kfDim + mapPoints.Count * mpDim;
        
        var H = Matrix<double>.Build.Dense(totalDim, totalDim);
        var b = Vector<double>.Build.Dense(totalDim);
        
        // Add reprojection residuals
        for (int k = 0; k < keyframes.Count; k++)
        {
            var kf = keyframes[k];
            
            foreach (var feature in kf.Features)
            {
                if (feature.MapPoint == null || !feature.MapPoint.IsValid)
                    continue;
                
                int mpIdx = mapPoints.IndexOf(feature.MapPoint);
                if (mpIdx < 0) continue;
                
                // Compute reprojection error and Jacobians
                var (error, J_kf, J_mp) = ComputeReprojectionError(kf, feature.MapPoint, feature.Pixel);
                
                // Add to H and b
                int kfStart = k * kfDim;
                int mpStart = keyframes.Count * kfDim + mpIdx * mpDim;
                
                // Simplified update
                for (int i = 0; i < 6; i++)
                {
                    b[kfStart + i] -= J_kf[i] * error;
                    for (int j = 0; j < 6; j++)
                    {
                        H[kfStart + i, kfStart + j] += J_kf[i] * J_kf[j];
                    }
                }
            }
        }
        
        // Add regularization
        for (int i = 0; i < totalDim; i++)
        {
            H[i, i] += 1e-6;
        }
        
        return (H, b);
    }
    
    private (double error, double[] J_kf, double[] J_mp) ComputeReprojectionError(
        Keyframe kf, MapPoint mp, Vector2D observation)
    {
        // Project point to image
        var R = kf.Rotation;
        var t = kf.Position;
        var pointCam = R.Transpose() * (mp.Position - t);
        
        double fx = _config.Fx, fy = _config.Fy;
        double cx = _config.Cx, cy = _config.Cy;
        
        double u = fx * pointCam[0] / pointCam[2] + cx;
        double v = fy * pointCam[1] / pointCam[2] + cy;
        
        double errU = u - observation.X;
        double errV = v - observation.Y;
        double error = System.Math.Sqrt(errU * errU + errV * errV);
        
        // Jacobians (simplified)
        var J_kf = new double[6];
        var J_mp = new double[3];
        
        return (error, J_kf, J_mp);
    }
    
    private void ApplyUpdate(List<Keyframe> keyframes, List<MapPoint> mapPoints, Vector<double> delta)
    {
        int kfDim = 15;
        int mpDim = 3;
        
        for (int k = 0; k < keyframes.Count; k++)
        {
            var kf = keyframes[k];
            int start = k * kfDim;
            
            // Update position
            kf.Position[0] += delta[start];
            kf.Position[1] += delta[start + 1];
            kf.Position[2] += delta[start + 2];
            
            // Update rotation (simplified)
            
            // Update velocity
            kf.Velocity[0] += delta[start + 6];
            kf.Velocity[1] += delta[start + 7];
            kf.Velocity[2] += delta[start + 8];
        }
        
        for (int m = 0; m < mapPoints.Count; m++)
        {
            var mp = mapPoints[m];
            int start = keyframes.Count * kfDim + m * mpDim;
            
            mp.Position[0] += delta[start];
            mp.Position[1] += delta[start + 1];
            mp.Position[2] += delta[start + 2];
        }
    }
}

// Supporting types

public class VioConfig
{
    public int WindowSize { get; set; } = 10;
    public int MaxKeyframes { get; set; } = 20;
    public int InitImuCount { get; set; } = 100;
    public int MinTrackedFeatures { get; set; } = 30;
    public int MinNewFeatures { get; set; } = 20;
    public int MaxFeatures { get; set; } = 150;
    public int TriangulationFrames { get; set; } = 5;
    public int OptimizationIterations { get; set; } = 5;
    public double KeyframeTranslationThreshold { get; set; } = 0.2;
    public double KeyframeRotationThreshold { get; set; } = 0.1;
    public double MaxReprojectionError { get; set; } = 0.01;
    public double MaxDepth { get; set; } = 10;
    public double Fx { get; set; } = 500;
    public double Fy { get; set; } = 500;
    public double Cx { get; set; } = 320;
    public double Cy { get; set; } = 240;
    
    public static VioConfig Default => new();
}

public class VioState
{
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Matrix<double> Rotation { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
}

public class VioImage
{
    public byte[] Data { get; set; } = [];
    public int Width { get; set; }
    public int Height { get; set; }
    public DateTime Timestamp { get; set; }
}

public class ImuMeasurement
{
    public double Gx { get; set; }
    public double Gy { get; set; }
    public double Gz { get; set; }
    public double Ax { get; set; }
    public double Ay { get; set; }
    public double Az { get; set; }
    public double Dt { get; set; }
    public DateTime Timestamp { get; set; }
}

public class ImuPreintegration
{
    public Matrix<double> DeltaR { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    public Vector<double> DeltaV { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> DeltaP { get; set; } = Vector<double>.Build.Dense(3);
    public double Dt { get; set; }
}

public class Keyframe
{
    public int Id { get; set; }
    public DateTime Timestamp { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Matrix<double> Rotation { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Matrix<double> BiasGyro { get; set; } = Matrix<double>.Build.Dense(3, 1);
    public Matrix<double> BiasAccel { get; set; } = Matrix<double>.Build.Dense(3, 1);
    public Matrix<double> Pose { get; set; } = Matrix<double>.Build.DenseIdentity(4);
    public List<TrackedFeature> Features { get; set; } = new();
    public ImuPreintegration ImuPreintegration { get; set; } = new();
}

public class TrackedFeature
{
    public int Id { get; set; }
    public Vector2D Pixel { get; set; }
    public int Age { get; set; }
    public bool HasDepth { get; set; }
    public double Depth { get; set; }
    public MapPoint? MapPoint { get; set; }
}

public class MapPoint
{
    public int Id { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public bool IsValid { get; set; }
    public int ObservationCount { get; set; }
}

public struct Vector2D
{
    public double X { get; set; }
    public double Y { get; set; }
    
    public Vector2D(double x, double y)
    {
        X = x;
        Y = y;
    }
}
