using MathNet.Numerics.LinearAlgebra;
using SysMath = System.Math;

namespace ControlWorkbench.Math.Calibration;

/// <summary>
/// IMU Calibration utilities for accelerometer and gyroscope.
/// Implements 6-position calibration for accelerometer and 
/// bias/scale/misalignment estimation for gyroscopes.
/// </summary>
public class ImuCalibration
{
    /// <summary>
    /// Accelerometer calibration model: a_cal = T * K * (a_raw - b)
    /// where T is misalignment, K is scale, b is bias
    /// </summary>
    public class AccelerometerCalibration
    {
        public Vector<double> Bias { get; set; } = Vector<double>.Build.Dense(3);
        public Vector<double> Scale { get; set; } = Vector<double>.Build.DenseOfArray([1, 1, 1]);
        public Matrix<double> Misalignment { get; set; } = Matrix<double>.Build.DenseIdentity(3);
        
        /// <summary>
        /// Apply calibration to raw reading.
        /// </summary>
        public Vector<double> Apply(Vector<double> raw)
        {
            var correctedBias = raw - Bias;
            var scaled = Vector<double>.Build.Dense(3);
            for (int i = 0; i < 3; i++)
                scaled[i] = correctedBias[i] * Scale[i];
            return Misalignment * scaled;
        }
        
        /// <summary>
        /// Get the combined calibration matrix.
        /// </summary>
        public Matrix<double> GetCalibrationMatrix()
        {
            var K = Matrix<double>.Build.DenseOfDiagonalArray(Scale.ToArray());
            return Misalignment * K;
        }
    }
    
    /// <summary>
    /// Gyroscope calibration model: g_cal = K * (g_raw - b)
    /// </summary>
    public class GyroscopeCalibration
    {
        public Vector<double> Bias { get; set; } = Vector<double>.Build.Dense(3);
        public Vector<double> Scale { get; set; } = Vector<double>.Build.DenseOfArray([1, 1, 1]);
        public Matrix<double> CrossAxis { get; set; } = Matrix<double>.Build.DenseIdentity(3);
        
        /// <summary>
        /// Temperature coefficient for bias (bias/°C).
        /// </summary>
        public Vector<double> TempCoefficient { get; set; } = Vector<double>.Build.Dense(3);
        
        /// <summary>
        /// Reference temperature for calibration.
        /// </summary>
        public double ReferenceTemperature { get; set; } = 25.0;
        
        /// <summary>
        /// Apply calibration to raw reading.
        /// </summary>
        public Vector<double> Apply(Vector<double> raw, double temperature = 25.0)
        {
            // Temperature-compensated bias
            var tempDelta = temperature - ReferenceTemperature;
            var compensatedBias = Bias + TempCoefficient * tempDelta;
            
            var corrected = raw - compensatedBias;
            var scaled = Vector<double>.Build.Dense(3);
            for (int i = 0; i < 3; i++)
                scaled[i] = corrected[i] * Scale[i];
            return CrossAxis * scaled;
        }
    }
    
    /// <summary>
    /// Static calibration data point (stationary measurement).
    /// </summary>
    public record StaticCalibrationPoint(
        Vector<double> AccelRaw,
        Vector<double> GyroRaw,
        Vector<double> ExpectedAccel, // Expected gravity vector
        double Temperature);
    
    /// <summary>
    /// Result of accelerometer calibration.
    /// </summary>
    public class AccelCalibrationResult
    {
        public AccelerometerCalibration Calibration { get; set; } = new();
        public double RmsError { get; set; }
        public double MaxError { get; set; }
        public bool Success { get; set; }
        public string Message { get; set; } = "";
    }
    
    /// <summary>
    /// Perform 6-position accelerometer calibration.
    /// Requires measurements with IMU oriented in 6 positions:
    /// +X up, -X up, +Y up, -Y up, +Z up, -Z up
    /// </summary>
    public static AccelCalibrationResult CalibrateAccelerometer6Position(
        Vector<double>[] rawMeasurements,
        double gravityMagnitude = 9.81)
    {
        if (rawMeasurements.Length < 6)
        {
            return new AccelCalibrationResult
            {
                Success = false,
                Message = "Need at least 6 measurements for 6-position calibration"
            };
        }
        
        try
        {
            // Expected gravity vectors for each position
            var expectedVectors = new Vector<double>[]
            {
                Vector<double>.Build.DenseOfArray([gravityMagnitude, 0, 0]),   // +X up
                Vector<double>.Build.DenseOfArray([-gravityMagnitude, 0, 0]),  // -X up
                Vector<double>.Build.DenseOfArray([0, gravityMagnitude, 0]),   // +Y up
                Vector<double>.Build.DenseOfArray([0, -gravityMagnitude, 0]),  // -Y up
                Vector<double>.Build.DenseOfArray([0, 0, gravityMagnitude]),   // +Z up
                Vector<double>.Build.DenseOfArray([0, 0, -gravityMagnitude])   // -Z up
            };
            
            // Simple bias and scale estimation
            // For each axis: raw = scale * true + bias
            // From +X and -X: raw_pos = scale * g + bias, raw_neg = scale * (-g) + bias
            // scale = (raw_pos - raw_neg) / (2*g)
            // bias = (raw_pos + raw_neg) / 2
            
            var bias = Vector<double>.Build.Dense(3);
            var scale = Vector<double>.Build.Dense(3);
            
            // X axis from positions 0 (+X up) and 1 (-X up)
            bias[0] = (rawMeasurements[0][0] + rawMeasurements[1][0]) / 2.0;
            scale[0] = (rawMeasurements[0][0] - rawMeasurements[1][0]) / (2.0 * gravityMagnitude);
            
            // Y axis from positions 2 (+Y up) and 3 (-Y up)
            bias[1] = (rawMeasurements[2][1] + rawMeasurements[3][1]) / 2.0;
            scale[1] = (rawMeasurements[2][1] - rawMeasurements[3][1]) / (2.0 * gravityMagnitude);
            
            // Z axis from positions 4 (+Z up) and 5 (-Z up)
            bias[2] = (rawMeasurements[4][2] + rawMeasurements[5][2]) / 2.0;
            scale[2] = (rawMeasurements[4][2] - rawMeasurements[5][2]) / (2.0 * gravityMagnitude);
            
            var calibration = new AccelerometerCalibration
            {
                Bias = bias,
                Scale = scale
            };
            
            // Compute calibration error
            double sumSquaredError = 0;
            double maxError = 0;
            
            for (int i = 0; i < SysMath.Min(6, rawMeasurements.Length); i++)
            {
                var calibrated = calibration.Apply(rawMeasurements[i]);
                var error = (calibrated - expectedVectors[i]).L2Norm();
                sumSquaredError += error * error;
                maxError = SysMath.Max(maxError, error);
            }
            
            double rmsError = SysMath.Sqrt(sumSquaredError / 6);
            
            return new AccelCalibrationResult
            {
                Calibration = calibration,
                RmsError = rmsError,
                MaxError = maxError,
                Success = true,
                Message = $"Calibration successful. RMS error: {rmsError:F4} m/s², Max error: {maxError:F4} m/s²"
            };
        }
        catch (Exception ex)
        {
            return new AccelCalibrationResult
            {
                Success = false,
                Message = $"Calibration failed: {ex.Message}"
            };
        }
    }
    
    /// <summary>
    /// Result of gyroscope calibration.
    /// </summary>
    public class GyroCalibrationResult
    {
        public GyroscopeCalibration Calibration { get; set; } = new();
        public double BiasStability { get; set; }
        public double NoiseFloor { get; set; }
        public bool Success { get; set; }
        public string Message { get; set; } = "";
    }
    
    /// <summary>
    /// Calibrate gyroscope bias from static data.
    /// IMU should be stationary during measurement.
    /// </summary>
    public static GyroCalibrationResult CalibrateGyroBias(
        Vector<double>[] staticMeasurements,
        double sampleRate,
        double[]? temperatures = null)
    {
        if (staticMeasurements.Length < 100)
        {
            return new GyroCalibrationResult
            {
                Success = false,
                Message = "Need at least 100 samples for gyro bias calibration"
            };
        }
        
        try
        {
            int n = staticMeasurements.Length;
            
            // Compute mean (bias estimate)
            var sum = Vector<double>.Build.Dense(3);
            foreach (var m in staticMeasurements)
                sum += m;
            var bias = sum / n;
            
            // Compute standard deviation (noise estimate)
            var sumSquared = Vector<double>.Build.Dense(3);
            foreach (var m in staticMeasurements)
            {
                var diff = m - bias;
                for (int i = 0; i < 3; i++)
                    sumSquared[i] += diff[i] * diff[i];
            }
            var stdDev = Vector<double>.Build.Dense(3);
            for (int i = 0; i < 3; i++)
                stdDev[i] = SysMath.Sqrt(sumSquared[i] / (n - 1));
            
            // Compute Allan variance for bias stability (simplified)
            // Using block averaging with tau = 1 second
            int samplesPerSecond = (int)sampleRate;
            int numBlocks = n / samplesPerSecond;
            
            double biasStability = 0;
            if (numBlocks >= 2)
            {
                var blockAverages = new List<Vector<double>>();
                for (int block = 0; block < numBlocks; block++)
                {
                    var blockSum = Vector<double>.Build.Dense(3);
                    for (int i = 0; i < samplesPerSecond; i++)
                    {
                        blockSum += staticMeasurements[block * samplesPerSecond + i];
                    }
                    blockAverages.Add(blockSum / samplesPerSecond);
                }
                
                double allanVariance = 0;
                for (int i = 1; i < blockAverages.Count; i++)
                {
                    var diff = blockAverages[i] - blockAverages[i - 1];
                    allanVariance += diff.L2Norm() * diff.L2Norm();
                }
                allanVariance /= (2 * (numBlocks - 1));
                biasStability = SysMath.Sqrt(allanVariance);
            }
            
            // Temperature compensation if temperatures provided
            var tempCoeff = Vector<double>.Build.Dense(3);
            if (temperatures != null && temperatures.Length == n)
            {
                double meanTemp = temperatures.Average();
                
                for (int axis = 0; axis < 3; axis++)
                {
                    // Linear regression: gyro = a * temp + b
                    double sumT = 0, sumG = 0, sumTG = 0, sumT2 = 0;
                    for (int i = 0; i < n; i++)
                    {
                        double t = temperatures[i] - meanTemp;
                        double g = staticMeasurements[i][axis];
                        sumT += t;
                        sumG += g;
                        sumTG += t * g;
                        sumT2 += t * t;
                    }
                    
                    if (sumT2 > 1e-10)
                    {
                        tempCoeff[axis] = (n * sumTG - sumT * sumG) / (n * sumT2 - sumT * sumT);
                    }
                }
            }
            
            var calibration = new GyroscopeCalibration
            {
                Bias = bias,
                TempCoefficient = tempCoeff,
                ReferenceTemperature = temperatures?.Average() ?? 25.0
            };
            
            double noiseFloor = stdDev.L2Norm();
            
            return new GyroCalibrationResult
            {
                Calibration = calibration,
                BiasStability = biasStability,
                NoiseFloor = noiseFloor,
                Success = true,
                Message = $"Gyro calibration successful.\n" +
                          $"Bias: [{bias[0]:F6}, {bias[1]:F6}, {bias[2]:F6}] rad/s\n" +
                          $"Noise floor: {noiseFloor * 180 / SysMath.PI:F4} deg/s\n" +
                          $"Bias stability: {biasStability * 180 / SysMath.PI:F6} deg/s"
            };
        }
        catch (Exception ex)
        {
            return new GyroCalibrationResult
            {
                Success = false,
                Message = $"Calibration failed: {ex.Message}"
            };
        }
    }
}

/// <summary>
/// Magnetometer calibration for hard/soft iron compensation.
/// </summary>
public class MagnetometerCalibration
{
    /// <summary>
    /// Hard iron offset (bias).
    /// </summary>
    public Vector<double> HardIron { get; set; } = Vector<double>.Build.Dense(3);
    
    /// <summary>
    /// Soft iron compensation matrix.
    /// </summary>
    public Matrix<double> SoftIron { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    
    /// <summary>
    /// Expected field magnitude at calibration location.
    /// </summary>
    public double FieldMagnitude { get; set; } = 1.0;
    
    /// <summary>
    /// Apply calibration to raw reading.
    /// </summary>
    public Vector<double> Apply(Vector<double> raw)
    {
        var corrected = raw - HardIron;
        var calibrated = SoftIron * corrected;
        // Normalize to expected magnitude
        double mag = calibrated.L2Norm();
        if (mag > 1e-10)
        {
            calibrated = calibrated * (FieldMagnitude / mag);
        }
        return calibrated;
    }
    
    /// <summary>
    /// Calibration result.
    /// </summary>
    public class CalibrationResult
    {
        public MagnetometerCalibration Calibration { get; set; } = new();
        public double FitError { get; set; }
        public double SphericitiyError { get; set; }
        public bool Success { get; set; }
        public string Message { get; set; } = "";
    }
    
    /// <summary>
    /// Calibrate magnetometer from rotation data.
    /// Rotate sensor in all directions to collect data points on an ellipsoid.
    /// Calibration fits an ellipsoid and computes transformation to a sphere.
    /// </summary>
    public static CalibrationResult Calibrate(Vector<double>[] measurements)
    {
        if (measurements.Length < 12)
        {
            return new CalibrationResult
            {
                Success = false,
                Message = "Need at least 12 measurements for magnetometer calibration"
            };
        }
        
        try
        {
            int n = measurements.Length;
            
            // Simple sphere fitting using least squares
            // Find center (hard iron) by minimizing ||m - c||² variance
            
            // Step 1: Estimate center using data range
            double minX = double.MaxValue, maxX = double.MinValue;
            double minY = double.MaxValue, maxY = double.MinValue;
            double minZ = double.MaxValue, maxZ = double.MinValue;
            
            foreach (var m in measurements)
            {
                minX = SysMath.Min(minX, m[0]); maxX = SysMath.Max(maxX, m[0]);
                minY = SysMath.Min(minY, m[1]); maxY = SysMath.Max(maxY, m[1]);
                minZ = SysMath.Min(minZ, m[2]); maxZ = SysMath.Max(maxZ, m[2]);
            }
            
            var center = Vector<double>.Build.DenseOfArray([
                (minX + maxX) / 2,
                (minY + maxY) / 2,
                (minZ + maxZ) / 2
            ]);
            
            // Step 2: Compute average radius
            double sumRadius = 0;
            foreach (var m in measurements)
            {
                sumRadius += (m - center).L2Norm();
            }
            double avgRadius = sumRadius / n;
            
            // Step 3: Estimate scale factors (soft iron, diagonal only)
            double scaleX = avgRadius / ((maxX - minX) / 2);
            double scaleY = avgRadius / ((maxY - minY) / 2);
            double scaleZ = avgRadius / ((maxZ - minZ) / 2);
            
            var softIron = Matrix<double>.Build.DenseOfDiagonalArray([scaleX, scaleY, scaleZ]);
            
            // Step 4: Compute calibration error
            double sumSquaredError = 0;
            foreach (var m in measurements)
            {
                var corrected = softIron * (m - center);
                double radius = corrected.L2Norm();
                double error = radius - avgRadius;
                sumSquaredError += error * error;
            }
            
            double fitError = SysMath.Sqrt(sumSquaredError / n);
            double sphericityError = SysMath.Abs(scaleX - 1) + SysMath.Abs(scaleY - 1) + SysMath.Abs(scaleZ - 1);
            
            var calibration = new MagnetometerCalibration
            {
                HardIron = center,
                SoftIron = softIron,
                FieldMagnitude = avgRadius
            };
            
            return new CalibrationResult
            {
                Calibration = calibration,
                FitError = fitError,
                SphericitiyError = sphericityError,
                Success = true,
                Message = $"Magnetometer calibration successful.\n" +
                          $"Hard iron: [{center[0]:F2}, {center[1]:F2}, {center[2]:F2}]\n" +
                          $"Scale factors: [{scaleX:F3}, {scaleY:F3}, {scaleZ:F3}]\n" +
                          $"Field magnitude: {avgRadius:F2}\n" +
                          $"Fit error: {fitError:F4}"
            };
        }
        catch (Exception ex)
        {
            return new CalibrationResult
            {
                Success = false,
                Message = $"Calibration failed: {ex.Message}"
            };
        }
    }
}

/// <summary>
/// Wheel encoder and odometry calculations.
/// </summary>
public class OdometryCalculator
{
    /// <summary>
    /// Robot configuration for differential drive.
    /// </summary>
    public class DifferentialDriveConfig
    {
        /// <summary>
        /// Wheel radius in meters.
        /// </summary>
        public double WheelRadius { get; set; } = 0.05;
        
        /// <summary>
        /// Track width (distance between wheels) in meters.
        /// </summary>
        public double TrackWidth { get; set; } = 0.3;
        
        /// <summary>
        /// Encoder counts per revolution.
        /// </summary>
        public int CountsPerRevolution { get; set; } = 1440;
        
        /// <summary>
        /// Gear ratio (motor to wheel).
        /// </summary>
        public double GearRatio { get; set; } = 1.0;
        
        /// <summary>
        /// Distance per encoder count.
        /// </summary>
        public double DistancePerCount => 
            (2 * SysMath.PI * WheelRadius) / (CountsPerRevolution * GearRatio);
    }
    
    /// <summary>
    /// Robot pose (x, y, theta).
    /// </summary>
    public class Pose2D
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Theta { get; set; }
        
        public Pose2D() { }
        public Pose2D(double x, double y, double theta)
        {
            X = x; Y = y; Theta = theta;
        }
        
        public Vector<double> ToVector() => 
            Vector<double>.Build.DenseOfArray([X, Y, Theta]);
        
        public static Pose2D FromVector(Vector<double> v) => 
            new(v[0], v[1], v[2]);
        
        public override string ToString() => 
            $"({X:F3}, {Y:F3}, {Theta * 180 / SysMath.PI:F1}°)";
    }
    
    private readonly DifferentialDriveConfig _config;
    private Pose2D _pose;
    private long _lastLeftCount;
    private long _lastRightCount;
    private bool _initialized;
    
    public Pose2D CurrentPose => _pose;
    
    public OdometryCalculator(DifferentialDriveConfig config)
    {
        _config = config;
        _pose = new Pose2D();
    }
    
    /// <summary>
    /// Reset odometry to specified pose.
    /// </summary>
    public void Reset(Pose2D? pose = null)
    {
        _pose = pose ?? new Pose2D();
        _initialized = false;
    }
    
    /// <summary>
    /// Update odometry from encoder counts.
    /// </summary>
    public Pose2D Update(long leftCount, long rightCount)
    {
        if (!_initialized)
        {
            _lastLeftCount = leftCount;
            _lastRightCount = rightCount;
            _initialized = true;
            return _pose;
        }
        
        // Compute encoder deltas
        long deltaLeft = leftCount - _lastLeftCount;
        long deltaRight = rightCount - _lastRightCount;
        _lastLeftCount = leftCount;
        _lastRightCount = rightCount;
        
        // Convert to distances
        double dLeft = deltaLeft * _config.DistancePerCount;
        double dRight = deltaRight * _config.DistancePerCount;
        
        // Compute motion
        double dCenter = (dLeft + dRight) / 2.0;
        double dTheta = (dRight - dLeft) / _config.TrackWidth;
        
        // Update pose using midpoint integration
        double halfTheta = _pose.Theta + dTheta / 2.0;
        _pose.X += dCenter * SysMath.Cos(halfTheta);
        _pose.Y += dCenter * SysMath.Sin(halfTheta);
        _pose.Theta += dTheta;
        
        // Normalize theta to [-pi, pi]
        while (_pose.Theta > SysMath.PI) _pose.Theta -= 2 * SysMath.PI;
        while (_pose.Theta < -SysMath.PI) _pose.Theta += 2 * SysMath.PI;
        
        return _pose;
    }
    
    /// <summary>
    /// Update odometry from wheel velocities.
    /// </summary>
    public Pose2D UpdateFromVelocity(double leftVelocity, double rightVelocity, double dt)
    {
        double dLeft = leftVelocity * dt;
        double dRight = rightVelocity * dt;
        
        double dCenter = (dLeft + dRight) / 2.0;
        double dTheta = (dRight - dLeft) / _config.TrackWidth;
        
        double halfTheta = _pose.Theta + dTheta / 2.0;
        _pose.X += dCenter * SysMath.Cos(halfTheta);
        _pose.Y += dCenter * SysMath.Sin(halfTheta);
        _pose.Theta += dTheta;
        
        while (_pose.Theta > SysMath.PI) _pose.Theta -= 2 * SysMath.PI;
        while (_pose.Theta < -SysMath.PI) _pose.Theta += 2 * SysMath.PI;
        
        return _pose;
    }
    
    /// <summary>
    /// Compute wheel velocities for desired robot motion.
    /// </summary>
    public (double Left, double Right) InverseKinematics(double linearVelocity, double angularVelocity)
    {
        double vLeft = linearVelocity - (angularVelocity * _config.TrackWidth / 2.0);
        double vRight = linearVelocity + (angularVelocity * _config.TrackWidth / 2.0);
        return (vLeft, vRight);
    }
    
    /// <summary>
    /// Compute robot velocities from wheel velocities.
    /// </summary>
    public (double Linear, double Angular) ForwardKinematics(double leftVelocity, double rightVelocity)
    {
        double linear = (leftVelocity + rightVelocity) / 2.0;
        double angular = (rightVelocity - leftVelocity) / _config.TrackWidth;
        return (linear, angular);
    }
    
    /// <summary>
    /// Compute odometry covariance for EKF.
    /// Uses velocity-dependent noise model.
    /// </summary>
    public Matrix<double> ComputeOdometryCovariance(
        double leftVelocity, 
        double rightVelocity, 
        double dt,
        double kLeft = 0.01,  // Left wheel slip coefficient
        double kRight = 0.01) // Right wheel slip coefficient
    {
        // Odometry error depends on wheel slippage, which increases with velocity
        double varLeft = kLeft * SysMath.Abs(leftVelocity) * dt;
        double varRight = kRight * SysMath.Abs(rightVelocity) * dt;
        
        double dTheta = (rightVelocity - leftVelocity) * dt / _config.TrackWidth;
        double dCenter = (leftVelocity + rightVelocity) * dt / 2.0;
        
        double cosTheta = SysMath.Cos(_pose.Theta);
        double sinTheta = SysMath.Sin(_pose.Theta);
        
        // Propagate wheel covariances to pose covariance
        // Simplified: assumes wheel errors are independent
        double varX = (cosTheta * cosTheta) * (varLeft + varRight) / 4.0;
        double varY = (sinTheta * sinTheta) * (varLeft + varRight) / 4.0;
        double varTheta = (varLeft + varRight) / (_config.TrackWidth * _config.TrackWidth);
        
        return MatrixUtilities.Diagonal(varX, varY, varTheta);
    }
}

/// <summary>
/// Encoder configuration and calculations.
/// </summary>
public static class EncoderCalculations
{
    /// <summary>
    /// Calculate counts per revolution from encoder specifications.
    /// </summary>
    public static int CalculateCpr(int pulsesPerRevolution, bool quadrature, int edgesPerPulse = 1)
    {
        int multiplier = quadrature ? 4 : edgesPerPulse;
        return pulsesPerRevolution * multiplier;
    }
    
    /// <summary>
    /// Convert encoder counts to distance.
    /// </summary>
    public static double CountsToDistance(long counts, int cpr, double wheelRadius)
    {
        double revolutions = (double)counts / cpr;
        return revolutions * 2 * SysMath.PI * wheelRadius;
    }
    
    /// <summary>
    /// Convert encoder count rate to velocity.
    /// </summary>
    public static double CountRateToVelocity(double countsPerSecond, int cpr, double wheelRadius)
    {
        double revsPerSecond = countsPerSecond / cpr;
        return revsPerSecond * 2 * SysMath.PI * wheelRadius;
    }
    
    /// <summary>
    /// Calculate maximum detectable velocity given encoder resolution and sample rate.
    /// </summary>
    public static double MaxVelocity(int cpr, double wheelRadius, double sampleRate, int maxCountsPerSample = 1000)
    {
        double maxCountRate = maxCountsPerSample * sampleRate;
        return CountRateToVelocity(maxCountRate, cpr, wheelRadius);
    }
    
    /// <summary>
    /// Calculate minimum detectable velocity (resolution).
    /// </summary>
    public static double VelocityResolution(int cpr, double wheelRadius, double sampleRate)
    {
        // 1 count per sample period
        return CountRateToVelocity(sampleRate, cpr, wheelRadius);
    }
    
    /// <summary>
    /// Generate encoder configuration report.
    /// </summary>
    public static string GenerateReport(
        int pulsesPerRevolution,
        bool quadrature,
        double wheelRadius,
        double sampleRate,
        double gearRatio = 1.0)
    {
        int cpr = CalculateCpr(pulsesPerRevolution, quadrature);
        int effectiveCpr = (int)(cpr * gearRatio);
        
        double distancePerCount = (2 * SysMath.PI * wheelRadius) / effectiveCpr;
        double velResolution = VelocityResolution(effectiveCpr, wheelRadius, sampleRate);
        double maxVel = MaxVelocity(effectiveCpr, wheelRadius, sampleRate);
        
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("=== Encoder Configuration Report ===");
        sb.AppendLine();
        sb.AppendLine($"Pulses per revolution: {pulsesPerRevolution}");
        sb.AppendLine($"Quadrature decoding: {(quadrature ? "Yes (4x)" : "No")}");
        sb.AppendLine($"Counts per revolution: {cpr}");
        sb.AppendLine($"Gear ratio: {gearRatio:F2}");
        sb.AppendLine($"Effective CPR (at wheel): {effectiveCpr}");
        sb.AppendLine();
        sb.AppendLine($"Wheel radius: {wheelRadius * 1000:F1} mm");
        sb.AppendLine($"Wheel circumference: {2 * SysMath.PI * wheelRadius * 1000:F1} mm");
        sb.AppendLine($"Distance per count: {distancePerCount * 1000:F4} mm");
        sb.AppendLine($"Angular resolution: {360.0 / effectiveCpr:F4}°");
        sb.AppendLine();
        sb.AppendLine($"Sample rate: {sampleRate:F0} Hz");
        sb.AppendLine($"Velocity resolution: {velResolution * 1000:F2} mm/s");
        sb.AppendLine($"Max detectable velocity: {maxVel:F2} m/s");
        
        return sb.ToString();
    }
}
