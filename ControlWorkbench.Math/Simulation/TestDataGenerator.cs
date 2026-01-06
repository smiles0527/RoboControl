using MathNet.Numerics.LinearAlgebra;
using SysMath = System.Math;

namespace ControlWorkbench.Math.Simulation;

/// <summary>
/// Generates realistic test data for sensor and control system testing.
/// </summary>
public static class TestDataGenerator
{
    private static readonly Random _random = new();
    
    /// <summary>
    /// Generate IMU data for a stationary sensor.
    /// </summary>
    public static ImuDataSet GenerateStationaryImu(
        ImuNoiseParameters noise,
        double duration,
        double sampleRate,
        Vector<double>? gravityVector = null)
    {
        int n = (int)(duration * sampleRate);
        var dataset = new ImuDataSet(n);
        
        gravityVector ??= Vector<double>.Build.DenseOfArray([0, 0, 9.81]);
        double dt = 1.0 / sampleRate;
        
        for (int i = 0; i < n; i++)
        {
            dataset.Time[i] = i * dt;
            
            // Accelerometer: gravity + noise + bias
            dataset.AccelX[i] = gravityVector[0] + noise.AccelBias[0] + 
                               GaussianNoise(noise.AccelNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.AccelY[i] = gravityVector[1] + noise.AccelBias[1] + 
                               GaussianNoise(noise.AccelNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.AccelZ[i] = gravityVector[2] + noise.AccelBias[2] + 
                               GaussianNoise(noise.AccelNoiseDensity * SysMath.Sqrt(sampleRate));
            
            // Gyroscope: bias + noise (no rotation when stationary)
            dataset.GyroX[i] = noise.GyroBias[0] + 
                              GaussianNoise(noise.GyroNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.GyroY[i] = noise.GyroBias[1] + 
                              GaussianNoise(noise.GyroNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.GyroZ[i] = noise.GyroBias[2] + 
                              GaussianNoise(noise.GyroNoiseDensity * SysMath.Sqrt(sampleRate));
            
            // Temperature with slow drift
            dataset.Temperature[i] = 25.0 + 5.0 * SysMath.Sin(2 * SysMath.PI * i / (n * 2));
        }
        
        return dataset;
    }
    
    /// <summary>
    /// Generate IMU data for a rotating sensor.
    /// </summary>
    public static ImuDataSet GenerateRotatingImu(
        ImuNoiseParameters noise,
        double duration,
        double sampleRate,
        RotationProfile profile)
    {
        int n = (int)(duration * sampleRate);
        var dataset = new ImuDataSet(n);
        
        double dt = 1.0 / sampleRate;
        double roll = 0, pitch = 0, yaw = 0;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            dataset.Time[i] = t;
            
            // Get angular velocities from profile
            var (wx, wy, wz) = profile.GetAngularVelocity(t);
            
            // Integrate to get attitude
            roll += wx * dt;
            pitch += wy * dt;
            yaw += wz * dt;
            
            // Compute gravity in body frame
            var Rx = RotationMatrix('x', roll);
            var Ry = RotationMatrix('y', pitch);
            var Rz = RotationMatrix('z', yaw);
            var R = Rz * Ry * Rx;
            var gravityBody = R.Transpose() * Vector<double>.Build.DenseOfArray([0, 0, 9.81]);
            
            // Accelerometer: rotated gravity + noise + bias
            dataset.AccelX[i] = gravityBody[0] + noise.AccelBias[0] + 
                               GaussianNoise(noise.AccelNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.AccelY[i] = gravityBody[1] + noise.AccelBias[1] + 
                               GaussianNoise(noise.AccelNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.AccelZ[i] = gravityBody[2] + noise.AccelBias[2] + 
                               GaussianNoise(noise.AccelNoiseDensity * SysMath.Sqrt(sampleRate));
            
            // Gyroscope: true rate + bias + noise
            dataset.GyroX[i] = wx + noise.GyroBias[0] + 
                              GaussianNoise(noise.GyroNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.GyroY[i] = wy + noise.GyroBias[1] + 
                              GaussianNoise(noise.GyroNoiseDensity * SysMath.Sqrt(sampleRate));
            dataset.GyroZ[i] = wz + noise.GyroBias[2] + 
                              GaussianNoise(noise.GyroNoiseDensity * SysMath.Sqrt(sampleRate));
            
            // True attitude for validation
            dataset.TrueRoll[i] = roll;
            dataset.TruePitch[i] = pitch;
            dataset.TrueYaw[i] = yaw;
        }
        
        return dataset;
    }
    
    /// <summary>
    /// Generate magnetometer calibration data (rotation around all axes).
    /// </summary>
    public static MagDataSet GenerateMagnetometerCalibrationData(
        MagCalibrationParams calib,
        int numSamples = 1000)
    {
        var dataset = new MagDataSet(numSamples);
        
        // Field vector in NED frame
        var fieldNed = Vector<double>.Build.DenseOfArray([
            calib.FieldMagnitude * SysMath.Cos(calib.InclinationRad),
            0,
            calib.FieldMagnitude * SysMath.Sin(calib.InclinationRad)
        ]);
        
        for (int i = 0; i < numSamples; i++)
        {
            // Random orientation
            double roll = _random.NextDouble() * 2 * SysMath.PI;
            double pitch = (_random.NextDouble() - 0.5) * SysMath.PI;
            double yaw = _random.NextDouble() * 2 * SysMath.PI;
            
            var Rx = RotationMatrix('x', roll);
            var Ry = RotationMatrix('y', pitch);
            var Rz = RotationMatrix('z', yaw);
            var R = Rz * Ry * Rx;
            
            // True field in body frame
            var fieldBody = R.Transpose() * fieldNed;
            
            // Apply soft iron distortion
            var distorted = calib.SoftIron * fieldBody;
            
            // Add hard iron offset and noise
            dataset.MagX[i] = distorted[0] + calib.HardIron[0] + GaussianNoise(calib.NoiseStdDev);
            dataset.MagY[i] = distorted[1] + calib.HardIron[1] + GaussianNoise(calib.NoiseStdDev);
            dataset.MagZ[i] = distorted[2] + calib.HardIron[2] + GaussianNoise(calib.NoiseStdDev);
            
            dataset.TrueRoll[i] = roll;
            dataset.TruePitch[i] = pitch;
            dataset.TrueYaw[i] = yaw;
        }
        
        return dataset;
    }
    
    /// <summary>
    /// Generate GPS trajectory data.
    /// </summary>
    public static GpsDataSet GenerateGpsTrajectory(
        GpsNoiseParameters noise,
        TrajectoryProfile trajectory,
        double duration,
        double sampleRate)
    {
        int n = (int)(duration * sampleRate);
        var dataset = new GpsDataSet(n);
        double dt = 1.0 / sampleRate;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            dataset.Time[i] = t;
            
            var (x, y, z) = trajectory.GetPosition(t);
            var (vx, vy, vz) = trajectory.GetVelocity(t);
            
            // Add noise
            dataset.X[i] = x + GaussianNoise(noise.HorizontalStdDev);
            dataset.Y[i] = y + GaussianNoise(noise.HorizontalStdDev);
            dataset.Z[i] = z + GaussianNoise(noise.VerticalStdDev);
            
            dataset.VelX[i] = vx + GaussianNoise(noise.VelocityStdDev);
            dataset.VelY[i] = vy + GaussianNoise(noise.VelocityStdDev);
            dataset.VelZ[i] = vz + GaussianNoise(noise.VelocityStdDev);
            
            // True values
            dataset.TrueX[i] = x;
            dataset.TrueY[i] = y;
            dataset.TrueZ[i] = z;
            
            dataset.NumSatellites[i] = noise.MinSatellites + 
                _random.Next(noise.MaxSatellites - noise.MinSatellites + 1);
            dataset.Hdop[i] = noise.BaseHdop + (_random.NextDouble() - 0.5) * 0.5;
        }
        
        return dataset;
    }
    
    /// <summary>
    /// Generate encoder data for a differential drive robot.
    /// </summary>
    public static EncoderDataSet GenerateEncoderData(
        EncoderParameters encoderParams,
        TrajectoryProfile trajectory,
        double duration,
        double sampleRate)
    {
        int n = (int)(duration * sampleRate);
        var dataset = new EncoderDataSet(n);
        double dt = 1.0 / sampleRate;
        
        long leftCount = 0;
        long rightCount = 0;
        double leftRemainder = 0;
        double rightRemainder = 0;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            dataset.Time[i] = t;
            
            var (vx, vy, vz) = trajectory.GetVelocity(t);
            double linearVel = SysMath.Sqrt(vx * vx + vy * vy);
            
            // Get heading change rate
            double angularVel = 0;
            if (i > 0)
            {
                var (x0, y0, _) = trajectory.GetPosition((i - 1) * dt);
                var (x1, y1, _) = trajectory.GetPosition(t);
                var (x2, y2, _) = trajectory.GetPosition(t + dt);
                
                double heading1 = SysMath.Atan2(y1 - y0, x1 - x0);
                double heading2 = SysMath.Atan2(y2 - y1, x2 - x1);
                angularVel = (heading2 - heading1) / dt;
            }
            
            // Compute wheel velocities
            double vLeft = linearVel - angularVel * encoderParams.TrackWidth / 2;
            double vRight = linearVel + angularVel * encoderParams.TrackWidth / 2;
            
            // Add slip noise
            vLeft *= (1 + GaussianNoise(encoderParams.SlipNoise));
            vRight *= (1 + GaussianNoise(encoderParams.SlipNoise));
            
            // Convert to encoder counts
            double leftDist = vLeft * dt;
            double rightDist = vRight * dt;
            
            double leftCounts = leftDist / encoderParams.DistancePerCount + leftRemainder;
            double rightCounts = rightDist / encoderParams.DistancePerCount + rightRemainder;
            
            long leftDelta = (long)SysMath.Round(leftCounts);
            long rightDelta = (long)SysMath.Round(rightCounts);
            
            leftRemainder = leftCounts - leftDelta;
            rightRemainder = rightCounts - rightDelta;
            
            leftCount += leftDelta;
            rightCount += rightDelta;
            
            dataset.LeftCount[i] = leftCount;
            dataset.RightCount[i] = rightCount;
            
            var (px, py, _) = trajectory.GetPosition(t);
            dataset.TrueX[i] = px;
            dataset.TrueY[i] = py;
        }
        
        return dataset;
    }
    
    /// <summary>
    /// Generate step response data for a system.
    /// </summary>
    public static StepResponseData GenerateStepResponse(
        SystemType systemType,
        SystemParameters parameters,
        double stepMagnitude,
        double duration,
        double sampleRate,
        double noiseLevel = 0)
    {
        int n = (int)(duration * sampleRate);
        var data = new StepResponseData(n);
        double dt = 1.0 / sampleRate;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            data.Time[i] = t;
            data.Input[i] = t >= 0 ? stepMagnitude : 0;
            
            double response = systemType switch
            {
                SystemType.FirstOrder => FirstOrderResponse(
                    t, stepMagnitude, parameters.Gain, parameters.TimeConstant),
                SystemType.SecondOrder => SecondOrderResponse(
                    t, stepMagnitude, parameters.Gain, parameters.NaturalFrequency, parameters.DampingRatio),
                SystemType.Integrator => IntegratorResponse(t, stepMagnitude, parameters.Gain),
                SystemType.Delay => DelayedFirstOrderResponse(
                    t, stepMagnitude, parameters.Gain, parameters.TimeConstant, parameters.Delay),
                _ => 0
            };
            
            data.Output[i] = response + GaussianNoise(noiseLevel);
            data.TrueOutput[i] = response;
        }
        
        return data;
    }
    
    /// <summary>
    /// Generate sinusoidal test signal.
    /// </summary>
    public static SignalData GenerateSineWave(
        double frequency,
        double amplitude,
        double duration,
        double sampleRate,
        double phaseOffset = 0,
        double dcOffset = 0,
        double noiseLevel = 0)
    {
        int n = (int)(duration * sampleRate);
        var data = new SignalData(n);
        double dt = 1.0 / sampleRate;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            data.Time[i] = t;
            data.Signal[i] = dcOffset + amplitude * SysMath.Sin(2 * SysMath.PI * frequency * t + phaseOffset)
                            + GaussianNoise(noiseLevel);
        }
        
        return data;
    }
    
    /// <summary>
    /// Generate chirp (frequency sweep) signal.
    /// </summary>
    public static SignalData GenerateChirp(
        double startFreq,
        double endFreq,
        double amplitude,
        double duration,
        double sampleRate,
        double noiseLevel = 0)
    {
        int n = (int)(duration * sampleRate);
        var data = new SignalData(n);
        double dt = 1.0 / sampleRate;
        
        double k = (endFreq - startFreq) / duration;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            data.Time[i] = t;
            double instantFreq = startFreq + k * t;
            double phase = 2 * SysMath.PI * (startFreq * t + 0.5 * k * t * t);
            data.Signal[i] = amplitude * SysMath.Sin(phase) + GaussianNoise(noiseLevel);
        }
        
        return data;
    }
    
    /// <summary>
    /// Generate PRBS (Pseudo-Random Binary Sequence) for system identification.
    /// </summary>
    public static SignalData GeneratePrbs(
        double amplitude,
        int registerLength,
        double switchPeriod,
        double duration,
        double sampleRate)
    {
        int n = (int)(duration * sampleRate);
        var data = new SignalData(n);
        double dt = 1.0 / sampleRate;
        
        // LFSR for PRBS generation
        int register = 1;
        int feedback;
        double switchTime = 0;
        bool currentBit = true;
        
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            data.Time[i] = t;
            
            if (t >= switchTime)
            {
                // Generate next bit using LFSR
                feedback = ((register >> 0) ^ (register >> 2)) & 1;
                register = (register >> 1) | (feedback << (registerLength - 1));
                currentBit = (register & 1) == 1;
                switchTime += switchPeriod;
            }
            
            data.Signal[i] = currentBit ? amplitude : -amplitude;
        }
        
        return data;
    }
    
    // Helper methods
    
    private static double GaussianNoise(double stdDev)
    {
        // Box-Muller transform
        double u1 = 1.0 - _random.NextDouble();
        double u2 = 1.0 - _random.NextDouble();
        double randStdNormal = SysMath.Sqrt(-2.0 * SysMath.Log(u1)) * SysMath.Sin(2.0 * SysMath.PI * u2);
        return stdDev * randStdNormal;
    }
    
    private static Matrix<double> RotationMatrix(char axis, double angle)
    {
        double c = SysMath.Cos(angle);
        double s = SysMath.Sin(angle);
        
        return axis switch
        {
            'x' => Matrix<double>.Build.DenseOfArray(new double[,] {
                { 1, 0, 0 },
                { 0, c, -s },
                { 0, s, c }
            }),
            'y' => Matrix<double>.Build.DenseOfArray(new double[,] {
                { c, 0, s },
                { 0, 1, 0 },
                { -s, 0, c }
            }),
            'z' => Matrix<double>.Build.DenseOfArray(new double[,] {
                { c, -s, 0 },
                { s, c, 0 },
                { 0, 0, 1 }
            }),
            _ => Matrix<double>.Build.DenseIdentity(3)
        };
    }
    
    private static double FirstOrderResponse(double t, double step, double K, double tau)
    {
        return K * step * (1 - SysMath.Exp(-t / tau));
    }
    
    private static double SecondOrderResponse(double t, double step, double K, double wn, double zeta)
    {
        if (zeta < 1)
        {
            double wd = wn * SysMath.Sqrt(1 - zeta * zeta);
            return K * step * (1 - SysMath.Exp(-zeta * wn * t) *
                (SysMath.Cos(wd * t) + zeta / SysMath.Sqrt(1 - zeta * zeta) * SysMath.Sin(wd * t)));
        }
        else if (zeta == 1)
        {
            return K * step * (1 - (1 + wn * t) * SysMath.Exp(-wn * t));
        }
        else
        {
            double s1 = -wn * (zeta - SysMath.Sqrt(zeta * zeta - 1));
            double s2 = -wn * (zeta + SysMath.Sqrt(zeta * zeta - 1));
            return K * step * (1 - (s2 * SysMath.Exp(s1 * t) - s1 * SysMath.Exp(s2 * t)) / (s2 - s1));
        }
    }
    
    private static double IntegratorResponse(double t, double step, double K)
    {
        return K * step * t;
    }
    
    private static double DelayedFirstOrderResponse(double t, double step, double K, double tau, double delay)
    {
        if (t < delay)
            return 0;
        return FirstOrderResponse(t - delay, step, K, tau);
    }
}

// Data set classes

public class ImuDataSet
{
    public double[] Time { get; }
    public double[] AccelX { get; }
    public double[] AccelY { get; }
    public double[] AccelZ { get; }
    public double[] GyroX { get; }
    public double[] GyroY { get; }
    public double[] GyroZ { get; }
    public double[] Temperature { get; }
    public double[] TrueRoll { get; }
    public double[] TruePitch { get; }
    public double[] TrueYaw { get; }
    
    public ImuDataSet(int n)
    {
        Time = new double[n];
        AccelX = new double[n];
        AccelY = new double[n];
        AccelZ = new double[n];
        GyroX = new double[n];
        GyroY = new double[n];
        GyroZ = new double[n];
        Temperature = new double[n];
        TrueRoll = new double[n];
        TruePitch = new double[n];
        TrueYaw = new double[n];
    }
    
    public int Length => Time.Length;
    
    public Vector<double> GetAccel(int i) => 
        Vector<double>.Build.DenseOfArray([AccelX[i], AccelY[i], AccelZ[i]]);
    
    public Vector<double> GetGyro(int i) => 
        Vector<double>.Build.DenseOfArray([GyroX[i], GyroY[i], GyroZ[i]]);
}

public class MagDataSet
{
    public double[] MagX { get; }
    public double[] MagY { get; }
    public double[] MagZ { get; }
    public double[] TrueRoll { get; }
    public double[] TruePitch { get; }
    public double[] TrueYaw { get; }
    
    public MagDataSet(int n)
    {
        MagX = new double[n];
        MagY = new double[n];
        MagZ = new double[n];
        TrueRoll = new double[n];
        TruePitch = new double[n];
        TrueYaw = new double[n];
    }
    
    public int Length => MagX.Length;
    
    public Vector<double> GetMag(int i) => 
        Vector<double>.Build.DenseOfArray([MagX[i], MagY[i], MagZ[i]]);
    
    public Vector<double>[] GetAllMeasurements()
    {
        var result = new Vector<double>[Length];
        for (int i = 0; i < Length; i++)
            result[i] = GetMag(i);
        return result;
    }
}

public class GpsDataSet
{
    public double[] Time { get; }
    public double[] X { get; }
    public double[] Y { get; }
    public double[] Z { get; }
    public double[] VelX { get; }
    public double[] VelY { get; }
    public double[] VelZ { get; }
    public double[] TrueX { get; }
    public double[] TrueY { get; }
    public double[] TrueZ { get; }
    public int[] NumSatellites { get; }
    public double[] Hdop { get; }
    
    public GpsDataSet(int n)
    {
        Time = new double[n];
        X = new double[n];
        Y = new double[n];
        Z = new double[n];
        VelX = new double[n];
        VelY = new double[n];
        VelZ = new double[n];
        TrueX = new double[n];
        TrueY = new double[n];
        TrueZ = new double[n];
        NumSatellites = new int[n];
        Hdop = new double[n];
    }
    
    public int Length => Time.Length;
}

public class EncoderDataSet
{
    public double[] Time { get; }
    public long[] LeftCount { get; }
    public long[] RightCount { get; }
    public double[] TrueX { get; }
    public double[] TrueY { get; }
    
    public EncoderDataSet(int n)
    {
        Time = new double[n];
        LeftCount = new long[n];
        RightCount = new long[n];
        TrueX = new double[n];
        TrueY = new double[n];
    }
    
    public int Length => Time.Length;
}

public class StepResponseData
{
    public double[] Time { get; }
    public double[] Input { get; }
    public double[] Output { get; }
    public double[] TrueOutput { get; }
    
    public StepResponseData(int n)
    {
        Time = new double[n];
        Input = new double[n];
        Output = new double[n];
        TrueOutput = new double[n];
    }
    
    public int Length => Time.Length;
}

public class SignalData
{
    public double[] Time { get; }
    public double[] Signal { get; }
    
    public SignalData(int n)
    {
        Time = new double[n];
        Signal = new double[n];
    }
    
    public int Length => Time.Length;
}

// Parameter classes

public class ImuNoiseParameters
{
    public double AccelNoiseDensity { get; set; } = 0.003; // m/s²/?Hz
    public double GyroNoiseDensity { get; set; } = 0.0003; // rad/s/?Hz
    public Vector<double> AccelBias { get; set; } = Vector<double>.Build.DenseOfArray([0.02, -0.01, 0.03]);
    public Vector<double> GyroBias { get; set; } = Vector<double>.Build.DenseOfArray([0.001, -0.0005, 0.0008]);
    
    public static ImuNoiseParameters LowCostMems => new()
    {
        AccelNoiseDensity = 0.01,
        GyroNoiseDensity = 0.001,
        AccelBias = Vector<double>.Build.DenseOfArray([0.1, -0.05, 0.08]),
        GyroBias = Vector<double>.Build.DenseOfArray([0.01, -0.005, 0.008])
    };
    
    public static ImuNoiseParameters TacticalGrade => new()
    {
        AccelNoiseDensity = 0.001,
        GyroNoiseDensity = 0.0001,
        AccelBias = Vector<double>.Build.DenseOfArray([0.005, -0.003, 0.004]),
        GyroBias = Vector<double>.Build.DenseOfArray([0.0001, -0.00005, 0.00008])
    };
}

public class MagCalibrationParams
{
    public double FieldMagnitude { get; set; } = 50.0; // µT
    public double InclinationRad { get; set; } = 60.0 * SysMath.PI / 180;
    public Vector<double> HardIron { get; set; } = Vector<double>.Build.DenseOfArray([10, -15, 5]);
    public Matrix<double> SoftIron { get; set; } = Matrix<double>.Build.DenseOfDiagonalArray([1.1, 0.95, 1.02]);
    public double NoiseStdDev { get; set; } = 0.5;
}

public class GpsNoiseParameters
{
    public double HorizontalStdDev { get; set; } = 2.5; // meters
    public double VerticalStdDev { get; set; } = 5.0; // meters
    public double VelocityStdDev { get; set; } = 0.1; // m/s
    public int MinSatellites { get; set; } = 6;
    public int MaxSatellites { get; set; } = 12;
    public double BaseHdop { get; set; } = 1.2;
}

public class EncoderParameters
{
    public double WheelRadius { get; set; } = 0.05;
    public double TrackWidth { get; set; } = 0.3;
    public int CountsPerRevolution { get; set; } = 1440;
    public double GearRatio { get; set; } = 1.0;
    public double SlipNoise { get; set; } = 0.01;
    
    public double DistancePerCount => 
        (2 * SysMath.PI * WheelRadius) / (CountsPerRevolution * GearRatio);
}

public class SystemParameters
{
    public double Gain { get; set; } = 1.0;
    public double TimeConstant { get; set; } = 1.0;
    public double NaturalFrequency { get; set; } = 1.0;
    public double DampingRatio { get; set; } = 0.7;
    public double Delay { get; set; } = 0;
}

public enum SystemType
{
    FirstOrder,
    SecondOrder,
    Integrator,
    Delay
}

// Profile interfaces

public interface RotationProfile
{
    (double Wx, double Wy, double Wz) GetAngularVelocity(double t);
}

public class SinusoidalRotation : RotationProfile
{
    public double Frequency { get; set; } = 0.5;
    public double AmplitudeX { get; set; } = 0.5;
    public double AmplitudeY { get; set; } = 0.3;
    public double AmplitudeZ { get; set; } = 0.8;
    
    public (double Wx, double Wy, double Wz) GetAngularVelocity(double t)
    {
        double phase = 2 * SysMath.PI * Frequency * t;
        return (
            AmplitudeX * SysMath.Sin(phase),
            AmplitudeY * SysMath.Sin(phase * 1.3),
            AmplitudeZ * SysMath.Sin(phase * 0.7)
        );
    }
}

public interface TrajectoryProfile
{
    (double X, double Y, double Z) GetPosition(double t);
    (double Vx, double Vy, double Vz) GetVelocity(double t);
}

public class CircularTrajectory : TrajectoryProfile
{
    public double Radius { get; set; } = 10.0;
    public double Speed { get; set; } = 2.0;
    public double Height { get; set; } = 0;
    
    public (double X, double Y, double Z) GetPosition(double t)
    {
        double omega = Speed / Radius;
        return (
            Radius * SysMath.Cos(omega * t),
            Radius * SysMath.Sin(omega * t),
            Height
        );
    }
    
    public (double Vx, double Vy, double Vz) GetVelocity(double t)
    {
        double omega = Speed / Radius;
        return (
            -Speed * SysMath.Sin(omega * t),
            Speed * SysMath.Cos(omega * t),
            0
        );
    }
}

public class FigureEightTrajectory : TrajectoryProfile
{
    public double Size { get; set; } = 10.0;
    public double Period { get; set; } = 20.0;
    
    public (double X, double Y, double Z) GetPosition(double t)
    {
        double phase = 2 * SysMath.PI * t / Period;
        return (
            Size * SysMath.Sin(phase),
            Size * SysMath.Sin(phase) * SysMath.Cos(phase),
            0
        );
    }
    
    public (double Vx, double Vy, double Vz) GetVelocity(double t)
    {
        double phase = 2 * SysMath.PI * t / Period;
        double omega = 2 * SysMath.PI / Period;
        return (
            Size * omega * SysMath.Cos(phase),
            Size * omega * (SysMath.Cos(phase) * SysMath.Cos(phase) - SysMath.Sin(phase) * SysMath.Sin(phase)),
            0
        );
    }
}

public class LinearTrajectory : TrajectoryProfile
{
    public double Speed { get; set; } = 1.0;
    public double Heading { get; set; } = 0; // radians
    
    public (double X, double Y, double Z) GetPosition(double t)
    {
        return (
            Speed * t * SysMath.Cos(Heading),
            Speed * t * SysMath.Sin(Heading),
            0
        );
    }
    
    public (double Vx, double Vy, double Vz) GetVelocity(double t)
    {
        return (
            Speed * SysMath.Cos(Heading),
            Speed * SysMath.Sin(Heading),
            0
        );
    }
}
