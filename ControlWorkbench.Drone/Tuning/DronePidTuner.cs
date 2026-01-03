using ControlWorkbench.Drone.Devices;

namespace ControlWorkbench.Drone.Tuning;

/// <summary>
/// Drone-specific PID tuning tools for rate and attitude controllers.
/// </summary>
public class DronePidTuner
{
    /// <summary>
    /// Standard ArduCopter/Betaflight PID structure.
    /// </summary>
    public class PidGains
    {
        public double P { get; set; }
        public double I { get; set; }
        public double D { get; set; }
        public double FF { get; set; }      // Feedforward
        public double Imax { get; set; }    // Integral limit
        public double Dmax { get; set; }    // D-term filter
        
        public override string ToString() => $"P={P:F3} I={I:F4} D={D:F4} FF={FF:F4}";
    }
    
    /// <summary>
    /// Complete PID set for a drone.
    /// </summary>
    public class DroneGains
    {
        // Rate controllers (inner loop)
        public PidGains RollRate { get; set; } = new() { P = 0.135, I = 0.135, D = 0.003, Imax = 0.5 };
        public PidGains PitchRate { get; set; } = new() { P = 0.135, I = 0.135, D = 0.003, Imax = 0.5 };
        public PidGains YawRate { get; set; } = new() { P = 0.18, I = 0.018, D = 0, Imax = 0.5 };
        
        // Attitude controllers (outer loop)
        public double RollAttP { get; set; } = 4.5;
        public double PitchAttP { get; set; } = 4.5;
        public double YawAttP { get; set; } = 4.5;
        
        // Position controllers
        public PidGains PosXY { get; set; } = new() { P = 1.0, I = 0.5, D = 0 };
        public PidGains PosZ { get; set; } = new() { P = 1.0, I = 2.0, D = 0 };
        public PidGains VelXY { get; set; } = new() { P = 2.0, I = 1.0, D = 0 };
        public PidGains VelZ { get; set; } = new() { P = 5.0, I = 1.0, D = 0 };
        
        // Filters
        public double GyroLpfHz { get; set; } = 80;
        public double DtermLpfHz { get; set; } = 40;
        public double NotchFilterHz { get; set; } = 0;  // 0 = disabled
    }
    
    /// <summary>
    /// Initial gain estimation based on drone specs.
    /// </summary>
    public static DroneGains EstimateInitialGains(DroneSpecs specs)
    {
        var gains = new DroneGains();
        
        // Scaling based on drone size/weight
        double thrustToWeight = specs.ThrustToWeightRatio;
        double sizeFactor = specs.ArmLengthMm / 250.0; // Normalized to 250mm arm
        
        // Higher T/W ratio = more aggressive gains possible
        double tFactor = System.Math.Sqrt(thrustToWeight / 2.0);
        
        // Larger drones need lower gains
        double sFactor = 1.0 / System.Math.Sqrt(sizeFactor);
        
        // Rate gains
        double rateP = 0.135 * tFactor * sFactor;
        gains.RollRate = new PidGains { P = rateP, I = rateP, D = rateP * 0.02, Imax = 0.5 };
        gains.PitchRate = new PidGains { P = rateP, I = rateP, D = rateP * 0.02, Imax = 0.5 };
        gains.YawRate = new PidGains { P = rateP * 1.5, I = rateP * 0.15, D = 0, Imax = 0.5 };
        
        // Attitude gains
        gains.RollAttP = 4.5 * tFactor;
        gains.PitchAttP = 4.5 * tFactor;
        gains.YawAttP = 4.5;
        
        // Filter frequencies - lower for larger drones
        gains.GyroLpfHz = 80 / sizeFactor;
        gains.DtermLpfHz = 40 / sizeFactor;
        
        return gains;
    }
    
    /// <summary>
    /// Betaflight-style CLI commands for gains.
    /// </summary>
    public static string ToBetaflightCli(DroneGains gains)
    {
        var sb = new System.Text.StringBuilder();
        
        // Roll
        sb.AppendLine($"set p_roll = {(int)(gains.RollRate.P * 1000)}");
        sb.AppendLine($"set i_roll = {(int)(gains.RollRate.I * 1000)}");
        sb.AppendLine($"set d_roll = {(int)(gains.RollRate.D * 10000)}");
        sb.AppendLine($"set f_roll = {(int)(gains.RollRate.FF * 1000)}");
        
        // Pitch
        sb.AppendLine($"set p_pitch = {(int)(gains.PitchRate.P * 1000)}");
        sb.AppendLine($"set i_pitch = {(int)(gains.PitchRate.I * 1000)}");
        sb.AppendLine($"set d_pitch = {(int)(gains.PitchRate.D * 10000)}");
        sb.AppendLine($"set f_pitch = {(int)(gains.PitchRate.FF * 1000)}");
        
        // Yaw
        sb.AppendLine($"set p_yaw = {(int)(gains.YawRate.P * 1000)}");
        sb.AppendLine($"set i_yaw = {(int)(gains.YawRate.I * 1000)}");
        sb.AppendLine($"set d_yaw = {(int)(gains.YawRate.D * 10000)}");
        sb.AppendLine($"set f_yaw = {(int)(gains.YawRate.FF * 1000)}");
        
        // Filters
        sb.AppendLine($"set gyro_lowpass_hz = {(int)gains.GyroLpfHz}");
        sb.AppendLine($"set dterm_lowpass_hz = {(int)gains.DtermLpfHz}");
        
        sb.AppendLine("save");
        
        return sb.ToString();
    }
    
    /// <summary>
    /// ArduCopter parameter file format.
    /// </summary>
    public static string ToArduCopterParams(DroneGains gains)
    {
        var sb = new System.Text.StringBuilder();
        
        sb.AppendLine("# ArduCopter PID parameters");
        sb.AppendLine("# Generated by ControlWorkbench");
        sb.AppendLine();
        
        // Rate PIDs
        sb.AppendLine($"ATC_RAT_RLL_P,{gains.RollRate.P:F4}");
        sb.AppendLine($"ATC_RAT_RLL_I,{gains.RollRate.I:F4}");
        sb.AppendLine($"ATC_RAT_RLL_D,{gains.RollRate.D:F5}");
        sb.AppendLine($"ATC_RAT_RLL_FF,{gains.RollRate.FF:F4}");
        sb.AppendLine($"ATC_RAT_RLL_IMAX,{gains.RollRate.Imax:F2}");
        sb.AppendLine();
        
        sb.AppendLine($"ATC_RAT_PIT_P,{gains.PitchRate.P:F4}");
        sb.AppendLine($"ATC_RAT_PIT_I,{gains.PitchRate.I:F4}");
        sb.AppendLine($"ATC_RAT_PIT_D,{gains.PitchRate.D:F5}");
        sb.AppendLine($"ATC_RAT_PIT_FF,{gains.PitchRate.FF:F4}");
        sb.AppendLine($"ATC_RAT_PIT_IMAX,{gains.PitchRate.Imax:F2}");
        sb.AppendLine();
        
        sb.AppendLine($"ATC_RAT_YAW_P,{gains.YawRate.P:F4}");
        sb.AppendLine($"ATC_RAT_YAW_I,{gains.YawRate.I:F4}");
        sb.AppendLine($"ATC_RAT_YAW_D,{gains.YawRate.D:F5}");
        sb.AppendLine($"ATC_RAT_YAW_FF,{gains.YawRate.FF:F4}");
        sb.AppendLine($"ATC_RAT_YAW_IMAX,{gains.YawRate.Imax:F2}");
        sb.AppendLine();
        
        // Attitude P gains
        sb.AppendLine($"ATC_ANG_RLL_P,{gains.RollAttP:F2}");
        sb.AppendLine($"ATC_ANG_PIT_P,{gains.PitchAttP:F2}");
        sb.AppendLine($"ATC_ANG_YAW_P,{gains.YawAttP:F2}");
        sb.AppendLine();
        
        // Position controller
        sb.AppendLine($"PSC_POSXY_P,{gains.PosXY.P:F2}");
        sb.AppendLine($"PSC_POSZ_P,{gains.PosZ.P:F2}");
        sb.AppendLine($"PSC_VELXY_P,{gains.VelXY.P:F2}");
        sb.AppendLine($"PSC_VELXY_I,{gains.VelXY.I:F2}");
        sb.AppendLine($"PSC_VELZ_P,{gains.VelZ.P:F2}");
        sb.AppendLine();
        
        // Filters
        sb.AppendLine($"INS_GYRO_FILTER,{(int)gains.GyroLpfHz}");
        
        return sb.ToString();
    }
    
    /// <summary>
    /// Analyze flight log for tuning issues.
    /// </summary>
    public static TuningAnalysis AnalyzeFlightData(List<FlightLogEntry> log)
    {
        var analysis = new TuningAnalysis();
        
        if (log.Count < 100) return analysis;
        
        // Calculate statistics for each axis
        analysis.RollRateStats = CalculateAxisStats(log, e => e.RollRate, e => e.RollRateSetpoint);
        analysis.PitchRateStats = CalculateAxisStats(log, e => e.PitchRate, e => e.PitchRateSetpoint);
        analysis.YawRateStats = CalculateAxisStats(log, e => e.YawRate, e => e.YawRateSetpoint);
        
        // Detect oscillations using FFT (simplified - count zero crossings)
        analysis.RollOscillationHz = DetectOscillation(log, e => e.RollRate);
        analysis.PitchOscillationHz = DetectOscillation(log, e => e.PitchRate);
        analysis.YawOscillationHz = DetectOscillation(log, e => e.YawRate);
        
        // Generate recommendations
        GenerateRecommendations(analysis);
        
        return analysis;
    }
    
    private static AxisStats CalculateAxisStats(List<FlightLogEntry> log, 
        Func<FlightLogEntry, double> getValue, Func<FlightLogEntry, double> getSetpoint)
    {
        var stats = new AxisStats();
        
        var errors = log.Select(e => getValue(e) - getSetpoint(e)).ToList();
        
        stats.MeanError = errors.Average();
        stats.MaxError = errors.Max(System.Math.Abs);
        stats.RmsError = System.Math.Sqrt(errors.Average(e => e * e));
        
        // Calculate tracking delay (cross-correlation would be better)
        var values = log.Select(getValue).ToList();
        stats.StdDev = System.Math.Sqrt(values.Average(v => v * v) - values.Average() * values.Average());
        
        return stats;
    }
    
    private static double DetectOscillation(List<FlightLogEntry> log, Func<FlightLogEntry, double> getValue)
    {
        // Simple zero-crossing frequency detection
        var values = log.Select(getValue).ToList();
        double mean = values.Average();
        
        int crossings = 0;
        for (int i = 1; i < values.Count; i++)
        {
            if ((values[i - 1] - mean) * (values[i] - mean) < 0)
                crossings++;
        }
        
        double duration = (log.Last().Timestamp - log.First().Timestamp).TotalSeconds;
        return crossings / (2 * duration); // Hz
    }
    
    private static void GenerateRecommendations(TuningAnalysis analysis)
    {
        // Roll axis
        if (analysis.RollRateStats.RmsError > 30)
        {
            if (analysis.RollOscillationHz > 20)
                analysis.Recommendations.Add("Roll: High frequency oscillation detected - lower D gain or increase D-term filter");
            else
                analysis.Recommendations.Add("Roll: Large tracking error - increase P gain");
        }
        
        // Pitch axis
        if (analysis.PitchRateStats.RmsError > 30)
        {
            if (analysis.PitchOscillationHz > 20)
                analysis.Recommendations.Add("Pitch: High frequency oscillation detected - lower D gain or increase D-term filter");
            else
                analysis.Recommendations.Add("Pitch: Large tracking error - increase P gain");
        }
        
        // Yaw axis
        if (analysis.YawRateStats.RmsError > 20)
        {
            analysis.Recommendations.Add("Yaw: Consider increasing P gain for better response");
        }
        
        // Check for I-term windup
        if (analysis.RollRateStats.MeanError > 10)
        {
            analysis.Recommendations.Add("Roll: Steady-state error detected - check I gain or mechanical issues");
        }
    }
}

public class FlightLogEntry
{
    public DateTimeOffset Timestamp { get; set; }
    
    public double RollRate { get; set; }
    public double PitchRate { get; set; }
    public double YawRate { get; set; }
    
    public double RollRateSetpoint { get; set; }
    public double PitchRateSetpoint { get; set; }
    public double YawRateSetpoint { get; set; }
    
    public double Roll { get; set; }
    public double Pitch { get; set; }
    public double Yaw { get; set; }
    
    public int[] MotorOutputs { get; set; } = new int[4];
}

public class TuningAnalysis
{
    public AxisStats RollRateStats { get; set; } = new();
    public AxisStats PitchRateStats { get; set; } = new();
    public AxisStats YawRateStats { get; set; } = new();
    
    public double RollOscillationHz { get; set; }
    public double PitchOscillationHz { get; set; }
    public double YawOscillationHz { get; set; }
    
    public List<string> Recommendations { get; } = new();
}

public class AxisStats
{
    public double MeanError { get; set; }
    public double MaxError { get; set; }
    public double RmsError { get; set; }
    public double StdDev { get; set; }
}

/// <summary>
/// Motor thrust test and analysis.
/// </summary>
public class MotorThrustTest
{
    /// <summary>
    /// Calculate motor mix issues from log data.
    /// </summary>
    public static MotorMixAnalysis AnalyzeMotorMix(List<FlightLogEntry> log)
    {
        var analysis = new MotorMixAnalysis();
        
        if (log.Count < 100) return analysis;
        
        // Calculate average motor output for each motor
        var motorAvgs = new double[4];
        foreach (var entry in log)
        {
            for (int i = 0; i < 4 && i < entry.MotorOutputs.Length; i++)
            {
                motorAvgs[i] += entry.MotorOutputs[i];
            }
        }
        
        for (int i = 0; i < 4; i++)
        {
            motorAvgs[i] /= log.Count;
        }
        
        analysis.MotorAverages = motorAvgs;
        
        // Check for imbalance
        double avgAll = motorAvgs.Average();
        analysis.MotorImbalance = motorAvgs.Max() - motorAvgs.Min();
        
        // Calculate CG offset estimate
        // Front-back imbalance indicates CG is off in pitch
        double frontAvg = (motorAvgs[0] + motorAvgs[3]) / 2;  // Assuming quad-X
        double backAvg = (motorAvgs[1] + motorAvgs[2]) / 2;
        analysis.EstimatedCgOffsetPitch = (frontAvg - backAvg) / avgAll * 100;
        
        // Left-right imbalance
        double leftAvg = (motorAvgs[2] + motorAvgs[3]) / 2;
        double rightAvg = (motorAvgs[0] + motorAvgs[1]) / 2;
        analysis.EstimatedCgOffsetRoll = (leftAvg - rightAvg) / avgAll * 100;
        
        // Recommendations
        if (System.Math.Abs(analysis.EstimatedCgOffsetPitch) > 5)
        {
            string dir = analysis.EstimatedCgOffsetPitch > 0 ? "forward" : "backward";
            analysis.Recommendations.Add($"CG appears to be too far {dir} - adjust battery position");
        }
        
        if (System.Math.Abs(analysis.EstimatedCgOffsetRoll) > 5)
        {
            string dir = analysis.EstimatedCgOffsetRoll > 0 ? "left" : "right";
            analysis.Recommendations.Add($"CG appears to be too far {dir} - adjust component placement");
        }
        
        if (analysis.MotorImbalance > 15)
        {
            analysis.Recommendations.Add("Large motor output variance - check for bent props or motor issues");
        }
        
        return analysis;
    }
}

public class MotorMixAnalysis
{
    public double[] MotorAverages { get; set; } = new double[4];
    public double MotorImbalance { get; set; }
    public double EstimatedCgOffsetPitch { get; set; }
    public double EstimatedCgOffsetRoll { get; set; }
    public List<string> Recommendations { get; } = new();
}

