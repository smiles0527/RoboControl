using MathNet.Numerics;
using MathNet.Numerics.IntegralTransforms;
using MathNet.Numerics.LinearAlgebra;
using SysComplex = System.Numerics.Complex;
using SysMath = System.Math;

namespace ControlWorkbench.Math.Analysis;

/// <summary>
/// Frequency domain analysis tools including FFT and Bode plots.
/// </summary>
public static class FrequencyAnalysis
{
    /// <summary>
    /// Compute FFT of a time-domain signal.
    /// </summary>
    public static FftResult ComputeFft(double[] signal, double sampleRate)
    {
        int n = signal.Length;
        
        // Pad to next power of 2 for efficiency
        int nfft = 1;
        while (nfft < n) nfft *= 2;
        
        // Convert to complex
        var complex = new SysComplex[nfft];
        for (int i = 0; i < n; i++)
        {
            complex[i] = new SysComplex(signal[i], 0);
        }
        for (int i = n; i < nfft; i++)
        {
            complex[i] = SysComplex.Zero;
        }
        
        // Perform FFT
        Fourier.Forward(complex, FourierOptions.Matlab);
        
        // Compute single-sided spectrum
        int numFreqs = nfft / 2 + 1;
        var frequencies = new double[numFreqs];
        var magnitudes = new double[numFreqs];
        var phases = new double[numFreqs];
        var powerSpectrum = new double[numFreqs];
        
        double freqStep = sampleRate / nfft;
        
        for (int i = 0; i < numFreqs; i++)
        {
            frequencies[i] = i * freqStep;
            magnitudes[i] = complex[i].Magnitude * 2.0 / n; // Scale for single-sided
            phases[i] = complex[i].Phase * 180.0 / SysMath.PI;
            powerSpectrum[i] = magnitudes[i] * magnitudes[i];
        }
        
        // DC component doesn't get doubled
        magnitudes[0] /= 2;
        powerSpectrum[0] = magnitudes[0] * magnitudes[0];
        
        return new FftResult
        {
            Frequencies = frequencies,
            Magnitudes = magnitudes,
            Phases = phases,
            PowerSpectrum = powerSpectrum,
            SampleRate = sampleRate,
            NumSamples = n
        };
    }
    
    /// <summary>
    /// Apply window function to signal before FFT.
    /// </summary>
    public static double[] ApplyWindow(double[] signal, WindowType window)
    {
        int n = signal.Length;
        var windowed = new double[n];
        
        for (int i = 0; i < n; i++)
        {
            double w = window switch
            {
                WindowType.Rectangular => 1.0,
                WindowType.Hanning => 0.5 * (1 - SysMath.Cos(2 * SysMath.PI * i / (n - 1))),
                WindowType.Hamming => 0.54 - 0.46 * SysMath.Cos(2 * SysMath.PI * i / (n - 1)),
                WindowType.Blackman => 0.42 - 0.5 * SysMath.Cos(2 * SysMath.PI * i / (n - 1)) 
                                          + 0.08 * SysMath.Cos(4 * SysMath.PI * i / (n - 1)),
                WindowType.FlatTop => 0.21557895 - 0.41663158 * SysMath.Cos(2 * SysMath.PI * i / (n - 1))
                                     + 0.277263158 * SysMath.Cos(4 * SysMath.PI * i / (n - 1))
                                     - 0.083578947 * SysMath.Cos(6 * SysMath.PI * i / (n - 1))
                                     + 0.006947368 * SysMath.Cos(8 * SysMath.PI * i / (n - 1)),
                _ => 1.0
            };
            windowed[i] = signal[i] * w;
        }
        
        return windowed;
    }
    
    /// <summary>
    /// Compute power spectral density using Welch's method.
    /// </summary>
    public static PsdResult ComputePsd(
        double[] signal, 
        double sampleRate,
        int segmentLength = 256,
        double overlapFraction = 0.5,
        WindowType window = WindowType.Hanning)
    {
        int n = signal.Length;
        int overlap = (int)(segmentLength * overlapFraction);
        int step = segmentLength - overlap;
        int numSegments = (n - overlap) / step;
        
        if (numSegments < 1)
        {
            // Fall back to single FFT
            var fftResult = ComputeFft(ApplyWindow(signal, window), sampleRate);
            return new PsdResult
            {
                Frequencies = fftResult.Frequencies,
                Psd = fftResult.PowerSpectrum.Select(p => p / sampleRate).ToArray(),
                SampleRate = sampleRate,
                NumSegments = 1
            };
        }
        
        int numFreqs = segmentLength / 2 + 1;
        var psdSum = new double[numFreqs];
        double freqStep = sampleRate / segmentLength;
        
        for (int seg = 0; seg < numSegments; seg++)
        {
            int start = seg * step;
            var segment = new double[segmentLength];
            Array.Copy(signal, start, segment, 0, SysMath.Min(segmentLength, n - start));
            
            var windowed = ApplyWindow(segment, window);
            var fft = ComputeFft(windowed, sampleRate);
            
            for (int i = 0; i < numFreqs; i++)
            {
                psdSum[i] += fft.PowerSpectrum[i];
            }
        }
        
        // Average and convert to PSD
        var psd = new double[numFreqs];
        var frequencies = new double[numFreqs];
        for (int i = 0; i < numFreqs; i++)
        {
            frequencies[i] = i * freqStep;
            psd[i] = psdSum[i] / numSegments / sampleRate;
        }
        
        return new PsdResult
        {
            Frequencies = frequencies,
            Psd = psd,
            SampleRate = sampleRate,
            NumSegments = numSegments
        };
    }
    
    /// <summary>
    /// Find dominant frequencies in a signal.
    /// </summary>
    public static List<FrequencyPeak> FindPeaks(
        FftResult fft,
        double minMagnitude = 0.01,
        int maxPeaks = 10)
    {
        var peaks = new List<FrequencyPeak>();
        
        // Skip DC
        for (int i = 1; i < fft.Magnitudes.Length - 1; i++)
        {
            if (fft.Magnitudes[i] > minMagnitude &&
                fft.Magnitudes[i] > fft.Magnitudes[i - 1] &&
                fft.Magnitudes[i] > fft.Magnitudes[i + 1])
            {
                // Quadratic interpolation for better frequency estimate
                double y0 = fft.Magnitudes[i - 1];
                double y1 = fft.Magnitudes[i];
                double y2 = fft.Magnitudes[i + 1];
                
                double delta = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2);
                double refinedFreq = fft.Frequencies[i] + delta * (fft.Frequencies[1] - fft.Frequencies[0]);
                double refinedMag = y1 - 0.25 * (y0 - y2) * delta;
                
                peaks.Add(new FrequencyPeak
                {
                    Frequency = refinedFreq,
                    Magnitude = refinedMag,
                    Phase = fft.Phases[i]
                });
            }
        }
        
        return peaks.OrderByDescending(p => p.Magnitude).Take(maxPeaks).ToList();
    }
}

/// <summary>
/// Bode plot computation for transfer functions.
/// </summary>
public static class BodePlot
{
    /// <summary>
    /// Compute Bode plot for a continuous-time transfer function.
    /// G(s) = num(s) / den(s)
    /// </summary>
    public static BodeResult Compute(
        double[] numerator,
        double[] denominator,
        double minFreqHz = 0.01,
        double maxFreqHz = 1000,
        int numPoints = 100)
    {
        var frequencies = new double[numPoints];
        var magnitudesDb = new double[numPoints];
        var phasesDeg = new double[numPoints];
        
        // Logarithmic frequency spacing
        double logMin = SysMath.Log10(minFreqHz);
        double logMax = SysMath.Log10(maxFreqHz);
        double logStep = (logMax - logMin) / (numPoints - 1);
        
        for (int i = 0; i < numPoints; i++)
        {
            double freqHz = SysMath.Pow(10, logMin + i * logStep);
            double omega = 2 * SysMath.PI * freqHz;
            
            // Evaluate G(j?)
            var s = new SysComplex(0, omega);
            
            var num = EvaluatePolynomial(numerator, s);
            var den = EvaluatePolynomial(denominator, s);
            var G = num / den;
            
            frequencies[i] = freqHz;
            magnitudesDb[i] = 20 * SysMath.Log10(G.Magnitude);
            phasesDeg[i] = G.Phase * 180 / SysMath.PI;
        }
        
        // Unwrap phase
        UnwrapPhase(phasesDeg);
        
        // Compute margins
        var margins = ComputeMargins(frequencies, magnitudesDb, phasesDeg);
        
        return new BodeResult
        {
            Frequencies = frequencies,
            MagnitudesDb = magnitudesDb,
            PhasesDeg = phasesDeg,
            GainMarginDb = margins.GainMargin,
            PhaseMarginDeg = margins.PhaseMargin,
            GainCrossoverHz = margins.GainCrossover,
            PhaseCrossoverHz = margins.PhaseCrossover
        };
    }
    
    /// <summary>
    /// Compute Bode plot for PID controller.
    /// C(s) = Kp + Ki/s + Kd*s
    /// </summary>
    public static BodeResult ComputePid(
        double kp, double ki, double kd,
        double minFreqHz = 0.01,
        double maxFreqHz = 1000,
        int numPoints = 100)
    {
        var frequencies = new double[numPoints];
        var magnitudesDb = new double[numPoints];
        var phasesDeg = new double[numPoints];
        
        double logMin = SysMath.Log10(minFreqHz);
        double logMax = SysMath.Log10(maxFreqHz);
        double logStep = (logMax - logMin) / (numPoints - 1);
        
        for (int i = 0; i < numPoints; i++)
        {
            double freqHz = SysMath.Pow(10, logMin + i * logStep);
            double omega = 2 * SysMath.PI * freqHz;
            
            var s = new SysComplex(0, omega);
            
            // C(s) = Kp + Ki/s + Kd*s
            var C = new SysComplex(kp, 0) + ki / s + kd * s;
            
            frequencies[i] = freqHz;
            magnitudesDb[i] = 20 * SysMath.Log10(C.Magnitude);
            phasesDeg[i] = C.Phase * 180 / SysMath.PI;
        }
        
        return new BodeResult
        {
            Frequencies = frequencies,
            MagnitudesDb = magnitudesDb,
            PhasesDeg = phasesDeg
        };
    }
    
    /// <summary>
    /// Compute Bode plot for first-order system: G(s) = K / (?s + 1)
    /// </summary>
    public static BodeResult ComputeFirstOrder(
        double gain,
        double timeConstant,
        double minFreqHz = 0.01,
        double maxFreqHz = 1000,
        int numPoints = 100)
    {
        // G(s) = K / (?s + 1) = K / (?*s + 1)
        // Numerator: [K]
        // Denominator: [?, 1]
        return Compute(
            [gain],
            [timeConstant, 1],
            minFreqHz, maxFreqHz, numPoints);
    }
    
    /// <summary>
    /// Compute Bode plot for second-order system: G(s) = ?n² / (s² + 2??n*s + ?n²)
    /// </summary>
    public static BodeResult ComputeSecondOrder(
        double naturalFrequency,
        double dampingRatio,
        double gain = 1.0,
        double minFreqHz = 0.01,
        double maxFreqHz = 1000,
        int numPoints = 100)
    {
        double wn = naturalFrequency;
        double zeta = dampingRatio;
        
        // Numerator: [K * ?n²]
        // Denominator: [1, 2??n, ?n²]
        return Compute(
            [gain * wn * wn],
            [1, 2 * zeta * wn, wn * wn],
            minFreqHz, maxFreqHz, numPoints);
    }
    
    private static SysComplex EvaluatePolynomial(double[] coefficients, SysComplex s)
    {
        // Coefficients are in descending order of power: [a_n, a_{n-1}, ..., a_1, a_0]
        var result = SysComplex.Zero;
        var sPower = SysComplex.One;
        
        for (int i = coefficients.Length - 1; i >= 0; i--)
        {
            result += coefficients[i] * sPower;
            sPower *= s;
        }
        
        return result;
    }
    
    private static void UnwrapPhase(double[] phases)
    {
        for (int i = 1; i < phases.Length; i++)
        {
            double diff = phases[i] - phases[i - 1];
            if (diff > 180)
                phases[i] -= 360;
            else if (diff < -180)
                phases[i] += 360;
        }
    }
    
    private static (double GainMargin, double PhaseMargin, double GainCrossover, double PhaseCrossover) 
        ComputeMargins(double[] frequencies, double[] magnitudesDb, double[] phasesDeg)
    {
        double gainMargin = double.PositiveInfinity;
        double phaseMargin = double.PositiveInfinity;
        double gainCrossover = double.NaN;
        double phaseCrossover = double.NaN;
        
        // Find gain crossover (magnitude = 0 dB)
        for (int i = 1; i < magnitudesDb.Length; i++)
        {
            if ((magnitudesDb[i - 1] > 0 && magnitudesDb[i] <= 0) ||
                (magnitudesDb[i - 1] < 0 && magnitudesDb[i] >= 0))
            {
                // Interpolate
                double t = -magnitudesDb[i - 1] / (magnitudesDb[i] - magnitudesDb[i - 1]);
                gainCrossover = frequencies[i - 1] + t * (frequencies[i] - frequencies[i - 1]);
                double phaseAtCrossover = phasesDeg[i - 1] + t * (phasesDeg[i] - phasesDeg[i - 1]);
                phaseMargin = 180 + phaseAtCrossover;
                break;
            }
        }
        
        // Find phase crossover (phase = -180°)
        for (int i = 1; i < phasesDeg.Length; i++)
        {
            if ((phasesDeg[i - 1] > -180 && phasesDeg[i] <= -180) ||
                (phasesDeg[i - 1] < -180 && phasesDeg[i] >= -180))
            {
                double t = (-180 - phasesDeg[i - 1]) / (phasesDeg[i] - phasesDeg[i - 1]);
                phaseCrossover = frequencies[i - 1] + t * (frequencies[i] - frequencies[i - 1]);
                double magAtCrossover = magnitudesDb[i - 1] + t * (magnitudesDb[i] - magnitudesDb[i - 1]);
                gainMargin = -magAtCrossover;
                break;
            }
        }
        
        return (gainMargin, phaseMargin, gainCrossover, phaseCrossover);
    }
}

/// <summary>
/// System identification from input/output data.
/// </summary>
public static class SystemIdentification
{
    /// <summary>
    /// Estimate first-order system parameters from step response data.
    /// G(s) = K / (?s + 1)
    /// </summary>
    public static FirstOrderEstimate EstimateFirstOrder(
        double[] times,
        double[] output,
        double stepInput)
    {
        if (times.Length != output.Length || times.Length < 10)
            throw new ArgumentException("Need at least 10 data points");
        
        double y0 = output[0];
        double yFinal = output[^1];
        double K = (yFinal - y0) / stepInput;
        
        // Find time constant (63.2% of final value)
        double y632 = y0 + 0.632 * (yFinal - y0);
        double tau = double.NaN;
        
        for (int i = 1; i < output.Length; i++)
        {
            if ((output[i - 1] < y632 && output[i] >= y632) ||
                (output[i - 1] > y632 && output[i] <= y632))
            {
                // Interpolate
                double t = (y632 - output[i - 1]) / (output[i] - output[i - 1]);
                tau = times[i - 1] + t * (times[i] - times[i - 1]);
                break;
            }
        }
        
        // Compute fit error
        double sumSquaredError = 0;
        for (int i = 0; i < times.Length; i++)
        {
            double predicted = y0 + K * stepInput * (1 - SysMath.Exp(-times[i] / tau));
            double error = output[i] - predicted;
            sumSquaredError += error * error;
        }
        double rmse = SysMath.Sqrt(sumSquaredError / times.Length);
        
        return new FirstOrderEstimate
        {
            Gain = K,
            TimeConstant = tau,
            InitialValue = y0,
            FinalValue = yFinal,
            Rmse = rmse
        };
    }
    
    /// <summary>
    /// Estimate second-order system parameters from step response.
    /// G(s) = K?n² / (s² + 2??n*s + ?n²)
    /// </summary>
    public static SecondOrderEstimate EstimateSecondOrder(
        double[] times,
        double[] output,
        double stepInput)
    {
        if (times.Length != output.Length || times.Length < 20)
            throw new ArgumentException("Need at least 20 data points");
        
        double y0 = output[0];
        double yFinal = output[^1];
        double K = (yFinal - y0) / stepInput;
        
        // Find peak (overshoot)
        double yPeak = output[0];
        double tPeak = times[0];
        for (int i = 1; i < output.Length - 1; i++)
        {
            if (output[i] > output[i - 1] && output[i] > output[i + 1] && output[i] > yPeak)
            {
                yPeak = output[i];
                tPeak = times[i];
            }
        }
        
        // Calculate damping ratio from overshoot
        double Mp = (yPeak - yFinal) / (yFinal - y0);
        double zeta;
        if (Mp <= 0)
        {
            zeta = 1.0; // Overdamped or critically damped
        }
        else
        {
            // Mp = exp(-?? / ?(1-?²))
            // Solve numerically
            zeta = 0.5;
            for (int iter = 0; iter < 20; iter++)
            {
                double predicted = SysMath.Exp(-zeta * SysMath.PI / SysMath.Sqrt(1 - zeta * zeta));
                double error = predicted - Mp;
                zeta += 0.1 * error; // Simple gradient
                zeta = SysMath.Clamp(zeta, 0.01, 0.99);
            }
        }
        
        // Calculate natural frequency from peak time
        double omegaN;
        if (zeta < 1 && tPeak > 0)
        {
            // tp = ? / (?n?(1-?²))
            double omegaD = SysMath.PI / tPeak;
            omegaN = omegaD / SysMath.Sqrt(1 - zeta * zeta);
        }
        else
        {
            // Estimate from rise time
            double t10 = double.NaN, t90 = double.NaN;
            double y10 = y0 + 0.1 * (yFinal - y0);
            double y90 = y0 + 0.9 * (yFinal - y0);
            
            for (int i = 1; i < output.Length; i++)
            {
                if (double.IsNaN(t10) && output[i] >= y10)
                    t10 = times[i];
                if (double.IsNaN(t90) && output[i] >= y90)
                    t90 = times[i];
            }
            
            double tr = t90 - t10;
            omegaN = 1.8 / tr; // Approximate for ? ? 1
        }
        
        // Compute fit error
        double sumSquaredError = 0;
        for (int i = 0; i < times.Length; i++)
        {
            double t = times[i];
            double predicted;
            if (zeta < 1)
            {
                double omegaD = omegaN * SysMath.Sqrt(1 - zeta * zeta);
                predicted = y0 + K * stepInput * (1 - SysMath.Exp(-zeta * omegaN * t) *
                    (SysMath.Cos(omegaD * t) + zeta / SysMath.Sqrt(1 - zeta * zeta) * SysMath.Sin(omegaD * t)));
            }
            else
            {
                double s1 = -omegaN * (zeta - SysMath.Sqrt(zeta * zeta - 1));
                double s2 = -omegaN * (zeta + SysMath.Sqrt(zeta * zeta - 1));
                predicted = y0 + K * stepInput * (1 - (s2 * SysMath.Exp(s1 * t) - s1 * SysMath.Exp(s2 * t)) / (s2 - s1));
            }
            
            double error = output[i] - predicted;
            sumSquaredError += error * error;
        }
        double rmse = SysMath.Sqrt(sumSquaredError / times.Length);
        
        return new SecondOrderEstimate
        {
            Gain = K,
            NaturalFrequency = omegaN,
            DampingRatio = zeta,
            Overshoot = Mp * 100,
            PeakTime = tPeak,
            InitialValue = y0,
            FinalValue = yFinal,
            Rmse = rmse
        };
    }
    
    /// <summary>
    /// Estimate ARX model parameters using least squares.
    /// y(k) = a1*y(k-1) + ... + an*y(k-n) + b1*u(k-1) + ... + bm*u(k-m)
    /// </summary>
    public static ArxEstimate EstimateArx(
        double[] input,
        double[] output,
        int na, // Number of auto-regressive terms
        int nb, // Number of exogenous input terms
        int nk = 1) // Input delay
    {
        int n = output.Length;
        int numParams = na + nb;
        int startIdx = SysMath.Max(na, nk + nb - 1);
        int numSamples = n - startIdx;
        
        if (numSamples < numParams)
            throw new ArgumentException("Not enough data points for estimation");
        
        // Build regression matrix
        var phi = Matrix<double>.Build.Dense(numSamples, numParams);
        var y = Vector<double>.Build.Dense(numSamples);
        
        for (int i = 0; i < numSamples; i++)
        {
            int idx = startIdx + i;
            y[i] = output[idx];
            
            // AR terms
            for (int j = 0; j < na; j++)
            {
                phi[i, j] = -output[idx - 1 - j];
            }
            
            // Exogenous terms
            for (int j = 0; j < nb; j++)
            {
                phi[i, na + j] = input[idx - nk - j];
            }
        }
        
        // Least squares: ? = (?'?)^-1 ?'y
        var phiT = phi.Transpose();
        var theta = (phiT * phi).Inverse() * phiT * y;
        
        var aCoeffs = new double[na];
        var bCoeffs = new double[nb];
        for (int i = 0; i < na; i++) aCoeffs[i] = theta[i];
        for (int i = 0; i < nb; i++) bCoeffs[i] = theta[na + i];
        
        // Compute prediction error
        var yPred = phi * theta;
        var error = y - yPred;
        double rmse = SysMath.Sqrt(error.DotProduct(error) / numSamples);
        
        return new ArxEstimate
        {
            ACoefficients = aCoeffs,
            BCoefficients = bCoeffs,
            InputDelay = nk,
            Rmse = rmse
        };
    }
}

// Result classes

public class FftResult
{
    public double[] Frequencies { get; set; } = [];
    public double[] Magnitudes { get; set; } = [];
    public double[] Phases { get; set; } = [];
    public double[] PowerSpectrum { get; set; } = [];
    public double SampleRate { get; set; }
    public int NumSamples { get; set; }
}

public class PsdResult
{
    public double[] Frequencies { get; set; } = [];
    public double[] Psd { get; set; } = [];
    public double SampleRate { get; set; }
    public int NumSegments { get; set; }
}

public class FrequencyPeak
{
    public double Frequency { get; set; }
    public double Magnitude { get; set; }
    public double Phase { get; set; }
}

public enum WindowType
{
    Rectangular,
    Hanning,
    Hamming,
    Blackman,
    FlatTop
}

public class BodeResult
{
    public double[] Frequencies { get; set; } = [];
    public double[] MagnitudesDb { get; set; } = [];
    public double[] PhasesDeg { get; set; } = [];
    public double GainMarginDb { get; set; }
    public double PhaseMarginDeg { get; set; }
    public double GainCrossoverHz { get; set; }
    public double PhaseCrossoverHz { get; set; }
    
    public string GetMarginsSummary()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("=== Stability Margins ===");
        if (!double.IsNaN(GainCrossoverHz) && !double.IsInfinity(PhaseMarginDeg))
            sb.AppendLine($"Phase Margin: {PhaseMarginDeg:F1}° at {GainCrossoverHz:F2} Hz");
        else
            sb.AppendLine("Phase Margin: N/A (no gain crossover)");
        
        if (!double.IsNaN(PhaseCrossoverHz) && !double.IsInfinity(GainMarginDb))
            sb.AppendLine($"Gain Margin: {GainMarginDb:F1} dB at {PhaseCrossoverHz:F2} Hz");
        else
            sb.AppendLine("Gain Margin: ? (no phase crossover)");
        
        return sb.ToString();
    }
}

public class FirstOrderEstimate
{
    public double Gain { get; set; }
    public double TimeConstant { get; set; }
    public double InitialValue { get; set; }
    public double FinalValue { get; set; }
    public double Rmse { get; set; }
    
    public string GetSummary() =>
        $"First Order: G(s) = {Gain:F3} / ({TimeConstant:F4}s + 1)\n" +
        $"Bandwidth: {1 / (2 * SysMath.PI * TimeConstant):F2} Hz\n" +
        $"RMSE: {Rmse:F4}";
}

public class SecondOrderEstimate
{
    public double Gain { get; set; }
    public double NaturalFrequency { get; set; }
    public double DampingRatio { get; set; }
    public double Overshoot { get; set; }
    public double PeakTime { get; set; }
    public double InitialValue { get; set; }
    public double FinalValue { get; set; }
    public double Rmse { get; set; }
    
    public string GetSummary() =>
        $"Second Order: G(s) = {Gain:F3}·?n² / (s² + 2??n·s + ?n²)\n" +
        $"?n = {NaturalFrequency:F3} rad/s ({NaturalFrequency / (2 * SysMath.PI):F2} Hz)\n" +
        $"? = {DampingRatio:F3}\n" +
        $"Overshoot: {Overshoot:F1}%\n" +
        $"RMSE: {Rmse:F4}";
}

public class ArxEstimate
{
    public double[] ACoefficients { get; set; } = [];
    public double[] BCoefficients { get; set; } = [];
    public int InputDelay { get; set; }
    public double Rmse { get; set; }
    
    public string GetSummary()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"ARX Model (na={ACoefficients.Length}, nb={BCoefficients.Length}, nk={InputDelay}):");
        sb.Append("y(k) = ");
        for (int i = 0; i < ACoefficients.Length; i++)
            sb.Append($"{-ACoefficients[i]:F4}·y(k-{i + 1}) + ");
        for (int i = 0; i < BCoefficients.Length; i++)
            sb.Append($"{BCoefficients[i]:F4}·u(k-{InputDelay + i}) + ");
        sb.AppendLine("e(k)");
        sb.AppendLine($"RMSE: {Rmse:F4}");
        return sb.ToString();
    }
}
