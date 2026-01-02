using System.Text;
using System.Windows.Input;
using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math;
using ControlWorkbench.Math.Control;
using ControlWorkbench.Math.Metrics;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// ViewModel for the Control Design tab (PID/LQR).
/// </summary>
public class ControlDesignViewModel : ViewModelBase
{
    // PID Parameters
    private double _pidKp = 1.0;
    private double _pidKi = 0.1;
    private double _pidKd = 0.01;
    private double _pidOutputLimit = 10.0;
    private double _pidIntegratorLimit = 5.0;
    private double _pidDerivativeFilterTf = 0.01;

    // Cascaded PID
    private double _innerLoopBandwidth = 10.0;
    private double _outerLoopBandwidthRatio = 0.2;
    private double _cascadeSampleTime = 0.01;
    private CascadeLoopType _innerLoopType = CascadeLoopType.Rate;
    private CascadeLoopType _outerLoopType = CascadeLoopType.Angle;

    // LQR Parameters
    private string _lqrAMatrix = "0, 1; -1, -0.5";
    private string _lqrBMatrix = "0; 1";
    private string _lqrQDiagonal = "1, 1";
    private string _lqrRDiagonal = "0.1";
    private bool _useContinuousTime = true;

    // Results
    private string _pidResult = "";
    private string _cascadeResult = "";
    private string _lqrResult = "";
    private string _stepResponseResult = "";

    public ControlDesignViewModel()
    {
        ComputePidCommand = new RelayCommand(ComputePid);
        DesignCascadeCommand = new RelayCommand(DesignCascade);
        SolveLqrCommand = new RelayCommand(SolveLqr);
        SimulateLqrStepResponseCommand = new RelayCommand(SimulateLqrStepResponse);
        ExportPidCodeCommand = new RelayCommand(ExportPidCode);
    }

    // Commands
    public ICommand ComputePidCommand { get; }
    public ICommand DesignCascadeCommand { get; }
    public ICommand SolveLqrCommand { get; }
    public ICommand SimulateLqrStepResponseCommand { get; }
    public ICommand ExportPidCodeCommand { get; }

    // PID Properties
    public double PidKp
    {
        get => _pidKp;
        set => SetProperty(ref _pidKp, value);
    }

    public double PidKi
    {
        get => _pidKi;
        set => SetProperty(ref _pidKi, value);
    }

    public double PidKd
    {
        get => _pidKd;
        set => SetProperty(ref _pidKd, value);
    }

    public double PidOutputLimit
    {
        get => _pidOutputLimit;
        set => SetProperty(ref _pidOutputLimit, value);
    }

    public double PidIntegratorLimit
    {
        get => _pidIntegratorLimit;
        set => SetProperty(ref _pidIntegratorLimit, value);
    }

    public double PidDerivativeFilterTf
    {
        get => _pidDerivativeFilterTf;
        set => SetProperty(ref _pidDerivativeFilterTf, value);
    }

    // Cascade Properties
    public double InnerLoopBandwidth
    {
        get => _innerLoopBandwidth;
        set => SetProperty(ref _innerLoopBandwidth, value);
    }

    public double OuterLoopBandwidthRatio
    {
        get => _outerLoopBandwidthRatio;
        set => SetProperty(ref _outerLoopBandwidthRatio, value);
    }

    public double CascadeSampleTime
    {
        get => _cascadeSampleTime;
        set => SetProperty(ref _cascadeSampleTime, value);
    }

    public CascadeLoopType[] CascadeLoopTypes => Enum.GetValues<CascadeLoopType>();

    public CascadeLoopType InnerLoopType
    {
        get => _innerLoopType;
        set => SetProperty(ref _innerLoopType, value);
    }

    public CascadeLoopType OuterLoopType
    {
        get => _outerLoopType;
        set => SetProperty(ref _outerLoopType, value);
    }

    // LQR Properties
    public string LqrAMatrix
    {
        get => _lqrAMatrix;
        set => SetProperty(ref _lqrAMatrix, value);
    }

    public string LqrBMatrix
    {
        get => _lqrBMatrix;
        set => SetProperty(ref _lqrBMatrix, value);
    }

    public string LqrQDiagonal
    {
        get => _lqrQDiagonal;
        set => SetProperty(ref _lqrQDiagonal, value);
    }

    public string LqrRDiagonal
    {
        get => _lqrRDiagonal;
        set => SetProperty(ref _lqrRDiagonal, value);
    }

    public bool UseContinuousTime
    {
        get => _useContinuousTime;
        set => SetProperty(ref _useContinuousTime, value);
    }

    // Result Properties
    public string PidResult
    {
        get => _pidResult;
        set => SetProperty(ref _pidResult, value);
    }

    public string CascadeResult
    {
        get => _cascadeResult;
        set => SetProperty(ref _cascadeResult, value);
    }

    public string LqrResult
    {
        get => _lqrResult;
        set => SetProperty(ref _lqrResult, value);
    }

    public string StepResponseResult
    {
        get => _stepResponseResult;
        set => SetProperty(ref _stepResponseResult, value);
    }

    private void ComputePid()
    {
        try
        {
            var pid = new PidController(PidKp, PidKi, PidKd)
            {
                DerivativeFilterTf = PidDerivativeFilterTf
            };
            pid.SetOutputLimits(PidOutputLimit);
            pid.SetIntegratorLimits(PidIntegratorLimit);

            var sb = new StringBuilder();
            sb.AppendLine("=== PID Controller Configuration ===");
            sb.AppendLine();
            sb.AppendLine($"Kp: {PidKp:F4}");
            sb.AppendLine($"Ki: {PidKi:F4}");
            sb.AppendLine($"Kd: {PidKd:F4}");
            sb.AppendLine();
            sb.AppendLine($"Output Limits: ±{PidOutputLimit:F2}");
            sb.AppendLine($"Integrator Limits: ±{PidIntegratorLimit:F2}");
            sb.AppendLine($"Derivative Filter Tf: {PidDerivativeFilterTf:F4} s");
            sb.AppendLine();
            sb.AppendLine("=== Transfer Function (ideal) ===");
            sb.AppendLine($"C(s) = Kp + Ki/s + Kd·s");
            sb.AppendLine($"     = {PidKp:F4} + {PidKi:F4}/s + {PidKd:F4}·s");
            sb.AppendLine();

            if (PidKi > 0)
            {
                double Ti = PidKp / PidKi;
                sb.AppendLine($"Integral Time Ti = Kp/Ki = {Ti:F4} s");
            }
            if (PidKd > 0)
            {
                double Td = PidKd / PidKp;
                sb.AppendLine($"Derivative Time Td = Kd/Kp = {Td:F4} s");
            }

            sb.AppendLine();
            sb.AppendLine("=== Discrete Implementation (Tustin) ===");
            sb.AppendLine("P[k] = Kp · e[k]");
            sb.AppendLine("I[k] = I[k-1] + Ki · (e[k] + e[k-1]) · dt / 2");
            sb.AppendLine("D[k] = ?·((e[k] - e[k-1])/dt) + (1-?)·D[k-1]  where ? = dt/(Tf+dt)");
            sb.AppendLine("u[k] = clamp(P[k] + I[k] + Kd·D[k], limits)");

            PidResult = sb.ToString();
        }
        catch (Exception ex)
        {
            PidResult = $"Error: {ex.Message}";
        }
    }

    private void DesignCascade()
    {
        try
        {
            var design = new CascadedPidDesign
            {
                InnerLoopBandwidth = InnerLoopBandwidth,
                OuterLoopBandwidthRatio = OuterLoopBandwidthRatio,
                SampleTime = CascadeSampleTime,
                InnerLoopType = InnerLoopType,
                OuterLoopType = OuterLoopType
            };

            var sb = new StringBuilder();
            sb.AppendLine(design.GetRecommendations());
            sb.AppendLine();

            var innerPid = design.DesignInnerLoop();
            sb.AppendLine("=== Recommended Inner Loop Gains ===");
            sb.AppendLine($"Kp: {innerPid.Kp:F4}");
            sb.AppendLine($"Ki: {innerPid.Ki:F4}");
            sb.AppendLine($"Output Limits: ±{innerPid.OutputMax:F2}");
            sb.AppendLine();

            var outerPid = design.DesignOuterLoop();
            sb.AppendLine("=== Recommended Outer Loop Gains ===");
            sb.AppendLine($"Kp: {outerPid.Kp:F4}");
            sb.AppendLine($"Ki: {outerPid.Ki:F4}");
            sb.AppendLine($"Output Limits: ±{outerPid.OutputMax:F2}");

            CascadeResult = sb.ToString();
        }
        catch (Exception ex)
        {
            CascadeResult = $"Error: {ex.Message}";
        }
    }

    private void SolveLqr()
    {
        try
        {
            var A = ParseMatrix(LqrAMatrix);
            var B = ParseMatrix(LqrBMatrix);
            var Q = ParseDiagonalMatrix(LqrQDiagonal);
            var R = ParseDiagonalMatrix(LqrRDiagonal);

            LqrResult result;
            if (UseContinuousTime)
            {
                result = LqrDesign.SolveContinuous(A, B, Q, R);
            }
            else
            {
                result = LqrDesign.SolveDiscrete(A, B, Q, R);
            }

            var sb = new StringBuilder();
            sb.AppendLine($"=== LQR Design ({(UseContinuousTime ? "Continuous" : "Discrete")}) ===");
            sb.AppendLine();
            sb.AppendLine("System Matrix A:");
            sb.AppendLine(MatrixUtilities.Format(A));
            sb.AppendLine("Input Matrix B:");
            sb.AppendLine(MatrixUtilities.Format(B));
            sb.AppendLine("State Weight Q:");
            sb.AppendLine(MatrixUtilities.Format(Q));
            sb.AppendLine("Input Weight R:");
            sb.AppendLine(MatrixUtilities.Format(R));
            sb.AppendLine();

            if (result.Success)
            {
                sb.AppendLine("=== Solution ===");
                sb.AppendLine();
                sb.AppendLine("State Feedback Gain K:");
                sb.AppendLine(MatrixUtilities.Format(result.K));
                sb.AppendLine();
                sb.AppendLine("Riccati Solution P:");
                sb.AppendLine(MatrixUtilities.Format(result.P));
                sb.AppendLine();
                sb.AppendLine("Closed-Loop Poles (A - B·K):");
                foreach (var pole in result.ClosedLoopPoles)
                {
                    if (System.Math.Abs(pole.Imaginary) < 1e-10)
                    {
                        sb.AppendLine($"  {pole.Real:F4}");
                    }
                    else
                    {
                        sb.AppendLine($"  {pole.Real:F4} ± {System.Math.Abs(pole.Imaginary):F4}j");
                    }
                }
                sb.AppendLine();

                // Check stability
                bool stable = result.ClosedLoopPoles.All(p =>
                    UseContinuousTime ? p.Real < 0 : p.Magnitude < 1);
                sb.AppendLine(stable ? "? System is stable" : "? System is unstable");
            }
            else
            {
                sb.AppendLine($"? LQR design failed: {result.ErrorMessage}");
            }

            LqrResult = sb.ToString();
        }
        catch (Exception ex)
        {
            LqrResult = $"Error: {ex.Message}";
        }
    }

    private void SimulateLqrStepResponse()
    {
        try
        {
            var A = ParseMatrix(LqrAMatrix);
            var B = ParseMatrix(LqrBMatrix);
            var Q = ParseDiagonalMatrix(LqrQDiagonal);
            var R = ParseDiagonalMatrix(LqrRDiagonal);

            var lqrResult = UseContinuousTime
                ? LqrDesign.SolveContinuous(A, B, Q, R)
                : LqrDesign.SolveDiscrete(A, B, Q, R);

            if (!lqrResult.Success)
            {
                StepResponseResult = $"Cannot simulate: {lqrResult.ErrorMessage}";
                return;
            }

            // Target state: first element = 1, rest = 0
            var target = Vector<double>.Build.Dense(A.RowCount);
            target[0] = 1.0;

            var (times, states) = LqrDesign.SimulateStepResponse(A, B, lqrResult.K, target, 5.0, 0.01);

            // Extract first state for metrics
            var signal = new double[times.Length];
            for (int i = 0; i < times.Length; i++)
            {
                signal[i] = states[i, 0];
            }

            var metrics = StepResponseAnalyzer.Analyze(times, signal, 1.0);

            var sb = new StringBuilder();
            sb.AppendLine("=== Step Response Simulation ===");
            sb.AppendLine();
            sb.AppendLine(StepResponseAnalyzer.FormatMetrics(metrics));
            sb.AppendLine();
            sb.AppendLine("Sample values:");
            sb.AppendLine("Time (s)    State[0]");
            for (int i = 0; i < times.Length; i += times.Length / 10)
            {
                sb.AppendLine($"{times[i]:F3}       {states[i, 0]:F4}");
            }

            StepResponseResult = sb.ToString();
        }
        catch (Exception ex)
        {
            StepResponseResult = $"Error: {ex.Message}";
        }
    }

    private void ExportPidCode()
    {
        var sb = new StringBuilder();
        sb.AppendLine("// C# PID Controller Implementation");
        sb.AppendLine("var pid = new PidController(");
        sb.AppendLine($"    kp: {PidKp},");
        sb.AppendLine($"    ki: {PidKi},");
        sb.AppendLine($"    kd: {PidKd}");
        sb.AppendLine(");");
        sb.AppendLine($"pid.DerivativeFilterTf = {PidDerivativeFilterTf};");
        sb.AppendLine($"pid.SetOutputLimits({PidOutputLimit});");
        sb.AppendLine($"pid.SetIntegratorLimits({PidIntegratorLimit});");
        sb.AppendLine();
        sb.AppendLine("// Usage:");
        sb.AppendLine("double output = pid.Compute(setpoint, measurement, dt);");

        try
        {
            System.Windows.Clipboard.SetText(sb.ToString());
            PidResult = "Code copied to clipboard!\n\n" + sb.ToString();
        }
        catch
        {
            PidResult = sb.ToString();
        }
    }

    private static Matrix<double> ParseMatrix(string input)
    {
        var rows = input.Split(';', StringSplitOptions.RemoveEmptyEntries);
        var data = rows.Select(row =>
            row.Split(',', StringSplitOptions.RemoveEmptyEntries)
               .Select(s => double.Parse(s.Trim()))
               .ToArray()
        ).ToArray();

        int m = data.Length;
        int n = data[0].Length;
        var matrix = Matrix<double>.Build.Dense(m, n);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                matrix[i, j] = data[i][j];
            }
        }
        return matrix;
    }

    private static Matrix<double> ParseDiagonalMatrix(string input)
    {
        var values = input.Split(',', StringSplitOptions.RemoveEmptyEntries)
            .Select(s => double.Parse(s.Trim()))
            .ToArray();
        return MatrixUtilities.Diagonal(values);
    }
}
