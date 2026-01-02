using System.Text;
using System.Windows.Input;
using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math;
using ControlWorkbench.Math.Filters;
using ControlWorkbench.Math.Jacobians;
using ControlWorkbench.Math.Models;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// ViewModel for the Math Workbench tab (EKF/Odometry).
/// </summary>
public class MathWorkbenchViewModel : ViewModelBase
{
    // Motion Model
    private MotionModelType _selectedMotionModelType = MotionModelType.Unicycle2D;
    private IMotionModel? _motionModel;

    // Measurement Model
    private MeasurementModelType _selectedMeasurementModelType = MeasurementModelType.GpsPosition2D;
    private IMeasurementModel? _measurementModel;

    // EKF Parameters
    private double _dt = 0.1;
    private string _stateInput = "0, 0, 0";
    private string _covarianceInput = "1, 0, 0; 0, 1, 0; 0, 0, 0.1";
    private string _processNoiseInput = "0.01, 0.01";
    private string _inputVectorInput = "1.0, 0.1";

    // Measurement Parameters
    private string _measurementInput = "0.1, 0.1";
    private string _measurementNoiseInput = "4, 4";
    private string _beaconPositionInput = "10, 10";

    // Results
    private string _predictionResult = "";
    private string _updateResult = "";
    private string _jacobianValidationResult = "";

    // Complementary Filter
    private double _cfSampleTime = 0.01;
    private double _cfCutoffFrequency = 0.5;
    private string _cfResult = "";

    public MathWorkbenchViewModel()
    {
        ComputePredictionCommand = new RelayCommand(ComputePrediction);
        ComputeUpdateCommand = new RelayCommand(ComputeUpdate);
        ValidateJacobiansCommand = new RelayCommand(ValidateJacobians);
        ExportResultsCommand = new RelayCommand(ExportResults);
        DesignComplementaryFilterCommand = new RelayCommand(DesignComplementaryFilter);
        ResetToDefaultsCommand = new RelayCommand(ResetToDefaults);

        UpdateMotionModel();
        UpdateMeasurementModel();
    }

    // Commands
    public ICommand ComputePredictionCommand { get; }
    public ICommand ComputeUpdateCommand { get; }
    public ICommand ValidateJacobiansCommand { get; }
    public ICommand ExportResultsCommand { get; }
    public ICommand DesignComplementaryFilterCommand { get; }
    public ICommand ResetToDefaultsCommand { get; }

    // Motion Model Properties
    public MotionModelType[] MotionModelTypes => Enum.GetValues<MotionModelType>();

    public MotionModelType SelectedMotionModelType
    {
        get => _selectedMotionModelType;
        set
        {
            if (SetProperty(ref _selectedMotionModelType, value))
            {
                UpdateMotionModel();
            }
        }
    }

    public string MotionModelDescription => _motionModel switch
    {
        Unicycle2DModel => "State: [px, py, ?], Input: [v, ?]\nf(x,u,dt) = [px + v·cos(?)·dt, py + v·sin(?)·dt, ? + ?·dt]",
        ConstantVelocity2DModel => "State: [px, py, vx, vy], Input: [ax, ay]\nf(x,u,dt) = [px + vx·dt + ½ax·dt², py + vy·dt + ½ay·dt², vx + ax·dt, vy + ay·dt]",
        YawOnlyStrapdownModel => "State: [yaw, bias_g], Input: [gyro_z]\nf(x,u,dt) = [yaw + (gyro - bias)·dt, bias]",
        _ => "Select a motion model"
    };

    public string StateNames => _motionModel != null ? string.Join(", ", _motionModel.StateNames) : "";
    public string InputNames => _motionModel != null ? string.Join(", ", _motionModel.InputNames) : "";

    // Measurement Model Properties
    public MeasurementModelType[] MeasurementModelTypes => Enum.GetValues<MeasurementModelType>();

    public MeasurementModelType SelectedMeasurementModelType
    {
        get => _selectedMeasurementModelType;
        set
        {
            if (SetProperty(ref _selectedMeasurementModelType, value))
            {
                UpdateMeasurementModel();
            }
        }
    }

    public string MeasurementModelDescription => _measurementModel switch
    {
        GpsPosition2DModel => "Measures: [px, py]\nh(x) = [x[0], x[1]]",
        YawMeasurementModel => "Measures: [yaw]\nh(x) = [x[2]] (or yaw index)",
        RangeBearingModel => "Measures: [range, bearing] to beacon\nrange = ?((px-bx)² + (py-by)²)\nbearing = atan2(py-by, px-bx) - ?",
        _ => "Select a measurement model"
    };

    public bool ShowBeaconPosition => SelectedMeasurementModelType == MeasurementModelType.RangeBearing;

    // Input Properties
    public double Dt
    {
        get => _dt;
        set => SetProperty(ref _dt, value);
    }

    public string StateInput
    {
        get => _stateInput;
        set => SetProperty(ref _stateInput, value);
    }

    public string CovarianceInput
    {
        get => _covarianceInput;
        set => SetProperty(ref _covarianceInput, value);
    }

    public string ProcessNoiseInput
    {
        get => _processNoiseInput;
        set => SetProperty(ref _processNoiseInput, value);
    }

    public string InputVectorInput
    {
        get => _inputVectorInput;
        set => SetProperty(ref _inputVectorInput, value);
    }

    public string MeasurementInput
    {
        get => _measurementInput;
        set => SetProperty(ref _measurementInput, value);
    }

    public string MeasurementNoiseInput
    {
        get => _measurementNoiseInput;
        set => SetProperty(ref _measurementNoiseInput, value);
    }

    public string BeaconPositionInput
    {
        get => _beaconPositionInput;
        set => SetProperty(ref _beaconPositionInput, value);
    }

    // Result Properties
    public string PredictionResult
    {
        get => _predictionResult;
        set => SetProperty(ref _predictionResult, value);
    }

    public string UpdateResult
    {
        get => _updateResult;
        set => SetProperty(ref _updateResult, value);
    }

    public string JacobianValidationResult
    {
        get => _jacobianValidationResult;
        set => SetProperty(ref _jacobianValidationResult, value);
    }

    // Complementary Filter Properties
    public double CfSampleTime
    {
        get => _cfSampleTime;
        set => SetProperty(ref _cfSampleTime, value);
    }

    public double CfCutoffFrequency
    {
        get => _cfCutoffFrequency;
        set => SetProperty(ref _cfCutoffFrequency, value);
    }

    public string CfResult
    {
        get => _cfResult;
        set => SetProperty(ref _cfResult, value);
    }

    private void UpdateMotionModel()
    {
        _motionModel = MotionModelFactory.Create(SelectedMotionModelType);
        OnPropertyChanged(nameof(MotionModelDescription));
        OnPropertyChanged(nameof(StateNames));
        OnPropertyChanged(nameof(InputNames));
        ResetToDefaults();
    }

    private void UpdateMeasurementModel()
    {
        _measurementModel = MeasurementModelFactory.Create(SelectedMeasurementModelType);
        OnPropertyChanged(nameof(MeasurementModelDescription));
        OnPropertyChanged(nameof(ShowBeaconPosition));
    }

    private void ResetToDefaults()
    {
        if (_motionModel == null) return;

        var defaultState = _motionModel.DefaultInitialState;
        StateInput = string.Join(", ", defaultState.ToArray().Select(v => v.ToString("F4")));

        int n = _motionModel.StateDimension;
        CovarianceInput = FormatDiagonalMatrix(n, 1.0);

        var defaultQ = _motionModel.DefaultProcessNoise;
        ProcessNoiseInput = string.Join(", ", Enumerable.Range(0, defaultQ.RowCount).Select(i => defaultQ[i, i].ToString("F4")));

        InputVectorInput = string.Join(", ", Enumerable.Repeat("0", _motionModel.InputDimension));
    }

    private static string FormatDiagonalMatrix(int n, double value)
    {
        var rows = new List<string>();
        for (int i = 0; i < n; i++)
        {
            var row = new List<string>();
            for (int j = 0; j < n; j++)
            {
                row.Add(i == j ? value.ToString("F4") : "0");
            }
            rows.Add(string.Join(", ", row));
        }
        return string.Join("; ", rows);
    }

    private void ComputePrediction()
    {
        try
        {
            if (_motionModel == null)
            {
                PredictionResult = "Error: No motion model selected.";
                return;
            }

            var state = ParseVector(StateInput);
            var P = ParseMatrix(CovarianceInput);
            var Q = ParseDiagonalMatrix(ProcessNoiseInput);
            var u = ParseVector(InputVectorInput);

            var result = ExtendedKalmanFilter.ComputePrediction(_motionModel, state, P, u, Dt, Q);

            var sb = new StringBuilder();
            sb.AppendLine("=== EKF Prediction Result ===");
            sb.AppendLine();
            sb.AppendLine("Predicted State x?:");
            sb.AppendLine(MatrixUtilities.Format(result.PredictedState));
            sb.AppendLine();
            sb.AppendLine("State Transition Jacobian F:");
            sb.AppendLine(MatrixUtilities.Format(result.F));
            sb.AppendLine();
            sb.AppendLine("Process Noise Jacobian G:");
            sb.AppendLine(MatrixUtilities.Format(result.G));
            sb.AppendLine();
            sb.AppendLine("Predicted Covariance P?:");
            sb.AppendLine(MatrixUtilities.Format(result.PredictedCovariance));

            PredictionResult = sb.ToString();
        }
        catch (Exception ex)
        {
            PredictionResult = $"Error: {ex.Message}";
        }
    }

    private void ComputeUpdate()
    {
        try
        {
            if (_measurementModel == null)
            {
                UpdateResult = "Error: No measurement model selected.";
                return;
            }

            var state = ParseVector(StateInput);
            var P = ParseMatrix(CovarianceInput);
            var z = ParseVector(MeasurementInput);
            var R = ParseDiagonalMatrix(MeasurementNoiseInput);

            object? measurementParams = null;
            if (SelectedMeasurementModelType == MeasurementModelType.RangeBearing)
            {
                var beacon = ParseVector(BeaconPositionInput);
                measurementParams = new RangeBearingParameters(beacon[0], beacon[1]);
            }

            var result = ExtendedKalmanFilter.ComputeUpdate(_measurementModel, state, P, z, R, measurementParams);

            var sb = new StringBuilder();
            sb.AppendLine("=== EKF Update Result ===");
            sb.AppendLine();
            sb.AppendLine("Measurement Jacobian H:");
            sb.AppendLine(MatrixUtilities.Format(result.H));
            sb.AppendLine();
            sb.AppendLine("Innovation y = z - h(x):");
            sb.AppendLine(MatrixUtilities.Format(result.Innovation));
            sb.AppendLine();
            sb.AppendLine("Innovation Covariance S = H·P·H? + R:");
            sb.AppendLine(MatrixUtilities.Format(result.InnovationCovariance));
            sb.AppendLine();
            sb.AppendLine("Kalman Gain K = P·H?·S?¹:");
            sb.AppendLine(MatrixUtilities.Format(result.KalmanGain));
            sb.AppendLine();
            sb.AppendLine("Updated State x? = x? + K·y:");
            sb.AppendLine(MatrixUtilities.Format(result.UpdatedState));
            sb.AppendLine();
            sb.AppendLine("Updated Covariance P?:");
            sb.AppendLine(MatrixUtilities.Format(result.UpdatedCovariance));

            UpdateResult = sb.ToString();
        }
        catch (Exception ex)
        {
            UpdateResult = $"Error: {ex.Message}";
        }
    }

    private void ValidateJacobians()
    {
        try
        {
            if (_motionModel == null)
            {
                JacobianValidationResult = "Error: No motion model selected.";
                return;
            }

            var state = ParseVector(StateInput);
            var u = ParseVector(InputVectorInput);

            // Get analytic Jacobian
            var analyticResult = _motionModel.Predict(state, u, Dt);
            var analyticF = analyticResult.F;

            // Compute numerical Jacobian
            Vector<double> f(Vector<double> x) => _motionModel.Predict(x, u, Dt).PredictedState;
            var numericalF = NumericalJacobian.Compute(f, state);

            double maxDiff = NumericalJacobian.MaxAbsoluteDifference(analyticF, numericalF);
            bool isValid = maxDiff < 1e-5;

            var sb = new StringBuilder();
            sb.AppendLine("=== Jacobian Validation ===");
            sb.AppendLine();
            sb.AppendLine("Analytic F:");
            sb.AppendLine(MatrixUtilities.Format(analyticF));
            sb.AppendLine();
            sb.AppendLine("Numerical F (finite differences):");
            sb.AppendLine(MatrixUtilities.Format(numericalF));
            sb.AppendLine();
            sb.AppendLine($"Maximum absolute difference: {maxDiff:E4}");
            sb.AppendLine(isValid ? "? Jacobian validation PASSED" : "? Jacobian validation FAILED");

            JacobianValidationResult = sb.ToString();
        }
        catch (Exception ex)
        {
            JacobianValidationResult = $"Error: {ex.Message}";
        }
    }

    private void DesignComplementaryFilter()
    {
        try
        {
            var design = new ComplementaryFilterDesign
            {
                SampleTime = CfSampleTime,
                CutoffFrequencyHz = CfCutoffFrequency
            };

            var validation = design.Validate();

            var sb = new StringBuilder();
            sb.AppendLine("=== Complementary Filter Design ===");
            sb.AppendLine();
            sb.AppendLine($"Sample Time: {design.SampleTime * 1000:F2} ms ({1.0 / design.SampleTime:F1} Hz)");
            sb.AppendLine($"Cutoff Frequency: {design.CutoffFrequencyHz:F2} Hz");
            sb.AppendLine($"Time Constant ?: {design.TimeConstant:F4} s");
            sb.AppendLine($"Alpha (gyro weight): {design.Alpha:F4}");
            sb.AppendLine($"Nyquist Frequency: {design.NyquistFrequency:F2} Hz");
            sb.AppendLine();
            sb.AppendLine($"Filter Equation:");
            sb.AppendLine($"?[k] = ?·(?[k-1] + gyro·dt) + (1-?)·accel_angle");
            sb.AppendLine($"     = {design.Alpha:F4}·(?[k-1] + gyro·{design.SampleTime:F4}) + {1 - design.Alpha:F4}·accel_angle");
            sb.AppendLine();

            if (validation.IsValid)
                sb.AppendLine("? " + validation.Message);
            else
                sb.AppendLine("? " + validation.Message);

            CfResult = sb.ToString();
        }
        catch (Exception ex)
        {
            CfResult = $"Error: {ex.Message}";
        }
    }

    private void ExportResults()
    {
        // In a full implementation, this would save to file
        // For now, just combine all results
        var sb = new StringBuilder();
        sb.AppendLine("=== Export ===");
        sb.AppendLine(PredictionResult);
        sb.AppendLine(UpdateResult);
        sb.AppendLine(JacobianValidationResult);
        sb.AppendLine(CfResult);
        
        // Copy to clipboard
        try
        {
            System.Windows.Clipboard.SetText(sb.ToString());
        }
        catch { }
    }

    private static Vector<double> ParseVector(string input)
    {
        var values = input.Split(',', StringSplitOptions.RemoveEmptyEntries)
            .Select(s => double.Parse(s.Trim()))
            .ToArray();
        return Vector<double>.Build.DenseOfArray(values);
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
