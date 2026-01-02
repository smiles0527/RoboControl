using System.Windows.Input;
using ControlWorkbench.Math.Metrics;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// ViewModel for the Logs/Metrics tab.
/// </summary>
public class LogsViewModel : ViewModelBase
{
    private string _logFilePath = "";
    private string _selectedSignal = "Roll";
    private double _stepTarget = 1.0;
    private double _initialValue = 0.0;
    private string _metricsResult = "";
    private string _logContent = "";

    // Sample data for demonstration
    private double[]? _times;
    private double[]? _values;

    public LogsViewModel()
    {
        OpenLogCommand = new RelayCommand(OpenLog);
        ComputeMetricsCommand = new RelayCommand(ComputeMetrics, () => _times != null && _values != null);
        ExportMetricsCommand = new RelayCommand(ExportMetrics);
        GenerateSampleDataCommand = new RelayCommand(GenerateSampleData);
    }

    // Commands
    public ICommand OpenLogCommand { get; }
    public ICommand ComputeMetricsCommand { get; }
    public ICommand ExportMetricsCommand { get; }
    public ICommand GenerateSampleDataCommand { get; }

    // Properties
    public string LogFilePath
    {
        get => _logFilePath;
        set => SetProperty(ref _logFilePath, value);
    }

    public string[] AvailableSignals => ["Roll", "Pitch", "Yaw", "X Position", "Y Position", "Velocity", "Custom"];

    public string SelectedSignal
    {
        get => _selectedSignal;
        set => SetProperty(ref _selectedSignal, value);
    }

    public double StepTarget
    {
        get => _stepTarget;
        set => SetProperty(ref _stepTarget, value);
    }

    public double InitialValue
    {
        get => _initialValue;
        set => SetProperty(ref _initialValue, value);
    }

    public string MetricsResult
    {
        get => _metricsResult;
        set => SetProperty(ref _metricsResult, value);
    }

    public string LogContent
    {
        get => _logContent;
        set => SetProperty(ref _logContent, value);
    }

    public bool HasData => _times != null && _values != null;

    private void OpenLog()
    {
        var dialog = new Microsoft.Win32.OpenFileDialog
        {
            Filter = "Log Files (*.csv;*.bin)|*.csv;*.bin|All Files (*.*)|*.*",
            Title = "Open Telemetry Log"
        };

        if (dialog.ShowDialog() == true)
        {
            LogFilePath = dialog.FileName;
            LoadLogFile(LogFilePath);
        }
    }

    private void LoadLogFile(string path)
    {
        try
        {
            // For demonstration, generate sample data instead of actual file loading
            GenerateSampleData();
            LogContent = $"Loaded: {path}\n\nSample data generated for demonstration.\n{_times?.Length ?? 0} data points loaded.";
        }
        catch (Exception ex)
        {
            LogContent = $"Error loading file: {ex.Message}";
        }
    }

    private void GenerateSampleData()
    {
        // Generate a sample step response with some dynamics
        int n = 500;
        _times = new double[n];
        _values = new double[n];

        double dt = 0.01;
        double wn = 5.0;      // Natural frequency
        double zeta = 0.7;    // Damping ratio
        double target = StepTarget;

        // Simulate second-order system step response
        for (int i = 0; i < n; i++)
        {
            double t = i * dt;
            _times[i] = t;

            if (zeta < 1.0)
            {
                double wd = wn * System.Math.Sqrt(1 - zeta * zeta);
                double decay = System.Math.Exp(-zeta * wn * t);
                double phase = System.Math.Atan2(zeta, System.Math.Sqrt(1 - zeta * zeta));
                _values[i] = target * (1 - decay * System.Math.Cos(wd * t - phase) / System.Math.Sqrt(1 - zeta * zeta));
            }
            else
            {
                double s1 = -wn * (zeta + System.Math.Sqrt(zeta * zeta - 1));
                double s2 = -wn * (zeta - System.Math.Sqrt(zeta * zeta - 1));
                _values[i] = target * (1 + (s1 * System.Math.Exp(s2 * t) - s2 * System.Math.Exp(s1 * t)) / (s2 - s1));
            }

            // Add small noise
            _values[i] += (new Random(i).NextDouble() - 0.5) * 0.02 * target;
        }

        OnPropertyChanged(nameof(HasData));
        LogContent = $"Generated sample step response data:\n{n} points, dt = {dt}s, duration = {(n - 1) * dt}s\n?n = {wn} rad/s, ? = {zeta}";
    }

    private void ComputeMetrics()
    {
        if (_times == null || _values == null)
        {
            MetricsResult = "No data loaded. Click 'Generate Sample Data' first.";
            return;
        }

        try
        {
            var metrics = StepResponseAnalyzer.Analyze(_times, _values, StepTarget, InitialValue);
            MetricsResult = StepResponseAnalyzer.FormatMetrics(metrics);
        }
        catch (Exception ex)
        {
            MetricsResult = $"Error computing metrics: {ex.Message}";
        }
    }

    private void ExportMetrics()
    {
        if (string.IsNullOrEmpty(MetricsResult))
        {
            return;
        }

        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Text Files (*.txt)|*.txt|CSV Files (*.csv)|*.csv",
            Title = "Export Metrics"
        };

        if (dialog.ShowDialog() == true)
        {
            try
            {
                System.IO.File.WriteAllText(dialog.FileName, MetricsResult);
                MetricsResult += $"\n\nExported to: {dialog.FileName}";
            }
            catch (Exception ex)
            {
                MetricsResult += $"\n\nExport failed: {ex.Message}";
            }
        }
    }
}
