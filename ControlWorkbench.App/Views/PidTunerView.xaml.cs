using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace ControlWorkbench.App.Views;

/// <summary>
/// PID tuning interface with real-time graphing and analysis.
/// </summary>
public partial class PidTunerView : UserControl
{
    // Data buffers for graphing
    private readonly List<double> _setpointData = new();
    private readonly List<double> _measurementData = new();
    private readonly List<double> _errorData = new();
    private readonly List<double> _pTermData = new();
    private readonly List<double> _iTermData = new();
    private readonly List<double> _dTermData = new();
    private readonly List<double> _outputData = new();
    private readonly List<double> _timeData = new();
    
    private const int MaxDataPoints = 500;
    
    // Current gains
    private double _kP = 10.0;
    private double _kI = 0.0;
    private double _kD = 30.0;
    
    // Test state
    private bool _isRunningTest = false;
    private readonly DispatcherTimer _testTimer;
    private double _testTime = 0;
    private double _testSetpoint = 0;
    private double _testMeasurement = 0;
    
    // PID state for simulation
    private double _integral = 0;
    private double _prevError = 0;
    
    // Metrics
    private double _riseTime = 0;
    private double _settleTime = 0;
    private double _overshoot = 0;
    private double _steadyStateError = 0;
    private double _peakTime = 0;
    
    // Prevent recursive updates
    private bool _isUpdating = false;
    
    public PidTunerView()
    {
        InitializeComponent();
        
        _testTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(20)
        };
        _testTimer.Tick += TestTimer_Tick;
        
        Loaded += OnLoaded;
        SizeChanged += OnSizeChanged;
    }
    
    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        UpdateGainDisplay();
        DrawGraphs();
    }
    
    private void OnSizeChanged(object sender, SizeChangedEventArgs e)
    {
        DrawGraphs();
    }
    
    private void UpdateGainDisplay()
    {
        if (KpSlider == null || KiSlider == null || KdSlider == null ||
            KpText == null || KiText == null || KdText == null) return;
            
        _isUpdating = true;
        
        KpSlider.Value = _kP;
        KiSlider.Value = _kI;
        KdSlider.Value = _kD;
        
        KpText.Text = _kP.ToString("F2");
        KiText.Text = _kI.ToString("F2");
        KdText.Text = _kD.ToString("F2");
        
        _isUpdating = false;
    }
    
    private void Gain_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (_isUpdating || KpSlider == null || KiSlider == null || KdSlider == null ||
            KpText == null || KiText == null || KdText == null) return;
        
        _kP = KpSlider.Value;
        _kI = KiSlider.Value;
        _kD = KdSlider.Value;
        
        _isUpdating = true;
        KpText.Text = _kP.ToString("F2");
        KiText.Text = _kI.ToString("F2");
        KdText.Text = _kD.ToString("F2");
        _isUpdating = false;
    }
    
    private void GainText_Changed(object sender, TextChangedEventArgs e)
    {
        if (_isUpdating || KpSlider == null || KiSlider == null || KdSlider == null ||
            KpText == null || KiText == null || KdText == null) return;
        
        if (double.TryParse(KpText.Text, out double kp))
        {
            _kP = System.Math.Clamp(kp, 0, 100);
            _isUpdating = true;
            KpSlider.Value = _kP;
            _isUpdating = false;
        }
        
        if (double.TryParse(KiText.Text, out double ki))
        {
            _kI = System.Math.Clamp(ki, 0, 10);
            _isUpdating = true;
            KiSlider.Value = _kI;
            _isUpdating = false;
        }
        
        if (double.TryParse(KdText.Text, out double kd))
        {
            _kD = System.Math.Clamp(kd, 0, 200);
            _isUpdating = true;
            KdSlider.Value = _kD;
            _isUpdating = false;
        }
    }
    
    private void ControllerList_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (ControllerList == null) return;
        
        // Load gains for selected controller
        var selected = ControllerList.SelectedItem as ListBoxItem;
        if (selected == null) return;
        
        string name = selected.Content?.ToString() ?? "";
        
        // Demo values for different controllers
        switch (name)
        {
            case "lateral":
                _kP = 10; _kI = 0; _kD = 30;
                break;
            case "angular":
                _kP = 2; _kI = 0; _kD = 10;
                break;
            case "drive_ff":
                _kP = 0.5; _kI = 0; _kD = 2;
                break;
            case "lift":
                _kP = 5; _kI = 0.1; _kD = 15;
                break;
            case "intake":
                _kP = 1; _kI = 0; _kD = 0;
                break;
        }
        
        UpdateGainDisplay();
    }
    
    private void StartTest_Click(object sender, RoutedEventArgs e)
    {
        if (_isRunningTest) return;
        
        // Clear previous data
        ClearData();
        
        // Reset PID state
        _integral = 0;
        _prevError = 0;
        _testTime = 0;
        _testMeasurement = 0;
        
        // Set test setpoint based on mode
        var mode = (TestModeCombo.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "Step Response";
        _testSetpoint = mode switch
        {
            "Step Response" => 100,
            "Ramp" => 0,
            "Sine Wave" => 0,
            "Square Wave" => 100,
            _ => 100
        };
        
        _isRunningTest = true;
        StartTestBtn.IsEnabled = false;
        StopTestBtn.IsEnabled = true;
        _testTimer.Start();
    }
    
    private void StopTest_Click(object sender, RoutedEventArgs e)
    {
        _testTimer.Stop();
        _isRunningTest = false;
        StartTestBtn.IsEnabled = true;
        StopTestBtn.IsEnabled = false;
        
        CalculateMetrics();
    }
    
    private void TestTimer_Tick(object? sender, EventArgs e)
    {
        double dt = 0.02; // 20ms
        _testTime += dt;
        
        // Update setpoint based on test mode
        var mode = (TestModeCombo.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "Step Response";
        _testSetpoint = mode switch
        {
            "Step Response" => 100,
            "Ramp" => System.Math.Min(_testTime * 50, 100),
            "Sine Wave" => 50 + 50 * System.Math.Sin(_testTime * 2 * System.Math.PI * 0.5),
            "Square Wave" => ((int)(_testTime * 2) % 2 == 0) ? 100 : 0,
            _ => 100
        };
        
        // Simulate system response with PID
        double error = _testSetpoint - _testMeasurement;
        _integral += error * dt;
        double derivative = (error - _prevError) / dt;
        
        double pTerm = _kP * error;
        double iTerm = _kI * _integral;
        double dTerm = _kD * derivative;
        double output = pTerm + iTerm + dTerm;
        
        // Simulate first-order system response
        // dy/dt = (output - y) / tau
        double tau = 0.1; // System time constant
        double systemGain = 1.0;
        _testMeasurement += (output * systemGain * 0.01 - _testMeasurement * 0.1) * dt / tau;
        _testMeasurement = System.Math.Clamp(_testMeasurement, -150, 150);
        
        _prevError = error;
        
        // Store data
        _timeData.Add(_testTime);
        _setpointData.Add(_testSetpoint);
        _measurementData.Add(_testMeasurement);
        _errorData.Add(error);
        _pTermData.Add(pTerm);
        _iTermData.Add(iTerm);
        _dTermData.Add(dTerm);
        _outputData.Add(output);
        
        // Limit buffer size
        if (_timeData.Count > MaxDataPoints)
        {
            _timeData.RemoveAt(0);
            _setpointData.RemoveAt(0);
            _measurementData.RemoveAt(0);
            _errorData.RemoveAt(0);
            _pTermData.RemoveAt(0);
            _iTermData.RemoveAt(0);
            _dTermData.RemoveAt(0);
            _outputData.RemoveAt(0);
        }
        
        // Update live values
        LiveSetpointText.Text = _testSetpoint.ToString("F1");
        LiveMeasurementText.Text = _testMeasurement.ToString("F1");
        LiveErrorText.Text = error.ToString("F2");
        LiveOutputText.Text = output.ToString("F1");
        
        DrawGraphs();
        
        // Auto-stop after 5 seconds
        if (_testTime >= 5.0)
        {
            StopTest_Click(null, null!);
        }
    }
    
    private void ClearData()
    {
        _timeData.Clear();
        _setpointData.Clear();
        _measurementData.Clear();
        _errorData.Clear();
        _pTermData.Clear();
        _iTermData.Clear();
        _dTermData.Clear();
        _outputData.Clear();
    }
    
    private void DrawGraphs()
    {
        if (ResponseCanvas == null || TermsCanvas == null) return;
        
        DrawResponseGraph();
        DrawTermsGraph();
    }
    
    private void DrawResponseGraph()
    {
        ResponseCanvas.Children.Clear();
        
        double width = ResponseCanvas.ActualWidth;
        double height = ResponseCanvas.ActualHeight;
        if (width <= 0 || height <= 0) return;
        
        // Draw grid
        DrawGrid(ResponseCanvas, width, height);
        
        if (_timeData.Count < 2) return;
        
        // Find data range
        double maxTime = _timeData.Count > 0 ? _timeData.Max() : 5;
        double minY = -20;
        double maxY = 120;
        
        // Draw setpoint
        DrawLine(ResponseCanvas, _timeData, _setpointData, width, height, 
                 maxTime, minY, maxY, Color.FromRgb(76, 175, 80));
        
        // Draw measurement
        DrawLine(ResponseCanvas, _timeData, _measurementData, width, height,
                 maxTime, minY, maxY, Color.FromRgb(33, 150, 243));
        
        // Draw error (scaled)
        var scaledError = _errorData.Select(e => e * 0.5 + 50).ToList();
        DrawLine(ResponseCanvas, _timeData, scaledError, width, height,
                 maxTime, minY, maxY, Color.FromRgb(255, 107, 107), 1, true);
    }
    
    private void DrawTermsGraph()
    {
        TermsCanvas.Children.Clear();
        
        double width = TermsCanvas.ActualWidth;
        double height = TermsCanvas.ActualHeight;
        if (width <= 0 || height <= 0) return;
        
        DrawGrid(TermsCanvas, width, height);
        
        if (_timeData.Count < 2) return;
        
        double maxTime = _timeData.Max();
        
        // Find output range
        double maxVal = 127;
        double minVal = -127;
        if (_outputData.Count > 0)
        {
            maxVal = System.Math.Max(System.Math.Abs(_outputData.Max()), System.Math.Abs(_outputData.Min()));
            maxVal = System.Math.Max(maxVal, 50);
            minVal = -maxVal;
        }
        
        // Draw terms
        DrawLine(TermsCanvas, _timeData, _pTermData, width, height, maxTime, minVal, maxVal, Color.FromRgb(76, 175, 80));
        DrawLine(TermsCanvas, _timeData, _iTermData, width, height, maxTime, minVal, maxVal, Color.FromRgb(33, 150, 243));
        DrawLine(TermsCanvas, _timeData, _dTermData, width, height, maxTime, minVal, maxVal, Color.FromRgb(255, 152, 0));
        DrawLine(TermsCanvas, _timeData, _outputData, width, height, maxTime, minVal, maxVal, Colors.White, 2);
    }
    
    private void DrawGrid(Canvas canvas, double width, double height)
    {
        // Horizontal lines
        for (int i = 0; i <= 4; i++)
        {
            double y = height * i / 4;
            var line = new Line
            {
                X1 = 0, Y1 = y, X2 = width, Y2 = y,
                Stroke = new SolidColorBrush(Color.FromRgb(50, 50, 50)),
                StrokeThickness = 1
            };
            canvas.Children.Add(line);
        }
        
        // Vertical lines
        for (int i = 0; i <= 10; i++)
        {
            double x = width * i / 10;
            var line = new Line
            {
                X1 = x, Y1 = 0, X2 = x, Y2 = height,
                Stroke = new SolidColorBrush(Color.FromRgb(50, 50, 50)),
                StrokeThickness = 1
            };
            canvas.Children.Add(line);
        }
    }
    
    private void DrawLine(Canvas canvas, List<double> xData, List<double> yData,
                          double width, double height, double maxX, double minY, double maxY,
                          Color color, double thickness = 2, bool dashed = false)
    {
        if (xData.Count < 2 || yData.Count < 2) return;
        
        var polyline = new Polyline
        {
            Stroke = new SolidColorBrush(color),
            StrokeThickness = thickness
        };
        
        if (dashed)
            polyline.StrokeDashArray = new DoubleCollection { 4, 2 };
        
        for (int i = 0; i < System.Math.Min(xData.Count, yData.Count); i++)
        {
            double x = (xData[i] / maxX) * width;
            double y = height - ((yData[i] - minY) / (maxY - minY)) * height;
            y = System.Math.Clamp(y, 0, height);
            polyline.Points.Add(new Point(x, y));
        }
        
        canvas.Children.Add(polyline);
    }
    
    private void CalculateMetrics()
    {
        if (_setpointData.Count < 10 || _measurementData.Count < 10) return;
        
        double target = _setpointData.Last();
        if (System.Math.Abs(target) < 0.01) return;
        
        // Rise time (10% to 90%)
        double threshold10 = target * 0.1;
        double threshold90 = target * 0.9;
        int idx10 = _measurementData.FindIndex(v => v >= threshold10);
        int idx90 = _measurementData.FindIndex(v => v >= threshold90);
        
        if (idx10 >= 0 && idx90 >= 0 && idx90 > idx10)
            _riseTime = _timeData[idx90] - _timeData[idx10];
        
        // Overshoot
        double peak = _measurementData.Max();
        _overshoot = System.Math.Max(0, (peak - target) / target * 100);
        
        // Peak time
        int peakIdx = _measurementData.IndexOf(peak);
        if (peakIdx >= 0)
            _peakTime = _timeData[peakIdx];
        
        // Settle time (within 2% of target)
        double settleThreshold = target * 0.02;
        _settleTime = 0;
        for (int i = _measurementData.Count - 1; i >= 0; i--)
        {
            if (System.Math.Abs(_measurementData[i] - target) > settleThreshold)
            {
                if (i < _timeData.Count - 1)
                    _settleTime = _timeData[i + 1];
                break;
            }
        }
        
        // Steady-state error
        if (_measurementData.Count > 10)
        {
            double avgLast = _measurementData.Skip(_measurementData.Count - 10).Average();
            _steadyStateError = System.Math.Abs((target - avgLast) / target * 100);
        }
        
        // Update display
        RiseTimeText.Text = $"{_riseTime:F2}s";
        SettleTimeText.Text = $"{_settleTime:F2}s";
        OvershootText.Text = $"{_overshoot:F1}%";
        OvershootText.Foreground = _overshoot > 20 ? Brushes.OrangeRed : 
                                    _overshoot > 10 ? new SolidColorBrush(Color.FromRgb(255, 152, 0)) :
                                    new SolidColorBrush(Color.FromRgb(76, 175, 80));
        SteadyStateText.Text = $"{_steadyStateError:F1}%";
        PeakTimeText.Text = $"{_peakTime:F2}s";
        
        // Calculate quality score
        CalculateQualityScore();
    }
    
    private void CalculateQualityScore()
    {
        int score = 100;
        
        // Penalize overshoot
        if (_overshoot > 30) score -= 30;
        else if (_overshoot > 20) score -= 20;
        else if (_overshoot > 10) score -= 10;
        
        // Penalize slow rise
        if (_riseTime > 1.0) score -= 20;
        else if (_riseTime > 0.5) score -= 10;
        
        // Penalize slow settle
        if (_settleTime > 2.0) score -= 20;
        else if (_settleTime > 1.0) score -= 10;
        
        // Penalize steady-state error
        if (_steadyStateError > 5) score -= 20;
        else if (_steadyStateError > 2) score -= 10;
        
        string grade;
        string description;
        Color color;
        
        if (score >= 90) { grade = "A+"; description = "Excellent tuning!"; color = Color.FromRgb(76, 175, 80); }
        else if (score >= 80) { grade = "A"; description = "Great response"; color = Color.FromRgb(76, 175, 80); }
        else if (score >= 70) { grade = "B+"; description = "Good response, slight overshoot"; color = Color.FromRgb(139, 195, 74); }
        else if (score >= 60) { grade = "B"; description = "Acceptable, room for improvement"; color = Color.FromRgb(255, 193, 7); }
        else if (score >= 50) { grade = "C"; description = "Needs tuning adjustments"; color = Color.FromRgb(255, 152, 0); }
        else { grade = "D"; description = "Poor response, retune recommended"; color = Color.FromRgb(244, 67, 54); }
        
        QualityScore.Text = grade;
        QualityScore.Foreground = new SolidColorBrush(color);
        QualityDescription.Text = description;
        
        UpdateSuggestions();
    }
    
    private void UpdateSuggestions()
    {
        SuggestionsList.Children.Clear();
        
        if (_overshoot > 15)
        {
            AddSuggestion("?? Reduce overshoot", "Increase kD by 20% or reduce kP by 10%", Color.FromRgb(255, 152, 0));
        }
        
        if (_riseTime > 0.5)
        {
            AddSuggestion("?? Slow rise time", "Increase kP for faster response", Color.FromRgb(255, 152, 0));
        }
        
        if (_steadyStateError > 2)
        {
            AddSuggestion("?? Steady-state error", "Add small kI (0.01-0.1) to eliminate offset", Color.FromRgb(255, 152, 0));
        }
        
        if (_overshoot < 10 && _riseTime < 0.4)
        {
            AddSuggestion("? Good response", "Tuning looks good for competition!", Color.FromRgb(76, 175, 80));
        }
    }
    
    private void AddSuggestion(string title, string description, Color color)
    {
        var border = new Border
        {
            Background = new SolidColorBrush(Color.FromRgb(62, 62, 66)),
            CornerRadius = new CornerRadius(3),
            Padding = new Thickness(8),
            Margin = new Thickness(0, 0, 0, 5)
        };
        
        var stack = new StackPanel();
        stack.Children.Add(new TextBlock
        {
            Text = title,
            Foreground = new SolidColorBrush(color),
            FontWeight = FontWeights.Bold
        });
        stack.Children.Add(new TextBlock
        {
            Text = description,
            Foreground = new SolidColorBrush(Color.FromRgb(136, 136, 136)),
            FontSize = 11,
            TextWrapping = TextWrapping.Wrap
        });
        
        border.Child = stack;
        SuggestionsList.Children.Add(border);
    }
    
    // Preset buttons
    private void PresetConservative_Click(object sender, RoutedEventArgs e)
    {
        _kP = 5; _kI = 0; _kD = 20;
        UpdateGainDisplay();
    }
    
    private void PresetBalanced_Click(object sender, RoutedEventArgs e)
    {
        _kP = 10; _kI = 0.05; _kD = 30;
        UpdateGainDisplay();
    }
    
    private void PresetAggressive_Click(object sender, RoutedEventArgs e)
    {
        _kP = 20; _kI = 0.1; _kD = 50;
        UpdateGainDisplay();
    }
    
    private void SendGains_Click(object sender, RoutedEventArgs e)
    {
        // TODO: Send gains to robot via telemetry
        var controller = (ControllerList.SelectedItem as ListBoxItem)?.Content?.ToString() ?? "lateral";
        MessageBox.Show($"Sending gains to '{controller}':\nkP = {_kP:F2}\nkI = {_kI:F2}\nkD = {_kD:F2}",
                        "Send Gains", MessageBoxButton.OK, MessageBoxImage.Information);
    }
    
    private void FetchGains_Click(object sender, RoutedEventArgs e)
    {
        // TODO: Fetch gains from robot
        MessageBox.Show("Fetching gains from robot...", "Fetch Gains", MessageBoxButton.OK, MessageBoxImage.Information);
    }
    
    private void ClearGraph_Click(object sender, RoutedEventArgs e)
    {
        ClearData();
        DrawGraphs();
    }
    
    private void ExportCsv_Click(object sender, RoutedEventArgs e)
    {
        var sb = new StringBuilder();
        sb.AppendLine("Time,Setpoint,Measurement,Error,P,I,D,Output");
        
        for (int i = 0; i < _timeData.Count; i++)
        {
            sb.AppendLine($"{_timeData[i]:F3},{_setpointData[i]:F2},{_measurementData[i]:F2}," +
                          $"{_errorData[i]:F2},{_pTermData[i]:F2},{_iTermData[i]:F2}," +
                          $"{_dTermData[i]:F2},{_outputData[i]:F2}");
        }
        
        Clipboard.SetText(sb.ToString());
        MessageBox.Show("CSV data copied to clipboard!", "Export", MessageBoxButton.OK, MessageBoxImage.Information);
    }
    
    private void AutoTune_Click(object sender, RoutedEventArgs e)
    {
        var method = (AutoTuneMethod.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "Ziegler-Nichols";
        MessageBox.Show($"Auto-tune using {method} method...\n\nThis would run on the connected robot.", 
                        "Auto-Tune", MessageBoxButton.OK, MessageBoxImage.Information);
    }
    
    private void SaveProfile_Click(object sender, RoutedEventArgs e)
    {
        var name = ProfileCombo.Text;
        MessageBox.Show($"Profile '{name}' saved.", "Save Profile", MessageBoxButton.OK, MessageBoxImage.Information);
    }
    
    private void LoadProfile_Click(object sender, RoutedEventArgs e)
    {
        var name = (ProfileCombo.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "Default";
        MessageBox.Show($"Loading profile '{name}'...", "Load Profile", MessageBoxButton.OK, MessageBoxImage.Information);
    }
    
    // Public method to receive PID data from robot
    public void UpdatePidData(double setpoint, double measurement, double error,
                               double pTerm, double iTerm, double dTerm, double output, double time)
    {
        _timeData.Add(time);
        _setpointData.Add(setpoint);
        _measurementData.Add(measurement);
        _errorData.Add(error);
        _pTermData.Add(pTerm);
        _iTermData.Add(iTerm);
        _dTermData.Add(dTerm);
        _outputData.Add(output);
        
        if (_timeData.Count > MaxDataPoints)
        {
            _timeData.RemoveAt(0);
            _setpointData.RemoveAt(0);
            _measurementData.RemoveAt(0);
            _errorData.RemoveAt(0);
            _pTermData.RemoveAt(0);
            _iTermData.RemoveAt(0);
            _dTermData.RemoveAt(0);
            _outputData.RemoveAt(0);
        }
        
        LiveSetpointText.Text = setpoint.ToString("F1");
        LiveMeasurementText.Text = measurement.ToString("F1");
        LiveErrorText.Text = error.ToString("F2");
        LiveOutputText.Text = output.ToString("F1");
        
        DrawGraphs();
    }
}
