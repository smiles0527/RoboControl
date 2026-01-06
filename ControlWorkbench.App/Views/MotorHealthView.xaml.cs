using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Motor health monitoring dashboard with temperature, current, and RPM tracking.
/// </summary>
public partial class MotorHealthView : UserControl
{
    // Motor data storage
    private readonly Dictionary<string, MotorData> _motors = new();
    
    // Temperature history for graphing
    private readonly Dictionary<string, List<double>> _tempHistory = new();
    private const int MaxHistoryPoints = 100;
    
    // Alert state
    private readonly List<MotorAlert> _alerts = new();
    
    // Update timer
    private readonly DispatcherTimer _updateTimer;
    private DateTime _startTime;
    private DateTime _lastUpdate;
    
    public MotorHealthView()
    {
        InitializeComponent();
        
        InitializeMotors();
        
        _updateTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(100)
        };
        _updateTimer.Tick += UpdateTimer_Tick;
        
        Loaded += OnLoaded;
        SizeChanged += OnSizeChanged;
    }
    
    private void InitializeMotors()
    {
        // Initialize motor data with default values
        string[] motorNames = { "LF", "LB", "RF", "RB", "Intake", "Lift" };
        
        foreach (var name in motorNames)
        {
            _motors[name] = new MotorData
            {
                Name = name,
                Temperature = 35 + Random.Shared.Next(10),
                Rpm = 0,
                Current = 0,
                Power = 0,
                IsConnected = true
            };
            
            _tempHistory[name] = new List<double>();
        }
    }
    
    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        _startTime = DateTime.Now;
        _lastUpdate = DateTime.Now;
        _updateTimer.Start();
        
        DrawTempHistory();
    }
    
    private void OnSizeChanged(object sender, SizeChangedEventArgs e)
    {
        DrawTempHistory();
    }
    
    private void UpdateTimer_Tick(object? sender, EventArgs e)
    {
        // Simulate motor data updates (in real use, this comes from telemetry)
        SimulateMotorData();
        
        // Update UI
        UpdateMotorCards();
        UpdateStatistics();
        UpdateAlerts();
        DrawTempHistory();
        
        // Update timing
        var elapsed = DateTime.Now - _lastUpdate;
        LastUpdateText.Text = $"Last update: {elapsed.TotalMilliseconds:F0}ms ago";
        _lastUpdate = DateTime.Now;
        
        var runtime = DateTime.Now - _startTime;
        RuntimeText.Text = $"{runtime.Minutes}:{runtime.Seconds:D2}";
    }
    
    private void SimulateMotorData()
    {
        // Simulate drive motors running
        double baseTemp = 40 + 5 * System.Math.Sin(DateTime.Now.Second * 0.1);
        double baseRpm = 580 + Random.Shared.Next(-10, 10);
        double baseCurrent = 1.2 + Random.Shared.NextDouble() * 0.3;
        
        _motors["LF"].Temperature = baseTemp + Random.Shared.Next(-2, 3);
        _motors["LF"].Rpm = baseRpm + Random.Shared.Next(-5, 5);
        _motors["LF"].Current = baseCurrent + Random.Shared.NextDouble() * 0.1;
        _motors["LF"].Power = 65;
        
        _motors["LB"].Temperature = baseTemp + 2 + Random.Shared.Next(-2, 3);
        _motors["LB"].Rpm = baseRpm + Random.Shared.Next(-5, 5);
        _motors["LB"].Current = baseCurrent + 0.1 + Random.Shared.NextDouble() * 0.1;
        _motors["LB"].Power = 65;
        
        _motors["RF"].Temperature = baseTemp + 10 + Random.Shared.Next(-2, 3); // Warmer
        _motors["RF"].Rpm = baseRpm + Random.Shared.Next(-5, 5);
        _motors["RF"].Current = baseCurrent + 0.3 + Random.Shared.NextDouble() * 0.1;
        _motors["RF"].Power = 65;
        
        _motors["RB"].Temperature = baseTemp + 1 + Random.Shared.Next(-2, 3);
        _motors["RB"].Rpm = baseRpm + Random.Shared.Next(-5, 5);
        _motors["RB"].Current = baseCurrent + Random.Shared.NextDouble() * 0.1;
        _motors["RB"].Power = 65;
        
        _motors["Intake"].Temperature = 38 + Random.Shared.Next(-2, 3);
        _motors["Intake"].Rpm = 600;
        _motors["Intake"].Current = 0.9;
        _motors["Intake"].Power = 80;
        
        _motors["Lift"].Temperature = 35 + Random.Shared.Next(-2, 3);
        _motors["Lift"].Rpm = 0;
        _motors["Lift"].Current = 0.1;
        _motors["Lift"].Power = 0;
        
        // Update temperature history
        foreach (var kvp in _motors)
        {
            var history = _tempHistory[kvp.Key];
            history.Add(kvp.Value.Temperature);
            if (history.Count > MaxHistoryPoints)
                history.RemoveAt(0);
        }
    }
    
    private void UpdateMotorCards()
    {
        // Left Front
        UpdateMotorCard("LF", LFTempText, LFRpmText, LFCurrentBar, LFCurrentText, 
                        LFPowerBar, LFPowerText, LFStatusDot, LFStatusText);
        
        // Left Back
        UpdateMotorCard("LB", LBTempText, LBRpmText, LBCurrentBar, LBCurrentText,
                        LBPowerBar, LBPowerText, LBStatusDot, LBStatusText);
        
        // Right Front
        UpdateMotorCard("RF", RFTempText, RFRpmText, RFCurrentBar, RFCurrentText,
                        RFPowerBar, RFPowerText, RFStatusDot, RFStatusText);
        
        // Right Back
        UpdateMotorCard("RB", RBTempText, RBRpmText, RBCurrentBar, RBCurrentText,
                        RBPowerBar, RBPowerText, RBStatusDot, RBStatusText);
        
        // Intake
        IntakeTempText.Text = $"{_motors["Intake"].Temperature:F0}°";
        IntakeRpmText.Text = $"{_motors["Intake"].Rpm:F0}";
        IntakeCurrentBar.Value = _motors["Intake"].Current * 30;
        IntakeCurrentText.Text = $"{_motors["Intake"].Current:F1}A";
        
        // Lift
        LiftTempText.Text = $"{_motors["Lift"].Temperature:F0}°";
        LiftRpmText.Text = $"{_motors["Lift"].Rpm:F0}";
    }
    
    private void UpdateMotorCard(string motorName, TextBlock tempText, TextBlock rpmText,
                                  ProgressBar currentBar, TextBlock currentText,
                                  ProgressBar powerBar, TextBlock powerText,
                                  Ellipse statusDot, TextBlock statusText)
    {
        var motor = _motors[motorName];
        
        tempText.Text = $"{motor.Temperature:F0}°";
        rpmText.Text = $"{motor.Rpm:F0}";
        currentBar.Value = motor.Current * 30;
        currentText.Text = $"{motor.Current:F1}A";
        powerBar.Value = motor.Power;
        powerText.Text = $"{motor.Power:F0}";
        
        // Temperature color
        Color tempColor;
        if (motor.Temperature < 45)
            tempColor = Color.FromRgb(76, 175, 80); // Green
        else if (motor.Temperature < 55)
            tempColor = Color.FromRgb(255, 152, 0); // Orange
        else
            tempColor = Color.FromRgb(244, 67, 54); // Red
        
        tempText.Foreground = new SolidColorBrush(tempColor);
        
        // Find the parent border with the temperature gauge
        var parent = tempText.Parent;
        while (parent != null && parent is not Border)
            parent = (parent as FrameworkElement)?.Parent;
        
        if (parent is Border border)
            border.BorderBrush = new SolidColorBrush(tempColor);
        
        // Status
        if (!motor.IsConnected)
        {
            statusDot.Fill = new SolidColorBrush(Color.FromRgb(244, 67, 54));
            statusText.Text = "Disconnected";
            statusText.Foreground = new SolidColorBrush(Color.FromRgb(244, 67, 54));
        }
        else if (motor.Temperature >= 55)
        {
            statusDot.Fill = new SolidColorBrush(Color.FromRgb(244, 67, 54));
            statusText.Text = "HOT!";
            statusText.Foreground = new SolidColorBrush(Color.FromRgb(244, 67, 54));
        }
        else if (motor.Temperature >= 45)
        {
            statusDot.Fill = new SolidColorBrush(Color.FromRgb(255, 152, 0));
            statusText.Text = "Warm";
            statusText.Foreground = new SolidColorBrush(Color.FromRgb(255, 152, 0));
        }
        else
        {
            statusDot.Fill = new SolidColorBrush(Color.FromRgb(76, 175, 80));
            statusText.Text = "Healthy";
            statusText.Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80));
        }
    }
    
    private void UpdateStatistics()
    {
        var driveMotors = new[] { _motors["LF"], _motors["LB"], _motors["RF"], _motors["RB"] };
        
        double totalPower = _motors.Values.Sum(m => m.Current * 12.0); // Approximate watts
        double avgTemp = driveMotors.Average(m => m.Temperature);
        double maxTemp = driveMotors.Max(m => m.Temperature);
        
        TotalPowerText.Text = $"{totalPower:F1}W";
        AvgTempText.Text = $"{avgTemp:F0}°C";
        MaxTempText.Text = $"{maxTemp:F0}°C";
        
        // Color max temp
        if (maxTemp >= 55)
            MaxTempText.Foreground = new SolidColorBrush(Color.FromRgb(244, 67, 54));
        else if (maxTemp >= 45)
            MaxTempText.Foreground = new SolidColorBrush(Color.FromRgb(255, 152, 0));
        else
            MaxTempText.Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80));
        
        // Overall status
        if (maxTemp >= 55 || _motors.Values.Any(m => !m.IsConnected))
        {
            OverallStatusBadge.Background = new SolidColorBrush(Color.FromRgb(244, 67, 54));
            OverallStatusText.Text = "Warning: Check Motors";
        }
        else if (maxTemp >= 45)
        {
            OverallStatusBadge.Background = new SolidColorBrush(Color.FromRgb(255, 152, 0));
            OverallStatusText.Text = "Caution: Motors Warm";
        }
        else
        {
            OverallStatusBadge.Background = new SolidColorBrush(Color.FromRgb(76, 175, 80));
            OverallStatusText.Text = "All Systems Normal";
        }
    }
    
    private void UpdateAlerts()
    {
        _alerts.Clear();
        
        foreach (var motor in _motors.Values)
        {
            if (!motor.IsConnected)
            {
                _alerts.Add(new MotorAlert
                {
                    Level = AlertLevel.Error,
                    Title = $"{motor.Name} disconnected",
                    Description = "Check motor connection and port"
                });
            }
            else if (motor.Temperature >= 55)
            {
                _alerts.Add(new MotorAlert
                {
                    Level = AlertLevel.Error,
                    Title = $"{motor.Name} overheating!",
                    Description = $"{motor.Temperature:F0}°C - Stop immediately"
                });
            }
            else if (motor.Temperature >= 45)
            {
                _alerts.Add(new MotorAlert
                {
                    Level = AlertLevel.Warning,
                    Title = $"{motor.Name} running warm",
                    Description = $"{motor.Temperature:F0}°C - Consider cooling time"
                });
            }
        }
        
        if (_alerts.Count == 0)
        {
            _alerts.Add(new MotorAlert
            {
                Level = AlertLevel.Info,
                Title = "All motors within limits",
                Description = "No action required"
            });
        }
        
        // Update alerts list UI
        AlertsList.Children.Clear();
        foreach (var alert in _alerts.Take(5))
        {
            var border = new Border
            {
                Background = new SolidColorBrush(Color.FromRgb(62, 62, 66)),
                CornerRadius = new CornerRadius(3),
                Padding = new Thickness(8),
                Margin = new Thickness(0, 0, 0, 5)
            };
            
            var stack = new StackPanel { Orientation = Orientation.Horizontal };
            
            string icon = alert.Level switch
            {
                AlertLevel.Error => "??",
                AlertLevel.Warning => "??",
                _ => "?"
            };
            
            Color titleColor = alert.Level switch
            {
                AlertLevel.Error => Color.FromRgb(244, 67, 54),
                AlertLevel.Warning => Color.FromRgb(255, 152, 0),
                _ => Color.FromRgb(76, 175, 80)
            };
            
            stack.Children.Add(new TextBlock
            {
                Text = icon,
                Margin = new Thickness(0, 0, 8, 0)
            });
            
            var innerStack = new StackPanel();
            innerStack.Children.Add(new TextBlock
            {
                Text = alert.Title,
                Foreground = new SolidColorBrush(titleColor),
                FontWeight = FontWeights.Bold,
                FontSize = 11
            });
            innerStack.Children.Add(new TextBlock
            {
                Text = alert.Description,
                Foreground = new SolidColorBrush(Color.FromRgb(136, 136, 136)),
                FontSize = 10
            });
            
            stack.Children.Add(innerStack);
            border.Child = stack;
            AlertsList.Children.Add(border);
        }
    }
    
    private void DrawTempHistory()
    {
        TempHistoryCanvas.Children.Clear();
        
        double width = TempHistoryCanvas.ActualWidth;
        double height = TempHistoryCanvas.ActualHeight;
        if (width <= 0 || height <= 0) return;
        
        // Draw grid
        for (int i = 0; i <= 4; i++)
        {
            double y = height * i / 4;
            var line = new Line
            {
                X1 = 0, Y1 = y, X2 = width, Y2 = y,
                Stroke = new SolidColorBrush(Color.FromRgb(50, 50, 50)),
                StrokeThickness = 1
            };
            TempHistoryCanvas.Children.Add(line);
        }
        
        // Temperature range
        double minTemp = 30;
        double maxTemp = 60;
        
        // Draw motor temperature lines
        var colors = new Dictionary<string, Color>
        {
            { "LF", Color.FromRgb(76, 175, 80) },
            { "RF", Color.FromRgb(33, 150, 243) },
            { "LB", Color.FromRgb(255, 152, 0) },
            { "RB", Color.FromRgb(156, 39, 176) }
        };
        
        foreach (var kvp in colors)
        {
            if (!_tempHistory.ContainsKey(kvp.Key)) continue;
            var history = _tempHistory[kvp.Key];
            if (history.Count < 2) continue;
            
            var polyline = new Polyline
            {
                Stroke = new SolidColorBrush(kvp.Value),
                StrokeThickness = 2
            };
            
            for (int i = 0; i < history.Count; i++)
            {
                double x = (double)i / MaxHistoryPoints * width;
                double y = height - ((history[i] - minTemp) / (maxTemp - minTemp)) * height;
                y = System.Math.Clamp(y, 0, height);
                polyline.Points.Add(new Point(x, y));
            }
            
            TempHistoryCanvas.Children.Add(polyline);
        }
    }
    
    // Public methods to receive motor data from telemetry
    public void UpdateMotor(int port, double temperature, double rpm, double current, double voltage, int power)
    {
        string name = port switch
        {
            1 => "LF",
            2 => "LB",
            5 => "Intake",
            6 => "Lift",
            11 => "RF",
            12 => "RB",
            _ => $"Port{port}"
        };
        
        if (!_motors.ContainsKey(name))
        {
            _motors[name] = new MotorData { Name = name };
            _tempHistory[name] = new List<double>();
        }
        
        _motors[name].Temperature = temperature;
        _motors[name].Rpm = rpm;
        _motors[name].Current = current;
        _motors[name].Power = power;
        _motors[name].IsConnected = true;
        
        _lastUpdate = DateTime.Now;
    }
    
    public void SetMotorDisconnected(int port)
    {
        string name = port switch
        {
            1 => "LF", 2 => "LB", 5 => "Intake", 6 => "Lift", 11 => "RF", 12 => "RB",
            _ => $"Port{port}"
        };
        
        if (_motors.ContainsKey(name))
            _motors[name].IsConnected = false;
    }
    
    private void ExportReport_Click(object sender, RoutedEventArgs e)
    {
        var sb = new StringBuilder();
        sb.AppendLine("Motor Health Report");
        sb.AppendLine($"Generated: {DateTime.Now}");
        sb.AppendLine($"Runtime: {RuntimeText.Text}");
        sb.AppendLine();
        sb.AppendLine("Motor,Temperature,RPM,Current,Power,Status");
        
        foreach (var motor in _motors.Values)
        {
            string status = motor.Temperature >= 55 ? "HOT" :
                           motor.Temperature >= 45 ? "Warm" : "OK";
            sb.AppendLine($"{motor.Name},{motor.Temperature:F1},{motor.Rpm:F0},{motor.Current:F2},{motor.Power},{status}");
        }
        
        sb.AppendLine();
        sb.AppendLine("Alerts:");
        foreach (var alert in _alerts)
        {
            sb.AppendLine($"[{alert.Level}] {alert.Title}: {alert.Description}");
        }
        
        Clipboard.SetText(sb.ToString());
        MessageBox.Show("Report copied to clipboard!", "Export Report", MessageBoxButton.OK, MessageBoxImage.Information);
    }
}

public class MotorData
{
    public string Name { get; set; } = "";
    public double Temperature { get; set; }
    public double Rpm { get; set; }
    public double Current { get; set; }
    public double Power { get; set; }
    public bool IsConnected { get; set; } = true;
}

public class MotorAlert
{
    public AlertLevel Level { get; set; }
    public string Title { get; set; } = "";
    public string Description { get; set; } = "";
}

public enum AlertLevel
{
    Info,
    Warning,
    Error
}
