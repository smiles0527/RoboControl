using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Live robot visualization with real-time field view.
/// </summary>
public partial class LiveRobotView : UserControl
{
    // Field dimensions (144" x 144" VRC field)
    private const double FieldSize = 144.0;
    
    // Canvas transformation
    private double _scale = 1.0;
    private Point _offset = new(0, 0);
    
    // Robot state
    private double _robotX = 24;
    private double _robotY = 24;
    private double _robotHeading = 45;
    private double _linearVelocity = 0;
    private double _angularVelocity = 0;
    
    // Trail history
    private readonly List<Point> _trailPoints = new();
    private const int MaxTrailPoints = 500;
    
    // Planned path
    private readonly List<Point> _plannedPath = new();
    
    // Target point
    private Point? _targetPoint;
    private Point? _lookaheadPoint;
    
    // Visual elements
    private Ellipse? _robotVisual;
    private Line? _robotHeadingArrow;
    private Polyline? _trailLine;
    private Polyline? _plannedPathLine;
    private Ellipse? _targetVisual;
    private Ellipse? _lookaheadVisual;
    
    // Debug values
    public ObservableCollection<DebugValue> DebugValues { get; } = new();
    
    // Update timer
    private readonly DispatcherTimer _updateTimer;
    private bool _isConnected = false;
    private DateTime _lastUpdate = DateTime.MinValue;
    
    public LiveRobotView()
    {
        InitializeComponent();
        
        DebugValuesList.ItemsSource = DebugValues;
        
        _updateTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(50) // 20 Hz UI update
        };
        _updateTimer.Tick += UpdateTimer_Tick;
        
        Loaded += OnLoaded;
        SizeChanged += OnSizeChanged;
    }
    
    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        DrawField();
        CreateRobotVisuals();
        AddSampleData();
        _updateTimer.Start();
    }
    
    private void OnSizeChanged(object sender, SizeChangedEventArgs e)
    {
        DrawField();
        UpdateRobotPosition();
    }
    
    private void RecalculateTransform()
    {
        double canvasWidth = FieldCanvas.ActualWidth;
        double canvasHeight = FieldCanvas.ActualHeight;
        
        if (canvasWidth < 100) canvasWidth = 500;
        if (canvasHeight < 100) canvasHeight = 500;
        
        double margin = 20;
        double availableSize = System.Math.Min(canvasWidth, canvasHeight) - margin * 2;
        
        _scale = availableSize / FieldSize;
        _offset = new Point(
            (canvasWidth - availableSize) / 2,
            (canvasHeight - availableSize) / 2
        );
    }
    
    private Point FieldToCanvas(double x, double y)
    {
        return new Point(
            _offset.X + x * _scale,
            _offset.Y + (FieldSize - y) * _scale
        );
    }
    
    private void DrawField()
    {
        FieldCanvas.Children.Clear();
        RecalculateTransform();
        
        double fieldPixelSize = FieldSize * _scale;
        
        // Field background
        var fieldBg = new Rectangle
        {
            Width = fieldPixelSize,
            Height = fieldPixelSize,
            Fill = new SolidColorBrush(Color.FromRgb(55, 55, 55)),
            Stroke = new SolidColorBrush(Color.FromRgb(150, 150, 150)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(fieldBg, _offset.X);
        Canvas.SetTop(fieldBg, _offset.Y);
        FieldCanvas.Children.Add(fieldBg);
        
        // Grid lines
        for (int i = 0; i <= 12; i++)
        {
            double pos = i * 12 * _scale;
            bool isMajor = i == 6;
            
            var stroke = isMajor 
                ? new SolidColorBrush(Color.FromRgb(100, 100, 100))
                : new SolidColorBrush(Color.FromRgb(70, 70, 70));
            
            var vLine = new Line
            {
                X1 = _offset.X + pos, Y1 = _offset.Y,
                X2 = _offset.X + pos, Y2 = _offset.Y + fieldPixelSize,
                Stroke = stroke, StrokeThickness = isMajor ? 2 : 1
            };
            FieldCanvas.Children.Add(vLine);
            
            var hLine = new Line
            {
                X1 = _offset.X, Y1 = _offset.Y + pos,
                X2 = _offset.X + fieldPixelSize, Y2 = _offset.Y + pos,
                Stroke = stroke, StrokeThickness = isMajor ? 2 : 1
            };
            FieldCanvas.Children.Add(hLine);
        }
        
        // Mobile goals
        DrawMobileGoal(36, 36);
        DrawMobileGoal(108, 36);
        DrawMobileGoal(36, 108);
        DrawMobileGoal(108, 108);
        DrawMobileGoal(72, 72);
        
        // Corner zones
        DrawCornerZone(0, 0, Colors.Red);
        DrawCornerZone(144, 144, Colors.Red);
        DrawCornerZone(0, 144, Colors.Blue);
        DrawCornerZone(144, 0, Colors.Blue);
    }
    
    private void DrawMobileGoal(double x, double y)
    {
        double size = 7 * _scale;
        var pos = FieldToCanvas(x, y);
        
        var mogo = new Ellipse
        {
            Width = size, Height = size,
            Fill = new SolidColorBrush(Color.FromRgb(255, 215, 0)),
            Stroke = new SolidColorBrush(Color.FromRgb(200, 160, 0)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(mogo, pos.X - size / 2);
        Canvas.SetTop(mogo, pos.Y - size / 2);
        FieldCanvas.Children.Add(mogo);
    }
    
    private void DrawCornerZone(double x, double y, Color color)
    {
        double size = 24 * _scale;
        var pos = FieldToCanvas(x, y);
        
        var corner = new Polygon
        {
            Fill = new SolidColorBrush(Color.FromArgb(60, color.R, color.G, color.B)),
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2
        };
        
        if (x < 72 && y < 72)
            corner.Points = new PointCollection { new(pos.X, pos.Y), new(pos.X + size, pos.Y), new(pos.X, pos.Y - size) };
        else if (x > 72 && y > 72)
            corner.Points = new PointCollection { new(pos.X, pos.Y), new(pos.X - size, pos.Y), new(pos.X, pos.Y + size) };
        else if (x < 72 && y > 72)
            corner.Points = new PointCollection { new(pos.X, pos.Y), new(pos.X + size, pos.Y), new(pos.X, pos.Y + size) };
        else
            corner.Points = new PointCollection { new(pos.X, pos.Y), new(pos.X - size, pos.Y), new(pos.X, pos.Y - size) };
        
        FieldCanvas.Children.Add(corner);
    }
    
    private void CreateRobotVisuals()
    {
        // Trail line
        _trailLine = new Polyline
        {
            Stroke = new SolidColorBrush(Color.FromRgb(255, 107, 107)),
            StrokeThickness = 2,
            Opacity = 0.7
        };
        FieldCanvas.Children.Add(_trailLine);
        
        // Planned path line
        _plannedPathLine = new Polyline
        {
            Stroke = new SolidColorBrush(Color.FromRgb(0, 200, 255)),
            StrokeThickness = 3,
            StrokeDashArray = new DoubleCollection { 4, 2 }
        };
        FieldCanvas.Children.Add(_plannedPathLine);
        
        // Target point
        _targetVisual = new Ellipse
        {
            Width = 16, Height = 16,
            Fill = new SolidColorBrush(Color.FromRgb(255, 215, 0)),
            Stroke = Brushes.White, StrokeThickness = 2,
            Visibility = Visibility.Collapsed
        };
        Panel.SetZIndex(_targetVisual, 90);
        FieldCanvas.Children.Add(_targetVisual);
        
        // Lookahead point
        _lookaheadVisual = new Ellipse
        {
            Width = 10, Height = 10,
            Fill = new SolidColorBrush(Color.FromRgb(255, 165, 0)),
            Visibility = Visibility.Collapsed
        };
        Panel.SetZIndex(_lookaheadVisual, 91);
        FieldCanvas.Children.Add(_lookaheadVisual);
        
        // Robot body
        double robotSize = 18 * _scale;
        _robotVisual = new Ellipse
        {
            Width = robotSize, Height = robotSize,
            Fill = new SolidColorBrush(Color.FromRgb(76, 175, 80)),
            Stroke = Brushes.White, StrokeThickness = 3
        };
        Panel.SetZIndex(_robotVisual, 100);
        FieldCanvas.Children.Add(_robotVisual);
        
        // Robot heading arrow
        _robotHeadingArrow = new Line
        {
            Stroke = Brushes.White,
            StrokeThickness = 3,
            StrokeEndLineCap = PenLineCap.Triangle
        };
        Panel.SetZIndex(_robotHeadingArrow, 101);
        FieldCanvas.Children.Add(_robotHeadingArrow);
    }
    
    private void UpdateRobotPosition()
    {
        if (_robotVisual == null || _robotHeadingArrow == null) return;
        
        var pos = FieldToCanvas(_robotX, _robotY);
        double robotSize = 18 * _scale;
        
        // Update robot position
        _robotVisual.Width = robotSize;
        _robotVisual.Height = robotSize;
        Canvas.SetLeft(_robotVisual, pos.X - robotSize / 2);
        Canvas.SetTop(_robotVisual, pos.Y - robotSize / 2);
        
        // Update heading arrow
        double arrowLen = robotSize * 0.8;
        double radians = (_robotHeading - 90) * System.Math.PI / 180;
        _robotHeadingArrow.X1 = pos.X;
        _robotHeadingArrow.Y1 = pos.Y;
        _robotHeadingArrow.X2 = pos.X + arrowLen * System.Math.Cos(radians);
        _robotHeadingArrow.Y2 = pos.Y + arrowLen * System.Math.Sin(radians);
        
        // Update trail
        if (ShowTrailCheck?.IsChecked == true && _trailLine != null)
        {
            _trailLine.Visibility = Visibility.Visible;
            _trailPoints.Add(new Point(_robotX, _robotY));
            if (_trailPoints.Count > MaxTrailPoints)
                _trailPoints.RemoveAt(0);
            
            _trailLine.Points.Clear();
            foreach (var pt in _trailPoints)
            {
                _trailLine.Points.Add(FieldToCanvas(pt.X, pt.Y));
            }
        }
        else if (_trailLine != null)
        {
            _trailLine.Visibility = Visibility.Collapsed;
        }
        
        // Update planned path
        if (ShowPlannedPathCheck?.IsChecked == true && _plannedPathLine != null && _plannedPath.Count > 0)
        {
            _plannedPathLine.Visibility = Visibility.Visible;
            _plannedPathLine.Points.Clear();
            foreach (var pt in _plannedPath)
            {
                _plannedPathLine.Points.Add(FieldToCanvas(pt.X, pt.Y));
            }
        }
        else if (_plannedPathLine != null)
        {
            _plannedPathLine.Visibility = Visibility.Collapsed;
        }
        
        // Update target
        if (_targetPoint.HasValue && _targetVisual != null)
        {
            var targetPos = FieldToCanvas(_targetPoint.Value.X, _targetPoint.Value.Y);
            Canvas.SetLeft(_targetVisual, targetPos.X - 8);
            Canvas.SetTop(_targetVisual, targetPos.Y - 8);
            _targetVisual.Visibility = Visibility.Visible;
        }
        
        // Update lookahead
        if (_lookaheadPoint.HasValue && _lookaheadVisual != null)
        {
            var laPos = FieldToCanvas(_lookaheadPoint.Value.X, _lookaheadPoint.Value.Y);
            Canvas.SetLeft(_lookaheadVisual, laPos.X - 5);
            Canvas.SetTop(_lookaheadVisual, laPos.Y - 5);
            _lookaheadVisual.Visibility = Visibility.Visible;
        }
        
        // Update UI texts
        PosXText.Text = _robotX.ToString("F2");
        PosYText.Text = _robotY.ToString("F2");
        HeadingText.Text = _robotHeading.ToString("F1");
        LinearVelText.Text = _linearVelocity.ToString("F1");
        AngularVelText.Text = _angularVelocity.ToString("F1");
    }
    
    private void AddSampleData()
    {
        // Sample planned path
        _plannedPath.Add(new Point(24, 24));
        _plannedPath.Add(new Point(36, 36));
        _plannedPath.Add(new Point(48, 60));
        _plannedPath.Add(new Point(55, 85));
        _plannedPath.Add(new Point(45, 105));
        
        _targetPoint = new Point(45, 105);
        _lookaheadPoint = new Point(36, 36);
        
        // Sample debug values
        DebugValues.Add(new DebugValue { Name = "left_motor_temp", Value = 42.5 });
        DebugValues.Add(new DebugValue { Name = "right_motor_temp", Value = 41.2 });
        DebugValues.Add(new DebugValue { Name = "intake_rpm", Value = 580.0 });
        DebugValues.Add(new DebugValue { Name = "lift_position", Value = 45.0 });
        
        UpdateRobotPosition();
    }
    
    private void UpdateTimer_Tick(object? sender, EventArgs e)
    {
        // Simulate robot movement for demo
        // In real use, this would receive data from the robot via telemetry
        
        // Demo: Move robot along path
        if (_plannedPath.Count > 0)
        {
            int pathIndex = (int)((DateTime.Now.Second / 2.0) % _plannedPath.Count);
            var target = _plannedPath[pathIndex];
            
            double dx = target.X - _robotX;
            double dy = target.Y - _robotY;
            double dist = System.Math.Sqrt(dx * dx + dy * dy);
            
            if (dist > 0.5)
            {
                _robotX += dx * 0.05;
                _robotY += dy * 0.05;
                _robotHeading = System.Math.Atan2(dx, dy) * 180 / System.Math.PI;
                _linearVelocity = 45.0;
                _angularVelocity = 12.0;
            }
            else
            {
                _linearVelocity = 0;
                _angularVelocity = 0;
            }
            
            _lookaheadPoint = target;
            
            // Update path progress
            double progress = (pathIndex + 1.0) / _plannedPath.Count * 100;
            PathProgressText.Text = $"{progress:F0}%";
            PathProgressBar.Value = progress;
            LookaheadText.Text = $"({target.X:F1}, {target.Y:F1})";
            CrosstrackText.Text = $"{(dist * 0.3):F1} in";
        }
        
        UpdateRobotPosition();
        
        // Update connection status
        if (_isConnected)
        {
            var elapsed = DateTime.Now - _lastUpdate;
            if (elapsed.TotalSeconds < 1)
            {
                ConnectionDot.Fill = new SolidColorBrush(Color.FromRgb(76, 175, 80));
                ConnectionText.Text = "Connected";
                ConnectionText.Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80));
            }
            else
            {
                ConnectionDot.Fill = new SolidColorBrush(Color.FromRgb(255, 193, 7));
                ConnectionText.Text = "Stale";
                ConnectionText.Foreground = new SolidColorBrush(Color.FromRgb(255, 193, 7));
            }
            LastUpdateText.Text = $"Last update: {elapsed.TotalMilliseconds:F0}ms ago";
        }
    }
    
    // Public methods to receive telemetry data
    public void UpdateOdometry(double x, double y, double theta, double vx, double vy, double omega)
    {
        _robotX = x;
        _robotY = y;
        _robotHeading = theta * 180 / System.Math.PI;
        _linearVelocity = System.Math.Sqrt(vx * vx + vy * vy);
        _angularVelocity = omega * 180 / System.Math.PI;
        _lastUpdate = DateTime.Now;
        _isConnected = true;
    }
    
    public void UpdatePathProgress(double progress, double robotX, double robotY, double targetX, double targetY)
    {
        _targetPoint = new Point(targetX, targetY);
        PathProgressBar.Value = progress * 100;
        PathProgressText.Text = $"{progress * 100:F0}%";
    }
    
    public void UpdatePidState(string name, double error, double pTerm, double iTerm, double dTerm, double output)
    {
        PidNameText.Text = name;
        PidErrorText.Text = error.ToString("F3");
        PidPText.Text = pTerm.ToString("F3");
        PidIText.Text = iTerm.ToString("F3");
        PidDText.Text = dTerm.ToString("F3");
        PidOutputText.Text = output.ToString("F2");
    }
    
    public void UpdateBattery(double percent, double voltage)
    {
        BatteryBar.Value = percent;
        BatteryText.Text = $"{percent:F0}%";
        VoltageText.Text = $"{voltage:F2}V";
        
        if (percent < 20)
            BatteryBar.Foreground = new SolidColorBrush(Color.FromRgb(244, 67, 54));
        else if (percent < 50)
            BatteryBar.Foreground = new SolidColorBrush(Color.FromRgb(255, 193, 7));
        else
            BatteryBar.Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80));
    }
    
    public void SetPlannedPath(List<Point> path)
    {
        _plannedPath.Clear();
        _plannedPath.AddRange(path);
    }
    
    public void AppendLog(string message)
    {
        ConsoleText.Text += "\n" + message;
        ConsoleScroller.ScrollToEnd();
    }
    
    private void ClearTrail_Click(object sender, RoutedEventArgs e)
    {
        _trailPoints.Clear();
        _trailLine?.Points.Clear();
    }
    
    private void CenterRobot_Click(object sender, RoutedEventArgs e)
    {
        // Reset pan to center on robot (if pan was implemented)
        DrawField();
        CreateRobotVisuals();
        UpdateRobotPosition();
    }
}

public class DebugValue
{
    public string Name { get; set; } = "";
    public double Value { get; set; }
}
