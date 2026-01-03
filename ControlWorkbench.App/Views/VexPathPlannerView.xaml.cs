using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace ControlWorkbench.App.Views;

/// <summary>
/// VEX Path Planner view with interactive field canvas.
/// </summary>
public partial class VexPathPlannerView : UserControl
{
    // Field dimensions (144" x 144" VRC field)
    private const double FieldSize = 144.0;
    private double _scale = 1.0;
    private Point _offset = new(0, 0);
    private readonly List<WaypointVisual> _waypoints = new();
    private WaypointVisual? _selectedWaypoint;
    private WaypointVisual? _draggingWaypoint;
    private bool _isDragging;
    private Point _lastMousePos;

    // Path visualization
    private readonly Path _pathLine;
    private readonly List<Ellipse> _waypointMarkers = new();

    public VexPathPlannerView()
    {
        InitializeComponent();

        _pathLine = new Path
        {
            Stroke = new SolidColorBrush(Color.FromRgb(0, 200, 255)),
            StrokeThickness = 3,
            StrokeDashArray = new DoubleCollection { 5, 3 }
        };

        Loaded += OnLoaded;
        SizeChanged += OnSizeChanged;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        DrawField();
        AddSamplePath();
    }

    private void OnSizeChanged(object sender, SizeChangedEventArgs e)
    {
        DrawField();
        RedrawPath();
    }

    private void DrawField()
    {
        FieldCanvas.Children.Clear();

        double canvasSize = System.Math.Min(FieldCanvas.ActualWidth, FieldCanvas.ActualHeight);
        if (canvasSize <= 0) canvasSize = 500;

        _scale = canvasSize / FieldSize;
        _offset = new Point(
            (FieldCanvas.ActualWidth - canvasSize) / 2,
            (FieldCanvas.ActualHeight - canvasSize) / 2
        );

        // Field background
        var fieldBg = new Rectangle
        {
            Width = canvasSize,
            Height = canvasSize,
            Fill = new SolidColorBrush(Color.FromRgb(60, 60, 60)),
            Stroke = new SolidColorBrush(Color.FromRgb(100, 100, 100)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(fieldBg, _offset.X);
        Canvas.SetTop(fieldBg, _offset.Y);
        FieldCanvas.Children.Add(fieldBg);

        // Grid lines (12" squares)
        for (int i = 0; i <= 12; i++)
        {
            double pos = i * 12 * _scale;

            // Vertical line
            var vLine = new Line
            {
                X1 = _offset.X + pos,
                Y1 = _offset.Y,
                X2 = _offset.X + pos,
                Y2 = _offset.Y + canvasSize,
                Stroke = new SolidColorBrush(Color.FromRgb(80, 80, 80)),
                StrokeThickness = i == 6 ? 2 : 1
            };
            FieldCanvas.Children.Add(vLine);

            // Horizontal line
            var hLine = new Line
            {
                X1 = _offset.X,
                Y1 = _offset.Y + pos,
                X2 = _offset.X + canvasSize,
                Y2 = _offset.Y + pos,
                Stroke = new SolidColorBrush(Color.FromRgb(80, 80, 80)),
                StrokeThickness = i == 6 ? 2 : 1
            };
            FieldCanvas.Children.Add(hLine);
        }

        // Draw High Stakes field elements
        DrawHighStakesElements(canvasSize);

        // Re-add path line
        FieldCanvas.Children.Add(_pathLine);

        // Re-add waypoint markers
        foreach (var marker in _waypointMarkers)
        {
            FieldCanvas.Children.Add(marker);
        }
    }

    private void DrawHighStakesElements(double canvasSize)
    {
        // Alliance stakes
        DrawStake(12, 72, Colors.Red, "Alliance Stake");
        DrawStake(132, 72, Colors.Blue, "Alliance Stake");

        // Neutral stakes
        DrawStake(72, 24, Colors.Gray, "Neutral");
        DrawStake(72, 120, Colors.Gray, "Neutral");

        // Wall stakes
        DrawWallStake(36, 0);
        DrawWallStake(108, 0);
        DrawWallStake(36, 144);
        DrawWallStake(108, 144);

        // Corners
        DrawCorner(0, 0, Colors.Red);
        DrawCorner(144, 144, Colors.Blue);
        DrawCorner(0, 144, Colors.Red);
        DrawCorner(144, 0, Colors.Blue);

        // Center ladder
        DrawLadder(72, 72);

        // Starting positions
        DrawStartPosition(18, 18, 45, Colors.Red, "Red +");
        DrawStartPosition(18, 126, -45, Colors.Red, "Red -");
        DrawStartPosition(126, 126, -135, Colors.Blue, "Blue +");
        DrawStartPosition(126, 18, 135, Colors.Blue, "Blue -");
    }

    private void DrawStake(double x, double y, Color color, string label)
    {
        var stake = new Ellipse
        {
            Width = 8 * _scale,
            Height = 8 * _scale,
            Fill = new SolidColorBrush(color),
            Stroke = Brushes.White,
            StrokeThickness = 2
        };
        Canvas.SetLeft(stake, _offset.X + x * _scale - stake.Width / 2);
        Canvas.SetTop(stake, _offset.Y + (FieldSize - y) * _scale - stake.Height / 2);
        FieldCanvas.Children.Add(stake);
    }

    private void DrawWallStake(double x, double y)
    {
        var stake = new Rectangle
        {
            Width = 6 * _scale,
            Height = 4 * _scale,
            Fill = Brushes.Orange,
            Stroke = Brushes.White,
            StrokeThickness = 1
        };
        Canvas.SetLeft(stake, _offset.X + x * _scale - stake.Width / 2);
        Canvas.SetTop(stake, _offset.Y + (FieldSize - y) * _scale - stake.Height / 2);
        FieldCanvas.Children.Add(stake);
    }

    private void DrawCorner(double x, double y, Color color)
    {
        var corner = new Polygon
        {
            Fill = new SolidColorBrush(Color.FromArgb(100, color.R, color.G, color.B)),
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2
        };

        double size = 24 * _scale;
        double cx = _offset.X + x * _scale;
        double cy = _offset.Y + (FieldSize - y) * _scale;

        // Determine corner orientation
        if (x < 72 && y < 72)
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx + size, cy), new(cx, cy - size) };
        }
        else if (x > 72 && y > 72)
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx - size, cy), new(cx, cy + size) };
        }
        else if (x < 72 && y > 72)
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx + size, cy), new(cx, cy + size) };
        }
        else
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx - size, cy), new(cx, cy - size) };
        }

        FieldCanvas.Children.Add(corner);
    }

    private void DrawLadder(double x, double y)
    {
        var ladder = new Rectangle
        {
            Width = 24 * _scale,
            Height = 24 * _scale,
            Fill = new SolidColorBrush(Color.FromArgb(80, 255, 215, 0)),
            Stroke = Brushes.Gold,
            StrokeThickness = 2
        };
        Canvas.SetLeft(ladder, _offset.X + x * _scale - ladder.Width / 2);
        Canvas.SetTop(ladder, _offset.Y + (FieldSize - y) * _scale - ladder.Height / 2);
        FieldCanvas.Children.Add(ladder);
    }

    private void DrawStartPosition(double x, double y, double heading, Color color, string label)
    {
        // Robot outline
        var robot = new Rectangle
        {
            Width = 18 * _scale,
            Height = 18 * _scale,
            Fill = Brushes.Transparent,
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2,
            StrokeDashArray = new DoubleCollection { 4, 2 }
        };

        var transform = new RotateTransform(heading, robot.Width / 2, robot.Height / 2);
        robot.RenderTransform = transform;

        Canvas.SetLeft(robot, _offset.X + x * _scale - robot.Width / 2);
        Canvas.SetTop(robot, _offset.Y + (FieldSize - y) * _scale - robot.Height / 2);
        FieldCanvas.Children.Add(robot);

        // Label
        var text = new TextBlock
        {
            Text = label,
            Foreground = new SolidColorBrush(color),
            FontSize = 10,
            FontWeight = FontWeights.Bold
        };
        Canvas.SetLeft(text, _offset.X + x * _scale - 15);
        Canvas.SetTop(text, _offset.Y + (FieldSize - y) * _scale + 12 * _scale);
        FieldCanvas.Children.Add(text);
    }

    private void AddSamplePath()
    {
        // Sample path for demonstration
        _waypoints.Clear();
        _waypoints.Add(new WaypointVisual { X = 18, Y = 18, Heading = 45, Velocity = 80 });
        _waypoints.Add(new WaypointVisual { X = 36, Y = 36, Heading = 45, Velocity = 60 });
        _waypoints.Add(new WaypointVisual { X = 48, Y = 60, Heading = 60, Velocity = 50 });
        _waypoints.Add(new WaypointVisual { X = 48, Y = 84, Heading = 90, Velocity = 40 });
        _waypoints.Add(new WaypointVisual { X = 24, Y = 96, Heading = 135, Velocity = 60 });

        RedrawPath();
    }

    private void RedrawPath()
    {
        // Clear existing markers
        foreach (var marker in _waypointMarkers)
        {
            FieldCanvas.Children.Remove(marker);
        }
        _waypointMarkers.Clear();

        if (_waypoints.Count < 2)
        {
            _pathLine.Data = null;
            return;
        }

        // Draw path line
        var geometry = new PathGeometry();
        var figure = new PathFigure
        {
            StartPoint = FieldToCanvas(_waypoints[0].X, _waypoints[0].Y)
        };

        // Use Bezier curves for smooth path
        for (int i = 1; i < _waypoints.Count; i++)
        {
            var prev = _waypoints[i - 1];
            var curr = _waypoints[i];

            var p0 = FieldToCanvas(prev.X, prev.Y);
            var p3 = FieldToCanvas(curr.X, curr.Y);

            // Control points based on heading
            double dist = System.Math.Sqrt(System.Math.Pow(curr.X - prev.X, 2) + System.Math.Pow(curr.Y - prev.Y, 2));
            double cp = dist * 0.4;

            var p1 = new Point(
                p0.X + cp * _scale * System.Math.Cos((90 - prev.Heading) * System.Math.PI / 180),
                p0.Y - cp * _scale * System.Math.Sin((90 - prev.Heading) * System.Math.PI / 180)
            );
            var p2 = new Point(
                p3.X - cp * _scale * System.Math.Cos((90 - curr.Heading) * System.Math.PI / 180),
                p3.Y + cp * _scale * System.Math.Sin((90 - curr.Heading) * System.Math.PI / 180)
            );

            figure.Segments.Add(new BezierSegment(p1, p2, p3, true));
        }

        geometry.Figures.Add(figure);
        _pathLine.Data = geometry;

        // Draw waypoint markers
        for (int i = 0; i < _waypoints.Count; i++)
        {
            var wp = _waypoints[i];
            var marker = new Ellipse
            {
                Width = 16,
                Height = 16,
                Fill = i == 0 ? Brushes.LimeGreen :
                       i == _waypoints.Count - 1 ? Brushes.Red :
                       Brushes.Yellow,
                Stroke = Brushes.White,
                StrokeThickness = 2,
                Cursor = Cursors.Hand,
                Tag = i
            };

            var pos = FieldToCanvas(wp.X, wp.Y);
            Canvas.SetLeft(marker, pos.X - 8);
            Canvas.SetTop(marker, pos.Y - 8);

            marker.MouseLeftButtonDown += Marker_MouseLeftButtonDown;
            marker.MouseMove += Marker_MouseMove;
            marker.MouseLeftButtonUp += Marker_MouseLeftButtonUp;

            _waypointMarkers.Add(marker);
            FieldCanvas.Children.Add(marker);

            // Draw heading arrow
            var arrow = new Line
            {
                X1 = pos.X,
                Y1 = pos.Y,
                X2 = pos.X + 20 * System.Math.Cos((90 - wp.Heading) * System.Math.PI / 180),
                Y2 = pos.Y - 20 * System.Math.Sin((90 - wp.Heading) * System.Math.PI / 180),
                Stroke = Brushes.White,
                StrokeThickness = 2
            };
            FieldCanvas.Children.Add(arrow);
        }

        UpdateCodePreview();
    }

    private void Marker_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        if (sender is Ellipse ellipse && ellipse.Tag is int index)
        {
            _draggingWaypoint = _waypoints[index];
            _selectedWaypoint = _draggingWaypoint;
            _isDragging = true;
            _lastMousePos = e.GetPosition(FieldCanvas);
            ellipse.CaptureMouse();

            // Update UI with selected waypoint
            WaypointX.Text = _selectedWaypoint.X.ToString("F1");
            WaypointY.Text = _selectedWaypoint.Y.ToString("F1");
            WaypointHeading.Text = _selectedWaypoint.Heading.ToString("F0");
            WaypointVel.Text = _selectedWaypoint.Velocity.ToString("F0");
        }
    }

    private void Marker_MouseMove(object sender, MouseEventArgs e)
    {
        if (_isDragging && _draggingWaypoint != null)
        {
            var currentPos = e.GetPosition(FieldCanvas);
            var fieldPos = CanvasToField(currentPos.X, currentPos.Y);

            _draggingWaypoint.X = System.Math.Clamp(fieldPos.X, 0, FieldSize);
            _draggingWaypoint.Y = System.Math.Clamp(fieldPos.Y, 0, FieldSize);

            WaypointX.Text = _draggingWaypoint.X.ToString("F1");
            WaypointY.Text = _draggingWaypoint.Y.ToString("F1");

            RedrawPath();
        }
    }

    private void Marker_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        if (sender is Ellipse ellipse)
        {
            ellipse.ReleaseMouseCapture();
        }
        _isDragging = false;
        _draggingWaypoint = null;
    }

    private Point FieldToCanvas(double x, double y)
    {
        return new Point(
            _offset.X + x * _scale,
            _offset.Y + (FieldSize - y) * _scale
        );
    }

    private Point CanvasToField(double canvasX, double canvasY)
    {
        return new Point(
            (canvasX - _offset.X) / _scale,
            FieldSize - (canvasY - _offset.Y) / _scale
        );
    }

    private void UpdateCodePreview()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("// LemLib path");
        sb.AppendLine("ASSET(path_txt) = {");

        foreach (var wp in _waypoints)
        {
            sb.AppendLine($"    {wp.X:F1}, {wp.Y:F1}, {wp.Heading:F0},");
        }

        sb.AppendLine("};");
        sb.AppendLine();
        sb.AppendLine("void run_path() {");
        sb.AppendLine("    chassis.follow(");
        sb.AppendLine($"        path_txt, {LookaheadSlider.Value:F0}, 4000);");
        sb.AppendLine("}");

        CodePreview.Text = sb.ToString();
    }
}

public class WaypointVisual
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Heading { get; set; }
    public double Velocity { get; set; } = 60;
    public List<string> Actions { get; } = new();
}
