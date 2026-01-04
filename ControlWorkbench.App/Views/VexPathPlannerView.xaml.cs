using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Microsoft.Win32;

namespace ControlWorkbench.App.Views;

/// <summary>
/// VEX Path Planner view with interactive field canvas.
/// </summary>
public partial class VexPathPlannerView : UserControl
{
    // Field dimensions (144" x 144" VRC field)
    private const double FieldSize = 144.0;
    private double _scale = 1.0;
    private double _zoom = 1.0;
    private Point _offset = new(0, 0);
    private Point _panOffset = new(0, 0);
    private readonly List<WaypointVisual> _waypoints = new();
    private WaypointVisual? _selectedWaypoint;
    private int _selectedWaypointIndex = -1;
    private WaypointVisual? _draggingWaypoint;
    private bool _isDraggingWaypoint;
    private bool _isPanningField;
    private Point _lastMousePos;
    private Point _contextMenuFieldPos;

    // Undo/Redo
    private readonly Stack<List<WaypointVisual>> _undoStack = new();
    private readonly Stack<List<WaypointVisual>> _redoStack = new();

    // Simulation
    private readonly DispatcherTimer _simTimer;
    private int _simIndex;
    private bool _isSimulating;

    // Path visualization
    private readonly Path _pathLine;

    public VexPathPlannerView()
    {
        InitializeComponent();

        _pathLine = new Path
        {
            Stroke = new SolidColorBrush(Color.FromRgb(0, 200, 255)),
            StrokeThickness = 3,
        };

        _simTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(50)
        };
        _simTimer.Tick += SimTimer_Tick;

        Loaded += OnLoaded;
        SizeChanged += OnSizeChanged;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        ShowGridCheck.Checked += (s, ev) => { DrawField(); RedrawPath(); };
        ShowGridCheck.Unchecked += (s, ev) => { DrawField(); RedrawPath(); };
        
        DrawField();
        AddSamplePath();
        UpdateStatistics();
    }

    private void OnSizeChanged(object sender, SizeChangedEventArgs e)
    {
        DrawField();
        RedrawPath();
    }

    private void DrawField()
    {
        FieldCanvas.Children.Clear();

        double canvasWidth = FieldCanvas.ActualWidth;
        double canvasHeight = FieldCanvas.ActualHeight;
        if (canvasWidth <= 0) canvasWidth = 600;
        if (canvasHeight <= 0) canvasHeight = 500;

        double baseSize = System.Math.Min(canvasWidth, canvasHeight) - 20;
        double fieldPixelSize = baseSize * _zoom;

        _scale = fieldPixelSize / FieldSize;
        _offset = new Point(
            (canvasWidth - fieldPixelSize) / 2 + _panOffset.X,
            (canvasHeight - fieldPixelSize) / 2 + _panOffset.Y
        );

        // Field background (gray foam tiles)
        var fieldBg = new Rectangle
        {
            Width = fieldPixelSize,
            Height = fieldPixelSize,
            Fill = new SolidColorBrush(Color.FromRgb(55, 55, 55)),
            Stroke = new SolidColorBrush(Color.FromRgb(150, 150, 150)),
            StrokeThickness = 3
        };
        Canvas.SetLeft(fieldBg, _offset.X);
        Canvas.SetTop(fieldBg, _offset.Y);
        FieldCanvas.Children.Add(fieldBg);

        // Grid lines (12" squares = 1 tile)
        if (ShowGridCheck.IsChecked == true)
        {
            for (int i = 0; i <= 12; i++)
            {
                double pos = i * 12 * _scale;
                bool isMajor = i == 6; // Center lines

                // Vertical line
                var vLine = new Line
                {
                    X1 = _offset.X + pos,
                    Y1 = _offset.Y,
                    X2 = _offset.X + pos,
                    Y2 = _offset.Y + fieldPixelSize,
                    Stroke = isMajor 
                        ? new SolidColorBrush(Color.FromRgb(100, 100, 100))
                        : new SolidColorBrush(Color.FromRgb(70, 70, 70)),
                    StrokeThickness = isMajor ? 2 : 1
                };
                FieldCanvas.Children.Add(vLine);

                // Horizontal line
                var hLine = new Line
                {
                    X1 = _offset.X,
                    Y1 = _offset.Y + pos,
                    X2 = _offset.X + fieldPixelSize,
                    Y2 = _offset.Y + pos,
                    Stroke = isMajor 
                        ? new SolidColorBrush(Color.FromRgb(100, 100, 100))
                        : new SolidColorBrush(Color.FromRgb(70, 70, 70)),
                    StrokeThickness = isMajor ? 2 : 1
                };
                FieldCanvas.Children.Add(hLine);
            }
        }

        // Draw High Stakes field elements
        DrawHighStakesElements();

        // Re-add path line
        FieldCanvas.Children.Add(_pathLine);
    }

    private void DrawHighStakesElements()
    {
        // Mobile Goals (circles on field) - approximate positions
        DrawMobileGoal(36, 36);
        DrawMobileGoal(108, 36);
        DrawMobileGoal(36, 108);
        DrawMobileGoal(108, 108);
        DrawMobileGoal(72, 72);

        // Alliance stakes (in the wall on each side)
        DrawStake(0, 72, Colors.Red, true);
        DrawStake(144, 72, Colors.Blue, true);

        // Neutral stakes (top and bottom walls)
        DrawStake(72, 0, Colors.Gray, true);
        DrawStake(72, 144, Colors.Gray, true);

        // Corners (positive and negative for each alliance)
        DrawCornerZone(0, 0, Colors.Blue, "Blue -");
        DrawCornerZone(144, 144, Colors.Red, "Red +");
        DrawCornerZone(0, 144, Colors.Blue, "Blue +");
        DrawCornerZone(144, 0, Colors.Red, "Red -");

        // Ladder/climb structure in center
        DrawLadder(72, 72);

        // Starting positions with labels
        DrawStartPosition(24, 24, 45, Colors.Red, "Red +");
        DrawStartPosition(24, 120, -45, Colors.Red, "Red -");
        DrawStartPosition(120, 120, -135, Colors.Blue, "Blue +");
        DrawStartPosition(120, 24, 135, Colors.Blue, "Blue -");
    }

    private void DrawMobileGoal(double x, double y)
    {
        double size = 7 * _scale; // Mobile goals are ~7" diameter
        var mogo = new Ellipse
        {
            Width = size,
            Height = size,
            Fill = new SolidColorBrush(Color.FromRgb(255, 215, 0)), // Gold/yellow
            Stroke = new SolidColorBrush(Color.FromRgb(200, 160, 0)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(mogo, _offset.X + x * _scale - size / 2);
        Canvas.SetTop(mogo, _offset.Y + (FieldSize - y) * _scale - size / 2);
        FieldCanvas.Children.Add(mogo);

        // Label
        var label = new TextBlock
        {
            Text = "Mogo",
            FontSize = 8,
            Foreground = Brushes.White
        };
        Canvas.SetLeft(label, _offset.X + x * _scale - 12);
        Canvas.SetTop(label, _offset.Y + (FieldSize - y) * _scale + size / 2 + 2);
        FieldCanvas.Children.Add(label);
    }

    private void DrawStake(double x, double y, Color color, bool isWallStake)
    {
        double size = isWallStake ? 4 * _scale : 6 * _scale;
        
        var stake = new Ellipse
        {
            Width = size,
            Height = size,
            Fill = new SolidColorBrush(color),
            Stroke = Brushes.White,
            StrokeThickness = 2
        };

        // Wall stakes are on the edge
        double drawX = x == 0 ? 2 : (x == 144 ? 144 - 2 : x);
        double drawY = y == 0 ? 2 : (y == 144 ? 144 - 2 : y);

        Canvas.SetLeft(stake, _offset.X + drawX * _scale - size / 2);
        Canvas.SetTop(stake, _offset.Y + (FieldSize - drawY) * _scale - size / 2);
        FieldCanvas.Children.Add(stake);
    }

    private void DrawCornerZone(double x, double y, Color color, string label)
    {
        double size = 24 * _scale;
        
        var corner = new Polygon
        {
            Fill = new SolidColorBrush(Color.FromArgb(60, color.R, color.G, color.B)),
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2
        };

        double cx = _offset.X + x * _scale;
        double cy = _offset.Y + (FieldSize - y) * _scale;

        // Create triangle for corner
        if (x < 72 && y < 72) // Bottom-left
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx + size, cy), new(cx, cy - size) };
        }
        else if (x > 72 && y > 72) // Top-right
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx - size, cy), new(cx, cy + size) };
        }
        else if (x < 72 && y > 72) // Top-left
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx + size, cy), new(cx, cy + size) };
        }
        else // Bottom-right
        {
            corner.Points = new PointCollection { new(cx, cy), new(cx - size, cy), new(cx, cy - size) };
        }

        FieldCanvas.Children.Add(corner);

        // Corner label
        var text = new TextBlock
        {
            Text = label,
            FontSize = 9,
            FontWeight = FontWeights.Bold,
            Foreground = new SolidColorBrush(color)
        };
        
        double labelX = x < 72 ? cx + 4 : cx - 35;
        double labelY = y < 72 ? cy - 15 : cy + 4;
        Canvas.SetLeft(text, labelX);
        Canvas.SetTop(text, labelY);
        FieldCanvas.Children.Add(text);
    }

    private void DrawLadder(double x, double y)
    {
        double size = 20 * _scale;
        var ladder = new Rectangle
        {
            Width = size,
            Height = size,
            Fill = new SolidColorBrush(Color.FromArgb(60, 255, 215, 0)),
            Stroke = new SolidColorBrush(Color.FromRgb(200, 160, 0)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(ladder, _offset.X + x * _scale - size / 2);
        Canvas.SetTop(ladder, _offset.Y + (FieldSize - y) * _scale - size / 2);
        FieldCanvas.Children.Add(ladder);

        var label = new TextBlock
        {
            Text = "Ladder",
            FontSize = 8,
            Foreground = Brushes.Gold,
            FontWeight = FontWeights.Bold
        };
        Canvas.SetLeft(label, _offset.X + x * _scale - 15);
        Canvas.SetTop(label, _offset.Y + (FieldSize - y) * _scale - 5);
        FieldCanvas.Children.Add(label);
    }

    private void DrawStartPosition(double x, double y, double heading, Color color, string label)
    {
        double robotSize = 18 * _scale; // 18" robot

        // Robot outline (dashed rectangle)
        var robot = new Rectangle
        {
            Width = robotSize,
            Height = robotSize,
            Fill = Brushes.Transparent,
            Stroke = new SolidColorBrush(Color.FromArgb(150, color.R, color.G, color.B)),
            StrokeThickness = 2,
            StrokeDashArray = new DoubleCollection { 4, 2 }
        };

        var transform = new RotateTransform(-heading, robotSize / 2, robotSize / 2);
        robot.RenderTransform = transform;

        Canvas.SetLeft(robot, _offset.X + x * _scale - robotSize / 2);
        Canvas.SetTop(robot, _offset.Y + (FieldSize - y) * _scale - robotSize / 2);
        FieldCanvas.Children.Add(robot);

        // Direction arrow
        double arrowLen = 12 * _scale;
        var arrow = new Line
        {
            X1 = _offset.X + x * _scale,
            Y1 = _offset.Y + (FieldSize - y) * _scale,
            X2 = _offset.X + x * _scale + arrowLen * System.Math.Cos((90 - heading) * System.Math.PI / 180),
            Y2 = _offset.Y + (FieldSize - y) * _scale - arrowLen * System.Math.Sin((90 - heading) * System.Math.PI / 180),
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2
        };
        FieldCanvas.Children.Add(arrow);

        // Label
        var text = new TextBlock
        {
            Text = label,
            Foreground = new SolidColorBrush(color),
            FontSize = 10,
            FontWeight = FontWeights.Bold
        };
        Canvas.SetLeft(text, _offset.X + x * _scale - 15);
        Canvas.SetTop(text, _offset.Y + (FieldSize - y) * _scale + robotSize / 2 + 2);
        FieldCanvas.Children.Add(text);
    }

    private void AddSamplePath()
    {
        _waypoints.Clear();
        
        // Sample autonomous path from Red + starting position
        _waypoints.Add(new WaypointVisual { X = 24, Y = 24, Heading = 45, Velocity = 100 });
        _waypoints.Add(new WaypointVisual { X = 36, Y = 36, Heading = 45, Velocity = 80 });
        _waypoints.Add(new WaypointVisual { X = 48, Y = 60, Heading = 70, Velocity = 60 });
        _waypoints.Add(new WaypointVisual { X = 55, Y = 85, Heading = 80, Velocity = 50 });
        _waypoints.Add(new WaypointVisual { X = 45, Y = 105, Heading = 120, Velocity = 70 });

        RedrawPath();
        SaveStateForUndo();
    }

    private void RedrawPath()
    {
        // Remove old waypoint visuals
        var toRemove = FieldCanvas.Children.OfType<UIElement>()
            .Where(e => e is Ellipse el && el.Tag is string s && s == "waypoint" ||
                        e is Line ln && ln.Tag is string s2 && s2 == "heading")
            .ToList();
        foreach (var item in toRemove)
            FieldCanvas.Children.Remove(item);

        if (_waypoints.Count < 1)
        {
            _pathLine.Data = null;
            UpdateStatistics();
            UpdateCodePreview();
            return;
        }

        // Draw path line using Bezier curves
        if (_waypoints.Count >= 2)
        {
            var geometry = new PathGeometry();
            var figure = new PathFigure
            {
                StartPoint = FieldToCanvas(_waypoints[0].X, _waypoints[0].Y)
            };

            for (int i = 1; i < _waypoints.Count; i++)
            {
                var prev = _waypoints[i - 1];
                var curr = _waypoints[i];

                var p0 = FieldToCanvas(prev.X, prev.Y);
                var p3 = FieldToCanvas(curr.X, curr.Y);

                double dx = curr.X - prev.X;
                double dy = curr.Y - prev.Y;
                double dist = System.Math.Sqrt(dx * dx + dy * dy);
                double cp = dist * 0.35 * _scale;

                var p1 = new Point(
                    p0.X + cp * System.Math.Cos((90 - prev.Heading) * System.Math.PI / 180),
                    p0.Y - cp * System.Math.Sin((90 - prev.Heading) * System.Math.PI / 180)
                );
                var p2 = new Point(
                    p3.X - cp * System.Math.Cos((90 - curr.Heading) * System.Math.PI / 180),
                    p3.Y + cp * System.Math.Sin((90 - curr.Heading) * System.Math.PI / 180)
                );

                figure.Segments.Add(new BezierSegment(p1, p2, p3, true));
            }

            geometry.Figures.Add(figure);
            _pathLine.Data = geometry;
        }

        // Draw waypoint markers
        for (int i = 0; i < _waypoints.Count; i++)
        {
            var wp = _waypoints[i];
            var pos = FieldToCanvas(wp.X, wp.Y);

            // Waypoint circle
            bool isSelected = i == _selectedWaypointIndex;
            var marker = new Ellipse
            {
                Width = isSelected ? 18 : 14,
                Height = isSelected ? 18 : 14,
                Fill = i == 0 ? Brushes.LimeGreen :
                       i == _waypoints.Count - 1 ? Brushes.OrangeRed :
                       Brushes.DodgerBlue,
                Stroke = isSelected ? Brushes.Yellow : Brushes.White,
                StrokeThickness = isSelected ? 3 : 2,
                Cursor = Cursors.Hand,
                Tag = "waypoint"
            };

            Canvas.SetLeft(marker, pos.X - marker.Width / 2);
            Canvas.SetTop(marker, pos.Y - marker.Height / 2);
            Panel.SetZIndex(marker, 100);

            int capturedIndex = i;
            marker.MouseLeftButtonDown += (s, e) => SelectAndStartDrag(capturedIndex, e);
            marker.MouseMove += Marker_MouseMove;
            marker.MouseLeftButtonUp += Marker_MouseLeftButtonUp;

            FieldCanvas.Children.Add(marker);

            // Heading arrow
            double arrowLen = 18;
            var arrow = new Line
            {
                X1 = pos.X,
                Y1 = pos.Y,
                X2 = pos.X + arrowLen * System.Math.Cos((90 - wp.Heading) * System.Math.PI / 180),
                Y2 = pos.Y - arrowLen * System.Math.Sin((90 - wp.Heading) * System.Math.PI / 180),
                Stroke = Brushes.White,
                StrokeThickness = 2,
                Tag = "heading"
            };
            Panel.SetZIndex(arrow, 99);
            FieldCanvas.Children.Add(arrow);

            // Waypoint number label
            var numLabel = new TextBlock
            {
                Text = (i + 1).ToString(),
                FontSize = 9,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.Black
            };
            Canvas.SetLeft(numLabel, pos.X - 4);
            Canvas.SetTop(numLabel, pos.Y - 5);
            Panel.SetZIndex(numLabel, 101);
            FieldCanvas.Children.Add(numLabel);
        }

        UpdateStatistics();
        UpdateCodePreview();
    }

    private void SelectAndStartDrag(int index, MouseButtonEventArgs e)
    {
        _selectedWaypointIndex = index;
        _selectedWaypoint = _waypoints[index];
        _draggingWaypoint = _selectedWaypoint;
        _isDraggingWaypoint = true;
        _lastMousePos = e.GetPosition(FieldCanvas);

        // Update UI
        WaypointX.Text = _selectedWaypoint.X.ToString("F1");
        WaypointY.Text = _selectedWaypoint.Y.ToString("F1");
        WaypointHeading.Text = _selectedWaypoint.Heading.ToString("F0");
        WaypointVel.Text = _selectedWaypoint.Velocity.ToString("F0");

        if (e.Source is UIElement el)
            el.CaptureMouse();

        e.Handled = true;
        RedrawPath();
    }

    private void Marker_MouseMove(object sender, MouseEventArgs e)
    {
        if (_isDraggingWaypoint && _draggingWaypoint != null)
        {
            var currentPos = e.GetPosition(FieldCanvas);
            var fieldPos = CanvasToField(currentPos.X, currentPos.Y);

            _draggingWaypoint.X = System.Math.Clamp(fieldPos.X, 0, FieldSize);
            _draggingWaypoint.Y = System.Math.Clamp(fieldPos.Y, 0, FieldSize);

            WaypointX.Text = _draggingWaypoint.X.ToString("F1");
            WaypointY.Text = _draggingWaypoint.Y.ToString("F1");

            RedrawPath();
            e.Handled = true;
        }
    }

    private void Marker_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        if (sender is UIElement el)
            el.ReleaseMouseCapture();
        
        if (_isDraggingWaypoint)
        {
            SaveStateForUndo();
        }
        
        _isDraggingWaypoint = false;
        _draggingWaypoint = null;
        e.Handled = true;
    }

    // Canvas panning (middle mouse or when not on a waypoint)
    private void FieldCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        // Only pan if not clicking on a waypoint
        if (!_isDraggingWaypoint)
        {
            _isPanningField = true;
            _lastMousePos = e.GetPosition(this);
            FieldCanvas.CaptureMouse();
        }
    }

    private void FieldCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        if (_isPanningField)
        {
            _isPanningField = false;
            FieldCanvas.ReleaseMouseCapture();
        }
    }

    private void FieldCanvas_MouseMove(object sender, MouseEventArgs e)
    {
        if (_isPanningField && e.LeftButton == MouseButtonState.Pressed)
        {
            var currentPos = e.GetPosition(this);
            double dx = currentPos.X - _lastMousePos.X;
            double dy = currentPos.Y - _lastMousePos.Y;

            _panOffset = new Point(_panOffset.X + dx, _panOffset.Y + dy);
            _lastMousePos = currentPos;

            DrawField();
            RedrawPath();
        }
    }

    private void FieldCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        double zoomFactor = e.Delta > 0 ? 1.1 : 0.9;
        _zoom = System.Math.Clamp(_zoom * zoomFactor, 0.5, 3.0);
        
        DrawField();
        RedrawPath();
    }

    private void FieldCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
    {
        _contextMenuFieldPos = CanvasToField(e.GetPosition(FieldCanvas).X, e.GetPosition(FieldCanvas).Y);
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

    // Toolbar buttons
    private void ZoomIn_Click(object sender, RoutedEventArgs e)
    {
        _zoom = System.Math.Min(_zoom * 1.2, 3.0);
        DrawField();
        RedrawPath();
    }

    private void ZoomOut_Click(object sender, RoutedEventArgs e)
    {
        _zoom = System.Math.Max(_zoom / 1.2, 0.5);
        DrawField();
        RedrawPath();
    }

    private void ResetView_Click(object sender, RoutedEventArgs e)
    {
        _zoom = 1.0;
        _panOffset = new Point(0, 0);
        DrawField();
        RedrawPath();
    }

    // Path management
    private void NewPath_Click(object sender, RoutedEventArgs e)
    {
        SaveStateForUndo();
        _waypoints.Clear();
        _selectedWaypointIndex = -1;
        _selectedWaypoint = null;
        RedrawPath();
    }

    private void CopyPath_Click(object sender, RoutedEventArgs e)
    {
        // Duplicate current path with offset
        var newWaypoints = _waypoints.Select(wp => new WaypointVisual
        {
            X = wp.X + 12,
            Y = wp.Y,
            Heading = wp.Heading,
            Velocity = wp.Velocity
        }).ToList();
        
        PathsList.Items.Add(new ListBoxItem { Content = $"Path Copy {PathsList.Items.Count + 1}" });
    }

    private void DeletePath_Click(object sender, RoutedEventArgs e)
    {
        if (PathsList.SelectedItem != null && PathsList.Items.Count > 1)
        {
            PathsList.Items.Remove(PathsList.SelectedItem);
        }
    }

    private void AddWaypoint_Click(object sender, RoutedEventArgs e)
    {
        SaveStateForUndo();
        
        // Add waypoint at a reasonable position
        double newX = _waypoints.Count > 0 ? _waypoints[^1].X + 12 : 24;
        double newY = _waypoints.Count > 0 ? _waypoints[^1].Y + 12 : 24;
        double newHeading = _waypoints.Count > 0 ? _waypoints[^1].Heading : 0;

        _waypoints.Add(new WaypointVisual 
        { 
            X = System.Math.Clamp(newX, 0, FieldSize), 
            Y = System.Math.Clamp(newY, 0, FieldSize), 
            Heading = newHeading, 
            Velocity = 80 
        });
        
        _selectedWaypointIndex = _waypoints.Count - 1;
        _selectedWaypoint = _waypoints[^1];
        
        RedrawPath();
    }

    private void AddWaypointHere_Click(object sender, RoutedEventArgs e)
    {
        SaveStateForUndo();
        
        double heading = _waypoints.Count > 0 ? _waypoints[^1].Heading : 0;
        _waypoints.Add(new WaypointVisual 
        { 
            X = System.Math.Clamp(_contextMenuFieldPos.X, 0, FieldSize), 
            Y = System.Math.Clamp(_contextMenuFieldPos.Y, 0, FieldSize), 
            Heading = heading, 
            Velocity = 80 
        });
        
        _selectedWaypointIndex = _waypoints.Count - 1;
        _selectedWaypoint = _waypoints[^1];
        
        RedrawPath();
    }

    private void AddTurn_Click(object sender, RoutedEventArgs e)
    {
        // Similar to AddWaypoint but marks it as a turn-in-place
        AddWaypoint_Click(sender, e);
    }

    private void AddTurnHere_Click(object sender, RoutedEventArgs e)
    {
        AddWaypointHere_Click(sender, e);
    }

    private void DeletePoint_Click(object sender, RoutedEventArgs e)
    {
        if (_selectedWaypointIndex >= 0 && _selectedWaypointIndex < _waypoints.Count)
        {
            SaveStateForUndo();
            _waypoints.RemoveAt(_selectedWaypointIndex);
            _selectedWaypointIndex = -1;
            _selectedWaypoint = null;
            RedrawPath();
        }
    }

    private void ClearPath_Click(object sender, RoutedEventArgs e)
    {
        SaveStateForUndo();
        _waypoints.Clear();
        _selectedWaypointIndex = -1;
        _selectedWaypoint = null;
        RedrawPath();
    }

    // Undo/Redo
    private void SaveStateForUndo()
    {
        var state = _waypoints.Select(wp => new WaypointVisual
        {
            X = wp.X, Y = wp.Y, Heading = wp.Heading, Velocity = wp.Velocity
        }).ToList();
        _undoStack.Push(state);
        _redoStack.Clear();
    }

    private void Undo_Click(object sender, RoutedEventArgs e)
    {
        if (_undoStack.Count > 0)
        {
            var current = _waypoints.Select(wp => new WaypointVisual
            {
                X = wp.X, Y = wp.Y, Heading = wp.Heading, Velocity = wp.Velocity
            }).ToList();
            _redoStack.Push(current);

            var prev = _undoStack.Pop();
            _waypoints.Clear();
            _waypoints.AddRange(prev);
            _selectedWaypointIndex = -1;
            _selectedWaypoint = null;
            RedrawPath();
        }
    }

    private void Redo_Click(object sender, RoutedEventArgs e)
    {
        if (_redoStack.Count > 0)
        {
            var current = _waypoints.Select(wp => new WaypointVisual
            {
                X = wp.X, Y = wp.Y, Heading = wp.Heading, Velocity = wp.Velocity
            }).ToList();
            _undoStack.Push(current);

            var next = _redoStack.Pop();
            _waypoints.Clear();
            _waypoints.AddRange(next);
            _selectedWaypointIndex = -1;
            _selectedWaypoint = null;
            RedrawPath();
        }
    }

    // Simulation
    private void Simulate_Click(object sender, RoutedEventArgs e)
    {
        if (_waypoints.Count < 2) return;
        
        _isSimulating = true;
        _simIndex = 0;
        TimelineSlider.Value = 0;
        _simTimer.Start();
        SimulateBtn.IsEnabled = false;
    }

    private void Stop_Click(object sender, RoutedEventArgs e)
    {
        _simTimer.Stop();
        _isSimulating = false;
        SimulateBtn.IsEnabled = true;
    }

    private void SimTimer_Tick(object? sender, EventArgs e)
    {
        if (!_isSimulating) return;

        TimelineSlider.Value += 2;
        
        if (TimelineSlider.Value >= 100)
        {
            Stop_Click(this, new RoutedEventArgs());
        }
    }

    private void TimelineSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (_waypoints.Count < 2) return;

        double t = e.NewValue / 100.0;
        double totalDist = CalculateTotalDistance();
        double currentDist = t * totalDist;

        TimelineText.Text = $"{t * CalculateEstimatedTime():F1}s / {CalculateEstimatedTime():F1}s";
        DistanceText.Text = $"{currentDist:F1} / {totalDist:F1} in";
    }

    private void WaypointValue_Changed(object sender, TextChangedEventArgs e)
    {
        if (_selectedWaypoint == null) return;

        if (double.TryParse(WaypointX.Text, out double x))
            _selectedWaypoint.X = System.Math.Clamp(x, 0, FieldSize);
        if (double.TryParse(WaypointY.Text, out double y))
            _selectedWaypoint.Y = System.Math.Clamp(y, 0, FieldSize);
        if (double.TryParse(WaypointHeading.Text, out double h))
            _selectedWaypoint.Heading = h;
        if (double.TryParse(WaypointVel.Text, out double v))
            _selectedWaypoint.Velocity = System.Math.Clamp(v, 0, 100);

        RedrawPath();
    }

    // Statistics
    private void UpdateStatistics()
    {
        double totalDist = CalculateTotalDistance();
        double estTime = CalculateEstimatedTime();

        TotalDistanceText.Text = $"{totalDist:F1} in";
        TotalTimeText.Text = $"{estTime:F1}s";
        WaypointStatsText.Text = _waypoints.Count.ToString();
        WaypointCountText.Text = _waypoints.Count.ToString();
        MaxCurvatureText.Text = $"{CalculateMaxCurvature():F3} /in";
        DistanceText.Text = $"0.0 / {totalDist:F1} in";
        TimelineText.Text = $"0.0s / {estTime:F1}s";
    }

    private double CalculateTotalDistance()
    {
        double dist = 0;
        for (int i = 1; i < _waypoints.Count; i++)
        {
            double dx = _waypoints[i].X - _waypoints[i - 1].X;
            double dy = _waypoints[i].Y - _waypoints[i - 1].Y;
            dist += System.Math.Sqrt(dx * dx + dy * dy);
        }
        return dist;
    }

    private double CalculateEstimatedTime()
    {
        double dist = CalculateTotalDistance();
        double avgVel = MaxVelSlider.Value * 0.7; // Assume 70% of max velocity average
        return avgVel > 0 ? dist / avgVel : 0;
    }

    private double CalculateMaxCurvature()
    {
        if (_waypoints.Count < 3) return 0;
        
        double maxCurv = 0;
        for (int i = 1; i < _waypoints.Count - 1; i++)
        {
            var p0 = _waypoints[i - 1];
            var p1 = _waypoints[i];
            var p2 = _waypoints[i + 1];

            double area = System.Math.Abs((p1.X - p0.X) * (p2.Y - p0.Y) - (p2.X - p0.X) * (p1.Y - p0.Y)) / 2;
            double a = System.Math.Sqrt((p1.X - p0.X) * (p1.X - p0.X) + (p1.Y - p0.Y) * (p1.Y - p0.Y));
            double b = System.Math.Sqrt((p2.X - p1.X) * (p2.X - p1.X) + (p2.Y - p1.Y) * (p2.Y - p1.Y));
            double c = System.Math.Sqrt((p2.X - p0.X) * (p2.X - p0.X) + (p2.Y - p0.Y) * (p2.Y - p0.Y));

            if (a > 0 && b > 0 && c > 0)
            {
                double curvature = 4 * area / (a * b * c);
                maxCurv = System.Math.Max(maxCurv, curvature);
            }
        }
        return maxCurv;
    }

    // Code generation
    private void UpdateCodePreview()
    {
        var sb = new StringBuilder();
        sb.AppendLine("// LemLib Pure Pursuit Path");
        sb.AppendLine("// Generated by ControlWorkbench");
        sb.AppendLine();
        
        string pathName = "auton_path";
        
        sb.AppendLine($"ASSET({pathName}_txt) = {{");
        foreach (var wp in _waypoints)
        {
            sb.AppendLine($"    {wp.X:F1}, {wp.Y:F1}, {wp.Heading:F0},");
        }
        sb.AppendLine("};");
        sb.AppendLine();
        sb.AppendLine($"void run_{pathName}() {{");
        sb.AppendLine($"    chassis.follow(");
        sb.AppendLine($"        {pathName}_txt,");
        sb.AppendLine($"        {LookaheadSlider.Value:F0},  // lookahead distance");
        sb.AppendLine($"        {(int)(CalculateEstimatedTime() * 1000 + 1000)}   // timeout (ms)");
        sb.AppendLine($"    );");
        sb.AppendLine("}");

        CodePreview.Text = sb.ToString();
    }

    private void CopyLemLibCode_Click(object sender, RoutedEventArgs e)
    {
        Clipboard.SetText(CodePreview.Text);
        MessageBox.Show("LemLib code copied to clipboard!", "Copied", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void CopyPurePursuitCode_Click(object sender, RoutedEventArgs e)
    {
        var sb = new StringBuilder();
        sb.AppendLine("// Pure Pursuit waypoints array");
        sb.AppendLine("std::vector<Point> path = {");
        foreach (var wp in _waypoints)
        {
            sb.AppendLine($"    {{{wp.X:F1}, {wp.Y:F1}}},");
        }
        sb.AppendLine("};");
        sb.AppendLine();
        sb.AppendLine($"double lookahead = {LookaheadSlider.Value:F1};");
        sb.AppendLine("followPath(path, lookahead);");

        Clipboard.SetText(sb.ToString());
        MessageBox.Show("Pure Pursuit code copied to clipboard!", "Copied", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void SavePath_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new SaveFileDialog
        {
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
            DefaultExt = ".json",
            FileName = "vex_path"
        };

        if (dialog.ShowDialog() == true)
        {
            var pathData = new
            {
                name = System.IO.Path.GetFileNameWithoutExtension(dialog.FileName),
                game = "High Stakes",
                alliance = RedAlliance.IsChecked == true ? "Red" : "Blue",
                maxVelocity = MaxVelSlider.Value,
                lookahead = LookaheadSlider.Value,
                waypoints = _waypoints.Select(wp => new
                {
                    x = wp.X,
                    y = wp.Y,
                    heading = wp.Heading,
                    velocity = wp.Velocity
                }).ToArray()
            };

            string json = JsonSerializer.Serialize(pathData, new JsonSerializerOptions { WriteIndented = true });
            System.IO.File.WriteAllText(dialog.FileName, json);
            MessageBox.Show("Path saved successfully!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private void LoadPath_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new OpenFileDialog
        {
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*"
        };

        if (dialog.ShowDialog() == true)
        {
            try
            {
                string json = System.IO.File.ReadAllText(dialog.FileName);
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;

                SaveStateForUndo();
                _waypoints.Clear();

                if (root.TryGetProperty("waypoints", out var waypoints))
                {
                    foreach (var wp in waypoints.EnumerateArray())
                    {
                        _waypoints.Add(new WaypointVisual
                        {
                            X = wp.GetProperty("x").GetDouble(),
                            Y = wp.GetProperty("y").GetDouble(),
                            Heading = wp.GetProperty("heading").GetDouble(),
                            Velocity = wp.TryGetProperty("velocity", out var vel) ? vel.GetDouble() : 80
                        });
                    }
                }

                if (root.TryGetProperty("maxVelocity", out var maxVel))
                    MaxVelSlider.Value = maxVel.GetDouble();
                if (root.TryGetProperty("lookahead", out var la))
                    LookaheadSlider.Value = la.GetDouble();

                RedrawPath();
                MessageBox.Show("Path loaded successfully!", "Loaded", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error loading path: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
    }
}

public class WaypointVisual
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Heading { get; set; }
    public double Velocity { get; set; } = 80;
    public List<string> Actions { get; } = new();
}
