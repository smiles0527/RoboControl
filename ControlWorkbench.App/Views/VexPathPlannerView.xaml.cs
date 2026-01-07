using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Microsoft.Win32;

// Alias to avoid collision with ControlWorkbench.Math namespace
using SysMath = System.Math;

namespace ControlWorkbench.App.Views;

/// <summary>
/// VEX Path Planner view with interactive field canvas.
/// Coordinate system: Field coordinates are (0,0) at bottom-left, (144,144) at top-right.
/// Canvas coordinates have (0,0) at top-left, with Y increasing downward.
/// </summary>
public partial class VexPathPlannerView : UserControl
{
    // Field dimensions (144" x 144" VRC field)
    private const double FieldSize = 144.0;
    
    // Canvas transformation parameters
    private double _scale = 1.0;        // pixels per inch
    private double _zoom = 1.0;         // zoom multiplier
    private Point _offset = new(0, 0);  // top-left corner of field on canvas
    private Point _panOffset = new(0, 0);
    
    // Waypoint state
    private readonly List<WaypointVisual> _waypoints = new();
    private WaypointVisual? _selectedWaypoint;
    private int _selectedWaypointIndex = -1;
    private WaypointVisual? _draggingWaypoint;
    
    // Interaction state
    private bool _isDraggingWaypoint;
    private bool _isPanningField;
    private Point _lastMousePos;
    private Point _panStartOffset;
    private Point _contextMenuFieldPos;
    private bool _isLoaded;
    private bool _isUpdatingTextBoxes;  // Prevent recursive updates

    // Performance: throttle redraws during drag
    private bool _redrawPending;

    // Undo/Redo
    private readonly Stack<List<WaypointData>> _undoStack = new();
    private readonly Stack<List<WaypointData>> _redoStack = new();

    // Simulation
    private readonly DispatcherTimer _simTimer;
    private bool _isSimulating;

    // Path visualization
    private readonly Path _pathLine;
    
    // Cached visual elements for waypoints
    private readonly List<UIElement> _waypointVisuals = new();

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
        if (_isLoaded) return;
        _isLoaded = true;
        
        if (ShowGridCheck != null)
        {
            ShowGridCheck.Checked += (s, ev) => FullRedraw();
            ShowGridCheck.Unchecked += (s, ev) => FullRedraw();
        }
        
        FullRedraw();
        AddSamplePath();
        UpdateStatistics();
    }

    private void OnSizeChanged(object sender, SizeChangedEventArgs e)
    {
        if (!_isLoaded) return;
        FullRedraw();
    }

    /// <summary>
    /// Calculate the transformation parameters based on canvas size and zoom.
    /// </summary>
    private void RecalculateTransform()
    {
        double canvasWidth = FieldCanvas.ActualWidth;
        double canvasHeight = FieldCanvas.ActualHeight;
        
        // Ensure minimum canvas size
        if (canvasWidth < 100) canvasWidth = 600;
        if (canvasHeight < 100) canvasHeight = 500;

        // Calculate base size to fit field in canvas with margin
        double margin = 20;
        double availableSize = SysMath.Min(canvasWidth, canvasHeight) - margin * 2;
        if (availableSize < 100) availableSize = 400;
        
        // Apply zoom
        double fieldPixelSize = availableSize * _zoom;
        
        // Scale: pixels per inch
        _scale = fieldPixelSize / FieldSize;
        if (_scale < 0.1) _scale = 1.0;  // Safety minimum
        
        // Offset: position of field (0,0) on canvas - which is bottom-left of field
        // We want field centered in canvas, then apply pan offset
        double fieldCenterX = canvasWidth / 2 + _panOffset.X;
        double fieldCenterY = canvasHeight / 2 + _panOffset.Y;
        
        // Offset is the top-left corner of the field rectangle
        _offset = new Point(
            fieldCenterX - fieldPixelSize / 2,
            fieldCenterY - fieldPixelSize / 2
        );
    }

    /// <summary>
    /// Convert field coordinates (0-144, origin bottom-left) to canvas coordinates.
    /// </summary>
    private Point FieldToCanvas(double fieldX, double fieldY)
    {
        // Field Y=0 is at bottom, Y=144 is at top
        // Canvas Y=0 is at top, increases downward
        return new Point(
            _offset.X + fieldX * _scale,
            _offset.Y + (FieldSize - fieldY) * _scale
        );
    }

    /// <summary>
    /// Convert canvas coordinates to field coordinates (0-144, origin bottom-left).
    /// </summary>
    private Point CanvasToField(double canvasX, double canvasY)
    {
        double fieldX = (canvasX - _offset.X) / _scale;
        double fieldY = FieldSize - (canvasY - _offset.Y) / _scale;
        return new Point(fieldX, fieldY);
    }

    /// <summary>
    /// Convert heading angle (0=up, positive=clockwise) to canvas angle for drawing.
    /// </summary>
    private double HeadingToCanvasAngle(double headingDeg)
    {
        // Heading: 0 = facing +Y (up on field), 90 = facing +X (right on field)
        // Canvas angle for drawing: 0 = right, 90 = down
        // So heading 0 (up) = canvas angle -90 (up)
        // heading 90 (right) = canvas angle 0 (right)
        return headingDeg - 90;
    }

    /// <summary>
    /// Get direction vector from heading (0=up, positive=clockwise).
    /// </summary>
    private (double dx, double dy) HeadingToCanvasDirection(double headingDeg, double length)
    {
        // Convert to radians, adjust for canvas coordinates
        double radians = (headingDeg - 90) * SysMath.PI / 180;
        // Canvas: positive X is right, positive Y is down
        double dx = SysMath.Cos(radians) * length;
        double dy = SysMath.Sin(radians) * length;
        return (dx, dy);
    }

    private void FullRedraw()
    {
        RecalculateTransform();
        DrawField();
        RedrawPath();
    }

    private void DrawField()
    {
        if (FieldCanvas == null) return;
        
        // Clear canvas
        FieldCanvas.Children.Clear();

        double fieldPixelSize = FieldSize * _scale;

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
        if (ShowGridCheck?.IsChecked == true)
        {
            DrawGridLines(fieldPixelSize);
        }

        // Draw High Stakes field elements
        DrawHighStakesElements();

        // Re-add path line (will be updated by RedrawPath)
        FieldCanvas.Children.Add(_pathLine);
    }

    private void DrawGridLines(double fieldPixelSize)
    {
        for (int i = 0; i <= 12; i++)
        {
            double pos = i * 12 * _scale;
            bool isMajor = i == 6; // Center lines

            var strokeBrush = isMajor 
                ? new SolidColorBrush(Color.FromRgb(100, 100, 100))
                : new SolidColorBrush(Color.FromRgb(70, 70, 70));
            double thickness = isMajor ? 2 : 1;

            // Vertical line
            var vLine = new Line
            {
                X1 = _offset.X + pos,
                Y1 = _offset.Y,
                X2 = _offset.X + pos,
                Y2 = _offset.Y + fieldPixelSize,
                Stroke = strokeBrush,
                StrokeThickness = thickness
            };
            FieldCanvas.Children.Add(vLine);

            // Horizontal line
            var hLine = new Line
            {
                X1 = _offset.X,
                Y1 = _offset.Y + pos,
                X2 = _offset.X + fieldPixelSize,
                Y2 = _offset.Y + pos,
                Stroke = strokeBrush,
                StrokeThickness = thickness
            };
            FieldCanvas.Children.Add(hLine);
        }
    }

    private void DrawHighStakesElements()
    {
        // Mobile Goals (circles on field) - approximate positions for High Stakes
        DrawMobileGoal(36, 36);
        DrawMobileGoal(108, 36);
        DrawMobileGoal(36, 108);
        DrawMobileGoal(108, 108);
        DrawMobileGoal(72, 72);  // Center

        // Alliance stakes (in the walls)
        DrawStake(0, 72, Colors.Red);      // Red alliance stake (left wall)
        DrawStake(144, 72, Colors.Blue);   // Blue alliance stake (right wall)

        // Neutral stakes (top and bottom walls)
        DrawStake(72, 0, Colors.Gray);     // Bottom wall
        DrawStake(72, 144, Colors.Gray);   // Top wall

        // Corner zones
        DrawCornerZone(0, 0, Colors.Red, "Red -");      // Bottom-left
        DrawCornerZone(144, 144, Colors.Red, "Red +");   // Top-right
        DrawCornerZone(0, 144, Colors.Blue, "Blue +");   // Top-left
        DrawCornerZone(144, 0, Colors.Blue, "Blue -");   // Bottom-right

        // Ladder in center
        DrawLadder(72, 72);

        // Starting positions
        DrawStartPosition(24, 24, 45, Colors.Red, "Red +");
        DrawStartPosition(24, 120, -45, Colors.Red, "Red -");
        DrawStartPosition(120, 120, -135, Colors.Blue, "Blue +");
        DrawStartPosition(120, 24, 135, Colors.Blue, "Blue -");
    }

    private void DrawMobileGoal(double x, double y)
    {
        double size = 7 * _scale;
        var pos = FieldToCanvas(x, y);
        
        var mogo = new Ellipse
        {
            Width = size,
            Height = size,
            Fill = new SolidColorBrush(Color.FromRgb(255, 215, 0)),
            Stroke = new SolidColorBrush(Color.FromRgb(200, 160, 0)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(mogo, pos.X - size / 2);
        Canvas.SetTop(mogo, pos.Y - size / 2);
        FieldCanvas.Children.Add(mogo);

        var label = new TextBlock
        {
            Text = "Mogo",
            FontSize = SysMath.Max(8, 8 * _zoom),
            Foreground = Brushes.White
        };
        Canvas.SetLeft(label, pos.X - 12);
        Canvas.SetTop(label, pos.Y + size / 2 + 2);
        FieldCanvas.Children.Add(label);
    }

    private void DrawStake(double x, double y, Color color)
    {
        double size = 4 * _scale;
        
        // Wall stakes: adjust position to be on the wall edge
        double drawX = x;
        double drawY = y;
        if (x <= 0) drawX = 2;
        else if (x >= 144) drawX = 142;
        if (y <= 0) drawY = 2;
        else if (y >= 144) drawY = 142;
        
        var pos = FieldToCanvas(drawX, drawY);
        
        var stake = new Ellipse
        {
            Width = size,
            Height = size,
            Fill = new SolidColorBrush(color),
            Stroke = Brushes.White,
            StrokeThickness = 2
        };
        Canvas.SetLeft(stake, pos.X - size / 2);
        Canvas.SetTop(stake, pos.Y - size / 2);
        FieldCanvas.Children.Add(stake);
    }

    private void DrawCornerZone(double x, double y, Color color, string label)
    {
        double size = 24 * _scale;
        var pos = FieldToCanvas(x, y);
        
        var corner = new Polygon
        {
            Fill = new SolidColorBrush(Color.FromArgb(60, color.R, color.G, color.B)),
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2
        };

        // Determine triangle orientation based on corner position
        PointCollection points;
        if (x < 72 && y < 72) // Bottom-left corner
        {
            points = new PointCollection { 
                new(pos.X, pos.Y), 
                new(pos.X + size, pos.Y), 
                new(pos.X, pos.Y - size) 
            };
        }
        else if (x > 72 && y > 72) // Top-right corner
        {
            points = new PointCollection { 
                new(pos.X, pos.Y), 
                new(pos.X - size, pos.Y), 
                new(pos.X, pos.Y + size) 
            };
        }
        else if (x < 72 && y > 72) // Top-left corner
        {
            points = new PointCollection { 
                new(pos.X, pos.Y), 
                new(pos.X + size, pos.Y), 
                new(pos.X, pos.Y + size) 
            };
        }
        else // Bottom-right corner
        {
            points = new PointCollection { 
                new(pos.X, pos.Y), 
                new(pos.X - size, pos.Y), 
                new(pos.X, pos.Y - size) 
            };
        }
        corner.Points = points;
        FieldCanvas.Children.Add(corner);

        // Corner label
        var text = new TextBlock
        {
            Text = label,
            FontSize = SysMath.Max(9, 9 * _zoom),
            FontWeight = FontWeights.Bold,
            Foreground = new SolidColorBrush(color)
        };
        
        double labelX = x < 72 ? pos.X + 4 : pos.X - 35;
        double labelY = y < 72 ? pos.Y - 15 : pos.Y + 4;
        Canvas.SetLeft(text, labelX);
        Canvas.SetTop(text, labelY);
        FieldCanvas.Children.Add(text);
    }

    private void DrawLadder(double x, double y)
    {
        double size = 20 * _scale;
        var pos = FieldToCanvas(x, y);
        
        var ladder = new Rectangle
        {
            Width = size,
            Height = size,
            Fill = new SolidColorBrush(Color.FromArgb(60, 255, 215, 0)),
            Stroke = new SolidColorBrush(Color.FromRgb(200, 160, 0)),
            StrokeThickness = 2
        };
        Canvas.SetLeft(ladder, pos.X - size / 2);
        Canvas.SetTop(ladder, pos.Y - size / 2);
        FieldCanvas.Children.Add(ladder);

        var label = new TextBlock
        {
            Text = "Ladder",
            FontSize = SysMath.Max(8, 8 * _zoom),
            Foreground = Brushes.Gold,
            FontWeight = FontWeights.Bold
        };
        Canvas.SetLeft(label, pos.X - 15);
        Canvas.SetTop(label, pos.Y - 5);
        FieldCanvas.Children.Add(label);
    }

    private void DrawStartPosition(double x, double y, double heading, Color color, string label)
    {
        double robotSize = 18 * _scale;
        var pos = FieldToCanvas(x, y);

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

        // Rotate around center
        robot.RenderTransform = new RotateTransform(-heading, robotSize / 2, robotSize / 2);
        Canvas.SetLeft(robot, pos.X - robotSize / 2);
        Canvas.SetTop(robot, pos.Y - robotSize / 2);
        FieldCanvas.Children.Add(robot);

        // Direction arrow
        double arrowLen = 12 * _scale;
        var (dx, dy) = HeadingToCanvasDirection(heading, arrowLen);
        
        var arrow = new Line
        {
            X1 = pos.X,
            Y1 = pos.Y,
            X2 = pos.X + dx,
            Y2 = pos.Y + dy,
            Stroke = new SolidColorBrush(color),
            StrokeThickness = 2
        };
        FieldCanvas.Children.Add(arrow);

        // Label
        var text = new TextBlock
        {
            Text = label,
            Foreground = new SolidColorBrush(color),
            FontSize = SysMath.Max(10, 10 * _zoom),
            FontWeight = FontWeights.Bold
        };
        Canvas.SetLeft(text, pos.X - 15);
        Canvas.SetTop(text, pos.Y + robotSize / 2 + 2);
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
        // Clear waypoint visuals
        foreach (var visual in _waypointVisuals)
        {
            FieldCanvas.Children.Remove(visual);
        }
        _waypointVisuals.Clear();

        if (_waypoints.Count < 1)
        {
            _pathLine.Data = null;
            UpdateStatistics();
            UpdateCodePreview();
            return;
        }

        // Draw path curve using Bezier splines
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

                // Calculate control point distance based on segment length
                double dx = curr.X - prev.X;
                double dy = curr.Y - prev.Y;
                double dist = SysMath.Sqrt(dx * dx + dy * dy);
                double cpDist = dist * 0.35 * _scale;

                // Control points extend in the direction of the heading
                var (prevDx, prevDy) = HeadingToCanvasDirection(prev.Heading, cpDist);
                var (currDx, currDy) = HeadingToCanvasDirection(curr.Heading, cpDist);

                var p1 = new Point(p0.X + prevDx, p0.Y + prevDy);
                var p2 = new Point(p3.X - currDx, p3.Y - currDy);

                figure.Segments.Add(new BezierSegment(p1, p2, p3, true));
            }

            geometry.Figures.Add(figure);
            _pathLine.Data = geometry;
        }
        else
        {
            _pathLine.Data = null;
        }

        // Draw waypoint markers
        for (int i = 0; i < _waypoints.Count; i++)
        {
            DrawWaypointMarker(i);
        }

        UpdateStatistics();
        UpdateCodePreview();
    }

    private void DrawWaypointMarker(int index)
    {
        var wp = _waypoints[index];
        var pos = FieldToCanvas(wp.X, wp.Y);
        bool isSelected = index == _selectedWaypointIndex;

        // Waypoint circle
        double markerSize = isSelected ? 18 : 14;
        var marker = new Ellipse
        {
            Width = markerSize,
            Height = markerSize,
            Fill = index == 0 ? Brushes.LimeGreen :
                   index == _waypoints.Count - 1 ? Brushes.OrangeRed :
                   Brushes.DodgerBlue,
            Stroke = isSelected ? Brushes.Yellow : Brushes.White,
            StrokeThickness = isSelected ? 3 : 2,
            Cursor = Cursors.Hand,
            Tag = index,
            IsHitTestVisible = true
        };
        Canvas.SetLeft(marker, pos.X - markerSize / 2);
        Canvas.SetTop(marker, pos.Y - markerSize / 2);
        Panel.SetZIndex(marker, 100);
        FieldCanvas.Children.Add(marker);
        _waypointVisuals.Add(marker);

        // Heading arrow
        double arrowLen = 18;
        var (dx, dy) = HeadingToCanvasDirection(wp.Heading, arrowLen);
        
        var arrow = new Line
        {
            X1 = pos.X,
            Y1 = pos.Y,
            X2 = pos.X + dx,
            Y2 = pos.Y + dy,
            Stroke = Brushes.White,
            StrokeThickness = 2,
            IsHitTestVisible = false
        };
        Panel.SetZIndex(arrow, 99);
        FieldCanvas.Children.Add(arrow);
        _waypointVisuals.Add(arrow);

        // Waypoint number
        var numLabel = new TextBlock
        {
            Text = (index + 1).ToString(),
            FontSize = 9,
            FontWeight = FontWeights.Bold,
            Foreground = Brushes.Black,
            IsHitTestVisible = false
        };
        Canvas.SetLeft(numLabel, pos.X - 4);
        Canvas.SetTop(numLabel, pos.Y - 5);
        Panel.SetZIndex(numLabel, 101);
        FieldCanvas.Children.Add(numLabel);
        _waypointVisuals.Add(numLabel);
    }

    private void UpdateWaypointPositions()
    {
        if (_waypoints.Count < 1)
        {
            _pathLine.Data = null;
            return;
        }

        // Update path geometry
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
                double dist = SysMath.Sqrt(dx * dx + dy * dy);
                double cpDist = dist * 0.35 * _scale;

                var (prevDx, prevDy) = HeadingToCanvasDirection(prev.Heading, cpDist);
                var (currDx, currDy) = HeadingToCanvasDirection(curr.Heading, cpDist);

                var p1 = new Point(p0.X + prevDx, p0.Y + prevDy);
                var p2 = new Point(p3.X - currDx, p3.Y - currDy);

                figure.Segments.Add(new BezierSegment(p1, p2, p3, true));
            }

            geometry.Figures.Add(figure);
            _pathLine.Data = geometry;
        }

        // Update waypoint visual positions (3 elements per waypoint)
        int visualIndex = 0;
        for (int i = 0; i < _waypoints.Count && visualIndex + 2 < _waypointVisuals.Count; i++)
        {
            var wp = _waypoints[i];
            var pos = FieldToCanvas(wp.X, wp.Y);
            
            // Marker
            if (_waypointVisuals[visualIndex] is Ellipse marker)
            {
                Canvas.SetLeft(marker, pos.X - marker.Width / 2);
                Canvas.SetTop(marker, pos.Y - marker.Height / 2);
            }
            visualIndex++;
            
            // Arrow
            if (_waypointVisuals[visualIndex] is Line arrow)
            {
                var (dx, dy) = HeadingToCanvasDirection(wp.Heading, 18);
                arrow.X1 = pos.X;
                arrow.Y1 = pos.Y;
                arrow.X2 = pos.X + dx;
                arrow.Y2 = pos.Y + dy;
            }
            visualIndex++;
            
            // Label
            if (_waypointVisuals[visualIndex] is TextBlock label)
            {
                Canvas.SetLeft(label, pos.X - 4);
                Canvas.SetTop(label, pos.Y - 5);
            }
            visualIndex++;
        }
    }
    
    private void RequestThrottledRedraw()
    {
        if (_redrawPending) return;
        
        _redrawPending = true;
        Dispatcher.BeginInvoke(DispatcherPriority.Render, () =>
        {
            _redrawPending = false;
            UpdateWaypointPositions();
        });
    }

    private int HitTestWaypoint(Point canvasPos)
    {
        const double hitRadius = 15;
        
        for (int i = _waypoints.Count - 1; i >= 0; i--)
        {
            var wp = _waypoints[i];
            var wpPos = FieldToCanvas(wp.X, wp.Y);
            double dx = canvasPos.X - wpPos.X;
            double dy = canvasPos.Y - wpPos.Y;
            double dist = SysMath.Sqrt(dx * dx + dy * dy);
            
            if (dist <= hitRadius)
            {
                return i;
            }
        }
        return -1;
    }

    // Mouse event handlers
    private void FieldCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        var canvasPos = e.GetPosition(FieldCanvas);
        int hitIndex = HitTestWaypoint(canvasPos);
        
        if (hitIndex >= 0)
        {
            // Start dragging waypoint
            _selectedWaypointIndex = hitIndex;
            _selectedWaypoint = _waypoints[hitIndex];
            _draggingWaypoint = _selectedWaypoint;
            _isDraggingWaypoint = true;
            _lastMousePos = canvasPos;

            UpdateWaypointTextBoxes();
            FieldCanvas.CaptureMouse();
            e.Handled = true;
            RedrawPath();
        }
        else
        {
            // Start panning
            _isPanningField = true;
            _lastMousePos = e.GetPosition(this);
            _panStartOffset = _panOffset;
            FieldCanvas.CaptureMouse();
        }
    }

    private void FieldCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        if (_isDraggingWaypoint)
        {
            SaveStateForUndo();
            _isDraggingWaypoint = false;
            _draggingWaypoint = null;
            RedrawPath();
        }
        
        _isPanningField = false;
        FieldCanvas.ReleaseMouseCapture();
    }

    private void FieldCanvas_MouseMove(object sender, MouseEventArgs e)
    {
        var canvasPos = e.GetPosition(FieldCanvas);
        
        if (_isDraggingWaypoint && _draggingWaypoint != null && e.LeftButton == MouseButtonState.Pressed)
        {
            var fieldPos = CanvasToField(canvasPos.X, canvasPos.Y);

            _draggingWaypoint.X = SysMath.Clamp(fieldPos.X, 0, FieldSize);
            _draggingWaypoint.Y = SysMath.Clamp(fieldPos.Y, 0, FieldSize);

            UpdateWaypointTextBoxes();
            RequestThrottledRedraw();
            e.Handled = true;
        }
        else if (_isPanningField && e.LeftButton == MouseButtonState.Pressed)
        {
            var currentPos = e.GetPosition(this);
            double dx = currentPos.X - _lastMousePos.X;
            double dy = currentPos.Y - _lastMousePos.Y;

            _panOffset = new Point(_panStartOffset.X + dx, _panStartOffset.Y + dy);
            
            // Limit pan to reasonable bounds
            double maxPan = FieldSize * _scale;
            _panOffset = new Point(
                SysMath.Clamp(_panOffset.X, -maxPan, maxPan),
                SysMath.Clamp(_panOffset.Y, -maxPan, maxPan)
            );

            FullRedraw();
        }
    }

    private void FieldCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        // Zoom centered on mouse position
        var mousePos = e.GetPosition(FieldCanvas);
        var fieldPos = CanvasToField(mousePos.X, mousePos.Y);
        
        double zoomFactor = e.Delta > 0 ? 1.15 : 0.87;
        double newZoom = SysMath.Clamp(_zoom * zoomFactor, 0.5, 4.0);
        
        if (SysMath.Abs(newZoom - _zoom) > 0.01)
        {
            _zoom = newZoom;
            FullRedraw();
        }
    }

    private void FieldCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
    {
        var canvasPos = e.GetPosition(FieldCanvas);
        _contextMenuFieldPos = CanvasToField(canvasPos.X, canvasPos.Y);
    }

    private void UpdateWaypointTextBoxes()
    {
        if (_selectedWaypoint == null) return;
        
        _isUpdatingTextBoxes = true;
        WaypointX.Text = _selectedWaypoint.X.ToString("F1");
        WaypointY.Text = _selectedWaypoint.Y.ToString("F1");
        WaypointHeading.Text = _selectedWaypoint.Heading.ToString("F0");
        WaypointVel.Text = _selectedWaypoint.Velocity.ToString("F0");
        _isUpdatingTextBoxes = false;
    }

    // Toolbar buttons
    private void ZoomIn_Click(object sender, RoutedEventArgs e)
    {
        _zoom = SysMath.Min(_zoom * 1.2, 4.0);
        FullRedraw();
    }

    private void ZoomOut_Click(object sender, RoutedEventArgs e)
    {
        _zoom = SysMath.Max(_zoom / 1.2, 0.5);
        FullRedraw();
    }

    private void ResetView_Click(object sender, RoutedEventArgs e)
    {
        _zoom = 1.0;
        _panOffset = new Point(0, 0);
        FullRedraw();
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
        
        double newX = _waypoints.Count > 0 ? _waypoints[^1].X + 12 : 24;
        double newY = _waypoints.Count > 0 ? _waypoints[^1].Y + 12 : 24;
        double newHeading = _waypoints.Count > 0 ? _waypoints[^1].Heading : 0;

        _waypoints.Add(new WaypointVisual 
        { 
            X = SysMath.Clamp(newX, 0, FieldSize), 
            Y = SysMath.Clamp(newY, 0, FieldSize), 
            Heading = newHeading, 
            Velocity = 80 
        });
        
        _selectedWaypointIndex = _waypoints.Count - 1;
        _selectedWaypoint = _waypoints[^1];
        UpdateWaypointTextBoxes();
        
        RedrawPath();
    }

    private void AddWaypointHere_Click(object sender, RoutedEventArgs e)
    {
        SaveStateForUndo();
        
        double heading = _waypoints.Count > 0 ? _waypoints[^1].Heading : 0;
        _waypoints.Add(new WaypointVisual 
        { 
            X = SysMath.Clamp(_contextMenuFieldPos.X, 0, FieldSize), 
            Y = SysMath.Clamp(_contextMenuFieldPos.Y, 0, FieldSize), 
            Heading = heading, 
            Velocity = 80 
        });
        
        _selectedWaypointIndex = _waypoints.Count - 1;
        _selectedWaypoint = _waypoints[^1];
        UpdateWaypointTextBoxes();
        
        RedrawPath();
    }

    private void AddTurn_Click(object sender, RoutedEventArgs e)
    {
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

    // Undo/Redo using data-only copies
    private void SaveStateForUndo()
    {
        var state = _waypoints.Select(wp => new WaypointData
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
            var current = _waypoints.Select(wp => new WaypointData
            {
                X = wp.X, Y = wp.Y, Heading = wp.Heading, Velocity = wp.Velocity
            }).ToList();
            _redoStack.Push(current);

            var prev = _undoStack.Pop();
            _waypoints.Clear();
            _waypoints.AddRange(prev.Select(d => new WaypointVisual 
            { 
                X = d.X, Y = d.Y, Heading = d.Heading, Velocity = d.Velocity 
            }));
            _selectedWaypointIndex = -1;
            _selectedWaypoint = null;
            RedrawPath();
        }
    }

    private void Redo_Click(object sender, RoutedEventArgs e)
    {
        if (_redoStack.Count > 0)
        {
            var current = _waypoints.Select(wp => new WaypointData
            {
                X = wp.X, Y = wp.Y, Heading = wp.Heading, Velocity = wp.Velocity
            }).ToList();
            _undoStack.Push(current);

            var next = _redoStack.Pop();
            _waypoints.Clear();
            _waypoints.AddRange(next.Select(d => new WaypointVisual 
            { 
                X = d.X, Y = d.Y, Heading = d.Heading, Velocity = d.Velocity 
            }));
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
        double estTime = CalculateEstimatedTime();

        TimelineText.Text = $"{t * estTime:F1}s / {estTime:F1}s";
        DistanceText.Text = $"{currentDist:F1} / {totalDist:F1} in";
    }

    private void WaypointValue_Changed(object sender, TextChangedEventArgs e)
    {
        if (_selectedWaypoint == null || _isUpdatingTextBoxes) return;

        bool changed = false;
        
        if (double.TryParse(WaypointX.Text, out double x))
        {
            _selectedWaypoint.X = SysMath.Clamp(x, 0, FieldSize);
            changed = true;
        }
        if (double.TryParse(WaypointY.Text, out double y))
        {
            _selectedWaypoint.Y = SysMath.Clamp(y, 0, FieldSize);
            changed = true;
        }
        if (double.TryParse(WaypointHeading.Text, out double h))
        {
            _selectedWaypoint.Heading = h;
            changed = true;
        }
        if (double.TryParse(WaypointVel.Text, out double v))
        {
            _selectedWaypoint.Velocity = SysMath.Clamp(v, 0, 100);
            changed = true;
        }

        if (changed)
        {
            RedrawPath();
        }
    }

    // Statistics
    private void UpdateStatistics()
    {
        if (!_isLoaded) return;
        
        double totalDist = CalculateTotalDistance();
        double estTime = CalculateEstimatedTime();

        if (TotalDistanceText != null) TotalDistanceText.Text = $"{totalDist:F1} in";
        if (TotalTimeText != null) TotalTimeText.Text = $"{estTime:F1}s";
        if (WaypointStatsText != null) WaypointStatsText.Text = _waypoints.Count.ToString();
        if (WaypointCountText != null) WaypointCountText.Text = _waypoints.Count.ToString();
        if (MaxCurvatureText != null) MaxCurvatureText.Text = $"{CalculateMaxCurvature():F3} /in";
        if (DistanceText != null) DistanceText.Text = $"0.0 / {totalDist:F1} in";
        if (TimelineText != null) TimelineText.Text = $"0.0s / {estTime:F1}s";
    }

    private double CalculateTotalDistance()
    {
        double dist = 0;
        for (int i = 1; i < _waypoints.Count; i++)
        {
            double dx = _waypoints[i].X - _waypoints[i - 1].X;
            double dy = _waypoints[i].Y - _waypoints[i - 1].Y;
            dist += SysMath.Sqrt(dx * dx + dy * dy);
        }
        return dist;
    }

    private double CalculateEstimatedTime()
    {
        double dist = CalculateTotalDistance();
        double avgVel = MaxVelSlider?.Value ?? 60 * 0.7;
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

            // Menger curvature
            double a = SysMath.Sqrt(SysMath.Pow(p1.X - p0.X, 2) + SysMath.Pow(p1.Y - p0.Y, 2));
            double b = SysMath.Sqrt(SysMath.Pow(p2.X - p1.X, 2) + SysMath.Pow(p2.Y - p1.Y, 2));
            double c = SysMath.Sqrt(SysMath.Pow(p2.X - p0.X, 2) + SysMath.Pow(p2.Y - p0.Y, 2));

            double area = SysMath.Abs((p1.X - p0.X) * (p2.Y - p0.Y) - (p2.X - p0.X) * (p1.Y - p0.Y)) / 2;
            double denom = a * b * c;

            if (denom > 0.001)
            {
                double curvature = 4 * area / denom;
                maxCurv = SysMath.Max(maxCurv, curvature);
            }
        }
        return maxCurv;
    }

    // Code generation
    private void UpdateCodePreview()
    {
        if (!_isLoaded || CodePreview == null || MaxVelSlider == null || LookaheadSlider == null) return;
        
        var sb = new StringBuilder();
        sb.AppendLine("// ControlWorkbench Path");
        sb.AppendLine("// For use with CWB PROS Library");
        sb.AppendLine();
        
        sb.AppendLine("std::vector<cwb::Waypoint> path = {");
        foreach (var wp in _waypoints)
        {
            sb.AppendLine($"    {{{wp.X:F1}, {wp.Y:F1}, {wp.Velocity:F0}, {wp.Heading:F0}}},");
        }
        sb.AppendLine("};");
        sb.AppendLine();
        sb.AppendLine($"// Max velocity: {MaxVelSlider.Value:F0} in/s");
        sb.AppendLine($"// Lookahead: {LookaheadSlider.Value:F0} in");
        sb.AppendLine($"// Estimated time: {CalculateEstimatedTime():F1}s");
        sb.AppendLine();
        sb.AppendLine("void run_path() {");
        sb.AppendLine("    chassis.follow_path(path, cwb::MoveOptions()");
        sb.AppendLine($"        .set_max_speed({(int)(MaxVelSlider.Value * 127 / 100)})");
        sb.AppendLine($"        .set_timeout({(int)(CalculateEstimatedTime() * 1000 + 2000)}));");
        sb.AppendLine("}");

        CodePreview.Text = sb.ToString();
    }

    private void CopyLemLibCode_Click(object sender, RoutedEventArgs e)
    {
        var sb = new StringBuilder();
        sb.AppendLine("// LemLib Path");
        sb.AppendLine($"ASSET(path_txt) = {{");
        foreach (var wp in _waypoints)
        {
            sb.AppendLine($"    {wp.X:F1}, {wp.Y:F1}, {wp.Heading:F0},");
        }
        sb.AppendLine("};");
        sb.AppendLine();
        sb.AppendLine($"chassis.follow(path_txt, {LookaheadSlider.Value:F0}, {(int)(CalculateEstimatedTime() * 1000 + 1000)});");
        
        Clipboard.SetText(sb.ToString());
        MessageBox.Show("LemLib code copied to clipboard!", "Copied", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void CopyPurePursuitCode_Click(object sender, RoutedEventArgs e)
    {
        Clipboard.SetText(CodePreview.Text);
        MessageBox.Show("Code copied to clipboard!", "Copied", MessageBoxButton.OK, MessageBoxImage.Information);
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

                _selectedWaypointIndex = -1;
                _selectedWaypoint = null;
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

/// <summary>
/// Immutable data for undo/redo.
/// </summary>
public record WaypointData
{
    public double X { get; init; }
    public double Y { get; init; }
    public double Heading { get; init; }
    public double Velocity { get; init; }
}

/// <summary>
/// Mutable waypoint for editing.
/// </summary>
public class WaypointVisual
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Heading { get; set; }
    public double Velocity { get; set; } = 80;
    public List<string> Actions { get; } = new();
}
