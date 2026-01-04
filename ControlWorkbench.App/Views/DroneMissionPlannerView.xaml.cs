using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using ControlWorkbench.Drone.Mission;
using ControlWorkbench.Drone.Services;
using ControlWorkbench.Drone.Devices;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Drone mission planner view with interactive map.
/// Uses backend MissionPlanner for mission management.
/// </summary>
public partial class DroneMissionPlannerView : UserControl
{
    // Backend mission planner
    private readonly MissionPlanner _missionPlanner = new();
    
    // Connection service for uploading missions
    private DroneConnectionService? _droneService;
    
    private int _selectedWaypointIndex = -1;
    private bool _isDragging;
    private Point _lastMousePos;
    private Point _mapOffset = new(0, 0);
    private double _zoom = 1.0;

    // Map bounds (simulated)
    private double _centerLat = 40.7128;
    private double _centerLon = -74.0060;
    private const double MetersPerPixel = 0.5;

    public DroneMissionPlannerView()
    {
        InitializeComponent();
        
        // Subscribe to mission statistics updates
        _missionPlanner.StatisticsUpdated += OnMissionStatisticsUpdated;
        
        Loaded += OnLoaded;
    }
    
    /// <summary>
    /// Set the drone connection service for mission upload capability.
    /// </summary>
    public void SetDroneService(DroneConnectionService service)
    {
        _droneService = service;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        // Set home position
        _missionPlanner.HomePosition = new GeoPoint(_centerLat, _centerLon, 0);
        
        // Add sample waypoints using the backend
        AddSampleMission();
        WireUpEvents();
        RedrawMap();
        UpdateMissionStats();
    }

    private void WireUpEvents()
    {
        MapCanvas.MouseLeftButtonDown += MapCanvas_MouseLeftButtonDown;
        MapCanvas.MouseLeftButtonUp += MapCanvas_MouseLeftButtonUp;
        MapCanvas.MouseMove += MapCanvas_MouseMove;
        MapCanvas.MouseWheel += MapCanvas_MouseWheel;
        MapCanvas.MouseRightButtonDown += MapCanvas_MouseRightButtonDown;
    }

    private void AddSampleMission()
    {
        _missionPlanner.Clear();
        
        // Takeoff
        _missionPlanner.AddTakeoff(50);

        // Waypoint 1
        _missionPlanner.AddWaypoint(40.7135, -74.0055, 50, 5);

        // Loiter
        _missionPlanner.AddLoiter(40.7140, -74.0045, 50, 30);

        // Waypoint 2
        _missionPlanner.AddWaypoint(40.7138, -74.0035, 50, 5);

        // RTL
        _missionPlanner.AddRTL();
    }
    
    private void OnMissionStatisticsUpdated(MissionStatistics stats)
    {
        Dispatcher.BeginInvoke(() =>
        {
            // Update statistics display if we have named elements
            // StatsText.Text = stats.ToString();
        });
    }

    private void RedrawMap()
    {
        MapCanvas.Children.Clear();

        double canvasWidth = MapCanvas.ActualWidth;
        double canvasHeight = MapCanvas.ActualHeight;
        if (canvasWidth <= 0) canvasWidth = 600;
        if (canvasHeight <= 0) canvasHeight = 400;

        // Draw grid (simulated map)
        DrawMapGrid(canvasWidth, canvasHeight);

        // Draw flight path
        DrawFlightPath();

        // Draw waypoint markers
        DrawWaypointMarkers();

        // Draw home icon
        if (_missionPlanner.HomePosition != null)
        {
            var homePos = LatLonToCanvas(_missionPlanner.HomePosition.Latitude, _missionPlanner.HomePosition.Longitude);
            DrawHomeIcon(homePos);
        }
    }

    private void DrawMapGrid(double width, double height)
    {
        // Dark background
        var bg = new Rectangle
        {
            Width = width,
            Height = height,
            Fill = new SolidColorBrush(Color.FromRgb(26, 26, 46))
        };
        MapCanvas.Children.Add(bg);

        // Grid lines
        double gridSpacing = 50 * _zoom;
        for (double x = _mapOffset.X % gridSpacing; x < width; x += gridSpacing)
        {
            var line = new Line
            {
                X1 = x, Y1 = 0,
                X2 = x, Y2 = height,
                Stroke = new SolidColorBrush(Color.FromRgb(40, 40, 70)),
                StrokeThickness = 1
            };
            MapCanvas.Children.Add(line);
        }
        for (double y = _mapOffset.Y % gridSpacing; y < height; y += gridSpacing)
        {
            var line = new Line
            {
                X1 = 0, Y1 = y,
                X2 = width, Y2 = y,
                Stroke = new SolidColorBrush(Color.FromRgb(40, 40, 70)),
                StrokeThickness = 1
            };
            MapCanvas.Children.Add(line);
        }

        // Map label
        var label = new TextBlock
        {
            Text = "Simulated Map View",
            FontSize = 14,
            Foreground = new SolidColorBrush(Color.FromRgb(60, 60, 100))
        };
        Canvas.SetLeft(label, 10);
        Canvas.SetTop(label, 10);
        MapCanvas.Children.Add(label);

        var coordsLabel = new TextBlock
        {
            Text = $"Center: {_centerLat:F4}, {_centerLon:F4}",
            FontSize = 11,
            Foreground = new SolidColorBrush(Color.FromRgb(80, 80, 120))
        };
        Canvas.SetLeft(coordsLabel, 10);
        Canvas.SetTop(coordsLabel, 28);
        MapCanvas.Children.Add(coordsLabel);
        
        // Mission statistics
        var stats = _missionPlanner.CalculateStatistics();
        var statsLabel = new TextBlock
        {
            Text = $"Distance: {stats.TotalDistanceMeters:F0}m | Time: {stats.EstimatedTime:mm\\:ss} | Waypoints: {stats.WaypointCount}",
            FontSize = 11,
            Foreground = new SolidColorBrush(Color.FromRgb(100, 180, 100))
        };
        Canvas.SetLeft(statsLabel, 10);
        Canvas.SetTop(statsLabel, 46);
        MapCanvas.Children.Add(statsLabel);
    }

    private void DrawFlightPath()
    {
        var items = _missionPlanner.Items.Where(i => i.Location != null).ToList();
        if (items.Count < 2) return;

        var pathLine = new Polyline
        {
            Stroke = new SolidColorBrush(Color.FromRgb(0, 188, 212)),
            StrokeThickness = 3,
            StrokeDashArray = new DoubleCollection { 5, 3 }
        };

        foreach (var item in items)
        {
            if (item.Location != null)
            {
                var pos = LatLonToCanvas(item.Location.Latitude, item.Location.Longitude);
                pathLine.Points.Add(pos);
            }
        }

        MapCanvas.Children.Add(pathLine);
    }

    private void DrawWaypointMarkers()
    {
        var items = _missionPlanner.Items;
        
        for (int i = 0; i < items.Count; i++)
        {
            var item = items[i];
            if (item.Location == null) continue;
            
            var pos = LatLonToCanvas(item.Location.Latitude, item.Location.Longitude);

            // Marker color based on type
            var color = item.Type switch
            {
                MissionItemType.Takeoff => Colors.LimeGreen,
                MissionItemType.Waypoint => Colors.DeepSkyBlue,
                MissionItemType.Loiter => Colors.Orange,
                MissionItemType.Orbit => Colors.Magenta,
                MissionItemType.ReturnToLaunch => Colors.Red,
                MissionItemType.Land => Colors.Yellow,
                _ => Colors.White
            };

            bool isSelected = i == _selectedWaypointIndex;

            var marker = new Ellipse
            {
                Width = isSelected ? 20 : 16,
                Height = isSelected ? 20 : 16,
                Fill = new SolidColorBrush(color),
                Stroke = isSelected ? Brushes.Yellow : Brushes.White,
                StrokeThickness = isSelected ? 3 : 2,
                Cursor = Cursors.Hand,
                Tag = i
            };

            Canvas.SetLeft(marker, pos.X - marker.Width / 2);
            Canvas.SetTop(marker, pos.Y - marker.Height / 2);
            Panel.SetZIndex(marker, 100);

            int capturedIndex = i;
            marker.MouseLeftButtonDown += (s, e) =>
            {
                _selectedWaypointIndex = capturedIndex;
                UpdateWaypointPanel();
                RedrawMap();
                e.Handled = true;
            };

            MapCanvas.Children.Add(marker);

            // Waypoint number
            var numLabel = new TextBlock
            {
                Text = (i + 1).ToString(),
                FontSize = 10,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.Black
            };
            Canvas.SetLeft(numLabel, pos.X - 4);
            Canvas.SetTop(numLabel, pos.Y - 5);
            Panel.SetZIndex(numLabel, 101);
            MapCanvas.Children.Add(numLabel);
            
            // Altitude label
            var altLabel = new TextBlock
            {
                Text = $"{item.Location.Altitude:F0}m",
                FontSize = 9,
                Foreground = new SolidColorBrush(Color.FromRgb(150, 150, 150))
            };
            Canvas.SetLeft(altLabel, pos.X + 12);
            Canvas.SetTop(altLabel, pos.Y - 5);
            Panel.SetZIndex(altLabel, 99);
            MapCanvas.Children.Add(altLabel);
        }
    }

    private void DrawHomeIcon(Point pos)
    {
        var home = new Path
        {
            Fill = Brushes.White,
            Data = Geometry.Parse("M10,20 L10,12 L5,12 L10,5 L15,12 L10,12 L10,20 Z")
        };
        Canvas.SetLeft(home, pos.X - 5);
        Canvas.SetTop(home, pos.Y + 15);
        MapCanvas.Children.Add(home);
    }

    private Point LatLonToCanvas(double lat, double lon)
    {
        double canvasWidth = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 600;
        double canvasHeight = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 400;

        // Simple Mercator-like projection for local area
        double scale = 100000 * _zoom; // meters to pixels roughly
        double x = (lon - _centerLon) * scale * System.Math.Cos(_centerLat * System.Math.PI / 180);
        double y = -((lat - _centerLat) * scale);

        return new Point(
            canvasWidth / 2 + x + _mapOffset.X,
            canvasHeight / 2 + y + _mapOffset.Y
        );
    }

    private (double lat, double lon) CanvasToLatLon(double canvasX, double canvasY)
    {
        double canvasWidth = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 600;
        double canvasHeight = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 400;

        double scale = 100000 * _zoom;
        double x = canvasX - canvasWidth / 2 - _mapOffset.X;
        double y = canvasY - canvasHeight / 2 - _mapOffset.Y;

        double lon = x / (scale * System.Math.Cos(_centerLat * System.Math.PI / 180)) + _centerLon;
        double lat = -y / scale + _centerLat;

        return (lat, lon);
    }

    private void UpdateWaypointPanel()
    {
        // This would update the right panel with selected waypoint details
        // For now, using the static XAML binding
    }

    private void UpdateMissionStats()
    {
        var stats = _missionPlanner.CalculateStatistics();
        // Update stats display - this triggers redraw which shows stats
        RedrawMap();
    }

    // Mouse events for map interaction
    private void MapCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        _isDragging = true;
        _lastMousePos = e.GetPosition(this);
        MapCanvas.CaptureMouse();
    }

    private void MapCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        _isDragging = false;
        MapCanvas.ReleaseMouseCapture();
    }

    private void MapCanvas_MouseMove(object sender, MouseEventArgs e)
    {
        if (_isDragging && e.LeftButton == MouseButtonState.Pressed)
        {
            var currentPos = e.GetPosition(this);
            double dx = currentPos.X - _lastMousePos.X;
            double dy = currentPos.Y - _lastMousePos.Y;

            _mapOffset = new Point(_mapOffset.X + dx, _mapOffset.Y + dy);
            _lastMousePos = currentPos;

            RedrawMap();
        }
    }

    private void MapCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        double zoomFactor = e.Delta > 0 ? 1.2 : 0.8;
        _zoom = System.Math.Clamp(_zoom * zoomFactor, 0.5, 5.0);
        RedrawMap();
    }

    private void MapCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
    {
        // Add waypoint at clicked position
        var pos = e.GetPosition(MapCanvas);
        var (lat, lon) = CanvasToLatLon(pos.X, pos.Y);

        // Insert before RTL if present
        var lastItem = _missionPlanner.Items.LastOrDefault();
        if (lastItem?.Type == MissionItemType.ReturnToLaunch)
        {
            // Remove RTL, add waypoint, re-add RTL
            _missionPlanner.RemoveItem(_missionPlanner.Items.Count - 1);
            _missionPlanner.AddWaypoint(lat, lon, 50, 5);
            _missionPlanner.AddRTL();
        }
        else
        {
            _missionPlanner.AddWaypoint(lat, lon, 50, 5);
        }

        RedrawMap();
        UpdateMissionStats();
    }

    // Export functions
    private void SaveWaypoints_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Waypoints file (*.waypoints)|*.waypoints|All files (*.*)|*.*",
            DefaultExt = ".waypoints",
            FileName = "mission"
        };

        if (dialog.ShowDialog() == true)
        {
            var sb = new StringBuilder();
            sb.AppendLine("QGC WPL 110");
            
            var waypoints = _missionPlanner.ToMavlinkWaypoints();
            for (int i = 0; i < waypoints.Count; i++)
            {
                var wp = waypoints[i];
                // Format: index current frame command p1 p2 p3 p4 lat lon alt autocontinue
                sb.AppendLine($"{i}\t{(i == 0 ? 1 : 0)}\t3\t{(int)wp.Command}\t{wp.Param1}\t{wp.Param2}\t{wp.Param3}\t{wp.Param4}\t{wp.Latitude:F7}\t{wp.Longitude:F7}\t{wp.Altitude:F1}\t1");
            }

            System.IO.File.WriteAllText(dialog.FileName, sb.ToString());
            MessageBox.Show("Mission saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private void SaveQGCPlan_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "QGroundControl Plan (*.plan)|*.plan|All files (*.*)|*.*",
            DefaultExt = ".plan",
            FileName = "mission"
        };

        if (dialog.ShowDialog() == true)
        {
            // Use the backend's QGC export
            string json = _missionPlanner.ToQgcPlanJson();
            System.IO.File.WriteAllText(dialog.FileName, json);
            MessageBox.Show("QGC Plan saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }
    
    /// <summary>
    /// Upload mission to connected flight controller.
    /// </summary>
    public async Task UploadMissionAsync()
    {
        if (_droneService == null || !_droneService.IsConnected)
        {
            MessageBox.Show("Not connected to a flight controller.", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        
        var fc = _droneService.GetFlightController();
        if (fc != null)
        {
            var waypoints = _missionPlanner.ToMavlinkWaypoints();
            await fc.UploadMissionAsync(waypoints);
            MessageBox.Show($"Uploaded {waypoints.Count} waypoints to flight controller.", "Upload Complete", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }
    
    /// <summary>
    /// Start the uploaded mission.
    /// </summary>
    public async Task StartMissionAsync()
    {
        if (_droneService == null || !_droneService.IsConnected)
        {
            MessageBox.Show("Not connected to a flight controller.", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        
        var fc = _droneService.GetFlightController();
        if (fc != null)
        {
            await fc.StartMissionAsync();
        }
    }
    
    /// <summary>
    /// Clear the mission.
    /// </summary>
    public void ClearMission()
    {
        _missionPlanner.Clear();
        _selectedWaypointIndex = -1;
        RedrawMap();
    }
    
    /// <summary>
    /// Add a survey grid pattern.
    /// </summary>
    public void AddSurveyGrid(double lat1, double lon1, double lat2, double lon2, double altitude, double spacing)
    {
        _missionPlanner.AddSurveyGrid(
            new GeoPoint(lat1, lon1, altitude),
            new GeoPoint(lat2, lon2, altitude),
            altitude,
            spacing);
        RedrawMap();
    }
    
    /// <summary>
    /// Get the mission planner for external access.
    /// </summary>
    public MissionPlanner GetMissionPlanner() => _missionPlanner;
}
