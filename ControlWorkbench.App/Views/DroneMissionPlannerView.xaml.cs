using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using ControlWorkbench.Drone.Mission;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Fully functional drone mission planner view with interactive map.
/// </summary>
public partial class DroneMissionPlannerView : UserControl
{
    private readonly MissionPlanner _missionPlanner = new();
    
    private int _selectedIndex = -1;
    private bool _isDragging;
    private Point _lastMousePos;
    private Point _mapOffset = new(0, 0);
    private double _zoom = 1.0;
    private bool _isLoaded;
    private bool _updatingInputs;

    private double _centerLat = 40.7128;
    private double _centerLon = -74.0060;

    public DroneMissionPlannerView()
    {
        InitializeComponent();
        Loaded += OnLoaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        if (_isLoaded) return;
        _isLoaded = true;
        
        _missionPlanner.HomePosition = new GeoPoint(_centerLat, _centerLon, 0);
        HomePositionText.Text = $"{_centerLat:F6}, {_centerLon:F6}";
        
        // Wire up map events
        MapCanvas.MouseLeftButtonDown += MapCanvas_MouseLeftButtonDown;
        MapCanvas.MouseLeftButtonUp += MapCanvas_MouseLeftButtonUp;
        MapCanvas.MouseMove += MapCanvas_MouseMove;
        MapCanvas.MouseWheel += MapCanvas_MouseWheel;
        MapCanvas.MouseRightButtonDown += MapCanvas_MouseRightButtonDown;
        MapCanvas.SizeChanged += (s, ev) => RedrawMap();
        
        RedrawMap();
        RefreshMissionList();
    }

    // ========== ADD MISSION ITEMS ==========
    
    private void AddTakeoff_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        _missionPlanner.AddTakeoff(alt);
        RefreshMissionList();
        RedrawMap();
    }

    private void AddWaypoint_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        double speed = ParseDouble(DefaultSpeedInput.Text, 5);
        
        // Add at center of current view
        _missionPlanner.AddWaypoint(_centerLat, _centerLon, alt, speed);
        RefreshMissionList();
        RedrawMap();
    }

    private void AddLoiter_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        _missionPlanner.AddLoiter(_centerLat, _centerLon, alt, 30);
        RefreshMissionList();
        RedrawMap();
    }

    private void AddRTL_Click(object sender, RoutedEventArgs e)
    {
        _missionPlanner.AddRTL();
        RefreshMissionList();
        RedrawMap();
    }

    private void AddLand_Click(object sender, RoutedEventArgs e)
    {
        _missionPlanner.AddLand();
        RefreshMissionList();
        RedrawMap();
    }

    private void DeleteItem_Click(object sender, RoutedEventArgs e)
    {
        if (_selectedIndex >= 0 && _selectedIndex < _missionPlanner.Items.Count)
        {
            _missionPlanner.RemoveItem(_selectedIndex);
            _selectedIndex = -1;
            RefreshMissionList();
            RedrawMap();
            UpdateWaypointPanel();
        }
    }

    private void ClearMission_Click(object sender, RoutedEventArgs e)
    {
        _missionPlanner.Clear();
        _selectedIndex = -1;
        RefreshMissionList();
        RedrawMap();
        UpdateWaypointPanel();
    }

    // ========== MISSION LIST ==========
    
    private void RefreshMissionList()
    {
        MissionItemsList.Items.Clear();
        
        for (int i = 0; i < _missionPlanner.Items.Count; i++)
        {
            var item = _missionPlanner.Items[i];
            var listItem = CreateMissionListItem(i, item);
            MissionItemsList.Items.Add(listItem);
        }
        
        if (_selectedIndex >= 0 && _selectedIndex < MissionItemsList.Items.Count)
        {
            MissionItemsList.SelectedIndex = _selectedIndex;
        }
        
        UpdateStatistics();
    }

    private ListBoxItem CreateMissionListItem(int index, MissionItem item)
    {
        var border = new Border
        {
            Padding = new Thickness(8, 6, 8, 6),
            Margin = new Thickness(0, 1, 0, 0),
            Background = new SolidColorBrush(Color.FromRgb(62, 62, 66)),
            CornerRadius = new CornerRadius(3)
        };

        var grid = new Grid();
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(25) });
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

        var indexText = new TextBlock
        {
            Text = (index + 1).ToString(),
            Foreground = new SolidColorBrush(Color.FromRgb(136, 136, 136)),
            FontWeight = FontWeights.Bold
        };
        Grid.SetColumn(indexText, 0);

        var stack = new StackPanel();
        Grid.SetColumn(stack, 1);

        var typeColor = item.Type switch
        {
            MissionItemType.Takeoff => Color.FromRgb(76, 175, 80),
            MissionItemType.Waypoint => Color.FromRgb(0, 188, 212),
            MissionItemType.Loiter => Color.FromRgb(255, 152, 0),
            MissionItemType.Land => Color.FromRgb(255, 235, 59),
            MissionItemType.ReturnToLaunch => Color.FromRgb(244, 67, 54),
            _ => Colors.White
        };

        var typeText = new TextBlock
        {
            Text = item.Type.ToString().ToUpper(),
            Foreground = new SolidColorBrush(typeColor),
            FontWeight = FontWeights.SemiBold
        };

        var detailsText = new TextBlock
        {
            Text = GetItemDetails(item),
            Foreground = new SolidColorBrush(Color.FromRgb(136, 136, 136)),
            FontSize = 10,
            FontFamily = new FontFamily("Consolas")
        };

        stack.Children.Add(typeText);
        stack.Children.Add(detailsText);

        grid.Children.Add(indexText);
        grid.Children.Add(stack);
        border.Child = grid;

        return new ListBoxItem { Content = border, Tag = index };
    }

    private string GetItemDetails(MissionItem item)
    {
        return item.Type switch
        {
            MissionItemType.Takeoff => $"Alt: {item.Location?.Altitude:F0}m",
            MissionItemType.Waypoint => item.Location != null 
                ? $"{item.Location.Latitude:F5}, {item.Location.Longitude:F5}, {item.Location.Altitude:F0}m" 
                : "",
            MissionItemType.Loiter => $"Hold {item.HoldTime:F0}s at {item.Location?.Altitude:F0}m",
            MissionItemType.Land => "Land at current position",
            MissionItemType.ReturnToLaunch => "Return to home",
            _ => ""
        };
    }

    private void MissionItemsList_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (MissionItemsList.SelectedItem is ListBoxItem listItem && listItem.Tag is int index)
        {
            _selectedIndex = index;
            UpdateWaypointPanel();
            RedrawMap();
        }
    }

    // ========== WAYPOINT EDITING ==========
    
    private void UpdateWaypointPanel()
    {
        if (_selectedIndex < 0 || _selectedIndex >= _missionPlanner.Items.Count)
        {
            SelectedWaypointTitle.Text = "No Selection";
            WaypointEditPanel.Visibility = Visibility.Collapsed;
            return;
        }

        var item = _missionPlanner.Items[_selectedIndex];
        SelectedWaypointTitle.Text = $"{item.Type} #{_selectedIndex + 1}";
        
        if (item.Location != null)
        {
            _updatingInputs = true;
            WpLatInput.Text = item.Location.Latitude.ToString("F6");
            WpLonInput.Text = item.Location.Longitude.ToString("F6");
            WpAltInput.Text = item.Location.Altitude.ToString("F0");
            WpSpeedInput.Text = item.Speed.ToString("F1");
            WpHoldInput.Text = item.HoldTime.ToString("F0");
            _updatingInputs = false;
            WaypointEditPanel.Visibility = Visibility.Visible;
        }
        else
        {
            WaypointEditPanel.Visibility = Visibility.Collapsed;
        }
    }

    private void WaypointInput_Changed(object sender, TextChangedEventArgs e)
    {
        if (_updatingInputs || _selectedIndex < 0 || _selectedIndex >= _missionPlanner.Items.Count)
            return;

        var item = _missionPlanner.Items[_selectedIndex];
        if (item.Location == null) return;

        if (double.TryParse(WpLatInput.Text, out double lat))
            item.Location.Latitude = lat;
        if (double.TryParse(WpLonInput.Text, out double lon))
            item.Location.Longitude = lon;
        if (double.TryParse(WpAltInput.Text, out double alt))
            item.Location.Altitude = alt;
        if (double.TryParse(WpSpeedInput.Text, out double speed))
            item.Speed = speed;
        if (double.TryParse(WpHoldInput.Text, out double hold))
            item.HoldTime = hold;

        RefreshMissionList();
        RedrawMap();
    }

    // ========== MAP RENDERING ==========
    
    private void RedrawMap()
    {
        if (MapCanvas == null || !_isLoaded) return;
        
        MapCanvas.Children.Clear();

        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 600;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 400;

        // Background
        var bg = new Rectangle { Width = w, Height = h, Fill = new SolidColorBrush(Color.FromRgb(26, 26, 46)) };
        MapCanvas.Children.Add(bg);

        // Grid
        DrawGrid(w, h);

        // Flight path
        DrawFlightPath();

        // Waypoint markers
        DrawWaypoints();

        // Home marker
        DrawHomeMarker();

        // Info text
        var infoText = new TextBlock
        {
            Text = $"Center: {_centerLat:F4}, {_centerLon:F4} | Zoom: {_zoom:F1}x",
            FontSize = 11,
            Foreground = new SolidColorBrush(Color.FromRgb(80, 80, 120))
        };
        Canvas.SetLeft(infoText, 10);
        Canvas.SetTop(infoText, 10);
        MapCanvas.Children.Add(infoText);
    }

    private void DrawGrid(double w, double h)
    {
        double spacing = 50 * _zoom;
        var gridBrush = new SolidColorBrush(Color.FromRgb(40, 40, 70));

        for (double x = (_mapOffset.X % spacing + spacing) % spacing; x < w; x += spacing)
        {
            MapCanvas.Children.Add(new Line { X1 = x, Y1 = 0, X2 = x, Y2 = h, Stroke = gridBrush, StrokeThickness = 1 });
        }
        for (double y = (_mapOffset.Y % spacing + spacing) % spacing; y < h; y += spacing)
        {
            MapCanvas.Children.Add(new Line { X1 = 0, Y1 = y, X2 = w, Y2 = y, Stroke = gridBrush, StrokeThickness = 1 });
        }
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

    private void DrawWaypoints()
    {
        for (int i = 0; i < _missionPlanner.Items.Count; i++)
        {
            var item = _missionPlanner.Items[i];
            if (item.Location == null) continue;

            var pos = LatLonToCanvas(item.Location.Latitude, item.Location.Longitude);

            var color = item.Type switch
            {
                MissionItemType.Takeoff => Colors.LimeGreen,
                MissionItemType.Waypoint => Colors.DeepSkyBlue,
                MissionItemType.Loiter => Colors.Orange,
                MissionItemType.Land => Colors.Yellow,
                MissionItemType.ReturnToLaunch => Colors.Red,
                _ => Colors.White
            };

            bool isSelected = i == _selectedIndex;
            double size = isSelected ? 20 : 16;

            var marker = new Ellipse
            {
                Width = size,
                Height = size,
                Fill = new SolidColorBrush(color),
                Stroke = isSelected ? Brushes.Yellow : Brushes.White,
                StrokeThickness = isSelected ? 3 : 2,
                Cursor = Cursors.Hand,
                Tag = i
            };

            Canvas.SetLeft(marker, pos.X - size / 2);
            Canvas.SetTop(marker, pos.Y - size / 2);
            Panel.SetZIndex(marker, 100);

            int capturedIndex = i;
            marker.MouseLeftButtonDown += (s, e) =>
            {
                _selectedIndex = capturedIndex;
                MissionItemsList.SelectedIndex = capturedIndex;
                UpdateWaypointPanel();
                RedrawMap();
                e.Handled = true;
            };

            MapCanvas.Children.Add(marker);

            // Number label
            var label = new TextBlock
            {
                Text = (i + 1).ToString(),
                FontSize = 10,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.Black
            };
            Canvas.SetLeft(label, pos.X - 4);
            Canvas.SetTop(label, pos.Y - 5);
            Panel.SetZIndex(label, 101);
            MapCanvas.Children.Add(label);
        }
    }

    private void DrawHomeMarker()
    {
        if (_missionPlanner.HomePosition == null) return;
        
        var pos = LatLonToCanvas(_missionPlanner.HomePosition.Latitude, _missionPlanner.HomePosition.Longitude);
        
        var home = new Polygon
        {
            Points = new PointCollection { new(0, 10), new(5, 0), new(10, 10), new(5, 8) },
            Fill = Brushes.White,
            Stroke = Brushes.Black,
            StrokeThickness = 1
        };
        Canvas.SetLeft(home, pos.X - 5);
        Canvas.SetTop(home, pos.Y + 12);
        MapCanvas.Children.Add(home);
    }

    // ========== MAP INTERACTION ==========
    
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
            _mapOffset.X += currentPos.X - _lastMousePos.X;
            _mapOffset.Y += currentPos.Y - _lastMousePos.Y;
            _lastMousePos = currentPos;
            RedrawMap();
        }
    }

    private void MapCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        _zoom = System.Math.Clamp(_zoom * (e.Delta > 0 ? 1.2 : 0.8), 0.5, 5.0);
        RedrawMap();
    }

    private void MapCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
    {
        var pos = e.GetPosition(MapCanvas);
        var (lat, lon) = CanvasToLatLon(pos.X, pos.Y);

        double alt = ParseDouble(DefaultAltInput.Text, 50);
        double speed = ParseDouble(DefaultSpeedInput.Text, 5);

        // Insert before RTL if last item is RTL
        var lastItem = _missionPlanner.Items.LastOrDefault();
        if (lastItem?.Type == MissionItemType.ReturnToLaunch)
        {
            _missionPlanner.RemoveItem(_missionPlanner.Items.Count - 1);
            _missionPlanner.AddWaypoint(lat, lon, alt, speed);
            _missionPlanner.AddRTL();
        }
        else
        {
            _missionPlanner.AddWaypoint(lat, lon, alt, speed);
        }

        RefreshMissionList();
        RedrawMap();
    }

    private void ZoomIn_Click(object sender, RoutedEventArgs e)
    {
        _zoom = System.Math.Min(_zoom * 1.3, 5.0);
        RedrawMap();
    }

    private void ZoomOut_Click(object sender, RoutedEventArgs e)
    {
        _zoom = System.Math.Max(_zoom / 1.3, 0.5);
        RedrawMap();
    }

    private void ResetView_Click(object sender, RoutedEventArgs e)
    {
        _zoom = 1.0;
        _mapOffset = new Point(0, 0);
        RedrawMap();
    }

    // ========== COORDINATE CONVERSION ==========
    
    private Point LatLonToCanvas(double lat, double lon)
    {
        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 600;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 400;
        double scale = 100000 * _zoom;
        double x = (lon - _centerLon) * scale * System.Math.Cos(_centerLat * System.Math.PI / 180);
        double y = -((lat - _centerLat) * scale);
        return new Point(w / 2 + x + _mapOffset.X, h / 2 + y + _mapOffset.Y);
    }

    private (double lat, double lon) CanvasToLatLon(double canvasX, double canvasY)
    {
        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 600;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 400;
        double scale = 100000 * _zoom;
        double x = canvasX - w / 2 - _mapOffset.X;
        double y = canvasY - h / 2 - _mapOffset.Y;
        double lon = x / (scale * System.Math.Cos(_centerLat * System.Math.PI / 180)) + _centerLon;
        double lat = -y / scale + _centerLat;
        return (lat, lon);
    }

    // ========== STATISTICS ==========
    
    private void UpdateStatistics()
    {
        var stats = _missionPlanner.CalculateStatistics();
        TotalDistanceText.Text = stats.TotalDistanceMeters < 1000 
            ? $"{stats.TotalDistanceMeters:F0} m" 
            : $"{stats.TotalDistanceMeters / 1000:F2} km";
        EstTimeText.Text = stats.EstimatedTime.ToString(@"m\:ss");
        WaypointCountText.Text = stats.WaypointCount.ToString();
        MaxAltText.Text = $"{stats.MaxAltitude:F0}m";
    }

    // ========== EXPORT ==========
    
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
                sb.AppendLine($"{i}\t{(i == 0 ? 1 : 0)}\t3\t{(int)wp.Command}\t{wp.Param1}\t{wp.Param2}\t{wp.Param3}\t{wp.Param4}\t{wp.Latitude:F7}\t{wp.Longitude:F7}\t{wp.Altitude:F1}\t1");
            }

            System.IO.File.WriteAllText(dialog.FileName, sb.ToString());
            MessageBox.Show($"Mission saved with {waypoints.Count} waypoints!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
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
            string json = _missionPlanner.ToQgcPlanJson();
            System.IO.File.WriteAllText(dialog.FileName, json);
            MessageBox.Show("QGC Plan saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    // ========== HELPERS ==========
    
    private static double ParseDouble(string text, double defaultValue)
    {
        return double.TryParse(text, out double result) ? result : defaultValue;
    }
}
