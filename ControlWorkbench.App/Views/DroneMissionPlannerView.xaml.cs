using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using ControlWorkbench.Drone.Mission;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Professional-grade drone mission planner with advanced features.
/// </summary>
public partial class DroneMissionPlannerView : UserControl
{
    private readonly MissionPlanner _missionPlanner = new();
    
    // Selection state
    private int _selectedIndex = -1;
    private bool _updatingInputs;
    
    // Map state
    private Point _mapOffset = new(0, 0);
    private double _zoom = 1.0;
    private bool _isDragging;
    private Point _lastMousePos;
    private bool _isLoaded;
    
    // Map center coordinates
    private double _centerLat = 40.7128;
    private double _centerLon = -74.0060;
    
    // Terrain data (simulated elevation grid)
    private readonly double[,] _terrainGrid = new double[100, 100];
    
    // Drawing constants
    private const double MetersPerDegree = 111320.0;
    
    // Performance: throttle redraws during drag
    private bool _redrawPending;
    private WriteableBitmap? _terrainBitmap;
    private bool _terrainDirty = true;
    private double _lastTerrainZoom;
    private Size _lastTerrainSize;

    public DroneMissionPlannerView()
    {
        InitializeComponent();
        InitializeTerrainData();
        Loaded += OnLoaded;
    }

    private void InitializeTerrainData()
    {
        // Generate simulated terrain (Perlin-like noise)
        var rand = new Random(42);
        for (int y = 0; y < 100; y++)
        {
            for (int x = 0; x < 100; x++)
            {
                // Simple multi-octave noise simulation
                double baseHeight = 20;
                double noise1 = System.Math.Sin(x * 0.1) * System.Math.Cos(y * 0.1) * 30;
                double noise2 = System.Math.Sin(x * 0.05 + 1) * System.Math.Sin(y * 0.07) * 50;
                double random = (rand.NextDouble() - 0.5) * 10;
                _terrainGrid[x, y] = System.Math.Max(0, baseHeight + noise1 + noise2 + random);
            }
        }
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        if (_isLoaded) return;
        _isLoaded = true;
        
        // Initialize home position
        _missionPlanner.HomePosition = new GeoPoint(_centerLat, _centerLon, 0);
        HomePositionText.Text = $"{_centerLat:F6}, {_centerLon:F6}";
        
        // Wire up map events
        MapCanvas.MouseLeftButtonDown += MapCanvas_MouseLeftButtonDown;
        MapCanvas.MouseLeftButtonUp += MapCanvas_MouseLeftButtonUp;
        MapCanvas.MouseMove += MapCanvas_MouseMove;
        MapCanvas.MouseWheel += MapCanvas_MouseWheel;
        MapCanvas.MouseRightButtonDown += MapCanvas_MouseRightButtonDown;
        MapCanvas.SizeChanged += (s, ev) => { _terrainDirty = true; RedrawMap(); RedrawAltitudeProfile(); };
        
        // Checkbox events
        ShowGridCheck.Checked += (s, ev) => RedrawMap();
        ShowGridCheck.Unchecked += (s, ev) => RedrawMap();
        ShowTerrainCheck.Checked += (s, ev) => { _terrainDirty = true; RedrawMap(); };
        ShowTerrainCheck.Unchecked += (s, ev) => RedrawMap();
        
        RedrawMap();
        RedrawAltitudeProfile();
        UpdateStatistics();
    }

    // ==================== MISSION ITEM COMMANDS ====================
    
    private void AddTakeoff_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        _missionPlanner.AddTakeoff(alt);
        OnMissionChanged();
    }

    private void AddWaypoint_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        double speed = ParseDouble(DefaultSpeedInput.Text, 5);
        
        // Offset from center based on existing waypoints
        double offsetLat = _missionPlanner.Items.Count * 0.0001;
        double offsetLon = _missionPlanner.Items.Count * 0.0001;
        
        _missionPlanner.AddWaypoint(_centerLat + offsetLat, _centerLon + offsetLon, alt, speed);
        OnMissionChanged();
    }

    private void AddLoiter_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        _missionPlanner.AddLoiter(_centerLat, _centerLon, alt, 30);
        OnMissionChanged();
    }

    private void AddOrbit_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        _missionPlanner.AddOrbit(_centerLat, _centerLon, alt, 50, 2); // 50m radius, 2 turns
        OnMissionChanged();
    }

    private void AddROI_Click(object sender, RoutedEventArgs e)
    {
        double alt = ParseDouble(DefaultAltInput.Text, 50);
        _missionPlanner.AddROI(_centerLat, _centerLon, alt);
        OnMissionChanged();
    }

    private void AddLand_Click(object sender, RoutedEventArgs e)
    {
        _missionPlanner.AddLand();
        OnMissionChanged();
    }

    private void AddRTL_Click(object sender, RoutedEventArgs e)
    {
        _missionPlanner.AddRTL();
        OnMissionChanged();
    }

    private void DeleteItem_Click(object sender, RoutedEventArgs e)
    {
        if (_selectedIndex >= 0 && _selectedIndex < _missionPlanner.Items.Count)
        {
            _missionPlanner.RemoveItem(_selectedIndex);
            _selectedIndex = -1;
            OnMissionChanged();
            UpdateWaypointPanel();
        }
    }

    private void ClearMission_Click(object sender, RoutedEventArgs e)
    {
        var result = MessageBox.Show("Clear all mission items?", "Confirm", MessageBoxButton.YesNo, MessageBoxImage.Warning);
        if (result == MessageBoxResult.Yes)
        {
            _missionPlanner.Clear();
            _selectedIndex = -1;
            OnMissionChanged();
            UpdateWaypointPanel();
        }
    }

    private void SetHome_Click(object sender, RoutedEventArgs e)
    {
        _missionPlanner.HomePosition = new GeoPoint(_centerLat, _centerLon, 0);
        HomePositionText.Text = $"{_centerLat:F6}, {_centerLon:F6}";
        RedrawMap();
    }

    // ==================== TOOLBAR COMMANDS ====================
    
    private void Upload_Click(object sender, RoutedEventArgs e)
    {
        if (_missionPlanner.Items.Count == 0)
        {
            MessageBox.Show("No mission items to upload.", "Upload", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        
        // Validate mission first
        var validation = ValidateMission();
        if (!validation.isValid)
        {
            MessageBox.Show($"Mission validation failed:\n{validation.message}", "Validation Error", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        
        MessageBox.Show($"Mission uploaded with {_missionPlanner.Items.Count} items.\n(Simulation mode - no actual connection)", 
            "Upload Complete", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void Download_Click(object sender, RoutedEventArgs e)
    {
        MessageBox.Show("Download mission from vehicle.\n(Simulation mode - generating sample mission)", 
            "Download", MessageBoxButton.OK, MessageBoxImage.Information);
        
        // Generate sample mission
        _missionPlanner.Clear();
        _missionPlanner.AddTakeoff(50);
        _missionPlanner.AddWaypoint(_centerLat + 0.001, _centerLon + 0.001, 50, 5);
        _missionPlanner.AddWaypoint(_centerLat + 0.002, _centerLon, 60, 5);
        _missionPlanner.AddWaypoint(_centerLat + 0.001, _centerLon - 0.001, 50, 5);
        _missionPlanner.AddRTL();
        OnMissionChanged();
    }

    private void Verify_Click(object sender, RoutedEventArgs e)
    {
        var validation = ValidateMission();
        if (validation.isValid)
        {
            SafetyStatusText.Text = "OK";
            SafetyStatusText.Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80));
            MessageBox.Show("Mission verification passed!\n\n" + validation.message, "Verification", MessageBoxButton.OK, MessageBoxImage.Information);
        }
        else
        {
            SafetyStatusText.Text = "WARN";
            SafetyStatusText.Foreground = new SolidColorBrush(Color.FromRgb(255, 152, 0));
            MessageBox.Show("Mission verification warnings:\n\n" + validation.message, "Verification", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
    }

    private (bool isValid, string message) ValidateMission()
    {
        var issues = new List<string>();
        var stats = _missionPlanner.CalculateStatistics();
        
        double maxAlt = ParseDouble(MaxAltLimitInput.Text, 120);
        double maxDist = ParseDouble(MaxDistLimitInput.Text, 5000);
        
        if (stats.MaxAltitude > maxAlt)
            issues.Add($"Max altitude ({stats.MaxAltitude:F0}m) exceeds limit ({maxAlt:F0}m)");
        
        if (stats.TotalDistanceMeters > maxDist)
            issues.Add($"Total distance ({stats.TotalDistanceMeters:F0}m) exceeds limit ({maxDist:F0}m)");
        
        // Check for takeoff
        if (!_missionPlanner.Items.Any(i => i.Type == MissionItemType.Takeoff))
            issues.Add("Mission should start with a Takeoff command");
        
        // Check for end command
        var lastItem = _missionPlanner.Items.LastOrDefault();
        if (lastItem != null && lastItem.Type != MissionItemType.ReturnToLaunch && lastItem.Type != MissionItemType.Land)
            issues.Add("Mission should end with RTL or Land command");
        
        // Check terrain clearance
        if (TerrainFollowCheck.IsChecked == true)
        {
            foreach (var item in _missionPlanner.Items)
            {
                if (item.Location != null)
                {
                    double terrain = GetTerrainHeight(item.Location.Latitude, item.Location.Longitude);
                    if (item.Location.Altitude < terrain + 10)
                        issues.Add($"Waypoint {item.Sequence} may be too close to terrain");
                }
            }
        }
        
        // Battery estimation
        double estBattery = EstimateBatteryUsage();
        double minBattery = ParseDouble(MinBatteryInput.Text, 25);
        if (100 - estBattery < minBattery)
            issues.Add($"Estimated remaining battery ({100 - estBattery:F0}%) below minimum ({minBattery:F0}%)");
        
        if (issues.Count == 0)
            return (true, $"All checks passed.\n• {stats.WaypointCount} waypoints\n• {stats.TotalDistanceMeters:F0}m total distance\n• {stats.EstimatedTime.TotalMinutes:F1} minutes estimated");
        
        return (false, string.Join("\n• ", issues));
    }

    private void FitToMission_Click(object sender, RoutedEventArgs e)
    {
        if (_missionPlanner.Items.Count == 0) return;
        
        var stats = _missionPlanner.CalculateStatistics();
        _centerLat = (stats.MinLatitude + stats.MaxLatitude) / 2;
        _centerLon = (stats.MinLongitude + stats.MaxLongitude) / 2;
        
        // Calculate zoom to fit
        double latSpan = stats.MaxLatitude - stats.MinLatitude;
        double lonSpan = stats.MaxLongitude - stats.MinLongitude;
        double maxSpan = System.Math.Max(latSpan, lonSpan);
        
        if (maxSpan > 0)
            _zoom = System.Math.Clamp(0.001 / maxSpan, 0.5, 5.0);
        
        _mapOffset = new Point(0, 0);
        _terrainDirty = true;
        RedrawMap();
    }

    private void ZoomIn_Click(object sender, RoutedEventArgs e)
    {
        _zoom = System.Math.Min(_zoom * 1.3, 10.0);
        ZoomLevelText.Text = $"Zoom: {_zoom:F1}x";
        _terrainDirty = true;
        RedrawMap();
    }

    private void ZoomOut_Click(object sender, RoutedEventArgs e)
    {
        _zoom = System.Math.Max(_zoom / 1.3, 0.3);
        ZoomLevelText.Text = $"Zoom: {_zoom:F1}x";
        _terrainDirty = true;
        RedrawMap();
    }

    // ==================== MISSION LIST ====================
    
    private void OnMissionChanged()
    {
        RefreshMissionList();
        RedrawMap();
        RedrawAltitudeProfile();
        UpdateStatistics();
    }

    private void RefreshMissionList()
    {
        MissionItemsList.Items.Clear();
        
        for (int i = 0; i < _missionPlanner.Items.Count; i++)
        {
            var item = _missionPlanner.Items[i];
            var listItem = CreateMissionListItem(i, item);
            MissionItemsList.Items.Add(listItem);
        }
        
        ItemCountText.Text = $"{_missionPlanner.Items.Count} items";
        
        if (_selectedIndex >= 0 && _selectedIndex < MissionItemsList.Items.Count)
            MissionItemsList.SelectedIndex = _selectedIndex;
    }

    private ListBoxItem CreateMissionListItem(int index, MissionItem item)
    {
        var border = new Border
        {
            Background = new SolidColorBrush(Color.FromRgb(26, 26, 37)),
            BorderBrush = new SolidColorBrush(Color.FromRgb(58, 58, 74)),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(4),
            Padding = new Thickness(10, 8, 10, 8)
        };

        var grid = new Grid();
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(30) });
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
        grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(60) });

        // Index circle
        var indexCircle = new Ellipse
        {
            Width = 22,
            Height = 22,
            Fill = GetTypeColor(item.Type),
            VerticalAlignment = VerticalAlignment.Center
        };
        var indexText = new TextBlock
        {
            Text = (index + 1).ToString(),
            Foreground = Brushes.Black,
            FontWeight = FontWeights.Bold,
            FontSize = 11,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center
        };
        var indexGrid = new Grid();
        indexGrid.Children.Add(indexCircle);
        indexGrid.Children.Add(indexText);
        Grid.SetColumn(indexGrid, 0);

        // Type and details
        var stack = new StackPanel { VerticalAlignment = VerticalAlignment.Center };
        stack.Children.Add(new TextBlock
        {
            Text = item.Type.ToString().ToUpper(),
            Foreground = GetTypeColor(item.Type),
            FontWeight = FontWeights.SemiBold,
            FontSize = 12
        });
        stack.Children.Add(new TextBlock
        {
            Text = GetItemDetails(item),
            Foreground = new SolidColorBrush(Color.FromRgb(102, 102, 119)),
            FontSize = 10,
            FontFamily = new FontFamily("Consolas")
        });
        Grid.SetColumn(stack, 1);

        // Altitude indicator
        var altStack = new StackPanel { VerticalAlignment = VerticalAlignment.Center, HorizontalAlignment = HorizontalAlignment.Right };
        if (item.Location != null)
        {
            altStack.Children.Add(new TextBlock
            {
                Text = $"{item.Location.Altitude:F0}m",
                Foreground = new SolidColorBrush(Color.FromRgb(0, 212, 255)),
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                HorizontalAlignment = HorizontalAlignment.Right
            });
        }
        Grid.SetColumn(altStack, 2);

        grid.Children.Add(indexGrid);
        grid.Children.Add(stack);
        grid.Children.Add(altStack);
        border.Child = grid;

        return new ListBoxItem { Content = border, Tag = index, Padding = new Thickness(0), Margin = new Thickness(0, 2, 0, 0) };
    }

    private SolidColorBrush GetTypeColor(MissionItemType type) => type switch
    {
        MissionItemType.Takeoff => new SolidColorBrush(Color.FromRgb(76, 175, 80)),
        MissionItemType.Waypoint => new SolidColorBrush(Color.FromRgb(0, 188, 212)),
        MissionItemType.Loiter => new SolidColorBrush(Color.FromRgb(255, 152, 0)),
        MissionItemType.Orbit => new SolidColorBrush(Color.FromRgb(156, 39, 176)),
        MissionItemType.Land => new SolidColorBrush(Color.FromRgb(255, 235, 59)),
        MissionItemType.ReturnToLaunch => new SolidColorBrush(Color.FromRgb(244, 67, 54)),
        MissionItemType.RegionOfInterest => new SolidColorBrush(Color.FromRgb(233, 30, 99)),
        _ => new SolidColorBrush(Colors.White)
    };

    private string GetItemDetails(MissionItem item) => item.Type switch
    {
        MissionItemType.Takeoff => $"Climb to {item.Location?.Altitude:F0}m",
        MissionItemType.Waypoint => item.Location != null ? $"{item.Location.Latitude:F5}, {item.Location.Longitude:F5}" : "",
        MissionItemType.Loiter => $"Hold {item.HoldTime:F0}s",
        MissionItemType.Orbit => $"Radius {item.Radius:F0}m, {item.Turns} turns",
        MissionItemType.Land => "Land at current position",
        MissionItemType.ReturnToLaunch => "Return to home",
        MissionItemType.RegionOfInterest => "Point camera here",
        _ => ""
    };

    private void MissionItemsList_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (MissionItemsList.SelectedItem is ListBoxItem listItem && listItem.Tag is int index)
        {
            _selectedIndex = index;
            UpdateWaypointPanel();
            RedrawMap();
        }
    }

    // ==================== WAYPOINT EDITING ====================
    
    private void UpdateWaypointPanel()
    {
        if (_selectedIndex < 0 || _selectedIndex >= _missionPlanner.Items.Count)
        {
            SelectedWaypointTitle.Text = "SELECT A WAYPOINT";
            SelectedWaypointSubtitle.Text = "Click on map or list to select";
            WaypointEditPanel.Visibility = Visibility.Collapsed;
            return;
        }

        var item = _missionPlanner.Items[_selectedIndex];
        SelectedWaypointTitle.Text = $"{item.Type.ToString().ToUpper()} #{_selectedIndex + 1}";
        SelectedWaypointSubtitle.Text = GetItemDetails(item);
        
        if (item.Location != null)
        {
            _updatingInputs = true;
            WpLatInput.Text = item.Location.Latitude.ToString("F7");
            WpLonInput.Text = item.Location.Longitude.ToString("F7");
            WpAltInput.Text = item.Location.Altitude.ToString("F1");
            WpSpeedInput.Text = item.Speed.ToString("F1");
            WpHoldInput.Text = item.HoldTime.ToString("F1");
            WpYawInput.Text = item.YawDegrees.ToString("F0");
            GimbalPitchInput.Text = item.GimbalPitch.ToString("F0");
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

        OnMissionChanged();
    }

    // ==================== MAP INTERACTION ====================
    
    private void MapCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        _isDragging = true;
        _lastMousePos = e.GetPosition(this);
        MapCanvas.CaptureMouse();
    }

    private void MapCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        bool wasDragging = _isDragging;
        _isDragging = false;
        MapCanvas.ReleaseMouseCapture();
        
        // Do a final full-quality redraw after drag ends
        if (wasDragging)
        {
            RedrawMap();
        }
    }

    private void MapCanvas_MouseMove(object sender, MouseEventArgs e)
    {
        var pos = e.GetPosition(MapCanvas);
        var (lat, lon) = CanvasToLatLon(pos.X, pos.Y);
        CursorPositionText.Text = $"Cursor: {lat:F6}, {lon:F6}";
        
        if (_isDragging && e.LeftButton == MouseButtonState.Pressed)
        {
            var currentPos = e.GetPosition(this);
            _mapOffset.X += currentPos.X - _lastMousePos.X;
            _mapOffset.Y += currentPos.Y - _lastMousePos.Y;
            _lastMousePos = currentPos;
            
            // Throttle redraws during drag - use cached terrain for speed
            RequestThrottledRedraw();
        }
    }
    
    private void RequestThrottledRedraw()
    {
        if (_redrawPending) return;
        
        _redrawPending = true;
        Dispatcher.BeginInvoke(DispatcherPriority.Render, () =>
        {
            _redrawPending = false;
            RedrawMapFast();
        });
    }
    
    /// <summary>
    /// Fast redraw for dragging - uses cached terrain bitmap
    /// </summary>
    private void RedrawMapFast()
    {
        if (MapCanvas == null || !_isLoaded) return;
        
        MapCanvas.Children.Clear();

        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 800;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 500;

        // Use cached terrain bitmap during drag (much faster than regenerating)
        if (ShowTerrainCheck.IsChecked == true && _terrainBitmap != null)
        {
            var terrainImage = new Image
            {
                Source = _terrainBitmap,
                Width = w,
                Height = h,
                Stretch = Stretch.Fill
            };
            MapCanvas.Children.Add(terrainImage);
        }

        // Draw grid (lightweight)
        if (ShowGridCheck.IsChecked == true)
            DrawGrid(w, h);

        // Draw geofence
        if (GeofenceCheck.IsChecked == true)
            DrawGeofence(w, h);

        // Draw flight path
        DrawFlightPath();

        // Draw orbit circles
        DrawOrbits();

        // Draw waypoint markers
        DrawWaypoints();

        // Draw home marker
        DrawHomeMarker();

        // Draw compass rose
        DrawCompass(w, h);
    }

    private void MapCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        _zoom = System.Math.Clamp(_zoom * (e.Delta > 0 ? 1.2 : 0.8), 0.3, 10.0);
        ZoomLevelText.Text = $"Zoom: {_zoom:F1}x";
        
        // Update scale bar
        double metersPerPixel = 100 / _zoom;
        ScaleText.Text = metersPerPixel > 1000 ? $"{metersPerPixel / 1000:F1}km" : $"{metersPerPixel:F0}m";
        ScaleBar.Width = 100;
        
        // Mark terrain as dirty so it regenerates at new zoom level
        _terrainDirty = true;
        
        RedrawMap();
    }

    private void MapCanvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
    {
        var pos = e.GetPosition(MapCanvas);
        var (lat, lon) = CanvasToLatLon(pos.X, pos.Y);

        double alt = ParseDouble(DefaultAltInput.Text, 50);
        double speed = ParseDouble(DefaultSpeedInput.Text, 5);

        _missionPlanner.AddWaypoint(lat, lon, alt, speed);
        OnMissionChanged();
    }

    // ==================== MAP RENDERING ====================
    
    private void RedrawMap()
    {
        if (MapCanvas == null || !_isLoaded) return;
        
        MapCanvas.Children.Clear();

        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 800;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 500;

        // Draw terrain if enabled - use bitmap for performance
        if (ShowTerrainCheck.IsChecked == true)
        {
            // Only regenerate terrain bitmap when zoom or size changes significantly
            if (_terrainDirty || _terrainBitmap == null || 
                System.Math.Abs(_lastTerrainZoom - _zoom) > 0.01 ||
                System.Math.Abs(_lastTerrainSize.Width - w) > 1 ||
                System.Math.Abs(_lastTerrainSize.Height - h) > 1)
            {
                RegenerateTerrainBitmap((int)w, (int)h);
                _lastTerrainZoom = _zoom;
                _lastTerrainSize = new Size(w, h);
                _terrainDirty = false;
            }
            
            if (_terrainBitmap != null)
            {
                var terrainImage = new Image
                {
                    Source = _terrainBitmap,
                    Width = w,
                    Height = h,
                    Stretch = Stretch.Fill
                };
                MapCanvas.Children.Add(terrainImage);
            }
        }

        // Draw grid
        if (ShowGridCheck.IsChecked == true)
            DrawGrid(w, h);

        // Draw geofence
        if (GeofenceCheck.IsChecked == true)
            DrawGeofence(w, h);

        // Draw flight path
        DrawFlightPath();

        // Draw orbit circles
        DrawOrbits();

        // Draw waypoint markers
        DrawWaypoints();

        // Draw home marker
        DrawHomeMarker();

        // Draw compass rose
        DrawCompass(w, h);
    }
    
    /// <summary>
    /// Generates terrain as a bitmap for efficient rendering (replaces DrawTerrain with rectangles)
    /// </summary>
    private void RegenerateTerrainBitmap(int width, int height)
    {
        if (width <= 0 || height <= 0) return;
        
        // Use lower resolution for terrain to improve performance
        int scale = 4; // Render at 1/4 resolution then scale up
        int bmpWidth = System.Math.Max(1, width / scale);
        int bmpHeight = System.Math.Max(1, height / scale);
        
        _terrainBitmap = new WriteableBitmap(bmpWidth, bmpHeight, 96, 96, PixelFormats.Bgr32, null);
        
        int stride = bmpWidth * 4;
        byte[] pixels = new byte[bmpHeight * stride];
        
        for (int y = 0; y < bmpHeight; y++)
        {
            for (int x = 0; x < bmpWidth; x++)
            {
                // Map pixel to lat/lon (accounting for lower resolution)
                var (lat, lon) = CanvasToLatLon(x * scale + scale / 2.0, y * scale + scale / 2.0);
                double elevation = GetTerrainHeight(lat, lon);
                
                // Color based on elevation
                byte green = (byte)System.Math.Clamp(50 + elevation * 2, 20, 100);
                byte brown = (byte)System.Math.Clamp(30 + elevation, 15, 60);
                
                int idx = (y * stride) + (x * 4);
                pixels[idx] = brown;     // B
                pixels[idx + 1] = green; // G
                pixels[idx + 2] = brown; // R
                pixels[idx + 3] = 255;   // A
            }
        }
        
        _terrainBitmap.WritePixels(new Int32Rect(0, 0, bmpWidth, bmpHeight), pixels, stride, 0);
    }

    private void DrawGrid(double w, double h)
    {
        double spacing = 60 * _zoom;
        var gridBrush = new SolidColorBrush(Color.FromArgb(40, 100, 100, 150));
        var majorBrush = new SolidColorBrush(Color.FromArgb(60, 100, 100, 150));

        int count = 0;
        for (double x = ((_mapOffset.X % spacing) + spacing) % spacing; x < w; x += spacing)
        {
            bool isMajor = count % 5 == 0;
            MapCanvas.Children.Add(new Line 
            { 
                X1 = x, Y1 = 0, X2 = x, Y2 = h, 
                Stroke = isMajor ? majorBrush : gridBrush, 
                StrokeThickness = isMajor ? 1 : 0.5 
            });
            count++;
        }
        
        count = 0;
        for (double y = ((_mapOffset.Y % spacing) + spacing) % spacing; y < h; y += spacing)
        {
            bool isMajor = count % 5 == 0;
            MapCanvas.Children.Add(new Line 
            { 
                X1 = 0, Y1 = y, X2 = w, Y2 = y, 
                Stroke = isMajor ? majorBrush : gridBrush, 
                StrokeThickness = isMajor ? 1 : 0.5 
            });
            count++;
        }
    }

    private void DrawGeofence(double w, double h)
    {
        double maxDist = ParseDouble(MaxDistLimitInput.Text, 5000);
        double radiusPixels = maxDist * _zoom * 0.01; // Approximate conversion
        
        var center = LatLonToCanvas(_missionPlanner.HomePosition?.Latitude ?? _centerLat, 
                                     _missionPlanner.HomePosition?.Longitude ?? _centerLon);
        
        var fence = new Ellipse
        {
            Width = radiusPixels * 2,
            Height = radiusPixels * 2,
            Stroke = new SolidColorBrush(Color.FromArgb(100, 255, 152, 0)),
            StrokeThickness = 2,
            StrokeDashArray = new DoubleCollection { 10, 5 },
            Fill = Brushes.Transparent
        };
        Canvas.SetLeft(fence, center.X - radiusPixels);
        Canvas.SetTop(fence, center.Y - radiusPixels);
        MapCanvas.Children.Add(fence);
    }

    private void DrawFlightPath()
    {
        var items = _missionPlanner.Items.Where(i => i.Location != null).ToList();
        if (items.Count < 2) return;

        // Draw path line
        var pathLine = new Polyline
        {
            Stroke = new SolidColorBrush(Color.FromRgb(0, 212, 255)),
            StrokeThickness = 3,
            StrokeLineJoin = PenLineJoin.Round
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

        // Draw direction arrows along path
        for (int i = 1; i < items.Count; i++)
        {
            if (items[i - 1].Location != null && items[i].Location != null)
            {
                var p1 = LatLonToCanvas(items[i - 1].Location!.Latitude, items[i - 1].Location!.Longitude);
                var p2 = LatLonToCanvas(items[i].Location!.Latitude, items[i].Location!.Longitude);
                
                // Arrow at midpoint
                var mid = new Point((p1.X + p2.X) / 2, (p1.Y + p2.Y) / 2);
                double angle = System.Math.Atan2(p2.Y - p1.Y, p2.X - p1.X) * 180 / System.Math.PI;
                
                var arrow = new Polygon
                {
                    Points = new PointCollection { new(-6, -4), new(6, 0), new(-6, 4) },
                    Fill = new SolidColorBrush(Color.FromRgb(0, 212, 255)),
                    RenderTransform = new TransformGroup
                    {
                        Children = { new RotateTransform(angle), new TranslateTransform(mid.X, mid.Y) }
                    }
                };
                MapCanvas.Children.Add(arrow);
            }
        }
    }

    private void DrawOrbits()
    {
        foreach (var item in _missionPlanner.Items.Where(i => i.Type == MissionItemType.Orbit && i.Location != null))
        {
            var center = LatLonToCanvas(item.Location!.Latitude, item.Location.Longitude);
            double radiusPixels = item.Radius * _zoom * 0.1;

            var orbit = new Ellipse
            {
                Width = radiusPixels * 2,
                Height = radiusPixels * 2,
                Stroke = new SolidColorBrush(Color.FromRgb(156, 39, 176)),
                StrokeThickness = 2,
                StrokeDashArray = new DoubleCollection { 5, 3 },
                Fill = new SolidColorBrush(Color.FromArgb(30, 156, 39, 176))
            };
            Canvas.SetLeft(orbit, center.X - radiusPixels);
            Canvas.SetTop(orbit, center.Y - radiusPixels);
            Panel.SetZIndex(orbit, 50);
            MapCanvas.Children.Add(orbit);
        }
    }

    private void DrawWaypoints()
    {
        for (int i = 0; i < _missionPlanner.Items.Count; i++)
        {
            var item = _missionPlanner.Items[i];
            if (item.Location == null) continue;

            var pos = LatLonToCanvas(item.Location.Latitude, item.Location.Longitude);
            bool isSelected = i == _selectedIndex;
            double size = isSelected ? 24 : 18;

            // Outer glow for selected
            if (isSelected)
            {
                var glow = new Ellipse
                {
                    Width = size + 10,
                    Height = size + 10,
                    Fill = new SolidColorBrush(Color.FromArgb(100, 255, 255, 0)),
                };
                Canvas.SetLeft(glow, pos.X - (size + 10) / 2);
                Canvas.SetTop(glow, pos.Y - (size + 10) / 2);
                Panel.SetZIndex(glow, 99);
                MapCanvas.Children.Add(glow);
            }

            // Marker
            var marker = new Ellipse
            {
                Width = size,
                Height = size,
                Fill = GetTypeColor(item.Type),
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
                FontSize = isSelected ? 12 : 10,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.Black
            };
            Canvas.SetLeft(label, pos.X - (i >= 9 ? 6 : 4));
            Canvas.SetTop(label, pos.Y - 6);
            Panel.SetZIndex(label, 101);
            MapCanvas.Children.Add(label);
        }
    }

    private void DrawHomeMarker()
    {
        if (_missionPlanner.HomePosition == null) return;
        
        var pos = LatLonToCanvas(_missionPlanner.HomePosition.Latitude, _missionPlanner.HomePosition.Longitude);
        
        // Home symbol (house shape)
        var home = new Polygon
        {
            Points = new PointCollection 
            { 
                new(0, -12), // Top
                new(10, -2), // Right top
                new(10, 8),  // Right bottom
                new(-10, 8), // Left bottom
                new(-10, -2) // Left top
            },
            Fill = Brushes.White,
            Stroke = new SolidColorBrush(Color.FromRgb(76, 175, 80)),
            StrokeThickness = 2,
            RenderTransform = new TranslateTransform(pos.X, pos.Y)
        };
        Panel.SetZIndex(home, 90);
        MapCanvas.Children.Add(home);

        // "H" label
        var label = new TextBlock
        {
            Text = "H",
            FontSize = 10,
            FontWeight = FontWeights.Bold,
            Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80))
        };
        Canvas.SetLeft(label, pos.X - 4);
        Canvas.SetTop(label, pos.Y - 5);
        Panel.SetZIndex(label, 91);
        MapCanvas.Children.Add(label);
    }

    private void DrawCompass(double w, double h)
    {
        double cx = w - 50;
        double cy = 50;
        double radius = 30;

        // Circle
        var circle = new Ellipse
        {
            Width = radius * 2,
            Height = radius * 2,
            Stroke = new SolidColorBrush(Color.FromArgb(150, 255, 255, 255)),
            StrokeThickness = 1,
            Fill = new SolidColorBrush(Color.FromArgb(100, 0, 0, 0))
        };
        Canvas.SetLeft(circle, cx - radius);
        Canvas.SetTop(circle, cy - radius);
        MapCanvas.Children.Add(circle);

        // North indicator
        var northLine = new Line
        {
            X1 = cx, Y1 = cy,
            X2 = cx, Y2 = cy - radius + 5,
            Stroke = new SolidColorBrush(Color.FromRgb(244, 67, 54)),
            StrokeThickness = 3
        };
        MapCanvas.Children.Add(northLine);

        // N label
        var nLabel = new TextBlock
        {
            Text = "N",
            FontSize = 10,
            FontWeight = FontWeights.Bold,
            Foreground = new SolidColorBrush(Color.FromRgb(244, 67, 54))
        };
        Canvas.SetLeft(nLabel, cx - 4);
        Canvas.SetTop(nLabel, cy - radius - 12);
        MapCanvas.Children.Add(nLabel);
    }

    // ==================== ALTITUDE PROFILE ====================
    
    private void RedrawAltitudeProfile()
    {
        if (AltitudeCanvas == null) return;
        
        AltitudeCanvas.Children.Clear();

        double w = AltitudeCanvas.ActualWidth > 0 ? AltitudeCanvas.ActualWidth : 600;
        double h = AltitudeCanvas.ActualHeight > 0 ? AltitudeCanvas.ActualHeight : 80;

        var items = _missionPlanner.Items.Where(i => i.Location != null).ToList();
        if (items.Count < 2) return;

        // Calculate path distances
        var distances = new List<double> { 0 };
        double totalDist = 0;
        for (int i = 1; i < items.Count; i++)
        {
            if (items[i - 1].Location != null && items[i].Location != null)
            {
                double dist = CalculateDistance(items[i - 1].Location!, items[i].Location!);
                totalDist += dist;
                distances.Add(totalDist);
            }
        }

        if (totalDist == 0) return;

        // Find max altitude
        double maxAlt = items.Max(i => i.Location?.Altitude ?? 0);
        maxAlt = System.Math.Max(maxAlt, 100);

        // Draw terrain profile
        var terrainPoints = new PointCollection();
        terrainPoints.Add(new Point(0, h));
        
        for (int i = 0; i < items.Count; i++)
        {
            double x = (distances[i] / totalDist) * w;
            double terrainAlt = GetTerrainHeight(items[i].Location!.Latitude, items[i].Location!.Longitude);
            double y = h - (terrainAlt / maxAlt) * (h - 10);
            terrainPoints.Add(new Point(x, y));
        }
        terrainPoints.Add(new Point(w, h));

        var terrainFill = new Polygon
        {
            Points = terrainPoints,
            Fill = new SolidColorBrush(Color.FromRgb(60, 80, 60)),
            Stroke = new SolidColorBrush(Color.FromRgb(100, 140, 100)),
            StrokeThickness = 1
        };
        AltitudeCanvas.Children.Add(terrainFill);

        // Draw altitude profile line
        var altLine = new Polyline
        {
            Stroke = new SolidColorBrush(Color.FromRgb(0, 212, 255)),
            StrokeThickness = 2
        };

        for (int i = 0; i < items.Count; i++)
        {
            double x = (distances[i] / totalDist) * w;
            double y = h - (items[i].Location!.Altitude / maxAlt) * (h - 10);
            altLine.Points.Add(new Point(x, y));
        }
        AltitudeCanvas.Children.Add(altLine);

        // Draw waypoint markers on profile
        for (int i = 0; i < items.Count; i++)
        {
            double x = (distances[i] / totalDist) * w;
            double y = h - (items[i].Location!.Altitude / maxAlt) * (h - 10);
            
            var marker = new Ellipse
            {
                Width = 8,
                Height = 8,
                Fill = GetTypeColor(items[i].Type),
                Stroke = Brushes.White,
                StrokeThickness = 1
            };
            Canvas.SetLeft(marker, x - 4);
            Canvas.SetTop(marker, y - 4);
            AltitudeCanvas.Children.Add(marker);
        }

        // Altitude scale
        var maxAltLabel = new TextBlock
        {
            Text = $"{maxAlt:F0}m",
            Foreground = new SolidColorBrush(Color.FromRgb(102, 102, 119)),
            FontSize = 9
        };
        Canvas.SetLeft(maxAltLabel, 5);
        Canvas.SetTop(maxAltLabel, 2);
        AltitudeCanvas.Children.Add(maxAltLabel);

        // Update terrain clearance
        double minClearance = double.MaxValue;
        foreach (var item in items)
        {
            double terrain = GetTerrainHeight(item.Location!.Latitude, item.Location!.Longitude);
            double clearance = item.Location.Altitude - terrain;
            minClearance = System.Math.Min(minClearance, clearance);
        }
        
        TerrainClearanceText.Text = $"{minClearance:F0}m min";
        TerrainClearanceText.Foreground = minClearance > 30 
            ? new SolidColorBrush(Color.FromRgb(76, 175, 80))
            : minClearance > 10 
                ? new SolidColorBrush(Color.FromRgb(255, 152, 0))
                : new SolidColorBrush(Color.FromRgb(244, 67, 54));
    }

    // ==================== COORDINATE HELPERS ====================
    
    private Point LatLonToCanvas(double lat, double lon)
    {
        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 800;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 500;
        double scale = 100000 * _zoom;
        double x = (lon - _centerLon) * scale * System.Math.Cos(_centerLat * System.Math.PI / 180);
        double y = -((lat - _centerLat) * scale);
        return new Point(w / 2 + x + _mapOffset.X, h / 2 + y + _mapOffset.Y);
    }

    private (double lat, double lon) CanvasToLatLon(double canvasX, double canvasY)
    {
        double w = MapCanvas.ActualWidth > 0 ? MapCanvas.ActualWidth : 800;
        double h = MapCanvas.ActualHeight > 0 ? MapCanvas.ActualHeight : 500;
        double scale = 100000 * _zoom;
        double x = canvasX - w / 2 - _mapOffset.X;
        double y = canvasY - h / 2 - _mapOffset.Y;
        double lon = x / (scale * System.Math.Cos(_centerLat * System.Math.PI / 180)) + _centerLon;
        double lat = -y / scale + _centerLat;
        return (lat, lon);
    }

    private double GetTerrainHeight(double lat, double lon)
    {
        // Map lat/lon to terrain grid
        int x = (int)(((lon - _centerLon) * 10000 + 50) % 100);
        int y = (int)(((lat - _centerLat) * 10000 + 50) % 100);
        x = System.Math.Clamp(x, 0, 99);
        y = System.Math.Clamp(y, 0, 99);
        return _terrainGrid[x, y];
    }

    private double CalculateDistance(GeoPoint p1, GeoPoint p2)
    {
        double dLat = (p2.Latitude - p1.Latitude) * System.Math.PI / 180;
        double dLon = (p2.Longitude - p1.Longitude) * System.Math.PI / 180;
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(p1.Latitude * System.Math.PI / 180) * System.Math.Cos(p2.Latitude * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        return 6371000 * 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
    }

    // ==================== STATISTICS ====================
    
    private void UpdateStatistics()
    {
        var stats = _missionPlanner.CalculateStatistics();
        
        TotalDistanceText.Text = stats.TotalDistanceMeters < 1000 
            ? $"{stats.TotalDistanceMeters:F0} m" 
            : $"{stats.TotalDistanceMeters / 1000:F2} km";
        
        EstTimeText.Text = stats.EstimatedTime.TotalMinutes >= 60 
            ? $"{stats.EstimatedTime.TotalHours:F1} hr"
            : $"{stats.EstimatedTime.TotalMinutes:F1} min";
        
        WaypointCountText.Text = stats.WaypointCount.ToString();
        MaxAltText.Text = $"{stats.MaxAltitude:F0} m";
        
        // Battery estimation
        double batteryUsed = EstimateBatteryUsage();
        BatteryEstText.Text = $"{100 - batteryUsed:F0}%";
        BatteryEstText.Foreground = (100 - batteryUsed) > 40 
            ? new SolidColorBrush(Color.FromRgb(76, 175, 80))
            : (100 - batteryUsed) > 20
                ? new SolidColorBrush(Color.FromRgb(255, 152, 0))
                : new SolidColorBrush(Color.FromRgb(244, 67, 54));
    }

    private double EstimateBatteryUsage()
    {
        var stats = _missionPlanner.CalculateStatistics();
        // Rough estimate: 2% per minute of flight + 0.5% per 100m distance
        return stats.EstimatedTimeSeconds / 60 * 2 + stats.TotalDistanceMeters / 200;
    }

    // ==================== EXPORT ====================
    
    private void SaveWaypoints_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Waypoints (*.waypoints)|*.waypoints|All files (*.*)|*.*",
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
            MessageBox.Show($"Mission saved: {waypoints.Count} waypoints", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
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

    private void SaveKML_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Google Earth KML (*.kml)|*.kml|All files (*.*)|*.*",
            DefaultExt = ".kml",
            FileName = "mission"
        };

        if (dialog.ShowDialog() == true)
        {
            var sb = new StringBuilder();
            sb.AppendLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
            sb.AppendLine("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
            sb.AppendLine("<Document>");
            sb.AppendLine("  <name>Drone Mission</name>");
            
            // Path
            sb.AppendLine("  <Placemark>");
            sb.AppendLine("    <name>Flight Path</name>");
            sb.AppendLine("    <Style><LineStyle><color>ffff8800</color><width>3</width></LineStyle></Style>");
            sb.AppendLine("    <LineString>");
            sb.AppendLine("      <altitudeMode>relativeToGround</altitudeMode>");
            sb.AppendLine("      <coordinates>");
            foreach (var item in _missionPlanner.Items.Where(i => i.Location != null))
            {
                sb.AppendLine($"        {item.Location!.Longitude:F7},{item.Location.Latitude:F7},{item.Location.Altitude:F1}");
            }
            sb.AppendLine("      </coordinates>");
            sb.AppendLine("    </LineString>");
            sb.AppendLine("  </Placemark>");
            
            // Waypoints
            int wpNum = 1;
            foreach (var item in _missionPlanner.Items.Where(i => i.Location != null))
            {
                sb.AppendLine($"  <Placemark>");
                sb.AppendLine($"    <name>WP{wpNum++}</name>");
                sb.AppendLine($"    <Point>");
                sb.AppendLine($"      <altitudeMode>relativeToGround</altitudeMode>");
                sb.AppendLine($"      <coordinates>{item.Location!.Longitude:F7},{item.Location.Latitude:F7},{item.Location.Altitude:F1}</coordinates>");
                sb.AppendLine($"    </Point>");
                sb.AppendLine($"  </Placemark>");
            }
            
            sb.AppendLine("</Document>");
            sb.AppendLine("</kml>");

            System.IO.File.WriteAllText(dialog.FileName, sb.ToString());
            MessageBox.Show("KML file saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private void SaveLitchi_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Litchi CSV (*.csv)|*.csv|All files (*.*)|*.*",
            DefaultExt = ".csv",
            FileName = "mission"
        };

        if (dialog.ShowDialog() == true)
        {
            var sb = new StringBuilder();
            sb.AppendLine("latitude,longitude,altitude(m),heading(deg),curvesize(m),rotationdir,gimbalmode,gimbalpitchangle,actiontype1,actionparam1");
            
            foreach (var item in _missionPlanner.Items.Where(i => i.Location != null))
            {
                sb.AppendLine($"{item.Location!.Latitude:F7},{item.Location!.Longitude:F7},{item.Location.Altitude:F1},{item.YawDegrees:F0},0.2,0,2,{item.GimbalPitch:F0},-1,0");
            }

            System.IO.File.WriteAllText(dialog.FileName, sb.ToString());
            MessageBox.Show("Litchi CSV saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    // ==================== HELPERS ====================
    
    private static double ParseDouble(string text, double defaultValue)
    {
        return double.TryParse(text, out double result) ? result : defaultValue;
    }
}
