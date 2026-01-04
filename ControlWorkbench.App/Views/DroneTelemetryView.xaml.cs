using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using ControlWorkbench.Drone.Devices;
using ControlWorkbench.Drone.Services;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Drone telemetry dashboard view with real-time data from flight controller or simulation.
/// </summary>
public partial class DroneTelemetryView : UserControl
{
    private readonly DispatcherTimer _uiUpdateTimer;
    private DateTime _startTime = DateTime.Now;
    
    // Drone connection service - the actual backend
    private readonly DroneConnectionService _droneService;
    private DroneTelemetry _latestTelemetry = new();
    private DroneStatus _latestStatus = new();
    
    // Motor outputs (0-100%)
    private double[] _motorOutputs = { 0, 0, 0, 0 };
    
    // Cell voltages
    private double[] _cellVoltages = { 4.2, 4.2, 4.2, 4.2 };
    
    // Home position
    private double _homeLat = 40.712776;
    private double _homeLon = -74.005974;
    
    // Messages
    private readonly List<(DateTime time, string message, MessageType type)> _messages = new();

    public DroneTelemetryView()
    {
        InitializeComponent();
        
        // Initialize the drone connection service
        _droneService = new DroneConnectionService();
        _droneService.TelemetryReceived += OnTelemetryReceived;
        _droneService.StatusChanged += OnStatusChanged;
        _droneService.LogReceived += OnLogReceived;
        _droneService.ConnectionStateChanged += OnConnectionStateChanged;
        
        // UI update timer (separate from telemetry rate)
        _uiUpdateTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(100)
        };
        _uiUpdateTimer.Tick += UiUpdateTimer_Tick;
        
        Loaded += OnLoaded;
        Unloaded += OnUnloaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        _startTime = DateTime.Now;
        
        // Start in simulation mode by default for demo purposes
        _droneService.StartSimulation(_homeLat, _homeLon, 0);
        
        AddMessage("ControlWorkbench Drone Telemetry", MessageType.Info);
        AddMessage("Connected to simulation", MessageType.Good);
        
        _uiUpdateTimer.Start();
    }

    private void OnUnloaded(object sender, RoutedEventArgs e)
    {
        _uiUpdateTimer.Stop();
        _droneService.DisconnectAsync().Wait();
    }
    
    // Event handlers from DroneConnectionService
    private void OnTelemetryReceived(DroneTelemetry telemetry)
    {
        // Store latest telemetry (thread-safe, UI updates on timer)
        _latestTelemetry = telemetry;
        
        // Update motor outputs from telemetry
        if (telemetry.MotorOutputs != null)
        {
            for (int i = 0; i < System.Math.Min(telemetry.MotorOutputs.Length, _motorOutputs.Length); i++)
            {
                _motorOutputs[i] = telemetry.MotorOutputs[i];
            }
        }
        
        // Calculate cell voltages from total voltage (assume 4S)
        if (telemetry.BatteryVoltage > 0)
        {
            double cellAvg = telemetry.BatteryVoltage / 4;
            for (int i = 0; i < 4; i++)
            {
                _cellVoltages[i] = cellAvg + (Random.Shared.NextDouble() - 0.5) * 0.02;
            }
        }
    }
    
    private void OnStatusChanged(DroneStatus status)
    {
        _latestStatus = status;
    }
    
    private void OnLogReceived(string message)
    {
        Dispatcher.BeginInvoke(() =>
        {
            var type = message.Contains("error", StringComparison.OrdinalIgnoreCase) ? MessageType.Error :
                       message.Contains("warning", StringComparison.OrdinalIgnoreCase) ? MessageType.Warning :
                       message.Contains("ok", StringComparison.OrdinalIgnoreCase) || 
                       message.Contains("good", StringComparison.OrdinalIgnoreCase) ? MessageType.Good :
                       MessageType.Info;
            AddMessage(message, type);
        });
    }
    
    private void OnConnectionStateChanged(ConnectionMode mode)
    {
        Dispatcher.BeginInvoke(() =>
        {
            string modeText = mode switch
            {
                ConnectionMode.Hardware => "Connected to Flight Controller",
                ConnectionMode.Simulation => "Simulation Mode",
                _ => "Disconnected"
            };
            AddMessage(modeText, mode == ConnectionMode.Disconnected ? MessageType.Warning : MessageType.Good);
        });
    }

    private void UiUpdateTimer_Tick(object? sender, EventArgs e)
    {
        UpdateDisplay();
    }

    private void UpdateDisplay()
    {
        var t = _latestTelemetry;
        
        // Flight time
        var elapsed = DateTime.Now - _startTime;
        FlightTimeText.Text = elapsed.ToString(@"mm\:ss");

        // Attitude
        RollValueText.Text = $"{t.Roll:F1} deg";
        PitchValueText.Text = $"{t.Pitch:F1} deg";
        HorizonRotation.Angle = -t.Roll;
        RollMarkerRotation.Angle = -t.Roll;

        // Heading
        double heading = t.Heading;
        if (heading < 0) heading += 360;
        HeadingText.Text = $"{heading:F0}";
        HeadingRotation.Angle = heading;
        HeadingCardinalText.Text = GetCardinalDirection(heading);

        // Altitude and speed
        AltitudeText.Text = $"{t.AltitudeRelative:F1}";
        ClimbRateText.Text = $"Climb: {t.ClimbRate:F1} m/s";
        ClimbRateText.Foreground = t.ClimbRate >= 0 
            ? new SolidColorBrush(Color.FromRgb(76, 175, 80)) // Green
            : new SolidColorBrush(Color.FromRgb(255, 152, 0)); // Orange
        
        GroundSpeedText.Text = $"{t.GroundSpeed:F1}";
        SpeedKmhText.Text = $"= {t.GroundSpeed * 3.6:F1} km/h";

        // GPS
        LatitudeText.Text = $"{t.Latitude:F6}";
        LongitudeText.Text = $"{t.Longitude:F6}";
        AltMslText.Text = $"{t.AltitudeMSL:F1} m";
        SatellitesText.Text = t.GpsSatellites.ToString();
        SatCountText.Text = t.GpsSatellites.ToString();
        DopText.Text = $"{t.GpsHdop:F1} / {t.GpsVdop:F1}";
        HdopText.Text = $"{t.GpsHdop:F1}";

        // Distance to home
        double distToHome = CalculateDistance(t.Latitude, t.Longitude, _homeLat, _homeLon);
        double bearingToHome = CalculateBearing(t.Latitude, t.Longitude, _homeLat, _homeLon);
        DistanceToHomeText.Text = distToHome < 1000 ? $"{distToHome:F0}m" : $"{distToHome/1000:F1}km";
        BearingToHomeText.Text = $" @ {bearingToHome:F0}";

        // Battery
        BatteryVoltageText.Text = $"{t.BatteryVoltage:F1}V";
        double battPercent = t.BatteryRemaining > 0 ? t.BatteryRemaining : 
            (t.BatteryVoltage - 13.2) / (16.8 - 13.2) * 100;
        battPercent = System.Math.Clamp(battPercent, 0, 100);
        BatteryInfoText.Text = $"{battPercent:F0}% | {t.BatteryCurrent:F1}A";
        
        // Color based on battery level
        if (battPercent > 50)
            BatteryVoltageText.Foreground = new SolidColorBrush(Color.FromRgb(76, 175, 80));
        else if (battPercent > 25)
            BatteryVoltageText.Foreground = new SolidColorBrush(Color.FromRgb(255, 152, 0));
        else
            BatteryVoltageText.Foreground = new SolidColorBrush(Color.FromRgb(244, 67, 54));

        CurrentText.Text = $"{t.BatteryCurrent:F1} A";
        PowerText.Text = $"{t.BatteryCurrent * t.BatteryVoltage:F0} W";
        ConsumedText.Text = $"{t.BatteryConsumed:F0} mAh";
        
        double remainingMah = 5000 - t.BatteryConsumed;
        double remainingMin = t.BatteryCurrent > 0 ? remainingMah / (t.BatteryCurrent * 1000 / 60) : 99;
        RemainingTimeText.Text = $"~{remainingMin:F0} min";

        // Cell voltages
        Cell1Text.Text = $"{_cellVoltages[0]:F2}V";
        Cell2Text.Text = $"{_cellVoltages[1]:F2}V";
        Cell3Text.Text = $"{_cellVoltages[2]:F2}V";
        Cell4Text.Text = $"{_cellVoltages[3]:F2}V";

        // Motor outputs
        Motor1Text.Text = $"{_motorOutputs[0]:F0}%";
        Motor1Bar.Value = _motorOutputs[0];
        Motor2Text.Text = $"{_motorOutputs[1]:F0}%";
        Motor2Bar.Value = _motorOutputs[1];
        Motor3Text.Text = $"{_motorOutputs[2]:F0}%";
        Motor3Bar.Value = _motorOutputs[2];
        Motor4Text.Text = $"{_motorOutputs[3]:F0}%";
        Motor4Bar.Value = _motorOutputs[3];

        // Armed/Mode status
        if (t.Armed)
        {
            RecordingIndicator.Visibility = (DateTime.Now.Second % 2 == 0) 
                ? Visibility.Visible 
                : Visibility.Hidden;
        }
        else
        {
            RecordingIndicator.Visibility = Visibility.Hidden;
        }
    }

    private void AddMessage(string message, MessageType type)
    {
        _messages.Insert(0, (DateTime.Now, message, type));
        if (_messages.Count > 20)
            _messages.RemoveAt(_messages.Count - 1);

        UpdateMessagesPanel();
    }

    private void UpdateMessagesPanel()
    {
        MessagesPanel.Children.Clear();

        foreach (var (time, message, type) in _messages.Take(10))
        {
            var border = new Border
            {
                Padding = new Thickness(8),
                CornerRadius = new CornerRadius(3),
                Margin = new Thickness(0, 2, 0, 0),
                Background = type switch
                {
                    MessageType.Good => new SolidColorBrush(Color.FromRgb(30, 61, 30)),
                    MessageType.Warning => new SolidColorBrush(Color.FromRgb(61, 61, 30)),
                    MessageType.Error => new SolidColorBrush(Color.FromRgb(61, 30, 30)),
                    _ => new SolidColorBrush(Color.FromRgb(30, 45, 61))
                }
            };

            var stack = new StackPanel();
            stack.Children.Add(new TextBlock
            {
                Text = time.ToString("HH:mm:ss"),
                Foreground = new SolidColorBrush(Color.FromRgb(136, 136, 136)),
                FontSize = 10
            });
            stack.Children.Add(new TextBlock
            {
                Text = message,
                Foreground = type switch
                {
                    MessageType.Good => new SolidColorBrush(Color.FromRgb(76, 175, 80)),
                    MessageType.Warning => new SolidColorBrush(Color.FromRgb(255, 152, 0)),
                    MessageType.Error => new SolidColorBrush(Color.FromRgb(244, 67, 54)),
                    _ => new SolidColorBrush(Color.FromRgb(33, 150, 243))
                },
                TextWrapping = TextWrapping.Wrap
            });

            border.Child = stack;
            MessagesPanel.Children.Add(border);
        }
    }

    private static string GetCardinalDirection(double heading)
    {
        string[] directions = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
        int index = (int)System.Math.Round(heading / 45) % 8;
        return directions[index];
    }

    private static double CalculateDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371000; // Earth radius in meters
        double dLat = (lat2 - lat1) * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1 * System.Math.PI / 180) * System.Math.Cos(lat2 * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        double c = 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
        return R * c;
    }

    private static double CalculateBearing(double lat1, double lon1, double lat2, double lon2)
    {
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double lat1Rad = lat1 * System.Math.PI / 180;
        double lat2Rad = lat2 * System.Math.PI / 180;
        
        double y = System.Math.Sin(dLon) * System.Math.Cos(lat2Rad);
        double x = System.Math.Cos(lat1Rad) * System.Math.Sin(lat2Rad) -
                   System.Math.Sin(lat1Rad) * System.Math.Cos(lat2Rad) * System.Math.Cos(dLon);
        
        double bearing = System.Math.Atan2(y, x) * 180 / System.Math.PI;
        return (bearing + 360) % 360;
    }
    
    // Public methods for external control
    
    /// <summary>
    /// Connect to a real flight controller.
    /// </summary>
    public async Task ConnectToFlightControllerAsync(string connectionString)
    {
        await _droneService.ConnectAsync(connectionString);
        _startTime = DateTime.Now;
    }
    
    /// <summary>
    /// Start simulation mode.
    /// </summary>
    public void StartSimulation()
    {
        _droneService.StartSimulation(_homeLat, _homeLon, 0);
        _startTime = DateTime.Now;
    }
    
    /// <summary>
    /// Arm the drone.
    /// </summary>
    public async Task ArmAsync()
    {
        await _droneService.ArmAsync();
    }
    
    /// <summary>
    /// Disarm the drone.
    /// </summary>
    public async Task DisarmAsync()
    {
        await _droneService.DisarmAsync();
    }
    
    /// <summary>
    /// Takeoff to specified altitude.
    /// </summary>
    public async Task TakeoffAsync(double altitude)
    {
        await _droneService.TakeoffAsync(altitude);
    }
    
    /// <summary>
    /// Land the drone.
    /// </summary>
    public async Task LandAsync()
    {
        await _droneService.LandAsync();
    }
    
    /// <summary>
    /// Return to launch.
    /// </summary>
    public async Task ReturnToLaunchAsync()
    {
        await _droneService.ReturnToLaunchAsync();
    }

    private enum MessageType
    {
        Info,
        Good,
        Warning,
        Error
    }
}
