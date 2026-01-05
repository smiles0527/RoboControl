using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using ControlWorkbench.Drone.Calculations;
using ControlWorkbench.Drone.Devices;
using ControlWorkbench.Drone.Tuning;

namespace ControlWorkbench.App.Views;

/// <summary>
/// Drone configuration and PID tuning view with functional calculations.
/// </summary>
public partial class DroneConfigView : UserControl
{
    private bool _isLoaded;

    public DroneConfigView()
    {
        InitializeComponent();
        Loaded += OnLoaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        if (_isLoaded) return;
        _isLoaded = true;
        
        WireUpEvents();
        UpdateCalculations();
    }

    private void WireUpEvents()
    {
        // Spec inputs
        WeightInput.TextChanged += OnSpecsChanged;
        ArmLengthInput.TextChanged += OnSpecsChanged;
        MotorKvInput.TextChanged += OnSpecsChanged;
        CapacityInput.TextChanged += OnSpecsChanged;
        
        FrameTypeCombo.SelectionChanged += OnSpecsChanged;
        PropSizeCombo.SelectionChanged += OnSpecsChanged;
        PropPitchCombo.SelectionChanged += OnSpecsChanged;
        CellCountCombo.SelectionChanged += OnSpecsChanged;

        // Rate inputs for max rate calculation
        RollRcRateInput.TextChanged += OnRatesChanged;
        RollSuperInput.TextChanged += OnRatesChanged;
        PitchRcRateInput.TextChanged += OnRatesChanged;
        PitchSuperInput.TextChanged += OnRatesChanged;
        YawRcRateInput.TextChanged += OnRatesChanged;
        YawSuperInput.TextChanged += OnRatesChanged;

        // Buttons
        CalculatePidsButton.Click += OnCalculatePids;
        CopyBetaflightButton.Click += OnCopyBetaflightCli;
        ExportBetaflightButton.Click += OnCopyBetaflightCli;
        ExportArduCopterButton.Click += OnExportArduCopter;
        ExportPx4Button.Click += OnExportPx4;
        SaveProfileButton.Click += OnSaveProfile;
    }

    private void OnSpecsChanged(object sender, RoutedEventArgs e) => UpdateCalculations();

    private void OnRatesChanged(object sender, RoutedEventArgs e)
    {
        if (!_isLoaded) return;
        UpdateMaxRateDisplay();
    }

    private void UpdateCalculations()
    {
        if (!_isLoaded) return;

        try
        {
            var specs = GetDroneSpecs();
            
            // Calculate performance estimates
            double thrustRatio = specs.ThrustToWeightRatio;
            double hoverThrottle = DroneCalculations.CalculateHoverThrottle(specs.WeightGrams, specs.MaxThrustGrams);
            double flightTime = DroneCalculations.EstimateFlightTimeMinutes(
                specs.BatteryCapacityMah, 
                specs.MaxThrustGrams * 0.3 * 9.81 / 1000 * GetBatteryVoltage(), // ~30% throttle avg
                GetBatteryVoltage(), 
                20);
            
            double maxSpeed = EstimateMaxSpeed(specs);
            double maxCurrent = EstimateMaxCurrent(specs);

            // Update display
            ThrustRatioText.Text = $"{thrustRatio:F1}:1";
            ThrustRatioText.Foreground = thrustRatio > 5 
                ? new SolidColorBrush(Color.FromRgb(76, 175, 80))
                : thrustRatio > 3 
                    ? new SolidColorBrush(Color.FromRgb(255, 152, 0))
                    : new SolidColorBrush(Color.FromRgb(244, 67, 54));

            HoverThrottleText.Text = $"{hoverThrottle:F0}%";
            FlightTimeText.Text = $"{flightTime:F0}-{flightTime * 1.3:F0} min";
            MaxSpeedText.Text = $"~{maxSpeed:F0} km/h";
            MaxCurrentText.Text = $"~{maxCurrent:F0}A";

            UpdateRecommendations(specs, thrustRatio, flightTime);
        }
        catch
        {
            // Ignore parsing errors during input
        }
    }

    private DroneSpecs GetDroneSpecs()
    {
        double weight = ParseDouble(WeightInput.Text, 650);
        double armLength = ParseDouble(ArmLengthInput.Text, 110);
        int motorKv = (int)ParseDouble(MotorKvInput.Text, 2400);
        int propSize = GetPropSize();
        double propPitch = GetPropPitch();
        int cellCount = GetCellCount();
        double batteryCapacity = ParseDouble(CapacityInput.Text, 1500);
        int motorCount = GetMotorCount();

        // Estimate thrust per motor based on prop size and KV
        double thrustPerMotor = EstimateThrustPerMotor(propSize, propPitch, motorKv, cellCount);
        double totalThrust = thrustPerMotor * motorCount;

        return new DroneSpecs
        {
            WeightGrams = weight,
            ArmLengthMm = armLength,
            MotorKv = motorKv,
            PropDiameterInches = propSize,
            PropPitchInches = propPitch,
            CellCount = cellCount,
            BatteryCapacityMah = batteryCapacity,
            MotorCount = motorCount,
            MaxThrustGrams = totalThrust
        };
    }

    private double EstimateThrustPerMotor(int propSize, double propPitch, int kv, int cells)
    {
        double voltage = cells * 3.7;
        // Simplified thrust estimation based on common setups
        double baseThrustGrams = propSize switch
        {
            3 => 180,
            4 => 350,
            5 => 600,
            6 => 900,
            7 => 1300,
            _ => 500
        };

        // KV affects thrust somewhat
        double kvFactor = System.Math.Sqrt(kv / 2000.0);
        // Voltage affects thrust
        double voltageFactor = voltage / 14.8;

        return baseThrustGrams * kvFactor * voltageFactor;
    }

    private double EstimateMaxSpeed(DroneSpecs specs)
    {
        // Higher T/W = higher max speed, rough estimate
        double baseSpeed = 80; // km/h
        return baseSpeed + (specs.ThrustToWeightRatio - 3) * 15;
    }

    private double EstimateMaxCurrent(DroneSpecs specs)
    {
        // Estimate based on motor count and typical motor current
        int motorCount = specs.MotorCount > 0 ? specs.MotorCount : 4;
        double perMotorCurrent = specs.MotorKv > 2000 ? 35 : 25;
        return motorCount * perMotorCurrent;
    }

    private void UpdateRecommendations(DroneSpecs specs, double thrustRatio, double flightTime)
    {
        // Clear and rebuild recommendations
        RecommendationsPanel.Children.Clear();
        RecommendationsPanel.Children.Add(new TextBlock
        {
            Text = "Recommendations",
            Foreground = Brushes.White,
            FontSize = 16,
            FontWeight = FontWeights.Bold,
            Margin = new Thickness(0, 0, 0, 10)
        });

        if (thrustRatio >= 5)
        {
            AddRecommendation("Good thrust to weight ratio for freestyle/racing", RecommendationType.Good);
        }
        else if (thrustRatio >= 3)
        {
            AddRecommendation("Moderate T/W - suitable for general flying", RecommendationType.Info);
        }
        else
        {
            AddRecommendation("Low thrust ratio - consider lighter build or stronger motors", RecommendationType.Warning);
        }

        if (specs.MotorKv > 2200 && specs.CellCount >= 5)
        {
            AddRecommendation("High KV on 5S+ may cause motor heating - monitor temps", RecommendationType.Warning);
        }

        if (flightTime < 4)
        {
            AddRecommendation("Short flight time - consider larger battery", RecommendationType.Warning);
        }
        else if (flightTime > 8)
        {
            AddRecommendation("Good flight time for the setup", RecommendationType.Good);
        }

        AddRecommendation("PIDs calculated based on specs - tune in flight", RecommendationType.Info);
    }

    private void AddRecommendation(string text, RecommendationType type)
    {
        var border = new Border
        {
            Padding = new Thickness(10),
            CornerRadius = new CornerRadius(3),
            Margin = new Thickness(0, 0, 0, 5),
            Background = type switch
            {
                RecommendationType.Good => new SolidColorBrush(Color.FromRgb(30, 61, 30)),
                RecommendationType.Warning => new SolidColorBrush(Color.FromRgb(61, 61, 30)),
                _ => new SolidColorBrush(Color.FromRgb(30, 45, 61))
            }
        };

        border.Child = new TextBlock
        {
            Text = text,
            Foreground = type switch
            {
                RecommendationType.Good => new SolidColorBrush(Color.FromRgb(76, 175, 80)),
                RecommendationType.Warning => new SolidColorBrush(Color.FromRgb(255, 152, 0)),
                _ => new SolidColorBrush(Color.FromRgb(33, 150, 243))
            },
            TextWrapping = TextWrapping.Wrap
        };

        RecommendationsPanel.Children.Add(border);
    }

    private void UpdateMaxRateDisplay()
    {
        try
        {
            double rcRate = ParseDouble(RollRcRateInput.Text, 1.0);
            double superRate = ParseDouble(RollSuperInput.Text, 0.7);
            
            // Betaflight rate calculation at max stick: rate = rcRate * 200 * (1 + superRate)
            double maxRate = rcRate * 200 * (1 + superRate);
            MaxRateText.Text = $"Max rate: {maxRate:F0} deg/s";
        }
        catch
        {
            // Ignore
        }
    }

    private void OnCalculatePids(object sender, RoutedEventArgs e)
    {
        try
        {
            var specs = GetDroneSpecs();
            var gains = DronePidTuner.EstimateInitialGains(specs);

            // Update PID fields (converting to Betaflight scale)
            RollPInput.Text = $"{gains.RollRate.P * 333:F0}";
            RollIInput.Text = $"{gains.RollRate.I * 600:F0}";
            RollDInput.Text = $"{gains.RollRate.D * 10000:F0}";
            RollFFInput.Text = $"{gains.RollRate.FF * 1000:F0}";

            PitchPInput.Text = $"{gains.PitchRate.P * 333:F0}";
            PitchIInput.Text = $"{gains.PitchRate.I * 600:F0}";
            PitchDInput.Text = $"{gains.PitchRate.D * 10000:F0}";
            PitchFFInput.Text = $"{gains.PitchRate.FF * 1000:F0}";

            YawPInput.Text = $"{gains.YawRate.P * 333:F0}";
            YawIInput.Text = $"{gains.YawRate.I * 600:F0}";
            YawDInput.Text = $"{gains.YawRate.D * 10000:F0}";
            YawFFInput.Text = $"{gains.YawRate.FF * 1000:F0}";

            GyroLpfInput.Text = $"{gains.GyroLpfHz:F0}";
            DtermLpfInput.Text = $"{gains.DtermLpfHz:F0}";

            MessageBox.Show("PIDs calculated based on drone specifications.\n\nThese are starting values - always tune in flight!", 
                "PIDs Updated", MessageBoxButton.OK, MessageBoxImage.Information);
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Error calculating PIDs: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void OnCopyBetaflightCli(object sender, RoutedEventArgs e)
    {
        var cli = GenerateBetaflightCli();
        Clipboard.SetText(cli);
        MessageBox.Show("Betaflight CLI commands copied to clipboard!", "Copied", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void OnExportArduCopter(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Parameter files (*.param)|*.param|All files (*.*)|*.*",
            DefaultExt = ".param",
            FileName = $"{DroneNameInput.Text.Replace(" ", "_")}_params"
        };

        if (dialog.ShowDialog() == true)
        {
            var paramFile = GenerateArduCopterParams();
            System.IO.File.WriteAllText(dialog.FileName, paramFile);
            MessageBox.Show("ArduCopter parameters saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private void OnExportPx4(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Parameter files (*.params)|*.params|All files (*.*)|*.*",
            DefaultExt = ".params",
            FileName = $"{DroneNameInput.Text.Replace(" ", "_")}_px4_params"
        };

        if (dialog.ShowDialog() == true)
        {
            var paramFile = GeneratePx4Params();
            System.IO.File.WriteAllText(dialog.FileName, paramFile);
            MessageBox.Show("PX4 parameters saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private void OnSaveProfile(object sender, RoutedEventArgs e)
    {
        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Filter = "Drone Profile (*.droneprofile)|*.droneprofile|All files (*.*)|*.*",
            DefaultExt = ".droneprofile",
            FileName = DroneNameInput.Text.Replace(" ", "_")
        };

        if (dialog.ShowDialog() == true)
        {
            var profile = GenerateProfileJson();
            System.IO.File.WriteAllText(dialog.FileName, profile);
            MessageBox.Show("Drone profile saved!", "Saved", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }

    private string GenerateBetaflightCli()
    {
        var sb = new StringBuilder();
        sb.AppendLine("# Betaflight CLI dump");
        sb.AppendLine($"# Profile: {DroneNameInput.Text}");
        sb.AppendLine($"# Generated by ControlWorkbench - {DateTime.Now:yyyy-MM-dd HH:mm}");
        sb.AppendLine();
        sb.AppendLine("# PID Profile");
        sb.AppendLine($"set p_pitch = {PitchPInput.Text}");
        sb.AppendLine($"set i_pitch = {PitchIInput.Text}");
        sb.AppendLine($"set d_pitch = {PitchDInput.Text}");
        sb.AppendLine($"set f_pitch = {PitchFFInput.Text}");
        sb.AppendLine($"set p_roll = {RollPInput.Text}");
        sb.AppendLine($"set i_roll = {RollIInput.Text}");
        sb.AppendLine($"set d_roll = {RollDInput.Text}");
        sb.AppendLine($"set f_roll = {RollFFInput.Text}");
        sb.AppendLine($"set p_yaw = {YawPInput.Text}");
        sb.AppendLine($"set i_yaw = {YawIInput.Text}");
        sb.AppendLine($"set d_yaw = {YawDInput.Text}");
        sb.AppendLine($"set f_yaw = {YawFFInput.Text}");
        sb.AppendLine();
        sb.AppendLine("# Filters");
        sb.AppendLine($"set gyro_lpf1_static_hz = {GyroLpfInput.Text}");
        sb.AppendLine($"set dterm_lpf1_static_hz = {DtermLpfInput.Text}");
        if (DynamicNotchCheck.IsChecked == true)
        {
            sb.AppendLine("set dyn_notch_count = 3");
            sb.AppendLine("set dyn_notch_q = 300");
        }
        if (RpmFilterCheck.IsChecked == true)
        {
            sb.AppendLine("set rpm_filter_harmonics = 3");
        }
        sb.AppendLine();
        sb.AppendLine("# Rates");
        sb.AppendLine($"set roll_rc_rate = {ParseDouble(RollRcRateInput.Text, 1.0) * 100:F0}");
        sb.AppendLine($"set roll_srate = {ParseDouble(RollSuperInput.Text, 0.7) * 100:F0}");
        sb.AppendLine($"set roll_expo = {ParseDouble(RollExpoInput.Text, 0.25) * 100:F0}");
        sb.AppendLine($"set pitch_rc_rate = {ParseDouble(PitchRcRateInput.Text, 1.0) * 100:F0}");
        sb.AppendLine($"set pitch_srate = {ParseDouble(PitchSuperInput.Text, 0.7) * 100:F0}");
        sb.AppendLine($"set pitch_expo = {ParseDouble(PitchExpoInput.Text, 0.25) * 100:F0}");
        sb.AppendLine($"set yaw_rc_rate = {ParseDouble(YawRcRateInput.Text, 1.0) * 100:F0}");
        sb.AppendLine($"set yaw_srate = {ParseDouble(YawSuperInput.Text, 0.7) * 100:F0}");
        sb.AppendLine($"set yaw_expo = {ParseDouble(YawExpoInput.Text, 0.15) * 100:F0}");
        sb.AppendLine();
        sb.AppendLine("save");
        return sb.ToString();
    }

    private string GenerateArduCopterParams()
    {
        var specs = GetDroneSpecs();
        var gains = DronePidTuner.EstimateInitialGains(specs);
        return DronePidTuner.ToArduCopterParams(gains);
    }

    private string GeneratePx4Params()
    {
        var sb = new StringBuilder();
        sb.AppendLine("# PX4 Parameters");
        sb.AppendLine("# Profile: " + DroneNameInput.Text);
        sb.AppendLine("# Generated by ControlWorkbench - " + DateTime.Now.ToString("yyyy-MM-dd HH:mm"));
        sb.AppendLine();
        sb.AppendLine("# Rate Controller");
        sb.AppendLine("MC_ROLLRATE_P\t0.15");
        sb.AppendLine("MC_ROLLRATE_I\t0.2");
        sb.AppendLine("MC_ROLLRATE_D\t0.003");
        sb.AppendLine("MC_PITCHRATE_P\t0.15");
        sb.AppendLine("MC_PITCHRATE_I\t0.2");
        sb.AppendLine("MC_PITCHRATE_D\t0.003");
        sb.AppendLine("MC_YAWRATE_P\t0.2");
        sb.AppendLine("MC_YAWRATE_I\t0.1");
        sb.AppendLine("MC_YAWRATE_D\t0.0");
        sb.AppendLine();
        sb.AppendLine("# Battery");
        sb.AppendLine("BAT_N_CELLS\t" + GetCellCount());
        sb.AppendLine("BAT_CAPACITY\t" + CapacityInput.Text);
        sb.AppendLine("BAT_V_EMPTY\t" + ParseDouble(LowVoltageInput.Text, 3.5).ToString("F2"));
        return sb.ToString();
    }

    private string GenerateProfileJson()
    {
        var sb = new StringBuilder();
        sb.AppendLine("{");
        sb.AppendLine("  \"name\": \"" + DroneNameInput.Text + "\",");
        sb.AppendLine("  \"frameType\": \"" + GetSelectedText(FrameTypeCombo) + "\",");
        sb.AppendLine("  \"flightController\": \"" + GetSelectedText(FlightControllerCombo) + "\",");
        sb.AppendLine("  \"weight\": " + WeightInput.Text + ",");
        sb.AppendLine("  \"armLength\": " + ArmLengthInput.Text + ",");
        sb.AppendLine("  \"propSize\": " + GetPropSize() + ",");
        sb.AppendLine("  \"motorKv\": " + MotorKvInput.Text + ",");
        sb.AppendLine("  \"cellCount\": " + GetCellCount() + ",");
        sb.AppendLine("  \"batteryCapacity\": " + CapacityInput.Text + ",");
        sb.AppendLine("  \"pids\": {");
        sb.AppendLine("    \"rollP\": " + RollPInput.Text + ", \"rollI\": " + RollIInput.Text + ", \"rollD\": " + RollDInput.Text + ",");
        sb.AppendLine("    \"pitchP\": " + PitchPInput.Text + ", \"pitchI\": " + PitchIInput.Text + ", \"pitchD\": " + PitchDInput.Text + ",");
        sb.AppendLine("    \"yawP\": " + YawPInput.Text + ", \"yawI\": " + YawIInput.Text + ", \"yawD\": " + YawDInput.Text);
        sb.AppendLine("  },");
        sb.AppendLine("  \"gyroLpf\": " + GyroLpfInput.Text + ",");
        sb.AppendLine("  \"dtermLpf\": " + DtermLpfInput.Text);
        sb.AppendLine("}");
        return sb.ToString();
    }

    // Helper methods
    private static double ParseDouble(string text, double defaultValue)
    {
        return double.TryParse(text, out double result) ? result : defaultValue;
    }

    private int GetPropSize()
    {
        return PropSizeCombo.SelectedIndex switch
        {
            0 => 3,
            1 => 4,
            2 => 5,
            3 => 6,
            4 => 7,
            _ => 5
        };
    }

    private double GetPropPitch()
    {
        return PropPitchCombo.SelectedIndex switch
        {
            0 => 4.0,
            1 => 4.5,
            2 => 5.0,
            _ => 4.5
        };
    }

    private int GetCellCount()
    {
        return CellCountCombo.SelectedIndex switch
        {
            0 => 2,
            1 => 3,
            2 => 4,
            3 => 5,
            4 => 6,
            _ => 4
        };
    }

    private double GetBatteryVoltage()
    {
        return GetCellCount() * 3.7;
    }

    private int GetMotorCount()
    {
        return FrameTypeCombo.SelectedIndex switch
        {
            0 or 1 or 2 => 4, // Quad X, +, H
            3 => 6, // Hex X
            4 => 8, // Octo X
            5 => 3, // Tricopter
            _ => 4
        };
    }

    private static string GetSelectedText(ComboBox combo)
    {
        return (combo.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "";
    }

    private enum RecommendationType
    {
        Good,
        Warning,
        Info
    }
}
