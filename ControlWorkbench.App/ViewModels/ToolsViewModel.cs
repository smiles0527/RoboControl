using System.Text;
using System.Windows.Input;
using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Analysis;
using ControlWorkbench.Math.Calibration;
using ControlWorkbench.Math.Simulation;
using ControlWorkbench.Math.Units;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// ViewModel for the Tools & Calibration tab.
/// </summary>
public class ToolsViewModel : ViewModelBase
{
    // Encoder Calculator
    private int _encoderPpr = 360;
    private bool _quadratureDecoding = true;
    private double _wheelRadius = 0.05;
    private double _gearRatio = 1.0;
    private double _encoderSampleRate = 100.0;
    private string _encoderResult = "";

    // Odometry Calculator
    private double _trackWidth = 0.3;
    private string _leftEncoderCounts = "1000";
    private string _rightEncoderCounts = "1050";
    private double _odomPoseX = 0;
    private double _odomPoseY = 0;
    private double _odomPoseTheta = 0;
    private string _odometryResult = "";

    // Unit Converter
    private double _inputValue = 1.0;
    private string _selectedConversion = "Degrees to Radians";
    private string _conversionResult = "";

    // System Identification
    private double _sysIdGain = 1.0;
    private double _sysIdTimeConstant = 1.0;
    private double _sysIdDamping = 0.7;
    private double _sysIdNaturalFreq = 1.0;
    private string _selectedSystemType = "First Order";
    private double _sysIdStepMagnitude = 1.0;
    private double _sysIdDuration = 10.0;
    private double _sysIdNoiseLevel = 0.05;
    private string _sysIdResult = "";

    // Frequency Analysis
    private double _fftSampleRate = 100.0;
    private double _fftSignalFreq = 5.0;
    private double _fftDuration = 5.0;
    private string _selectedWindow = "Hanning";
    private string _fftResult = "";

    // Navigation Calculations
    private double _lat1 = 37.7749;
    private double _lon1 = -122.4194;
    private double _lat2 = 34.0522;
    private double _lon2 = -118.2437;
    private string _navigationResult = "";

    // Robotics Formulas
    private double _motorKt = 0.05;
    private double _motorKe = 0.05;
    private double _motorR = 1.0;
    private double _motorVoltage = 12.0;
    private double _motorCurrent = 2.0;
    private string _motorResult = "";

    public ToolsViewModel()
    {
        ComputeEncoderCommand = new RelayCommand(ComputeEncoder);
        ComputeOdometryCommand = new RelayCommand(ComputeOdometry);
        ResetOdometryCommand = new RelayCommand(ResetOdometry);
        ConvertUnitsCommand = new RelayCommand(ConvertUnits);
        GenerateTestDataCommand = new RelayCommand(GenerateTestData);
        ComputeFftCommand = new RelayCommand(ComputeFft);
        ComputeNavigationCommand = new RelayCommand(ComputeNavigation);
        ComputeMotorCommand = new RelayCommand(ComputeMotor);
    }

    // Commands
    public ICommand ComputeEncoderCommand { get; }
    public ICommand ComputeOdometryCommand { get; }
    public ICommand ResetOdometryCommand { get; }
    public ICommand ConvertUnitsCommand { get; }
    public ICommand GenerateTestDataCommand { get; }
    public ICommand ComputeFftCommand { get; }
    public ICommand ComputeNavigationCommand { get; }
    public ICommand ComputeMotorCommand { get; }

    // Encoder Properties
    public int EncoderPpr { get => _encoderPpr; set => SetProperty(ref _encoderPpr, value); }
    public bool QuadratureDecoding { get => _quadratureDecoding; set => SetProperty(ref _quadratureDecoding, value); }
    public double WheelRadius { get => _wheelRadius; set => SetProperty(ref _wheelRadius, value); }
    public double GearRatio { get => _gearRatio; set => SetProperty(ref _gearRatio, value); }
    public double EncoderSampleRate { get => _encoderSampleRate; set => SetProperty(ref _encoderSampleRate, value); }
    public string EncoderResult { get => _encoderResult; set => SetProperty(ref _encoderResult, value); }

    // Odometry Properties
    public double TrackWidth { get => _trackWidth; set => SetProperty(ref _trackWidth, value); }
    public string LeftEncoderCounts { get => _leftEncoderCounts; set => SetProperty(ref _leftEncoderCounts, value); }
    public string RightEncoderCounts { get => _rightEncoderCounts; set => SetProperty(ref _rightEncoderCounts, value); }
    public double OdomPoseX { get => _odomPoseX; set => SetProperty(ref _odomPoseX, value); }
    public double OdomPoseY { get => _odomPoseY; set => SetProperty(ref _odomPoseY, value); }
    public double OdomPoseTheta { get => _odomPoseTheta; set => SetProperty(ref _odomPoseTheta, value); }
    public string OdometryResult { get => _odometryResult; set => SetProperty(ref _odometryResult, value); }

    // Unit Converter Properties
    public double InputValue { get => _inputValue; set => SetProperty(ref _inputValue, value); }
    public string SelectedConversion { get => _selectedConversion; set => SetProperty(ref _selectedConversion, value); }
    public string ConversionResult { get => _conversionResult; set => SetProperty(ref _conversionResult, value); }

    public string[] ConversionTypes => [
        "Degrees to Radians",
        "Radians to Degrees",
        "RPM to rad/s",
        "rad/s to RPM",
        "m/s to km/h",
        "km/h to m/s",
        "m/s to mph",
        "mph to m/s",
        "G to m/s²",
        "m/s² to G",
        "Hz to rad/s",
        "rad/s to Hz",
        "Gauss to µT",
        "µT to Gauss",
        "Nm to oz-in",
        "oz-in to Nm"
    ];

    // System ID Properties
    public double SysIdGain { get => _sysIdGain; set => SetProperty(ref _sysIdGain, value); }
    public double SysIdTimeConstant { get => _sysIdTimeConstant; set => SetProperty(ref _sysIdTimeConstant, value); }
    public double SysIdDamping { get => _sysIdDamping; set => SetProperty(ref _sysIdDamping, value); }
    public double SysIdNaturalFreq { get => _sysIdNaturalFreq; set => SetProperty(ref _sysIdNaturalFreq, value); }
    public string SelectedSystemType { get => _selectedSystemType; set => SetProperty(ref _selectedSystemType, value); }
    public double SysIdStepMagnitude { get => _sysIdStepMagnitude; set => SetProperty(ref _sysIdStepMagnitude, value); }
    public double SysIdDuration { get => _sysIdDuration; set => SetProperty(ref _sysIdDuration, value); }
    public double SysIdNoiseLevel { get => _sysIdNoiseLevel; set => SetProperty(ref _sysIdNoiseLevel, value); }
    public string SysIdResult { get => _sysIdResult; set => SetProperty(ref _sysIdResult, value); }
    public string[] SystemTypes => ["First Order", "Second Order"];

    // FFT Properties
    public double FftSampleRate { get => _fftSampleRate; set => SetProperty(ref _fftSampleRate, value); }
    public double FftSignalFreq { get => _fftSignalFreq; set => SetProperty(ref _fftSignalFreq, value); }
    public double FftDuration { get => _fftDuration; set => SetProperty(ref _fftDuration, value); }
    public string SelectedWindow { get => _selectedWindow; set => SetProperty(ref _selectedWindow, value); }
    public string FftResult { get => _fftResult; set => SetProperty(ref _fftResult, value); }
    public string[] WindowTypes => ["Rectangular", "Hanning", "Hamming", "Blackman"];

    // Navigation Properties
    public double Lat1 { get => _lat1; set => SetProperty(ref _lat1, value); }
    public double Lon1 { get => _lon1; set => SetProperty(ref _lon1, value); }
    public double Lat2 { get => _lat2; set => SetProperty(ref _lat2, value); }
    public double Lon2 { get => _lon2; set => SetProperty(ref _lon2, value); }
    public string NavigationResult { get => _navigationResult; set => SetProperty(ref _navigationResult, value); }

    // Motor Properties
    public double MotorKt { get => _motorKt; set => SetProperty(ref _motorKt, value); }
    public double MotorKe { get => _motorKe; set => SetProperty(ref _motorKe, value); }
    public double MotorR { get => _motorR; set => SetProperty(ref _motorR, value); }
    public double MotorVoltage { get => _motorVoltage; set => SetProperty(ref _motorVoltage, value); }
    public double MotorCurrent { get => _motorCurrent; set => SetProperty(ref _motorCurrent, value); }
    public string MotorResult { get => _motorResult; set => SetProperty(ref _motorResult, value); }

    private void ComputeEncoder()
    {
        EncoderResult = EncoderCalculations.GenerateReport(
            EncoderPpr, QuadratureDecoding, WheelRadius, EncoderSampleRate, GearRatio);
    }

    private OdometryCalculator? _odometryCalc;
    private long _lastLeftCount = 0;
    private long _lastRightCount = 0;

    private void ComputeOdometry()
    {
        try
        {
            int cpr = EncoderCalculations.CalculateCpr(EncoderPpr, QuadratureDecoding);
            
            if (_odometryCalc == null)
            {
                var config = new OdometryCalculator.DifferentialDriveConfig
                {
                    WheelRadius = WheelRadius,
                    TrackWidth = TrackWidth,
                    CountsPerRevolution = cpr,
                    GearRatio = GearRatio
                };
                _odometryCalc = new OdometryCalculator(config);
            }

            long leftCount = long.Parse(LeftEncoderCounts);
            long rightCount = long.Parse(RightEncoderCounts);

            var pose = _odometryCalc.Update(leftCount, rightCount);

            OdomPoseX = pose.X;
            OdomPoseY = pose.Y;
            OdomPoseTheta = pose.Theta * 180 / System.Math.PI;

            var sb = new StringBuilder();
            sb.AppendLine("=== Odometry Update ===");
            sb.AppendLine();
            sb.AppendLine($"Encoder Counts: L={leftCount}, R={rightCount}");
            sb.AppendLine($"Delta Counts: L={leftCount - _lastLeftCount}, R={rightCount - _lastRightCount}");
            sb.AppendLine();
            sb.AppendLine($"Current Pose:");
            sb.AppendLine($"  X: {pose.X:F4} m");
            sb.AppendLine($"  Y: {pose.Y:F4} m");
            sb.AppendLine($"  ?: {pose.Theta * 180 / System.Math.PI:F2}°");

            _lastLeftCount = leftCount;
            _lastRightCount = rightCount;

            OdometryResult = sb.ToString();
        }
        catch (Exception ex)
        {
            OdometryResult = $"Error: {ex.Message}";
        }
    }

    private void ResetOdometry()
    {
        _odometryCalc = null;
        _lastLeftCount = 0;
        _lastRightCount = 0;
        OdomPoseX = 0;
        OdomPoseY = 0;
        OdomPoseTheta = 0;
        LeftEncoderCounts = "0";
        RightEncoderCounts = "0";
        OdometryResult = "Odometry reset.";
    }

    private void ConvertUnits()
    {
        double result = SelectedConversion switch
        {
            "Degrees to Radians" => UnitConverter.DegToRad(InputValue),
            "Radians to Degrees" => UnitConverter.RadToDeg(InputValue),
            "RPM to rad/s" => UnitConverter.RpmToRadPerSec(InputValue),
            "rad/s to RPM" => UnitConverter.RadPerSecToRpm(InputValue),
            "m/s to km/h" => UnitConverter.MpsToKph(InputValue),
            "km/h to m/s" => UnitConverter.KphToMps(InputValue),
            "m/s to mph" => UnitConverter.MpsToMph(InputValue),
            "mph to m/s" => UnitConverter.MphToMps(InputValue),
            "G to m/s²" => UnitConverter.GToMps2(InputValue),
            "m/s² to G" => UnitConverter.Mps2ToG(InputValue),
            "Hz to rad/s" => UnitConverter.HzToRadPerSec(InputValue),
            "rad/s to Hz" => UnitConverter.RadPerSecToHz(InputValue),
            "Gauss to µT" => UnitConverter.GaussToMicroTesla(InputValue),
            "µT to Gauss" => UnitConverter.MicroTeslaToGauss(InputValue),
            "Nm to oz-in" => UnitConverter.NmToOzIn(InputValue),
            "oz-in to Nm" => UnitConverter.OzInToNm(InputValue),
            _ => InputValue
        };

        ConversionResult = $"{InputValue} ? {result:G6}";
    }

    private void GenerateTestData()
    {
        try
        {
            var systemType = SelectedSystemType == "First Order" ? SystemType.FirstOrder : SystemType.SecondOrder;
            var parameters = new SystemParameters
            {
                Gain = SysIdGain,
                TimeConstant = SysIdTimeConstant,
                DampingRatio = SysIdDamping,
                NaturalFrequency = SysIdNaturalFreq
            };

            var data = TestDataGenerator.GenerateStepResponse(
                systemType, parameters, SysIdStepMagnitude, SysIdDuration, 100, SysIdNoiseLevel);

            // Run system identification on the generated data
            var sb = new StringBuilder();
            sb.AppendLine("=== Generated Test Data & System ID ===");
            sb.AppendLine();
            sb.AppendLine($"True Parameters:");
            sb.AppendLine($"  Gain: {SysIdGain:F3}");
            
            if (systemType == SystemType.FirstOrder)
            {
                sb.AppendLine($"  Time Constant: {SysIdTimeConstant:F3} s");
                
                var estimate = SystemIdentification.EstimateFirstOrder(
                    data.Time, data.Output, SysIdStepMagnitude);
                
                sb.AppendLine();
                sb.AppendLine("Estimated Parameters:");
                sb.AppendLine($"  Gain: {estimate.Gain:F3} (error: {System.Math.Abs(estimate.Gain - SysIdGain) / SysIdGain * 100:F1}%)");
                sb.AppendLine($"  Time Constant: {estimate.TimeConstant:F3} s (error: {System.Math.Abs(estimate.TimeConstant - SysIdTimeConstant) / SysIdTimeConstant * 100:F1}%)");
                sb.AppendLine($"  RMSE: {estimate.Rmse:F4}");
            }
            else
            {
                sb.AppendLine($"  Natural Frequency: {SysIdNaturalFreq:F3} rad/s");
                sb.AppendLine($"  Damping Ratio: {SysIdDamping:F3}");
                
                var estimate = SystemIdentification.EstimateSecondOrder(
                    data.Time, data.Output, SysIdStepMagnitude);
                
                sb.AppendLine();
                sb.AppendLine("Estimated Parameters:");
                sb.AppendLine($"  Gain: {estimate.Gain:F3}");
                sb.AppendLine($"  Natural Frequency: {estimate.NaturalFrequency:F3} rad/s");
                sb.AppendLine($"  Damping Ratio: {estimate.DampingRatio:F3}");
                sb.AppendLine($"  Overshoot: {estimate.Overshoot:F1}%");
                sb.AppendLine($"  RMSE: {estimate.Rmse:F4}");
            }

            sb.AppendLine();
            sb.AppendLine("Sample Data Points:");
            sb.AppendLine("Time (s)     Input    Output");
            for (int i = 0; i < data.Length; i += data.Length / 15)
            {
                sb.AppendLine($"{data.Time[i]:F2}         {data.Input[i]:F2}     {data.Output[i]:F4}");
            }

            SysIdResult = sb.ToString();
        }
        catch (Exception ex)
        {
            SysIdResult = $"Error: {ex.Message}";
        }
    }

    private void ComputeFft()
    {
        try
        {
            // Generate test signal
            var signal = TestDataGenerator.GenerateSineWave(
                FftSignalFreq, 1.0, FftDuration, FftSampleRate,
                noiseLevel: 0.1);

            // Apply window
            var windowType = SelectedWindow switch
            {
                "Rectangular" => WindowType.Rectangular,
                "Hanning" => WindowType.Hanning,
                "Hamming" => WindowType.Hamming,
                "Blackman" => WindowType.Blackman,
                _ => WindowType.Hanning
            };

            var windowed = FrequencyAnalysis.ApplyWindow(signal.Signal, windowType);
            var fft = FrequencyAnalysis.ComputeFft(windowed, FftSampleRate);

            var peaks = FrequencyAnalysis.FindPeaks(fft, 0.05, 5);

            var sb = new StringBuilder();
            sb.AppendLine("=== FFT Analysis ===");
            sb.AppendLine();
            sb.AppendLine($"Signal: {FftSignalFreq:F1} Hz sine wave");
            sb.AppendLine($"Sample Rate: {FftSampleRate:F0} Hz");
            sb.AppendLine($"Duration: {FftDuration:F1} s ({signal.Length} samples)");
            sb.AppendLine($"Window: {SelectedWindow}");
            sb.AppendLine($"Frequency Resolution: {FftSampleRate / signal.Length:F3} Hz");
            sb.AppendLine();
            sb.AppendLine("Detected Peaks:");
            foreach (var peak in peaks)
            {
                sb.AppendLine($"  {peak.Frequency:F2} Hz  (magnitude: {peak.Magnitude:F4})");
            }
            sb.AppendLine();
            sb.AppendLine("Spectrum (first 20 bins):");
            sb.AppendLine("Freq (Hz)    Magnitude");
            for (int i = 0; i < System.Math.Min(20, fft.Frequencies.Length); i++)
            {
                sb.AppendLine($"{fft.Frequencies[i]:F2}         {fft.Magnitudes[i]:F4}");
            }

            FftResult = sb.ToString();
        }
        catch (Exception ex)
        {
            FftResult = $"Error: {ex.Message}";
        }
    }

    private void ComputeNavigation()
    {
        try
        {
            double distance = NavigationFormulas.HaversineDistance(Lat1, Lon1, Lat2, Lon2);
            double bearing = NavigationFormulas.Bearing(Lat1, Lon1, Lat2, Lon2);

            var sb = new StringBuilder();
            sb.AppendLine("=== Navigation Calculations ===");
            sb.AppendLine();
            sb.AppendLine($"Point 1: ({Lat1:F4}°, {Lon1:F4}°)");
            sb.AppendLine($"Point 2: ({Lat2:F4}°, {Lon2:F4}°)");
            sb.AppendLine();
            sb.AppendLine($"Distance: {distance:F0} m ({distance / 1000:F2} km)");
            sb.AppendLine($"Bearing: {bearing:F1}°");
            sb.AppendLine();

            // Compute a waypoint
            var (midLat, midLon) = NavigationFormulas.DestinationPoint(Lat1, Lon1, bearing, distance / 2);
            sb.AppendLine($"Midpoint: ({midLat:F4}°, {midLon:F4}°)");
            sb.AppendLine();

            // ENU conversion
            var (e, n, u) = NavigationFormulas.GpsToEnu(Lat2, Lon2, 0, Lat1, Lon1, 0);
            sb.AppendLine($"ENU from Point 1 to Point 2:");
            sb.AppendLine($"  East: {e:F1} m");
            sb.AppendLine($"  North: {n:F1} m");
            sb.AppendLine($"  Up: {u:F1} m");

            NavigationResult = sb.ToString();
        }
        catch (Exception ex)
        {
            NavigationResult = $"Error: {ex.Message}";
        }
    }

    private void ComputeMotor()
    {
        try
        {
            double torque = RoboticsFormulas.MotorTorque(MotorCurrent, MotorKt);
            double noLoadSpeed = RoboticsFormulas.NoLoadSpeed(MotorVoltage, MotorKe);
            double stallTorque = RoboticsFormulas.StallTorque(MotorVoltage, MotorR, MotorKt);
            double speed = RoboticsFormulas.MotorSpeed(MotorVoltage, MotorCurrent, MotorR, MotorKe);
            double power = RoboticsFormulas.MotorPower(torque, speed);

            var sb = new StringBuilder();
            sb.AppendLine("=== Motor Calculations ===");
            sb.AppendLine();
            sb.AppendLine("Motor Constants:");
            sb.AppendLine($"  Kt (torque): {MotorKt:F4} Nm/A");
            sb.AppendLine($"  Ke (back-EMF): {MotorKe:F4} V/(rad/s)");
            sb.AppendLine($"  R (resistance): {MotorR:F2} ?");
            sb.AppendLine();
            sb.AppendLine("Operating Point:");
            sb.AppendLine($"  Voltage: {MotorVoltage:F1} V");
            sb.AppendLine($"  Current: {MotorCurrent:F2} A");
            sb.AppendLine();
            sb.AppendLine("Results:");
            sb.AppendLine($"  Torque: {torque:F4} Nm ({UnitConverter.NmToOzIn(torque):F2} oz-in)");
            sb.AppendLine($"  Speed: {speed:F2} rad/s ({UnitConverter.RadPerSecToRpm(speed):F0} RPM)");
            sb.AppendLine($"  Power: {power:F2} W");
            sb.AppendLine();
            sb.AppendLine("Motor Limits:");
            sb.AppendLine($"  No-load speed: {noLoadSpeed:F2} rad/s ({UnitConverter.RadPerSecToRpm(noLoadSpeed):F0} RPM)");
            sb.AppendLine($"  Stall torque: {stallTorque:F4} Nm ({UnitConverter.NmToOzIn(stallTorque):F2} oz-in)");
            sb.AppendLine($"  Stall current: {MotorVoltage / MotorR:F2} A");

            MotorResult = sb.ToString();
        }
        catch (Exception ex)
        {
            MotorResult = $"Error: {ex.Message}";
        }
    }
}
