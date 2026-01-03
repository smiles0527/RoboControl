namespace ControlWorkbench.VEX;

/// <summary>
/// Wheel/tracking wheel odometry for VEX robots.
/// Supports 2-wheel and 3-wheel configurations.
/// </summary>
public class VexOdometry
{
    private readonly OdometryConfig _config;
    private double _x, _y, _theta;
    private double _prevLeft, _prevRight, _prevBack;
    private bool _initialized;

    public double X => _x;
    public double Y => _y;
    public double Theta => _theta;
    public double ThetaDegrees => _theta * 180.0 / System.Math.PI;

    public VexOdometry(OdometryConfig config)
    {
        _config = config;
        Reset();
    }

    /// <summary>
    /// Update odometry with new encoder values.
    /// </summary>
    /// <param name="left">Left encoder value (degrees or inches depending on config)</param>
    /// <param name="right">Right encoder value</param>
    /// <param name="back">Back encoder value (for 3-wheel, 0 for 2-wheel)</param>
    public void Update(double left, double right, double back = 0)
    {
        if (!_initialized)
        {
            _prevLeft = left;
            _prevRight = right;
            _prevBack = back;
            _initialized = true;
            return;
        }

        // Calculate deltas
        double deltaLeft = (left - _prevLeft) * _config.LeftWheelMultiplier;
        double deltaRight = (right - _prevRight) * _config.RightWheelMultiplier;
        double deltaBack = (back - _prevBack) * _config.BackWheelMultiplier;

        // Convert to distance if encoders report degrees
        if (_config.EncoderReportsDegrees)
        {
            deltaLeft = deltaLeft / 360.0 * _config.WheelCircumference;
            deltaRight = deltaRight / 360.0 * _config.WheelCircumference;
            deltaBack = deltaBack / 360.0 * _config.BackWheelCircumference;
        }

        // Calculate heading change
        double deltaTheta;
        if (_config.UseExternalHeading)
        {
            // External heading source (e.g., IMU) - back is actually heading in radians
            deltaTheta = back - _theta;
            _theta = back;
        }
        else if (_config.TrackingConfiguration == TrackingConfig.ThreeWheel)
        {
            // 3-wheel odometry
            deltaTheta = (deltaRight - deltaLeft) / _config.TrackWidth;
        }
        else
        {
            // 2-wheel odometry
            deltaTheta = (deltaRight - deltaLeft) / _config.TrackWidth;
        }

        // Calculate local displacement
        double localX, localY;
        
        if (_config.TrackingConfiguration == TrackingConfig.ThreeWheel)
        {
            // With back wheel for strafe detection
            if (System.Math.Abs(deltaTheta) < 1e-6)
            {
                // Straight line motion
                localX = deltaBack;
                localY = (deltaLeft + deltaRight) / 2.0;
            }
            else
            {
                // Arc motion
                double radiusRight = deltaRight / deltaTheta;
                double radiusBack = deltaBack / deltaTheta;
                
                localY = 2.0 * System.Math.Sin(deltaTheta / 2.0) * 
                    (radiusRight - _config.TrackWidth / 2.0 + _config.RightOffset);
                localX = 2.0 * System.Math.Sin(deltaTheta / 2.0) * 
                    (radiusBack + _config.BackOffset);
            }
        }
        else
        {
            // 2-wheel (no strafe detection)
            if (System.Math.Abs(deltaTheta) < 1e-6)
            {
                localY = (deltaLeft + deltaRight) / 2.0;
                localX = 0;
            }
            else
            {
                double radius = (deltaLeft + deltaRight) / (2.0 * deltaTheta);
                localY = 2.0 * radius * System.Math.Sin(deltaTheta / 2.0);
                localX = 0;
            }
        }

        // Rotate to global frame and update position
        double avgTheta = _theta + deltaTheta / 2.0;
        double cosTheta = System.Math.Cos(avgTheta);
        double sinTheta = System.Math.Sin(avgTheta);

        _x += localY * sinTheta + localX * cosTheta;
        _y += localY * cosTheta - localX * sinTheta;
        
        if (!_config.UseExternalHeading)
        {
            _theta += deltaTheta;
            // Normalize theta to [-?, ?]
            while (_theta > System.Math.PI) _theta -= 2.0 * System.Math.PI;
            while (_theta < -System.Math.PI) _theta += 2.0 * System.Math.PI;
        }

        // Store for next iteration
        _prevLeft = left;
        _prevRight = right;
        _prevBack = back;
    }

    /// <summary>
    /// Update odometry with IMU heading.
    /// </summary>
    public void UpdateWithImu(double left, double right, double imuHeadingRadians)
    {
        if (!_initialized)
        {
            _prevLeft = left;
            _prevRight = right;
            _theta = imuHeadingRadians;
            _initialized = true;
            return;
        }

        // Calculate encoder deltas
        double deltaLeft = (left - _prevLeft) * _config.LeftWheelMultiplier;
        double deltaRight = (right - _prevRight) * _config.RightWheelMultiplier;

        if (_config.EncoderReportsDegrees)
        {
            deltaLeft = deltaLeft / 360.0 * _config.WheelCircumference;
            deltaRight = deltaRight / 360.0 * _config.WheelCircumference;
        }

        // Use IMU for heading
        double deltaTheta = imuHeadingRadians - _theta;
        
        // Handle wrap-around
        while (deltaTheta > System.Math.PI) deltaTheta -= 2.0 * System.Math.PI;
        while (deltaTheta < -System.Math.PI) deltaTheta += 2.0 * System.Math.PI;

        // Calculate forward displacement
        double deltaForward = (deltaLeft + deltaRight) / 2.0;

        // Update position using midpoint heading
        double avgTheta = _theta + deltaTheta / 2.0;
        _x += deltaForward * System.Math.Sin(avgTheta);
        _y += deltaForward * System.Math.Cos(avgTheta);
        _theta = imuHeadingRadians;

        _prevLeft = left;
        _prevRight = right;
    }

    /// <summary>
    /// Reset odometry to origin or specified position.
    /// </summary>
    public void Reset(double x = 0, double y = 0, double theta = 0)
    {
        _x = x;
        _y = y;
        _theta = theta;
        _initialized = false;
    }

    /// <summary>
    /// Set position without resetting encoder tracking.
    /// </summary>
    public void SetPosition(double x, double y, double theta)
    {
        _x = x;
        _y = y;
        _theta = theta;
    }

    /// <summary>
    /// Get current pose as a tuple.
    /// </summary>
    public (double x, double y, double theta) GetPose() => (_x, _y, _theta);
}

/// <summary>
/// Odometry configuration.
/// </summary>
public class OdometryConfig
{
    /// <summary>
    /// Tracking wheel configuration type.
    /// </summary>
    public TrackingConfig TrackingConfiguration { get; set; } = TrackingConfig.TwoWheel;

    /// <summary>
    /// Distance between left and right tracking wheels (center to center) in inches.
    /// </summary>
    public double TrackWidth { get; set; } = 12.0;

    /// <summary>
    /// Tracking wheel diameter in inches.
    /// </summary>
    public double WheelDiameter { get; set; } = 2.75;

    /// <summary>
    /// Wheel circumference (calculated from diameter).
    /// </summary>
    public double WheelCircumference => WheelDiameter * System.Math.PI;

    /// <summary>
    /// Back wheel diameter (for 3-wheel config).
    /// </summary>
    public double BackWheelDiameter { get; set; } = 2.75;

    /// <summary>
    /// Back wheel circumference.
    /// </summary>
    public double BackWheelCircumference => BackWheelDiameter * System.Math.PI;

    /// <summary>
    /// Offset of right wheel from center (positive = right of center).
    /// </summary>
    public double RightOffset { get; set; } = 0;

    /// <summary>
    /// Offset of back wheel from center (positive = behind center).
    /// </summary>
    public double BackOffset { get; set; } = 0;

    /// <summary>
    /// Left wheel direction multiplier (1 or -1).
    /// </summary>
    public double LeftWheelMultiplier { get; set; } = 1.0;

    /// <summary>
    /// Right wheel direction multiplier (1 or -1).
    /// </summary>
    public double RightWheelMultiplier { get; set; } = 1.0;

    /// <summary>
    /// Back wheel direction multiplier (1 or -1).
    /// </summary>
    public double BackWheelMultiplier { get; set; } = 1.0;

    /// <summary>
    /// Whether encoders report in degrees (true) or already converted to distance (false).
    /// </summary>
    public bool EncoderReportsDegrees { get; set; } = true;

    /// <summary>
    /// Whether to use external heading source (IMU) instead of wheel-based heading.
    /// </summary>
    public bool UseExternalHeading { get; set; } = false;

    /// <summary>
    /// Creates a standard 2-wheel configuration.
    /// </summary>
    public static OdometryConfig TwoWheelConfig(double trackWidth, double wheelDiameter = 2.75)
    {
        return new OdometryConfig
        {
            TrackingConfiguration = TrackingConfig.TwoWheel,
            TrackWidth = trackWidth,
            WheelDiameter = wheelDiameter
        };
    }

    /// <summary>
    /// Creates a 3-wheel configuration with back wheel.
    /// </summary>
    public static OdometryConfig ThreeWheelConfig(
        double trackWidth, 
        double backOffset,
        double wheelDiameter = 2.75)
    {
        return new OdometryConfig
        {
            TrackingConfiguration = TrackingConfig.ThreeWheel,
            TrackWidth = trackWidth,
            BackOffset = backOffset,
            WheelDiameter = wheelDiameter,
            BackWheelDiameter = wheelDiameter
        };
    }

    /// <summary>
    /// Creates a 2-wheel + IMU configuration.
    /// </summary>
    public static OdometryConfig TwoWheelWithImuConfig(double trackWidth, double wheelDiameter = 2.75)
    {
        return new OdometryConfig
        {
            TrackingConfiguration = TrackingConfig.TwoWheel,
            TrackWidth = trackWidth,
            WheelDiameter = wheelDiameter,
            UseExternalHeading = true
        };
    }
}

/// <summary>
/// Tracking wheel configuration type.
/// </summary>
public enum TrackingConfig
{
    /// <summary>
    /// Two parallel tracking wheels (left/right).
    /// </summary>
    TwoWheel,

    /// <summary>
    /// Three tracking wheels (left, right, back perpendicular).
    /// </summary>
    ThreeWheel,

    /// <summary>
    /// Integrated motor encoders (less accurate).
    /// </summary>
    IntegratedEncoders
}

/// <summary>
/// Sensor fusion combining odometry with IMU and optionally GPS.
/// </summary>
public class VexSensorFusion
{
    private readonly VexOdometry _odometry;
    private readonly ComplementaryFilter _headingFilter;
    private readonly ComplementaryFilter _xFilter;
    private readonly ComplementaryFilter _yFilter;
    private bool _hasGps;
    private double _gpsX, _gpsY, _gpsHeading;
    private double _gpsWeight = 0.1;

    public double X { get; private set; }
    public double Y { get; private set; }
    public double Theta { get; private set; }
    public double ThetaDegrees => Theta * 180.0 / System.Math.PI;

    public VexSensorFusion(OdometryConfig odomConfig, double imuWeight = 0.98)
    {
        _odometry = new VexOdometry(odomConfig);
        _headingFilter = new ComplementaryFilter(imuWeight);
        _xFilter = new ComplementaryFilter(0.95);
        _yFilter = new ComplementaryFilter(0.95);
    }

    /// <summary>
    /// Update with encoder and IMU data.
    /// </summary>
    public void Update(double leftEncoder, double rightEncoder, double imuHeadingDeg, double dt)
    {
        // Update odometry
        double imuHeadingRad = imuHeadingDeg * System.Math.PI / 180.0;
        _odometry.UpdateWithImu(leftEncoder, rightEncoder, imuHeadingRad);

        // Fuse heading (odometry heading is already from IMU in this mode)
        Theta = _odometry.Theta;

        // Apply GPS correction if available
        if (_hasGps)
        {
            X = _xFilter.Update(_gpsX, _odometry.X);
            Y = _yFilter.Update(_gpsY, _odometry.Y);
        }
        else
        {
            X = _odometry.X;
            Y = _odometry.Y;
        }
    }

    /// <summary>
    /// Update with 3-wheel odometry and IMU.
    /// </summary>
    public void Update(double leftEncoder, double rightEncoder, double backEncoder, 
                       double imuHeadingDeg, double dt)
    {
        double imuHeadingRad = imuHeadingDeg * System.Math.PI / 180.0;
        
        // Use IMU heading with 3-wheel odometry for position
        _odometry.Update(leftEncoder, rightEncoder, backEncoder);
        
        // Fuse odometry heading with IMU
        Theta = _headingFilter.Update(imuHeadingRad, _odometry.Theta);

        if (_hasGps)
        {
            X = _xFilter.Update(_gpsX, _odometry.X);
            Y = _yFilter.Update(_gpsY, _odometry.Y);
        }
        else
        {
            X = _odometry.X;
            Y = _odometry.Y;
        }
    }

    /// <summary>
    /// Provide GPS update for fusion.
    /// </summary>
    public void UpdateGps(double x, double y, double headingDeg, int quality)
    {
        if (quality > 50) // Only use good GPS fixes
        {
            _gpsX = x;
            _gpsY = y;
            _gpsHeading = headingDeg * System.Math.PI / 180.0;
            _hasGps = true;
            
            // Adjust weight based on quality
            _gpsWeight = quality / 1000.0; // 0.05 to 0.1
            _xFilter.SetAlpha(1.0 - _gpsWeight);
            _yFilter.SetAlpha(1.0 - _gpsWeight);
        }
    }

    /// <summary>
    /// Reset to specified position.
    /// </summary>
    public void Reset(double x = 0, double y = 0, double theta = 0)
    {
        _odometry.Reset(x, y, theta);
        X = x;
        Y = y;
        Theta = theta;
        _hasGps = false;
    }

    public (double x, double y, double theta) GetPose() => (X, Y, Theta);
}

/// <summary>
/// Simple complementary filter for sensor fusion.
/// </summary>
public class ComplementaryFilter
{
    private double _alpha;
    private double _value;
    private bool _initialized;

    public ComplementaryFilter(double alpha = 0.98)
    {
        _alpha = alpha;
    }

    public double Update(double highFreqMeasurement, double lowFreqMeasurement)
    {
        if (!_initialized)
        {
            _value = lowFreqMeasurement;
            _initialized = true;
            return _value;
        }

        _value = _alpha * highFreqMeasurement + (1 - _alpha) * lowFreqMeasurement;
        return _value;
    }

    public void SetAlpha(double alpha) => _alpha = alpha;
    public void Reset() => _initialized = false;
    public double Value => _value;
}
