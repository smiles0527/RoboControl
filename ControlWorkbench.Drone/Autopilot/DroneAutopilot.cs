using ControlWorkbench.Drone.Devices;
using ControlWorkbench.Drone.Mission;
using ControlWorkbench.Drone.Simulation;

namespace ControlWorkbench.Drone.Autopilot;

/// <summary>
/// High-level autopilot for autonomous drone operation.
/// Implements waypoint navigation, orbit, and various flight modes.
/// </summary>
public class DroneAutopilot
{
    private readonly DroneSimulator _simulator;
    private readonly PidController _rollPid;
    private readonly PidController _pitchPid;
    private readonly PidController _yawPid;
    private readonly PidController _altPid;
    private readonly PidController _posNorthPid;
    private readonly PidController _posEastPid;
    
    private AutopilotMode _mode = AutopilotMode.Manual;
    private List<GeoPoint> _waypoints = new();
    private int _currentWaypointIndex = 0;
    private GeoPoint? _targetPosition;
    private double _targetAltitude = 50;
    private double _targetYaw = 0;
    private double _orbitRadius = 50;
    private double _orbitSpeed = 5;
    private bool _orbitClockwise = true;
    private double _orbitAngle = 0;
    
    public event Action<string>? StatusChanged;
    public event Action<int>? WaypointReached;
    
    public AutopilotMode Mode => _mode;
    public int CurrentWaypointIndex => _currentWaypointIndex;
    public int TotalWaypoints => _waypoints.Count;
    public GeoPoint? TargetPosition => _targetPosition;
    public double TargetAltitude => _targetAltitude;

    public DroneAutopilot(DroneSimulator simulator)
    {
        _simulator = simulator;
        
        // Initialize PIDs with typical drone tuning
        _rollPid = new PidController(4.0, 0.1, 0.5) { OutputMin = -0.5, OutputMax = 0.5 };
        _pitchPid = new PidController(4.0, 0.1, 0.5) { OutputMin = -0.5, OutputMax = 0.5 };
        _yawPid = new PidController(2.0, 0.05, 0.1) { OutputMin = -0.3, OutputMax = 0.3 };
        _altPid = new PidController(0.5, 0.1, 0.3) { OutputMin = -0.5, OutputMax = 0.5 };
        _posNorthPid = new PidController(1.0, 0.05, 0.3) { OutputMin = -15, OutputMax = 15 };
        _posEastPid = new PidController(1.0, 0.05, 0.3) { OutputMin = -15, OutputMax = 15 };
    }

    /// <summary>
    /// Sets the autopilot mode.
    /// </summary>
    public void SetMode(AutopilotMode mode)
    {
        _mode = mode;
        ResetControllers();
        StatusChanged?.Invoke($"Mode: {mode}");
    }

    /// <summary>
    /// Loads a mission for auto mode.
    /// </summary>
    public void LoadMission(List<GeoPoint> waypoints)
    {
        _waypoints = waypoints;
        _currentWaypointIndex = 0;
        StatusChanged?.Invoke($"Mission loaded: {waypoints.Count} waypoints");
    }

    /// <summary>
    /// Sets target position for guided mode.
    /// </summary>
    public void SetGuidedTarget(double lat, double lon, double alt)
    {
        _targetPosition = new GeoPoint(lat, lon, alt);
        _targetAltitude = alt;
        StatusChanged?.Invoke($"Guided target: {lat:F6}, {lon:F6}, {alt:F1}m");
    }

    /// <summary>
    /// Sets target altitude for altitude hold.
    /// </summary>
    public void SetTargetAltitude(double altitude)
    {
        _targetAltitude = altitude;
    }

    /// <summary>
    /// Sets target yaw for heading hold.
    /// </summary>
    public void SetTargetYaw(double yawDegrees)
    {
        _targetYaw = yawDegrees * System.Math.PI / 180;
    }

    /// <summary>
    /// Configures orbit mode.
    /// </summary>
    public void SetOrbitParameters(GeoPoint center, double radius, double speed, bool clockwise)
    {
        _targetPosition = center;
        _orbitRadius = radius;
        _orbitSpeed = speed;
        _orbitClockwise = clockwise;
        _orbitAngle = 0;
    }

    /// <summary>
    /// Main update loop - call this at regular intervals (e.g., 50-100 Hz).
    /// </summary>
    public void Update(double dt)
    {
        var state = _simulator.State;
        
        switch (_mode)
        {
            case AutopilotMode.Manual:
                // No autopilot control
                break;
                
            case AutopilotMode.Stabilize:
                UpdateStabilize(state, dt);
                break;
                
            case AutopilotMode.AltHold:
                UpdateAltHold(state, dt);
                break;
                
            case AutopilotMode.Loiter:
                UpdateLoiter(state, dt);
                break;
                
            case AutopilotMode.Guided:
                UpdateGuided(state, dt);
                break;
                
            case AutopilotMode.Auto:
                UpdateAuto(state, dt);
                break;
                
            case AutopilotMode.RTL:
                UpdateRTL(state, dt);
                break;
                
            case AutopilotMode.Orbit:
                UpdateOrbit(state, dt);
                break;
                
            case AutopilotMode.Land:
                UpdateLand(state, dt);
                break;
        }
    }

    private void UpdateStabilize(DroneState state, double dt)
    {
        // Just attitude stabilization, no position control
        double rollError = 0 - state.Roll;
        double pitchError = 0 - state.Pitch;
        double yawError = NormalizeAngle(_targetYaw - state.Yaw);
        
        double rollCmd = _rollPid.Update(rollError, dt);
        double pitchCmd = _pitchPid.Update(pitchError, dt);
        double yawCmd = _yawPid.Update(yawError, dt);
        
        // Manual throttle would be from RC, using hover for now
        double throttle = 0.5;
        
        _simulator.SetAttitudeCommand(rollCmd, pitchCmd, yawCmd, throttle);
    }

    private void UpdateAltHold(DroneState state, double dt)
    {
        // Altitude control
        double altError = _targetAltitude - state.Altitude;
        double throttleAdj = _altPid.Update(altError, dt);
        double throttle = 0.5 + throttleAdj;
        
        // Level attitude
        double rollError = 0 - state.Roll;
        double pitchError = 0 - state.Pitch;
        double yawError = NormalizeAngle(_targetYaw - state.Yaw);
        
        double rollCmd = _rollPid.Update(rollError, dt);
        double pitchCmd = _pitchPid.Update(pitchError, dt);
        double yawCmd = _yawPid.Update(yawError, dt);
        
        _simulator.SetAttitudeCommand(rollCmd, pitchCmd, yawCmd, throttle);
    }

    private void UpdateLoiter(DroneState state, double dt)
    {
        if (_targetPosition == null)
        {
            _targetPosition = new GeoPoint(state.Latitude, state.Longitude, state.Altitude);
            _targetAltitude = state.Altitude;
        }
        
        NavigateToPosition(state, _targetPosition, dt);
    }

    private void UpdateGuided(DroneState state, double dt)
    {
        if (_targetPosition == null)
            return;
        
        NavigateToPosition(state, _targetPosition, dt);
    }

    private void UpdateAuto(DroneState state, double dt)
    {
        if (_waypoints.Count == 0)
            return;
        
        if (_currentWaypointIndex >= _waypoints.Count)
        {
            // Mission complete
            SetMode(AutopilotMode.Loiter);
            return;
        }
        
        var wp = _waypoints[_currentWaypointIndex];
        double distance = CalculateDistance(state.Latitude, state.Longitude, wp.Latitude, wp.Longitude);
        
        // Check if waypoint reached
        if (distance < 2.0 && System.Math.Abs(state.Altitude - wp.Altitude) < 1.0)
        {
            WaypointReached?.Invoke(_currentWaypointIndex);
            _currentWaypointIndex++;
            StatusChanged?.Invoke($"Waypoint {_currentWaypointIndex}/{_waypoints.Count}");
            return;
        }
        
        NavigateToPosition(state, wp, dt);
    }

    private void UpdateRTL(DroneState state, double dt)
    {
        var home = new GeoPoint(
            _simulator.Specs.Motors.Length > 0 ? 0 : state.Latitude, // Use actual home if available
            0,
            _targetAltitude > state.Altitude ? _targetAltitude : state.Altitude + 10);
        
        double distance = CalculateDistance(state.Latitude, state.Longitude, home.Latitude, home.Longitude);
        
        if (distance < 3.0)
        {
            SetMode(AutopilotMode.Land);
            return;
        }
        
        NavigateToPosition(state, home, dt);
    }

    private void UpdateOrbit(DroneState state, double dt)
    {
        if (_targetPosition == null)
            return;
        
        // Calculate orbit position
        double direction = _orbitClockwise ? -1 : 1;
        _orbitAngle += direction * _orbitSpeed / _orbitRadius * dt;
        
        double targetLat = _targetPosition.Latitude + _orbitRadius / 111320 * System.Math.Cos(_orbitAngle);
        double lonScale = 111320 * System.Math.Cos(_targetPosition.Latitude * System.Math.PI / 180);
        double targetLon = _targetPosition.Longitude + _orbitRadius / lonScale * System.Math.Sin(_orbitAngle);
        
        var orbitPoint = new GeoPoint(targetLat, targetLon, _targetAltitude);
        NavigateToPosition(state, orbitPoint, dt);
        
        // Point towards center
        double bearing = CalculateBearing(state.Latitude, state.Longitude, 
                                          _targetPosition.Latitude, _targetPosition.Longitude);
        _targetYaw = bearing * System.Math.PI / 180;
    }

    private void UpdateLand(DroneState state, double dt)
    {
        // Descend at 0.5 m/s
        _targetAltitude = System.Math.Max(0, state.Altitude - 0.5 * dt);
        
        // Hold position
        if (_targetPosition == null)
            _targetPosition = new GeoPoint(state.Latitude, state.Longitude, 0);
        
        NavigateToPosition(state, _targetPosition, dt);
        
        if (state.Altitude < 0.1)
        {
            _simulator.State.Armed = false;
            StatusChanged?.Invoke("Landed and disarmed");
        }
    }

    private void NavigateToPosition(DroneState state, GeoPoint target, double dt)
    {
        // Calculate position errors in NED frame
        double latError = target.Latitude - state.Latitude;
        double lonError = target.Longitude - state.Longitude;
        
        // Convert to meters
        double metersPerDegreeLat = 111320;
        double metersPerDegreeLon = 111320 * System.Math.Cos(state.Latitude * System.Math.PI / 180);
        
        double northError = latError * metersPerDegreeLat;
        double eastError = lonError * metersPerDegreeLon;
        
        // Position controller outputs desired velocity
        double velNorthCmd = _posNorthPid.Update(northError, dt);
        double velEastCmd = _posEastPid.Update(eastError, dt);
        
        // Limit velocity
        double maxVel = 10; // m/s
        double velMag = System.Math.Sqrt(velNorthCmd * velNorthCmd + velEastCmd * velEastCmd);
        if (velMag > maxVel)
        {
            velNorthCmd = velNorthCmd / velMag * maxVel;
            velEastCmd = velEastCmd / velMag * maxVel;
        }
        
        // Velocity error
        double velNorthError = velNorthCmd - state.VelocityNorth;
        double velEastError = velEastCmd - state.VelocityEast;
        
        // Convert velocity error to desired pitch/roll
        // Rotate by yaw to get body frame commands
        double cosYaw = System.Math.Cos(state.Yaw);
        double sinYaw = System.Math.Sin(state.Yaw);
        
        double pitchCmd = (velNorthError * cosYaw + velEastError * sinYaw) * 0.1;
        double rollCmd = (-velNorthError * sinYaw + velEastError * cosYaw) * 0.1;
        
        // Limit attitude commands
        pitchCmd = System.Math.Clamp(pitchCmd, -0.3, 0.3);
        rollCmd = System.Math.Clamp(rollCmd, -0.3, 0.3);
        
        // Altitude control
        double altError = target.Altitude - state.Altitude;
        double throttleAdj = _altPid.Update(altError, dt);
        double throttle = System.Math.Clamp(0.5 + throttleAdj, 0.2, 0.9);
        
        // Yaw control - point towards next waypoint or target
        double targetYaw = CalculateBearing(state.Latitude, state.Longitude, target.Latitude, target.Longitude);
        double yawError = NormalizeAngle(targetYaw * System.Math.PI / 180 - state.Yaw);
        double yawCmd = _yawPid.Update(yawError, dt);
        
        // Inner attitude loop
        double rollError = rollCmd * 0.5 - state.Roll;
        double pitchError = pitchCmd * 0.5 - state.Pitch;
        
        double rollOutput = _rollPid.Update(rollError, dt);
        double pitchOutput = _pitchPid.Update(pitchError, dt);
        
        _simulator.SetAttitudeCommand(rollOutput, pitchOutput, yawCmd, throttle);
    }

    private void ResetControllers()
    {
        _rollPid.Reset();
        _pitchPid.Reset();
        _yawPid.Reset();
        _altPid.Reset();
        _posNorthPid.Reset();
        _posEastPid.Reset();
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > System.Math.PI) angle -= 2 * System.Math.PI;
        while (angle < -System.Math.PI) angle += 2 * System.Math.PI;
        return angle;
    }

    private static double CalculateDistance(double lat1, double lon1, double lat2, double lon2)
    {
        double dLat = (lat2 - lat1) * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1 * System.Math.PI / 180) * System.Math.Cos(lat2 * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        return 6371000 * 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
    }

    private static double CalculateBearing(double lat1, double lon1, double lat2, double lon2)
    {
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double y = System.Math.Sin(dLon) * System.Math.Cos(lat2 * System.Math.PI / 180);
        double x = System.Math.Cos(lat1 * System.Math.PI / 180) * System.Math.Sin(lat2 * System.Math.PI / 180) -
                   System.Math.Sin(lat1 * System.Math.PI / 180) * System.Math.Cos(lat2 * System.Math.PI / 180) * System.Math.Cos(dLon);
        double bearing = System.Math.Atan2(y, x) * 180 / System.Math.PI;
        return (bearing + 360) % 360;
    }
}

/// <summary>
/// Autopilot flight modes.
/// </summary>
public enum AutopilotMode
{
    Manual,
    Stabilize,
    AltHold,
    Loiter,
    Guided,
    Auto,
    RTL,
    Orbit,
    Land,
    Takeoff
}

/// <summary>
/// Simple PID controller.
/// </summary>
public class PidController
{
    public double Kp { get; set; }
    public double Ki { get; set; }
    public double Kd { get; set; }
    public double OutputMin { get; set; } = double.MinValue;
    public double OutputMax { get; set; } = double.MaxValue;
    public double IntegralMax { get; set; } = 100;
    
    private double _integral;
    private double _previousError;
    private bool _hasLastError;

    public PidController(double kp, double ki, double kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public double Update(double error, double dt)
    {
        // Proportional
        double p = Kp * error;
        
        // Integral with anti-windup
        _integral += error * dt;
        _integral = System.Math.Clamp(_integral, -IntegralMax, IntegralMax);
        double i = Ki * _integral;
        
        // Derivative
        double derivative = 0;
        if (_hasLastError && dt > 0)
        {
            derivative = (error - _previousError) / dt;
        }
        double d = Kd * derivative;
        
        _previousError = error;
        _hasLastError = true;
        
        // Output with saturation
        double output = p + i + d;
        return System.Math.Clamp(output, OutputMin, OutputMax);
    }

    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _hasLastError = false;
    }
}

/// <summary>
/// Mission executor for complex autonomous missions.
/// </summary>
public class MissionExecutor
{
    private readonly DroneAutopilot _autopilot;
    private readonly List<MissionItem> _mission = new();
    private int _currentItem = 0;
    private MissionState _state = MissionState.Idle;
    private double _itemStartTime;
    private double _elapsedTime;
    
    public event Action<int, MissionItem>? ItemStarted;
    public event Action<int, MissionItem>? ItemCompleted;
    public event Action? MissionCompleted;
    public event Action<string>? StatusMessage;
    
    public MissionState State => _state;
    public int CurrentItemIndex => _currentItem;
    public MissionItem? CurrentItem => _currentItem < _mission.Count ? _mission[_currentItem] : null;

    public MissionExecutor(DroneAutopilot autopilot)
    {
        _autopilot = autopilot;
    }

    public void LoadMission(IEnumerable<MissionItem> items)
    {
        _mission.Clear();
        _mission.AddRange(items);
        _currentItem = 0;
        _state = MissionState.Ready;
        StatusMessage?.Invoke($"Loaded {_mission.Count} items");
    }

    public void Start()
    {
        if (_mission.Count == 0)
        {
            StatusMessage?.Invoke("No mission loaded");
            return;
        }
        
        _currentItem = 0;
        _state = MissionState.Running;
        StartCurrentItem();
    }

    public void Pause()
    {
        if (_state == MissionState.Running)
        {
            _state = MissionState.Paused;
            _autopilot.SetMode(AutopilotMode.Loiter);
            StatusMessage?.Invoke("Mission paused");
        }
    }

    public void Resume()
    {
        if (_state == MissionState.Paused)
        {
            _state = MissionState.Running;
            StartCurrentItem();
            StatusMessage?.Invoke("Mission resumed");
        }
    }

    public void Stop()
    {
        _state = MissionState.Idle;
        _autopilot.SetMode(AutopilotMode.Loiter);
        StatusMessage?.Invoke("Mission stopped");
    }

    public void Update(double dt, DroneState droneState)
    {
        if (_state != MissionState.Running)
            return;
        
        _elapsedTime += dt;
        
        var item = CurrentItem;
        if (item == null)
        {
            _state = MissionState.Completed;
            MissionCompleted?.Invoke();
            return;
        }
        
        bool itemComplete = CheckItemComplete(item, droneState);
        
        if (itemComplete)
        {
            ItemCompleted?.Invoke(_currentItem, item);
            _currentItem++;
            
            if (_currentItem >= _mission.Count)
            {
                _state = MissionState.Completed;
                _autopilot.SetMode(AutopilotMode.Loiter);
                MissionCompleted?.Invoke();
                StatusMessage?.Invoke("Mission completed!");
            }
            else
            {
                StartCurrentItem();
            }
        }
    }

    private void StartCurrentItem()
    {
        var item = CurrentItem;
        if (item == null) return;
        
        _itemStartTime = _elapsedTime;
        ItemStarted?.Invoke(_currentItem, item);
        StatusMessage?.Invoke($"Item {_currentItem + 1}: {item.Type}");
        
        switch (item.Type)
        {
            case MissionItemType.Takeoff:
                _autopilot.SetTargetAltitude(item.Location?.Altitude ?? 10);
                _autopilot.SetMode(AutopilotMode.Takeoff);
                break;
                
            case MissionItemType.Waypoint:
                if (item.Location != null)
                    _autopilot.SetGuidedTarget(item.Location.Latitude, item.Location.Longitude, item.Location.Altitude);
                _autopilot.SetMode(AutopilotMode.Guided);
                break;
                
            case MissionItemType.Loiter:
                if (item.Location != null)
                    _autopilot.SetGuidedTarget(item.Location.Latitude, item.Location.Longitude, item.Location.Altitude);
                _autopilot.SetMode(AutopilotMode.Loiter);
                break;
                
            case MissionItemType.Orbit:
                if (item.Location != null)
                    _autopilot.SetOrbitParameters(item.Location, item.Radius, item.Speed, true);
                _autopilot.SetMode(AutopilotMode.Orbit);
                break;
                
            case MissionItemType.Land:
                _autopilot.SetMode(AutopilotMode.Land);
                break;
                
            case MissionItemType.ReturnToLaunch:
                _autopilot.SetMode(AutopilotMode.RTL);
                break;
        }
    }

    private bool CheckItemComplete(MissionItem item, DroneState state)
    {
        double itemElapsed = _elapsedTime - _itemStartTime;
        
        switch (item.Type)
        {
            case MissionItemType.Takeoff:
                return state.Altitude >= (item.Location?.Altitude ?? 10) - 1;
                
            case MissionItemType.Waypoint:
                if (item.Location == null) return true;
                double dist = CalculateDistance(state.Latitude, state.Longitude, 
                                                item.Location.Latitude, item.Location.Longitude);
                bool positionReached = dist < 2.0 && System.Math.Abs(state.Altitude - item.Location.Altitude) < 1.0;
                
                // Check hold time
                if (positionReached && item.HoldTime > 0)
                    return itemElapsed >= item.HoldTime;
                return positionReached;
                
            case MissionItemType.Loiter:
                return itemElapsed >= item.HoldTime;
                
            case MissionItemType.Orbit:
                double turns = item.Turns > 0 ? item.Turns : 1;
                double circumference = 2 * System.Math.PI * item.Radius;
                double timePerOrbit = circumference / item.Speed;
                return itemElapsed >= turns * timePerOrbit;
                
            case MissionItemType.Land:
                return state.Altitude < 0.1;
                
            case MissionItemType.ReturnToLaunch:
                return state.Altitude < 0.1; // Lands after RTL
                
            default:
                return true;
        }
    }

    private static double CalculateDistance(double lat1, double lon1, double lat2, double lon2)
    {
        double dLat = (lat2 - lat1) * System.Math.PI / 180;
        double dLon = (lon2 - lon1) * System.Math.PI / 180;
        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1 * System.Math.PI / 180) * System.Math.Cos(lat2 * System.Math.PI / 180) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);
        return 6371000 * 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
    }
}

public enum MissionState
{
    Idle,
    Ready,
    Running,
    Paused,
    Completed,
    Aborted
}
