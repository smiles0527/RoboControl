using ControlWorkbench.Drone.Devices;
using ControlWorkbench.Drone.Simulation;

namespace ControlWorkbench.Drone.Services;

/// <summary>
/// Service that manages drone connections and provides a unified telemetry source.
/// Can connect to real flight controllers via MAVLink or run in simulation mode.
/// </summary>
public class DroneConnectionService : IDisposable
{
    private MavlinkFlightController? _flightController;
    private DroneSimulator? _simulator;
    private CancellationTokenSource? _simCts;
    private Task? _simTask;
    
    public ConnectionMode Mode { get; private set; } = ConnectionMode.Disconnected;
    public bool IsConnected => Mode != ConnectionMode.Disconnected;
    public DroneSpecs CurrentSpecs { get; set; } = new();
    
    // Events for UI binding
    public event Action<DroneTelemetry>? TelemetryReceived;
    public event Action<DroneStatus>? StatusChanged;
    public event Action<string>? LogReceived;
    public event Action<ConnectionMode>? ConnectionStateChanged;
    
    /// <summary>
    /// Connect to a real flight controller via MAVLink.
    /// </summary>
    /// <param name="connectionString">Connection string (e.g., "tcp:127.0.0.1:5760", "serial:COM3:57600")</param>
    public async Task<bool> ConnectAsync(string connectionString, CancellationToken ct = default)
    {
        await DisconnectAsync();
        
        try
        {
            _flightController = new MavlinkFlightController("Connected FC", FlightControllerType.ArduPilot);
            
            // Wire up events
            _flightController.TelemetryReceived += OnTelemetryReceived;
            _flightController.StatusChanged += OnStatusChanged;
            _flightController.LogReceived += OnLogReceived;
            
            bool success = await _flightController.ConnectAsync(connectionString, ct);
            
            if (success)
            {
                Mode = ConnectionMode.Hardware;
                ConnectionStateChanged?.Invoke(Mode);
                LogReceived?.Invoke($"Connected to flight controller: {connectionString}");
                return true;
            }
            else
            {
                _flightController = null;
                LogReceived?.Invoke("Failed to connect to flight controller");
                return false;
            }
        }
        catch (Exception ex)
        {
            LogReceived?.Invoke($"Connection error: {ex.Message}");
            _flightController = null;
            return false;
        }
    }
    
    /// <summary>
    /// Start simulation mode with the current drone specs.
    /// </summary>
    public void StartSimulation(double latitude = 40.712776, double longitude = -74.005974, double heading = 0)
    {
        DisconnectAsync().Wait();
        
        _simulator = new DroneSimulator
        {
            Specs = CurrentSpecs
        };
        
        _simulator.Reset(latitude, longitude, 0, heading);
        _simulator.StateUpdated += OnSimulatorStateUpdated;
        _simulator.LogMessage += OnLogReceived;
        
        // Start simulation loop
        _simCts = new CancellationTokenSource();
        _simTask = Task.Run(() => SimulationLoopAsync(_simCts.Token));
        
        Mode = ConnectionMode.Simulation;
        ConnectionStateChanged?.Invoke(Mode);
        LogReceived?.Invoke("Simulation started");
        
        // Send initial status
        var status = new DroneStatus
        {
            Connected = true,
            GpsOk = true,
            EkfOk = true,
            CompassOk = true,
            BarometerOk = true,
            AccelerometerOk = true,
            GyroOk = true,
            RcOk = true,
            BatteryOk = true
        };
        StatusChanged?.Invoke(status);
    }
    
    /// <summary>
    /// Disconnect from hardware or stop simulation.
    /// </summary>
    public async Task DisconnectAsync()
    {
        if (_flightController != null)
        {
            _flightController.TelemetryReceived -= OnTelemetryReceived;
            _flightController.StatusChanged -= OnStatusChanged;
            _flightController.LogReceived -= OnLogReceived;
            await _flightController.DisconnectAsync();
            _flightController = null;
        }
        
        if (_simCts != null)
        {
            _simCts.Cancel();
            if (_simTask != null)
            {
                try { await _simTask; } catch { }
            }
            _simCts.Dispose();
            _simCts = null;
        }
        
        if (_simulator != null)
        {
            _simulator.StateUpdated -= OnSimulatorStateUpdated;
            _simulator.LogMessage -= OnLogReceived;
            _simulator = null;
        }
        
        Mode = ConnectionMode.Disconnected;
        ConnectionStateChanged?.Invoke(Mode);
    }
    
    // Flight controller commands
    public async Task<bool> ArmAsync(CancellationToken ct = default)
    {
        if (Mode == ConnectionMode.Hardware && _flightController != null)
        {
            return await _flightController.ArmAsync(ct);
        }
        else if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.State.Armed = true;
            LogReceived?.Invoke("Armed (simulation)");
            return true;
        }
        return false;
    }
    
    public async Task<bool> DisarmAsync(CancellationToken ct = default)
    {
        if (Mode == ConnectionMode.Hardware && _flightController != null)
        {
            return await _flightController.DisarmAsync(ct);
        }
        else if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.State.Armed = false;
            LogReceived?.Invoke("Disarmed (simulation)");
            return true;
        }
        return false;
    }
    
    public async Task SetFlightModeAsync(FlightMode mode, CancellationToken ct = default)
    {
        if (Mode == ConnectionMode.Hardware && _flightController != null)
        {
            await _flightController.SetFlightModeAsync(mode, ct);
        }
        else if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.State.Mode = mode;
            LogReceived?.Invoke($"Mode set to {mode} (simulation)");
        }
    }
    
    public async Task TakeoffAsync(double altitudeMeters, CancellationToken ct = default)
    {
        if (Mode == ConnectionMode.Hardware && _flightController != null)
        {
            await _flightController.TakeoffAsync(altitudeMeters, ct);
        }
        else if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.State.Armed = true;
            _simulator.State.Mode = FlightMode.Guided;
            // Set target altitude for simulation
            LogReceived?.Invoke($"Takeoff to {altitudeMeters}m (simulation)");
        }
    }
    
    public async Task LandAsync(CancellationToken ct = default)
    {
        if (Mode == ConnectionMode.Hardware && _flightController != null)
        {
            await _flightController.LandAsync(ct);
        }
        else if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.State.Mode = FlightMode.Land;
            LogReceived?.Invoke("Landing (simulation)");
        }
    }
    
    public async Task ReturnToLaunchAsync(CancellationToken ct = default)
    {
        if (Mode == ConnectionMode.Hardware && _flightController != null)
        {
            await _flightController.ReturnToLaunchAsync(ct);
        }
        else if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.State.Mode = FlightMode.RTL;
            LogReceived?.Invoke("RTL (simulation)");
        }
    }
    
    /// <summary>
    /// Set motor/attitude commands for simulation.
    /// </summary>
    public void SetSimulationCommands(double roll, double pitch, double yaw, double throttle)
    {
        if (Mode == ConnectionMode.Simulation && _simulator != null)
        {
            _simulator.SetAttitudeCommand(roll, pitch, yaw, throttle);
        }
    }
    
    /// <summary>
    /// Get the underlying flight controller for advanced operations.
    /// </summary>
    public MavlinkFlightController? GetFlightController() => _flightController;
    
    /// <summary>
    /// Get the simulator for advanced operations.
    /// </summary>
    public DroneSimulator? GetSimulator() => _simulator;
    
    private async Task SimulationLoopAsync(CancellationToken ct)
    {
        const double dt = 0.02; // 50Hz
        var targetThrottle = 0.0;
        var altitude = 0.0;
        
        while (!ct.IsCancellationRequested)
        {
            try
            {
                if (_simulator != null)
                {
                    var state = _simulator.State;
                    
                    // Simple altitude hold controller for simulation
                    if (state.Armed && state.Mode != FlightMode.Disarmed)
                    {
                        double targetAlt = state.Mode switch
                        {
                            FlightMode.Land => 0,
                            FlightMode.RTL => 20, // Return at 20m
                            _ => altitude > 0 ? altitude : 10 // Default hover at 10m
                        };
                        
                        if (targetAlt > altitude) altitude = System.Math.Min(altitude + 0.5, targetAlt);
                        else if (targetAlt < altitude) altitude = System.Math.Max(altitude - 0.5, targetAlt);
                        
                        double altError = altitude - state.Altitude;
                        targetThrottle = 0.5 + altError * 0.1; // Simple P controller
                        targetThrottle = System.Math.Clamp(targetThrottle, 0.1, 0.9);
                        
                        // Add some gentle random movements
                        double roll = (Random.Shared.NextDouble() - 0.5) * 0.02;
                        double pitch = (Random.Shared.NextDouble() - 0.5) * 0.02;
                        
                        _simulator.SetAttitudeCommand(roll, pitch, 0, targetThrottle);
                    }
                    else
                    {
                        _simulator.SetAttitudeCommand(0, 0, 0, 0);
                        altitude = 0;
                    }
                    
                    _simulator.Step(dt);
                }
                
                await Task.Delay((int)(dt * 1000), ct);
            }
            catch (OperationCanceledException) { break; }
            catch { }
        }
    }
    
    private void OnTelemetryReceived(DroneTelemetry telemetry)
    {
        TelemetryReceived?.Invoke(telemetry);
    }
    
    private void OnStatusChanged(DroneStatus status)
    {
        StatusChanged?.Invoke(status);
    }
    
    private void OnLogReceived(string message)
    {
        LogReceived?.Invoke(message);
    }
    
    private void OnSimulatorStateUpdated(DroneState state)
    {
        // Convert DroneState to DroneTelemetry
        var telemetry = _simulator?.GetNoisyTelemetry() ?? new DroneTelemetry();
        TelemetryReceived?.Invoke(telemetry);
    }
    
    public void Dispose()
    {
        DisconnectAsync().Wait();
    }
}

public enum ConnectionMode
{
    Disconnected,
    Hardware,
    Simulation
}
