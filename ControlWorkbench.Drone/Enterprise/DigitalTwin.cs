using MathNet.Numerics.LinearAlgebra;
using System.Collections.Concurrent;

namespace ControlWorkbench.Drone.Enterprise;

/// <summary>
/// Real-Time Digital Twin for Drone Systems.
/// Provides synchronized virtual replica with predictive capabilities.
/// 
/// Features:
/// - Real-time state synchronization with physical vehicle
/// - High-fidelity physics simulation
/// - What-if scenario analysis
/// - Predictive trajectory visualization
/// - Hardware-in-the-loop testing support
/// 
/// Based on:
/// - "Digital Twin Technology" (Tao et al., 2019)
/// - "Cyber-Physical Systems for Autonomous Vehicles" (Lee, 2020)
/// </summary>
public class DigitalTwin : IDisposable
{
    private readonly DigitalTwinConfig _config;
    private readonly PhysicsEngine _physics;
    private readonly StateEstimator _stateEstimator;
    private readonly ModelUpdater _modelUpdater;
    private readonly ScenarioSimulator _scenarioSimulator;
    
    private DigitalTwinState _currentState;
    private readonly ConcurrentQueue<StateSyncMessage> _syncQueue = new();
    private CancellationTokenSource? _cts;
    private Task? _updateTask;
    
    private bool _synchronized;
    private DateTime _lastSyncTime;
    private double _syncLatencyMs;
    
    public event Action<DigitalTwinState>? StateUpdated;
    public event Action<PredictedTrajectory>? TrajectoryPredicted;
    public event Action<ModelDeviation>? ModelDeviationDetected;
    
    public DigitalTwinState State => _currentState;
    public bool IsSynchronized => _synchronized;
    public double SyncLatencyMs => _syncLatencyMs;
    
    public DigitalTwin(DigitalTwinConfig config)
    {
        _config = config;
        _physics = new PhysicsEngine(config.PhysicsParams);
        _stateEstimator = new StateEstimator(config);
        _modelUpdater = new ModelUpdater(config);
        _scenarioSimulator = new ScenarioSimulator(_physics);
        
        _currentState = new DigitalTwinState();
    }
    
    /// <summary>
    /// Start the digital twin update loop.
    /// </summary>
    public void Start()
    {
        _cts = new CancellationTokenSource();
        _updateTask = Task.Run(() => UpdateLoop(_cts.Token), _cts.Token);
    }
    
    /// <summary>
    /// Stop the digital twin.
    /// </summary>
    public async Task StopAsync()
    {
        _cts?.Cancel();
        if (_updateTask != null)
        {
            try { await _updateTask; } catch { }
        }
    }
    
    /// <summary>
    /// Synchronize with real vehicle telemetry.
    /// </summary>
    public void Sync(RealVehicleTelemetry telemetry)
    {
        var syncMsg = new StateSyncMessage
        {
            Timestamp = telemetry.Timestamp,
            ReceivedAt = DateTime.UtcNow,
            Position = telemetry.Position,
            Velocity = telemetry.Velocity,
            Orientation = telemetry.Orientation,
            AngularVelocity = telemetry.AngularVelocity,
            MotorCommands = telemetry.MotorCommands,
            BatteryState = telemetry.BatteryState
        };
        
        _syncQueue.Enqueue(syncMsg);
        
        // Update sync metrics
        _syncLatencyMs = (syncMsg.ReceivedAt - syncMsg.Timestamp).TotalMilliseconds;
        _lastSyncTime = DateTime.UtcNow;
        _synchronized = true;
    }
    
    /// <summary>
    /// Predict trajectory for given control sequence.
    /// </summary>
    public PredictedTrajectory PredictTrajectory(double[] controlSequence, double dt, int steps)
    {
        var trajectory = new PredictedTrajectory();
        
        var state = _currentState.Clone();
        
        for (int i = 0; i < steps; i++)
        {
            // Get control for this step
            int cmdIdx = System.Math.Min(i * 4, controlSequence.Length - 4);
            var control = new double[] {
                controlSequence[cmdIdx],
                controlSequence[cmdIdx + 1],
                controlSequence[cmdIdx + 2],
                controlSequence[cmdIdx + 3]
            };
            
            // Simulate one step
            var nextState = _physics.Step(state, control, dt);
            
            trajectory.Positions.Add(nextState.Position.Clone());
            trajectory.Velocities.Add(nextState.Velocity.Clone());
            trajectory.Timestamps.Add(state.Time + dt);
            
            state = nextState;
        }
        
        trajectory.FinalState = state;
        TrajectoryPredicted?.Invoke(trajectory);
        
        return trajectory;
    }
    
    /// <summary>
    /// Run what-if scenario simulation.
    /// </summary>
    public ScenarioResult RunScenario(Scenario scenario)
    {
        return _scenarioSimulator.Simulate(scenario, _currentState);
    }
    
    /// <summary>
    /// Run multiple scenarios in parallel.
    /// </summary>
    public async Task<List<ScenarioResult>> RunScenariosAsync(
        IEnumerable<Scenario> scenarios,
        CancellationToken ct = default)
    {
        var tasks = scenarios.Select(s => Task.Run(() => 
            _scenarioSimulator.Simulate(s, _currentState), ct));
        
        return (await Task.WhenAll(tasks)).ToList();
    }
    
    /// <summary>
    /// Get model fidelity metrics.
    /// </summary>
    public ModelFidelityMetrics GetFidelityMetrics()
    {
        return _modelUpdater.GetMetrics();
    }
    
    /// <summary>
    /// Enable Hardware-in-the-Loop mode.
    /// </summary>
    public void EnableHilMode(IHilInterface hilInterface)
    {
        _physics.SetHilMode(hilInterface);
    }
    
    private async Task UpdateLoop(CancellationToken ct)
    {
        var lastUpdate = DateTime.UtcNow;
        
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var now = DateTime.UtcNow;
                double dt = (now - lastUpdate).TotalSeconds;
                lastUpdate = now;
                
                // Process sync messages
                while (_syncQueue.TryDequeue(out var syncMsg))
                {
                    ProcessSyncMessage(syncMsg, dt);
                }
                
                // Free-run physics if no sync
                if ((now - _lastSyncTime).TotalMilliseconds > 100)
                {
                    _currentState = _physics.Step(_currentState, _currentState.MotorCommands, dt);
                    _synchronized = false;
                }
                
                StateUpdated?.Invoke(_currentState);
                
                await Task.Delay(TimeSpan.FromMilliseconds(_config.UpdateIntervalMs), ct);
            }
            catch (OperationCanceledException) { break; }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Digital twin error: {ex.Message}");
            }
        }
    }
    
    private void ProcessSyncMessage(StateSyncMessage msg, double dt)
    {
        // Compare predicted state with actual
        var predicted = _currentState;
        var actual = msg;
        
        // Compute deviation
        double posError = (predicted.Position - actual.Position).L2Norm();
        double velError = (predicted.Velocity - actual.Velocity).L2Norm();
        
        if (posError > _config.DeviationThreshold || velError > _config.DeviationThreshold)
        {
            ModelDeviationDetected?.Invoke(new ModelDeviation
            {
                Timestamp = msg.Timestamp,
                PositionError = posError,
                VelocityError = velError,
                Description = $"Model deviation: pos={posError:F3}m, vel={velError:F3}m/s"
            });
            
            // Update model parameters
            _modelUpdater.Update(predicted, msg);
        }
        
        // Fuse predicted and actual states
        _currentState = _stateEstimator.Fuse(predicted, msg, dt);
        _currentState.MotorCommands = actual.MotorCommands;
        _currentState.BatteryVoltage = actual.BatteryState.Voltage;
        _currentState.BatteryPercent = actual.BatteryState.Percent;
    }
    
    public void Dispose()
    {
        _cts?.Cancel();
    }
}

/// <summary>
/// High-fidelity physics engine for digital twin.
/// </summary>
public class PhysicsEngine
{
    private readonly PhysicsParams _params;
    private IHilInterface? _hilInterface;
    
    public PhysicsEngine(PhysicsParams parameters)
    {
        _params = parameters;
    }
    
    public void SetHilMode(IHilInterface hilInterface)
    {
        _hilInterface = hilInterface;
    }
    
    public DigitalTwinState Step(DigitalTwinState state, double[] motorCommands, double dt)
    {
        if (_hilInterface != null)
        {
            return StepHil(state, motorCommands, dt);
        }
        
        return StepPhysics(state, motorCommands, dt);
    }
    
    private DigitalTwinState StepPhysics(DigitalTwinState state, double[] motorCommands, double dt)
    {
        var newState = state.Clone();
        
        // Motor dynamics
        var motorSpeeds = new double[4];
        for (int i = 0; i < 4; i++)
        {
            double targetSpeed = motorCommands[i] * _params.MaxMotorSpeed;
            motorSpeeds[i] = state.MotorSpeeds[i] + (targetSpeed - state.MotorSpeeds[i]) * (1 - System.Math.Exp(-dt / _params.MotorTimeConstant));
        }
        newState.MotorSpeeds = motorSpeeds;
        
        // Compute forces and moments
        var (thrust, moments) = ComputeThrustAndMoments(motorSpeeds, state);
        
        // Rotation matrix from body to world
        var R = QuaternionToMatrix(state.Orientation);
        
        // Gravity in world frame
        var gravity = Vector<double>.Build.DenseOfArray([0, 0, _params.Mass * 9.81]);
        
        // Thrust in world frame
        var thrustWorld = R * thrust;
        
        // Total force
        var force = thrustWorld - gravity;
        
        // Add aerodynamic drag
        double speed = state.Velocity.L2Norm();
        if (speed > 0.1)
        {
            var drag = -0.5 * _params.DragCoefficient * _params.AirDensity * speed * state.Velocity;
            force += drag;
        }
        
        // Linear acceleration
        var accel = force / _params.Mass;
        
        // Angular acceleration using Euler's equations
        var omega = state.AngularVelocity;
        var I = _params.Inertia;
        var omegaCross = CrossProduct(omega, I * omega);
        var angAccel = I.Inverse() * (moments - omegaCross);
        
        // Integration (RK4 would be better)
        newState.Velocity = state.Velocity + accel * dt;
        newState.Position = state.Position + state.Velocity * dt + 0.5 * accel * dt * dt;
        
        newState.AngularVelocity = state.AngularVelocity + angAccel * dt;
        newState.Orientation = IntegrateQuaternion(state.Orientation, state.AngularVelocity, dt);
        
        // Ground collision
        if (newState.Position[2] > 0)
        {
            newState.Position[2] = 0;
            newState.Velocity[2] = System.Math.Min(0, newState.Velocity[2]);
        }
        
        newState.Time = state.Time + dt;
        
        return newState;
    }
    
    private DigitalTwinState StepHil(DigitalTwinState state, double[] motorCommands, double dt)
    {
        // Send commands to hardware
        _hilInterface!.SendMotorCommands(motorCommands);
        
        // Wait for response
        var sensorData = _hilInterface.ReadSensorData();
        
        var newState = state.Clone();
        newState.Position = sensorData.Position;
        newState.Velocity = sensorData.Velocity;
        newState.Orientation = sensorData.Orientation;
        newState.AngularVelocity = sensorData.AngularVelocity;
        newState.Time = state.Time + dt;
        
        return newState;
    }
    
    private (Vector<double> thrust, Vector<double> moments) ComputeThrustAndMoments(double[] speeds, DigitalTwinState state)
    {
        var thrust = Vector<double>.Build.Dense(3);
        var moments = Vector<double>.Build.Dense(3);
        
        // Quad-X configuration
        double kf = _params.ThrustCoefficient;
        double km = _params.TorqueCoefficient;
        double L = _params.ArmLength;
        
        for (int i = 0; i < 4; i++)
        {
            double f = kf * speeds[i] * speeds[i];
            double m = km * speeds[i] * speeds[i];
            
            // Thrust along -z body axis
            thrust[2] -= f;
            
            // Moments
            double angle = System.Math.PI / 4 + i * System.Math.PI / 2;
            moments[0] += f * L * System.Math.Sin(angle);  // Roll
            moments[1] += f * L * System.Math.Cos(angle);  // Pitch
            moments[2] += (i % 2 == 0 ? 1 : -1) * m;       // Yaw
        }
        
        return (thrust, moments);
    }
    
    private Matrix<double> QuaternionToMatrix(Quaternion q)
    {
        double xx = q.X * q.X, yy = q.Y * q.Y, zz = q.Z * q.Z;
        double xy = q.X * q.Y, xz = q.X * q.Z, yz = q.Y * q.Z;
        double wx = q.W * q.X, wy = q.W * q.Y, wz = q.W * q.Z;
        
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy) },
            { 2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx) },
            { 2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy) }
        });
    }
    
    private Quaternion IntegrateQuaternion(Quaternion q, Vector<double> omega, double dt)
    {
        var omegaQuat = new Quaternion(0, omega[0], omega[1], omega[2]);
        var qDot = q * omegaQuat * 0.5;
        
        return new Quaternion(
            q.W + qDot.W * dt,
            q.X + qDot.X * dt,
            q.Y + qDot.Y * dt,
            q.Z + qDot.Z * dt
        ).Normalized();
    }
    
    private static Vector<double> CrossProduct(Vector<double> a, Vector<double> b)
    {
        return Vector<double>.Build.DenseOfArray([
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ]);
    }
}

/// <summary>
/// State estimator for fusing predicted and measured states.
/// </summary>
public class StateEstimator
{
    private readonly DigitalTwinConfig _config;
    
    public StateEstimator(DigitalTwinConfig config)
    {
        _config = config;
    }
    
    public DigitalTwinState Fuse(DigitalTwinState predicted, StateSyncMessage actual, double dt)
    {
        var fused = predicted.Clone();
        
        // Complementary filter for state fusion
        double alpha = 0.9; // Trust actual more
        
        fused.Position = alpha * actual.Position + (1 - alpha) * predicted.Position;
        fused.Velocity = alpha * actual.Velocity + (1 - alpha) * predicted.Velocity;
        fused.Orientation = SlerpQuaternion(predicted.Orientation, actual.Orientation, alpha);
        fused.AngularVelocity = alpha * actual.AngularVelocity + (1 - alpha) * predicted.AngularVelocity;
        
        return fused;
    }
    
    private Quaternion SlerpQuaternion(Quaternion a, Quaternion b, double t)
    {
        double dot = a.W * b.W + a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        
        if (dot < 0)
        {
            b = new Quaternion(-b.W, -b.X, -b.Y, -b.Z);
            dot = -dot;
        }
        
        if (dot > 0.9995)
        {
            return new Quaternion(
                a.W + t * (b.W - a.W),
                a.X + t * (b.X - a.X),
                a.Y + t * (b.Y - a.Y),
                a.Z + t * (b.Z - a.Z)
            ).Normalized();
        }
        
        double theta = System.Math.Acos(dot);
        double sinTheta = System.Math.Sin(theta);
        double wa = System.Math.Sin((1 - t) * theta) / sinTheta;
        double wb = System.Math.Sin(t * theta) / sinTheta;
        
        return new Quaternion(
            wa * a.W + wb * b.W,
            wa * a.X + wb * b.X,
            wa * a.Y + wb * b.Y,
            wa * a.Z + wb * b.Z
        );
    }
}

/// <summary>
/// Online model parameter updater.
/// </summary>
public class ModelUpdater
{
    private readonly DigitalTwinConfig _config;
    private readonly Queue<(DigitalTwinState predicted, StateSyncMessage actual)> _history = new();
    private ModelFidelityMetrics _metrics = new();
    
    public ModelUpdater(DigitalTwinConfig config)
    {
        _config = config;
    }
    
    public void Update(DigitalTwinState predicted, StateSyncMessage actual)
    {
        _history.Enqueue((predicted, actual));
        while (_history.Count > 1000)
            _history.Dequeue();
        
        // Update metrics
        _metrics.SampleCount++;
        
        double posError = (predicted.Position - actual.Position).L2Norm();
        double velError = (predicted.Velocity - actual.Velocity).L2Norm();
        
        _metrics.MeanPositionError = (_metrics.MeanPositionError * (_metrics.SampleCount - 1) + posError) / _metrics.SampleCount;
        _metrics.MeanVelocityError = (_metrics.MeanVelocityError * (_metrics.SampleCount - 1) + velError) / _metrics.SampleCount;
        _metrics.MaxPositionError = System.Math.Max(_metrics.MaxPositionError, posError);
        _metrics.MaxVelocityError = System.Math.Max(_metrics.MaxVelocityError, velError);
        
        // Could implement adaptive parameter estimation here
    }
    
    public ModelFidelityMetrics GetMetrics() => _metrics;
}

/// <summary>
/// Scenario simulator for what-if analysis.
/// </summary>
public class ScenarioSimulator
{
    private readonly PhysicsEngine _physics;
    
    public ScenarioSimulator(PhysicsEngine physics)
    {
        _physics = physics;
    }
    
    public ScenarioResult Simulate(Scenario scenario, DigitalTwinState initialState)
    {
        var result = new ScenarioResult
        {
            ScenarioName = scenario.Name,
            StartTime = DateTime.UtcNow
        };
        
        var state = initialState.Clone();
        
        foreach (var step in scenario.Steps)
        {
            switch (step.Type)
            {
                case ScenarioStepType.SetControl:
                    state.MotorCommands = step.MotorCommands!;
                    break;
                
                case ScenarioStepType.Wait:
                    int substeps = (int)(step.Duration / scenario.TimeStep);
                    for (int i = 0; i < substeps; i++)
                    {
                        state = _physics.Step(state, state.MotorCommands, scenario.TimeStep);
                        result.Trajectory.Add(state.Clone());
                    }
                    break;
                
                case ScenarioStepType.InjectFault:
                    ApplyFault(state, step.FaultType!, step.FaultSeverity);
                    break;
                
                case ScenarioStepType.SetWind:
                    // Would modify physics engine wind model
                    break;
            }
        }
        
        result.EndTime = DateTime.UtcNow;
        result.FinalState = state;
        result.Success = EvaluateSuccess(scenario, result);
        
        return result;
    }
    
    private void ApplyFault(DigitalTwinState state, string faultType, double severity)
    {
        switch (faultType)
        {
            case "MotorFailure":
                int motor = (int)(severity * 4) % 4;
                state.MotorSpeeds[motor] *= (1 - severity);
                break;
            
            case "BatteryDegradation":
                state.BatteryVoltage *= (1 - severity * 0.2);
                break;
        }
    }
    
    private bool EvaluateSuccess(Scenario scenario, ScenarioResult result)
    {
        foreach (var criterion in scenario.SuccessCriteria)
        {
            switch (criterion.Type)
            {
                case SuccessCriterionType.ReachPosition:
                    var target = criterion.TargetPosition!;
                    var final = result.FinalState!.Position;
                    double distance = (final - target).L2Norm();
                    if (distance > criterion.Tolerance)
                        return false;
                    break;
                
                case SuccessCriterionType.MaintainAltitude:
                    if (result.Trajectory.Any(s => System.Math.Abs(s.Position[2] - criterion.TargetAltitude) > criterion.Tolerance))
                        return false;
                    break;
                
                case SuccessCriterionType.AvoidCollision:
                    // Check for ground collision
                    if (result.Trajectory.Any(s => s.Position[2] > 0))
                        return false;
                    break;
            }
        }
        
        return true;
    }
}

/// <summary>
/// Interface for Hardware-in-the-Loop testing.
/// </summary>
public interface IHilInterface
{
    void SendMotorCommands(double[] commands);
    HilSensorData ReadSensorData();
}

public class HilSensorData
{
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
}

// Data types

public class DigitalTwinConfig
{
    public PhysicsParams PhysicsParams { get; set; } = new();
    public double UpdateIntervalMs { get; set; } = 10;
    public double DeviationThreshold { get; set; } = 0.5;
}

public class PhysicsParams
{
    public double Mass { get; set; } = 1.5;
    public Matrix<double> Inertia { get; set; } = Matrix<double>.Build.DenseOfDiagonalArray([0.029, 0.029, 0.055]);
    public double ArmLength { get; set; } = 0.25;
    public double ThrustCoefficient { get; set; } = 1e-5;
    public double TorqueCoefficient { get; set; } = 1e-7;
    public double MaxMotorSpeed { get; set; } = 1500;
    public double MotorTimeConstant { get; set; } = 0.02;
    public double DragCoefficient { get; set; } = 0.5;
    public double AirDensity { get; set; } = 1.225;
}

public class DigitalTwinState
{
    public double Time { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public double[] MotorSpeeds { get; set; } = new double[4];
    public double[] MotorCommands { get; set; } = new double[4];
    public double BatteryVoltage { get; set; } = 16.8;
    public double BatteryPercent { get; set; } = 100;
    
    public DigitalTwinState Clone()
    {
        return new DigitalTwinState
        {
            Time = Time,
            Position = Position.Clone(),
            Velocity = Velocity.Clone(),
            Orientation = Orientation,
            AngularVelocity = AngularVelocity.Clone(),
            MotorSpeeds = (double[])MotorSpeeds.Clone(),
            MotorCommands = (double[])MotorCommands.Clone(),
            BatteryVoltage = BatteryVoltage,
            BatteryPercent = BatteryPercent
        };
    }
}

public class Quaternion
{
    public double W { get; }
    public double X { get; }
    public double Y { get; }
    public double Z { get; }
    
    public Quaternion(double w, double x, double y, double z)
    {
        W = w; X = x; Y = y; Z = z;
    }
    
    public static Quaternion Identity => new(1, 0, 0, 0);
    
    public Quaternion Normalized()
    {
        double n = System.Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
        return new Quaternion(W / n, X / n, Y / n, Z / n);
    }
    
    public static Quaternion operator *(Quaternion a, Quaternion b)
    {
        return new Quaternion(
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W
        );
    }
    
    public static Quaternion operator *(Quaternion q, double s) => new(q.W * s, q.X * s, q.Y * s, q.Z * s);
}

public class StateSyncMessage
{
    public DateTime Timestamp { get; set; }
    public DateTime ReceivedAt { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public double[] MotorCommands { get; set; } = new double[4];
    public BatteryState BatteryState { get; set; } = new();
}

public class RealVehicleTelemetry
{
    public DateTime Timestamp { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public double[] MotorCommands { get; set; } = new double[4];
    public BatteryState BatteryState { get; set; } = new();
}

public class BatteryState
{
    public double Voltage { get; set; } = 16.8;
    public double Current { get; set; }
    public double Percent { get; set; } = 100;
    public double Temperature { get; set; } = 25;
}

public class PredictedTrajectory
{
    public List<Vector<double>> Positions { get; set; } = new();
    public List<Vector<double>> Velocities { get; set; } = new();
    public List<double> Timestamps { get; set; } = new();
    public DigitalTwinState? FinalState { get; set; }
}

public class ModelDeviation
{
    public DateTime Timestamp { get; set; }
    public double PositionError { get; set; }
    public double VelocityError { get; set; }
    public string Description { get; set; } = "";
}

public class ModelFidelityMetrics
{
    public int SampleCount { get; set; }
    public double MeanPositionError { get; set; }
    public double MeanVelocityError { get; set; }
    public double MaxPositionError { get; set; }
    public double MaxVelocityError { get; set; }
}

public class Scenario
{
    public string Name { get; set; } = "";
    public double TimeStep { get; set; } = 0.01;
    public List<ScenarioStep> Steps { get; set; } = new();
    public List<SuccessCriterion> SuccessCriteria { get; set; } = new();
}

public class ScenarioStep
{
    public ScenarioStepType Type { get; set; }
    public double Duration { get; set; }
    public double[]? MotorCommands { get; set; }
    public string? FaultType { get; set; }
    public double FaultSeverity { get; set; }
    public Vector<double>? WindVelocity { get; set; }
}

public enum ScenarioStepType
{
    SetControl,
    Wait,
    InjectFault,
    SetWind
}

public class ScenarioResult
{
    public string ScenarioName { get; set; } = "";
    public DateTime StartTime { get; set; }
    public DateTime EndTime { get; set; }
    public bool Success { get; set; }
    public List<DigitalTwinState> Trajectory { get; set; } = new();
    public DigitalTwinState? FinalState { get; set; }
}

public class SuccessCriterion
{
    public SuccessCriterionType Type { get; set; }
    public Vector<double>? TargetPosition { get; set; }
    public double TargetAltitude { get; set; }
    public double Tolerance { get; set; } = 0.5;
}

public enum SuccessCriterionType
{
    ReachPosition,
    MaintainAltitude,
    AvoidCollision,
    MaintainAttitude
}
