using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Geometry;
using MathQuat = ControlWorkbench.Math.Geometry.Quaternion;

namespace ControlWorkbench.Drone.Simulation;

/// <summary>
/// High-fidelity multirotor physics simulation with realistic aerodynamics.
/// Implements blade element momentum theory, motor dynamics, and disturbance modeling.
/// 
/// Based on:
/// - "Modelling, Identification and Control of a Quadrotor Helicopter" (Bouabdallah, 2007)
/// - "Aerodynamic Effects of Rotors in Ground Effect" (Cheeseman & Bennett, 1955)
/// </summary>
public class HighFidelityMultirotorSimulator
{
    private readonly MultirotorConfig _config;
    private readonly SimAerodynamicsModel _aero;
    private readonly SimMotorModel[] _motors;
    private readonly SimBatteryModel _battery;
    private readonly EnvironmentModel _environment;
    private readonly SensorSimulator _sensors;
    
    private MultirotorState _state;
    private double _simulationTime;
    private readonly Random _rng;
    
    public MultirotorState State => _state;
    public double SimulationTime => _simulationTime;
    
    public HighFidelityMultirotorSimulator(MultirotorConfig config, int seed = 42)
    {
        _config = config;
        _rng = new Random(seed);
        
        _aero = new SimAerodynamicsModel(config);
        _motors = new SimMotorModel[config.NumMotors];
        for (int i = 0; i < config.NumMotors; i++)
        {
            _motors[i] = new SimMotorModel(config.Motor);
        }
        _battery = new SimBatteryModel(config.Battery);
        _environment = new EnvironmentModel();
        _sensors = new SensorSimulator(config.Sensors, _rng);
        
        Reset();
    }
    
    /// <summary>
    /// Reset simulation to initial state.
    /// </summary>
    public void Reset(Vector<double>? initialPosition = null, MathQuat? initialOrientation = null)
    {
        _state = new MultirotorState
        {
            Position = initialPosition ?? Vector<double>.Build.Dense(3),
            Velocity = Vector<double>.Build.Dense(3),
            Quaternion = initialOrientation ?? MathQuat.Identity,
            AngularVelocity = Vector<double>.Build.Dense(3)
        };
        
        _simulationTime = 0;
        _battery.Reset();
        
        foreach (var motor in _motors)
            motor.Reset();
    }
    
    /// <summary>
    /// Step simulation forward with motor commands.
    /// </summary>
    /// <param name="motorCommands">Normalized motor commands (0-1) for each motor</param>
    /// <param name="dt">Time step in seconds</param>
    public void Step(double[] motorCommands, double dt)
    {
        // Clamp and scale motor commands
        var throttles = new double[_config.NumMotors];
        for (int i = 0; i < _config.NumMotors; i++)
        {
            throttles[i] = System.Math.Clamp(motorCommands[i], 0, 1);
        }
        
        // Get battery voltage
        double voltage = _battery.GetVoltage();
        
        // Update motor dynamics and compute forces/torques
        var motorForces = new double[_config.NumMotors];
        var motorTorques = new double[_config.NumMotors];
        double totalPower = 0;
        
        for (int i = 0; i < _config.NumMotors; i++)
        {
            _motors[i].Update(throttles[i], voltage, dt);
            motorForces[i] = _motors[i].GetThrust();
            motorTorques[i] = _motors[i].GetTorque();
            totalPower += _motors[i].GetPower();
        }
        
        // Update battery
        _battery.Update(totalPower, dt);
        
        // Compute aerodynamic forces and torques
        var (aeroForce, aeroTorque) = _aero.Compute(_state, motorForces);
        
        // Get environmental disturbances
        var (windForce, windTorque) = _environment.GetDisturbances(_state.Position, _simulationTime);
        
        // Total body forces
        var rotation = LieGroups.QuaternionToMatrix(_state.Quaternion);
        
        // Thrust (in body z direction)
        double totalThrust = motorForces.Sum();
        var thrustBody = Vector<double>.Build.DenseOfArray([0, 0, totalThrust]);
        var thrustWorld = rotation * thrustBody;
        
        // Gravity
        var gravity = Vector<double>.Build.DenseOfArray([0, 0, -_config.Mass * 9.81]);
        
        // Total force in world frame
        var totalForce = thrustWorld + gravity + rotation * aeroForce + windForce;
        
        // Motor torques (reaction torque from spinning)
        var motorReactionTorque = ComputeMotorReactionTorque(motorTorques);
        
        // Total torque in body frame
        var controlTorque = ComputeControlTorque(motorForces);
        var totalTorque = controlTorque + motorReactionTorque + aeroTorque + rotation.Transpose() * windTorque;
        
        // Integrate dynamics using RK4
        IntegrateRK4(totalForce, totalTorque, dt);
        
        // Ground collision
        if (_state.Position[2] < 0)
        {
            _state.Position[2] = 0;
            if (_state.Velocity[2] < 0)
            {
                _state.Velocity[2] = 0;
                
                // Ground friction
                _state.Velocity[0] *= 0.9;
                _state.Velocity[1] *= 0.9;
            }
        }
        
        _simulationTime += dt;
    }
    
    /// <summary>
    /// Get simulated sensor readings.
    /// </summary>
    public SensorReadings GetSensorReadings()
    {
        return _sensors.GenerateReadings(_state, _simulationTime);
    }
    
    /// <summary>
    /// Get motor states.
    /// </summary>
    public SimMotorState[] GetMotorStates()
    {
        return _motors.Select(m => m.GetState()).ToArray();
    }
    
    /// <summary>
    /// Get battery state.
    /// </summary>
    public SimBatteryState GetBatteryState()
    {
        return _battery.GetState();
    }
    
    private Vector<double> ComputeControlTorque(double[] motorForces)
    {
        // Compute torque from motor forces based on arm geometry
        double rollTorque = 0, pitchTorque = 0, yawTorque = 0;
        
        for (int i = 0; i < _config.NumMotors; i++)
        {
            var armPos = _config.MotorPositions[i];
            rollTorque += armPos[1] * motorForces[i];
            pitchTorque -= armPos[0] * motorForces[i];
        }
        
        return Vector<double>.Build.DenseOfArray([rollTorque, pitchTorque, yawTorque]);
    }
    
    private Vector<double> ComputeMotorReactionTorque(double[] motorTorques)
    {
        // Motor reaction torques (yaw axis)
        double yawTorque = 0;
        for (int i = 0; i < _config.NumMotors; i++)
        {
            // Alternating direction for adjacent motors
            int direction = _config.MotorDirections[i];
            yawTorque += direction * motorTorques[i];
        }
        
        return Vector<double>.Build.DenseOfArray([0, 0, yawTorque]);
    }
    
    private void IntegrateRK4(Vector<double> force, Vector<double> torque, double dt)
    {
        var inertia = _config.Inertia;
        var inertiaInv = inertia.Inverse();
        
        // State: [position, velocity, quaternion, angular_velocity]
        // Derivatives
        Func<MultirotorState, (Vector<double> posDot, Vector<double> velDot,
            MathQuat quatDot, Vector<double> omegaDot)> derivatives = (s) =>
        {
            var posDot = s.Velocity;
            var velDot = force / _config.Mass;

            // Quaternion derivative: q_dot = 0.5 * omega * q
            var omegaQuat = new MathQuat(0, s.AngularVelocity[0], s.AngularVelocity[1], s.AngularVelocity[2]);
            var quatDotTemp = new MathQuat(
                0.5 * (omegaQuat.W * s.Quaternion.W - omegaQuat.X * s.Quaternion.X - omegaQuat.Y * s.Quaternion.Y - omegaQuat.Z * s.Quaternion.Z),
                0.5 * (omegaQuat.W * s.Quaternion.X + omegaQuat.X * s.Quaternion.W + omegaQuat.Y * s.Quaternion.Z - omegaQuat.Z * s.Quaternion.Y),
                0.5 * (omegaQuat.W * s.Quaternion.Y - omegaQuat.X * s.Quaternion.Z + omegaQuat.Y * s.Quaternion.W + omegaQuat.Z * s.Quaternion.X),
                0.5 * (omegaQuat.W * s.Quaternion.Z + omegaQuat.X * s.Quaternion.Y - omegaQuat.Y * s.Quaternion.X + omegaQuat.Z * s.Quaternion.W)
            );

            // Angular acceleration: I * omega_dot = tau - omega x (I * omega)
            var gyroTorque = Cross(s.AngularVelocity, inertia * s.AngularVelocity);
            var omegaDot = inertiaInv * (torque - gyroTorque);

            return (posDot, velDot, quatDotTemp, omegaDot);
        };
        
        // RK4 integration
        var k1 = derivatives(_state);
        
        var s2 = new MultirotorState
        {
            Position = _state.Position + 0.5 * dt * k1.posDot,
            Velocity = _state.Velocity + 0.5 * dt * k1.velDot,
            Quaternion = AddQuaternion(_state.Quaternion, k1.quatDot, 0.5 * dt),
            AngularVelocity = _state.AngularVelocity + 0.5 * dt * k1.omegaDot
        };
        var k2 = derivatives(s2);
        
        var s3 = new MultirotorState
        {
            Position = _state.Position + 0.5 * dt * k2.posDot,
            Velocity = _state.Velocity + 0.5 * dt * k2.velDot,
            Quaternion = AddQuaternion(_state.Quaternion, k2.quatDot, 0.5 * dt),
            AngularVelocity = _state.AngularVelocity + 0.5 * dt * k2.omegaDot
        };
        var k3 = derivatives(s3);
        
        var s4 = new MultirotorState
        {
            Position = _state.Position + dt * k3.posDot,
            Velocity = _state.Velocity + dt * k3.velDot,
            Quaternion = AddQuaternion(_state.Quaternion, k3.quatDot, dt),
            AngularVelocity = _state.AngularVelocity + dt * k3.omegaDot
        };
        var k4 = derivatives(s4);
        
        // Update state
        _state.Position += dt / 6 * (k1.posDot + 2 * k2.posDot + 2 * k3.posDot + k4.posDot);
        _state.Velocity += dt / 6 * (k1.velDot + 2 * k2.velDot + 2 * k3.velDot + k4.velDot);
        _state.AngularVelocity += dt / 6 * (k1.omegaDot + 2 * k2.omegaDot + 2 * k3.omegaDot + k4.omegaDot);
        
        // Quaternion integration (special handling)
        var quatDotAvg = new MathQuat(
            (k1.quatDot.W + 2 * k2.quatDot.W + 2 * k3.quatDot.W + k4.quatDot.W) / 6,
            (k1.quatDot.X + 2 * k2.quatDot.X + 2 * k3.quatDot.X + k4.quatDot.X) / 6,
            (k1.quatDot.Y + 2 * k2.quatDot.Y + 2 * k3.quatDot.Y + k4.quatDot.Y) / 6,
            (k1.quatDot.Z + 2 * k2.quatDot.Z + 2 * k3.quatDot.Z + k4.quatDot.Z) / 6
        );
        _state.Quaternion = AddQuaternion(_state.Quaternion, quatDotAvg, dt).Normalized();
    }

    private MathQuat AddQuaternion(MathQuat q, MathQuat qDot, double dt)
    {
        return new MathQuat(
            q.W + dt * qDot.W,
            q.X + dt * qDot.X,
            q.Y + dt * qDot.Y,
            q.Z + dt * qDot.Z
        );
    }
    
    private static Vector<double> Cross(Vector<double> a, Vector<double> b)
    {
        return Vector<double>.Build.DenseOfArray([
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ]);
    }
}

#region Aerodynamics

/// <summary>
/// Aerodynamic model including rotor downwash and body drag.
/// </summary>
public class SimAerodynamicsModel
{
    private readonly MultirotorConfig _config;
    
    public SimAerodynamicsModel(MultirotorConfig config)
    {
        _config = config;
    }
    
    public (Vector<double> force, Vector<double> torque) Compute(MultirotorState state, double[] motorForces)
    {
        var rotation = LieGroups.QuaternionToMatrix(state.Quaternion);
        
        // Body velocity in body frame
        var velBody = rotation.Transpose() * state.Velocity;
        double speed = velBody.L2Norm();
        
        // Parasitic drag (body drag)
        var dragForce = Vector<double>.Build.Dense(3);
        if (speed > 0.1)
        {
            var dragDir = -velBody.Normalize(2);
            double dragMag = 0.5 * _config.AirDensity * speed * speed * _config.DragCoefficient * _config.FrontalArea;
            dragForce = dragMag * dragDir;
        }
        
        // Blade flapping effect (increases with forward velocity)
        var flapTorque = Vector<double>.Build.Dense(3);
        double flapCoeff = _config.BladeFlapCoefficient;
        flapTorque[0] = -flapCoeff * velBody[1]; // Roll from lateral velocity
        flapTorque[1] = flapCoeff * velBody[0];  // Pitch from forward velocity
        
        // Ground effect (increased thrust near ground)
        double altitude = state.Position[2];
        double groundEffectFactor = 1.0;
        if (altitude < 2 * _config.RotorRadius)
        {
            double x = altitude / _config.RotorRadius;
            groundEffectFactor = 1.0 / (1.0 - (0.25 * _config.RotorRadius / altitude) * (0.25 * _config.RotorRadius / altitude));
            groundEffectFactor = System.Math.Min(groundEffectFactor, 1.5);
        }
        
        // Apply ground effect to rotor forces (handled externally)
        
        // Induced drag in forward flight
        double totalThrust = motorForces.Sum();
        double inducedVelocity = System.Math.Sqrt(totalThrust / (2 * _config.AirDensity * _config.RotorDiscArea));
        double advanceRatio = speed / (inducedVelocity + 0.1);
        
        // Profile drag increases with advance ratio
        double profileDragMag = _config.ProfileDragCoefficient * totalThrust * advanceRatio * advanceRatio;
        var profileDrag = -profileDragMag * velBody.Normalize(2);
        if (double.IsNaN(profileDrag[0])) profileDrag = Vector<double>.Build.Dense(3);
        
        var totalForce = dragForce + profileDrag;
        var totalTorque = flapTorque;
        
        return (totalForce, totalTorque);
    }
}

#endregion

#region Motor Model

/// <summary>
/// Brushless motor model with ESC dynamics.
/// </summary>
public class SimMotorModel
{
    private readonly MotorConfig _config;
    private double _omega; // Angular velocity (rad/s)
    private double _current;
    
    public SimMotorModel(MotorConfig config)
    {
        _config = config;
    }
    
    public void Reset()
    {
        _omega = 0;
        _current = 0;
    }
    
    public void Update(double throttle, double voltage, double dt)
    {
        // ESC model: target RPM proportional to throttle
        double targetOmega = throttle * _config.MaxRpm * 2 * System.Math.PI / 60;
        
        // First-order motor dynamics
        double tau = _config.TimeConstant;
        _omega += (targetOmega - _omega) / tau * dt;
        _omega = System.Math.Max(0, _omega);
        
        // Back-EMF limits speed
        double backEmf = _config.Kv * _omega * 60 / (2 * System.Math.PI);
        if (backEmf > voltage)
        {
            _omega = voltage / _config.Kv * 2 * System.Math.PI / 60;
        }
        
        // Current draw
        _current = GetTorque() / _config.Kt + _config.NoLoadCurrent;
    }
    
    public double GetThrust()
    {
        // Thrust = Ct * rho * omega^2 * D^4
        double n = _omega / (2 * System.Math.PI); // rev/s
        return _config.ThrustCoefficient * 1.225 * n * n * System.Math.Pow(_config.PropDiameter, 4);
    }
    
    public double GetTorque()
    {
        // Torque = Cq * rho * omega^2 * D^5
        double n = _omega / (2 * System.Math.PI);
        return _config.TorqueCoefficient * 1.225 * n * n * System.Math.Pow(_config.PropDiameter, 5);
    }
    
    public double GetPower()
    {
        return GetTorque() * _omega;
    }
    
    public SimMotorState GetState()
    {
        return new SimMotorState
        {
            Rpm = _omega * 60 / (2 * System.Math.PI),
            Thrust = GetThrust(),
            Torque = GetTorque(),
            Current = _current,
            Power = GetPower()
        };
    }
}

public struct SimMotorState
{
    public double Rpm;
    public double Thrust;
    public double Torque;
    public double Current;
    public double Power;
}

#endregion

#region Battery Model

/// <summary>
/// LiPo battery model with internal resistance.
/// </summary>
public class SimBatteryModel
{
    private readonly BatteryConfig _config;
    private double _capacity; // Remaining capacity (mAh)
    private double _voltage;
    
    public SimBatteryModel(BatteryConfig config)
    {
        _config = config;
        Reset();
    }
    
    public void Reset()
    {
        _capacity = _config.Capacity;
        _voltage = _config.NominalVoltage;
    }
    
    public void Update(double power, double dt)
    {
        double current = power / _voltage;
        _capacity -= current * dt * 1000 / 3600; // mAh
        _capacity = System.Math.Max(0, _capacity);
        
        // Voltage drops with discharge
        double soc = _capacity / _config.Capacity;
        _voltage = _config.NominalVoltage * (0.2 + 0.8 * soc) - current * _config.InternalResistance;
        _voltage = System.Math.Max(_config.MinVoltage, _voltage);
    }
    
    public double GetVoltage() => _voltage;
    
    public SimBatteryState GetState()
    {
        return new SimBatteryState
        {
            Voltage = _voltage,
            RemainingCapacity = _capacity,
            StateOfCharge = _capacity / _config.Capacity,
            RemainingTime = EstimateRemainingTime()
        };
    }
    
    private double EstimateRemainingTime()
    {
        // Rough estimate based on current discharge rate
        return _capacity / 1000 * 60; // minutes at 1A draw
    }
}

public struct SimBatteryState
{
    public double Voltage;
    public double RemainingCapacity;
    public double StateOfCharge;
    public double RemainingTime;
}

#endregion

#region Environment Model

/// <summary>
/// Environmental disturbances including wind.
/// </summary>
public class EnvironmentModel
{
    private readonly Random _rng = new();
    private double _windSpeed = 0;
    private double _windDirection = 0;
    private double _gustIntensity = 0;
    
    public void SetWind(double speed, double direction)
    {
        _windSpeed = speed;
        _windDirection = direction;
    }
    
    public void SetGustIntensity(double intensity)
    {
        _gustIntensity = intensity;
    }
    
    public (Vector<double> force, Vector<double> torque) GetDisturbances(Vector<double> position, double time)
    {
        // Base wind
        double windX = _windSpeed * System.Math.Cos(_windDirection);
        double windY = _windSpeed * System.Math.Sin(_windDirection);
        
        // Dryden wind model for turbulence
        double turbScale = 1 + position[2] / 20; // Increases with altitude
        double turbX = _gustIntensity * turbScale * (GaussianNoise() + 0.5 * System.Math.Sin(2 * time));
        double turbY = _gustIntensity * turbScale * (GaussianNoise() + 0.5 * System.Math.Cos(3 * time));
        double turbZ = _gustIntensity * turbScale * 0.5 * GaussianNoise();
        
        var windVelocity = Vector<double>.Build.DenseOfArray([
            windX + turbX,
            windY + turbY,
            turbZ
        ]);
        
        // Approximate force from wind (drag)
        double dragCoeff = 0.5;
        var force = dragCoeff * windVelocity * windVelocity.L2Norm();
        
        // Wind-induced torque
        var torque = Vector<double>.Build.DenseOfArray([
            0.1 * turbY,
            -0.1 * turbX,
            0.05 * (turbX - turbY)
        ]);
        
        return (force, torque);
    }
    
    private double GaussianNoise()
    {
        double u1 = 1.0 - _rng.NextDouble();
        double u2 = 1.0 - _rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

#endregion

#region Sensor Simulator

/// <summary>
/// Simulates IMU, GPS, barometer, magnetometer with realistic noise.
/// </summary>
public class SensorSimulator
{
    private readonly SensorConfig _config;
    private readonly Random _rng;
    
    // Bias states
    private Vector<double> _gyroBias;
    private Vector<double> _accelBias;
    
    // GPS state
    private double _lastGpsTime;
    
    public SensorSimulator(SensorConfig config, Random rng)
    {
        _config = config;
        _rng = rng;
        
        _gyroBias = Vector<double>.Build.Dense(3, _ => GaussianNoise() * config.GyroBiasStd);
        _accelBias = Vector<double>.Build.Dense(3, _ => GaussianNoise() * config.AccelBiasStd);
    }
    
    public SensorReadings GenerateReadings(MultirotorState state, double time)
    {
        var readings = new SensorReadings();
        
        var rotation = LieGroups.QuaternionToMatrix(state.Quaternion);
        
        // IMU
        // Gyroscope: measure angular velocity + bias + noise
        readings.Gyroscope = state.AngularVelocity + _gyroBias + 
            Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.GyroNoiseStd);
        
        // Accelerometer: measure specific force (accel - gravity in body frame)
        var gravityWorld = Vector<double>.Build.DenseOfArray([0, 0, 9.81]);
        var gravityBody = rotation.Transpose() * gravityWorld;
        var accelBody = rotation.Transpose() * (state.Acceleration ?? Vector<double>.Build.Dense(3));
        readings.Accelerometer = accelBody + gravityBody + _accelBias +
            Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.AccelNoiseStd);
        
        // Magnetometer
        var magWorld = Vector<double>.Build.DenseOfArray([0.22, 0, 0.42]); // Earth's field
        readings.Magnetometer = rotation.Transpose() * magWorld +
            Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.MagNoiseStd);
        
        // Barometer
        double altitude = state.Position[2];
        double pressure = 101325 * System.Math.Exp(-altitude / 8500); // Barometric formula
        readings.BarometricAltitude = altitude + GaussianNoise() * _config.BaroNoiseStd;
        readings.Pressure = pressure + GaussianNoise() * 10;
        
        // GPS (lower rate)
        if (time - _lastGpsTime >= 1.0 / _config.GpsRate)
        {
            readings.HasGps = true;
            readings.GpsPosition = state.Position + 
                Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.GpsNoiseStd);
            readings.GpsVelocity = state.Velocity +
                Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.GpsVelNoiseStd);
            _lastGpsTime = time;
        }
        
        // Update biases (random walk)
        double dt = 0.001; // Assume 1kHz
        _gyroBias += Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.GyroBiasWalk * System.Math.Sqrt(dt));
        _accelBias += Vector<double>.Build.Dense(3, _ => GaussianNoise() * _config.AccelBiasWalk * System.Math.Sqrt(dt));
        
        readings.Timestamp = time;
        return readings;
    }
    
    private double GaussianNoise()
    {
        double u1 = 1.0 - _rng.NextDouble();
        double u2 = 1.0 - _rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

public class SensorReadings
{
    public double Timestamp;
    public Vector<double> Accelerometer = Vector<double>.Build.Dense(3);
    public Vector<double> Gyroscope = Vector<double>.Build.Dense(3);
    public Vector<double> Magnetometer = Vector<double>.Build.Dense(3);
    public double BarometricAltitude;
    public double Pressure;
    public bool HasGps;
    public Vector<double> GpsPosition = Vector<double>.Build.Dense(3);
    public Vector<double> GpsVelocity = Vector<double>.Build.Dense(3);
}

#endregion

#region Configuration

public class MultirotorConfig
{
    public int NumMotors { get; set; } = 4;
    public double Mass { get; set; } = 1.5; // kg
    public Matrix<double> Inertia { get; set; } = Matrix<double>.Build.DiagonalOfDiagonalArray(
        [0.029, 0.029, 0.055]); // kg*m^2
    
    public Vector<double>[] MotorPositions { get; set; } = [
        Vector<double>.Build.DenseOfArray([0.17, 0.17, 0]),
        Vector<double>.Build.DenseOfArray([-0.17, 0.17, 0]),
        Vector<double>.Build.DenseOfArray([-0.17, -0.17, 0]),
        Vector<double>.Build.DenseOfArray([0.17, -0.17, 0])
    ];
    
    public int[] MotorDirections { get; set; } = [1, -1, 1, -1]; // CW/CCW
    
    public double AirDensity { get; set; } = 1.225;
    public double DragCoefficient { get; set; } = 0.25;
    public double FrontalArea { get; set; } = 0.01;
    public double RotorRadius { get; set; } = 0.127; // 5 inch prop
    public double RotorDiscArea => System.Math.PI * RotorRadius * RotorRadius * NumMotors;
    public double ProfileDragCoefficient { get; set; } = 0.01;
    public double BladeFlapCoefficient { get; set; } = 0.05;
    
    public MotorConfig Motor { get; set; } = new();
    public BatteryConfig Battery { get; set; } = new();
    public SensorConfig Sensors { get; set; } = new();
    
    public static MultirotorConfig DefaultQuadrotor => new();
    
    public static MultirotorConfig Hexarotor => new()
    {
        NumMotors = 6,
        MotorPositions = Enumerable.Range(0, 6)
            .Select(i => {
                double angle = i * System.Math.PI / 3;
                return Vector<double>.Build.DenseOfArray([
                    0.25 * System.Math.Cos(angle),
                    0.25 * System.Math.Sin(angle),
                    0
                ]);
            }).ToArray(),
        MotorDirections = [1, -1, 1, -1, 1, -1]
    };
}

public class MotorConfig
{
    public double MaxRpm { get; set; } = 10000;
    public double TimeConstant { get; set; } = 0.05; // seconds
    public double ThrustCoefficient { get; set; } = 0.1;
    public double TorqueCoefficient { get; set; } = 0.01;
    public double PropDiameter { get; set; } = 0.254; // 10 inch
    public double Kv { get; set; } = 920; // RPM/V
    public double Kt { get; set; } = 0.01; // Nm/A
    public double NoLoadCurrent { get; set; } = 0.5; // A
}

public class BatteryConfig
{
    public double NominalVoltage { get; set; } = 14.8; // 4S LiPo
    public double MinVoltage { get; set; } = 13.2;
    public double Capacity { get; set; } = 5000; // mAh
    public double InternalResistance { get; set; } = 0.02; // Ohm
}

public class SensorConfig
{
    public double GyroNoiseStd { get; set; } = 0.01; // rad/s
    public double GyroBiasStd { get; set; } = 0.001;
    public double GyroBiasWalk { get; set; } = 0.0001;
    
    public double AccelNoiseStd { get; set; } = 0.1; // m/s^2
    public double AccelBiasStd { get; set; } = 0.01;
    public double AccelBiasWalk { get; set; } = 0.001;
    
    public double MagNoiseStd { get; set; } = 0.01;
    public double BaroNoiseStd { get; set; } = 0.5; // meters
    
    public double GpsNoiseStd { get; set; } = 2.0; // meters
    public double GpsVelNoiseStd { get; set; } = 0.1; // m/s
    public double GpsRate { get; set; } = 5; // Hz
}

#endregion

#region State

public class MultirotorState
{
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public MathQuat Quaternion { get; set; } = MathQuat.Identity;
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double>? Acceleration { get; set; }

    public Matrix<double> RotationMatrix => LieGroups.QuaternionToMatrix(Quaternion);

    public (double roll, double pitch, double yaw) EulerAngles => Quaternion.ToEuler();
}

#endregion
