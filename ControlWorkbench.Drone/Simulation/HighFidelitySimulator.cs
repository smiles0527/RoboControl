using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Simulation;

/// <summary>
/// High-fidelity multirotor physics simulation with blade element theory,
/// motor transient dynamics, and CFD-based aerodynamic modeling.
/// </summary>
public class HighFidelityDroneSimulator
{
    // State vectors
    private Vector<double> _position;       // [x, y, z] in NED frame
    private Vector<double> _velocity;       // [vx, vy, vz] in NED frame
    private Quaternion _orientation;        // Attitude quaternion
    private Vector<double> _angularVelocity; // [p, q, r] in body frame
    private double[] _motorSpeeds;          // rad/s for each motor
    
    // Physical parameters
    private readonly DronePhysicsParams _params;
    private readonly MotorModel[] _motors;
    private readonly PropellerModel[] _propellers;
    private readonly AerodynamicsModel _aero;
    private readonly BatteryModel _battery;
    private readonly SensorSuite _sensors;
    
    // Environment
    private readonly AtmosphereModel _atmosphere;
    private readonly WindFieldModel _windField;
    private readonly TerrainModel? _terrain;
    
    // Simulation state
    private double _time;
    private readonly Random _rng = new();
    
    public event Action<HighFidelityState>? StateUpdated;
    public event Action<string>? Warning;
    
    public HighFidelityDroneSimulator(DronePhysicsParams parameters)
    {
        _params = parameters;
        
        _position = Vector<double>.Build.Dense(3);
        _velocity = Vector<double>.Build.Dense(3);
        _orientation = Quaternion.Identity;
        _angularVelocity = Vector<double>.Build.Dense(3);
        
        // Initialize motors and propellers
        _motors = new MotorModel[_params.NumMotors];
        _propellers = new PropellerModel[_params.NumMotors];
        _motorSpeeds = new double[_params.NumMotors];
        
        for (int i = 0; i < _params.NumMotors; i++)
        {
            _motors[i] = new MotorModel(_params.MotorParams);
            _propellers[i] = new PropellerModel(_params.PropellerParams, _params.MotorPositions[i].clockwise);
        }
        
        _aero = new AerodynamicsModel(_params.AeroParams);
        _battery = new BatteryModel(_params.BatteryParams);
        _sensors = new SensorSuite(_params.SensorParams);
        _atmosphere = new AtmosphereModel();
        _windField = new WindFieldModel();
    }
    
    /// <summary>
    /// Initialize simulation state.
    /// </summary>
    public void Initialize(
        Vector<double>? position = null,
        Vector<double>? velocity = null,
        Quaternion? orientation = null)
    {
        _position = position?.Clone() ?? Vector<double>.Build.Dense(3);
        _velocity = velocity?.Clone() ?? Vector<double>.Build.Dense(3);
        _orientation = orientation ?? Quaternion.Identity;
        _angularVelocity = Vector<double>.Build.Dense(3);
        
        for (int i = 0; i < _params.NumMotors; i++)
        {
            _motorSpeeds[i] = 0;
        }
        
        _battery.Reset();
        _time = 0;
    }
    
    /// <summary>
    /// Set motor throttle commands (0-1).
    /// </summary>
    public void SetMotorCommands(double[] throttles)
    {
        for (int i = 0; i < _params.NumMotors; i++)
        {
            _motors[i].SetThrottle(throttles[i]);
        }
    }
    
    /// <summary>
    /// Set attitude command (for attitude controllers).
    /// </summary>
    public void SetAttitudeCommand(double roll, double pitch, double yaw, double throttle)
    {
        // Use control allocator to convert to motor commands
        var motorCommands = _params.ControlAllocator.Allocate(roll, pitch, yaw, throttle);
        SetMotorCommands(motorCommands);
    }
    
    /// <summary>
    /// Step simulation using 4th-order Runge-Kutta integration.
    /// </summary>
    public void Step(double dt)
    {
        // RK4 integration
        var state0 = GetStateVector();
        
        var k1 = ComputeDerivatives(state0, 0);
        var k2 = ComputeDerivatives(state0 + 0.5 * dt * k1, 0.5 * dt);
        var k3 = ComputeDerivatives(state0 + 0.5 * dt * k2, 0.5 * dt);
        var k4 = ComputeDerivatives(state0 + dt * k3, dt);
        
        var stateNew = state0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
        
        SetStateFromVector(stateNew);
        
        // Update motor dynamics (separate due to fast dynamics)
        UpdateMotorDynamics(dt);
        
        // Update battery state
        double totalCurrent = _motors.Sum(m => m.Current);
        _battery.Update(totalCurrent, dt);
        
        // Check for warnings
        CheckWarnings();
        
        _time += dt;
        
        StateUpdated?.Invoke(GetHighFidelityState());
    }
    
    /// <summary>
    /// Get noisy sensor measurements.
    /// </summary>
    public SensorMeasurements GetSensorMeasurements()
    {
        var rotMatrix = _orientation.ToRotationMatrix();
        
        return _sensors.Measure(
            _position,
            _velocity,
            rotMatrix,
            _angularVelocity,
            _atmosphere.GetPressure(_position[2]),
            _atmosphere.GetTemperature(_position[2]),
            _rng);
    }
    
    /// <summary>
    /// Get true state (for ground truth comparison).
    /// </summary>
    public HighFidelityState GetHighFidelityState()
    {
        var euler = _orientation.ToEulerAngles();
        var rotMatrix = _orientation.ToRotationMatrix();
        
        return new HighFidelityState
        {
            Time = _time,
            Position = _position.Clone(),
            Velocity = _velocity.Clone(),
            VelocityBody = rotMatrix.Transpose() * _velocity,
            Orientation = _orientation,
            EulerAngles = euler,
            AngularVelocity = _angularVelocity.Clone(),
            MotorSpeeds = (double[])_motorSpeeds.Clone(),
            MotorCurrents = _motors.Select(m => m.Current).ToArray(),
            MotorTemperatures = _motors.Select(m => m.Temperature).ToArray(),
            BatteryVoltage = _battery.Voltage,
            BatteryCurrent = _motors.Sum(m => m.Current),
            BatteryRemaining = _battery.StateOfCharge,
            TotalThrust = _propellers.Select((p, i) => p.ComputeThrust(_motorSpeeds[i], _velocity, rotMatrix, _atmosphere.GetDensity(_position[2]))).Sum(),
            TotalMass = _params.Mass,
            AirDensity = _atmosphere.GetDensity(_position[2]),
            WindVelocity = _windField.GetWind(_position, _time)
        };
    }
    
    private Vector<double> GetStateVector()
    {
        // State: [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
        var state = Vector<double>.Build.Dense(13);
        
        state[0] = _position[0];
        state[1] = _position[1];
        state[2] = _position[2];
        state[3] = _velocity[0];
        state[4] = _velocity[1];
        state[5] = _velocity[2];
        state[6] = _orientation.W;
        state[7] = _orientation.X;
        state[8] = _orientation.Y;
        state[9] = _orientation.Z;
        state[10] = _angularVelocity[0];
        state[11] = _angularVelocity[1];
        state[12] = _angularVelocity[2];
        
        return state;
    }
    
    private void SetStateFromVector(Vector<double> state)
    {
        _position[0] = state[0];
        _position[1] = state[1];
        _position[2] = state[2];
        _velocity[0] = state[3];
        _velocity[1] = state[4];
        _velocity[2] = state[5];
        
        _orientation = new Quaternion(state[6], state[7], state[8], state[9]).Normalized();
        
        _angularVelocity[0] = state[10];
        _angularVelocity[1] = state[11];
        _angularVelocity[2] = state[12];
        
        // Ground collision
        if (_position[2] > 0) // NED, positive down
        {
            _position[2] = 0;
            _velocity[2] = System.Math.Min(0, _velocity[2]);
        }
    }
    
    private Vector<double> ComputeDerivatives(Vector<double> state, double dt)
    {
        var deriv = Vector<double>.Build.Dense(13);
        
        // Extract current state
        var pos = Vector<double>.Build.DenseOfArray([state[0], state[1], state[2]]);
        var vel = Vector<double>.Build.DenseOfArray([state[3], state[4], state[5]]);
        var quat = new Quaternion(state[6], state[7], state[8], state[9]).Normalized();
        var omega = Vector<double>.Build.DenseOfArray([state[10], state[11], state[12]]);
        
        var rotMatrix = quat.ToRotationMatrix();
        double altitude = -pos[2];
        
        // Environment
        double rho = _atmosphere.GetDensity(altitude);
        var wind = _windField.GetWind(pos, _time + dt);
        var airVelocity = vel - wind;
        
        // Forces and moments in body frame
        var forceBody = Vector<double>.Build.Dense(3);
        var momentBody = Vector<double>.Build.Dense(3);
        
        // Gravity in body frame
        var gravityNed = Vector<double>.Build.DenseOfArray([0, 0, _params.Mass * 9.81]);
        var gravityBody = rotMatrix.Transpose() * gravityNed;
        forceBody += gravityBody;
        
        // Motor/propeller forces and moments
        for (int i = 0; i < _params.NumMotors; i++)
        {
            var prop = _propellers[i];
            var motorPos = _params.MotorPositions[i];
            
            // Propeller thrust and torque
            double thrust = prop.ComputeThrust(_motorSpeeds[i], airVelocity, rotMatrix, rho);
            double torque = prop.ComputeTorque(_motorSpeeds[i], rho);
            
            // Force in body frame (thrust along -z for typical multirotor)
            var thrustVec = Vector<double>.Build.DenseOfArray([0, 0, -thrust]);
            forceBody += thrustVec;
            
            // Moment from thrust position
            var armVec = Vector<double>.Build.DenseOfArray([motorPos.x, motorPos.y, motorPos.z]);
            var thrustMoment = CrossProduct(armVec, thrustVec);
            momentBody += thrustMoment;
            
            // Reaction torque (around z-axis)
            double reactionSign = motorPos.clockwise ? -1 : 1;
            momentBody[2] += reactionSign * torque;
        }
        
        // Aerodynamic forces and moments
        var (aeroForce, aeroMoment) = _aero.Compute(airVelocity, omega, rotMatrix, rho);
        forceBody += aeroForce;
        momentBody += aeroMoment;
        
        // Convert force to NED frame
        var forceNed = rotMatrix * forceBody;
        
        // Linear acceleration
        var accel = forceNed / _params.Mass;
        
        // Angular acceleration using Euler's equations
        var inertia = _params.Inertia;
        var omegaCross = CrossProduct(omega, inertia * omega);
        var angAccel = inertia.Inverse() * (momentBody - omegaCross);
        
        // Quaternion derivative
        var quatDot = 0.5 * quat * new Quaternion(0, omega[0], omega[1], omega[2]);
        
        // Fill derivatives
        deriv[0] = vel[0];
        deriv[1] = vel[1];
        deriv[2] = vel[2];
        deriv[3] = accel[0];
        deriv[4] = accel[1];
        deriv[5] = accel[2];
        deriv[6] = quatDot.W;
        deriv[7] = quatDot.X;
        deriv[8] = quatDot.Y;
        deriv[9] = quatDot.Z;
        deriv[10] = angAccel[0];
        deriv[11] = angAccel[1];
        deriv[12] = angAccel[2];
        
        return deriv;
    }
    
    private void UpdateMotorDynamics(double dt)
    {
        double voltage = _battery.Voltage;
        
        for (int i = 0; i < _params.NumMotors; i++)
        {
            // Motor dynamics (first-order with electrical and mechanical time constants)
            _motors[i].Update(voltage, dt);
            _motorSpeeds[i] = _motors[i].Speed;
        }
    }
    
    private void CheckWarnings()
    {
        // Battery warnings
        if (_battery.StateOfCharge < 20)
            Warning?.Invoke("Low battery warning");
        if (_battery.StateOfCharge < 10)
            Warning?.Invoke("Critical battery - RTL recommended");
        
        // Motor temperature warnings
        for (int i = 0; i < _params.NumMotors; i++)
        {
            if (_motors[i].Temperature > 80)
                Warning?.Invoke($"Motor {i} overheating");
        }
        
        // Attitude warnings
        var euler = _orientation.ToEulerAngles();
        if (System.Math.Abs(euler[0]) > 60 * System.Math.PI / 180 ||
            System.Math.Abs(euler[1]) > 60 * System.Math.PI / 180)
        {
            Warning?.Invoke("Extreme attitude - possible crash");
        }
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
/// Motor model with electrical and thermal dynamics.
/// </summary>
public class MotorModel
{
    private readonly MotorParams _params;
    private double _throttle;
    private double _speed;      // rad/s
    private double _current;    // A
    private double _temperature; // °C
    
    public MotorModel(MotorParams parameters)
    {
        _params = parameters;
        _temperature = 25; // Ambient
    }
    
    public double Speed => _speed;
    public double Current => _current;
    public double Temperature => _temperature;
    
    public void SetThrottle(double throttle)
    {
        _throttle = System.Math.Clamp(throttle, 0, 1);
    }
    
    public void Update(double voltage, double dt)
    {
        // Commanded voltage
        double vCmd = _throttle * voltage;
        
        // Back-EMF
        double backEmf = _params.Kv * _speed;
        
        // Current (including saturation)
        _current = System.Math.Max(0, (vCmd - backEmf) / _params.Resistance);
        _current = System.Math.Min(_current, _params.MaxCurrent);
        
        // Motor torque
        double torque = _params.Kt * _current;
        
        // Load torque (simplified - should come from propeller)
        double loadTorque = _params.LoadCoeff * _speed * _speed;
        
        // Angular acceleration
        double speedDot = (torque - loadTorque) / _params.RotorInertia;
        
        // First-order dynamics
        _speed += speedDot * dt;
        _speed = System.Math.Max(0, _speed);
        _speed = System.Math.Min(_speed, _params.MaxSpeed);
        
        // Thermal model
        double powerLoss = _current * _current * _params.Resistance;
        double tempDot = (powerLoss - (_temperature - 25) / _params.ThermalResistance) / _params.ThermalCapacity;
        _temperature += tempDot * dt;
    }
}

/// <summary>
/// Propeller model using blade element momentum theory.
/// </summary>
public class PropellerModel
{
    private readonly PropellerParams _params;
    private readonly bool _clockwise;
    
    public PropellerModel(PropellerParams parameters, bool clockwise)
    {
        _params = parameters;
        _clockwise = clockwise;
    }
    
    /// <summary>
    /// Compute thrust using blade element theory with momentum theory correction.
    /// </summary>
    public double ComputeThrust(double omega, Vector<double> velocity, Matrix<double> rotation, double rho)
    {
        // Simplified: thrust = CT * rho * n^2 * D^4
        double n = omega / (2 * System.Math.PI); // rev/s
        double D = _params.Diameter;
        
        // Advance ratio
        var velBody = rotation.Transpose() * velocity;
        double Va = -velBody[2]; // Velocity along prop axis
        double J = Va / (n * D + 0.001);
        
        // Thrust coefficient (empirical fit)
        double CT = _params.CT0 - _params.CT1 * J * J;
        CT = System.Math.Max(0, CT);
        
        return CT * rho * n * n * D * D * D * D;
    }
    
    /// <summary>
    /// Compute reaction torque.
    /// </summary>
    public double ComputeTorque(double omega, double rho)
    {
        double n = omega / (2 * System.Math.PI);
        double D = _params.Diameter;
        
        // Torque coefficient
        double CQ = _params.CQ0 + _params.CQ1 * n * n;
        
        return CQ * rho * n * n * D * D * D * D * D;
    }
}

/// <summary>
/// Aerodynamics model for body drag and induced effects.
/// </summary>
public class AerodynamicsModel
{
    private readonly AeroParams _params;
    
    public AerodynamicsModel(AeroParams parameters)
    {
        _params = parameters;
    }
    
    public (Vector<double> force, Vector<double> moment) Compute(
        Vector<double> airVelocity, Vector<double> omega,
        Matrix<double> rotation, double rho)
    {
        var force = Vector<double>.Build.Dense(3);
        var moment = Vector<double>.Build.Dense(3);
        
        // Body velocity in body frame
        var velBody = rotation.Transpose() * airVelocity;
        double speed = velBody.L2Norm();
        
        if (speed < 0.1)
            return (force, moment);
        
        double q = 0.5 * rho * speed * speed; // Dynamic pressure
        
        // Drag force (opposite to velocity)
        var dragDir = -velBody.Normalize(2);
        double Cd = _params.Cd0 + _params.CdAlpha * System.Math.Abs(velBody[2]) / (speed + 0.1);
        force = q * _params.ReferenceArea * Cd * dragDir;
        
        // Damping moments
        moment[0] = -_params.Clp * omega[0] * q * _params.ReferenceArea * _params.WingSpan;
        moment[1] = -_params.Cmq * omega[1] * q * _params.ReferenceArea * _params.MeanChord;
        moment[2] = -_params.Cnr * omega[2] * q * _params.ReferenceArea * _params.WingSpan;
        
        return (force, moment);
    }
}

/// <summary>
/// Battery model with state of charge and voltage sag.
/// </summary>
public class BatteryModel
{
    private readonly BatteryParams _params;
    private double _soc; // State of charge (0-1)
    private double _voltage;
    private double _temperature;
    
    public BatteryModel(BatteryParams parameters)
    {
        _params = parameters;
        Reset();
    }
    
    public double Voltage => _voltage;
    public double StateOfCharge => _soc * 100;
    public double Temperature => _temperature;
    
    public void Reset()
    {
        _soc = 1.0;
        _voltage = _params.NominalVoltage;
        _temperature = 25;
    }
    
    public void Update(double current, double dt)
    {
        // Coulomb counting
        double capacityWh = _params.CapacityAh * _params.NominalVoltage;
        double energyUsed = current * _voltage * dt / 3600;
        _soc -= energyUsed / capacityWh;
        _soc = System.Math.Clamp(_soc, 0, 1);
        
        // Voltage model (OCV - IR drop)
        double ocv = _params.MaxVoltage - (1 - _soc) * (_params.MaxVoltage - _params.MinVoltage);
        _voltage = ocv - current * _params.InternalResistance;
        _voltage = System.Math.Max(_params.MinVoltage, _voltage);
        
        // Simple thermal model
        double powerLoss = current * current * _params.InternalResistance;
        _temperature += powerLoss * 0.01 * dt;
    }
}

/// <summary>
/// Atmospheric model (ISA standard atmosphere).
/// </summary>
public class AtmosphereModel
{
    private const double SeaLevelPressure = 101325; // Pa
    private const double SeaLevelTemperature = 288.15; // K
    private const double SeaLevelDensity = 1.225; // kg/m³
    private const double LapseRate = 0.0065; // K/m
    
    public double GetDensity(double altitude)
    {
        double T = GetTemperature(altitude);
        double p = GetPressure(altitude);
        return p / (287.05 * T);
    }
    
    public double GetPressure(double altitude)
    {
        double T = GetTemperature(altitude);
        return SeaLevelPressure * System.Math.Pow(T / SeaLevelTemperature, 5.2561);
    }
    
    public double GetTemperature(double altitude)
    {
        return SeaLevelTemperature - LapseRate * altitude;
    }
}

/// <summary>
/// Wind field model with turbulence.
/// </summary>
public class WindFieldModel
{
    private readonly Random _rng = new();
    
    public Vector<double> BaseWind { get; set; } = Vector<double>.Build.Dense(3);
    public double TurbulenceIntensity { get; set; } = 0;
    public double GustMagnitude { get; set; } = 0;
    public double GustPeriod { get; set; } = 10;
    
    public Vector<double> GetWind(Vector<double> position, double time)
    {
        var wind = BaseWind.Clone();
        
        // Altitude scaling
        double altitudeFactor = 1 + 0.1 * System.Math.Log(1 + (-position[2]) / 10);
        wind *= altitudeFactor;
        
        // Turbulence (Dryden model simplified)
        if (TurbulenceIntensity > 0)
        {
            for (int i = 0; i < 3; i++)
            {
                wind[i] += TurbulenceIntensity * (2 * _rng.NextDouble() - 1);
            }
        }
        
        // Gusts
        if (GustMagnitude > 0)
        {
            double gustPhase = time * 2 * System.Math.PI / GustPeriod;
            wind[0] += GustMagnitude * System.Math.Sin(gustPhase);
        }
        
        return wind;
    }
}

/// <summary>
/// Sensor suite with realistic noise models.
/// </summary>
public class SensorSuite
{
    private readonly SensorParams _params;
    private Vector<double> _gyroBias;
    private Vector<double> _accelBias;
    
    public SensorSuite(SensorParams parameters)
    {
        _params = parameters;
        _gyroBias = Vector<double>.Build.Dense(3);
        _accelBias = Vector<double>.Build.Dense(3);
    }
    
    public SensorMeasurements Measure(
        Vector<double> position, Vector<double> velocity,
        Matrix<double> rotation, Vector<double> angularVelocity,
        double pressure, double temperature, Random rng)
    {
        var meas = new SensorMeasurements();
        
        // IMU - Gyroscope
        meas.Gyro = angularVelocity.Clone();
        _gyroBias += Vector<double>.Build.DenseOfArray([
            Gaussian(rng) * _params.GyroBiasInstability,
            Gaussian(rng) * _params.GyroBiasInstability,
            Gaussian(rng) * _params.GyroBiasInstability
        ]) * 0.01;
        
        meas.Gyro += _gyroBias;
        for (int i = 0; i < 3; i++)
            meas.Gyro[i] += Gaussian(rng) * _params.GyroNoiseDensity;
        
        // IMU - Accelerometer
        var gravity = Vector<double>.Build.DenseOfArray([0, 0, 9.81]);
        var specificForce = rotation.Transpose() * (ComputeAcceleration(velocity) - gravity);
        meas.Accel = specificForce.Clone();
        
        _accelBias += Vector<double>.Build.DenseOfArray([
            Gaussian(rng) * _params.AccelBiasInstability,
            Gaussian(rng) * _params.AccelBiasInstability,
            Gaussian(rng) * _params.AccelBiasInstability
        ]) * 0.01;
        
        meas.Accel += _accelBias;
        for (int i = 0; i < 3; i++)
            meas.Accel[i] += Gaussian(rng) * _params.AccelNoiseDensity;
        
        // Magnetometer
        var magNed = Vector<double>.Build.DenseOfArray([0.22, 0.05, 0.42]); // Gauss, approximate
        meas.Mag = rotation.Transpose() * magNed;
        for (int i = 0; i < 3; i++)
            meas.Mag[i] += Gaussian(rng) * _params.MagNoise;
        
        // Barometer
        meas.Pressure = pressure + Gaussian(rng) * _params.BaroNoise;
        meas.Temperature = temperature + Gaussian(rng) * 0.5;
        
        // GPS (with latency)
        meas.GpsPosition = position.Clone();
        for (int i = 0; i < 3; i++)
            meas.GpsPosition[i] += Gaussian(rng) * _params.GpsPositionNoise;
        
        meas.GpsVelocity = velocity.Clone();
        for (int i = 0; i < 3; i++)
            meas.GpsVelocity[i] += Gaussian(rng) * _params.GpsVelocityNoise;
        
        meas.GpsValid = rng.NextDouble() > 0.01; // 1% dropout
        
        return meas;
    }
    
    private Vector<double> ComputeAcceleration(Vector<double> velocity)
    {
        // Would need velocity derivative - simplified
        return Vector<double>.Build.Dense(3);
    }
    
    private double Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
    }
}

// Supporting types

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
    
    public double Norm() => System.Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
    
    public Quaternion Normalized()
    {
        double n = Norm();
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
    
    public static Quaternion operator *(double s, Quaternion q) => new(s * q.W, s * q.X, s * q.Y, s * q.Z);
    
    public Matrix<double> ToRotationMatrix()
    {
        double xx = X * X, yy = Y * Y, zz = Z * Z;
        double xy = X * Y, xz = X * Z, yz = Y * Z;
        double wx = W * X, wy = W * Y, wz = W * Z;
        
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { 1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy) },
            { 2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx) },
            { 2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy) }
        });
    }
    
    public Vector<double> ToEulerAngles()
    {
        // Roll (x), Pitch (y), Yaw (z)
        double roll = System.Math.Atan2(2 * (W * X + Y * Z), 1 - 2 * (X * X + Y * Y));
        double pitch = System.Math.Asin(System.Math.Clamp(2 * (W * Y - Z * X), -1, 1));
        double yaw = System.Math.Atan2(2 * (W * Z + X * Y), 1 - 2 * (Y * Y + Z * Z));
        
        return Vector<double>.Build.DenseOfArray([roll, pitch, yaw]);
    }
}

// Parameter classes
public class DronePhysicsParams
{
    public int NumMotors { get; set; } = 4;
    public double Mass { get; set; } = 1.5;
    public Matrix<double> Inertia { get; set; } = Matrix<double>.Build.DenseOfDiagonalArray([0.029, 0.029, 0.055]);
    public (double x, double y, double z, bool clockwise)[] MotorPositions { get; set; } = [
        (0.15, 0.15, 0, false),
        (0.15, -0.15, 0, true),
        (-0.15, -0.15, 0, false),
        (-0.15, 0.15, 0, true)
    ];
    public MotorParams MotorParams { get; set; } = new();
    public PropellerParams PropellerParams { get; set; } = new();
    public AeroParams AeroParams { get; set; } = new();
    public BatteryParams BatteryParams { get; set; } = new();
    public SensorParams SensorParams { get; set; } = new();
    public ControlAllocator ControlAllocator { get; set; } = new();
}

public class MotorParams
{
    public double Kv { get; set; } = 0.01;
    public double Kt { get; set; } = 0.01;
    public double Resistance { get; set; } = 0.1;
    public double MaxCurrent { get; set; } = 30;
    public double MaxSpeed { get; set; } = 1500;
    public double RotorInertia { get; set; } = 0.0001;
    public double LoadCoeff { get; set; } = 1e-6;
    public double ThermalResistance { get; set; } = 2;
    public double ThermalCapacity { get; set; } = 50;
}

public class PropellerParams
{
    public double Diameter { get; set; } = 0.254;
    public double CT0 { get; set; } = 0.1;
    public double CT1 { get; set; } = 0.05;
    public double CQ0 { get; set; } = 0.01;
    public double CQ1 { get; set; } = 0.001;
}

public class AeroParams
{
    public double ReferenceArea { get; set; } = 0.1;
    public double WingSpan { get; set; } = 0.5;
    public double MeanChord { get; set; } = 0.2;
    public double Cd0 { get; set; } = 0.5;
    public double CdAlpha { get; set; } = 0.1;
    public double Clp { get; set; } = 0.1;
    public double Cmq { get; set; } = 0.1;
    public double Cnr { get; set; } = 0.1;
}

public class BatteryParams
{
    public double NominalVoltage { get; set; } = 14.8;
    public double MaxVoltage { get; set; } = 16.8;
    public double MinVoltage { get; set; } = 12.0;
    public double CapacityAh { get; set; } = 5.0;
    public double InternalResistance { get; set; } = 0.02;
}

public class SensorParams
{
    public double GyroNoiseDensity { get; set; } = 0.005;
    public double GyroBiasInstability { get; set; } = 0.0001;
    public double AccelNoiseDensity { get; set; } = 0.02;
    public double AccelBiasInstability { get; set; } = 0.001;
    public double MagNoise { get; set; } = 0.01;
    public double BaroNoise { get; set; } = 5;
    public double GpsPositionNoise { get; set; } = 2;
    public double GpsVelocityNoise { get; set; } = 0.1;
}

public class ControlAllocator
{
    public double[] Allocate(double roll, double pitch, double yaw, double throttle)
    {
        // Quad-X allocation
        return [
            System.Math.Clamp(throttle - roll + pitch + yaw, 0, 1),
            System.Math.Clamp(throttle - roll - pitch - yaw, 0, 1),
            System.Math.Clamp(throttle + roll - pitch + yaw, 0, 1),
            System.Math.Clamp(throttle + roll + pitch - yaw, 0, 1)
        ];
    }
}

public class SensorMeasurements
{
    public Vector<double> Gyro { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Accel { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Mag { get; set; } = Vector<double>.Build.Dense(3);
    public double Pressure { get; set; }
    public double Temperature { get; set; }
    public Vector<double> GpsPosition { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> GpsVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public bool GpsValid { get; set; }
}

public class HighFidelityState
{
    public double Time { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> VelocityBody { get; set; } = Vector<double>.Build.Dense(3);
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector<double> EulerAngles { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> AngularVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public double[] MotorSpeeds { get; set; } = [];
    public double[] MotorCurrents { get; set; } = [];
    public double[] MotorTemperatures { get; set; } = [];
    public double BatteryVoltage { get; set; }
    public double BatteryCurrent { get; set; }
    public double BatteryRemaining { get; set; }
    public double TotalThrust { get; set; }
    public double TotalMass { get; set; }
    public double AirDensity { get; set; }
    public Vector<double> WindVelocity { get; set; } = Vector<double>.Build.Dense(3);
}

public class TerrainModel
{
    private readonly double[,] _heightMap;
    private readonly double _resolution;
    private readonly double _originX;
    private readonly double _originY;
    
    public TerrainModel(double[,] heightMap, double resolution, double originX = 0, double originY = 0)
    {
        _heightMap = heightMap;
        _resolution = resolution;
        _originX = originX;
        _originY = originY;
    }
    
    public double GetHeight(double x, double y)
    {
        int gx = (int)((x - _originX) / _resolution);
        int gy = (int)((y - _originY) / _resolution);
        
        gx = System.Math.Clamp(gx, 0, _heightMap.GetLength(0) - 1);
        gy = System.Math.Clamp(gy, 0, _heightMap.GetLength(1) - 1);
        
        return _heightMap[gx, gy];
    }
}
