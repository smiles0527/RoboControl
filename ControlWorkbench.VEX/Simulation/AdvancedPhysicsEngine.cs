namespace ControlWorkbench.VEX.Simulation;

/// <summary>
/// Advanced physics simulation for more accurate robot behavior.
/// Includes wheel slip, motor thermal modeling, and battery sag.
/// </summary>
public class AdvancedPhysicsEngine
{
    // Physical constants
    public double GravityAccel { get; set; } = 386.1;       // in/s² (9.81 m/s²)
    public double RobotMass { get; set; } = 15.0;           // lbs
    public double MomentOfInertia { get; set; } = 500;      // lb*in²
    public double WheelRadius { get; set; } = 2.0;          // inches
    public double TrackWidth { get; set; } = 12.0;          // inches

    // Friction coefficients (on VRC foam tiles)
    public double StaticFriction { get; set; } = 1.1;       // Foam tile grip
    public double KineticFriction { get; set; } = 0.9;
    public double RollingResistance { get; set; } = 0.02;

    // Motor characteristics (V5 Blue cartridge)
    public double MotorStallTorque { get; set; } = 2.1;     // N·m
    public double MotorFreeSpeed { get; set; } = 600;       // RPM
    public double MotorResistance { get; set; } = 1.0;      // Ohms (approximate)
    public double MotorInductance { get; set; } = 0.001;    // H

    // Battery simulation
    public double BatteryNominalVoltage { get; set; } = 12.8;
    public double BatteryInternalResistance { get; set; } = 0.1; // Ohms
    public double BatteryCapacityAh { get; set; } = 4.0;
    private double _batteryStateOfCharge = 1.0;             // 0-1

    // Thermal simulation
    private readonly Dictionary<int, MotorThermalState> _motorTemps = new();

    /// <summary>
    /// Calculate motor output with electrical and thermal modeling.
    /// </summary>
    public MotorOutput CalculateMotorOutput(int port, double commandVoltage, double currentRpm)
    {
        // Get or create thermal state
        if (!_motorTemps.TryGetValue(port, out var thermal))
        {
            thermal = new MotorThermalState();
            _motorTemps[port] = thermal;
        }

        // Battery voltage sag under load
        double totalCurrent = _motorTemps.Values.Sum(m => m.LastCurrent);
        double batteryVoltage = BatteryNominalVoltage * _batteryStateOfCharge -
                                totalCurrent * BatteryInternalResistance;

        // Limit command to available voltage
        double actualVoltage = Math.Clamp(commandVoltage, -batteryVoltage, batteryVoltage);

        // Motor back-EMF
        double backEmf = (currentRpm / MotorFreeSpeed) * BatteryNominalVoltage;

        // Motor current (V = IR + back-EMF)
        double current = (actualVoltage - Math.Sign(currentRpm) * backEmf) / MotorResistance;
        current = Math.Clamp(current, -2.5, 2.5); // 2.5A current limit

        // Thermal limiting
        double thermalFactor = CalculateThermalDerating(thermal);
        current *= thermalFactor;

        // Torque (proportional to current)
        double torque = current * (MotorStallTorque / 2.5); // N·m

        // Convert to force at wheel
        double force = torque / (WheelRadius * 0.0254); // Convert inches to meters

        // Update thermal state
        UpdateThermalState(thermal, current);

        return new MotorOutput
        {
            Voltage = actualVoltage,
            Current = Math.Abs(current),
            Torque = torque,
            Force = force,
            Temperature = thermal.Temperature,
            ThermalFactor = thermalFactor
        };
    }

    /// <summary>
    /// Calculate wheel forces including slip and friction.
    /// </summary>
    public WheelForces CalculateWheelForces(
        double leftForce, double rightForce,
        double leftVelocity, double rightVelocity,
        double robotVelocity, double robotOmega)
    {
        // Normal force (weight)
        double normalForce = RobotMass * GravityAccel / 2; // Per side

        // Maximum friction force
        double maxFrictionForce = normalForce * StaticFriction;

        // Check for slip
        double leftWheelVel = leftVelocity * WheelRadius * Math.PI / 30; // RPM to in/s
        double rightWheelVel = rightVelocity * WheelRadius * Math.PI / 30;

        // Expected wheel velocity from robot motion
        double expectedLeftVel = robotVelocity - robotOmega * TrackWidth / 2;
        double expectedRightVel = robotVelocity + robotOmega * TrackWidth / 2;

        // Slip ratio
        double leftSlip = Math.Abs(leftWheelVel - expectedLeftVel) / (Math.Abs(leftWheelVel) + 1);
        double rightSlip = Math.Abs(rightWheelVel - expectedRightVel) / (Math.Abs(rightWheelVel) + 1);

        // Apply friction limiting
        double leftEffectiveForce = leftForce;
        double rightEffectiveForce = rightForce;
        bool leftSlipping = false;
        bool rightSlipping = false;

        if (Math.Abs(leftForce) > maxFrictionForce)
        {
            leftEffectiveForce = Math.Sign(leftForce) * maxFrictionForce * KineticFriction / StaticFriction;
            leftSlipping = true;
        }

        if (Math.Abs(rightForce) > maxFrictionForce)
        {
            rightEffectiveForce = Math.Sign(rightForce) * maxFrictionForce * KineticFriction / StaticFriction;
            rightSlipping = true;
        }

        // Rolling resistance
        double rollingResistForce = RollingResistance * normalForce;
        leftEffectiveForce -= Math.Sign(leftWheelVel) * rollingResistForce;
        rightEffectiveForce -= Math.Sign(rightWheelVel) * rollingResistForce;

        return new WheelForces
        {
            LeftForce = leftEffectiveForce,
            RightForce = rightEffectiveForce,
            LeftSlipping = leftSlipping,
            RightSlipping = rightSlipping,
            LeftSlipRatio = leftSlip,
            RightSlipRatio = rightSlip
        };
    }

    /// <summary>
    /// Simulate robot dynamics for one timestep.
    /// </summary>
    public RobotState SimulateStep(RobotState current, double leftVoltage, double rightVoltage, double dt)
    {
        // Calculate motor outputs
        var leftMotor = CalculateMotorOutput(1, leftVoltage, current.LeftMotorRpm);
        var rightMotor = CalculateMotorOutput(4, rightVoltage, current.RightMotorRpm);

        // Calculate wheel forces with friction
        var wheelForces = CalculateWheelForces(
            leftMotor.Force, rightMotor.Force,
            current.LeftMotorRpm, current.RightMotorRpm,
            current.LinearVelocity, current.AngularVelocity
        );

        // Net force and torque on robot
        double netForce = wheelForces.LeftForce + wheelForces.RightForce;
        double netTorque = (wheelForces.RightForce - wheelForces.LeftForce) * TrackWidth / 2;

        // Accelerations
        double linearAccel = netForce / RobotMass;
        double angularAccel = netTorque / MomentOfInertia;

        // Update velocities
        double newLinearVel = current.LinearVelocity + linearAccel * dt;
        double newAngularVel = current.AngularVelocity + angularAccel * dt;

        // Limit velocities (physical constraints)
        double maxLinearVel = MotorFreeSpeed * Math.PI * WheelRadius * 2 / 60; // in/s
        newLinearVel = Math.Clamp(newLinearVel, -maxLinearVel, maxLinearVel);

        // Update position (trapezoidal integration)
        double avgLinearVel = (current.LinearVelocity + newLinearVel) / 2;
        double avgAngularVel = (current.AngularVelocity + newAngularVel) / 2;
        double newHeading = current.Heading + avgAngularVel * dt;

        double avgHeading = (current.Heading + newHeading) / 2;
        double newX = current.X + avgLinearVel * Math.Cos(avgHeading) * dt;
        double newY = current.Y + avgLinearVel * Math.Sin(avgHeading) * dt;

        // Update motor RPMs (simplified - in reality affected by load)
        double leftRpmChange = (leftVoltage / BatteryNominalVoltage * MotorFreeSpeed - current.LeftMotorRpm) * 5 * dt;
        double rightRpmChange = (rightVoltage / BatteryNominalVoltage * MotorFreeSpeed - current.RightMotorRpm) * 5 * dt;

        return new RobotState
        {
            X = newX,
            Y = newY,
            Heading = NormalizeAngle(newHeading),
            LinearVelocity = newLinearVel,
            AngularVelocity = newAngularVel,
            LeftMotorRpm = current.LeftMotorRpm + leftRpmChange,
            RightMotorRpm = current.RightMotorRpm + rightRpmChange,
            LeftMotorTemp = leftMotor.Temperature,
            RightMotorTemp = rightMotor.Temperature,
            BatteryVoltage = BatteryNominalVoltage * _batteryStateOfCharge,
            LeftSlipping = wheelForces.LeftSlipping,
            RightSlipping = wheelForces.RightSlipping
        };
    }

    private double CalculateThermalDerating(MotorThermalState thermal)
    {
        // Start derating at 55°C, full cutoff at 70°C
        if (thermal.Temperature < 55) return 1.0;
        if (thermal.Temperature > 70) return 0.1; // Not zero to prevent instability
        return 1.0 - 0.9 * (thermal.Temperature - 55) / 15;
    }

    private void UpdateThermalState(MotorThermalState thermal, double current)
    {
        // Power dissipated as heat (I²R)
        double powerDissipated = current * current * MotorResistance;

        // Thermal mass and cooling
        double thermalMass = 50; // J/°C (approximate for V5 motor)
        double coolingRate = 0.5; // W/°C

        double heatIn = powerDissipated * 0.01; // Per timestep
        double heatOut = coolingRate * (thermal.Temperature - 25) * 0.01;

        thermal.Temperature += (heatIn - heatOut) / thermalMass * 100;
        thermal.Temperature = Math.Clamp(thermal.Temperature, 25, 80);
        thermal.LastCurrent = Math.Abs(current);
    }

    public void DrainBattery(double ampHours)
    {
        _batteryStateOfCharge -= ampHours / BatteryCapacityAh;
        _batteryStateOfCharge = Math.Clamp(_batteryStateOfCharge, 0.1, 1.0);
    }

    public void ResetBattery()
    {
        _batteryStateOfCharge = 1.0;
    }

    public void ResetMotorTemperatures()
    {
        foreach (var thermal in _motorTemps.Values)
        {
            thermal.Temperature = 25;
            thermal.LastCurrent = 0;
        }
    }

    private static double NormalizeAngle(double angle)
    {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

public class MotorOutput
{
    public double Voltage { get; set; }
    public double Current { get; set; }
    public double Torque { get; set; }
    public double Force { get; set; }
    public double Temperature { get; set; }
    public double ThermalFactor { get; set; }
}

public class WheelForces
{
    public double LeftForce { get; set; }
    public double RightForce { get; set; }
    public bool LeftSlipping { get; set; }
    public bool RightSlipping { get; set; }
    public double LeftSlipRatio { get; set; }
    public double RightSlipRatio { get; set; }
}

public class RobotState
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Heading { get; set; }
    public double LinearVelocity { get; set; }
    public double AngularVelocity { get; set; }
    public double LeftMotorRpm { get; set; }
    public double RightMotorRpm { get; set; }
    public double LeftMotorTemp { get; set; }
    public double RightMotorTemp { get; set; }
    public double BatteryVoltage { get; set; }
    public bool LeftSlipping { get; set; }
    public bool RightSlipping { get; set; }
}

public class MotorThermalState
{
    public double Temperature { get; set; } = 25;
    public double LastCurrent { get; set; }
}

/// <summary>
/// Sensor noise and error modeling for realistic simulation.
/// </summary>
public class SensorNoiseModel
{
    private readonly Random _random = new();

    // IMU noise characteristics (based on V5 IMU specs)
    public double ImuHeadingDrift { get; set; } = 0.01;        // degrees/second
    public double ImuHeadingNoise { get; set; } = 0.1;         // degrees RMS
    public double ImuAccelNoise { get; set; } = 0.02;          // g RMS

    // Encoder noise
    public double EncoderCountNoise { get; set; } = 0.5;       // counts RMS

    // Distance sensor noise
    public double DistanceNoise { get; set; } = 5;             // mm RMS
    public double DistanceMaxRange { get; set; } = 2000;       // mm

    // GPS sensor noise (V5 GPS)
    public double GpsPositionNoise { get; set; } = 0.5;        // inches RMS
    public double GpsHeadingNoise { get; set; } = 2;           // degrees RMS
    public double GpsUpdateLatency { get; set; } = 0.05;       // seconds

    private double _accumulatedDrift = 0;
    private double _lastGpsUpdate = 0;

    public double AddImuHeadingNoise(double trueHeading, double dt)
    {
        // Accumulate drift
        _accumulatedDrift += ImuHeadingDrift * dt * (_random.NextDouble() - 0.5) * 2;

        // Add instantaneous noise
        double noise = GaussianNoise(ImuHeadingNoise);

        return trueHeading + _accumulatedDrift + noise;
    }

    public (double x, double y, double heading) AddGpsNoise(
        double trueX, double trueY, double trueHeading, double time)
    {
        // Check if GPS update is available
        if (time - _lastGpsUpdate < GpsUpdateLatency)
        {
            return (double.NaN, double.NaN, double.NaN); // No update yet
        }
        _lastGpsUpdate = time;

        // Add position noise
        double noisyX = trueX + GaussianNoise(GpsPositionNoise);
        double noisyY = trueY + GaussianNoise(GpsPositionNoise);
        double noisyHeading = trueHeading + GaussianNoise(GpsHeadingNoise);

        return (noisyX, noisyY, noisyHeading);
    }

    public double AddEncoderNoise(double trueCounts)
    {
        return trueCounts + GaussianNoise(EncoderCountNoise);
    }

    public double AddDistanceNoise(double trueDistance)
    {
        if (trueDistance > DistanceMaxRange)
            return DistanceMaxRange; // Out of range

        return Math.Max(0, trueDistance + GaussianNoise(DistanceNoise));
    }

    public void ResetDrift()
    {
        _accumulatedDrift = 0;
    }

    private double GaussianNoise(double stdDev)
    {
        // Box-Muller transform
        double u1 = 1.0 - _random.NextDouble();
        double u2 = 1.0 - _random.NextDouble();
        double normal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
        return normal * stdDev;
    }
}
