namespace ControlWorkbench.Core.Robots;

/// <summary>
/// Base interface for all robot types.
/// </summary>
public interface IRobot
{
    string Name { get; }
    RobotType Type { get; }
    RobotState State { get; }
    
    Task ArmAsync();
    Task DisarmAsync();
    Task SetModeAsync(string mode);
    Task<bool> IsArmedAsync();
}

/// <summary>
/// Robot type enumeration.
/// </summary>
public enum RobotType
{
    Unknown,
    
    // Aerial vehicles
    Quadcopter,
    Hexacopter,
    Octocopter,
    FixedWing,
    VTOL,
    Helicopter,
    
    // Ground vehicles
    DifferentialDrive,
    AckermannSteering,
    Omnidirectional,
    TrackedVehicle,
    Legged,
    
    // Marine vehicles
    SurfaceVessel,
    Submarine,
    
    // Industrial robots
    SerialManipulator,
    ParallelManipulator,
    SCARA,
    Delta,
    CartesianGantry,
    
    // Other
    MobileManipulator,
    Swarm,
    Custom
}

/// <summary>
/// Robot operational state.
/// </summary>
public enum RobotState
{
    Unknown,
    Initializing,
    Disarmed,
    Armed,
    InFlight,
    Landing,
    Emergency,
    Fault,
    Calibrating,
    Charging
}

/// <summary>
/// Multirotor (drone) specific configuration and control.
/// </summary>
public class MultirotorConfig
{
    public int MotorCount { get; set; } = 4;
    public MultirotorFrameType FrameType { get; set; } = MultirotorFrameType.QuadX;
    public double ArmLength { get; set; } = 0.25; // meters
    public double Mass { get; set; } = 1.5; // kg
    public double MaxThrust { get; set; } = 40.0; // N (total)
    public double PropellerDiameter { get; set; } = 0.254; // m (10 inch)
    
    // Motor mixing matrix (thrust allocation)
    public double[,]? MixerMatrix { get; set; }
    
    // Inertia tensor
    public double Ixx { get; set; } = 0.01; // kg*m²
    public double Iyy { get; set; } = 0.01;
    public double Izz { get; set; } = 0.02;
    
    // Limits
    public double MaxRollRate { get; set; } = 720.0; // deg/s
    public double MaxPitchRate { get; set; } = 720.0;
    public double MaxYawRate { get; set; } = 200.0;
    public double MaxTiltAngle { get; set; } = 45.0; // deg
    public double MaxVerticalSpeed { get; set; } = 5.0; // m/s
    public double MaxHorizontalSpeed { get; set; } = 15.0; // m/s
}

public enum MultirotorFrameType
{
    QuadX,
    QuadPlus,
    HexX,
    HexPlus,
    OctoX,
    OctoPlus,
    Y6,
    Coaxial
}

/// <summary>
/// Fixed-wing aircraft configuration.
/// </summary>
public class FixedWingConfig
{
    public double Wingspan { get; set; } = 1.5; // m
    public double WingArea { get; set; } = 0.3; // m²
    public double Mass { get; set; } = 2.0; // kg
    public double ThrustMax { get; set; } = 15.0; // N
    
    // Aerodynamic coefficients
    public double Cl0 { get; set; } = 0.2; // Zero-lift coefficient
    public double Cla { get; set; } = 5.0; // Lift slope
    public double Cd0 { get; set; } = 0.02; // Zero-lift drag
    public double K { get; set; } = 0.04; // Induced drag factor
    
    // Flight envelope
    public double StallSpeed { get; set; } = 10.0; // m/s
    public double CruiseSpeed { get; set; } = 20.0; // m/s
    public double MaxSpeed { get; set; } = 35.0; // m/s
    public double MaxBankAngle { get; set; } = 60.0; // deg
    public double MaxClimbRate { get; set; } = 5.0; // m/s
    public double MinTurnRadius { get; set; } = 30.0; // m
}

/// <summary>
/// Ground robot (rover) configuration.
/// </summary>
public class GroundRobotConfig
{
    public GroundRobotType Type { get; set; } = GroundRobotType.DifferentialDrive;
    public double WheelRadius { get; set; } = 0.05; // m
    public double WheelBase { get; set; } = 0.3; // m (length)
    public double TrackWidth { get; set; } = 0.25; // m (width)
    public double Mass { get; set; } = 5.0; // kg
    public double MaxSpeed { get; set; } = 2.0; // m/s
    public double MaxAcceleration { get; set; } = 1.0; // m/s²
    public double MaxSteeringAngle { get; set; } = 30.0; // deg (for Ackermann)
    public double MinTurnRadius { get; set; } = 0.5; // m
}

public enum GroundRobotType
{
    DifferentialDrive,
    AckermannSteering,
    Omnidirectional,
    Tracked
}

/// <summary>
/// Robotic arm (manipulator) configuration.
/// </summary>
public class ManipulatorConfig
{
    public int JointCount { get; set; } = 6;
    public JointConfig[] Joints { get; set; } = Array.Empty<JointConfig>();
    public double[] DHParameters { get; set; } = Array.Empty<double>(); // DH table flattened
    public double MaxPayload { get; set; } = 5.0; // kg
    public double Reach { get; set; } = 1.0; // m
    
    // Tool center point (TCP) offset
    public double[] TcpOffset { get; set; } = new double[6]; // x, y, z, rx, ry, rz
}

public class JointConfig
{
    public int Index { get; set; }
    public JointType Type { get; set; } = JointType.Revolute;
    public double MinLimit { get; set; }
    public double MaxLimit { get; set; }
    public double MaxVelocity { get; set; }
    public double MaxAcceleration { get; set; }
    public double MaxTorque { get; set; }
    public double GearRatio { get; set; } = 1.0;
    public double Offset { get; set; } = 0.0;
}

public enum JointType
{
    Revolute,
    Prismatic,
    Continuous,
    Fixed
}

/// <summary>
/// Underwater vehicle (AUV/ROV) configuration.
/// </summary>
public class UnderwaterVehicleConfig
{
    public UnderwaterVehicleType Type { get; set; } = UnderwaterVehicleType.ROV;
    public int ThrusterCount { get; set; } = 6;
    public double Mass { get; set; } = 10.0; // kg
    public double Buoyancy { get; set; } = 100.0; // N
    public double MaxDepth { get; set; } = 100.0; // m
    
    // Thruster configuration matrix
    public double[,]? ThrusterMatrix { get; set; }
    
    // Hydrodynamic coefficients
    public double DragX { get; set; } = 10.0;
    public double DragY { get; set; } = 15.0;
    public double DragZ { get; set; } = 15.0;
    public double AddedMassX { get; set; } = 5.0;
    public double AddedMassY { get; set; } = 8.0;
    public double AddedMassZ { get; set; } = 8.0;
}

public enum UnderwaterVehicleType
{
    ROV, // Remotely Operated Vehicle
    AUV, // Autonomous Underwater Vehicle
    Glider
}

/// <summary>
/// Swarm robot configuration.
/// </summary>
public class SwarmConfig
{
    public int AgentCount { get; set; } = 10;
    public SwarmBehavior Behavior { get; set; } = SwarmBehavior.Formation;
    public double CommunicationRange { get; set; } = 50.0; // m
    public double SeparationDistance { get; set; } = 2.0; // m
    public double CohesionWeight { get; set; } = 1.0;
    public double SeparationWeight { get; set; } = 1.5;
    public double AlignmentWeight { get; set; } = 1.0;
    public double GoalWeight { get; set; } = 0.5;
}

public enum SwarmBehavior
{
    Formation,
    Coverage,
    Search,
    Pursuit,
    Flocking,
    Custom
}

/// <summary>
/// Standard flight/drive modes.
/// </summary>
public static class ControlModes
{
    // Multirotor modes
    public const string Stabilize = "STABILIZE";
    public const string Acro = "ACRO";
    public const string AltHold = "ALT_HOLD";
    public const string Loiter = "LOITER";
    public const string PosHold = "POSHOLD";
    public const string Auto = "AUTO";
    public const string Guided = "GUIDED";
    public const string Land = "LAND";
    public const string RTL = "RTL";
    public const string Circle = "CIRCLE";
    public const string Follow = "FOLLOW";
    public const string Brake = "BRAKE";
    
    // Fixed-wing modes
    public const string Manual = "MANUAL";
    public const string FBW_A = "FBW_A";
    public const string FBW_B = "FBW_B";
    public const string Cruise = "CRUISE";
    public const string Takeoff = "TAKEOFF";
    
    // Ground vehicle modes
    public const string Steering = "STEERING";
    public const string Hold = "HOLD";
    public const string Simple = "SIMPLE";
    
    // Common modes
    public const string Mission = "MISSION";
    public const string Offboard = "OFFBOARD";
    public const string Emergency = "EMERGENCY";
}

/// <summary>
/// Failsafe actions.
/// </summary>
public enum FailsafeAction
{
    None,
    Land,
    ReturnToLaunch,
    Loiter,
    Descend,
    Terminate,
    LockDown
}

/// <summary>
/// Geofence configuration.
/// </summary>
public class GeofenceConfig
{
    public bool Enabled { get; set; } = false;
    public GeofenceType Type { get; set; } = GeofenceType.Cylinder;
    public double MaxRadius { get; set; } = 100.0; // m
    public double MaxAltitude { get; set; } = 120.0; // m
    public double MinAltitude { get; set; } = 0.0; // m
    public GeofencePoint[]? Polygon { get; set; }
    public FailsafeAction BreachAction { get; set; } = FailsafeAction.ReturnToLaunch;
}

public enum GeofenceType
{
    Cylinder,
    Polygon,
    InclusionCircles,
    ExclusionCircles
}

public record GeofencePoint(double Latitude, double Longitude);

/// <summary>
/// Mission waypoint.
/// </summary>
public class Waypoint
{
    public int Index { get; set; }
    public WaypointType Type { get; set; } = WaypointType.Navigate;
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Altitude { get; set; }
    public double? Param1 { get; set; }
    public double? Param2 { get; set; }
    public double? Param3 { get; set; }
    public double? Param4 { get; set; }
    public bool Autocontinue { get; set; } = true;
    public string? Frame { get; set; } = "global_relative";
}

public enum WaypointType
{
    Navigate,
    Takeoff,
    Land,
    Loiter,
    LoiterTime,
    LoiterTurns,
    ReturnToLaunch,
    SetSpeed,
    SetServo,
    SetRelay,
    Condition,
    Do,
    Jump,
    ChangeAlt,
    SetROI,
    Delay
}
