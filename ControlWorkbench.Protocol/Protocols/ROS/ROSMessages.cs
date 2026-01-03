namespace ControlWorkbench.Protocol.Protocols.ROS;

/// <summary>
/// ROS (Robot Operating System) message type definitions.
/// Provides compatibility with ROS1 and ROS2 ecosystems.
/// </summary>
public static class ROSConstants
{
    public const string DefaultMasterUri = "http://localhost:11311";
    public const int DefaultDomainId = 0; // ROS2
}

/// <summary>
/// Standard ROS message types from std_msgs.
/// </summary>
public static class StdMsgs
{
    public record Bool(bool Data);
    public record Byte(byte Data);
    public record Char(char Data);
    public record ColorRGBA(float R, float G, float B, float A);
    public record Duration(int Secs, uint Nsecs);
    public record Empty();
    public record Float32(float Data);
    public record Float64(double Data);
    public record Header(uint Seq, Time Stamp, string FrameId);
    public record Int8(sbyte Data);
    public record Int16(short Data);
    public record Int32(int Data);
    public record Int64(long Data);
    public record MultiArrayDimension(string Label, uint Size, uint Stride);
    public record MultiArrayLayout(MultiArrayDimension[] Dim, uint DataOffset);
    public record String(string Data);
    public record Time(int Secs, uint Nsecs);
    public record UInt8(byte Data);
    public record UInt16(ushort Data);
    public record UInt32(uint Data);
    public record UInt64(ulong Data);
}

/// <summary>
/// Geometry message types from geometry_msgs.
/// </summary>
public static class GeometryMsgs
{
    public record Accel(Vector3 Linear, Vector3 Angular);
    public record AccelStamped(StdMsgs.Header Header, Accel Accel);
    public record AccelWithCovariance(Accel Accel, double[] Covariance);
    public record AccelWithCovarianceStamped(StdMsgs.Header Header, AccelWithCovariance Accel);
    public record Inertia(double M, Vector3 Com, double Ixx, double Ixy, double Ixz, double Iyy, double Iyz, double Izz);
    public record InertiaStamped(StdMsgs.Header Header, Inertia Inertia);
    public record Point(double X, double Y, double Z);
    public record Point32(float X, float Y, float Z);
    public record PointStamped(StdMsgs.Header Header, Point Point);
    public record Polygon(Point32[] Points);
    public record PolygonStamped(StdMsgs.Header Header, Polygon Polygon);
    public record Pose(Point Position, Quaternion Orientation);
    public record Pose2D(double X, double Y, double Theta);
    public record PoseArray(StdMsgs.Header Header, Pose[] Poses);
    public record PoseStamped(StdMsgs.Header Header, Pose Pose);
    public record PoseWithCovariance(Pose Pose, double[] Covariance);
    public record PoseWithCovarianceStamped(StdMsgs.Header Header, PoseWithCovariance Pose);
    public record Quaternion(double X, double Y, double Z, double W);
    public record QuaternionStamped(StdMsgs.Header Header, Quaternion Quaternion);
    public record Transform(Vector3 Translation, Quaternion Rotation);
    public record TransformStamped(StdMsgs.Header Header, string ChildFrameId, Transform Transform);
    public record Twist(Vector3 Linear, Vector3 Angular);
    public record TwistStamped(StdMsgs.Header Header, Twist Twist);
    public record TwistWithCovariance(Twist Twist, double[] Covariance);
    public record TwistWithCovarianceStamped(StdMsgs.Header Header, TwistWithCovariance Twist);
    public record Vector3(double X, double Y, double Z);
    public record Vector3Stamped(StdMsgs.Header Header, Vector3 Vector);
    public record Wrench(Vector3 Force, Vector3 Torque);
    public record WrenchStamped(StdMsgs.Header Header, Wrench Wrench);
}

/// <summary>
/// Sensor message types from sensor_msgs.
/// </summary>
public static class SensorMsgs
{
    public record BatteryState(
        StdMsgs.Header Header,
        float Voltage,
        float Temperature,
        float Current,
        float Charge,
        float Capacity,
        float DesignCapacity,
        float Percentage,
        byte PowerSupplyStatus,
        byte PowerSupplyHealth,
        byte PowerSupplyTechnology,
        bool Present,
        float[] CellVoltage,
        float[] CellTemperature,
        string Location,
        string SerialNumber);

    public record CameraInfo(
        StdMsgs.Header Header,
        uint Height,
        uint Width,
        string DistortionModel,
        double[] D,
        double[] K,
        double[] R,
        double[] P,
        uint BinningX,
        uint BinningY,
        RegionOfInterest Roi);

    public record ChannelFloat32(string Name, float[] Values);

    public record CompressedImage(StdMsgs.Header Header, string Format, byte[] Data);

    public record FluidPressure(StdMsgs.Header Header, double FluidPressureValue, double Variance);

    public record Illuminance(StdMsgs.Header Header, double IlluminanceValue, double Variance);

    public record Image(
        StdMsgs.Header Header,
        uint Height,
        uint Width,
        string Encoding,
        byte IsBigEndian,
        uint Step,
        byte[] Data);

    public record Imu(
        StdMsgs.Header Header,
        GeometryMsgs.Quaternion Orientation,
        double[] OrientationCovariance,
        GeometryMsgs.Vector3 AngularVelocity,
        double[] AngularVelocityCovariance,
        GeometryMsgs.Vector3 LinearAcceleration,
        double[] LinearAccelerationCovariance);

    public record JointState(
        StdMsgs.Header Header,
        string[] Name,
        double[] Position,
        double[] Velocity,
        double[] Effort);

    public record Joy(StdMsgs.Header Header, float[] Axes, int[] Buttons);

    public record JoyFeedback(byte Type, byte Id, float Intensity);

    public record JoyFeedbackArray(JoyFeedback[] Array);

    public record LaserEcho(float[] Echoes);

    public record LaserScan(
        StdMsgs.Header Header,
        float AngleMin,
        float AngleMax,
        float AngleIncrement,
        float TimeIncrement,
        float ScanTime,
        float RangeMin,
        float RangeMax,
        float[] Ranges,
        float[] Intensities);

    public record MagneticField(
        StdMsgs.Header Header,
        GeometryMsgs.Vector3 MagneticFieldValue,
        double[] MagneticFieldCovariance);

    public record MultiDOFJointState(
        StdMsgs.Header Header,
        string[] JointNames,
        GeometryMsgs.Transform[] Transforms,
        GeometryMsgs.Twist[] Twist,
        GeometryMsgs.Wrench[] Wrench);

    public record MultiEchoLaserScan(
        StdMsgs.Header Header,
        float AngleMin,
        float AngleMax,
        float AngleIncrement,
        float TimeIncrement,
        float ScanTime,
        float RangeMin,
        float RangeMax,
        LaserEcho[] Ranges,
        LaserEcho[] Intensities);

    public record NavSatFix(
        StdMsgs.Header Header,
        NavSatStatus Status,
        double Latitude,
        double Longitude,
        double Altitude,
        double[] PositionCovariance,
        byte PositionCovarianceType);

    public record NavSatStatus(sbyte Status, ushort Service);

    public record PointCloud(
        StdMsgs.Header Header,
        GeometryMsgs.Point32[] Points,
        ChannelFloat32[] Channels);

    public record PointCloud2(
        StdMsgs.Header Header,
        uint Height,
        uint Width,
        PointField[] Fields,
        bool IsBigendian,
        uint PointStep,
        uint RowStep,
        byte[] Data,
        bool IsDense);

    public record PointField(string Name, uint Offset, byte Datatype, uint Count);

    public record Range(
        StdMsgs.Header Header,
        byte RadiationType,
        float FieldOfView,
        float MinRange,
        float MaxRange,
        float RangeValue);

    public record RegionOfInterest(
        uint XOffset,
        uint YOffset,
        uint Height,
        uint Width,
        bool DoRectify);

    public record RelativeHumidity(StdMsgs.Header Header, double RelativeHumidityValue, double Variance);

    public record Temperature(StdMsgs.Header Header, double TemperatureValue, double Variance);

    public record TimeReference(StdMsgs.Header Header, StdMsgs.Time TimeRef, string Source);
}

/// <summary>
/// Navigation message types from nav_msgs.
/// </summary>
public static class NavMsgs
{
    public record GridCells(StdMsgs.Header Header, float CellWidth, float CellHeight, GeometryMsgs.Point[] Cells);

    public record MapMetaData(
        StdMsgs.Time MapLoadTime,
        float Resolution,
        uint Width,
        uint Height,
        GeometryMsgs.Pose Origin);

    public record OccupancyGrid(StdMsgs.Header Header, MapMetaData Info, sbyte[] Data);

    public record Odometry(
        StdMsgs.Header Header,
        string ChildFrameId,
        GeometryMsgs.PoseWithCovariance Pose,
        GeometryMsgs.TwistWithCovariance Twist);

    public record Path(StdMsgs.Header Header, GeometryMsgs.PoseStamped[] Poses);

    public record GetMapAction();
    public record GetMapGoal();
    public record GetMapResult(OccupancyGrid Map);
    public record GetMapFeedback();
}

/// <summary>
/// TF2 (Transform) message types.
/// </summary>
public static class Tf2Msgs
{
    public record TFMessage(GeometryMsgs.TransformStamped[] Transforms);
}

/// <summary>
/// Action types for robot actions.
/// </summary>
public static class ActionlibMsgs
{
    public record GoalID(StdMsgs.Time Stamp, string Id);

    public record GoalStatus(GoalID GoalId, byte Status, string Text);

    public record GoalStatusArray(StdMsgs.Header Header, GoalStatus[] StatusList);

    public static class GoalStatusValues
    {
        public const byte Pending = 0;
        public const byte Active = 1;
        public const byte Preempted = 2;
        public const byte Succeeded = 3;
        public const byte Aborted = 4;
        public const byte Rejected = 5;
        public const byte Preempting = 6;
        public const byte Recalling = 7;
        public const byte Recalled = 8;
        public const byte Lost = 9;
    }
}

/// <summary>
/// Diagnostic message types from diagnostic_msgs.
/// </summary>
public static class DiagnosticMsgs
{
    public record DiagnosticArray(StdMsgs.Header Header, DiagnosticStatus[] Status);

    public record DiagnosticStatus(
        byte Level,
        string Name,
        string Message,
        string HardwareId,
        KeyValue[] Values);

    public record KeyValue(string Key, string Value);

    public static class DiagnosticLevel
    {
        public const byte Ok = 0;
        public const byte Warn = 1;
        public const byte Error = 2;
        public const byte Stale = 3;
    }
}
