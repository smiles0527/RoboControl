namespace ControlWorkbench.Core.Units;

/// <summary>
/// Represents an angle with explicit radians/degrees distinction.
/// Internally stored as radians for mathematical consistency.
/// </summary>
public readonly struct Angle : IEquatable<Angle>, IComparable<Angle>
{
    private const double RadToDeg = 180.0 / System.Math.PI;
    private const double DegToRad = System.Math.PI / 180.0;

    /// <summary>
    /// The angle value in radians.
    /// </summary>
    public double Radians { get; }

    /// <summary>
    /// The angle value in degrees.
    /// </summary>
    public double Degrees => Radians * RadToDeg;

    private Angle(double radians)
    {
        Radians = radians;
    }

    /// <summary>
    /// Creates an angle from a value in radians.
    /// </summary>
    public static Angle FromRadians(double radians) => new(radians);

    /// <summary>
    /// Creates an angle from a value in degrees.
    /// </summary>
    public static Angle FromDegrees(double degrees) => new(degrees * DegToRad);

    /// <summary>
    /// Zero angle.
    /// </summary>
    public static Angle Zero => new(0);

    /// <summary>
    /// Normalizes the angle to the range [-?, ?].
    /// </summary>
    public Angle Normalize()
    {
        double normalized = Radians;
        while (normalized > System.Math.PI) normalized -= 2.0 * System.Math.PI;
        while (normalized < -System.Math.PI) normalized += 2.0 * System.Math.PI;
        return new Angle(normalized);
    }

    /// <summary>
    /// Normalizes the angle to the range [0, 2?].
    /// </summary>
    public Angle NormalizePositive()
    {
        double normalized = Radians % (2.0 * System.Math.PI);
        if (normalized < 0) normalized += 2.0 * System.Math.PI;
        return new Angle(normalized);
    }

    public static Angle operator +(Angle a, Angle b) => new(a.Radians + b.Radians);
    public static Angle operator -(Angle a, Angle b) => new(a.Radians - b.Radians);
    public static Angle operator -(Angle a) => new(-a.Radians);
    public static Angle operator *(Angle a, double scalar) => new(a.Radians * scalar);
    public static Angle operator *(double scalar, Angle a) => new(a.Radians * scalar);
    public static Angle operator /(Angle a, double scalar) => new(a.Radians / scalar);

    public static bool operator ==(Angle left, Angle right) => left.Equals(right);
    public static bool operator !=(Angle left, Angle right) => !left.Equals(right);
    public static bool operator <(Angle left, Angle right) => left.Radians < right.Radians;
    public static bool operator >(Angle left, Angle right) => left.Radians > right.Radians;
    public static bool operator <=(Angle left, Angle right) => left.Radians <= right.Radians;
    public static bool operator >=(Angle left, Angle right) => left.Radians >= right.Radians;

    public bool Equals(Angle other) => Radians.Equals(other.Radians);
    public override bool Equals(object? obj) => obj is Angle other && Equals(other);
    public override int GetHashCode() => Radians.GetHashCode();
    public int CompareTo(Angle other) => Radians.CompareTo(other.Radians);

    public override string ToString() => $"{Degrees:F2}°";
    public string ToStringRadians() => $"{Radians:F4} rad";
}
