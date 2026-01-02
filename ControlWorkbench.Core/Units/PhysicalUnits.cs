namespace ControlWorkbench.Core.Units;

/// <summary>
/// Represents a velocity with explicit units (m/s).
/// </summary>
public readonly struct Velocity : IEquatable<Velocity>, IComparable<Velocity>
{
    /// <summary>
    /// The velocity value in meters per second.
    /// </summary>
    public double MetersPerSecond { get; }

    /// <summary>
    /// The velocity value in kilometers per hour.
    /// </summary>
    public double KilometersPerHour => MetersPerSecond * 3.6;

    /// <summary>
    /// The velocity value in knots.
    /// </summary>
    public double Knots => MetersPerSecond * 1.94384;

    private Velocity(double metersPerSecond)
    {
        MetersPerSecond = metersPerSecond;
    }

    public static Velocity FromMetersPerSecond(double mps) => new(mps);
    public static Velocity FromKilometersPerHour(double kph) => new(kph / 3.6);
    public static Velocity FromKnots(double knots) => new(knots / 1.94384);

    public static Velocity Zero => new(0);

    public static Velocity operator +(Velocity a, Velocity b) => new(a.MetersPerSecond + b.MetersPerSecond);
    public static Velocity operator -(Velocity a, Velocity b) => new(a.MetersPerSecond - b.MetersPerSecond);
    public static Velocity operator -(Velocity a) => new(-a.MetersPerSecond);
    public static Velocity operator *(Velocity a, double scalar) => new(a.MetersPerSecond * scalar);
    public static Velocity operator *(double scalar, Velocity a) => new(a.MetersPerSecond * scalar);
    public static Velocity operator /(Velocity a, double scalar) => new(a.MetersPerSecond / scalar);

    public static bool operator ==(Velocity left, Velocity right) => left.Equals(right);
    public static bool operator !=(Velocity left, Velocity right) => !left.Equals(right);
    public static bool operator <(Velocity left, Velocity right) => left.MetersPerSecond < right.MetersPerSecond;
    public static bool operator >(Velocity left, Velocity right) => left.MetersPerSecond > right.MetersPerSecond;
    public static bool operator <=(Velocity left, Velocity right) => left.MetersPerSecond <= right.MetersPerSecond;
    public static bool operator >=(Velocity left, Velocity right) => left.MetersPerSecond >= right.MetersPerSecond;

    public bool Equals(Velocity other) => MetersPerSecond.Equals(other.MetersPerSecond);
    public override bool Equals(object? obj) => obj is Velocity other && Equals(other);
    public override int GetHashCode() => MetersPerSecond.GetHashCode();
    public int CompareTo(Velocity other) => MetersPerSecond.CompareTo(other.MetersPerSecond);

    public override string ToString() => $"{MetersPerSecond:F2} m/s";
}

/// <summary>
/// Represents a distance with explicit units (meters).
/// </summary>
public readonly struct Distance : IEquatable<Distance>, IComparable<Distance>
{
    /// <summary>
    /// The distance value in meters.
    /// </summary>
    public double Meters { get; }

    /// <summary>
    /// The distance value in kilometers.
    /// </summary>
    public double Kilometers => Meters / 1000.0;

    /// <summary>
    /// The distance value in feet.
    /// </summary>
    public double Feet => Meters * 3.28084;

    private Distance(double meters)
    {
        Meters = meters;
    }

    public static Distance FromMeters(double meters) => new(meters);
    public static Distance FromKilometers(double km) => new(km * 1000.0);
    public static Distance FromFeet(double feet) => new(feet / 3.28084);

    public static Distance Zero => new(0);

    public static Distance operator +(Distance a, Distance b) => new(a.Meters + b.Meters);
    public static Distance operator -(Distance a, Distance b) => new(a.Meters - b.Meters);
    public static Distance operator -(Distance a) => new(-a.Meters);
    public static Distance operator *(Distance a, double scalar) => new(a.Meters * scalar);
    public static Distance operator *(double scalar, Distance a) => new(a.Meters * scalar);
    public static Distance operator /(Distance a, double scalar) => new(a.Meters / scalar);

    public static bool operator ==(Distance left, Distance right) => left.Equals(right);
    public static bool operator !=(Distance left, Distance right) => !left.Equals(right);
    public static bool operator <(Distance left, Distance right) => left.Meters < right.Meters;
    public static bool operator >(Distance left, Distance right) => left.Meters > right.Meters;
    public static bool operator <=(Distance left, Distance right) => left.Meters <= right.Meters;
    public static bool operator >=(Distance left, Distance right) => left.Meters >= right.Meters;

    public bool Equals(Distance other) => Meters.Equals(other.Meters);
    public override bool Equals(object? obj) => obj is Distance other && Equals(other);
    public override int GetHashCode() => Meters.GetHashCode();
    public int CompareTo(Distance other) => Meters.CompareTo(other.Meters);

    public override string ToString() => $"{Meters:F2} m";
}

/// <summary>
/// Represents angular velocity with explicit units (rad/s).
/// </summary>
public readonly struct AngularVelocity : IEquatable<AngularVelocity>, IComparable<AngularVelocity>
{
    private const double RadToDeg = 180.0 / System.Math.PI;
    private const double DegToRad = System.Math.PI / 180.0;

    /// <summary>
    /// The angular velocity in radians per second.
    /// </summary>
    public double RadiansPerSecond { get; }

    /// <summary>
    /// The angular velocity in degrees per second.
    /// </summary>
    public double DegreesPerSecond => RadiansPerSecond * RadToDeg;

    private AngularVelocity(double radiansPerSecond)
    {
        RadiansPerSecond = radiansPerSecond;
    }

    public static AngularVelocity FromRadiansPerSecond(double radps) => new(radps);
    public static AngularVelocity FromDegreesPerSecond(double degps) => new(degps * DegToRad);

    public static AngularVelocity Zero => new(0);

    public static AngularVelocity operator +(AngularVelocity a, AngularVelocity b) => new(a.RadiansPerSecond + b.RadiansPerSecond);
    public static AngularVelocity operator -(AngularVelocity a, AngularVelocity b) => new(a.RadiansPerSecond - b.RadiansPerSecond);
    public static AngularVelocity operator -(AngularVelocity a) => new(-a.RadiansPerSecond);
    public static AngularVelocity operator *(AngularVelocity a, double scalar) => new(a.RadiansPerSecond * scalar);
    public static AngularVelocity operator *(double scalar, AngularVelocity a) => new(a.RadiansPerSecond * scalar);
    public static AngularVelocity operator /(AngularVelocity a, double scalar) => new(a.RadiansPerSecond / scalar);

    public static bool operator ==(AngularVelocity left, AngularVelocity right) => left.Equals(right);
    public static bool operator !=(AngularVelocity left, AngularVelocity right) => !left.Equals(right);
    public static bool operator <(AngularVelocity left, AngularVelocity right) => left.RadiansPerSecond < right.RadiansPerSecond;
    public static bool operator >(AngularVelocity left, AngularVelocity right) => left.RadiansPerSecond > right.RadiansPerSecond;
    public static bool operator <=(AngularVelocity left, AngularVelocity right) => left.RadiansPerSecond <= right.RadiansPerSecond;
    public static bool operator >=(AngularVelocity left, AngularVelocity right) => left.RadiansPerSecond >= right.RadiansPerSecond;

    public bool Equals(AngularVelocity other) => RadiansPerSecond.Equals(other.RadiansPerSecond);
    public override bool Equals(object? obj) => obj is AngularVelocity other && Equals(other);
    public override int GetHashCode() => RadiansPerSecond.GetHashCode();
    public int CompareTo(AngularVelocity other) => RadiansPerSecond.CompareTo(other.RadiansPerSecond);

    public override string ToString() => $"{DegreesPerSecond:F2}°/s";
    public string ToStringRadians() => $"{RadiansPerSecond:F4} rad/s";
}

/// <summary>
/// Represents acceleration with explicit units (m/s²).
/// </summary>
public readonly struct Acceleration : IEquatable<Acceleration>, IComparable<Acceleration>
{
    /// <summary>
    /// The acceleration in meters per second squared.
    /// </summary>
    public double MetersPerSecondSquared { get; }

    /// <summary>
    /// The acceleration in g-force.
    /// </summary>
    public double G => MetersPerSecondSquared / 9.80665;

    private Acceleration(double metersPerSecondSquared)
    {
        MetersPerSecondSquared = metersPerSecondSquared;
    }

    public static Acceleration FromMetersPerSecondSquared(double mps2) => new(mps2);
    public static Acceleration FromG(double g) => new(g * 9.80665);

    public static Acceleration Zero => new(0);
    public static Acceleration Gravity => new(9.80665);

    public static Acceleration operator +(Acceleration a, Acceleration b) => new(a.MetersPerSecondSquared + b.MetersPerSecondSquared);
    public static Acceleration operator -(Acceleration a, Acceleration b) => new(a.MetersPerSecondSquared - b.MetersPerSecondSquared);
    public static Acceleration operator -(Acceleration a) => new(-a.MetersPerSecondSquared);
    public static Acceleration operator *(Acceleration a, double scalar) => new(a.MetersPerSecondSquared * scalar);
    public static Acceleration operator *(double scalar, Acceleration a) => new(a.MetersPerSecondSquared * scalar);
    public static Acceleration operator /(Acceleration a, double scalar) => new(a.MetersPerSecondSquared / scalar);

    public static bool operator ==(Acceleration left, Acceleration right) => left.Equals(right);
    public static bool operator !=(Acceleration left, Acceleration right) => !left.Equals(right);
    public static bool operator <(Acceleration left, Acceleration right) => left.MetersPerSecondSquared < right.MetersPerSecondSquared;
    public static bool operator >(Acceleration left, Acceleration right) => left.MetersPerSecondSquared > right.MetersPerSecondSquared;
    public static bool operator <=(Acceleration left, Acceleration right) => left.MetersPerSecondSquared <= right.MetersPerSecondSquared;
    public static bool operator >=(Acceleration left, Acceleration right) => left.MetersPerSecondSquared >= right.MetersPerSecondSquared;

    public bool Equals(Acceleration other) => MetersPerSecondSquared.Equals(other.MetersPerSecondSquared);
    public override bool Equals(object? obj) => obj is Acceleration other && Equals(other);
    public override int GetHashCode() => MetersPerSecondSquared.GetHashCode();
    public int CompareTo(Acceleration other) => MetersPerSecondSquared.CompareTo(other.MetersPerSecondSquared);

    public override string ToString() => $"{MetersPerSecondSquared:F2} m/s²";
    public string ToStringG() => $"{G:F3} g";
}
