namespace ControlWorkbench.VEX.PathPlanning;

/// <summary>
/// Visual path planner with field map integration.
/// Generates smooth paths for autonomous routines.
/// </summary>
public class VisualPathPlanner
{
    public FieldMap Field { get; }
    public List<PlannedPath> Paths { get; } = new();
    public PlannedPath? CurrentPath { get; private set; }

    public event Action<PlannedPath>? PathCreated;
    public event Action<PlannedPath>? PathModified;

    public VisualPathPlanner(VrcGame game = VrcGame.HighStakes)
    {
        Field = new FieldMap(game);
    }

    /// <summary>
    /// Start creating a new path.
    /// </summary>
    public PlannedPath StartNewPath(string name)
    {
        CurrentPath = new PlannedPath(name);
        return CurrentPath;
    }

    /// <summary>
    /// Add a waypoint to the current path.
    /// </summary>
    public void AddWaypoint(double x, double y, double? heading = null, double? velocity = null)
    {
        if (CurrentPath == null)
            StartNewPath("Unnamed Path");

        var waypoint = new PathWaypoint
        {
            X = x,
            Y = y,
            Heading = heading,
            Velocity = velocity ?? 60,
            Index = CurrentPath!.Waypoints.Count
        };

        CurrentPath.Waypoints.Add(waypoint);
        CurrentPath.RegeneratePath();
        PathModified?.Invoke(CurrentPath);
    }

    /// <summary>
    /// Remove a waypoint by index.
    /// </summary>
    public void RemoveWaypoint(int index)
    {
        if (CurrentPath == null || index < 0 || index >= CurrentPath.Waypoints.Count)
            return;

        CurrentPath.Waypoints.RemoveAt(index);
        for (int i = index; i < CurrentPath.Waypoints.Count; i++)
            CurrentPath.Waypoints[i].Index = i;

        CurrentPath.RegeneratePath();
        PathModified?.Invoke(CurrentPath);
    }

    /// <summary>
    /// Move a waypoint to a new position.
    /// </summary>
    public void MoveWaypoint(int index, double x, double y)
    {
        if (CurrentPath == null || index < 0 || index >= CurrentPath.Waypoints.Count)
            return;

        CurrentPath.Waypoints[index].X = x;
        CurrentPath.Waypoints[index].Y = y;
        CurrentPath.RegeneratePath();
        PathModified?.Invoke(CurrentPath);
    }

    /// <summary>
    /// Finish and save the current path.
    /// </summary>
    public PlannedPath FinishPath()
    {
        if (CurrentPath == null)
            throw new InvalidOperationException("No path in progress");

        CurrentPath.RegeneratePath();
        Paths.Add(CurrentPath);
        PathCreated?.Invoke(CurrentPath);

        var completed = CurrentPath;
        CurrentPath = null;
        return completed;
    }

    /// <summary>
    /// Generate LemLib path code.
    /// </summary>
    public string ExportToLemLib(PlannedPath path)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"// Path: {path.Name}");
        sb.AppendLine($"// Generated: {DateTime.Now}");
        sb.AppendLine($"// Total distance: {path.TotalDistance:F1} inches");
        sb.AppendLine();

        // Export as asset
        sb.AppendLine($"ASSET({path.Name.Replace(" ", "_")}_txt) = {{");

        foreach (var wp in path.Waypoints)
        {
            string headingStr = wp.Heading.HasValue ? $", {wp.Heading.Value:F1}" : "";
            sb.AppendLine($"    {wp.X:F1}, {wp.Y:F1}{headingStr},");
        }

        sb.AppendLine("};");
        sb.AppendLine();

        // Follow code
        sb.AppendLine($"void run_{path.Name.Replace(" ", "_").ToLower()}() {{");
        sb.AppendLine($"    chassis.follow({path.Name.Replace(" ", "_")}_txt, 15, {path.Waypoints.FirstOrDefault()?.Velocity ?? 60});");
        sb.AppendLine("}");

        return sb.ToString();
    }

    /// <summary>
    /// Generate Pure Pursuit code.
    /// </summary>
    public string ExportToPurePursuit(PlannedPath path)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"// Path: {path.Name}");
        sb.AppendLine($"// Pure Pursuit waypoints");
        sb.AppendLine();

        sb.AppendLine($"std::vector<Waypoint> {path.Name.Replace(" ", "_").ToLower()}_path = {{");

        foreach (var wp in path.Waypoints)
        {
            sb.AppendLine($"    {{{wp.X:F1}, {wp.Y:F1}, {wp.Velocity:F0}}},");
        }

        sb.AppendLine("};");

        return sb.ToString();
    }

    /// <summary>
    /// Load paths from a file.
    /// </summary>
    public void LoadPaths(string json)
    {
        // Simple JSON parsing (in practice, use System.Text.Json)
        // This is a placeholder for actual JSON deserialization
    }

    /// <summary>
    /// Save all paths to JSON.
    /// </summary>
    public string SavePaths()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("{");
        sb.AppendLine("  \"paths\": [");

        for (int i = 0; i < Paths.Count; i++)
        {
            var path = Paths[i];
            sb.AppendLine("    {");
            sb.AppendLine($"      \"name\": \"{path.Name}\",");
            sb.AppendLine("      \"waypoints\": [");

            for (int j = 0; j < path.Waypoints.Count; j++)
            {
                var wp = path.Waypoints[j];
                string comma = j < path.Waypoints.Count - 1 ? "," : "";
                sb.AppendLine($"        {{\"x\": {wp.X:F1}, \"y\": {wp.Y:F1}, \"heading\": {wp.Heading ?? 0}, \"velocity\": {wp.Velocity}}}{comma}");
            }

            sb.AppendLine("      ]");
            sb.AppendLine($"    }}{(i < Paths.Count - 1 ? "," : "")}");
        }

        sb.AppendLine("  ]");
        sb.AppendLine("}");

        return sb.ToString();
    }
}

/// <summary>
/// A planned path with waypoints and generated trajectory.
/// </summary>
public class PlannedPath
{
    public string Name { get; set; }
    public List<PathWaypoint> Waypoints { get; } = new();
    public List<TrajectoryPoint> Trajectory { get; private set; } = new();

    public double TotalDistance { get; private set; }
    public double TotalTime { get; private set; }

    public PathSettings Settings { get; } = new();

    public PlannedPath(string name)
    {
        Name = name;
    }

    /// <summary>
    /// Regenerate the smooth trajectory from waypoints.
    /// </summary>
    public void RegeneratePath()
    {
        if (Waypoints.Count < 2)
        {
            Trajectory = new List<TrajectoryPoint>();
            TotalDistance = 0;
            TotalTime = 0;
            return;
        }

        // Generate smooth path using cubic spline interpolation
        var smoothPoints = GenerateSmoothPath();

        // Inject distance and curvature
        InjectDistanceAndCurvature(smoothPoints);

        // Apply velocity constraints
        ApplyVelocityConstraints(smoothPoints);

        Trajectory = smoothPoints;
        TotalDistance = smoothPoints.LastOrDefault()?.Distance ?? 0;
        TotalTime = smoothPoints.LastOrDefault()?.Time ?? 0;
    }

    private List<TrajectoryPoint> GenerateSmoothPath()
    {
        var points = new List<TrajectoryPoint>();
        double spacing = Settings.PointSpacing;

        for (int i = 0; i < Waypoints.Count - 1; i++)
        {
            var p0 = i > 0 ? Waypoints[i - 1] : Waypoints[i];
            var p1 = Waypoints[i];
            var p2 = Waypoints[i + 1];
            var p3 = i < Waypoints.Count - 2 ? Waypoints[i + 2] : Waypoints[i + 1];

            // Catmull-Rom spline
            int segments = (int)(Distance(p1.X, p1.Y, p2.X, p2.Y) / spacing);
            segments = Math.Max(segments, 2);

            for (int j = 0; j < segments; j++)
            {
                double t = j / (double)segments;
                var (x, y) = CatmullRom(p0, p1, p2, p3, t);
                points.Add(new TrajectoryPoint { X = x, Y = y });
            }
        }

        // Add final point
        var last = Waypoints.Last();
        points.Add(new TrajectoryPoint { X = last.X, Y = last.Y });

        return points;
    }

    private (double x, double y) CatmullRom(PathWaypoint p0, PathWaypoint p1, PathWaypoint p2, PathWaypoint p3, double t)
    {
        double t2 = t * t;
        double t3 = t2 * t;

        double x = 0.5 * ((2 * p1.X) +
            (-p0.X + p2.X) * t +
            (2 * p0.X - 5 * p1.X + 4 * p2.X - p3.X) * t2 +
            (-p0.X + 3 * p1.X - 3 * p2.X + p3.X) * t3);

        double y = 0.5 * ((2 * p1.Y) +
            (-p0.Y + p2.Y) * t +
            (2 * p0.Y - 5 * p1.Y + 4 * p2.Y - p3.Y) * t2 +
            (-p0.Y + 3 * p1.Y - 3 * p2.Y + p3.Y) * t3);

        return (x, y);
    }

    private void InjectDistanceAndCurvature(List<TrajectoryPoint> points)
    {
        if (points.Count == 0) return;

        points[0].Distance = 0;
        points[0].Curvature = 0;

        for (int i = 1; i < points.Count; i++)
        {
            double dx = points[i].X - points[i - 1].X;
            double dy = points[i].Y - points[i - 1].Y;
            double dist = Math.Sqrt(dx * dx + dy * dy);

            points[i].Distance = points[i - 1].Distance + dist;
            points[i].Heading = Math.Atan2(dy, dx);

            // Curvature using three points
            if (i >= 2)
            {
                double x1 = points[i - 2].X, y1 = points[i - 2].Y;
                double x2 = points[i - 1].X, y2 = points[i - 1].Y;
                double x3 = points[i].X, y3 = points[i].Y;

                double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2 + 0.0001);
                double k2 = (y1 - y2) / (x1 - x2 + 0.0001);
                double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) /
                    (x3 * k2 - y3 + y2 - x2 * k2 + 0.0001);
                double a = k1 - k2 * b;
                double r = Math.Sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));

                points[i].Curvature = 1.0 / (r + 0.0001);
            }
        }
    }

    private void ApplyVelocityConstraints(List<TrajectoryPoint> points)
    {
        if (points.Count == 0) return;

        double maxVel = Settings.MaxVelocity;
        double maxAccel = Settings.MaxAcceleration;
        double maxDecel = Settings.MaxDeceleration;

        // Forward pass - limit by curvature
        for (int i = 0; i < points.Count; i++)
        {
            double curvatureLimit = Math.Sqrt(maxAccel / (points[i].Curvature + 0.0001));
            points[i].Velocity = Math.Min(maxVel, curvatureLimit);

            // Find nearest waypoint and use its velocity as max
            double minWpDist = double.MaxValue;
            double wpVel = maxVel;
            foreach (var wp in Waypoints)
            {
                double dist = Distance(points[i].X, points[i].Y, wp.X, wp.Y);
                if (dist < minWpDist)
                {
                    minWpDist = dist;
                    wpVel = wp.Velocity;
                }
            }
            points[i].Velocity = Math.Min(points[i].Velocity, wpVel);
        }

        // Backward pass - limit by deceleration
        for (int i = points.Count - 2; i >= 0; i--)
        {
            double dist = points[i + 1].Distance - points[i].Distance;
            double maxVelFromDecel = Math.Sqrt(points[i + 1].Velocity * points[i + 1].Velocity + 2 * maxDecel * dist);
            points[i].Velocity = Math.Min(points[i].Velocity, maxVelFromDecel);
        }

        // Forward pass - limit by acceleration
        for (int i = 1; i < points.Count; i++)
        {
            double dist = points[i].Distance - points[i - 1].Distance;
            double maxVelFromAccel = Math.Sqrt(points[i - 1].Velocity * points[i - 1].Velocity + 2 * maxAccel * dist);
            points[i].Velocity = Math.Min(points[i].Velocity, maxVelFromAccel);
        }

        // Calculate time
        double time = 0;
        for (int i = 0; i < points.Count; i++)
        {
            points[i].Time = time;
            if (i < points.Count - 1)
            {
                double dist = points[i + 1].Distance - points[i].Distance;
                double avgVel = (points[i].Velocity + points[i + 1].Velocity) / 2;
                time += dist / (avgVel + 0.0001);
            }
        }
    }

    private double Distance(double x1, double y1, double x2, double y2)
    {
        return Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
}

public class PathWaypoint
{
    public int Index { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double? Heading { get; set; }
    public double Velocity { get; set; } = 60;

    // Actions at this waypoint
    public List<WaypointAction> Actions { get; } = new();
}

public class WaypointAction
{
    public WaypointActionType Type { get; set; }
    public string Parameter { get; set; } = "";
    public double Value { get; set; }
}

public enum WaypointActionType
{
    Wait,
    SetIntake,
    TogglePneumatic,
    RunSubroutine,
    PlaySound
}

public class TrajectoryPoint
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Heading { get; set; }
    public double Distance { get; set; }
    public double Velocity { get; set; }
    public double Curvature { get; set; }
    public double Time { get; set; }
}

public class PathSettings
{
    public double PointSpacing { get; set; } = 1.0;        // inches
    public double MaxVelocity { get; set; } = 60;          // in/sec
    public double MaxAcceleration { get; set; } = 80;      // in/sec²
    public double MaxDeceleration { get; set; } = 100;     // in/sec²
    public double TurnSmoothness { get; set; } = 0.8;      // Catmull-Rom tension
}

/// <summary>
/// VRC field map with game elements.
/// </summary>
public class FieldMap
{
    public VrcGame Game { get; }
    public double Width { get; } = 144;
    public double Height { get; } = 144;

    public List<FieldElement> Elements { get; } = new();
    public List<StartingPosition> StartPositions { get; } = new();

    public FieldMap(VrcGame game)
    {
        Game = game;
        SetupField();
    }

    private void SetupField()
    {
        switch (Game)
        {
            case VrcGame.HighStakes:
                SetupHighStakes();
                break;
            case VrcGame.OverUnder:
                SetupOverUnder();
                break;
            case VrcGame.SpinUp:
                SetupSpinUp();
                break;
        }
    }

    private void SetupHighStakes()
    {
        // Starting positions
        StartPositions.Add(new StartingPosition("Red Left", 18, 18, 45, Alliance.Red));
        StartPositions.Add(new StartingPosition("Red Right", 18, 126, -45, Alliance.Red));
        StartPositions.Add(new StartingPosition("Blue Left", 126, 126, -135, Alliance.Blue));
        StartPositions.Add(new StartingPosition("Blue Right", 126, 18, 135, Alliance.Blue));

        // Stakes
        Elements.Add(new FieldElement("Alliance Stake Red", 12, 72, FieldElementType.AllianceStake, Alliance.Red));
        Elements.Add(new FieldElement("Alliance Stake Blue", 132, 72, FieldElementType.AllianceStake, Alliance.Blue));
        Elements.Add(new FieldElement("Neutral Stake 1", 72, 24, FieldElementType.NeutralStake));
        Elements.Add(new FieldElement("Neutral Stake 2", 72, 120, FieldElementType.NeutralStake));

        // Wall stakes
        Elements.Add(new FieldElement("Wall Stake 1", 36, 0, FieldElementType.WallStake));
        Elements.Add(new FieldElement("Wall Stake 2", 108, 0, FieldElementType.WallStake));
        Elements.Add(new FieldElement("Wall Stake 3", 36, 144, FieldElementType.WallStake));
        Elements.Add(new FieldElement("Wall Stake 4", 108, 144, FieldElementType.WallStake));

        // Corners
        Elements.Add(new FieldElement("Positive Corner Red", 0, 0, FieldElementType.Corner, Alliance.Red));
        Elements.Add(new FieldElement("Negative Corner Red", 0, 144, FieldElementType.Corner, Alliance.Red));
        Elements.Add(new FieldElement("Positive Corner Blue", 144, 144, FieldElementType.Corner, Alliance.Blue));
        Elements.Add(new FieldElement("Negative Corner Blue", 144, 0, FieldElementType.Corner, Alliance.Blue));

        // Ladder/elevation
        Elements.Add(new FieldElement("Ladder", 72, 72, FieldElementType.Ladder));
    }

    private void SetupOverUnder()
    {
        // Over Under specific elements
        StartPositions.Add(new StartingPosition("Red Offensive", 12, 60, 0, Alliance.Red));
        StartPositions.Add(new StartingPosition("Red Defensive", 12, 132, -90, Alliance.Red));
        StartPositions.Add(new StartingPosition("Blue Offensive", 132, 84, 180, Alliance.Blue));
        StartPositions.Add(new StartingPosition("Blue Defensive", 132, 12, 90, Alliance.Blue));

        Elements.Add(new FieldElement("Barrier", 72, 72, FieldElementType.Barrier));
        Elements.Add(new FieldElement("Goal Red", 6, 72, FieldElementType.Goal, Alliance.Red));
        Elements.Add(new FieldElement("Goal Blue", 138, 72, FieldElementType.Goal, Alliance.Blue));
    }

    private void SetupSpinUp()
    {
        // Spin Up specific elements
        StartPositions.Add(new StartingPosition("Red Near Roller", 36, 9, 90, Alliance.Red));
        StartPositions.Add(new StartingPosition("Blue Near Roller", 108, 135, -90, Alliance.Blue));

        for (int i = 0; i < 4; i++)
        {
            Elements.Add(new FieldElement($"Roller {i + 1}", i < 2 ? 0 : 144, i % 2 == 0 ? 36 : 108, FieldElementType.Roller));
        }

        Elements.Add(new FieldElement("High Goal Red", 6, 72, FieldElementType.HighGoal, Alliance.Red));
        Elements.Add(new FieldElement("High Goal Blue", 138, 72, FieldElementType.HighGoal, Alliance.Blue));
    }
}

public class FieldElement
{
    public string Name { get; }
    public double X { get; }
    public double Y { get; }
    public FieldElementType Type { get; }
    public Alliance? Alliance { get; }

    public FieldElement(string name, double x, double y, FieldElementType type, Alliance? alliance = null)
    {
        Name = name; X = x; Y = y; Type = type; Alliance = alliance;
    }
}

public enum FieldElementType
{
    AllianceStake,
    NeutralStake,
    WallStake,
    MobileGoal,
    Corner,
    Ladder,
    Barrier,
    Goal,
    Roller,
    HighGoal,
    LowGoal
}

public class StartingPosition
{
    public string Name { get; }
    public double X { get; }
    public double Y { get; }
    public double Heading { get; }
    public Alliance Alliance { get; }

    public StartingPosition(string name, double x, double y, double heading, Alliance alliance)
    {
        Name = name; X = x; Y = y; Heading = heading; Alliance = alliance;
    }
}

public enum Alliance { Red, Blue }

public enum VrcGame
{
    HighStakes,     // 2024-2025
    OverUnder,      // 2023-2024
    SpinUp,         // 2022-2023
    TippingPoint,   // 2021-2022
    ChangeUp        // 2020-2021
}
