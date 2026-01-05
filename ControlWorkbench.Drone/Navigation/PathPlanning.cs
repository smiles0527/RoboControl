using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Navigation;

/// <summary>
/// A* path planner for 2D grid-based navigation.
/// Compatible with occupancy grid and cost map representations.
/// </summary>
public class AStarPlanner
{
    private readonly int _width;
    private readonly int _height;
    private readonly double _resolution;
    private readonly bool _allowDiagonal;
    
    private readonly int[] _dx;
    private readonly int[] _dy;
    private readonly double[] _costs;

    public AStarPlanner(
        int width,
        int height,
        double resolution,
        bool allowDiagonal = true)
    {
        _width = width;
        _height = height;
        _resolution = resolution;
        _allowDiagonal = allowDiagonal;
        
        if (allowDiagonal)
        {
            _dx = [-1, 0, 1, 0, -1, -1, 1, 1];
            _dy = [0, -1, 0, 1, -1, 1, -1, 1];
            double sqrt2 = System.Math.Sqrt(2);
            _costs = [1, 1, 1, 1, sqrt2, sqrt2, sqrt2, sqrt2];
        }
        else
        {
            _dx = [-1, 0, 1, 0];
            _dy = [0, -1, 0, 1];
            _costs = [1, 1, 1, 1];
        }
    }

    /// <summary>
    /// Plans a path from start to goal using occupancy grid.
    /// </summary>
    public AStarResult? Plan(
        OccupancyGrid2D grid,
        (int x, int y) start,
        (int x, int y) goal,
        double occupancyThreshold = 0.65)
    {
        return Plan(start, goal, (x, y) => 
            grid.IsOccupied(x, y, occupancyThreshold) ? double.PositiveInfinity : 1.0);
    }

    /// <summary>
    /// Plans a path from start to goal using cost map.
    /// </summary>
    public AStarResult? Plan(
        CostMap2D costMap,
        (int x, int y) start,
        (int x, int y) goal,
        float maxTraversableCost = CostMap2D.CostLethal - 1)
    {
        return Plan(start, goal, (x, y) =>
        {
            float cost = costMap.GetCost(x, y);
            return cost > maxTraversableCost ? double.PositiveInfinity : 1.0 + cost / 100.0;
        });
    }

    /// <summary>
    /// Plans a path using custom cost function.
    /// </summary>
    public AStarResult? Plan(
        (int x, int y) start,
        (int x, int y) goal,
        Func<int, int, double> cellCostFunc)
    {
        if (!IsInBounds(start.x, start.y) || !IsInBounds(goal.x, goal.y))
            return null;
        
        if (double.IsPositiveInfinity(cellCostFunc(start.x, start.y)) ||
            double.IsPositiveInfinity(cellCostFunc(goal.x, goal.y)))
            return null;
        
        var openSet = new PriorityQueue<(int x, int y), double>();
        var cameFrom = new Dictionary<(int, int), (int, int)>();
        var gScore = new Dictionary<(int, int), double>();
        var fScore = new Dictionary<(int, int), double>();
        var closedSet = new HashSet<(int, int)>();
        
        gScore[start] = 0;
        fScore[start] = Heuristic(start, goal);
        openSet.Enqueue(start, fScore[start]);
        
        int nodesExpanded = 0;
        
        while (openSet.Count > 0)
        {
            var current = openSet.Dequeue();
            nodesExpanded++;
            
            if (current == goal)
            {
                return new AStarResult
                {
                    Path = ReconstructPath(cameFrom, current),
                    PathCost = gScore[current],
                    NodesExpanded = nodesExpanded,
                    Success = true
                };
            }
            
            if (closedSet.Contains(current))
                continue;
            
            closedSet.Add(current);
            
            for (int i = 0; i < _dx.Length; i++)
            {
                int nx = current.x + _dx[i];
                int ny = current.y + _dy[i];
                var neighbor = (nx, ny);
                
                if (!IsInBounds(nx, ny) || closedSet.Contains(neighbor))
                    continue;
                
                double cellCost = cellCostFunc(nx, ny);
                if (double.IsPositiveInfinity(cellCost))
                    continue;
                
                double tentativeG = gScore[current] + _costs[i] * cellCost;
                
                if (!gScore.TryGetValue(neighbor, out double existingG) || tentativeG < existingG)
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + Heuristic(neighbor, goal);
                    openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }
        
        return new AStarResult
        {
            Path = [],
            PathCost = double.PositiveInfinity,
            NodesExpanded = nodesExpanded,
            Success = false
        };
    }

    /// <summary>
    /// Plans a path with waypoint smoothing.
    /// </summary>
    public AStarResult? PlanSmooth(
        OccupancyGrid2D grid,
        (int x, int y) start,
        (int x, int y) goal,
        double occupancyThreshold = 0.65,
        double robotRadius = 0)
    {
        var result = Plan(grid, start, goal, occupancyThreshold);
        if (result == null || !result.Success)
            return result;
        
        // Smooth the path
        var smoothed = SmoothPath(result.Path, grid, occupancyThreshold, robotRadius);
        
        return new AStarResult
        {
            Path = smoothed,
            PathCost = result.PathCost,
            NodesExpanded = result.NodesExpanded,
            Success = true
        };
    }

    private List<(int x, int y)> SmoothPath(
        List<(int x, int y)> path,
        OccupancyGrid2D grid,
        double threshold,
        double robotRadius)
    {
        if (path.Count <= 2)
            return path;
        
        var smoothed = new List<(int x, int y)> { path[0] };
        int current = 0;
        
        while (current < path.Count - 1)
        {
            int farthest = current + 1;
            
            // Find farthest visible point
            for (int i = path.Count - 1; i > current + 1; i--)
            {
                var (sx, sy) = grid.GridToWorld(path[current].x, path[current].y);
                var (ex, ey) = grid.GridToWorld(path[i].x, path[i].y);
                
                if (grid.IsPathFree(sx, sy, ex, ey, robotRadius, threshold))
                {
                    farthest = i;
                    break;
                }
            }
            
            smoothed.Add(path[farthest]);
            current = farthest;
        }
        
        return smoothed;
    }

    private double Heuristic((int x, int y) a, (int x, int y) b)
    {
        // Octile distance for diagonal movement
        int dx = System.Math.Abs(a.x - b.x);
        int dy = System.Math.Abs(a.y - b.y);
        
        if (_allowDiagonal)
        {
            const double D = 1.0;
            const double D2 = 1.41421356;
            return D * (dx + dy) + (D2 - 2 * D) * System.Math.Min(dx, dy);
        }
        
        return dx + dy; // Manhattan distance
    }

    private List<(int x, int y)> ReconstructPath(
        Dictionary<(int, int), (int, int)> cameFrom,
        (int x, int y) current)
    {
        var path = new List<(int x, int y)> { current };
        
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Add(current);
        }
        
        path.Reverse();
        return path;
    }

    private bool IsInBounds(int x, int y)
    {
        return x >= 0 && x < _width && y >= 0 && y < _height;
    }
}

/// <summary>
/// Result of A* path planning.
/// </summary>
public class AStarResult
{
    public required List<(int x, int y)> Path { get; init; }
    public double PathCost { get; init; }
    public int NodesExpanded { get; init; }
    public bool Success { get; init; }
    
    /// <summary>
    /// Converts grid path to world coordinates.
    /// </summary>
    public List<(double x, double y)> ToWorldPath(
        double originX, double originY, double resolution)
    {
        return Path.Select(p => (
            originX + (p.x + 0.5) * resolution,
            originY + (p.y + 0.5) * resolution
        )).ToList();
    }
}

/// <summary>
/// 3D path planner for UAV navigation using RRT*.
/// </summary>
public class RrtStarPlanner3D
{
    private readonly OccupancyGrid3D _grid;
    private readonly double _stepSize;
    private readonly double _goalBias;
    private readonly double _rewireRadius;
    private readonly int _maxIterations;
    private readonly Random _rng;

    public RrtStarPlanner3D(
        OccupancyGrid3D grid,
        double stepSize = 1.0,
        double goalBias = 0.1,
        double rewireRadius = 3.0,
        int maxIterations = 5000)
    {
        _grid = grid;
        _stepSize = stepSize;
        _goalBias = goalBias;
        _rewireRadius = rewireRadius;
        _maxIterations = maxIterations;
        _rng = new Random();
    }

    /// <summary>
    /// Plans a 3D path from start to goal.
    /// </summary>
    public RrtResult? Plan(
        Vector<double> start,
        Vector<double> goal,
        double goalTolerance = 1.0,
        double robotRadius = 0.5)
    {
        var nodes = new List<RrtNode>
        {
            new() { Position = start, Parent = null, Cost = 0 }
        };
        
        RrtNode? goalNode = null;
        
        for (int iter = 0; iter < _maxIterations; iter++)
        {
            // Sample random point (with goal bias)
            Vector<double> sample;
            if (_rng.NextDouble() < _goalBias)
            {
                sample = goal;
            }
            else
            {
                sample = RandomSample();
            }
            
            // Find nearest node
            var nearest = FindNearest(nodes, sample);
            
            // Steer towards sample
            var newPos = Steer(nearest.Position, sample);
            
            // Check collision
            if (!_grid.IsPathFree(nearest.Position, newPos, robotRadius))
                continue;
            
            // Find nodes within rewire radius
            var nearNodes = FindNear(nodes, newPos, _rewireRadius);
            
            // Choose best parent
            var bestParent = nearest;
            double bestCost = nearest.Cost + Distance(nearest.Position, newPos);
            
            foreach (var nearNode in nearNodes)
            {
                double cost = nearNode.Cost + Distance(nearNode.Position, newPos);
                if (cost < bestCost && _grid.IsPathFree(nearNode.Position, newPos, robotRadius))
                {
                    bestParent = nearNode;
                    bestCost = cost;
                }
            }
            
            // Add new node
            var newNode = new RrtNode
            {
                Position = newPos,
                Parent = bestParent,
                Cost = bestCost
            };
            nodes.Add(newNode);
            
            // Rewire
            foreach (var nearNode in nearNodes)
            {
                double newCost = newNode.Cost + Distance(newNode.Position, nearNode.Position);
                if (newCost < nearNode.Cost && _grid.IsPathFree(newNode.Position, nearNode.Position, robotRadius))
                {
                    nearNode.Parent = newNode;
                    nearNode.Cost = newCost;
                }
            }
            
            // Check goal
            if (Distance(newPos, goal) < goalTolerance)
            {
                if (goalNode == null || newNode.Cost < goalNode.Cost)
                {
                    goalNode = newNode;
                }
            }
        }
        
        if (goalNode == null)
            return null;
        
        // Extract path
        var path = new List<Vector<double>>();
        var current = goalNode;
        while (current != null)
        {
            path.Add(current.Position);
            current = current.Parent;
        }
        path.Reverse();
        
        return new RrtResult
        {
            Path = path,
            Cost = goalNode.Cost,
            NodesGenerated = nodes.Count
        };
    }

    private Vector<double> RandomSample()
    {
        return Vector<double>.Build.Dense([
            _rng.NextDouble() * _grid.SizeX * _grid.Resolution,
            _rng.NextDouble() * _grid.SizeY * _grid.Resolution,
            _rng.NextDouble() * _grid.SizeZ * _grid.Resolution
        ]);
    }

    private RrtNode FindNearest(List<RrtNode> nodes, Vector<double> point)
    {
        return nodes.MinBy(n => Distance(n.Position, point))!;
    }

    private List<RrtNode> FindNear(List<RrtNode> nodes, Vector<double> point, double radius)
    {
        return nodes.Where(n => Distance(n.Position, point) < radius).ToList();
    }

    private Vector<double> Steer(Vector<double> from, Vector<double> to)
    {
        var direction = to - from;
        double dist = direction.L2Norm();
        
        if (dist <= _stepSize)
            return to;
        
        return from + direction.Normalize(2) * _stepSize;
    }

    private static double Distance(Vector<double> a, Vector<double> b)
    {
        return (a - b).L2Norm();
    }

    private class RrtNode
    {
        public required Vector<double> Position { get; init; }
        public RrtNode? Parent { get; set; }
        public double Cost { get; set; }
    }
}

/// <summary>
/// Result of RRT* planning.
/// </summary>
public class RrtResult
{
    public required List<Vector<double>> Path { get; init; }
    public double Cost { get; init; }
    public int NodesGenerated { get; init; }
}

/// <summary>
/// Terrain analyzer for drone navigation.
/// </summary>
public class TerrainAnalyzer
{
    private readonly double[,] _elevationMap;
    private readonly int _width;
    private readonly int _height;
    private readonly double _resolution;
    private readonly double _originX;
    private readonly double _originY;

    public TerrainAnalyzer(
        double[,] elevationMap,
        double resolution,
        double originX = 0,
        double originY = 0)
    {
        _elevationMap = elevationMap;
        _width = elevationMap.GetLength(0);
        _height = elevationMap.GetLength(1);
        _resolution = resolution;
        _originX = originX;
        _originY = originY;
    }

    /// <summary>
    /// Gets elevation at world coordinates using bilinear interpolation.
    /// </summary>
    public double GetElevation(double worldX, double worldY)
    {
        double gx = (worldX - _originX) / _resolution;
        double gy = (worldY - _originY) / _resolution;
        
        int x0 = (int)System.Math.Floor(gx);
        int y0 = (int)System.Math.Floor(gy);
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        
        // Clamp to bounds
        x0 = System.Math.Clamp(x0, 0, _width - 1);
        x1 = System.Math.Clamp(x1, 0, _width - 1);
        y0 = System.Math.Clamp(y0, 0, _height - 1);
        y1 = System.Math.Clamp(y1, 0, _height - 1);
        
        double fx = gx - System.Math.Floor(gx);
        double fy = gy - System.Math.Floor(gy);
        
        // Bilinear interpolation
        double v00 = _elevationMap[x0, y0];
        double v10 = _elevationMap[x1, y0];
        double v01 = _elevationMap[x0, y1];
        double v11 = _elevationMap[x1, y1];
        
        return v00 * (1 - fx) * (1 - fy) +
               v10 * fx * (1 - fy) +
               v01 * (1 - fx) * fy +
               v11 * fx * fy;
    }

    /// <summary>
    /// Computes the slope map (gradient magnitude).
    /// </summary>
    public double[,] ComputeSlopeMap()
    {
        var slopes = new double[_width, _height];
        
        for (int x = 1; x < _width - 1; x++)
        {
            for (int y = 1; y < _height - 1; y++)
            {
                // Sobel gradient
                double gx = (_elevationMap[x + 1, y - 1] + 2 * _elevationMap[x + 1, y] + _elevationMap[x + 1, y + 1]) -
                           (_elevationMap[x - 1, y - 1] + 2 * _elevationMap[x - 1, y] + _elevationMap[x - 1, y + 1]);
                double gy = (_elevationMap[x - 1, y + 1] + 2 * _elevationMap[x, y + 1] + _elevationMap[x + 1, y + 1]) -
                           (_elevationMap[x - 1, y - 1] + 2 * _elevationMap[x, y - 1] + _elevationMap[x + 1, y - 1]);
                
                gx /= 8 * _resolution;
                gy /= 8 * _resolution;
                
                slopes[x, y] = System.Math.Sqrt(gx * gx + gy * gy);
            }
        }
        
        return slopes;
    }

    /// <summary>
    /// Computes aspect map (direction of steepest descent).
    /// </summary>
    public double[,] ComputeAspectMap()
    {
        var aspects = new double[_width, _height];
        
        for (int x = 1; x < _width - 1; x++)
        {
            for (int y = 1; y < _height - 1; y++)
            {
                double gx = (_elevationMap[x + 1, y] - _elevationMap[x - 1, y]) / (2 * _resolution);
                double gy = (_elevationMap[x, y + 1] - _elevationMap[x, y - 1]) / (2 * _resolution);
                
                aspects[x, y] = System.Math.Atan2(gy, gx);
            }
        }
        
        return aspects;
    }

    /// <summary>
    /// Computes roughness (standard deviation of elevation in neighborhood).
    /// </summary>
    public double[,] ComputeRoughnessMap(int windowSize = 3)
    {
        var roughness = new double[_width, _height];
        int halfWindow = windowSize / 2;
        
        for (int x = halfWindow; x < _width - halfWindow; x++)
        {
            for (int y = halfWindow; y < _height - halfWindow; y++)
            {
                // Compute mean
                double sum = 0;
                int count = 0;
                for (int dx = -halfWindow; dx <= halfWindow; dx++)
                {
                    for (int dy = -halfWindow; dy <= halfWindow; dy++)
                    {
                        sum += _elevationMap[x + dx, y + dy];
                        count++;
                    }
                }
                double mean = sum / count;
                
                // Compute variance
                double variance = 0;
                for (int dx = -halfWindow; dx <= halfWindow; dx++)
                {
                    for (int dy = -halfWindow; dy <= halfWindow; dy++)
                    {
                        double diff = _elevationMap[x + dx, y + dy] - mean;
                        variance += diff * diff;
                    }
                }
                
                roughness[x, y] = System.Math.Sqrt(variance / count);
            }
        }
        
        return roughness;
    }

    /// <summary>
    /// Finds safe landing zones based on slope and roughness thresholds.
    /// </summary>
    public List<(int x, int y)> FindLandingZones(
        double maxSlope = 0.15,
        double maxRoughness = 0.3,
        int minAreaCells = 9)
    {
        var slopes = ComputeSlopeMap();
        var roughness = ComputeRoughnessMap();
        
        // Mark safe cells
        var safe = new bool[_width, _height];
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                safe[x, y] = slopes[x, y] <= maxSlope && roughness[x, y] <= maxRoughness;
            }
        }
        
        // Find connected components of safe cells
        var visited = new bool[_width, _height];
        var zones = new List<(int x, int y)>();
        
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                if (safe[x, y] && !visited[x, y])
                {
                    var cells = FloodFill(safe, visited, x, y);
                    if (cells.Count >= minAreaCells)
                    {
                        // Return center of zone
                        int cx = (int)cells.Average(c => c.x);
                        int cy = (int)cells.Average(c => c.y);
                        zones.Add((cx, cy));
                    }
                }
            }
        }
        
        return zones;
    }

    /// <summary>
    /// Computes line-of-sight coverage from a viewpoint.
    /// </summary>
    public bool[,] ComputeViewshed(
        double viewX, double viewY, double viewHeight,
        double targetHeight = 0)
    {
        var viewshed = new bool[_width, _height];
        
        double viewElevation = GetElevation(viewX, viewY) + viewHeight;
        int viewGx = (int)((viewX - _originX) / _resolution);
        int viewGy = (int)((viewY - _originY) / _resolution);
        
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                if (x == viewGx && y == viewGy)
                {
                    viewshed[x, y] = true;
                    continue;
                }
                
                viewshed[x, y] = IsVisible(viewGx, viewGy, viewElevation, x, y, targetHeight);
            }
        }
        
        return viewshed;
    }

    private bool IsVisible(
        int fromX, int fromY, double fromElevation,
        int toX, int toY, double targetHeight)
    {
        double targetElevation = _elevationMap[toX, toY] + targetHeight;
        
        // Bresenham line
        int dx = System.Math.Abs(toX - fromX);
        int dy = System.Math.Abs(toY - fromY);
        int sx = fromX < toX ? 1 : -1;
        int sy = fromY < toY ? 1 : -1;
        int err = dx - dy;
        
        int x = fromX, y = fromY;
        double totalDist = System.Math.Sqrt(dx * dx + dy * dy);
        
        while (x != toX || y != toY)
        {
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx) { err += dx; y += sy; }
            
            if (x == toX && y == toY)
                break;
            
            // Check if this cell blocks the view
            double dist = System.Math.Sqrt((x - fromX) * (x - fromX) + (y - fromY) * (y - fromY));
            double t = dist / totalDist;
            double expectedElevation = fromElevation + t * (targetElevation - fromElevation);
            
            if (_elevationMap[x, y] > expectedElevation)
                return false;
        }
        
        return true;
    }

    private static List<(int x, int y)> FloodFill(bool[,] grid, bool[,] visited, int startX, int startY)
    {
        var cells = new List<(int x, int y)>();
        var queue = new Queue<(int x, int y)>();
        queue.Enqueue((startX, startY));
        
        int width = grid.GetLength(0);
        int height = grid.GetLength(1);
        
        while (queue.Count > 0)
        {
            var (x, y) = queue.Dequeue();
            
            if (x < 0 || x >= width || y < 0 || y >= height)
                continue;
            if (visited[x, y] || !grid[x, y])
                continue;
            
            visited[x, y] = true;
            cells.Add((x, y));
            
            queue.Enqueue((x - 1, y));
            queue.Enqueue((x + 1, y));
            queue.Enqueue((x, y - 1));
            queue.Enqueue((x, y + 1));
        }
        
        return cells;
    }
}

/// <summary>
/// Local obstacle avoidance using Vector Field Histogram (VFH+).
/// </summary>
public class VectorFieldHistogram
{
    private readonly int _numSectors;
    private readonly double _sectorWidth;
    private readonly double[] _histogram;
    private readonly double _safetyDistance;
    private readonly double _robotRadius;
    
    public VectorFieldHistogram(
        int numSectors = 72,
        double safetyDistance = 0.5,
        double robotRadius = 0.3)
    {
        _numSectors = numSectors;
        _sectorWidth = 2 * System.Math.PI / numSectors;
        _histogram = new double[numSectors];
        _safetyDistance = safetyDistance;
        _robotRadius = robotRadius;
    }

    /// <summary>
    /// Updates histogram from lidar scan.
    /// </summary>
    public void UpdateFromLidar(LidarScan2D scan)
    {
        Array.Clear(_histogram, 0, _numSectors);
        
        for (int i = 0; i < scan.Ranges.Length; i++)
        {
            double range = scan.Ranges[i];
            if (range < scan.MinRange || range > scan.MaxRange)
                continue;
            
            double angle = scan.AngleMin + i * scan.AngleIncrement;
            int sector = AngleToSector(angle);
            
            // Certainty based on range (closer = higher certainty)
            double certainty = 1.0 - (range / scan.MaxRange);
            double magnitude = certainty * certainty;
            
            // Enlarge obstacle based on robot radius
            double enlargement = System.Math.Asin((_robotRadius + _safetyDistance) / 
                                                   System.Math.Max(range, _robotRadius + _safetyDistance));
            int enlargedSectors = (int)System.Math.Ceiling(enlargement / _sectorWidth);
            
            for (int ds = -enlargedSectors; ds <= enlargedSectors; ds++)
            {
                int s = (sector + ds + _numSectors) % _numSectors;
                _histogram[s] += magnitude;
            }
        }
    }

    /// <summary>
    /// Finds the best direction towards goal while avoiding obstacles.
    /// </summary>
    public double? FindBestDirection(double goalAngle, double threshold = 1.0)
    {
        // Find all valleys (openings)
        var valleys = new List<(int start, int end)>();
        int? valleyStart = null;
        
        for (int i = 0; i < _numSectors; i++)
        {
            if (_histogram[i] < threshold)
            {
                if (valleyStart == null)
                    valleyStart = i;
            }
            else
            {
                if (valleyStart != null)
                {
                    valleys.Add((valleyStart.Value, i - 1));
                    valleyStart = null;
                }
            }
        }
        
        // Handle wrap-around
        if (valleyStart != null)
        {
            if (valleys.Count > 0 && valleys[0].start == 0)
            {
                // Merge with first valley
                valleys[0] = (valleyStart.Value, valleys[0].end + _numSectors);
            }
            else
            {
                valleys.Add((valleyStart.Value, _numSectors - 1));
            }
        }
        
        if (valleys.Count == 0)
            return null;
        
        // Find best valley (closest to goal direction)
        int goalSector = AngleToSector(goalAngle);
        double bestCost = double.MaxValue;
        double bestDirection = goalAngle;
        
        foreach (var (start, end) in valleys)
        {
            // Valley center
            int valleySize = (end - start + _numSectors) % _numSectors + 1;
            int center = (start + valleySize / 2) % _numSectors;
            
            double centerAngle = SectorToAngle(center);
            double cost = AngleDifference(centerAngle, goalAngle);
            
            if (cost < bestCost)
            {
                bestCost = cost;
                
                // Choose near or far edge based on goal
                if (valleySize > 10) // Wide opening
                {
                    bestDirection = goalAngle; // Can go directly towards goal
                }
                else
                {
                    bestDirection = centerAngle;
                }
            }
        }
        
        return bestDirection;
    }

    /// <summary>
    /// Gets the histogram values for visualization.
    /// </summary>
    public double[] GetHistogram() => (double[])_histogram.Clone();

    private int AngleToSector(double angle)
    {
        // Normalize to [0, 2*PI]
        while (angle < 0) angle += 2 * System.Math.PI;
        while (angle >= 2 * System.Math.PI) angle -= 2 * System.Math.PI;
        
        return (int)(angle / _sectorWidth) % _numSectors;
    }

    private double SectorToAngle(int sector)
    {
        return (sector + 0.5) * _sectorWidth;
    }

    private static double AngleDifference(double a, double b)
    {
        double diff = System.Math.Abs(a - b);
        return System.Math.Min(diff, 2 * System.Math.PI - diff);
    }
}

/// <summary>
/// Dynamic Window Approach for velocity selection.
/// </summary>
public class DynamicWindowApproach
{
    private readonly DwaConfig _config;

    public DynamicWindowApproach(DwaConfig? config = null)
    {
        _config = config ?? DwaConfig.Default;
    }

    /// <summary>
    /// Computes optimal velocity command.
    /// </summary>
    public (double linearVel, double angularVel) ComputeVelocity(
        double currentV,
        double currentW,
        double goalX,
        double goalY,
        double goalTheta,
        OccupancyGrid2D grid,
        double posX,
        double posY,
        double heading)
    {
        // Dynamic window
        double minV = System.Math.Max(_config.MinVelocity, currentV - _config.MaxAccel * _config.Dt);
        double maxV = System.Math.Min(_config.MaxVelocity, currentV + _config.MaxAccel * _config.Dt);
        double minW = System.Math.Max(-_config.MaxYawRate, currentW - _config.MaxYawAccel * _config.Dt);
        double maxW = System.Math.Min(_config.MaxYawRate, currentW + _config.MaxYawAccel * _config.Dt);
        
        double bestV = 0, bestW = 0;
        double bestCost = double.MinValue;
        
        // Sample velocities
        for (double v = minV; v <= maxV; v += _config.VelocityResolution)
        {
            for (double w = minW; w <= maxW; w += _config.YawRateResolution)
            {
                // Simulate trajectory
                var trajectory = SimulateTrajectory(posX, posY, heading, v, w);
                
                // Check for collision
                double distToObstacle = MinDistanceToObstacle(trajectory, grid);
                if (distToObstacle < _config.RobotRadius)
                    continue;
                
                // Compute costs
                double headingCost = ComputeHeadingCost(trajectory, goalX, goalY);
                double velocityCost = v / _config.MaxVelocity;
                double obstacleCost = distToObstacle / _config.MaxObstacleDistance;
                
                double totalCost = _config.HeadingWeight * headingCost +
                                   _config.VelocityWeight * velocityCost +
                                   _config.ObstacleWeight * obstacleCost;
                
                if (totalCost > bestCost)
                {
                    bestCost = totalCost;
                    bestV = v;
                    bestW = w;
                }
            }
        }
        
        return (bestV, bestW);
    }

    private List<(double x, double y, double theta)> SimulateTrajectory(
        double x, double y, double theta, double v, double w)
    {
        var trajectory = new List<(double, double, double)>();
        
        for (double t = 0; t <= _config.PredictTime; t += _config.Dt)
        {
            trajectory.Add((x, y, theta));
            x += v * System.Math.Cos(theta) * _config.Dt;
            y += v * System.Math.Sin(theta) * _config.Dt;
            theta += w * _config.Dt;
        }
        
        return trajectory;
    }

    private double MinDistanceToObstacle(
        List<(double x, double y, double theta)> trajectory,
        OccupancyGrid2D grid)
    {
        double minDist = double.MaxValue;
        
        foreach (var (x, y, _) in trajectory)
        {
            var obstacle = grid.FindNearestObstacle(x, y, _config.MaxObstacleDistance);
            if (obstacle != null)
            {
                minDist = System.Math.Min(minDist, obstacle.Value.distance);
            }
        }
        
        return minDist;
    }

    private double ComputeHeadingCost(
        List<(double x, double y, double theta)> trajectory,
        double goalX, double goalY)
    {
        if (trajectory.Count == 0)
            return 0;
        
        var (lastX, lastY, lastTheta) = trajectory[^1];
        double goalAngle = System.Math.Atan2(goalY - lastY, goalX - lastX);
        double headingError = System.Math.Abs(lastTheta - goalAngle);
        headingError = System.Math.Min(headingError, 2 * System.Math.PI - headingError);
        
        return (System.Math.PI - headingError) / System.Math.PI;
    }
}

/// <summary>
/// Configuration for Dynamic Window Approach.
/// </summary>
public class DwaConfig
{
    public double MaxVelocity { get; set; } = 1.0;
    public double MinVelocity { get; set; } = 0.0;
    public double MaxYawRate { get; set; } = 1.0;
    public double MaxAccel { get; set; } = 0.5;
    public double MaxYawAccel { get; set; } = 2.0;
    public double VelocityResolution { get; set; } = 0.1;
    public double YawRateResolution { get; set; } = 0.1;
    public double Dt { get; set; } = 0.1;
    public double PredictTime { get; set; } = 3.0;
    public double RobotRadius { get; set; } = 0.3;
    public double MaxObstacleDistance { get; set; } = 3.0;
    
    public double HeadingWeight { get; set; } = 1.0;
    public double VelocityWeight { get; set; } = 1.0;
    public double ObstacleWeight { get; set; } = 1.0;
    
    public static DwaConfig Default => new();
}
