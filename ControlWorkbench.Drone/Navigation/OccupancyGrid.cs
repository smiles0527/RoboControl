using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Drone.Navigation;

/// <summary>
/// 2D Occupancy Grid Map for navigation and obstacle avoidance.
/// Implements log-odds probabilistic updates similar to ArduPilot/PX4 avoidance systems.
/// </summary>
public class OccupancyGrid2D
{
    private readonly float[,] _logOdds;
    private readonly int _width;
    private readonly int _height;
    private readonly double _resolution;
    private readonly double _originX;
    private readonly double _originY;
    
    // Log-odds parameters
    private readonly float _logOddsOccupied;
    private readonly float _logOddsFree;
    private readonly float _logOddsMax;
    private readonly float _logOddsMin;
    private readonly float _logOddsPrior;
    
    private long _lastUpdateUs;
    private int _updateCount;

    /// <summary>
    /// Creates a new 2D occupancy grid.
    /// </summary>
    /// <param name="widthMeters">Width of the map in meters</param>
    /// <param name="heightMeters">Height of the map in meters</param>
    /// <param name="resolution">Cell size in meters</param>
    /// <param name="originX">X coordinate of map origin (bottom-left)</param>
    /// <param name="originY">Y coordinate of map origin (bottom-left)</param>
    /// <param name="config">Grid configuration parameters</param>
    public OccupancyGrid2D(
        double widthMeters,
        double heightMeters,
        double resolution,
        double originX = 0,
        double originY = 0,
        OccupancyGridConfig? config = null)
    {
        config ??= OccupancyGridConfig.Default;
        
        _resolution = resolution;
        _originX = originX;
        _originY = originY;
        _width = (int)System.Math.Ceiling(widthMeters / resolution);
        _height = (int)System.Math.Ceiling(heightMeters / resolution);
        
        _logOdds = new float[_width, _height];
        
        // Convert probabilities to log-odds
        _logOddsOccupied = ProbabilityToLogOdds(config.ProbOccupied);
        _logOddsFree = ProbabilityToLogOdds(config.ProbFree);
        _logOddsMax = ProbabilityToLogOdds(config.ProbMax);
        _logOddsMin = ProbabilityToLogOdds(config.ProbMin);
        _logOddsPrior = ProbabilityToLogOdds(config.ProbPrior);
        
        // Initialize with prior
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _logOdds[x, y] = _logOddsPrior;
            }
        }
    }

    /// <summary>
    /// Width of the grid in cells.
    /// </summary>
    public int Width => _width;
    
    /// <summary>
    /// Height of the grid in cells.
    /// </summary>
    public int Height => _height;
    
    /// <summary>
    /// Resolution in meters per cell.
    /// </summary>
    public double Resolution => _resolution;
    
    /// <summary>
    /// Origin X coordinate in world frame.
    /// </summary>
    public double OriginX => _originX;
    
    /// <summary>
    /// Origin Y coordinate in world frame.
    /// </summary>
    public double OriginY => _originY;

    /// <summary>
    /// Updates the grid with a range sensor measurement using ray casting.
    /// </summary>
    /// <param name="sensorX">Sensor X position in world frame</param>
    /// <param name="sensorY">Sensor Y position in world frame</param>
    /// <param name="angle">Beam angle in radians (world frame)</param>
    /// <param name="range">Measured range in meters</param>
    /// <param name="maxRange">Maximum sensor range</param>
    /// <param name="timestampUs">Measurement timestamp</param>
    public void UpdateWithRay(
        double sensorX, double sensorY,
        double angle, double range, double maxRange,
        long timestampUs = 0)
    {
        // Endpoint of the ray
        bool hitObstacle = range < maxRange;
        double endX = sensorX + range * System.Math.Cos(angle);
        double endY = sensorY + range * System.Math.Sin(angle);
        
        // Convert to grid coordinates
        var (startGridX, startGridY) = WorldToGrid(sensorX, sensorY);
        var (endGridX, endGridY) = WorldToGrid(endX, endY);
        
        // Ray cast using Bresenham's algorithm
        var cells = BresenhamLine(startGridX, startGridY, endGridX, endGridY);
        
        // Update cells along the ray as free (except the last one if hit)
        for (int i = 0; i < cells.Count - 1; i++)
        {
            var (cx, cy) = cells[i];
            if (IsInBounds(cx, cy))
            {
                UpdateCell(cx, cy, false);
            }
        }
        
        // Update endpoint as occupied if hit, free otherwise
        if (cells.Count > 0)
        {
            var (lastX, lastY) = cells[^1];
            if (IsInBounds(lastX, lastY))
            {
                UpdateCell(lastX, lastY, hitObstacle);
            }
        }
        
        _lastUpdateUs = timestampUs;
        _updateCount++;
    }

    /// <summary>
    /// Updates the grid with a point cloud from a 2D lidar scan.
    /// </summary>
    public void UpdateWithLidarScan(
        double sensorX, double sensorY, double sensorYaw,
        LidarScan2D scan,
        long timestampUs = 0)
    {
        for (int i = 0; i < scan.Ranges.Length; i++)
        {
            double range = scan.Ranges[i];
            if (range < scan.MinRange || range > scan.MaxRange)
                continue;
            
            double beamAngle = sensorYaw + scan.AngleMin + i * scan.AngleIncrement;
            UpdateWithRay(sensorX, sensorY, beamAngle, range, scan.MaxRange, timestampUs);
        }
    }

    /// <summary>
    /// Updates the grid with a depth camera image.
    /// </summary>
    public void UpdateWithDepthImage(
        double sensorX, double sensorY, double sensorYaw,
        DepthImageSlice depthSlice,
        long timestampUs = 0)
    {
        double fovRad = depthSlice.HorizontalFovDeg * System.Math.PI / 180.0;
        double anglePerPixel = fovRad / depthSlice.Depths.Length;
        double startAngle = sensorYaw - fovRad / 2;
        
        for (int i = 0; i < depthSlice.Depths.Length; i++)
        {
            double depth = depthSlice.Depths[i];
            if (depth < depthSlice.MinDepth || depth > depthSlice.MaxDepth)
                continue;
            
            double beamAngle = startAngle + i * anglePerPixel;
            UpdateWithRay(sensorX, sensorY, beamAngle, depth, depthSlice.MaxDepth, timestampUs);
        }
    }

    /// <summary>
    /// Gets the occupancy probability of a cell.
    /// </summary>
    public double GetProbability(int gridX, int gridY)
    {
        if (!IsInBounds(gridX, gridY))
            return 0.5; // Unknown
        
        return LogOddsToProbability(_logOdds[gridX, gridY]);
    }

    /// <summary>
    /// Gets the occupancy probability at a world position.
    /// </summary>
    public double GetProbabilityAtWorld(double worldX, double worldY)
    {
        var (gx, gy) = WorldToGrid(worldX, worldY);
        return GetProbability(gx, gy);
    }

    /// <summary>
    /// Checks if a cell is occupied (probability > threshold).
    /// </summary>
    public bool IsOccupied(int gridX, int gridY, double threshold = 0.65)
    {
        return GetProbability(gridX, gridY) > threshold;
    }

    /// <summary>
    /// Checks if a cell is free (probability < threshold).
    /// </summary>
    public bool IsFree(int gridX, int gridY, double threshold = 0.35)
    {
        return GetProbability(gridX, gridY) < threshold;
    }

    /// <summary>
    /// Checks if a straight-line path is collision-free.
    /// </summary>
    public bool IsPathFree(
        double startX, double startY,
        double endX, double endY,
        double robotRadius = 0,
        double threshold = 0.65)
    {
        var (gsx, gsy) = WorldToGrid(startX, startY);
        var (gex, gey) = WorldToGrid(endX, endY);
        
        var cells = BresenhamLine(gsx, gsy, gex, gey);
        int radiusCells = (int)System.Math.Ceiling(robotRadius / _resolution);
        
        foreach (var (cx, cy) in cells)
        {
            // Check cell and surrounding cells for robot radius
            for (int dx = -radiusCells; dx <= radiusCells; dx++)
            {
                for (int dy = -radiusCells; dy <= radiusCells; dy++)
                {
                    if (dx * dx + dy * dy <= radiusCells * radiusCells)
                    {
                        if (IsOccupied(cx + dx, cy + dy, threshold))
                            return false;
                    }
                }
            }
        }
        
        return true;
    }

    /// <summary>
    /// Finds the nearest obstacle from a given position.
    /// </summary>
    public (double distance, double angle)? FindNearestObstacle(
        double worldX, double worldY,
        double searchRadius,
        double threshold = 0.65)
    {
        var (cx, cy) = WorldToGrid(worldX, worldY);
        int searchCells = (int)System.Math.Ceiling(searchRadius / _resolution);
        
        double minDistance = double.MaxValue;
        double obstacleAngle = 0;
        bool found = false;
        
        for (int dx = -searchCells; dx <= searchCells; dx++)
        {
            for (int dy = -searchCells; dy <= searchCells; dy++)
            {
                int gx = cx + dx;
                int gy = cy + dy;
                
                if (IsOccupied(gx, gy, threshold))
                {
                    var (ox, oy) = GridToWorld(gx, gy);
                    double dist = System.Math.Sqrt((ox - worldX) * (ox - worldX) + (oy - worldY) * (oy - worldY));
                    
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        obstacleAngle = System.Math.Atan2(oy - worldY, ox - worldX);
                        found = true;
                    }
                }
            }
        }
        
        return found ? (minDistance, obstacleAngle) : null;
    }

    /// <summary>
    /// Gets a distance field (distance to nearest obstacle for each cell).
    /// </summary>
    public float[,] ComputeDistanceField(double threshold = 0.65)
    {
        var distanceField = new float[_width, _height];
        var queue = new Queue<(int x, int y)>();
        
        // Initialize: obstacles = 0, others = infinity
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                if (IsOccupied(x, y, threshold))
                {
                    distanceField[x, y] = 0;
                    queue.Enqueue((x, y));
                }
                else
                {
                    distanceField[x, y] = float.MaxValue;
                }
            }
        }
        
        // BFS to compute distances
        int[] dx = [-1, 0, 1, 0, -1, -1, 1, 1];
        int[] dy = [0, -1, 0, 1, -1, 1, -1, 1];
        float[] costs = [1, 1, 1, 1, 1.414f, 1.414f, 1.414f, 1.414f];
        
        while (queue.Count > 0)
        {
            var (cx, cy) = queue.Dequeue();
            
            for (int i = 0; i < 8; i++)
            {
                int nx = cx + dx[i];
                int ny = cy + dy[i];
                
                if (IsInBounds(nx, ny))
                {
                    float newDist = distanceField[cx, cy] + costs[i];
                    if (newDist < distanceField[nx, ny])
                    {
                        distanceField[nx, ny] = newDist;
                        queue.Enqueue((nx, ny));
                    }
                }
            }
        }
        
        // Convert to meters
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                distanceField[x, y] *= (float)_resolution;
            }
        }
        
        return distanceField;
    }

    /// <summary>
    /// Inflates obstacles by a given radius (for path planning with robot footprint).
    /// </summary>
    public OccupancyGrid2D GetInflatedGrid(double inflationRadius, double threshold = 0.65)
    {
        var inflated = new OccupancyGrid2D(
            _width * _resolution,
            _height * _resolution,
            _resolution,
            _originX,
            _originY);
        
        int radiusCells = (int)System.Math.Ceiling(inflationRadius / _resolution);
        
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                if (IsOccupied(x, y, threshold))
                {
                    // Inflate this obstacle
                    for (int dx = -radiusCells; dx <= radiusCells; dx++)
                    {
                        for (int dy = -radiusCells; dy <= radiusCells; dy++)
                        {
                            if (dx * dx + dy * dy <= radiusCells * radiusCells)
                            {
                                int nx = x + dx;
                                int ny = y + dy;
                                if (inflated.IsInBounds(nx, ny))
                                {
                                    inflated._logOdds[nx, ny] = inflated._logOddsMax;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        return inflated;
    }

    /// <summary>
    /// Clears all cells to unknown (prior probability).
    /// </summary>
    public void Clear()
    {
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _logOdds[x, y] = _logOddsPrior;
            }
        }
        _updateCount = 0;
    }

    /// <summary>
    /// Applies decay to all cells (moves towards prior).
    /// </summary>
    public void ApplyDecay(double decayFactor)
    {
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _logOdds[x, y] = (float)(_logOddsPrior + decayFactor * (_logOdds[x, y] - _logOddsPrior));
            }
        }
    }

    /// <summary>
    /// Converts world coordinates to grid coordinates.
    /// </summary>
    public (int gridX, int gridY) WorldToGrid(double worldX, double worldY)
    {
        int gx = (int)System.Math.Floor((worldX - _originX) / _resolution);
        int gy = (int)System.Math.Floor((worldY - _originY) / _resolution);
        return (gx, gy);
    }

    /// <summary>
    /// Converts grid coordinates to world coordinates (cell center).
    /// </summary>
    public (double worldX, double worldY) GridToWorld(int gridX, int gridY)
    {
        double wx = _originX + (gridX + 0.5) * _resolution;
        double wy = _originY + (gridY + 0.5) * _resolution;
        return (wx, wy);
    }

    /// <summary>
    /// Checks if grid coordinates are within bounds.
    /// </summary>
    public bool IsInBounds(int gridX, int gridY)
    {
        return gridX >= 0 && gridX < _width && gridY >= 0 && gridY < _height;
    }

    /// <summary>
    /// Gets the raw log-odds array for serialization or visualization.
    /// </summary>
    public float[,] GetLogOddsArray() => (float[,])_logOdds.Clone();

    /// <summary>
    /// Gets the probability array for visualization.
    /// </summary>
    public double[,] GetProbabilityArray()
    {
        var probs = new double[_width, _height];
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                probs[x, y] = LogOddsToProbability(_logOdds[x, y]);
            }
        }
        return probs;
    }

    private void UpdateCell(int gridX, int gridY, bool occupied)
    {
        float update = occupied ? _logOddsOccupied : _logOddsFree;
        _logOdds[gridX, gridY] = System.Math.Clamp(
            _logOdds[gridX, gridY] + update - _logOddsPrior,
            _logOddsMin,
            _logOddsMax);
    }

    private static float ProbabilityToLogOdds(double p)
    {
        p = System.Math.Clamp(p, 0.001, 0.999);
        return (float)System.Math.Log(p / (1 - p));
    }

    private static double LogOddsToProbability(float logOdds)
    {
        return 1.0 / (1.0 + System.Math.Exp(-logOdds));
    }

    private static List<(int x, int y)> BresenhamLine(int x0, int y0, int x1, int y1)
    {
        var cells = new List<(int, int)>();
        
        int dx = System.Math.Abs(x1 - x0);
        int dy = System.Math.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        while (true)
        {
            cells.Add((x0, y0));
            
            if (x0 == x1 && y0 == y1)
                break;
            
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
        
        return cells;
    }
}

/// <summary>
/// 3D Occupancy Grid (Voxel Grid) for 3D obstacle mapping.
/// Used for full 3D planning and obstacle avoidance.
/// </summary>
public class OccupancyGrid3D
{
    private readonly float[,,] _logOdds;
    private readonly int _sizeX, _sizeY, _sizeZ;
    private readonly double _resolution;
    private readonly Vector<double> _origin;
    
    private readonly float _logOddsOccupied;
    private readonly float _logOddsFree;
    private readonly float _logOddsMax;
    private readonly float _logOddsMin;
    private readonly float _logOddsPrior;

    /// <summary>
    /// Creates a new 3D occupancy grid.
    /// </summary>
    public OccupancyGrid3D(
        double sizeXMeters,
        double sizeYMeters,
        double sizeZMeters,
        double resolution,
        Vector<double>? origin = null,
        OccupancyGridConfig? config = null)
    {
        config ??= OccupancyGridConfig.Default;
        
        _resolution = resolution;
        _origin = origin ?? Vector<double>.Build.Dense([0, 0, 0]);
        _sizeX = (int)System.Math.Ceiling(sizeXMeters / resolution);
        _sizeY = (int)System.Math.Ceiling(sizeYMeters / resolution);
        _sizeZ = (int)System.Math.Ceiling(sizeZMeters / resolution);
        
        _logOdds = new float[_sizeX, _sizeY, _sizeZ];
        
        _logOddsOccupied = ProbabilityToLogOdds(config.ProbOccupied);
        _logOddsFree = ProbabilityToLogOdds(config.ProbFree);
        _logOddsMax = ProbabilityToLogOdds(config.ProbMax);
        _logOddsMin = ProbabilityToLogOdds(config.ProbMin);
        _logOddsPrior = ProbabilityToLogOdds(config.ProbPrior);
        
        // Initialize with prior
        for (int x = 0; x < _sizeX; x++)
        {
            for (int y = 0; y < _sizeY; y++)
            {
                for (int z = 0; z < _sizeZ; z++)
                {
                    _logOdds[x, y, z] = _logOddsPrior;
                }
            }
        }
    }

    public int SizeX => _sizeX;
    public int SizeY => _sizeY;
    public int SizeZ => _sizeZ;
    public double Resolution => _resolution;

    /// <summary>
    /// Updates the grid with a 3D point cloud.
    /// </summary>
    public void UpdateWithPointCloud(
        Vector<double> sensorPosition,
        IEnumerable<Vector<double>> points,
        double maxRange)
    {
        var sensorGrid = WorldToGrid(sensorPosition);
        
        foreach (var point in points)
        {
            double dist = (point - sensorPosition).L2Norm();
            if (dist > maxRange || dist < 0.1)
                continue;
            
            var pointGrid = WorldToGrid(point);
            
            // Ray cast from sensor to point
            var cells = Bresenham3D(
                sensorGrid.x, sensorGrid.y, sensorGrid.z,
                pointGrid.x, pointGrid.y, pointGrid.z);
            
            // Mark cells along ray as free (except last one)
            for (int i = 0; i < cells.Count - 1; i++)
            {
                var (cx, cy, cz) = cells[i];
                if (IsInBounds(cx, cy, cz))
                {
                    UpdateCell(cx, cy, cz, false);
                }
            }
            
            // Mark endpoint as occupied
            if (cells.Count > 0)
            {
                var (lx, ly, lz) = cells[^1];
                if (IsInBounds(lx, ly, lz))
                {
                    UpdateCell(lx, ly, lz, true);
                }
            }
        }
    }

    /// <summary>
    /// Updates the grid with a depth camera frame.
    /// </summary>
    public void UpdateWithDepthImage(
        Vector<double> cameraPosition,
        Matrix<double> cameraRotation,
        float[,] depthImage,
        DepthCameraIntrinsics intrinsics)
    {
        int height = depthImage.GetLength(0);
        int width = depthImage.GetLength(1);
        
        // Subsample for efficiency
        int step = System.Math.Max(1, System.Math.Min(width, height) / 80);
        
        for (int v = 0; v < height; v += step)
        {
            for (int u = 0; u < width; u += step)
            {
                float depth = depthImage[v, u];
                if (depth <= intrinsics.MinDepth || depth >= intrinsics.MaxDepth)
                    continue;
                
                // Deproject to camera frame
                double x = (u - intrinsics.Cx) * depth / intrinsics.Fx;
                double y = (v - intrinsics.Cy) * depth / intrinsics.Fy;
                double z = depth;
                
                // Transform to world frame
                var pointCamera = Vector<double>.Build.Dense([x, y, z]);
                var pointWorld = cameraPosition + cameraRotation * pointCamera;
                
                // Ray cast
                var sensorGrid = WorldToGrid(cameraPosition);
                var pointGrid = WorldToGrid(pointWorld);
                
                var cells = Bresenham3D(
                    sensorGrid.x, sensorGrid.y, sensorGrid.z,
                    pointGrid.x, pointGrid.y, pointGrid.z);
                
                for (int i = 0; i < cells.Count - 1; i++)
                {
                    var (cx, cy, cz) = cells[i];
                    if (IsInBounds(cx, cy, cz))
                        UpdateCell(cx, cy, cz, false);
                }
                
                if (cells.Count > 0)
                {
                    var (lx, ly, lz) = cells[^1];
                    if (IsInBounds(lx, ly, lz))
                        UpdateCell(lx, ly, lz, true);
                }
            }
        }
    }

    /// <summary>
    /// Gets occupancy probability at grid coordinates.
    /// </summary>
    public double GetProbability(int gx, int gy, int gz)
    {
        if (!IsInBounds(gx, gy, gz))
            return 0.5;
        return LogOddsToProbability(_logOdds[gx, gy, gz]);
    }

    /// <summary>
    /// Checks if a cell is occupied.
    /// </summary>
    public bool IsOccupied(int gx, int gy, int gz, double threshold = 0.65)
    {
        return GetProbability(gx, gy, gz) > threshold;
    }

    /// <summary>
    /// Checks if a 3D path is collision-free.
    /// </summary>
    public bool IsPathFree(
        Vector<double> start,
        Vector<double> end,
        double robotRadius = 0,
        double threshold = 0.65)
    {
        var (sx, sy, sz) = WorldToGrid(start);
        var (ex, ey, ez) = WorldToGrid(end);
        
        var cells = Bresenham3D(sx, sy, sz, ex, ey, ez);
        int radiusCells = (int)System.Math.Ceiling(robotRadius / _resolution);
        
        foreach (var (cx, cy, cz) in cells)
        {
            for (int dx = -radiusCells; dx <= radiusCells; dx++)
            {
                for (int dy = -radiusCells; dy <= radiusCells; dy++)
                {
                    for (int dz = -radiusCells; dz <= radiusCells; dz++)
                    {
                        if (dx * dx + dy * dy + dz * dz <= radiusCells * radiusCells)
                        {
                            if (IsOccupied(cx + dx, cy + dy, cz + dz, threshold))
                                return false;
                        }
                    }
                }
            }
        }
        
        return true;
    }

    /// <summary>
    /// Projects the 3D grid to a 2D height map (maximum occupancy per column).
    /// </summary>
    public OccupancyGrid2D ProjectToHeightMap(double threshold = 0.65)
    {
        var grid2D = new OccupancyGrid2D(
            _sizeX * _resolution,
            _sizeY * _resolution,
            _resolution,
            _origin[0],
            _origin[1]);
        
        for (int x = 0; x < _sizeX; x++)
        {
            for (int y = 0; y < _sizeY; y++)
            {
                double maxProb = 0;
                for (int z = 0; z < _sizeZ; z++)
                {
                    maxProb = System.Math.Max(maxProb, GetProbability(x, y, z));
                }
                
                // Update the 2D grid
                if (maxProb > threshold)
                {
                    var logOddsArray = grid2D.GetLogOddsArray();
                    logOddsArray[x, y] = ProbabilityToLogOdds(maxProb);
                }
            }
        }
        
        return grid2D;
    }

    /// <summary>
    /// Gets the occupied voxels as a list of world coordinates.
    /// </summary>
    public List<Vector<double>> GetOccupiedVoxels(double threshold = 0.65)
    {
        var voxels = new List<Vector<double>>();
        
        for (int x = 0; x < _sizeX; x++)
        {
            for (int y = 0; y < _sizeY; y++)
            {
                for (int z = 0; z < _sizeZ; z++)
                {
                    if (IsOccupied(x, y, z, threshold))
                    {
                        voxels.Add(GridToWorld(x, y, z));
                    }
                }
            }
        }
        
        return voxels;
    }

    /// <summary>
    /// Clears all voxels.
    /// </summary>
    public void Clear()
    {
        for (int x = 0; x < _sizeX; x++)
        {
            for (int y = 0; y < _sizeY; y++)
            {
                for (int z = 0; z < _sizeZ; z++)
                {
                    _logOdds[x, y, z] = _logOddsPrior;
                }
            }
        }
    }

    /// <summary>
    /// Converts world coordinates to grid coordinates.
    /// </summary>
    public (int x, int y, int z) WorldToGrid(Vector<double> world)
    {
        int gx = (int)System.Math.Floor((world[0] - _origin[0]) / _resolution);
        int gy = (int)System.Math.Floor((world[1] - _origin[1]) / _resolution);
        int gz = (int)System.Math.Floor((world[2] - _origin[2]) / _resolution);
        return (gx, gy, gz);
    }

    /// <summary>
    /// Converts grid coordinates to world coordinates (voxel center).
    /// </summary>
    public Vector<double> GridToWorld(int gx, int gy, int gz)
    {
        return Vector<double>.Build.Dense([
            _origin[0] + (gx + 0.5) * _resolution,
            _origin[1] + (gy + 0.5) * _resolution,
            _origin[2] + (gz + 0.5) * _resolution
        ]);
    }

    /// <summary>
    /// Checks if coordinates are within bounds.
    /// </summary>
    public bool IsInBounds(int gx, int gy, int gz)
    {
        return gx >= 0 && gx < _sizeX && 
               gy >= 0 && gy < _sizeY && 
               gz >= 0 && gz < _sizeZ;
    }

    private void UpdateCell(int gx, int gy, int gz, bool occupied)
    {
        float update = occupied ? _logOddsOccupied : _logOddsFree;
        _logOdds[gx, gy, gz] = System.Math.Clamp(
            _logOdds[gx, gy, gz] + update - _logOddsPrior,
            _logOddsMin,
            _logOddsMax);
    }

    private static float ProbabilityToLogOdds(double p)
    {
        p = System.Math.Clamp(p, 0.001, 0.999);
        return (float)System.Math.Log(p / (1 - p));
    }

    private static double LogOddsToProbability(float logOdds)
    {
        return 1.0 / (1.0 + System.Math.Exp(-logOdds));
    }

    private static List<(int x, int y, int z)> Bresenham3D(
        int x0, int y0, int z0, int x1, int y1, int z1)
    {
        var cells = new List<(int, int, int)>();
        
        int dx = System.Math.Abs(x1 - x0);
        int dy = System.Math.Abs(y1 - y0);
        int dz = System.Math.Abs(z1 - z0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int sz = z0 < z1 ? 1 : -1;
        
        int dm = System.Math.Max(dx, System.Math.Max(dy, dz));
        int x = x0, y = y0, z = z0;
        int errX = dm / 2, errY = dm / 2, errZ = dm / 2;
        
        for (int i = 0; i <= dm; i++)
        {
            cells.Add((x, y, z));
            
            errX -= dx;
            if (errX < 0)
            {
                errX += dm;
                x += sx;
            }
            
            errY -= dy;
            if (errY < 0)
            {
                errY += dm;
                y += sy;
            }
            
            errZ -= dz;
            if (errZ < 0)
            {
                errZ += dm;
                z += sz;
            }
        }
        
        return cells;
    }
}

/// <summary>
/// Configuration for occupancy grids.
/// </summary>
public class OccupancyGridConfig
{
    /// <summary>Probability to assign for occupied observations.</summary>
    public double ProbOccupied { get; set; } = 0.7;
    
    /// <summary>Probability to assign for free observations.</summary>
    public double ProbFree { get; set; } = 0.4;
    
    /// <summary>Maximum probability (clamping).</summary>
    public double ProbMax { get; set; } = 0.97;
    
    /// <summary>Minimum probability (clamping).</summary>
    public double ProbMin { get; set; } = 0.03;
    
    /// <summary>Prior probability (unknown).</summary>
    public double ProbPrior { get; set; } = 0.5;
    
    public static OccupancyGridConfig Default => new();
    
    /// <summary>
    /// Configuration optimized for indoor environments.
    /// </summary>
    public static OccupancyGridConfig Indoor => new()
    {
        ProbOccupied = 0.75,
        ProbFree = 0.35,
        ProbMax = 0.98,
        ProbMin = 0.02
    };
    
    /// <summary>
    /// Configuration optimized for outdoor environments.
    /// </summary>
    public static OccupancyGridConfig Outdoor => new()
    {
        ProbOccupied = 0.65,
        ProbFree = 0.45,
        ProbMax = 0.95,
        ProbMin = 0.05
    };
}

#region Sensor Data Structures

/// <summary>
/// 2D lidar scan data.
/// </summary>
public class LidarScan2D
{
    /// <summary>Range measurements in meters.</summary>
    public double[] Ranges { get; init; } = [];
    
    /// <summary>Intensity values (optional).</summary>
    public double[]? Intensities { get; init; }
    
    /// <summary>Minimum scan angle in radians.</summary>
    public double AngleMin { get; init; }
    
    /// <summary>Maximum scan angle in radians.</summary>
    public double AngleMax { get; init; }
    
    /// <summary>Angular resolution in radians.</summary>
    public double AngleIncrement { get; init; }
    
    /// <summary>Minimum valid range.</summary>
    public double MinRange { get; init; } = 0.1;
    
    /// <summary>Maximum valid range.</summary>
    public double MaxRange { get; init; } = 30.0;
    
    /// <summary>Timestamp in microseconds.</summary>
    public long TimestampUs { get; init; }
}

/// <summary>
/// Single row/slice of depth image for 2D mapping.
/// </summary>
public class DepthImageSlice
{
    /// <summary>Depth values in meters.</summary>
    public double[] Depths { get; init; } = [];
    
    /// <summary>Horizontal field of view in degrees.</summary>
    public double HorizontalFovDeg { get; init; } = 60;
    
    /// <summary>Minimum valid depth.</summary>
    public double MinDepth { get; init; } = 0.1;
    
    /// <summary>Maximum valid depth.</summary>
    public double MaxDepth { get; init; } = 10.0;
}

/// <summary>
/// Depth camera intrinsic parameters.
/// </summary>
public class DepthCameraIntrinsics
{
    /// <summary>Focal length in x (pixels).</summary>
    public double Fx { get; init; }
    
    /// <summary>Focal length in y (pixels).</summary>
    public double Fy { get; init; }
    
    /// <summary>Principal point x (pixels).</summary>
    public double Cx { get; init; }
    
    /// <summary>Principal point y (pixels).</summary>
    public double Cy { get; init; }
    
    /// <summary>Image width in pixels.</summary>
    public int Width { get; init; }
    
    /// <summary>Image height in pixels.</summary>
    public int Height { get; init; }
    
    /// <summary>Minimum valid depth.</summary>
    public double MinDepth { get; init; } = 0.1;
    
    /// <summary>Maximum valid depth.</summary>
    public double MaxDepth { get; init; } = 10.0;
    
    /// <summary>
    /// Creates intrinsics for Intel RealSense D435.
    /// </summary>
    public static DepthCameraIntrinsics RealSenseD435 => new()
    {
        Fx = 386.0, Fy = 386.0,
        Cx = 320.0, Cy = 240.0,
        Width = 640, Height = 480,
        MinDepth = 0.1, MaxDepth = 10.0
    };
    
    /// <summary>
    /// Creates intrinsics for Intel RealSense D455.
    /// </summary>
    public static DepthCameraIntrinsics RealSenseD455 => new()
    {
        Fx = 390.0, Fy = 390.0,
        Cx = 320.0, Cy = 240.0,
        Width = 640, Height = 480,
        MinDepth = 0.4, MaxDepth = 20.0
    };
}

#endregion

/// <summary>
/// Cost map for path planning with configurable cost layers.
/// </summary>
public class CostMap2D
{
    private readonly float[,] _costs;
    private readonly int _width;
    private readonly int _height;
    private readonly double _resolution;
    private readonly double _originX;
    private readonly double _originY;
    
    private readonly List<CostLayer> _layers = new();
    
    public const float CostFree = 0f;
    public const float CostUnknown = 128f;
    public const float CostLethal = 254f;
    public const float CostNoInfo = 255f;

    public CostMap2D(
        double widthMeters,
        double heightMeters,
        double resolution,
        double originX = 0,
        double originY = 0)
    {
        _resolution = resolution;
        _originX = originX;
        _originY = originY;
        _width = (int)System.Math.Ceiling(widthMeters / resolution);
        _height = (int)System.Math.Ceiling(heightMeters / resolution);
        _costs = new float[_width, _height];
        
        // Initialize as no info
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _costs[x, y] = CostNoInfo;
            }
        }
    }

    public int Width => _width;
    public int Height => _height;
    public double Resolution => _resolution;

    /// <summary>
    /// Adds a cost layer (e.g., static obstacles, inflation, preferences).
    /// </summary>
    public void AddLayer(CostLayer layer)
    {
        _layers.Add(layer);
    }

    /// <summary>
    /// Updates the master cost map from all layers.
    /// </summary>
    public void UpdateFromLayers()
    {
        // Reset to free
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _costs[x, y] = CostFree;
            }
        }
        
        // Apply each layer
        foreach (var layer in _layers)
        {
            for (int x = 0; x < _width; x++)
            {
                for (int y = 0; y < _height; y++)
                {
                    float layerCost = layer.GetCost(x, y);
                    _costs[x, y] = System.Math.Max(_costs[x, y], layerCost);
                }
            }
        }
    }

    /// <summary>
    /// Updates from an occupancy grid.
    /// </summary>
    public void UpdateFromOccupancyGrid(
        OccupancyGrid2D occupancyGrid,
        double inflationRadius,
        double inscribedRadius)
    {
        int inflationCells = (int)System.Math.Ceiling(inflationRadius / _resolution);
        int inscribedCells = (int)System.Math.Ceiling(inscribedRadius / _resolution);
        
        // First pass: mark lethal and inscribed obstacles
        for (int x = 0; x < _width && x < occupancyGrid.Width; x++)
        {
            for (int y = 0; y < _height && y < occupancyGrid.Height; y++)
            {
                double prob = occupancyGrid.GetProbability(x, y);
                
                if (prob > 0.65) // Occupied
                {
                    _costs[x, y] = CostLethal;
                }
                else if (prob < 0.35) // Free
                {
                    _costs[x, y] = CostFree;
                }
                else // Unknown
                {
                    _costs[x, y] = CostUnknown;
                }
            }
        }
        
        // Second pass: inflate obstacles
        var tempCosts = (float[,])_costs.Clone();
        
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                if (tempCosts[x, y] == CostLethal)
                {
                    // Inflate around this cell
                    for (int dx = -inflationCells; dx <= inflationCells; dx++)
                    {
                        for (int dy = -inflationCells; dy <= inflationCells; dy++)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if (nx < 0 || nx >= _width || ny < 0 || ny >= _height)
                                continue;
                            
                            double dist = System.Math.Sqrt(dx * dx + dy * dy) * _resolution;
                            
                            if (dist <= inscribedRadius)
                            {
                                _costs[nx, ny] = CostLethal;
                            }
                            else if (dist <= inflationRadius)
                            {
                                // Exponential decay
                                float cost = (float)(CostLethal * System.Math.Exp(-3.0 * (dist - inscribedRadius) / (inflationRadius - inscribedRadius)));
                                _costs[nx, ny] = System.Math.Max(_costs[nx, ny], cost);
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Gets cost at grid coordinates.
    /// </summary>
    public float GetCost(int gx, int gy)
    {
        if (gx < 0 || gx >= _width || gy < 0 || gy >= _height)
            return CostNoInfo;
        return _costs[gx, gy];
    }

    /// <summary>
    /// Gets cost at world coordinates.
    /// </summary>
    public float GetCostAtWorld(double worldX, double worldY)
    {
        int gx = (int)System.Math.Floor((worldX - _originX) / _resolution);
        int gy = (int)System.Math.Floor((worldY - _originY) / _resolution);
        return GetCost(gx, gy);
    }

    /// <summary>
    /// Checks if a cell is traversable.
    /// </summary>
    public bool IsTraversable(int gx, int gy, float maxCost = CostLethal - 1)
    {
        return GetCost(gx, gy) <= maxCost;
    }

    /// <summary>
    /// Gets the cost array for visualization.
    /// </summary>
    public float[,] GetCostArray() => (float[,])_costs.Clone();

    /// <summary>
    /// Converts world to grid coordinates.
    /// </summary>
    public (int gx, int gy) WorldToGrid(double worldX, double worldY)
    {
        int gx = (int)System.Math.Floor((worldX - _originX) / _resolution);
        int gy = (int)System.Math.Floor((worldY - _originY) / _resolution);
        return (gx, gy);
    }

    /// <summary>
    /// Converts grid to world coordinates.
    /// </summary>
    public (double worldX, double worldY) GridToWorld(int gx, int gy)
    {
        double wx = _originX + (gx + 0.5) * _resolution;
        double wy = _originY + (gy + 0.5) * _resolution;
        return (wx, wy);
    }
}

/// <summary>
/// Abstract cost layer for the cost map.
/// </summary>
public abstract class CostLayer
{
    public abstract float GetCost(int gx, int gy);
    public virtual void Update() { }
}

/// <summary>
/// Static obstacle layer.
/// </summary>
public class StaticObstacleLayer : CostLayer
{
    private readonly float[,] _costs;
    
    public StaticObstacleLayer(int width, int height)
    {
        _costs = new float[width, height];
    }
    
    public void SetObstacle(int gx, int gy)
    {
        if (gx >= 0 && gx < _costs.GetLength(0) && gy >= 0 && gy < _costs.GetLength(1))
        {
            _costs[gx, gy] = CostMap2D.CostLethal;
        }
    }
    
    public void ClearObstacle(int gx, int gy)
    {
        if (gx >= 0 && gx < _costs.GetLength(0) && gy >= 0 && gy < _costs.GetLength(1))
        {
            _costs[gx, gy] = CostMap2D.CostFree;
        }
    }
    
    public override float GetCost(int gx, int gy)
    {
        if (gx >= 0 && gx < _costs.GetLength(0) && gy >= 0 && gy < _costs.GetLength(1))
        {
            return _costs[gx, gy];
        }
        return CostMap2D.CostNoInfo;
    }
}

/// <summary>
/// Proximity preference layer (e.g., prefer staying near walls for inspection).
/// </summary>
public class ProximityPreferenceLayer : CostLayer
{
    private readonly OccupancyGrid2D _occupancyGrid;
    private readonly double _preferredDistance;
    private readonly float _maxCost;
    private float[,]? _costs;
    
    public ProximityPreferenceLayer(
        OccupancyGrid2D occupancyGrid,
        double preferredDistance,
        float maxCost = 50f)
    {
        _occupancyGrid = occupancyGrid;
        _preferredDistance = preferredDistance;
        _maxCost = maxCost;
    }
    
    public override void Update()
    {
        _costs = _occupancyGrid.ComputeDistanceField();
    }
    
    public override float GetCost(int gx, int gy)
    {
        if (_costs == null) return 0;
        
        if (gx >= 0 && gx < _costs.GetLength(0) && gy >= 0 && gy < _costs.GetLength(1))
        {
            double dist = _costs[gx, gy];
            double diff = System.Math.Abs(dist - _preferredDistance);
            return (float)System.Math.Min(_maxCost, diff / _preferredDistance * _maxCost);
        }
        return 0;
    }
}
