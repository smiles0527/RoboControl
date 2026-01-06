using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Geometry;
using ControlWorkbench.Math.Optimization;

namespace ControlWorkbench.Drone.Swarm;

/// <summary>
/// Multi-Robot Coordination and Swarm Intelligence Framework.
/// Implements distributed consensus, formation control, task allocation, and cooperative SLAM.
/// 
/// Based on:
/// - "Distributed Consensus in Multi-Robot Systems" (Ren & Beard, 2008)
/// - "Multi-Robot SLAM with Unknown Initial Correspondences" (Thrun & Liu, 2005)
/// - "Decentralized MPC for Multi-Agent Systems" (Dunbar & Murray, 2006)
/// </summary>
public class SwarmCoordinator
{
    private readonly int _numAgents;
    private readonly SwarmConfig _config;
    private readonly Dictionary<int, AgentState> _agentStates = new();
    private readonly CommunicationGraph _commGraph;
    private readonly ConsensusController _consensus;
    private readonly FormationController _formation;
    private readonly TaskAllocator _taskAllocator;
    private readonly CooperativeSlamBackend _coopSlam;
    
    public int AgentCount => _numAgents;
    
    public SwarmCoordinator(int numAgents, SwarmConfig? config = null)
    {
        _numAgents = numAgents;
        _config = config ?? new SwarmConfig();
        
        for (int i = 0; i < numAgents; i++)
        {
            _agentStates[i] = new AgentState { AgentId = i };
        }
        
        _commGraph = new CommunicationGraph(numAgents, _config.CommunicationRadius);
        _consensus = new ConsensusController(_config);
        _formation = new FormationController(_config);
        _taskAllocator = new TaskAllocator(numAgents);
        _coopSlam = new CooperativeSlamBackend(numAgents);
    }
    
    #region State Management
    
    /// <summary>
    /// Update agent state.
    /// </summary>
    public void UpdateAgentState(int agentId, AgentState state)
    {
        state.LastUpdateTime = DateTime.UtcNow;
        _agentStates[agentId] = state;
        
        // Update communication graph
        _commGraph.UpdatePosition(agentId, state.Position);
    }
    
    /// <summary>
    /// Get agent state.
    /// </summary>
    public AgentState GetAgentState(int agentId) => _agentStates[agentId];
    
    /// <summary>
    /// Get all agent states.
    /// </summary>
    public IReadOnlyDictionary<int, AgentState> GetAllStates() => _agentStates;
    
    /// <summary>
    /// Get neighbors of an agent.
    /// </summary>
    public List<int> GetNeighbors(int agentId) => _commGraph.GetNeighbors(agentId);
    
    #endregion
    
    #region Consensus Control
    
    /// <summary>
    /// Compute consensus control for all agents to reach agreement.
    /// </summary>
    public Dictionary<int, Vector<double>> ComputeConsensusControls()
    {
        return _consensus.ComputeControls(_agentStates, _commGraph);
    }
    
    /// <summary>
    /// Compute consensus on a custom variable (e.g., target position).
    /// </summary>
    public Vector<double> ComputeConsensusValue(int agentId, 
        Func<int, Vector<double>> getValue)
    {
        return _consensus.ComputeConsensusValue(agentId, _commGraph, getValue);
    }
    
    #endregion
    
    #region Formation Control
    
    /// <summary>
    /// Set desired formation shape.
    /// </summary>
    public void SetFormation(FormationType type, double scale = 1.0)
    {
        _formation.SetFormation(type, _numAgents, scale);
    }
    
    /// <summary>
    /// Set custom formation offsets.
    /// </summary>
    public void SetCustomFormation(Dictionary<int, Vector<double>> offsets)
    {
        _formation.SetCustomFormation(offsets);
    }
    
    /// <summary>
    /// Compute formation control for all agents.
    /// </summary>
    public Dictionary<int, Vector<double>> ComputeFormationControls(
        Vector<double> formationCenter, double formationHeading)
    {
        return _formation.ComputeControls(_agentStates, _commGraph, formationCenter, formationHeading);
    }
    
    #endregion
    
    #region Task Allocation
    
    /// <summary>
    /// Add task for allocation.
    /// </summary>
    public void AddTask(SwarmTask task)
    {
        _taskAllocator.AddTask(task);
    }
    
    /// <summary>
    /// Compute task allocation using auction-based method.
    /// </summary>
    public Dictionary<int, List<SwarmTask>> AllocateTasks()
    {
        return _taskAllocator.Allocate(_agentStates);
    }
    
    /// <summary>
    /// Get current task for an agent.
    /// </summary>
    public SwarmTask? GetCurrentTask(int agentId)
    {
        return _taskAllocator.GetCurrentTask(agentId);
    }
    
    #endregion
    
    #region Cooperative SLAM
    
    /// <summary>
    /// Add local odometry from an agent.
    /// </summary>
    public void AddOdometry(int agentId, Matrix<double> relativePose, Matrix<double> covariance)
    {
        _coopSlam.AddOdometry(agentId, relativePose, covariance);
    }
    
    /// <summary>
    /// Add inter-robot observation (relative pose measurement).
    /// </summary>
    public void AddInterRobotObservation(int fromAgent, int toAgent, 
        Matrix<double> relativePose, Matrix<double> covariance)
    {
        _coopSlam.AddInterRobotObservation(fromAgent, toAgent, relativePose, covariance);
    }
    
    /// <summary>
    /// Add shared landmark observation.
    /// </summary>
    public void AddLandmarkObservation(int agentId, string landmarkId,
        Vector<double> observation, Matrix<double> covariance)
    {
        _coopSlam.AddLandmarkObservation(agentId, landmarkId, observation, covariance);
    }
    
    /// <summary>
    /// Optimize the cooperative map.
    /// </summary>
    public void OptimizeCooperativeMap()
    {
        _coopSlam.Optimize();
    }
    
    /// <summary>
    /// Get optimized pose for an agent.
    /// </summary>
    public Matrix<double>? GetOptimizedPose(int agentId, int timestep)
    {
        return _coopSlam.GetPose(agentId, timestep);
    }
    
    #endregion
    
    #region Collision Avoidance
    
    /// <summary>
    /// Compute collision-free velocities using ORCA.
    /// </summary>
    public Dictionary<int, Vector<double>> ComputeCollisionFreeVelocities(
        Dictionary<int, Vector<double>> preferredVelocities)
    {
        var safeVelocities = new Dictionary<int, Vector<double>>();
        
        foreach (var (agentId, prefVel) in preferredVelocities)
        {
            var state = _agentStates[agentId];
            var neighbors = _commGraph.GetNeighbors(agentId);
            
            // Compute ORCA velocity
            var orcaVel = ComputeOrcaVelocity(
                state, prefVel, 
                neighbors.Select(n => _agentStates[n]).ToList());
            
            safeVelocities[agentId] = orcaVel;
        }
        
        return safeVelocities;
    }
    
    private Vector<double> ComputeOrcaVelocity(AgentState agent, Vector<double> preferredVel,
        List<AgentState> neighbors)
    {
        var orcaPlanes = new List<(Vector<double> point, Vector<double> normal)>();
        double tau = 2.0; // Time horizon
        
        foreach (var neighbor in neighbors)
        {
            var relPos = neighbor.Position - agent.Position;
            var relVel = agent.Velocity - neighbor.Velocity;
            double dist = relPos.L2Norm();
            double combinedRadius = agent.Radius + neighbor.Radius;
            
            if (dist > combinedRadius)
            {
                // No current collision
                double leg = System.Math.Sqrt(dist * dist - combinedRadius * combinedRadius);
                
                // Compute velocity obstacle
                Vector<double> leftLegDir, rightLegDir;
                if (relPos[0] * relPos[1] > 0)
                {
                    leftLegDir = Vector<double>.Build.DenseOfArray([
                        relPos[0] * leg - relPos[1] * combinedRadius,
                        relPos[0] * combinedRadius + relPos[1] * leg
                    ]) / (dist * dist);
                    rightLegDir = Vector<double>.Build.DenseOfArray([
                        relPos[0] * leg + relPos[1] * combinedRadius,
                        -relPos[0] * combinedRadius + relPos[1] * leg
                    ]) / (dist * dist);
                }
                else
                {
                    leftLegDir = relPos.Normalize(2);
                    rightLegDir = -leftLegDir;
                }
                
                // ORCA half-plane
                var u = relPos / tau - relVel;
                double uLen = u.L2Norm();
                
                if (uLen > 0)
                {
                    var uDir = u / uLen;
                    var point = agent.Velocity + 0.5 * uDir * System.Math.Max(0, uLen - combinedRadius / tau);
                    orcaPlanes.Add((point, uDir));
                }
            }
            else
            {
                // Collision - compute escape velocity
                var escapeDir = (agent.Position - neighbor.Position).Normalize(2);
                var point = agent.Velocity + escapeDir * (combinedRadius - dist);
                orcaPlanes.Add((point, escapeDir));
            }
        }
        
        // Solve linear program to find best velocity satisfying all ORCA constraints
        return SolveOrcaLP(preferredVel, orcaPlanes, agent.MaxSpeed);
    }
    
    private Vector<double> SolveOrcaLP(Vector<double> preferred, 
        List<(Vector<double> point, Vector<double> normal)> planes, double maxSpeed)
    {
        var velocity = preferred.Clone();
        
        // Simple projection onto half-planes
        foreach (var (point, normal) in planes)
        {
            double dot = (velocity - point) * normal;
            if (dot < 0)
            {
                velocity -= dot * normal;
            }
        }
        
        // Clamp to max speed
        if (velocity.L2Norm() > maxSpeed)
        {
            velocity = velocity.Normalize(2) * maxSpeed;
        }
        
        return velocity;
    }
    
    #endregion
}

#region Consensus Control

/// <summary>
/// Distributed consensus controller.
/// </summary>
public class ConsensusController
{
    private readonly SwarmConfig _config;
    
    public ConsensusController(SwarmConfig config)
    {
        _config = config;
    }
    
    /// <summary>
    /// Compute consensus-based controls for position agreement.
    /// </summary>
    public Dictionary<int, Vector<double>> ComputeControls(
        Dictionary<int, AgentState> states, CommunicationGraph graph)
    {
        var controls = new Dictionary<int, Vector<double>>();
        
        foreach (var (agentId, state) in states)
        {
            var neighbors = graph.GetNeighbors(agentId);
            var control = Vector<double>.Build.Dense(3);
            
            foreach (var neighborId in neighbors)
            {
                var neighborState = states[neighborId];
                
                // Position consensus
                control += _config.PositionGain * (neighborState.Position - state.Position);
                
                // Velocity consensus (for second-order dynamics)
                control += _config.VelocityGain * (neighborState.Velocity - state.Velocity);
            }
            
            controls[agentId] = control;
        }
        
        return controls;
    }
    
    /// <summary>
    /// Compute consensus value for a custom variable.
    /// </summary>
    public Vector<double> ComputeConsensusValue(int agentId, CommunicationGraph graph,
        Func<int, Vector<double>> getValue)
    {
        var neighbors = graph.GetNeighbors(agentId);
        var myValue = getValue(agentId);
        
        if (neighbors.Count == 0)
            return myValue;
        
        // Average with neighbors
        var sum = myValue.Clone();
        foreach (var neighborId in neighbors)
        {
            sum += getValue(neighborId);
        }
        
        return sum / (neighbors.Count + 1);
    }
}

#endregion

#region Formation Control

/// <summary>
/// Formation controller using graph-based approach.
/// </summary>
public class FormationController
{
    private readonly SwarmConfig _config;
    private Dictionary<int, Vector<double>> _formationOffsets = new();
    
    public FormationController(SwarmConfig config)
    {
        _config = config;
    }
    
    /// <summary>
    /// Set standard formation pattern.
    /// </summary>
    public void SetFormation(FormationType type, int numAgents, double scale)
    {
        _formationOffsets = type switch
        {
            FormationType.Line => GenerateLineFormation(numAgents, scale),
            FormationType.Circle => GenerateCircleFormation(numAgents, scale),
            FormationType.Grid => GenerateGridFormation(numAgents, scale),
            FormationType.Wedge => GenerateWedgeFormation(numAgents, scale),
            FormationType.Diamond => GenerateDiamondFormation(numAgents, scale),
            _ => GenerateCircleFormation(numAgents, scale)
        };
    }
    
    /// <summary>
    /// Set custom formation offsets.
    /// </summary>
    public void SetCustomFormation(Dictionary<int, Vector<double>> offsets)
    {
        _formationOffsets = new Dictionary<int, Vector<double>>(offsets);
    }
    
    /// <summary>
    /// Compute formation tracking controls.
    /// </summary>
    public Dictionary<int, Vector<double>> ComputeControls(
        Dictionary<int, AgentState> states,
        CommunicationGraph graph,
        Vector<double> formationCenter,
        double formationHeading)
    {
        var controls = new Dictionary<int, Vector<double>>();
        var rotationMatrix = RotationMatrix2D(formationHeading);
        
        foreach (var (agentId, state) in states)
        {
            // Compute desired position
            var offset = _formationOffsets.GetValueOrDefault(agentId, Vector<double>.Build.Dense(3));
            var rotatedOffset = Rotate2D(offset, rotationMatrix);
            var desiredPos = formationCenter + rotatedOffset;
            
            // Position error
            var posError = desiredPos - state.Position;
            
            // Formation control with inter-agent constraints
            var control = _config.FormationGain * posError;
            
            // Add neighbor-relative constraints for rigidity
            var neighbors = graph.GetNeighbors(agentId);
            foreach (var neighborId in neighbors)
            {
                var neighborState = states[neighborId];
                var desiredDiff = _formationOffsets.GetValueOrDefault(agentId, Vector<double>.Build.Dense(3)) - 
                                  _formationOffsets.GetValueOrDefault(neighborId, Vector<double>.Build.Dense(3));
                var actualDiff = state.Position - neighborState.Position;
                
                control += _config.RigidityGain * (Rotate2D(desiredDiff, rotationMatrix) - actualDiff);
            }
            
            controls[agentId] = control;
        }
        
        return controls;
    }
    
    private Dictionary<int, Vector<double>> GenerateLineFormation(int n, double spacing)
    {
        var offsets = new Dictionary<int, Vector<double>>();
        for (int i = 0; i < n; i++)
        {
            offsets[i] = Vector<double>.Build.DenseOfArray([
                (i - (n - 1) / 2.0) * spacing, 0, 0
            ]);
        }
        return offsets;
    }
    
    private Dictionary<int, Vector<double>> GenerateCircleFormation(int n, double radius)
    {
        var offsets = new Dictionary<int, Vector<double>>();
        for (int i = 0; i < n; i++)
        {
            double angle = 2 * System.Math.PI * i / n;
            offsets[i] = Vector<double>.Build.DenseOfArray([
                radius * System.Math.Cos(angle),
                radius * System.Math.Sin(angle),
                0
            ]);
        }
        return offsets;
    }
    
    private Dictionary<int, Vector<double>> GenerateGridFormation(int n, double spacing)
    {
        var offsets = new Dictionary<int, Vector<double>>();
        int cols = (int)System.Math.Ceiling(System.Math.Sqrt(n));
        
        for (int i = 0; i < n; i++)
        {
            int row = i / cols;
            int col = i % cols;
            offsets[i] = Vector<double>.Build.DenseOfArray([
                (col - (cols - 1) / 2.0) * spacing,
                (row - (n / cols - 1) / 2.0) * spacing,
                0
            ]);
        }
        return offsets;
    }
    
    private Dictionary<int, Vector<double>> GenerateWedgeFormation(int n, double spacing)
    {
        var offsets = new Dictionary<int, Vector<double>>();
        offsets[0] = Vector<double>.Build.Dense(3); // Leader at origin
        
        int idx = 1;
        int row = 1;
        while (idx < n)
        {
            for (int i = 0; i <= row && idx < n; i++, idx++)
            {
                offsets[idx] = Vector<double>.Build.DenseOfArray([
                    -row * spacing,
                    (i - row / 2.0) * spacing,
                    0
                ]);
            }
            row++;
        }
        return offsets;
    }
    
    private Dictionary<int, Vector<double>> GenerateDiamondFormation(int n, double spacing)
    {
        var offsets = new Dictionary<int, Vector<double>>();
        int half = n / 2;
        
        for (int i = 0; i < n; i++)
        {
            double x, y;
            if (i < half)
            {
                x = i * spacing / 2;
                y = i * spacing / 2;
            }
            else
            {
                x = (n - 1 - i) * spacing / 2;
                y = -(n - 1 - i) * spacing / 2;
            }
            offsets[i] = Vector<double>.Build.DenseOfArray([x, y, 0]);
        }
        return offsets;
    }
    
    private Matrix<double> RotationMatrix2D(double angle)
    {
        double c = System.Math.Cos(angle);
        double s = System.Math.Sin(angle);
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { c, -s, 0 },
            { s, c, 0 },
            { 0, 0, 1 }
        });
    }
    
    private Vector<double> Rotate2D(Vector<double> v, Matrix<double> R)
    {
        if (v.Count < 3) return v;
        return R * v;
    }
}

public enum FormationType
{
    Line,
    Circle,
    Grid,
    Wedge,
    Diamond,
    Custom
}

#endregion

#region Task Allocation

/// <summary>
/// Auction-based task allocator for multi-robot systems.
/// Implements consensus-based bundle algorithm (CBBA).
/// </summary>
public class TaskAllocator
{
    private readonly int _numAgents;
    private readonly List<SwarmTask> _tasks = new();
    private readonly Dictionary<int, List<SwarmTask>> _bundles = new();
    private readonly Dictionary<int, SwarmTask?> _currentTasks = new();
    
    public TaskAllocator(int numAgents)
    {
        _numAgents = numAgents;
        for (int i = 0; i < numAgents; i++)
        {
            _bundles[i] = new List<SwarmTask>();
            _currentTasks[i] = null;
        }
    }
    
    public void AddTask(SwarmTask task)
    {
        _tasks.Add(task);
    }
    
    /// <summary>
    /// Allocate tasks using auction algorithm.
    /// </summary>
    public Dictionary<int, List<SwarmTask>> Allocate(Dictionary<int, AgentState> states)
    {
        // Initialize bids
        var bids = new Dictionary<(int agent, int task), double>();
        
        foreach (var (agentId, state) in states)
        {
            for (int t = 0; t < _tasks.Count; t++)
            {
                bids[(agentId, t)] = ComputeBid(state, _tasks[t]);
            }
        }
        
        // Clear previous allocations
        foreach (var bundle in _bundles.Values)
            bundle.Clear();
        
        // Auction rounds
        var assigned = new HashSet<int>();
        
        for (int round = 0; round < _tasks.Count; round++)
        {
            // Find best agent for each unassigned task
            for (int t = 0; t < _tasks.Count; t++)
            {
                if (assigned.Contains(t)) continue;
                
                int bestAgent = -1;
                double bestBid = double.NegativeInfinity;
                
                foreach (var (agentId, _) in states)
                {
                    double bid = bids[(agentId, t)];
                    
                    // Penalize if agent already has tasks
                    bid -= _bundles[agentId].Count * 0.1;
                    
                    if (bid > bestBid)
                    {
                        bestBid = bid;
                        bestAgent = agentId;
                    }
                }
                
                if (bestAgent >= 0 && bestBid > 0)
                {
                    _bundles[bestAgent].Add(_tasks[t]);
                    assigned.Add(t);
                }
            }
        }
        
        // Set current tasks
        foreach (var (agentId, bundle) in _bundles)
        {
            _currentTasks[agentId] = bundle.FirstOrDefault();
        }
        
        return new Dictionary<int, List<SwarmTask>>(_bundles);
    }
    
    public SwarmTask? GetCurrentTask(int agentId)
    {
        return _currentTasks.GetValueOrDefault(agentId);
    }
    
    private double ComputeBid(AgentState agent, SwarmTask task)
    {
        // Bid = task value - cost to reach
        double distance = (task.Position - agent.Position).L2Norm();
        double travelTime = distance / agent.MaxSpeed;
        
        double reward = task.Priority * task.Value;
        double cost = travelTime + task.EstimatedDuration;
        
        // Check deadline
        if (DateTime.UtcNow.AddSeconds(travelTime) > task.Deadline)
            return double.NegativeInfinity;
        
        // Capability check
        if (!task.RequiredCapabilities.All(c => agent.Capabilities.Contains(c)))
            return double.NegativeInfinity;
        
        return reward - cost;
    }
}

public class SwarmTask
{
    public string TaskId { get; set; } = Guid.NewGuid().ToString();
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public double Priority { get; set; } = 1.0;
    public double Value { get; set; } = 1.0;
    public double EstimatedDuration { get; set; } = 60;
    public DateTime Deadline { get; set; } = DateTime.MaxValue;
    public HashSet<string> RequiredCapabilities { get; set; } = new();
    public TaskStatus Status { get; set; } = TaskStatus.Pending;
}

public enum TaskStatus
{
    Pending,
    Assigned,
    InProgress,
    Completed,
    Failed
}

#endregion

#region Cooperative SLAM

/// <summary>
/// Multi-robot cooperative SLAM backend using factor graphs.
/// </summary>
public class CooperativeSlamBackend
{
    private readonly int _numRobots;
    private readonly FactorGraph _graph;
    private readonly Dictionary<(int robot, int timestep), string> _poseKeys = new();
    private readonly Dictionary<string, string> _landmarkKeys = new();
    private int[] _currentTimesteps;
    
    public CooperativeSlamBackend(int numRobots)
    {
        _numRobots = numRobots;
        _graph = new FactorGraph();
        _currentTimesteps = new int[numRobots];
    }
    
    /// <summary>
    /// Add odometry factor for a robot.
    /// </summary>
    public void AddOdometry(int robotId, Matrix<double> relativePose, Matrix<double> covariance)
    {
        int t = _currentTimesteps[robotId];
        
        // Create new pose variable
        string key1 = GetPoseKey(robotId, t);
        string key2 = GetPoseKey(robotId, t + 1);
        
        if (!_graph.HasVariable(key2))
        {
            var prevPose = _graph.HasVariable(key1) 
                ? _graph.GetVariable<PoseVariable>(key1).Value 
                : Matrix<double>.Build.DenseIdentity(4);
            var newPose = LieGroups.ComposeSE3(prevPose, relativePose);
            _graph.AddPoseVariable(key2, newPose);
        }
        
        _graph.AddBetweenFactor(key1, key2, relativePose, covariance);
        _currentTimesteps[robotId]++;
    }
    
    /// <summary>
    /// Add inter-robot relative pose observation.
    /// </summary>
    public void AddInterRobotObservation(int fromRobot, int toRobot,
        Matrix<double> relativePose, Matrix<double> covariance)
    {
        string key1 = GetPoseKey(fromRobot, _currentTimesteps[fromRobot]);
        string key2 = GetPoseKey(toRobot, _currentTimesteps[toRobot]);
        
        // Use robust factor for inter-robot measurements (may be outliers)
        _graph.AddLoopClosureFactor(key1, key2, relativePose, covariance, 3.0);
    }
    
    /// <summary>
    /// Add landmark observation.
    /// </summary>
    public void AddLandmarkObservation(int robotId, string landmarkId,
        Vector<double> observation, Matrix<double> covariance)
    {
        string poseKey = GetPoseKey(robotId, _currentTimesteps[robotId]);
        string lmKey = GetLandmarkKey(landmarkId);
        
        if (!_graph.HasVariable(lmKey))
        {
            // Initialize landmark from first observation
            var pose = _graph.GetVariable<PoseVariable>(poseKey).Value;
            var R = pose.SubMatrix(0, 3, 0, 3);
            var t = Vector<double>.Build.DenseOfArray([pose[0, 3], pose[1, 3], pose[2, 3]]);
            var landmarkPos = t + R * observation;
            _graph.AddPointVariable(lmKey, landmarkPos);
        }
        
        // Add projection factor
        var calibration = new CameraCalibration { Fx = 500, Fy = 500, Cx = 320, Cy = 240 };
        _graph.AddProjectionFactor(poseKey, lmKey, observation.SubVector(0, 2), covariance, calibration);
    }
    
    /// <summary>
    /// Optimize the cooperative map.
    /// </summary>
    public void Optimize()
    {
        _graph.Optimize(50);
    }
    
    /// <summary>
    /// Get optimized pose.
    /// </summary>
    public Matrix<double>? GetPose(int robotId, int timestep)
    {
        string key = GetPoseKey(robotId, timestep);
        if (_graph.HasVariable(key))
        {
            return _graph.GetVariable<PoseVariable>(key).Value;
        }
        return null;
    }
    
    private string GetPoseKey(int robotId, int timestep)
    {
        if (!_poseKeys.TryGetValue((robotId, timestep), out var key))
        {
            key = $"x_{robotId}_{timestep}";
            _poseKeys[(robotId, timestep)] = key;
            
            if (timestep == 0 && !_graph.HasVariable(key))
            {
                // Add initial pose with prior
                _graph.AddPoseVariable(key, Matrix<double>.Build.DenseIdentity(4));
                _graph.AddPosePrior(key, Matrix<double>.Build.DenseIdentity(4),
                    Matrix<double>.Build.DenseIdentity(6) * 0.001);
            }
        }
        return key;
    }
    
    private string GetLandmarkKey(string landmarkId)
    {
        if (!_landmarkKeys.TryGetValue(landmarkId, out var key))
        {
            key = $"l_{landmarkId}";
            _landmarkKeys[landmarkId] = key;
        }
        return key;
    }
}

#endregion

#region Communication Graph

/// <summary>
/// Dynamic communication graph based on distance.
/// </summary>
public class CommunicationGraph
{
    private readonly int _numAgents;
    private readonly double _commRadius;
    private readonly Vector<double>[] _positions;
    
    public CommunicationGraph(int numAgents, double commRadius)
    {
        _numAgents = numAgents;
        _commRadius = commRadius;
        _positions = new Vector<double>[numAgents];
        
        for (int i = 0; i < numAgents; i++)
        {
            _positions[i] = Vector<double>.Build.Dense(3);
        }
    }
    
    public void UpdatePosition(int agentId, Vector<double> position)
    {
        _positions[agentId] = position;
    }
    
    public List<int> GetNeighbors(int agentId)
    {
        var neighbors = new List<int>();
        
        for (int i = 0; i < _numAgents; i++)
        {
            if (i == agentId) continue;
            
            double dist = (_positions[i] - _positions[agentId]).L2Norm();
            if (dist <= _commRadius)
            {
                neighbors.Add(i);
            }
        }
        
        return neighbors;
    }
    
    /// <summary>
    /// Get adjacency matrix.
    /// </summary>
    public Matrix<double> GetAdjacencyMatrix()
    {
        var A = Matrix<double>.Build.Dense(_numAgents, _numAgents);
        
        for (int i = 0; i < _numAgents; i++)
        {
            for (int j = 0; j < _numAgents; j++)
            {
                if (i != j)
                {
                    double dist = (_positions[i] - _positions[j]).L2Norm();
                    A[i, j] = dist <= _commRadius ? 1 : 0;
                }
            }
        }
        
        return A;
    }
    
    /// <summary>
    /// Get Laplacian matrix.
    /// </summary>
    public Matrix<double> GetLaplacianMatrix()
    {
        var A = GetAdjacencyMatrix();
        var D = Matrix<double>.Build.Dense(_numAgents, _numAgents);
        
        for (int i = 0; i < _numAgents; i++)
        {
            D[i, i] = A.Row(i).Sum();
        }
        
        return D - A;
    }
    
    /// <summary>
    /// Check if graph is connected.
    /// </summary>
    public bool IsConnected()
    {
        var L = GetLaplacianMatrix();
        var eigenvalues = L.Evd().EigenValues;
        
        // Graph is connected if second smallest eigenvalue > 0
        var sorted = eigenvalues.OrderBy(e => e.Real).ToList();
        return sorted.Count > 1 && sorted[1].Real > 1e-6;
    }
}

#endregion

#region Supporting Types

public class AgentState
{
    public int AgentId { get; set; }
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> Velocity { get; set; } = Vector<double>.Build.Dense(3);
    public Matrix<double> Rotation { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    public double Radius { get; set; } = 0.5;
    public double MaxSpeed { get; set; } = 5.0;
    public HashSet<string> Capabilities { get; set; } = new();
    public DateTime LastUpdateTime { get; set; } = DateTime.UtcNow;
}

public class SwarmConfig
{
    public double CommunicationRadius { get; set; } = 50.0;
    public double PositionGain { get; set; } = 1.0;
    public double VelocityGain { get; set; } = 0.5;
    public double FormationGain { get; set; } = 2.0;
    public double RigidityGain { get; set; } = 0.5;
    public double SafetyRadius { get; set; } = 2.0;
}

#endregion
