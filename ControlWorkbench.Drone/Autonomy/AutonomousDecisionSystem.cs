using MathNet.Numerics.LinearAlgebra;
using System.Collections.Concurrent;

namespace ControlWorkbench.Drone.Autonomy;

/// <summary>
/// Intelligent Autonomous Decision Making System.
/// Implements behavior trees, goal-based planning, and mission intelligence.
/// 
/// Features:
/// - Hierarchical Behavior Trees with dynamic modification
/// - Goal-Oriented Action Planning (GOAP)
/// - Situation awareness and threat assessment
/// - Dynamic mission replanning
/// - Multi-objective decision making under uncertainty
/// 
/// Based on:
/// - "Game AI Pro: Behavior Trees" (Champandard, 2014)
/// - "Goal-Oriented Action Planning" (Orkin, 2003)
/// - "Decision Making Under Uncertainty" (Thrun et al.)
/// </summary>
public class AutonomousDecisionSystem
{
    private readonly DecisionConfig _config;
    private readonly BehaviorTreeExecutor _behaviorTree;
    private readonly GoalPlanner _goalPlanner;
    private readonly SituationAwareness _situationAwareness;
    private readonly MissionIntelligence _missionIntelligence;
    private readonly RiskAssessor _riskAssessor;
    
    private readonly ConcurrentQueue<SituationUpdate> _situationQueue = new();
    private WorldState _worldState;
    private MissionState _missionState;
    
    private CancellationTokenSource? _cts;
    private Task? _decisionTask;
    
    public event Action<AutonomousAction>? ActionDecided;
    public event Action<MissionReplan>? MissionReplanned;
    public event Action<ThreatAssessment>? ThreatIdentified;
    
    public WorldState CurrentWorldState => _worldState;
    public MissionState CurrentMissionState => _missionState;
    
    public AutonomousDecisionSystem(DecisionConfig? config = null)
    {
        _config = config ?? new DecisionConfig();
        _behaviorTree = new BehaviorTreeExecutor();
        _goalPlanner = new GoalPlanner(_config);
        _situationAwareness = new SituationAwareness(_config);
        _missionIntelligence = new MissionIntelligence(_config);
        _riskAssessor = new RiskAssessor(_config);
        
        _worldState = new WorldState();
        _missionState = new MissionState();
        
        BuildDefaultBehaviorTree();
    }
    
    /// <summary>
    /// Start the autonomous decision loop.
    /// </summary>
    public void Start()
    {
        _cts = new CancellationTokenSource();
        _decisionTask = Task.Run(() => DecisionLoop(_cts.Token), _cts.Token);
    }
    
    /// <summary>
    /// Stop the decision system.
    /// </summary>
    public async Task StopAsync()
    {
        _cts?.Cancel();
        if (_decisionTask != null)
        {
            try { await _decisionTask; } catch { }
        }
    }
    
    /// <summary>
    /// Update situation with new sensor data.
    /// </summary>
    public void UpdateSituation(SituationUpdate update)
    {
        _situationQueue.Enqueue(update);
    }
    
    /// <summary>
    /// Set mission goals.
    /// </summary>
    public void SetMission(MissionGoal mission)
    {
        _missionState.CurrentMission = mission;
        _missionState.Status = MissionStatus.Active;
        
        // Plan initial actions
        var plan = _goalPlanner.Plan(_worldState, mission);
        _missionState.CurrentPlan = plan;
    }
    
    /// <summary>
    /// Request immediate action override.
    /// </summary>
    public void RequestAction(AutonomousAction action)
    {
        ActionDecided?.Invoke(action);
    }
    
    /// <summary>
    /// Get current decision explanation.
    /// </summary>
    public DecisionExplanation GetExplanation()
    {
        return _behaviorTree.GetCurrentExplanation();
    }
    
    /// <summary>
    /// Configure behavior tree from JSON.
    /// </summary>
    public void LoadBehaviorTree(string json)
    {
        _behaviorTree.LoadFromJson(json);
    }
    
    private async Task DecisionLoop(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                // Process situation updates
                while (_situationQueue.TryDequeue(out var update))
                {
                    ProcessSituationUpdate(update);
                }
                
                // Update situation awareness
                var awareness = _situationAwareness.Analyze(_worldState);
                
                // Assess threats
                var threats = _riskAssessor.AssessThreats(_worldState, awareness);
                foreach (var threat in threats.Where(t => t.Level >= ThreatLevel.High))
                {
                    ThreatIdentified?.Invoke(threat);
                }
                
                // Execute behavior tree
                var context = new BehaviorContext
                {
                    WorldState = _worldState,
                    MissionState = _missionState,
                    Awareness = awareness,
                    Threats = threats
                };
                
                var result = _behaviorTree.Tick(context);
                
                if (result.Action != null)
                {
                    ActionDecided?.Invoke(result.Action);
                }
                
                // Check if mission replanning is needed
                if (_missionState.CurrentMission != null && result.RequiresReplan)
                {
                    var newPlan = _missionIntelligence.Replan(
                        _worldState, _missionState, awareness, threats);
                    
                    if (newPlan != null && !newPlan.Equals(_missionState.CurrentPlan))
                    {
                        _missionState.CurrentPlan = newPlan;
                        MissionReplanned?.Invoke(new MissionReplan
                        {
                            Reason = result.ReplanReason ?? "Conditions changed",
                            NewPlan = newPlan
                        });
                    }
                }
                
                await Task.Delay(_config.DecisionIntervalMs, ct);
            }
            catch (OperationCanceledException) { break; }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Decision loop error: {ex.Message}");
            }
        }
    }
    
    private void ProcessSituationUpdate(SituationUpdate update)
    {
        switch (update.Type)
        {
            case UpdateType.Position:
                _worldState.VehiclePosition = update.Position!;
                break;
            case UpdateType.Velocity:
                _worldState.VehicleVelocity = update.Velocity!;
                break;
            case UpdateType.Attitude:
                _worldState.VehicleAttitude = update.Attitude!;
                break;
            case UpdateType.Battery:
                _worldState.BatteryPercent = update.BatteryPercent;
                break;
            case UpdateType.Obstacle:
                _worldState.Obstacles.Add(update.Obstacle!);
                break;
            case UpdateType.Weather:
                _worldState.WindSpeed = update.WindSpeed;
                _worldState.WindDirection = update.WindDirection;
                break;
            case UpdateType.Geofence:
                _worldState.GeofenceViolation = update.GeofenceViolation;
                break;
        }
        
        _worldState.LastUpdateTime = DateTime.UtcNow;
    }
    
    private void BuildDefaultBehaviorTree()
    {
        // Root selector - tries children in order until one succeeds
        var root = new SelectorNode("Root");
        
        // Emergency handling (highest priority)
        var emergency = new SequenceNode("EmergencyHandler");
        emergency.AddChild(new ConditionNode("IsEmergency", ctx => 
            ctx.Threats.Any(t => t.Level == ThreatLevel.Critical) ||
            ctx.WorldState.BatteryPercent < 5 ||
            ctx.WorldState.GeofenceViolation));
        emergency.AddChild(new ActionNode("ExecuteEmergencyRTL", ctx =>
        {
            return new BehaviorResult
            {
                Status = NodeStatus.Success,
                Action = new AutonomousAction
                {
                    Type = ActionType.ReturnToLaunch,
                    Priority = ActionPriority.Emergency,
                    Reason = "Emergency condition detected"
                }
            };
        }));
        root.AddChild(emergency);
        
        // Low battery handling
        var lowBattery = new SequenceNode("LowBatteryHandler");
        lowBattery.AddChild(new ConditionNode("IsLowBattery", ctx =>
            ctx.WorldState.BatteryPercent < 20));
        lowBattery.AddChild(new ActionNode("ReturnForBattery", ctx =>
        {
            return new BehaviorResult
            {
                Status = NodeStatus.Success,
                Action = new AutonomousAction
                {
                    Type = ActionType.ReturnToLaunch,
                    Priority = ActionPriority.High,
                    Reason = "Low battery"
                }
            };
        }));
        root.AddChild(lowBattery);
        
        // Obstacle avoidance
        var obstacleAvoidance = new SequenceNode("ObstacleAvoidance");
        obstacleAvoidance.AddChild(new ConditionNode("ObstacleNearby", ctx =>
            ctx.WorldState.Obstacles.Any(o => 
                (o.Position - ctx.WorldState.VehiclePosition).L2Norm() < 10)));
        obstacleAvoidance.AddChild(new ActionNode("AvoidObstacle", ctx =>
        {
            var nearest = ctx.WorldState.Obstacles
                .OrderBy(o => (o.Position - ctx.WorldState.VehiclePosition).L2Norm())
                .First();
            
            var avoidDir = (ctx.WorldState.VehiclePosition - nearest.Position).Normalize(2);
            var newTarget = ctx.WorldState.VehiclePosition + avoidDir * 5;
            
            return new BehaviorResult
            {
                Status = NodeStatus.Success,
                Action = new AutonomousAction
                {
                    Type = ActionType.GoToPosition,
                    TargetPosition = newTarget,
                    Priority = ActionPriority.High,
                    Reason = "Avoiding obstacle"
                }
            };
        }));
        root.AddChild(obstacleAvoidance);
        
        // Mission execution
        var missionExec = new SequenceNode("MissionExecution");
        missionExec.AddChild(new ConditionNode("HasActiveMission", ctx =>
            ctx.MissionState.CurrentMission != null &&
            ctx.MissionState.Status == MissionStatus.Active));
        missionExec.AddChild(new ActionNode("ExecuteMissionStep", ctx =>
        {
            var step = ctx.MissionState.CurrentPlan?.GetCurrentStep();
            if (step == null)
            {
                ctx.MissionState.Status = MissionStatus.Completed;
                return new BehaviorResult { Status = NodeStatus.Success };
            }
            
            return new BehaviorResult
            {
                Status = NodeStatus.Running,
                Action = step.ToAction()
            };
        }));
        root.AddChild(missionExec);
        
        // Default behavior - hover
        root.AddChild(new ActionNode("DefaultHover", ctx =>
        {
            return new BehaviorResult
            {
                Status = NodeStatus.Success,
                Action = new AutonomousAction
                {
                    Type = ActionType.Hover,
                    TargetPosition = ctx.WorldState.VehiclePosition,
                    Priority = ActionPriority.Low,
                    Reason = "No active task"
                }
            };
        }));
        
        _behaviorTree.SetRoot(root);
    }
}

#region Behavior Tree

/// <summary>
/// Behavior tree executor with tick-based evaluation.
/// </summary>
public class BehaviorTreeExecutor
{
    private BehaviorNode? _root;
    private DecisionExplanation _lastExplanation = new();
    
    public void SetRoot(BehaviorNode root)
    {
        _root = root;
    }
    
    public BehaviorResult Tick(BehaviorContext context)
    {
        if (_root == null)
            return new BehaviorResult { Status = NodeStatus.Failure };
        
        _lastExplanation = new DecisionExplanation
        {
            Timestamp = DateTime.UtcNow,
            Path = new List<string>()
        };
        
        return _root.Execute(context, _lastExplanation);
    }
    
    public DecisionExplanation GetCurrentExplanation() => _lastExplanation;
    
    public void LoadFromJson(string json)
    {
        // Would parse JSON and build tree dynamically
    }
}

public abstract class BehaviorNode
{
    public string Name { get; protected set; }
    
    protected BehaviorNode(string name)
    {
        Name = name;
    }
    
    public abstract BehaviorResult Execute(BehaviorContext context, DecisionExplanation explanation);
}

public class SelectorNode : BehaviorNode
{
    private readonly List<BehaviorNode> _children = new();
    
    public SelectorNode(string name) : base(name) { }
    
    public void AddChild(BehaviorNode child) => _children.Add(child);
    
    public override BehaviorResult Execute(BehaviorContext context, DecisionExplanation explanation)
    {
        explanation.Path.Add($"Selector:{Name}");
        
        foreach (var child in _children)
        {
            var result = child.Execute(context, explanation);
            
            if (result.Status == NodeStatus.Success || result.Status == NodeStatus.Running)
            {
                return result;
            }
        }
        
        return new BehaviorResult { Status = NodeStatus.Failure };
    }
}

public class SequenceNode : BehaviorNode
{
    private readonly List<BehaviorNode> _children = new();
    
    public SequenceNode(string name) : base(name) { }
    
    public void AddChild(BehaviorNode child) => _children.Add(child);
    
    public override BehaviorResult Execute(BehaviorContext context, DecisionExplanation explanation)
    {
        explanation.Path.Add($"Sequence:{Name}");
        
        foreach (var child in _children)
        {
            var result = child.Execute(context, explanation);
            
            if (result.Status == NodeStatus.Failure)
            {
                return result;
            }
            
            if (result.Status == NodeStatus.Running)
            {
                return result;
            }
        }
        
        return new BehaviorResult { Status = NodeStatus.Success };
    }
}

public class ConditionNode : BehaviorNode
{
    private readonly Func<BehaviorContext, bool> _condition;
    
    public ConditionNode(string name, Func<BehaviorContext, bool> condition) : base(name)
    {
        _condition = condition;
    }
    
    public override BehaviorResult Execute(BehaviorContext context, DecisionExplanation explanation)
    {
        bool result = _condition(context);
        explanation.Path.Add($"Condition:{Name}={result}");
        
        return new BehaviorResult
        {
            Status = result ? NodeStatus.Success : NodeStatus.Failure
        };
    }
}

public class ActionNode : BehaviorNode
{
    private readonly Func<BehaviorContext, BehaviorResult> _action;
    
    public ActionNode(string name, Func<BehaviorContext, BehaviorResult> action) : base(name)
    {
        _action = action;
    }
    
    public override BehaviorResult Execute(BehaviorContext context, DecisionExplanation explanation)
    {
        explanation.Path.Add($"Action:{Name}");
        var result = _action(context);
        explanation.FinalAction = result.Action;
        explanation.Reasoning = result.Action?.Reason ?? "";
        return result;
    }
}

public enum NodeStatus { Success, Failure, Running }

public class BehaviorResult
{
    public NodeStatus Status { get; set; }
    public AutonomousAction? Action { get; set; }
    public bool RequiresReplan { get; set; }
    public string? ReplanReason { get; set; }
}

public class BehaviorContext
{
    public WorldState WorldState { get; set; } = new();
    public MissionState MissionState { get; set; } = new();
    public SituationAwarenessResult Awareness { get; set; } = new();
    public List<ThreatAssessment> Threats { get; set; } = new();
}

#endregion

#region Goal-Oriented Action Planning

/// <summary>
/// GOAP-style goal planner.
/// </summary>
public class GoalPlanner
{
    private readonly DecisionConfig _config;
    private readonly List<PlanningAction> _availableActions = new();
    
    public GoalPlanner(DecisionConfig config)
    {
        _config = config;
        RegisterDefaultActions();
    }
    
    public ActionPlan Plan(WorldState current, MissionGoal goal)
    {
        var plan = new ActionPlan();
        
        // A* search through action space
        var openSet = new PriorityQueue<PlanState, double>();
        var closedSet = new HashSet<string>();
        
        var startState = new PlanState
        {
            State = current.ToSymbolic(),
            Actions = new List<PlanningAction>(),
            Cost = 0
        };
        
        openSet.Enqueue(startState, Heuristic(startState.State, goal));
        
        while (openSet.Count > 0)
        {
            var current_ = openSet.Dequeue();
            
            if (GoalSatisfied(current_.State, goal))
            {
                plan.Steps = current_.Actions.Select(a => a.ToStep()).ToList();
                plan.EstimatedDuration = current_.Actions.Sum(a => a.Duration);
                return plan;
            }
            
            var stateKey = GetStateKey(current_.State);
            if (closedSet.Contains(stateKey))
                continue;
            closedSet.Add(stateKey);
            
            foreach (var action in _availableActions)
            {
                if (action.CanExecute(current_.State))
                {
                    var newState = action.Apply(current_.State);
                    var newActions = new List<PlanningAction>(current_.Actions) { action };
                    var newCost = current_.Cost + action.Cost;
                    
                    var newPlanState = new PlanState
                    {
                        State = newState,
                        Actions = newActions,
                        Cost = newCost
                    };
                    
                    openSet.Enqueue(newPlanState, newCost + Heuristic(newState, goal));
                }
            }
        }
        
        return plan; // No plan found
    }
    
    private void RegisterDefaultActions()
    {
        _availableActions.Add(new TakeoffAction());
        _availableActions.Add(new LandAction());
        _availableActions.Add(new GoToAction());
        _availableActions.Add(new HoverAction());
        _availableActions.Add(new SurveyAction());
    }
    
    private double Heuristic(Dictionary<string, object> state, MissionGoal goal)
    {
        // Simple distance-based heuristic
        if (goal.TargetPosition != null && state.TryGetValue("Position", out var posObj))
        {
            var pos = (Vector<double>)posObj;
            return (goal.TargetPosition - pos).L2Norm();
        }
        return 0;
    }
    
    private bool GoalSatisfied(Dictionary<string, object> state, MissionGoal goal)
    {
        foreach (var condition in goal.Conditions)
        {
            if (!state.TryGetValue(condition.Key, out var value))
                return false;
            
            if (!condition.Value.Equals(value))
                return false;
        }
        return true;
    }
    
    private string GetStateKey(Dictionary<string, object> state)
    {
        return string.Join(",", state.Select(kvp => $"{kvp.Key}={kvp.Value}"));
    }
}

public abstract class PlanningAction
{
    public abstract string Name { get; }
    public abstract double Cost { get; }
    public abstract double Duration { get; }
    
    public abstract bool CanExecute(Dictionary<string, object> state);
    public abstract Dictionary<string, object> Apply(Dictionary<string, object> state);
    public abstract PlanStep ToStep();
}

public class TakeoffAction : PlanningAction
{
    public override string Name => "Takeoff";
    public override double Cost => 10;
    public override double Duration => 10;
    
    public override bool CanExecute(Dictionary<string, object> state)
    {
        return state.TryGetValue("IsFlying", out var flying) && !(bool)flying;
    }
    
    public override Dictionary<string, object> Apply(Dictionary<string, object> state)
    {
        var newState = new Dictionary<string, object>(state);
        newState["IsFlying"] = true;
        newState["Altitude"] = 10.0;
        return newState;
    }
    
    public override PlanStep ToStep() => new() { Type = PlanStepType.Takeoff, Altitude = 10 };
}

public class LandAction : PlanningAction
{
    public override string Name => "Land";
    public override double Cost => 10;
    public override double Duration => 15;
    
    public override bool CanExecute(Dictionary<string, object> state)
    {
        return state.TryGetValue("IsFlying", out var flying) && (bool)flying;
    }
    
    public override Dictionary<string, object> Apply(Dictionary<string, object> state)
    {
        var newState = new Dictionary<string, object>(state);
        newState["IsFlying"] = false;
        newState["Altitude"] = 0.0;
        return newState;
    }
    
    public override PlanStep ToStep() => new() { Type = PlanStepType.Land };
}

public class GoToAction : PlanningAction
{
    public Vector<double>? TargetPosition { get; set; }
    
    public override string Name => "GoTo";
    public override double Cost => 1;
    public override double Duration => 30;
    
    public override bool CanExecute(Dictionary<string, object> state)
    {
        return state.TryGetValue("IsFlying", out var flying) && (bool)flying;
    }
    
    public override Dictionary<string, object> Apply(Dictionary<string, object> state)
    {
        var newState = new Dictionary<string, object>(state);
        if (TargetPosition != null)
            newState["Position"] = TargetPosition;
        return newState;
    }
    
    public override PlanStep ToStep() => new() 
    { 
        Type = PlanStepType.GoTo, 
        TargetPosition = TargetPosition 
    };
}

public class HoverAction : PlanningAction
{
    public double Duration_ { get; set; } = 5;
    
    public override string Name => "Hover";
    public override double Cost => 0.5;
    public override double Duration => Duration_;
    
    public override bool CanExecute(Dictionary<string, object> state)
    {
        return state.TryGetValue("IsFlying", out var flying) && (bool)flying;
    }
    
    public override Dictionary<string, object> Apply(Dictionary<string, object> state)
    {
        return new Dictionary<string, object>(state);
    }
    
    public override PlanStep ToStep() => new() { Type = PlanStepType.Hover, Duration = Duration_ };
}

public class SurveyAction : PlanningAction
{
    public override string Name => "Survey";
    public override double Cost => 50;
    public override double Duration => 300;
    
    public override bool CanExecute(Dictionary<string, object> state)
    {
        return state.TryGetValue("IsFlying", out var flying) && (bool)flying;
    }
    
    public override Dictionary<string, object> Apply(Dictionary<string, object> state)
    {
        var newState = new Dictionary<string, object>(state);
        newState["SurveyComplete"] = true;
        return newState;
    }
    
    public override PlanStep ToStep() => new() { Type = PlanStepType.Survey };
}

public class PlanState
{
    public Dictionary<string, object> State { get; set; } = new();
    public List<PlanningAction> Actions { get; set; } = new();
    public double Cost { get; set; }
}

#endregion

#region Situation Awareness

/// <summary>
/// Situation awareness and threat assessment.
/// </summary>
public class SituationAwareness
{
    private readonly DecisionConfig _config;
    
    public SituationAwareness(DecisionConfig config)
    {
        _config = config;
    }
    
    public SituationAwarenessResult Analyze(WorldState state)
    {
        var result = new SituationAwarenessResult();
        
        // Level 1: Perception
        result.PerceivedObjects = state.Obstacles.Count;
        result.GpsQuality = state.GpsSatellites >= 6 ? "Good" : state.GpsSatellites >= 4 ? "Fair" : "Poor";
        
        // Level 2: Comprehension
        result.IsInRestrictedArea = state.GeofenceViolation;
        result.BatteryStatus = state.BatteryPercent > 50 ? "Healthy" :
                               state.BatteryPercent > 20 ? "Low" : "Critical";
        result.WeatherConditions = state.WindSpeed < 5 ? "Favorable" :
                                   state.WindSpeed < 10 ? "Moderate" : "Adverse";
        
        // Level 3: Projection
        double flightTime = EstimateRemainingFlightTime(state);
        result.EstimatedFlightTimeRemaining = TimeSpan.FromMinutes(flightTime);
        result.CanCompleteCurrentMission = true; // Would check against mission requirements
        result.RecommendedAction = DetermineRecommendedAction(state, result);
        
        return result;
    }
    
    private double EstimateRemainingFlightTime(WorldState state)
    {
        double batteryRemaining = state.BatteryPercent;
        double consumptionRate = 0.5; // %/min estimate
        return batteryRemaining / consumptionRate;
    }
    
    private string DetermineRecommendedAction(WorldState state, SituationAwarenessResult awareness)
    {
        if (awareness.BatteryStatus == "Critical")
            return "Return to launch immediately";
        if (awareness.IsInRestrictedArea)
            return "Exit restricted area";
        if (awareness.WeatherConditions == "Adverse")
            return "Consider landing or RTL";
        return "Continue mission";
    }
}

/// <summary>
/// Risk and threat assessment.
/// </summary>
public class RiskAssessor
{
    private readonly DecisionConfig _config;
    
    public RiskAssessor(DecisionConfig config)
    {
        _config = config;
    }
    
    public List<ThreatAssessment> AssessThreats(WorldState state, SituationAwarenessResult awareness)
    {
        var threats = new List<ThreatAssessment>();
        
        // Battery threat
        if (state.BatteryPercent < 20)
        {
            threats.Add(new ThreatAssessment
            {
                Type = "LowBattery",
                Level = state.BatteryPercent < 10 ? ThreatLevel.Critical : ThreatLevel.High,
                Description = $"Battery at {state.BatteryPercent:F0}%",
                MitigationAction = "Return to launch"
            });
        }
        
        // Collision threat
        foreach (var obstacle in state.Obstacles)
        {
            double distance = (obstacle.Position - state.VehiclePosition).L2Norm();
            if (distance < 20)
            {
                threats.Add(new ThreatAssessment
                {
                    Type = "CollisionRisk",
                    Level = distance < 5 ? ThreatLevel.Critical : 
                            distance < 10 ? ThreatLevel.High : ThreatLevel.Medium,
                    Description = $"Obstacle at {distance:F1}m",
                    MitigationAction = "Avoid obstacle"
                });
            }
        }
        
        // Geofence threat
        if (state.GeofenceViolation)
        {
            threats.Add(new ThreatAssessment
            {
                Type = "GeofenceViolation",
                Level = ThreatLevel.Critical,
                Description = "Vehicle is outside geofence",
                MitigationAction = "Return to geofence"
            });
        }
        
        // Weather threat
        if (state.WindSpeed > 10)
        {
            threats.Add(new ThreatAssessment
            {
                Type = "HighWind",
                Level = state.WindSpeed > 15 ? ThreatLevel.High : ThreatLevel.Medium,
                Description = $"Wind speed: {state.WindSpeed:F1} m/s",
                MitigationAction = "Reduce altitude or land"
            });
        }
        
        return threats;
    }
}

/// <summary>
/// Mission intelligence for dynamic replanning.
/// </summary>
public class MissionIntelligence
{
    private readonly DecisionConfig _config;
    
    public MissionIntelligence(DecisionConfig config)
    {
        _config = config;
    }
    
    public ActionPlan? Replan(WorldState state, MissionState mission, 
        SituationAwarenessResult awareness, List<ThreatAssessment> threats)
    {
        if (mission.CurrentMission == null)
            return null;
        
        // Check if current plan is still valid
        var currentPlan = mission.CurrentPlan;
        if (currentPlan == null)
            return null;
        
        // Evaluate remaining plan feasibility
        double remainingDistance = EstimateRemainingDistance(state, currentPlan);
        double remainingTime = EstimateTime(remainingDistance, state);
        
        if (remainingTime > awareness.EstimatedFlightTimeRemaining.TotalSeconds * 0.8)
        {
            // Not enough battery - create abort plan
            return CreateAbortPlan(state);
        }
        
        // Check for obstacles in path
        bool pathBlocked = IsPathBlocked(state, currentPlan);
        if (pathBlocked)
        {
            return CreateAlternativePlan(state, currentPlan);
        }
        
        return currentPlan; // No replan needed
    }
    
    private double EstimateRemainingDistance(WorldState state, ActionPlan plan)
    {
        double total = 0;
        var pos = state.VehiclePosition;
        
        foreach (var step in plan.Steps.Skip(plan.CurrentStepIndex))
        {
            if (step.TargetPosition != null)
            {
                total += (step.TargetPosition - pos).L2Norm();
                pos = step.TargetPosition;
            }
        }
        
        return total;
    }
    
    private double EstimateTime(double distance, WorldState state)
    {
        double speed = state.VehicleVelocity.L2Norm();
        if (speed < 1) speed = 5; // Default speed
        return distance / speed;
    }
    
    private bool IsPathBlocked(WorldState state, ActionPlan plan)
    {
        foreach (var step in plan.Steps.Skip(plan.CurrentStepIndex))
        {
            if (step.TargetPosition != null)
            {
                foreach (var obstacle in state.Obstacles)
                {
                    // Simple line-obstacle intersection check
                    var toTarget = step.TargetPosition - state.VehiclePosition;
                    var toObstacle = obstacle.Position - state.VehiclePosition;
                    
                    double projection = (toTarget * toObstacle) / toTarget.L2Norm();
                    if (projection > 0 && projection < toTarget.L2Norm())
                    {
                        // Check perpendicular distance
                        var perpendicular = toObstacle - projection * toTarget.Normalize(2);
                        if (perpendicular.L2Norm() < obstacle.Radius + 5)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
    
    private ActionPlan CreateAbortPlan(WorldState state)
    {
        return new ActionPlan
        {
            Steps = new List<PlanStep>
            {
                new() { Type = PlanStepType.RTL }
            },
            EstimatedDuration = 60
        };
    }
    
    private ActionPlan CreateAlternativePlan(WorldState state, ActionPlan original)
    {
        // Create plan that avoids obstacles
        var newPlan = new ActionPlan();
        
        foreach (var step in original.Steps.Skip(original.CurrentStepIndex))
        {
            if (step.TargetPosition != null)
            {
                // Add intermediate waypoint to go around obstacle
                var altitudeOffset = Vector<double>.Build.DenseOfArray([0, 0, -10]); // Go up 10m
                var intermediatePos = state.VehiclePosition + altitudeOffset;
                
                newPlan.Steps.Add(new PlanStep
                {
                    Type = PlanStepType.GoTo,
                    TargetPosition = intermediatePos
                });
            }
            
            newPlan.Steps.Add(step);
        }
        
        return newPlan;
    }
}

#endregion

// Data types

public class DecisionConfig
{
    public int DecisionIntervalMs { get; set; } = 100;
    public double SafetyMargin { get; set; } = 5.0;
    public double MinBatteryForMission { get; set; } = 30;
}

public class WorldState
{
    public Vector<double> VehiclePosition { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> VehicleVelocity { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> VehicleAttitude { get; set; } = Vector<double>.Build.Dense(3);
    public double BatteryPercent { get; set; } = 100;
    public int GpsSatellites { get; set; } = 12;
    public double WindSpeed { get; set; }
    public double WindDirection { get; set; }
    public bool GeofenceViolation { get; set; }
    public List<Obstacle> Obstacles { get; set; } = new();
    public DateTime LastUpdateTime { get; set; }
    
    public Dictionary<string, object> ToSymbolic()
    {
        return new Dictionary<string, object>
        {
            ["Position"] = VehiclePosition,
            ["Velocity"] = VehicleVelocity,
            ["IsFlying"] = VehiclePosition[2] < -1,
            ["Altitude"] = -VehiclePosition[2],
            ["BatteryPercent"] = BatteryPercent
        };
    }
}

public class Obstacle
{
    public Vector<double> Position { get; set; } = Vector<double>.Build.Dense(3);
    public double Radius { get; set; } = 1;
    public string Type { get; set; } = "Unknown";
}

public class MissionState
{
    public MissionGoal? CurrentMission { get; set; }
    public ActionPlan? CurrentPlan { get; set; }
    public MissionStatus Status { get; set; }
}

public enum MissionStatus
{
    Pending,
    Active,
    Paused,
    Completed,
    Aborted
}

public class MissionGoal
{
    public string Name { get; set; } = "";
    public Vector<double>? TargetPosition { get; set; }
    public Dictionary<string, object> Conditions { get; set; } = new();
    public double Priority { get; set; } = 1;
}

public class ActionPlan
{
    public List<PlanStep> Steps { get; set; } = new();
    public int CurrentStepIndex { get; set; }
    public double EstimatedDuration { get; set; }
    
    public PlanStep? GetCurrentStep()
    {
        if (CurrentStepIndex < Steps.Count)
            return Steps[CurrentStepIndex];
        return null;
    }
}

public class PlanStep
{
    public PlanStepType Type { get; set; }
    public Vector<double>? TargetPosition { get; set; }
    public double Altitude { get; set; }
    public double Speed { get; set; } = 5;
    public double Duration { get; set; }
    
    public AutonomousAction ToAction()
    {
        return new AutonomousAction
        {
            Type = Type switch
            {
                PlanStepType.Takeoff => ActionType.Takeoff,
                PlanStepType.Land => ActionType.Land,
                PlanStepType.GoTo => ActionType.GoToPosition,
                PlanStepType.Hover => ActionType.Hover,
                PlanStepType.RTL => ActionType.ReturnToLaunch,
                PlanStepType.Survey => ActionType.Survey,
                _ => ActionType.Hover
            },
            TargetPosition = TargetPosition,
            Altitude = Altitude,
            Speed = Speed
        };
    }
}

public enum PlanStepType
{
    Takeoff,
    Land,
    GoTo,
    Hover,
    RTL,
    Survey,
    Orbit
}

public class SituationUpdate
{
    public UpdateType Type { get; set; }
    public Vector<double>? Position { get; set; }
    public Vector<double>? Velocity { get; set; }
    public Vector<double>? Attitude { get; set; }
    public double BatteryPercent { get; set; }
    public Obstacle? Obstacle { get; set; }
    public double WindSpeed { get; set; }
    public double WindDirection { get; set; }
    public bool GeofenceViolation { get; set; }
}

public enum UpdateType
{
    Position,
    Velocity,
    Attitude,
    Battery,
    Obstacle,
    Weather,
    Geofence
}

public class AutonomousAction
{
    public ActionType Type { get; set; }
    public Vector<double>? TargetPosition { get; set; }
    public double Altitude { get; set; }
    public double Speed { get; set; } = 5;
    public double Heading { get; set; }
    public ActionPriority Priority { get; set; }
    public string Reason { get; set; } = "";
}

public enum ActionType
{
    Hover,
    Takeoff,
    Land,
    GoToPosition,
    ReturnToLaunch,
    Survey,
    Orbit,
    AvoidObstacle,
    EmergencyLand
}

public enum ActionPriority
{
    Low,
    Normal,
    High,
    Emergency
}

public class SituationAwarenessResult
{
    public int PerceivedObjects { get; set; }
    public string GpsQuality { get; set; } = "";
    public bool IsInRestrictedArea { get; set; }
    public string BatteryStatus { get; set; } = "";
    public string WeatherConditions { get; set; } = "";
    public TimeSpan EstimatedFlightTimeRemaining { get; set; }
    public bool CanCompleteCurrentMission { get; set; }
    public string RecommendedAction { get; set; } = "";
}

public class ThreatAssessment
{
    public string Type { get; set; } = "";
    public ThreatLevel Level { get; set; }
    public string Description { get; set; } = "";
    public string MitigationAction { get; set; } = "";
}

public enum ThreatLevel
{
    None,
    Low,
    Medium,
    High,
    Critical
}

public class MissionReplan
{
    public string Reason { get; set; } = "";
    public ActionPlan NewPlan { get; set; } = new();
}

public class DecisionExplanation
{
    public DateTime Timestamp { get; set; }
    public List<string> Path { get; set; } = new();
    public AutonomousAction? FinalAction { get; set; }
    public string Reasoning { get; set; } = "";
}
