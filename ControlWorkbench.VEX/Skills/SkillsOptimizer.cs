using ControlWorkbench.VEX.PathPlanning;

namespace ControlWorkbench.VEX.Skills;

/// <summary>
/// Skills run optimizer - helps maximize autonomous and driver skills scores.
/// </summary>
public class SkillsOptimizer
{
    public VrcGame Game { get; set; } = VrcGame.HighStakes;
    public List<SkillsAction> Actions { get; } = new();
    public SkillsRun CurrentRun { get; private set; } = new();

    public event Action<int>? ScoreChanged;

    /// <summary>
    /// Start tracking a new skills run.
    /// </summary>
    public void StartRun(SkillsType type)
    {
        CurrentRun = new SkillsRun
        {
            Type = type,
            StartTime = DateTime.UtcNow
        };
        Actions.Clear();
    }

    /// <summary>
    /// Record a scoring action.
    /// </summary>
    public void RecordAction(SkillsActionType action, int points, string? description = null)
    {
        var elapsed = DateTime.UtcNow - CurrentRun.StartTime;

        Actions.Add(new SkillsAction
        {
            Type = action,
            Points = points,
            Timestamp = elapsed,
            Description = description ?? action.ToString()
        });

        CurrentRun.TotalScore = Actions.Sum(a => a.Points);
        ScoreChanged?.Invoke(CurrentRun.TotalScore);
    }

    /// <summary>
    /// Get optimal action sequence for High Stakes.
    /// </summary>
    public List<SkillsStrategyStep> GetHighStakesStrategy(bool isAutonomous)
    {
        var steps = new List<SkillsStrategyStep>();
        int timeLimit = isAutonomous ? 60 : 60;

        if (isAutonomous)
        {
            // Autonomous priority order
            steps.Add(new SkillsStrategyStep
            {
                Order = 1,
                Action = "Score preload on alliance stake",
                Points = 3,
                EstimatedTime = 3,
                Priority = StrategyPriority.Critical,
                Notes = "Quick and reliable points"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 2,
                Action = "Grab mobile goal",
                Points = 0,
                EstimatedTime = 4,
                Priority = StrategyPriority.Critical,
                Notes = "Need for ring scoring"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 3,
                Action = "Score 4-5 rings on mobile goal",
                Points = 4,  // 1 point each for rings on mogo
                EstimatedTime = 15,
                Priority = StrategyPriority.High,
                Notes = "Stack as many as possible"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 4,
                Action = "Score on neutral stake",
                Points = 5,
                EstimatedTime = 8,
                Priority = StrategyPriority.High,
                Notes = "Higher point value"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 5,
                Action = "Score on wall stake",
                Points = 3,
                EstimatedTime = 10,
                Priority = StrategyPriority.Medium,
                Notes = "If time permits"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 6,
                Action = "Place mobile goal in corner",
                Points = 6,
                EstimatedTime = 8,
                Priority = StrategyPriority.High,
                Notes = "Doubles ring points"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 7,
                Action = "Grab second mobile goal",
                Points = 0,
                EstimatedTime = 5,
                Priority = StrategyPriority.Medium,
                Notes = "For more rings"
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 8,
                Action = "Touch ladder (elevation)",
                Points = 3,
                EstimatedTime = 5,
                Priority = StrategyPriority.Low,
                Notes = "Last seconds"
            });
        }
        else
        {
            // Driver skills additions
            steps.Add(new SkillsStrategyStep
            {
                Order = 1,
                Action = "Alliance stake scoring",
                Points = 3,
                EstimatedTime = 2,
                Priority = StrategyPriority.Critical
            });

            steps.Add(new SkillsStrategyStep
            {
                Order = 2,
                Action = "Full mobile goal cycle (grab + fill + corner)",
                Points = 14,  // 6 rings + corner bonus
                EstimatedTime = 20,
                Priority = StrategyPriority.Critical
            });

            // Continue with more cycles...
        }

        // Calculate running totals
        int runningTime = 0;
        int runningScore = 0;
        foreach (var step in steps)
        {
            runningTime += step.EstimatedTime;
            runningScore += step.Points;
            step.CumulativeTime = runningTime;
            step.CumulativeScore = runningScore;
        }

        return steps;
    }

    /// <summary>
    /// Analyze a completed run and suggest improvements.
    /// </summary>
    public SkillsAnalysis AnalyzeRun(SkillsRun run)
    {
        var analysis = new SkillsAnalysis();

        // Calculate efficiency
        double totalTime = run.Duration.TotalSeconds;
        analysis.PointsPerSecond = run.TotalScore / totalTime;

        // Compare to theoretical max
        int theoreticalMax = CalculateTheoreticalMax(run.Type);
        analysis.TheoreticalMax = theoreticalMax;
        analysis.Efficiency = (double)run.TotalScore / theoreticalMax * 100;

        // Identify bottlenecks
        var sortedActions = run.Actions.OrderByDescending(a => a.Duration.TotalSeconds).ToList();
        foreach (var action in sortedActions.Take(3))
        {
            if (action.Duration.TotalSeconds > 5)
            {
                analysis.Bottlenecks.Add(new SkillsBottleneck
                {
                    Action = action.Description,
                    Duration = action.Duration,
                    Suggestion = GetImprovementSuggestion(action)
                });
            }
        }

        // Suggest improvements
        if (analysis.Efficiency < 50)
        {
            analysis.Suggestions.Add("Focus on consistent scoring before optimization");
            analysis.Suggestions.Add("Practice individual mechanisms separately");
        }
        else if (analysis.Efficiency < 70)
        {
            analysis.Suggestions.Add("Reduce transition times between scoring elements");
            analysis.Suggestions.Add("Consider parallel actions (intake while driving)");
        }
        else
        {
            analysis.Suggestions.Add("Optimize pathing for shortest distances");
            analysis.Suggestions.Add("Fine-tune motor speeds for max efficiency");
        }

        return analysis;
    }

    private int CalculateTheoreticalMax(SkillsType type)
    {
        return Game switch
        {
            VrcGame.HighStakes => type == SkillsType.Autonomous ? 48 : 78,
            VrcGame.OverUnder => type == SkillsType.Autonomous ? 45 : 65,
            VrcGame.SpinUp => type == SkillsType.Autonomous ? 150 : 180,
            _ => 100
        };
    }

    private string GetImprovementSuggestion(SkillsAction action)
    {
        return action.Type switch
        {
            SkillsActionType.Intake => "Increase intake speed or add more rollers",
            SkillsActionType.Score => "Pre-align before scoring, use sensors",
            SkillsActionType.Navigate => "Optimize path, use Pure Pursuit",
            SkillsActionType.Grab => "Add guide rails or use distance sensor",
            _ => "Practice this action more"
        };
    }
}

public class SkillsRun
{
    public SkillsType Type { get; set; }
    public DateTime StartTime { get; set; }
    public TimeSpan Duration => DateTime.UtcNow - StartTime;
    public int TotalScore { get; set; }
    public List<SkillsAction> Actions { get; } = new();
    public string Notes { get; set; } = "";
}

public class SkillsAction
{
    public SkillsActionType Type { get; set; }
    public int Points { get; set; }
    public TimeSpan Timestamp { get; set; }
    public TimeSpan Duration { get; set; }
    public string Description { get; set; } = "";
}

public enum SkillsActionType
{
    Score,
    Intake,
    Navigate,
    Grab,
    Release,
    Elevate,
    Other
}

public enum SkillsType
{
    Autonomous,
    Driver
}

public class SkillsStrategyStep
{
    public int Order { get; set; }
    public string Action { get; set; } = "";
    public int Points { get; set; }
    public int EstimatedTime { get; set; }  // seconds
    public int CumulativeTime { get; set; }
    public int CumulativeScore { get; set; }
    public StrategyPriority Priority { get; set; }
    public string Notes { get; set; } = "";
}

public enum StrategyPriority
{
    Critical,
    High,
    Medium,
    Low,
    Optional
}

public class SkillsAnalysis
{
    public double PointsPerSecond { get; set; }
    public int TheoreticalMax { get; set; }
    public double Efficiency { get; set; }
    public List<SkillsBottleneck> Bottlenecks { get; } = new();
    public List<string> Suggestions { get; } = new();
}

public class SkillsBottleneck
{
    public string Action { get; set; } = "";
    public TimeSpan Duration { get; set; }
    public string Suggestion { get; set; } = "";
}

/// <summary>
/// Scouting and match analysis for competition.
/// </summary>
public class ScoutingManager
{
    public List<ScoutedTeam> Teams { get; } = new();
    public List<ScoutedMatch> Matches { get; } = new();

    /// <summary>
    /// Add or update team data.
    /// </summary>
    public ScoutedTeam AddTeam(string teamNumber, string teamName = "")
    {
        var existing = Teams.FirstOrDefault(t => t.TeamNumber == teamNumber);
        if (existing != null)
            return existing;

        var team = new ScoutedTeam { TeamNumber = teamNumber, TeamName = teamName };
        Teams.Add(team);
        return team;
    }

    /// <summary>
    /// Record match result.
    /// </summary>
    public void RecordMatch(string matchId, string[] redTeams, string[] blueTeams, int redScore, int blueScore, string? notes = null)
    {
        var match = new ScoutedMatch
        {
            MatchId = matchId,
            RedTeams = redTeams.ToList(),
            BlueTeams = blueTeams.ToList(),
            RedScore = redScore,
            BlueScore = blueScore,
            Timestamp = DateTime.UtcNow,
            Notes = notes ?? ""
        };

        Matches.Add(match);

        // Update team stats
        foreach (var teamNum in redTeams)
        {
            var team = AddTeam(teamNum);
            team.MatchesPlayed++;
            team.TotalPoints += redScore;
            if (redScore > blueScore) team.Wins++;
            else if (redScore < blueScore) team.Losses++;
            else team.Ties++;
        }

        foreach (var teamNum in blueTeams)
        {
            var team = AddTeam(teamNum);
            team.MatchesPlayed++;
            team.TotalPoints += blueScore;
            if (blueScore > redScore) team.Wins++;
            else if (blueScore < redScore) team.Losses++;
            else team.Ties++;
        }
    }

    /// <summary>
    /// Get team rankings by OPR (Offensive Power Rating).
    /// </summary>
    public List<(string TeamNumber, double Opr)> CalculateOpr()
    {
        // Simplified OPR calculation
        // Real OPR uses linear algebra with match matrices
        var oprs = new Dictionary<string, double>();

        foreach (var team in Teams)
        {
            if (team.MatchesPlayed > 0)
            {
                // Simplified: average contribution
                oprs[team.TeamNumber] = team.TotalPoints / (double)team.MatchesPlayed / 2.0;
            }
        }

        return oprs.OrderByDescending(x => x.Value)
            .Select(x => (x.Key, x.Value))
            .ToList();
    }

    /// <summary>
    /// Predict match outcome.
    /// </summary>
    public MatchPrediction PredictMatch(string[] redTeams, string[] blueTeams)
    {
        var prediction = new MatchPrediction();

        double redOpr = 0, blueOpr = 0;
        var oprList = CalculateOpr().ToDictionary(x => x.TeamNumber, x => x.Opr);

        foreach (var team in redTeams)
        {
            if (oprList.TryGetValue(team, out var opr))
                redOpr += opr;
        }

        foreach (var team in blueTeams)
        {
            if (oprList.TryGetValue(team, out var opr))
                blueOpr += opr;
        }

        prediction.PredictedRedScore = (int)redOpr;
        prediction.PredictedBlueScore = (int)blueOpr;
        prediction.RedWinProbability = redOpr / (redOpr + blueOpr + 0.0001) * 100;
        prediction.BlueWinProbability = 100 - prediction.RedWinProbability;

        return prediction;
    }

    /// <summary>
    /// Get alliance recommendations for alliance selection.
    /// </summary>
    public List<AllianceRecommendation> GetAllianceRecommendations(string myTeam)
    {
        var recommendations = new List<AllianceRecommendation>();
        var oprList = CalculateOpr();
        var myOpr = oprList.FirstOrDefault(x => x.TeamNumber == myTeam).Opr;

        foreach (var (teamNumber, opr) in oprList)
        {
            if (teamNumber == myTeam) continue;

            var team = Teams.First(t => t.TeamNumber == teamNumber);
            var rec = new AllianceRecommendation
            {
                TeamNumber = teamNumber,
                TeamName = team.TeamName,
                Opr = opr,
                CombinedOpr = myOpr + opr,
                Compatibility = CalculateCompatibility(myTeam, teamNumber),
                Notes = team.Notes
            };

            recommendations.Add(rec);
        }

        return recommendations.OrderByDescending(r => r.CombinedOpr * r.Compatibility).ToList();
    }

    private double CalculateCompatibility(string team1, string team2)
    {
        // In real implementation, this would consider:
        // - Robot capabilities (complementary vs overlapping)
        // - Driving styles
        // - Past alliance performance
        return 1.0; // Default to neutral compatibility
    }

    /// <summary>
    /// Export scouting data to CSV.
    /// </summary>
    public string ExportToCsv()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("Team,Name,Matches,Wins,Losses,Ties,TotalPoints,AvgPoints,WinRate,Notes");

        foreach (var team in Teams.OrderByDescending(t => t.WinRate))
        {
            sb.AppendLine($"{team.TeamNumber},{team.TeamName},{team.MatchesPlayed},{team.Wins},{team.Losses},{team.Ties},{team.TotalPoints},{team.AveragePoints:F1},{team.WinRate:F1}%,\"{team.Notes}\"");
        }

        return sb.ToString();
    }
}

public class ScoutedTeam
{
    public string TeamNumber { get; set; } = "";
    public string TeamName { get; set; } = "";
    public int MatchesPlayed { get; set; }
    public int Wins { get; set; }
    public int Losses { get; set; }
    public int Ties { get; set; }
    public int TotalPoints { get; set; }
    public double AveragePoints => MatchesPlayed > 0 ? TotalPoints / (double)MatchesPlayed : 0;
    public double WinRate => MatchesPlayed > 0 ? Wins / (double)MatchesPlayed * 100 : 0;

    // Robot capabilities
    public bool CanScoreStakes { get; set; }
    public bool CanScoreMogoCorner { get; set; }
    public bool CanElevate { get; set; }
    public bool HasAutonomous { get; set; }
    public int MaxRings { get; set; }

    public string Notes { get; set; } = "";
}

public class ScoutedMatch
{
    public string MatchId { get; set; } = "";
    public List<string> RedTeams { get; set; } = new();
    public List<string> BlueTeams { get; set; } = new();
    public int RedScore { get; set; }
    public int BlueScore { get; set; }
    public DateTime Timestamp { get; set; }
    public string Notes { get; set; } = "";
}

public class MatchPrediction
{
    public int PredictedRedScore { get; set; }
    public int PredictedBlueScore { get; set; }
    public double RedWinProbability { get; set; }
    public double BlueWinProbability { get; set; }
}

public class AllianceRecommendation
{
    public string TeamNumber { get; set; } = "";
    public string TeamName { get; set; } = "";
    public double Opr { get; set; }
    public double CombinedOpr { get; set; }
    public double Compatibility { get; set; }
    public string Notes { get; set; } = "";
}
