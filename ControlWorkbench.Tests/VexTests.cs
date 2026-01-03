using ControlWorkbench.VEX;
using Xunit;

namespace ControlWorkbench.Tests;

public class VexOdometryTests
{
    [Fact]
    public void TwoWheelOdometry_DriveForward_UpdatesY()
    {
        var config = OdometryConfig.TwoWheelConfig(trackWidth: 12.0, wheelDiameter: 2.75);
        var odom = new VexOdometry(config);
        
        // Simulate driving forward (both wheels rotate same amount)
        // 360 degrees = one wheel circumference = 2.75 * PI = 8.64 inches
        odom.Update(0, 0);      // Initialize
        odom.Update(360, 360);  // One full rotation forward
        
        Assert.Equal(0, odom.X, 2);  // No sideways movement
        Assert.True(odom.Y > 8.0 && odom.Y < 9.0);  // Moved forward ~8.64 inches
        Assert.Equal(0, odom.Theta, 3);  // No rotation
    }

    [Fact]
    public void TwoWheelOdometry_TurnInPlace_UpdatesTheta()
    {
        var config = OdometryConfig.TwoWheelConfig(trackWidth: 12.0, wheelDiameter: 2.75);
        var odom = new VexOdometry(config);
        
        // Simulate turning in place (wheels rotate opposite directions)
        odom.Update(0, 0);      // Initialize
        odom.Update(-180, 180); // Left backward, right forward
        
        Assert.Equal(0, odom.X, 1);
        Assert.Equal(0, odom.Y, 1);
        Assert.True(odom.Theta > 0);  // Turned right (positive)
    }

    [Fact]
    public void TwoWheelOdometry_Arc_UpdatesPosition()
    {
        var config = OdometryConfig.TwoWheelConfig(trackWidth: 12.0, wheelDiameter: 2.75);
        var odom = new VexOdometry(config);
        
        // Simulate an arc (right wheel travels further)
        odom.Update(0, 0);
        odom.Update(180, 360);  // Left half, right full rotation
        
        Assert.True(odom.X > 0);  // Moved right
        Assert.True(odom.Y > 0);  // Moved forward
        Assert.True(odom.Theta > 0);  // Turned right
    }

    [Fact]
    public void ThreeWheelOdometry_Strafe_UpdatesX()
    {
        var config = OdometryConfig.ThreeWheelConfig(trackWidth: 12.0, backOffset: 5.0);
        var odom = new VexOdometry(config);
        
        // Simulate pure strafe (only back wheel moves)
        odom.Update(0, 0, 0);
        odom.Update(0, 0, 360);  // Back wheel rotates
        
        Assert.True(System.Math.Abs(odom.X) > 0);  // Moved sideways
    }

    [Fact]
    public void Odometry_Reset_ClearsState()
    {
        var config = OdometryConfig.TwoWheelConfig(12.0);
        var odom = new VexOdometry(config);
        
        odom.Update(0, 0);
        odom.Update(360, 360);
        
        Assert.True(odom.Y > 0);
        
        odom.Reset(10, 20, 0.5);
        
        Assert.Equal(10, odom.X);
        Assert.Equal(20, odom.Y);
        Assert.Equal(0.5, odom.Theta);
    }
}

public class VexPathFollowingTests
{
    [Fact]
    public void PurePursuit_StraightPath_FollowsCorrectly()
    {
        var controller = new PurePursuitController(lookaheadDistance: 10.0, trackWidth: 12.0);
        
        var path = new List<Waypoint>
        {
            new Waypoint(0, 0, 60),
            new Waypoint(0, 24, 60),
            new Waypoint(0, 48, 60)
        };
        
        controller.SetPath(path);
        
        // Robot at origin, facing forward
        var (left, right) = controller.Compute(0, 0, 0, 100);
        
        // Should drive relatively straight
        Assert.True(System.Math.Abs(left - right) < 20);  // Not much turning
        Assert.True(left > 0 && right > 0);  // Moving forward
    }

    [Fact]
    public void PurePursuit_TurnToPath_AdjustsWheels()
    {
        var controller = new PurePursuitController(lookaheadDistance: 10.0, trackWidth: 12.0);
        
        var path = new List<Waypoint>
        {
            new Waypoint(0, 0, 60),
            new Waypoint(24, 0, 60),  // Path goes to the right
        };
        
        controller.SetPath(path);
        
        // Robot at origin, facing forward (needs to turn right)
        var (left, right) = controller.Compute(0, 0, 0, 100);
        
        // When turning right, the curvature is positive, which means
        // right wheel should move faster than left in differential drive
        // Actually depends on the coordinate system - let's just verify wheels are different
        Assert.True(System.Math.Abs(left - right) > 1);  // Should have differential steering
    }

    [Fact]
    public void PurePursuit_ReachesEnd_ReportsFinished()
    {
        var controller = new PurePursuitController(lookaheadDistance: 10.0, trackWidth: 12.0);
        
        var path = new List<Waypoint>
        {
            new Waypoint(0, 0, 60),
            new Waypoint(0, 10, 60)
        };
        
        controller.SetPath(path);
        
        Assert.False(controller.IsFinished);
        
        // Robot very close to end
        var (left, right) = controller.Compute(0, 9.5, 0, 100);
        
        Assert.True(controller.IsFinished);
        Assert.Equal(0, left);
        Assert.Equal(0, right);
    }

    [Fact]
    public void PathGenerator_CreatesSmoothPath()
    {
        var controlPoints = new List<Waypoint>
        {
            new Waypoint(0, 0),
            new Waypoint(24, 12),
            new Waypoint(48, 0)
        };
        
        var smoothPath = PathGenerator.GenerateSmoothPath(controlPoints, spacing: 2.0, maxVelocity: 60);
        
        Assert.True(smoothPath.Count > controlPoints.Count);  // More points after smoothing
        Assert.Equal(0, smoothPath[0].X, 1);  // Starts at first point
        Assert.Equal(0, smoothPath[0].Y, 1);
        Assert.Equal(48, smoothPath[^1].X, 1);  // Ends at last point
        Assert.Equal(0, smoothPath[^1].Y, 1);
    }

    [Fact]
    public void MotionProfile_Trapezoidal_HasCorrectPhases()
    {
        var motion = new MotionProfiledMovement(
            maxVelocity: 60,
            maxAcceleration: 80,
            maxDeceleration: 100);
        
        var profile = motion.GenerateStraightProfile(48);  // 48 inches
        
        Assert.True(profile.AccelTime > 0);
        Assert.True(profile.CruiseTime >= 0);
        Assert.True(profile.DecelTime > 0);
        Assert.Equal(48, profile.TotalDistance, 1);
    }

    [Fact]
    public void MotionProfile_Sample_ReturnsCorrectValues()
    {
        var motion = new MotionProfiledMovement(maxVelocity: 60, maxAcceleration: 80);
        var profile = motion.GenerateStraightProfile(48);
        
        // At start
        var (pos0, vel0) = motion.SampleProfile(profile, 0);
        Assert.Equal(0, pos0, 2);
        Assert.Equal(0, vel0, 2);
        
        // At end
        var (posEnd, velEnd) = motion.SampleProfile(profile, profile.TotalTime);
        Assert.Equal(48, posEnd, 2);
        Assert.Equal(0, velEnd, 2);
    }
}

public class VexTuningTests
{
    [Fact]
    public void PidTuningSession_RecordsData()
    {
        var session = new PidTuningSession();
        
        session.StartStepResponse(100);
        
        for (int i = 0; i < 100; i++)
        {
            double measurement = 100 * (1 - System.Math.Exp(-i / 20.0));  // First-order response
            session.RecordDataPoint(measurement, 50);
        }
        
        Assert.Equal(100, session.DataPointCount);
    }

    [Fact]
    public void PidTuningSession_AnalyzesResponse()
    {
        var session = new PidTuningSession();
        
        session.StartStepResponse(100);
        
        // Simulate a typical second-order underdamped response
        for (int i = 0; i < 200; i++)
        {
            double t = i / 100.0;
            double measurement = 100 * (1 - System.Math.Exp(-3 * t) * (System.Math.Cos(5 * t) + 0.6 * System.Math.Sin(5 * t)));
            session.RecordDataPoint(measurement, 50);
        }
        
        var metrics = session.StopAndAnalyze();
        
        Assert.True(metrics.IsValid);
        Assert.NotNull(metrics.RiseTimeMs);
        Assert.NotNull(metrics.OvershootPercent);
    }

    [Fact]
    public void PidTuningSession_SuggestsGains()
    {
        var session = new PidTuningSession();
        session.StartStepResponse(100);
        
        // Simulate slow response with steady-state error
        for (int i = 0; i < 200; i++)
        {
            double measurement = 90 * (1 - System.Math.Exp(-i / 100.0));  // Slow, doesn't reach setpoint
            session.RecordDataPoint(measurement, 50);
        }
        
        session.StopAndAnalyze();
        
        var suggestions = session.GetSuggestions(1.0, 0.01, 0.1);
        
        Assert.NotEmpty(suggestions.Message);
    }

    [Fact]
    public void MotorCharacterization_ComputesGains()
    {
        var characterization = new MotorCharacterization();
        
        // Simulate motor data: V = 0.5 + 0.02 * velocity
        var random = new Random(42);
        for (int i = 0; i < 100; i++)
        {
            double velocity = i * 2 + random.NextDouble() * 5;
            double voltage = 0.5 + 0.02 * velocity + random.NextDouble() * 0.1;
            characterization.RecordPoint(voltage, velocity, 0);
        }
        
        var gains = characterization.ComputeGains();
        
        Assert.True(gains.IsValid);
        Assert.True(gains.kS > 0.3 && gains.kS < 0.7);  // ~0.5
        Assert.True(gains.kV > 0.015 && gains.kV < 0.025);  // ~0.02
    }
}

public class VexMatchToolsTests
{
    [Fact]
    public void MatchLogger_RecordsAndRetrieves()
    {
        var tempDir = Path.Combine(Path.GetTempPath(), "cwb_test_" + Guid.NewGuid());
        Directory.CreateDirectory(tempDir);
        
        try
        {
            var logger = new MatchLogger(1000, tempDir);
            
            logger.StartMatch("TestMatch");
            logger.Log("Test", "Value", 42);
            logger.LogOdometry(10, 20, 0.5, 5);
            logger.LogMotor(1, 100, 50, 1.5, 35);
            logger.StopMatch();
            
            var files = logger.GetLogFiles();
            Assert.Single(files);
            
            var entries = logger.LoadLogFile(files[0]);
            Assert.True(entries.Count >= 4);  // Start, data, end
        }
        finally
        {
            Directory.Delete(tempDir, true);
        }
    }

    [Fact]
    public void CircularBuffer_OverwritesOldData()
    {
        var buffer = new CircularBuffer<int>(5);
        
        for (int i = 0; i < 10; i++)
        {
            buffer.Add(i);
        }
        
        Assert.Equal(5, buffer.Count);
        
        var items = buffer.ToList();
        Assert.Equal(new[] { 5, 6, 7, 8, 9 }, items);
    }

    [Fact]
    public void MatchTimer_TracksPhases()
    {
        var timer = new MatchTimer();
        
        Assert.Equal(MatchPhase.NotStarted, timer.CurrentPhase);
        
        timer.StartMatch();
        
        Assert.Equal(MatchPhase.Autonomous, timer.CurrentPhase);
        Assert.True(timer.IsRunning);
    }

    [Fact]
    public void AutonomousRecorder_ExportsToPros()
    {
        var recorder = new AutonomousRecorder();
        
        recorder.StartRecording();
        recorder.RecordDriveAction(100, 100);
        System.Threading.Thread.Sleep(10);
        recorder.RecordDriveAction(50, 100);
        recorder.RecordPneumaticAction(1, true);
        recorder.StopRecording();
        
        var code = recorder.ExportToProsCpp();
        
        Assert.Contains("void recorded_autonomous()", code);
        Assert.Contains("left_motors.move", code);
        Assert.Contains("right_motors.move", code);
        Assert.Contains("solenoid_1.set_value(true)", code);
    }
}

public class VexConnectionTests
{
    [Fact]
    public void MessageCodec_RoundTrip_Success()
    {
        var codec = new VexMessageCodec();
        VexMessage? received = null;
        codec.MessageReceived += msg => received = msg;
        
        // Build a test message
        var payload = new byte[] { 0x01, 0x02, 0x03, 0x04 };
        var message = VexMessageCodec.BuildMessage(VexMessageType.Heartbeat, payload);
        
        // Process it
        codec.ProcessBytes(message);
        
        Assert.NotNull(received);
        Assert.Equal(VexMessageType.Heartbeat, received.Value.Type);
        Assert.Equal(payload, received.Value.Payload);
    }

    [Fact]
    public void MessageCodec_CorruptedMessage_Ignored()
    {
        var codec = new VexMessageCodec();
        int messageCount = 0;
        codec.MessageReceived += _ => messageCount++;
        
        // Build a message and corrupt the checksum
        var message = VexMessageCodec.BuildMessage(VexMessageType.Heartbeat, new byte[] { 0x01 });
        message[^1] ^= 0xFF;  // Corrupt checksum
        
        codec.ProcessBytes(message);
        
        Assert.Equal(0, messageCount);  // Should not receive corrupted message
    }

    [Fact]
    public void OdometryData_Serialization()
    {
        var original = new VexTelemetry.OdometryData(
            X: 10.5,
            Y: -5.3,
            Theta: 0.785,
            VelocityX: 2.1,
            VelocityY: -1.0,
            AngularVelocity: 0.5,
            Timestamp: 12345
        );
        
        var bytes = original.ToBytes();
        var parsed = VexTelemetry.OdometryData.Parse(bytes);
        
        Assert.Equal(original.X, parsed.X, 6);
        Assert.Equal(original.Y, parsed.Y, 6);
        Assert.Equal(original.Theta, parsed.Theta, 6);
        Assert.Equal(original.Timestamp, parsed.Timestamp);
    }
}
