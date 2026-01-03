namespace ControlWorkbench.VEX.Learning;

/// <summary>
/// Interactive tutorials and examples for learning VEX programming concepts.
/// </summary>
public class LearningCenter
{
    public List<Tutorial> Tutorials { get; } = new();
    public List<CodeExample> Examples { get; } = new();

    public LearningCenter()
    {
        LoadTutorials();
        LoadExamples();
    }

    private void LoadTutorials()
    {
        Tutorials.Add(new Tutorial
        {
            Id = "pid-basics",
            Title = "Understanding PID Control",
            Category = TutorialCategory.Controls,
            Difficulty = Difficulty.Beginner,
            EstimatedTime = TimeSpan.FromMinutes(15),
            Description = "Learn how PID controllers work and how to tune them for your robot.",
            Sections = new List<TutorialSection>
            {
                new TutorialSection
                {
                    Title = "What is PID?",
                    Content = @"PID stands for Proportional-Integral-Derivative. It's a control algorithm that helps your robot move accurately.

**The Problem:**
If you just tell a motor to spin at 100% power until you reach a target, you'll overshoot. 
If you slow down too early, you'll be sluggish.

**The Solution:**
PID calculates the perfect motor power based on:
- **P** (Proportional): How far are you from the target?
- **I** (Integral): Have you been stuck short of the target?
- **D** (Derivative): How fast are you approaching the target?

Think of it like driving to a stop sign:
- P = You slow down more as you get closer
- I = If you're rolling slowly and not reaching it, push harder
- D = If you're coming in too fast, brake harder"
                },
                new TutorialSection
                {
                    Title = "The Math (Simplified)",
                    Content = @"Don't worry - you don't need to understand all the math to use PID!

```
error = target - current_position
output = (kP * error) + (kI * total_error) + (kD * rate_of_change)
```

- **kP** (0.1 - 10): Higher = faster response, but can oscillate
- **kI** (0.001 - 0.1): Higher = eliminates steady-state error, but can overshoot  
- **kD** (0.01 - 1): Higher = dampens oscillations, but can be jerky

**Start with just P!** Many robots work fine with kI = 0 and kD = 0.",
                    CodeSnippet = @"// Simple P controller example
double kP = 2.0;

void drive_to_position(double target_inches) {
    double current = get_encoder_inches();
    double error = target - current;
    
    double power = kP * error;
    power = clamp(power, -127, 127);  // Limit motor power
    
    set_drive_power(power);
}"
                },
                new TutorialSection
                {
                    Title = "Tuning Your PID",
                    Content = @"**Step-by-step tuning process:**

1. **Set kI = 0, kD = 0**
2. **Increase kP** until the robot oscillates back and forth
3. **Reduce kP by 20%** to get a stable response
4. **If needed, add small kD** (start at kP/10) to reduce oscillations
5. **Only add kI if necessary** to eliminate small steady-state errors

**Quick Reference - Starting Values:**

| Application | kP | kI | kD |
|------------|----|----|---| 
| Drive Distance | 2.0 | 0.01 | 0.1 |
| Turning | 1.5 | 0.005 | 0.2 |
| Lift Position | 1.0 | 0.02 | 0.05 |
| Flywheel Speed | 0.5 | 0.1 | 0.01 |

**Pro Tip:** Use ControlWorkbench's auto-tuning feature to find good values automatically!"
                }
            }
        });

        Tutorials.Add(new Tutorial
        {
            Id = "odometry-intro",
            Title = "Introduction to Odometry",
            Category = TutorialCategory.Odometry,
            Difficulty = Difficulty.Intermediate,
            EstimatedTime = TimeSpan.FromMinutes(20),
            Description = "Learn how robots track their position on the field.",
            Sections = new List<TutorialSection>
            {
                new TutorialSection
                {
                    Title = "What is Odometry?",
                    Content = @"Odometry is how your robot knows where it is on the field.

**Why do you need it?**
- Navigate to specific field locations
- Follow paths accurately
- Correct for drift during autonomous

**How it works:**
Your robot uses sensors (usually encoders and an IMU) to track how far it has traveled and which direction it's facing.

**Basic idea:**
- Track left and right wheel rotations
- Calculate forward distance: (left + right) / 2
- Calculate turn angle: (right - left) / track_width
- Update X, Y, and heading based on these values"
                },
                new TutorialSection
                {
                    Title = "Coordinate Systems",
                    Content = @"VRC uses a standard coordinate system:

```
        +Y (144"")
         ?
         ?   (Your robot starts here)
         ?   ? heading = 0°
         ?
?????????????????? +X (144"")
         ?
         ?
         ?
```

- Origin (0, 0) is typically at the center of the field or a corner
- Heading is usually 0° pointing up (positive Y direction)
- Positive angles are counter-clockwise

**Common coordinate systems:**
- Field-centric: (0,0) at field center
- Corner-centric: (0,0) at a field corner
- Robot-centric: (0,0) at robot center, heading always 0"
                },
                new TutorialSection
                {
                    Title = "Setting Up Odometry",
                    Content = @"**Option 1: Built-in Motor Encoders (Simplest)**
Use the encoders in your drive motors. Works okay but can slip.

**Option 2: Tracking Wheels (More Accurate)**
Unpowered wheels with rotation sensors. Can't slip because they're not driving.

**Option 3: GPS Sensor (Easiest but Limited)**
V5 GPS sensor gives you position directly, but can be blocked.

**Recommended Setup for Accuracy:**
- 2 parallel tracking wheels (left/right)
- 1 perpendicular tracking wheel (for strafing/drift)
- 1 IMU for heading (more accurate than wheel-based heading)",
                    CodeSnippet = @"// LemLib odometry setup
lemlib::OdomSensors sensors {
    &left_tracking_wheel,   // vertical wheel 1
    &right_tracking_wheel,  // vertical wheel 2
    &horizontal_wheel,      // horizontal wheel
    nullptr,                // horizontal wheel 2 (not used)
    &imu                    // inertial sensor
};

// Call this to get position
double x = chassis.getPose().x;
double y = chassis.getPose().y;
double heading = chassis.getPose().theta;"
                }
            }
        });

        Tutorials.Add(new Tutorial
        {
            Id = "auton-programming",
            Title = "Writing Autonomous Routines",
            Category = TutorialCategory.Autonomous,
            Difficulty = Difficulty.Beginner,
            EstimatedTime = TimeSpan.FromMinutes(25),
            Description = "Step-by-step guide to creating winning autonomous routines.",
            Sections = new List<TutorialSection>
            {
                new TutorialSection
                {
                    Title = "Planning Your Autonomous",
                    Content = @"**Before you write any code:**

1. **Know the game** - Understand scoring, bonuses, and timing
2. **Identify priorities** - What gives the most points per second?
3. **Map your route** - Draw it on paper or use ControlWorkbench's path planner
4. **Time your actions** - Practice each step manually to know how long it takes
5. **Have a backup plan** - What if something fails?

**Match Autonomous (15 seconds):**
- Focus on 2-3 high-value actions
- Reliable > risky
- Consider your alliance partner

**Skills Autonomous (60 seconds):**
- Maximize points per second
- Full field coverage
- More complex multi-step routines"
                },
                new TutorialSection
                {
                    Title = "Basic Structure",
                    Content = @"A typical autonomous has these phases:

1. **Pre-position actions** - Deploy, prepare mechanisms
2. **Movement 1** - Drive to first scoring location  
3. **Score 1** - Execute scoring action
4. **Movement 2** - Drive to next location
5. **Score 2** - Execute next action
6. ...repeat...
7. **End position** - Get ready for driver control",
                    CodeSnippet = @"void autonomous() {
    // Phase 1: Score preload
    intake.run(127);
    chassis.moveToPoint(12, 0, 2000);
    intake.stop();
    
    // Phase 2: Grab mobile goal
    chassis.moveToPoint(-24, 12, 3000);
    mogo_clamp.extend();
    pros::delay(200);
    
    // Phase 3: Score rings on mogo
    chassis.moveToPoint(-24, 36, 3000);
    intake.run(127);
    pros::delay(1000);
    
    // Phase 4: Place in corner
    chassis.moveToPoint(6, 6, 3000);
    mogo_clamp.retract();
}"
                },
                new TutorialSection
                {
                    Title = "Common Patterns",
                    Content = @"**Wait Until Done:**
```cpp
chassis.moveToPoint(x, y, timeout);
chassis.waitUntilDone();  // Blocks until movement complete
```

**Parallel Actions:**
```cpp
// Start movement
chassis.moveToPoint(x, y, timeout, {.async = true});
// Do something while moving
intake.run(127);
chassis.waitUntilDone();  // Wait for movement
intake.stop();
```

**Conditional Logic:**
```cpp
if (optical_sensor.get_hue() < 30) {
    // Red object detected
    intake.run(127);
} else {
    intake.run(-127);  // Reject
}
```

**Timeout Protection:**
```cpp
uint32_t start = pros::millis();
while (!at_target() && pros::millis() - start < 3000) {
    // Keep trying for 3 seconds max
    pros::delay(10);
}
```"
                },
                new TutorialSection
                {
                    Title = "Testing Tips",
                    Content = @"**Test incrementally:**
1. Test each movement separately first
2. Combine movements one at a time
3. Test full routine multiple times

**Use the simulator:**
ControlWorkbench's simulator lets you test autonomous code without wearing out your robot!

**Debug output:**
```cpp
pros::lcd::print(0, ""X: %.1f  Y: %.1f"", chassis.getPose().x, chassis.getPose().y);
printf(""Position: %.1f, %.1f\n"", x, y);  // USB output
```

**Record and replay:**
Use ControlWorkbench to record a driver run, then convert it to autonomous!"
                }
            }
        });

        Tutorials.Add(new Tutorial
        {
            Id = "pure-pursuit",
            Title = "Pure Pursuit Path Following",
            Category = TutorialCategory.PathFollowing,
            Difficulty = Difficulty.Advanced,
            EstimatedTime = TimeSpan.FromMinutes(30),
            Description = "Learn how Pure Pursuit algorithm works and how to use it.",
            Sections = new List<TutorialSection>
            {
                new TutorialSection
                {
                    Title = "What is Pure Pursuit?",
                    Content = @"Pure Pursuit is a path-following algorithm that makes your robot smoothly follow a curved path.

**The Basic Idea:**
Imagine you're chasing a carrot on a stick. The carrot is always a fixed distance ahead of you on the path. You just keep steering toward the carrot, and you naturally follow the path.

**Key Concepts:**
- **Path**: A list of waypoints (x, y coordinates)
- **Lookahead**: How far ahead on the path to aim
- **Pursuit Point**: The point on the path you're currently chasing

**Why use Pure Pursuit?**
- Smooth, curved movements
- Handles paths with turns naturally
- Less jerky than point-to-point driving"
                },
                new TutorialSection
                {
                    Title = "Lookahead Distance",
                    Content = @"The lookahead distance is the most important tuning parameter.

**Small lookahead (6-10 inches):**
? Follows path precisely
? Handles tight turns
? Can be jerky
? Can oscillate

**Large lookahead (15-24 inches):**
? Smooth movement
? Stable
? Cuts corners
? May miss tight turns

**Recommended:** Start with 12-15 inches and adjust based on:
- Your robot's size
- Path complexity
- Required accuracy",
                    CodeSnippet = @"// LemLib pure pursuit
ASSET(example_path_txt) = ""0, 0, 90\n24, 24, 90\n48, 0, 90"";

void follow_path() {
    // Parameters: path, lookahead (inches), timeout (ms)
    chassis.follow(example_path_txt, 15, 4000);
    chassis.waitUntilDone();
}"
                }
            }
        });
    }

    private void LoadExamples()
    {
        Examples.Add(new CodeExample
        {
            Id = "simple-drive",
            Title = "Tank Drive Control",
            Category = ExampleCategory.Drivetrain,
            Difficulty = Difficulty.Beginner,
            Description = "Basic tank drive implementation with deadband.",
            Code = @"void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_drive({-1, -2, -3});
    pros::MotorGroup right_drive({4, 5, 6});
    
    while (true) {
        int left = master.get_analog(ANALOG_LEFT_Y);
        int right = master.get_analog(ANALOG_RIGHT_Y);
        
        // Apply deadband (ignore small values)
        if (abs(left) < 10) left = 0;
        if (abs(right) < 10) right = 0;
        
        left_drive.move(left);
        right_drive.move(right);
        
        pros::delay(10);
    }
}",
            Explanation = @"This is the simplest form of drive control:
1. Read joystick values (-127 to 127)
2. Apply deadband to prevent drift
3. Send directly to motors
4. Small delay to prevent CPU overload"
        });

        Examples.Add(new CodeExample
        {
            Id = "arcade-drive",
            Title = "Arcade Drive Control",
            Category = ExampleCategory.Drivetrain,
            Difficulty = Difficulty.Beginner,
            Description = "Single-stick arcade drive with turning.",
            Code = @"void arcade_drive(int forward, int turn) {
    left_drive.move(forward + turn);
    right_drive.move(forward - turn);
}

void opcontrol() {
    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        
        arcade_drive(forward, turn);
        pros::delay(10);
    }
}",
            Explanation = @"Arcade drive uses one stick for forward/back and another for turning.
Adding forward and turn gives left motor power.
Subtracting gives right motor power.
This creates intuitive single-stick control."
        });

        Examples.Add(new CodeExample
        {
            Id = "pid-drive",
            Title = "PID Drive Distance",
            Category = ExampleCategory.PID,
            Difficulty = Difficulty.Intermediate,
            Description = "Drive a specific distance using PID control.",
            Code = @"void drive_distance(double inches) {
    double kP = 2.0, kI = 0.01, kD = 0.1;
    double integral = 0, prev_error = 0;
    double target = left_drive.get_position() + inches_to_ticks(inches);
    
    uint32_t start = pros::millis();
    int settle_count = 0;
    
    while (pros::millis() - start < 5000) {  // 5 second timeout
        double current = left_drive.get_position();
        double error = target - current;
        
        // Check if settled
        if (fabs(error) < 10) {  // 10 ticks tolerance
            settle_count++;
            if (settle_count > 20) break;  // Settled for 200ms
        } else {
            settle_count = 0;
        }
        
        // PID calculation
        integral += error;
        double derivative = error - prev_error;
        prev_error = error;
        
        double power = (kP * error) + (kI * integral) + (kD * derivative);
        power = std::clamp(power, -127.0, 127.0);
        
        left_drive.move(power);
        right_drive.move(power);
        
        pros::delay(10);
    }
    
    left_drive.move(0);
    right_drive.move(0);
}",
            Explanation = @"This PID loop:
1. Calculates error (how far from target)
2. Accumulates integral (total error over time)
3. Computes derivative (rate of error change)
4. Combines with tuned gains
5. Checks for settled condition
6. Has timeout protection"
        });

        Examples.Add(new CodeExample
        {
            Id = "color-sorting",
            Title = "Color Sorting Intake",
            Category = ExampleCategory.Sensors,
            Difficulty = Difficulty.Intermediate,
            Description = "Use optical sensor to sort game objects by color.",
            Code = @"bool is_our_color(double hue) {
    if (alliance == RED) {
        return hue < 30 || hue > 330;  // Red hue
    } else {
        return hue > 180 && hue < 260;  // Blue hue
    }
}

void smart_intake() {
    while (true) {
        double hue = optical_sensor.get_hue();
        bool object_detected = optical_sensor.get_proximity() > 100;
        
        if (object_detected) {
            if (is_our_color(hue)) {
                intake.move(127);    // Keep it
            } else {
                intake.move(-80);    // Reject!
                pros::delay(200);    // Time to eject
            }
        } else if (master.get_digital(DIGITAL_R1)) {
            intake.move(127);        // Manual intake
        } else if (master.get_digital(DIGITAL_R2)) {
            intake.move(-127);       // Manual outtake
        } else {
            intake.move(0);
        }
        
        pros::delay(10);
    }
}",
            Explanation = @"This smart intake:
1. Reads color from optical sensor
2. Checks if object is detected (proximity)
3. Compares to alliance color
4. Keeps correct color, ejects wrong color
5. Falls back to manual control when no detection"
        });

        Examples.Add(new CodeExample
        {
            Id = "mogo-detection",
            Title = "Mobile Goal Pickup with Distance Sensor",
            Category = ExampleCategory.Sensors,
            Difficulty = Difficulty.Intermediate,
            Description = "Automatically detect and grab mobile goals.",
            Code = @"void auto_grab_mogo() {
    const double DETECT_DISTANCE = 100;  // mm
    const double GRAB_DISTANCE = 30;     // mm
    
    // Drive forward slowly
    while (distance_sensor.get() > GRAB_DISTANCE) {
        if (distance_sensor.get() < DETECT_DISTANCE) {
            // Mogo detected, slow approach
            left_drive.move(40);
            right_drive.move(40);
        } else {
            // Searching, normal speed
            left_drive.move(60);
            right_drive.move(60);
        }
        pros::delay(10);
    }
    
    // In position, grab it!
    left_drive.move(0);
    right_drive.move(0);
    mogo_clamp.extend();
    pros::delay(200);
}",
            Explanation = @"This uses a distance sensor to:
1. Search for mobile goal at normal speed
2. Slow down when detected
3. Stop at optimal grab distance
4. Activate clamp mechanism"
        });

        Examples.Add(new CodeExample
        {
            Id = "toggle-pneumatic",
            Title = "Toggle Pneumatic with Button",
            Category = ExampleCategory.Pneumatics,
            Difficulty = Difficulty.Beginner,
            Description = "Toggle a pneumatic device with a single button press.",
            Code = @"void opcontrol() {
    pros::adi::Pneumatics clamp('A', false);
    bool clamp_extended = false;
    
    while (true) {
        // Toggle on new press (not held)
        if (master.get_digital_new_press(DIGITAL_L1)) {
            clamp_extended = !clamp_extended;
            clamp.set_value(clamp_extended);
        }
        
        // Other controls...
        pros::delay(10);
    }
}",
            Explanation = @"Key points:
1. get_digital_new_press() only triggers once per press
2. Toggle boolean with !
3. set_value() controls the solenoid
4. false = retracted, true = extended"
        });
    }

    /// <summary>
    /// Get tutorials by category.
    /// </summary>
    public IEnumerable<Tutorial> GetTutorialsByCategory(TutorialCategory category)
    {
        return Tutorials.Where(t => t.Category == category);
    }

    /// <summary>
    /// Get examples by difficulty.
    /// </summary>
    public IEnumerable<CodeExample> GetExamplesByDifficulty(Difficulty difficulty)
    {
        return Examples.Where(e => e.Difficulty == difficulty);
    }

    /// <summary>
    /// Search tutorials and examples.
    /// </summary>
    public (List<Tutorial> tutorials, List<CodeExample> examples) Search(string query)
    {
        var q = query.ToLower();
        var tutorials = Tutorials.Where(t =>
            t.Title.ToLower().Contains(q) ||
            t.Description.ToLower().Contains(q)).ToList();

        var examples = Examples.Where(e =>
            e.Title.ToLower().Contains(q) ||
            e.Description.ToLower().Contains(q) ||
            e.Code.ToLower().Contains(q)).ToList();

        return (tutorials, examples);
    }
}

public class Tutorial
{
    public string Id { get; set; } = "";
    public string Title { get; set; } = "";
    public TutorialCategory Category { get; set; }
    public Difficulty Difficulty { get; set; }
    public TimeSpan EstimatedTime { get; set; }
    public string Description { get; set; } = "";
    public List<TutorialSection> Sections { get; set; } = new();
}

public class TutorialSection
{
    public string Title { get; set; } = "";
    public string Content { get; set; } = "";
    public string? CodeSnippet { get; set; }
    public string? ImagePath { get; set; }
}

public enum TutorialCategory
{
    GettingStarted,
    Drivetrain,
    Controls,
    Sensors,
    Autonomous,
    Odometry,
    PathFollowing,
    Mechanisms,
    Competition
}

public class CodeExample
{
    public string Id { get; set; } = "";
    public string Title { get; set; } = "";
    public ExampleCategory Category { get; set; }
    public Difficulty Difficulty { get; set; }
    public string Description { get; set; } = "";
    public string Code { get; set; } = "";
    public string Explanation { get; set; } = "";
}

public enum ExampleCategory
{
    Drivetrain,
    PID,
    Sensors,
    Pneumatics,
    Motors,
    Autonomous,
    PathFollowing
}

public enum Difficulty
{
    Beginner,
    Intermediate,
    Advanced,
    Expert
}
