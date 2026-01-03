namespace ControlWorkbench.VEX.Learning;

/// <summary>
/// Advanced control theory tutorials for competitive robotics.
/// </summary>
public static class AdvancedTutorials
{
    public static Tutorial MotionProfiling => new()
    {
        Id = "motion-profiling",
        Title = "Motion Profiling & Trapezoidal Velocity",
        Category = TutorialCategory.Controls,
        Difficulty = Difficulty.Advanced,
        EstimatedTime = TimeSpan.FromMinutes(30),
        Description = "Learn how to create smooth acceleration profiles for consistent autonomous movement.",
        Sections = new List<TutorialSection>
        {
            new()
            {
                Title = "Why Motion Profiling?",
                Content = @"**The Problem with Basic PID:**
A PID controller trying to move 100 inches will output maximum power at the start (huge error), then slam on the brakes at the end. This causes:
- Wheel slip at the start
- Jerky movement
- Inconsistent timing
- Odometry errors

**Motion Profiling solves this by:**
- Limiting acceleration
- Limiting velocity
- Creating smooth, predictable motion
- Making autonomous timing consistent

**Types of Motion Profiles:**
1. **Trapezoidal** - Constant acceleration, cruise, deceleration
2. **S-Curve** - Smooth jerk-limited acceleration
3. **Sine** - Natural, smooth movement"
            },
            new()
            {
                Title = "Trapezoidal Profile Math",
                Content = @"A trapezoidal profile has three phases:

```
Velocity
   ?
vmax ???????????????????????
         ?             ?
         ?   Cruise    ?
         ?             ?
    ?????             ?????? Time
    Accel             Decel
```

**Parameters:**
- `max_velocity` - Top speed (in/s)
- `max_accel` - How fast to speed up (in/s²)
- `max_decel` - How fast to slow down (in/s²)
- `distance` - Target distance (in)

**Calculations:**
```
accel_time = max_velocity / max_accel
accel_distance = 0.5 * max_accel * accel_time²

decel_time = max_velocity / max_decel
decel_distance = 0.5 * max_decel * decel_time²

cruise_distance = distance - accel_distance - decel_distance
cruise_time = cruise_distance / max_velocity

total_time = accel_time + cruise_time + decel_time
```

**Note:** If `cruise_distance < 0`, you can't reach max velocity - use a triangular profile instead.",
                CodeSnippet = @"struct MotionProfile {
    double distance;
    double max_vel;
    double max_accel;
    double max_decel;
    
    double accel_time, cruise_time, decel_time;
    double accel_dist, cruise_dist;
    
    void calculate() {
        accel_time = max_vel / max_accel;
        double accel_dist_needed = 0.5 * max_accel * accel_time * accel_time;
        
        double decel_time_needed = max_vel / max_decel;
        double decel_dist_needed = 0.5 * max_decel * decel_time_needed * decel_time_needed;
        
        if (accel_dist_needed + decel_dist_needed > distance) {
            // Triangular profile - can't reach max velocity
            accel_time = sqrt(2 * distance * max_decel / 
                             (max_accel * (max_accel + max_decel)));
            cruise_time = 0;
            decel_time = accel_time * max_accel / max_decel;
            accel_dist = 0.5 * max_accel * accel_time * accel_time;
            cruise_dist = 0;
        } else {
            // Normal trapezoidal
            accel_dist = accel_dist_needed;
            cruise_dist = distance - accel_dist_needed - decel_dist_needed;
            cruise_time = cruise_dist / max_vel;
            decel_time = decel_time_needed;
        }
    }
    
    double get_velocity(double t) {
        if (t < accel_time) {
            return max_accel * t;
        } else if (t < accel_time + cruise_time) {
            return max_vel;
        } else if (t < accel_time + cruise_time + decel_time) {
            return max_vel - max_decel * (t - accel_time - cruise_time);
        }
        return 0;
    }
};"
            },
            new()
            {
                Title = "Combining with PID",
                Content = @"Motion profiling works **alongside** PID, not instead of it.

**Architecture:**
```
[Motion Profile] ? [Target Velocity] ? [PID] ? [Motor Output]
                          ?
                   [Feedforward] ??????????????? +
```

**Feedforward provides the base power:**
`ff_output = kV * target_velocity + kA * target_acceleration + kS * sign(velocity)`

**PID corrects for errors:**
`pid_output = kP * (target_position - actual_position)`

**Total output:**
`motor_power = ff_output + pid_output`

This is much more consistent than PID alone because:
- Most of the work is done by feedforward (predictable)
- PID only handles small corrections (easier to tune)",
                CodeSnippet = @"void follow_profile(MotionProfile& profile) {
    double start_pos = get_position();
    double start_time = pros::millis() / 1000.0;
    
    while (!profile.is_complete()) {
        double t = pros::millis() / 1000.0 - start_time;
        
        // Get targets from profile
        double target_vel = profile.get_velocity(t);
        double target_accel = profile.get_acceleration(t);
        double target_pos = start_pos + profile.get_position(t);
        
        // Feedforward
        double ff = kV * target_vel + kA * target_accel;
        if (target_vel != 0) ff += kS * signum(target_vel);
        
        // Feedback (PID on position error)
        double error = target_pos - get_position();
        double fb = kP * error;
        
        // Combined output
        set_motor_voltage(ff + fb);
        
        pros::delay(10);
    }
    
    set_motor_voltage(0);
}"
            }
        }
    };

    public static Tutorial OdometryDeepDive => new()
    {
        Id = "odometry-deep-dive",
        Title = "Odometry Deep Dive: 3-Wheel Tracking",
        Category = TutorialCategory.Odometry,
        Difficulty = Difficulty.Advanced,
        EstimatedTime = TimeSpan.FromMinutes(35),
        Description = "Understand the math behind 3-wheel odometry for accurate position tracking.",
        Sections = new List<TutorialSection>
        {
            new()
            {
                Title = "Why 3 Wheels?",
                Content = @"**2 Parallel Wheels give you:**
- Forward/backward distance
- Rotation angle

**But they miss:**
- Lateral (sideways) movement
- Getting pushed by other robots
- Drifting during turns

**The 3rd perpendicular wheel captures:**
- Any sideways movement
- Turning drift
- Being pushed sideways

**Physical Layout:**
```
        [Back Wheel]
             ?
             ? offset
     ?????????????????
    [Left]       [Right]
      ?????????????
          track
```"
            },
            new()
            {
                Title = "The Math",
                Content = @"**Variables:**
- `dL` = left wheel distance traveled
- `dR` = right wheel distance traveled  
- `dB` = back wheel distance traveled
- `trackWidth` = distance between left and right wheels
- `backOffset` = distance from center to back wheel

**Step 1: Calculate rotation**
```
dTheta = (dR - dL) / trackWidth
```

**Step 2: Calculate local displacement**

If `dTheta ? 0` (going straight):
```
localX = dB  // sideways
localY = (dL + dR) / 2  // forward
```

If `dTheta ? 0` (turning):
```
// Arc lengths to chord lengths
localY = 2 * sin(dTheta/2) * ((dL + dR) / (2 * dTheta))
localX = 2 * sin(dTheta/2) * (dB/dTheta + backOffset)
```

**Step 3: Rotate to global coordinates**
```
avgTheta = currentTheta + dTheta/2

globalX += localY * sin(avgTheta) + localX * cos(avgTheta)
globalY += localY * cos(avgTheta) - localX * sin(avgTheta)
currentTheta += dTheta
```",
                CodeSnippet = @"void update_odometry() {
    // Get encoder changes
    double dL = (left_encoder.get() - prev_left) * WHEEL_CIRC / 360.0;
    double dR = (right_encoder.get() - prev_right) * WHEEL_CIRC / 360.0;
    double dB = (back_encoder.get() - prev_back) * WHEEL_CIRC / 360.0;
    
    prev_left = left_encoder.get();
    prev_right = right_encoder.get();
    prev_back = back_encoder.get();
    
    // Calculate angle change
    double dTheta = (dR - dL) / TRACK_WIDTH;
    
    // Calculate local displacement
    double localX, localY;
    if (fabs(dTheta) < 1e-6) {
        // Going straight
        localX = dB;
        localY = (dL + dR) / 2.0;
    } else {
        // Turning
        localY = 2.0 * sin(dTheta / 2.0) * ((dL + dR) / (2.0 * dTheta));
        localX = 2.0 * sin(dTheta / 2.0) * (dB / dTheta + BACK_OFFSET);
    }
    
    // Rotate to global frame
    double avgTheta = theta + dTheta / 2.0;
    x += localY * sin(avgTheta) + localX * cos(avgTheta);
    y += localY * cos(avgTheta) - localX * sin(avgTheta);
    theta += dTheta;
}"
            },
            new()
            {
                Title = "Common Problems & Solutions",
                Content = @"**Problem: Drifting heading**
- Cause: Wheel diameter differences, mounting errors
- Solution: Measure track width empirically (turn 10 rotations, divide measured by expected)

**Problem: Position drifts over time**
- Cause: Accumulated small errors
- Solution: Use IMU for heading instead of wheel-based

**Problem: Jumpy readings**
- Cause: Encoder noise, wheel slip
- Solution: Filter readings, check for unreasonable jumps

**Calibration procedure:**
1. Place robot at known position
2. Drive forward 100 inches
3. Compare measured vs actual, adjust wheel diameter
4. Spin in place 10 full rotations
5. Compare measured vs actual (3600°), adjust track width
6. Strafe 50 inches
7. Compare measured vs actual, adjust back offset"
            }
        }
    };

    public static Tutorial PurePursuitAdvanced => new()
    {
        Id = "pure-pursuit-advanced",
        Title = "Pure Pursuit: Advanced Techniques",
        Category = TutorialCategory.PathFollowing,
        Difficulty = Difficulty.Expert,
        EstimatedTime = TimeSpan.FromMinutes(40),
        Description = "Master pure pursuit path following with adaptive lookahead and curvature optimization.",
        Sections = new List<TutorialSection>
        {
            new()
            {
                Title = "Adaptive Lookahead",
                Content = @"**Problem with fixed lookahead:**
- Too small on straights ? oscillation
- Too large on turns ? cuts corners

**Solution: Adapt lookahead based on:**
1. **Curvature** - smaller on tight turns
2. **Speed** - larger at high speed
3. **Error** - smaller when off path

**Formula:**
```
lookahead = base_lookahead * velocity_factor * curvature_factor

velocity_factor = 0.5 + 0.5 * (current_vel / max_vel)
curvature_factor = 1.0 / (1.0 + curvature * curvature_sensitivity)
```

**Typical values:**
- `base_lookahead` = 12-15 inches
- `curvature_sensitivity` = 5-10",
                CodeSnippet = @"double calculate_lookahead(double velocity, double curvature) {
    const double BASE_LOOKAHEAD = 12.0;
    const double MIN_LOOKAHEAD = 6.0;
    const double MAX_LOOKAHEAD = 24.0;
    const double CURVATURE_SENS = 8.0;
    
    double vel_factor = 0.5 + 0.5 * (velocity / MAX_VELOCITY);
    double curv_factor = 1.0 / (1.0 + fabs(curvature) * CURVATURE_SENS);
    
    double lookahead = BASE_LOOKAHEAD * vel_factor * curv_factor;
    return clamp(lookahead, MIN_LOOKAHEAD, MAX_LOOKAHEAD);
}"
            },
            new()
            {
                Title = "Finding the Pursuit Point",
                Content = @"The pursuit point is where the lookahead circle intersects the path.

**Algorithm:**
1. Find the closest point on path to robot
2. Search forward from that point
3. Find where path intersects lookahead circle
4. Use the furthest intersection (in path direction)

**Circle-line intersection:**
For line segment from P1 to P2, circle center C, radius r:
```
d = P2 - P1 (direction vector)
f = P1 - C (vector from center to line start)

a = d·d
b = 2 * f·d
c = f·f - r²

discriminant = b² - 4ac

if discriminant < 0: no intersection
t1 = (-b - sqrt(discriminant)) / (2a)
t2 = (-b + sqrt(discriminant)) / (2a)

Valid intersection if 0 ? t ? 1
Intersection point = P1 + t * d
```",
                CodeSnippet = @"Point find_pursuit_point(const Path& path, Point robot, double lookahead) {
    Point closest = path.get_closest_point(robot);
    int closest_idx = path.get_closest_index(robot);
    
    // Search forward through path segments
    for (int i = closest_idx; i < path.size() - 1; i++) {
        Point p1 = path[i];
        Point p2 = path[i + 1];
        
        // Circle-line intersection
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double fx = p1.x - robot.x;
        double fy = p1.y - robot.y;
        
        double a = dx*dx + dy*dy;
        double b = 2 * (fx*dx + fy*dy);
        double c = fx*fx + fy*fy - lookahead*lookahead;
        
        double disc = b*b - 4*a*c;
        if (disc >= 0) {
            disc = sqrt(disc);
            double t1 = (-b - disc) / (2*a);
            double t2 = (-b + disc) / (2*a);
            
            // Use t2 (further point) if valid
            if (t2 >= 0 && t2 <= 1) {
                return {p1.x + t2*dx, p1.y + t2*dy};
            }
            if (t1 >= 0 && t1 <= 1) {
                return {p1.x + t1*dx, p1.y + t1*dy};
            }
        }
    }
    
    // If no intersection, return end of path
    return path.back();
}"
            },
            new()
            {
                Title = "Curvature to Motor Speeds",
                Content = @"Once you have the pursuit point, calculate the curvature to reach it, then convert to wheel speeds.

**Curvature calculation:**
```
dx = pursuit.x - robot.x
dy = pursuit.y - robot.y

// Transform to robot frame
local_x = dx * cos(-heading) - dy * sin(-heading)
local_y = dx * sin(-heading) + dy * cos(-heading)

// Curvature = 2x / L² (where L = distance to point)
L = sqrt(local_x² + local_y²)
curvature = 2 * local_x / (L * L)
```

**Wheel speeds:**
```
target_vel = velocity_at_pursuit_point

left_vel = target_vel * (1 - curvature * track_width / 2)
right_vel = target_vel * (1 + curvature * track_width / 2)
```

**Important: Limit inside wheel speed, not outside**
If turning right, right wheel is inside and should be limited first.",
                CodeSnippet = @"void follow_path(Path& path) {
    while (!path.is_complete(get_pose())) {
        Pose robot = get_pose();
        double lookahead = calculate_lookahead(get_velocity(), path.get_curvature());
        Point pursuit = find_pursuit_point(path, robot, lookahead);
        
        // Transform to robot frame
        double dx = pursuit.x - robot.x;
        double dy = pursuit.y - robot.y;
        double local_x = dx * cos(-robot.heading) - dy * sin(-robot.heading);
        double local_y = dx * sin(-robot.heading) + dy * cos(-robot.heading);
        
        double L = sqrt(local_x*local_x + local_y*local_y);
        double curvature = 2 * local_x / (L * L);
        
        double target_vel = path.get_velocity_at(pursuit);
        
        double left_vel = target_vel * (1 - curvature * TRACK_WIDTH / 2);
        double right_vel = target_vel * (1 + curvature * TRACK_WIDTH / 2);
        
        // Limit speeds while preserving ratio
        double max_speed = std::max(fabs(left_vel), fabs(right_vel));
        if (max_speed > MAX_VEL) {
            left_vel *= MAX_VEL / max_speed;
            right_vel *= MAX_VEL / max_speed;
        }
        
        set_drive_velocity(left_vel, right_vel);
        pros::delay(10);
    }
    stop();
}"
            }
        }
    };

    public static List<Tutorial> GetAllAdvancedTutorials()
    {
        return new List<Tutorial>
        {
            MotionProfiling,
            OdometryDeepDive,
            PurePursuitAdvanced
        };
    }
}

/// <summary>
/// Code snippets library for common VEX patterns.
/// </summary>
public static class CodeSnippetLibrary
{
    public static Dictionary<string, CodeSnippet> Snippets { get; } = new()
    {
        ["exponential-drive"] = new CodeSnippet
        {
            Name = "Exponential Drive Curve",
            Description = "Makes driving feel more responsive at low speeds and controlled at high speeds",
            Language = "cpp",
            Code = @"// Exponential drive curve
// Makes driving feel more responsive and controlled
double exponential_curve(int input, double curve = 0.2) {
    // Normalize to -1 to 1
    double normalized = input / 127.0;
    
    // Apply exponential curve
    double curved = (1 - curve) * normalized + curve * normalized * normalized * normalized;
    
    // Scale back to motor range
    return curved * 127;
}

void opcontrol() {
    while (true) {
        int left_raw = master.get_analog(ANALOG_LEFT_Y);
        int right_raw = master.get_analog(ANALOG_RIGHT_Y);
        
        int left_curved = exponential_curve(left_raw, 0.3);
        int right_curved = exponential_curve(right_raw, 0.3);
        
        left_drive.move(left_curved);
        right_drive.move(right_curved);
        
        pros::delay(10);
    }
}"
        },

        ["hold-position"] = new CodeSnippet
        {
            Name = "Motor Position Hold",
            Description = "Hold a mechanism in position using PID when not being controlled",
            Language = "cpp",
            Code = @"class PositionHold {
    double target;
    double kP = 1.0;
    bool holding = false;
    pros::Motor& motor;
    
public:
    PositionHold(pros::Motor& m) : motor(m), target(0) {}
    
    void update(int manual_power) {
        if (manual_power != 0) {
            // Manual control
            motor.move(manual_power);
            holding = false;
        } else {
            // Hold position
            if (!holding) {
                target = motor.get_position();
                holding = true;
            }
            
            double error = target - motor.get_position();
            int power = (int)(kP * error);
            motor.move(power);
        }
    }
};

// Usage:
PositionHold lift_hold(lift_motor);

void opcontrol() {
    while (true) {
        int lift_input = 0;
        if (master.get_digital(DIGITAL_L1)) lift_input = 127;
        if (master.get_digital(DIGITAL_L2)) lift_input = -127;
        
        lift_hold.update(lift_input);
        pros::delay(10);
    }
}"
        },

        ["debounce-button"] = new CodeSnippet
        {
            Name = "Button Debouncing",
            Description = "Prevent rapid triggering of button presses",
            Language = "cpp",
            Code = @"class DebouncedButton {
    pros::controller_digital_e_t button;
    uint32_t last_press = 0;
    uint32_t debounce_ms;
    bool last_state = false;
    
public:
    DebouncedButton(pros::controller_digital_e_t btn, uint32_t debounce = 200)
        : button(btn), debounce_ms(debounce) {}
    
    bool pressed(pros::Controller& controller) {
        bool current = controller.get_digital(button);
        uint32_t now = pros::millis();
        
        if (current && !last_state && (now - last_press > debounce_ms)) {
            last_state = true;
            last_press = now;
            return true;
        }
        
        if (!current) {
            last_state = false;
        }
        
        return false;
    }
};

// Usage:
DebouncedButton mogo_button(DIGITAL_L1, 200);

void opcontrol() {
    while (true) {
        if (mogo_button.pressed(master)) {
            mogo_clamp.toggle();
        }
        pros::delay(10);
    }
}"
        },

        ["slew-rate"] = new CodeSnippet
        {
            Name = "Slew Rate Limiter",
            Description = "Gradually change motor power to prevent sudden jerks",
            Language = "cpp",
            Code = @"class SlewRateLimiter {
    double current_value = 0;
    double max_rate;  // Change per second
    uint32_t last_time;
    
public:
    SlewRateLimiter(double rate) : max_rate(rate), last_time(pros::millis()) {}
    
    double calculate(double target) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        
        double max_change = max_rate * dt;
        double change = target - current_value;
        
        if (fabs(change) > max_change) {
            change = max_change * (change > 0 ? 1 : -1);
        }
        
        current_value += change;
        return current_value;
    }
    
    void reset() {
        current_value = 0;
    }
};

// Usage:
SlewRateLimiter left_slew(200);  // 200 units/second max change
SlewRateLimiter right_slew(200);

void opcontrol() {
    while (true) {
        int left_target = master.get_analog(ANALOG_LEFT_Y);
        int right_target = master.get_analog(ANALOG_RIGHT_Y);
        
        left_drive.move(left_slew.calculate(left_target));
        right_drive.move(right_slew.calculate(right_target));
        
        pros::delay(10);
    }
}"
        },

        ["moving-average"] = new CodeSnippet
        {
            Name = "Moving Average Filter",
            Description = "Smooth noisy sensor readings",
            Language = "cpp",
            Code = @"template<size_t N>
class MovingAverage {
    double buffer[N] = {0};
    size_t index = 0;
    double sum = 0;
    bool filled = false;
    
public:
    double add(double value) {
        // Remove oldest value from sum
        sum -= buffer[index];
        
        // Add new value
        buffer[index] = value;
        sum += value;
        
        // Advance index
        index = (index + 1) % N;
        if (index == 0) filled = true;
        
        // Calculate average
        return sum / (filled ? N : index);
    }
    
    void reset() {
        for (size_t i = 0; i < N; i++) buffer[i] = 0;
        index = 0;
        sum = 0;
        filled = false;
    }
};

// Usage:
MovingAverage<10> distance_filter;  // 10-sample average

void opcontrol() {
    while (true) {
        double raw_distance = distance_sensor.get();
        double filtered = distance_filter.add(raw_distance);
        
        // Use filtered value
        if (filtered < 100) {
            // Object detected
        }
        
        pros::delay(20);
    }
}"
        }
    };
}

public class CodeSnippet
{
    public string Name { get; set; } = "";
    public string Description { get; set; } = "";
    public string Language { get; set; } = "cpp";
    public string Code { get; set; } = "";
}
