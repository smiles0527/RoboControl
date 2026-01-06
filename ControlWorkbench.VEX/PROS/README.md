# ControlWorkbench PROS Library

<div align="center">

**The most comprehensive VEX V5 robotics library for PROS**

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/controlworkbench/pros-library)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![PROS](https://img.shields.io/badge/PROS-4.0-orange.svg)](https://pros.cs.purdue.edu/)

</div>

---

## ? Features

| Feature | Description |
|---------|-------------|
| ?? **Chassis Control** | Pure Pursuit, Boomerang, move-to-point, turn-to-heading |
| ?? **Odometry** | Tracking wheels or motor encoders with IMU fusion |
| ??? **PID Controllers** | Anti-windup, derivative filtering, motion profiling |
| ?? **Telemetry** | Real-time data streaming to ControlWorkbench |
| ?? **Remote Tuning** | Adjust PID gains live without re-uploading |
| ?? **Subsystems** | Ready-to-use intake, lift, pneumatic, flywheel |
| ?? **Auton Selector** | Brain screen selector with descriptions |
| ??? **Driver Curves** | Exponential, Desmos, deadzone control |
| ?? **Match Timer** | Endgame warnings, skills timing |
| ?? **Emergency Stop** | Remote kill switch for safety |

---

## ?? Installation (30 seconds)

1. **Copy** the `cwb` folder to your PROS project's `include` directory

2. **In ONE .cpp file** (usually `main.cpp`):
   ```cpp
   #define CWB_IMPLEMENTATION
   #include "cwb/cwb.hpp"
   ```

3. **In other files**, just:
   ```cpp
   #include "cwb/cwb.hpp"
   ```

That's it! No Makefile changes, no dependencies.

---

## ?? Quick Start

```cpp
#define CWB_IMPLEMENTATION
#include "cwb/cwb.hpp"

// Hardware
pros::MotorGroup left_drive({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({4, 5, 6}, pros::MotorGearset::blue);
pros::Imu imu(10);

// CWB Objects
cwb::Chassis chassis(left_drive, right_drive, imu);
cwb::AutonSelector selector;
cwb::Controller master(pros::E_CONTROLLER_MASTER);

void initialize() {
    cwb::init();
    chassis.calibrate();
    
    // Add autons
    selector.add("Red +", red_positive);
    selector.add("Skills", skills);
    selector.init();
    
    // CRITICAL: Emergency stop
    cwb::on_emergency_stop([]() {
        left_drive.move(0);
        right_drive.move(0);
    });
}

void autonomous() {
    selector.run();
}

void opcontrol() {
    master.set_curve(cwb::DriverCurve::desmos, 5.0);
    
    while (true) {
        cwb::update();
        chassis.update();
        chassis.arcade(master.get_left_y(), master.get_right_x());
        pros::delay(10);
    }
}
```

---

## ?? Complete API Reference

### Chassis Control

```cpp
cwb::Chassis chassis(left_motors, right_motors, imu);

// Configuration
cwb::ChassisConfig config;
config.wheel_diameter = 3.25;
config.track_width = 12.5;
chassis.configure(config);

// Optional: Tracking wheels
chassis.set_tracking_wheels(&left_tracker, &right_tracker, 2.75);

// Motion commands
chassis.move_to_point(24, 24);              // Drive to point
chassis.turn_to_heading(90);                // Turn to heading
chassis.move_distance(24);                  // Drive forward
chassis.turn_angle(90);                     // Turn right
chassis.follow_path(path);                  // Pure Pursuit
chassis.boomerang(24, 24, 90);              // Curved approach

// With options
chassis.move_to_point(24, 24, cwb::MoveOptions()
    .set_max_speed(80)
    .set_timeout(3000)
    .set_forwards(false));

// Driver control
chassis.arcade(forward, turn);
chassis.tank(left, right);
chassis.curvature(throttle, curve);
```

### Subsystems

```cpp
// Intake with color sorting
cwb::Intake intake(motor, optical);
intake.set_target_color(cwb::Intake::Color::Red);  // Keep red, eject others
intake.enable_sorting(true);
intake.run();
intake.update();  // Call every loop for sorting

// Lift with presets
cwb::Lift lift(motor_group);
lift.add_position("down", 0);
lift.add_position("score", 180);
lift.add_position("high", 360);
lift.go_to("score");
lift.update();  // Call every loop

// Pneumatics
cwb::Pneumatic clamp('A');             // Single-acting
cwb::Pneumatic shifter('B', 'C');      // Double-acting
clamp.toggle();
clamp.extend();
clamp.retract();

// Flywheel with velocity control
cwb::Flywheel flywheel(motor);
flywheel.set_target_rpm(3000);
if (flywheel.at_speed()) { /* ready to shoot */ }
flywheel.update();  // Call every loop
```

### Autonomous Selector

```cpp
cwb::AutonSelector selector;

// Add routines with optional descriptions
selector.add("Red +", red_positive, "Red side, positive corner");
selector.add("Red -", red_negative, "Red side, negative corner");
selector.add("Skills", skills, "60-second skills run");

// In initialize():
selector.init();  // Shows on brain screen

// In autonomous():
selector.run();   // Runs selected routine
```

### Driver Control

```cpp
cwb::Controller master(pros::E_CONTROLLER_MASTER);

// Apply input curve
master.set_curve(cwb::DriverCurve::desmos, 5.0);  // Smooth curve
master.set_curve(cwb::DriverCurve::exponential, 2.0);  // More aggressive
master.set_deadzone(5);

// Get curved inputs
int forward = master.get_left_y();   // Curve applied
int turn = master.get_right_x();     // Curve applied

// Button helpers
if (master.pressed(DIGITAL_A)) { }   // New press only
if (master.held(DIGITAL_A)) { }      // While held

// Rumble feedback
master.rumble("..--");
master.rumble_short();
master.rumble_long();
```

### Match Timer

```cpp
// Start timer
cwb::MatchTimer::start(cwb::MatchTimer::Mode::Autonomous);  // 15 sec
cwb::MatchTimer::start(cwb::MatchTimer::Mode::Driver);      // 1:45
cwb::MatchTimer::start(cwb::MatchTimer::Mode::Skills);      // 60 sec

// Check time
int ms = cwb::MatchTimer::remaining_ms();
double sec = cwb::MatchTimer::remaining_sec();
std::string time = cwb::MatchTimer::format_remaining();  // "1:23"

// Endgame warning
if (cwb::MatchTimer::is_endgame(30000)) {
    master.rumble("-");  // 30 seconds left!
}
```

### PID Controllers

```cpp
cwb::PIDController pid("name", kp, ki, kd);

// Compute output
double output = pid.compute(error, dt);
double output = pid.compute(target, measurement, dt);

// Check if settled
if (pid.is_settled(1.0, 100)) { }  // Within 1" for 100ms

// Configuration
pid.set_integral_limit(500);
pid.set_output_limit(127);
pid.set_derivative_filter(0.7);
pid.set_slew_rate(200);
pid.set_deadband(0.5);

// Send telemetry
pid.send_telemetry(0);
```

### Tunable Parameters

```cpp
// Create remotely-tunable parameter
cwb::TunableParam& speed = cwb::param("speed", 100, 0, 127);

// Use it
motor.move(speed.get());
motor.move(speed);  // Implicit conversion

// Check if changed from ControlWorkbench
if (speed.was_updated()) {
    // React to change
}
```

### Telemetry

```cpp
cwb::send_odometry(x, y, theta);
cwb::send_motor(motor);
cwb::send_battery();
cwb::send_debug_value("name", value);

cwb::log_info("message");
cwb::log_warning("warning");
cwb::log_error("error");
```

### Callbacks

```cpp
// Emergency stop - ALWAYS IMPLEMENT!
cwb::on_emergency_stop([]() {
    // STOP EVERYTHING!
});

// Odometry reset
cwb::on_odometry_reset([](double x, double y, double theta) {
    chassis.set_pose(x, y, theta);
});

// Parameter changed
cwb::on_parameter_changed([](const std::string& name, double value) {
    cwb::log_info(name + " = " + std::to_string(value));
});
```

---

## ?? Competition Template

See `examples/competition_template.cpp` for a complete, competition-ready robot with:

- ? 6-motor drivetrain with Pure Pursuit
- ? Intake with color sorting
- ? Lift with PID hold and presets
- ? Pneumatic subsystems
- ? Brain screen auton selector
- ? Driver curves and deadzones
- ? Match timer with endgame warnings
- ? Multi-page debug display
- ? Full telemetry streaming

---

## ?? Modules

| File | Description |
|------|-------------|
| `cwb.hpp` | Main include - brings in everything |
| `telemetry.hpp` | Communication with ControlWorkbench |
| `pid.hpp` | PID controllers with motion profiling |
| `odometry.hpp` | Position tracking |
| `chassis.hpp` | Drivetrain control with Pure Pursuit |
| `subsystems.hpp` | Intake, lift, pneumatic, flywheel |
| `utils.hpp` | Auton selector, driver curves, timer |

---

## ? Troubleshooting

### No telemetry
- ? Call `cwb::update()` every loop
- ? Check COM port in ControlWorkbench
- ? Verify baud rate is 115200

### Odometry drifting
- ? Calibrate IMU before use
- ? Use tracking wheels for better accuracy
- ? Measure wheel diameter and track width carefully

### Robot not stopping on E-stop
- ? Always implement `cwb::on_emergency_stop()`
- ? Stop ALL motors and actuators

### Auton not running
- ? Call `selector.init()` in `initialize()`
- ? Call `selector.run()` in `autonomous()`

---

## ?? LemLib Compatibility

Use CWB for telemetry alongside LemLib for motion:

```cpp
#include "lemlib/api.hpp"
#include "cwb/cwb.hpp"

void opcontrol() {
    while (true) {
        cwb::update();
        
        // Send LemLib pose to ControlWorkbench
        auto pose = chassis.getPose();
        cwb::send_odometry(pose.x, pose.y, pose.theta * M_PI / 180);
        
        pros::delay(10);
    }
}
```

---

## ?? License

MIT License - Use freely in your robotics projects!

---

<div align="center">

**Built with ?? for the VEX Robotics community**

[Documentation](https://controlworkbench.io/docs) · [Discord](https://discord.gg/controlworkbench) · [GitHub](https://github.com/controlworkbench)

</div>
