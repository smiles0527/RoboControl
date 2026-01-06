# ControlWorkbench PROS Library

<div align="center">

# ?? THE ONLY LIBRARY YOU NEED FOR VRC ??

**Complete VEX V5 robotics library for PROS**

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![PROS](https://img.shields.io/badge/PROS-4.0-orange.svg)](https://pros.cs.purdue.edu/)

*Everything included. No other libraries required. Just add and go.*

</div>

---

## ?? What's Included

| Module | Features |
|--------|----------|
| **Chassis** | Pure Pursuit, Boomerang, move-to-point, turn-to-heading, swing turns |
| **Odometry** | Tracking wheels, motor encoders, IMU fusion, GPS fusion |
| **PID** | Anti-windup, derivative filtering, motion profiling, settle detection |
| **Subsystems** | Intake (color sorting), Lift (presets), Pneumatics, Flywheel |
| **Sensors** | Vision, GPS, Distance, Optical, Line Tracker, IMU (all wrapped) |
| **Path Gen** | Hermite splines, Bezier curves, velocity profiling, path mirroring |
| **Logging** | SD card recording, match replay, performance analysis |
| **Utilities** | Auton selector, driver curves, match timer, debug display |
| **Telemetry** | Real-time streaming, remote tuning, emergency stop |

---

## ?? Installation (30 seconds)

```
your_project/
??? include/
?   ??? cwb/              ? Copy this folder
?   ?   ??? cwb.hpp
?   ?   ??? telemetry.hpp
?   ?   ??? pid.hpp
?   ?   ??? odometry.hpp
?   ?   ??? chassis.hpp
?   ?   ??? subsystems.hpp
?   ?   ??? sensors.hpp
?   ?   ??? path.hpp
?   ?   ??? logger.hpp
?   ?   ??? utils.hpp
?   ??? main.h
??? src/
    ??? main.cpp
```

**In ONE .cpp file** (usually `main.cpp`):
```cpp
#define CWB_IMPLEMENTATION
#include "cwb/cwb.hpp"
```

**In other files**, just:
```cpp
#include "cwb/cwb.hpp"
```

**That's it!** No Makefile changes. No other dependencies.

---

## ?? Complete Example

```cpp
#define CWB_IMPLEMENTATION
#include "cwb/cwb.hpp"

// ???????????????????????????????????????????????????????????????????
// HARDWARE
// ???????????????????????????????????????????????????????????????????
pros::MotorGroup left_drive({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({4, 5, 6}, pros::MotorGearset::blue);
pros::Imu imu(10);
pros::Motor intake_motor(7);
pros::MotorGroup lift_motors({-8, 9});
pros::Optical optical(15);

// ???????????????????????????????????????????????????????????????????
// CWB OBJECTS
// ???????????????????????????????????????????????????????????????????
cwb::Chassis chassis(left_drive, right_drive, imu);
cwb::Intake intake(intake_motor, optical);
cwb::Lift lift(lift_motors);
cwb::Pneumatic mogo_clamp('A');
cwb::AutonSelector selector;
cwb::Controller master(pros::E_CONTROLLER_MASTER);
cwb::Logger logger;

// ???????????????????????????????????????????????????????????????????
// AUTONOMOUS ROUTINES
// ???????????????????????????????????????????????????????????????????
void red_positive() {
    chassis.move_to_point(24, 24);
    mogo_clamp.extend();
    intake.run();
    chassis.follow_path({{24, 36}, {24, 48}, {36, 48}});
    chassis.move_to_point(12, 12);
    mogo_clamp.retract();
}

void skills() {
    cwb::MatchTimer::start(cwb::MatchTimer::Mode::Skills);
    logger.start("skills");
    
    // Generate smooth path
    cwb::PathGenerator gen;
    auto path = gen.generate({{0, 0}, {24, 24}, {48, 24}, {48, 48}});
    chassis.follow_path(path);
    
    // Continue with more actions...
    logger.stop();
}

// ???????????????????????????????????????????????????????????????????
// INITIALIZE
// ???????????????????????????????????????????????????????????????????
void initialize() {
    cwb::init();
    
    // CRITICAL: Emergency stop
    cwb::on_emergency_stop([]() {
        left_drive.move(0);
        right_drive.move(0);
        intake_motor.move(0);
        lift_motors.move(0);
    });
    
    // Configure
    chassis.calibrate();
    lift.add_position("down", 0);
    lift.add_position("score", 180);
    intake.set_target_color(cwb::Intake::Color::Red);
    master.set_curve(cwb::DriverCurve::desmos, 5.0);
    
    // Auton selector
    selector.add("Red +", red_positive);
    selector.add("Skills", skills);
    selector.init();
    
    cwb::print_diagnostics();
}

// ???????????????????????????????????????????????????????????????????
// AUTONOMOUS
// ???????????????????????????????????????????????????????????????????
void autonomous() {
    selector.run();
}

// ???????????????????????????????????????????????????????????????????
// DRIVER CONTROL
// ???????????????????????????????????????????????????????????????????
void opcontrol() {
    while (true) {
        cwb::update();
        chassis.update();
        
        // Drive
        chassis.arcade(master.get_left_y(), master.get_right_x());
        
        // Intake
        if (master.held(DIGITAL_R1)) intake.run();
        else if (master.held(DIGITAL_R2)) intake.reverse();
        else intake.stop();
        intake.update();
        
        // Lift
        if (master.pressed(DIGITAL_Y)) lift.go_to("score");
        if (master.pressed(DIGITAL_A)) lift.go_to("down");
        lift.update();
        
        // Pneumatics
        if (master.pressed(DIGITAL_L1)) mogo_clamp.toggle();
        
        pros::delay(10);
    }
}
```

---

## ?? API Reference

### Chassis Control

```cpp
cwb::Chassis chassis(left_motors, right_motors, imu);

// Setup
chassis.set_tracking_wheels(&left, &right, 2.75);  // Optional
chassis.calibrate();

// Motion commands
chassis.move_to_point(24, 24);                     // Drive to point
chassis.move_to_point(24, 24, cwb::MoveOptions()
    .set_max_speed(80)
    .set_timeout(3000));
chassis.turn_to_heading(90);                       // Turn to heading
chassis.move_distance(24);                         // Relative distance
chassis.follow_path(path);                         // Pure Pursuit
chassis.boomerang(24, 24, 90);                     // Curved approach

// Driver control
chassis.arcade(forward, turn);
chassis.tank(left, right);
```

### Subsystems

```cpp
// Intake with color sorting
cwb::Intake intake(motor, optical);
intake.set_target_color(cwb::Intake::Color::Red);
intake.enable_sorting(true);
intake.run();
intake.update();  // Call every loop

// Lift with presets
cwb::Lift lift(motors);
lift.add_position("down", 0);
lift.add_position("score", 180);
lift.go_to("score");
lift.update();  // Call every loop

// Pneumatics
cwb::Pneumatic clamp('A');
clamp.extend();
clamp.retract();
clamp.toggle();

// Flywheel
cwb::Flywheel flywheel(motor);
flywheel.set_target_rpm(3000);
if (flywheel.at_speed()) { /* shoot */ }
flywheel.update();
```

### Sensors

```cpp
// Vision
cwb::Vision vision(1);
auto obj = vision.get_largest(1);  // Get largest object with signature 1
if (obj && obj->is_centered()) { /* aligned */ }

// GPS
cwb::GPS gps(1, 0, 0);  // Port, X offset, Y offset
auto pos = gps.get_position();
if (gps.has_fix()) { /* reliable position */ }

// Distance
cwb::Distance dist(1);
if (dist.is_within(12)) { /* object within 12 inches */ }

// Optical
cwb::Optical optical(1);
if (optical.is_red()) { /* red object */ }

// Line tracker array
cwb::LineArray lines({'A', 'B', 'C'});
double error = lines.get_position();  // -1 to 1

// IMU
cwb::IMU imu(1);
if (imu.is_tipping()) { /* robot tipping! */ }
```

### Path Generation

```cpp
cwb::PathGenerator gen;
gen.set_constraints({.max_velocity = 60, .max_acceleration = 120});

// Generate smooth path from waypoints
std::vector<cwb::Waypoint> waypoints = {{0, 0}, {24, 24}, {48, 24}};
auto path = gen.generate(waypoints);

// Utilities
path = cwb::PathUtils::mirror(path);   // For opposite alliance
path = cwb::PathUtils::reverse(path);  // For driving backward
path = cwb::PathUtils::rotate(path, 90);

// Bezier curves
auto curve = cwb::BezierPath::cubic(p0, p1, p2, p3);
```

### Data Logging

```cpp
// Simple value logging
cwb::Logger logger;
logger.start("match1");
logger.log("x", x);
logger.log("y", y);
logger.tick();  // Call every loop
logger.stop();

// Match recording for replay
cwb::MatchRecorder recorder;
recorder.start_autonomous();
recorder.record(x, y, theta, left_power, right_power);
recorder.save("skills.csv");

// Performance analysis
auto analysis = cwb::PerformanceAnalyzer::analyze(recorder);
cwb::log_info(analysis.summary());
```

### Auton Selector

```cpp
cwb::AutonSelector selector;
selector.add("Red +", red_positive, "Red alliance, positive corner");
selector.add("Skills", skills, "60-second skills run");
selector.init();  // In initialize()
selector.run();   // In autonomous()
```

### Driver Control

```cpp
cwb::Controller master(pros::E_CONTROLLER_MASTER);
master.set_curve(cwb::DriverCurve::desmos, 5.0);
master.set_deadzone(5);

int forward = master.get_left_y();   // Curved
int turn = master.get_right_x();

if (master.pressed(DIGITAL_A)) { }   // New press
if (master.held(DIGITAL_A)) { }      // While held

master.rumble("-.");  // Feedback
```

### Match Timer

```cpp
cwb::MatchTimer::start(cwb::MatchTimer::Mode::Skills);  // 60 sec

int ms = cwb::MatchTimer::remaining_ms();
std::string time = cwb::MatchTimer::format_remaining();

if (cwb::MatchTimer::is_endgame(30000)) {
    // 30 seconds left!
}
```

---

## ?? Configuration Tips

### PID Tuning
All PID gains are remotely tunable via ControlWorkbench:
```cpp
// Gains are automatically created as: lateral.kP, lateral.kI, lateral.kD
// Adjust in real-time without re-uploading code!
```

### Color Sorting
```cpp
intake.set_target_color(cwb::Intake::Color::Red);   // Keep red, eject blue
intake.set_eject_color(cwb::Intake::Color::Blue);   // Explicitly eject blue
```

### Path Mirroring
```cpp
// Create path for Red alliance
auto red_path = gen.generate({{12, 12}, {24, 24}});

// Mirror for Blue alliance
auto blue_path = cwb::PathUtils::mirror(red_path);
```

---

## ? Troubleshooting

| Issue | Solution |
|-------|----------|
| No telemetry | Call `cwb::update()` every loop |
| Odometry drifting | Calibrate IMU, measure dimensions carefully |
| Robot not stopping on E-stop | Implement `cwb::on_emergency_stop()` |
| Auton not running | Call `selector.init()` and `selector.run()` |
| Color sorting not working | Call `intake.update()` every loop |
| SD card not working | Check card format (FAT32), use `/usd/` path |

---

## ?? Memory Usage

| Module | Flash | RAM |
|--------|-------|-----|
| Core (telemetry, PID) | ~15KB | ~2KB |
| Chassis + Odometry | ~12KB | ~1KB |
| Subsystems | ~8KB | ~0.5KB |
| Sensors | ~10KB | ~0.5KB |
| Path + Logger | ~12KB | ~1KB |
| Utils | ~6KB | ~0.5KB |
| **Total** | **~63KB** | **~5.5KB** |

V5 Brain has 32MB flash and 512KB RAM - plenty of room!

---

## ?? License

MIT License - Use freely in your robotics projects!

---

<div align="center">

**Built for VRC Champions ??**

*One library. Complete robot control. Win Worlds.*

</div>
