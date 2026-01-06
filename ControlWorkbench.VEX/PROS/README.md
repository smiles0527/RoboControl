# ControlWorkbench PROS Library

A complete C++ library for real-time telemetry and remote tuning between VEX V5 robots running PROS and the ControlWorkbench desktop application.

## Features

- **Real-time Telemetry**: Stream odometry, motor data, IMU, battery status to ControlWorkbench
- **Remote Parameter Tuning**: Adjust PID gains, speeds, and other values without re-uploading code
- **Odometry Visualization**: See your robot's position on the field in real-time
- **Logging**: Send debug messages to ControlWorkbench console
- **Emergency Stop**: Remote kill switch for safety
- **Path Integration**: Stream paths to/from the path planner

## Quick Installation

### Method 1: Copy Files (Recommended)

1. Copy the entire `cwb` folder to your PROS project's `include` directory:
   ```
   your_project/
   ??? include/
   ?   ??? cwb/              <-- Copy this folder here
   ?   ?   ??? telemetry.hpp
   ?   ??? main.h
   ??? src/
       ??? main.cpp
   ```

2. In **exactly one** `.cpp` file (usually `main.cpp`), add before including:
   ```cpp
   #define CWB_IMPLEMENTATION
   #include "cwb/telemetry.hpp"
   ```

3. In all other files that need the library, just include normally:
   ```cpp
   #include "cwb/telemetry.hpp"
   ```

### Method 2: Download from ControlWorkbench

1. In ControlWorkbench, go to **Tools ? Export PROS Library**
2. Extract to your project's `include` folder

## Basic Usage

```cpp
#define CWB_IMPLEMENTATION
#include "cwb/telemetry.hpp"
#include "main.h"

// Create tunable parameters - adjust these from ControlWorkbench!
cwb::TunableParam& max_speed = cwb::param("max_speed", 100, 0, 127);
cwb::PIDGains drive_pid = cwb::make_pid_gains("drive", 1.0, 0.01, 0.1);

void initialize() {
    // Initialize with UART port 1 (or 0 for USB)
    cwb::init(1);
    
    // Set up emergency stop callback
    cwb::on_emergency_stop([]() {
        // STOP ALL MOTORS!
        left_motor.move(0);
        right_motor.move(0);
    });
    
    cwb::log_info("Robot initialized!");
}

void opcontrol() {
    while (true) {
        // IMPORTANT: Call update() every loop iteration
        cwb::update();
        
        // Send telemetry
        cwb::send_odometry(robot_x, robot_y, robot_theta);
        cwb::send_motor(left_motor);
        cwb::send_motor(right_motor);
        
        // Use tunable parameters
        int speed = controller.get_analog(ANALOG_LEFT_Y) * (max_speed / 127.0);
        
        // Send debug values for graphing
        cwb::send_debug_value("speed", speed);
        cwb::send_debug_value("error", pid_error);
        
        // Check connection status
        if (cwb::is_connected()) {
            // Connected to ControlWorkbench
        }
        
        pros::delay(10);
    }
}
```

## API Reference

### Initialization

```cpp
// Initialize on UART port (1 or 2)
cwb::init(1);

// Initialize on USB (limited bidirectional support)
cwb::init(0);

// Clean up (optional)
cwb::shutdown();
```

### Telemetry

```cpp
// Odometry (position in inches, angle in radians)
cwb::send_odometry(x, y, theta);
cwb::send_odometry(x, y, theta, vel_x, vel_y, angular_vel);

// Motor telemetry
cwb::send_motor(port, position, velocity, current, voltage, temp, power, torque);
cwb::send_motor(my_motor);  // Pass PROS motor directly

// IMU data
cwb::send_imu(heading, pitch, roll, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

// Battery
cwb::send_battery();  // Reads from PROS battery API
cwb::send_battery(voltage, current, capacity, temperature);

// PID state (for tuning visualization)
cwb::send_pid_state(id, setpoint, measurement, error, integral, derivative, output, kP, kI, kD);

// Path progress
cwb::send_path_progress(progress, current_x, current_y, target_x, target_y);
```

### Logging

```cpp
cwb::log_debug("Debug message");
cwb::log_info("Info message");
cwb::log_warning("Warning message");
cwb::log_error("Error message");

// Custom debug values (for graphing)
cwb::send_debug_value("my_value", 42.0);
```

### Tunable Parameters

```cpp
// Create a tunable parameter with default value
cwb::TunableParam& my_param = cwb::param("name", default_value);

// With min/max bounds
cwb::TunableParam& speed = cwb::param("max_speed", 100, 0, 127);

// Use the parameter
double value = my_param.get();
double value = my_param;  // Implicit conversion

// Check if it was changed this frame
if (my_param.was_updated()) {
    // React to change
}

// Get any parameter by name
double val = cwb::get_param("max_speed");
```

### PID Gains

```cpp
// Create tunable PID gains (creates drive.kP, drive.kI, drive.kD parameters)
cwb::PIDGains drive_pid = cwb::make_pid_gains("drive", 1.0, 0.01, 0.1);

// With feedforward term
cwb::PIDGains drive_pid = cwb::make_pid_gains("drive", 1.0, 0.01, 0.1, 0.0);

// Use the gains
double output = drive_pid.p() * error + 
                drive_pid.i() * integral + 
                drive_pid.d() * derivative +
                drive_pid.f() * target_velocity;

// Check if any gain changed
if (drive_pid.was_updated()) {
    pid_controller.reset();  // Reset integral, etc.
}
```

### Feedforward Gains

```cpp
// Create feedforward gains for motion profiling
cwb::FeedforwardGains ff = cwb::make_ff_gains("drive_ff", 
    0.1,   // kS - static friction
    0.05,  // kV - velocity
    0.01   // kA - acceleration
);

// Calculate feedforward output
double ff_output = ff.calculate(target_velocity, target_acceleration);

// For arms with gravity compensation
cwb::FeedforwardGains arm_ff = cwb::make_ff_gains("arm_ff", 0.1, 0.05, 0.01, 0.3);
double arm_output = arm_ff.calculate(velocity, acceleration, cos(arm_angle));
```

### Callbacks

```cpp
// Odometry reset (from ControlWorkbench "Reset Position" button)
cwb::on_odometry_reset([](double x, double y, double theta) {
    odom.reset(x, y, theta);
});

// Emergency stop (ALWAYS implement this!)
cwb::on_emergency_stop([]() {
    left_motors.move(0);
    right_motors.move(0);
    intake.move(0);
    // Stop ALL actuators!
});

// Path waypoint received
cwb::on_waypoint([](double x, double y, double theta) {
    path.push_back({x, y, theta});
});

// Run path command
cwb::on_run_path([](int path_id) {
    run_autonomous_path(path_id);
});

// Any parameter changed
cwb::on_parameter_changed([](const std::string& name, double value) {
    cwb::log_info("Parameter " + name + " = " + std::to_string(value));
});
```

### Connection Status

```cpp
// Check if connected to ControlWorkbench
if (cwb::is_connected()) {
    pros::lcd::print(0, "CWB: Connected");
} else {
    pros::lcd::print(0, "CWB: Disconnected");
}

// Get time since last communication
uint32_t age_ms = cwb::get_last_comm_age();

// Get statistics
cwb::Stats stats = cwb::get_stats();
// stats.messages_sent, stats.messages_received, etc.
```

## Complete Example

See `example_main.cpp` for a complete working example with:
- Odometry tracking
- PID control with remote tuning
- Motor telemetry
- Emergency stop handling
- Point-to-point autonomous

## Wiring

Connect the V5 Brain to ControlWorkbench via:

1. **USB** (Recommended for testing): Direct USB cable, use `cwb::init(0)`
2. **UART**: Smart port with UART adapter, use `cwb::init(1)` or `cwb::init(2)`

## Troubleshooting

### Not receiving telemetry
- Make sure `cwb::update()` is called every loop iteration
- Check the correct COM port is selected in ControlWorkbench
- Verify baud rate is 115200

### Parameters not updating
- Ensure parameter names match exactly (case-sensitive)
- Call `cwb::update()` before reading parameter values

### Emergency stop not working
- Always implement `on_emergency_stop` callback
- Stop ALL motors and actuators in the callback

## LemLib Integration

```cpp
#include "lemlib/api.hpp"

// In opcontrol():
cwb::send_odometry(
    chassis.getPose().x,
    chassis.getPose().y, 
    chassis.getPose().theta * M_PI / 180  // Convert to radians
);
```

## License

MIT License - Use freely in your robotics projects!
