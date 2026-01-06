/**
 * ControlWorkbench PROS Library - Complete Example
 * 
 * This file demonstrates all features of the cwb telemetry library.
 * Copy this to your project as a starting point!
 * 
 * Setup:
 *   1. Copy the 'cwb' folder to your include directory
 *   2. Define CWB_IMPLEMENTATION in exactly one .cpp file
 *   3. Connect via USB or UART to ControlWorkbench
 */

// ============================================================================
// IMPORTANT: Define CWB_IMPLEMENTATION in exactly ONE .cpp file!
// ============================================================================
#define CWB_IMPLEMENTATION
#include "cwb/telemetry.hpp"

#include "main.h"
#include <cmath>

// ============================================================================
// ROBOT CONFIGURATION
// ============================================================================

// Motors (adjust ports for your robot)
pros::Motor left_front(1, pros::E_MOTOR_GEARSET_18, false);
pros::Motor left_back(2, pros::E_MOTOR_GEARSET_18, false);
pros::Motor right_front(3, pros::E_MOTOR_GEARSET_18, true);
pros::Motor right_back(4, pros::E_MOTOR_GEARSET_18, true);

// Sensors
pros::Imu imu(10);
pros::Rotation left_encoder(11);
pros::Rotation right_encoder(12);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ============================================================================
// TUNABLE PARAMETERS
// These can be adjusted in real-time from ControlWorkbench!
// ============================================================================

// PID gains for driving
cwb::PIDGains drive_pid = cwb::make_pid_gains("drive", 1.5, 0.02, 0.15);
cwb::PIDGains turn_pid = cwb::make_pid_gains("turn", 2.5, 0.01, 0.25);

// Feedforward gains for motion profiling
cwb::FeedforwardGains drive_ff = cwb::make_ff_gains("drive_ff", 0.05, 0.02, 0.005);

// General parameters
cwb::TunableParam& max_speed = cwb::param("max_speed", 100, 0, 127);
cwb::TunableParam& accel_limit = cwb::param("accel_limit", 8, 1, 20);
cwb::TunableParam& turn_speed = cwb::param("turn_speed", 80, 0, 127);
cwb::TunableParam& lookahead_dist = cwb::param("lookahead", 12, 4, 24);

// ============================================================================
// ODOMETRY
// ============================================================================

struct Odometry {
    double x = 0;
    double y = 0;
    double theta = 0;  // radians
    double vel_x = 0;
    double vel_y = 0;
    double angular_vel = 0;
    
    double prev_left = 0;
    double prev_right = 0;
    bool initialized = false;
    uint32_t last_update = 0;
    
    static constexpr double TRACK_WIDTH = 12.0;  // inches between wheels
    static constexpr double WHEEL_DIAMETER = 2.75;
    static constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
    static constexpr double TICKS_PER_REV = 36000;  // centidegrees
    
    void update() {
        double left_deg = left_encoder.get_position() / 100.0;
        double right_deg = right_encoder.get_position() / 100.0;
        
        uint32_t now = pros::millis();
        double dt = (now - last_update) / 1000.0;
        last_update = now;
        
        if (!initialized) {
            prev_left = left_deg;
            prev_right = right_deg;
            initialized = true;
            return;
        }
        
        // Calculate wheel displacements in inches
        double d_left = (left_deg - prev_left) / 360.0 * WHEEL_CIRCUMFERENCE;
        double d_right = (right_deg - prev_right) / 360.0 * WHEEL_CIRCUMFERENCE;
        
        // Calculate change in position and heading
        double d_theta = (d_right - d_left) / TRACK_WIDTH;
        double d_forward = (d_left + d_right) / 2.0;
        
        // Update position using arc approximation
        double avg_theta = theta + d_theta / 2.0;
        double dx = d_forward * sin(avg_theta);
        double dy = d_forward * cos(avg_theta);
        
        x += dx;
        y += dy;
        theta += d_theta;
        
        // Normalize theta to [-PI, PI]
        while (theta > M_PI) theta -= 2 * M_PI;
        while (theta < -M_PI) theta += 2 * M_PI;
        
        // Calculate velocities
        if (dt > 0) {
            vel_x = dx / dt;
            vel_y = dy / dt;
            angular_vel = d_theta / dt;
        }
        
        prev_left = left_deg;
        prev_right = right_deg;
    }
    
    void reset(double new_x = 0, double new_y = 0, double new_theta = 0) {
        x = new_x;
        y = new_y;
        theta = new_theta;
        vel_x = vel_y = angular_vel = 0;
        initialized = false;
    }
} odom;

// ============================================================================
// PID CONTROLLER
// ============================================================================

class PIDController {
public:
    void set_gains(cwb::PIDGains* g) { gains = g; }
    
    double compute(double error, double dt) {
        if (!gains || dt <= 0) return 0;
        
        integral += error * dt;
        
        // Anti-windup
        integral = std::clamp(integral, -max_integral, max_integral);
        
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        
        return gains->p() * error + 
               gains->i() * integral + 
               gains->d() * derivative;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    double max_integral = 1000;
    
private:
    cwb::PIDGains* gains = nullptr;
    double integral = 0;
    double prev_error = 0;
};

PIDController drive_controller;
PIDController turn_controller;

// ============================================================================
// MOTOR HELPERS
// ============================================================================

void set_drive(int left, int right) {
    left = std::clamp(left, -127, 127);
    right = std::clamp(right, -127, 127);
    
    left_front.move(left);
    left_back.move(left);
    right_front.move(right);
    right_back.move(right);
}

void stop_drive() {
    set_drive(0, 0);
}

void brake_drive() {
    left_front.brake();
    left_back.brake();
    right_front.brake();
    right_back.brake();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");
    
    // ========================================
    // Initialize ControlWorkbench telemetry
    // ========================================
    // Use port 0 for USB, or 1/2 for UART
    cwb::init(1);
    
    // Set up odometry reset callback
    cwb::on_odometry_reset([](double x, double y, double theta) {
        odom.reset(x, y, theta);
        cwb::log_info("Odometry reset to (" + 
            std::to_string(x) + ", " + 
            std::to_string(y) + ", " + 
            std::to_string(theta * 180 / M_PI) + "°)");
    });
    
    // IMPORTANT: Always implement emergency stop!
    cwb::on_emergency_stop([]() {
        stop_drive();
        // Stop any other motors here too!
        cwb::log_warning("EMERGENCY STOP!");
    });
    
    // Callback when PID gains change
    cwb::on_parameter_changed([](const std::string& name, double value) {
        // Reset integral when gains change to prevent windup issues
        if (name.find("drive") != std::string::npos) {
            drive_controller.reset();
        }
        if (name.find("turn") != std::string::npos) {
            turn_controller.reset();
        }
    });
    
    // Initialize PID controllers
    drive_controller.set_gains(&drive_pid);
    turn_controller.set_gains(&turn_pid);
    
    // Calibrate IMU
    imu.reset();
    int timeout = 3000;
    while (imu.is_calibrating() && timeout > 0) {
        pros::lcd::set_text(2, "Calibrating IMU...");
        pros::delay(20);
        timeout -= 20;
    }
    
    odom.last_update = pros::millis();
    
    pros::lcd::set_text(1, "Ready!");
    cwb::log_info("Robot initialized - ControlWorkbench v" + std::string(cwb::VERSION));
}

void disabled() {
    // Robot is disabled
}

void competition_initialize() {
    // Competition-specific initialization
}

// ============================================================================
// AUTONOMOUS ROUTINES
// ============================================================================

/**
 * Drive to a point on the field using PID control.
 */
void drive_to_point(double target_x, double target_y, int timeout_ms = 5000) {
    drive_controller.reset();
    turn_controller.reset();
    
    uint32_t start = pros::millis();
    uint32_t last_time = start;
    
    cwb::log_info("Driving to (" + std::to_string(target_x) + ", " + 
                  std::to_string(target_y) + ")");
    
    while (true) {
        // Update timing
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        
        // Timeout check
        if (now - start > timeout_ms) {
            cwb::log_warning("Drive timeout!");
            break;
        }
        
        // Update odometry
        odom.update();
        
        // Calculate errors
        double dx = target_x - odom.x;
        double dy = target_y - odom.y;
        double distance = sqrt(dx * dx + dy * dy);
        double target_heading = atan2(dx, dy);
        double heading_error = target_heading - odom.theta;
        
        // Normalize heading error to [-PI, PI]
        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;
        
        // Check if we've arrived
        if (distance < 1.0) {
            cwb::log_info("Arrived at target!");
            break;
        }
        
        // Compute PID outputs
        double drive_output = drive_controller.compute(distance, dt);
        double turn_output = turn_controller.compute(heading_error, dt);
        
        // Add feedforward
        double ff = drive_ff.calculate(max_speed.get() * 0.5, 0);
        drive_output += ff;
        
        // Clamp to max speed
        drive_output = std::clamp(drive_output, -max_speed.get(), max_speed.get());
        
        // Calculate motor outputs
        int left = static_cast<int>(drive_output + turn_output);
        int right = static_cast<int>(drive_output - turn_output);
        
        set_drive(left, right);
        
        // Send telemetry to ControlWorkbench
        cwb::send_odometry(odom.x, odom.y, odom.theta, 
                          odom.vel_x, odom.vel_y, odom.angular_vel);
        
        cwb::send_pid_state(0, // controller ID
            distance, 0, distance,  // setpoint, measurement, error
            drive_controller.max_integral, // integral (simplified)
            0, drive_output,  // derivative, output
            drive_pid.p(), drive_pid.i(), drive_pid.d()
        );
        
        cwb::send_path_progress(
            1.0 - (distance / sqrt(dx*dx + dy*dy + 0.001)),
            odom.x, odom.y,
            target_x, target_y
        );
        
        // Update ControlWorkbench communication
        cwb::update();
        
        pros::delay(10);
    }
    
    // Stop and brake
    brake_drive();
}

/**
 * Turn to face a specific heading (in degrees).
 */
void turn_to_heading(double target_deg, int timeout_ms = 2000) {
    turn_controller.reset();
    
    uint32_t start = pros::millis();
    uint32_t last_time = start;
    double target_rad = target_deg * M_PI / 180.0;
    
    cwb::log_info("Turning to " + std::to_string(target_deg) + "°");
    
    while (true) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        
        if (now - start > timeout_ms) break;
        
        odom.update();
        
        double error = target_rad - odom.theta;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        
        if (fabs(error) < 0.02) break;  // ~1 degree
        
        double output = turn_controller.compute(error, dt);
        output = std::clamp(output, -turn_speed.get(), turn_speed.get());
        
        set_drive(-output, output);
        
        cwb::send_odometry(odom.x, odom.y, odom.theta);
        cwb::update();
        
        pros::delay(10);
    }
    
    brake_drive();
}

void autonomous() {
    cwb::log_info("Autonomous started");
    
    // Example autonomous routine
    // Adjust coordinates for your field!
    
    drive_to_point(24, 24);  // Drive to first point
    turn_to_heading(90);     // Turn to face right
    drive_to_point(48, 24);  // Drive to second point
    turn_to_heading(0);      // Turn to face forward
    
    cwb::log_info("Autonomous complete");
}

// ============================================================================
// DRIVER CONTROL
// ============================================================================

void opcontrol() {
    cwb::log_info("Driver control started");

    uint32_t last_telemetry = 0;
    int prev_left = 0;
    int prev_right = 0;
    
    while (true) {
        // ========================================
        // IMPORTANT: Call update() every iteration!
        // ========================================
        cwb::update();
        
        // Update odometry
        odom.update();
        
        // ========================================
        // Tank drive with acceleration limiting
        // ========================================
        int target_left = master.get_analog(ANALOG_LEFT_Y);
        int target_right = master.get_analog(ANALOG_RIGHT_Y);
        
        // Apply max speed limit (tunable from ControlWorkbench!)
        double speed_scale = max_speed.get() / 127.0;
        target_left = static_cast<int>(target_left * speed_scale);
        target_right = static_cast<int>(target_right * speed_scale);
        
        // Apply acceleration limiting (also tunable!)
        int accel = static_cast<int>(accel_limit.get());
        if (target_left > prev_left + accel) target_left = prev_left + accel;
        if (target_left < prev_left - accel) target_left = prev_left - accel;
        if (target_right > prev_right + accel) target_right = prev_right + accel;
        if (target_right < prev_right - accel) target_right = prev_right - accel;
        
        prev_left = target_left;
        prev_right = target_right;
        
        set_drive(target_left, target_right);
        
        // ========================================
        // Send telemetry at ~50Hz
        // ========================================
        if (pros::millis() - last_telemetry >= 20) {
            last_telemetry = pros::millis();
            
            // Odometry
            cwb::send_odometry(odom.x, odom.y, odom.theta,
                              odom.vel_x, odom.vel_y, odom.angular_vel);
            
            // IMU data
            cwb::send_imu(
                imu.get_heading(), imu.get_pitch(), imu.get_roll(),
                imu.get_gyro_rate().x, imu.get_gyro_rate().y, imu.get_gyro_rate().z,
                imu.get_accel().x, imu.get_accel().y, imu.get_accel().z
            );
            
            // Motor telemetry
            cwb::send_motor(left_front);
            cwb::send_motor(right_front);
            
            // Battery status
            cwb::send_battery();
            
            // Competition status
            cwb::send_competition_status(
                pros::competition::is_autonomous(),
                !pros::competition::is_disabled(),
                pros::competition::is_connected()
            );
            
            // Custom debug values (for graphing)
            cwb::send_debug_value("left_power", target_left);
            cwb::send_debug_value("right_power", target_right);
            cwb::send_debug_value("heading_deg", odom.theta * 180 / M_PI);
        }
        
        // ========================================
        // Display on brain screen
        // ========================================
        pros::lcd::print(0, "X: %.1f  Y: %.1f", odom.x, odom.y);
        pros::lcd::print(1, "Theta: %.1f deg", odom.theta * 180 / M_PI);
        pros::lcd::print(2, "MaxSpd: %.0f  Accel: %.0f", max_speed.get(), accel_limit.get());
        pros::lcd::print(3, "CWB: %s", cwb::is_connected() ? "Connected" : "---");
        
        if (cwb::is_connected()) {
            auto stats = cwb::get_stats();
            pros::lcd::print(4, "TX: %lu  RX: %lu", stats.messages_sent, stats.messages_received);
        }
        
        pros::delay(10);
    }
}
