/**
 * ControlWorkbench PROS Library - Example Usage
 * 
 * This file demonstrates how to use the cwb telemetry library
 * for real-time tuning and monitoring from ControlWorkbench.
 */

// In exactly ONE .cpp file, define CWB_IMPLEMENTATION before including
#define CWB_IMPLEMENTATION
#include "cwb/telemetry.hpp"

// In other files, just include normally:
// #include "cwb/telemetry.hpp"

#include "main.h"

// ============================================================================
// Robot Configuration
// ============================================================================

// Motors
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
// Tunable Parameters
// ============================================================================

// These can be adjusted in real-time from ControlWorkbench!
cwb::PIDGains drive_pid = cwb::make_pid_gains("drive", 1.0, 0.01, 0.1);
cwb::PIDGains turn_pid = cwb::make_pid_gains("turn", 2.0, 0.0, 0.2);

cwb::TunableParam& max_speed = cwb::param("max_speed", 127);
cwb::TunableParam& accel_limit = cwb::param("accel_limit", 10);

// ============================================================================
// Odometry
// ============================================================================

struct Odometry {
    double x = 0;
    double y = 0; 
    double theta = 0;
    
    double prev_left = 0;
    double prev_right = 0;
    bool initialized = false;
    
    static constexpr double TRACK_WIDTH = 12.0;  // inches
    static constexpr double WHEEL_DIAMETER = 2.75;
    static constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
    
    void update() {
        double left_deg = left_encoder.get_position() / 100.0;  // centidegrees to degrees
        double right_deg = right_encoder.get_position() / 100.0;
        
        if (!initialized) {
            prev_left = left_deg;
            prev_right = right_deg;
            initialized = true;
            return;
        }
        
        double d_left = (left_deg - prev_left) / 360.0 * WHEEL_CIRCUMFERENCE;
        double d_right = (right_deg - prev_right) / 360.0 * WHEEL_CIRCUMFERENCE;
        
        double d_theta = (d_right - d_left) / TRACK_WIDTH;
        double d_forward = (d_left + d_right) / 2.0;
        
        double avg_theta = theta + d_theta / 2.0;
        x += d_forward * sin(avg_theta);
        y += d_forward * cos(avg_theta);
        theta += d_theta;
        
        prev_left = left_deg;
        prev_right = right_deg;
    }
    
    void reset(double new_x = 0, double new_y = 0, double new_theta = 0) {
        x = new_x;
        y = new_y;
        theta = new_theta;
        initialized = false;
    }
} odom;

// ============================================================================
// PID Controller
// ============================================================================

class PID {
public:
    double compute(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        
        return gains->p() * error + gains->i() * integral + gains->d() * derivative;
    }
    
    void reset() { integral = 0; prev_error = 0; }
    void set_gains(cwb::PIDGains* g) { gains = g; }
    
private:
    cwb::PIDGains* gains = nullptr;
    double integral = 0;
    double prev_error = 0;
};

PID drive_controller;
PID turn_controller;

// ============================================================================
// Initialization
// ============================================================================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");
    
    // Initialize ControlWorkbench telemetry
    cwb::init(1);  // UART port 1
    
    // Set up callbacks
    cwb::on_odometry_reset([](double x, double y, double theta) {
        odom.reset(x, y, theta);
        cwb::log_info("Odometry reset to (" + 
            std::to_string(x) + ", " + 
            std::to_string(y) + ", " + 
            std::to_string(theta) + ")");
    });
    
    cwb::on_emergency_stop([]() {
        left_front.move(0);
        left_back.move(0);
        right_front.move(0);
        right_back.move(0);
        cwb::log_warning("EMERGENCY STOP activated!");
    });
    
    // Calibrate IMU
    imu.reset();
    while (imu.is_calibrating()) {
        pros::lcd::set_text(2, "Calibrating IMU...");
        pros::delay(20);
    }
    
    // Initialize PID controllers with tunable gains
    drive_controller.set_gains(&drive_pid);
    turn_controller.set_gains(&turn_pid);
    
    pros::lcd::set_text(1, "Ready!");
    cwb::log_info("Robot initialized successfully");
}

void disabled() {}
void competition_initialize() {}

// ============================================================================
// Autonomous
// ============================================================================

void drive_to_point(double target_x, double target_y) {
    drive_controller.reset();
    turn_controller.reset();
    
    while (true) {
        odom.update();
        
        double dx = target_x - odom.x;
        double dy = target_y - odom.y;
        double distance = sqrt(dx * dx + dy * dy);
        double target_heading = atan2(dx, dy);
        double heading_error = target_heading - odom.theta;
        
        // Normalize heading error
        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;
        
        if (distance < 1.0) break;  // Within 1 inch
        
        double drive_output = drive_controller.compute(distance, 0.01);
        double turn_output = turn_controller.compute(heading_error, 0.01);
        
        drive_output = std::clamp(drive_output, -max_speed.get(), max_speed.get());
        
        int left = drive_output + turn_output;
        int right = drive_output - turn_output;
        
        left_front.move(left);
        left_back.move(left);
        right_front.move(right);
        right_back.move(right);
        
        // Send telemetry
        cwb::send_odometry(odom.x, odom.y, odom.theta);
        cwb::send_pid_state(0, distance, 0, distance, 0, 0, drive_output,
                           drive_pid.p(), drive_pid.i(), drive_pid.d());
        
        pros::delay(10);
    }
    
    // Stop motors
    left_front.move(0);
    left_back.move(0);
    right_front.move(0);
    right_back.move(0);
}

void autonomous() {
    cwb::log_info("Autonomous started");
    
    // Example: drive to a point
    drive_to_point(24, 24);  // 24 inches forward and right
    
    cwb::log_info("Autonomous complete");
}

// ============================================================================
// Driver Control
// ============================================================================

void opcontrol() {
    cwb::log_info("Driver control started");
    
    uint32_t last_telemetry = 0;
    
    while (true) {
        // Update ControlWorkbench communication
        cwb::update();
        
        // Update odometry
        odom.update();
        
        // Tank drive
        int left_stick = master.get_analog(ANALOG_LEFT_Y);
        int right_stick = master.get_analog(ANALOG_RIGHT_Y);
        
        // Apply max speed limit (tunable!)
        double speed_scale = max_speed.get() / 127.0;
        left_stick = left_stick * speed_scale;
        right_stick = right_stick * speed_scale;
        
        left_front.move(left_stick);
        left_back.move(left_stick);
        right_front.move(right_stick);
        right_back.move(right_stick);
        
        // Send telemetry at 50Hz
        if (pros::millis() - last_telemetry >= 20) {
            last_telemetry = pros::millis();
            
            // Odometry
            cwb::send_odometry(odom.x, odom.y, odom.theta);
            
            // IMU
            cwb::send_imu(imu.get_heading(), imu.get_pitch(), imu.get_roll(),
                         imu.get_gyro_rate().x, imu.get_gyro_rate().y, imu.get_gyro_rate().z,
                         imu.get_accel().x, imu.get_accel().y, imu.get_accel().z);
            
            // Motor telemetry (just front motors as example)
            cwb::send_motor(1, left_front.get_position(), left_front.get_actual_velocity(),
                           left_front.get_current_draw(), left_front.get_voltage(),
                           left_front.get_temperature(), left_front.get_power(),
                           left_front.get_torque());
            
            // Battery
            cwb::send_battery(pros::battery::get_voltage() / 1000.0,
                             pros::battery::get_current() / 1000.0,
                             pros::battery::get_capacity(),
                             pros::battery::get_temperature());
        }
        
        // Display on brain screen
        pros::lcd::print(1, "X: %.1f  Y: %.1f", odom.x, odom.y);
        pros::lcd::print(2, "Theta: %.1f deg", odom.theta * 180 / M_PI);
        pros::lcd::print(3, "CWB: %s", cwb::is_connected() ? "Connected" : "Disconnected");
        
        pros::delay(10);
    }
}
