/**
 * ControlWorkbench Odometry Library
 * Version: 1.0.0
 * 
 * Production-ready odometry with:
 * - Multiple tracking wheel configurations
 * - IMU fusion
 * - Velocity estimation
 * - Auto-calibration support
 * - Real-time visualization
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include "api.h"
#include <cmath>
#include <array>

namespace cwb {

// =============================================================================
// POSE REPRESENTATION
// =============================================================================

struct Pose {
    double x = 0;        // inches
    double y = 0;        // inches  
    double theta = 0;    // radians (-PI to PI)
    
    Pose() = default;
    Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    
    // Distance to another pose
    double distance_to(const Pose& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Angle to another pose
    double angle_to(const Pose& other) const {
        return std::atan2(other.x - x, other.y - y);
    }
    
    // Heading error to reach a point
    double heading_error_to(const Pose& other) const {
        double target_angle = angle_to(other);
        double error = target_angle - theta;
        // Normalize to [-PI, PI]
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        return error;
    }
    
    // Operator overloads
    Pose operator+(const Pose& other) const { return {x + other.x, y + other.y, theta + other.theta}; }
    Pose operator-(const Pose& other) const { return {x - other.x, y - other.y, theta - other.theta}; }
    Pose operator*(double scalar) const { return {x * scalar, y * scalar, theta * scalar}; }
};

// =============================================================================
// ODOMETRY CONFIGURATIONS
// =============================================================================

/**
 * Tracking wheel configuration.
 */
struct TrackingWheelConfig {
    pros::Rotation* sensor = nullptr;
    double diameter = 2.75;      // inches
    double offset = 0;           // offset from tracking center (+ = right/forward)
    bool reversed = false;
    
    double get_distance() const {
        if (!sensor) return 0;
        double deg = sensor->get_position() / 100.0; // centidegrees to degrees
        if (reversed) deg = -deg;
        return (deg / 360.0) * diameter * M_PI;
    }
    
    void reset() {
        if (sensor) sensor->reset_position();
    }
};

/**
 * Motor encoder configuration (for drivetrains without tracking wheels).
 */
struct MotorEncoderConfig {
    pros::MotorGroup* motors = nullptr;
    double wheel_diameter = 3.25;
    double gear_ratio = 1.0;     // driven/driving
    double track_width = 12.0;   // only for differential calculation
    
    double get_average_position() const {
        if (!motors) return 0;
        auto positions = motors->get_position_all();
        double sum = 0;
        for (auto p : positions) sum += p;
        return (sum / positions.size() / 360.0) * wheel_diameter * M_PI * gear_ratio;
    }
};

// =============================================================================
// ODOMETRY CLASS
// =============================================================================

/**
 * High-precision odometry tracker.
 * 
 * Supports multiple configurations:
 * - 2 tracking wheels + IMU (recommended)
 * - 3 tracking wheels (no IMU needed)
 * - Motor encoders + IMU (least accurate)
 */
class Odometry {
public:
    Odometry();
    
    /**
     * Configure with 2 parallel tracking wheels + IMU.
     * This is the most common and recommended setup.
     */
    void configure_2_wheel(
        pros::Rotation* left_wheel,
        pros::Rotation* right_wheel,
        pros::Imu* imu,
        double wheel_diameter,
        double track_width,
        bool left_reversed = false,
        bool right_reversed = false
    );
    
    /**
     * Configure with 2 parallel + 1 horizontal tracking wheel.
     * No IMU needed (heading from tracking wheels).
     */
    void configure_3_wheel(
        pros::Rotation* left_wheel,
        pros::Rotation* right_wheel,
        pros::Rotation* horizontal_wheel,
        double wheel_diameter,
        double track_width,
        double horizontal_offset // positive = in front of tracking center
    );
    
    /**
     * Configure using drive motor encoders (least accurate).
     */
    void configure_motor_encoders(
        pros::MotorGroup* left_motors,
        pros::MotorGroup* right_motors,
        pros::Imu* imu,
        double wheel_diameter,
        double track_width,
        double gear_ratio = 1.0
    );
    
    /**
     * Update odometry. Call this every loop iteration.
     */
    void update();
    
    /**
     * Get current pose.
     */
    Pose get_pose() const { return pose_; }
    double get_x() const { return pose_.x; }
    double get_y() const { return pose_.y; }
    double get_theta() const { return pose_.theta; }
    double get_theta_deg() const { return pose_.theta * 180.0 / M_PI; }
    
    /**
     * Get velocities.
     */
    double get_velocity() const { return linear_velocity_; }
    double get_angular_velocity() const { return angular_velocity_; }
    
    /**
     * Reset pose.
     */
    void reset(double x = 0, double y = 0, double theta = 0);
    void reset(const Pose& pose) { reset(pose.x, pose.y, pose.theta); }
    
    /**
     * Send telemetry to ControlWorkbench.
     */
    void send_telemetry();
    
    /**
     * Calibrate sensors.
     */
    void calibrate();
    bool is_calibrating() const;

private:
    enum class OdomType { None, TwoWheel, ThreeWheel, MotorEncoder };
    OdomType type_ = OdomType::None;
    
    Pose pose_;
    double linear_velocity_ = 0;
    double angular_velocity_ = 0;
    
    // Two wheel config
    TrackingWheelConfig left_wheel_;
    TrackingWheelConfig right_wheel_;
    TrackingWheelConfig horizontal_wheel_;
    pros::Imu* imu_ = nullptr;
    double track_width_ = 12.0;
    double horizontal_offset_ = 0;
    
    // Motor encoder config
    MotorEncoderConfig left_motors_;
    MotorEncoderConfig right_motors_;
    
    // Previous values
    double prev_left_ = 0;
    double prev_right_ = 0;
    double prev_horizontal_ = 0;
    double prev_theta_ = 0;
    uint32_t prev_time_ = 0;
    bool initialized_ = false;
    
    void update_two_wheel();
    void update_three_wheel();
    void update_motor_encoder();
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

Odometry::Odometry() {
    prev_time_ = pros::millis();
}

void Odometry::configure_2_wheel(
    pros::Rotation* left_wheel,
    pros::Rotation* right_wheel,
    pros::Imu* imu,
    double wheel_diameter,
    double track_width,
    bool left_reversed,
    bool right_reversed
) {
    type_ = OdomType::TwoWheel;
    left_wheel_.sensor = left_wheel;
    left_wheel_.diameter = wheel_diameter;
    left_wheel_.offset = -track_width / 2;
    left_wheel_.reversed = left_reversed;
    
    right_wheel_.sensor = right_wheel;
    right_wheel_.diameter = wheel_diameter;
    right_wheel_.offset = track_width / 2;
    right_wheel_.reversed = right_reversed;
    
    imu_ = imu;
    track_width_ = track_width;
    initialized_ = false;
}

void Odometry::configure_3_wheel(
    pros::Rotation* left_wheel,
    pros::Rotation* right_wheel,
    pros::Rotation* horizontal_wheel,
    double wheel_diameter,
    double track_width,
    double horizontal_offset
) {
    type_ = OdomType::ThreeWheel;
    left_wheel_.sensor = left_wheel;
    left_wheel_.diameter = wheel_diameter;
    left_wheel_.offset = -track_width / 2;
    
    right_wheel_.sensor = right_wheel;
    right_wheel_.diameter = wheel_diameter;
    right_wheel_.offset = track_width / 2;
    
    horizontal_wheel_.sensor = horizontal_wheel;
    horizontal_wheel_.diameter = wheel_diameter;
    horizontal_offset_ = horizontal_offset;
    
    track_width_ = track_width;
    initialized_ = false;
}

void Odometry::configure_motor_encoders(
    pros::MotorGroup* left_motors,
    pros::MotorGroup* right_motors,
    pros::Imu* imu,
    double wheel_diameter,
    double track_width,
    double gear_ratio
) {
    type_ = OdomType::MotorEncoder;
    left_motors_.motors = left_motors;
    left_motors_.wheel_diameter = wheel_diameter;
    left_motors_.gear_ratio = gear_ratio;
    left_motors_.track_width = track_width;
    
    right_motors_.motors = right_motors;
    right_motors_.wheel_diameter = wheel_diameter;
    right_motors_.gear_ratio = gear_ratio;
    right_motors_.track_width = track_width;
    
    imu_ = imu;
    track_width_ = track_width;
    initialized_ = false;
}

void Odometry::update() {
    switch (type_) {
        case OdomType::TwoWheel: update_two_wheel(); break;
        case OdomType::ThreeWheel: update_three_wheel(); break;
        case OdomType::MotorEncoder: update_motor_encoder(); break;
        default: break;
    }
}

void Odometry::update_two_wheel() {
    // Get current values
    double left = left_wheel_.get_distance();
    double right = right_wheel_.get_distance();
    
    uint32_t now = pros::millis();
    double dt = (now - prev_time_) / 1000.0;
    prev_time_ = now;
    
    if (!initialized_) {
        prev_left_ = left;
        prev_right_ = right;
        if (imu_) prev_theta_ = imu_->get_heading() * M_PI / 180.0;
        initialized_ = true;
        return;
    }
    
    // Calculate deltas
    double d_left = left - prev_left_;
    double d_right = right - prev_right_;
    
    // Get heading from IMU (more accurate than wheel encoders)
    double d_theta;
    if (imu_) {
        double current_theta = imu_->get_heading() * M_PI / 180.0;
        d_theta = current_theta - prev_theta_;
        
        // Handle IMU wraparound (0-360 degrees)
        while (d_theta > M_PI) d_theta -= 2 * M_PI;
        while (d_theta < -M_PI) d_theta += 2 * M_PI;
        
        prev_theta_ = current_theta;
    } else {
        // Fall back to wheel-based heading (less accurate)
        d_theta = (d_right - d_left) / track_width_;
    }
    
    // Calculate local displacement using arc approximation
    // For small angles: displacement ? chord length
    // For larger angles: use arc formula
    double d_center = (d_left + d_right) / 2.0;
    
    double dx, dy;
    if (std::abs(d_theta) < 1e-6) {
        // Straight line motion (avoid division by zero)
        dx = d_center * std::sin(pose_.theta);
        dy = d_center * std::cos(pose_.theta);
    } else {
        // Arc motion - use exact arc formula
        // radius = d_center / d_theta
        // chord = 2 * radius * sin(d_theta/2)
        double radius = d_center / d_theta;
        double chord = 2.0 * radius * std::sin(d_theta / 2.0);
        
        // Direction is at mid-angle
        double mid_theta = pose_.theta + d_theta / 2.0;
        dx = chord * std::sin(mid_theta);
        dy = chord * std::cos(mid_theta);
    }
    
    // Update pose
    pose_.x += dx;
    pose_.y += dy;
    pose_.theta += d_theta;
    
    // Normalize theta to [-PI, PI]
    while (pose_.theta > M_PI) pose_.theta -= 2 * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2 * M_PI;
    
    // Calculate velocities
    if (dt > 0.001) {
        linear_velocity_ = d_center / dt;
        angular_velocity_ = d_theta / dt;
    }
    
    prev_left_ = left;
    prev_right_ = right;
}

void Odometry::update_three_wheel() {
    double left = left_wheel_.get_distance();
    double right = right_wheel_.get_distance();
    double horizontal = horizontal_wheel_.get_distance();
    
    uint32_t now = pros::millis();
    double dt = (now - prev_time_) / 1000.0;
    prev_time_ = now;
    
    if (!initialized_) {
        prev_left_ = left;
        prev_right_ = right;
        prev_horizontal_ = horizontal;
        initialized_ = true;
        return;
    }
    
    double d_left = left - prev_left_;
    double d_right = right - prev_right_;
    double d_horizontal = horizontal - prev_horizontal_;
    
    // Calculate heading change from vertical wheels
    double d_theta = (d_right - d_left) / track_width_;
    
    // Calculate local displacement
    double d_forward = (d_left + d_right) / 2.0;
    
    // Compensate horizontal wheel for rotation
    // When robot rotates, horizontal wheel travels arc = offset * d_theta
    double d_strafe = d_horizontal - horizontal_offset_ * d_theta;
    
    // Transform to global coordinates
    double dx, dy;
    if (std::abs(d_theta) < 1e-6) {
        // Straight line motion
        double sin_t = std::sin(pose_.theta);
        double cos_t = std::cos(pose_.theta);
        dx = d_forward * sin_t + d_strafe * cos_t;
        dy = d_forward * cos_t - d_strafe * sin_t;
    } else {
        // Arc motion with strafe - use rotation matrix at mid-angle
        double mid_theta = pose_.theta + d_theta / 2.0;
        double sin_m = std::sin(mid_theta);
        double cos_m = std::cos(mid_theta);
        
        // Adjust for arc curvature
        double sin_half = std::sin(d_theta / 2.0);
        double scale = (std::abs(d_theta) > 1e-10) ? (2.0 * sin_half / d_theta) : 1.0;
        
        dx = scale * (d_forward * sin_m + d_strafe * cos_m);
        dy = scale * (d_forward * cos_m - d_strafe * sin_m);
    }
    
    pose_.x += dx;
    pose_.y += dy;
    pose_.theta += d_theta;
    
    while (pose_.theta > M_PI) pose_.theta -= 2 * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2 * M_PI;
    
    if (dt > 0.001) {
        linear_velocity_ = std::sqrt(d_forward * d_forward + d_strafe * d_strafe) / dt;
        angular_velocity_ = d_theta / dt;
    }
    
    prev_left_ = left;
    prev_right_ = right;
    prev_horizontal_ = horizontal;
}

void Odometry::update_motor_encoder() {
    double left = left_motors_.get_average_position();
    double right = right_motors_.get_average_position();
    double theta = imu_ ? (imu_->get_heading() * M_PI / 180.0) : pose_.theta;
    
    uint32_t now = pros::millis();
    double dt = (now - prev_time_) / 1000.0;
    prev_time_ = now;
    
    if (!initialized_) {
        prev_left_ = left;
        prev_right_ = right;
        prev_theta_ = theta;
        initialized_ = true;
        return;
    }
    
    double d_left = left - prev_left_;
    double d_right = right - prev_right_;
    double d_theta = theta - prev_theta_;
    
    if (imu_) {
        while (d_theta > M_PI) d_theta -= 2 * M_PI;
        while (d_theta < -M_PI) d_theta += 2 * M_PI;
    } else {
        d_theta = (d_right - d_left) / track_width_;
    }
    
    double d_forward = (d_left + d_right) / 2.0;
    double avg_theta = pose_.theta + d_theta / 2.0;
    
    pose_.x += d_forward * std::sin(avg_theta);
    pose_.y += d_forward * std::cos(avg_theta);
    pose_.theta += d_theta;
    
    while (pose_.theta > M_PI) pose_.theta -= 2 * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2 * M_PI;
    
    if (dt > 0) {
        linear_velocity_ = d_forward / dt;
        angular_velocity_ = d_theta / dt;
    }
    
    prev_left_ = left;
    prev_right_ = right;
    prev_theta_ = theta;
}

void Odometry::reset(double x, double y, double theta) {
    pose_ = {x, y, theta};
    linear_velocity_ = 0;
    angular_velocity_ = 0;
    initialized_ = false;
    
    left_wheel_.reset();
    right_wheel_.reset();
    horizontal_wheel_.reset();
    
    if (imu_) {
        imu_->set_heading(theta * 180.0 / M_PI);
    }
}

void Odometry::send_telemetry() {
    cwb::send_odometry(pose_.x, pose_.y, pose_.theta, 
                       linear_velocity_ * std::sin(pose_.theta),
                       linear_velocity_ * std::cos(pose_.theta),
                       angular_velocity_);
}

void Odometry::calibrate() {
    if (imu_) {
        imu_->reset();
    }
    left_wheel_.reset();
    right_wheel_.reset();
    horizontal_wheel_.reset();
    initialized_ = false;
}

bool Odometry::is_calibrating() const {
    return imu_ && imu_->is_calibrating();
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
