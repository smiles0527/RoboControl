/**
 * ControlWorkbench Chassis Control Library
 * Version: 1.0.0
 * 
 * Production-ready motion control with:
 * - Drive to point
 * - Turn to heading
 * - Pure pursuit path following
 * - Boomerang controller
 * - Motion profiling
 * - Remote tuning
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include "pid.hpp"
#include "odometry.hpp"
#include "api.h"
#include <vector>
#include <functional>

namespace cwb {

// =============================================================================
// CHASSIS CONFIGURATION
// =============================================================================

struct ChassisConfig {
    // Physical dimensions
    double track_width = 12.0;      // inches between wheel centers
    double wheel_diameter = 3.25;    // inches
    double gear_ratio = 1.0;         // driven/driving
    
    // Motion limits
    double max_velocity = 60.0;      // in/sec
    double max_acceleration = 120.0; // in/sec²
    double max_angular_velocity = 180.0; // deg/sec
    
    // Default PID gains
    double lateral_kp = 10.0;
    double lateral_ki = 0.0;
    double lateral_kd = 30.0;
    
    double angular_kp = 2.0;
    double angular_ki = 0.0;
    double angular_kd = 10.0;
};

// =============================================================================
// MOTION OPTIONS
// =============================================================================

struct MoveOptions {
    double max_speed = 127;          // Max motor power (0-127)
    double min_speed = 0;            // Min motor power
    double slew_rate = 0;            // Motor power change per second (0 = disabled)
    bool forwards = true;            // true = forwards, false = backwards
    double early_exit_range = 0;     // Exit early if within this range (inches)
    int timeout = 5000;              // Timeout in ms (0 = no timeout)
    bool async = false;              // Return immediately if true
    
    MoveOptions& set_max_speed(double s) { max_speed = s; return *this; }
    MoveOptions& set_timeout(int t) { timeout = t; return *this; }
    MoveOptions& set_forwards(bool f) { forwards = f; return *this; }
    MoveOptions& set_async(bool a) { async = a; return *this; }
    MoveOptions& set_slew(double s) { slew_rate = s; return *this; }
};

struct TurnOptions {
    double max_speed = 100;
    double min_speed = 0;
    int timeout = 3000;
    bool async = false;
    
    enum class Direction { Auto, Left, Right };
    Direction direction = Direction::Auto;
    
    TurnOptions& set_max_speed(double s) { max_speed = s; return *this; }
    TurnOptions& set_timeout(int t) { timeout = t; return *this; }
    TurnOptions& set_direction(Direction d) { direction = d; return *this; }
};

// =============================================================================
// PATH WAYPOINT
// =============================================================================

struct Waypoint {
    double x;
    double y;
    double velocity = 0;     // Target velocity at this point (0 = default)
    double heading = NAN;    // Optional target heading (NAN = calculate from path)
    
    Waypoint(double x, double y) : x(x), y(y) {}
    Waypoint(double x, double y, double velocity) : x(x), y(y), velocity(velocity) {}
    Waypoint(double x, double y, double velocity, double heading) 
        : x(x), y(y), velocity(velocity), heading(heading) {}
};

// =============================================================================
// CHASSIS CONTROLLER
// =============================================================================

/**
 * Complete chassis controller with motion commands.
 * 
 * Usage:
 *   cwb::Chassis chassis(left_motors, right_motors, imu);
 *   chassis.calibrate();
 *   
 *   // In autonomous:
 *   chassis.move_to_point(24, 24);
 *   chassis.turn_to_heading(90);
 *   chassis.follow_path(path);
 */
class Chassis {
public:
    /**
     * Create chassis controller.
     */
    Chassis(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu);
    
    /**
     * Configure chassis parameters.
     */
    void configure(const ChassisConfig& config);
    
    /**
     * Set up tracking wheels (if used).
     */
    void set_tracking_wheels(pros::Rotation* left, pros::Rotation* right, 
                             double diameter = 2.75, bool left_rev = false, bool right_rev = false);
    
    /**
     * Set up horizontal tracking wheel.
     */
    void set_horizontal_wheel(pros::Rotation* wheel, double offset, double diameter = 2.75);
    
    /**
     * Calibrate sensors. Call in initialize().
     */
    void calibrate();
    bool is_calibrating() const { return odom_.is_calibrating(); }
    
    // =========================================================================
    // MOTION COMMANDS
    // =========================================================================
    
    /**
     * Move to a point on the field.
     */
    void move_to_point(double x, double y, MoveOptions options = {});
    void move_to_point(double x, double y, int timeout) { move_to_point(x, y, MoveOptions().set_timeout(timeout)); }
    
    /**
     * Turn to face a heading (in degrees).
     */
    void turn_to_heading(double heading, TurnOptions options = {});
    void turn_to_heading(double heading, int timeout) { turn_to_heading(heading, TurnOptions().set_timeout(timeout)); }
    
    /**
     * Turn to face a point.
     */
    void turn_to_point(double x, double y, TurnOptions options = {});
    
    /**
     * Drive forward/backward a distance (relative).
     */
    void move_distance(double distance, MoveOptions options = {});
    
    /**
     * Turn by an angle (relative).
     */
    void turn_angle(double angle, TurnOptions options = {});
    
    /**
     * Follow a path using Pure Pursuit.
     */
    void follow_path(const std::vector<Waypoint>& path, MoveOptions options = {});
    
    /**
     * Move using boomerang controller (curved approach).
     */
    void boomerang(double x, double y, double heading, double lead = 0.6, MoveOptions options = {});
    
    /**
     * Swing turn (one side stopped).
     */
    void swing_to_heading(double heading, bool left_swing, TurnOptions options = {});
    
    /**
     * Arc movement (different speeds on each side).
     */
    void arc_move(double left_distance, double right_distance, MoveOptions options = {});
    
    // =========================================================================
    // DRIVER CONTROL
    // =========================================================================
    
    /**
     * Tank drive control.
     */
    void tank(int left_power, int right_power);
    
    /**
     * Arcade drive control.
     */
    void arcade(int forward, int turn);
    
    /**
     * Curvature drive (like arcade but smoother turning at low speeds).
     */
    void curvature(int throttle, int curvature, bool quick_turn = false);
    
    // =========================================================================
    // STATE
    // =========================================================================
    
    /**
     * Check if a motion command is running.
     */
    bool is_moving() const { return motion_running_; }
    
    /**
     * Wait for motion to complete.
     */
    void wait_until_done();
    
    /**
     * Cancel current motion.
     */
    void cancel();
    
    /**
     * Get current pose.
     */
    Pose get_pose() const { return odom_.get_pose(); }
    double get_x() const { return odom_.get_x(); }
    double get_y() const { return odom_.get_y(); }
    double get_heading() const { return odom_.get_theta_deg(); }
    
    /**
     * Set current pose.
     */
    void set_pose(double x, double y, double heading);
    void set_pose(const Pose& pose) { set_pose(pose.x, pose.y, pose.theta * 180 / M_PI); }
    
    /**
     * Update odometry and telemetry. Call every loop iteration.
     */
    void update();
    
    /**
     * Set brake mode.
     */
    void set_brake_mode(pros::motor_brake_mode_e mode);
    
    // Access internal components
    Odometry& get_odom() { return odom_; }
    PIDController& get_lateral_pid() { return lateral_pid_; }
    PIDController& get_angular_pid() { return angular_pid_; }

private:
    pros::MotorGroup& left_motors_;
    pros::MotorGroup& right_motors_;
    pros::Imu& imu_;
    
    Odometry odom_;
    ChassisConfig config_;
    
    PIDController lateral_pid_;
    PIDController angular_pid_;
    ExitCondition exit_condition_;
    
    pros::Rotation* left_tracker_ = nullptr;
    pros::Rotation* right_tracker_ = nullptr;
    pros::Rotation* horizontal_tracker_ = nullptr;
    
    bool motion_running_ = false;
    bool cancel_requested_ = false;
    
    // Motion task
    pros::Task* motion_task_ = nullptr;
    
    void set_motors(double left, double right);
    void stop_motors();
    double normalize_angle(double angle);
};

// =============================================================================
// PURE PURSUIT
// =============================================================================

/**
 * Pure Pursuit path following algorithm.
 */
class PurePursuit {
public:
    PurePursuit(double lookahead_distance = 12.0);
    
    /**
     * Set the path to follow.
     */
    void set_path(const std::vector<Waypoint>& path);
    
    /**
     * Calculate lookahead point.
     * @return Lookahead point, or nullopt if path complete
     */
    std::optional<Pose> calculate_lookahead(const Pose& current);
    
    /**
     * Calculate curvature to reach lookahead point.
     */
    double calculate_curvature(const Pose& current, const Pose& lookahead);
    
    /**
     * Check if path is complete.
     */
    bool is_finished() const { return finished_; }
    
    /**
     * Get current progress (0-1).
     */
    double get_progress() const;
    
    /**
     * Reset for new path.
     */
    void reset();
    
    // Tunable lookahead
    TunableParam* lookahead_dist;

private:
    std::vector<Waypoint> path_;
    int closest_index_ = 0;
    bool finished_ = false;
    
    double distance_to_line(const Pose& point, const Waypoint& start, const Waypoint& end);
    std::optional<Pose> line_circle_intersection(const Pose& start, const Pose& end, 
                                                  const Pose& center, double radius);
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

Chassis::Chassis(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu)
    : left_motors_(left), right_motors_(right), imu_(imu),
      lateral_pid_("lateral", 10, 0, 30),
      angular_pid_("angular", 2, 0, 10),
      exit_condition_("motion") {
    
    // Default motor encoder odometry
    odom_.configure_motor_encoders(&left_motors_, &right_motors_, &imu_, 
                                   config_.wheel_diameter, config_.track_width, config_.gear_ratio);
}

void Chassis::configure(const ChassisConfig& config) {
    config_ = config;
    odom_.configure_motor_encoders(&left_motors_, &right_motors_, &imu_,
                                   config.wheel_diameter, config.track_width, config.gear_ratio);
}

void Chassis::set_tracking_wheels(pros::Rotation* left, pros::Rotation* right,
                                   double diameter, bool left_rev, bool right_rev) {
    left_tracker_ = left;
    right_tracker_ = right;
    odom_.configure_2_wheel(left, right, &imu_, diameter, config_.track_width, left_rev, right_rev);
}

void Chassis::set_horizontal_wheel(pros::Rotation* wheel, double offset, double diameter) {
    horizontal_tracker_ = wheel;
    if (left_tracker_ && right_tracker_) {
        odom_.configure_3_wheel(left_tracker_, right_tracker_, wheel, diameter, config_.track_width, offset);
    }
}

void Chassis::calibrate() {
    imu_.reset();
    odom_.calibrate();
}

void Chassis::update() {
    odom_.update();
    odom_.send_telemetry();
}

void Chassis::set_pose(double x, double y, double heading) {
    odom_.reset(x, y, heading * M_PI / 180.0);
}

void Chassis::set_motors(double left, double right) {
    left = std::clamp(left, -127.0, 127.0);
    right = std::clamp(right, -127.0, 127.0);
    left_motors_.move(static_cast<int>(left));
    right_motors_.move(static_cast<int>(right));
}

void Chassis::stop_motors() {
    left_motors_.move(0);
    right_motors_.move(0);
}

void Chassis::set_brake_mode(pros::motor_brake_mode_e mode) {
    left_motors_.set_brake_mode(mode);
    right_motors_.set_brake_mode(mode);
}

double Chassis::normalize_angle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void Chassis::tank(int left_power, int right_power) {
    set_motors(left_power, right_power);
}

void Chassis::arcade(int forward, int turn) {
    set_motors(forward + turn, forward - turn);
}

void Chassis::curvature(int throttle, int curvature, bool quick_turn) {
    double left, right;
    if (quick_turn) {
        left = throttle + curvature;
        right = throttle - curvature;
    } else {
        double curve = curvature / 127.0;
        left = throttle * (1 + curve);
        right = throttle * (1 - curve);
    }
    set_motors(left, right);
}

void Chassis::move_to_point(double x, double y, MoveOptions options) {
    motion_running_ = true;
    cancel_requested_ = false;
    lateral_pid_.reset();
    angular_pid_.reset();
    exit_condition_.reset();
    
    uint32_t start_time = pros::millis();
    uint32_t last_time = start_time;
    double prev_output = 0;
    
    while (!cancel_requested_) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        
        if (dt <= 0) dt = 0.01;
        
        update();
        Pose current = get_pose();
        
        // Calculate distance and heading to target
        double dx = x - current.x;
        double dy = y - current.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_heading = std::atan2(dx, dy) * 180.0 / M_PI;
        
        // Reverse if going backwards
        if (!options.forwards) {
            target_heading = normalize_angle(target_heading + 180);
            distance = -distance;
        }
        
        double heading_error = normalize_angle(target_heading - get_heading());
        
        // Check exit conditions
        if (options.timeout > 0 && now - start_time > static_cast<uint32_t>(options.timeout)) {
            log_warning("move_to_point timeout");
            break;
        }
        if (std::abs(distance) < 1.0 && std::abs(heading_error) < 5.0) {
            log_info("move_to_point complete");
            break;
        }
        if (options.early_exit_range > 0 && std::abs(distance) < options.early_exit_range) {
            break;
        }
        
        // PID control
        double linear_output = lateral_pid_.compute(distance, dt);
        double angular_output = angular_pid_.compute(heading_error * M_PI / 180.0, dt);
        
        // Scale down linear when heading error is large
        double heading_scale = std::cos(heading_error * M_PI / 180.0);
        heading_scale = std::max(0.0, heading_scale);
        linear_output *= heading_scale;
        
        // Apply slew rate
        if (options.slew_rate > 0) {
            double max_change = options.slew_rate * dt;
            if (linear_output > prev_output + max_change) linear_output = prev_output + max_change;
            if (linear_output < prev_output - max_change) linear_output = prev_output - max_change;
        }
        prev_output = linear_output;
        
        // Clamp to max speed
        linear_output = std::clamp(linear_output, -options.max_speed, options.max_speed);
        angular_output = std::clamp(angular_output * 50, -options.max_speed * 0.5, options.max_speed * 0.5);
        
        // Set motors
        double left = linear_output + angular_output;
        double right = linear_output - angular_output;
        set_motors(left, right);
        
        // Send telemetry
        lateral_pid_.send_telemetry(0);
        send_path_progress(1.0 - std::abs(distance) / 100.0, current.x, current.y, x, y);
        
        pros::delay(10);
    }
    
    stop_motors();
    motion_running_ = false;
}

void Chassis::turn_to_heading(double heading, TurnOptions options) {
    motion_running_ = true;
    cancel_requested_ = false;
    angular_pid_.reset();
    
    uint32_t start_time = pros::millis();
    uint32_t last_time = start_time;
    
    while (!cancel_requested_) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        if (dt <= 0) dt = 0.01;
        
        update();
        
        double error = normalize_angle(heading - get_heading());
        
        // Force direction if specified
        if (options.direction == TurnOptions::Direction::Left && error > 0) {
            error -= 360;
        } else if (options.direction == TurnOptions::Direction::Right && error < 0) {
            error += 360;
        }
        
        if (options.timeout > 0 && now - start_time > static_cast<uint32_t>(options.timeout)) {
            log_warning("turn_to_heading timeout");
            break;
        }
        if (std::abs(error) < 1.0 && angular_pid_.is_settled(1.0, 100)) {
            log_info("turn_to_heading complete");
            break;
        }
        
        double output = angular_pid_.compute(error * M_PI / 180.0, dt) * 50;
        output = std::clamp(output, -options.max_speed, options.max_speed);
        
        // Min speed
        if (std::abs(output) < options.min_speed && std::abs(error) > 1.0) {
            output = options.min_speed * (output > 0 ? 1 : -1);
        }
        
        set_motors(-output, output);
        angular_pid_.send_telemetry(1);
        
        pros::delay(10);
    }
    
    stop_motors();
    motion_running_ = false;
}

void Chassis::turn_to_point(double x, double y, TurnOptions options) {
    Pose current = get_pose();
    double heading = std::atan2(x - current.x, y - current.y) * 180.0 / M_PI;
    turn_to_heading(heading, options);
}

void Chassis::move_distance(double distance, MoveOptions options) {
    Pose start = get_pose();
    double target_x = start.x + distance * std::sin(start.theta);
    double target_y = start.y + distance * std::cos(start.theta);
    options.forwards = distance >= 0;
    move_to_point(target_x, target_y, options);
}

void Chassis::turn_angle(double angle, TurnOptions options) {
    double target = get_heading() + angle;
    turn_to_heading(target, options);
}

void Chassis::wait_until_done() {
    while (motion_running_) {
        pros::delay(10);
    }
}

void Chassis::cancel() {
    cancel_requested_ = true;
    wait_until_done();
}

void Chassis::follow_path(const std::vector<Waypoint>& path, MoveOptions options) {
    if (path.empty()) return;
    
    motion_running_ = true;
    cancel_requested_ = false;
    
    PurePursuit pursuit(12.0);
    pursuit.set_path(path);
    
    uint32_t start_time = pros::millis();
    uint32_t last_time = start_time;
    
    while (!cancel_requested_ && !pursuit.is_finished()) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        if (dt <= 0) dt = 0.01;
        
        update();
        Pose current = get_pose();
        
        if (options.timeout > 0 && now - start_time > static_cast<uint32_t>(options.timeout)) {
            log_warning("follow_path timeout");
            break;
        }
        
        auto lookahead = pursuit.calculate_lookahead(current);
        if (!lookahead) break;
        
        double curvature = pursuit.calculate_curvature(current, *lookahead);
        
        // Calculate wheel velocities from curvature
        double velocity = options.max_speed;
        double left_vel = velocity * (1 + curvature * config_.track_width / 2);
        double right_vel = velocity * (1 - curvature * config_.track_width / 2);
        
        // Normalize if one side exceeds max
        double max_vel = std::max(std::abs(left_vel), std::abs(right_vel));
        if (max_vel > options.max_speed) {
            left_vel = left_vel / max_vel * options.max_speed;
            right_vel = right_vel / max_vel * options.max_speed;
        }
        
        set_motors(left_vel, right_vel);
        
        send_path_progress(pursuit.get_progress(), current.x, current.y, 
                          lookahead->x, lookahead->y, lookahead->x, lookahead->y);
        
        pros::delay(10);
    }
    
    stop_motors();
    motion_running_ = false;
}

void Chassis::boomerang(double x, double y, double heading, double lead, MoveOptions options) {
    motion_running_ = true;
    cancel_requested_ = false;
    lateral_pid_.reset();
    angular_pid_.reset();
    
    uint32_t start_time = pros::millis();
    uint32_t last_time = start_time;
    
    double heading_rad = heading * M_PI / 180.0;
    
    while (!cancel_requested_) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        if (dt <= 0) dt = 0.01;
        
        update();
        Pose current = get_pose();
        
        double dx = x - current.x;
        double dy = y - current.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (options.timeout > 0 && now - start_time > static_cast<uint32_t>(options.timeout)) break;
        if (distance < 1.0) break;
        
        // Calculate carrot point (lead point in front of target)
        double carrot_x = x - lead * distance * std::sin(heading_rad);
        double carrot_y = y - lead * distance * std::cos(heading_rad);
        
        double target_heading = std::atan2(carrot_x - current.x, carrot_y - current.y) * 180.0 / M_PI;
        double heading_error = normalize_angle(target_heading - get_heading());
        
        double linear_output = lateral_pid_.compute(distance, dt);
        double angular_output = angular_pid_.compute(heading_error * M_PI / 180.0, dt);
        
        linear_output *= std::cos(heading_error * M_PI / 180.0);
        linear_output = std::clamp(linear_output, -options.max_speed, options.max_speed);
        angular_output = std::clamp(angular_output * 50, -options.max_speed * 0.5, options.max_speed * 0.5);
        
        set_motors(linear_output + angular_output, linear_output - angular_output);
        
        pros::delay(10);
    }
    
    stop_motors();
    motion_running_ = false;
}

void Chassis::swing_to_heading(double heading, bool left_swing, TurnOptions options) {
    motion_running_ = true;
    cancel_requested_ = false;
    angular_pid_.reset();
    
    uint32_t start_time = pros::millis();
    uint32_t last_time = start_time;
    
    while (!cancel_requested_) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        last_time = now;
        if (dt <= 0) dt = 0.01;
        
        update();
        double error = normalize_angle(heading - get_heading());
        
        if (options.timeout > 0 && now - start_time > static_cast<uint32_t>(options.timeout)) break;
        if (std::abs(error) < 1.0) break;
        
        double output = angular_pid_.compute(error * M_PI / 180.0, dt) * 50;
        output = std::clamp(output, -options.max_speed, options.max_speed);
        
        if (left_swing) {
            set_motors(0, output);
        } else {
            set_motors(-output, 0);
        }
        
        pros::delay(10);
    }
    
    stop_motors();
    motion_running_ = false;
}

void Chassis::arc_move(double left_distance, double right_distance, MoveOptions options) {
    motion_running_ = true;
    cancel_requested_ = false;
    
    double left_start = left_motors_.get_position();
    double right_start = right_motors_.get_position();
    
    double deg_per_inch = 360.0 / (config_.wheel_diameter * M_PI) / config_.gear_ratio;
    double left_target = left_start + left_distance * deg_per_inch;
    double right_target = right_start + right_distance * deg_per_inch;
    
    uint32_t start_time = pros::millis();
    
    while (!cancel_requested_) {
        update();
        
        double left_pos = left_motors_.get_position();
        double right_pos = right_motors_.get_position();
        
        double left_error = left_target - left_pos;
        double right_error = right_target - right_pos;
        
        if (options.timeout > 0 && pros::millis() - start_time > static_cast<uint32_t>(options.timeout)) break;
        if (std::abs(left_error) < 10 && std::abs(right_error) < 10) break;
        
        double left_output = std::clamp(left_error * 0.5, -options.max_speed, options.max_speed);
        double right_output = std::clamp(right_error * 0.5, -options.max_speed, options.max_speed);
        
        set_motors(left_output, right_output);
        pros::delay(10);
    }
    
    stop_motors();
    motion_running_ = false;
}

// PurePursuit implementation
PurePursuit::PurePursuit(double lookahead_distance) {
    lookahead_dist = &param("pursuit.lookahead", lookahead_distance, 4, 36);
}

void PurePursuit::set_path(const std::vector<Waypoint>& path) {
    path_ = path;
    closest_index_ = 0;
    finished_ = false;
}

void PurePursuit::reset() {
    closest_index_ = 0;
    finished_ = false;
}

double PurePursuit::get_progress() const {
    if (path_.empty()) return 1.0;
    return static_cast<double>(closest_index_) / path_.size();
}

std::optional<Pose> PurePursuit::calculate_lookahead(const Pose& current) {
    if (path_.empty() || finished_) return std::nullopt;
    
    double lookahead = lookahead_dist->get();
    
    // Find closest point on path
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = closest_index_; i < path_.size(); i++) {
        double dx = path_[i].x - current.x;
        double dy = path_[i].y - current.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_index_ = static_cast<int>(i);
        }
    }
    
    // Look for intersection with lookahead circle
    for (size_t i = closest_index_; i < path_.size() - 1; i++) {
        Pose start{path_[i].x, path_[i].y, 0};
        Pose end{path_[i + 1].x, path_[i + 1].y, 0};
        
        auto intersection = line_circle_intersection(start, end, current, lookahead);
        if (intersection) {
            return intersection;
        }
    }
    
    // If no intersection, return last point
    if (closest_index_ >= static_cast<int>(path_.size()) - 1) {
        finished_ = true;
        return Pose{path_.back().x, path_.back().y, 0};
    }
    
    return Pose{path_[closest_index_].x, path_[closest_index_].y, 0};
}

double PurePursuit::calculate_curvature(const Pose& current, const Pose& lookahead) {
    double dx = lookahead.x - current.x;
    double dy = lookahead.y - current.y;
    double L = std::sqrt(dx * dx + dy * dy);
    
    if (L < 0.001) return 0;
    
    // Transform lookahead point to robot's local coordinate frame
    // Robot is at origin facing +Y (theta = 0 means facing +Y in our coordinate system)
    // Local X is to the robot's right, Local Y is forward
    double sin_t = std::sin(current.theta);
    double cos_t = std::cos(current.theta);
    
    // Note: Our coordinate system uses atan2(dx, dy) for heading
    // So forward is +Y, right is +X
    // Local X (lateral) = global displacement rotated by -theta
    double local_x = dx * cos_t - dy * sin_t;  // Positive = target is to the right
    // double local_y = dx * sin_t + dy * cos_t;  // Positive = target is ahead
    
    // Curvature formula: ? = 2 * lateral_offset / L²
    // This comes from the geometry of a circular arc passing through
    // the robot and the lookahead point
    return 2.0 * local_x / (L * L);
}

std::optional<Pose> PurePursuit::line_circle_intersection(const Pose& start, const Pose& end,
                                                           const Pose& center, double radius) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double fx = start.x - center.x;
    double fy = start.y - center.y;
    
    double a = dx * dx + dy * dy;
    double b = 2 * (fx * dx + fy * dy);
    double c = fx * fx + fy * fy - radius * radius;
    
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return std::nullopt;
    
    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);
    
    // Use the further intersection (t2) if it's on the segment
    double t = t2;
    if (t < 0 || t > 1) t = t1;
    if (t < 0 || t > 1) return std::nullopt;
    
    return Pose{start.x + t * dx, start.y + t * dy, 0};
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
