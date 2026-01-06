/**
 * ControlWorkbench PID Controller Library
 * Version: 1.0.0
 * 
 * Production-ready PID controllers with:
 * - Remote tuning via ControlWorkbench
 * - Anti-windup
 * - Derivative filtering
 * - Output slew limiting
 * - Settle detection
 * - Motion profiling integration
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include <cmath>
#include <functional>

namespace cwb {

// =============================================================================
// PID CONTROLLER - Production Ready
// =============================================================================

/**
 * Full-featured PID controller with remote tuning support.
 * 
 * Usage:
 *   cwb::PIDController drive_pid("drive", 1.0, 0.01, 0.1);
 *   
 *   while (!drive_pid.is_settled()) {
 *       double output = drive_pid.compute(target - current, dt);
 *       motor.move(output);
 *   }
 */
class PIDController {
public:
    /**
     * Create a PID controller with tunable gains.
     * @param name Prefix for tunable parameters (creates name.kP, name.kI, etc.)
     * @param kp Initial proportional gain
     * @param ki Initial integral gain
     * @param kd Initial derivative gain
     */
    PIDController(const std::string& name, double kp, double ki, double kd);
    
    /**
     * Create a PD controller (no integral, avoids windup).
     */
    static PIDController createPD(const std::string& name, double kp, double kd);
    
    /**
     * Compute PID output.
     * @param error Current error (setpoint - measurement)
     * @param dt Time step in seconds
     * @return Controller output
     */
    double compute(double error, double dt);
    
    /**
     * Compute with explicit setpoint and measurement.
     */
    double compute(double setpoint, double measurement, double dt);
    
    /**
     * Reset controller state (integral, derivative, etc.)
     */
    void reset();
    
    /**
     * Check if controller has settled at target.
     * @param tolerance Error tolerance
     * @param settle_time_ms Time to stay within tolerance
     */
    bool is_settled(double tolerance = 1.0, int settle_time_ms = 100) const;
    
    /**
     * Send telemetry for this controller.
     * Call this periodically to visualize in ControlWorkbench.
     */
    void send_telemetry(uint8_t controller_id = 0);
    
    // Configuration
    void set_integral_limit(double max_integral) { max_integral_ = max_integral; }
    void set_output_limit(double max_output) { max_output_ = max_output; }
    void set_derivative_filter(double alpha) { d_filter_alpha_ = alpha; } // 0-1, lower = more filtering
    void set_slew_rate(double rate) { slew_rate_ = rate; } // Max output change per second
    void set_deadband(double db) { deadband_ = db; }
    
    // Reset on gains update
    void set_reset_on_update(bool reset) { reset_on_update_ = reset; }
    
    // Getters for debugging
    double get_error() const { return last_error_; }
    double get_integral() const { return integral_; }
    double get_derivative() const { return filtered_derivative_; }
    double get_output() const { return last_output_; }
    double get_p_term() const { return p_term_; }
    double get_i_term() const { return i_term_; }
    double get_d_term() const { return d_term_; }

private:
    std::string name_;
    PIDGains gains_;
    ControllerConfig config_;
    
    double integral_ = 0;
    double prev_error_ = 0;
    double filtered_derivative_ = 0;
    double last_output_ = 0;
    double last_error_ = 0;
    
    double p_term_ = 0;
    double i_term_ = 0;
    double d_term_ = 0;
    
    double max_integral_ = 1000;
    double max_output_ = 127;
    double d_filter_alpha_ = 0.7; // Lower = more filtering
    double slew_rate_ = 0; // 0 = disabled
    double deadband_ = 0;
    bool reset_on_update_ = true;
    
    // Settle detection
    mutable int settle_count_ = 0;
    double last_setpoint_ = 0;
    double last_measurement_ = 0;
    
    bool first_iteration_ = true;
};

// =============================================================================
// FEEDFORWARD CONTROLLER
// =============================================================================

/**
 * Feedforward controller for motion profiling.
 * 
 * Output = kS * sign(v) + kV * velocity + kA * acceleration + kG * cos(angle)
 */
class FeedforwardController {
public:
    /**
     * Create feedforward controller with tunable gains.
     * @param name Prefix for parameters
     * @param ks Static friction (volts to overcome)
     * @param kv Velocity coefficient (volts per unit/sec)
     * @param ka Acceleration coefficient (volts per unit/sec²)
     * @param kg Gravity coefficient (for arms, volts at horizontal)
     */
    FeedforwardController(const std::string& name, double ks = 0, double kv = 0, double ka = 0, double kg = 0);
    
    /**
     * Calculate feedforward output.
     */
    double calculate(double velocity, double acceleration = 0, double cos_angle = 0) const;
    
    /**
     * Calculate for rotational system.
     */
    double calculate_rotational(double angular_velocity, double angular_acceleration = 0) const;
    
    // Getters
    double get_ks() const { return gains_.s(); }
    double get_kv() const { return gains_.v(); }
    double get_ka() const { return gains_.a(); }
    double get_kg() const { return gains_.g(); }

private:
    FeedforwardGains gains_;
};

// =============================================================================
// MOTION PROFILE
// =============================================================================

/**
 * Trapezoidal motion profile for smooth acceleration.
 */
class TrapezoidalProfile {
public:
    struct Constraints {
        double max_velocity;
        double max_acceleration;
    };
    
    struct State {
        double position;
        double velocity;
    };
    
    TrapezoidalProfile(const Constraints& constraints);
    
    /**
     * Set target state.
     */
    void set_goal(const State& goal);
    
    /**
     * Calculate desired state at time t.
     */
    State calculate(double t) const;
    
    /**
     * Check if profile is complete.
     */
    bool is_finished(double t) const;
    
    /**
     * Get total profile duration.
     */
    double total_time() const { return total_time_; }
    
    /**
     * Reset profile with new initial state.
     */
    void reset(const State& initial);

private:
    Constraints constraints_;
    State initial_;
    State goal_;
    double accel_time_ = 0;
    double cruise_time_ = 0;
    double decel_time_ = 0;
    double total_time_ = 0;
    double cruise_velocity_ = 0;
    
    void recalculate();
};

// =============================================================================
// PROFILED PID CONTROLLER
// =============================================================================

/**
 * PID controller with integrated motion profiling.
 * Combines smooth acceleration with feedback control.
 */
class ProfiledPIDController {
public:
    ProfiledPIDController(const std::string& name, 
                          double kp, double ki, double kd,
                          double max_velocity, double max_acceleration);
    
    /**
     * Set new target.
     */
    void set_target(double target);
    
    /**
     * Update controller.
     * @param measurement Current position
     * @param dt Time step
     * @return Motor output
     */
    double update(double measurement, double dt);
    
    /**
     * Check if at target.
     */
    bool at_target(double tolerance = 1.0) const;
    
    /**
     * Reset controller.
     */
    void reset(double current_position);
    
    // Access subcomponents
    PIDController& get_pid() { return pid_; }
    TrapezoidalProfile& get_profile() { return profile_; }

private:
    PIDController pid_;
    FeedforwardController ff_;
    TrapezoidalProfile profile_;
    double profile_start_time_ = 0;
    bool profile_running_ = false;
};

// =============================================================================
// EXIT CONDITIONS
// =============================================================================

/**
 * Configurable exit conditions for motion commands.
 */
class ExitCondition {
public:
    ExitCondition(const std::string& name);
    
    /**
     * Check if should exit.
     * @param error Current error
     * @param dt Time step
     * @return true if exit condition met
     */
    bool should_exit(double error, double dt);
    
    /**
     * Reset exit condition.
     */
    void reset();
    
    /**
     * Get reason for exit (for debugging).
     */
    std::string get_exit_reason() const { return exit_reason_; }

private:
    TunableParam* small_error_range_;
    TunableParam* small_error_timeout_;
    TunableParam* large_error_range_;
    TunableParam* large_error_timeout_;
    TunableParam* max_time_;
    
    double small_error_time_ = 0;
    double large_error_time_ = 0;
    double total_time_ = 0;
    std::string exit_reason_;
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

PIDController::PIDController(const std::string& name, double kp, double ki, double kd)
    : name_(name), gains_(name, kp, ki, kd, 0), config_(name, 0, 0, 0, 127) {}

PIDController PIDController::createPD(const std::string& name, double kp, double kd) {
    return PIDController(name, kp, 0, kd);
}

double PIDController::compute(double error, double dt) {
    if (dt <= 0) return last_output_;
    
    // Check if gains were updated
    if (reset_on_update_ && gains_.was_updated()) {
        reset();
    }
    
    // Apply deadband
    if (deadband_ > 0 && std::abs(error) < deadband_) {
        error = 0;
        integral_ = 0;
    }
    
    last_error_ = error;
    
    // Proportional
    p_term_ = gains_.p() * error;
    
    // Integral with anti-windup
    if (gains_.i() > 0) {
        integral_ += error * dt;
        if (max_integral_ > 0) {
            integral_ = std::clamp(integral_, -max_integral_, max_integral_);
        }
        i_term_ = gains_.i() * integral_;
    } else {
        i_term_ = 0;
    }
    
    // Derivative with low-pass filter
    double raw_derivative = first_iteration_ ? 0 : (error - prev_error_) / dt;
    filtered_derivative_ = d_filter_alpha_ * raw_derivative + (1 - d_filter_alpha_) * filtered_derivative_;
    d_term_ = gains_.d() * filtered_derivative_;
    prev_error_ = error;
    first_iteration_ = false;
    
    // Compute output
    double output = p_term_ + i_term_ + d_term_;
    
    // Apply slew rate limiting
    if (slew_rate_ > 0) {
        double max_change = slew_rate_ * dt;
        if (output > last_output_ + max_change) output = last_output_ + max_change;
        if (output < last_output_ - max_change) output = last_output_ - max_change;
    }
    
    // Clamp output
    output = std::clamp(output, -max_output_, max_output_);
    
    last_output_ = output;
    
    // Update settle counter
    if (std::abs(error) < 1.0) { // Use small threshold
        settle_count_++;
    } else {
        settle_count_ = 0;
    }
    
    return output;
}

double PIDController::compute(double setpoint, double measurement, double dt) {
    last_setpoint_ = setpoint;
    last_measurement_ = measurement;
    return compute(setpoint - measurement, dt);
}

void PIDController::reset() {
    integral_ = 0;
    prev_error_ = 0;
    filtered_derivative_ = 0;
    last_output_ = 0;
    settle_count_ = 0;
    first_iteration_ = true;
}

bool PIDController::is_settled(double tolerance, int settle_time_ms) const {
    return std::abs(last_error_) < tolerance && settle_count_ * 10 >= settle_time_ms;
}

void PIDController::send_telemetry(uint8_t controller_id) {
    cwb::send_pid_state(controller_id, last_setpoint_, last_measurement_, 
                        last_error_, integral_, filtered_derivative_, last_output_,
                        gains_.p(), gains_.i(), gains_.d());
}

// FeedforwardController
FeedforwardController::FeedforwardController(const std::string& name, double ks, double kv, double ka, double kg)
    : gains_(name, ks, kv, ka, kg) {}

double FeedforwardController::calculate(double velocity, double acceleration, double cos_angle) const {
    return gains_.calculate(velocity, acceleration, cos_angle);
}

double FeedforwardController::calculate_rotational(double angular_velocity, double angular_acceleration) const {
    return gains_.calculate(angular_velocity, angular_acceleration, 0);
}

// TrapezoidalProfile
TrapezoidalProfile::TrapezoidalProfile(const Constraints& constraints)
    : constraints_(constraints), initial_{0, 0}, goal_{0, 0} {}

void TrapezoidalProfile::set_goal(const State& goal) {
    goal_ = goal;
    recalculate();
}

void TrapezoidalProfile::reset(const State& initial) {
    initial_ = initial;
    recalculate();
}

void TrapezoidalProfile::recalculate() {
    double distance = goal_.position - initial_.position;
    double direction = distance >= 0 ? 1.0 : -1.0;
    distance = std::abs(distance);
    
    double max_v = constraints_.max_velocity;
    double max_a = constraints_.max_acceleration;
    
    // Time to reach max velocity
    double accel_dist = (max_v * max_v) / (2 * max_a);
    
    if (2 * accel_dist >= distance) {
        // Triangle profile
        cruise_velocity_ = std::sqrt(distance * max_a);
        accel_time_ = cruise_velocity_ / max_a;
        cruise_time_ = 0;
        decel_time_ = accel_time_;
    } else {
        // Trapezoidal profile
        cruise_velocity_ = max_v;
        accel_time_ = max_v / max_a;
        double cruise_dist = distance - 2 * accel_dist;
        cruise_time_ = cruise_dist / max_v;
        decel_time_ = accel_time_;
    }
    
    total_time_ = accel_time_ + cruise_time_ + decel_time_;
    cruise_velocity_ *= direction;
}

TrapezoidalProfile::State TrapezoidalProfile::calculate(double t) const {
    if (t <= 0) return initial_;
    if (t >= total_time_) return goal_;
    
    double direction = (goal_.position >= initial_.position) ? 1.0 : -1.0;
    double max_a = constraints_.max_acceleration * direction;
    
    State result;
    
    if (t < accel_time_) {
        // Accelerating
        result.velocity = max_a * t;
        result.position = initial_.position + 0.5 * max_a * t * t;
    } else if (t < accel_time_ + cruise_time_) {
        // Cruising
        double cruise_t = t - accel_time_;
        result.velocity = cruise_velocity_;
        result.position = initial_.position + 
                         0.5 * max_a * accel_time_ * accel_time_ +
                         cruise_velocity_ * cruise_t;
    } else {
        // Decelerating
        double decel_t = t - accel_time_ - cruise_time_;
        result.velocity = cruise_velocity_ - max_a * decel_t;
        result.position = initial_.position +
                         0.5 * max_a * accel_time_ * accel_time_ +
                         cruise_velocity_ * cruise_time_ +
                         cruise_velocity_ * decel_t - 0.5 * max_a * decel_t * decel_t;
    }
    
    return result;
}

bool TrapezoidalProfile::is_finished(double t) const {
    return t >= total_time_;
}

// ProfiledPIDController
ProfiledPIDController::ProfiledPIDController(const std::string& name, 
                                             double kp, double ki, double kd,
                                             double max_velocity, double max_acceleration)
    : pid_(name, kp, ki, kd),
      ff_(name + "_ff"),
      profile_({{max_velocity, max_acceleration}}) {}

void ProfiledPIDController::set_target(double target) {
    profile_.set_goal({target, 0});
    profile_start_time_ = pros::millis() / 1000.0;
    profile_running_ = true;
}

double ProfiledPIDController::update(double measurement, double dt) {
    if (!profile_running_) return 0;
    
    double t = pros::millis() / 1000.0 - profile_start_time_;
    auto desired = profile_.calculate(t);
    
    double fb = pid_.compute(desired.position, measurement, dt);
    double ff = ff_.calculate(desired.velocity);
    
    return fb + ff;
}

bool ProfiledPIDController::at_target(double tolerance) const {
    return pid_.is_settled(tolerance);
}

void ProfiledPIDController::reset(double current_position) {
    profile_.reset({current_position, 0});
    pid_.reset();
    profile_running_ = false;
}

// ExitCondition
ExitCondition::ExitCondition(const std::string& name) {
    small_error_range_ = &param(name + ".small_error", 1.0, 0, 100);
    small_error_timeout_ = &param(name + ".small_timeout", 100, 0, 5000);
    large_error_range_ = &param(name + ".large_error", 3.0, 0, 100);
    large_error_timeout_ = &param(name + ".large_timeout", 500, 0, 10000);
    max_time_ = &param(name + ".max_time", 5000, 0, 60000);
}

bool ExitCondition::should_exit(double error, double dt) {
    double abs_error = std::abs(error);
    total_time_ += dt * 1000;
    
    // Check timeout
    if (total_time_ >= max_time_->get()) {
        exit_reason_ = "timeout";
        return true;
    }
    
    // Check small error
    if (abs_error < small_error_range_->get()) {
        small_error_time_ += dt * 1000;
        if (small_error_time_ >= small_error_timeout_->get()) {
            exit_reason_ = "settled";
            return true;
        }
    } else {
        small_error_time_ = 0;
    }
    
    // Check large error
    if (abs_error < large_error_range_->get()) {
        large_error_time_ += dt * 1000;
        if (large_error_time_ >= large_error_timeout_->get()) {
            exit_reason_ = "large_settled";
            return true;
        }
    } else {
        large_error_time_ = 0;
    }
    
    return false;
}

void ExitCondition::reset() {
    small_error_time_ = 0;
    large_error_time_ = 0;
    total_time_ = 0;
    exit_reason_ = "";
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
