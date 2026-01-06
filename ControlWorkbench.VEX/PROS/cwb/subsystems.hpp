/**
 * ControlWorkbench Subsystems Library
 * Version: 1.0.0
 * 
 * Ready-to-use subsystem templates for:
 * - Intake (with color sorting)
 * - Lift/Arm (with PID hold)
 * - Pneumatics (single/double acting)
 * - Flywheel (velocity control)
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include "pid.hpp"
#include "api.h"
#include <functional>

namespace cwb {

// =============================================================================
// INTAKE SUBSYSTEM
// =============================================================================

/**
 * Intake with optional color sorting.
 * 
 * Usage:
 *   cwb::Intake intake(motor, optical);
 *   intake.set_target_color(cwb::Intake::Color::Blue);  // Only keep blue
 *   
 *   // In opcontrol:
 *   if (R1) intake.run();
 *   else if (R2) intake.reverse();
 *   else intake.stop();
 *   intake.update();  // Required for color sorting
 */
class Intake {
public:
    enum class Color { None, Red, Blue };
    enum class State { Stopped, Running, Reversing, Ejecting };
    
    /**
     * Create intake with motor only.
     */
    explicit Intake(pros::Motor& motor);
    
    /**
     * Create intake with motor and optical sensor for color sorting.
     */
    Intake(pros::Motor& motor, pros::Optical& optical);
    
    // Basic control
    void run() { run(static_cast<int>(intake_speed_.get())); }
    void run(int power);
    void reverse() { run(-static_cast<int>(outtake_speed_.get())); }
    void reverse(int power) { run(-power); }
    void stop();
    
    // Color sorting
    void set_target_color(Color color) { target_color_ = color; }
    void set_eject_color(Color color) { eject_color_ = color; }
    void enable_sorting(bool enabled) { sorting_enabled_ = enabled; }
    
    // Must call every loop for color sorting
    void update();
    
    // State
    State get_state() const { return state_; }
    bool is_jammed() const;
    Color get_detected_color() const;
    
    // Telemetry
    void send_telemetry();
    
    // Configuration (all remotely tunable)
    TunableParam& intake_speed() { return intake_speed_; }
    TunableParam& outtake_speed() { return outtake_speed_; }
    TunableParam& eject_time() { return eject_time_; }
    TunableParam& jam_current() { return jam_current_; }

private:
    pros::Motor& motor_;
    pros::Optical* optical_ = nullptr;
    
    TunableParam& intake_speed_;
    TunableParam& outtake_speed_;
    TunableParam& eject_time_;
    TunableParam& jam_current_;
    TunableParam& red_hue_low_;
    TunableParam& red_hue_high_;
    TunableParam& blue_hue_low_;
    TunableParam& blue_hue_high_;
    
    State state_ = State::Stopped;
    Color target_color_ = Color::None;
    Color eject_color_ = Color::None;
    bool sorting_enabled_ = false;
    uint32_t eject_start_ = 0;
    int requested_power_ = 0;
};

// =============================================================================
// LIFT/ARM SUBSYSTEM
// =============================================================================

/**
 * Lift or arm with PID control and position presets.
 * 
 * Usage:
 *   cwb::Lift lift(motor_group);
 *   lift.add_position("down", 0);
 *   lift.add_position("score", 180);
 *   lift.add_position("high", 360);
 *   
 *   // Go to preset
 *   lift.go_to("score");
 *   
 *   // Or direct position
 *   lift.go_to_position(200);
 *   
 *   // Must call every loop
 *   lift.update();
 */
class Lift {
public:
    explicit Lift(pros::Motor& motor, const std::string& name = "lift");
    Lift(pros::MotorGroup& motors, const std::string& name = "lift");
    
    // Preset positions
    void add_position(const std::string& name, double position);
    void go_to(const std::string& preset);
    void go_to_position(double position);
    
    // Manual control (overrides PID)
    void manual_move(int power);
    void manual_stop();
    
    // State
    double get_position() const;
    bool at_target(double tolerance = 5) const;
    bool is_manual() const { return manual_mode_; }
    std::string get_current_preset() const { return current_preset_; }
    
    // Limits
    void set_limits(double min_pos, double max_pos);
    
    // Must call every loop
    void update();
    
    // Telemetry
    void send_telemetry();
    
    // Access PID for tuning
    PIDController& get_pid() { return pid_; }
    TunableParam& hold_power() { return hold_power_; }

private:
    pros::Motor* single_motor_ = nullptr;
    pros::MotorGroup* motor_group_ = nullptr;
    
    PIDController pid_;
    TunableParam& hold_power_;
    TunableParam& max_speed_;
    
    std::map<std::string, double> presets_;
    std::string current_preset_;
    double target_position_ = 0;
    bool manual_mode_ = true;
    double min_position_ = -1e9;
    double max_position_ = 1e9;
    
    void set_power(int power);
    double get_raw_position() const;
};

// =============================================================================
// PNEUMATIC SUBSYSTEM
// =============================================================================

/**
 * Single or double-acting pneumatic cylinder.
 * 
 * Usage:
 *   cwb::Pneumatic mogo_clamp('A');           // Single-acting
 *   cwb::Pneumatic shifter('B', 'C');         // Double-acting
 *   
 *   mogo_clamp.extend();
 *   mogo_clamp.toggle();
 */
class Pneumatic {
public:
    /**
     * Single-acting pneumatic (one solenoid).
     */
    explicit Pneumatic(char port, bool default_extended = false);
    
    /**
     * Double-acting pneumatic (two solenoids).
     */
    Pneumatic(char extend_port, char retract_port);
    
    void extend();
    void retract();
    void toggle();
    void set(bool extended);
    
    bool is_extended() const { return extended_; }

private:
    pros::adi::DigitalOut* single_solenoid_ = nullptr;
    pros::adi::DigitalOut* extend_solenoid_ = nullptr;
    pros::adi::DigitalOut* retract_solenoid_ = nullptr;
    bool extended_ = false;
    bool is_double_acting_ = false;
};

// =============================================================================
// FLYWHEEL SUBSYSTEM
// =============================================================================

/**
 * Velocity-controlled flywheel with bang-bang or PID control.
 * 
 * Usage:
 *   cwb::Flywheel flywheel(motor);
 *   flywheel.set_target_rpm(3000);
 *   
 *   // In opcontrol:
 *   flywheel.update();
 *   if (flywheel.at_speed()) {
 *       // Ready to shoot
 *   }
 */
class Flywheel {
public:
    enum class ControlMode { BangBang, PID, TBH };  // TBH = Take-Back-Half
    
    explicit Flywheel(pros::Motor& motor, const std::string& name = "flywheel");
    Flywheel(pros::MotorGroup& motors, const std::string& name = "flywheel");
    
    void set_target_rpm(double rpm);
    void stop();
    
    void set_control_mode(ControlMode mode) { control_mode_ = mode; }
    
    double get_rpm() const;
    double get_target_rpm() const { return target_rpm_; }
    bool at_speed(double tolerance = 50) const;
    
    // Must call every loop
    void update();
    
    // Telemetry
    void send_telemetry();
    
    // Access controllers
    PIDController& get_pid() { return pid_; }

private:
    pros::Motor* single_motor_ = nullptr;
    pros::MotorGroup* motor_group_ = nullptr;
    
    PIDController pid_;
    TunableParam& recovery_speed_;
    
    ControlMode control_mode_ = ControlMode::TBH;
    double target_rpm_ = 0;
    double tbh_output_ = 0;
    double prev_error_ = 0;
    
    void set_voltage(double voltage);
    double get_raw_velocity() const;
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

// Intake Implementation
Intake::Intake(pros::Motor& motor)
    : motor_(motor),
      intake_speed_(param("intake.speed", 127, 0, 127)),
      outtake_speed_(param("intake.outtake_speed", 100, 0, 127)),
      eject_time_(param("intake.eject_time", 150, 50, 500)),
      jam_current_(param("intake.jam_current", 2500, 1000, 5000)),
      red_hue_low_(param("intake.red_low", 0, 0, 360)),
      red_hue_high_(param("intake.red_high", 30, 0, 360)),
      blue_hue_low_(param("intake.blue_low", 200, 0, 360)),
      blue_hue_high_(param("intake.blue_high", 240, 0, 360)) {}

Intake::Intake(pros::Motor& motor, pros::Optical& optical)
    : Intake(motor) {
    optical_ = &optical;
    optical_->set_led_pwm(100);
}

void Intake::run(int power) {
    requested_power_ = power;
    if (state_ != State::Ejecting) {
        state_ = power > 0 ? State::Running : (power < 0 ? State::Reversing : State::Stopped);
    }
}

void Intake::stop() {
    requested_power_ = 0;
    if (state_ != State::Ejecting) {
        state_ = State::Stopped;
    }
}

void Intake::update() {
    // Handle ejection timing
    if (state_ == State::Ejecting) {
        if (pros::millis() - eject_start_ > static_cast<uint32_t>(eject_time_.get())) {
            state_ = requested_power_ > 0 ? State::Running : State::Stopped;
        } else {
            motor_.move(-127);  // Eject
            return;
        }
    }
    
    // Color sorting
    if (sorting_enabled_ && optical_ && state_ == State::Running) {
        Color detected = get_detected_color();
        
        bool should_eject = false;
        if (eject_color_ != Color::None && detected == eject_color_) {
            should_eject = true;
        }
        if (target_color_ != Color::None && detected != Color::None && detected != target_color_) {
            should_eject = true;
        }
        
        if (should_eject) {
            state_ = State::Ejecting;
            eject_start_ = pros::millis();
            motor_.move(-127);
            return;
        }
    }
    
    // Normal operation
    motor_.move(requested_power_);
}

Intake::Color Intake::get_detected_color() const {
    if (!optical_ || optical_->get_proximity() < 200) return Color::None;
    
    double hue = optical_->get_hue();
    if (hue >= red_hue_low_.get() && hue <= red_hue_high_.get()) return Color::Red;
    if (hue >= blue_hue_low_.get() && hue <= blue_hue_high_.get()) return Color::Blue;
    return Color::None;
}

bool Intake::is_jammed() const {
    return state_ == State::Running && motor_.get_current_draw() > jam_current_.get();
}

void Intake::send_telemetry() {
    send_debug_value("intake_state", static_cast<double>(state_));
    send_debug_value("intake_current", motor_.get_current_draw());
    if (optical_) {
        send_debug_value("intake_hue", optical_->get_hue());
    }
}

// Lift Implementation
Lift::Lift(pros::Motor& motor, const std::string& name)
    : single_motor_(&motor),
      pid_(name, 0.5, 0.01, 0.05),
      hold_power_(param(name + ".hold_power", 10, 0, 50)),
      max_speed_(param(name + ".max_speed", 127, 0, 127)) {}

Lift::Lift(pros::MotorGroup& motors, const std::string& name)
    : motor_group_(&motors),
      pid_(name, 0.5, 0.01, 0.05),
      hold_power_(param(name + ".hold_power", 10, 0, 50)),
      max_speed_(param(name + ".max_speed", 127, 0, 127)) {}

void Lift::add_position(const std::string& name, double position) {
    presets_[name] = position;
}

void Lift::go_to(const std::string& preset) {
    auto it = presets_.find(preset);
    if (it != presets_.end()) {
        current_preset_ = preset;
        go_to_position(it->second);
    }
}

void Lift::go_to_position(double position) {
    target_position_ = std::clamp(position, min_position_, max_position_);
    manual_mode_ = false;
    pid_.reset();
}

void Lift::manual_move(int power) {
    manual_mode_ = true;
    set_power(power);
}

void Lift::manual_stop() {
    manual_mode_ = true;
    set_power(0);
}

void Lift::set_limits(double min_pos, double max_pos) {
    min_position_ = min_pos;
    max_position_ = max_pos;
}

void Lift::update() {
    if (manual_mode_) return;
    
    double current = get_position();
    double output = pid_.compute(target_position_, current, 0.01);
    
    // Add hold power when near target
    if (at_target(10)) {
        output += hold_power_.get();
    }
    
    output = std::clamp(output, -max_speed_.get(), max_speed_.get());
    set_power(static_cast<int>(output));
}

double Lift::get_position() const {
    return get_raw_position();
}

double Lift::get_raw_position() const {
    if (single_motor_) return single_motor_->get_position();
    if (motor_group_) {
        auto positions = motor_group_->get_position_all();
        double sum = 0;
        for (auto p : positions) sum += p;
        return sum / positions.size();
    }
    return 0;
}

bool Lift::at_target(double tolerance) const {
    return std::abs(get_position() - target_position_) < tolerance;
}

void Lift::set_power(int power) {
    if (single_motor_) single_motor_->move(power);
    if (motor_group_) motor_group_->move(power);
}

void Lift::send_telemetry() {
    send_debug_value("lift_position", get_position());
    send_debug_value("lift_target", target_position_);
    pid_.send_telemetry(2);
}

// Pneumatic Implementation
Pneumatic::Pneumatic(char port, bool default_extended)
    : extended_(default_extended), is_double_acting_(false) {
    single_solenoid_ = new pros::adi::DigitalOut(port, default_extended);
}

Pneumatic::Pneumatic(char extend_port, char retract_port)
    : extended_(false), is_double_acting_(true) {
    extend_solenoid_ = new pros::adi::DigitalOut(extend_port, false);
    retract_solenoid_ = new pros::adi::DigitalOut(retract_port, true);
}

void Pneumatic::extend() {
    extended_ = true;
    if (is_double_acting_) {
        extend_solenoid_->set_value(true);
        retract_solenoid_->set_value(false);
    } else {
        single_solenoid_->set_value(true);
    }
}

void Pneumatic::retract() {
    extended_ = false;
    if (is_double_acting_) {
        extend_solenoid_->set_value(false);
        retract_solenoid_->set_value(true);
    } else {
        single_solenoid_->set_value(false);
    }
}

void Pneumatic::toggle() {
    if (extended_) retract();
    else extend();
}

void Pneumatic::set(bool ext) {
    if (ext) extend();
    else retract();
}

// Flywheel Implementation
Flywheel::Flywheel(pros::Motor& motor, const std::string& name)
    : single_motor_(&motor),
      pid_(name, 0.01, 0.0, 0.0),
      recovery_speed_(param(name + ".recovery", 12000, 0, 12000)) {}

Flywheel::Flywheel(pros::MotorGroup& motors, const std::string& name)
    : motor_group_(&motors),
      pid_(name, 0.01, 0.0, 0.0),
      recovery_speed_(param(name + ".recovery", 12000, 0, 12000)) {}

void Flywheel::set_target_rpm(double rpm) {
    target_rpm_ = rpm;
    if (rpm == 0) {
        tbh_output_ = 0;
    }
}

void Flywheel::stop() {
    set_target_rpm(0);
    set_voltage(0);
}

double Flywheel::get_rpm() const {
    return get_raw_velocity();
}

double Flywheel::get_raw_velocity() const {
    if (single_motor_) return single_motor_->get_actual_velocity();
    if (motor_group_) {
        auto velocities = motor_group_->get_actual_velocity_all();
        double sum = 0;
        for (auto v : velocities) sum += v;
        return sum / velocities.size();
    }
    return 0;
}

bool Flywheel::at_speed(double tolerance) const {
    return std::abs(get_rpm() - target_rpm_) < tolerance;
}

void Flywheel::update() {
    if (target_rpm_ == 0) {
        set_voltage(0);
        return;
    }
    
    double current_rpm = get_rpm();
    double error = target_rpm_ - current_rpm;
    
    double output = 0;
    
    switch (control_mode_) {
        case ControlMode::BangBang:
            output = (error > 0) ? 12000 : recovery_speed_.get();
            break;
            
        case ControlMode::PID:
            output = 12000 * (target_rpm_ / 600.0);  // Feedforward
            output += pid_.compute(error, 0.01) * 100;
            break;
            
        case ControlMode::TBH: {
            // Take-Back-Half algorithm
            output = tbh_output_ + error * 0.01;
            output = std::clamp(output, 0.0, 12000.0);
            
            // Take back half on zero crossing
            if ((error > 0) != (prev_error_ > 0)) {
                tbh_output_ = 0.5 * (tbh_output_ + output);
            }
            
            prev_error_ = error;
            tbh_output_ = output;
            break;
        }
    }
    
    set_voltage(std::clamp(output, 0.0, 12000.0));
}

void Flywheel::set_voltage(double voltage) {
    int mv = static_cast<int>(voltage);
    if (single_motor_) single_motor_->move_voltage(mv);
    if (motor_group_) motor_group_->move_voltage(mv);
}

void Flywheel::send_telemetry() {
    send_debug_value("flywheel_rpm", get_rpm());
    send_debug_value("flywheel_target", target_rpm_);
    send_debug_value("flywheel_at_speed", at_speed() ? 1.0 : 0.0);
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
