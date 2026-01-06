/**
 * ControlWorkbench Utilities Library
 * Version: 1.0.0
 * 
 * Competition utilities:
 * - Autonomous selector (brain screen)
 * - Driver curves and deadzones
 * - Match timer
 * - Debug display
 * - Button helpers
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include "api.h"
#include <vector>
#include <functional>
#include <string>

namespace cwb {

// =============================================================================
// AUTONOMOUS SELECTOR
// =============================================================================

/**
 * Brain screen autonomous selector.
 * 
 * Usage:
 *   cwb::AutonSelector selector;
 *   selector.add("Red Positive", red_positive);
 *   selector.add("Red Negative", red_negative);
 *   selector.add("Blue Positive", blue_positive);
 *   selector.add("Skills", skills);
 *   
 *   // In initialize():
 *   selector.init();  // Shows selector on brain screen
 *   
 *   // In autonomous():
 *   selector.run();
 */
class AutonSelector {
public:
    using AutonFunction = std::function<void()>;
    
    struct Auton {
        std::string name;
        AutonFunction function;
        std::string description;
    };
    
    AutonSelector();
    
    /**
     * Add an autonomous routine.
     */
    void add(const std::string& name, AutonFunction func, const std::string& description = "");
    
    /**
     * Initialize the selector (show on brain screen).
     * Call in initialize().
     */
    void init();
    
    /**
     * Run the selected autonomous.
     * Call in autonomous().
     */
    void run();
    
    /**
     * Get name of currently selected auton.
     */
    std::string get_selected_name() const;
    
    /**
     * Select by name.
     */
    void select(const std::string& name);
    
    /**
     * Select by index.
     */
    void select(int index);
    
    /**
     * Select next auton.
     */
    void next();
    
    /**
     * Select previous auton.
     */
    void prev();
    
    /**
     * Get number of autons.
     */
    int count() const { return static_cast<int>(autons_.size()); }
    
    /**
     * Get current selection index.
     */
    int get_index() const { return selected_; }

private:
    std::vector<Auton> autons_;
    int selected_ = 0;
    
    void update_display();
};

// =============================================================================
// DRIVER CURVES
// =============================================================================

/**
 * Driver input curves for smoother control.
 */
class DriverCurve {
public:
    /**
     * No curve (linear).
     */
    static int linear(int input) { return input; }
    
    /**
     * Exponential curve (default VEX curve).
     * Higher exponent = more fine control near center.
     */
    static int exponential(int input, double exponent = 2.0) {
        double normalized = input / 127.0;
        double curved = std::copysign(std::pow(std::abs(normalized), exponent), normalized);
        return static_cast<int>(curved * 127);
    }
    
    /**
     * Desmos curve (commonly used by top teams).
     * t = curve strength (0-10 typical, higher = sharper curve)
     */
    static int desmos(int input, double t = 5.0) {
        if (input == 0) return 0;
        double x = input;
        double result = (std::exp(-t / 10.0) + std::exp((std::abs(x) - 127) / 10.0) * (1 - std::exp(-t / 10.0))) * x;
        return static_cast<int>(result);
    }
    
    /**
     * Apply deadzone to input.
     */
    static int deadzone(int input, int threshold = 5) {
        if (std::abs(input) < threshold) return 0;
        return input;
    }
    
    /**
     * Apply deadzone and scale remaining range.
     */
    static int deadzone_scaled(int input, int threshold = 10) {
        if (std::abs(input) < threshold) return 0;
        int sign = (input > 0) ? 1 : -1;
        double scaled = (std::abs(input) - threshold) * 127.0 / (127 - threshold);
        return static_cast<int>(scaled * sign);
    }
};

// =============================================================================
// CONTROLLER WRAPPER
// =============================================================================

/**
 * Enhanced controller with curves, button helpers, and rumble.
 * 
 * Usage:
 *   cwb::Controller master(pros::E_CONTROLLER_MASTER);
 *   master.set_curve(cwb::DriverCurve::desmos, 5.0);
 *   
 *   // In opcontrol:
 *   int forward = master.get_left_y();  // Automatically applies curve
 *   
 *   // Button with new press detection
 *   if (master.pressed(DIGITAL_A)) {
 *       // Fires once per press
 *   }
 */
class Controller {
public:
    using CurveFunction = std::function<int(int)>;
    
    explicit Controller(pros::controller_id_e_t id);
    
    // Analog inputs (with curves applied)
    int get_left_x() const;
    int get_left_y() const;
    int get_right_x() const;
    int get_right_y() const;
    
    // Raw analog (no curves)
    int get_left_x_raw() const { return controller_.get_analog(ANALOG_LEFT_X); }
    int get_left_y_raw() const { return controller_.get_analog(ANALOG_LEFT_Y); }
    int get_right_x_raw() const { return controller_.get_analog(ANALOG_RIGHT_X); }
    int get_right_y_raw() const { return controller_.get_analog(ANALOG_RIGHT_Y); }
    
    // Digital inputs
    bool held(pros::controller_digital_e_t button) const { return controller_.get_digital(button); }
    bool pressed(pros::controller_digital_e_t button) { return controller_.get_digital_new_press(button); }
    
    // Curve configuration
    void set_curve(CurveFunction curve) { curve_ = curve; }
    void set_curve(int (*curve)(int, double), double param) {
        double p = param;
        curve_ = [curve, p](int x) { return curve(x, p); };
    }
    void set_deadzone(int threshold) { deadzone_ = threshold; }
    
    // Rumble
    void rumble(const char* pattern) { controller_.rumble(pattern); }
    void rumble_short() { rumble("."); }
    void rumble_long() { rumble("-"); }
    
    // Display
    void print(int line, const char* format, ...);
    void clear_line(int line) { controller_.clear_line(line); }
    void clear() { controller_.clear(); }
    
    // Access underlying controller
    pros::Controller& raw() { return controller_; }

private:
    pros::Controller controller_;
    CurveFunction curve_;
    int deadzone_ = 5;
    
    int apply_curve(int input) const;
};

// =============================================================================
// MATCH TIMER
// =============================================================================

/**
 * Match and skills timer utilities.
 */
class MatchTimer {
public:
    enum class Mode { Driver, Autonomous, Skills };
    
    static constexpr int AUTON_DURATION = 15000;    // 15 seconds
    static constexpr int DRIVER_DURATION = 105000;  // 1:45
    static constexpr int SKILLS_DURATION = 60000;   // 60 seconds
    
    /**
     * Start timer for match mode.
     */
    static void start(Mode mode);
    
    /**
     * Get remaining time in milliseconds.
     */
    static int remaining_ms();
    
    /**
     * Get remaining time in seconds.
     */
    static double remaining_sec() { return remaining_ms() / 1000.0; }
    
    /**
     * Get elapsed time in milliseconds.
     */
    static int elapsed_ms() { return static_cast<int>(pros::millis() - start_time_); }
    
    /**
     * Check if time is running low.
     */
    static bool is_endgame(int threshold_ms = 30000) { return remaining_ms() < threshold_ms; }
    
    /**
     * Check if time has expired.
     */
    static bool is_expired() { return remaining_ms() <= 0; }
    
    /**
     * Get formatted time string "M:SS".
     */
    static std::string format_remaining();

private:
    static uint32_t start_time_;
    static int duration_;
    static Mode mode_;
};

// =============================================================================
// DEBUG DISPLAY
// =============================================================================

/**
 * On-brain debug display with multiple pages.
 * 
 * Usage:
 *   cwb::DebugDisplay debug;
 *   debug.add_page("Odom", []() {
 *       debug.print(0, "X: %.1f", odom.get_x());
 *       debug.print(1, "Y: %.1f", odom.get_y());
 *   });
 *   
 *   // In opcontrol:
 *   debug.update();  // Touch screen to switch pages
 */
class DebugDisplay {
public:
    using PageFunction = std::function<void()>;
    
    DebugDisplay();
    
    /**
     * Add a debug page.
     */
    void add_page(const std::string& name, PageFunction render);
    
    /**
     * Update display. Call every loop.
     * Handles touch input for page switching.
     */
    void update();
    
    /**
     * Print to a line on the current page.
     */
    void print(int line, const char* format, ...);
    
    /**
     * Manually switch to next page.
     */
    void next_page();
    
    /**
     * Manually switch to previous page.
     */
    void prev_page();
    
    /**
     * Get current page name.
     */
    std::string get_current_page() const;

private:
    struct Page {
        std::string name;
        PageFunction render;
    };
    
    std::vector<Page> pages_;
    int current_page_ = 0;
    uint32_t last_touch_ = 0;
};

// =============================================================================
// BUTTON COMBO
// =============================================================================

/**
 * Detect button combinations and sequences.
 * 
 * Usage:
 *   // Combo: A + B pressed together
 *   if (cwb::ButtonCombo::is_combo(master, {DIGITAL_A, DIGITAL_B})) {
 *       // Do something
 *   }
 *   
 *   // Sequence: Up, Up, Down, Down, A
 *   cwb::ButtonSequence konami(master, {DIGITAL_UP, DIGITAL_UP, DIGITAL_DOWN, DIGITAL_DOWN, DIGITAL_A});
 *   if (konami.check()) {
 *       // Secret unlocked!
 *   }
 */
class ButtonCombo {
public:
    /**
     * Check if all buttons in combo are pressed.
     */
    static bool is_combo(pros::Controller& controller, 
                         std::initializer_list<pros::controller_digital_e_t> buttons);
    
    /**
     * Check if combo was just pressed (all buttons newly pressed).
     */
    static bool combo_pressed(pros::Controller& controller,
                              std::initializer_list<pros::controller_digital_e_t> buttons);
};

/**
 * Detect a sequence of button presses.
 */
class ButtonSequence {
public:
    ButtonSequence(pros::Controller& controller, 
                   std::initializer_list<pros::controller_digital_e_t> sequence,
                   int timeout_ms = 2000);
    
    /**
     * Check sequence. Call every loop.
     * Returns true when sequence is completed.
     */
    bool check();
    
    /**
     * Reset sequence progress.
     */
    void reset();

private:
    pros::Controller& controller_;
    std::vector<pros::controller_digital_e_t> sequence_;
    int current_index_ = 0;
    int timeout_ms_;
    uint32_t last_press_ = 0;
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

// Static members
uint32_t MatchTimer::start_time_ = 0;
int MatchTimer::duration_ = 0;
MatchTimer::Mode MatchTimer::mode_ = MatchTimer::Mode::Driver;

// AutonSelector
AutonSelector::AutonSelector() {}

void AutonSelector::add(const std::string& name, AutonFunction func, const std::string& description) {
    autons_.push_back({name, func, description});
}

void AutonSelector::init() {
    pros::lcd::initialize();
    
    // Register touch callbacks
    pros::lcd::register_btn0_cb([this]() { prev(); });
    pros::lcd::register_btn2_cb([this]() { next(); });
    
    update_display();
}

void AutonSelector::run() {
    if (autons_.empty()) {
        log_warning("No autons registered!");
        return;
    }
    
    log_info("Running auton: " + autons_[selected_].name);
    autons_[selected_].function();
}

std::string AutonSelector::get_selected_name() const {
    if (autons_.empty()) return "None";
    return autons_[selected_].name;
}

void AutonSelector::select(const std::string& name) {
    for (size_t i = 0; i < autons_.size(); i++) {
        if (autons_[i].name == name) {
            selected_ = static_cast<int>(i);
            update_display();
            return;
        }
    }
}

void AutonSelector::select(int index) {
    if (index >= 0 && index < static_cast<int>(autons_.size())) {
        selected_ = index;
        update_display();
    }
}

void AutonSelector::next() {
    selected_ = (selected_ + 1) % static_cast<int>(autons_.size());
    update_display();
}

void AutonSelector::prev() {
    selected_ = (selected_ - 1 + static_cast<int>(autons_.size())) % static_cast<int>(autons_.size());
    update_display();
}

void AutonSelector::update_display() {
    pros::lcd::clear();
    pros::lcd::print(0, "=== AUTON SELECTOR ===");
    pros::lcd::print(2, "< [%d/%d] >", selected_ + 1, static_cast<int>(autons_.size()));
    pros::lcd::print(3, "  %s  ", autons_[selected_].name.c_str());
    if (!autons_[selected_].description.empty()) {
        pros::lcd::print(5, "%s", autons_[selected_].description.c_str());
    }
    pros::lcd::print(7, "Touch: < prev     next >");
}

// Controller
Controller::Controller(pros::controller_id_e_t id) 
    : controller_(id), curve_(DriverCurve::linear) {}

int Controller::apply_curve(int input) const {
    if (std::abs(input) < deadzone_) return 0;
    return curve_(input);
}

int Controller::get_left_x() const { return apply_curve(get_left_x_raw()); }
int Controller::get_left_y() const { return apply_curve(get_left_y_raw()); }
int Controller::get_right_x() const { return apply_curve(get_right_x_raw()); }
int Controller::get_right_y() const { return apply_curve(get_right_y_raw()); }

void Controller::print(int line, const char* format, ...) {
    char buffer[32];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    controller_.print(line, 0, "%s", buffer);
}

// MatchTimer
void MatchTimer::start(Mode mode) {
    mode_ = mode;
    start_time_ = pros::millis();
    
    switch (mode) {
        case Mode::Autonomous: duration_ = AUTON_DURATION; break;
        case Mode::Driver: duration_ = DRIVER_DURATION; break;
        case Mode::Skills: duration_ = SKILLS_DURATION; break;
    }
}

int MatchTimer::remaining_ms() {
    int elapsed = static_cast<int>(pros::millis() - start_time_);
    return std::max(0, duration_ - elapsed);
}

std::string MatchTimer::format_remaining() {
    int ms = remaining_ms();
    int seconds = ms / 1000;
    int minutes = seconds / 60;
    seconds %= 60;
    
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d:%02d", minutes, seconds);
    return std::string(buffer);
}

// DebugDisplay
DebugDisplay::DebugDisplay() {
    pros::lcd::initialize();
}

void DebugDisplay::add_page(const std::string& name, PageFunction render) {
    pages_.push_back({name, render});
}

void DebugDisplay::update() {
    if (pages_.empty()) return;
    
    // Check for touch to switch pages
    auto touch = pros::lcd::read_buttons();
    if (touch && pros::millis() - last_touch_ > 300) {
        if (touch & LCD_BTN_LEFT) prev_page();
        if (touch & LCD_BTN_RIGHT) next_page();
        last_touch_ = pros::millis();
    }
    
    // Render current page
    pros::lcd::clear();
    pros::lcd::print(0, "< %s [%d/%d] >", 
                     pages_[current_page_].name.c_str(),
                     current_page_ + 1,
                     static_cast<int>(pages_.size()));
    
    pages_[current_page_].render();
}

void DebugDisplay::print(int line, const char* format, ...) {
    char buffer[32];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    pros::lcd::print(line + 1, "%s", buffer);  // +1 to skip header
}

void DebugDisplay::next_page() {
    current_page_ = (current_page_ + 1) % static_cast<int>(pages_.size());
}

void DebugDisplay::prev_page() {
    current_page_ = (current_page_ - 1 + static_cast<int>(pages_.size())) % static_cast<int>(pages_.size());
}

std::string DebugDisplay::get_current_page() const {
    if (pages_.empty()) return "";
    return pages_[current_page_].name;
}

// ButtonCombo
bool ButtonCombo::is_combo(pros::Controller& controller,
                           std::initializer_list<pros::controller_digital_e_t> buttons) {
    for (auto btn : buttons) {
        if (!controller.get_digital(btn)) return false;
    }
    return true;
}

bool ButtonCombo::combo_pressed(pros::Controller& controller,
                                std::initializer_list<pros::controller_digital_e_t> buttons) {
    bool any_new = false;
    for (auto btn : buttons) {
        if (!controller.get_digital(btn)) return false;
        if (controller.get_digital_new_press(btn)) any_new = true;
    }
    return any_new;
}

// ButtonSequence
ButtonSequence::ButtonSequence(pros::Controller& controller,
                               std::initializer_list<pros::controller_digital_e_t> sequence,
                               int timeout_ms)
    : controller_(controller), sequence_(sequence), timeout_ms_(timeout_ms) {}

bool ButtonSequence::check() {
    // Check timeout
    if (current_index_ > 0 && pros::millis() - last_press_ > static_cast<uint32_t>(timeout_ms_)) {
        reset();
    }
    
    // Check for next button in sequence
    if (controller_.get_digital_new_press(sequence_[current_index_])) {
        current_index_++;
        last_press_ = pros::millis();
        
        if (current_index_ >= static_cast<int>(sequence_.size())) {
            reset();
            return true;
        }
    }
    
    return false;
}

void ButtonSequence::reset() {
    current_index_ = 0;
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
