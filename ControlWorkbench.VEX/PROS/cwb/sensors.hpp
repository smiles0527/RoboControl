/**
 * ControlWorkbench Sensors Library
 * Version: 1.0.0
 * 
 * Complete sensor wrappers for VEX V5:
 * - Vision sensor with object tracking
 * - GPS sensor with odometry fusion
 * - Distance sensor with filtering
 * - Optical sensor helpers
 * - Line tracker array
 * - Limit switches and bumpers
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include "api.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace cwb {

// =============================================================================
// VISION SENSOR
// =============================================================================

/**
 * Enhanced vision sensor with object tracking and filtering.
 * 
 * Usage:
 *   cwb::Vision vision(1);
 *   vision.add_signature(1, RED_RING);   // Configure in V5 Vision Utility
 *   vision.add_signature(2, BLUE_RING);
 *   
 *   // In loop:
 *   auto objects = vision.get_objects(1);  // Get red rings
 *   if (!objects.empty()) {
 *       auto closest = vision.get_largest(1);
 *       // Track the object...
 *   }
 */
class Vision {
public:
    struct Object {
        int16_t x;           // X center (-158 to 158, 0 = center)
        int16_t y;           // Y center (-105 to 105, 0 = center)
        int16_t width;       // Object width in pixels
        int16_t height;      // Object height in pixels
        uint16_t area;       // width * height
        uint8_t signature;   // Which signature detected this
        
        // Calculated properties
        double distance_estimate() const {
            // Rough distance based on object size (calibrate for your object)
            return 1000.0 / std::sqrt(area);
        }
        
        double angle() const {
            // Angle from center in degrees (-30 to 30 typical)
            return x * 0.19;  // Approximate conversion
        }
        
        bool is_centered(int tolerance = 20) const {
            return std::abs(x) < tolerance;
        }
    };
    
    explicit Vision(int port);
    
    /**
     * Get all objects matching a signature.
     * @param signature Signature ID (1-7)
     * @param max_count Maximum objects to return
     */
    std::vector<Object> get_objects(uint8_t signature, int max_count = 10);
    
    /**
     * Get the largest object of a signature.
     */
    std::optional<Object> get_largest(uint8_t signature);
    
    /**
     * Get the closest object (by estimated distance).
     */
    std::optional<Object> get_closest(uint8_t signature);
    
    /**
     * Get the most centered object.
     */
    std::optional<Object> get_centered(uint8_t signature);
    
    /**
     * Check if any object is visible.
     */
    bool has_object(uint8_t signature);
    
    /**
     * Get object count.
     */
    int count(uint8_t signature);
    
    /**
     * Set LED brightness and color.
     */
    void set_led(uint8_t brightness, uint32_t color = 0xFFFFFF);
    void led_on() { set_led(100); }
    void led_off() { set_led(0); }
    
    /**
     * Configure white balance.
     */
    void set_white_balance(uint32_t rgb);
    void set_auto_white_balance(bool enabled);
    
    /**
     * Set exposure.
     */
    void set_exposure(uint8_t percent);
    
    /**
     * Get raw sensor.
     */
    pros::Vision& raw() { return sensor_; }

private:
    pros::Vision sensor_;
};

// =============================================================================
// GPS SENSOR
// =============================================================================

/**
 * GPS sensor wrapper with coordinate conversion and odometry fusion.
 * 
 * Usage:
 *   cwb::GPS gps(1, 0, 0);  // Port, X offset, Y offset from robot center
 *   
 *   // Get position
 *   auto pos = gps.get_position();
 *   double x = pos.x;
 *   double y = pos.y;
 *   double heading = pos.theta;
 */
class GPS {
public:
    struct Position {
        double x;       // X position in inches
        double y;       // Y position in inches
        double theta;   // Heading in radians
        double error;   // Position error estimate
        bool valid;     // True if position is reliable
    };
    
    /**
     * Create GPS sensor.
     * @param port Smart port (1-21)
     * @param x_offset X offset from robot center (inches)
     * @param y_offset Y offset from robot center (inches)
     */
    GPS(int port, double x_offset = 0, double y_offset = 0);
    
    /**
     * Get current position.
     */
    Position get_position() const;
    
    /**
     * Get position in field coordinates (0-144 inches).
     */
    Position get_field_position() const;
    
    /**
     * Get heading in degrees (0-360).
     */
    double get_heading_deg() const;
    
    /**
     * Get heading in radians (-PI to PI).
     */
    double get_heading_rad() const;
    
    /**
     * Check if GPS has a valid fix.
     */
    bool has_fix() const;
    
    /**
     * Get error estimate (lower = more accurate).
     */
    double get_error() const;
    
    /**
     * Get rotation rate (deg/sec).
     */
    double get_rotation_rate() const;
    
    /**
     * Set initial position (for GPS calibration).
     */
    void set_position(double x, double y, double heading_deg);
    
    /**
     * Set sensor offset from robot center.
     */
    void set_offset(double x, double y);
    
    /**
     * Set data rate (5-200ms, default 50ms).
     */
    void set_data_rate(uint32_t ms);
    
    /**
     * Get raw sensor.
     */
    pros::Gps& raw() { return sensor_; }

private:
    pros::Gps sensor_;
    double x_offset_;
    double y_offset_;
};

// =============================================================================
// DISTANCE SENSOR
// =============================================================================

/**
 * Distance sensor with filtering and object detection.
 * 
 * Usage:
 *   cwb::Distance dist(1);
 *   
 *   if (dist.get_distance() < 12) {
 *       // Object within 12 inches
 *   }
 */
class Distance {
public:
    explicit Distance(int port);
    
    /**
     * Get distance in inches.
     * Returns -1 if no object detected.
     */
    double get_distance() const;
    
    /**
     * Get distance in millimeters.
     */
    int32_t get_distance_mm() const;
    
    /**
     * Get filtered distance (rolling average).
     */
    double get_filtered(int samples = 5);
    
    /**
     * Check if object is within range.
     */
    bool object_detected() const;
    
    /**
     * Check if object is within distance.
     */
    bool is_within(double inches) const { return get_distance() <= inches && get_distance() > 0; }
    
    /**
     * Get object size (0-400, larger = bigger object).
     */
    int32_t get_object_size() const;
    
    /**
     * Get object velocity (mm/s, positive = approaching).
     */
    double get_velocity() const;
    
    /**
     * Get confidence (0-63, higher = more confident).
     */
    int32_t get_confidence() const;
    
    /**
     * Get raw sensor.
     */
    pros::Distance& raw() { return sensor_; }

private:
    pros::Distance sensor_;
    mutable std::vector<double> history_;
};

// =============================================================================
// OPTICAL SENSOR (ENHANCED)
// =============================================================================

/**
 * Enhanced optical sensor for color detection and line following.
 * 
 * Usage:
 *   cwb::Optical optical(1);
 *   optical.set_led(100);
 *   
 *   auto color = optical.get_color();
 *   if (color == cwb::Optical::Color::Red) { ... }
 */
class Optical {
public:
    enum class Color { Unknown, Red, Orange, Yellow, Green, Blue, Purple };
    
    explicit Optical(int port);
    
    /**
     * Get detected color.
     */
    Color get_color() const;
    
    /**
     * Get color name as string.
     */
    std::string get_color_name() const;
    
    /**
     * Get hue (0-360).
     */
    double get_hue() const;
    
    /**
     * Get saturation (0-1).
     */
    double get_saturation() const;
    
    /**
     * Get brightness (0-1).
     */
    double get_brightness() const;
    
    /**
     * Get proximity (0-255, higher = closer).
     */
    int32_t get_proximity() const;
    
    /**
     * Check if object is close.
     */
    bool object_detected(int threshold = 200) const { return get_proximity() > threshold; }
    
    /**
     * Is the detected object the specified color?
     */
    bool is_color(Color color) const { return get_color() == color; }
    bool is_red() const { return is_color(Color::Red); }
    bool is_blue() const { return is_color(Color::Blue); }
    
    /**
     * Set LED brightness (0-100).
     */
    void set_led(uint8_t brightness);
    void led_on() { set_led(100); }
    void led_off() { set_led(0); }
    
    /**
     * Set integration time (affects accuracy vs speed).
     */
    void set_integration_time(double ms);
    
    /**
     * Enable gesture detection.
     */
    void enable_gesture();
    pros::optical_direction_e_t get_gesture();
    
    /**
     * Get raw sensor.
     */
    pros::Optical& raw() { return sensor_; }

private:
    pros::Optical sensor_;
    
    Color hue_to_color(double hue) const;
};

// =============================================================================
// LINE TRACKER ARRAY
// =============================================================================

/**
 * Multiple line sensors for line following.
 * 
 * Usage:
 *   cwb::LineArray lines({'A', 'B', 'C'});  // 3 sensors
 *   
 *   double error = lines.get_position();  // -1 to 1 (-1 = left, 1 = right)
 *   motor_turn = error * 50;  // Simple P control
 */
class LineArray {
public:
    /**
     * Create line tracker array.
     * @param ports ADI ports (left to right)
     */
    LineArray(std::initializer_list<char> ports);
    
    /**
     * Get line position (-1 to 1).
     * -1 = line is to the left
     *  0 = line is centered
     *  1 = line is to the right
     */
    double get_position() const;
    
    /**
     * Get raw values for each sensor (0-4095).
     */
    std::vector<int> get_raw() const;
    
    /**
     * Check if line is detected.
     */
    bool line_detected() const;
    
    /**
     * Calibrate sensors on white/black.
     */
    void calibrate_white();
    void calibrate_black();
    
    /**
     * Get individual sensor state (true = on line).
     */
    bool sensor_on_line(int index) const;

private:
    std::vector<std::unique_ptr<pros::adi::LineSensor>> sensors_;
    std::vector<int> white_values_;
    std::vector<int> black_values_;
    int threshold_ = 2048;
};

// =============================================================================
// LIMIT SWITCH / BUMPER
// =============================================================================

/**
 * Digital input for limit switches, bumpers, etc.
 */
class LimitSwitch {
public:
    explicit LimitSwitch(char port);
    
    /**
     * Is the switch pressed?
     */
    bool is_pressed() const;
    
    /**
     * Was the switch just pressed (new press)?
     */
    bool just_pressed();
    
    /**
     * Was the switch just released?
     */
    bool just_released();

private:
    pros::adi::DigitalIn sensor_;
    bool last_state_ = false;
};

// =============================================================================
// POTENTIOMETER
// =============================================================================

/**
 * Analog potentiometer for position sensing.
 */
class Potentiometer {
public:
    /**
     * Create potentiometer.
     * @param port ADI port
     * @param reversed True if rotation is reversed
     */
    Potentiometer(char port, bool reversed = false);
    
    /**
     * Get angle in degrees (0-250 typical).
     */
    double get_angle() const;
    
    /**
     * Get raw value (0-4095).
     */
    int get_raw() const;
    
    /**
     * Get value as percentage (0-100).
     */
    double get_percent() const;
    
    /**
     * Calibrate min/max positions.
     */
    void set_range(int min_raw, int max_raw);

private:
    pros::adi::Potentiometer sensor_;
    bool reversed_;
    int min_value_ = 0;
    int max_value_ = 4095;
};

// =============================================================================
// INERTIAL SENSOR (ENHANCED)
// =============================================================================

/**
 * Enhanced IMU with additional features.
 */
class IMU {
public:
    explicit IMU(int port);
    
    /**
     * Get heading (0-360, clockwise positive).
     */
    double get_heading() const;
    
    /**
     * Get rotation (continuous, can exceed 360).
     */
    double get_rotation() const;
    
    /**
     * Get pitch angle.
     */
    double get_pitch() const;
    
    /**
     * Get roll angle.
     */
    double get_roll() const;
    
    /**
     * Get acceleration (g's).
     */
    struct Accel { double x, y, z; };
    Accel get_accel() const;
    
    /**
     * Get gyro rates (deg/sec).
     */
    struct Gyro { double x, y, z; };
    Gyro get_gyro() const;
    
    /**
     * Reset heading to specific value.
     */
    void reset(double heading = 0);
    
    /**
     * Calibrate the IMU.
     */
    void calibrate();
    bool is_calibrating() const;
    
    /**
     * Detect if robot is tipping.
     */
    bool is_tipping(double threshold = 15) const;
    
    /**
     * Detect if robot is upside down.
     */
    bool is_inverted() const;
    
    /**
     * Get raw sensor.
     */
    pros::Imu& raw() { return sensor_; }

private:
    pros::Imu sensor_;
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

// Vision
Vision::Vision(int port) : sensor_(port) {}

std::vector<Vision::Object> Vision::get_objects(uint8_t signature, int max_count) {
    std::vector<Object> result;
    
    int count = sensor_.get_object_count();
    if (count <= 0) return result;
    
    for (int i = 0; i < std::min(count, max_count); i++) {
        auto obj = sensor_.get_by_sig(i, signature);
        if (obj.signature == signature) {
            result.push_back({
                obj.x_middle_coord,
                obj.y_middle_coord,
                obj.width,
                obj.height,
                static_cast<uint16_t>(obj.width * obj.height),
                static_cast<uint8_t>(obj.signature)
            });
        }
    }
    
    return result;
}

std::optional<Vision::Object> Vision::get_largest(uint8_t signature) {
    auto objects = get_objects(signature);
    if (objects.empty()) return std::nullopt;
    
    return *std::max_element(objects.begin(), objects.end(),
        [](const Object& a, const Object& b) { return a.area < b.area; });
}

std::optional<Vision::Object> Vision::get_closest(uint8_t signature) {
    auto objects = get_objects(signature);
    if (objects.empty()) return std::nullopt;
    
    return *std::max_element(objects.begin(), objects.end(),
        [](const Object& a, const Object& b) { return a.area < b.area; });  // Larger = closer
}

std::optional<Vision::Object> Vision::get_centered(uint8_t signature) {
    auto objects = get_objects(signature);
    if (objects.empty()) return std::nullopt;
    
    return *std::min_element(objects.begin(), objects.end(),
        [](const Object& a, const Object& b) { return std::abs(a.x) < std::abs(b.x); });
}

bool Vision::has_object(uint8_t signature) {
    return count(signature) > 0;
}

int Vision::count(uint8_t signature) {
    int total = 0;
    int count = sensor_.get_object_count();
    for (int i = 0; i < count; i++) {
        auto obj = sensor_.get_by_sig(i, signature);
        if (obj.signature == signature) total++;
    }
    return total;
}

void Vision::set_led(uint8_t brightness, uint32_t color) {
    sensor_.set_led(static_cast<pros::vision_zero>(brightness));
}

void Vision::set_white_balance(uint32_t rgb) {
    sensor_.set_white_balance(rgb);
}

void Vision::set_auto_white_balance(bool enabled) {
    if (enabled) sensor_.set_auto_white_balance(1);
}

void Vision::set_exposure(uint8_t percent) {
    sensor_.set_exposure(percent);
}

// GPS
GPS::GPS(int port, double x_offset, double y_offset)
    : sensor_(port), x_offset_(x_offset), y_offset_(y_offset) {}

GPS::Position GPS::get_position() const {
    auto status = sensor_.get_status();
    return {
        status.x * 39.37 + x_offset_,  // meters to inches
        status.y * 39.37 + y_offset_,
        status.yaw * M_PI / 180.0,
        sensor_.get_error(),
        sensor_.get_error() < 0.1
    };
}

GPS::Position GPS::get_field_position() const {
    auto pos = get_position();
    // Convert from GPS coordinates to field coordinates (0-144)
    pos.x += 72;  // Center is at (72, 72)
    pos.y += 72;
    return pos;
}

double GPS::get_heading_deg() const {
    double h = sensor_.get_heading();
    while (h < 0) h += 360;
    while (h >= 360) h -= 360;
    return h;
}

double GPS::get_heading_rad() const {
    return sensor_.get_heading() * M_PI / 180.0;
}

bool GPS::has_fix() const {
    return sensor_.get_error() < 0.5;
}

double GPS::get_error() const {
    return sensor_.get_error();
}

double GPS::get_rotation_rate() const {
    return sensor_.get_gyro_rate().z;
}

void GPS::set_position(double x, double y, double heading_deg) {
    sensor_.set_position(x / 39.37, y / 39.37, heading_deg);
}

void GPS::set_offset(double x, double y) {
    sensor_.set_offset(x / 39.37, y / 39.37);
    x_offset_ = x;
    y_offset_ = y;
}

void GPS::set_data_rate(uint32_t ms) {
    sensor_.set_data_rate(ms);
}

// Distance
Distance::Distance(int port) : sensor_(port) {
    history_.reserve(10);
}

double Distance::get_distance() const {
    int32_t mm = sensor_.get();
    if (mm == 0 || mm == PROS_ERR) return -1;
    return mm / 25.4;  // mm to inches
}

int32_t Distance::get_distance_mm() const {
    return sensor_.get();
}

double Distance::get_filtered(int samples) {
    double current = get_distance();
    if (current < 0) return -1;
    
    history_.push_back(current);
    if (static_cast<int>(history_.size()) > samples) {
        history_.erase(history_.begin());
    }
    
    double sum = 0;
    for (double v : history_) sum += v;
    return sum / history_.size();
}

bool Distance::object_detected() const {
    return get_distance() > 0;
}

int32_t Distance::get_object_size() const {
    return sensor_.get_object_size();
}

double Distance::get_velocity() const {
    return sensor_.get_object_velocity();
}

int32_t Distance::get_confidence() const {
    return sensor_.get_confidence();
}

// Optical
Optical::Optical(int port) : sensor_(port) {}

Optical::Color Optical::get_color() const {
    if (get_proximity() < 100) return Color::Unknown;
    return hue_to_color(get_hue());
}

std::string Optical::get_color_name() const {
    switch (get_color()) {
        case Color::Red: return "Red";
        case Color::Orange: return "Orange";
        case Color::Yellow: return "Yellow";
        case Color::Green: return "Green";
        case Color::Blue: return "Blue";
        case Color::Purple: return "Purple";
        default: return "Unknown";
    }
}

double Optical::get_hue() const {
    return sensor_.get_hue();
}

double Optical::get_saturation() const {
    return sensor_.get_saturation();
}

double Optical::get_brightness() const {
    return sensor_.get_brightness();
}

int32_t Optical::get_proximity() const {
    return sensor_.get_proximity();
}

void Optical::set_led(uint8_t brightness) {
    sensor_.set_led_pwm(brightness);
}

void Optical::set_integration_time(double ms) {
    sensor_.set_integration_time(ms);
}

void Optical::enable_gesture() {
    sensor_.enable_gesture();
}

pros::optical_direction_e_t Optical::get_gesture() {
    return sensor_.get_gesture();
}

Optical::Color Optical::hue_to_color(double hue) const {
    if (hue < 15 || hue > 345) return Color::Red;
    if (hue < 45) return Color::Orange;
    if (hue < 75) return Color::Yellow;
    if (hue < 165) return Color::Green;
    if (hue < 260) return Color::Blue;
    if (hue < 345) return Color::Purple;
    return Color::Unknown;
}

// LineArray
LineArray::LineArray(std::initializer_list<char> ports) {
    for (char port : ports) {
        sensors_.push_back(std::make_unique<pros::adi::LineSensor>(port));
        white_values_.push_back(4095);
        black_values_.push_back(0);
    }
}

double LineArray::get_position() const {
    if (sensors_.empty()) return 0;
    
    double weighted_sum = 0;
    double total_weight = 0;
    int n = static_cast<int>(sensors_.size());
    
    for (int i = 0; i < n; i++) {
        int value = sensors_[i]->get_value();
        // Normalize to 0-1 (1 = on line)
        double normalized = std::clamp(
            1.0 - static_cast<double>(value - black_values_[i]) / (white_values_[i] - black_values_[i]),
            0.0, 1.0);
        
        // Position weight: -1 for leftmost, +1 for rightmost
        double position = (2.0 * i / (n - 1)) - 1.0;
        
        weighted_sum += position * normalized;
        total_weight += normalized;
    }
    
    if (total_weight < 0.1) return 0;  // No line detected
    return weighted_sum / total_weight;
}

std::vector<int> LineArray::get_raw() const {
    std::vector<int> result;
    for (const auto& sensor : sensors_) {
        result.push_back(sensor->get_value());
    }
    return result;
}

bool LineArray::line_detected() const {
    for (size_t i = 0; i < sensors_.size(); i++) {
        if (sensor_on_line(static_cast<int>(i))) return true;
    }
    return false;
}

void LineArray::calibrate_white() {
    for (size_t i = 0; i < sensors_.size(); i++) {
        white_values_[i] = sensors_[i]->get_value();
    }
}

void LineArray::calibrate_black() {
    for (size_t i = 0; i < sensors_.size(); i++) {
        black_values_[i] = sensors_[i]->get_value();
    }
}

bool LineArray::sensor_on_line(int index) const {
    if (index < 0 || index >= static_cast<int>(sensors_.size())) return false;
    return sensors_[index]->get_value() < threshold_;
}

// LimitSwitch
LimitSwitch::LimitSwitch(char port) : sensor_(port) {}

bool LimitSwitch::is_pressed() const {
    return !sensor_.get_value();  // Active low
}

bool LimitSwitch::just_pressed() {
    bool current = is_pressed();
    bool result = current && !last_state_;
    last_state_ = current;
    return result;
}

bool LimitSwitch::just_released() {
    bool current = is_pressed();
    bool result = !current && last_state_;
    last_state_ = current;
    return result;
}

// Potentiometer
Potentiometer::Potentiometer(char port, bool reversed)
    : sensor_(port), reversed_(reversed) {}

double Potentiometer::get_angle() const {
    double value = get_percent();
    return value * 2.5;  // Typical 250 degree range
}

int Potentiometer::get_raw() const {
    return sensor_.get_value();
}

double Potentiometer::get_percent() const {
    int raw = get_raw();
    double percent = 100.0 * (raw - min_value_) / (max_value_ - min_value_);
    return reversed_ ? 100.0 - percent : percent;
}

void Potentiometer::set_range(int min_raw, int max_raw) {
    min_value_ = min_raw;
    max_value_ = max_raw;
}

// IMU
IMU::IMU(int port) : sensor_(port) {}

double IMU::get_heading() const {
    return sensor_.get_heading();
}

double IMU::get_rotation() const {
    return sensor_.get_rotation();
}

double IMU::get_pitch() const {
    return sensor_.get_pitch();
}

double IMU::get_roll() const {
    return sensor_.get_roll();
}

IMU::Accel IMU::get_accel() const {
    auto a = sensor_.get_accel();
    return {a.x, a.y, a.z};
}

IMU::Gyro IMU::get_gyro() const {
    auto g = sensor_.get_gyro_rate();
    return {g.x, g.y, g.z};
}

void IMU::reset(double heading) {
    sensor_.set_heading(heading);
}

void IMU::calibrate() {
    sensor_.reset();
}

bool IMU::is_calibrating() const {
    return sensor_.is_calibrating();
}

bool IMU::is_tipping(double threshold) const {
    return std::abs(get_pitch()) > threshold || std::abs(get_roll()) > threshold;
}

bool IMU::is_inverted() const {
    return std::abs(get_roll()) > 90;
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
