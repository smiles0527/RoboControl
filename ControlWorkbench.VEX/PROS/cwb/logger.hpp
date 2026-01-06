/**
 * ControlWorkbench Data Logging Library
 * Version: 1.0.0
 * 
 * SD card logging and match recording:
 * - Match data recording
 * - Autonomous replay
 * - CSV export
 * - Performance analysis
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "telemetry.hpp"
#include "api.h"
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>

namespace cwb {

// =============================================================================
// DATA LOGGER
// =============================================================================

/**
 * SD card data logger for match recording and analysis.
 * 
 * Usage:
 *   cwb::Logger logger;
 *   logger.start("match1");
 *   
 *   // In loop:
 *   logger.log("x", odom.get_x());
 *   logger.log("y", odom.get_y());
 *   logger.tick();  // Writes row every N ms
 *   
 *   // After match:
 *   logger.stop();
 */
class Logger {
public:
    Logger(int log_interval_ms = 20);
    ~Logger();
    
    /**
     * Start logging to a file.
     * @param name Base name for file (timestamp added automatically)
     */
    bool start(const std::string& name = "log");
    
    /**
     * Stop logging and close file.
     */
    void stop();
    
    /**
     * Log a named value.
     */
    void log(const std::string& name, double value);
    void log(const std::string& name, int value) { log(name, static_cast<double>(value)); }
    void log(const std::string& name, bool value) { log(name, value ? 1.0 : 0.0); }
    
    /**
     * Log a string message.
     */
    void log_message(const std::string& message);
    
    /**
     * Write current row (call every loop).
     * Only writes at configured interval.
     */
    void tick();
    
    /**
     * Force write current row.
     */
    void flush();
    
    /**
     * Is logging active?
     */
    bool is_logging() const { return is_logging_; }
    
    /**
     * Get current file path.
     */
    std::string get_file_path() const { return file_path_; }
    
    /**
     * Get bytes written.
     */
    size_t bytes_written() const { return bytes_written_; }
    
    /**
     * Set log interval.
     */
    void set_interval(int ms) { log_interval_ms_ = ms; }

private:
    std::ofstream file_;
    std::string file_path_;
    bool is_logging_ = false;
    int log_interval_ms_;
    uint32_t last_log_time_ = 0;
    size_t bytes_written_ = 0;
    
    std::vector<std::string> column_names_;
    std::vector<double> current_values_;
    std::vector<std::string> messages_;
    bool header_written_ = false;
    
    void write_header();
    void write_row();
    std::string generate_filename(const std::string& base);
};

// =============================================================================
// MATCH RECORDER
// =============================================================================

/**
 * Records complete match data for replay and analysis.
 * 
 * Usage:
 *   cwb::MatchRecorder recorder;
 *   
 *   // In autonomous:
 *   recorder.start_autonomous();
 *   
 *   // In opcontrol:
 *   recorder.start_driver();
 *   
 *   // Every loop:
 *   recorder.record(pose, left_power, right_power, ...);
 *   
 *   // After match:
 *   recorder.save();
 */
class MatchRecorder {
public:
    struct MatchFrame {
        uint32_t timestamp;
        double x, y, theta;
        double left_power, right_power;
        double velocity, angular_velocity;
        double battery_voltage;
        uint8_t subsystem_states;  // Bitfield for intake, lift, etc.
        
        std::string to_csv() const;
        static MatchFrame from_csv(const std::string& line);
    };
    
    MatchRecorder();
    
    /**
     * Start recording autonomous.
     */
    void start_autonomous();
    
    /**
     * Start recording driver control.
     */
    void start_driver();
    
    /**
     * Record a frame.
     */
    void record(double x, double y, double theta,
                double left_power, double right_power,
                double velocity = 0, double angular_velocity = 0);
    
    /**
     * Stop recording.
     */
    void stop();
    
    /**
     * Save to SD card.
     */
    bool save(const std::string& filename = "");
    
    /**
     * Load from SD card.
     */
    bool load(const std::string& filename);
    
    /**
     * Get recorded frames.
     */
    const std::vector<MatchFrame>& get_frames() const { return frames_; }
    
    /**
     * Clear recorded data.
     */
    void clear() { frames_.clear(); }
    
    /**
     * Get recording duration in seconds.
     */
    double duration() const;
    
    /**
     * Get average velocity.
     */
    double average_velocity() const;
    
    /**
     * Get total distance traveled.
     */
    double total_distance() const;

private:
    std::vector<MatchFrame> frames_;
    bool recording_ = false;
    bool is_autonomous_ = false;
    uint32_t start_time_ = 0;
    int record_interval_ = 20;
    uint32_t last_record_ = 0;
};

// =============================================================================
// AUTONOMOUS REPLAY
// =============================================================================

/**
 * Replay recorded autonomous runs.
 * 
 * Usage:
 *   cwb::AutonReplay replay;
 *   replay.load("skills_run.csv");
 *   
 *   // In autonomous:
 *   while (!replay.is_finished()) {
 *       auto target = replay.get_current();
 *       chassis.move_to_point(target.x, target.y);
 *       replay.advance();
 *   }
 */
class AutonReplay {
public:
    AutonReplay();
    
    /**
     * Load replay from file.
     */
    bool load(const std::string& filename);
    
    /**
     * Load from MatchRecorder.
     */
    void load_from(const MatchRecorder& recorder);
    
    /**
     * Reset to beginning.
     */
    void reset();
    
    /**
     * Get current target.
     */
    MatchRecorder::MatchFrame get_current() const;
    
    /**
     * Advance to next frame.
     */
    void advance();
    
    /**
     * Is replay finished?
     */
    bool is_finished() const { return current_index_ >= frames_.size(); }
    
    /**
     * Get progress (0-1).
     */
    double progress() const;
    
    /**
     * Get frame at specific time.
     */
    MatchRecorder::MatchFrame get_at_time(double seconds) const;

private:
    std::vector<MatchRecorder::MatchFrame> frames_;
    size_t current_index_ = 0;
};

// =============================================================================
// PERFORMANCE ANALYZER
// =============================================================================

/**
 * Analyze match performance from recorded data.
 */
class PerformanceAnalyzer {
public:
    struct Analysis {
        double avg_velocity;
        double max_velocity;
        double total_distance;
        double idle_time;      // Time with velocity < threshold
        double efficiency;     // distance / (time * max_velocity)
        
        // Motor analysis
        double avg_current_draw;
        double peak_current_draw;
        double min_battery_voltage;
        
        // Path analysis
        double path_deviation;  // Average deviation from intended path
        double turn_count;
        
        std::string summary() const;
    };
    
    /**
     * Analyze recorded match.
     */
    static Analysis analyze(const MatchRecorder& recorder);
    
    /**
     * Compare two recordings.
     */
    static std::string compare(const MatchRecorder& a, const MatchRecorder& b);
    
    /**
     * Export analysis to file.
     */
    static bool export_report(const Analysis& analysis, const std::string& filename);
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

#ifdef CWB_IMPLEMENTATION

// Logger
Logger::Logger(int log_interval_ms) : log_interval_ms_(log_interval_ms) {}

Logger::~Logger() {
    if (is_logging_) stop();
}

std::string Logger::generate_filename(const std::string& base) {
    // Use current time for filename
    uint32_t t = pros::millis();
    std::stringstream ss;
    ss << "/usd/" << base << "_" << t << ".csv";
    return ss.str();
}

bool Logger::start(const std::string& name) {
    if (is_logging_) stop();
    
    file_path_ = generate_filename(name);
    file_.open(file_path_);
    
    if (!file_.is_open()) {
        log_error("Failed to open log file: " + file_path_);
        return false;
    }
    
    is_logging_ = true;
    header_written_ = false;
    column_names_.clear();
    current_values_.clear();
    bytes_written_ = 0;
    last_log_time_ = pros::millis();
    
    log_info("Started logging to: " + file_path_);
    return true;
}

void Logger::stop() {
    if (!is_logging_) return;
    
    flush();
    file_.close();
    is_logging_ = false;
    
    log_info("Stopped logging. Wrote " + std::to_string(bytes_written_) + " bytes");
}

void Logger::log(const std::string& name, double value) {
    if (!is_logging_) return;
    
    // Find or add column
    auto it = std::find(column_names_.begin(), column_names_.end(), name);
    size_t index;
    if (it == column_names_.end()) {
        column_names_.push_back(name);
        current_values_.push_back(value);
        index = column_names_.size() - 1;
    } else {
        index = it - column_names_.begin();
        current_values_[index] = value;
    }
}

void Logger::log_message(const std::string& message) {
    messages_.push_back(message);
}

void Logger::tick() {
    if (!is_logging_) return;
    
    uint32_t now = pros::millis();
    if (now - last_log_time_ >= static_cast<uint32_t>(log_interval_ms_)) {
        write_row();
        last_log_time_ = now;
    }
}

void Logger::flush() {
    if (!is_logging_) return;
    write_row();
    file_.flush();
}

void Logger::write_header() {
    if (header_written_ || column_names_.empty()) return;
    
    std::stringstream ss;
    ss << "timestamp";
    for (const auto& name : column_names_) {
        ss << "," << name;
    }
    ss << ",messages\n";
    
    std::string header = ss.str();
    file_ << header;
    bytes_written_ += header.size();
    header_written_ = true;
}

void Logger::write_row() {
    if (!is_logging_ || current_values_.empty()) return;
    
    if (!header_written_) {
        write_header();
    }
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << pros::millis();
    
    for (double value : current_values_) {
        ss << "," << value;
    }
    
    // Add messages
    ss << ",\"";
    for (const auto& msg : messages_) {
        ss << msg << ";";
    }
    ss << "\"";
    messages_.clear();
    
    ss << "\n";
    
    std::string row = ss.str();
    file_ << row;
    bytes_written_ += row.size();
}

// MatchRecorder
MatchRecorder::MatchRecorder() {}

std::string MatchRecorder::MatchFrame::to_csv() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << timestamp << ","
       << x << "," << y << "," << theta << ","
       << left_power << "," << right_power << ","
       << velocity << "," << angular_velocity << ","
       << battery_voltage << ","
       << static_cast<int>(subsystem_states);
    return ss.str();
}

MatchRecorder::MatchFrame MatchRecorder::MatchFrame::from_csv(const std::string& line) {
    MatchFrame f;
    std::stringstream ss(line);
    char comma;
    ss >> f.timestamp >> comma
       >> f.x >> comma >> f.y >> comma >> f.theta >> comma
       >> f.left_power >> comma >> f.right_power >> comma
       >> f.velocity >> comma >> f.angular_velocity >> comma
       >> f.battery_voltage >> comma;
    int states;
    ss >> states;
    f.subsystem_states = static_cast<uint8_t>(states);
    return f;
}

void MatchRecorder::start_autonomous() {
    frames_.clear();
    recording_ = true;
    is_autonomous_ = true;
    start_time_ = pros::millis();
    last_record_ = 0;
    log_info("Started recording autonomous");
}

void MatchRecorder::start_driver() {
    frames_.clear();
    recording_ = true;
    is_autonomous_ = false;
    start_time_ = pros::millis();
    last_record_ = 0;
    log_info("Started recording driver control");
}

void MatchRecorder::record(double x, double y, double theta,
                            double left_power, double right_power,
                            double velocity, double angular_velocity) {
    if (!recording_) return;
    
    uint32_t now = pros::millis();
    if (now - last_record_ < static_cast<uint32_t>(record_interval_)) return;
    
    MatchFrame f;
    f.timestamp = now - start_time_;
    f.x = x;
    f.y = y;
    f.theta = theta;
    f.left_power = left_power;
    f.right_power = right_power;
    f.velocity = velocity;
    f.angular_velocity = angular_velocity;
    f.battery_voltage = pros::battery::get_voltage() / 1000.0;
    f.subsystem_states = 0;  // TODO: Add subsystem state tracking
    
    frames_.push_back(f);
    last_record_ = now;
}

void MatchRecorder::stop() {
    recording_ = false;
    log_info("Stopped recording. " + std::to_string(frames_.size()) + " frames captured");
}

bool MatchRecorder::save(const std::string& filename) {
    std::string path = filename.empty() ? 
        "/usd/match_" + std::to_string(pros::millis()) + ".csv" : 
        "/usd/" + filename;
    
    std::ofstream file(path);
    if (!file.is_open()) {
        log_error("Failed to save: " + path);
        return false;
    }
    
    // Header
    file << "timestamp,x,y,theta,left_power,right_power,velocity,angular_velocity,battery,subsystems\n";
    
    // Data
    for (const auto& f : frames_) {
        file << f.to_csv() << "\n";
    }
    
    file.close();
    log_info("Saved to: " + path);
    return true;
}

bool MatchRecorder::load(const std::string& filename) {
    std::string path = "/usd/" + filename;
    std::ifstream file(path);
    if (!file.is_open()) {
        log_error("Failed to load: " + path);
        return false;
    }
    
    frames_.clear();
    std::string line;
    std::getline(file, line);  // Skip header
    
    while (std::getline(file, line)) {
        if (!line.empty()) {
            frames_.push_back(MatchFrame::from_csv(line));
        }
    }
    
    log_info("Loaded " + std::to_string(frames_.size()) + " frames from: " + path);
    return true;
}

double MatchRecorder::duration() const {
    if (frames_.empty()) return 0;
    return frames_.back().timestamp / 1000.0;
}

double MatchRecorder::average_velocity() const {
    if (frames_.empty()) return 0;
    double sum = 0;
    for (const auto& f : frames_) sum += std::abs(f.velocity);
    return sum / frames_.size();
}

double MatchRecorder::total_distance() const {
    if (frames_.size() < 2) return 0;
    double total = 0;
    for (size_t i = 1; i < frames_.size(); i++) {
        double dx = frames_[i].x - frames_[i-1].x;
        double dy = frames_[i].y - frames_[i-1].y;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

// AutonReplay
AutonReplay::AutonReplay() {}

bool AutonReplay::load(const std::string& filename) {
    MatchRecorder recorder;
    if (!recorder.load(filename)) return false;
    frames_ = recorder.get_frames();
    reset();
    return true;
}

void AutonReplay::load_from(const MatchRecorder& recorder) {
    frames_ = recorder.get_frames();
    reset();
}

void AutonReplay::reset() {
    current_index_ = 0;
}

MatchRecorder::MatchFrame AutonReplay::get_current() const {
    if (current_index_ >= frames_.size()) {
        return {};
    }
    return frames_[current_index_];
}

void AutonReplay::advance() {
    if (current_index_ < frames_.size()) {
        current_index_++;
    }
}

double AutonReplay::progress() const {
    if (frames_.empty()) return 1.0;
    return static_cast<double>(current_index_) / frames_.size();
}

MatchRecorder::MatchFrame AutonReplay::get_at_time(double seconds) const {
    uint32_t target_ms = static_cast<uint32_t>(seconds * 1000);
    
    for (const auto& f : frames_) {
        if (f.timestamp >= target_ms) return f;
    }
    
    return frames_.empty() ? MatchRecorder::MatchFrame{} : frames_.back();
}

// PerformanceAnalyzer
PerformanceAnalyzer::Analysis PerformanceAnalyzer::analyze(const MatchRecorder& recorder) {
    Analysis a = {};
    const auto& frames = recorder.get_frames();
    
    if (frames.empty()) return a;
    
    double sum_velocity = 0;
    double sum_current = 0;
    int idle_frames = 0;
    
    for (const auto& f : frames) {
        sum_velocity += std::abs(f.velocity);
        sum_current += (std::abs(f.left_power) + std::abs(f.right_power)) / 2;
        
        if (std::abs(f.velocity) < 1.0) idle_frames++;
        if (std::abs(f.velocity) > a.max_velocity) a.max_velocity = std::abs(f.velocity);
        if (f.battery_voltage < a.min_battery_voltage || a.min_battery_voltage == 0) {
            a.min_battery_voltage = f.battery_voltage;
        }
    }
    
    a.avg_velocity = sum_velocity / frames.size();
    a.avg_current_draw = sum_current / frames.size();
    a.total_distance = recorder.total_distance();
    a.idle_time = idle_frames * 0.02;  // Assuming 20ms intervals
    
    double duration = recorder.duration();
    if (duration > 0 && a.max_velocity > 0) {
        a.efficiency = a.total_distance / (duration * a.max_velocity);
    }
    
    return a;
}

std::string PerformanceAnalyzer::Analysis::summary() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "=== Performance Analysis ===\n";
    ss << "Average Velocity: " << avg_velocity << " in/s\n";
    ss << "Max Velocity: " << max_velocity << " in/s\n";
    ss << "Total Distance: " << total_distance << " in\n";
    ss << "Idle Time: " << idle_time << " s\n";
    ss << "Efficiency: " << (efficiency * 100) << "%\n";
    ss << "Min Battery: " << min_battery_voltage << " V\n";
    return ss.str();
}

std::string PerformanceAnalyzer::compare(const MatchRecorder& a, const MatchRecorder& b) {
    auto aa = analyze(a);
    auto ab = analyze(b);
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "=== Comparison ===\n";
    ss << "Distance: " << aa.total_distance << " vs " << ab.total_distance;
    ss << " (" << (ab.total_distance - aa.total_distance) << " diff)\n";
    ss << "Avg Velocity: " << aa.avg_velocity << " vs " << ab.avg_velocity << "\n";
    ss << "Efficiency: " << (aa.efficiency*100) << "% vs " << (ab.efficiency*100) << "%\n";
    
    return ss.str();
}

bool PerformanceAnalyzer::export_report(const Analysis& analysis, const std::string& filename) {
    std::string path = "/usd/" + filename;
    std::ofstream file(path);
    if (!file.is_open()) return false;
    
    file << analysis.summary();
    file.close();
    return true;
}

#endif // CWB_IMPLEMENTATION

} // namespace cwb
