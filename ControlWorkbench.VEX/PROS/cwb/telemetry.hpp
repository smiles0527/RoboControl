/**
 * ControlWorkbench PROS Telemetry Library
 * Version: 1.0.0
 * 
 * Single-header library for real-time communication between
 * VEX V5 robots running PROS and ControlWorkbench on PC.
 * 
 * INSTALLATION:
 *   1. Copy the 'cwb' folder to your PROS project's 'include' directory
 *   2. In exactly ONE .cpp file, add before including:
 *        #define CWB_IMPLEMENTATION
 *        #include "cwb/telemetry.hpp"
 *   3. In all other files, just:
 *        #include "cwb/telemetry.hpp"
 * 
 * QUICK START:
 *   void initialize() {
 *       cwb::init();
 *   }
 *   
 *   void opcontrol() {
 *       while (true) {
 *           cwb::update();
 *           cwb::send_odometry(x, y, theta);
 *           pros::delay(10);
 *       }
 *   }
 * 
 * FEATURES:
 *   - Bidirectional telemetry over USB/UART
 *   - Remote parameter tuning (PID gains, speeds, etc.)
 *   - Odometry visualization
 *   - Motor/sensor monitoring
 *   - Logging to ControlWorkbench console
 *   - Emergency stop callback
 *   - Path waypoint streaming
 * 
 * Copyright (c) 2024 ControlWorkbench
 * MIT License
 */

#pragma once

#include "api.h"
#include <cstdint>
#include <string>
#include <functional>
#include <map>
#include <vector>
#include <memory>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace cwb {

// =============================================================================
// CONFIGURATION
// =============================================================================

/// Library version
constexpr const char* VERSION = "1.0.0";

/// Protocol constants (must match C# VexProtocol)
constexpr uint8_t PREAMBLE1 = 0xAA;
constexpr uint8_t PREAMBLE2 = 0x55;
constexpr int BAUD_RATE = 115200;
constexpr int MAX_PAYLOAD = 255;

/// Default update intervals
constexpr int DEFAULT_HEARTBEAT_MS = 500;
constexpr int CONNECTION_TIMEOUT_MS = 2000;

// =============================================================================
// MESSAGE TYPES
// =============================================================================

/// Message types (must match C# VexMessageType enum)
enum class MessageType : uint8_t {
    // System messages
    Heartbeat = 0x00,
    Acknowledge = 0x01,
    Error = 0x02,
    SystemStatus = 0x03,
    BatteryStatus = 0x04,
    CompetitionStatus = 0x05,
    
    // Sensor telemetry
    Odometry = 0x10,
    ImuData = 0x11,
    MotorTelemetry = 0x12,
    EncoderData = 0x13,
    RotationSensor = 0x14,
    DistanceSensor = 0x15,
    
    // Parameter tuning
    SetParameter = 0x30,
    GetParameter = 0x31,
    ParameterValue = 0x32,
    ParameterList = 0x33,
    PidState = 0x34,
    
    // Path following
    PathWaypoint = 0x50,
    PathComplete = 0x51,
    PathProgress = 0x52,
    
    // Logging
    LogMessage = 0x60,
    DebugValue = 0x61,
    GraphData = 0x62,
    
    // Commands
    StartAutonomous = 0x70,
    StopAutonomous = 0x71,
    RunPath = 0x72,
    ResetOdometry = 0x73,
    CalibrateIMU = 0x74,
    EmergencyStop = 0x7F
};

/// Log levels
enum class LogLevel : uint8_t {
    Debug = 0,
    Info = 1,
    Warning = 2,
    Error = 3
};

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

class TunableParam;
struct PIDGains;

// Internal functions
namespace detail {
    void process_set_parameter(const uint8_t* payload, int len);
}

// =============================================================================
// CORE FUNCTIONS
// =============================================================================

/**
 * Initialize the telemetry system.
 * Call this in initialize().
 * 
 * @param port UART port number (1 or 2). Use 0 for USB (stdout).
 * @return true if initialization succeeded
 */
bool init(int port = 0);

/**
 * Shutdown the telemetry system.
 */
void shutdown();

/**
 * Process incoming messages and send heartbeat.
 * Call this periodically in your control loop (every 10-20ms).
 */
void update();

/**
 * Check if connected to ControlWorkbench.
 * @return true if heartbeat received within timeout
 */
bool is_connected();

/**
 * Get time since last communication in milliseconds.
 */
uint32_t get_last_comm_age();

// =============================================================================
// TELEMETRY SENDING
// =============================================================================

/**
 * Send odometry data to ControlWorkbench.
 * Position in inches, angle in radians.
 */
void send_odometry(double x, double y, double theta,
                   double vel_x = 0, double vel_y = 0, double angular_vel = 0);

/**
 * Send IMU data.
 * Angles in degrees, rates in degrees/sec, accel in g's.
 */
void send_imu(double heading, double pitch, double roll,
              double gyro_x = 0, double gyro_y = 0, double gyro_z = 0,
              double accel_x = 0, double accel_y = 0, double accel_z = 0);

/**
 * Send motor telemetry.
 * @param port Motor port (1-21)
 * @param position Position in degrees
 * @param velocity Velocity in RPM
 * @param current Current draw in mA
 * @param voltage Voltage in mV
 * @param temperature Temperature in degrees C
 * @param power Power in watts
 * @param torque Torque in Nm
 */
void send_motor(uint8_t port, double position, double velocity, 
                double current, double voltage, double temperature,
                double power = 0, double torque = 0);

/**
 * Send motor telemetry for a PROS motor object.
 */
void send_motor(const pros::Motor& motor);

/**
 * Send PID controller state for tuning visualization.
 */
void send_pid_state(uint8_t controller_id, 
                    double setpoint, double measurement,
                    double error, double integral, double derivative,
                    double output, double kp, double ki, double kd);

/**
 * Send battery status.
 */
void send_battery(double voltage, double current, double capacity, double temperature);

/**
 * Send battery status from PROS battery API.
 */
void send_battery();

/**
 * Send competition status.
 */
void send_competition_status(bool is_autonomous, bool is_enabled, bool is_connected);

/**
 * Send path following progress.
 * @param progress 0.0 to 1.0
 * @param current_x Current X position
 * @param current_y Current Y position  
 * @param target_x Target X position
 * @param target_y Target Y position
 * @param lookahead_x Lookahead point X
 * @param lookahead_y Lookahead point Y
 */
void send_path_progress(double progress, 
                        double current_x, double current_y,
                        double target_x, double target_y,
                        double lookahead_x = 0, double lookahead_y = 0);

// =============================================================================
// LOGGING
// =============================================================================

/**
 * Send a log message to ControlWorkbench.
 */
void log(LogLevel level, const std::string& message);
void log_debug(const std::string& message);
void log_info(const std::string& message);
void log_warning(const std::string& message);
void log_error(const std::string& message);

/**
 * Send a named debug value for graphing.
 * Use this to plot any value in real-time.
 */
void send_debug_value(const std::string& name, double value);

/**
 * Send multiple graph data points at once.
 */
void send_graph_data(const std::string& series_name, 
                     const std::vector<std::pair<double, double>>& points);

// =============================================================================
// TUNABLE PARAMETERS
// =============================================================================

/**
 * A parameter that can be adjusted remotely from ControlWorkbench.
 * 
 * Usage:
 *   cwb::TunableParam& max_speed = cwb::param("max_speed", 127);
 *   motor.move(max_speed.get());  // or just: motor.move(max_speed);
 */
class TunableParam {
public:
    TunableParam(const std::string& name, double default_value, 
                 double min_val = -1e9, double max_val = 1e9);
    
    /// Get current value
    double get() const { return value_; }
    
    /// Implicit conversion to double
    operator double() const { return value_; }
    
    /// Set value (also called remotely)
    void set(double v);
    
    /// Get parameter name
    const std::string& name() const { return name_; }
    
    /// Check if this parameter was updated this frame
    bool was_updated() const { return updated_this_frame_; }
    
    /// Get min/max bounds
    double min() const { return min_; }
    double max() const { return max_; }
    
private:
    std::string name_;
    double value_;
    double min_;
    double max_;
    bool updated_this_frame_ = false;
    
    friend void detail::process_set_parameter(const uint8_t* payload, int len);
    friend void update();
};

/**
 * Create or get a tunable parameter.
 * Returns a reference to the parameter for easy access.
 */
TunableParam& param(const std::string& name, double default_value,
                    double min_val = -1e9, double max_val = 1e9);

/**
 * Get a parameter value by name.
 * Returns 0 if parameter doesn't exist.
 */
double get_param(const std::string& name);

/**
 * Check if a parameter exists.
 */
bool has_param(const std::string& name);

/**
 * PID gains wrapper for easy tuning.
 * Creates three linked parameters: prefix.kP, prefix.kI, prefix.kD
 */
struct PIDGains {
    TunableParam* kP;
    TunableParam* kI;
    TunableParam* kD;
    TunableParam* kF;  // Optional feedforward
    
    PIDGains() : kP(nullptr), kI(nullptr), kD(nullptr), kF(nullptr) {}
    PIDGains(const std::string& prefix, double p, double i, double d, double f = 0);
    
    double p() const { return kP ? kP->get() : 0; }
    double i() const { return kI ? kI->get() : 0; }
    double d() const { return kD ? kD->get() : 0; }
    double f() const { return kF ? kF->get() : 0; }
    
    /// Check if any gain was updated this frame
    bool was_updated() const;
};

/**
 * Create tunable PID gains.
 */
PIDGains make_pid_gains(const std::string& prefix, double kp, double ki, double kd, double kf = 0);

/**
 * Feedforward gains for motion profiling.
 */
struct FeedforwardGains {
    TunableParam* kS;  // Static friction
    TunableParam* kV;  // Velocity
    TunableParam* kA;  // Acceleration
    TunableParam* kG;  // Gravity (for arms)
    
    FeedforwardGains() : kS(nullptr), kV(nullptr), kA(nullptr), kG(nullptr) {}
    FeedforwardGains(const std::string& prefix, double s, double v, double a, double g = 0);
    
    double s() const { return kS ? kS->get() : 0; }
    double v() const { return kV ? kV->get() : 0; }
    double a() const { return kA ? kA->get() : 0; }
    double g() const { return kG ? kG->get() : 0; }
    
    /// Calculate feedforward output
    double calculate(double velocity, double acceleration = 0, double cos_angle = 0) const;
};

/**
 * Create tunable feedforward gains.
 */
FeedforwardGains make_ff_gains(const std::string& prefix, double ks, double kv, double ka = 0, double kg = 0);

// =============================================================================
// CALLBACKS
// =============================================================================

/**
 * Set callback for odometry reset command.
 * Called when ControlWorkbench sends a reset position command.
 */
void on_odometry_reset(std::function<void(double x, double y, double theta)> callback);

/**
 * Set callback for emergency stop command.
 * You should stop all motors immediately when this is called!
 */
void on_emergency_stop(std::function<void()> callback);

/**
 * Set callback for path waypoint received.
 * Called when ControlWorkbench streams a path to the robot.
 */
void on_waypoint(std::function<void(double x, double y, double theta)> callback);

/**
 * Set callback for run path command.
 * Called when user clicks "Run" on a path in ControlWorkbench.
 */
void on_run_path(std::function<void(int path_id)> callback);

/**
 * Set callback for autonomous start command.
 */
void on_start_autonomous(std::function<void(int auton_id)> callback);

/**
 * Set callback for any parameter change.
 */
void on_parameter_changed(std::function<void(const std::string& name, double value)> callback);

// =============================================================================
// LOW-LEVEL FUNCTIONS
// =============================================================================

/**
 * Send a raw message with payload.
 */
void send_message(MessageType type, const uint8_t* payload, int len);

/**
 * Send a message with no payload.
 */
void send_message(MessageType type);

/**
 * Get the number of bytes available to read.
 */
int bytes_available();

/**
 * Get statistics.
 */
struct Stats {
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t bytes_sent;
    uint32_t bytes_received;
    uint32_t checksum_errors;
    uint32_t parse_errors;
};

Stats get_stats();

} // namespace cwb


// =============================================================================
// =============================================================================
// IMPLEMENTATION
// =============================================================================
// =============================================================================

#ifdef CWB_IMPLEMENTATION

namespace cwb {

// -----------------------------------------------------------------------------
// Internal state
// -----------------------------------------------------------------------------

namespace detail {

static pros::Serial* serial_port = nullptr;
static bool use_stdout = false;
static uint32_t last_heartbeat_recv = 0;
static uint32_t last_heartbeat_sent = 0;
static bool connected = false;
static bool initialized = false;

static std::map<std::string, TunableParam*> parameters;
static std::vector<std::unique_ptr<TunableParam>> param_storage;

// Callbacks
static std::function<void(double, double, double)> odom_reset_callback;
static std::function<void()> emergency_stop_callback;
static std::function<void(double, double, double)> waypoint_callback;
static std::function<void(int)> run_path_callback;
static std::function<void(int)> start_auton_callback;
static std::function<void(const std::string&, double)> param_changed_callback;

// Statistics
static Stats stats = {0, 0, 0, 0, 0, 0};

// Message buffer
static uint8_t tx_buffer[MAX_PAYLOAD + 10];
static uint8_t rx_buffer[MAX_PAYLOAD + 10];

// Parser state
enum class ParseState {
    WaitPreamble1,
    WaitPreamble2,
    WaitType,
    WaitLength,
    ReadPayload,
    WaitChecksum
};

static ParseState parse_state = ParseState::WaitPreamble1;
static MessageType current_type;
static int current_length = 0;
static int payload_index = 0;

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

static uint8_t compute_checksum(MessageType type, const uint8_t* payload, int len) {
    uint8_t cs = static_cast<uint8_t>(type) ^ static_cast<uint8_t>(len);
    for (int i = 0; i < len; i++) {
        cs ^= payload[i];
    }
    return cs;
}

static void write_bytes(const uint8_t* data, int len) {
    if (use_stdout) {
        fwrite(data, 1, len, stdout);
        fflush(stdout);
    } else if (serial_port) {
        serial_port->write(data, len);
    }
    stats.bytes_sent += len;
}

static int read_byte() {
    if (use_stdout) {
        // Non-blocking read from stdin
        // Note: This requires proper terminal setup
        return -1;  // Simplified - use serial for bidirectional
    } else if (serial_port && serial_port->get_read_avail() > 0) {
        uint8_t byte;
        if (serial_port->read(&byte, 1) == 1) {
            stats.bytes_received++;
            return byte;
        }
    }
    return -1;
}

// -----------------------------------------------------------------------------
// Message processing
// -----------------------------------------------------------------------------

void process_set_parameter(const uint8_t* payload, int len) {
    // Find null terminator for name
    int null_pos = -1;
    for (int i = 0; i < len && i < 128; i++) {
        if (payload[i] == 0) {
            null_pos = i;
            break;
        }
    }
    if (null_pos < 0 || null_pos + 9 > len) return;
    
    std::string name(reinterpret_cast<const char*>(payload), null_pos);
    double value;
    std::memcpy(&value, payload + null_pos + 1, sizeof(double));
    
    auto it = parameters.find(name);
    if (it != parameters.end()) {
        it->second->set(value);
        it->second->updated_this_frame_ = true;
        
        if (param_changed_callback) {
            param_changed_callback(name, value);
        }
        
        send_message(MessageType::Acknowledge);
    }
}

static void process_message(MessageType type, const uint8_t* payload, int len) {
    stats.messages_received++;
    
    switch (type) {
        case MessageType::Heartbeat:
            last_heartbeat_recv = pros::millis();
            connected = true;
            send_message(MessageType::Acknowledge);
            break;
            
        case MessageType::SetParameter:
            process_set_parameter(payload, len);
            break;
            
        case MessageType::GetParameter: {
            if (len < 1) break;
            std::string name(reinterpret_cast<const char*>(payload));
            auto it = parameters.find(name);
            if (it != parameters.end()) {
                uint8_t resp[256];
                int name_len = static_cast<int>(name.length());
                std::memcpy(resp, name.c_str(), name_len + 1);
                double val = it->second->get();
                std::memcpy(resp + name_len + 1, &val, sizeof(double));
                send_message(MessageType::ParameterValue, resp, name_len + 9);
            }
            break;
        }
        
        case MessageType::ResetOdometry:
            if (len >= 24 && odom_reset_callback) {
                double x, y, theta;
                std::memcpy(&x, payload, sizeof(double));
                std::memcpy(&y, payload + 8, sizeof(double));
                std::memcpy(&theta, payload + 16, sizeof(double));
                odom_reset_callback(x, y, theta);
            }
            break;
            
        case MessageType::EmergencyStop:
            if (emergency_stop_callback) {
                emergency_stop_callback();
            }
            break;
            
        case MessageType::PathWaypoint:
            if (len >= 24 && waypoint_callback) {
                double x, y, theta;
                std::memcpy(&x, payload, sizeof(double));
                std::memcpy(&y, payload + 8, sizeof(double));
                std::memcpy(&theta, payload + 16, sizeof(double));
                waypoint_callback(x, y, theta);
            }
            break;
            
        case MessageType::RunPath:
            if (len >= 4 && run_path_callback) {
                int32_t path_id;
                std::memcpy(&path_id, payload, sizeof(int32_t));
                run_path_callback(path_id);
            }
            break;
            
        case MessageType::StartAutonomous:
            if (len >= 4 && start_auton_callback) {
                int32_t auton_id;
                std::memcpy(&auton_id, payload, sizeof(int32_t));
                start_auton_callback(auton_id);
            }
            break;
            
        default:
            break;
    }
}

static void process_byte(uint8_t b) {
    switch (parse_state) {
        case ParseState::WaitPreamble1:
            if (b == PREAMBLE1) parse_state = ParseState::WaitPreamble2;
            break;
            
        case ParseState::WaitPreamble2:
            if (b == PREAMBLE2) parse_state = ParseState::WaitType;
            else {
                parse_state = ParseState::WaitPreamble1;
                stats.parse_errors++;
            }
            break;
            
        case ParseState::WaitType:
            current_type = static_cast<MessageType>(b);
            parse_state = ParseState::WaitLength;
            break;
            
        case ParseState::WaitLength:
            current_length = b;
            payload_index = 0;
            if (current_length == 0) {
                parse_state = ParseState::WaitChecksum;
            } else if (current_length > MAX_PAYLOAD) {
                parse_state = ParseState::WaitPreamble1;
                stats.parse_errors++;
            } else {
                parse_state = ParseState::ReadPayload;
            }
            break;
            
        case ParseState::ReadPayload:
            rx_buffer[payload_index++] = b;
            if (payload_index >= current_length) {
                parse_state = ParseState::WaitChecksum;
            }
            break;
            
        case ParseState::WaitChecksum: {
            uint8_t expected = compute_checksum(current_type, rx_buffer, current_length);
            if (b == expected) {
                process_message(current_type, rx_buffer, current_length);
            } else {
                stats.checksum_errors++;
            }
            parse_state = ParseState::WaitPreamble1;
            break;
        }
    }
}

} // namespace detail

// -----------------------------------------------------------------------------
// Public API implementation
// -----------------------------------------------------------------------------

bool init(int port) {
    if (detail::initialized) return true;
    
    if (port == 0) {
        detail::use_stdout = true;
        detail::serial_port = nullptr;
    } else {
        detail::use_stdout = false;
        detail::serial_port = new pros::Serial(port, BAUD_RATE);
    }
    
    detail::last_heartbeat_recv = 0;
    detail::last_heartbeat_sent = 0;
    detail::connected = false;
    detail::initialized = true;
    detail::stats = {0, 0, 0, 0, 0, 0};
    
    log_info("ControlWorkbench telemetry initialized v" + std::string(VERSION));
    
    return true;
}

void shutdown() {
    if (detail::serial_port) {
        delete detail::serial_port;
        detail::serial_port = nullptr;
    }
    detail::initialized = false;
}

void update() {
    if (!detail::initialized) return;
    
    // Clear updated flags on all parameters
    for (auto& pair : detail::parameters) {
        pair.second->updated_this_frame_ = false;
    }
    
    // Process incoming data
    int byte;
    int max_reads = 256;  // Prevent infinite loop
    while ((byte = detail::read_byte()) >= 0 && max_reads-- > 0) {
        detail::process_byte(static_cast<uint8_t>(byte));
    }
    
    // Send heartbeat periodically
    uint32_t now = pros::millis();
    if (now - detail::last_heartbeat_sent >= DEFAULT_HEARTBEAT_MS) {
        send_message(MessageType::Heartbeat);
        detail::last_heartbeat_sent = now;
    }
    
    // Check connection timeout
    if (detail::last_heartbeat_recv > 0 && 
        now - detail::last_heartbeat_recv > CONNECTION_TIMEOUT_MS) {
        detail::connected = false;
    }
}

bool is_connected() {
    return detail::connected;
}

uint32_t get_last_comm_age() {
    if (detail::last_heartbeat_recv == 0) return UINT32_MAX;
    return pros::millis() - detail::last_heartbeat_recv;
}

void send_message(MessageType type, const uint8_t* payload, int len) {
    if (!detail::initialized) return;
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
    
    detail::tx_buffer[0] = PREAMBLE1;
    detail::tx_buffer[1] = PREAMBLE2;
    detail::tx_buffer[2] = static_cast<uint8_t>(type);
    detail::tx_buffer[3] = static_cast<uint8_t>(len);
    
    if (len > 0 && payload) {
        std::memcpy(detail::tx_buffer + 4, payload, len);
    }
    
    detail::tx_buffer[4 + len] = detail::compute_checksum(type, payload, len);
    
    detail::write_bytes(detail::tx_buffer, 5 + len);
    detail::stats.messages_sent++;
}

void send_message(MessageType type) {
    send_message(type, nullptr, 0);
}

void send_odometry(double x, double y, double theta,
                   double vel_x, double vel_y, double angular_vel) {
    uint8_t payload[52];
    uint32_t ts = pros::millis();
    std::memcpy(payload, &x, 8);
    std::memcpy(payload + 8, &y, 8);
    std::memcpy(payload + 16, &theta, 8);
    std::memcpy(payload + 24, &vel_x, 8);
    std::memcpy(payload + 32, &vel_y, 8);
    std::memcpy(payload + 40, &angular_vel, 8);
    std::memcpy(payload + 48, &ts, 4);
    send_message(MessageType::Odometry, payload, 52);
}

void send_imu(double heading, double pitch, double roll,
              double gyro_x, double gyro_y, double gyro_z,
              double accel_x, double accel_y, double accel_z) {
    uint8_t payload[72];
    std::memcpy(payload, &heading, 8);
    std::memcpy(payload + 8, &pitch, 8);
    std::memcpy(payload + 16, &roll, 8);
    std::memcpy(payload + 24, &gyro_x, 8);
    std::memcpy(payload + 32, &gyro_y, 8);
    std::memcpy(payload + 40, &gyro_z, 8);
    std::memcpy(payload + 48, &accel_x, 8);
    std::memcpy(payload + 56, &accel_y, 8);
    std::memcpy(payload + 64, &accel_z, 8);
    send_message(MessageType::ImuData, payload, 72);
}

void send_motor(uint8_t port, double position, double velocity, 
                double current, double voltage, double temperature,
                double power, double torque) {
    uint8_t payload[67];
    payload[0] = port;
    std::memcpy(payload + 1, &position, 8);
    std::memcpy(payload + 9, &velocity, 8);
    std::memcpy(payload + 17, &current, 8);
    std::memcpy(payload + 25, &voltage, 8);
    std::memcpy(payload + 33, &temperature, 8);
    std::memcpy(payload + 41, &power, 8);
    std::memcpy(payload + 49, &torque, 8);
    std::memset(payload + 57, 0, 10);  // Reserved
    send_message(MessageType::MotorTelemetry, payload, 67);
}

void send_motor(const pros::Motor& motor) {
    send_motor(
        motor.get_port(),
        motor.get_position(),
        motor.get_actual_velocity(),
        motor.get_current_draw(),
        motor.get_voltage(),
        motor.get_temperature(),
        motor.get_power(),
        motor.get_torque()
    );
}

void send_pid_state(uint8_t controller_id, 
                    double setpoint, double measurement,
                    double error, double integral, double derivative,
                    double output, double kp, double ki, double kd) {
    uint8_t payload[73];
    payload[0] = controller_id;
    std::memcpy(payload + 1, &setpoint, 8);
    std::memcpy(payload + 9, &measurement, 8);
    std::memcpy(payload + 17, &error, 8);
    std::memcpy(payload + 25, &integral, 8);
    std::memcpy(payload + 33, &derivative, 8);
    std::memcpy(payload + 41, &output, 8);
    std::memcpy(payload + 49, &kp, 8);
    std::memcpy(payload + 57, &ki, 8);
    std::memcpy(payload + 65, &kd, 8);
    send_message(MessageType::PidState, payload, 73);
}

void send_battery(double voltage, double current, double capacity, double temperature) {
    uint8_t payload[32];
    std::memcpy(payload, &voltage, 8);
    std::memcpy(payload + 8, &current, 8);
    std::memcpy(payload + 16, &capacity, 8);
    std::memcpy(payload + 24, &temperature, 8);
    send_message(MessageType::BatteryStatus, payload, 32);
}

void send_battery() {
    send_battery(
        pros::battery::get_voltage() / 1000.0,
        pros::battery::get_current() / 1000.0,
        pros::battery::get_capacity(),
        pros::battery::get_temperature()
    );
}

void send_competition_status(bool is_autonomous, bool is_enabled, bool is_connected) {
    uint8_t payload[3];
    payload[0] = is_autonomous ? 1 : 0;
    payload[1] = is_enabled ? 1 : 0;
    payload[2] = is_connected ? 1 : 0;
    send_message(MessageType::CompetitionStatus, payload, 3);
}

void send_path_progress(double progress, 
                        double current_x, double current_y,
                        double target_x, double target_y,
                        double lookahead_x, double lookahead_y) {
    uint8_t payload[56];
    std::memcpy(payload, &progress, 8);
    std::memcpy(payload + 8, &current_x, 8);
    std::memcpy(payload + 16, &current_y, 8);
    std::memcpy(payload + 24, &target_x, 8);
    std::memcpy(payload + 32, &target_y, 8);
    std::memcpy(payload + 40, &lookahead_x, 8);
    std::memcpy(payload + 48, &lookahead_y, 8);
    send_message(MessageType::PathProgress, payload, 56);
}

void log(LogLevel level, const std::string& message) {
    uint8_t payload[256];
    payload[0] = static_cast<uint8_t>(level);
    uint32_t ts = pros::millis();
    std::memcpy(payload + 1, &ts, 4);
    int len = std::min(static_cast<int>(message.length()), 250);
    std::memcpy(payload + 5, message.c_str(), len);
    send_message(MessageType::LogMessage, payload, 5 + len);
}

void log_debug(const std::string& message) { log(LogLevel::Debug, message); }
void log_info(const std::string& message) { log(LogLevel::Info, message); }
void log_warning(const std::string& message) { log(LogLevel::Warning, message); }
void log_error(const std::string& message) { log(LogLevel::Error, message); }

void send_debug_value(const std::string& name, double value) {
    uint8_t payload[256];
    int name_len = std::min(static_cast<int>(name.length()), 240);
    std::memcpy(payload, name.c_str(), name_len + 1);
    std::memcpy(payload + name_len + 1, &value, 8);
    send_message(MessageType::DebugValue, payload, name_len + 9);
}

void send_graph_data(const std::string& series_name, 
                     const std::vector<std::pair<double, double>>& points) {
    // Limited implementation - send first few points
    int max_points = 10;
    int count = std::min(static_cast<int>(points.size()), max_points);
    
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(series_name.length()));
    for (char c : series_name) payload.push_back(c);
    payload.push_back(static_cast<uint8_t>(count));
    
    for (int i = 0; i < count; i++) {
        const uint8_t* xb = reinterpret_cast<const uint8_t*>(&points[i].first);
        const uint8_t* yb = reinterpret_cast<const uint8_t*>(&points[i].second);
        for (int j = 0; j < 8; j++) payload.push_back(xb[j]);
        for (int j = 0; j < 8; j++) payload.push_back(yb[j]);
    }
    
    send_message(MessageType::GraphData, payload.data(), static_cast<int>(payload.size()));
}

// TunableParam implementation
TunableParam::TunableParam(const std::string& name, double default_value,
                           double min_val, double max_val)
    : name_(name), value_(default_value), min_(min_val), max_(max_val) {
    detail::parameters[name] = this;
}

void TunableParam::set(double v) {
    value_ = std::clamp(v, min_, max_);
}

TunableParam& param(const std::string& name, double default_value,
                    double min_val, double max_val) {
    auto it = detail::parameters.find(name);
    if (it != detail::parameters.end()) {
        return *it->second;
    }
    detail::param_storage.push_back(
        std::make_unique<TunableParam>(name, default_value, min_val, max_val));
    return *detail::param_storage.back();
}

double get_param(const std::string& name) {
    auto it = detail::parameters.find(name);
    if (it != detail::parameters.end()) {
        return it->second->get();
    }
    return 0;
}

bool has_param(const std::string& name) {
    return detail::parameters.find(name) != detail::parameters.end();
}

// PIDGains implementation
PIDGains::PIDGains(const std::string& prefix, double p, double i, double d, double f) {
    kP = &param(prefix + ".kP", p, 0, 1000);
    kI = &param(prefix + ".kI", i, 0, 1000);
    kD = &param(prefix + ".kD", d, 0, 1000);
    kF = &param(prefix + ".kF", f, 0, 1000);
}

bool PIDGains::was_updated() const {
    return (kP && kP->was_updated()) || 
           (kI && kI->was_updated()) || 
           (kD && kD->was_updated()) ||
           (kF && kF->was_updated());
}

PIDGains make_pid_gains(const std::string& prefix, double kp, double ki, double kd, double kf) {
    return PIDGains(prefix, kp, ki, kd, kf);
}

// FeedforwardGains implementation
FeedforwardGains::FeedforwardGains(const std::string& prefix, double s, double v, double a, double g) {
    kS = &param(prefix + ".kS", s, 0, 100);
    kV = &param(prefix + ".kV", v, 0, 100);
    kA = &param(prefix + ".kA", a, 0, 100);
    kG = &param(prefix + ".kG", g, 0, 100);
}

double FeedforwardGains::calculate(double velocity, double acceleration, double cos_angle) const {
    double sign = (velocity > 0) ? 1.0 : ((velocity < 0) ? -1.0 : 0.0);
    return s() * sign + v() * velocity + a() * acceleration + g() * cos_angle;
}

FeedforwardGains make_ff_gains(const std::string& prefix, double ks, double kv, double ka, double kg) {
    return FeedforwardGains(prefix, ks, kv, ka, kg);
}

// Callbacks
void on_odometry_reset(std::function<void(double, double, double)> callback) {
    detail::odom_reset_callback = callback;
}

void on_emergency_stop(std::function<void()> callback) {
    detail::emergency_stop_callback = callback;
}

void on_waypoint(std::function<void(double, double, double)> callback) {
    detail::waypoint_callback = callback;
}

void on_run_path(std::function<void(int)> callback) {
    detail::run_path_callback = callback;
}

void on_start_autonomous(std::function<void(int)> callback) {
    detail::start_auton_callback = callback;
}

void on_parameter_changed(std::function<void(const std::string&, double)> callback) {
    detail::param_changed_callback = callback;
}

int bytes_available() {
    if (detail::serial_port) {
        return detail::serial_port->get_read_avail();
    }
    return 0;
}

Stats get_stats() {
    return detail::stats;
}

} // namespace cwb

#endif // CWB_IMPLEMENTATION
