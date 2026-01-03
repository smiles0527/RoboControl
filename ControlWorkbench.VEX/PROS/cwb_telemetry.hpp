/**
 * ControlWorkbench PROS Library
 * 
 * Companion library for real-time communication between
 * VEX V5 robots and ControlWorkbench on PC.
 * 
 * Features:
 * - Bidirectional telemetry
 * - Remote parameter tuning
 * - Odometry visualization
 * - PID state monitoring
 * - Match recording
 * 
 * Usage:
 *   #include "cwb/telemetry.hpp"
 *   
 *   void initialize() {
 *       cwb::init();
 *   }
 *   
 *   void opcontrol() {
 *       while (true) {
 *           cwb::send_odometry(x, y, theta);
 *           cwb::update();  // Process incoming commands
 *           pros::delay(10);
 *       }
 *   }
 */

#pragma once

#include "api.h"
#include <cstdint>
#include <string>
#include <functional>
#include <map>

namespace cwb {

// Protocol constants
constexpr uint8_t PREAMBLE1 = 0xAA;
constexpr uint8_t PREAMBLE2 = 0x55;
constexpr int BAUD_RATE = 115200;
constexpr int MAX_PAYLOAD = 255;

// Message types (must match C# VexMessageType enum)
enum class MessageType : uint8_t {
    Heartbeat = 0x00,
    Acknowledge = 0x01,
    Error = 0x02,
    SystemStatus = 0x03,
    BatteryStatus = 0x04,
    CompetitionStatus = 0x05,
    
    Odometry = 0x10,
    ImuData = 0x11,
    MotorTelemetry = 0x12,
    EncoderData = 0x13,
    RotationSensor = 0x14,
    DistanceSensor = 0x15,
    
    SetParameter = 0x30,
    GetParameter = 0x31,
    ParameterValue = 0x32,
    PidState = 0x34,
    
    PathWaypoint = 0x50,
    PathProgress = 0x52,
    
    LogMessage = 0x60,
    DebugValue = 0x61,
    GraphData = 0x62,
    
    ResetOdometry = 0x73,
    EmergencyStop = 0x7F
};

// Log levels
enum class LogLevel : uint8_t {
    Debug = 0,
    Info = 1,
    Warning = 2,
    Error = 3
};

// ============================================================================
// Core Functions
// ============================================================================

/**
 * Initialize the telemetry system.
 * Call this in initialize().
 * 
 * @param port UART port (1-2, default 1)
 */
void init(int port = 1);

/**
 * Process incoming messages and send heartbeat.
 * Call this periodically in your control loop.
 */
void update();

/**
 * Check if connected to ControlWorkbench.
 */
bool is_connected();

// ============================================================================
// Telemetry Sending
// ============================================================================

/**
 * Send odometry data to ControlWorkbench.
 */
void send_odometry(double x, double y, double theta,
                   double vel_x = 0, double vel_y = 0, double angular_vel = 0);

/**
 * Send IMU data.
 */
void send_imu(double heading, double pitch, double roll,
              double gyro_x, double gyro_y, double gyro_z,
              double accel_x = 0, double accel_y = 0, double accel_z = 0);

/**
 * Send motor telemetry.
 */
void send_motor(uint8_t port, double position, double velocity, 
                double current, double voltage, double temperature,
                double power, double torque);

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
 * Send a log message.
 */
void log(LogLevel level, const std::string& message);
void log_debug(const std::string& message);
void log_info(const std::string& message);
void log_warning(const std::string& message);
void log_error(const std::string& message);

/**
 * Send a named debug value for graphing.
 */
void send_debug_value(const std::string& name, double value);

// ============================================================================
// Parameter Tuning
// ============================================================================

/**
 * Tunable parameter that can be modified from ControlWorkbench.
 */
class TunableParam {
public:
    TunableParam(const std::string& name, double default_value);
    
    double get() const { return value; }
    operator double() const { return value; }
    
    void set(double v) { value = v; }
    
    const std::string& name() const { return param_name; }
    
private:
    std::string param_name;
    double value;
    
    friend void process_set_parameter(const uint8_t* payload, int len);
};

/**
 * Register a tunable parameter.
 * Returns a reference to the parameter for easy access.
 */
TunableParam& param(const std::string& name, double default_value);

/**
 * Get a parameter value by name.
 */
double get_param(const std::string& name);

/**
 * Check if a parameter was updated this frame.
 */
bool param_updated(const std::string& name);

/**
 * PID gains wrapper for easy tuning.
 */
struct PIDGains {
    TunableParam* kP;
    TunableParam* kI;
    TunableParam* kD;
    
    PIDGains(const std::string& prefix, double p, double i, double d);
    
    double p() const { return kP->get(); }
    double i() const { return kI->get(); }
    double d() const { return kD->get(); }
};

/**
 * Create tunable PID gains.
 */
PIDGains make_pid_gains(const std::string& prefix, double kp, double ki, double kd);

// ============================================================================
// Callbacks
// ============================================================================

/**
 * Set callback for odometry reset command.
 */
void on_odometry_reset(std::function<void(double, double, double)> callback);

/**
 * Set callback for emergency stop command.
 */
void on_emergency_stop(std::function<void()> callback);

/**
 * Set callback for path waypoint received.
 */
void on_waypoint(std::function<void(double, double, double)> callback);

// ============================================================================
// Low-level functions
// ============================================================================

/**
 * Send a raw message.
 */
void send_message(MessageType type, const uint8_t* payload, int len);

/**
 * Send a message with no payload.
 */
void send_message(MessageType type);

} // namespace cwb


// ============================================================================
// IMPLEMENTATION
// ============================================================================

#ifdef CWB_IMPLEMENTATION

#include <cstring>
#include <cmath>

namespace cwb {

static pros::Serial* serial_port = nullptr;
static uint32_t last_heartbeat = 0;
static bool connected = false;
static std::map<std::string, TunableParam*> parameters;
static std::map<std::string, bool> param_updated_flags;

// Callbacks
static std::function<void(double, double, double)> odom_reset_callback = nullptr;
static std::function<void()> emergency_stop_callback = nullptr;
static std::function<void(double, double, double)> waypoint_callback = nullptr;

// Message buffer
static uint8_t tx_buffer[MAX_PAYLOAD + 10];
static uint8_t rx_buffer[MAX_PAYLOAD + 10];
static int rx_index = 0;

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

static uint8_t compute_checksum(MessageType type, const uint8_t* payload, int len) {
    uint8_t cs = static_cast<uint8_t>(type) ^ static_cast<uint8_t>(len);
    for (int i = 0; i < len; i++) {
        cs ^= payload[i];
    }
    return cs;
}

void send_message(MessageType type, const uint8_t* payload, int len) {
    if (!serial_port) return;
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
    
    tx_buffer[0] = PREAMBLE1;
    tx_buffer[1] = PREAMBLE2;
    tx_buffer[2] = static_cast<uint8_t>(type);
    tx_buffer[3] = static_cast<uint8_t>(len);
    if (len > 0 && payload) {
        memcpy(tx_buffer + 4, payload, len);
    }
    tx_buffer[4 + len] = compute_checksum(type, payload, len);
    
    serial_port->write(tx_buffer, 5 + len);
}

void send_message(MessageType type) {
    send_message(type, nullptr, 0);
}

void process_set_parameter(const uint8_t* payload, int len) {
    // Find null terminator
    int null_pos = -1;
    for (int i = 0; i < len; i++) {
        if (payload[i] == 0) {
            null_pos = i;
            break;
        }
    }
    if (null_pos < 0 || null_pos + 9 > len) return;
    
    std::string name(reinterpret_cast<const char*>(payload), null_pos);
    double value;
    memcpy(&value, payload + null_pos + 1, sizeof(double));
    
    auto it = parameters.find(name);
    if (it != parameters.end()) {
        it->second->set(value);
        param_updated_flags[name] = true;
        
        // Send acknowledgment
        send_message(MessageType::Acknowledge);
    }
}

void process_message(MessageType type, const uint8_t* payload, int len) {
    switch (type) {
        case MessageType::Heartbeat:
            last_heartbeat = pros::millis();
            connected = true;
            send_message(MessageType::Acknowledge);
            break;
            
        case MessageType::SetParameter:
            process_set_parameter(payload, len);
            break;
            
        case MessageType::GetParameter: {
            std::string name(reinterpret_cast<const char*>(payload));
            auto it = parameters.find(name);
            if (it != parameters.end()) {
                uint8_t resp[256];
                int name_len = name.length();
                memcpy(resp, name.c_str(), name_len + 1);
                double val = it->second->get();
                memcpy(resp + name_len + 1, &val, sizeof(double));
                send_message(MessageType::ParameterValue, resp, name_len + 9);
            }
            break;
        }
        
        case MessageType::ResetOdometry:
            if (len >= 24 && odom_reset_callback) {
                double x, y, theta;
                memcpy(&x, payload, sizeof(double));
                memcpy(&y, payload + 8, sizeof(double));
                memcpy(&theta, payload + 16, sizeof(double));
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
                double x, y, vel;
                memcpy(&x, payload, sizeof(double));
                memcpy(&y, payload + 8, sizeof(double));
                memcpy(&vel, payload + 16, sizeof(double));
                waypoint_callback(x, y, vel);
            }
            break;
            
        default:
            break;
    }
}

void process_byte(uint8_t b) {
    switch (parse_state) {
        case ParseState::WaitPreamble1:
            if (b == PREAMBLE1) parse_state = ParseState::WaitPreamble2;
            break;
            
        case ParseState::WaitPreamble2:
            if (b == PREAMBLE2) parse_state = ParseState::WaitType;
            else parse_state = ParseState::WaitPreamble1;
            break;
            
        case ParseState::WaitType:
            current_type = static_cast<MessageType>(b);
            parse_state = ParseState::WaitLength;
            break;
            
        case ParseState::WaitLength:
            current_length = b;
            payload_index = 0;
            if (current_length == 0) parse_state = ParseState::WaitChecksum;
            else parse_state = ParseState::ReadPayload;
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
            }
            parse_state = ParseState::WaitPreamble1;
            break;
        }
    }
}

void init(int port) {
    serial_port = new pros::Serial(port, BAUD_RATE);
    last_heartbeat = 0;
    connected = false;
}

void update() {
    if (!serial_port) return;
    
    // Clear updated flags
    for (auto& pair : param_updated_flags) {
        pair.second = false;
    }
    
    // Process incoming data
    while (serial_port->get_read_avail() > 0) {
        uint8_t byte;
        if (serial_port->read(&byte, 1) == 1) {
            process_byte(byte);
        }
    }
    
    // Send heartbeat periodically
    static uint32_t last_sent = 0;
    if (pros::millis() - last_sent > 500) {
        send_message(MessageType::Heartbeat);
        last_sent = pros::millis();
    }
    
    // Check connection timeout
    if (pros::millis() - last_heartbeat > 2000) {
        connected = false;
    }
}

bool is_connected() {
    return connected;
}

void send_odometry(double x, double y, double theta,
                   double vel_x, double vel_y, double angular_vel) {
    uint8_t payload[52];
    uint32_t ts = pros::millis();
    memcpy(payload, &x, 8);
    memcpy(payload + 8, &y, 8);
    memcpy(payload + 16, &theta, 8);
    memcpy(payload + 24, &vel_x, 8);
    memcpy(payload + 32, &vel_y, 8);
    memcpy(payload + 40, &angular_vel, 8);
    memcpy(payload + 48, &ts, 4);
    send_message(MessageType::Odometry, payload, 52);
}

void send_imu(double heading, double pitch, double roll,
              double gyro_x, double gyro_y, double gyro_z,
              double accel_x, double accel_y, double accel_z) {
    uint8_t payload[72];
    memcpy(payload, &heading, 8);
    memcpy(payload + 8, &pitch, 8);
    memcpy(payload + 16, &roll, 8);
    memcpy(payload + 24, &gyro_x, 8);
    memcpy(payload + 32, &gyro_y, 8);
    memcpy(payload + 40, &gyro_z, 8);
    memcpy(payload + 48, &accel_x, 8);
    memcpy(payload + 56, &accel_y, 8);
    memcpy(payload + 64, &accel_z, 8);
    send_message(MessageType::ImuData, payload, 72);
}

void send_motor(uint8_t port, double position, double velocity, 
                double current, double voltage, double temperature,
                double power, double torque) {
    uint8_t payload[67];
    payload[0] = port;
    memcpy(payload + 1, &position, 8);
    memcpy(payload + 9, &velocity, 8);
    memcpy(payload + 17, &current, 8);
    memcpy(payload + 25, &voltage, 8);
    memcpy(payload + 33, &temperature, 8);
    memcpy(payload + 41, &power, 8);
    memcpy(payload + 49, &torque, 8);
    // target_pos, target_vel, flags filled with zeros
    memset(payload + 57, 0, 10);
    send_message(MessageType::MotorTelemetry, payload, 67);
}

void send_pid_state(uint8_t controller_id, 
                    double setpoint, double measurement,
                    double error, double integral, double derivative,
                    double output, double kp, double ki, double kd) {
    uint8_t payload[73];
    payload[0] = controller_id;
    memcpy(payload + 1, &setpoint, 8);
    memcpy(payload + 9, &measurement, 8);
    memcpy(payload + 17, &error, 8);
    memcpy(payload + 25, &integral, 8);
    memcpy(payload + 33, &derivative, 8);
    memcpy(payload + 41, &output, 8);
    memcpy(payload + 49, &kp, 8);
    memcpy(payload + 57, &ki, 8);
    memcpy(payload + 65, &kd, 8);
    send_message(MessageType::PidState, payload, 73);
}

void send_battery(double voltage, double current, double capacity, double temperature) {
    uint8_t payload[32];
    memcpy(payload, &voltage, 8);
    memcpy(payload + 8, &current, 8);
    memcpy(payload + 16, &capacity, 8);
    memcpy(payload + 24, &temperature, 8);
    send_message(MessageType::BatteryStatus, payload, 32);
}

void log(LogLevel level, const std::string& message) {
    uint8_t payload[256];
    payload[0] = static_cast<uint8_t>(level);
    uint32_t ts = pros::millis();
    memcpy(payload + 1, &ts, 4);
    int len = std::min((int)message.length(), 250);
    memcpy(payload + 5, message.c_str(), len);
    send_message(MessageType::LogMessage, payload, 5 + len);
}

void log_debug(const std::string& message) { log(LogLevel::Debug, message); }
void log_info(const std::string& message) { log(LogLevel::Info, message); }
void log_warning(const std::string& message) { log(LogLevel::Warning, message); }
void log_error(const std::string& message) { log(LogLevel::Error, message); }

void send_debug_value(const std::string& name, double value) {
    uint8_t payload[256];
    int name_len = std::min((int)name.length(), 240);
    memcpy(payload, name.c_str(), name_len + 1);
    memcpy(payload + name_len + 1, &value, 8);
    send_message(MessageType::DebugValue, payload, name_len + 9);
}

TunableParam::TunableParam(const std::string& name, double default_value)
    : param_name(name), value(default_value) {
    parameters[name] = this;
    param_updated_flags[name] = false;
}

TunableParam& param(const std::string& name, double default_value) {
    auto it = parameters.find(name);
    if (it != parameters.end()) {
        return *it->second;
    }
    static std::vector<std::unique_ptr<TunableParam>> storage;
    storage.push_back(std::make_unique<TunableParam>(name, default_value));
    return *storage.back();
}

double get_param(const std::string& name) {
    auto it = parameters.find(name);
    if (it != parameters.end()) {
        return it->second->get();
    }
    return 0;
}

bool param_updated(const std::string& name) {
    auto it = param_updated_flags.find(name);
    if (it != param_updated_flags.end()) {
        return it->second;
    }
    return false;
}

PIDGains::PIDGains(const std::string& prefix, double p, double i, double d) {
    kP = &param(prefix + ".kP", p);
    kI = &param(prefix + ".kI", i);
    kD = &param(prefix + ".kD", d);
}

PIDGains make_pid_gains(const std::string& prefix, double kp, double ki, double kd) {
    return PIDGains(prefix, kp, ki, kd);
}

void on_odometry_reset(std::function<void(double, double, double)> callback) {
    odom_reset_callback = callback;
}

void on_emergency_stop(std::function<void()> callback) {
    emergency_stop_callback = callback;
}

void on_waypoint(std::function<void(double, double, double)> callback) {
    waypoint_callback = callback;
}

} // namespace cwb

#endif // CWB_IMPLEMENTATION
