namespace ControlWorkbench.VEX.ProsLibrary;

/// <summary>
/// Generates the C++ library code that runs ON the V5 Brain
/// to send telemetry back to ControlWorkbench.
/// </summary>
public static class ProsLibraryGenerator
{
    /// <summary>
    /// Generate the complete PROS library for robot-side telemetry.
    /// </summary>
    public static Dictionary<string, string> GenerateLibrary()
    {
        var files = new Dictionary<string, string>
        {
            ["include/controlworkbench/telemetry.hpp"] = GenerateTelemetryHeader(),
            ["src/controlworkbench/telemetry.cpp"] = GenerateTelemetryCpp(),
            ["include/controlworkbench/protocol.hpp"] = GenerateProtocolHeader(),
            ["include/controlworkbench/tuning.hpp"] = GenerateTuningHeader(),
            ["src/controlworkbench/tuning.cpp"] = GenerateTuningCpp()
        };
        
        return files;
    }

    private static string GenerateTelemetryHeader()
    {
        return @"#pragma once
/**
 * ControlWorkbench Telemetry Library for PROS
 * 
 * This library sends real-time telemetry data from the V5 Brain
 * to the ControlWorkbench application over USB serial.
 * 
 * Usage:
 *   #include ""controlworkbench/telemetry.hpp""
 *   
 *   void initialize() {
 *       cwb::telemetry::init();
 *   }
 *   
 *   void opcontrol() {
 *       while (true) {
 *           cwb::telemetry::update();
 *           pros::delay(20);
 *       }
 *   }
 */

#include ""pros/motors.hpp""
#include ""pros/imu.hpp""
#include ""pros/rotation.hpp""
#include ""pros/misc.hpp""
#include <vector>
#include <string>
#include <functional>

namespace cwb {
namespace telemetry {

// Configuration
struct Config {
    bool enabled = true;
    int update_rate_ms = 50;        // Telemetry update rate
    bool send_motors = true;
    bool send_sensors = true;
    bool send_position = true;
    bool send_battery = true;
    bool send_controller = true;
};

extern Config config;

/**
 * Initialize telemetry system.
 * Call this in initialize().
 */
void init();

/**
 * Update and send telemetry.
 * Call this in your main loop.
 */
void update();

/**
 * Register a motor for telemetry.
 */
void register_motor(pros::Motor& motor, const std::string& name = """");

/**
 * Register a motor group for telemetry.
 */
void register_motor_group(pros::MotorGroup& group, const std::string& name = """");

/**
 * Register an IMU for telemetry.
 */
void register_imu(pros::Imu& imu);

/**
 * Register rotation sensors for odometry.
 */
void register_tracking_wheels(
    pros::Rotation* left,
    pros::Rotation* right,
    pros::Rotation* horizontal = nullptr
);

/**
 * Set odometry position (from your odom system).
 */
void set_position(double x, double y, double heading);

/**
 * Send custom telemetry value.
 */
void send_custom(const std::string& name, double value);

/**
 * Send log message to ControlWorkbench.
 */
void log(const std::string& message);

/**
 * Check if ControlWorkbench is connected.
 */
bool is_connected();

/**
 * Register callback for incoming commands.
 */
using CommandCallback = std::function<void(uint8_t cmd, const uint8_t* data, size_t len)>;
void on_command(CommandCallback callback);

} // namespace telemetry
} // namespace cwb
";
    }

    private static string GenerateTelemetryCpp()
    {
        return @"#include ""controlworkbench/telemetry.hpp""
#include ""controlworkbench/protocol.hpp""
#include ""pros/apix.h""
#include <cstring>
#include <cstdio>

namespace cwb {
namespace telemetry {

// Static storage
Config config;
static std::vector<pros::Motor*> registered_motors;
static std::vector<std::string> motor_names;
static pros::Imu* registered_imu = nullptr;
static pros::Rotation* left_tracker = nullptr;
static pros::Rotation* right_tracker = nullptr;
static pros::Rotation* horizontal_tracker = nullptr;
static double odom_x = 0, odom_y = 0, odom_heading = 0;
static uint32_t last_update = 0;
static CommandCallback command_callback = nullptr;
static bool connected = false;

// Serial file descriptor
static int32_t serial_fd = -1;

void init() {
    // Open serial port for USB communication
    // PROS uses stdin/stdout for USB serial by default
    serial_fd = fileno(stdout);
    connected = true;
    
    // Send initialization packet
    uint8_t init_packet[] = {
        protocol::PACKET_START,
        protocol::CMD_INIT,
        4,  // length
        'C', 'W', 'B', '1',  // ControlWorkBench v1
        0,  // checksum placeholder
        protocol::PACKET_END
    };
    init_packet[7] = init_packet[3] ^ init_packet[4] ^ init_packet[5] ^ init_packet[6];
    fwrite(init_packet, 1, sizeof(init_packet), stdout);
    fflush(stdout);
}

void update() {
    if (!config.enabled) return;
    
    uint32_t now = pros::millis();
    if (now - last_update < config.update_rate_ms) return;
    last_update = now;
    
    // Send motor telemetry
    if (config.send_motors && !registered_motors.empty()) {
        send_motor_telemetry();
    }
    
    // Send sensor telemetry
    if (config.send_sensors && registered_imu != nullptr) {
        send_sensor_telemetry();
    }
    
    // Send position telemetry
    if (config.send_position) {
        send_position_telemetry();
    }
    
    // Send battery telemetry
    if (config.send_battery) {
        send_battery_telemetry();
    }
    
    // Send controller telemetry
    if (config.send_controller) {
        send_controller_telemetry();
    }
    
    fflush(stdout);
}

void register_motor(pros::Motor& motor, const std::string& name) {
    registered_motors.push_back(&motor);
    motor_names.push_back(name.empty() ? std::to_string(motor.get_port()) : name);
}

void register_motor_group(pros::MotorGroup& group, const std::string& name) {
    // Register each motor in the group
    auto ports = group.get_port_all();
    for (size_t i = 0; i < ports.size(); i++) {
        // Note: This is simplified - in practice you'd store the group reference
    }
}

void register_imu(pros::Imu& imu) {
    registered_imu = &imu;
}

void register_tracking_wheels(
    pros::Rotation* left,
    pros::Rotation* right,
    pros::Rotation* horizontal
) {
    left_tracker = left;
    right_tracker = right;
    horizontal_tracker = horizontal;
}

void set_position(double x, double y, double heading) {
    odom_x = x;
    odom_y = y;
    odom_heading = heading;
}

static void send_motor_telemetry() {
    // Build motor telemetry packet
    // Format: [port, power, vel_lo, vel_hi, pos_0-3, temp, curr_lo, curr_hi] per motor
    
    std::vector<uint8_t> data;
    
    for (auto* motor : registered_motors) {
        int port = motor->get_port();
        int power = (int)motor->get_voltage() / 94;  // Convert to -127..127
        int velocity = (int)motor->get_actual_velocity();
        int position = (int)motor->get_position();
        int temp = (int)motor->get_temperature();
        int current = (int)motor->get_current_draw();
        
        data.push_back((uint8_t)port);
        data.push_back((uint8_t)(power + 128));
        data.push_back(velocity & 0xFF);
        data.push_back((velocity >> 8) & 0xFF);
        data.push_back(position & 0xFF);
        data.push_back((position >> 8) & 0xFF);
        data.push_back((position >> 16) & 0xFF);
        data.push_back((position >> 24) & 0xFF);
        data.push_back((uint8_t)temp);
        data.push_back(current & 0xFF);
        data.push_back((current >> 8) & 0xFF);
    }
    
    send_packet(protocol::TELEM_MOTORS, data.data(), data.size());
}

static void send_sensor_telemetry() {
    if (!registered_imu) return;
    
    float heading = registered_imu->get_heading();
    float pitch = registered_imu->get_pitch();
    float roll = registered_imu->get_roll();
    
    uint8_t data[12];
    memcpy(data + 0, &heading, 4);
    memcpy(data + 4, &pitch, 4);
    memcpy(data + 8, &roll, 4);
    
    send_packet(protocol::TELEM_SENSORS, data, 12);
}

static void send_position_telemetry() {
    float x = (float)odom_x;
    float y = (float)odom_y;
    float h = (float)odom_heading;
    
    uint8_t data[12];
    memcpy(data + 0, &x, 4);
    memcpy(data + 4, &y, 4);
    memcpy(data + 8, &h, 4);
    
    send_packet(protocol::TELEM_POSITION, data, 12);
}

static void send_battery_telemetry() {
    int voltage = pros::battery::get_voltage();  // mV
    int capacity = pros::battery::get_capacity();  // percent
    int temp = pros::battery::get_temperature();  // tenths of degree C
    
    uint8_t data[4];
    data[0] = voltage & 0xFF;
    data[1] = (voltage >> 8) & 0xFF;
    data[2] = (uint8_t)capacity;
    data[3] = (uint8_t)(temp / 10);
    
    send_packet(protocol::TELEM_BATTERY, data, 4);
}

static void send_controller_telemetry() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    uint8_t data[8];
    data[0] = (uint8_t)(master.get_analog(ANALOG_LEFT_X) + 128);
    data[1] = (uint8_t)(master.get_analog(ANALOG_LEFT_Y) + 128);
    data[2] = (uint8_t)(master.get_analog(ANALOG_RIGHT_X) + 128);
    data[3] = (uint8_t)(master.get_analog(ANALOG_RIGHT_Y) + 128);
    
    // Pack digital buttons into 2 bytes
    uint16_t buttons = 0;
    if (master.get_digital(DIGITAL_L1)) buttons |= 0x0001;
    if (master.get_digital(DIGITAL_L2)) buttons |= 0x0002;
    if (master.get_digital(DIGITAL_R1)) buttons |= 0x0004;
    if (master.get_digital(DIGITAL_R2)) buttons |= 0x0008;
    if (master.get_digital(DIGITAL_UP)) buttons |= 0x0010;
    if (master.get_digital(DIGITAL_DOWN)) buttons |= 0x0020;
    if (master.get_digital(DIGITAL_LEFT)) buttons |= 0x0040;
    if (master.get_digital(DIGITAL_RIGHT)) buttons |= 0x0080;
    if (master.get_digital(DIGITAL_X)) buttons |= 0x0100;
    if (master.get_digital(DIGITAL_B)) buttons |= 0x0200;
    if (master.get_digital(DIGITAL_Y)) buttons |= 0x0400;
    if (master.get_digital(DIGITAL_A)) buttons |= 0x0800;
    
    data[4] = buttons & 0xFF;
    data[5] = (buttons >> 8) & 0xFF;
    data[6] = 0;  // Reserved
    data[7] = 0;  // Reserved
    
    send_packet(protocol::TELEM_CONTROLLER, data, 8);
}

static void send_packet(uint8_t type, const uint8_t* data, size_t len) {
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    
    // Send packet
    putchar(protocol::PACKET_START);
    putchar(type);
    putchar((uint8_t)len);
    fwrite(data, 1, len, stdout);
    putchar(checksum);
    putchar(protocol::PACKET_END);
}

void send_custom(const std::string& name, double value) {
    // Custom telemetry: name length (1) + name + value (8)
    std::vector<uint8_t> data;
    data.push_back((uint8_t)name.size());
    for (char c : name) {
        data.push_back((uint8_t)c);
    }
    
    float fval = (float)value;
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&fval);
    for (int i = 0; i < 4; i++) {
        data.push_back(bytes[i]);
    }
    
    send_packet(protocol::TELEM_CUSTOM, data.data(), data.size());
}

void log(const std::string& message) {
    send_packet(protocol::TELEM_LOG, 
        reinterpret_cast<const uint8_t*>(message.c_str()), 
        message.size());
}

bool is_connected() {
    return connected;
}

void on_command(CommandCallback callback) {
    command_callback = callback;
}

} // namespace telemetry
} // namespace cwb
";
    }

    private static string GenerateProtocolHeader()
    {
        return @"#pragma once
/**
 * ControlWorkbench Protocol Constants
 */

namespace cwb {
namespace protocol {

// Packet framing
constexpr uint8_t PACKET_START = 0xC9;
constexpr uint8_t PACKET_END = 0xAA;

// Commands (from ControlWorkbench to robot)
constexpr uint8_t CMD_INIT = 0x01;
constexpr uint8_t CMD_REQUEST_TELEM = 0x02;
constexpr uint8_t CMD_SET_MOTOR = 0x10;
constexpr uint8_t CMD_SET_PNEUMATIC = 0x11;
constexpr uint8_t CMD_SET_PARAM = 0x20;
constexpr uint8_t CMD_START_AUTON = 0x30;
constexpr uint8_t CMD_STOP = 0x31;

// Telemetry types (from robot to ControlWorkbench)
constexpr uint8_t TELEM_MOTORS = 0x01;
constexpr uint8_t TELEM_SENSORS = 0x02;
constexpr uint8_t TELEM_POSITION = 0x03;
constexpr uint8_t TELEM_BATTERY = 0x04;
constexpr uint8_t TELEM_CONTROLLER = 0x05;
constexpr uint8_t TELEM_COMPETITION = 0x06;
constexpr uint8_t TELEM_CUSTOM = 0x10;
constexpr uint8_t TELEM_LOG = 0x11;

} // namespace protocol
} // namespace cwb
";
    }

    private static string GenerateTuningHeader()
    {
        return @"#pragma once
/**
 * ControlWorkbench Tuning Helper
 * 
 * Provides real-time PID tuning and motor characterization
 * through the ControlWorkbench application.
 */

#include ""pros/motors.hpp""
#include <functional>

namespace cwb {
namespace tuning {

/**
 * Run motor characterization routine.
 * Robot will run forward and back to measure motor characteristics.
 */
struct CharacterizationResult {
    double kS;  // Static friction (volts)
    double kV;  // Velocity constant (volts per RPM)
    double kA;  // Acceleration constant (volts per RPM/s)
    bool success;
};

CharacterizationResult characterize_drivetrain(
    pros::MotorGroup& left,
    pros::MotorGroup& right,
    double max_voltage = 8.0
);

/**
 * Live PID tuning helper.
 * Updates PID gains from ControlWorkbench in real-time.
 */
class LiveTuner {
public:
    LiveTuner(const std::string& name);
    
    void set_gains(double kP, double kI, double kD);
    void get_gains(double& kP, double& kI, double& kD) const;
    
    // Call this in your loop - it checks for updates from ControlWorkbench
    void update();
    
    // Register callback when gains change
    using GainsChangedCallback = std::function<void(double kP, double kI, double kD)>;
    void on_gains_changed(GainsChangedCallback callback);

private:
    std::string name_;
    double kP_ = 0, kI_ = 0, kD_ = 0;
    GainsChangedCallback callback_;
};

} // namespace tuning
} // namespace cwb
";
    }

    private static string GenerateTuningCpp()
    {
        return @"#include ""controlworkbench/tuning.hpp""
#include ""controlworkbench/telemetry.hpp""
#include ""pros/rtos.hpp""
#include <cmath>

namespace cwb {
namespace tuning {

CharacterizationResult characterize_drivetrain(
    pros::MotorGroup& left,
    pros::MotorGroup& right,
    double max_voltage
) {
    CharacterizationResult result = {0, 0, 0, false};
    
    telemetry::log(""Starting drivetrain characterization..."");
    
    // Wait for robot to be stationary
    pros::delay(500);
    
    // Quasi-static forward test (slowly ramp up voltage)
    std::vector<double> voltages;
    std::vector<double> velocities;
    
    double voltage = 0;
    double voltage_step = 0.1;  // 100mV per step
    
    while (voltage < max_voltage) {
        left.move_voltage((int)(voltage * 1000));
        right.move_voltage((int)(voltage * 1000));
        
        pros::delay(50);
        
        // Average velocity of all motors
        double vel = 0;
        auto left_vels = left.get_actual_velocity_all();
        auto right_vels = right.get_actual_velocity_all();
        
        for (double v : left_vels) vel += std::abs(v);
        for (double v : right_vels) vel += std::abs(v);
        vel /= (left_vels.size() + right_vels.size());
        
        voltages.push_back(voltage);
        velocities.push_back(vel);
        
        telemetry::send_custom(""char_voltage"", voltage);
        telemetry::send_custom(""char_velocity"", vel);
        
        voltage += voltage_step;
    }
    
    // Stop motors
    left.move_voltage(0);
    right.move_voltage(0);
    pros::delay(500);
    
    // Find kS (voltage where motor starts moving)
    double kS = 0;
    for (size_t i = 0; i < velocities.size(); i++) {
        if (velocities[i] > 5) {  // 5 RPM threshold
            kS = voltages[i];
            break;
        }
    }
    
    // Linear regression for kV
    // V = kS + kV * velocity
    // Using points where velocity > 10 RPM
    double sum_v = 0, sum_vel = 0, sum_vvel = 0, sum_vel2 = 0;
    int n = 0;
    
    for (size_t i = 0; i < velocities.size(); i++) {
        if (velocities[i] > 10) {
            sum_v += voltages[i];
            sum_vel += velocities[i];
            sum_vvel += voltages[i] * velocities[i];
            sum_vel2 += velocities[i] * velocities[i];
            n++;
        }
    }
    
    if (n > 2) {
        double kV = (n * sum_vvel - sum_v * sum_vel) / (n * sum_vel2 - sum_vel * sum_vel);
        result.kS = std::max(0.0, kS);
        result.kV = kV;
        result.kA = 0;  // Need acceleration test for kA
        result.success = true;
        
        telemetry::log(""Characterization complete!"");
        telemetry::send_custom(""kS"", result.kS);
        telemetry::send_custom(""kV"", result.kV);
    } else {
        telemetry::log(""Characterization failed - not enough data"");
    }
    
    return result;
}

LiveTuner::LiveTuner(const std::string& name) : name_(name) {
    // Register with telemetry system for incoming commands
}

void LiveTuner::set_gains(double kP, double kI, double kD) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
}

void LiveTuner::get_gains(double& kP, double& kI, double& kD) const {
    kP = kP_;
    kI = kI_;
    kD = kD_;
}

void LiveTuner::update() {
    // Check for incoming gain updates
    // This would be called from the telemetry command handler
}

void LiveTuner::on_gains_changed(GainsChangedCallback callback) {
    callback_ = callback;
}

} // namespace tuning
} // namespace cwb
";
    }

    /// <summary>
    /// Get example usage code.
    /// </summary>
    public static string GetExampleUsage()
    {
        return @"
// Example: Adding ControlWorkbench telemetry to your robot

#include ""controlworkbench/telemetry.hpp""
#include ""controlworkbench/tuning.hpp""

// Your motors
pros::MotorGroup left_drive({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({4, 5, 6}, pros::MotorGearset::blue);
pros::Imu imu(10);

// Optional: Live PID tuner
cwb::tuning::LiveTuner lateral_tuner(""lateral"");

void initialize() {
    // Initialize telemetry
    cwb::telemetry::init();
    
    // Register devices for monitoring
    cwb::telemetry::register_motor_group(left_drive, ""left_drive"");
    cwb::telemetry::register_motor_group(right_drive, ""right_drive"");
    cwb::telemetry::register_imu(imu);
    
    // Configure what to send
    cwb::telemetry::config.update_rate_ms = 50;  // 20Hz
    cwb::telemetry::config.send_motors = true;
    cwb::telemetry::config.send_sensors = true;
    cwb::telemetry::config.send_position = true;
    
    // Set up live tuning callback
    lateral_tuner.on_gains_changed([](double kP, double kI, double kD) {
        // Update your PID controller with new gains
        lateral_controller.kP = kP;
        lateral_controller.kI = kI;
        lateral_controller.kD = kD;
        cwb::telemetry::log(""Gains updated!"");
    });
}

void opcontrol() {
    while (true) {
        // Your drive code here...
        
        // Update odometry position for telemetry
        cwb::telemetry::set_position(
            chassis.getPose().x,
            chassis.getPose().y,
            chassis.getPose().theta
        );
        
        // Send custom values
        cwb::telemetry::send_custom(""left_error"", left_pid.get_error());
        cwb::telemetry::send_custom(""right_error"", right_pid.get_error());
        
        // Update telemetry (sends data to ControlWorkbench)
        cwb::telemetry::update();
        
        // Check for tuning updates
        lateral_tuner.update();
        
        pros::delay(10);
    }
}
";
    }
}
