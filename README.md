<div align="center">

# ??? ControlWorkbench

**Professional Control Systems Engineering Platform**

[![.NET Desktop](https://github.com/smiles0527/RoboControl/actions/workflows/dotnet-desktop.yml/badge.svg)](https://github.com/smiles0527/RoboControl/actions)
[![.NET 8](https://img.shields.io/badge/.NET-8.0-512BD4?logo=dotnet)](https://dotnet.microsoft.com/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows-0078D6?logo=windows)](https://www.microsoft.com/windows)

*A unified platform for VEX V5 Robotics and Drone/UAV control systems development*

![Control Workbench](https://img.shields.io/badge/Version-2.0-00D9FF?style=for-the-badge)

</div>

---

## ?? Overview

ControlWorkbench is a comprehensive desktop application for robotics and control systems engineering. It provides a modern, cyberpunk-themed interface for:

- **VEX V5 Robotics** - Path planning, PID tuning, motor health monitoring, and live telemetry
- **Drone/UAV Systems** - Mission planning, flight simulation, MAVLink integration, and fleet management

Built with WPF on .NET 8, featuring real-time visualization with ScottPlot.

---

## ? Features

### ?? VEX V5 Robotics

| Feature | Description |
|---------|-------------|
| **Path Planner** | Visual Pure Pursuit path creation with LemLib/EZ-Template export |
| **Live Robot View** | Real-time odometry visualization and robot tracking |
| **PID Tuner** | Interactive gain tuning with live step response graphs |
| **Motor Health** | Temperature, current draw, and efficiency monitoring |
| **Robot Configuration** | Drivetrain setup, tracking wheel configuration |
| **Telemetry Dashboard** | Real-time data streaming and visualization |
| **Code Generation** | Export to PROS C++ with complete autonomous routines |
| **Robot Simulation** | Advanced physics-based robot simulation |
| **Skills Optimizer** | Autonomous skills route optimization |

### ?? Drone/UAV Platform

| Feature | Description |
|---------|-------------|
| **Mission Planner** | Visual waypoint creation and mission design |
| **Drone Configuration** | Frame type, PID gains, flight controller setup |
| **Live Telemetry** | Real-time attitude, GPS, battery monitoring |
| **Flight Simulation** | High-fidelity multirotor physics simulation |
| **MAVLink Integration** | ArduPilot and PX4 protocol support |
| **Fleet Management** | Multi-vehicle coordination and scheduling |
| **Survey Patterns** | Automated survey pattern generation |
| **Safety Monitoring** | Geofencing, battery alerts, fail-safes |
| **Digital Twin** | Real-time digital twin synchronization |
| **SLAM/VIO** | Visual-inertial odometry support |

### ?? Control Systems & Math

| Feature | Description |
|---------|-------------|
| **PID Controller** | Classic PID with anti-windup, derivative filtering |
| **PID Variants** | PI-D, I-PD, cascaded PID, gain scheduling |
| **Model Predictive Control** | MPC with constraints and receding horizon |
| **LQR Design** | Linear Quadratic Regulator synthesis |
| **Adaptive Control** | Recursive least squares parameter estimation |
| **Sliding Mode Control** | Robust control with boundary layer |
| **Fuzzy Logic Control** | Rule-based fuzzy inference systems |
| **Neural Network Control** | Multi-layer perceptron controllers |
| **Kalman Filters** | EKF, UKF, complementary filters |
| **Trajectory Generation** | Minimum-jerk, minimum-snap trajectories |
| **Frequency Analysis** | Bode plots, stability margins |
| **Sensor Calibration** | IMU, magnetometer, multi-sensor fusion |

### ?? Tools & Utilities

| Feature | Description |
|---------|-------------|
| **Unit Converter** | Comprehensive robotics unit conversions |
| **Math Workbench** | Matrix operations, kinematics calculations |
| **Connection Manager** | Serial port management, auto-detection |
| **Data Logging** | CSV export, match recording, replay |
| **Step Response Analyzer** | Overshoot, settling time, rise time metrics |

---

## ??? Architecture

```
ControlWorkbench/
??? ControlWorkbench.App        # WPF Desktop Application
?   ??? Views/                  # XAML UI components
?   ??? ViewModels/             # MVVM view models
?   ??? App.xaml                # Theme & styles
??? ControlWorkbench.Core       # Core abstractions & types
?   ??? Units/                  # Physical units (Angle, etc.)
?   ??? Sensors/                # Sensor type definitions
?   ??? Collections/            # Ring buffers, message queues
??? ControlWorkbench.Math       # Control theory implementations
?   ??? Control/                # PID, MPC, LQR, adaptive
?   ??? Filters/                # Kalman, complementary
?   ??? Trajectory/             # Motion planning
?   ??? Kinematics/             # Robot kinematics
??? ControlWorkbench.VEX        # VEX V5 specific
?   ??? PathPlanning/           # Pure Pursuit, path generation
?   ??? Simulation/             # Physics engine
?   ??? CodeGen/                # PROS code generator
?   ??? Communication/          # V5 serial protocol
?   ??? PROS/                   # Header-only C++ library
??? ControlWorkbench.Drone      # Drone/UAV specific
?   ??? Mission/                # Mission planning
?   ??? Simulation/             # Multirotor physics
?   ??? Fleet/                  # Fleet management
?   ??? Hardware/               # MAVLink connections
?   ??? SLAM/                   # Visual-inertial odometry
?   ??? Safety/                 # Geofencing, monitoring
??? ControlWorkbench.Protocol   # Communication protocols
??? ControlWorkbench.Transport  # Serial, network transports
```

---

## ?? Installation

### Requirements

- Windows 10/11
- .NET 8.0 Runtime
- VEX V5 Brain (for robot features) or MAVLink-compatible flight controller

### Quick Start

1. Clone the repository:
   ```bash
   git clone https://github.com/smiles0527/RoboControl.git
   cd RoboControl
   ```

2. Build and run:
   ```bash
   dotnet build
   dotnet run --project ControlWorkbench.App
   ```

Or open `ControlWorkbench.sln` in Visual Studio 2022.

---

## ?? Usage

### Getting Started

1. **Launch the application** - Select your platform (VEX V5 or Drone)
2. **Connect your device** - Use the connection panel to select COM port
3. **Start building** - Use the tabbed interface to access features

### VEX V5 Quick Start

1. **Path Planner** - Click to place waypoints on the field, adjust speeds
2. **Export Code** - Generate PROS C++ code for your autonomous routine
3. **PID Tuner** - Connect to robot, tune gains with live feedback
4. **Motor Health** - Monitor motor temperatures during matches

### Drone Quick Start

1. **Mission Planner** - Create waypoint missions with altitude profiles
2. **Configure** - Set up frame type, PID gains, flight modes
3. **Simulate** - Test missions in the physics simulator
4. **Connect** - Link to flight controller via MAVLink

---

## ?? VEX PROS Library

ControlWorkbench includes a complete header-only C++ library for PROS:

```cpp
#define CWB_IMPLEMENTATION
#include "cwb/cwb.hpp"

cwb::Chassis chassis(left_motors, right_motors, imu);
chassis.move_to_point(24, 24);
chassis.follow_path(path);
```

See [`ControlWorkbench.VEX/PROS/README.md`](ControlWorkbench.VEX/PROS/README.md) for full documentation.

**Features:**
- Pure Pursuit path following
- Odometry (tracking wheels, motor encoders, IMU fusion)
- PID with anti-windup and motion profiling
- Subsystems (Intake with color sorting, Lift presets, Pneumatics)
- Autonomous selector with LCD display
- Real-time telemetry streaming
- Match logging and replay

---

## ?? Advanced Control Theory

ControlWorkbench implements graduate-level control algorithms:

### Model Predictive Control
```csharp
var mpc = new ModelPredictiveController(A, B, Q, R, horizonLength: 20);
mpc.SetInputConstraints(uMin, uMax);
var u = mpc.ComputeControl(currentState, reference);
```

### Extended Kalman Filter
```csharp
var ekf = new ExtendedKalmanFilter(stateSize, measurementSize);
ekf.Predict(controlInput, dt);
ekf.Update(measurement);
var estimate = ekf.State;
```

### LQR Controller
```csharp
var lqr = new LqrController();
var K = lqr.ComputeGains(A, B, Q, R);
var u = -K * x;
```

---

## ??? Development

### Building from Source

```bash
# Restore dependencies
dotnet restore

# Build all projects
dotnet build

# Run tests
dotnet test

# Run application
dotnet run --project ControlWorkbench.App
```

### Project Dependencies

- **ScottPlot.WPF** - Real-time plotting and visualization
- **MathNet.Numerics** - Linear algebra and numerical methods
- **System.IO.Ports** - Serial communication

---

## ?? Telemetry Protocol

ControlWorkbench uses a binary protocol for high-speed telemetry:

| Field | Type | Description |
|-------|------|-------------|
| Header | 2 bytes | `0xCB 0x01` |
| Message ID | 1 byte | Message type |
| Length | 2 bytes | Payload length |
| Payload | N bytes | Message data |
| CRC | 2 bytes | CRC-16 checksum |

Supports 100+ Hz update rates over serial.

---

## ?? Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## ?? License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## ?? Acknowledgments

- **VEX Robotics** - V5 platform and competition ecosystem
- **PROS** - Open-source development environment for VEX
- **LemLib** - Inspiration for path following algorithms
- **ArduPilot/PX4** - Open-source autopilot platforms
- **ScottPlot** - Excellent plotting library for .NET

---

<div align="center">

**Built for Competition ??**

*From VRC to commercial drones - one platform for all your control needs.*

[Report Bug](https://github.com/smiles0527/RoboControl/issues) · [Request Feature](https://github.com/smiles0527/RoboControl/issues)

</div>
