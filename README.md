# ControlWorkbench

A .NET desktop engineering tool for robotics and drones. ControlWorkbench is a calculation, tuning, calibration, and telemetry workbench that helps users perform rigorous math for PID control, odometry, and sensors (IMU/GPS/magnetometer).

## Features

### Connection & Telemetry
- **Multi-transport support**: Serial, UDP, and Mock telemetry source
- **Real-time telemetry display**: Live numeric panels for attitude, IMU, GPS, and state estimates
- **Custom binary protocol**: Efficient framing with CRC32 validation
- **Statistics monitoring**: Bytes/packets per second, error counts

### Math Workbench (EKF/Odometry)
- **Motion Models**: Unicycle 2D, Constant Velocity 2D, Yaw-only Strapdown
- **Measurement Models**: GPS position, Yaw (magnetometer), Range-bearing
- **EKF Prediction**: Compute F, G matrices and propagate covariance
- **EKF Update**: Compute H, S, K matrices and update state
- **Jacobian Validation**: Compare analytic vs numerical Jacobians
- **Complementary Filter Design**: Design filters for gyro+mag/acc fusion

### Control Design (PID/LQR)
- **PID Controller**: Configurable gains, output limits, integrator limits, derivative filtering
- **Cascaded PID Design**: Automatic bandwidth separation recommendations
- **LQR Design**: Continuous (CARE) and Discrete (DARE) Riccati solvers
- **Step Response Simulation**: Simulate closed-loop response

### Logs & Metrics
- **Step Response Analysis**: Rise time, overshoot, settling time, steady-state error
- **Sample Data Generation**: Test without hardware

## Project Structure

```
ControlWorkbench/
??? ControlWorkbench.App/       # WPF application (UI)
??? ControlWorkbench.Core/      # Common utilities, units, collections
??? ControlWorkbench.Math/      # Linear algebra, EKF, PID, LQR
??? ControlWorkbench.Protocol/  # Binary protocol encoder/decoder
??? ControlWorkbench.Transport/ # Serial, UDP, Mock transports
??? ControlWorkbench.Tests/     # xUnit tests
```

## Requirements

- .NET 8.0 SDK
- Windows (WPF application)

## Build & Run

```bash
# Clone and navigate to directory
cd ControlWorkbench

# Restore and build
dotnet restore ControlWorkbench.sln
dotnet build ControlWorkbench.sln

# Run the application
dotnet run --project ControlWorkbench.App

# Run tests
dotnet test ControlWorkbench.Tests
```

## Quick Start

1. **Launch the application**
2. **Go to Connection tab**
3. **Select "Mock Telemetry Source"** and click Connect
4. **Navigate to Telemetry Dashboard** to see live data
5. **Explore Math Workbench** for EKF calculations
6. **Use Control Design** for PID and LQR tuning

## Protocol Specification

### Frame Format
```
| Preamble | Version | MsgType | PayloadLen | Payload    | CRC32  |
| 2 bytes  | 1 byte  | 1 byte  | 2 bytes    | 0-4096     | 4 bytes|
| 0xAA 0x55| 0x01    |         | LE         |            | IEEE   |
```

### Message Types
| Type | ID   | Description |
|------|------|-------------|
| Heartbeat | 0x01 | System status |
| ImuRaw | 0x02 | Gyro/Accel data |
| Attitude | 0x03 | Euler angles |
| Gps | 0x04 | Position/velocity |
| StateEstimate2D | 0x05 | EKF state |
| ParamValue | 0x10 | Parameter read |
| ParamSet | 0x11 | Parameter write |

## Dependencies

- **MathNet.Numerics**: Linear algebra operations
- **ScottPlot.WPF**: Live plotting (placeholder integration)
- **System.IO.Ports**: Serial communication
- **xUnit**: Unit testing

## License

MIT License
