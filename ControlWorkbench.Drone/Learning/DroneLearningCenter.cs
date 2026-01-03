namespace ControlWorkbench.Drone.Learning;

/// <summary>
/// Comprehensive learning center for drone pilots and builders.
/// </summary>
public class DroneLearningCenter
{
    public List<DroneTutorial> Tutorials { get; } = new()
    {
        QuadcopterBasics,
        PidTuningForDrones,
        MissionPlanning,
        FpvBasics,
        BatteryManagement,
        FailsafesAndSafety,
        GpsAndNavigation,
        AutonomousFlight
    };
    
    public static DroneTutorial QuadcopterBasics => new()
    {
        Id = "quadcopter-basics",
        Title = "Quadcopter Flight Principles",
        Category = DroneCategory.Fundamentals,
        Difficulty = Difficulty.Beginner,
        EstimatedTime = TimeSpan.FromMinutes(20),
        Sections = new List<Section>
        {
            new()
            {
                Title = "How Quadcopters Fly",
                Content = @"A quadcopter has 4 motors arranged in an X or + pattern. Each motor spins a propeller that generates thrust (lift).

**Motor Numbering (X configuration):**
```
    Front
  4 ?   1 ?
     \ /
      X
     / \
  3 ?   2 ?
    Back
```

**Key Principles:**
- Motors 1 & 3 spin clockwise (CW)
- Motors 2 & 4 spin counter-clockwise (CCW)
- This cancels out the rotational torque

**Control Axes:**
- **Throttle**: All motors speed up or slow down together
- **Pitch** (forward/back): Front motors vs back motors
- **Roll** (left/right): Left motors vs right motors
- **Yaw** (rotation): CW motors vs CCW motors"
            },
            new()
            {
                Title = "Control Mixing",
                Content = @"The flight controller mixes your stick inputs to control each motor:

```
Motor 1 (FR) = Throttle - Roll + Pitch + Yaw
Motor 2 (BR) = Throttle - Roll - Pitch - Yaw
Motor 3 (BL) = Throttle + Roll - Pitch + Yaw
Motor 4 (FL) = Throttle + Roll + Pitch - Yaw
```

**Example: Roll Right**
- Motors 3 & 4 (left side) speed up
- Motors 1 & 2 (right side) slow down
- The drone tilts right and moves right

**Example: Yaw Right**
- Motors 1 & 3 (CW) speed up
- Motors 2 & 4 (CCW) slow down
- The drone rotates clockwise"
            },
            new()
            {
                Title = "Flight Modes",
                Content = @"**Stabilized Modes (beginner-friendly):**
- **Stabilize/Angle**: Stick controls angle, releases = level
- **Alt Hold**: Throttle controls climb rate, holds altitude
- **Loiter/GPS Hold**: Holds position using GPS

**Acro Mode (advanced):**
- Stick controls rotation rate, not angle
- Releases stick = maintains current angle
- Required for flips, rolls, freestyle flying

**Autonomous Modes:**
- **Auto**: Follows pre-programmed waypoints
- **Guided**: External control (ground station)
- **RTL**: Automatic return to launch point
- **Land**: Automatic landing"
            }
        }
    };
    
    public static DroneTutorial PidTuningForDrones => new()
    {
        Id = "drone-pid-tuning",
        Title = "PID Tuning for Multirotors",
        Category = DroneCategory.Tuning,
        Difficulty = Difficulty.Intermediate,
        EstimatedTime = TimeSpan.FromMinutes(30),
        Sections = new List<Section>
        {
            new()
            {
                Title = "Drone PID Structure",
                Content = @"Drones use cascaded PID loops:

```
Stick Input ? [Attitude P] ? [Rate PID] ? Motor Output
                   ?              ?
             Target angle    Target rate
```

**Rate PID (inner loop):**
- Runs at 4-8kHz
- Controls how fast the drone rotates
- Most important for feel and stability

**Attitude P (outer loop):**
- Runs at 1kHz
- Converts desired angle to desired rate
- Single P gain, no I or D

**Position PID (if GPS):**
- Runs at 50-100Hz
- Controls location in space"
            },
            new()
            {
                Title = "Rate PID Tuning",
                Content = @"**Start with P (Proportional):**
1. Set I and D to 0
2. Increase P until you see oscillation
3. Back off 20-30%

**Add D (Derivative):**
1. D dampens oscillation
2. Increase until you see motor noise/heat
3. Back off 20%

**Finally I (Integral):**
1. I corrects steady-state error
2. Usually set equal to P for rate PIDs
3. Too much = slow wobbles, 'I-term windup'

**Typical Starting Values:**
| Axis | P | I | D |
|------|-----|-----|------|
| Roll | 45 | 45 | 25 |
| Pitch | 47 | 47 | 27 |
| Yaw | 35 | 35 | 0 |

**Note:** These are Betaflight 4.x scale (divide by ~1000 for ArduCopter)"
            },
            new()
            {
                Title = "Diagnosing Issues",
                Content = @"**Problem: Fast oscillation (propwash)**
- Cause: D too high or P too high
- Fix: Lower D, add D-term filtering

**Problem: Slow wobble/bounce**
- Cause: I too high or P too low
- Fix: Lower I or increase P

**Problem: Feels 'soft' or 'floaty'**
- Cause: P too low
- Fix: Increase P

**Problem: Toilet bowl (circling on loiter)**
- Cause: Compass issues, not PID
- Fix: Compass calibration, move away from metal

**Problem: Drifts in one direction**
- Cause: Accelerometer or CG offset
- Fix: Calibrate accelerometer, balance CG

**Use Blackbox logging:**
1. Enable blackbox recording
2. Fly figure-8s and quick stops
3. Analyze with Blackbox Explorer
4. Look for gyro tracking setpoint"
            }
        }
    };
    
    public static DroneTutorial MissionPlanning => new()
    {
        Id = "mission-planning",
        Title = "Autonomous Mission Planning",
        Category = DroneCategory.Autonomy,
        Difficulty = Difficulty.Intermediate,
        EstimatedTime = TimeSpan.FromMinutes(25),
        Sections = new List<Section>
        {
            new()
            {
                Title = "Mission Basics",
                Content = @"A mission is a sequence of commands the drone executes automatically.

**Common Commands:**
- **NAV_WAYPOINT**: Fly to location
- **NAV_LOITER_TIME**: Hover for X seconds
- **NAV_LAND**: Land at location
- **NAV_RETURN_TO_LAUNCH**: Return home

**Mission Flow:**
```
1. ARM
2. TAKEOFF to 50m
3. WAYPOINT 1
4. WAYPOINT 2 (hold 10s)
5. WAYPOINT 3
6. LAND
```

**Coordinate Systems:**
- **Absolute (AMSL)**: Altitude above sea level
- **Relative (AGL)**: Altitude above home point
- Most missions use relative altitude"
            },
            new()
            {
                Title = "Survey/Mapping Missions",
                Content = @"**Grid Pattern:**
For mapping, create a lawnmower pattern:
```
  ????????????????
  ? ???????????? ?
  ? ???????????? ?
  ? ? ????????????
  ? ??????????????
  ? ?
Start
```

**Key Parameters:**
- **Overlap**: 70-80% front, 60-70% side
- **Altitude**: Higher = larger coverage, lower resolution
- **Speed**: Slower = sharper images

**Camera Trigger:**
- Distance-based: Trigger every X meters
- Time-based: Trigger every X seconds

**GSD (Ground Sample Distance):**
```
GSD (cm/pixel) = (Altitude × Sensor Width) / (Focal Length × Image Width)
```

For 1cm/pixel at 100m with typical drone camera:
- About 80m altitude"
            },
            new()
            {
                Title = "Safety Considerations",
                Content = @"**Pre-flight Checklist:**
- [ ] GPS satellites > 10
- [ ] HDOP < 2.0
- [ ] Compass calibrated
- [ ] Battery > 80%
- [ ] Home point set correctly
- [ ] Geofence configured
- [ ] RTL altitude safe

**Failsafe Settings:**
- **RC Loss**: RTL or Land
- **GPS Loss**: Land (not RTL!)
- **Low Battery**: RTL
- **Geofence Breach**: RTL or Land

**Mission Validation:**
- Max altitude legal in your area
- No fly zones avoided
- Battery sufficient for mission + RTL + reserve
- Clear line of sight maintained
- Alternate landing sites identified"
            }
        }
    };
    
    public static DroneTutorial FpvBasics => new()
    {
        Id = "fpv-basics",
        Title = "FPV (First Person View) Flying",
        Category = DroneCategory.Fpv,
        Difficulty = Difficulty.Intermediate,
        EstimatedTime = TimeSpan.FromMinutes(25),
        Sections = new List<Section>
        {
            new()
            {
                Title = "FPV System Components",
                Content = @"**On the Drone:**
1. **Camera**: Low latency camera (CMOS, not rolling shutter)
2. **VTX**: Video transmitter (25mW-600mW)
3. **Antenna**: Circular polarized (RHCP usually)

**On the Ground:**
1. **Goggles or Monitor**: Fatshark, DJI, Walksnail, etc.
2. **VRX**: Video receiver (often built into goggles)
3. **Antenna**: Matching polarization to VTX

**Frequencies:**
- 5.8GHz: Most common, 40+ channels
- 2.4GHz: Better penetration, fewer channels
- DJI/Walksnail: Digital, HD quality

**Important Settings:**
- Analog: Choose clean channel
- Keep VTX power appropriate
- Never power VTX without antenna!"
            },
            new()
            {
                Title = "Flying Acro Mode",
                Content = @"**Key Differences from Stabilized:**
- Releasing stick = maintains angle (not level)
- You control rotation RATE, not position
- Required for freestyle and racing

**Learning Progression:**
1. Simulator first! (Velocidrone, Liftoff)
2. Practice hovering in place
3. Figure-8s at low altitude
4. Smooth turns maintaining altitude
5. Power loops and split-S

**Key Techniques:**
- **Throttle management**: Cut throttle in dives, add in climbs
- **Coordinated turns**: Roll and pitch together
- **Air mode**: Keeps control authority at zero throttle

**Rates Setup:**
- Center sensitivity: 100-300 deg/s
- Max rate: 600-1000 deg/s
- Expo: 0.2-0.5 (makes center less sensitive)"
            },
            new()
            {
                Title = "FPV Racing",
                Content = @"**Racing Setup Priorities:**
1. Low latency (camera, VTX, goggles)
2. Light weight (every gram counts)
3. Durability (you WILL crash)
4. Power (high KV motors, 6S battery)

**Typical Race Quad Specs:**
- Frame: 5"" carbon fiber, ~100g
- Motors: 2306-2407, 1700-2400KV
- Props: 5"" triblade
- Battery: 6S 1300-1500mAh
- AUW: 650-750g

**Gate/Flag Types:**
- Single gate: Fly through
- Split-S gate: Approach, flip, dive through
- Ladder: Climb through
- Dive gate: Approach high, dive through

**Racing Line:**
- Cut inside of gates
- Maintain momentum
- Look ahead to next gate
- Throttle management through curves"
            }
        }
    };
    
    public static DroneTutorial BatteryManagement => new()
    {
        Id = "battery-management",
        Title = "LiPo Battery Safety & Care",
        Category = DroneCategory.Safety,
        Difficulty = Difficulty.Beginner,
        EstimatedTime = TimeSpan.FromMinutes(15),
        Sections = new List<Section>
        {
            new()
            {
                Title = "LiPo Battery Basics",
                Content = @"**Cell Voltage:**
- Full: 4.2V per cell
- Nominal: 3.7V per cell
- Empty: 3.0V per cell (NEVER go below!)
- Storage: 3.8V per cell

**Battery Notation:**
- **4S**: 4 cells in series (4 × 3.7V = 14.8V nominal)
- **1500mAh**: Capacity in milliamp-hours
- **100C**: Discharge rate (100 × 1.5A = 150A max)

**Common Sizes:**
| Config | Voltage | Use Case |
|--------|---------|----------|
| 1S | 3.7V | Tiny whoops |
| 3S | 11.1V | Small quads |
| 4S | 14.8V | 5"" freestyle |
| 6S | 22.2V | 5"" racing, 7""+ |"
            },
            new()
            {
                Title = "Safety Rules",
                Content = @"**NEVER:**
- Puncture a battery
- Charge unattended
- Charge immediately after flight (wait 15 min)
- Over-discharge below 3.0V/cell
- Use a puffy/damaged battery

**ALWAYS:**
- Use a LiPo-safe charging bag
- Balance charge batteries
- Store at 3.8V/cell if not using for >1 week
- Check cells before charging
- Dispose of damaged batteries properly

**If a battery catches fire:**
1. DO NOT use water
2. Use sand, fire extinguisher, or let it burn in safe area
3. Ventilate the area (toxic fumes)
4. Do not inhale smoke"
            },
            new()
            {
                Title = "Maximizing Battery Life",
                Content = @"**Charging Best Practices:**
- Use 1C charge rate (1500mAh = 1.5A)
- Always balance charge
- Don't charge cold batteries (<10°C)
- Let cool after flight before charging

**Flight Practices:**
- Don't drain below 3.5V/cell under load
- Land at 3.6-3.7V/cell under load
- Avoid very high current draws for extended time

**Storage:**
- Store at 3.8V/cell
- Cool, dry location
- In fireproof container/bag
- Not fully charged or discharged

**Battery Health:**
- Internal resistance increases over time
- Cells should stay within 0.01V of each other
- Replace if puffy, damaged, or IR too high"
            }
        }
    };
    
    public static DroneTutorial FailsafesAndSafety => new()
    {
        Id = "failsafes-safety",
        Title = "Failsafes and Flight Safety",
        Category = DroneCategory.Safety,
        Difficulty = Difficulty.Beginner,
        EstimatedTime = TimeSpan.FromMinutes(20),
        Sections = new List<Section>
        {
            new()
            {
                Title = "Common Failsafes",
                Content = @"**Radio Failsafe:**
Triggers when RC link is lost.
- **Land**: Land immediately
- **RTL**: Return to home, then land
- **Hover**: Hold position (requires GPS)

**GPS Failsafe:**
Triggers on GPS loss in GPS modes.
- Usually switches to Altitude Hold
- Or lands if configured

**Battery Failsafe:**
- Voltage-based: e.g., 3.5V/cell = RTL
- Capacity-based: e.g., 80% used = warn, 90% = RTL

**Geofence:**
Virtual boundary the drone won't cross.
- Altitude limit
- Distance from home limit
- Polygon exclusion zones"
            },
            new()
            {
                Title = "Pre-flight Safety",
                Content = @"**Pre-arm Checks:**
- Propellers secure and correct direction
- Battery strapped securely
- All connections secure
- Camera/VTX working
- GPS lock (>6 satellites)
- Compass healthy
- No error messages
- Failsafes configured

**Environmental Checks:**
- Wind within limits
- No precipitation
- Clear of obstacles
- No fly zone check
- Permission if needed
- People cleared from area

**Post-arm Check:**
- Motors respond correctly
- Gyros steady
- No vibrations
- RTL works (test if new setup)"
            }
        }
    };
    
    public static DroneTutorial GpsAndNavigation => new()
    {
        Id = "gps-navigation",
        Title = "GPS and Navigation Systems",
        Category = DroneCategory.Autonomy,
        Difficulty = Difficulty.Intermediate,
        EstimatedTime = TimeSpan.FromMinutes(25),
        Sections = new List<Section>
        {
            new()
            {
                Title = "GPS Technology",
                Content = @"**GPS Fix Types:**
- **No Fix**: Can't determine position
- **2D Fix**: Lat/lon only, no altitude
- **3D Fix**: Full position, needs 4+ satellites
- **DGPS**: Differential, more accurate
- **RTK**: Centimeter accuracy

**Key Metrics:**
- **Satellites**: More = better. Want >10
- **HDOP**: Horizontal precision (lower = better). Want <2.0
- **VDOP**: Vertical precision. Want <3.0

**Multi-constellation:**
- GPS (USA)
- GLONASS (Russia)
- Galileo (Europe)
- BeiDou (China)

Using multiple constellations improves accuracy and fix time."
            },
            new()
            {
                Title = "RTK GPS",
                Content = @"**What is RTK?**
Real-Time Kinematic GPS provides centimeter-level accuracy using:
1. **Base station**: Fixed location, known position
2. **Rover**: On the drone
3. **Correction data**: Sent via radio or internet

**RTK Fix Types:**
- **Float**: Partial fix, ~0.5-1m accuracy
- **Fixed**: Full fix, 1-2cm accuracy

**Requirements:**
- RTK GPS module ($200-500+)
- Base station or NTRIP service
- Clear sky view
- Processing power

**Use Cases:**
- Precision agriculture
- Surveying/mapping
- Infrastructure inspection
- Precision landing"
            }
        }
    };
    
    public static DroneTutorial AutonomousFlight => new()
    {
        Id = "autonomous-flight",
        Title = "Autonomous Flight & Programming",
        Category = DroneCategory.Autonomy,
        Difficulty = Difficulty.Advanced,
        EstimatedTime = TimeSpan.FromMinutes(35),
        Sections = new List<Section>
        {
            new()
            {
                Title = "MAVLink Protocol",
                Content = @"**What is MAVLink?**
A lightweight protocol for drone communication.
- Used by ArduPilot, PX4, and many others
- Binary format, efficient
- Supports telemetry, commands, parameters

**Key Message Types:**
- **HEARTBEAT**: System alive, current mode
- **ATTITUDE**: Roll, pitch, yaw angles
- **GLOBAL_POSITION_INT**: GPS location
- **COMMAND_LONG**: Send commands
- **MISSION_ITEM**: Waypoint definition

**Connection Types:**
- Serial (USB, telemetry radio)
- UDP (network, SITL)
- TCP (ground control station)"
            },
            new()
            {
                Title = "DroneKit / MAVSDK",
                Content = @"**Python Example (DroneKit):**
```python
from dronekit import connect, VehicleMode

# Connect to drone
vehicle = connect('/dev/ttyUSB0', baud=57600)

# Wait for GPS
while not vehicle.is_armable:
    print('Waiting for GPS...')
    time.sleep(1)

# Arm and takeoff
vehicle.mode = VehicleMode('GUIDED')
vehicle.arm()

vehicle.simple_takeoff(10)  # 10 meters

# Fly to location
target = LocationGlobalRelative(lat, lon, alt)
vehicle.simple_goto(target)

# Wait until arrival
while True:
    remaining = get_distance(vehicle.location.global_frame, target)
    if remaining < 1:
        break
    time.sleep(1)

# Land
vehicle.mode = VehicleMode('LAND')
```

**C# Example (ControlWorkbench):**
```csharp
var fc = new MavlinkFlightController();
await fc.ConnectAsync(""serial:COM3:57600"");

await fc.ArmAsync();
await fc.TakeoffAsync(10);

await fc.GotoLocationAsync(lat, lon, alt);

await fc.LandAsync();
```"
            }
        }
    };
}

public class DroneTutorial
{
    public string Id { get; set; } = "";
    public string Title { get; set; } = "";
    public DroneCategory Category { get; set; }
    public Difficulty Difficulty { get; set; }
    public TimeSpan EstimatedTime { get; set; }
    public List<Section> Sections { get; set; } = new();
}

public class Section
{
    public string Title { get; set; } = "";
    public string Content { get; set; } = "";
    public string? CodeSnippet { get; set; }
}

public enum DroneCategory
{
    Fundamentals,
    Tuning,
    Fpv,
    Autonomy,
    Safety,
    Hardware
}

public enum Difficulty
{
    Beginner,
    Intermediate,
    Advanced,
    Expert
}
