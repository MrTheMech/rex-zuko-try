# Zuko Quadruped Robot - Project Overview

## Introduction

**Zuko** is a quadruped robot dog project built using ROS2, featuring a Raspberry Pi 4 with custom expansion boards for motor and peripheral control. The robot uses a Playstation 4 controller for teleoperation and implements advanced inverse kinematics for a linked-leg system.

## Project Status

**Current Status:** Development  
**Frame Version:** v2.2 (being tested, nearly ready for replication)

## Project Structure

```
rex-zuko-try/
├── quad_ws/              # ROS2 workspace (main codebase)
│   ├── src/              # ROS2 packages
│   └── dev-notes/        # Development notes and setup instructions
├── cad/                  # CAD design files (v1.0, v2.0, v2.1, v2.2)
│   ├── v1.0/             # Initial frame design
│   ├── v2.0/             # Second iteration
│   ├── v2.1/             # Third iteration
│   └── v2.2/             # Current frame design
├── stl/                  # 3D printable STL files
│   ├── tools/            # Printing tools
│   └── v2.2/             # Current frame STL files
├── pcbs/                 # PCB designs (KiCad 6.0)
│   ├── battery-board/    # 8x 18650 battery holder with BMS
│   ├── expansion-board/  # RPi 4 expansion board
│   └── motor-controller/ # Teensy 4.0 motor controller
├── docs/                 # Documentation
│   ├── bom.xlsx          # Bill of materials
│   ├── todo.txt          # Task list
│   └── assembling.txt    # Assembly instructions
├── images/               # Project images and photos
├── diagrams/             # Kinematic diagrams (draw.io)
├── scripts/              # Standalone Python scripts
│   └── kinematics-simulation/  # Kinematics testing scripts
└── video/                # Test videos
```

## ROS2 Workspace Architecture

The main codebase is organized into ROS2 packages within `quad_ws/src/`:

### Core Packages

#### 1. **quad_main** - Main Control System
The central control package that orchestrates robot movement.

**Key Components:**
- `quad_main.py` - ROS2 wrapper and main entry point
- `quad_commander.py` - High-level control interface
- `kinematics.py` - Inverse kinematics solver for linked-leg system
- `bezier_gait.py` - Bezier curve-based gait generation
- `joystick_interpreter.py` - Gamepad input processing
- `gamepad_map.py` - Controller button/axis mapping
- `motion_inputs.py` - Motion input data structures
- `matrix_transforms.py` - 3D transformation utilities

**Configuration Files:**
- `frame_parameters.yaml` - Physical frame dimensions (meters)
- `motion_parameters.yaml` - Motion limits and gait parameters
- `linked_leg_parameters.yaml` - Linked-leg kinematic parameters

**Launch Files:**
- `quad_all.launch.py` - Launch all nodes (main, gamepad, simulation)
- `quad_live.launch.py` - Launch for live robot operation
- `quad_main.launch.py` - Launch main node only

#### 2. **quad_motors** - Servo Control
Handles servo motor control via PCA9685 I2C PWM driver.

**Key Components:**
- `quad_motors.py` - ROS2 node subscribing to joint angles
- `servo_controller.py` - PCA9685 interface and servo management
- `PCA9685Servos.py` - Low-level PCA9685 driver
- `servo_calibration.py` - Interactive servo calibration tool

**Configuration Files:**
- `servo_parameters.yaml` - Servo calibration data (zero positions, limits, pin mapping)

**Features:**
- 12 servo control (one per joint)
- Per-servo calibration (zero position, min/max limits)
- Direction inversion support
- Hardware pin mapping

#### 3. **quad_gamepad** - Controller Input
Playstation 4 controller interface for teleoperation.

**Key Components:**
- `quad_gamepad.py` - ROS2 node publishing Joy messages
- `Gamepad.py` - Controller library (from piborg/Gamepad)
- `Controllers.py` - Controller type definitions

**Features:**
- Wired and wireless PS4 controller support
- Publishes `sensor_msgs/Joy` messages
- Configurable joystick number
- 10ms publish rate

#### 4. **quad_interfaces** - Message Definitions
ROS2 custom message types.

**Messages:**
- `JointAngles.msg` - Joint angle commands (12 joints, with linked-leg variants)
- `Outputs.msg` - LED and NeoPixel control outputs

#### 5. **quad_simulation** - Simulation Environment
PyBullet-based simulation for testing and development.

**Key Components:**
- `quad_simulation.py` - Main simulation node
- `gym_env.py` - Gym environment wrapper
- `kinematics.py` - Simulation kinematics
- `model.py` - Robot model definition
- `motor_model.py` - Motor dynamics simulation
- `heightfield.py` - Terrain generation
- `gui_param_control.py` - GUI parameter adjustment

**Features:**
- PyBullet physics simulation
- URDF model integration
- STL mesh visualization
- Parameter tuning GUI

#### 6. **quad_udrf** - URDF Robot Model
Robot description for visualization and simulation.

**Components:**
- `zuko.urdf` / `zuko.xacro` - Robot description file
- STL files for visualization
- RViz configuration

**Usage:**
```bash
ros2 launch quad_udrf display.launch.py
```

#### 7. **quad_sensors** - Sensor Handling
Sensor data processing (IMU, foot sensors, etc.)

#### 8. **quad_output** - Output Control
Peripheral control (LEDs, NeoPixels, etc.)

## System Architecture

### Data Flow

```
PS4 Controller (Physical Input)
    ↓
quad_gamepad (Joy messages)
    ↓
quad_main/joystick_interpreter (Process input)
    ↓
quad_main/quad_commander (High-level control)
    ├── bezier_gait (Generate foot trajectories)
    └── kinematics (Inverse kinematics)
    ↓
JointAngles message (12 joint angles)
    ↓
quad_motors (Servo control)
    ↓
PCA9685 I2C Driver
    ↓
12 RC Servos (Physical actuators)
```

### Key Algorithms

#### Linked-Leg Kinematics
The robot uses a linked-leg design where the lower leg is connected via a linkage system. This requires special inverse kinematics calculations to convert desired foot positions into joint angles.

**Parameters:**
- Link lengths (L2-L10)
- Origin offsets (A, D)
- Frame dimensions (shoulder, upper leg, lower leg lengths)

#### Bezier Gait Generation
Uses Bezier curves to generate smooth foot trajectories for walking gaits.

**Parameters:**
- Step velocity
- Swing period
- Clearance height
- Penetration depth
- Lateral fraction

## Hardware Specifications

### Frame (v2.2)
- **Material:** 3D printed (PLA/PETG)
- **Weight:** ~2200g total (frame + electronics + batteries)
- **Dimensions:**
  - Hip spacing: 238mm (x) × 91mm (y)
  - Foot spacing: 238mm (x) × 154.5mm (y)
  - Body height: 195mm
  - Upper leg: 100mm
  - Lower leg: 112.25mm

### Electronics

#### Option 1: Expansion Board (Recommended)
- **Platform:** Raspberry Pi 4 direct connection
- **Interface:** I2C (PWM and ADC controllers)
- **Features:**
  - 12 servo PWM outputs
  - I2C IMU support
  - Touchdown switches
  - LED control
  - NeoPixel (WS2812B) support
  - Current/voltage sensing
  - Auxiliary servo outputs
- **Fabrication:** 4-layer, 1-2oz copper

#### Option 2: Motor Controller Board
- **Platform:** Teensy 4.0 microcontroller
- **Interface:** micro-ROS
- **Features:** Similar to expansion board
- **Power:** Requires external DC-DC converter (5V)

### Power System
- **Battery Board:** 8x 18650 LiPo batteries
- **BMS:** 4x HY2120 (or similar) battery management systems
- **Capacity:** ~94g per battery × 8 = 752g
- **Fabrication:** 2-layer, 1-2oz copper

### Actuators
- **Type:** 12x RC servos
- **Configuration:** 3 servos per leg (hip, upper leg, lower leg)
- **Control:** PCA9685 I2C PWM driver (0x40 address)

## Configuration Files

### Frame Parameters (`frame_parameters.yaml`)
Physical dimensions of the robot frame:
- `shoulder_length` - Hip to upper leg pivot distance
- `upper_leg_length` - Upper leg length
- `lower_leg_length` - Lower leg length
- `hip_x`, `hip_y` - Hip spacing
- `foot_x`, `foot_y` - Foot spacing
- `height` - Body height from ground
- `com_offset` - Center of mass offset

### Motion Parameters (`motion_parameters.yaml`)
Motion limits and gait parameters:
- `step_velocity` - Walking speed
- `swing_period` - Leg swing time
- `clearance_height` - Foot lift height
- `penetration_depth` - Foot ground penetration
- `pos_x/y/z_min/max` - Position limits
- `orn_x/y/z_min/max` - Orientation limits
- `yaw_rate_min/max` - Rotation limits
- `step_length_min/max` - Step length limits

### Linked Leg Parameters (`linked_leg_parameters.yaml`)
Kinematic parameters for linked-leg system:
- Link lengths: L2 through L10 (in mm)
- Origin points: A, D (x, y coordinates)

### Servo Parameters (`servo_parameters.yaml`)
Per-servo calibration data:
- `zero_degrees_pulse_width` - Zero position pulse width
- `pulse_width_per_degree` - Servo sensitivity
- `min_degrees`, `max_degrees` - Joint limits
- `invert_direction` - Direction inversion flags
- `map_joint_index_to_driver_pin` - Hardware pin mapping

## Development Workflow

### Building the Workspace
```bash
cd quad_ws
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
```

### Running the Robot
```bash
# Launch all nodes (main, gamepad, simulation)
ros2 launch quad_main quad_all.launch.py

# Launch for live robot operation
ros2 launch quad_main quad_live.launch.py

# Launch individual nodes
ros2 launch quad_main quad_main.launch.py
ros2 launch quad_motors quad_motors.launch.py
```

### Servo Calibration
```bash
cd quad_ws/src/quad_motors/quad_motors/
./servo_calibration.py
```

## Current Development Tasks

### Immediate Tasks
- Complete ROS2 tasks
- Fabricate and test expansion-board PCB
- Update BOM (Bill of Materials)
- Create PCB BOM
- Upload STL files
- Create print time/weight documentation
- Create end-user documentation

### Long-term Goals
- Reconnect machine learning for terrain navigation
- Update URDF for Gazebo
- Add speakers for barks
- Add camera for computer vision
- Code in IMU, voltage/current sensing, LEDs
- Pythonize and clean up code

## Key Contributions

- Frame redesign with larger servos and improved leg/hip system
- Migration from ROS1 to ROS2
- Inverse kinematics for linked-leg quadruped system
- Parameterized hard-coded values
- Servo calibration script
- Feature-rich motor and peripheral expansion board
- 8x 18650 battery holder with onboard BMS

## Credits and References

- **Spot Mini Mini:** Simulation, kinematics, bezier curves, and machine learning
  - https://github.com/OpenQuadruped/spot_mini_mini
- **SpotMicroAI:** Base quadruped project
  - https://spotmicroai.readthedocs.io/en/latest/
- **Kangel Robot Dog:** Frame and leg linkage design inspiration
  - https://grabcad.com/library/diy-quadruped-robot-1

## Communities

- **SpotMicroAI Discord:** https://discord.gg/m5fsENhT
- **Quadrupedalism:** https://quadrupedalism.com/

## Technical Details

### ROS2 Version
- **Distribution:** Foxy Fitzroy
- **Python Version:** Python 3.8+

### Dependencies
- **ROS2:** Foxy desktop (desktop) or ros-base (RPi)
- **Python:** numpy, yaml, rclpy
- **Hardware:** smbus2 (for I2C on RPi)
- **Simulation:** pybullet, gym (desktop only)
- **Bluetooth:** pi-bluetooth (RPi for wireless controller)

### Communication
- **Topics:**
  - `/joy` - Gamepad input (sensor_msgs/Joy)
  - `/joint_angles` - Joint commands (quad_interfaces/JointAngles)
- **Nodes:**
  - `joy_subscriber_node` - Receives gamepad input
  - `joint_angles_publisher_node` - Publishes joint commands
  - `joint_angles_subscriber_node` - Receives joint commands for motors
  - `quad_gamepad` - Gamepad publisher

## File Organization

### Source Code Structure
```
quad_ws/src/
├── quad_main/
│   ├── quad_main/
│   │   ├── quad_main.py          # Main entry point
│   │   └── src/                   # Core algorithms
│   ├── config/                    # YAML configuration files
│   └── launch/                    # Launch files
├── quad_motors/
│   ├── quad_motors/
│   │   ├── quad_motors.py        # Motor control node
│   │   └── src/                   # Servo drivers
│   └── config/                    # Servo calibration
└── [other packages...]
```

### Configuration Management
All configuration is YAML-based and loaded at runtime:
- Frame parameters define physical dimensions
- Motion parameters define behavior limits
- Servo parameters define hardware calibration
- Parameters are passed via ROS2 parameter system

## Troubleshooting

### Common Issues
1. **I2C Permission Errors:** Add user to i2c group or use `sudo chmod a+rw /dev/i2c-*`
2. **Controller Not Detected:** Check bluetooth pairing and joystick number parameter
3. **Servo Calibration Missing:** Run `servo_calibration.py` first
4. **Build Errors:** Ensure all dependencies are installed and ROS2 is sourced

## Future Enhancements

- Machine learning integration for adaptive locomotion
- Gazebo simulation support
- Audio system (speakers, barks)
- Computer vision (camera integration)
- Enhanced sensor fusion (IMU, foot sensors)
- Improved documentation and tutorials

---

*Last Updated: Based on project structure analysis*
