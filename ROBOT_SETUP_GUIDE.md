# Zuko Robot - Complete Setup Guide

This guide will walk you through the complete setup process for your Zuko quadruped robot, from software installation to first movement.

## Prerequisites Checklist

Before starting, ensure you have:

- [ ] Raspberry Pi 4 (4GB or 8GB recommended)
- [ ] MicroSD card (32GB+, Class 10)
- [ ] USB-C power supply (5V, 3A minimum)
- [ ] PCA9685 servo driver board (I2C address 0x40)
- [ ] 12 RC servos connected to PCA9685 according to pin layout
- [ ] Power supply for servos
- [ ] PS4 controller (wired or wireless)
- [ ] Network connection (Ethernet or Wi-Fi)
- [ ] Assembled Zuko frame v2.2

## Step 1: Hardware Wiring

### PCA9685 Pin Layout

**IMPORTANT:** Connect your servos to the PCA9685 according to this pin layout:

| Pin | Servo Connection | Joint |
|-----|------------------|-------|
| 0   | FRT (Front Right Tibia) | Joint 5 |
| 1   | FLT (Front Left Tibia) | Joint 2 |
| 2   | FRF (Front Right Femur) | Joint 4 |
| 3   | FLF (Front Left Femur) | Joint 1 |
| 4   | FL (Front Left Hip) | Joint 0 |
| 5   | FR (Front Right Hip) | Joint 3 |
| 6   | BRF (Back Right Femur) | Joint 10 |
| 7   | BLF (Back Left Femur) | Joint 7 |
| 8   | BRT (Back Right Tibia) | Joint 11 |
| 9   | BLT (Back Left Tibia) | Joint 8 |
| 10  | BL (Back Left Hip) | Joint 6 |
| 11  | BR (Back Right Hip) | Joint 9 |

### Wiring Checklist

- [ ] PCA9685 VCC → 5V power supply
- [ ] PCA9685 GND → Common ground (RPi + servos)
- [ ] PCA9685 SDA → Raspberry Pi GPIO 2 (I2C SDA)
- [ ] PCA9685 SCL → Raspberry Pi GPIO 3 (I2C SCL)
- [ ] All 12 servos connected to correct pins (see table above)
- [ ] Servo power supply connected (ensure adequate current capacity)
- [ ] All grounds connected together

**Note:** The pin mapping is already configured in the software. Just ensure your physical wiring matches the table above.

## Step 2: Software Installation

Follow the **INSTALL_RPI.md** guide for complete software installation. Quick summary:

### 2.1 Install Ubuntu 22.04 LTS
- Use Raspberry Pi Imager
- Download Ubuntu 22.04 LTS Server 64-bit
- Default credentials: `ubuntu`/`ubuntu` (change immediately!)

### 2.2 Install ROS2 Humble
```bash
sudo apt update
sudo apt upgrade -y
# Follow ROS2 Humble installation steps from INSTALL_RPI.md
```

### 2.3 Clone and Build Repository

**Option 1: Sparse Clone (Recommended - saves space):**
```bash
cd ~
git clone --filter=blob:none --sparse https://github.com/MrTheMech/rex-zuko-try.git
cd zuko
git sparse-checkout set quad_ws docs scripts
cd quad_ws
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

**Option 2: Full Clone:**
```bash
cd ~
git clone https://github.com/MrTheMech/rex-zuko-try.git
cd zuko/quad_ws
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

**Note:** Sparse clone only downloads:
- `quad_ws/` - ROS2 workspace (required)
- `docs/` - Documentation (useful)
- `scripts/` - Kinematics scripts (optional)

This saves significant space by excluding CAD files, PCB designs, images, and videos.

### 2.4 Install Dependencies
```bash
pip3 install smbus2 numpy pyyaml
sudo apt install python3-colcon-common-extensions i2c-tools -y
```

### 2.5 Configure I2C
```bash
sudo raspi-config
# Navigate to: Interfacing Options → I2C → Enable
echo "sudo chmod a+rw /dev/i2c-*" >> ~/.bashrc
```

### 2.6 Verify I2C Connection
```bash
sudo i2cdetect -y 1
# Should show PCA9685 at address 0x40
```

## Step 3: Servo Calibration

**CRITICAL:** You must calibrate all servos before running the robot!

### 3.1 Safety First
- [ ] **Detach all legs from the frame** to prevent damage during calibration
- [ ] Ensure servos are free to move without hitting mechanical limits

### 3.2 Run Calibration
```bash
cd ~/zuko/quad_ws/src/quad_motors/quad_motors/
chmod +x servo_calibration.py
./servo_calibration.py
```

### 3.3 Calibration Process

For each servo (0-11):

1. **Select servo:** Type `select <number>` (0-11)
2. **Set zero position:** 
   - Move servo to zero position manually or with commands
   - Type `zero <pulse_width>` (typically 1500, range 500-2500)
3. **Set pulse width per degree:** Type `ratio <value>` (typically 11.5)
4. **Set limits:** 
   - Type `min <degrees>` (e.g., `min -30` for hips)
   - Type `max <degrees>` (e.g., `max 90` for femurs)
5. **Invert if needed:** Type `invert` to reverse direction
6. **Test:** Type a pulse width value (e.g., `1500`) to test movement

### 3.4 Calibration Reference

Expected joint limits (adjust based on your frame):
- **Hips (0, 3, 6, 9):** -30° to +30°
- **Femurs (1, 4, 7, 10):** 0° to 90°
- **Tibias (2, 5, 8, 11):** 0° to 90° (or as needed for linked-leg system)

### 3.5 Verify Calibration File
```bash
cat ~/zuko/quad_ws/src/quad_motors/config/servo_parameters.yaml
# Verify all values are set correctly
```

## Step 4: Controller Setup

### 4.1 Wired Controller
- Simply connect PS4 controller via USB
- Verify detection: `ls /dev/input/js*`

### 4.2 Wireless Controller (Optional)
```bash
sudo apt install pi-bluetooth -y
sudo reboot
sudo bluetoothctl
# In bluetoothctl:
scan on
# Put controller in pairing mode (Share + PS buttons)
connect <MAC_ADDRESS>
trust <MAC_ADDRESS>
exit
```

### 4.3 Test Controller
```bash
sudo apt install jstest-gtk -y
jstest-gtk
# Test all buttons and axes
```

## Step 5: First Test Run

### 5.1 Reattach Legs
- [ ] Carefully reattach all legs to the frame
- [ ] Verify all servos are at zero position
- [ ] Ensure robot is on a stable, flat surface

### 5.2 Test Individual Components

**Test I2C:**
```bash
sudo i2cdetect -y 1
# Should show 0x40
```

**Test Servo Communication:**
```bash
cd ~/zuko/quad_ws/src/quad_motors/quad_motors/
./servo_calibration.py
# Use "zero_all" command to set all servos to zero
```

**Test Gamepad:**
```bash
cd ~/zuko/quad_ws
source install/local_setup.bash
ros2 run quad_gamepad quad_gamepad --ros-args -p joystick_number:=0
# In another terminal:
ros2 topic echo /joy
# Move controller and verify messages
```

### 5.3 Launch Robot (First Time)

**Start with simulation to verify software:**
```bash
cd ~/zuko/quad_ws
source install/local_setup.bash
ros2 launch quad_main quad_all.launch.py
```

**For live robot operation:**
```bash
cd ~/zuko/quad_ws
source install/local_setup.bash
ros2 launch quad_main quad_live.launch.py
```

### 5.4 Monitor Robot Status

In a separate terminal:
```bash
# List all nodes
ros2 node list

# Monitor joint angles
ros2 topic echo /joint_angles

# Monitor gamepad input
ros2 topic echo /joy

# View node graph
rqt_graph
```

## Step 6: Tuning and Optimization

### 6.1 Motion Parameters

Edit motion parameters for your specific robot:
```bash
nano ~/zuko/quad_ws/src/quad_main/config/motion_parameters.yaml
```

Key parameters to adjust:
- `step_velocity`: Walking speed (start low: 0.001)
- `clearance_height`: Foot lift height (0.045m default)
- `penetration_depth`: Foot ground contact (0.003m default)
- `pos_x/y/z_min/max`: Position limits
- `orn_x/y/z_min/max`: Orientation limits

### 6.2 Frame Parameters

If your frame dimensions differ, update:
```bash
nano ~/zuko/quad_ws/src/quad_main/config/frame_parameters.yaml
```

### 6.3 Servo Parameters

If you need to adjust servo calibration:
```bash
cd ~/zuko/quad_ws/src/quad_motors/quad_motors/
./servo_calibration.py
```

## Step 7: Auto-Start Configuration (Optional)

To automatically start the robot on boot:

1. Create a systemd service (advanced) or
2. Add to `.bashrc`:
```bash
echo "cd ~/zuko/quad_ws && source install/local_setup.bash && ros2 launch quad_main quad_live.launch.py" >> ~/.bashrc
```

**Note:** Auto-start is not recommended until you've thoroughly tested the robot.

## Troubleshooting

### Servos Not Moving

1. **Check I2C connection:**
   ```bash
   sudo i2cdetect -y 1
   # Should show 0x40
   ```

2. **Check I2C permissions:**
   ```bash
   sudo chmod a+rw /dev/i2c-*
   ```

3. **Verify servo calibration file exists:**
   ```bash
   ls ~/zuko/quad_ws/src/quad_motors/config/servo_parameters.yaml
   ```

4. **Check servo power supply:**
   - Ensure adequate current capacity (12 servos can draw significant current)
   - Verify voltage is correct (typically 5-6V)

### Controller Not Detected

1. **Check device:**
   ```bash
   ls /dev/input/js*
   ```

2. **Try different joystick number:**
   ```bash
   ros2 run quad_gamepad quad_gamepad --ros-args -p joystick_number:=1
   ```

3. **For wireless:** Re-pair controller

### Robot Moves Incorrectly

1. **Verify pin mapping:** Check `servo_parameters.yaml` matches your wiring
2. **Check servo calibration:** Run calibration script again
3. **Verify joint limits:** Ensure min/max degrees are correct
4. **Check invert_direction:** May need to invert some servos

### ROS2 Nodes Not Starting

1. **Rebuild workspace:**
   ```bash
   cd ~/zuko/quad_ws
   colcon build
   source install/local_setup.bash
   ```

2. **Check for errors:**
   ```bash
   ros2 node list
   ros2 topic list
   ```

## Safety Reminders

- ⚠️ **Always detach legs during servo calibration**
- ⚠️ **Start with slow movements and low step_velocity**
- ⚠️ **Test on a soft surface first (carpet, foam)**
- ⚠️ **Keep hands clear during first movements**
- ⚠️ **Monitor servo temperatures during extended use**
- ⚠️ **Ensure adequate power supply capacity**

## Next Steps

After successful setup:

1. Practice basic movements with controller
2. Tune motion parameters for stability
3. Test different gaits and speeds
4. Experiment with orientation and position control
5. Review PROJECT_OVERVIEW.md for advanced features

## Quick Reference

### Essential Commands

```bash
# Build workspace
cd ~/zuko/quad_ws
colcon build

# Source workspace
source install/local_setup.bash

# Launch robot
ros2 launch quad_main quad_live.launch.py

# Calibrate servos
cd src/quad_motors/quad_motors/
./servo_calibration.py

# Check I2C
sudo i2cdetect -y 1

# Monitor topics
ros2 topic echo /joint_angles
ros2 topic echo /joy
```

### File Locations

- Servo calibration: `quad_ws/src/quad_motors/config/servo_parameters.yaml`
- Motion parameters: `quad_ws/src/quad_main/config/motion_parameters.yaml`
- Frame parameters: `quad_ws/src/quad_main/config/frame_parameters.yaml`
- Pin mapping reference: `quad_ws/src/quad_motors/PIN_MAPPING.md`

## Support

- Project Overview: See `PROJECT_OVERVIEW.md`
- Installation Guide: See `INSTALL_RPI.md`
- Pin Mapping: See `quad_ws/src/quad_motors/PIN_MAPPING.md`
- Development Notes: See `quad_ws/dev-notes/`

---

**Good luck with your Zuko robot! 🐕**
