# Zuko Robot - Raspberry Pi Installation Guide

This guide will walk you through installing and setting up the Zuko quadruped robot software on a Raspberry Pi 4.

## Prerequisites

- Raspberry Pi 4 (4GB or 8GB recommended)
- MicroSD card (32GB or larger, Class 10 recommended)
- USB-C power supply (5V, 3A minimum)
- Network connection (Ethernet or Wi-Fi)
- Access to the robot hardware (PCA9685 servo driver, servos, etc.)

## Step 1: Install Ubuntu 20.04 LTS

### Using Raspberry Pi Imager

1. Download and install [Raspberry Pi Imager](https://www.raspberrypi.org/software/)
2. Download Ubuntu 20.04.4 LTS Server 64-bit image:
   - Visit: https://ubuntu.com/download/raspberry-pi
   - Download Ubuntu 20.04.4 LTS Server 64-bit
3. Write the image to your microSD card using Raspberry Pi Imager
4. Insert the microSD card into your Raspberry Pi 4
5. Connect power, keyboard, monitor, and network (or use headless setup)

### First Boot

1. Boot the Raspberry Pi
2. Default credentials:
   - **Username:** `ubuntu`
   - **Password:** `ubuntu`
3. **IMPORTANT:** Change the password immediately:
   ```bash
   passwd
   ```

## Step 2: Configure Network (Wi-Fi Setup)

If you need to configure Wi-Fi:

1. Edit the netplan configuration:
   ```bash
   sudo nano /etc/netplan/50-cloud-init.yaml
   ```

2. Add Wi-Fi configuration (use spaces, not tabs, keep quotation marks):
   ```yaml
   version: 2
   ethernets:
     eth0:
       dhcp4: true
       optional: true
   wifis:
     wlan0:
       dhcp4: true
       optional: true
       access-points:
         YOUR_WIFI_SSID:
           password: "your_wifi_password"
   ```

3. Disable cloud-init networking:
   ```bash
   sudo touch /etc/cloud/cloud.cfg.d/99-custom-networking.cfg
   sudo nano /etc/cloud/cloud.cfg.d/99-custom-networking.cfg
   ```
   Add this line:
   ```
   network: {config: disabled}
   ```

4. Apply the network configuration:
   ```bash
   sudo netplan apply
   ```

5. Verify connection:
   ```bash
   ip a
   ```

### SSH Access (Optional but Recommended)

For easier development, you can SSH into the Raspberry Pi:

1. Find the Pi's IP address:
   ```bash
   hostname -I
   ```

2. From your development machine, connect via SSH:
   ```bash
   ssh ubuntu@<PI_IP_ADDRESS>
   ```

**Tip:** Use PuTTY (Windows) or terminal (Linux/Mac) for SSH access with copy/paste capability.

## Step 3: Install ROS2 Foxy

Follow the official ROS2 installation guide for Raspberry Pi:

1. Update package lists:
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

2. Install ROS2 Foxy base:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update
   sudo apt install ros-foxy-ros-base -y
   ```

3. Install additional ROS2 tools:
   ```bash
   sudo apt install python3-argcomplete -y
   ```

4. Source ROS2:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

5. Verify installation:
   ```bash
   printenv | grep ROS
   ```

## Step 4: Install Python Dependencies

1. Install pip if not already installed:
   ```bash
   sudo apt install python3-pip -y
   ```

2. Install required Python packages:
   ```bash
   pip3 install smbus2
   pip3 install numpy
   pip3 install pyyaml
   ```

## Step 5: Clone and Build the Repository

1. Clone the Zuko repository:
   ```bash
   cd ~
   git clone https://github.com/reubenstr/zuko.git
   ```

   **OR** if you have the repository locally, copy it to the Pi:
   ```bash
   scp -r /path/to/rex-zuko-try ubuntu@<PI_IP>:/home/ubuntu/zuko
   ```

2. Navigate to the workspace:
   ```bash
   cd ~/zuko/quad_ws
   ```

3. Source ROS2:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

4. Install colcon build tools:
   ```bash
   sudo apt install python3-colcon-common-extensions -y
   ```

5. Build the workspace:
   ```bash
   colcon build
   ```

   **Note:** Some packages may show warnings if parameters are missing, but they should still build.

6. Source the workspace:
   ```bash
   source install/local_setup.bash
   ```

## Step 6: Configure Auto-Sourcing (Optional but Recommended)

Add ROS2 and workspace sourcing to your `.bashrc` for automatic setup on login:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "cd ~/zuko/quad_ws" >> ~/.bashrc
echo "source install/local_setup.bash" >> ~/.bashrc
```

Reload your bash configuration:
```bash
source ~/.bashrc
```

## Step 7: Configure I2C Access

The PCA9685 servo driver communicates via I2C. You need to grant access to I2C devices:

1. Add I2C permission to `.bashrc`:
   ```bash
   echo "sudo chmod a+rw /dev/i2c-*" >> ~/.bashrc
   ```

2. Install I2C tools (for debugging):
   ```bash
   sudo apt-get install i2c-tools -y
   ```

3. Enable I2C interface (if not already enabled):
   ```bash
   sudo raspi-config
   ```
   Navigate to: **Interfacing Options → I2C → Enable**

4. Verify I2C is working:
   ```bash
   sudo i2cdetect -y 1
   ```
   You should see the PCA9685 at address `0x40` (or your configured address).

## Step 8: Setup Bluetooth for Wireless Controller (Optional)

If you want to use a wireless PS4 controller:

1. Install Bluetooth support:
   ```bash
   sudo apt install pi-bluetooth -y
   sudo reboot
   ```

2. After reboot, pair the controller:
   ```bash
   sudo bluetoothctl
   ```

3. In the bluetoothctl terminal:
   ```bash
   # Scan for devices
   scan on
   # Wait for your controller to appear, then:
   scan off
   ```

4. Put your PS4 controller in pairing mode:
   - Press and hold **Share** and **PS** buttons simultaneously
   - The LED will start flashing

5. Connect to the controller (replace with your MAC address):
   ```bash
   connect 84:30:95:48:0F:3C
   ```

6. Trust the device for automatic connection:
   ```bash
   trust 84:30:95:48:0F:3C
   ```

7. Exit bluetoothctl:
   ```bash
   exit
   ```

8. Verify controller is connected:
   ```bash
   cat /proc/bus/input/devices
   ls /dev/input/by-id/
   ```

## Step 9: Calibrate Servos

**IMPORTANT:** You must calibrate the servos before running the robot!

1. Navigate to the servo calibration script:
   ```bash
   cd ~/zuko/quad_ws/src/quad_motors/quad_motors/
   ```

2. Make the script executable:
   ```bash
   chmod +x servo_calibration.py
   ```

3. Run the calibration:
   ```bash
   ./servo_calibration.py
   ```

4. Follow the on-screen instructions to calibrate each servo:
   - Set zero positions
   - Set minimum and maximum limits
   - The calibration will save to `servo_parameters.yaml`

5. Verify the calibration file was created:
   ```bash
   ls ~/zuko/quad_ws/src/quad_motors/config/servo_parameters.yaml
   ```

## Step 10: Verify Installation

1. Check ROS2 environment:
   ```bash
   printenv | grep ROS
   ```

2. List available packages:
   ```bash
   ros2 pkg list | grep quad
   ```

3. Test individual nodes (without hardware):
   ```bash
   # Test gamepad node (requires controller)
   ros2 run quad_gamepad quad_gamepad --ros-args -p joystick_number:=0
   ```

4. Check topics:
   ```bash
   ros2 topic list
   ```

## Step 11: Launch the Robot

### For Live Robot Operation

1. Ensure all hardware is connected:
   - PCA9685 servo driver (I2C address 0x40)
   - 12 servos connected
   - Power supply connected
   - PS4 controller connected (wired or wireless)

2. Launch the robot:
   ```bash
   cd ~/zuko/quad_ws
   source install/local_setup.bash
   ros2 launch quad_main quad_live.launch.py
   ```

### For Testing (Simulation)

If you want to test without hardware:

1. Install simulation dependencies (on desktop, not RPi):
   ```bash
   pip3 install gym pybullet
   ```

2. Launch with simulation:
   ```bash
   ros2 launch quad_main quad_all.launch.py
   ```

## Troubleshooting

### I2C Permission Denied

```bash
# Add user to i2c group
sudo usermod -aG i2c $USER
# Log out and back in, or:
newgrp i2c
```

Or use the workaround in `.bashrc`:
```bash
sudo chmod a+rw /dev/i2c-*
```

### Controller Not Detected

1. Check if controller is connected:
   ```bash
   ls /dev/input/js*
   ```

2. Test controller:
   ```bash
   sudo apt install jstest-gtk -y
   jstest-gtk
   ```

3. Verify joystick number in launch file or command:
   ```bash
   ros2 run quad_gamepad quad_gamepad --ros-args -p joystick_number:=0
   ```

### Servo Calibration File Missing

If you get an error about missing servo parameters:

1. Run the calibration script:
   ```bash
   cd ~/zuko/quad_ws/src/quad_motors/quad_motors/
   ./servo_calibration.py
   ```

2. Verify the file exists:
   ```bash
   ls ~/zuko/quad_ws/src/quad_motors/config/servo_parameters.yaml
   ```

### Build Errors

1. Ensure all dependencies are installed:
   ```bash
   sudo apt update
   sudo apt install python3-pip python3-colcon-common-extensions -y
   pip3 install smbus2 numpy pyyaml
   ```

2. Clean and rebuild:
   ```bash
   cd ~/zuko/quad_ws
   rm -rf build install log
   colcon build
   ```

### ROS2 Not Found

If ROS2 commands don't work:

1. Source ROS2:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

2. Verify installation:
   ```bash
   which ros2
   ```

3. Check if ROS2 is in your PATH:
   ```bash
   echo $ROS_DISTRO
   ```

## Useful Commands

### ROS2 Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /joy
ros2 topic echo /joint_angles

# View node information
ros2 node info /joy_subscriber_node

# View topic information
ros2 topic info /joint_angles
```

### Building Specific Packages

```bash
# Build a single package
colcon build --packages-select quad_main

# Build multiple packages
colcon build --packages-select quad_main quad_motors

# Build and run
colcon build --packages-select quad_gamepad && ros2 run quad_gamepad quad_gamepad
```

### I2C Debugging

```bash
# Scan for I2C devices
sudo i2cdetect -y 1

# Dump registers from PCA9685 (address 0x40)
i2cdump -y 1 0x40
```

## Next Steps

1. **Calibrate Servos:** Run the calibration script for your specific hardware
2. **Test Controller:** Verify gamepad input is working
3. **Test Motors:** Run individual motor tests
4. **Tune Parameters:** Adjust `motion_parameters.yaml` for your robot
5. **Practice:** Start with slow movements and gradually increase speed

## Additional Resources

- **ROS2 Documentation:** https://docs.ros.org/en/foxy/
- **Project Overview:** See `PROJECT_OVERVIEW.md`
- **Development Notes:** See `quad_ws/dev-notes/`
- **ROS2 Commands Reference:** See `quad_ws/dev-notes/ros2-commands.txt`

## Support

For issues and questions:
- Check the project README.md
- Review development notes in `quad_ws/dev-notes/`
- Check ROS2 documentation
- Join the SpotMicroAI Discord: https://discord.gg/m5fsENhT

---

**Note:** This installation guide is based on Ubuntu 20.04 LTS and ROS2 Foxy. If you're using a different version, adjust the commands accordingly.

**Last Updated:** Based on project structure and setup notes
