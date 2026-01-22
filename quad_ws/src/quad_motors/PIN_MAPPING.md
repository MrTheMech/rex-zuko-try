# PCA9685 Pin Mapping Reference

This document describes the mapping between joint indices and PCA9685 hardware pins for the Zuko quadruped robot.

## Pin Layout

| Pin | Name | Description | Joint Index |
|-----|------|-------------|-------------|
| 0   | FRT  | Front Right Tibia (lower leg) | 5 |
| 1   | FLT  | Front Left Tibia (lower leg) | 2 |
| 2   | FRF  | Front Right Femur (upper leg) | 4 |
| 3   | FLF  | Front Left Femur (upper leg) | 1 |
| 4   | FL   | Front Left Hip | 0 |
| 5   | FR   | Front Right Hip | 3 |
| 6   | BRF  | Back Right Femur (upper leg) | 10 |
| 7   | BLF  | Back Left Femur (upper leg) | 7 |
| 8   | BRT  | Back Right Tibia (lower leg) | 11 |
| 9   | BLT  | Back Left Tibia (lower leg) | 8 |
| 10  | BL   | Back Left Hip | 6 |
| 11  | BR   | Back Right Hip | 9 |

## Naming Convention

- **1st digit:** F (Front) or B (Back)
- **2nd digit:** L (Left) or R (Right)
- **3rd digit:** F (Femur/top bone), T (Tibia/lower bone), or nothing (Hip)

## Joint Index Order

The robot has 12 joints total, organized by leg:

- **Joints 0-2:** Front Left (FL) - Hip, Femur, Tibia
- **Joints 3-5:** Front Right (FR) - Hip, Femur, Tibia
- **Joints 6-8:** Back Left (BL) - Hip, Femur, Tibia
- **Joints 9-11:** Back Right (BR) - Hip, Femur, Tibia

## Mapping Array

The mapping from joint index to PCA9685 pin is defined in `servo_parameters.yaml`:

```yaml
map_joint_index_to_driver_pin:
- 4   # Joint 0: FL Hip -> PIN4
- 3   # Joint 1: FL Femur -> PIN3
- 1   # Joint 2: FL Tibia -> PIN1
- 5   # Joint 3: FR Hip -> PIN5
- 2   # Joint 4: FR Femur -> PIN2
- 0   # Joint 5: FR Tibia -> PIN0
- 10  # Joint 6: BL Hip -> PIN10
- 7   # Joint 7: BL Femur -> PIN7
- 9   # Joint 8: BL Tibia -> PIN9
- 11  # Joint 9: BR Hip -> PIN11
- 6   # Joint 10: BR Femur -> PIN6
- 8   # Joint 11: BR Tibia -> PIN8
```

## Usage

The `ServoController` class automatically uses this mapping when setting servo angles. The mapping is loaded from the `servo_parameters.yaml` configuration file.

When calibrating servos using `servo_calibration.py`, the joint names will display with their corresponding pin numbers for easy reference.
