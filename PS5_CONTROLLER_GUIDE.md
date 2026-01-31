# PS5 Controller Guide for Zuko Quadruped Robot

This document describes how to use the PS5 (DualSense) controller to operate the robot.

> **Note:** The PS5 controller uses PS4 compatibility mode, so button mappings are the same as PS4.

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────┐
│                    PS5 CONTROLLER LAYOUT                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│    [L2] (unused)                    [R2] (unused)                  │
│    [L1] Cycle Modes             [R1] (unused)                  │
│                                                                 │
│    ┌───┐                              [△] POSE Mode             │
│    │ ↑ │ Forward                                                │
│ ┌──┼───┼──┐                      [□]            [○]             │
│ │← │   │ →│ Strafe              JOG              SIT            │
│ └──┼───┼──┘                                                     │
│    │ ↓ │ Backward                    [✕] STAND Mode             │
│    └───┘                                                        │
│                                                                 │
│   [Left Stick]                      [Right Stick]               │
│   POSE: Roll/Pitch                  POSE: Yaw/Height            │
│   JOG:  Speed                       JOG:  Turn/Height           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Robot Modes

The robot has **4 modes** that determine its behavior:

| Mode | Button | Description |
|------|--------|-------------|
| **STAND** | ✕ Cross | Default standing position. Robot stands upright with neutral body. |
| **SIT** | ○ Circle | Sitting/rest position. Robot lowers to a resting stance. |
| **POSE** | △ Triangle | Body orientation control. Use sticks to tilt and rotate body. |
| **JOG** | □ Square | Walking mode. Use D-Pad and sticks to move around. |

### Cycling Through Modes
- **L1 Button**: Press to cycle through modes in order: STAND → SIT → POSE → JOG → STAND...

---

## Movement Controls (JOG Mode)

Movement is controlled by the D-Pad using **hold-to-move** - the robot moves only while you hold the button.

| D-Pad Direction | Movement |
|-----------------|----------|
| **Up** | Walk Forward |
| **Down** | Walk Backward |
| **Left** | Strafe Left |
| **Right** | Strafe Right |

### Analog Stick Controls in JOG Mode

| Control | Action |
|---------|--------|
| **Left Stick Y** | Fine speed control (step length) |
| **Right Stick X** | Turn rate (yaw) while walking |
| **Right Stick Y** | Body height adjustment |

---

## Body Pose Controls (POSE Mode)

In POSE mode, the analog sticks control body orientation without walking.

| Control | Action |
|---------|--------|
| **Left Stick X** | Roll - Tilt body left/right |
| **Left Stick Y** | Pitch - Tilt body forward/backward |
| **Right Stick X** | Yaw - Rotate body left/right |
| **Right Stick Y** | Height - Raise/lower body |

---

## Button Reference

### Active Buttons

| Button | Function |
|--------|----------|
| **✕ Cross** | Switch to STAND mode |
| **○ Circle** | Switch to SIT mode |
| **△ Triangle** | Switch to POSE mode |
| **□ Square** | Switch to JOG mode |
| **L1** | Cycle through modes (STAND → SIT → POSE → JOG) |

### Unused Buttons (Reserved for Future)

| Button | Status |
|--------|--------|
| L2 | Unused |
| R1 | Unused |
| R2 | Unused |
| L3 (Left Stick Click) | Unused |
| R3 (Right Stick Click) | Unused |
| Share/Create | Unused |
| Options | Unused |
| PS Button | System button |
| Touchpad | Unused |

---

## Getting Started

1. **Connect your PS5 controller**
   - Via USB cable, or
   - Via Bluetooth (pair with Share + PS buttons)

2. **Launch the robot**
   ```bash
   ros2 launch quad_main quad_main.launch.py
   ```

3. **Start in STAND mode**
   - Robot begins in STAND mode by default
   - Press ✕ (Cross) to ensure you're in STAND

4. **Try POSE mode**
   - Press △ (Triangle) to enter POSE mode
   - Move the left stick to tilt the body
   - Move the right stick to rotate and adjust height

5. **Try JOG mode**
   - Press □ (Square) to enter JOG mode
   - Hold D-Pad Up to walk forward
   - Use right stick to turn while walking

6. **Rest the robot**
   - Press ○ (Circle) to enter SIT mode

---

## Troubleshooting

### Controller Not Detected
1. Check USB connection or Bluetooth pairing
2. Try a different joystick number:
   ```bash
   ros2 run quad_gamepad quad_gamepad --ros-args -p joystick_number:=1
   ```

### Robot Not Responding
1. Verify `/joy` topic is receiving data:
   ```bash
   ros2 topic echo /joy
   ```
2. Check that the correct mode is active

### Unexpected Behavior
1. Press ✕ (Cross) to reset to STAND mode
2. Release all sticks and D-Pad buttons

---

## Technical Details

### Files Modified
- `quad_main/src/motion_inputs.py` - Motion states and data class
- `quad_main/src/joystick_interpreter.py` - Controller input processing
- `quad_main/src/gamepad_map.py` - Button/axis index mapping

### Motion States (Enum)
```python
class MotionState(Enum):
    STAND = 1  # Default standing
    SIT = 2    # Sitting/rest position
    POSE = 3   # Body orientation control
    JOG = 4    # Walking mode
```

### Button Mapping (Index Values)
```python
class ButtonMap(Enum):
    CROSS = 0      # ✕
    CIRCLE = 1     # ○
    TRIANGLE = 2   # △
    SQUARE = 3     # □
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    SHARE = 8
    OPTIONS = 9
    PS = 10
    L3 = 11
    R3 = 12
```

---

*Last Updated: January 31, 2026*
