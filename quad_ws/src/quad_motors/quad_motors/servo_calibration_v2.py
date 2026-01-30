#!/usr/bin/env python3
'''
    Enhanced Servo Calibration Tool v2.0 for DS3218SSG Pro Servos
    
    Features:
    - Degree-based movement (instead of raw pulse widths)
    - Interactive angle adjustments (+5, -10, etc.)
    - Visual position indicator
    - Sweep mode to find mechanical limits
    - Group calibration (all hips, femurs, or tibias)
    - Test routines for verification
    - DS3218SSG Pro optimized defaults
    
    Does not require ROS to execute.
'''

from time import sleep
import yaml
import sys
import os
from src.PCA9685Servos import PCA9685Servos

# DS3218SSG Pro Specifications
DS3218_PULSE_PER_DEGREE = 7.41  # (2500-500) / 270° = 7.41
DS3218_MIN_PULSE = 500
DS3218_MAX_PULSE = 2500
DS3218_CENTER_PULSE = 1500  # Nominal center

# Map joint indexes to PCA9685 pins
MAP_JOINT_TO_PIN = [4, 3, 1, 5, 2, 0, 10, 7, 9, 11, 6, 8]

# Joint groups for batch calibration
JOINT_GROUPS = {
    'hips': [0, 3, 6, 9],
    'femurs': [1, 4, 7, 10],
    'tibias': [2, 5, 8, 11],
    'front_left': [0, 1, 2],
    'front_right': [3, 4, 5],
    'back_left': [6, 7, 8],
    'back_right': [9, 10, 11],
    'all': list(range(12))
}

SERVO_NAMES = {
    0: "FL Hip", 1: "FL Femur", 2: "FL Tibia",
    3: "FR Hip", 4: "FR Femur", 5: "FR Tibia",
    6: "BL Hip", 7: "BL Femur", 8: "BL Tibia",
    9: "BR Hip", 10: "BR Femur", 11: "BR Tibia"
}

# Default limits for different joint types
DEFAULT_LIMITS = {
    'hip': {'min': -30, 'max': 30},
    'femur': {'min': 0, 'max': 90},
    'tibia': {'min': 0, 'max': 90}
}


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def clamp(num, min_val, max_val):
    return max(min(num, max_val), min_val)


def degrees_to_pulse(degrees, zero_pulse, pulse_per_deg, inverted):
    """Convert degrees to pulse width."""
    direction = -1 if inverted else 1
    pulse = zero_pulse + (degrees * pulse_per_deg * direction)
    return int(clamp(pulse, DS3218_MIN_PULSE, DS3218_MAX_PULSE))


def pulse_to_degrees(pulse, zero_pulse, pulse_per_deg, inverted):
    """Convert pulse width to degrees."""
    direction = -1 if inverted else 1
    return (pulse - zero_pulse) / (pulse_per_deg * direction)


def create_position_bar(current_deg, min_deg, max_deg, width=40):
    """Create ASCII progress bar showing servo position."""
    if max_deg == min_deg:
        pos = width // 2
    else:
        pos = int((current_deg - min_deg) / (max_deg - min_deg) * width)
    pos = clamp(pos, 0, width)
    
    bar = ['─'] * width
    zero_pos = int((0 - min_deg) / (max_deg - min_deg) * width) if max_deg != min_deg else width // 2
    zero_pos = clamp(zero_pos, 0, width - 1)
    bar[zero_pos] = '│'
    
    indicator = '●' if 0 <= pos < width else ('◄' if pos < 0 else '►')
    if 0 <= pos < width:
        bar[pos] = indicator
    
    return ''.join(bar)


def load_parameters(path):
    """Load parameters from YAML file."""
    default_params = {
        "zero_degrees_pulse_width": [1500] * 12,
        "pulse_width_per_degree": [DS3218_PULSE_PER_DEGREE] * 12,
        "invert_direction": [False, False, False, False, True, True,
                            False, False, False, False, True, True],
        "min_degrees": [-30, 0, 0, -30, 0, 0, -30, 0, 0, -30, 0, 0],
        "max_degrees": [30, 90, 90, 30, 90, 90, 30, 90, 90, 30, 90, 90],
        "map_joint_index_to_driver_pin": MAP_JOINT_TO_PIN
    }
    
    try:
        with open(path, 'r') as f:
            params = yaml.safe_load(f)
            # Merge with defaults for any missing keys
            for key in default_params:
                if key not in params:
                    params[key] = default_params[key]
            return params
    except FileNotFoundError:
        return default_params


def save_parameters(path, params):
    """Save parameters to YAML file."""
    with open(path, 'w', encoding='utf8') as f:
        yaml.dump(params, f, default_flow_style=False, allow_unicode=True)


def print_help():
    """Print help information."""
    clear_screen()
    print("╔════════════════════════════════════════════════════════════════╗")
    print("║        DS3218SSG Pro Servo Calibration Tool v2.0               ║")
    print("╠════════════════════════════════════════════════════════════════╣")
    print("║  MOVEMENT COMMANDS                                             ║")
    print("║    goto <deg>     Move to angle (e.g., 'goto 45')              ║")
    print("║    +<deg>/-<deg>  Adjust by degrees (e.g., '+5' or '-10')      ║")
    print("║    center         Move to 0 degrees                            ║")
    print("║    pulse <pw>     Set raw pulse width (500-2500)               ║")
    print("║                                                                 ║")
    print("║  CALIBRATION COMMANDS                                          ║")
    print("║    set_zero       Mark current position as 0 degrees           ║")
    print("║    zero <pw>      Set zero pulse width directly                ║")
    print("║    invert         Toggle direction inversion                   ║")
    print("║    min <deg>      Set minimum angle limit                      ║")
    print("║    max <deg>      Set maximum angle limit                      ║")
    print("║                                                                 ║")
    print("║  SELECTION COMMANDS                                            ║")
    print("║    select <n>     Select servo 0-11                            ║")
    print("║    next/prev      Select next/previous servo                   ║")
    print("║                                                                 ║")
    print("║  GROUP COMMANDS                                                ║")
    print("║    group <name>   Select group (hips/femurs/tibias/all)        ║")
    print("║    zero_all       Send all servos to their zero position       ║")
    print("║                                                                 ║")
    print("║  TEST COMMANDS                                                  ║")
    print("║    sweep          Sweep between limits (test range)            ║")
    print("║    test           Run movement test pattern                    ║")
    print("║    find_center    Slowly find mechanical center                ║")
    print("║                                                                 ║")
    print("║  OTHER                                                          ║")
    print("║    help           Show this help                               ║")
    print("║    status         Show current calibration status              ║")
    print("║    exit           Save and exit                                ║")
    print("╚════════════════════════════════════════════════════════════════╝")
    input("\nPress Enter to continue...")


def print_status(selected, current_angles, params):
    """Print current calibration status for all servos."""
    clear_screen()
    
    GREEN = "\033[0;32m"
    YELLOW = "\033[0;33m"
    WHITE = "\033[0;37m"
    RESET = "\033[0m"
    
    print("╔═══════════════════════════════════════════════════════════════════════════════╗")
    print("║         DS3218SSG Pro Servo Calibration v2.0                                  ║")
    print("╠═══╦════════════╦═══════╦═══════╦═══════╦═══════╦═══════╦═══════════════════════╣")
    print("║ # ║   Servo    ║ Angle ║ Zero  ║ Ratio ║  Min  ║  Max  ║      Position         ║")
    print("╠═══╬════════════╬═══════╬═══════╬═══════╬═══════╬═══════╬═══════════════════════╣")
    
    for i in range(12):
        color = GREEN if i == selected else WHITE
        angle = current_angles[i]
        zero_pw = params['zero_degrees_pulse_width'][i]
        ratio = params['pulse_width_per_degree'][i]
        min_deg = params['min_degrees'][i]
        max_deg = params['max_degrees'][i]
        inverted = params['invert_direction'][i]
        
        inv_marker = "←" if inverted else " "
        name = SERVO_NAMES[i]
        pos_bar = create_position_bar(angle, min_deg, max_deg, 20)
        
        print(f"{color}║{i:>2} ║ {name:<10} ║{angle:>6.1f}°║ {zero_pw:>5} ║ {ratio:>5.2f} ║{min_deg:>6.0f}°║{max_deg:>6.0f}°║ {pos_bar} {inv_marker}║{RESET}")
    
    print("╚═══╩════════════╩═══════╩═══════╩═══════╩═══════╩═══════╩═══════════════════════╝")
    print()
    print(f"  Selected: {selected} ({SERVO_NAMES[selected]})  │  Type 'help' for commands  │  '←' = inverted")
    print()


def warn_user():
    """Display safety warning."""
    clear_screen()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║                      ⚠️  WARNING  ⚠️                          ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  Servo calibration may move beyond mechanical limits!        ║")
    print("║                                                              ║")
    print("║  BEFORE PROCEEDING:                                          ║")
    print("║  • Detach all legs from the robot frame                      ║")
    print("║  • Ensure servos can rotate freely                           ║")
    print("║  • Have servo power supply ready                             ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print()
    response = input("  Are you ready to proceed? (y/n): ").lower().strip()
    if not response.startswith('y'):
        print("  Exiting...")
        sys.exit(0)


def sweep_servo(driver, servo_idx, params, steps=20, delay=0.1):
    """Sweep servo between its limits to test range."""
    print(f"\n  Sweeping {SERVO_NAMES[servo_idx]}...")
    
    pin = params['map_joint_index_to_driver_pin'][servo_idx]
    zero_pw = params['zero_degrees_pulse_width'][servo_idx]
    ratio = params['pulse_width_per_degree'][servo_idx]
    inverted = params['invert_direction'][servo_idx]
    min_deg = params['min_degrees'][servo_idx]
    max_deg = params['max_degrees'][servo_idx]
    
    # Sweep from min to max
    for i in range(steps + 1):
        angle = min_deg + (max_deg - min_deg) * i / steps
        pw = degrees_to_pulse(angle, zero_pw, ratio, inverted)
        driver.set_pulse_width(pin, pw)
        print(f"\r  Angle: {angle:>6.1f}°  Pulse: {pw:>4}", end='', flush=True)
        sleep(delay)
    
    # Sweep back to min
    for i in range(steps + 1):
        angle = max_deg - (max_deg - min_deg) * i / steps
        pw = degrees_to_pulse(angle, zero_pw, ratio, inverted)
        driver.set_pulse_width(pin, pw)
        print(f"\r  Angle: {angle:>6.1f}°  Pulse: {pw:>4}", end='', flush=True)
        sleep(delay)
    
    # Return to zero
    pw = degrees_to_pulse(0, zero_pw, ratio, inverted)
    driver.set_pulse_width(pin, pw)
    print(f"\r  Sweep complete! Returned to 0°              ")


def find_center_routine(driver, servo_idx, params):
    """Interactive routine to find servo center position."""
    print(f"\n  Finding center for {SERVO_NAMES[servo_idx]}...")
    print("  The servo will move slowly. Press Enter when at center, or 'q' to cancel.")
    
    pin = params['map_joint_index_to_driver_pin'][servo_idx]
    
    import select
    import tty
    import termios
    
    # Start at current zero pulse
    current_pw = params['zero_degrees_pulse_width'][servo_idx]
    
    print(f"\n  Current zero pulse: {current_pw}")
    print("  Use '+' and '-' to adjust by 10, '<' and '>' by 1")
    print("  Press Enter to confirm, 'q' to cancel")
    
    while True:
        driver.set_pulse_width(pin, current_pw)
        print(f"\r  Pulse Width: {current_pw:>4}  ", end='', flush=True)
        
        try:
            cmd = input()
            if cmd == '' or cmd.lower() == 'enter':
                params['zero_degrees_pulse_width'][servo_idx] = current_pw
                print(f"\n  ✓ Zero position set to {current_pw}")
                return current_pw
            elif cmd == 'q':
                print("\n  Cancelled")
                return None
            elif cmd == '+':
                current_pw = clamp(current_pw + 10, DS3218_MIN_PULSE, DS3218_MAX_PULSE)
            elif cmd == '-':
                current_pw = clamp(current_pw - 10, DS3218_MIN_PULSE, DS3218_MAX_PULSE)
            elif cmd == '>':
                current_pw = clamp(current_pw + 1, DS3218_MIN_PULSE, DS3218_MAX_PULSE)
            elif cmd == '<':
                current_pw = clamp(current_pw - 1, DS3218_MIN_PULSE, DS3218_MAX_PULSE)
            elif cmd.lstrip('-').isdigit():
                current_pw = clamp(int(cmd), DS3218_MIN_PULSE, DS3218_MAX_PULSE)
        except KeyboardInterrupt:
            print("\n  Cancelled")
            return None


def test_movement(driver, servo_idx, params):
    """Run a test movement pattern."""
    print(f"\n  Testing {SERVO_NAMES[servo_idx]}...")
    
    pin = params['map_joint_index_to_driver_pin'][servo_idx]
    zero_pw = params['zero_degrees_pulse_width'][servo_idx]
    ratio = params['pulse_width_per_degree'][servo_idx]
    inverted = params['invert_direction'][servo_idx]
    
    # Test pattern: 0 -> +15 -> -15 -> +30 -> -30 -> 0
    pattern = [0, 15, -15, 30, -30, 45, -45, 0]
    
    for angle in pattern:
        pw = degrees_to_pulse(angle, zero_pw, ratio, inverted)
        driver.set_pulse_width(pin, pw)
        print(f"  → {angle:>3}°", end='', flush=True)
        sleep(0.5)
    
    print("\n  ✓ Test complete!")


def main():
    warn_user()
    
    config_path = "../config/servo_parameters.yaml"
    params = load_parameters(config_path)
    
    # Current state
    selected = 0
    current_angles = [0.0] * 12  # Track current angle for each servo
    current_pulses = list(params['zero_degrees_pulse_width'])  # Track current pulse
    
    # Initialize servo driver
    try:
        driver = PCA9685Servos(1, 0x40)
    except FileNotFoundError:
        clear_screen()
        print("╔══════════════════════════════════════════════════════════════╗")
        print("║  ERROR: I2C device not found!                                ║")
        print("║                                                              ║")
        print("║  Check that:                                                 ║")
        print("║  • PCA9685 is connected properly                             ║")
        print("║  • I2C is enabled (sudo raspi-config)                        ║")
        print("║  • I2C permissions are set (sudo chmod a+rw /dev/i2c-*)      ║")
        print("╚══════════════════════════════════════════════════════════════╝")
        sys.exit(1)
    
    # Set all servos to zero position on startup
    print("\n  Initializing all servos to zero position...")
    for i in range(12):
        pin = params['map_joint_index_to_driver_pin'][i]
        pw = params['zero_degrees_pulse_width'][i]
        driver.set_pulse_width(pin, pw)
        current_pulses[i] = pw
        current_angles[i] = 0.0
        sleep(0.05)
    
    print("  ✓ All servos initialized!")
    sleep(0.5)
    
    # Main loop
    while True:
        print_status(selected, current_angles, params)
        
        try:
            cmd_input = input("  > ").strip().lower()
        except KeyboardInterrupt:
            break
        except EOFError:
            break
        
        if not cmd_input:
            continue
        
        parts = cmd_input.split()
        cmd = parts[0]
        args = parts[1:] if len(parts) > 1 else []
        
        pin = params['map_joint_index_to_driver_pin'][selected]
        zero_pw = params['zero_degrees_pulse_width'][selected]
        ratio = params['pulse_width_per_degree'][selected]
        inverted = params['invert_direction'][selected]
        
        # Movement commands
        if cmd.startswith('+') or cmd.startswith('-'):
            try:
                delta = float(cmd)
                new_angle = current_angles[selected] + delta
                new_angle = clamp(new_angle, -135, 135)
                pw = degrees_to_pulse(new_angle, zero_pw, ratio, inverted)
                driver.set_pulse_width(pin, pw)
                current_angles[selected] = new_angle
                current_pulses[selected] = pw
            except ValueError:
                print("  Invalid angle adjustment")
                sleep(0.5)
        
        elif cmd == 'goto':
            if args:
                try:
                    angle = float(args[0])
                    angle = clamp(angle, -135, 135)
                    pw = degrees_to_pulse(angle, zero_pw, ratio, inverted)
                    driver.set_pulse_width(pin, pw)
                    current_angles[selected] = angle
                    current_pulses[selected] = pw
                except ValueError:
                    print("  Invalid angle")
                    sleep(0.5)
        
        elif cmd == 'center':
            pw = degrees_to_pulse(0, zero_pw, ratio, inverted)
            driver.set_pulse_width(pin, pw)
            current_angles[selected] = 0
            current_pulses[selected] = pw
        
        elif cmd == 'pulse':
            if args:
                try:
                    pw = int(args[0])
                    pw = clamp(pw, DS3218_MIN_PULSE, DS3218_MAX_PULSE)
                    driver.set_pulse_width(pin, pw)
                    current_pulses[selected] = pw
                    current_angles[selected] = pulse_to_degrees(pw, zero_pw, ratio, inverted)
                except ValueError:
                    print("  Invalid pulse width")
                    sleep(0.5)
        
        # Calibration commands
        elif cmd == 'set_zero':
            # Set current position as zero
            params['zero_degrees_pulse_width'][selected] = current_pulses[selected]
            current_angles[selected] = 0
            print(f"  ✓ Zero position set to {current_pulses[selected]}")
            save_parameters(config_path, params)
            sleep(0.5)
        
        elif cmd == 'zero':
            if args:
                try:
                    pw = int(args[0])
                    pw = clamp(pw, DS3218_MIN_PULSE, DS3218_MAX_PULSE)
                    params['zero_degrees_pulse_width'][selected] = pw
                    save_parameters(config_path, params)
                    # Recalculate current angle with new zero
                    current_angles[selected] = pulse_to_degrees(current_pulses[selected], pw, ratio, inverted)
                except ValueError:
                    print("  Invalid pulse width")
                    sleep(0.5)
        
        elif cmd == 'invert':
            params['invert_direction'][selected] = not params['invert_direction'][selected]
            save_parameters(config_path, params)
            print(f"  ✓ Direction {'inverted' if params['invert_direction'][selected] else 'normal'}")
            sleep(0.5)
        
        elif cmd == 'min':
            if args:
                try:
                    val = float(args[0])
                    params['min_degrees'][selected] = clamp(val, -180, 180)
                    save_parameters(config_path, params)
                except ValueError:
                    print("  Invalid angle")
                    sleep(0.5)
        
        elif cmd == 'max':
            if args:
                try:
                    val = float(args[0])
                    params['max_degrees'][selected] = clamp(val, -180, 180)
                    save_parameters(config_path, params)
                except ValueError:
                    print("  Invalid angle")
                    sleep(0.5)
        
        # Selection commands
        elif cmd == 'select':
            if args:
                try:
                    selected = clamp(int(args[0]), 0, 11)
                except ValueError:
                    print("  Invalid servo number")
                    sleep(0.5)
        
        elif cmd == 'next':
            selected = (selected + 1) % 12
        
        elif cmd == 'prev':
            selected = (selected - 1) % 12
        
        # Group commands
        elif cmd == 'group':
            if args and args[0] in JOINT_GROUPS:
                group = JOINT_GROUPS[args[0]]
                print(f"  Moving group {args[0]} to center...")
                for idx in group:
                    p = params['map_joint_index_to_driver_pin'][idx]
                    z = params['zero_degrees_pulse_width'][idx]
                    r = params['pulse_width_per_degree'][idx]
                    inv = params['invert_direction'][idx]
                    pw = degrees_to_pulse(0, z, r, inv)
                    driver.set_pulse_width(p, pw)
                    current_angles[idx] = 0
                    current_pulses[idx] = pw
                    sleep(0.05)
                print(f"  ✓ Group {args[0]} centered")
                sleep(0.5)
            else:
                print(f"  Available groups: {', '.join(JOINT_GROUPS.keys())}")
                sleep(1)
        
        elif cmd == 'zero_all':
            print("  Setting all servos to zero...")
            for i in range(12):
                p = params['map_joint_index_to_driver_pin'][i]
                pw = params['zero_degrees_pulse_width'][i]
                driver.set_pulse_width(p, pw)
                current_pulses[i] = pw
                current_angles[i] = 0
                sleep(0.05)
            print("  ✓ All servos at zero")
            sleep(0.5)
        
        # Test commands
        elif cmd == 'sweep':
            sweep_servo(driver, selected, params)
            input("  Press Enter to continue...")
        
        elif cmd == 'test':
            test_movement(driver, selected, params)
            input("  Press Enter to continue...")
        
        elif cmd == 'find_center':
            find_center_routine(driver, selected, params)
            save_parameters(config_path, params)
            input("  Press Enter to continue...")
        
        # Other commands
        elif cmd == 'help':
            print_help()
        
        elif cmd == 'status':
            print_status(selected, current_angles, params)
            input("  Press Enter to continue...")
        
        elif cmd == 'exit':
            save_parameters(config_path, params)
            print("\n  ✓ Parameters saved!")
            break
        
        else:
            # Try to parse as a direct angle
            try:
                angle = float(cmd)
                angle = clamp(angle, -135, 135)
                pw = degrees_to_pulse(angle, zero_pw, ratio, inverted)
                driver.set_pulse_width(pin, pw)
                current_angles[selected] = angle
                current_pulses[selected] = pw
            except ValueError:
                print(f"  Unknown command: {cmd}. Type 'help' for commands.")
                sleep(0.5)
    
    print("\n  Goodbye! 🐕")


if __name__ == '__main__':
    main()
