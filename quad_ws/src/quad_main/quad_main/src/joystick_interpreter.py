
'''
    PS5 Controller Input Interpreter for Quadruped Robot
    
    Converts joystick axes and button input into motion parameters for robot control.
    
    CONTROL SCHEME:
    ===============
    
    LEFT ANALOG STICK (Movement - JOG Mode):
        Up/Down   = Walk Forward/Backward
        Left/Right = Strafe Left/Right
    
    RIGHT ANALOG STICK (Always Active):
        X-axis = Turn rate (yaw) while walking
        Y-axis = Body height adjustment
    
    FACE BUTTONS (Direct Mode Selection):
        Cross (X)    = STAND mode - Default standing position
        Circle (O)   = SIT mode - Sitting/rest position
        Triangle (△) = POSE mode - Body orientation control
        Square (□)   = JOG mode - Walking/jogging mode
    
    L1 BUTTON:
        Press to cycle through modes: STAND → SIT → POSE → JOG → STAND...
    
    ANALOG STICKS IN POSE MODE:
        Left Stick X  = Roll (body tilt left/right)
        Left Stick Y  = Pitch (body tilt forward/back)
        Right Stick X = Yaw (body rotation)
        Right Stick Y = Height (body up/down)
    
    D-PAD (Reserved for future use)
    
    Button map derived from Controllers.py in quad_gamepad/quad_gamepad/src
'''

import copy
import numpy as np
from .motion_inputs import MotionInputs, MotionState
from .gamepad_map import AxesMap, ButtonMap


class JoystickInterpreter():
    """Interprets PS5 controller input into robot motion commands"""
    
    # Mode cycle order for L1 button
    MODE_CYCLE = [MotionState.STAND, MotionState.SIT, MotionState.POSE, MotionState.JOG]
    
    # Deadzone for analog sticks (helps prevent drift)
    STICK_DEADZONE = 0.1
    
    def __init__(self, motion_parameters):  
        self.motion_parameters = motion_parameters      
        self.motion_inputs = MotionInputs()
        
        # Button release flags to prevent repeated triggers
        self.l1_button_released = True
        self.cross_button_released = True
        self.circle_button_released = True
        self.triangle_button_released = True
        self.square_button_released = True
        
        # Track current mode index for L1 cycling
        self.current_mode_index = 0

    def map(self, n, in_min, in_max, out_min, out_max):
        """Map a value from one range to another"""
        return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def apply_deadzone(self, value, deadzone=None):
        """Apply deadzone to eliminate stick drift"""
        if deadzone is None:
            deadzone = self.STICK_DEADZONE
        if abs(value) < deadzone:
            return 0.0
        # Scale the remaining range to 0-1
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def _handle_mode_buttons(self, buttons):
        """Handle face buttons for direct mode selection and L1 for cycling"""
        
        # Cross (X) - STAND mode
        if buttons[ButtonMap.CROSS.value]:
            if self.cross_button_released:
                self.cross_button_released = False
                self.motion_inputs.motion_state = MotionState.STAND
                self.current_mode_index = 0
        else:
            self.cross_button_released = True
        
        # Circle (O) - SIT mode
        if buttons[ButtonMap.CIRCLE.value]:
            if self.circle_button_released:
                self.circle_button_released = False
                self.motion_inputs.motion_state = MotionState.SIT
                self.current_mode_index = 1
        else:
            self.circle_button_released = True
        
        # Triangle (△) - POSE mode
        if buttons[ButtonMap.TRIANGLE.value]:
            if self.triangle_button_released:
                self.triangle_button_released = False
                self.motion_inputs.motion_state = MotionState.POSE
                self.current_mode_index = 2
        else:
            self.triangle_button_released = True
        
        # Square (□) - JOG mode
        if buttons[ButtonMap.SQUARE.value]:
            if self.square_button_released:
                self.square_button_released = False
                self.motion_inputs.motion_state = MotionState.JOG
                self.current_mode_index = 3
        else:
            self.square_button_released = True
        
        # L1 Button - Cycle through modes
        if buttons[ButtonMap.L1.value]:
            if self.l1_button_released:
                self.l1_button_released = False
                self.current_mode_index = (self.current_mode_index + 1) % len(self.MODE_CYCLE)
                self.motion_inputs.motion_state = self.MODE_CYCLE[self.current_mode_index]
        else:
            self.l1_button_released = True

    def _handle_pose_mode(self, axes):
        """Handle analog sticks in POSE mode - control body orientation"""
        
        # Left Stick X = Roll (inverted)
        left_x = self.apply_deadzone(axes[AxesMap.LEFT_X.value])
        self.motion_inputs.orn[0] = -self.map(
            left_x, -1, 1, 
            self.motion_parameters['orn_x_min'], 
            self.motion_parameters['orn_x_max'])
        
        # Left Stick Y = Pitch
        left_y = self.apply_deadzone(axes[AxesMap.LEFT_Y.value])
        self.motion_inputs.orn[1] = self.map(
            left_y, -1, 1, 
            self.motion_parameters['orn_y_min'], 
            self.motion_parameters['orn_y_max'])
        
        # Right Stick X = Yaw
        right_x = self.apply_deadzone(axes[AxesMap.RIGHT_X.value])
        self.motion_inputs.orn[2] = self.map(
            right_x, -1, 1, 
            self.motion_parameters['orn_z_min'], 
            self.motion_parameters['orn_z_max'])
        
        # Right Stick Y = Height (inverted)
        right_y = self.apply_deadzone(axes[AxesMap.RIGHT_Y.value])
        self.motion_inputs.pos[2] = -self.map(
            right_y, -1, 1, 
            self.motion_parameters['pos_z_min'], 
            self.motion_parameters['pos_z_max'])

    def _handle_jog_mode(self, axes):
        """Handle analog sticks in JOG mode - control walking with left stick"""
        
        # LEFT STICK Y = Forward/Backward movement (step length)
        left_y = self.apply_deadzone(axes[AxesMap.LEFT_Y.value])
        self.motion_inputs.step_length = self.map(
            left_y, -1, 1, 
            self.motion_parameters['step_length_min'], 
            self.motion_parameters['step_length_max'])
        
        # LEFT STICK X = Strafe Left/Right (lateral fraction)
        left_x = self.apply_deadzone(axes[AxesMap.LEFT_X.value])
        self.motion_inputs.lateral_fraction = left_x  # -1 = left, 1 = right
        
        # RIGHT STICK X = Yaw rate (turning)
        right_x = self.apply_deadzone(axes[AxesMap.RIGHT_X.value])
        self.motion_inputs.yaw_rate = self.map(
            right_x, -1, 1, 
            self.motion_parameters['yaw_rate_min'], 
            self.motion_parameters['yaw_rate_max'])
        
        # RIGHT STICK Y = Height (inverted)
        right_y = self.apply_deadzone(axes[AxesMap.RIGHT_Y.value])
        self.motion_inputs.pos[2] = -self.map(
            right_y, -1, 1, 
            self.motion_parameters['pos_z_min'], 
            self.motion_parameters['pos_z_max'])

    def get_motion_inputs(self, axes, buttons):
        """Main method to process controller input and return motion commands"""
        
        # Invert Y axes (stick up = positive)
        axes = list(axes)  # Make a copy to avoid modifying original
        axes[AxesMap.LEFT_Y.value] = -axes[AxesMap.LEFT_Y.value]
        axes[AxesMap.RIGHT_Y.value] = -axes[AxesMap.RIGHT_Y.value]
        
        # Handle mode selection buttons
        self._handle_mode_buttons(buttons)
        
        # Handle analog sticks based on current mode
        if self.motion_inputs.motion_state == MotionState.POSE:
            self._handle_pose_mode(axes)
        elif self.motion_inputs.motion_state == MotionState.JOG:
            self._handle_jog_mode(axes)
        elif self.motion_inputs.motion_state == MotionState.STAND:
            # In STAND mode, reset everything to neutral standing position
            # Zero position from calibration = standing position
            self.motion_inputs.pos = np.array([0.0, 0.0, 0.0])
            self.motion_inputs.orn = np.array([0.0, 0.0, 0.0])
            self.motion_inputs.step_length = 0.0
            self.motion_inputs.yaw_rate = 0.0
            self.motion_inputs.lateral_fraction = 0.0
        elif self.motion_inputs.motion_state == MotionState.SIT:
            # In SIT mode, lower body significantly and pitch forward slightly
            # pos_z_min = -0.08 lowers the body, creating a sitting posture
            self.motion_inputs.pos = np.array([0.0, 0.0, self.motion_parameters['pos_z_min']])
            # Slight forward pitch for natural sit pose (optional)
            self.motion_inputs.orn = np.array([0.0, 0.1, 0.0])  # ~5.7° forward pitch
            self.motion_inputs.step_length = 0.0
            self.motion_inputs.yaw_rate = 0.0
            self.motion_inputs.lateral_fraction = 0.0

        return copy.deepcopy(self.motion_inputs)
