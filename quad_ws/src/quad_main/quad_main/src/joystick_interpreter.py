
'''
    PS5 Controller Input Interpreter for Quadruped Robot
    
    Converts joystick axes and button input into motion parameters for robot control.
    
    CONTROL SCHEME:
    ===============
    
    D-PAD (Hold-to-Move):
        Up    = Move Forward
        Down  = Move Backward
        Left  = Strafe Left
        Right = Strafe Right
    
    FACE BUTTONS (Direct Mode Selection):
        Cross (X)    = STAND mode - Default standing position
        Circle (O)   = SIT mode - Sitting/rest position
        Triangle (△) = POSE mode - Body orientation control
        Square (□)   = JOG mode - Walking/jogging mode
    
    L1 BUTTON:
        Press to cycle through modes: STAND → SIT → POSE → JOG → STAND...
    
    ANALOG STICKS (Context-Sensitive):
        In POSE mode:
            Left Stick X  = Roll (body tilt left/right)
            Left Stick Y  = Pitch (body tilt forward/back)
            Right Stick X = Yaw (body rotation)
            Right Stick Y = Height (body up/down)
        
        In JOG mode:
            Left Stick Y  = Speed/Step Length
            Right Stick X = Turn Rate (yaw rate)
            Right Stick Y = Height
    
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

    def _handle_dpad_movement(self, axes):
        """Handle D-Pad for hold-to-move directional control"""
        
        # D-Pad X axis: -1 = Left, 1 = Right, 0 = None
        # D-Pad Y axis: -1 = Up, 1 = Down, 0 = None
        dpad_x = axes[AxesMap.DPAD_X.value]
        dpad_y = axes[AxesMap.DPAD_Y.value]
        
        # Reset movement flags
        self.motion_inputs.move_forward = False
        self.motion_inputs.move_backward = False
        self.motion_inputs.move_left = False
        self.motion_inputs.move_right = False
        
        # Set movement flags based on D-Pad (hold-to-move)
        if dpad_y < -0.5:  # Up pressed
            self.motion_inputs.move_forward = True
        elif dpad_y > 0.5:  # Down pressed
            self.motion_inputs.move_backward = True
            
        if dpad_x < -0.5:  # Left pressed
            self.motion_inputs.move_left = True
        elif dpad_x > 0.5:  # Right pressed
            self.motion_inputs.move_right = True

    def _handle_pose_mode(self, axes):
        """Handle analog sticks in POSE mode - control body orientation"""
        
        # Left Stick X = Roll (inverted)
        self.motion_inputs.orn[0] = -self.map(
            axes[AxesMap.LEFT_X.value], -1, 1, 
            self.motion_parameters['orn_x_min'], 
            self.motion_parameters['orn_x_max'])
        
        # Left Stick Y = Pitch
        self.motion_inputs.orn[1] = self.map(
            axes[AxesMap.LEFT_Y.value], -1, 1, 
            self.motion_parameters['orn_y_min'], 
            self.motion_parameters['orn_y_max'])
        
        # Right Stick X = Yaw
        self.motion_inputs.orn[2] = self.map(
            axes[AxesMap.RIGHT_X.value], -1, 1, 
            self.motion_parameters['orn_z_min'], 
            self.motion_parameters['orn_z_max'])
        
        # Right Stick Y = Height (inverted)
        self.motion_inputs.pos[2] = -self.map(
            axes[AxesMap.RIGHT_Y.value], -1, 1, 
            self.motion_parameters['pos_z_min'], 
            self.motion_parameters['pos_z_max'])

    def _handle_jog_mode(self, axes):
        """Handle analog sticks in JOG mode - control walking"""
        
        # Left Stick Y = Step length (forward/backward speed)
        self.motion_inputs.step_length = self.map(
            axes[AxesMap.LEFT_Y.value], -1, 1, 
            self.motion_parameters['step_length_min'], 
            self.motion_parameters['step_length_max'])
        
        # Right Stick X = Yaw rate (turning)
        self.motion_inputs.yaw_rate = self.map(
            axes[AxesMap.RIGHT_X.value], -1, 1, 
            self.motion_parameters['yaw_rate_min'], 
            self.motion_parameters['yaw_rate_max'])
        
        # Right Stick Y = Height (inverted)
        self.motion_inputs.pos[2] = -self.map(
            axes[AxesMap.RIGHT_Y.value], -1, 1, 
            self.motion_parameters['pos_z_min'], 
            self.motion_parameters['pos_z_max'])
        
        # Apply D-Pad movement to step length and lateral fraction
        if self.motion_inputs.move_forward:
            self.motion_inputs.step_length = self.motion_parameters['step_length_max']
        elif self.motion_inputs.move_backward:
            self.motion_inputs.step_length = self.motion_parameters['step_length_min']
        
        if self.motion_inputs.move_left:
            self.motion_inputs.lateral_fraction = -1.0  # Strafe left
        elif self.motion_inputs.move_right:
            self.motion_inputs.lateral_fraction = 1.0   # Strafe right
        else:
            self.motion_inputs.lateral_fraction = 0.0

    def get_motion_inputs(self, axes, buttons):
        """Main method to process controller input and return motion commands"""
        
        # Invert Y axes (stick up = positive)
        axes = list(axes)  # Make a copy to avoid modifying original
        axes[AxesMap.LEFT_Y.value] = -axes[AxesMap.LEFT_Y.value]
        axes[AxesMap.RIGHT_Y.value] = -axes[AxesMap.RIGHT_Y.value]
        
        # Handle mode selection buttons
        self._handle_mode_buttons(buttons)
        
        # Handle D-Pad movement (always active for JOG mode)
        self._handle_dpad_movement(axes)
        
        # Handle analog sticks based on current mode
        if self.motion_inputs.motion_state == MotionState.POSE:
            self._handle_pose_mode(axes)
        elif self.motion_inputs.motion_state == MotionState.JOG:
            self._handle_jog_mode(axes)
        elif self.motion_inputs.motion_state == MotionState.STAND:
            # In STAND mode, reset orientation to neutral
            self.motion_inputs.orn = np.array([0.0, 0.0, 0.0])
            self.motion_inputs.step_length = 0.0
            self.motion_inputs.yaw_rate = 0.0
            self.motion_inputs.lateral_fraction = 0.0
        elif self.motion_inputs.motion_state == MotionState.SIT:
            # In SIT mode, lower body height
            self.motion_inputs.pos[2] = self.motion_parameters['pos_z_min']
            self.motion_inputs.step_length = 0.0
            self.motion_inputs.yaw_rate = 0.0
            self.motion_inputs.lateral_fraction = 0.0

        return copy.deepcopy(self.motion_inputs)
