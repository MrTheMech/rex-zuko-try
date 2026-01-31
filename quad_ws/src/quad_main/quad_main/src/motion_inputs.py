""" 
    Data class containing motion parameters controlling
    the quadruped's movement and states.
    
    Updated for PS5 controller with joystick-based movement controls.
"""

import numpy as np
import copy
from dataclasses import dataclass, field
from enum import Enum


class MotionState(Enum):
    """Robot motion states controlled by face buttons and L1 cycle"""
    STAND = 1    # Default standing position (Cross button)
    SIT = 2      # Sitting/rest position (Circle button)
    POSE = 3     # Body orientation control mode (Triangle button)
    JOG = 4      # Walking/jogging mode (Square button)


@dataclass
class MotionInputs:
    """Data class for robot motion parameters"""
    motion_state: MotionState = MotionState.STAND
    
    # Position: X, Y, Z coordinates
    pos: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    
    # Orientation: Roll, Pitch, Yaw angles
    orn: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    
    # Movement parameters (for JOG mode)
    step_length: float = 0.0       # Forward/backward movement (Left Stick Y)
    lateral_fraction: float = 0.0  # Left/right strafing (Left Stick X)
    yaw_rate: float = 0.0          # Turning rate (Right Stick X)
